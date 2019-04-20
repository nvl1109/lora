// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lora.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_DIO_MAPPING_2        0x41
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

  static void onDio0Rise();

  long _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
static spi_device_handle_t s_spi;
  void (*_onReceive)(int);

  static const char *TAG = "lora";

static void IRAM_ATTR di0_isr_handler(void* arg);

int loraInit(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=RFM95_MISO,
        .mosi_io_num=RFM95_MOSI,
        .sclk_io_num=RFM95_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=255,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_NATIVE_PINS | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_DUAL,
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=SPI_MASTER_FREQ_8M,     //Clock out at 8 MHz
        .mode=0,                                //SPI mode 0
        .spics_io_num=RFM95_nCS,                //CS pin
        .queue_size=2,                          //We want to be able to queue 2 transactions at a time
        .pre_cb=NULL,
        .command_bits = 0,
        .address_bits = 8,
        .flags = SPI_DEVICE_NO_DUMMY,
        .dummy_bits = 0,
    };

    _frequency = 0;
    _packetIndex = 0;
    _implicitHeaderMode = 0;
    _onReceive = NULL;

    // start SPI
    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &s_spi);
    ESP_ERROR_CHECK(ret);


    // gpio_config_t io_conf;
    // //interrupt of rising edge
    // io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    // //bit mask of the pins, use GPIO4/5 here
    // io_conf.pin_bit_mask = 1 << RFM95_DIO0;
    // //set as input mode
    // io_conf.mode = GPIO_MODE_INPUT;
    // //enable pull-up mode
    // io_conf.pull_up_en = 1;
    // gpio_config(&io_conf);

    // //change gpio intrrupt type for one pin
    // gpio_set_intr_type(RFM95_DIO0, GPIO_INTR_POSEDGE);
    // gpio_isr_handler_add(RFM95_DIO0, di0_isr_handler, (void*) RFM95_DIO0);

    return 0;
}

int loraBegin(long frequency)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1 << RFM95_RESET;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_direction(RFM95_RESET, GPIO_MODE_OUTPUT);

    // perform reset
    gpio_set_level(RFM95_RESET, 0);
    vTaskDelay(1/portTICK_RATE_MS);
    gpio_set_level(RFM95_RESET, 1);
    vTaskDelay(10/portTICK_RATE_MS);
    gpio_set_direction(RFM95_RESET, GPIO_MODE_INPUT);

  // check version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  loraSleep();

  // set frequency
  setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  setTxPower(17, PA_OUTPUT_PA_BOOST_PIN);

  // put in standby mode
  loraIdle();

  return 1;
}

void loraEnd()
{
  // put in sleep mode
  loraSleep();

  // stop SPI
  spi_bus_remove_device(s_spi);
  spi_bus_free(VSPI_HOST);
}

int beginPacket(int implicitHeader)
{
  if (isTransmitting()) {
    return 0;
  }

  // put in standby mode
  loraIdle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int endPacket(bool async)
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  if (async) {
    // grace time is required for the radio
    // ets_delay_us(150);
    vTaskDelay(1/portTICK_RATE_MS);
  } else {
    // wait for TX done
    while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
      // yield();
        // TODO: here
        taskYIELD();
    }
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return 1;
}

bool isTransmitting()
{
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

int parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    loraIdle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long packetFrequencyError()
{
  int32_t freqError = 0;
  freqError = (int32_t)(readRegister(REG_FREQ_ERROR_MSB) & 0x7);
  freqError <<= 8L;
  freqError += (int32_t)(readRegister(REG_FREQ_ERROR_MID));
  freqError <<= 8L;
  freqError += (int32_t)(readRegister(REG_FREQ_ERROR_LSB));

  if (readRegister(REG_FREQ_ERROR_MSB) & 0x8) { // Sign bit is on
     freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = (((float)(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

  return (long)(fError);
}

size_t loraWriteByte(uint8_t byte)
{
  return loraWriteBuf(&byte, sizeof(byte));
}

size_t loraWriteBuf(const uint8_t *buffer, size_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    writeRegister(REG_FIFO, buffer[i]);
  }

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

int loraAvailable()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int loraRead()
{
  if (!loraAvailable()) {
    return -1;
  }

  _packetIndex++;

  return readRegister(REG_FIFO);
}

int loraPeek()
{
  if (!loraAvailable()) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = readRegister(REG_FIFO);

  // restore FIFO address
  writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

void loraFlush()
{
}

void loraIdle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void loraSleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      writeRegister(REG_PA_DAC, 0x87);
      setOCP(140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      writeRegister(REG_PA_DAC, 0x84);
      setOCP(100);
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

int getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

void setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  setLdoFlag();
}

long getSignalBandwidth()
{
  byte bw = (readRegister(REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3;
  }

  return -1;
}

void setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  setLdoFlag();
}

void setLdoFlag()
{
  // Section 4.1.1.5
  long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

  // Section 4.1.1.6
  int ldoOn = symbolDuration > 16;

  uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
  ldoOn ? ((config3) |= (1UL << (3))) : ((config3) &= ~(1UL << (3)));
  writeRegister(REG_MODEM_CONFIG_3, config3);
}

void setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void enableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x66);
  writeRegister(REG_INVERTIQ2, 0x19);
}

void disableInvertIQ()
{
  writeRegister(REG_INVERTIQ,  0x27);
  writeRegister(REG_INVERTIQ2, 0x1d);
}

void setOCP(uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

byte random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

void setSPIFrequency(uint32_t frequency)
{
  // _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
    ESP_LOGE(TAG, "setSPIFrequency is not implemeted");
}

void dumpRegisters()
{
    ESP_LOGI(TAG, "RFM95 Registers:");
  for (int i = 0; i < 128; i++) {
    ESP_LOGI(TAG, "addr 0x%02x = 0x%x", i, readRegister(i));
  }
}

void explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void handleDio0Rise()
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

uint8_t readRegister(uint8_t address)
{
    // ESP_LOGI(TAG, "read 0x%x", address);
  return singleTransfer(address & 0x7f, 0x00);
}

void writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

uint8_t singleTransfer(uint8_t address, uint8_t value)
{
    esp_err_t ret;
    spi_transaction_t t;

  // gpio_set_level(RFM95_nCS, 0);

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length=8;                     //Command is 8 bits
    t.addr = address;
    t.tx_data[0] = value;
    t.rxlength = 8;
    ret=spi_device_polling_transmit(s_spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.

  // gpio_set_level(RFM95_nCS, 1);

  return t.rx_data[0];
}

void onDio0Rise()
{
  // LoRa.handleDio0Rise();
}

void crc() { enableCrc(); }
void noCrc() { disableCrc(); }

static void IRAM_ATTR di0_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}