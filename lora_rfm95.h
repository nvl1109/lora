/* Filename    : lora_rfm95 header
   Description : this file will have lower layer declarations to read/write
                 from rfm95 lora module
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef LORA_H
#define LORA_H

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#ifndef byte
    typedef unsigned char byte;
#endif

#define RFM95_DIO0                  4
#define RFM95_MISO                  19
#define RFM95_MOSI                  23
#define RFM95_SCK                   18
#define RFM95_NCS                   5
#define RFM95_RESET                 12

#define PA_OUTPUT_RFO_PIN           0
#define PA_OUTPUT_PA_BOOST_PIN      1

  int loraInit(void);
  int loraBegin(long frequency);
  void loraEnd();

  int beginPacket(int implicitHeader);
  int endPacket(bool async);

  int parsePacket(int size);
  int packetRssi();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  size_t loraWriteByte(uint8_t byte);
  size_t loraWriteBuf(const uint8_t *buffer, size_t size);

  // from Stream
  int  loraAvailable();
  int  loraRead();
  int  loraPeek();
  void loraFlush();

  void loraIdle();
  void loraSleep();

  void setTxPower(int level, int outputPin);
  void setFrequency(long frequency);
  void setSpreadingFactor(int sf);
  void setSignalBandwidth(long sbw);
  void setCodingRate4(int denominator);
  void setPreambleLength(long length);
  void setSyncWord(int sw);
  void enableCrc();
  void disableCrc();
  void enableInvertIQ();
  void disableInvertIQ();

  void setOCP(uint8_t mA); // Over Current Protection control

  // deprecated
  void crc();
  void noCrc();

  byte random();

  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters();

  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();
  bool isTransmitting();

  int  getSpreadingFactor();
  long getSignalBandwidth();

  void setLdoFlag();

  uint8_t readRegister(uint8_t address);
  void    writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

#endif
