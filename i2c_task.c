/* Filename    : i2c task source
   Description : this file will have code to read/write from i2C slave
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "i2c_task.h"

static const char *TAG = "i2c_task";

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
static esp_err_t i2c_master_sensor_read(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data_h, ACK_VAL);
    i2c_master_read_byte(cmd, data_l, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/*Display data from buffer*/
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        ESP_LOGI(TAG, "%02x ", buf[i]);
        if ((i + 1) % 16 == 0) {
            ESP_LOGI(TAG, "\n");
        }
    }
    ESP_LOGI(TAG, "\n");
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

void i2c_task(void *pvParameters)
{
  int i = 0;
  int ret;
  //uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
  uint8_t sensor_data_h, sensor_data_l;
  int cnt = 0;

  ESP_ERROR_CHECK(i2c_master_init());

  while (1) {
      //ESP_LOGI(TAG, "test cnt: %d", cnt++);
      ret = i2c_master_sensor_read(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
      if (ret == ESP_ERR_TIMEOUT) {
          ESP_LOGE(TAG, "I2C Timeout");
      } else if (ret == ESP_OK) {
          /*ESP_LOGI(TAG, " MASTER READ SENSOR( BH1750 )\n");
          ESP_LOGI(TAG, "data_h: %02x\n", sensor_data_h);
          ESP_LOGI(TAG, "data_l: %02x\n", sensor_data_l);
          ESP_LOGI(TAG, "sensor val: %.02f [Lux]\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);*/
      } else {
          //ESP_LOGI(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
      }
      vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS ) / portTICK_RATE_MS);
      //------------------read-from-slave---------------------------------
      ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_DATA_LENGTH);

      if (ret == ESP_ERR_TIMEOUT) {
         // ESP_LOGE(TAG, "I2C Timeout");
      } else if (ret == ESP_OK) {
         // ESP_LOGI(TAG, "TASK MASTER READ FROM SLAVE");
          disp_buf(data_rd, RW_DATA_LENGTH);
      } else {
         // ESP_LOGI(TAG, "TASK %s: Master read slave error, IO not connected...",
        //          esp_err_to_name(ret));
      }
      //------------------write-to-slave---------------------------------
      //int size;
      for (i = 0; i < DATA_LENGTH; i++) {
          data_wr[i] = i + 10;
      }
      ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_DATA_LENGTH);

      if (ret == ESP_ERR_TIMEOUT) {
          //ESP_LOGE(TAG, "I2C Timeout");
      } else if (ret == ESP_OK) {
          //ESP_LOGI(TAG, "TASK  MASTER WRITE TO SLAVE\n");
          disp_buf(data_wr, RW_DATA_LENGTH);
      } else {
          //ESP_LOGI(TAG, "TASK %s: Master write slave error, IO not connected....",
          //         esp_err_to_name(ret));
      }
      vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS ) / portTICK_RATE_MS);
  }
  ESP_LOGI(TAG, "i2c_task exited");
  vTaskDelete( NULL );
}
