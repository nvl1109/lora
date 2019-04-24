/* Filename    : i2c task header
   Description : this file will have code to read/write from i2C slave
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _I2C_TASK_H_
#define _I2C_TASK_H_

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH                      512              /*!< Data buffer length of test buffer */
#define RW_DATA_LENGTH                   128              /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS      1000             /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO                22               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO                21               /*!< gpio number for I2C master data  */
#define CONFIG_I2C_MASTER_PORT_NUM       1
#define I2C_MASTER_NUM                   I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ               100000           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE        0                /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE        0                /*!< I2C master doesn't need buffer */

#define BH1750_SENSOR_ADDR               0x23             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                 0x10             /*!< Operation mode */
#define ESP_SLAVE_ADDR                   0x23                           /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                        I2C_MASTER_WRITE               /*!< I2C master write */
#define READ_BIT                         I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN                     0x1                            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                    0x0                            /*!< I2C master will not check ack from slave */
#define ACK_VAL                          0x0                            /*!< I2C ack value */
#define NACK_VAL                         0x1                            /*!< I2C nack value */

#define I2C_STACK_SIZE                   4096
#define I2C_TASK_PRIORITY                5

void i2c_task(void *pvParameters);

#endif
