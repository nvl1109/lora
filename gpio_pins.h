/* Filename    : gpio_pins.h
   Description : Contains pin definiations used for our IoT device
                 it has SPI (1), i2C (1), UART (2), 12 bit ADC (5), GPIO (8)
                 1 Short circuit detector pin
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _GPIO_PINS_H_
#define _GPIO_PINS_H_

//RFM95 definations
#define RFM95W_RESET 2
#define RFM95W_NSS   5
#define RFM95W_MISO  19
#define RFM95W_MOSI  23
#define RFM95W_SCK   18
#define RFM95W_DIO0  4

//Short circuit detector  ADC1 channel 5
#define SHORT_CIRCUIT_DETECT 33

//UART0 definations
#define UART0_TXD 1
#define UART0_RXD 3

//UART2 definations
#define UART2_TXD 17
#define UART2_RXD 16

//I2C definations
#define I2C_SDA 21
#define I2C_SCL 22

// 8 GPIO pins
#define ESP_GPIO_00 0
#define ESP_GPIO_12 12
#define ESP_GPIO_13 13
#define ESP_GPIO_14 14
#define ESP_GPIO_15 15
#define ESP_GPIO_26 26
#define ESP_GPIO_27 27
#define ESP_GPIO_25 25

//Below 5 ADC we can use when wifi is running
#define ESP_ADC00 36
#define ESP_ADC03 39
#define ESP_ADC06 34
#define ESP_ADC07 35
#define ESP_ADC04 32

#endif
