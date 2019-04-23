/* Filename    : gpio task header
   Description : configures GPIO as input and output contains definations
                 Output pins (15,25,26,27) Input pins (0,12,13,14)
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _GPIO_TASK_H_
#define _GPIO_TASK_H_

//these pins are configured as outputs
#define GPIO_OUTPUT_IO_0    15
#define GPIO_OUTPUT_IO_1    25
#define GPIO_OUTPUT_IO_2    26
#define GPIO_OUTPUT_IO_3    27
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1) | (1ULL<<GPIO_OUTPUT_IO_2) | (1ULL<<GPIO_OUTPUT_IO_3))

//these pins are configured as inputs
#define GPIO_INPUT_IO_0     0 // This pin is used to put the device in BOOT mode
//#define GPIO_INPUT_IO_1     12 // used by Lora module
#define GPIO_INPUT_IO_2     13
#define GPIO_INPUT_IO_3     14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_2) | (1ULL<<GPIO_INPUT_IO_3))
#define ESP_INTR_FLAG_DEFAULT 0

#define GPIO_STACK_SIZE     4096
#define GPIO_TASK_PRIORITY  5

void gpio_task(void * pvParameter);

#endif
