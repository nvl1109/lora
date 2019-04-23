/* Filename    : gpio task source
   Description : configures GPIO as input and output to read/write from GPIO
                 Output pins (15,25,26,27) Input pins (0,12,13,14)
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "gpio_task.h"

static const char *TAG = "gpio_task";

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_read_pin_level_task(void* arg)
{
    uint32_t io_num;
    for(;;) {
        // do not block
        if(xQueueReceive(gpio_evt_queue, &io_num, 0)) {
            //ESP_LOGI(TAG, "GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

void gpio_task(void *pvParameters)
{
  gpio_config_t io_conf;
  //disable interrupt
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

  //interrupt of rising edge
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);

  //change gpio intrrupt type for one pin
  gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
  //gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(GPIO_INPUT_IO_2, GPIO_INTR_ANYEDGE);
  gpio_set_intr_type(GPIO_INPUT_IO_3, GPIO_INTR_ANYEDGE);

  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  //start gpio task
  xTaskCreate(gpio_read_pin_level_task, "gpio_read_pin_level_task", 2048, NULL, 10, NULL);

  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
  //gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
  gpio_isr_handler_add(GPIO_INPUT_IO_2, gpio_isr_handler, (void*) GPIO_INPUT_IO_2);
  gpio_isr_handler_add(GPIO_INPUT_IO_3, gpio_isr_handler, (void*) GPIO_INPUT_IO_3);

  int cnt = 0;
  while(1) {
      //ESP_LOGI(TAG, "cnt: %d\n", cnt++);
      vTaskDelay(1000 / portTICK_RATE_MS);
      gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
      gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
      gpio_set_level(GPIO_OUTPUT_IO_2, cnt % 2);
      gpio_set_level(GPIO_OUTPUT_IO_3, cnt % 2);
  }
  ESP_LOGI(TAG, "gpio_task exited");
  vTaskDelete( NULL );
}
