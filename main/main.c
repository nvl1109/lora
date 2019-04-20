/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <string.h>

#include "lora_task.h"

TaskHandle_t g_loratsk;
TaskHandle_t send_tsk;

void task_send_demo(void * pvParameter);

void app_main()
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    xTaskCreate(lora_task, "LORA", LORA_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &g_loratsk);
    configASSERT( g_loratsk );

#if LORA_TEST_TX
    xTaskCreate(task_send_demo, "SEND", 2048, NULL, tskIDLE_PRIORITY + 1, &send_tsk);
#endif
}

void task_send_demo(void * pvParameter)
{
  struct lora_msg msg;
  int count = 0;
  while (1) {
    vTaskDelay(5000/portTICK_RATE_MS);
    msg.length = sprintf(msg.payload, "Hello %d", count++);

    printf(">>> %s\n", msg.payload);
    lora_send(&msg);
  }
}
