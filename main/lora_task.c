/* Filename    : lora task source
   Description : this file will have upper layer code to read/write from  lora
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
#include <string.h>
#include "esp_log.h"
#include "lora_task.h"
#include "lora.h"

static const char *TAG = "lora_task";

static QueueHandle_t s_lora_queue = NULL;

void lora_task(void *pvParameters)
{
  int count = 0;
  struct lora_msg msg;
  struct lora_msg *msg_ptr = NULL;

    s_lora_queue = xQueueCreate(5, sizeof(struct lora_msg));

    loraInit();
    printf("loraInit done\n");

    while (!loraBegin(LORA_FREQUENCY_EU)) {
      printf(".");
      vTaskDelay(1000 / portTICK_RATE_MS);
    }

    setSyncWord(0xF3);

    printf("\nLora init done\n");

    dumpRegisters();
    printf("DONE!!!\n");

    enableCrc();


  while(1)
  {
    if (xQueueReceive(s_lora_queue, &msg, 10/portTICK_RATE_MS)) {
      // Has data to send
      beginPacket(0);
      loraWriteBuf((uint8_t *)msg.payload, msg.length);
      endPacket(0);
      printf("Sent: '%s'\n", msg.payload);
    }
    // RX
    count = parsePacket(0);
    if (count) {
      memset(&msg, 0, sizeof(msg));
      count = 0;
      while (loraAvailable()) {
        msg.payload[count + 5] = loraRead();
        count ++;
      }
      printf("Recv: '%s' with RSSI %d\n", msg.payload + 5, packetRssi());
      msg.length = count + 5;

#if ! LORA_TEST_TX
      // Echo back
      msg.payload[0] = 'R';
      msg.payload[1] = 'e';
      msg.payload[2] = 'c';
      msg.payload[3] = 'v';
      msg.payload[4] = ' ';
      xQueueSend(s_lora_queue, &msg, 0);
#endif
    }
  }
}

int lora_send(struct lora_msg *msg)
{
  xQueueSend(s_lora_queue, msg, 0);
  return 0;
}
