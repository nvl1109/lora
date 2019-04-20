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
  int not_interested = 0;
  int send_count = 0;

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
    if (xQueueReceive(s_lora_queue, &msg, 5/portTICK_RATE_MS)) {
      // Has data to send
      beginPacket(0);
      loraWriteBuf((uint8_t *)msg.payload, msg.length);
      endPacket(0);
      printf("Sent: '%s'\n", msg.payload);
      vTaskDelay(5/portTICK_RATE_MS);
    }
    // RX
    not_interested = 0;
    count = parsePacket(0);
    if (count) {
      memset(&msg, 0, sizeof(msg));
      count = 0;
      while (loraAvailable()) {
        msg.payload[count] = loraRead();
        count ++;

        if ((count >= 2) && ((msg.payload[0] != 0x11) || msg.payload[1] != 0x42)) {
          // Not interested packet
          printf("Not interested packet, ignore 0x%x 0x%x\n", msg.payload[0], msg.payload[1]);
          loraSleep(); // Enter sleep mode to clear FIFO
          vTaskDelay(2/portTICK_RATE_MS);
          loraIdle();  // Back to standby mode
          not_interested = 1;
          break;
        }
      }
      if (not_interested) continue;
      printf("Recv: message '%s' with RSSI %d:", msg.payload + 2, packetRssi());
      msg.length = count;
      dumpPayload(msg.payload, count);
#if ! LORA_TEST_TX
      // Echo back
      if (send_count++ & 0x1) {
        msg.payload[0] = 0x12;
      }
      msg.payload[3] = 'a';
      lora_send(&msg);
#endif
    }
  }
}

int lora_send(struct lora_msg *msg)
{
  xQueueSend(s_lora_queue, msg, 0);
  return 0;
}
