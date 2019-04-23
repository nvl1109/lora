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
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "supervisor.h"
#include "lora_task.h"
#include "lora_rfm95.h"

static const char *TAG = "lora_task";
bool lora_ready = false;
int  idx = 0;
static uint16_t last_requestID = 0;
//static uint16_t last_subReqID = 0;
//static uint16_t last_frameNumber = 0;

void initalize_lora(){
    loraInit();
    ESP_LOGI(TAG, "loraInit done");

    while (!loraBegin(LORA_FREQUENCY_EU)) {
      ESP_LOGI(TAG, ".");
      vTaskDelay(1000 / portTICK_RATE_MS);
    }
    setSyncWord(0xF3);
    ESP_LOGI(TAG, "Lora init done");
    dumpRegisters();
    ESP_LOGI(TAG, "DONE!!!");
    // enable CRC check
    enableCrc();
    lora_ready = true;
}

void lora_task(void *pvParameters)
{
    int count = 0, len = 0;
    struct messageFormat msg;
    union splitData Data;
    union splitSignature sign;
    char *msgPtr = (char *) &msg;
    char data;
    bool not_interested = false;

    initalize_lora();
    sign.signature = db_signature;
    while(1)
    {
      if (xQueueReceive(txqueue, &msg, 2/portTICK_RATE_MS)) {
        //Has data to send
        beginPacket(0);
        loraWriteBuf((uint8_t *)&msg, LORA_TX_BUFF_SIZE + msg.length);
        endPacket(0);
        vTaskDelay(2/portTICK_RATE_MS);
      }
      //Recieve packet
      count = parsePacket(0);
      if (count) {
          not_interested = false;
          memset(&msg, 0, sizeof(msg));
          count = 0;
          while (loraAvailable()) {
               data = loraRead();
               if (count < LORA_TX_BUFF_SIZE)
               {
                  msgPtr[count] = data;
               }
              //you can clear FIFO in case packet does not belong to your node
              if(count == 3)
              {
                if (msgPtr[1] != db_device_id || msgPtr[2] != sign.sign_low || msgPtr[3] != sign.sign_high) {
                  //ESP_LOGI(TAG, " msgPtr[2] = %d and msgPtr[3] = %d", msgPtr[2], msgPtr[3]);
                  loraSleep(); // Enter sleep mode to clear FIFO
                  vTaskDelay(2/portTICK_RATE_MS);
                  loraIdle();  // Back to standby mode
                  not_interested = true;
                  break;
                }
              }
              count++;
          }
         if(not_interested){
           continue;
         }
         if(last_requestID != msg.requestID) {
             Data.packetID = msg.packetID;
             //ESP_LOGI(TAG, " packetID = %d and totalFrames = %d", msg.packetID, Data.totalFrames);
             if (Data.totalFrames == 1) {
               //this frame has just one packet hence no more packet for this request id
               last_requestID = msg.requestID;
               allReq[idx].requestID = msg.requestID;
               allReq[idx].packetTyp = msg.packetTyp;
               allReq[idx].packetID = msg.packetID;
               if(msg.length > 0) {
                  memcpy(allReq[idx].lora_Rxmessage, msg.payload, msg.length);
               }
               if (RxdataAvailable != NULL) {
                   xSemaphoreGive(RxdataAvailable);
               }
             } else {
                   //we recived multiple frame packet Data.totalFrames
                   sendResp(msg.requestID, msg.packetID, RESP_INVALID_FRAME_NUMBER_ID);
             }
         } else {
              //resend ack as the mesg is recieved again
              sendResp(msg.requestID, msg.packetID, RESP_ACKNOWLEDGE_PACKET);
         }
      }
    }
}
