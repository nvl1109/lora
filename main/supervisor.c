/* Filename    : supervisor program
   Description : Probably this will handle multiple things
                 configures, GPIO, handles other tasks
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
#include <stdlib.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

//my own headers
#include "supervisor.h"
#include "adc_task.h"
#include "uart_task.h"
#include "gpio_task.h"
#include "i2c_task.h"
#include "lora_task.h"

static const char *TAG = "supervisor_task";
struct messageFormat txData;
struct allRequests   allReq[MAX_REQUESTS];

void sendResp(uint16_t currentReqID, uint16_t currentPktID, PACKET_TYPE typ);

void vSupervisor_Start( unsigned portBASE_TYPE uxPriority )
{

  /*Start ADC tasks */
  xTaskCreate(&adc_task, "adc_task",
        ADC_STACK_SIZE, NULL, ADC_TASK_PRIORITY, NULL);
  /*Start UART task*/
  xTaskCreate(&uart_task, "uart_task",
        UART_STACK_SIZE, NULL, UART_TASK_PRIORITY, NULL);
  /*Start GPIO task*/
  xTaskCreate(&gpio_task, "gpio_task",
        GPIO_STACK_SIZE, NULL, GPIO_TASK_PRIORITY, NULL);
  /*Start I2C task*/
  xTaskCreate(&i2c_task, "i2c_task",
        I2C_STACK_SIZE, NULL, I2C_TASK_PRIORITY, NULL);
  /*Start LoRa task*/
  xTaskCreate(&lora_task, "lora_task",
        LORA_STACK_SIZE, NULL, LORA_TASK_PRIORITY, NULL);
  /*Supervisory task*/
  xTaskCreate(&supervisor_task, "supervisor_task",
        SUPERVISOR_STACK_SIZE, NULL, uxPriority, NULL);

}

void processRequest()
{

}

void supervisor_task(void *pvParameters)
{
  struct timeval nowtime;
  uint32_t lasttime  = 0;
  struct allRequests *ptrData;
  uint16_t reqID;
  while(1) {
    if ( xSemaphoreTake( RxdataAvailable, ( TickType_t ) 0 ) == pdTRUE ) {
      ptrData = &allReq[idx];
      idx = (1 - idx); //all data is recieved now switch the index
      if (ptrData->packetTyp >= REQ_INTERIM_PACKET && ptrData->packetTyp <= REQ_READ_UART2_DATA) {
          ESP_LOGI(TAG, "Recieved packet of requestID = 0x%02x , packetID = 0x%02x packet typ = %d" , ptrData->requestID, ptrData->packetID, ptrData->packetTyp);
          sendResp(ptrData->requestID, ptrData->packetID, RESP_ACKNOWLEDGE_PACKET);
          //processRequest();
      } else{
          ESP_LOGI(TAG, "Invalid packet requestID = 0x%02x , packetID = 0x%02x packet typ = %d\n" , ptrData->requestID, ptrData->packetID, ptrData->packetTyp);
          sendResp(ptrData->requestID, ptrData->packetID, RESP_INVALID_PACKET);
      }
    } else
    {
      gettimeofday(&nowtime, NULL);
      uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
      if ((nowseconds - lasttime) >= 3) {
          lasttime = nowseconds;
          if (lora_ready) {
              //Send STATUS reply with random number
              reqID = (rand() % 10000);
              ESP_LOGI(TAG, "##########################################################");
              ESP_LOGI(TAG, "SENT STATUS packet reqID = 0x%02x packetID = 0x101", reqID);
              sendResp(reqID, PACKET_END_NO_FRAMES, RESP_STATUS_PACKET);
          }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  ESP_LOGI(TAG, "supervisor_task exited");
  vTaskDelete( NULL );
}

void sendResp(uint16_t currentReqID, uint16_t currentPktID, PACKET_TYPE typ)
{
  txData.srcAddress  = (uint8_t) db_device_id;
  txData.destAddress = (uint8_t) MASTER_DEVICE_ID;
  txData.signature   = SSLA_SIGNATURE;
  txData.requestID   = currentReqID;
  txData.packetID    = currentPktID;
  txData.packetTyp   = typ;
  txData.length      = 0;
  xQueueSend( txqueue, &txData, 0);
}
