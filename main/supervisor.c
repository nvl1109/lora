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
struct allRequests *ptrData;

void sendResp(uint16_t currentReqID, uint16_t currentPktID, PACKET_TYPE typ);
static void periodic_timer_callback(void* arg);
static void oneshot_timer_callback(void* arg);
esp_timer_handle_t periodic_timer;
esp_timer_handle_t oneshot_timer;
PACKET_TYPE sendRespType;
bool syncSetUsingOneShortTimer = false;

void vSupervisor_Start( unsigned portBASE_TYPE uxPriority )
{
  const esp_timer_create_args_t periodic_timer_args = {
          .callback = &periodic_timer_callback,
          /* sends ACK, RESP, STATUS packets */
          .name = "slave"
  };

  const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &oneshot_timer_callback,
        /* argument specified here will be passed to timer callback function */
        //.arg = (void*) periodic_timer,
        .name = "one-shot"
  };

  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

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
  //struct timeval nowtime;
  //uint32_t lasttime  = 0;

  while(1) {
    if ( xSemaphoreTake( RxdataAvailable, ( TickType_t ) 0 ) == pdTRUE ) {
      ptrData = &allReq[idx];
      idx = (1 - idx); //all data is recieved now switch the index
      if (ptrData->packetTyp >= REQ_INTERIM_PACKET && ptrData->packetTyp <= REQ_READ_UART2_DATA) {
          ESP_LOGI(TAG, "Recieved packet of requestID = 0x%02x , packetID = 0x%02x packet typ = %d" , ptrData->requestID, ptrData->packetID, ptrData->packetTyp);
          sendRespType = RESP_ACKNOWLEDGE_PACKET;
          //processRequest();
      } else{
          ESP_LOGI(TAG, "Invalid packet requestID = 0x%02x , packetID = 0x%02x packet typ = %d\n" , ptrData->requestID, ptrData->packetID, ptrData->packetTyp);
          sendRespType = RESP_INVALID_PACKET;
      }
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
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

void startPeriodicTimer()
{
    //ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MAXIMUM_RESP_TIME_DURATION));
}

void startOneShotTimer(uint8_t devID)
{
   int64_t network_status_time, my_device_sync_time, time_gap;
   network_status_time = (MAXIMUM_RESP_TIME_DURATION + (devID * EACH_SLAVE_TIME_SLOT));
   my_device_sync_time = (MAXIMUM_RESP_TIME_DURATION + (db_device_id * EACH_SLAVE_TIME_SLOT));
   time_gap =  (network_status_time - my_device_sync_time);
   if (time_gap < 0) {
     time_gap = abs(time_gap);
   } else {
     time_gap = ( MAXIMUM_RESP_TIME_DURATION - time_gap );
   }
   ESP_LOGI(TAG, "Gap time = %lld nw = %lld md = %lld devID = %d", time_gap, network_status_time , my_device_sync_time, devID);
   ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, time_gap));
   syncSetUsingOneShortTimer = true;
}
static void periodic_timer_callback(void* arg)
{
    uint16_t reqID;
    int64_t time_since_sync, time_after_xdelay;

    time_since_sync = esp_timer_get_time();
    if ( db_device_id > 0 && !syncSetUsingOneShortTimer) {
        TickType_t xDelay = (FREERTOS_EACH_SLAVE_TIME_SLOT / portTICK_PERIOD_MS);
        xDelay = (db_device_id * xDelay);
        vTaskDelay(xDelay);
    }
    time_after_xdelay = esp_timer_get_time();
    switch (sendRespType) {
      case RESP_ACKNOWLEDGE_PACKET:
          sendResp(ptrData->requestID, ptrData->packetID, RESP_ACKNOWLEDGE_PACKET);
      break;

      case RESP_INVALID_PACKET:
          sendResp(ptrData->requestID, ptrData->packetID, RESP_INVALID_PACKET);
      break;

      default:
      if (lora_ready) {
          //Send STATUS reply with random number
          reqID = (rand() % 10000);
          ESP_LOGI(TAG, "##########################################################");
          ESP_LOGI(TAG, "SENT STATUS packet reqID  = 0x%02x packetID = 0x101", reqID);
          //ESP_LOGI(TAG, "SENT STATUS packet reqID = 0x%02x packetID = 0x101 at %lld  after xDelay = %lld", reqID, time_since_sync, time_after_xdelay);
          sendResp(reqID, PACKET_END_NO_FRAMES, RESP_STATUS_PACKET);
      }
    }
    sendRespType = RESP_STATUS_PACKET;
}

static void oneshot_timer_callback(void* arg)
{
    startPeriodicTimer();
    ESP_ERROR_CHECK(esp_timer_delete(oneshot_timer));
}
