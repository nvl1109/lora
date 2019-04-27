/* Filename    : main program
   Description : Contains OTA update and kickstarts other tasks
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "software_update.h"
#include "supervisor.h"

static const char *TAG = "main";

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static uint8_t BUILD_VER[] = "unknown";
static uint8_t COMMIT_ID[] = "unknown";
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;
nvs_handle my_handle;
unsigned short db_build_ver=0;
unsigned short db_lora_reg = 0;
unsigned short db_device_id = 0;
unsigned short db_signature = 0;

esp_err_t err;
SemaphoreHandle_t RxdataAvailable = NULL;
xQueueHandle txqueue;
TaskHandle_t s_software_update_tsk = NULL;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
	if (s_software_update_tsk == NULL) {
            // Task isn't created --> create it
    	    xTaskCreate(&software_update_task, "software_update_task", SOFTWARE_UPDATE_STACK_SIZE, NULL, SOFTWARE_UPDATE_TASK_PRIORITY, &s_software_update_tsk);
	    if(s_software_update_tsk == NULL) {
		ESP_LOGE(TAG, "Software update task creation failed");
	    }
	}

        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PSWD,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void ota_task(void * pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA Task ");

    app_wifi_wait_connected();

    ESP_LOGI(TAG, "Connected to WiFi network! Exiting from OTA task");
    vTaskDelete( NULL );
}

void app_main()
{
    // Initialize NVS.
    ESP_LOGI(TAG,"Build vers = %s  , commit id = %s\n" , BUILD_VER, COMMIT_ID);
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 1.OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // 2.NVS partition contains data in new format and cannot be recognized by this version of code.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_LOGE(TAG, "NVS was erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
    ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle...");
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
    } else {
        // Read
        err = nvs_get_i16(my_handle, "build_ver", (int16_t *)&db_build_ver);
        err = nvs_get_i16(my_handle, "lora_reg", (int16_t *)&db_lora_reg);
        err = nvs_get_i16(my_handle, "device_id", (int16_t *)&db_device_id);
        err = nvs_get_i16(my_handle, "ssla_signature", (int16_t *)&db_signature);
        //if (db_signature == 0) {
        //    db_signature = SSLA_SIGNATURE;
        //}
        //db_device_id = 1;
        ESP_LOGI(TAG, "Current software version = 0x%x in decimal = %d" , db_build_ver, db_build_ver);
        ESP_LOGI(TAG, "db_lora_reg = 0x%x, db_device_id = 0x%x, db_signature = 0x%x", db_lora_reg, db_device_id, db_signature);
    }
    initialise_wifi();
    
    vSemaphoreCreateBinary(RxdataAvailable);
    if(RxdataAvailable == NULL)
    {
      ESP_LOGI(TAG, "Heap unavailable");
    }
    else
    {
      ESP_LOGI(TAG, "created semaphore");
    }
    if( xSemaphoreTake( RxdataAvailable, ( TickType_t ) 100 ) != pdTRUE ) {
      ESP_LOGI(TAG, "Failed to get semaphore ");
    }
    else{
        ESP_LOGI(TAG, "Got semaphore ");
    }
    //create message queue for transmitting lora packet
    txqueue = xQueueCreate(TX_MESSGAGE_QUEUE_SIZE, sizeof(txData));

    vSupervisor_Start( SUPERVISOR_TASK_PRIORITY);
    ESP_LOGI(TAG, "app_main exited");
    vTaskDelete(NULL);
}

void app_wifi_wait_connected(){
  /* Wait for the callback to set the CONNECTED_BIT in the
     event group.
  */
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                      false, true, portMAX_DELAY);
}

void update_latest_software(unsigned char* file_n, unsigned short build_v)
{
    char firmare_url[250] = "";
    strcpy(firmare_url,FIRMWARE_UPGRADE_URL);
    strcat(firmare_url , (char *)file_n);
    ESP_LOGI(TAG, "Updating software from %s " , firmare_url);
    esp_http_client_config_t config = {
        .url = firmare_url,
        .event_handler = _http_event_handler,
    };
    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        err = nvs_set_i16(my_handle, "build_ver", (int16_t *)build_v);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        ESP_LOGI(TAG, "Firmware upgrade success now Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
}
