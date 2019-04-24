/* Filename    : software update program
   Description : Contains code to check if new software update is required or Note
                 if new software update is required then it will update the latest software
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"

//#include "esp_wifi.h"
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
//my own headers
#include "software_update.h"

extern unsigned short db_build_ver;

const char *REQUEST = "GET " FIRMWARE_UPGRADE_URL " HTTP/1.0\r\n"
    "Host: "WEB_SERVER"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "\r\n";

static const char *TAG = "software_update";
int32_t time_elapsed=0;
unsigned char http_buf[16]; // contains file name without extension and dot

union ToShort
{
    uint32_t number;
    uint16_t filename[2];
} toShort;

static void check_elapsed_time(void* arg)
{
    time_elapsed+=1;
}

/*Check and see if we have a valid 4 digit hex file */
bool check_hex_file_valid()
{
   if (isxdigit(http_buf[0]) && isxdigit(http_buf[1]) && isxdigit(http_buf[2]) && isxdigit(http_buf[3]) ) {
     // has valid extension
     if (http_buf[4] == '.' && http_buf[5] == 'b' && http_buf[6] == 'i' && http_buf[7] == 'n') {
        http_buf[4] = '\0';
        return true;
     }
   }
   ESP_LOGI(TAG, "Invalid file found %s", http_buf);
   return false;
}


void software_update_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    bool file_found = false;
    esp_timer_handle_t software_update_timer;


    const esp_timer_create_args_t periodic_timer_args = {
             .callback = &check_elapsed_time,
             /* name is optional, but may help identify the timer when debugging */
             .name = "software_update_timer"
     };

     ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &software_update_timer));

     ESP_ERROR_CHECK(esp_timer_start_periodic(software_update_timer, 1000000));

     ESP_LOGI(TAG, "Started software_update_task");
     while(!file_found && time_elapsed < MAX_TIME_TO_WAIT_FOR_BIN_FILE) {
        /* Wait for the callback to set the CONNECTED_BIT in the
           event group.
        */
        app_wifi_wait_connected();
        ESP_LOGI(TAG, "Connected to AP");

        int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

        if (write(s, REQUEST, strlen(REQUEST)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(http_buf, sizeof(http_buf));
            r = read(s, http_buf, 1);
            if (http_buf[0] == '.'){
              r = read(s, http_buf, 1);
                if (http_buf[0] == 'b'){
                    r = read(s, http_buf, 1);
                    if (http_buf[0] == 'i'){
                        r = read(s, http_buf, 1);
                        if (http_buf[0] == 'n'){
                            r = read(s, http_buf, 2);
                            r = read(s, http_buf, 8);
                            if (http_buf[8] != '\0') http_buf[8] = '\0';
                            if (check_hex_file_valid() == true)
                            {
                                ESP_LOGI(TAG, "file name found %s", http_buf);
                                toShort.number = (int)strtol((char *)http_buf, NULL, 16);
                                if (db_build_ver == toShort.filename[0]){
                                    ESP_LOGI(TAG, "Firmware already the latest version 0x%x and decimal in = %d", toShort.filename[0] ,toShort.filename[0]);
                                } else{
                                    http_buf[4] = '.';
                                    http_buf[5] = 'b';
                                    http_buf[6] = 'i';
                                    http_buf[7] = 'n';
                                    http_buf[8] = '\0';
                                    update_latest_software(http_buf, toShort.filename[0]);
                                }
                                file_found = true;
                               break;
                             }
                        }
                    }
                }
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d\r\n", r, errno);
        close(s);
        for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
    if (!file_found)
    {
      ESP_LOGI(TAG, "Firmware file is not present or invalid firmware filename");
      ESP_LOGI(TAG, "Note: filename must a 4 digit with extension .bin For ex: 4354.bin");
    }
    ESP_ERROR_CHECK(esp_timer_stop(software_update_timer));
    ESP_ERROR_CHECK(esp_timer_delete(software_update_timer));
    ESP_LOGI(TAG, "Closing software_update!");
    vTaskDelete( NULL );
}
