/* Filename    : software update header
   Description : Contains code to check if new software update is required or Note
                 if new software update is required then it will update the latest software
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _SOFTWARE_UPDATE_H_
#define _SOFTWARE_UPDATE_H_

/* Constants for software update */
#define WEB_SERVER "www.ssla.co.uk"
#define WEB_PORT 80
#define FIRMWARE_UPGRADE_URL "http://www.ssla.co.uk/downloads/IoT/"
#define MAX_TIME_TO_WAIT_FOR_BIN_FILE 10 //maximum time to search for file in http server

/*constants for wifi connectivity*/
#define WIFI_SSID "iPhone"
#define WIFI_PSWD "insfreed"

void software_update_task(void * pvParameter);
void app_wifi_wait_connected();
void update_latest_software(unsigned char* file_n, unsigned short build_v);

#endif
