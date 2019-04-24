/* Filename    : uart task source
   Description : this file will have code to read/write from UART0 , UART2
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "uart_task.h"

static const char *TAG = "uart_task";
uint8_t *uart_data[2];

void uart_task(void *pvParameters)
{
    /* Configure parameters of an UART0, UART2 driver,
     * communication pins and install the driver */
    uart_config_t uart0_config = {
        .baud_rate = UART0_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_config_t uart2_config = {
        .baud_rate = UART2_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart0_config);
    uart_set_pin(UART_NUM_0, UART0_TXD, UART0_RXD, UART0_RTS, UART0_CTS);
    uart_driver_install(UART_NUM_0, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_NUM_2, &uart2_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, UART_BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uart_data[0] = (uint8_t *) malloc(UART_BUF_SIZE);
    uart_data[1] = (uint8_t *) malloc(UART_BUF_SIZE);

    while (1) {
        // Read data from the UART0
        int len = uart_read_bytes(UART_NUM_0, uart_data[0], UART_BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART0
        uart_write_bytes(UART_NUM_0, (const char *) uart_data[0], len);

        // Read data from the UART2
        len = uart_read_bytes(UART_NUM_2, uart_data[1], UART_BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART2
        uart_write_bytes(UART_NUM_2, (const char *) uart_data[1], len);

        vTaskDelay(100 / portTICK_PERIOD_MS);

    }
    ESP_LOGI(TAG, "uart_task exited");
    vTaskDelete( NULL );
}
