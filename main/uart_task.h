/* Filename    : uart task header
   Description : this file will have declarations for UART0 , UART2
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _UART_TASK_H_
#define _UART_TASK_H_

#define UART0_TXD  (GPIO_NUM_1)
#define UART0_RXD  (GPIO_NUM_3)
#define UART0_RTS  (UART_PIN_NO_CHANGE)
#define UART0_CTS  (UART_PIN_NO_CHANGE)
#define UART0_BAUD  115200

#define UART2_TXD  (GPIO_NUM_17)
#define UART2_RXD  (GPIO_NUM_16)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)
#define UART2_BAUD  115200

#define UART_BUF_SIZE (1024)

#define UART_STACK_SIZE     4096
#define UART_TASK_PRIORITY  5

void uart_task(void * pvParameter);

#endif
