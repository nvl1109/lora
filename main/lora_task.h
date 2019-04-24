/* Filename    : lora task header
   Description : this file will have definations for
                 upper layer code to read/write from  lora
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _LORA_TASK_H_
#define _LORA_TASK_H_

#define LORA_TEST_TX 0
#define LORA_TEST_RX 0

#define LORA_STACK_SIZE                            8192
#define LORA_TASK_PRIORITY                         5

#define RX_MODE                                    0
#define TX_MODE                                    (!RX_MODE)

#define LEN                                        32
#define LORA_FREQUENCY_EU                          866000
#define LORA_FREQUENCY_US                          915000
#define LORA_FREQUENCY_AS                          433000
#define TX_OUTPUT_POWER                            10
#define PREAMBLE_LENGTH                            8
#define TX_PAYLOAD_LENGTH                          21

//For OOK/FSK/GFSK parameter
#define SYMBOL_TIME                                208000
#define DEVIATION                                  35
#define BANDWIDTH                                  100
#define SYNC_LENGTH                                2


void lora_task(void * pvParameter);
extern bool lora_ready;
extern int  idx;
//int lora_send(struct lora_msg *msg);

union splitData {
   struct {
      uint8_t  totalFrames;
      uint8_t  FrameNumber;
   } s;
   uint16_t packetID;
};

union splitSignature {
   struct {
      uint8_t  sign_high;
      uint8_t  sign_low;
   } s;
   uint16_t signature;
};


#endif
