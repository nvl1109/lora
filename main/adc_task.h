/* Filename    : adc_task header
   Description : this file handles reading of ADC values from ADC inputs
                 ESP_ADC00-36,ESP_ADC03-39,ESP_ADC06-34,ESP_ADC07-35
                 ESP_ADC04-32
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#ifndef _ADC_TASK_H_
#define _ADC_TASK_H_

#define DEFAULT_VREF       3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES      64          //Multisampling
#define ADC_STACK_SIZE     8192
#define ADC_TASK_PRIORITY  5
#define MAX_ADCS           6
#define MAX_SUPPLY_VOLTAGE 5

#define SHORT_CIRCUIT_DETECT ADC1_CHANNEL_5

void adc_task(void * pvParameter);

#endif
