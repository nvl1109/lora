/* Filename    : adc_task program
   Description : this file handles reading of ADC values from ADC inputs
                 ESP_ADC00-36,ESP_ADC03-39,ESP_ADC06-34,ESP_ADC07-35
                 ESP_ADC04-32,  GPIO33(ADC1_CHANNEL_5/SHORT_CIRCUIT_DETECT),
   Author      : http://www.ssla.co.uk

   This software is SSLA licensed
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
//own headers
#include "adc_task.h"

static const char *TAG = "adc_task";
static esp_adc_cal_characteristics_t *adc_chars;

//GPIO36, GPIO39, GPIO34, GPIO33, GPIO35, GPIO32 if ADC1
static const adc_channel_t channels[MAX_ADCS] = {ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4, SHORT_CIRCUIT_DETECT, ADC_CHANNEL_6, ADC_CHANNEL_7, };
static const adc_atten_t atten = ADC_ATTEN_DB_11;         //maximum signal voltage level is 3.3V

uint32_t adc_samples[MAX_ADCS][NO_OF_SAMPLES];

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG,"eFuse Two Point: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: Supported\n");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "Characterized using eFuse Vref\n");
    } else {
        ESP_LOGI(TAG, "Characterized using Default Vref\n");
    }
}

void adc_task(void *pvParameters)
{
  int i = 0;
  check_efuse();

  adc1_config_width(ADC_WIDTH_BIT_12);
  for (i = 0; i < MAX_ADCS; i++) {
      adc1_config_channel_atten(channels[i], atten);
  }

  //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);

  //Continuously sample all ADC1
  while (1) {
      uint32_t adc_reading = 0;
      //Multisampling
      for (int i = 0; i < MAX_ADCS; i++) {
          for (int j = 0; j < NO_OF_SAMPLES; j++) {
              adc_samples[i][j] = adc1_get_raw((adc1_channel_t)channels[i]);
              adc_reading += adc_samples[i][j];
          }
          adc_reading /= NO_OF_SAMPLES;
          //Convert adc_reading to voltage in mV
          uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
          //ESP_LOGI(TAG, "Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
          if(i == 3 && ((voltage*2) < MAX_SUPPLY_VOLTAGE)) {
              ESP_LOGI(TAG, "Possibly there is a short circuit\n");
              // need to alert someone on this one
          }
          vTaskDelay(pdMS_TO_TICKS(100));
      }
  }
  ESP_LOGI(TAG, "adc_task exited");
  vTaskDelete( NULL );
}
