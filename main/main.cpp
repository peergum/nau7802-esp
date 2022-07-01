/**
 * @file main.cpp
 * @author Phil Hilger AKA "PeerGum"
 * @brief
 * @version 0.1
 * @date 2022-06-30
 *
 * @copyright Copyright (c) 2022 PeerGum
 *
 */

#include "NAU7802.h"
#include "main.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

static const char *TAG = "example";
NAU7802 sensor(NAU7802_I2C_PORT, NAU7802_I2CADDR_DEFAULT, NAU7802_INT_PIN);
TaskHandle_t xMainTask;

#ifdef __cplusplus
extern "C" {
#endif

void app_main();

#ifdef __cplusplus
}
#endif

void mainTask(void *arg) {
  if (!sensor.init()) {
    ESP_LOGI(TAG, "Failed to find NAU7802");
    while (1) {
      esp_task_wdt_reset();
    }
  }
  ESP_LOGI(TAG, "Found NAU7802");

  sensor.setLDO(NAU7802_3V0);
  ESP_LOGI(TAG, "LDO voltage set to ");
  switch (sensor.getLDO()) {
  case NAU7802_4V5:
    ESP_LOGI(TAG, "4.5V");
    break;
  case NAU7802_4V2:
    ESP_LOGI(TAG, "4.2V");
    break;
  case NAU7802_3V9:
    ESP_LOGI(TAG, "3.9V");
    break;
  case NAU7802_3V6:
    ESP_LOGI(TAG, "3.6V");
    break;
  case NAU7802_3V3:
    ESP_LOGI(TAG, "3.3V");
    break;
  case NAU7802_3V0:
    ESP_LOGI(TAG, "3.0V");
    break;
  case NAU7802_2V7:
    ESP_LOGI(TAG, "2.7V");
    break;
  case NAU7802_2V4:
    ESP_LOGI(TAG, "2.4V");
    break;
  case NAU7802_EXTERNAL:
    ESP_LOGI(TAG, "External");
    break;
  }

  sensor.setGain(NAU7802_GAIN_128);
  ESP_LOGI(TAG, "Gain set to ");
  switch (sensor.getGain()) {
  case NAU7802_GAIN_1:
    ESP_LOGI(TAG, "1x");
    break;
  case NAU7802_GAIN_2:
    ESP_LOGI(TAG, "2x");
    break;
  case NAU7802_GAIN_4:
    ESP_LOGI(TAG, "4x");
    break;
  case NAU7802_GAIN_8:
    ESP_LOGI(TAG, "8x");
    break;
  case NAU7802_GAIN_16:
    ESP_LOGI(TAG, "16x");
    break;
  case NAU7802_GAIN_32:
    ESP_LOGI(TAG, "32x");
    break;
  case NAU7802_GAIN_64:
    ESP_LOGI(TAG, "64x");
    break;
  case NAU7802_GAIN_128:
    ESP_LOGI(TAG, "128x");
    break;
  }

  sensor.setRate(NAU7802_RATE_10SPS);
  ESP_LOGI(TAG, "Conversion rate set to ");
  switch (sensor.getRate()) {
  case NAU7802_RATE_10SPS:
    ESP_LOGI(TAG, "10 SPS");
    break;
  case NAU7802_RATE_20SPS:
    ESP_LOGI(TAG, "20 SPS");
    break;
  case NAU7802_RATE_40SPS:
    ESP_LOGI(TAG, "40 SPS");
    break;
  case NAU7802_RATE_80SPS:
    ESP_LOGI(TAG, "80 SPS");
    break;
  case NAU7802_RATE_320SPS:
    ESP_LOGI(TAG, "320 SPS");
    break;
  }

  // Take 10 readings to flush out readings
  for (uint8_t i = 0; i < 10; i++) {
    while (!sensor.available())
      vTaskDelay(pdMS_TO_TICKS(1));
    sensor.read();
  }

  while (!sensor.calibrate(NAU7802_CALMOD_INTERNAL)) {
    ESP_LOGI(TAG, "Failed to calibrate internal offset, retrying!");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGI(TAG, "Calibrated internal offset");

  while (!sensor.calibrate(NAU7802_CALMOD_OFFSET)) {
    ESP_LOGI(TAG, "Failed to calibrate system offset, retrying!");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  ESP_LOGI(TAG, "Calibrated system offset");

  // main loop
  while (1) {
    esp_task_wdt_reset();
    if (sensor.available()) {
      int32_t val = sensor.read();
      ESP_LOGI(TAG, "Read ");
      ESP_LOGI(TAG, "%d", val);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  vTaskDelete(xMainTask);
}

void app_main() {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO, // select GPIO specific to your project
      .scl_io_num = I2C_MASTER_SCL_IO, // select GPIO specific to your project
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master =
          {
              .clk_speed = I2C_MASTER_FREQ_HZ, // select frequency specific to
                                               // your project
          },
      .clk_flags = 0, /*!< Optional, you can use I2C_SCLK_SRC_FLAG_*
// flags to choose i2c source clock here. */
  };

  i2c_param_config(NAU7802_I2C_PORT, &conf);
  i2c_driver_install(NAU7802_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

  assert(pdPASS == xTaskCreate(mainTask, "main", 1024, NULL, 20, &xMainTask));
  ESP_ERROR_CHECK(esp_task_wdt_add(&xMainTask));
}