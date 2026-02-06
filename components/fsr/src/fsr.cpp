/* fsr.cpp
 *  implementation of simple force sensitice resistor driver
 */

#include "fsr.hpp"
#include "adc_types.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"

using namespace SENSORS;

static const char *TAG = "FSR";

fsr::fsr(adc_channel_t channel)
    : channels_{{.unit = ADC_UNIT_1, .channel = channel, .attenuation = ADC_ATTEN_DB_12}},
      adc_({.unit = ADC_UNIT_1, .channels = {channels_}}), pressure_threshold_(0),
      is_pressed_(false), baseline_mv_(0), max_mv_(3300) {
  ESP_LOGI(TAG, "FSR initialized on ADC channel %d", channel);
}

fsr::~fsr() {
  // Nothing for now
}

float fsr::read() {
  auto result = adc_.read_mv(channels_[0]);
  if (result.has_value()) {
    float voltage = result.value();

    // Check threshold if set
    if (pressure_threshold_ > 0 && voltage >= pressure_threshold_) {
      if (!is_pressed_) {
        ESP_LOGI(TAG, "Pressure detected: %.2f mV (threshold: %.2f mV)", voltage,
                 pressure_threshold_);
      }
      is_pressed_ = true;
    } else if (pressure_threshold_ > 0) {
      if (is_pressed_) {
        ESP_LOGD(TAG, "Pressure released: %.2f mV", voltage);
      }
      is_pressed_ = false;
    }

    return voltage;
  }
  ESP_LOGW(TAG, "Failed to read ADC value");
  return 0.0f;
}

int fsr::read_raw() {
  auto result = adc_.read_raw(channels_[0]);
  if (result.has_value()) {
    return result.value();
  }
  ESP_LOGW(TAG, "Failed to read raw ADC value");
  return 0;
}

void fsr::set_pressure_threshold(float threshold_mv) {
  pressure_threshold_ = threshold_mv;
  ESP_LOGI(TAG, "Pressure threshold set to %.2f mV", threshold_mv);
}

bool fsr::check_pressure(float threshold_mv) {
  float voltage = read();
  return voltage >= threshold_mv;
}

void fsr::calibrate_baseline() {
  ESP_LOGI(TAG, "Calibrating baseline (no pressure)...");
  // Take average of 10 samples for baseline
  float sum = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += read();
    // Small delay between samples
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  baseline_mv_ = sum / samples;
  ESP_LOGI(TAG, "Baseline calibrated: %.2f mV", baseline_mv_);
}

void fsr::calibrate_max() {
  ESP_LOGI(TAG, "Calibrating maximum pressure...");
  // Take average of 10 samples for max pressure
  float sum = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    sum += read();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  max_mv_ = sum / samples;
  ESP_LOGI(TAG, "Maximum calibrated: %.2f mV", max_mv_);
}

void fsr::set_calibration(float baseline_mv, float max_mv) {
  baseline_mv_ = baseline_mv;
  max_mv_ = max_mv;
  ESP_LOGI(TAG, "Calibration set manually - baseline: %.2f mV, max: %.2f mV", baseline_mv_,
           max_mv_);
}

float fsr::read_percentage() {
  float voltage = read();
  if (max_mv_ <= baseline_mv_) {
    ESP_LOGW(TAG, "Invalid calibration: max (%.2f) <= baseline (%.2f)", max_mv_, baseline_mv_);
    return 0.0f; // Invalid calibration
  }

  float percentage = ((voltage - baseline_mv_) / (max_mv_ - baseline_mv_)) * 100.0f;

  // Clamp to 0-100%
  if (percentage < 0.0f)
    percentage = 0.0f;
  if (percentage > 100.0f)
    percentage = 100.0f;

  return percentage;
}
