#include "lcm20948.hpp"
#include "esp_log.h"

static const char* TAG = "LCM20948";

LCM20948::LCM20948() {}

void LCM20948::initialize() {
    ESP_LOGI(TAG, "LCM20948 initialized");
}

AccelData LCM20948::getAcceleration() {
    // Replace with real sensor read
    return {0.1f, 0.0f, -9.8f};
}
