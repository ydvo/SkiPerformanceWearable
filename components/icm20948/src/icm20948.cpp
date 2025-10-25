#include "icm20948.hpp"
#include "esp_log.h"

static const char* TAG = "ICM20948";

ICM20948::ICM20948() {}

void ICM20948::initialize() {
    ESP_LOGI(TAG, "ICM20948 initialized");
}

AccelData ICM20948::getAcceleration() {
    // Replace with real sensor read
    return {0.1f, 0.0f, -9.8f};
}
