#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "icm20948.hpp"
#include "force_sensor.hpp"
#include "bluetooth_manager.hpp"
#include "sensor_fusion.hpp"

extern "C" void app_main() {
    ESP_LOGI("MAIN", "Starting Ski Wearable...");

    // IMU imu;
    // ForceSensor force;
    // BluetoothManager bt;
    // SensorFusion fusion;
    //
    // imu.initialize();
    // force.initialize();
    // bt.startAdvertising();

    // Main event loop
    while (true) {

      ESP_LOGI("test", "looping!");
      vTaskDelay(pdMS_TO_TICKS(100));  // allow other tasks & WDT feed
                                 //
        // auto accel = imu.getAcceleration();
        // auto forceVal = force.readForce();
        // auto fused = fusion.compute(accel, forceVal);
        //
        // bt.sendData(fused);
        //
        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}
