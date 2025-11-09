#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GPIO.hpp"
#include "I2C.hpp"
#include "imu.hpp"
#include "led.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

/* Constants */
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr int ICM20948_I2C_HZ{400000};

constexpr const char *TAG = "MAIN";

constexpr gpio_num_t sda{GPIO_NUM_3};
constexpr gpio_num_t scl{GPIO_NUM_4};

/* Main function */
extern "C" void app_main() {
  ESP_LOGI(TAG, "Starting Ski Wearable...");

  /* GPIO */
  // enable QT Stemma Port
  Common::GPIO stemma_qt_power = Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, 1);
  // led
  LED::led red_led = LED::led(LED::RED_LED);

  /* I2C */

  Common::I2C i2c(I2C_NUM_0, sda, scl);

  // init icm20948
  i2c.init_device(ICM20948_ADRESS, ICM20948_I2C_HZ);

  // try read
  uint8_t buffer = 0;
  if (i2c.reg_read(ICM20948_ADRESS, 0x00, &buffer, 1) == ESP_OK) {
    printf("Who Am I: %d\n\r", buffer);
  }

  /* I2C Scan */
  // ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");
  //
  // for (uint8_t addr = 1; addr < 0x7F; ++addr) {
  //   uint8_t data;
  //   i2c.init_device(addr, ICM20948_I2C_HZ);
  //   esp_err_t ret = i2c.reg_read(addr, 0x00, &data, 1); // try reading WHOAMI
  //   if (ret == ESP_OK) {
  //     ESP_LOGI("I2C_SCAN", "Device found at 0x%02X, WHOAMI=0x%02X", addr, data);
  //   }
  // }
  //
  // ESP_LOGI("I2C_SCAN", "Scan complete");

  /* IMU */
  // setup config
  Imu::Config imu_config{.device_address = ICM20948_ADRESS,
                         .write = [&i2c](uint8_t addr, const uint8_t *data, size_t len) -> bool {
                           return i2c.write(addr, data, len);
                         },
                         .read = [&i2c](uint8_t addr, uint8_t *data, size_t len) -> bool {
                           return i2c.read(addr, data, len);
                         },
                         .imu_config = {.accelerometer_range = ACCELEROMETER_RANGE,
                                        .gyroscope_range = GYROSCOPE_RANGE,
                                        .accelerometer_sample_rate_divider = 9, // 100 Hz
                                        .gyroscope_sample_rate_divider = 9,
                                        .magnetometer_mode = MAGNETOMETER_MODE},
                         .madgwick_beta = 0.1f,
                         .auto_init = true};

  // init device
  Imu imu(imu_config);
  ESP_LOGI(TAG, "IMU initialized successfully");

  auto t0 = std::chrono::steady_clock::now();

  red_led.turn_on();
  /* Main event loop */
  while (true) {

    // vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed
    //
    // red_led.turn_off();
    // vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed

    auto t1 = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(t1 - t0).count();
    t0 = t1;

    if (!imu.update(dt)) {
      ESP_LOGE(TAG, "IMU update failed");
      std::this_thread::sleep_for(50ms);
      continue;
    }

    // Get raw and Madgwick-filtered orientation
    auto raw = imu.get_raw();
    auto orientation = imu.get_orientation();

    // Log raw IMU data
    ESP_LOGI(TAG, "Accel: %.2f %.2f %.2f  Gyro: %.2f %.2f %.2f  Temp: %.2f", raw.accel.x,
             raw.accel.y, raw.accel.z, raw.gyro.x, raw.gyro.y, raw.gyro.z, raw.temperature);

    // // Log Madgwick orientation
    // ESP_LOGI(TAG, "Madgwick Orientation (rad): Roll %.3f Pitch %.3f Yaw %.3f", orientation.roll,
    //          orientation.pitch, orientation.yaw);

    std::this_thread::sleep_for(15ms); // ~66 Hz
  }
}
