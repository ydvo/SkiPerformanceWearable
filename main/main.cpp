#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GPIO.hpp"
#include "I2C.hpp"
#include "imu.hpp"
#include "led.hpp"

using namespace SENSORS;

/* Constants */
constexpr const char *TAG = "MAIN";

// i2c pins
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
    printf("Who Am I: %x\n\r", buffer);
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
  // init device
  Imu imu(i2c);
  ESP_LOGI(TAG, "IMU initialized successfully");

  red_led.turn_on();

  /* Main event loop */
  while (true) {
    // vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed
    //
    // red_led.turn_off();
    // vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed
  }
}
