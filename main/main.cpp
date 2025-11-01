#include "GPIO.hpp"
#include "I2C.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "GPIO.hpp"
#include "bluetooth_manager.hpp"
#include "force_sensor.hpp"
#include "hal/i2c_types.h"
#include "icm20948.hpp"
#include "led.hpp"
#include "sensor_fusion.hpp"
#include "soc/gpio_num.h"
#include <cstdio>

// led
LED::led red_led = LED::led(LED::RED_LED);

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr int ICM20948_I2C_HZ{400000};

extern "C" void app_main() {
  // enable QT Stemma Port
  Common::GPIO stemma_qt_power = Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, 1);

  gpio_num_t sda{GPIO_NUM_3};
  gpio_num_t scl{GPIO_NUM_4};

  ESP_LOGI("MAIN", "Starting Ski Wearable...");

  Common::I2C i2c(I2C_NUM_0, sda, scl);

  i2c.init_device(ICM20948_ADRESS, ICM20948_I2C_HZ);

  // try read
  uint8_t buffer = 0;
  i2c.reg_read(ICM20948_ADRESS, 0x00, &buffer, 1);
  printf("Who Am I: %d\n\r", buffer);

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

  // Main event loop
  while (true) {

    red_led.turn_on();
    vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed

    red_led.turn_off();
    vTaskDelay(pdMS_TO_TICKS(1000)); // allow other tasks & WDT feed
  }
}
