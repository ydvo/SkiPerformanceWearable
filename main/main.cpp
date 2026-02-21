#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_err.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"

#include "encoder.hpp"
#include <cstdio>
#include <stdint.h>

/* Constants */

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// fsr pin
constexpr adc_channel_t fsr_pin{ADC_CHANNEL_4};

// Encoder pins (quadrature channels A and B)
constexpr gpio_num_t encoder_pin_a{GPIO_NUM_15};
constexpr gpio_num_t encoder_pin_b{GPIO_NUM_14};
constexpr uint32_t encoder_cpr{2400}; // counts per revolution

/* Components */

// Logging
espp::Logger logger({.tag = "MAIN", .level = espp::Logger::Verbosity::INFO});

// I2C
espp::I2c i2c({
    .port = i2c_port,
    .sda_io_num = i2c_sda,
    .scl_io_num = i2c_scl,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .auto_init = false,
});

// IMU
SENSORS::Imu imu(i2c);

// Encoder
SENSORS::Encoder encoder(encoder_pin_a, encoder_pin_b, encoder_cpr);

/*
 * initSystem()
 *  - initialization function, runs once before main loop
 */
void initSystem() {

  logger.info("Initializing...");

  // enable QT Stemma Port
  Common::GPIO stemma_qt_power =
      Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, Common::GPIO::Level::ON);
  logger.info("Enabled QT Stemma Port");

  // led
  LED::led red_led = LED::led(LED::RED_LED);

  // turn on led
  red_led.turn_on();

  // create i2c instance
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);

  std::error_code ec;
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
  }

  vTaskDelay(pdMS_TO_TICKS(500)); // give i2c time to startup

  // init imu
  if (!imu.init()) {
    logger.error("Imu init error");
  }
}

/*
 * mainLoop
 *  - runs repeatedly, contains update logic
 */
void mainLoop(auto dt) {
  // get imu data
  if (imu.update(dt)) {
    SENSORS::Imu::Quaternion quat = imu.get_orientation();
    // printf("%0.2f %0.2f %0.2f %0.2f", quat.w, quat.x, quat.y, quat.z);
  }

  // get encoder orientation
  static int32_t prev_count = encoder.get_count();

  int32_t count = encoder.get_count();
  if (count != prev_count) {
    float angle = encoder.get_angle();
    ESP_LOGI("ENCODER", "count=%ld  angle=%.2f deg", (long)count, angle);
    prev_count = count;
  }
}

/* Application Entry Point */
extern "C" void app_main() {
  initSystem(); // called once

  // Main event loop
  while (true) {
    // get elapsed time in between loops
    auto now{std::chrono::system_clock::now()};
    static auto t0{now};
    auto t1{now};
    std::chrono::duration<float> dt_ = t1 - t0;
    auto dt = dt_.count();
    t0 = t1;
    // logger.info("Elapsed time in float seconds: {}", dt);

    mainLoop(dt); // run repeatedly

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
