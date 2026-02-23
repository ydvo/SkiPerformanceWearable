#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_attr.h"
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
constexpr uint32_t encoder_cpr{10000}; // counts per revolution

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

// Boot button (used to zero encoder)
Common::GPIO boot_button(GPIO_NUM_0, Common::GPIO::Direction::INPUT, Common::GPIO::Level::ON,
                         Common::GPIO::PULLUP);

/* boot_button_isr
 *  - ISR for boot button, zeros the encoder count
 */
static void IRAM_ATTR boot_button_isr(void *arg) {
  auto *enc = static_cast<SENSORS::Encoder *>(arg);
  enc->reset();
}

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

  // init encoder
  encoder.init();

  // register boot button interrupt to zero encoder on press
  boot_button.set_interrupt(Common::GPIO::INTERRUPT_FALLING_EDGE, boot_button_isr, &encoder);

  // turn on led
  red_led.turn_on();
}

/*
 * mainLoop
 *  - runs repeatedly, contains update logic
 */
void mainLoop(auto dt) {
  // update imu
  imu.update(dt);

  // get imu euler angles and encoder angle
  auto euler = imu.get_euler();
  float enc_angle = encoder.get_angle();

  // output CSV: DATA,<timestamp_ms>,<encoder_angle>,<pitch>,<roll>,<yaw>
  uint32_t ts = xTaskGetTickCount() * portTICK_PERIOD_MS;
  printf("DATA,%lu,%.2f,%.2f,%.2f,%.2f\n", (unsigned long)ts, enc_angle, euler.pitch, euler.roll,
         euler.yaw);
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

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
