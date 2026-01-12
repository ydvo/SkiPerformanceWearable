#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_err.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "icm20948.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "flash.hpp"

#include <cstdio>

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// flash spi pins
static constexpr auto flash_spi_sck {GPIO_NUM_12}; 
static constexpr auto flash_spi_mosi {GPIO_NUM_11}; 
static constexpr auto flash_spi_miso {GPIO_NUM_13}; 
static constexpr auto flash_spi_cs {GPIO_NUM_10}; 

// Logging
espp::Logger logger({.tag = "Ski-wearable module", .level = espp::Logger::Verbosity::INFO});

// I2C
espp::I2c i2c({
    .port = i2c_port,
    .sda_io_num = i2c_sda,
    .scl_io_num = i2c_scl,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .auto_init = false,
});

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
  i2c.init(ec);

  // i2c scanner
  // logger.info("Scanning I2C devices");
  // std::vector<uint8_t> found_addresses;
  // for (uint8_t address = 1; address < 128; address++) {
  //   if (i2c.probe_device(address)) {
  //     found_addresses.push_back(address);
  //   }
  // }
  // logger.info("Found devices at addresses: {::#02x}", found_addresses);
  //

  logger.info("Creating IMU");

  red_led.turn_on();

  STORAGE::SpiFlashDevice spi_flash ({
    .sck_port = flash_spi_sck,
    .mosi_port = flash_spi_mosi,
    .miso_port = flash_spi_miso,
    .spi_cs_port = flash_spi_cs,
  }); 

  spi_flash.init(); 
}

/*
 * mainLoop
 *  - runs repeatedly, contains update logic
 */
void mainLoop() {}

/* Application Entry Point */
extern "C" void app_main() {
  initSystem(); // called once

  // Main event loop
  while (true) {
    // delay
    auto now{std::chrono::system_clock::now()};
    static auto t0{now};
    auto t1{now};
    std::chrono::duration<float> dt_ = t1 - t0;
    float dt = dt_.count();
    t0 = t1;
    logger.info("Elapsed time in float seconds: {}", dt);

    mainLoop(); // run repeatedly

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
