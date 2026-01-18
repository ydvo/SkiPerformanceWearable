#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "driver/spi_common.h"

#include "esp_err.h"
#include "esp_check.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "icm20948.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "flash.hpp"
#include "flash_log.hpp"

#include <cstdio>

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// spi 2 pins
static constexpr auto spi2_sck {GPIO_NUM_12}; 
static constexpr auto spi2_mosi {GPIO_NUM_11}; 
static constexpr auto spi2_miso {GPIO_NUM_13}; 

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

// SPI Flash Device
STORAGE::SpiFlashDevice spi_flash ({
  .host = SPI2_HOST,
  .cs = flash_spi_cs
});

// Flash Log
STORAGE::FlashLog<STORAGE::ImuValue> flash_log(
  spi_flash
);

/*
 * initSystem()
 *  - initialization function, runs once before main loop
 */
esp_err_t initSystem() {

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
  logger.info("Creating IMU");
  red_led.turn_on();

  spi_bus_config_t spi2_bus_config {
    .mosi_io_num = spi2_mosi, 
    .miso_io_num = spi2_miso, 
    .sclk_io_num = spi2_sck,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1, 
  }; 

  ESP_RETURN_ON_ERROR(
    spi_bus_initialize(SPI2_HOST, &spi2_bus_config, SPI_DMA_CH_AUTO), 
    "SYS_INIT", "Failed to initialize SPI_2 bus."
  ); 

  ESP_RETURN_ON_ERROR(
    spi_flash.init(), 
    "SYS_INIT", "Failed to initialize SPI flash."
  ); 

  ESP_RETURN_ON_ERROR(
    flash_log.init(), 
    "SYS_INIT", "Failed to initialize flash log."
  );

  return ESP_OK; 
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

    auto microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count(); 

    STORAGE::ImuValue sample {
      .w = 1.0f, 
      .x = 1.0f, 
      .y = 1.0f, 
      .z = 1.0f
    }; 

    if (flash_log.append(sample, microseconds_since_epoch) != ESP_OK) {
      logger.info("Failed appending to the flash log."); 
    }

    mainLoop(); // run repeatedly

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}
