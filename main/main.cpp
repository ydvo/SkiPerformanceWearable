#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "driver/spi_common.h"

#include "esp_err.h"
#include "esp_check.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "flash.hpp"
#include "flash_log.hpp"

#include <cstdio>
#include <stdint.h>
#include <array>

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

// SPI Flash Device
STORAGE::SpiFlashDevice spi_flash ({
  .host = SPI2_HOST,
  .cs = flash_spi_cs
});

// Flash Log
STORAGE::FlashLog<SENSORS::Imu::Quaternion> flash_log(
  spi_flash
);
// IMU
SENSORS::Imu imu(i2c);

// Filescope Vars
float dt = 0;

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
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
    return ESP_ERR_INVALID_STATE; 
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  // init imu
  bool imu_initialized = imu.init();
  // ensure imu is configured correctly
  if (!imu_initialized) {
    logger.error("Failed to initialize imu"); 
    return ESP_ERR_INVALID_STATE;
  }

  vTaskDelay(pdMS_TO_TICKS(100)); // give imu time to startup before first i2c read

  uint8_t test = imu.get_whoami();
  if (test != 0xEA) {
    logger.error("Invalid imu device id {}", test);
  }

  logger.info("Initialized imu"); 

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
esp_err_t mainLoop() {
  static std::array<STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame, 10> frame_read_buf {};
  static std::array<SENSORS::Imu::Quaternion, 10 * 14> quat_write_buf {};
  static auto quat_write_ptr = quat_write_buf.begin(); 

  if (imu.update(dt)) {
    // get timestamp
    auto now{std::chrono::system_clock::now()};
    auto timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count(); 

    SENSORS::Imu::Quaternion quat = imu.get_orientation();
    *quat_write_ptr = quat; 

    ESP_RETURN_ON_ERROR(
      flash_log.append(quat, timestamp_us), 
      "MAIN_LOOP", "Failed appending to the flash log"
    ); 

    quat_write_ptr++;

    if (quat_write_ptr == quat_write_buf.end()) {
      quat_write_ptr = quat_write_buf.begin();

      size_t frames_read; 
      ESP_RETURN_ON_ERROR(
        flash_log.read(frame_read_buf.begin(), frame_read_buf.size(), &frames_read), 
        "MAIN_LOOP", "Failed to read the flash log."
      );

      logger.info("Read {} frames out of {} frames.", frames_read, frame_read_buf.size()); 

      {
        auto w_it = quat_write_buf.begin(); 
        STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame frame; 
        SENSORS::Imu::Quaternion read_quat; 
        SENSORS::Imu::Quaternion expected_quat;

        for (size_t i = 0; i != frames_read; ++i) {
          frame = frame_read_buf[i]; 
          for (size_t j = 0; j != 14; ++j) {
            read_quat = frame.payload.data[j]; 
            expected_quat = *w_it; 

            if (
              read_quat.w != expected_quat.w || 
              read_quat.x != expected_quat.x || 
              read_quat.y != expected_quat.y || 
              read_quat.z != expected_quat.z
            ) {
              ESP_LOGI("MAIN_LOOP", "Data mismatch. Expected {w: %0.3f, x: %0.3f, y: %0.3f, z: %0.3f}. Received {w: %0.3f, x: %0.3f, y: %0.3f, z: %0.3f}", 
                expected_quat.w, expected_quat.x, expected_quat.y, expected_quat.z, read_quat.w, read_quat.x, read_quat.y, read_quat.z); 
            }

            w_it++; 
          }
        }
      } 
    }

  }

  return ESP_OK;
}

/* Application Entry Point */
extern "C" void app_main() {
  if (initSystem() != ESP_OK) {
    logger.error("Failed initializing a system."); 
  } // called once

  // Main event loop
  while (true) {
    // delay
    auto now{std::chrono::system_clock::now()};
    static auto t0{now};
    auto t1{now};
    std::chrono::duration<float> dt_ = t1 - t0;
    dt = dt_.count();
    t0 = t1;

    mainLoop(); // run repeatedly

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
