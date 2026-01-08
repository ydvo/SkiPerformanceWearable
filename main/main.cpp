#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "i2c.hpp"
#include "logger.hpp"

#include <cstdio>

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

extern "C" void app_main() {
  // enable QT Stemma Port
  gpio_num_t vcc = GPIO_NUM_7;
  gpio_set_direction(vcc, GPIO_MODE_OUTPUT);
  gpio_set_level(vcc, 1);

  espp::Logger logger({.tag = "I2C", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  ESP_LOGI("MAIN", "Starting Ski Wearable...");

  espp::I2c i2c({
    .port = I2C_NUM_0,
    .sda_io_num = (gpio_num_t) GPIO_NUM_3,
    .scl_io_num = (gpio_num_t) GPIO_NUM_4,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
  });

  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 1; address < 128; address++) {
    if (i2c.probe_device(address)) {
      logger.info("Found device at addresses: {}", address);
      found_addresses.push_back(address);
    }
  }

  logger.info("Found devices at addresses: {::#02x}", found_addresses);

  // Main event loop
  while (true) {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1s);
  }
}
