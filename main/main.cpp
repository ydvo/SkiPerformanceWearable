#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_err.h"

#include "GPIO.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "ble.hpp"

#include <cstdio>
#include <stdint.h>

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};

// i2c pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

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

// BLE - declare at file scope, initialize in initSystem()
BLE::BleModule *ble_module_ptr = nullptr;

// Filescope Vars
float dt = 0;

uint8_t battery_level = 100;
bool was_connected = false;

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

  // init imu
  bool imu_initialized = imu.init();
  // ensure imu is configured correctly

  vTaskDelay(pdMS_TO_TICKS(10)); // give imu time to startup before first i2c read

  uint8_t test = imu.get_whoami();
  if (test != 0xEA && !imu_initialized) {
    logger.error("Could not initialize imu");
  } else {
    logger.info("Imu initialized");
  }

  red_led.turn_on();

  // Configure BLE with custom settings
  BLE::BleModule::Config ble_config;
  ble_config.device_name = "Ski Wearable Test";
  ble_config.manufacturer_name = "ESP-CPP";
  ble_config.model_number = "ski-wearable-01";
  ble_config.serial_number = "TEST123456";

  // Set up connection callbacks
  ble_config.on_connect = [](NimBLEConnInfo &info) {
    printf("BLE: Client connected!\n");
  };

  ble_config.on_disconnect = [](NimBLEConnInfo &info,
                                   espp::BleGattServer::DisconnectReason reason) {
    printf("BLE: Client disconnected\n");
  };

  ble_config.on_authenticated = [](const NimBLEConnInfo &info) {
    printf("BLE: Client authenticated successfully\n");
  };

  // Create and initialize BLE module
  ble_module_ptr = new BLE::BleModule(ble_config);

  if (!ble_module_ptr->init()) {
    logger.error("Failed to initialize BLE module");
    return;
  }

  logger.info("BLE module initialized successfully");

  // Start advertising
  if (!ble_module_ptr->start_advertising()) {
    logger.error("Failed to start advertising");
    return;
  }

  logger.info("BLE advertising started. Device name: {}", ble_config.device_name);
  logger.info("Connect with your phone's BLE scanner app");
}

/*
 * mainLoop
 *  - runs repeatedly, contains update logic
 */
void mainLoop() {
  // if (imu.update(dt)) {
  //   SENSORS::Imu::Quaternion quat = imu.get_orientation();
  //   printf("DATA %0.4f %0.4f %0.4f %0.4f\r\n", quat.w, quat.x, quat.y, quat.z);
  // }

  // Check connection status (only if BLE is initialized)
  if (ble_module_ptr && ble_module_ptr->is_connected()) {
    if (!was_connected) {
      // Just connected
      logger.info("Device connected! Getting device info...");

      auto devices = ble_module_ptr->get_connected_device_infos();
      for (const auto &device : devices) {
        auto rssi = ble_module_ptr->get_rssi(device);
        auto name = ble_module_ptr->get_device_name(device);
        logger.info("  Device: {}, RSSI: {} dBm", name, rssi);
      }

      was_connected = true;
    }

    // Update battery level (simulate discharge)
    ble_module_ptr->set_battery_level(battery_level);
    battery_level = (battery_level == 0) ? 100 : battery_level - 1;

    logger.info("Battery level updated: {}%", battery_level);

  } else if (ble_module_ptr) {
    if (was_connected) {
      // Just disconnected
      logger.info("Device disconnected. Resuming advertising...");
      was_connected = false;
      battery_level = 100; // Reset battery for next connection
    }

    // Not connected - show waiting message periodically
    static int wait_count = 0;
    if (wait_count++ % 10 == 0) {
      logger.info("Waiting for BLE connection...");
    }
  }
}

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
    dt = dt_.count();
    t0 = t1;
    // logger.info("Elapsed time in float seconds: {}", dt);

    mainLoop(); // run repeatedly

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
