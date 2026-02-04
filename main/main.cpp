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
#include "esp_timer.h"

// ------------------------------------------------------------
//  Payload that will be sent in every BLE notification
// ------------------------------------------------------------
#pragma pack(push,1)               // no padding – exact 24 bytes
struct quat_payload_t {
    int64_t timestamp_us;   // microseconds since boot (esp_timer_get_time())
    float   w, x, y, z;     // quaternion, little‑endian IEEE‑754
};
#pragma pack(pop)

static constexpr size_t QUAT_PAYLOAD_LEN = sizeof(quat_payload_t);   // = 24
// ------------------------------------------------------------

// icm
constexpr uint8_t ICM20948_ADRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{100000};

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

    // *** ADD THIS: Give IMU time to power up ***
  vTaskDelay(pdMS_TO_TICKS(100));  // 100ms power-up delay

  // led
  LED::led red_led = LED::led(LED::RED_LED);

  // create i2c instance
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);

  std::error_code ec;
  i2c.init(ec); // initialize
  if (ec) {
    logger.error("Error initializing i2c");
  }

  //i2c scanner
  logger.info("Scanning I2C devices");
  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 1; address < 128; address++) {
    if (i2c.probe_device(address)) {
      found_addresses.push_back(address);
    }
  }
  logger.info("Found devices at addresses: {::#02x}", found_addresses);
  //

  // init imu
  bool imu_initialized = imu.init();
  // ensure imu is configured correctly

  vTaskDelay(pdMS_TO_TICKS(100)); // give imu time to startup before first i2c read

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

  // --------------------------------------------------------------
  //  Register the custom Quaternion service BEFORE advertising
  // --------------------------------------------------------------
  //ble_module_ptr->init_quat_service();          // <-- NEW
  NimBLEDevice::setMTU(247);                    // <-- NEW (optional but recommended)



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
  static int loop_count = 0;
  
  if (!ble_module_ptr) return;
  
  bool is_now_connected = ble_module_ptr->is_connected();
  
  // *** ALWAYS try to read IMU ***
  static SENSORS::Imu::Quaternion last_quat = {1, 0, 0, 0};
  if (imu.update(dt)) {
    last_quat = imu.get_orientation();
    
    // *** Log IMU every 10 seconds (same as send rate) ***
    if (loop_count % 100 == 0) {
      logger.info("📊 IMU: w={:.3f} x={:.3f} y={:.3f} z={:.3f}", 
                  last_quat.w, last_quat.x, last_quat.y, last_quat.z);
    }
  } else {
    // Still log errors more frequently to catch issues
    if (loop_count % 100 == 0) {
      logger.error("❌ IMU update failed");
    }
  }
  
  if (is_now_connected) {
    if (!was_connected) {
      logger.info("Device connected! Getting device info...");
      auto devices = ble_module_ptr->get_connected_device_infos();
      for (const auto &device : devices) {
        auto rssi = ble_module_ptr->get_rssi(device);
        auto name = ble_module_ptr->get_device_name(device);
        logger.info("  Device: {}, RSSI: {} dBm", name, rssi);
      }
      was_connected = true;
    }
    
    // *** Send quaternion update every 10 seconds ***
    if (loop_count % 100 == 0) {
      // Build packet
      quat_payload_t pkt;
      pkt.timestamp_us = esp_timer_get_time();
      pkt.w = last_quat.w;
      pkt.x = last_quat.x;
      pkt.y = last_quat.y;
      pkt.z = last_quat.z;
      
      // Log hex dump
      const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&pkt);
      logger.info("📦 Sending quaternion packet (24 bytes):");
      logger.info("   Timestamp: {} (0x{:016X})", pkt.timestamp_us, pkt.timestamp_us);
      logger.info("   w: {:.6f} = 0x{:08X}", pkt.w, *reinterpret_cast<const uint32_t*>(&pkt.w));
      logger.info("   x: {:.6f} = 0x{:08X}", pkt.x, *reinterpret_cast<const uint32_t*>(&pkt.x));
      logger.info("   y: {:.6f} = 0x{:08X}", pkt.y, *reinterpret_cast<const uint32_t*>(&pkt.y));
      logger.info("   z: {:.6f} = 0x{:08X}", pkt.z, *reinterpret_cast<const uint32_t*>(&pkt.z));
      
      // Full hex dump
      std::string hex_dump;
      for (size_t i = 0; i < QUAT_PAYLOAD_LEN; i++) {
        hex_dump += fmt::format("{:02X} ", bytes[i]);
        if ((i + 1) % 8 == 0) hex_dump += " ";
      }
      logger.info("   Raw hex: {}", hex_dump);
      
      // Send it
      ble_module_ptr->notify_quaternion(
          reinterpret_cast<const uint8_t*>(&pkt), QUAT_PAYLOAD_LEN);
    }
    
    // Battery update every 50 seconds
    if (loop_count % 500 == 0) {
      ble_module_ptr->set_battery_level(battery_level);
      battery_level = (battery_level == 0) ? 100 : battery_level - 1;
      logger.info("Battery: {}%", battery_level);
    }
  }
  else {
    if (was_connected) {
      logger.info("Device disconnected");
      was_connected = false;
      battery_level = 100;
    }
    
    if (loop_count % 100 == 0) {
      logger.info("Waiting for BLE connection...");
    }
  }
  
  loop_count++;
}

extern "C" void app_main() {
  initSystem();
  
  int64_t last_time_us = esp_timer_get_time();
  
  while (true) {
    int64_t now_us = esp_timer_get_time();
    dt = (now_us - last_time_us) / 1000000.0f;  // Convert to seconds
    last_time_us = now_us;
    
    mainLoop();
    
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms = 10Hz
  }
}