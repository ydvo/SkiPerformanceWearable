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
  static int64_t last_send_time_us = 0;
  static int64_t last_timeout_check_us = 0;
  const int64_t TIMEOUT_US = 30000000;  // 30 seconds
  
  if (!ble_module_ptr) {
    logger.error("❌ ble_module_ptr is NULL!");
    return;
  }
  
  int64_t now_us = esp_timer_get_time();
  bool is_now_connected = ble_module_ptr->is_connected();
  
  // *** ALWAYS read IMU ***
  static SENSORS::Imu::Quaternion last_quat = {1, 0, 0, 0};
  if (imu.update(dt)) {
    last_quat = imu.get_orientation();
  } else {
    static int64_t last_imu_error_log = 0;
    if (now_us - last_imu_error_log >= 10000000) {  // Every 10s
      last_imu_error_log = now_us;
      logger.error("❌ IMU update failed");
    }
  }
  
  if (is_now_connected) {
    // Just connected?
    if (!was_connected) {
      logger.info("✅ Device connected! Getting device info...");
      auto devices = ble_module_ptr->get_connected_device_infos();
      for (const auto &device : devices) {
        auto rssi = ble_module_ptr->get_rssi(device);
        auto name = ble_module_ptr->get_device_name(device);
        logger.info("  📱 Device: {}, RSSI: {} dBm", name, rssi);
      }
      was_connected = true;
      ble_module_ptr->reset_ack_on_connect();
      last_send_time_us = now_us;
      last_timeout_check_us = now_us;
      logger.info("🔄 ACK state reset for new connection");
    }
    
    // Check if we can send (ACK received or first send)
    bool can_send = ble_module_ptr->is_ack_received();
    bool timeout_occurred = (now_us - last_send_time_us) >= TIMEOUT_US;
    
    // Log state periodically
    static int64_t last_state_log = 0;
    if (now_us - last_state_log >= 5000000) {  // Every 5s
      last_state_log = now_us;
      int64_t time_since_last_send_s = (now_us - last_send_time_us) / 1000000;
      
      if (can_send) {
        logger.info("✅ Ready to send (ACK received or first send)");
      } else {
        logger.info("⏳ Waiting for ACK... ({} seconds since last send)", time_since_last_send_s);
      }
    }
    
    // Send if ACK received OR timeout
    if (can_send || timeout_occurred) {
      if (timeout_occurred && !can_send) {
        logger.warn("⚠️  TIMEOUT: 30s passed with no ACK - sending anyway!");
      }
      
      // Build packet
      quat_payload_t pkt;
      pkt.timestamp_us = now_us;
      pkt.w = last_quat.w;
      pkt.x = last_quat.x;
      pkt.y = last_quat.y;
      pkt.z = last_quat.z;
      
      // Log hex dump
      const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&pkt);
      logger.info("📦 Sending quaternion packet (24 bytes):");
      logger.info("   Timestamp: {} (0x{:016X})", pkt.timestamp_us, pkt.timestamp_us);
      logger.info("   w: {:.6f} x: {:.6f} y: {:.6f} z: {:.6f}", 
                  pkt.w, pkt.x, pkt.y, pkt.z);
      
      std::string hex_dump;
      for (size_t i = 0; i < QUAT_PAYLOAD_LEN; i++) {
        hex_dump += fmt::format("{:02X} ", bytes[i]);
        if ((i + 1) % 8 == 0) hex_dump += " ";
      }
      logger.info("   Raw hex: {}", hex_dump);
      
      // Send it
      ble_module_ptr->notify_quaternion(
          reinterpret_cast<const uint8_t*>(&pkt), QUAT_PAYLOAD_LEN);
      
      // Reset ACK flag and update timestamp
      ble_module_ptr->reset_ack();
      last_send_time_us = now_us;
      
      logger.info("✉️  Packet sent - waiting for ACK from phone...");
    }
    
    // Battery update every 50 seconds
    static int64_t last_battery_update = 0;
    if (now_us - last_battery_update >= 50000000) {
      last_battery_update = now_us;
      ble_module_ptr->set_battery_level(battery_level);
      battery_level = (battery_level == 0) ? 100 : battery_level - 1;
      logger.info("🔋 Battery: {}%", battery_level);
    }
  }
  else {  // NOT CONNECTED
    if (was_connected) {
      logger.info("❌ Device disconnected");
      was_connected = false;
      battery_level = 100;
    }
    
    static int64_t last_waiting_log = 0;
    if (now_us - last_waiting_log >= 10000000) {  // Every 10s
      last_waiting_log = now_us;
      logger.info("⏳ Waiting for BLE connection...");
    }
  }
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