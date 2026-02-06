#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"

#include "esp_err.h"

#include "imu.hpp"       // ← Move this FIRST (before ble.hpp)
#include "flash_log.hpp" // ← Also early
#include "GPIO.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "ble.hpp"       // ← Move this AFTER imu.hpp

#include <cstdio>
#include <stdint.h>
#include "esp_timer.h"

// ---------------------------------------------------------------------
//  Flash‑log (defined elsewhere in the project)
// ---------------------------------------------------------------------
extern STORAGE::FlashLog<SENSORS::Imu::Quaternion> flash_log;

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
  // --------------------------------------------------------------
  //  Timing / state helpers (unchanged from your previous code)
  // --------------------------------------------------------------
  static int64_t last_send_time_us       = 0;
  static int64_t last_state_log_us       = 0;
  static int64_t last_battery_update_us  = 0;
  static int64_t last_waiting_log_us     = 0;
  const int64_t ACK_TIMEOUT_US = 30'000'000;   // 30 s

  if (!ble_module_ptr) {
    logger.error("❌ ble_module_ptr is NULL!");
    return;
  }

  int64_t now_us = esp_timer_get_time();
  bool    is_connected = ble_module_ptr->is_connected();

  // --------------------------------------------------------------
  //  1️⃣  Always read the live IMU (keeps orientation fresh)
  // --------------------------------------------------------------
  static SENSORS::Imu::Quaternion last_quat = {1, 0, 0, 0};
  if (imu.update(dt)) {
    last_quat = imu.get_orientation();
  } else {
    static int64_t last_imu_err_us = 0;
    if (now_us - last_imu_err_us >= 10'000'000) {   // every 10 s
      last_imu_err_us = now_us;
      logger.error("❌ IMU update failed");
    }
  }

  // --------------------------------------------------------------
  //  2️⃣  Connection handling
  // --------------------------------------------------------------
  if (is_connected) {
    if (!was_connected) {
      logger.info("✅ Device connected! Getting device info…");
      auto devs = ble_module_ptr->get_connected_device_infos();
      for (const auto &d : devs) {
        logger.info("  📱 Device: {}, RSSI: {} dBm",
                    ble_module_ptr->get_device_name(d),
                    ble_module_ptr->get_rssi(d));
      }
      was_connected = true;
      ble_module_ptr->reset_ack_on_connect();   // first‑send flag = true, ack = false
      last_send_time_us = now_us;
    }

    // --------------------------------------------------------------
    //  3️⃣  ACK / timeout handling
    // --------------------------------------------------------------
    bool can_send = ble_module_ptr->is_ack_received();
    bool timeout  = (now_us - last_send_time_us) >= ACK_TIMEOUT_US;

    // --------------------------------------------------------------
    //  4️⃣  Periodic status logging (every 5 s)
    // --------------------------------------------------------------
    if (now_us - last_state_log_us >= 5'000'000) {   // 5 s
      last_state_log_us = now_us;
      int64_t secs_since_last = (now_us - last_send_time_us) / 1'000'000;
      if (can_send)
        logger.info("✅ Ready to send (ACK received or first‑send pending)");
      else
        logger.info("⏳ Waiting for ACK – {} s since last send", secs_since_last);
    }

     // --------------------------------------------------------------
    //  5️⃣  SEND LOOP – keep sending as long as the phone ACKs
    // --------------------------------------------------------------
    while (can_send || timeout) {
      // ------------------------------------------------------------
      // 5a) Try to get a full frame from the flash‑log
      // ------------------------------------------------------------
      STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame flash_frame{};
      size_t frames_read = 0;
      esp_err_t err = flash_log.read(&flash_frame, 1, &frames_read);

      if (err != ESP_OK) {
        ESP_LOGE("MAIN_LOOP", "flash_log.read() failed: %s",
                 esp_err_to_name(err));
        // On a flash error we break out of the burst loop – we’ll try again later.
        break;
      }

      if (frames_read == 0) {
        // ------------------------------------------------------------
        // 5b) No stored frame → fall back to live quaternion.
        // ------------------------------------------------------------
        quat_payload_t live_pkt{};
        live_pkt.timestamp_us = now_us;
        live_pkt.w = last_quat.w;
        live_pkt.x = last_quat.x;
        live_pkt.y = last_quat.y;
        live_pkt.z = last_quat.z;

        const uint8_t *b = reinterpret_cast<const uint8_t*>(&live_pkt);
        std::string hex;
        for (size_t i = 0; i < QUAT_PAYLOAD_LEN; ++i) {
          hex += fmt::format("{:02X} ", b[i]);
        }
        logger.info("📦 Live quaternion (24 B): {}", hex);

        ble_module_ptr->notify_quaternion(
            reinterpret_cast<const uint8_t*>(&live_pkt), QUAT_PAYLOAD_LEN);
        ble_module_ptr->reset_ack();               // clear ACK flag
        last_send_time_us = now_us;
        break;                                     // Nothing else to send now
      }

      // ------------------------------------------------------------
      // 5c) We HAVE a frame – pack it into the 244‑byte bulk payload
      // ------------------------------------------------------------
      bulk_frame_t bulk{};
      bulk.seq      = flash_frame.seq;
      bulk.start_us = flash_frame.payload.start_t_us;
      bulk.end_us   = flash_frame.payload.end_t_us;
      // copy the 14 quaternions (each quaternion = 16 B)
      memcpy(bulk.quats,
             flash_frame.payload.data,
             sizeof(bulk.quats));

      // ------------------------------------------------------------
      // 5d) Debug print – keep it reasonably short (first 64 B)
      // ------------------------------------------------------------
      const uint8_t *buf = reinterpret_cast<const uint8_t*>(&bulk);
      std::string hex;
      for (size_t i = 0; i < sizeof(bulk) && i < 64; ++i) {
        hex += fmt::format("{:02X} ", buf[i]);
        if ((i + 1) % 16 == 0) hex += "\n";
      }
      uint32_t seq_copy = bulk.seq;  // Copy to avoid packed field reference issue
      logger.info("📦 Bulk frame seq {} ({} B) – first 64 B:\n{}",
          seq_copy, sizeof(bulk), hex);

      // ------------------------------------------------------------
      // 5e) Send the bulk packet (still using the quaternion characteristic)
      // ------------------------------------------------------------
      ble_module_ptr->notify_quaternion(
          reinterpret_cast<const uint8_t*>(&bulk), sizeof(bulk));

      // ------------------------------------------------------------
      // 5f) Clean up state for the next iteration
      // ------------------------------------------------------------
      ble_module_ptr->reset_ack();               // ready for the next ACK
      last_send_time_us = now_us;

      // ------------------------------------------------------------
      // 5g) Re‑evaluate the sending condition:
      //      - If the phone already wrote another ACK, `can_send` becomes true.
      //      - If we hit the timeout condition we stop after the first burst.
      // ------------------------------------------------------------
      can_send = ble_module_ptr->is_ack_received();
      timeout  = false;   // after a successful burst we don’t want to send again
    }

    // --------------------------------------------------------------
    // 6️⃣  Battery update (every 50 s, unchanged)
    // --------------------------------------------------------------
    if (now_us - last_battery_update_us >= 50'000'000) {
      last_battery_update_us = now_us;
      ble_module_ptr->set_battery_level(battery_level);
      battery_level = (battery_level == 0) ? 100 : battery_level - 1;
      logger.info("🔋 Battery level: {}%", battery_level);
    }
  } else {
    // --------------------------------------------------------------
    // 7️⃣  NOT CONNECTED – clean‑up & periodic waiting log
    // --------------------------------------------------------------
    if (was_connected) {
      logger.info("🔌 Device disconnected – resetting state");
      was_connected = false;
      battery_level = 100;
    }
    if (now_us - last_waiting_log_us >= 10'000'000) {   // every 10 s
      last_waiting_log_us = now_us;
      logger.info("⏳ Waiting for BLE connection…");
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