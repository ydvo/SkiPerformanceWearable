#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"
#include "soc/gpio_num.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "GPIO.hpp"
#include "i2c.hpp"
#include "imu.hpp"
#include "led.hpp"
#include "logger.hpp"
#include "flash.hpp"
#include "flash_log.hpp"
#include "ble.hpp"
#include <cstdio>
#include <stdint.h>
#include <array>
#include <vector>

// ---------------------------------------------------------------------
//  BLE Packet Structures
// ---------------------------------------------------------------------

// 24-byte live quaternion packet
#pragma pack(push, 1)
struct quat_payload_t {
    int64_t timestamp_us;   // microseconds since boot
    float   w, x, y, z;     // quaternion components
};
#pragma pack(pop)
static constexpr size_t QUAT_PAYLOAD_LEN = sizeof(quat_payload_t);  // = 24

// 244-byte bulk frame packet (for draining flash storage)
#pragma pack(push, 1)
struct bulk_frame_t {
    uint32_t seq;                           // 4 bytes - frame sequence number
    uint64_t start_us;                      // 8 bytes - timestamp of first sample
    uint64_t end_us;                        // 8 bytes - timestamp of last sample
    SENSORS::Imu::Quaternion quats[14];     // 224 bytes (14 × 16 bytes)
    // Total: 4 + 8 + 8 + 224 = 244 bytes
};
#pragma pack(pop)
static constexpr size_t BULK_FRAME_LEN = sizeof(bulk_frame_t);  // = 244

// ---------------------------------------------------------------------
//  Hardware Pin Definitions
// ---------------------------------------------------------------------

// ICM-20948 IMU
constexpr uint8_t ICM20948_ADDRESS{0x69};
constexpr uint32_t ICM20948_I2C_HZ{400000};  // 400 kHz

// I2C pins
constexpr i2c_port_t i2c_port{I2C_NUM_0};
constexpr gpio_num_t i2c_sda{GPIO_NUM_3};
constexpr gpio_num_t i2c_scl{GPIO_NUM_4};

// SPI2 pins (for flash storage)
static constexpr auto spi2_sck{GPIO_NUM_12};
static constexpr auto spi2_mosi{GPIO_NUM_11};
static constexpr auto spi2_miso{GPIO_NUM_13};
static constexpr auto flash_spi_cs{GPIO_NUM_10};

// ---------------------------------------------------------------------
//  Global Objects
// ---------------------------------------------------------------------

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
STORAGE::SpiFlashDevice spi_flash({
    .host = SPI2_HOST,
    .cs = flash_spi_cs
});

// Flash Log (stores quaternion frames)
STORAGE::FlashLog<SENSORS::Imu::Quaternion> flash_log(spi_flash);

// IMU
SENSORS::Imu imu(i2c);

// BLE Module
BLE::BleModule *ble_module_ptr = nullptr;

// State Variables
float dt = 0;
uint8_t battery_level = 100;
bool was_connected = false;

// ---------------------------------------------------------------------
//  System Initialization
// ---------------------------------------------------------------------

esp_err_t initSystem() {
    logger.info("Initializing...");

    // Enable QT Stemma Port power
    Common::GPIO stemma_qt_power =
        Common::GPIO(GPIO_NUM_7, Common::GPIO::Direction::OUTPUT, Common::GPIO::Level::ON);
    logger.info("Enabled QT Stemma Port");

    // Power-up delay for IMU
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialize LED
    LED::led red_led = LED::led(LED::RED_LED);

    // Initialize I2C
    logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);
    std::error_code ec;
    i2c.init(ec);
    if (ec) {
        logger.error("Error initializing i2c");
        return ESP_ERR_INVALID_STATE;
    }

    // I2C device scan
    logger.info("Scanning I2C devices");
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 1; address < 128; address++) {
        if (i2c.probe_device(address)) {
            found_addresses.push_back(address);
        }
    }
    logger.info("Found devices at addresses: {::#02x}", found_addresses);

    // Initialize IMU
    vTaskDelay(pdMS_TO_TICKS(100));
    bool imu_initialized = imu.init();
    
    // Verify IMU
    uint8_t whoami = imu.get_whoami();
    if (whoami != 0xEA || !imu_initialized) {
        logger.error("Failed to initialize IMU (WHOAMI: 0x{:02X})", whoami);
        return ESP_ERR_INVALID_STATE;
    }
    logger.info("IMU initialized successfully");

    red_led.turn_on();

    // Initialize SPI bus for flash
    spi_bus_config_t spi2_bus_config{
        .mosi_io_num = spi2_mosi,
        .miso_io_num = spi2_miso,
        .sclk_io_num = spi2_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_initialize(SPI2_HOST, &spi2_bus_config, SPI_DMA_CH_AUTO),
        "SYS_INIT", "Failed to initialize SPI2 bus."
    );

    // Initialize flash device
    ESP_RETURN_ON_ERROR(
        spi_flash.init(),
        "SYS_INIT", "Failed to initialize SPI flash."
    );

    // Initialize flash log
    ESP_RETURN_ON_ERROR(
        flash_log.init(),
        "SYS_INIT", "Failed to initialize flash log."
    );

    // Configure BLE
    BLE::BleModule::Config ble_config;
    ble_config.device_name = "Ski Wearable Test";
    ble_config.manufacturer_name = "ESP-CPP";
    ble_config.model_number = "ski-wearable-01";
    ble_config.serial_number = "TEST123456";

    // BLE connection callbacks
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
        return ESP_ERR_INVALID_STATE;
    }

    // Set MTU for larger packets
    NimBLEDevice::setMTU(247);

    logger.info("BLE module initialized successfully");

    // Start advertising
    if (!ble_module_ptr->start_advertising()) {
        logger.error("Failed to start advertising");
        return ESP_ERR_INVALID_STATE;
    }

    logger.info("BLE advertising started. Device name: {}", ble_config.device_name);
    logger.info("Connect with your phone's BLE scanner app");

    return ESP_OK;
}

// ---------------------------------------------------------------------
//  Main Loop
// ---------------------------------------------------------------------

esp_err_t mainLoop() {
    // Timing state
    static int64_t last_send_time_us = 0;
    static int64_t last_state_log_us = 0;
    static int64_t last_battery_update_us = 0;
    static int64_t last_waiting_log_us = 0;
    static int64_t last_imu_err_us = 0;
    
    const int64_t ACK_TIMEOUT_US = 30'000'000;  // 30 seconds

    if (!ble_module_ptr) {
        logger.error("❌ ble_module_ptr is NULL!");
        return ESP_ERR_INVALID_STATE;
    }

    int64_t now_us = esp_timer_get_time();
    bool is_connected = ble_module_ptr->is_connected();

    // ---------------------------------------------------------------------
    //  1️⃣  Read IMU and write to flash (continuous, regardless of BLE state)
    // ---------------------------------------------------------------------
    static SENSORS::Imu::Quaternion last_quat = {1, 0, 0, 0};
    
    if (imu.update(dt)) {
        last_quat = imu.get_orientation();
        
        // Write to flash log
        esp_err_t append_result = flash_log.append(last_quat, now_us);
        if (append_result != ESP_OK) {
            if (append_result == ESP_ERR_NO_MEM) {
                // Flash is full - this is expected behavior in circular buffer
                static int64_t last_full_warning_us = 0;
                if (now_us - last_full_warning_us >= 10'000'000) {  // Every 10s
                    last_full_warning_us = now_us;
                    logger.warn("⚠️  Flash log full - oldest data will be overwritten");
                }
            } else {
                logger.error("❌ Flash log append failed: {}", esp_err_to_name(append_result));
            }
        }
    } else {
        if (now_us - last_imu_err_us >= 10'000'000) {  // Every 10 seconds
            last_imu_err_us = now_us;
            logger.error("❌ IMU update failed");
        }
    }

    // ---------------------------------------------------------------------
    //  2️⃣  BLE Connection Handling
    // ---------------------------------------------------------------------
    if (is_connected) {
        if (!was_connected) {
            logger.info("✅ Device connected! Getting device info…");
            auto devs = ble_module_ptr->get_connected_device_infos();
            for (const auto &d : devs) {
                logger.info("  📱 Device: {}, RSSI: {} dBm",
                            ble_module_ptr->get_device_name(d),
                            ble_module_ptr->get_rssi(d));
            }
            was_connected = true;
            ble_module_ptr->reset_ack_on_connect();
            last_send_time_us = now_us;
        }

        // ACK/timeout handling
        bool can_send = ble_module_ptr->is_ack_received();
        bool timeout = (now_us - last_send_time_us) >= ACK_TIMEOUT_US;

        // Periodic status logging (every 5 seconds)
        if (now_us - last_state_log_us >= 5'000'000) {
            last_state_log_us = now_us;
            int64_t secs_since_last = (now_us - last_send_time_us) / 1'000'000;
            if (can_send) {
                logger.info("✅ Ready to send (ACK received or first-send pending)");
            } else {
                logger.info("⏳ Waiting for ACK – {} s since last send", secs_since_last);
            }
        }

        // ---------------------------------------------------------------------
        //  3️⃣  SEND LOOP – drain flash when ACK received
        // ---------------------------------------------------------------------
        while (can_send || timeout) {
            // Try to read a frame from flash
            STORAGE::FlashLog<SENSORS::Imu::Quaternion>::Frame flash_frame{};
            size_t frames_read = 0;
            esp_err_t err = flash_log.read(&flash_frame, 1, &frames_read);
            
            if (err != ESP_OK) {
                logger.error("❌ flash_log.read() failed: {}", esp_err_to_name(err));
                break;
            }

            if (frames_read == 0) {
                // No stored frames - send live quaternion
                quat_payload_t live_pkt{};
                live_pkt.timestamp_us = now_us;
                live_pkt.w = last_quat.w;
                live_pkt.x = last_quat.x;
                live_pkt.y = last_quat.y;
                live_pkt.z = last_quat.z;

                logger.info("📦 Sending live quaternion (24 B)");
                ble_module_ptr->notify_quaternion(
                    reinterpret_cast<const uint8_t*>(&live_pkt), QUAT_PAYLOAD_LEN);
                
                ble_module_ptr->reset_ack();
                last_send_time_us = now_us;
                break;
            }

            // We have a frame - pack into bulk format
            bulk_frame_t bulk{};
            bulk.seq = flash_frame.seq;
            bulk.start_us = flash_frame.payload.start_t_us;
            bulk.end_us = flash_frame.payload.end_t_us;
            memcpy(bulk.quats, flash_frame.payload.data, sizeof(bulk.quats));

            logger.info("📦 Sending bulk frame seq {} (244 B)", bulk.seq);
            ble_module_ptr->notify_quaternion(
                reinterpret_cast<const uint8_t*>(&bulk), BULK_FRAME_LEN);

            ble_module_ptr->reset_ack();
            last_send_time_us = now_us;

            // Re-evaluate send condition
            can_send = ble_module_ptr->is_ack_received();
            timeout = false;  // Only send one packet per timeout
        }

        // ---------------------------------------------------------------------
        //  4️⃣  Battery update (every 50 seconds)
        // ---------------------------------------------------------------------
        if (now_us - last_battery_update_us >= 50'000'000) {
            last_battery_update_us = now_us;
            ble_module_ptr->set_battery_level(battery_level);
            battery_level = (battery_level == 0) ? 100 : battery_level - 1;
            logger.info("🔋 Battery level: {}%", battery_level);
        }

    } else {
        // ---------------------------------------------------------------------
        //  5️⃣  NOT CONNECTED – cleanup & periodic waiting log
        // ---------------------------------------------------------------------
        if (was_connected) {
            logger.info("🔌 Device disconnected – resetting state");
            was_connected = false;
            battery_level = 100;
        }
        
        if (now_us - last_waiting_log_us >= 10'000'000) {  // Every 10 seconds
            last_waiting_log_us = now_us;
            logger.info("⏳ Waiting for BLE connection…");
        }
    }

    return ESP_OK;
}

// ---------------------------------------------------------------------
//  Application Entry Point
// ---------------------------------------------------------------------

extern "C" void app_main() {
    // Initialize system
    if (initSystem() != ESP_OK) {
        logger.error("Failed initializing system - halting");
        return;
    }

    // Main event loop
    int64_t last_time_us = esp_timer_get_time();
    
    while (true) {
        // Calculate delta time
        int64_t now_us = esp_timer_get_time();
        dt = (now_us - last_time_us) / 1'000'000.0f;  // Convert to seconds
        last_time_us = now_us;

        // Run main loop
        if (mainLoop() != ESP_OK) {
            logger.error("Main loop error");
        }

        // 100 Hz loop rate
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
