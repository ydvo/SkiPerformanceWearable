#pragma once

#include "ble_gatt_server.hpp"
#include "ble_gatt_server_menu.hpp"
#include "logger.hpp"

#include "NimBLEDevice.h"

#include <string>
#include <functional>
#include <vector>
#include <cstdint>

namespace BLE {

constexpr size_t SAMPLES_PER_FRAME = 8; 

struct __attribute__((packed)) FrameHeader {
  uint16_t frame_seq; 
  uint16_t sample_count; 
  uint16_t payload_len; 
  uint16_t flags; 
};

struct __attribute__((packed)) FrameSample {
  uint64_t timestamp; 
  float w;
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) Frame {
  FrameHeader header; 
  FrameSample payload[SAMPLES_PER_FRAME];
};

struct __attribute__((packed)) Ack {
  uint16_t frame_seq; 
}; 

static_assert(sizeof(FrameSample) == 24, "Frame sample expected to be 24 bytes packed.");
static_assert(sizeof(FrameHeader) == 8, "Frame header expected to be 8 bytes packed."); 
static_assert(sizeof(Frame) <= 244, "Frame size expected to fit into 1 notify packet.");  

/* BLE Module Class */
class BleModule {
public:
  /* Configuration structure */
  struct Config {
    // Device info
    std::string device_name{"Ski Performance Wearable"};
    std::string manufacturer_name{"ESP-CPP"};
    std::string model_number{"ski-wearable-01"};
    std::string serial_number{"0000000001"};
    std::string software_version{"1.0.0"};
    std::string firmware_version{"1.0.0"};
    std::string hardware_version{"1.0.0"};

    bool bonding{true};
    bool mitm{false};
    bool secure_connections{true};
    uint32_t passkey{123456};
    uint8_t io_capabilities{BLE_HS_IO_NO_INPUT_OUTPUT};
    uint8_t init_key_dist{BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID};
    uint8_t resp_key_dist{BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO};
    
    // PnP ID
    uint8_t vendor_source{0x01};
    uint16_t vendor_id{0xCafe};
    uint16_t product_id{0xFace};
    uint16_t product_version{0x0100};
    
    // Callbacks
    std::function<void(NimBLEConnInfo&)> on_connect;
    std::function<void(NimBLEConnInfo&, espp::BleGattServer::DisconnectReason)> on_disconnect;
    std::function<void(const NimBLEConnInfo&)> on_authenticated;
  };

  /**
   * @brief Constructor with configuration
   * @param config Configuration structure
   */
  explicit BleModule(const Config& config);

  /**
   * @brief Default constructor with default configuration
   */
  BleModule();

  /**
   * @brief Destructor - deinitializes BLE
   */
  ~BleModule();

  /**
   * @brief Initialize the BLE module
   * @return true if successful, false otherwise
   */
  esp_err_t init();

  /**
   * @brief Start advertising
   * @return true if successful, false otherwise
   */
  esp_err_t start_advertising();

  /**
   * @brief Stop advertising
   * @return true if successful, false otherwise
   */
  esp_err_t stop_advertising();

  /**
   * @brief Check if a device is connected
   * @return true if connected, false otherwise
   */
  bool is_connected() const;

  /**
   * @brief Update battery level
   * @param level Battery level (0-100%)
   */
  esp_err_t set_battery_level(uint8_t level);

  /**
   * @brief Get the current battery level
   * @return Battery level (0-100%)
   */
  uint8_t get_battery_level() const;

  /**
   * @brief Get the GATT server instance
   * @return Reference to the BleGattServer
   */
  espp::BleGattServer& get_server();

  /**
   * @brief Get connection info for connected devices
   * @return Vector of connection info
   */
  std::vector<NimBLEConnInfo> get_connected_device_infos() const;

  /**
   * @brief Get RSSI for a connected device
   * @param info Connection info
   * @return RSSI value in dBm
   */
  int get_rssi(const NimBLEConnInfo& info) const;

  /**
   * @brief Get name of connected device
   * @param info Connection info
   * @return Device name
   */
  std::string get_device_name(const NimBLEConnInfo& info) const;

  /**
   * @brief Set log level
   * @param level Log verbosity level
   */
  void set_log_level(espp::Logger::Verbosity level);
      
  /** Register the custom Quaternion service/characteristic. Call once before advertising. */
  esp_err_t init_quat_service();
  
  /** Send a quaternion notification (payload must be 24 bytes). */
  esp_err_t notify_quaternion(const uint8_t *payload, size_t len);
  
  /** Returns true if a connected client has enabled notifications on the quat char. */
  bool quat_notify_enabled() const;
  
  /** Check if ACK was received (or if first send is pending) */
  bool is_ack_received() const;
  
  /** Clear ACK flag after sending (call after notify_quaternion) */
  void reset_ack();
  
  /** Reset state on new connection */
  void reset_ack_on_connect();

private:
  class QuatCCCDCallbacks: public NimBLEDescriptorCallbacks {
    BleModule *parent_; 
  public:
    explicit QuatCCCDCallbacks(BleModule *parent): parent_(parent) {}; 
    void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override;
  }; 

  class AckCharCallbacks: public NimBLECharacteristicCallbacks {
    BleModule *parent_; 
  public: 
    explicit AckCharCallbacks(BleModule *parent): parent_(parent) {}; 
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
  };

  NimBLECharacteristic *quat_char_ = nullptr;
  NimBLECharacteristic *ack_char_ = nullptr; 

  QuatCCCDCallbacks quat_cccd_cb_ {this}; 
  AckCharCallbacks quat_ack_cb_ {this}; 

  bool quat_notifications_enabled_ = false;
  bool ack_received_ = false;
  bool first_send_pending_ = false;
  
  Config config_;
  espp::BleGattServer ble_gatt_server_;
  uint8_t battery_level_{100};
  bool initialized_{false};

  void setup_callbacks();
  void setup_security();
  void setup_device_info();
  esp_err_t setup_advertising();
};

} // namespace BLE