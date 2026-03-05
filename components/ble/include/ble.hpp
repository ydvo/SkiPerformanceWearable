/**
 * @file ble.hpp
 * @ingroup communication
 * @brief BLE module interface and GATT service definitions.
 *
 * Provides a NimBLE‑based GATT server exposing a custom Quaternion
 * service for streaming IMU data, battery level characteristic, and an
 * ACK characteristic used by the host to acknowledge received frames.
 * All structures are packed to match the BLE packet layout.
 */
#pragma once

#include "ble_gatt_server.hpp"
#include "ble_gatt_server_menu.hpp"
#include "logger.hpp"

#include "NimBLEDevice.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace BLE {

constexpr size_t SAMPLES_PER_FRAME = 8;

/**
 * @brief Header for a BLE data frame.
 *
 * Contains sequence number, sample count, payload length in bytes, and
 * flags for future extensions. All fields are 16‑bit little‑endian as per
 * BLE GATT specification.
 */
struct __attribute__((packed)) FrameHeader {
  uint16_t frame_seq;
  uint16_t sample_count;
  uint16_t payload_len;
  uint16_t flags;
};

/**
 * @brief Single quaternion sample transmitted over BLE.
 *
 * `timestamp` is a 64‑bit microsecond value obtained from `esp_timer_get_time()`.
 * Quaternion components are stored as 32‑bit floats in order (w, x, y, z).
 */
struct __attribute__((packed)) FrameSample {
  uint64_t timestamp;
  float w;
  float x;
  float y;
  float z;
};

/**
 * @brief BLE notification frame containing a header and an array of quaternion samples.
 *
 * The payload size is limited to `SAMPLES_PER_FRAME` entries to fit within a
 * single BLE notification (max 244 bytes with current packing).
 */
struct __attribute__((packed)) Frame {
  FrameHeader header;
  FrameSample payload[SAMPLES_PER_FRAME];
};

/**
 * @brief ACK payload sent by the central device.
 *
 * Contains the `frame_seq` of the last successfully received frame. The
 * peripheral uses this to confirm transmission before sending the next batch.
 */
struct __attribute__((packed)) Ack {
  uint16_t frame_seq;
};

static_assert(sizeof(FrameSample) == 24, "Frame sample expected to be 24 bytes packed.");
static_assert(sizeof(FrameHeader) == 8, "Frame header expected to be 8 bytes packed.");
static_assert(sizeof(Frame) <= 244, "Frame size expected to fit into 1 notify packet.");

/**
 * @brief High‑level BLE peripheral manager.
 *
 * Wraps a NimBLE GATT server exposing a custom Quaternion service for streaming
 * orientation data, a battery level characteristic, and an ACK characteristic to
 * provide reliable transfer semantics. Handles advertising, connection callbacks,
 * security configuration, and log level control.
 */
class BleModule {
public:
  /* Configuration structure */
   /**
    * @brief Configuration parameters for the BLE module.
    *
    * All fields have sensible defaults; modify as needed before calling
    * `init()`. Callback members may be left empty if not required.
    */
   struct Config {
     // Device info
     std::string device_name{"Ski Performance Wearable"};      ///< Local device name advertised.
     std::string manufacturer_name{"ESP-CPP"};                ///< Manufacturer string for PnP ID.
     std::string model_number{"ski-wearable-01"};            ///< Model identifier.
     std::string serial_number{"0000000001"};                ///< Unique serial number.
     std::string software_version{"1.0.0"};                  ///< Software version string.
     std::string firmware_version{"1.0.0"};                  ///< Firmware version string.
     std::string hardware_version{"1.0.0"};                  ///< Hardware revision string.

     bool bonding{true};                                        ///< Enable pairing bonding.
     bool mitm{false};                                          ///< Man‑in‑the‑middle protection flag.
     bool secure_connections{true};                             ///< Use BLE Secure Connections.
     uint32_t passkey{123456};                                   ///< Numeric passkey for pairing.
     uint8_t io_capabilities{BLE_HS_IO_NO_INPUT_OUTPUT};        ///< IO capabilities for pairing.
     uint8_t init_key_dist{BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID}; ///< Key distribution from initiator.
     uint8_t resp_key_dist{BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID}; ///< Key distribution from responder.
     espp::Logger::Verbosity log_level{espp::Logger::Verbosity::INFO}; ///< Log verbosity for BLE module.

     // PnP ID
     uint8_t vendor_source{0x01};                               ///< Vendor source (0x01 = Bluetooth SIG).
     uint16_t vendor_id{0xCafe};                                 ///< Vendor ID assigned by SIG.
     uint16_t product_id{0xFace};                                ///< Product ID.
     uint16_t product_version{0x0100};                           ///< Product version.

     // Callbacks
     std::function<void(NimBLEConnInfo &)> on_connect;          ///< Invoked on BLE connection.
     std::function<void(NimBLEConnInfo &, espp::BleGattServer::DisconnectReason)> on_disconnect; ///< Invoked on BLE disconnection.
     std::function<void(const NimBLEConnInfo &)> on_authenticated; ///< Invoked after authentication.
     /**
      * @brief Invoked when the central writes to the control characteristic.
      *
      * The argument is the command byte received:
      *   - 0x01 : toggle recording (start if idle, stop if running)
      */
     std::function<void(uint8_t cmd)> on_control_command;      ///< Invoked on control characteristic write.
   };

  /**
   * @brief Constructor with configuration
   * @param config Configuration structure
   */
  explicit BleModule(const Config &config);

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
   * @brief Deinitialize the BLE module.
   *
   * Stops advertising, disconnects all peers, and tears down the NimBLE stack.
   * After calling this the module cannot be re-initialized without a reboot.
   * @return ESP_OK on success.
   */
  esp_err_t deinit();

  /**
   * @brief Apply a new configuration before init() is called.
   *        Must not be called after init().
   * @param config New configuration to apply
   */
  void reconfigure(const Config &config);

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
  espp::BleGattServer &get_server();

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
  int get_rssi(const NimBLEConnInfo &info) const;

  /**
   * @brief Get name of connected device
   * @param info Connection info
   * @return Device name
   */
  std::string get_connected_device_name(const NimBLEConnInfo &info) const;

  /**
   * @brief Set log level
   * @param level Log verbosity level
   */
  void set_log_level(espp::Logger::Verbosity level);

  /** Register the custom Quaternion service/characteristic. Call once before advertising. */
  esp_err_t init_quat_service();

  /** Send a quaternion notification. */
  esp_err_t notify_quaternion(const uint8_t *payload, size_t len);

  /**
   * @brief Notify the central that all flash data has been uploaded.
   *
   * Writes 0x01 to the upload-status characteristic and sends a BLE
   * notification. The central should subscribe to this characteristic
   * (enable CCCD) to receive the event. Returns ESP_ERR_INVALID_STATE if
   * BLE is not connected or the characteristic is not initialized.
   */
  esp_err_t notify_upload_complete();

  /** Returns true if a connected client has enabled notifications on the quat char. */
  bool quat_notify_enabled() const;

  /**
   * @brief Arm the ACK wait for the frame that was just sent.
   *        Call this AFTER notify_quaternion so the ACK callback cannot fire
   *        before ack_armed_ is set.
   * @param seq The frame_seq just sent - the incoming Ack.frame_seq must match.
   */
  void arm_ack(uint16_t seq);

  /**
   * @brief Returns true once a valid ACK matching the armed seq has been received,
   *        or if arm_ack() has not yet been called (idle / not armed).
   */
  bool is_ack_received() const;

  /** Reset ACK state on new connection */
  void reset_ack_on_connect();

  /** Simple getters **/
  std::string get_name() const {
    return config_.device_name;
  }

private:
  class QuatCCCDCallbacks : public NimBLEDescriptorCallbacks {
    BleModule *parent_;

  public:
    explicit QuatCCCDCallbacks(BleModule *parent) : parent_(parent) {};
    void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override;
  };

  class AckCharCallbacks : public NimBLECharacteristicCallbacks {
    BleModule *parent_;

  public:
    explicit AckCharCallbacks(BleModule *parent) : parent_(parent) {};
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
  };

  class ControlCharCallbacks : public NimBLECharacteristicCallbacks {
    BleModule *parent_;

  public:
    explicit ControlCharCallbacks(BleModule *parent) : parent_(parent) {};
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override;
  };

  NimBLECharacteristic *quat_char_ = nullptr;
  NimBLECharacteristic *ack_char_ = nullptr;
  NimBLECharacteristic *ctrl_char_ = nullptr;
  NimBLECharacteristic *status_char_ = nullptr;

  QuatCCCDCallbacks quat_cccd_cb_{this};
  AckCharCallbacks quat_ack_cb_{this};
  ControlCharCallbacks ctrl_cb_{this};

  std::atomic<bool> quat_notifications_enabled_{false};

  // ack_armed_: set by arm_ack() after a notify is sent, cleared once a valid ACK arrives.
  std::atomic<bool> ack_armed_{false};
  // ack_received_: set to true by AckCharCallbacks when a valid seq-matched ACK arrives.
  std::atomic<bool> ack_received_{false};
  // The frame_seq we expect the client to echo in its ACK write.
  std::atomic<uint16_t> ack_expected_seq_{0};

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
