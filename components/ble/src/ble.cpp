#include "ble.hpp"
#include "NimBLEDevice.h"
#include "esp_check.h"
#include "esp_log.h"

namespace BLE {
static const char *TAG = "BLE";

static const NimBLEUUID QUAT_SVC_UUID{"16fd3a8f-f37e-4155-8ebf-654df4d3f700"};
static const NimBLEUUID QUAT_CHAR_UUID{"16fd3a8f-f37e-4155-8ebf-654df4d3f701"};
static const NimBLEUUID QUAT_ACK_UUID{"16fd3a8f-f37e-4155-8ebf-654df4d3f702"};

BleModule::BleModule() : config_(), battery_level_(100), initialized_(false) {}

BleModule::BleModule(const Config &config)
    : config_(config), battery_level_(100), initialized_(false) {}

BleModule::~BleModule() {
  if (initialized_) {
    ble_gatt_server_.deinit();
  }
}

esp_err_t BleModule::init() {
  if (initialized_)
    return ESP_OK;

  ble_gatt_server_.set_log_level(config_.log_level);
  setup_callbacks();

  ESP_RETURN_ON_FALSE(ble_gatt_server_.init(config_.device_name), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to initialize GATT Server");
  ble_gatt_server_.set_advertise_on_disconnect(true);
  setup_security();

  ble_gatt_server_.start_services();
  setup_device_info();

  ESP_RETURN_ON_ERROR(init_quat_service(), TAG, "Failed to initialize quaternion service.");
  ESP_RETURN_ON_ERROR(setup_advertising(), TAG, "Failed to setup advertising");

  initialized_ = true;
  ESP_LOGI(TAG, "BLE module initialized");
  return ESP_OK;
}

void BleModule::reconfigure(const Config &config) {
  if (initialized_) {
    ESP_LOGE(TAG, "reconfigure() called after init() - ignoring");
    return;
  }
  config_ = config;
  ESP_LOGI(TAG, "BleModule reconfigured (device_name: %s)", config_.device_name.c_str());
}

esp_err_t BleModule::start_advertising() {
  if (!initialized_) {
    ESP_LOGE(TAG, "BLE not initialized, cannot start advertising");
    return ESP_ERR_INVALID_STATE;
  }

  ESP_RETURN_ON_FALSE(ble_gatt_server_.start_advertising(), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to start advertising");

  ESP_LOGI(TAG, "BLE advertising started");

  return ESP_OK;
}

esp_err_t BleModule::stop_advertising() {
  ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG,
                      "BLE not initialized, cannot stop advertising");

  ble_gatt_server_.stop_advertising();
  ESP_LOGI(TAG, "BLE advertising stopped");
  return ESP_OK;
}

bool BleModule::is_connected() const {
  return ble_gatt_server_.is_connected();
}

esp_err_t BleModule::set_battery_level(uint8_t level) {
  ESP_RETURN_ON_FALSE(initialized_, ESP_ERR_INVALID_STATE, TAG,
                      "BLE Module must be initialized to set battery level");

  ESP_RETURN_ON_FALSE(level <= 100, ESP_ERR_INVALID_ARG, TAG,
                      "Battery level %d exceeds 100%%, ignoring", level);

  battery_level_ = level;

  auto &battery_service = ble_gatt_server_.battery_service();
  battery_service.set_battery_level(battery_level_);
  ESP_LOGD(TAG, "Battery level updated to %d%%", battery_level_);
  return ESP_OK;
}

uint8_t BleModule::get_battery_level() const {
  return battery_level_;
}

espp::BleGattServer &BleModule::get_server() {
  return ble_gatt_server_;
}

std::vector<NimBLEConnInfo> BleModule::get_connected_device_infos() const {
  return ble_gatt_server_.get_connected_device_infos();
}

int BleModule::get_rssi(const NimBLEConnInfo &info) const {
  return ble_gatt_server_.get_connected_device_rssi(info);
}

std::string BleModule::get_connected_device_name(const NimBLEConnInfo &info) const {
  return ble_gatt_server_.get_connected_device_name(info);
}

void BleModule::set_log_level(espp::Logger::Verbosity level) {
  config_.log_level = level;
  ble_gatt_server_.set_log_level(level);
}

void BleModule::setup_callbacks() {
  espp::BleGattServer::Callbacks callbacks;

  callbacks.connect_callback = [this](NimBLEConnInfo &conn) {
    ESP_LOGI(TAG, "Device connected – resetting ACK state");
    reset_ack_on_connect();
    if (config_.on_connect) {
      config_.on_connect(conn);
    }
  };

  callbacks.disconnect_callback = [this](NimBLEConnInfo &conn_info,
                                         espp::BleGattServer::DisconnectReason reason) {
    ESP_LOGI(TAG, "Device disconnected: %d – disarming ACK", reason);
    ack_armed_ = false;
    ack_received_ = false;
    if (config_.on_disconnect) {
      config_.on_disconnect(conn_info, reason);
    }
  };

  callbacks.authentication_complete_callback = [this](const NimBLEConnInfo &conn_info) {
    ESP_LOGI(TAG, "Device authenticated");
    if (config_.on_authenticated) {
      config_.on_authenticated(conn_info);
    }
  };

  callbacks.get_passkey_callback = []() {
    ESP_LOGI(TAG, "Getting passkey");
    return NimBLEDevice::getSecurityPasskey();
  };

  callbacks.confirm_passkey_callback = [](const NimBLEConnInfo &conn_info, uint32_t passkey) {
    ESP_LOGI(TAG, "Confirming passkey: %lu", (unsigned long)passkey);
    NimBLEDevice::injectConfirmPasskey(conn_info, passkey == NimBLEDevice::getSecurityPasskey());
  };

  ble_gatt_server_.set_callbacks(callbacks);
}

void BleModule::setup_security() {
  // Configure security
  ble_gatt_server_.set_security(config_.bonding, config_.mitm, config_.secure_connections);

  // Set passkey and I/O capabilities
  NimBLEDevice::setSecurityPasskey(config_.passkey);
  ble_gatt_server_.set_io_capabilities(config_.io_capabilities);

  // Set key distribution
  ble_gatt_server_.set_init_key_distribution(config_.init_key_dist);
  ble_gatt_server_.set_resp_key_distribution(config_.resp_key_dist);

  ESP_LOGI(TAG, "Security configured - bonding: %d, mitm: %d, secure_conn: %d", config_.bonding,
           config_.mitm, config_.secure_connections);
}

void BleModule::setup_device_info() {
  auto &device_info_service = ble_gatt_server_.device_info_service();

  // Set PnP ID
  device_info_service.set_pnp_id(config_.vendor_source, config_.vendor_id, config_.product_id,
                                 config_.product_version);

  // Set device info strings
  device_info_service.set_manufacturer_name(config_.manufacturer_name);
  device_info_service.set_model_number(config_.model_number);
  device_info_service.set_serial_number(config_.serial_number);
  device_info_service.set_software_version(config_.software_version);
  device_info_service.set_firmware_version(config_.firmware_version);
  device_info_service.set_hardware_version(config_.hardware_version);

  ESP_LOGI(TAG, "Device info configured");
}

esp_err_t BleModule::setup_advertising() {
  // Set the advertising data
  espp::BleGattServer::AdvertisedData adv_data;
  espp::BleGattServer::AdvertisedData scan_rsp;

  // Set flags for general discoverable mode
  ESP_RETURN_ON_FALSE(adv_data.setFlags(BLE_HS_ADV_F_DISC_GEN), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to set advertised data flag");

  ESP_RETURN_ON_FALSE(adv_data.addServiceUUID(QUAT_SVC_UUID), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to add quaternion service uuid to advertised data");

  ESP_RETURN_ON_FALSE(scan_rsp.setName(config_.device_name), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to set advertised data name");
  ESP_RETURN_ON_FALSE(scan_rsp.setAppearance((uint16_t)espp::BleAppearance::GENERIC_COMPUTER),
                      ESP_ERR_INVALID_STATE, TAG, "Failed to set advertised data appearance");
  ESP_RETURN_ON_FALSE(scan_rsp.addTxPower(), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to add advertised data tx power");

  ble_gatt_server_.set_advertisement_data(adv_data);
  ble_gatt_server_.set_scan_response_data(scan_rsp);

  ESP_RETURN_ON_FALSE(ble_gatt_server_.start(), ESP_ERR_INVALID_STATE, TAG,
                      "Failed to start ble gatt server");

  ESP_LOGI(TAG, "Advertising configured");
  ESP_LOGI(TAG, "Quaternion characteristic handle %d", quat_char_->getHandle());
  ESP_LOGI(TAG, "ACK characteristic handle %d", ack_char_->getHandle());

  return ESP_OK;
}

/* ----------------------------------------------------------------
 *  Create the custom Quaternion service + NOTIFY characteristic
 * ---------------------------------------------------------------- */
esp_err_t BleModule::init_quat_service() {
  NimBLEServer *pServer = NimBLEDevice::getServer();
  ESP_RETURN_ON_FALSE(pServer != nullptr, ESP_ERR_INVALID_STATE, TAG, "Failed to get server");

  ESP_LOGI(TAG, "Service UUID: %s, Quat Char UUID: %s, ACK Char UUID: %s",
           QUAT_SVC_UUID.toString().c_str(), QUAT_CHAR_UUID.toString().c_str(),
           QUAT_ACK_UUID.toString().c_str());

  ESP_LOGI(TAG, "Creating service...");

  NimBLEService *svc = pServer->createService(QUAT_SVC_UUID);
  ESP_RETURN_ON_FALSE(svc != nullptr, ESP_ERR_INVALID_STATE, TAG, "Failed to create service.");

  ESP_LOGI(TAG, "Service created");

  ESP_LOGI(TAG, "Creating quaternion characteristic...");

  quat_char_ =
      svc->createCharacteristic(QUAT_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ);

  ESP_RETURN_ON_FALSE(quat_char_ != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Failed to create quaternion characteristic");

  ESP_LOGI(TAG, "Quaternion characteristic created");

  ESP_LOGI(TAG, "Creating CCCD descriptor");

  NimBLEDescriptor *cccd = quat_char_->createDescriptor(
      NimBLEUUID((uint16_t)0x2902), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

  ESP_RETURN_ON_FALSE(cccd != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Failed to create cccd descriptor, returned nullptr");

  ESP_LOGI(TAG, "CCCD descriptor created");

  cccd->setCallbacks(&quat_cccd_cb_);
  ESP_LOGI(TAG, "CCCD callbacks set");

  ESP_LOGI(TAG, "Creating user descriptor for quaternion");
  NimBLEDescriptor *userDescriptor =
      quat_char_->createDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ);

  ESP_RETURN_ON_FALSE(userDescriptor != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Failed to create user descriptor");

  userDescriptor->setValue("Quaternion Data (NOTIFY)");
  ESP_LOGI(TAG, "User Descriptor set");

  ESP_LOGI(TAG, "Creating ACK characteristic");
  ack_char_ = svc->createCharacteristic(QUAT_ACK_UUID, NIMBLE_PROPERTY::WRITE);

  ESP_RETURN_ON_FALSE(ack_char_ != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Failed to create ack characteristic");

  ESP_LOGI(TAG, "ACK characteristic created");

  ack_char_->setCallbacks(&quat_ack_cb_);
  ESP_LOGI(TAG, "ACK callbacks set");

  ESP_LOGI(TAG, "Creating User Descriptor for ACK...");
  NimBLEDescriptor *ackUserDescriptor =
      ack_char_->createDescriptor(NimBLEUUID((uint16_t)0x2901), NIMBLE_PROPERTY::READ);

  ESP_RETURN_ON_FALSE(ackUserDescriptor != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Failed to create ack user descriptor, returned nullptr");

  ackUserDescriptor->setValue("Acknowledgement (WRITE Ack struct with frame_seq)");
  ESP_LOGI(TAG, "ACK User Description set");

  ESP_LOGI(TAG, "Starting service");
  svc->start();
  ESP_LOGI(TAG, "Quaternion service started");

  return ESP_OK;
}

/* ----------------------------------------------------------------
 *  Send a quaternion notification
 * ---------------------------------------------------------------- */
esp_err_t BleModule::notify_quaternion(const uint8_t *payload, size_t len) {
  ESP_RETURN_ON_FALSE(quat_char_ != nullptr, ESP_ERR_INVALID_STATE, TAG,
                      "Quaternion characteristic is not initialized");

  ESP_RETURN_ON_FALSE(is_connected(), ESP_ERR_INVALID_STATE, TAG, "BLE Server not connected");

  quat_char_->setValue(payload, len);
  ESP_RETURN_ON_FALSE(quat_char_->notify(), ESP_ERR_INVALID_STATE, TAG, "Failed to notify");

  ESP_LOGD(TAG, "notify_quaternion: sent %u bytes", (unsigned)len);

  return ESP_OK;
}

bool BleModule::quat_notify_enabled() const {
  return (quat_char_ != nullptr) && quat_notifications_enabled_.load();
}

/* ----------------------------------------------------------------
 *  ACK helpers
 * ---------------------------------------------------------------- */

void BleModule::arm_ack(uint16_t seq) {
  // Clear any stale ACK from before this send, then record what we expect,
  // then arm the wait.  Order matters: write ack_received_ = false BEFORE
  // setting ack_armed_ = true, so the callback cannot see armed=true while
  // ack_received_ is still a leftover true from the previous round.
  ack_received_.store(false, std::memory_order_relaxed);
  ack_expected_seq_.store(seq, std::memory_order_relaxed);
  ack_armed_.store(true, std::memory_order_release);
  ESP_LOGD(TAG, "arm_ack: waiting for ACK seq %u", (unsigned)seq);
}

bool BleModule::is_ack_received() const {
  return ack_received_.load(std::memory_order_acquire);
}

void BleModule::reset_ack_on_connect() {
  ack_armed_ = false;
  ack_received_ = false;
  ack_expected_seq_ = 0;
  ESP_LOGD(TAG, "reset_ack_on_connect: ACK state cleared");
}

/* ----------------------------------------------------------------
 *  CCCD callback – client enabled/disabled notifications
 * ---------------------------------------------------------------- */
void BleModule::QuatCCCDCallbacks::onWrite(NimBLEDescriptor *pDescriptor,
                                           NimBLEConnInfo &connInfo) {
  ESP_RETURN_VOID_ON_FALSE(pDescriptor != nullptr, ESP_ERR_INVALID_STATE, TAG,
                           "Failed CCCD onWrite due to null descriptor");

  uint16_t value = pDescriptor->getValue<uint16_t>();
  bool enabled = (value & 0x0001) != 0;
  ESP_LOGI(TAG, "Quaternion CCCD Write: 0x%04x -> Notifications %s", value,
           enabled ? "enabled" : "disabled");

  ESP_RETURN_VOID_ON_FALSE(parent_ != nullptr, ESP_ERR_INVALID_STATE, TAG,
                           "Failed CCCD onWrite due to null parent");

  parent_->quat_notifications_enabled_.store(enabled, std::memory_order_relaxed);

  if (!enabled) {
    // Disabling notifications: disarm so uploadTask does not stay stuck
    parent_->ack_armed_.store(false, std::memory_order_relaxed);
    parent_->ack_received_.store(false, std::memory_order_relaxed);
    ESP_LOGI(TAG, "CCCD: notifications disabled – ACK state cleared");
  } else {
    ESP_LOGI(TAG, "CCCD: notifications enabled");
  }
}

/* ----------------------------------------------------------------
 *  ACK write callback – client acknowledges a received frame
 * ---------------------------------------------------------------- */
void BleModule::AckCharCallbacks::onWrite(NimBLECharacteristic *pCharacteristic,
                                          NimBLEConnInfo &connInfo) {
  ESP_RETURN_VOID_ON_FALSE(pCharacteristic != nullptr, ESP_ERR_INVALID_STATE, TAG,
                           "Failed ACK onWrite due to null characteristic");

  ESP_RETURN_VOID_ON_FALSE(parent_ != nullptr, ESP_ERR_INVALID_STATE, TAG,
                           "Failed ACK onWrite due to null parent");

  // Parse payload – must be exactly sizeof(Ack) = 2 bytes
  std::string raw = pCharacteristic->getValue();
  ESP_LOGD(TAG, "ACK onWrite: received %u bytes", (unsigned)raw.size());

  if (raw.size() < sizeof(Ack)) {
    ESP_LOGW(TAG, "ACK onWrite: payload too short (%u bytes, expected %u) – ignoring",
             (unsigned)raw.size(), (unsigned)sizeof(Ack));
    return;
  }

  Ack ack;
  memcpy(&ack, raw.data(), sizeof(Ack));

  uint16_t expected = parent_->ack_expected_seq_.load(std::memory_order_relaxed);
  bool armed = parent_->ack_armed_.load(std::memory_order_acquire);

  ESP_LOGI(TAG, "ACK onWrite: received seq=%u  expected=%u  armed=%d",
           (unsigned)ack.frame_seq, (unsigned)expected, (int)armed);

  if (!armed) {
    // No send is in flight – this is a late/spurious ACK from a previous transaction
    ESP_LOGW(TAG, "ACK onWrite: received ACK seq=%u but not armed (stale ACK) – ignoring",
             (unsigned)ack.frame_seq);
    return;
  }

  if (ack.frame_seq != expected) {
    // Wrong sequence – client is acknowledging an old frame
    ESP_LOGW(TAG, "ACK onWrite: seq mismatch – got %u, expected %u – ignoring",
             (unsigned)ack.frame_seq, (unsigned)expected);
    return;
  }

  if (parent_->ack_received_.load(std::memory_order_relaxed)) {
    // Already processed a valid ACK for this frame
    ESP_LOGW(TAG, "ACK onWrite: duplicate valid ACK for seq=%u – ignoring",
             (unsigned)ack.frame_seq);
    return;
  }

  // Valid, in-sequence, first ACK for this frame
  parent_->ack_armed_.store(false, std::memory_order_relaxed);
  parent_->ack_received_.store(true, std::memory_order_release);
  ESP_LOGI(TAG, "ACK onWrite: ACK seq=%u accepted", (unsigned)ack.frame_seq);
}

} // namespace BLE
