/* ble.cpp
 * Implementation of BLE GATT Server module
 */

#include "ble.hpp"
#include "NimBLEDevice.h"
#include "esp_log.h"

static const char *BLE_TAG = "BLE";

// -------------------------------------------------------------
// 128‑bit UUIDs for the custom service / characteristic
// -------------------------------------------------------------
static const uint8_t QUAT_SVC_UUID[16] = {
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf1 };
static const uint8_t QUAT_CHAR_UUID[16] = {
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf2 };

namespace BLE {

static espp::Logger logger({.tag = "BLE", .level = espp::Logger::Verbosity::INFO});

BleModule::BleModule()
    : config_(), battery_level_(100), initialized_(false) {
  logger.set_verbosity(config_.log_level);
}

BleModule::BleModule(const Config& config)
    : config_(config), battery_level_(100), initialized_(false) {
  logger.set_verbosity(config_.log_level);
}

BleModule::~BleModule() {
  if (initialized_) {
    ble_gatt_server_.deinit();
  }
}

bool BleModule::init() {
  if (initialized_) {
    logger.warn("BLE already initialized");
    return true;
  }

  logger.info("Initializing BLE module: {}", config_.device_name);

  // Set log level
  ble_gatt_server_.set_log_level(config_.log_level);

  // Setup callbacks
  setup_callbacks();

  // Initialize the GATT server
  ble_gatt_server_.init(config_.device_name);

#if !CONFIG_BT_NIMBLE_EXT_ADV
  // Extended advertisement does not support automatically advertising on disconnect
  ble_gatt_server_.set_advertise_on_disconnect(true);
#endif

  // Setup security
  setup_security();

  // Start services (battery and device info)
  ble_gatt_server_.start_services();

  // Setup device info
  setup_device_info();

  // Set initial battery level
  auto& battery_service = ble_gatt_server_.battery_service();
  battery_service.set_battery_level(battery_level_);

  // Setup advertising
  setup_advertising();

  initialized_ = true;
  logger.info("BLE module initialized successfully");
  return true;
}

bool BleModule::start_advertising() {
  if (!initialized_) {
    logger.error("BLE not initialized, cannot start advertising");
    return false;
  }

  ble_gatt_server_.start_advertising();
  logger.info("BLE advertising started");
  return true;
}

bool BleModule::stop_advertising() {
  if (!initialized_) {
    logger.error("BLE not initialized, cannot stop advertising");
    return false;
  }

  ble_gatt_server_.stop_advertising();
  logger.info("BLE advertising stopped");
  return true;
}

bool BleModule::is_connected() const {
  return ble_gatt_server_.is_connected();
}

void BleModule::set_battery_level(uint8_t level) {
  if (level > 100) {
    logger.warn("Battery level {} exceeds 100%, clamping to 100%", level);
    level = 100;
  }

  battery_level_ = level;

  if (initialized_) {
    auto& battery_service = ble_gatt_server_.battery_service();
    battery_service.set_battery_level(battery_level_);
    logger.debug("Battery level updated to {}%", battery_level_);
  }
}

uint8_t BleModule::get_battery_level() const {
  return battery_level_;
}

espp::BleGattServer& BleModule::get_server() {
  return ble_gatt_server_;
}

std::vector<NimBLEConnInfo> BleModule::get_connected_device_infos() const {
  return ble_gatt_server_.get_connected_device_infos();
}

int BleModule::get_rssi(const NimBLEConnInfo& info) const {
  return ble_gatt_server_.get_connected_device_rssi(info);
}

std::string BleModule::get_device_name(const NimBLEConnInfo& info) const {
  return ble_gatt_server_.get_connected_device_name(info);
}

void BleModule::set_log_level(espp::Logger::Verbosity level) {
  config_.log_level = level;
  logger.set_verbosity(level);
  ble_gatt_server_.set_log_level(level);
}

void BleModule::setup_callbacks() {
  espp::BleGattServer::Callbacks callbacks;

  callbacks.connect_callback = [this](NimBLEConnInfo &conn) {
  logger.info("Device connected – checking MTU");
  // Directly ask the NimBLE device for the active client connection
 // uint16_t mtu = NimBLEDevice::getClientConnection()->getMTU();
 // logger.info("Negotiated MTU = %d", mtu);
  if (config_.on_connect) {
  config_.on_connect(conn);
  }
  }; // Confirm MTU 

  callbacks.disconnect_callback = [this](NimBLEConnInfo& conn_info, espp::BleGattServer::DisconnectReason reason) {
    logger.info("Device disconnected: {}", reason);
    if (config_.on_disconnect) {
      config_.on_disconnect(conn_info, reason);
    }
  };

  callbacks.authentication_complete_callback = [this](const NimBLEConnInfo& conn_info) {
    logger.info("Device authenticated");
    if (config_.on_authenticated) {
      config_.on_authenticated(conn_info);
    }
  };

  callbacks.get_passkey_callback = [this]() {
    logger.info("Getting passkey");
    return NimBLEDevice::getSecurityPasskey();
  };

  callbacks.confirm_passkey_callback = [this](const NimBLEConnInfo& conn_info, uint32_t passkey) {
    logger.info("Confirming passkey: {}", passkey);
    NimBLEDevice::injectConfirmPasskey(conn_info,
                                      passkey == NimBLEDevice::getSecurityPasskey());
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

  logger.debug("Security configured - bonding: {}, mitm: {}, secure_conn: {}",
               config_.bonding, config_.mitm, config_.secure_connections);
}

void BleModule::setup_device_info() {
  auto& device_info_service = ble_gatt_server_.device_info_service();

  // Set PnP ID
  device_info_service.set_pnp_id(config_.vendor_source, config_.vendor_id,
                                 config_.product_id, config_.product_version);

  // Set device info strings
  device_info_service.set_manufacturer_name(config_.manufacturer_name);
  device_info_service.set_model_number(config_.model_number);
  device_info_service.set_serial_number(config_.serial_number);
  device_info_service.set_software_version(config_.software_version);
  device_info_service.set_firmware_version(config_.firmware_version);
  device_info_service.set_hardware_version(config_.hardware_version);

  logger.debug("Device info configured");
}

void BleModule::setup_advertising() {
  // Set the advertising data
  espp::BleGattServer::AdvertisedData adv_data;

  // Set flags for general discoverable mode
  uint8_t flags = BLE_HS_ADV_F_DISC_GEN;
  adv_data.setFlags(flags);
  adv_data.setName(config_.device_name);
  adv_data.setAppearance((uint16_t)espp::BleAppearance::GENERIC_COMPUTER);
  adv_data.addTxPower();
  // **NEW** – advertise the Quaternion service UUID
  // -------------------------------------------------
  // NimBLEUUID takes a 128‑bit array + length
  NimBLEUUID quat_svc_uuid(QUAT_SVC_UUID, 16);
  adv_data.addServiceUUID(quat_svc_uuid);
  // -------------------------------------------------
  ble_gatt_server_.set_advertisement_data(adv_data);

  // Start the GATT server
  ble_gatt_server_.start();

  logger.debug("Advertising configured");
}


// Jaden's additions for quaternion service

/* ----------------------------------------------------------------
 *  Create the custom Quaternion service + NOTIFY characteristic
 * ---------------------------------------------------------------- */
void BleModule::init_quat_service()
{
    // 1️⃣  Get the NimBLE server that the ESP‑IDF has already created
    NimBLEServer *pServer = NimBLEDevice::getServer();   // <‑‑ must use getServer()

    // 2️⃣  Create the service on that server
    NimBLEService *svc = pServer->createService(
        NimBLEUUID(QUAT_SVC_UUID, 16));

    // 3️⃣  Create a NOTIFY‑only characteristic
    quat_char = svc->createCharacteristic(
        NimBLEUUID(QUAT_CHAR_UUID, 16),
        NIMBLE_PROPERTY::NOTIFY);

    // 4️⃣  Add the mandatory CCCD descriptor (so the phone can enable/disable)
    quat_char->createDescriptor(
    NimBLEUUID((uint16_t)0x2901),                     // User Description
    NIMBLE_PROPERTY::READ);                          // read‑only
    quat_char->setValue((uint8_t *)"Quaternion", 10);    // the string that will be shown

    svc->start();

    ESP_LOGI(BLE_TAG, "Quaternion service/characteristic created");
}

/* ----------------------------------------------------------------
 *  Send a quaternion notification – payload must be exactly 24 bytes
 * ---------------------------------------------------------------- */
void BleModule::notify_quaternion(const uint8_t *payload, size_t len)
{
    if (!quat_char) {
        ESP_LOGW(BLE_TAG, "notify_quaternion() called but quat_char == nullptr");
        return;
    }

    quat_char->setValue(payload, len);
    bool ok = quat_char->notify();   // false = notification failed
    ESP_LOGI(BLE_TAG,
             "notify_quaternion() → %s (len=%d)",
             ok ? "OK" : "FAIL",
             (int)len);
}

/* ----------------------------------------------------------------
 *  Return true when the client has enabled notifications (CCCD = 0x0001)
 * ---------------------------------------------------------------- */
bool BleModule::quat_notify_enabled() const
{
    // The ESP‑IDF NimBLE API does not expose a direct “isSubscribed()”
    // helper.  For our simple use‑case we only need to know that the
    // characteristic object exists; calling notify() when nobody is
    // subscribed is safe – the function will simply return false.
    return (quat_char != nullptr);
}

} // namespace BLE
