#include "ble.hpp"
#include "NimBLEDevice.h"
#include "esp_log.h"
#include "esp_check.h"

namespace BLE {
  static const char *TAG = "BLE";

  static const NimBLEUUID QUAT_SVC_UUID {"16fd3a8f-f37e-4155-8ebf-654df4d3f700"}; 
  static const NimBLEUUID QUAT_CHAR_UUID {"16fd3a8f-f37e-4155-8ebf-654df4d3f701"}; 
  static const NimBLEUUID QUAT_ACK_UUID {"16fd3a8f-f37e-4155-8ebf-654df4d3f702"}; 

  static espp::Logger logger({.tag = TAG, .level = espp::Logger::Verbosity::INFO});

  BleModule::BleModule(): config_(), battery_level_(100), initialized_(false) {
    logger.set_verbosity(config_.log_level);
  }

  BleModule::BleModule(const Config& config): config_(config), battery_level_(100), initialized_(false) {
    logger.set_verbosity(config_.log_level);
  }

  BleModule::~BleModule() {
    if (initialized_) {
      ble_gatt_server_.deinit();
    }
  }

  esp_err_t BleModule::init() {
    if (initialized_) return ESP_OK;
 
    ble_gatt_server_.set_log_level(config_.log_level);
    setup_callbacks();

    ESP_RETURN_ON_FALSE(
      ble_gatt_server_.init(config_.device_name), ESP_ERR_INVALID_STATE,
      TAG, "Failed to initialize GATT Server"
    );
    ble_gatt_server_.set_advertise_on_disconnect(true);
    setup_security();

    ble_gatt_server_.start_services();
    setup_device_info();
    init_quat_service();
    setup_advertising();

    initialized_ = true;
    logger.info("BLE module initialized");
    return ESP_OK;
  }

  esp_err_t BleModule::start_advertising() {
    if (!initialized_) {
      logger.error("BLE not initialized, cannot start advertising");
      return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_FALSE(
      ble_gatt_server_.start_advertising(), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to start advertising"
    );

    logger.info("BLE advertising started");

    return ESP_OK;
  }

  esp_err_t BleModule::stop_advertising() {
    if (!initialized_) {
      logger.error("BLE not initialized, cannot stop advertising");
      return ESP_ERR_INVALID_STATE;
    }

    ble_gatt_server_.stop_advertising();
    logger.info("BLE advertising stopped");
    return ESP_OK;
  }

  bool BleModule::is_connected() const {
    return ble_gatt_server_.is_connected();
  }

  esp_err_t BleModule::set_battery_level(uint8_t level) {
    ESP_RETURN_ON_FALSE(
      initialized_, ESP_ERR_INVALID_STATE, 
      TAG, "BLE Module must be initialized to set battery level"
    );

    if (level > 100) {
      logger.warn("Battery level {} exceeds 100%, clamping to 100%", level);
      level = 100;
      return ESP_ERR_INVALID_ARG; 
    }

    battery_level_ = level;

    auto& battery_service = ble_gatt_server_.battery_service();
    battery_service.set_battery_level(battery_level_);
    logger.debug("Battery level updated to {}%", battery_level_);
    return ESP_OK; 
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

    logger.debug(
      "Security configured - bonding: {}, mitm: {}, secure_conn: {}",
      config_.bonding, config_.mitm, config_.secure_connections
    );
  }

  void BleModule::setup_device_info() {
    auto& device_info_service = ble_gatt_server_.device_info_service();

    // Set PnP ID
    device_info_service.set_pnp_id(config_.vendor_source, config_.vendor_id, config_.product_id, config_.product_version);

    // Set device info strings
    device_info_service.set_manufacturer_name(config_.manufacturer_name);
    device_info_service.set_model_number(config_.model_number);
    device_info_service.set_serial_number(config_.serial_number);
    device_info_service.set_software_version(config_.software_version);
    device_info_service.set_firmware_version(config_.firmware_version);
    device_info_service.set_hardware_version(config_.hardware_version);

    logger.debug("Device info configured");
  }

  esp_err_t BleModule::setup_advertising() {
    // Set the advertising data
    espp::BleGattServer::AdvertisedData adv_data;

    // Set flags for general discoverable mode
    ESP_RETURN_ON_FALSE(
      adv_data.setFlags(BLE_HS_ADV_F_DISC_GEN), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to set advertised data flag"
    );
    ESP_RETURN_ON_FALSE(
      adv_data.setName(config_.device_name), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to set advertised data name"
    ); 
    ESP_RETURN_ON_FALSE(
      adv_data.setAppearance((uint16_t) espp::BleAppearance::GENERIC_COMPUTER), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to set advertised data appearance"
    );
    ESP_RETURN_ON_FALSE(
      adv_data.addTxPower(), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to add advertised data tx power"
    );

    ESP_RETURN_ON_FALSE(
      adv_data.addServiceUUID(QUAT_SVC_UUID), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to add quaternion service uuid to advertised data"
    );
    
    ble_gatt_server_.set_advertisement_data(adv_data);

    ESP_RETURN_ON_FALSE(
      ble_gatt_server_.start(), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to start ble gatt server"
    );

    logger.debug("Advertising configured");

    return ESP_OK; 
  }

  /* ----------------------------------------------------------------
  *  Create the custom Quaternion service + NOTIFY characteristic
  * ---------------------------------------------------------------- */
  esp_err_t BleModule::init_quat_service(){
    NimBLEServer *pServer = NimBLEDevice::getServer();
    ESP_RETURN_ON_FALSE(
      pServer != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed to get server"
    );

    logger.info("Service UUID: %s, Quat Char UUID: %s, ACK Char UUID: %s", 
      QUAT_SVC_UUID.toString().c_str(), QUAT_CHAR_UUID.toString().c_str(), QUAT_CHAR_UUID.toString().c_str()); 

    logger.info("Creating service...");
    
    NimBLEService *svc = pServer->createService(QUAT_SVC_UUID);
    ESP_RETURN_ON_FALSE(
      svc != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed to create service."
    ); 

    logger.info("Service created");

    logger.info("Creating quaternion characteristic..."); 

    quat_char_ = svc->createCharacteristic(
      QUAT_CHAR_UUID, 
      NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    ); 

    ESP_RETURN_ON_FALSE(
      quat_char_ != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed to create quaternion characteristic"
    ); 

    logger.info("Quaternion characteristic created");

    uint8_t dummy[24] = {0}; 
    quat_char_->setValue(dummy, 24); 
    logger.info("Quaternion character, set initial 24-byte value"); 
 
    logger.info("Creating CCCD desccriptor"); 

    NimBLEDescriptor *cccd = quat_char_->createDescriptor(
      NimBLEUUID((uint16_t)0x2902),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    ); 

    if (cccd == nullptr) {
      logger.error("Failed to create cccd descriptor, returned nullptr"); 
      return ESP_ERR_INVALID_STATE;  
    }

    logger.info("CCCD descriptor created");

    cccd->setCallbacks(&quat_cccd_cb_); 
    logger.info("CCCD callbacks set"); 

    logger.info("Creating user descriptor for quaternion"); 
    NimBLEDescriptor *userDescriptor = quat_char_->createDescriptor(
      NimBLEUUID((uint16_t)0x2901), 
      NIMBLE_PROPERTY::READ
    ); 

    ESP_RETURN_ON_FALSE(
      userDescriptor != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed to create user descriptor"
    ); 

    userDescriptor->setValue("Quaternion Data (NOTIFY)"); 
    logger.info("User Descriptor set"); 

    logger.info("Creating ACK characteristic"); 
    ack_char_ = svc->createCharacteristic(
      QUAT_ACK_UUID, 
      NIMBLE_PROPERTY::WRITE
    ); 

    ESP_RETURN_ON_FALSE(
      ack_char_ != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed to create ack characteristic"
    ); 

    logger.info("ACK characteristic created"); 
 
    ack_char_->setCallbacks(&quat_ack_cb_); 
    logger.info("ACK callbacks set"); 

    logger.info("Creating User Descriptor for ACK..."); 
    NimBLEDescriptor *ackUserDescriptor = ack_char_->createDescriptor(
      NimBLEUUID((uint16_t)0x2901), 
      NIMBLE_PROPERTY::READ
    );

    if (ackUserDescriptor == nullptr) {
      logger.error("Failed to create ack user descriptor, returned nullptr");
      return ESP_ERR_INVALID_STATE; 
    }

    ackUserDescriptor->setValue("Acknowledgement (WRITE any value)"); 
    logger.info("ACK User Description set"); 

    logger.info("Starting service"); 
    svc->start(); 
    logger.info("Quaternion service started"); 
    logger.info("Service handle: %d", svc->getHandle());
    logger.info("Quaternion characteristic handle %d", quat_char_->getHandle()); 
    logger.info("ACK characteristic handle %d", ack_char_->getHandle());  

    return ESP_OK; 
  }

  /* ----------------------------------------------------------------
  *  Send a quaternion notification – payload must be exactly 24 bytes
  * ---------------------------------------------------------------- */
  esp_err_t BleModule::notify_quaternion(const uint8_t *payload, size_t len){
    ESP_RETURN_ON_FALSE(
      quat_char_ != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Quaternion characteristic is not initialized"
    );

    ESP_RETURN_ON_FALSE(
      is_connected(), ESP_ERR_INVALID_STATE, 
      TAG, "BLE Server not connected"
    ); 

    quat_char_->setValue(payload, len); 
    ESP_RETURN_ON_FALSE(
      quat_char_->notify(), ESP_ERR_INVALID_STATE, 
      TAG, "Failed to notify"
    ); 

    return ESP_OK;
  }

  bool BleModule::quat_notify_enabled() const {
    return (quat_char_ != nullptr) && quat_notifications_enabled_;
  }

  bool BleModule::is_ack_received() const {
    return ack_received_ || first_send_pending_;
  }

  void BleModule::reset_ack() {
    first_send_pending_ = false; 
    ack_received_ = false; 
  }

  void BleModule::reset_ack_on_connect() {
    first_send_pending_ = true;
    ack_received_ = false;
  }

  void BleModule::QuatCCCDCallbacks::onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) {
    ESP_RETURN_VOID_ON_FALSE(
      pDescriptor != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed CCCD onWrite due to null descriptor"
    );

    uint16_t value = pDescriptor->getValue<uint16_t>(); 
    bool enabled = (value & 0x0001) != 0; 
    logger.info("Quaternion CCCD Write: 0x%04x -> Notifications %s", value, enabled ? "enabled" : "disabled"); 

    ESP_RETURN_VOID_ON_FALSE(
      parent_ != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed CCCD onWrite due to null parent"
    );

    parent_->quat_notifications_enabled_ = enabled; 
    if (enabled) {
      logger.info("Notification enabled"); 
      parent_->first_send_pending_ = true; 
      parent_->ack_received_ = false; 
    }
  }

  void BleModule::AckCharCallbacks::onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) {
    ESP_RETURN_VOID_ON_FALSE(
      pCharacteristic != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed ACK onWrite due to null descriptor"
    ); 

    std::string value = pCharacteristic->getValue(); 
    logger.info("ACK Received: %d bytes", value.length()); 

    ESP_RETURN_VOID_ON_FALSE(
      parent_ != nullptr, ESP_ERR_INVALID_STATE, 
      TAG, "Failed ACK onWrite due to null parent"
    );

    if (parent_->ack_received_) {
      logger.warn("Duplicate ACK received"); 
      return;
    }

    parent_->ack_received_ = true; 
    logger.info("ACK processed succesfully."); 
  }

}
