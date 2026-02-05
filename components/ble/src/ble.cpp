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
    static const uint8_t ACK_CHAR_UUID[16] = {
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0,
    0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf3 };  // Note: f3 (different from f2)

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

bool BleModule::init()
{
    // ... existing code ...
    ble_gatt_server_.set_log_level(config_.log_level);
    setup_callbacks();
    ble_gatt_server_.init(config_.device_name);
    ble_gatt_server_.set_advertise_on_disconnect(true);
    setup_security();
    ble_gatt_server_.start_services();
    setup_device_info();
    
    // ✅ Create quaternion service BEFORE starting
    init_quat_service();
    
    setup_advertising();
    initialized_ = true;
    logger.info("BLE module initialized");
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
    ESP_LOGI(BLE_TAG, "🚀 init_quat_service() CALLED");
    
    NimBLEServer *pServer = NimBLEDevice::getServer();
    if (!pServer) {
        ESP_LOGE(BLE_TAG, "❌ FATAL: getServer() returned NULL!");
        return;
    }
    ESP_LOGI(BLE_TAG, "✅ Got server pointer: %p", pServer);
    
    NimBLEUUID svc_uuid(QUAT_SVC_UUID, 16);
    NimBLEUUID char_uuid(QUAT_CHAR_UUID, 16);
    NimBLEUUID ack_uuid(ACK_CHAR_UUID, 16);
    
    ESP_LOGI(BLE_TAG, "📋 Service UUID:    %s", svc_uuid.toString().c_str());
    ESP_LOGI(BLE_TAG, "📋 Quat Char UUID:  %s", char_uuid.toString().c_str());
    ESP_LOGI(BLE_TAG, "📋 ACK Char UUID:   %s", ack_uuid.toString().c_str());
    
    // Create service
    ESP_LOGI(BLE_TAG, "Creating service...");
    NimBLEService *svc = pServer->createService(svc_uuid);
    if (!svc) {
        ESP_LOGE(BLE_TAG, "❌ FATAL: createService() returned NULL!");
        return;
    }
    ESP_LOGI(BLE_TAG, "✅ Service created: %p", svc);
    
    // ========== QUATERNION CHARACTERISTIC (NOTIFY + READ) ==========
    ESP_LOGI(BLE_TAG, "Creating quaternion characteristic...");
    quat_char = svc->createCharacteristic(
        char_uuid, 
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );
    
    if (!quat_char) {
        ESP_LOGE(BLE_TAG, "❌ FATAL: createCharacteristic(quat) returned NULL!");
        return;
    }
    ESP_LOGI(BLE_TAG, "✅ Quat characteristic created: %p", quat_char);
    
    // Set initial value
    uint8_t dummy[24] = {0};
    quat_char->setValue(dummy, 24);
    ESP_LOGI(BLE_TAG, "✅ Quat char: set initial 24-byte value");
    
    // CCCD for quaternion notifications
    class QuatCCCDCallbacks : public NimBLEDescriptorCallbacks {
    public:
        BleModule* parent = nullptr;
        
        void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override {
            if (!pDescriptor) {
                ESP_LOGE(BLE_TAG, "❌ CCCD onWrite: NULL descriptor!");
                return;
            }
            
            uint16_t value = pDescriptor->getValue<uint16_t>();
            bool enabled = (value & 0x0001) != 0;
            ESP_LOGI(BLE_TAG, "📱 QUAT CCCD Write: 0x%04x -> Notifications %s", 
                     value, enabled ? "ENABLED ✅" : "DISABLED ❌");
            
            if (parent) {
                parent->quat_notifications_enabled = enabled;
                
                if (enabled) {
                    ESP_LOGI(BLE_TAG, "🎯 Notifications enabled - setting first_send_pending=true");
                    parent->first_send_pending = true;
                    parent->ack_received = false;
                }
            } else {
                ESP_LOGE(BLE_TAG, "❌ CCCD callback: parent is NULL!");
            }
        }
    };
    
    ESP_LOGI(BLE_TAG, "Creating CCCD callbacks for quat...");
    QuatCCCDCallbacks* cccd_cb = new QuatCCCDCallbacks();
    cccd_cb->parent = this;
    ESP_LOGI(BLE_TAG, "✅ CCCD callbacks created");
    
    ESP_LOGI(BLE_TAG, "Creating CCCD descriptor...");
    NimBLEDescriptor *cccd = quat_char->createDescriptor(
        NimBLEUUID((uint16_t)0x2902),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    
    if (!cccd) {
        ESP_LOGE(BLE_TAG, "❌ FATAL: createDescriptor(CCCD) returned NULL!");
        delete cccd_cb;
        return;
    }
    ESP_LOGI(BLE_TAG, "✅ CCCD descriptor created");
    
    cccd->setCallbacks(cccd_cb);
    ESP_LOGI(BLE_TAG, "✅ CCCD callbacks set");
    
    // User Description for quaternion
    ESP_LOGI(BLE_TAG, "Creating User Description for quat...");
    NimBLEDescriptor *userDesc = quat_char->createDescriptor(
        NimBLEUUID((uint16_t)0x2901),
        NIMBLE_PROPERTY::READ
    );
    if (userDesc) {
        userDesc->setValue("Quaternion Data (NOTIFY)");
        ESP_LOGI(BLE_TAG, "✅ User Description set");
    } else {
        ESP_LOGW(BLE_TAG, "⚠️  Failed to create User Description");
    }
    
    // ========== ACK CHARACTERISTIC (WRITE) ==========
    ESP_LOGI(BLE_TAG, "Creating ACK characteristic...");
    ack_char = svc->createCharacteristic(
        ack_uuid,
        NIMBLE_PROPERTY::WRITE
    );
    
    if (!ack_char) {
        ESP_LOGE(BLE_TAG, "❌ FATAL: createCharacteristic(ack) returned NULL!");
        return;
    }
    ESP_LOGI(BLE_TAG, "✅ ACK characteristic created: %p", ack_char);
    
    // ACK write callback
    class AckCharCallbacks : public NimBLECharacteristicCallbacks {
    public:
        BleModule* parent = nullptr;
        
        void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
            if (!pCharacteristic) {
                ESP_LOGE(BLE_TAG, "❌ ACK onWrite: NULL characteristic!");
                return;
            }
            
            std::string value = pCharacteristic->getValue();
            ESP_LOGI(BLE_TAG, "📬 ACK RECEIVED: %d bytes", (int)value.length());
            
            // Log hex dump of ACK value
            if (value.length() > 0) {
                char hex_str[128] = {0};
                int pos = 0;
                for (size_t i = 0; i < value.length() && i < 32; i++) {
                    pos += snprintf(hex_str + pos, sizeof(hex_str) - pos, 
                                  "%02X ", (uint8_t)value[i]);
                }
                ESP_LOGI(BLE_TAG, "   ACK hex: %s", hex_str);
            }
            
            if (parent) {
                if (!parent->ack_received) {
                    parent->ack_received = true;
                    ESP_LOGI(BLE_TAG, "✅ ACK flag set to TRUE - ready for next send");
                } else {
                    ESP_LOGW(BLE_TAG, "⚠️  Duplicate ACK received (already set)");
                }
            } else {
                ESP_LOGE(BLE_TAG, "❌ ACK callback: parent is NULL!");
            }
        }
    };
    
    ESP_LOGI(BLE_TAG, "Creating ACK callbacks...");
    AckCharCallbacks* ack_cb = new AckCharCallbacks();
    ack_cb->parent = this;
    ack_char->setCallbacks(ack_cb);
    ESP_LOGI(BLE_TAG, "✅ ACK callbacks set");
    
    // User Description for ACK
    ESP_LOGI(BLE_TAG, "Creating User Description for ACK...");
    NimBLEDescriptor *ackUserDesc = ack_char->createDescriptor(
        NimBLEUUID((uint16_t)0x2901),
        NIMBLE_PROPERTY::READ
    );
    if (ackUserDesc) {
        ackUserDesc->setValue("Acknowledgment (WRITE any value)");
        ESP_LOGI(BLE_TAG, "✅ ACK User Description set");
    } else {
        ESP_LOGW(BLE_TAG, "⚠️  Failed to create ACK User Description");
    }
    
    // ========== START SERVICE ==========
    ESP_LOGI(BLE_TAG, "Starting service...");
    svc->start();
    ESP_LOGI(BLE_TAG, "✅✅✅ QUATERNION SERVICE STARTED ✅✅✅");
    ESP_LOGI(BLE_TAG, "   Service handle: %d", svc->getHandle());
    ESP_LOGI(BLE_TAG, "   Quat char handle: %d", quat_char->getHandle());
    ESP_LOGI(BLE_TAG, "   ACK char handle: %d", ack_char->getHandle());
}

/* ----------------------------------------------------------------
 *  Send a quaternion notification – payload must be exactly 24 bytes
 * ---------------------------------------------------------------- */
void BleModule::notify_quaternion(const uint8_t *payload, size_t len)
{
    if (!quat_char) {
        ESP_LOGW(BLE_TAG, "❌ quat_char is NULL!");
        return;
    }
    
    if (!is_connected()) {
        // Only log occasionally to avoid spam
        static int not_connected_count = 0;
        if (not_connected_count++ % 100 == 0) {
            ESP_LOGW(BLE_TAG, "Not connected");
        }
        return;
    }
    
    // *** REMOVED: notification enabled check ***
    // Just send it regardless
    
    quat_char->setValue(payload, len);
    bool success = quat_char->notify();
    
    // Only log failures or occasionally log success
    static int send_count = 0;
    if (!success) {
        ESP_LOGE(BLE_TAG, "❌ notify() failed!");
    } else if (send_count++ % 10 == 0) {
        ESP_LOGI(BLE_TAG, "✅ Quat sent (%d bytes)", (int)len);
    }
}

bool BleModule::quat_notify_enabled() const
{
    return (quat_char != nullptr) && quat_notifications_enabled;
}
bool BleModule::is_ack_received() const {
    return ack_received || first_send_pending;
}

void BleModule::reset_ack() {
    if (first_send_pending) {
        ESP_LOGI(BLE_TAG, "🎯 First send completed - clearing first_send_pending flag");
        first_send_pending = false;
    }
    
    if (ack_received) {
        ESP_LOGI(BLE_TAG, "🔄 Resetting ACK flag to FALSE - waiting for next ACK");
        ack_received = false;
    } else {
        ESP_LOGW(BLE_TAG, "⚠️  reset_ack() called but ack_received was already FALSE");
    }
}

void BleModule::reset_ack_on_connect() {
    ESP_LOGI(BLE_TAG, "🔌 Connection state reset:");
    ESP_LOGI(BLE_TAG, "   first_send_pending: %s -> TRUE", first_send_pending ? "true" : "false");
    ESP_LOGI(BLE_TAG, "   ack_received: %s -> FALSE", ack_received ? "true" : "false");
    
    first_send_pending = true;
    ack_received = false;
}

} // namespace BLE
