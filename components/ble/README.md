# BLE Module

BLE GATT Server module for Ski Performance Wearable. This module provides BLE connectivity with battery and device information services.

## Features

- BLE GATT Server with configurable security
- Battery service with level updates
- Device information service
- Connection callbacks
- Configurable advertising data
- Simple API for initialization and control

## Usage

### Basic Initialization

```cpp
#include "ble.hpp"

// Create BLE module with default configuration
BLE::BleModule ble_module;

// Initialize
if (!ble_module.init()) {
    // Handle initialization error
}

// Start advertising
ble_module.start_advertising();
```

### Custom Configuration

```cpp
#include "ble.hpp"

// Create custom configuration
BLE::BleModule::Config config;
config.device_name = "My Ski Wearable";
config.manufacturer_name = "My Company";
config.model_number = "v2.0";
config.serial_number = "123456789";

// Set up callbacks
config.on_connect = [](const NimBLEConnInfo& info) {
    // Handle connection
};

config.on_disconnect = [](const NimBLEConnInfo& info, int reason) {
    // Handle disconnection
};

// Create module with config
BLE::BleModule ble_module(config);
ble_module.init();
ble_module.start_advertising();
```

### Updating Battery Level

```cpp
// Update battery level (0-100%)
ble_module.set_battery_level(85);
```

### Checking Connection Status

```cpp
if (ble_module.is_connected()) {
    // Get connected device information
    auto devices = ble_module.get_connected_device_infos();

    for (const auto& device : devices) {
        auto rssi = ble_module.get_rssi(device);
        auto name = ble_module.get_device_name(device);

        printf("Device: %s, RSSI: %d dBm\n", name.c_str(), rssi);
    }
}
```

## Testing in main.cpp

Here's a complete example to test the BLE module in your `main.cpp`:

```cpp
#include <chrono>
#include <thread>
#include "ble.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Main", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting BLE test...");

  // Configure BLE with custom settings
  BLE::BleModule::Config config;
  config.device_name = "Ski Wearable Test";
  config.manufacturer_name = "ESP-CPP";
  config.model_number = "ski-wearable-01";
  config.serial_number = "TEST123456";

  // Set up connection callbacks
  config.on_connect = [&logger](NimBLEConnInfo& info) {
    logger.info("Client connected!");
  };

  config.on_disconnect = [&logger](NimBLEConnInfo& info,
                                   espp::BleGattServer::DisconnectReason reason) {
    logger.info("Client disconnected: {}", reason);
  };

  config.on_authenticated = [&logger](const NimBLEConnInfo& info) {
    logger.info("Client authenticated successfully");
  };

  // Create and initialize BLE module
  BLE::BleModule ble_module(config);

  if (!ble_module.init()) {
    logger.error("Failed to initialize BLE module");
    return;
  }

  logger.info("BLE module initialized successfully");

  // Start advertising
  if (!ble_module.start_advertising()) {
    logger.error("Failed to start advertising");
    return;
  }

  logger.info("BLE advertising started. Device name: {}", config.device_name);
  logger.info("Connect with your phone's BLE scanner app");

  // Main loop: update battery level and check connection
  uint8_t battery_level = 100;
  bool was_connected = false;

  while (true) {
    auto start = std::chrono::steady_clock::now();

    // Check connection status
    if (ble_module.is_connected()) {
      if (!was_connected) {
        // Just connected
        logger.info("Device connected! Getting device info...");

        auto devices = ble_module.get_connected_device_infos();
        for (const auto& device : devices) {
          auto rssi = ble_module.get_rssi(device);
          auto name = ble_module.get_device_name(device);
          logger.info("  Device: {}, RSSI: {} dBm", name, rssi);
        }

        was_connected = true;
      }

      // Update battery level (simulate discharge)
      ble_module.set_battery_level(battery_level);
      battery_level = (battery_level == 0) ? 100 : battery_level - 1;

      logger.info("Battery level updated: {}%", battery_level);

    } else {
      if (was_connected) {
        // Just disconnected
        logger.info("Device disconnected. Resuming advertising...");
        was_connected = false;
        battery_level = 100;  // Reset battery for next connection
      }

      // Not connected - show waiting message periodically
      static int wait_count = 0;
      if (wait_count++ % 10 == 0) {
        logger.info("Waiting for BLE connection...");
      }
    }

    // Sleep until next iteration (1 second intervals)
    std::this_thread::sleep_until(start + 1s);
  }
}
```

### What to Expect

When you flash this test code:

1. **Serial Monitor Output:**
   ```
   [Main] Starting BLE test...
   [BLE] Initializing BLE module: Ski Wearable Test
   [Main] BLE module initialized successfully
   [Main] BLE advertising started. Device name: Ski Wearable Test
   [Main] Connect with your phone's BLE scanner app
   [Main] Waiting for BLE connection...
   ```

2. **Connect with a BLE Scanner App:**
   - iOS: Use "LightBlue" app
   - Android: Use "nRF Connect" app
   - Look for device named "Ski Wearable Test"

3. **After Connection:**
   ```
   [BLE] Device connected
   [Main] Device connected! Getting device info...
   [Main]   Device: iPhone, RSSI: -45 dBm
   [Main] Battery level updated: 100%
   [Main] Battery level updated: 99%
   ```

4. **Services Available:**
   - **Battery Service (0x180F)**: Shows current battery level
   - **Device Information Service (0x180A)**: Shows manufacturer, model, serial number, etc.

### Testing Tips

- Enable BLE in `sdkconfig`: `CONFIG_BT_ENABLED=y` and `CONFIG_BT_NIMBLE_ENABLED=y`
- Monitor serial output with `idf.py monitor`
- You can read the battery characteristic in your BLE scanner app and watch it count down
- Try disconnecting/reconnecting to test the callbacks
- Check that device information appears correctly in the scanner app

## Configuration Options

### Security Settings

- `bonding`: Enable bonding (default: true)
- `mitm`: Man-in-the-middle protection (default: false)
- `secure_connections`: Use secure connections (default: true)
- `passkey`: Security passkey (default: 123456)
- `io_capabilities`: I/O capabilities (default: BLE_HS_IO_NO_INPUT_OUTPUT)

### Device Information

- `device_name`: Advertised device name
- `manufacturer_name`: Manufacturer name
- `model_number`: Model number
- `serial_number`: Serial number
- `software_version`: Software version
- `firmware_version`: Firmware version
- `hardware_version`: Hardware version

### PnP ID

- `vendor_source`: Vendor ID source (default: 0x01)
- `vendor_id`: Vendor ID (default: 0xCafe)
- `product_id`: Product ID (default: 0xFace)
- `product_version`: Product version (default: 0x0100)

## API Reference

### Initialization

- `bool init()`: Initialize the BLE module
- `bool start_advertising()`: Start BLE advertising
- `bool stop_advertising()`: Stop BLE advertising

### Connection Management

- `bool is_connected()`: Check if a device is connected
- `std::vector<NimBLEConnInfo> get_connected_device_infos()`: Get connection info for all connected devices
- `int get_rssi(const NimBLEConnInfo& info)`: Get RSSI for a connected device
- `std::string get_device_name(const NimBLEConnInfo& info)`: Get name of connected device

### Battery Service

- `void set_battery_level(uint8_t level)`: Update battery level (0-100%)
- `uint8_t get_battery_level()`: Get current battery level

### Advanced

- `espp::BleGattServer& get_server()`: Get direct access to the underlying GATT server
- `void set_log_level(espp::Logger::Verbosity level)`: Set logging verbosity

## Dependencies

- espp__ble_gatt_server
- espp__cli
- NimBLE (ESP-IDF)
