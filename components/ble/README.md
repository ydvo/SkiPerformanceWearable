# BLE Component

**Purpose**
Implements a NimBLE‑based GATT server that streams orientation data (quaternions) from the IMU, reports battery level, and provides a reliable ACK mechanism for frame transfer. All BLE packets are packed to match the on‑the‑wire layout.

**Key Types**
- `BLE::FrameHeader` – 8‑byte header (sequence, sample count, payload length, flags).
- `BLE::FrameSample` – 24‑byte quaternion sample (timestamp + w/x/y/z).
- `BLE::Frame` – Header + up to `SAMPLES_PER_FRAME` samples (fits in a single notification).
- `BLE::Ack` – 2‑byte acknowledgment (last received `frame_seq`).

**Public API (class `BLE::BleModule`)**
- Constructors: `BleModule(const Config&)` and default `BleModule()`.
- `esp_err_t init()` – allocate NimBLE resources and configure the device.
- `esp_err_t start_advertising()` / `stop_advertising()` – control advertising.
- `bool is_connected() const` – query connection state.
- `esp_err_t set_battery_level(uint8_t)` / `uint8_t get_battery_level() const` – manage Battery‑Level characteristic.
- `esp_err_t init_quat_service()` – register the custom Quaternion service (must be called before advertising).
- `esp_err_t notify_quaternion(const uint8_t *payload, size_t len)` – send a quaternion notification.
- `void arm_ack(uint16_t seq)` – arm the ACK waiter for a just‑sent frame.
- `bool is_ack_received() const` – query whether a matching ACK has arrived.
- Callback registration via `Config` members (`on_connect`, `on_disconnect`, `on_authenticated`).

**Thread‑Safety**
All public methods are safe to call from any FreeRTOS task after `init()`. Internal state (notification enable flag, ACK handling) is protected with `std::atomic`.

**Typical Usage**
```cpp
BLE::BleModule ble;
ble.init();
ble.init_quat_service();
ble.start_advertising();

while (true) {
    // Fill a Frame with quaternion samples …
    ble.notify_quaternion(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    ble.arm_ack(frame.header.frame_seq);
    while (!ble.is_ack_received()) { vTaskDelay(pdMS_TO_TICKS(10)); }
}
```
