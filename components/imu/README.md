# IMU Component (ICM‑20948 + Madgwick)

**Purpose**
Interfaces with the ICM‑20948 9‑axis IMU over I²C, reads raw accelerometer, gyroscope, and magnetometer data, and runs a Madgwick filter to produce a drift‑corrected orientation quaternion.

**Key Headers**
- `imu.hpp` – high‑level `IMU::Imu` class (initialization, data acquisition, quaternion output).
- `madgwick_filter_quat.hpp` – template implementation of the Madgwick algorithm for quaternion output.

**Public API (class `IMU::Imu`)**
- Constructor `Imu(const Config &cfg)` – store I²C bus configuration, sampling rates, filter gains.
- `esp_err_t init()` – configure I²C, reset sensor, enable required modules (ACCEL, GYRO, MAG).
- `esp_err_t read_raw(IMU::RawData &out)` – fetch raw sensor registers.
- `esp_err_t update()` – read raw data, feed Madgwick filter, update internal quaternion.
- `Quaternion get_quaternion() const` – retrieve latest filtered orientation (w, x, y, z).

**Thread‑Safety**
`update()` may be called from a dedicated sensor task; internal state (`Quaternion`) is protected by a mutex so `get_quaternion()` can be called from other tasks.

**Typical Usage**
```cpp
IMU::Imu imu({ .i2c_port = I2C_NUM_0, .address = 0x68 });
imu.init();

while (true) {
    imu.update();
    auto q = imu.get_quaternion();
    // transmit q over BLE, log, or use for control …
    vTaskDelay(pdMS_TO_TICKS(10));
}
```