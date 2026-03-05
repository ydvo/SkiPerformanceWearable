# IMU Component (ICM‑20948 + Madgwick)

**Purpose**
Interfaces with the ICM‑20948 9‑axis IMU over I²C, reads raw accelerometer, gyroscope, and magnetometer data, and runs a Madgwick filter to produce a drift‑corrected orientation quaternion.

**Key Header**
- `imu.hpp` – high‑level `SENSORS::Imu` class (initialization, data acquisition, quaternion output, low‑power control).

**Public API (class `SENSORS::Imu`)**
- Constructor `Imu(espp::I2c &i2c)` – creates the IMU using a pre‑configured I²C instance.
- `bool init()` – configure the sensor, enable required modules, and set filter parameters.
- `bool update(float dt)` – read raw data, update Madgwick filter, and store latest quaternion.
- `bool update_raw(float dt)` – read raw sensor values without filtering.
- `Quaternion get_orientation() const` – retrieve the latest filtered quaternion.
- `Raw get_raw() const` – access the most recent raw sensor readings.
- `esp_err_t sleep()` – put the ICM‑20948 into hardware sleep mode (~8 µA).
- `esp_err_t wake()` – bring the sensor out of sleep mode.

**Thread‑Safety**
`update()` and `update_raw()` are intended to be called from a dedicated sensor task. The quaternion is stored in a plain struct without a mutex; callers must ensure exclusive access when reading it from other tasks.

**Typical Usage**
```cpp
SENSORS::Imu imu(i2c);
if (!imu.init()) {
    // handle init error
}

// Normal operation
imu.update(0.01f);
auto q = imu.get_orientation();

// Before deep‑sleep
imu.sleep();

// After wake‑up
imu.wake();
```
