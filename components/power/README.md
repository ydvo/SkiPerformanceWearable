# Power Component (MAX1704X Fuel Gauge)

**Purpose**
Reads battery voltage, state‑of‑charge (SOC), and health metrics from a MAX1704X fuel‑gauge over I²C. Provides a simple API for battery monitoring and low‑battery warnings.

**Public API (class `POWER::FuelGauge`)**
- Constructor `FuelGauge(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint8_t address = 0x36)` – configure I²C.
- `esp_err_t init()` – perform I²C initialization, verify device ID.
- `float get_voltage() const` – return battery voltage (V).
- `float get_soc() const` – return state‑of‑charge (%).
- `float get_current() const` – return instantaneous battery current (A).

**Thread‑Safety**
All getters lock an internal mutex before performing I²C reads; safe for concurrent access.

**Typical Usage**
```cpp
POWER::FuelGauge gauge(I2C_NUM_1, GPIO_NUM_18, GPIO_NUM_19);
gauge.init();

float soc = gauge.get_soc();
ESP_LOGI("POWER", "Battery %.1f%%", soc);
```