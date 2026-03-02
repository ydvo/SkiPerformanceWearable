# Encoder Component

**Purpose**
Implements a high‑resolution quadrature rotary encoder driver using a lookup‑table ISR. Provides atomic tick counting and on‑the‑fly angle calculation.

**Public API (class `SENSORS::Encoder`)**
- Constructor `Encoder(gpio_num_t pin_a, gpio_num_t pin_b, uint32_t cpr)` – store pins and counts‑per‑revolution; no hardware access yet.
- `esp_err_t init()` – configure both pins for any‑edge interrupts and register ISR.
- `int32_t get_count() const` – atomic read of tick count (positive = CW, negative = CCW).
- `float get_angle() const` – compute mechanical angle in degrees, wrapped to `[0,360)`. Uses `fmodf` for rollover handling.
- `void reset()` – zero the atomic counter.

**Thread‑Safety**
Tick count is stored in a `std::atomic<int32_t>`; all getters read the atomic value with relaxed ordering, safe from any FreeRTOS task.

**Typical Usage**
```cpp
SENSORS::Encoder wheel_encoder(GPIO_NUM_5, GPIO_NUM_6, 1024);
wheel_encoder.init();

while (true) {
    int32_t ticks = wheel_encoder.get_count();
    float angle = wheel_encoder.get_angle();
    // Use ticks/angle …
    vTaskDelay(pdMS_TO_TICKS(20));
}
```