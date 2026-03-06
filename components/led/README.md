# LED Component

**Purpose**
Controls multi‑color (RGB) LEDs via GPIO PWM (LEDC) or a simple on/off GPIO for status indication.

**Public API (class `LED::Led`)**
- Constructor `Led(gpio_num_t red, gpio_num_t green, gpio_num_t blue)` – store PWM‑capable pins.
- `esp_err_t init()` – configure each pin for LEDC (PWM) channel, set default duty 0.
- `esp_err_t set_color(uint8_t r, uint8_t g, uint8_t b)` – set PWM duty (0‑255) for each channel.
- `esp_err_t blink(uint32_t on_ms, uint32_t off_ms)` – start a non‑blocking blink using a FreeRTOS timer.

**Thread‑Safety**
All methods acquire the LED driver’s internal mutex; safe to call from any task.

**Typical Usage**
```cpp
LED::Led status_led(GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_14);
status_led.init();
status_led.set_color(0, 255, 0); // green
```