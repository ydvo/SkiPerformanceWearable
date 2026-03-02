# Common GPIO Component

**Purpose**
Provides a thin C++ wrapper around the ESP‑IDF GPIO driver, offering convenient configuration, level control, and interrupt registration for a single pin.

**Public API (class `Common::GPIO`)**
- Constructor `GPIO(gpio_num_t pin, Direction dir = INPUT, bool initial_level = false, Pull pull = NONE)` – configure mode, optional initial level, and pull‑up/down.
- `esp_err_t setLevel(bool level)` – drive output high/low.
- `bool getLevel() const` – read current pin level.
- `gpio_num_t getPin() const` – retrieve the managed pin number.
- `esp_err_t set_interrupt(InterruptType type, gpio_isr_t handler, void *arg = nullptr)` – install ISR service (if needed) and attach handler.
- `esp_err_t disable_interrupt()` – detach ISR and disable the interrupt.

**Thread‑Safety**
The wrapper itself is not re‑entrant; however, ISR registration and level changes are safe to invoke from any task. The ISR service is installed only once per system.

**Typical Usage**
```cpp
Common::GPIO led_gpio(GPIO_NUM_2, Common::GPIO::OUTPUT, false);
led_gpio.setLevel(true);

Common::GPIO encoder_a(GPIO_NUM_4);
encoder_a.set_interrupt(Common::GPIO::INTERRUPT_ANY_EDGE, encoder_isr);
```