# Haptics Component (DRV2605L)

**Purpose**
Controls a DRV2605L haptic‑motor driver over I²C, allowing playback of predefined waveforms and custom effects for tactile feedback.

**Public API (class `HAPTICS::DRV2605L`)**
- Constructor `DRV2605L(espp::I2c &i2c, uint8_t address = DEFAULT_ADDRESS)` – store I²C reference.
- `bool init()` – initialize the device, check status, set default mode, motor type, and library.
- `void set_mode(Mode mode)` – select operation mode.
- `void select_motor(MotorType motor)` – choose ERM or LRA motor type.
- `void select_library(Library lib)` – choose waveform library.
- `bool play(const uint8_t *effects, uint8_t count)` – program up to 8 effects and start playback.
- `void stop()` – halt any ongoing vibration.
- `void set_realtime_value(uint8_t value)` – set intensity for realtime mode.
- `void enter_standby()` – put the driver into standby mode (~25 µA) by setting the standby bit in the MODE register.
- Diagnostic getters (`get_battery_voltage()`, `get_lra_resonance_period_us()`, ...).

**Thread‑Safety**
All public methods acquire an internal mutex before issuing I²C transactions, making them safe to call from multiple FreeRTOS tasks.

**Typical Usage**
```cpp
HAPTICS::DRV2605L haptic(i2c);
if (!haptic.init()) {
    // handle init error
}
haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);
uint8_t seq[] = {HAPTICS::DRV2605L::EFFECTS::DOUBLE_CLICK,
                 HAPTICS::DRV2605L::EFFECTS::END};
haptic.play(seq, 2);

// Before deep‑sleep
haptic.enter_standby();
```
