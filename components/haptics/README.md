# Haptics Component (DRV2605L)

**Purpose**
Controls a DRV2605L haptic‑motor driver over I²C, allowing playback of predefined waveforms and custom effects for tactile feedback.

**Public API (class `HAPTICS::HapticMotor`)**
- Constructor `HapticMotor(i2c_port_t i2c_port, gpio_num_t sda, gpio_num_t scl, uint8_t address = 0x5A)` – store I²C configuration.
- `esp_err_t init()` – initialize I²C peripheral, reset the DRV2605L, set default mode (e.g., **Feedback**).
- `esp_err_t play_effect(uint8_t effect_id)` – trigger a built‑in waveform.
- `esp_err_t set_intensity(uint8_t level)` – configure output strength (0‑255).
- `esp_err_t stop()` – halt any ongoing vibration.

**Thread‑Safety**
All public methods acquire an internal mutex before issuing I²C transactions; safe to call from multiple FreeRTOS tasks.

**Typical Usage**
```cpp
HAPTICS::HapticMotor vib(i2c_port, GPIO_NUM_21, GPIO_NUM_22);
vib.init();
vib.set_intensity(180);
vib.play_effect(3);   // strong click
vTaskDelay(pdMS_TO_TICKS(200));
vib.stop();
```