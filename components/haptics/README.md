# Haptics Component - DRV2605L Driver

ESP32-S3 compatible driver for the Texas Instruments DRV2605L Haptic Motor Driver IC.

## Features

- Full DRV2605L register access
- Support for ERM (Eccentric Rotating Mass) and LRA (Linear Resonant Actuator) motors
- Multiple operation modes:
  - Internal trigger with waveform library
  - External trigger
  - Real-time playback
  - Audio-to-vibe
  - Auto-calibration
  - Diagnostics
- 6 ERM waveform libraries + 1 LRA library with 100+ effects
- Battery voltage monitoring
- LRA resonance period detection

## Hardware

- **IC**: Texas Instruments DRV2605L
- **I2C Address**: 0x5A (default)
- **Supply Voltage**: 2.0V - 5.2V
- **Motor Types**: ERM or LRA

## Usage Example

```cpp
#include "haptic_motor.hpp"
#include "i2c.hpp"

// Initialize I2C
espp::I2c::Config i2c_config = {
    .port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_8,
    .scl_io_num = GPIO_NUM_9,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .clk_speed = 400000
};
espp::I2c i2c(i2c_config);

// Initialize haptic driver
HAPTICS::DRV2605L haptic(i2c);

if (haptic.init()) {
    // Set to internal trigger mode
    haptic.set_mode(HAPTICS::DRV2605L::Mode::INTERNAL_TRIGGER);

    // Select ERM motor
    haptic.select_motor(HAPTICS::DRV2605L::MotorType::ERM);

    // Select library (try different ones!)
    haptic.select_library(HAPTICS::DRV2605L::Library::ERM_LIB_A);

    // Set waveform sequence
    haptic.set_waveform(0, 1);   // Sequencer 0, effect 1 (strong click)
    haptic.set_waveform(1, 0);   // End of sequence

    // Trigger the waveform
    haptic.go();

    // Wait for effect to complete
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## Example: Real-Time Playback Mode

```cpp
// Set to real-time playback mode
haptic.set_mode(HAPTICS::DRV2605L::Mode::REALTIME_PLAYBACK);

// Drive motor with varying intensity
for (int i = 0; i < 255; i++) {
    haptic.set_realtime_value(i);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Stop motor
haptic.set_realtime_value(0);
```

## Example: Auto-Calibration (LRA motors)

```cpp
// Select LRA motor
haptic.select_motor(HAPTICS::DRV2605L::MotorType::LRA);

// Set rated voltage (depends on your motor specs)
haptic.set_rated_voltage(0x40);  // Example value

// Set overdrive clamp voltage
haptic.set_overdrive_clamp(0x8C);  // Example value

// Run auto-calibration
haptic.set_mode(HAPTICS::DRV2605L::Mode::AUTO_CALIBRATION);
haptic.go();

// Wait for calibration to complete (~2 seconds)
vTaskDelay(pdMS_TO_TICKS(2000));

// Check results
uint8_t comp = haptic.get_comp_result();
uint8_t bemf = haptic.get_backemf_result();
printf("Calibration complete - Comp: 0x%02X, BEMF: 0x%02X\n", comp, bemf);
```

## Waveform Libraries

### ERM Libraries
- **ERM_LIB_A**: General purpose, good starting point
- **ERM_LIB_B**: Alternative effects
- **ERM_LIB_C**: Strong emphasis
- **ERM_LIB_D**: Sharp response
- **ERM_LIB_E**: Soft response

### LRA Library
- **LRA_LIB**: Optimized for Linear Resonant Actuators

Each library contains ~123 different effects. See the [DRV2605L datasheet](https://www.ti.com/lit/ds/symlink/drv2605l.pdf) for a complete list of waveforms.

## Common Waveform Effects

| Effect # | Description |
|----------|-------------|
| 1 | Strong Click - 100% |
| 10 | Double Click - 100% |
| 14 | Triple Click - 100% |
| 47 | Buzz 1 - 100% |
| 52 | Pulsing Strong 1 - 100% |
| 70 | Transition Ramp Down Long Smooth 1 |
| 108 | Short transition ramp up |

## API Reference

### Initialization
- `bool init()` - Initialize and check device presence
- `bool is_device_ready()` - Check if device is ready (no errors)
- `uint8_t get_status()` - Read status register

### Mode Control
- `void set_mode(Mode mode)` - Set operation mode
- `void select_motor(MotorType motor)` - Select ERM or LRA motor
- `void select_library(Library lib)` - Select waveform library

### Waveform Control
- `void set_waveform(uint8_t sequencer, uint8_t waveform)` - Set waveform for sequencer 0-7
- `void go()` - Start waveform playback
- `void stop()` - Stop waveform playback

### Real-Time Playback
- `void set_realtime_value(uint8_t value)` - Set motor drive (0-255)

### Monitoring
- `float get_battery_voltage()` - Read battery voltage
- `float get_lra_resonance_period_us()` - Read LRA resonance period

### Advanced Control
- `void set_rated_voltage(uint8_t value)` - Set rated voltage
- `void set_overdrive_clamp(uint8_t value)` - Set overdrive clamp
- `void set_control1/2/3/4/5(uint8_t value)` - Set control registers
- See header file for complete API

## Datasheet

[TI DRV2605L Datasheet](https://www.ti.com/lit/ds/symlink/drv2605l.pdf)

## Credits

Adapted from SparkFun Electronics DRV2605L Arduino Library by Mary West.
Modified for ESP32-S3 with Claude Clode
