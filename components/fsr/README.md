# FSR (Force Sensitive Resistor) Driver

Simple driver for force sensitive resistors using ESP-IDF's ADC peripheral. Designed for the ski boot wearable to detect lean angle through pressure measurements.

## Features

- Read raw ADC values or calibrated voltage (mV)
- Automatic pressure threshold detection
- Calibration support (baseline and maximum pressure)
- Percentage-based pressure readings (0-100%)
- Easy-to-use API

## Basic Usage

```cpp
#include "fsr.hpp"

// Create FSR instance on ADC channel 0
SENSOR::fsr pressure_sensor(ADC_CHANNEL_0);

// Read voltage in millivolts
float voltage = pressure_sensor.read();

// Read raw ADC value (0-4095 for 12-bit)
int raw = pressure_sensor.read_raw();
```

## Pressure Threshold Detection

```cpp
// Set a threshold in mV - sensor will track when pressure exceeds this
pressure_sensor.set_pressure_threshold(1500.0f);

// Check if pressure is above threshold
while (true) {
    float voltage = pressure_sensor.read();  // This updates the pressed state

    if (pressure_sensor.is_pressed()) {
        printf("Pressure detected! %.2f mV\n", voltage);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

// Or check against a specific threshold
if (pressure_sensor.check_pressure(2000.0f)) {
    printf("Pressure exceeds 2000 mV\n");
}
```

## Calibration

For more accurate readings, calibrate the sensor:

```cpp
// Calibrate baseline (no pressure)
printf("Remove all pressure from sensor...\n");
vTaskDelay(pdMS_TO_TICKS(2000));
pressure_sensor.calibrate_baseline();
printf("Baseline: %.2f mV\n", pressure_sensor.get_baseline());

// Calibrate maximum (full pressure)
printf("Apply maximum pressure...\n");
vTaskDelay(pdMS_TO_TICKS(2000));
pressure_sensor.calibrate_max();
printf("Maximum: %.2f mV\n", pressure_sensor.get_max());

// Now you can read pressure as a percentage
while (true) {
    float percentage = pressure_sensor.read_percentage();
    printf("Pressure: %.1f%%\n", percentage);
    vTaskDelay(pdMS_TO_TICKS(100));
}
```

## Manual Calibration

If you already know the calibration values:

```cpp
// Set calibration values manually
pressure_sensor.set_calibration(100.0f, 2500.0f);  // baseline, max in mV
```

## Ski Boot Application Example

```cpp
#include "fsr.hpp"

// FSR connected to GPIO1 (ADC1_CHANNEL_0)
SENSOR::fsr lean_sensor(ADC_CHANNEL_0);

void detect_ski_lean() {
    // Calibrate when boot is not in use
    printf("Calibrating baseline (boot relaxed)...\n");
    lean_sensor.calibrate_baseline();

    printf("Apply maximum lean pressure...\n");
    vTaskDelay(pdMS_TO_TICKS(3000));
    lean_sensor.calibrate_max();

    // Set threshold at 30% for detecting intentional lean
    float threshold = lean_sensor.get_baseline() +
                     (lean_sensor.get_max() - lean_sensor.get_baseline()) * 0.3f;
    lean_sensor.set_pressure_threshold(threshold);

    // Monitor lean angle
    while (true) {
        float lean_percentage = lean_sensor.read_percentage();

        if (lean_percentage > 30.0f) {
            printf("Skier leaning: %.1f%%\n", lean_percentage);

            if (lean_percentage > 80.0f) {
                printf("Maximum lean detected!\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz sampling
    }
}
```

## Hardware Setup

- Connect FSR between VCC and ADC pin
- Use a pull-down resistor (10k typical) from ADC pin to GND
- The driver uses ADC_ATTEN_DB_12 (0-3300mV range)
- ADC unit is set to ADC_UNIT_1

## API Reference

### Constructor
- `fsr(adc_channel_t channel)` - Initialize FSR on specified ADC channel

### Reading Methods
- `float read()` - Read voltage in millivolts
- `int read_raw()` - Read raw ADC value
- `float read_percentage()` - Read pressure as percentage (requires calibration)

### Threshold Detection
- `void set_pressure_threshold(float threshold_mv)` - Set pressure threshold
- `float get_pressure_threshold()` - Get current threshold
- `bool is_pressed()` - Check if last reading exceeded threshold
- `bool check_pressure(float threshold_mv)` - Check if current reading exceeds threshold

### Calibration
- `void calibrate_baseline()` - Auto-calibrate baseline (no pressure)
- `void calibrate_max()` - Auto-calibrate maximum (full pressure)
- `void set_calibration(float baseline_mv, float max_mv)` - Manual calibration
- `float get_baseline()` - Get baseline value
- `float get_max()` - Get maximum value
