# Ski Performance Wearable

A wearable device for tracking ski performance metrics using an ESP32-S3 microcontroller. The device captures orientation data via a 9-axis IMU, detects lean angle using a force sensitive resistor, and logs data to external flash for later analysis.

## Features

- Real-time orientation tracking at 100Hz using ICM-20948 9-axis IMU with Madgwick sensor fusion
- Lean angle detection via force sensitive resistor (FSR)
- Data logging to external SPI flash with CRC integrity checking
- BLE connectivity for wireless data transfer
- Battery monitoring via MAX1704X fuel gauge
- Haptic feedback using DRV2605L motor driver
- State machine for recording control (IDLE -> RECORDING -> COMPLETE -> FLUSHING -> UPLOADING)

## Hardware

- ESP32-S3 microcontroller
- ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer)
- Force sensitive resistor for lean angle detection
- External SPI flash for data storage
- MAX1704X battery fuel gauge
- DRV2605L haptic motor driver
- Status LED

## Project Structure

```
.
├── main/                 Main application code
├── components/           Custom components
│   ├── ble/             BLE GATT server
│   ├── common/          GPIO utilities
│   ├── flash/           SPI flash storage
│   ├── fsr/             Force sensitive resistor driver
│   ├── haptics/         DRV2605L haptic driver
│   ├── imu/             ICM-20948 IMU with Madgwick filter
│   ├── led/             LED control
│   └── power/           Battery monitoring
├── docs/                 Datasheets
└── scripts/              Python utilities for data processing
```

## Setup

### Prerequisites

- ESP-IDF v5.5 or later
- Python 3.x (for utility scripts)

### Installation

1. Install ESP-IDF following the official guide:
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/

2. Clone this repository:
   ```
   git clone <repository-url>
   cd SkiPerformanceWearable
   ```

3. Set up the ESP-IDF environment:
   ```
   . $IDF_PATH/export.sh
   ```

4. Build the project:
   ```
   idf.py build
   ```

5. Flash to device:
   ```
   idf.py -p <PORT> flash
   ```

6. Monitor serial output:
   ```
   idf.py -p <PORT> monitor
   ```

## Important Note: Magnetometer Fix

Until the espp repository is updated with a new release, you need to manually copy over the `icm20948.cpp` file from this project to the managed_components/espp__icm20948 directory in order to get accurate magnetometer data. The upstream version has issues with magnetometer readings that have been fixed locally.

## Usage

- Press the boot button to cycle through states
- Data is recorded when in RECORDING state
- Use the Python scripts in `scripts/` to convert flash dumps to CSV for analysis

## License

See LICENSE file for details.
