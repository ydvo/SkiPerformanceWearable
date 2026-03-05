# Ski Performance Wearable

A wearable device for tracking ski performance metrics using an ESP32-S3 microcontroller. The device captures orientation data via a 9-axis IMU, detects lean angle using a force sensitive resistor, and logs data to external flash for later analysis.

## Features

- Real-time orientation tracking at 100Hz using ICM-20948 9-axis IMU with Madgwick sensor fusion
- Lean angle detection via force sensitive resistor (FSR) with continuous haptic warning
- Physical button calibration for FSR threshold (long press)
- Data logging to external SPI flash with CRC integrity checking
- BLE connectivity for wireless data transfer with ACK-based flow control
- Battery monitoring via MAX1704X fuel gauge
- Haptic feedback using DRV2605L motor driver (event-based and real-time playback)
- FreeRTOS task-based architecture with event-driven state machine

## Hardware

- ESP32-S3 microcontroller (Xtensa dual-core)
- ICM-20948 9-axis IMU (accelerometer, gyroscope, magnetometer)
- Force sensitive resistor for lean angle detection
- External SPI flash for data storage
- MAX1704X battery fuel gauge
- DRV2605L haptic motor driver (ERM motor)
- Status LED

## Build Environment

- **ESP-IDF**: v5.5.1
- **Target chip**: ESP32-S3
- **Flash size**: 2 MB
- **Flash mode**: DIO @ 80 MHz
- **BLE stack**: NimBLE (esp-nimble-cpp v2.3.4)
- **Compiler optimization**: Debug (with -O3 override in CMakeLists)

## Project Structure

```
.
├── main/                 Main application code
├── components/           Custom components
│   ├── ble/             BLE GATT server
│   ├── common/          GPIO utilities
│   ├── encoder/         Rotary encoder driver
│   ├── flash/           SPI flash storage and flash log
│   ├── fsr/             Force sensitive resistor driver
│   ├── haptics/         DRV2605L haptic driver
│   ├── imu/             ICM-20948 IMU with Madgwick filter
│   ├── led/             LED control
│   └── power/           Battery monitoring (MAX1704X)
├── docs/                 Datasheets
└── scripts/              Python utilities for data processing
```

## Setup

### Prerequisites

- ESP-IDF v5.5.1 or later
- Python 3.x (for utility scripts)

### Installation

1. Install ESP-IDF following the official guide:
   https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/get-started/

2. Clone this repository:
   ```
   git clone <repository-url>
   cd SkiPerformanceWearable
   ```

3. Set up the ESP-IDF environment:
   ```
   . $IDF_PATH/export.sh
   ```

4. Set the target to ESP32-S3:
   ```
   idf.py set-target esp32s3
   ```

5. Build the project:
   ```
   idf.py build
   ```

### Flashing

Flash the firmware to the device over USB:

```
idf.py -p <PORT> flash
```

Where `<PORT>` is the serial port (e.g. `/dev/ttyUSB0` on Linux, `/dev/cu.usbmodem*` on macOS).

To build and flash in one step:

```
idf.py -p <PORT> flash monitor
```

The flash command writes three binaries to the chip:

| Binary                  | Address  | Description              |
|-------------------------|----------|--------------------------|
| bootloader.bin          | 0x0      | Second-stage bootloader  |
| partition-table.bin     | 0x8000   | Partition table          |
| ski_performance_wearable.bin | 0x10000 | Application firmware |

### Monitoring Logs

Open the serial monitor to view runtime logs:

```
idf.py -p <PORT> monitor
```

The default baud rate is 921600. Press `Ctrl+]` to exit the monitor.

Log levels are configured per-component in `app_main()`:
- `FLASH_LOG` and `BLE` are set to `ESP_LOG_WARN` to reduce noise
- The main logger uses `espp::Logger` at INFO level

To change the default log level, modify `CONFIG_LOG_DEFAULT_LEVEL` in
menuconfig (`idf.py menuconfig` > Component config > Log output).

### Partition Table

The project uses the default ESP-IDF single-app partition layout
(`partitions_singleapp.csv`). The partition table is located at offset
0x8000 and the application starts at 0x10000.

| Partition | Type | Offset  | Size   |
|-----------|------|---------|--------|
| nvs       | data | 0x9000  | 24 KB  |
| phy_init  | data | 0xf000  | 4 KB   |
| factory   | app  | 0x10000 | 1 MB   |

The external SPI flash (connected via SPI2) is managed directly by the
flash component and is not part of the ESP32's internal partition table.
It stores sensor data frames using the FlashLog driver.

## Control Flow

The device is driven by a FreeRTOS event-driven state machine. States are
tracked with an EventGroup and transitions are dispatched through an event
queue processed by the control task.

### States

```
IDLE  -->  READY  -->  RUNNING
  ^          |  ^         |
  |          |  |         |
  |          v  |         |
  |      CALIBRATING      |
  |                       |
  +-----------------------+
```

- **IDLE** -- BLE advertising, waiting for a client to connect.
- **READY** -- BLE connected, waiting for the user to start a run or
  calibrate.
- **RUNNING** -- Actively sampling the IMU at 100 Hz, writing to flash,
  uploading over BLE, and monitoring FSR pressure with haptic warnings.
- **CALIBRATING** -- Two-phase FSR calibration sequence (entered from READY
  via long press).

### State Transitions

| Current State | Event              | Next State  | Action                              |
|---------------|--------------------|-------------|-------------------------------------|
| IDLE          | BLE_CONNECTED      | READY       | Haptic: BLE connect                 |
| READY         | TOGGLE_RUN         | RUNNING     | Start sensors, haptic: run start    |
| READY         | START_CALIBRATION  | CALIBRATING | Begin FSR calibration sequence      |
| READY         | BLE_DISCONNECTED   | IDLE        | Re-advertise, haptic: BLE disconnect|
| RUNNING       | TOGGLE_RUN         | READY       | Stop recording, haptic: run stop    |
| RUNNING       | BLE_DISCONNECTED   | IDLE        | Re-advertise, haptic: BLE disconnect|
| CALIBRATING   | CALIBRATION_DONE   | READY       | Threshold updated                   |
| CALIBRATING   | BLE_DISCONNECTED   | IDLE        | Abort, re-advertise                 |

### Button Controls

The physical button (GPIO 6) is polled at 20 Hz by the button task. It is now used **only for power management**:

- **Long press** (≥ 3 s): Sends `SLEEP_REQUEST` event. The device will enter deep‑sleep. Deep‑sleep can be exited only by a **sustained hold** of the button after wake‑up (see *Wake‑up handling*).
- **Short press**: Ignored – runs are started and stopped exclusively via BLE control commands.

### FSR Calibration Sequence

When the user long-presses the boot button in READY state:

1. Haptic ramp-up plays -- release all pressure from the FSR.
2. 3-second delay, then 10 baseline samples are averaged.
3. Haptic double-click plays -- apply full pressure to the FSR.
4. 3-second delay, then 10 max-pressure samples are averaged.
5. Threshold is computed at 50% of the calibrated range.
6. Haptic ramp-down plays -- calibration complete.
7. Device returns to READY state.

Calibration values are session-only and reset to defaults on reboot.

### FSR Lean Warning

During RUNNING state the FSR task polls the pressure sensor at 20 Hz. When
the reading exceeds the configured threshold:

- The DRV2605L switches to real-time playback mode and continuously drives
  the haptic motor.
- When pressure drops below the threshold, the motor stops and the driver
  returns to internal trigger mode for normal haptic events.

### FreeRTOS Tasks

| Task          | Priority | Stack  | Description                                    |
|---------------|----------|--------|------------------------------------------------|
| control       | 9        | 4096   | State machine, processes event queue           |
| imu_task      | 8        | 4096   | Samples IMU at 100 Hz, pushes to queue         |
| button        | 7        | 2048   | Polls boot button, detects short/long press    |
| flash_writer  | 6        | 4096   | Batch-reads IMU queue, writes to flash         |
| haptic        | 5        | 3072   | Plays haptic effect sequences from queue       |
| fsr_task      | 5        | 4096   | Monitors FSR, drives real-time haptic warning  |
| calibrate     | 5        | 4096   | Runs FSR calibration sequence when triggered   |
| upload        | 4        | 4096   | Reads flash frames, sends over BLE with ACKs   |
| battery       | 2        | 3072   | Updates BLE battery level every 30 s           |

## sdkconfig Notes

The project uses a single `sdkconfig` file (no `sdkconfig.defaults`).
Notable settings:

- **Target**: `CONFIG_IDF_TARGET="esp32s3"`
- **Flash**: 2 MB, DIO mode, 80 MHz
- **Bluetooth**: NimBLE stack enabled (`CONFIG_BT_NIMBLE_ENABLED`), max 3
  connections, preferred MTU 256, pinned to core 0
- **FreeRTOS tick rate**: 100 Hz (`CONFIG_FREERTOS_HZ=100`)
- **Log level**: INFO by default (`CONFIG_LOG_DEFAULT_LEVEL_INFO`)
- **Monitor baud**: 921600 (`CONFIG_ESPTOOLPY_MONITOR_BAUD`)
- **Main task stack**: 3584 bytes

When changing settings, run `idf.py menuconfig` and then rebuild. The
`sdkconfig` file is checked into the repository -- do not add it to
`.gitignore`.

## Important Note: Magnetometer Fix

Until the espp repository is updated with a new release, you need to
manually copy over the most recent `icm20948.cpp` file from the espp project to the
`managed_components/espp__icm20948` directory in order avoid initialisation
issues with the gyroscope low pass filter.

## License

See LICENSE file for details.
