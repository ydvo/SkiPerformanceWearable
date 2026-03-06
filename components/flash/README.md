# Flash Component

**Purpose**
Manages external SPI flash storage for persistent logging. Provides a thin wrapper over the ESP‑IDF SPI flash driver with CRC‑protected log records.

**Key Headers**
- `flash.hpp` – primary flash device abstraction (`STORAGE::SpiFlashDevice`).
- `flash_log.hpp` – circular log that writes frames with CRC, supports reading back for post‑flight analysis.

**Public API (excerpt)**
- `STORAGE::SpiFlashDevice::init()` – configure SPI peripheral and verify chip ID.
- `STORAGE::SpiFlashDevice::write(uint32_t addr, const void *data, size_t len)` – raw write.
- `STORAGE::SpiFlashDevice::read(uint32_t addr, void *out, size_t len)` – raw read.
- `STORAGE::FlashLog::append(const void *record, size_t size)` – append a CRC‑checked record; automatically wraps when full.
- `STORAGE::FlashLog::read_latest(void *buffer, size_t max_len)` – retrieve most recent record.

**Thread‑Safety**
`FlashLog` internally protects the write pointer with a mutex (FreeRTOS semaphore) to allow concurrent appends from multiple tasks.

**Typical Usage**
```cpp
STORAGE::SpiFlashDevice flash;
flash.init();

STORAGE::FlashLog log(flash, 0x100000, 0x20000); // base address, size
log.append(&sample, sizeof(sample));
```