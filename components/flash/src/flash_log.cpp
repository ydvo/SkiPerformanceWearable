/**
 * @file flash_log.cpp - Implementation of templated flash log.
 *
 * Defines the `FlashLog<T>` member functions for managing buffered writes,
 * frame construction, CRC computation, and reading with validation.
 */
#include "flash_log.hpp"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_check.h"
#include "imu.hpp"
#include "esp_random.h"

namespace STORAGE {

  static const char *TAG = "FLASH_LOG";

  template <typename T>
  FlashLog<T>::FlashLog(STORAGE::SpiFlashDevice &dev) noexcept:
    dev_{dev} {}

  template <typename T>
  esp_err_t FlashLog<T>::init() {
    if (mutex_ == nullptr) {
      mutex_ = xSemaphoreCreateMutex();
      if (mutex_ == nullptr) return ESP_ERR_NO_MEM;
    }

    uint32_t num_sectors = dev_.size_bytes() / dev_.sector_size();
    uint32_t start_sector = esp_random() % num_sectors;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    write_addr_ = start_sector * dev_.sector_size();
    read_addr_  = write_addr_;
    seq_ = 0;
    sample_idx_ = 0;
    xSemaphoreGive(mutex_);

    ESP_LOGI(
      TAG, "Initialized flash with read=0x%x write=0x%x seq=%u", read_addr_, write_addr_, seq_
    );

    return ESP_OK;
  }

  template <typename T>
  esp_err_t FlashLog<T>::append(const T &sample, const uint64_t timestamp_us) {
    xSemaphoreTake(mutex_, portMAX_DELAY);

    sample_buffer_[sample_idx_++] = {
      .timestamp_us = timestamp_us,
      .data = sample
    };

    if (sample_idx_ < SAMPLES_PER_FRAME) {
      xSemaphoreGive(mutex_);
      return ESP_OK;
    }

    // Buffer full — snapshot what flush() needs, then release before SPI I/O.
    std::array<Sample, SAMPLES_PER_FRAME> payload{sample_buffer_};
    sample_idx_ = 0;

    uint32_t w    = write_addr_;
    uint32_t next = next_addr(w);
    uint32_t seq  = seq_;

    if (would_overrun(next)) {
      xSemaphoreGive(mutex_);
      return ESP_ERR_NO_MEM;
    }

    xSemaphoreGive(mutex_);

    // SPI I/O without the mutex held.
    esp_err_t err = flush(payload, w, seq);
    if (err != ESP_OK) return err;

    // Update pointers under the mutex.
    xSemaphoreTake(mutex_, portMAX_DELAY);
    write_addr_ = next;
    ++seq_;
    xSemaphoreGive(mutex_);

    return ESP_OK;
  }

  template <typename T>
  bool FlashLog<T>::is_full() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool full = would_overrun(next_addr(write_addr_));
    xSemaphoreGive(mutex_);
    return full;
  }

  template <typename T>
  uint32_t FlashLog<T>::compute_crc(const Frame &frame) const {
    return esp_crc32_le(
      0, reinterpret_cast<const uint8_t *> (&frame), offsetof(Frame, crc)
    );
  }

  template <typename T>
  bool FlashLog<T>::valid(const Frame &frame) const {
    return frame.magic == MAGIC && frame.crc == compute_crc(frame);
  }

  template <typename T>
  esp_err_t FlashLog<T>::flush(const std::array<Sample, SAMPLES_PER_FRAME> &payload,
                               uint32_t addr, uint32_t seq_num) {
    if (addr % dev_.sector_size() == 0) {
      ESP_RETURN_ON_ERROR(
        erase_sector(addr),
        TAG, "Failed to erase the region before writing."
      );
    }

    Frame frame {};
    frame.magic   = MAGIC;
    frame.seq     = seq_num;
    frame.payload = payload;
    frame.crc     = compute_crc(frame);

    ESP_RETURN_ON_ERROR(
      dev_.write(addr, &frame, sizeof(Frame)),
      TAG, "Failed to write payload to the chip."
    );

    ESP_LOGI(TAG, "Flushed %d bytes of data into 0x%x address.", sizeof(Frame), addr);

    return ESP_OK;
  }

  template <typename T>
  uint32_t FlashLog<T>::next_addr(uint32_t addr) const {
    addr += sizeof(Frame);
    if (addr + sizeof(Frame) > dev_.size_bytes()) {
      addr = 0;
    }

    return addr;
  }

  template <typename T>
  bool FlashLog<T>::would_overrun(uint32_t next_write) const {
    uint32_t next_sector = next_write / dev_.sector_size();
    uint32_t read_sector = read_addr_ / dev_.sector_size();

    return next_sector == read_sector && next_write % dev_.sector_size() == 0;
  }

  template <typename T>
  esp_err_t FlashLog<T>::read(Frame *dst, size_t max_frames, size_t *frames_read) {
    *frames_read = 0;

    // Snapshot both pointers under the mutex so we don't race with flush().
    xSemaphoreTake(mutex_, portMAX_DELAY);
    uint32_t cur_read   = read_addr_;
    uint32_t snap_write = write_addr_;
    xSemaphoreGive(mutex_);

    // Limit invalid-frame skips per call to one sector's worth of frames.
    // This prevents the read from blocking for seconds scanning erased regions
    // while the flash writer holds the SPI bus for a sector erase.
    const size_t max_skip = dev_.sector_size() / sizeof(Frame);
    size_t skipped = 0;

    while (*frames_read < max_frames && cur_read != snap_write && skipped < max_skip) {
      Frame frame{};
      esp_err_t err = dev_.read(cur_read, &frame, sizeof(Frame));
      if (err != ESP_OK) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        read_addr_ = cur_read;
        xSemaphoreGive(mutex_);
        return err;
      }

      uint32_t next = next_addr(cur_read);
      cur_read = next;

      if (!valid(frame)) {
        skipped++;
        continue;
      }

      dst[(*frames_read)++] = frame;
    }

    // Commit the new read pointer.
    xSemaphoreTake(mutex_, portMAX_DELAY);
    read_addr_ = cur_read;
    xSemaphoreGive(mutex_);

    return ESP_OK;
  }

  template <typename T>
  uint32_t FlashLog<T>::read_addr() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    uint32_t addr = read_addr_;
    xSemaphoreGive(mutex_);
    return addr;
  }

  template <typename T>
  esp_err_t FlashLog<T>::read_immut(Frame *dst, size_t max_frames, size_t *frames_read,
                                    uint32_t read_addr_from, uint32_t *read_addr_new) const {
    *frames_read = 0;
    uint32_t read_addr = read_addr_from;

    xSemaphoreTake(mutex_, portMAX_DELAY);
    uint32_t snap_write = write_addr_;
    xSemaphoreGive(mutex_);

    while (*frames_read < max_frames && read_addr != snap_write) {
      Frame frame{};
      ESP_RETURN_ON_ERROR(
        dev_.read(read_addr, &frame, sizeof(Frame)),
        TAG, "Failed to read the flash device."
      );

      if (!valid(frame)) {
        read_addr = next_addr(read_addr);
        continue;
      }

      dst[(*frames_read)++] = frame;
      read_addr = next_addr(read_addr);
    }

    *read_addr_new = read_addr;

    return ESP_OK;
  }

  template <typename T>
  esp_err_t FlashLog<T>::erase_sector(uint32_t addr) {
    uint32_t sector = addr - (addr % dev_.sector_size());

    return dev_.erase_region(sector, dev_.sector_size());
  }

template class FlashLog<Quaternion>;
}
