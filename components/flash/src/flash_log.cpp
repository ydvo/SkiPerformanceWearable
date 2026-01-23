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
    uint32_t num_sectors = dev_.size_bytes() / dev_.sector_size(); 
    uint32_t start_sector = esp_random() % num_sectors; 

    write_addr_ = start_sector * dev_.sector_size();
    read_addr_  = write_addr_;
    seq_ = 0;
    sample_idx_ = 0; 

    ESP_LOGI(
      TAG, "Initialized flash with read=0x%x write=0x%x seq=%u", read_addr_, write_addr_, seq_
    ); 

    return ESP_OK; 
  }

  template <typename T>
  esp_err_t FlashLog<T>::append(const T &sample, const uint64_t timestamp_us) {
    if (sample_idx_ == 0) start_timestamp_us_ = timestamp_us; 

    sample_buffer_[sample_idx_++] = sample; 

    if (sample_idx_ < SAMPLES_PER_FRAME) {
      return ESP_OK; 
    }

    Payload payload {};
    payload.start_t_us = start_timestamp_us_; 
    memcpy(payload.data, sample_buffer_.data(), sizeof(sample_buffer_)); 
    payload.end_t_us = timestamp_us; 

    sample_idx_ = 0; 
    return flush(payload); 
  }

  template <typename T>
  bool FlashLog<T>::is_full() const {
    return would_overrun(next_addr(write_addr_));
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
  esp_err_t FlashLog<T>::flush(const Payload &payload) {
    uint32_t next = next_addr(write_addr_); 
    if (would_overrun(next)) return ESP_ERR_NO_MEM; 

    if (write_addr_ % dev_.sector_size() == 0) {
      ESP_RETURN_ON_ERROR(
        erase_sector(write_addr_), 
        TAG, "Failed to erase the region before writing."
      );
    }

    Frame frame {}; 
    frame.magic = MAGIC;
    frame.seq = seq_; 
    frame.payload = payload;

    frame.crc = compute_crc(frame); 

    ESP_RETURN_ON_ERROR(
      dev_.write(write_addr_, &frame, sizeof(Frame)), 
      TAG, "Failed to write payload to the chip."
    );

    ESP_LOGI(TAG, "Flushed %d bytes of data into 0x%x address.", sizeof(Frame), write_addr_);

    write_addr_ = next; 
    ++seq_; 
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

    return next_sector == read_sector && read_addr_ != write_addr_;  
  }

  template <typename T>
  esp_err_t FlashLog<T>::read(Frame *dst, size_t max_frames, size_t *frames_read) {

    *frames_read = 0; 
    while (*frames_read < max_frames && read_addr_ != write_addr_) {
      Frame frame{}; 

      ESP_RETURN_ON_ERROR(
        dev_.read(read_addr_, &frame, sizeof(Frame)), 
        TAG, "Failed to read the flash device."
      );

      if (!valid(frame)) {
        read_addr_ = next_addr(read_addr_); 
        continue;
      }; 

      dst[*frames_read] = frame; 
      read_addr_ = next_addr(read_addr_); 
      (*frames_read)++; 
    }

    return ESP_OK; 
  }

  template <typename T>
  esp_err_t FlashLog<T>::erase_sector(uint32_t addr) {
    uint32_t sector = addr - (addr % dev_.sector_size());

    return dev_.erase_region(sector, dev_.sector_size()); 
  }
template class FlashLog<SENSORS::Imu::Quaternion>; 
}