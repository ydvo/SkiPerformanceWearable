#include "flash_log.hpp"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_check.h"

namespace STORAGE {

  static const char *TAG = "FLASH_LOG"; 

  template <typename T>
  FlashLog<T>::FlashLog(STORAGE::SpiFlashDevice &dev) noexcept: 
    dev_{dev}, write_addr_{0}, seq_{0} {}

  template <typename T>
  esp_err_t FlashLog<T>::init() {
    write_addr_ = scan_flash(); 
    size_t sector_size = dev_.sector_size(); 
    uint32_t erase_addr = write_addr_ - (write_addr_ % sector_size); 

    ESP_RETURN_ON_ERROR(
      dev_.erase_region(erase_addr, sector_size), 
      TAG, "Failed to erase the region on initialization"
    ); 

    ESP_LOGI(TAG, "Initialized FlashLog with a write address of 0x%x. ", write_addr_); 
    return ESP_OK; 
  }

  template <typename T>
  esp_err_t FlashLog<T>::append(const T &sample, const uint64_t timestamp_us) {
    static uint32_t sample_idx = 0; 
    static uint64_t start_timestamp_us = timestamp_us; 

    buffer_[sample_idx] = sample;
    if (sample_idx == 0) {
      start_timestamp_us = timestamp_us; 
    }

    if (++sample_idx < 14) {
      return ESP_OK; 
    }

    Payload payload {};
    payload.start_t_us = start_timestamp_us; 
    memcpy(payload.data, buffer_, sizeof(buffer_)); 
    payload.end_t_us = timestamp_us; 

    sample_idx = 0; 

    return flush(payload); 
  }

  template <typename T>
  uint32_t FlashLog<T>::compute_crc(const Frame &frame) const {
    return esp_crc32_le(
      0, reinterpret_cast<const uint8_t *> (&frame), offsetof(Frame, crc)
    );
  }

  template <typename T>
  esp_err_t FlashLog<T>::flush(const Payload &payload) {
    Frame frame {}; 
    frame.magic = MAGIC; 
    frame.sample_idx = seq_; 
    frame.seq = seq_; 
    frame.payload = payload; 

    frame.crc = compute_crc(frame); 

    if (seq_ != 0 && write_addr_ % dev_.sector_size() == 0) {
      ESP_RETURN_ON_ERROR(
        dev_.erase_region(write_addr_, dev_.sector_size()), 
        TAG, "Failed to erase the region before writing."
      ); 
    }; 

    ESP_RETURN_ON_ERROR(
      dev_.write(write_addr_, &frame, sizeof(Frame)), 
      TAG, "Failed to write payload to the chip."
    );

    ESP_LOGI(TAG, "Flushed %d bytes of data into 0x%x address.", sizeof(Frame), write_addr_);

    write_addr_ += sizeof(Frame); 
    if (dev_.size_bytes() - write_addr_ < sizeof(Frame)) {
      write_addr_ = 0; 
    }

    ++seq_; 

    return ESP_OK; 
  }

  template <typename T>
  uint32_t FlashLog<T>::scan_flash() const {
    uint32_t read_addr {0x0}; 
    Frame current_frame {};

    while (true) {
      if (dev_.read(read_addr, &current_frame, sizeof(Frame)) != ESP_OK) {
        break; 
      }

      if (current_frame.magic != MAGIC || current_frame.crc != compute_crc(current_frame)) {
        break; 
      }

      read_addr += sizeof(Frame); 
    }

    return read_addr;
  }

template class FlashLog<ImuValue>; 
}