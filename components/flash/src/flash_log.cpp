#include "flash_log.hpp"
#include "esp_crc.h"
#include "esp_log.h"

namespace STORAGE {

  static const char *TAG = "FLASH_LOG"; 

  template <typename T>
  FlashLog<T>::FlashLog(STORAGE::SpiFlashDevice &dev, uint32_t base) noexcept: 
    dev_{dev}, write_addr_{base}, seq_{0} {}

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

    esp_err_t err = dev_.write(write_addr_, &frame, sizeof(Frame)); 
    if (err != ESP_OK) {
      return err;
    }

    ESP_LOGI(TAG, "Flushed %d bytes of data into 0x%x address.", sizeof(Frame), write_addr_);

    write_addr_ += sizeof(Frame); 
    ++seq_; 

    return ESP_OK; 
  }

template class FlashLog<ImuValue>; 
}