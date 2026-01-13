#include "flash_log.hpp"
#include "esp_crc.h"

namespace STORAGE {

  static const char *TAG = "FLASH_LOG"; 

  template <typename T>
  FlashLog<T>::FlashLog(STORAGE::SpiFlashDevice &dev, uint32_t base) noexcept: 
    dev_{dev}, write_addr_{base}, seq_{0} {}

  template <typename T>
  esp_err_t FlashLog<T>::append(const T &sample) {
    Frame frame {}; 
    frame.magic = MAGIC; 
    frame.sample_idx = seq_; 
    frame.seq = seq_; 
    frame.payload = sample; 
    frame.crc = compute_crc(frame);

    return dev_.write(write_addr_, &frame, sizeof(frame)); 
  }

  template <typename T>
  uint32_t FlashLog<T>::compute_crc(const Frame &frame) const {
    return esp_crc32_le(
      0, reinterpret_cast<const uint8_t *> (&frame), offsetof(Frame, crc)
    );
  }

template class FlashLog<Payload>; 
}