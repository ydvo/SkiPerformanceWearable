#pragma once
#include "flash.hpp"

namespace STORAGE {

constexpr uint32_t MAGIC = 0xDEADBEEF;

template <typename T>
class FlashLog { 
public: 
  struct Frame {
    uint32_t magic; 
    uint32_t seq; 
    uint32_t sample_idx; 
    T payload; 
    uint32_t crc; 
  } __attribute__((packed));

  static_assert(256 % sizeof(Frame) == 0, "frame size must divide page size");
  static_assert(std::is_trivially_copyable_v<Frame>, "frame must be trivially copyable"); 

  explicit FlashLog(STORAGE::SpiFlashDevice &dev, uint32_t base) noexcept;
  esp_err_t append(const T& sample);

private: 
  STORAGE::SpiFlashDevice &dev_; 
  uint32_t write_addr_;
  uint32_t seq_; 

  uint32_t compute_crc(const Frame &frame) const; 
}; 

struct Payload {
  float w;
  float x; 
  float y; 
  float z; 
};
}