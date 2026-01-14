#pragma once
#include "flash.hpp"

namespace STORAGE {

constexpr uint32_t MAGIC = 0xDEADBEEF;

template <typename T>
class FlashLog { 
public: 
  struct Payload {
    int64_t start_t_us;
    T data[14]; 
    int64_t end_t_us;
  };

  struct Frame {
    uint32_t magic; 
    uint32_t seq; 
    uint32_t sample_idx; 
    Payload payload; 
    uint32_t crc; 
  } __attribute__((packed));

  static_assert(sizeof(Frame) == 256, "frame size expected to be 256 bytes");
  static_assert(std::is_trivially_copyable_v<Frame>, "frame must be trivially copyable"); 

  explicit FlashLog(STORAGE::SpiFlashDevice &dev, uint32_t base) noexcept;
  esp_err_t append(const T &sample, const uint64_t timestamp_us);

private: 
  T buffer_[14]; 
  STORAGE::SpiFlashDevice &dev_; 
  uint32_t write_addr_;
  uint32_t seq_; 

  uint32_t compute_crc(const Frame &frame) const; 
  esp_err_t flush(const Payload& payload); 
}; 

struct ImuValue {
  float w; 
  float x; 
  float y; 
  float z; 
};
}