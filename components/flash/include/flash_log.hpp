#pragma once
#include "flash.hpp"

namespace STORAGE {

constexpr uint32_t MAGIC = 0xDEADBEEF;
constexpr size_t SAMPLES_PER_FRAME = 14; 

template <typename T>
class FlashLog { 
public: 
  struct Payload {
    int64_t start_t_us;
    T data[SAMPLES_PER_FRAME]; 
    int64_t end_t_us;
  } __attribute__((packed));

  struct Frame {
    uint32_t magic; 
    uint32_t seq;
    Payload payload;  
    uint32_t padding; 
    uint32_t crc;
  } __attribute__((packed));

  static_assert(sizeof(Frame) == 256, "frame size expected to be 256 bytes");
  static_assert(std::is_trivially_copyable_v<Frame>, "frame must be trivially copyable"); 

  explicit FlashLog(STORAGE::SpiFlashDevice &dev) noexcept;
  esp_err_t init(); 
  esp_err_t append(const T &sample, const uint64_t timestamp_us);
  esp_err_t read(Frame *dst, size_t max_frames, size_t *frames_read);
  bool is_full() const;

private: 
  // persistent write and read state
  uint32_t write_addr_{0}; 
  uint32_t read_addr_{0}; 
  uint32_t seq_{0}; 

  // batching
  std::array<T, SAMPLES_PER_FRAME> sample_buffer_{}; 
  uint32_t sample_idx_{0}; 
  uint64_t start_timestamp_us_{0}; 

  STORAGE::SpiFlashDevice &dev_;

  uint32_t compute_crc(const Frame &frame) const; 
  bool valid(const Frame &frame) const; 

  uint32_t next_addr(uint32_t addr) const;
  bool would_overrun(uint32_t next_write) const;

  esp_err_t erase_sector(uint32_t addr);
  esp_err_t flush(const Payload& payload); 
}; 
}