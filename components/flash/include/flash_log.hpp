/**
 * @file flash_log.hpp
 * @ingroup storage
 * @brief Flash log wrapper for data frames.
 *
 * Provides a templated `FlashLog<T>` class that batches samples into fixed‑size
 * frames, computes CRC32 checksums, and stores them sequentially in SPI flash.
 * The log is double‑buffered with read/write pointers and validates frames on
 * read.
 */
#pragma once
#include "flash.hpp"
#include "imu.hpp"

namespace STORAGE {

/** @brief Magic number used to identify valid flash log frames. */
constexpr uint32_t MAGIC = 0xDEADBEEF;
/** @brief Number of samples stored per flash log frame. */
constexpr size_t SAMPLES_PER_FRAME = 10; 

/**
 * @brief Quaternion data type stored in flash frames.
 *
 * Each component is a 32‑bit float; the struct is packed to 16 bytes.
 */
struct __attribute__((packed)) Quaternion {
  float w;
  float x;
  float y;
  float z;
};

static_assert(sizeof(Quaternion) == 16, "Quaternion must be 16 bytes packed."); 

template <typename T>
class FlashLog { 
public: 
  /**
   * @brief Single sample entry.
+   *
+   * @param timestamp_us Timestamp in microseconds when the sample was taken.
+   * @param data         Sample payload of type `T` (e.g., Quaternion).
+   */
  struct Sample {
    uint64_t timestamp_us; 
    T data;
  } __attribute__((packed));

  /**
   * @brief Flash log frame layout.
+   *
+   * `magic` marks the frame start, `seq` is a monotonically increasing frame
+   * sequence number, `payload` holds `SAMPLES_PER_FRAME` samples, `padding`
+   * reserves space to align the struct to 256 bytes, and `crc` stores a CRC32
+   * checksum of the frame (excluding the CRC field itself).
+   */
  struct Frame {
    uint32_t magic; 
    uint32_t seq;
    std::array<Sample, SAMPLES_PER_FRAME> payload;  
    uint32_t padding; 
    uint32_t crc;
  } __attribute__((packed));

  static_assert(sizeof(Frame) == 256, "frame size expected to be 256 bytes");
  static_assert(std::is_trivially_copyable_v<Frame>, "frame must be trivially copyable"); 

  /**
   * @brief Construct a FlashLog bound to a specific SPI flash device.
+   *
+   * @param dev Reference to an already‑constructed `SpiFlashDevice`.
+   */
  explicit FlashLog(STORAGE::SpiFlashDevice &dev) noexcept;
  /**
   * @brief Initialize the flash log.
+   *
+   * Randomizes the start sector to distribute wear, resets write/read pointers,
+   * and clears the sample buffer. Must be called before any other operation.
+   * @return ESP_OK on success.
+   */
  esp_err_t init();
  /**
   * @brief Append a sample to the log.
+   *
+   * Samples are buffered until `SAMPLES_PER_FRAME` is reached, then a frame is
+   * flushed to flash. The timestamp is stored alongside the sample data.
+   *
+   * @param sample Sample payload of type `T`.
+   * @param timestamp_us Timestamp in microseconds.
+   * @return ESP_OK on success, or an error if flushing fails.
+   */
  esp_err_t append(const T &sample, const uint64_t timestamp_us);
  /**
   * @brief Read up to `max_frames` from the log into `dst`.
+   *
+   * The function reads sequential frames starting from the current read pointer
+   * and stops when `max_frames` have been copied or the write pointer is
+   * reached. Valid frames are verified via CRC; invalid frames are skipped.
+   *
+   * @param dst Destination buffer (array of `Frame`).
+   * @param max_frames Maximum number of frames to read.
+   * @param frames_read Output parameter: number of frames actually read.
+   * @return ESP_OK on success.
+   */
  esp_err_t read(Frame *dst, size_t max_frames, size_t *frames_read);
  /**
   * @brief Retrieve the current read address (byte offset).
+   * @return Read address.
+   */
  uint32_t read_addr() const;
  /**
   * @brief Immutable read of frames starting from a specific address.
+   *
+   * Does not modify the internal read pointer. Useful for replaying data from a
+   * known position.
+   *
+   * @param dst Destination buffer for frames.
+   * @param max_frames Maximum frames to read.
+   * @param frames_read Output count of frames read.
+   * @param read_addr_from Starting address (byte offset) for this read.
+   * @param read_addr_new Output: address after the last frame read.
+   * @return ESP_OK on success.
+   */
  esp_err_t read_immut(Frame *dst, size_t max_frames, size_t *frames_read, uint32_t read_addr_from, uint32_t *read_addr_new) const;
  /**
   * @brief Check if the log is full (next write would overrun the read sector).
+   * @return True if full, false otherwise.
+   */
  bool is_full() const;

private: 
  // persistent write and read state
  uint32_t write_addr_{0}; 
  uint32_t read_addr_{0}; 
  uint32_t seq_{0}; 

  // batching
  std::array<Sample, SAMPLES_PER_FRAME> sample_buffer_{}; 
  uint32_t sample_idx_{0}; 

  STORAGE::SpiFlashDevice &dev_;

  uint32_t compute_crc(const Frame &frame) const; 
  bool valid(const Frame &frame) const; 

  uint32_t next_addr(uint32_t addr) const;
  bool would_overrun(uint32_t next_write) const;

  esp_err_t erase_sector(uint32_t addr);
  esp_err_t flush(const std::array<Sample, SAMPLES_PER_FRAME>& payload); 
}; 
}