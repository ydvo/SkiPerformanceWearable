/**
 * @file powerstats.hpp
 * @ingroup power
 * @brief Track and monitor power statistics.
 */

#pragma once
#include "fuelgauge.hpp"
#include <cstdint>
#include <map>
#include <vector>

namespace POWER {

class powerstats {
public:
  enum class Activity {
    IDLE,
    BLUETOOTH,
    CPU_INTENSIVE,
    IMU_READING,
    DISPLAY,
    GPS,
    STORAGE_WRITE,
    STORAGE_READ,
    NETWORK,
    SENSOR_FUSION,
    CUSTOM
  };

  struct snapshot {
    float voltage_v;
    float soc_percent;
    int64_t timestamp_us; // microseconds since boot
  };

  struct activity_stats {
    Activity activity;
    float total_energy_mwh; // milliwatt-hours consumed
    float avg_power_mw;     // average power in milliwatts
    uint32_t sample_count;
    uint32_t total_duration_ms;
    int64_t last_start_time_us; // timestamp of last start
    bool is_active;
  };
  static constexpr float BATTERY_CAPACITY_MAH{2000};

  explicit powerstats(fuelgauge &fg);

  // Battery snapshot methods
  snapshot take_snapshot();
  const snapshot &get_last_snapshot() const;
  const std::vector<snapshot> &get_snapshot_history() const;

  // Activity tracking methods
  void start_activity(Activity activity);
  void stop_activity(Activity activity);
  bool is_activity_running(Activity activity) const;

  // Statistics methods
  activity_stats get_activity_stats(Activity activity) const;
  std::map<Activity, activity_stats> get_all_stats() const;
  void reset_stats();
  void reset_activity_stats(Activity activity);

  // Power calculation methods
  float calculate_power_mw(const snapshot &start, const snapshot &end, uint32_t duration_ms) const;
  float get_current_power_mw() const;

  // Utility methods
  static const char *activity_to_string(Activity activity);
  void enable_auto_snapshot(bool enable, uint32_t interval_ms = 1000);

private:
  fuelgauge &fuel_gauge_;
  std::vector<snapshot> snapshot_history_;
  std::map<Activity, activity_stats> stats_map_;
  snapshot last_snapshot_;
  bool auto_snapshot_enabled_;
  uint32_t auto_snapshot_interval_ms_;
  int64_t last_auto_snapshot_time_us_;

  void update_activity_energy(Activity activity, const snapshot &start, const snapshot &end,
                              uint32_t duration_ms);
  int64_t get_time_us() const;
};

} // namespace POWER
