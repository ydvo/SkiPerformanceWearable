/*
 * powerstats.cpp
 *  - track and monitor power stats
 */

#include "powerstats.hpp"
#include <esp_timer.h>

using namespace POWER;

powerstats::powerstats(fuelgauge &fg)
    : fuel_gauge_(fg), auto_snapshot_enabled_(false), auto_snapshot_interval_ms_(1000),
      last_auto_snapshot_time_us_(0) {

  // Initialize with a first snapshot
  last_snapshot_ = take_snapshot();

  // Initialize all activity stats
  for (int i = static_cast<int>(Activity::IDLE); i <= static_cast<int>(Activity::CUSTOM); ++i) {
    Activity activity = static_cast<Activity>(i);
    stats_map_[activity] = {activity, 0.0f, 0.0f, 0, 0, 0, false};
  }
}

/*
 * Take a snapshot of current battery state
 */
powerstats::snapshot powerstats::take_snapshot() {
  snapshot snap;
  snap.voltage_v = fuel_gauge_.cellVoltage();
  snap.soc_percent = fuel_gauge_.cellPercent();
  snap.timestamp_us = get_time_us();

  last_snapshot_ = snap;
  snapshot_history_.push_back(snap);

  // Limit history size to prevent memory issues
  if (snapshot_history_.size() > 1000) {
    snapshot_history_.erase(snapshot_history_.begin());
  }

  return snap;
}

/*
 * Get the last snapshot taken
 */
const powerstats::snapshot &powerstats::get_last_snapshot() const {
  return last_snapshot_;
}

/*
 * Get the full snapshot history
 */
const std::vector<powerstats::snapshot> &powerstats::get_snapshot_history() const {
  return snapshot_history_;
}

/*
 * Start tracking an activity
 */
void powerstats::start_activity(Activity activity) {
  auto &stats = stats_map_[activity];

  if (!stats.is_active) {
    stats.is_active = true;
    stats.last_start_time_us = get_time_us();

    // Take a snapshot at activity start
    take_snapshot();
  }
}

/*
 * Stop tracking an activity and update its statistics
 */
void powerstats::stop_activity(Activity activity) {
  auto &stats = stats_map_[activity];

  if (stats.is_active) {
    // Take a snapshot at activity end
    snapshot end_snap = take_snapshot();

    // Calculate duration
    int64_t duration_us = end_snap.timestamp_us - stats.last_start_time_us;
    uint32_t duration_ms = static_cast<uint32_t>(duration_us / 1000);

    // Find the snapshot closest to when activity started
    snapshot start_snap = last_snapshot_;
    for (auto it = snapshot_history_.rbegin(); it != snapshot_history_.rend(); ++it) {
      if (it->timestamp_us <= stats.last_start_time_us) {
        start_snap = *it;
        break;
      }
    }

    // Update energy statistics
    update_activity_energy(activity, start_snap, end_snap, duration_ms);

    stats.is_active = false;
    stats.total_duration_ms += duration_ms;
    stats.sample_count++;

    // Recalculate average power
    if (stats.total_duration_ms > 0) {
      stats.avg_power_mw =
          (stats.total_energy_mwh * 3600000.0f) / static_cast<float>(stats.total_duration_ms);
    }
  }
}

/*
 * Check if an activity is currently running
 */
bool powerstats::is_activity_running(Activity activity) const {
  auto it = stats_map_.find(activity);
  if (it != stats_map_.end()) {
    return it->second.is_active;
  }
  return false;
}

/*
 * Get statistics for a specific activity
 */
powerstats::activity_stats powerstats::get_activity_stats(Activity activity) const {
  auto it = stats_map_.find(activity);
  if (it != stats_map_.end()) {
    return it->second;
  }

  // Return empty stats if not found
  return {activity, 0.0f, 0.0f, 0, 0, 0, false};
}

/*
 * Get statistics for all activities
 */
std::map<powerstats::Activity, powerstats::activity_stats> powerstats::get_all_stats() const {
  return stats_map_;
}

/*
 * Reset all statistics
 */
void powerstats::reset_stats() {
  for (auto &pair : stats_map_) {
    pair.second.total_energy_mwh = 0.0f;
    pair.second.avg_power_mw = 0.0f;
    pair.second.sample_count = 0;
    pair.second.total_duration_ms = 0;
    pair.second.last_start_time_us = 0;
    pair.second.is_active = false;
  }

  snapshot_history_.clear();
  last_snapshot_ = take_snapshot();
}

/*
 * Reset statistics for a specific activity
 */
void powerstats::reset_activity_stats(Activity activity) {
  auto &stats = stats_map_[activity];
  stats.total_energy_mwh = 0.0f;
  stats.avg_power_mw = 0.0f;
  stats.sample_count = 0;
  stats.total_duration_ms = 0;
  stats.last_start_time_us = 0;
  stats.is_active = false;
}

/*
 * Calculate power consumption between two snapshots
 * Uses voltage drop and state of charge change to estimate power
 */
float powerstats::calculate_power_mw(const snapshot &start, const snapshot &end,
                                     uint32_t duration_ms) const {
  if (duration_ms == 0) {
    return 0.0f;
  }

  // Calculate energy from voltage and SOC change
  // Energy consumed in mWh = capacity * voltage * delta_soc / 100
  float soc_change = start.soc_percent - end.soc_percent;
  float avg_voltage = (start.voltage_v + end.voltage_v) / 2.0f;
  float energy_mwh = BATTERY_CAPACITY_MAH * avg_voltage * (soc_change / 100.0f);

  // Power = Energy / Time
  float duration_hours = duration_ms / 3600000.0f;
  float power_mw = (duration_hours > 0) ? (energy_mwh / duration_hours) : 0.0f;

  return power_mw;
}

/*
 * Get current power consumption based on recent snapshots
 */
float powerstats::get_current_power_mw() const {
  if (snapshot_history_.size() < 2) {
    return 0.0f;
  }

  // Use last two snapshots
  const snapshot &prev = snapshot_history_[snapshot_history_.size() - 2];
  const snapshot &current = last_snapshot_;

  uint32_t duration_ms = static_cast<uint32_t>((current.timestamp_us - prev.timestamp_us) / 1000);

  return calculate_power_mw(prev, current, duration_ms);
}

/*
 * Convert activity enum to string
 */
const char *powerstats::activity_to_string(Activity activity) {
  switch (activity) {
  case Activity::IDLE:
    return "IDLE";
  case Activity::BLUETOOTH:
    return "BLUETOOTH";
  case Activity::CPU_INTENSIVE:
    return "CPU_INTENSIVE";
  case Activity::IMU_READING:
    return "IMU_READING";
  case Activity::DISPLAY:
    return "DISPLAY";
  case Activity::GPS:
    return "GPS";
  case Activity::STORAGE_WRITE:
    return "STORAGE_WRITE";
  case Activity::STORAGE_READ:
    return "STORAGE_READ";
  case Activity::NETWORK:
    return "NETWORK";
  case Activity::SENSOR_FUSION:
    return "SENSOR_FUSION";
  case Activity::CUSTOM:
    return "CUSTOM";
  default:
    return "UNKNOWN";
  }
}

/*
 * Enable or disable automatic snapshot taking
 */
void powerstats::enable_auto_snapshot(bool enable, uint32_t interval_ms) {
  auto_snapshot_enabled_ = enable;
  auto_snapshot_interval_ms_ = interval_ms;

  if (enable) {
    last_auto_snapshot_time_us_ = get_time_us();
  }
}

/*
 * Update energy consumption for an activity
 */
void powerstats::update_activity_energy(Activity activity, const snapshot &start,
                                        const snapshot &end, uint32_t duration_ms) {
  auto &stats = stats_map_[activity];

  // Calculate energy consumed during this activity period
  float soc_change = start.soc_percent - end.soc_percent;
  float avg_voltage = (start.voltage_v + end.voltage_v) / 2.0f;
  float energy_mwh = BATTERY_CAPACITY_MAH * avg_voltage * (soc_change / 100.0f);

  // Accumulate energy
  if (energy_mwh > 0) {
    stats.total_energy_mwh += energy_mwh;
  }
}

/*
 * Get current time in microseconds (using ESP32 timer)
 */
int64_t powerstats::get_time_us() const {
  return esp_timer_get_time();
}
