#pragma once

#include <cstdint>

namespace esphome {
namespace concept2 {

// Stroke state as reported by the PM (CSAFE_PM_GET_STROKESTATE / 0xBF).
enum class StrokeState : uint8_t {
  WAITING_MIN_SPEED = 0,
  WAITING_ACCELERATE = 1,
  DRIVING = 2,
  DWELLING = 3,
  RECOVERY = 4,
  UNKNOWN = 0xFF,
};

// Workout state as reported by the PM (CSAFE_PM_GET_WORKOUTSTATE / 0x8D).
// Values 0..13 per the CSAFE Communication Definition. We only special-case a
// few of them; the raw value is kept for downstream consumers.
enum class WorkoutState : uint8_t {
  WAIT_TO_BEGIN = 0,
  WORKOUT_ROW = 1,
  COUNTDOWN_PAUSE = 2,
  INTERVAL_REST = 3,
  INTERVAL_WORK_TIME = 4,
  INTERVAL_WORK_DISTANCE = 5,
  WORKOUT_END = 10,
  TERMINATE = 11,
  WORKOUT_LOGGED = 12,
  REARM = 13,
  UNKNOWN = 0xFF,
};

// Normalized, transport-agnostic snapshot of the rowing session. Every producer
// (CSAFE decode) writes this; every consumer (FTMS/BLE, DirCon, optional
// sensors) reads it. All fields are in SI-ish base units so the FTMS encoder can
// apply the spec's fixed scaling in one place.
struct RowingMetrics {
  // Cadence / strokes.
  float stroke_rate_spm{0.0f};     // strokes per minute
  uint16_t stroke_count{0};        // total strokes this session
  float avg_stroke_rate_spm{0.0f}; // strokes per minute, session average

  // Distance & pace.
  float total_distance_m{0.0f};    // meters
  uint16_t inst_pace_s500{0};      // seconds per 500 m (0 = no pace / not moving)
  uint16_t avg_pace_s500{0};       // seconds per 500 m, session average

  // Power.
  int16_t inst_power_w{0};         // watts
  int16_t avg_power_w{0};          // watts, session average

  // Energy.
  uint16_t total_energy_kcal{0};   // kcal
  uint16_t energy_per_hour_kcal{0};// kcal/hour
  uint8_t energy_per_min_kcal{0};  // kcal/min

  // Physiology.
  uint8_t heart_rate_bpm{0};       // bpm (0 = no HR belt)

  // Time.
  uint16_t elapsed_time_s{0};      // seconds

  // PM state machine.
  StrokeState stroke_state{StrokeState::UNKNOWN};
  WorkoutState workout_state{WorkoutState::UNKNOWN};
  uint8_t drag_factor{0};
  uint16_t flywheel_rpm{0};

  // True once at least one full poll cycle has populated real data.
  bool valid{false};
};

}  // namespace concept2
}  // namespace esphome
