#include "ftms_encode.h"

namespace esphome {
namespace concept2 {
namespace ftms {

namespace {
inline void put_u8(uint8_t *&p, uint8_t v) { *p++ = v; }
inline void put_u16(uint8_t *&p, uint16_t v) {
  *p++ = static_cast<uint8_t>(v & 0xFF);
  *p++ = static_cast<uint8_t>(v >> 8);
}
inline void put_u24(uint8_t *&p, uint32_t v) {
  *p++ = static_cast<uint8_t>(v & 0xFF);
  *p++ = static_cast<uint8_t>((v >> 8) & 0xFF);
  *p++ = static_cast<uint8_t>((v >> 16) & 0xFF);
}

// Rower Data flags. NOTE bit 0 ("More Data") is INVERTED: Stroke Rate + Stroke
// Count are present when bit 0 is CLEAR. All other bits mean "present when set".
constexpr uint16_t F_MORE_DATA = 1 << 0;         // clear -> stroke rate+count present
constexpr uint16_t F_AVG_STROKE_RATE = 1 << 1;
constexpr uint16_t F_TOTAL_DISTANCE = 1 << 2;
constexpr uint16_t F_INST_PACE = 1 << 3;
constexpr uint16_t F_INST_POWER = 1 << 5;
constexpr uint16_t F_EXPENDED_ENERGY = 1 << 8;   // gates all three energy fields
constexpr uint16_t F_HEART_RATE = 1 << 9;
constexpr uint16_t F_ELAPSED_TIME = 1 << 11;
}  // namespace

size_t encode_rower_data(const RowingMetrics &m, uint8_t *out) {
  uint8_t *p = out;

  const uint16_t flags = F_AVG_STROKE_RATE | F_TOTAL_DISTANCE | F_INST_PACE | F_INST_POWER |
                         F_EXPENDED_ENERGY | F_HEART_RATE | F_ELAPSED_TIME;
  // bit 0 stays clear so stroke rate + stroke count are included.
  put_u16(p, flags);

  // Stroke Rate (uint8, resolution 0.5 /min) + Stroke Count (uint16).
  put_u8(p, static_cast<uint8_t>(m.stroke_rate_spm * 2.0f));
  put_u16(p, m.stroke_count);

  // Average Stroke Rate (uint8, resolution 0.5 /min).
  put_u8(p, static_cast<uint8_t>(m.avg_stroke_rate_spm * 2.0f));

  // Total Distance (uint24, meters).
  put_u24(p, static_cast<uint32_t>(m.total_distance_m));

  // Instantaneous Pace (uint16, seconds per 500 m).
  put_u16(p, m.inst_pace_s500);

  // Instantaneous Power (sint16, watts).
  put_u16(p, static_cast<uint16_t>(m.inst_power_w));

  // Expended energy group: Total (uint16 kcal), per-hour (uint16 kcal),
  // per-minute (uint8 kcal). 0xFFFF / 0xFF mean "not available".
  put_u16(p, m.total_energy_kcal);
  put_u16(p, m.energy_per_hour_kcal);
  put_u8(p, m.energy_per_min_kcal);

  // Heart Rate (uint8, bpm).
  put_u8(p, m.heart_rate_bpm);

  // Elapsed Time (uint16, seconds).
  put_u16(p, m.elapsed_time_s);

  return static_cast<size_t>(p - out);
}

void encode_feature(uint8_t *out) {
  // Fitness Machine Features (uint32 LE): Cadence(b1, stroke rate), Total
  // Distance(b2), Pace(b5), Expended Energy(b9), Heart Rate(b10), Elapsed
  // Time(b12), Power Measurement(b14).
  const uint32_t machine = (1u << 1) | (1u << 2) | (1u << 5) | (1u << 9) | (1u << 10) |
                           (1u << 12) | (1u << 14);
  // Target Setting Features (uint32 LE): none (device is read-only).
  const uint32_t target = 0;
  uint8_t *p = out;
  put_u16(p, static_cast<uint16_t>(machine & 0xFFFF));
  put_u16(p, static_cast<uint16_t>(machine >> 16));
  put_u16(p, static_cast<uint16_t>(target & 0xFFFF));
  put_u16(p, static_cast<uint16_t>(target >> 16));
}

}  // namespace ftms
}  // namespace concept2
}  // namespace esphome
