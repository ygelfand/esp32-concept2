#include "concept2.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "csafe.h"

namespace esphome {
namespace concept2 {

static const char *const TAG = "concept2";

void Concept2Component::setup() {
#ifdef USE_ESP_IDF
  this->usb_.set_frame_callback(
      [this](const uint8_t *data, size_t len) { this->on_frame_(data, len); });
  if (!this->usb_.begin()) {
    this->mark_failed();
    return;
  }
  if (this->enable_ble_)
    this->ble_.begin(this->device_name_);
  if (this->dircon_enabled_)
    this->dircon_.begin(this->dircon_port_, this->device_name_);
#else
  ESP_LOGE(TAG, "concept2 requires the esp-idf framework");
  this->mark_failed();
#endif
}

void Concept2Component::update() {
#ifdef USE_ESP_IDF
  if (!this->usb_.connected())
    return;
  uint8_t frame[96];
  size_t n = csafe::build_poll_frame(frame, sizeof(frame));
  if (n == 0)
    return;
  bool sent = this->usb_.write_frame(frame, n);
  uint32_t now = millis();
  if (now - this->last_tx_log_ms_ >= 1000) {
    this->last_tx_log_ms_ = now;
    ESP_LOGD(TAG, "poll TX %u bytes (sent=%d): %s", (unsigned) n, sent,
             format_hex_pretty(frame, n).c_str());
  }
#endif
}

void Concept2Component::loop() {
#ifdef USE_ESP_IDF
  if (!this->have_new_)
    return;

  RowingMetrics m;
  portENTER_CRITICAL(&this->mux_);
  m = this->shared_metrics_;
  this->have_new_ = false;
  portEXIT_CRITICAL(&this->mux_);

  this->update_derived_(m);

  // Throttled bring-up log so decoded metrics are observable on the console.
  uint32_t now = millis();
  if (now - this->last_log_ms_ >= 1000) {
    this->last_log_ms_ = now;
    ESP_LOGD(TAG,
             "dist=%.1fm pace=%us/500m power=%dW rate=%.0fspm hr=%u cal=%u "
             "t=%us stroke=%u workout=%u drag=%u",
             m.total_distance_m, m.inst_pace_s500, m.inst_power_w, m.stroke_rate_spm,
             m.heart_rate_bpm, m.total_energy_kcal, m.elapsed_time_s,
             static_cast<unsigned>(m.stroke_state), static_cast<unsigned>(m.workout_state),
             m.drag_factor);
  }

  if (this->enable_ble_)
    this->ble_.publish(m);
  if (this->dircon_enabled_)
    this->dircon_.publish(m);
#ifdef USE_SENSOR
  this->publish_sensors_(m);
#endif
#endif  // USE_ESP_IDF
}

#ifdef USE_ESP_IDF
void Concept2Component::on_frame_(const uint8_t *data, size_t len) {
  // Runs on the USB task. Parse into the persistent target (keeps fields not
  // present in this frame), then hand a snapshot to the main loop.
  bool ok = csafe::parse_response(data, len, this->parse_metrics_);
  uint32_t now = millis();
  if (now - this->last_rx_log_ms_ >= 1000) {
    this->last_rx_log_ms_ = now;
    ESP_LOGD(TAG, "RX %u bytes parse=%s: %s", (unsigned) len, ok ? "OK" : "FAIL",
             format_hex_pretty(data, len).c_str());
  }
  if (!ok)
    return;
  portENTER_CRITICAL(&this->mux_);
  this->shared_metrics_ = this->parse_metrics_;
  this->have_new_ = true;
  portEXIT_CRITICAL(&this->mux_);
}

void Concept2Component::update_derived_(RowingMetrics &m) {
  // Reset session-derived state when the PM starts a new piece.
  if (m.workout_state == WorkoutState::WAIT_TO_BEGIN || m.elapsed_time_s < this->derived_.last_elapsed)
    this->derived_.reset();
  this->derived_.last_elapsed = m.elapsed_time_s;

  // Stroke count: increment on each transition into the DRIVING phase.
  if (m.stroke_state == StrokeState::DRIVING && this->derived_.prev_state != StrokeState::DRIVING)
    this->derived_.stroke_count++;
  this->derived_.prev_state = m.stroke_state;
  m.stroke_count = this->derived_.stroke_count;

  // Running session averages (sampled while the athlete is active).
  if (m.stroke_rate_spm > 0) {
    this->derived_.sr_sum += m.stroke_rate_spm;
    this->derived_.sr_n++;
  }
  m.avg_stroke_rate_spm =
      this->derived_.sr_n ? static_cast<float>(this->derived_.sr_sum / this->derived_.sr_n) : 0.0f;

  if (m.inst_power_w > 0) {
    this->derived_.p_sum += m.inst_power_w;
    this->derived_.p_n++;
  }
  m.avg_power_w =
      this->derived_.p_n ? static_cast<int16_t>(this->derived_.p_sum / this->derived_.p_n) : 0;

  // Energy rate from session totals (stable vs. per-sample deltas).
  if (m.elapsed_time_s > 0 && m.total_energy_kcal > 0) {
    float per_hour = static_cast<float>(m.total_energy_kcal) * 3600.0f / m.elapsed_time_s;
    m.energy_per_hour_kcal = static_cast<uint16_t>(per_hour);
    float per_min = per_hour / 60.0f;
    m.energy_per_min_kcal = static_cast<uint8_t>(per_min > 255.0f ? 255.0f : per_min);
  }
}
#endif  // USE_ESP_IDF

#ifdef USE_SENSOR
void Concept2Component::publish_sensors_(const RowingMetrics &m) {
  // Throttle HA sensor updates to ~1 Hz; the 10 Hz stream is only for FTMS.
  uint32_t now = millis();
  if (now - this->last_sensor_pub_ms_ < 1000)
    return;
  this->last_sensor_pub_ms_ = now;

  if (this->distance_sensor_ != nullptr)
    this->distance_sensor_->publish_state(m.total_distance_m);
  if (this->pace_sensor_ != nullptr)
    this->pace_sensor_->publish_state(m.inst_pace_s500);
  if (this->power_sensor_ != nullptr)
    this->power_sensor_->publish_state(m.inst_power_w);
  if (this->stroke_rate_sensor_ != nullptr)
    this->stroke_rate_sensor_->publish_state(m.stroke_rate_spm);
  if (this->stroke_count_sensor_ != nullptr)
    this->stroke_count_sensor_->publish_state(m.stroke_count);
  if (this->heart_rate_sensor_ != nullptr)
    this->heart_rate_sensor_->publish_state(m.heart_rate_bpm);
  if (this->calories_sensor_ != nullptr)
    this->calories_sensor_->publish_state(m.total_energy_kcal);
  if (this->elapsed_time_sensor_ != nullptr)
    this->elapsed_time_sensor_->publish_state(m.elapsed_time_s);
  if (this->drag_factor_sensor_ != nullptr)
    this->drag_factor_sensor_->publish_state(m.drag_factor);
}
#endif  // USE_SENSOR

void Concept2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Concept2 PM bridge:");
  ESP_LOGCONFIG(TAG, "  Device name: %s", this->device_name_.c_str());
  ESP_LOGCONFIG(TAG, "  BLE FTMS: %s", YESNO(this->enable_ble_));
  ESP_LOGCONFIG(TAG, "  DirCon: %s (tcp/%u)", YESNO(this->dircon_enabled_), this->dircon_port_);
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->get_update_interval());
#ifdef USE_ESP_IDF
  if (this->usb_.connected())
    ESP_LOGCONFIG(TAG, "  USB: connected (PID 0x%04X)", this->usb_.product_id());
  else
    ESP_LOGCONFIG(TAG, "  USB: waiting for PM");
#endif
}

}  // namespace concept2
}  // namespace esphome
