#include "concept2.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "csafe.h"

namespace esphome {
namespace concept2 {

static const char *const TAG = "concept2";

void Concept2Component::setup() {
  if (this->pause_button_ != nullptr)
    this->pause_button_->setup();
  this->last_activity_ms_ = millis();
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

void Concept2Component::update() {}

#ifdef USE_ESP_IDF
void Concept2Component::send_next_poll_() {
  uint8_t frame[96];
  size_t idx = this->poll_index_;
  size_t n = csafe::build_poll_frame(idx, frame, sizeof(frame));
  this->poll_index_ = (this->poll_index_ + 1) % csafe::poll_block_count();
  if (n == 0)
    return;
  bool sent = this->usb_.write_frame(frame, n);
  this->last_poll_ms_ = millis();
  this->awaiting_ = true;
  if (this->last_poll_ms_ - this->last_tx_log_ms_ >= 1000) {
    this->last_tx_log_ms_ = this->last_poll_ms_;
    ESP_LOGV(TAG, "poll[%u] TX %u bytes (sent=%d): %s", (unsigned) idx, (unsigned) n, sent,
             format_hex_pretty(frame, n).c_str());
  }
}
#endif

void Concept2Component::loop() {
#ifdef USE_ESP_IDF
  uint32_t nowm = millis();

  // Multi-tap button: count debounced presses within a window, then act
  // (1 = pause, 2 = reset workout, 3 = sleep, any tap while asleep = wake).
  if (this->pause_button_ != nullptr) {
    bool pressed = !this->pause_button_->digital_read();  // boot button is active-low
    if (pressed && !this->button_prev_ && (nowm - this->last_button_ms_ > 60)) {
      this->last_button_ms_ = nowm;
      this->last_tap_ms_ = nowm;
      this->last_activity_ms_ = nowm;
      this->tap_count_++;
    }
    this->button_prev_ = pressed;
    if (this->tap_count_ > 0 && (nowm - this->last_tap_ms_ > 400)) {
      this->handle_taps_(this->tap_count_);
      this->tap_count_ = 0;
    }
  }

  // Track (re)connection so the sleep timer starts fresh when a PM attaches.
  bool conn = this->usb_.connected();
  if (conn && !this->was_connected_)
    this->last_activity_ms_ = nowm;
  this->was_connected_ = conn;

  // Auto-sleep: after the timeout with no rowing/button activity, drop the port.
  if (!this->sleeping_ && conn && this->sleep_timeout_ms_ > 0 &&
      (nowm - this->last_activity_ms_ > this->sleep_timeout_ms_) &&
      (!this->active_ || this->autosleep_on_idle_))
    this->enter_sleep_();

#ifdef USE_LIGHT
  this->update_status_led_();
#endif

  // Synchronous poll cycle: send the next block only once the previous reply has
  // been parsed (awaiting_ cleared), honoring a min inter-frame gap; resend on
  // timeout so a silent block can't stall the rotation.
  if (conn && this->active_ && !this->sleeping_) {
    if ((nowm - this->last_poll_ms_ >= 50) &&
        (!this->awaiting_ || (nowm - this->last_poll_ms_ > 300)))
      this->send_next_poll_();
  }

  if (!this->have_new_)
    return;

  RowingMetrics m;
  portENTER_CRITICAL(&this->mux_);
  m = this->shared_metrics_;
  this->have_new_ = false;
  portEXIT_CRITICAL(&this->mux_);

  this->update_derived_(m);
  // "Rowing" = the PM is in an active stroke phase (power lingers as the wheel
  // coasts, so it's a poor signal). Feeds both the LED hold and the sleep timer.
  bool active_stroke = m.stroke_state == StrokeState::DRIVING ||
                       m.stroke_state == StrokeState::DWELLING ||
                       m.stroke_state == StrokeState::RECOVERY;
  if (active_stroke)
    this->last_activity_ms_ = millis();
#ifdef USE_LIGHT
  // Hold green ~3 s past the last stroke so it doesn't flicker between strokes.
  if (active_stroke)
    this->last_active_ms_ = millis();
  this->led_rowing_ = (millis() - this->last_active_ms_) < 3000;
#endif

  // Throttled bring-up log so decoded metrics are observable on the console.
  uint32_t now = millis();
  if (now - this->last_log_ms_ >= 1000) {
    this->last_log_ms_ = now;
    ESP_LOGV(TAG,
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
  size_t f1 = 0;
  while (f1 < len && data[f1] != csafe::FRAME_START_STD && data[f1] != csafe::FRAME_START_EXT)
    f1++;
  if (f1 >= len)
    return;
  size_t f2 = f1 + 1;
  while (f2 < len && data[f2] != csafe::FRAME_STOP)
    f2++;
  if (f2 >= len)
    return;  // no complete frame in this report

  size_t flen = f2 - f1 + 1;
  bool ok = csafe::parse_response(data + f1, flen, this->parse_metrics_);
  uint32_t now = millis();
  if (now - this->last_rx_log_ms_ >= 1000) {
    this->last_rx_log_ms_ = now;
    ESP_LOGV(TAG, "frame %u bytes parse=%s: %s", (unsigned) flen, ok ? "OK" : "FAIL",
             format_hex_pretty(data + f1, flen).c_str());
  }
  if (ok) {
    portENTER_CRITICAL(&this->mux_);
    this->shared_metrics_ = this->parse_metrics_;
    this->have_new_ = true;
    portEXIT_CRITICAL(&this->mux_);
    this->awaiting_ = false;  // reply received; loop() may send the next block
  }
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

void Concept2Component::send_command_(uint8_t cmd) {
  uint8_t contents[1] = {cmd};
  uint8_t frame[8];
  size_t n = csafe::build_frame(contents, 1, frame, sizeof(frame));
  if (n > 0)
    this->usb_.write_frame(frame, n);
}

void Concept2Component::send_screenstate_(uint8_t type, uint8_t value) {
  // F1 76 04 13 02 <type> <value> <cksum> F2  (0x76 = proprietary SET wrapper).
  uint8_t contents[] = {csafe::CMD_SETPMCFG, 0x04, csafe::PM_SET_SCREENSTATE, 0x02, type, value};
  uint8_t frame[16];
  size_t n = csafe::build_frame(contents, sizeof(contents), frame, sizeof(frame));
  if (n > 0) {
    ESP_LOGI(TAG, "screenstate TX (type=%u val=%u): %s", type, value,
             format_hex_pretty(frame, n).c_str());
    this->usb_.write_frame(frame, n);
  }
}

void Concept2Component::reset_workout_() {
  ESP_LOGI(TAG, "reset workout");
  this->send_command_(csafe::CMD_RESET);
}

void Concept2Component::enter_sleep_() {
  ESP_LOGI(TAG, "sleep: SET_SCREENSTATE prepare-to-sleep");
  this->send_screenstate_(csafe::SCREENTYPE_RACE, csafe::SCREENVALUE_PREPARETOSLEEP);
  this->sleeping_ = true;
}

void Concept2Component::wake_() {
  ESP_LOGI(TAG, "wake: go to main screen + resume");
  this->sleeping_ = false;
  this->last_activity_ms_ = millis();
  this->send_screenstate_(csafe::SCREENTYPE_WORKOUT, csafe::SCREENVALUE_GOTOMAINSCREEN);
}

void Concept2Component::handle_taps_(uint8_t count) {
  if (this->sleeping_) {
    this->wake_();
    return;
  }
  switch (count) {
    case 1:
      this->toggle_active();
      ESP_LOGI(TAG, "polling %s", this->active_ ? "resumed" : "paused");
      break;
    case 2:
      this->reset_workout_();
      break;
    default:  // 3 or more taps
      this->enter_sleep_();
      break;
  }
}
#endif  // USE_ESP_IDF

void Concept2Component::publish_active_() {
#ifdef USE_SWITCH
  if (this->active_switch_ != nullptr)
    this->active_switch_->publish_state(this->active_);
#endif
}

#ifdef USE_SENSOR
void Concept2Component::publish_sensors_(const RowingMetrics &m) {
  // Throttle to ~1 Hz, and only publish a sensor when its value actually
  // changed (no HA/log spam while idle). The 10 Hz stream is only for FTMS.
  uint32_t now = millis();
  if (now - this->last_sensor_pub_ms_ < 1000)
    return;
  this->last_sensor_pub_ms_ = now;

  auto pub = [](sensor::Sensor *s, float v) {
    if (s != nullptr && (!s->has_state() || s->state != v))
      s->publish_state(v);
  };
  pub(this->distance_sensor_, m.total_distance_m);
  pub(this->pace_sensor_, m.inst_pace_s500);
  pub(this->power_sensor_, m.inst_power_w);
  pub(this->stroke_rate_sensor_, m.stroke_rate_spm);
  pub(this->stroke_count_sensor_, m.stroke_count);
  pub(this->heart_rate_sensor_, m.heart_rate_bpm);
  pub(this->calories_sensor_, m.total_energy_kcal);
  pub(this->elapsed_time_sensor_, m.elapsed_time_s);
  pub(this->drag_factor_sensor_, m.drag_factor);
  pub(this->flywheel_sensor_, m.flywheel_rpm);
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

#ifdef USE_LIGHT
void Concept2Component::update_status_led_() {
  if (this->status_light_ == nullptr)
    return;
  if (this->sleeping_) {
    if (this->last_led_status_ == 4)
      return;
    this->last_led_status_ = 4;
    this->status_light_->turn_off().perform();
    return;
  }
  int status;
  float r, g, b;
  if (!this->active_) {
    status = 1;  // blue: paused (we intentionally drop the port)
    r = 0.0f, g = 0.0f, b = 1.0f;
  } else if (!this->pm_connected()) {
    status = 0;  // red: no PM attached
    r = 1.0f, g = 0.0f, b = 0.0f;
  } else if (this->led_rowing_) {
    status = 2;  // green: actively rowing
    r = 0.0f, g = 1.0f, b = 0.0f;
  } else {
    status = 3;  // amber: connected, idle
    r = 1.0f, g = 0.6f, b = 0.0f;
  }
  if (status == this->last_led_status_)
    return;
  this->last_led_status_ = status;
  auto call = this->status_light_->turn_on();
  call.set_rgb(r, g, b);
  call.set_brightness(0.4f);
  call.set_transition_length(0);
  call.perform();
}
#endif

}  // namespace concept2
}  // namespace esphome
