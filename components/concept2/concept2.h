#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "rowing_metrics.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_LIGHT
#include "esphome/components/light/light_state.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#ifdef USE_ESP_IDF
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_ftms.h"
#include "dircon.h"
#include "pm_usb_host.h"
#endif

namespace esphome {
namespace concept2 {

#ifdef USE_SWITCH
class Concept2Switch;
#endif

// Top-level orchestrator. As a PollingComponent it drives the CSAFE poll cycle
// on `update_interval`; received frames are parsed off the USB task and the
// normalized metrics are fanned out to the enabled transports from loop().
class Concept2Component : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_enable_ble(bool b) { this->enable_ble_ = b; }
  void set_dircon_enabled(bool b) { this->dircon_enabled_ = b; }
  void set_dircon_port(uint16_t p) { this->dircon_port_ = p; }
  void set_device_name(const std::string &n) { this->device_name_ = n; }

  // Pause/resume CSAFE polling. When paused the PM stops receiving frames and
  // goes to sleep on its own inactivity timeout.
  // Pause = stop polling only (does not drop the port or reset the workout).
  void set_active(bool active) {
    this->active_ = active;
    this->publish_active_();
  }
  void toggle_active() { this->set_active(!this->active_); }
  bool is_active() const { return this->active_; }
  void set_sleep_timeout(uint32_t ms) { this->sleep_timeout_ms_ = ms; }
  void set_autosleep_on_idle(bool b) { this->autosleep_on_idle_ = b; }
#ifdef USE_SWITCH
  void set_active_switch(Concept2Switch *sw) { this->active_switch_ = sw; }
#endif

#ifdef USE_ESP_IDF
  bool pm_connected() const { return this->usb_.connected(); }
#else
  bool pm_connected() const { return false; }
#endif

  void set_pause_button(InternalGPIOPin *pin) { this->pause_button_ = pin; }
  // Optional GPIO driving a high-side VBUS load switch (e.g. TPS22918 EN).
  // When set, sleep physically cuts 5V to the PM so it powers off; wake
  // restores it. Unset = sleep is a soft standby (stop polling only).
  void set_vbus_pin(GPIOPin *pin) { this->vbus_pin_ = pin; }
#ifdef USE_LIGHT
  void set_status_light(light::LightState *l) { this->status_light_ = l; }
#endif

#ifdef USE_ESP_IDF
  // Called by esp32_ble's dispatcher (registered from __init__.py); forwards to
  // the BLE FTMS server.
  void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param) {
    this->ble_.gatts_event_handler(event, gatts_if, param);
  }
#endif

#ifdef USE_SENSOR
  void set_distance_sensor(sensor::Sensor *s) { this->distance_sensor_ = s; }
  void set_pace_sensor(sensor::Sensor *s) { this->pace_sensor_ = s; }
  void set_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_stroke_rate_sensor(sensor::Sensor *s) { this->stroke_rate_sensor_ = s; }
  void set_stroke_count_sensor(sensor::Sensor *s) { this->stroke_count_sensor_ = s; }
  void set_heart_rate_sensor(sensor::Sensor *s) { this->heart_rate_sensor_ = s; }
  void set_calories_sensor(sensor::Sensor *s) { this->calories_sensor_ = s; }
  void set_elapsed_time_sensor(sensor::Sensor *s) { this->elapsed_time_sensor_ = s; }
  void set_drag_factor_sensor(sensor::Sensor *s) { this->drag_factor_sensor_ = s; }
  void set_flywheel_sensor(sensor::Sensor *s) { this->flywheel_sensor_ = s; }
#endif

 protected:
  bool active_{true};
  bool enable_ble_{true};
  bool dircon_enabled_{true};

  InternalGPIOPin *pause_button_{nullptr};
  GPIOPin *vbus_pin_{nullptr};
  bool button_prev_{false};
  bool hold_fired_{false};
  uint32_t press_start_ms_{0};
  uint32_t last_edge_ms_{0};
  uint32_t last_release_ms_{0};
  uint8_t tap_count_{0};
  uint32_t last_activity_ms_{0};
  uint32_t sleep_timeout_ms_{120000};
  bool autosleep_on_idle_{true};
  bool sleeping_{false};
  bool was_connected_{false};
  void publish_active_();
#ifdef USE_SWITCH
  Concept2Switch *active_switch_{nullptr};
#endif
#ifdef USE_ESP_IDF
  void handle_button_(uint32_t nowm);
  void enter_sleep_();
  void wake_();
  void reset_workout_();
  void send_command_(uint8_t cmd);
#endif
#ifdef USE_LIGHT
  void update_status_led_();
  light::LightState *status_light_{nullptr};
  int last_led_status_{-1};
  bool led_rowing_{false};
  uint32_t last_active_ms_{0};
#endif
  uint16_t dircon_port_{36866};
  std::string device_name_{"Concept2 Rower"};

#ifdef USE_SENSOR
  void publish_sensors_(const RowingMetrics &m);
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *pace_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *stroke_rate_sensor_{nullptr};
  sensor::Sensor *stroke_count_sensor_{nullptr};
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *calories_sensor_{nullptr};
  sensor::Sensor *elapsed_time_sensor_{nullptr};
  sensor::Sensor *drag_factor_sensor_{nullptr};
  sensor::Sensor *flywheel_sensor_{nullptr};
  uint32_t last_sensor_pub_ms_{0};
#endif

#ifdef USE_ESP_IDF
  // Called from the USB task when a full CSAFE frame arrives.
  void on_frame_(const uint8_t *data, size_t len);
  // Derive stroke count / averages / energy rates from a fresh parse.
  void update_derived_(RowingMetrics &m);

  PmUsbHost usb_;
  BleFtms ble_;
  DirCon dircon_;

  // Persistent parse target owned by the USB task (accumulates fields across
  // frames), plus the hand-off to the main loop guarded by a spinlock.
  RowingMetrics parse_metrics_{};
  RowingMetrics shared_metrics_{};
  void send_next_poll_();
  size_t poll_index_{0};
  volatile bool awaiting_{false};
  uint32_t last_poll_ms_{0};
  volatile bool have_new_{false};
  uint32_t last_log_ms_{0};
  uint32_t last_tx_log_ms_{0};
  uint32_t last_rx_log_ms_{0};
  uint32_t last_raw_log_ms_{0};
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

  // Locally derived session state.
  struct Derived {
    uint16_t stroke_count{0};
    StrokeState prev_state{StrokeState::UNKNOWN};
    double sr_sum{0};
    uint32_t sr_n{0};
    double p_sum{0};
    uint32_t p_n{0};
    uint16_t last_elapsed{0};
    void reset() { *this = Derived(); }
  } derived_;
#endif
};

#ifdef USE_SWITCH
// Native pause switch: reflects and controls the component's active_ state.
class Concept2Switch : public switch_::Switch {
 public:
  void set_parent(Concept2Component *p) { this->parent_ = p; }
  void write_state(bool state) override { this->parent_->set_active(state); }

 protected:
  Concept2Component *parent_{nullptr};
};
#endif

}  // namespace concept2
}  // namespace esphome
