#pragma once

#ifdef USE_ESP_IDF

#include <string>

#include "esp_gatts_api.h"

#include "esphome/components/esp32_ble/ble.h"

#include "ftms_encode.h"
#include "rowing_metrics.h"

namespace esphome {
namespace concept2 {

// BLE FTMS Rower peripheral.
//
// Integrates with ESPHome's `esp32_ble` component rather than owning the BLE
// stack: esp32_ble initializes the controller + Bluedroid and manages WiFi
// coexistence and advertising, while this class registers its OWN GATTS
// application + attribute table and receives events via esp32_ble's dispatcher.
// This keeps the FTMS service composable with other ESPHome BLE features.
//
// Exposes the Fitness Machine Service (0x1826) with Rower Data (0x2AD1, notify),
// Fitness Machine Feature (0x2ACC, read), Control Point (0x2AD9, write/indicate)
// and Status (0x2ADA, notify) characteristics.
//
// GATTS events are delivered by esp32_ble's callback registry: the Python
// codegen registers Concept2Component::gatts_event_handler(), which forwards
// here. See __init__.py's esp32_ble.register_gatts_event_handler().
class BleFtms {
 public:
  // Attribute-table indices (declaration + value + optional CCCD per char).
  enum AttrIndex {
    IDX_SVC,
    IDX_ROWER_CHAR,
    IDX_ROWER_VAL,
    IDX_ROWER_CCC,
    IDX_FEATURE_CHAR,
    IDX_FEATURE_VAL,
    IDX_CP_CHAR,
    IDX_CP_VAL,
    IDX_CP_CCC,
    IDX_STATUS_CHAR,
    IDX_STATUS_VAL,
    IDX_STATUS_CCC,
    IDX_NB,
  };

  // Register our GATTS app with esp32_ble and advertise the FTMS service.
  // Returns false if esp32_ble is not available.
  bool begin(const std::string &device_name);

  // Push the latest metrics as a Rower Data notification (no-op if no client is
  // connected/subscribed).
  void publish(const RowingMetrics &m);

  bool connected() const { return this->conn_id_ != 0xFFFF; }

  // Invoked (via Concept2Component) for every GATTS event esp32_ble dispatches;
  // we claim only events for our own registered application.
  void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                           esp_ble_gatts_cb_param_t *param);

 private:
  void handle_control_point_(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

  std::string device_name_{"Concept2 Rower"};
  esp_gatt_if_t gatts_if_{ESP_GATT_IF_NONE};
  uint16_t app_id_{0x00C2};  // our GATTS app id (distinct from other components)
  uint16_t conn_id_{0xFFFF};
  uint16_t handle_table_[IDX_NB]{};
  bool rower_subscribed_{false};
};

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
