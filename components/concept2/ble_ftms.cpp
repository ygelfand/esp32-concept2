#include "ble_ftms.h"

#ifdef USE_ESP_IDF

#include <cstring>

#include "esphome/core/log.h"

#include "esphome/components/esp32_ble/ble.h"
#include "esphome/components/esp32_ble/ble_uuid.h"

namespace esphome {
namespace concept2 {

static const char *const TAG = "concept2.ble";

// ---- GATT database static descriptors -------------------------------------
namespace {
const uint16_t PRIMARY_SERVICE_UUID = ESP_GATT_UUID_PRI_SERVICE;
const uint16_t CHAR_DECL_UUID = ESP_GATT_UUID_CHAR_DECLARE;
const uint16_t CHAR_CCC_UUID = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

const uint16_t FTMS_SVC_UUID = ftms::UUID_SERVICE;
const uint16_t ROWER_UUID = ftms::UUID_ROWER_DATA;
const uint16_t FEATURE_UUID = ftms::UUID_FEATURE;
const uint16_t CP_UUID = ftms::UUID_CONTROL_POINT;
const uint16_t STATUS_UUID = ftms::UUID_STATUS;

const uint8_t PROP_NOTIFY = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
const uint8_t PROP_READ = ESP_GATT_CHAR_PROP_BIT_READ;
const uint8_t PROP_WRITE_INDICATE = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_INDICATE;

// Mutable value buffers referenced by the attribute table.
uint8_t s_feature_val[ftms::FEATURE_LEN] = {0};
uint8_t s_rower_val[ftms::ROWER_DATA_MAX] = {0};
uint8_t s_cp_val[20] = {0};
uint8_t s_status_val[4] = {0};
uint8_t s_ccc_rower[2] = {0, 0};
uint8_t s_ccc_cp[2] = {0, 0};
uint8_t s_ccc_status[2] = {0, 0};

// The attribute table. Order MUST match BleFtms::AttrIndex.
const esp_gatts_attr_db_t GATT_DB[BleFtms::IDX_NB] = {
    // Service declaration.
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &PRIMARY_SERVICE_UUID, ESP_GATT_PERM_READ, sizeof(uint16_t),
      sizeof(FTMS_SVC_UUID), (uint8_t *) &FTMS_SVC_UUID}},

    // Rower Data: declaration / value / CCCD.
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_DECL_UUID, ESP_GATT_PERM_READ, sizeof(uint8_t),
      sizeof(uint8_t), (uint8_t *) &PROP_NOTIFY}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &ROWER_UUID, ESP_GATT_PERM_READ, sizeof(s_rower_val), 0,
      s_rower_val}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_CCC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(s_ccc_rower), sizeof(s_ccc_rower), s_ccc_rower}},

    // Fitness Machine Feature: declaration / value (read).
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_DECL_UUID, ESP_GATT_PERM_READ, sizeof(uint8_t),
      sizeof(uint8_t), (uint8_t *) &PROP_READ}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &FEATURE_UUID, ESP_GATT_PERM_READ, sizeof(s_feature_val),
      sizeof(s_feature_val), s_feature_val}},

    // Control Point: declaration / value (write, app-handled) / CCCD.
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_DECL_UUID, ESP_GATT_PERM_READ, sizeof(uint8_t),
      sizeof(uint8_t), (uint8_t *) &PROP_WRITE_INDICATE}},
    {{ESP_GATT_RSP_BY_APP},
     {ESP_UUID_LEN_16, (uint8_t *) &CP_UUID, ESP_GATT_PERM_WRITE, sizeof(s_cp_val), 0, s_cp_val}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_CCC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(s_ccc_cp), sizeof(s_ccc_cp), s_ccc_cp}},

    // Fitness Machine Status: declaration / value / CCCD.
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_DECL_UUID, ESP_GATT_PERM_READ, sizeof(uint8_t),
      sizeof(uint8_t), (uint8_t *) &PROP_NOTIFY}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &STATUS_UUID, ESP_GATT_PERM_READ, sizeof(s_status_val), 0,
      s_status_val}},
    {{ESP_GATT_AUTO_RSP},
     {ESP_UUID_LEN_16, (uint8_t *) &CHAR_CCC_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(s_ccc_status), sizeof(s_ccc_status), s_ccc_status}},
};
}  // namespace

// ---------------------------------------------------------------------------

bool BleFtms::begin(const std::string &device_name) {
  this->device_name_ = device_name;
  ftms::encode_feature(s_feature_val);

  if (esp32_ble::global_ble == nullptr) {
    ESP_LOGE(TAG, "esp32_ble is not available - is the esp32_ble component enabled?");
    return false;
  }

  // Our GATTS event handler is wired up by Python codegen (see __init__.py).
  // Here we just register our own GATTS application on the shared stack.
  esp_err_t err = esp_ble_gatts_app_register(this->app_id_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "gatts_app_register failed: %s", esp_err_to_name(err));
    return false;
  }

  // Advertise the FTMS service UUID via esp32_ble's advertising manager.
  esp32_ble::global_ble->advertising_add_service_uuid(
      esp32_ble::ESPBTUUID::from_uint16(ftms::UUID_SERVICE));
  esp32_ble::global_ble->advertising_start();
  ESP_LOGI(TAG, "BLE FTMS registered (advertising service 0x%04X)", ftms::UUID_SERVICE);
  return true;
}

void BleFtms::gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t *param) {
  // esp32_ble fans every GATTS event out to all handlers; claim only our app.
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.app_id != this->app_id_)
      return;
    this->gatts_if_ = gatts_if;
    esp_ble_gatts_create_attr_tab(GATT_DB, gatts_if, IDX_NB, 0);
    return;
  }
  if (gatts_if != this->gatts_if_)
    return;

  switch (event) {
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
      if (param->add_attr_tab.status != ESP_GATT_OK ||
          param->add_attr_tab.num_handle != IDX_NB) {
        ESP_LOGE(TAG, "create attr table failed (status %d, handles %d)",
                 param->add_attr_tab.status, param->add_attr_tab.num_handle);
        break;
      }
      memcpy(this->handle_table_, param->add_attr_tab.handles, sizeof(this->handle_table_));
      esp_ble_gatts_start_service(this->handle_table_[IDX_SVC]);
      ESP_LOGI(TAG, "FTMS service started");
      break;

    case ESP_GATTS_CONNECT_EVT:
      this->conn_id_ = param->connect.conn_id;
      ESP_LOGI(TAG, "client connected (conn %u)", this->conn_id_);
      break;

    case ESP_GATTS_DISCONNECT_EVT:
      // esp32_ble handles re-advertising on disconnect.
      ESP_LOGI(TAG, "client disconnected (reason 0x%x)", param->disconnect.reason);
      this->conn_id_ = 0xFFFF;
      this->rower_subscribed_ = false;
      break;

    case ESP_GATTS_WRITE_EVT:
      if (!param->write.is_prep) {
        if (param->write.handle == this->handle_table_[IDX_ROWER_CCC] && param->write.len >= 2) {
          this->rower_subscribed_ = (param->write.value[0] & 0x01) != 0;
          ESP_LOGD(TAG, "rower notifications %s", this->rower_subscribed_ ? "on" : "off");
        } else if (param->write.handle == this->handle_table_[IDX_CP_VAL]) {
          this->handle_control_point_(gatts_if, param);
        }
      }
      break;

    default:
      break;
  }
}

void BleFtms::handle_control_point_(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  uint8_t op = param->write.len >= 1 ? param->write.value[0] : 0xFF;

  // Acknowledge the write itself (this characteristic is app-handled).
  if (param->write.need_rsp)
    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                ESP_GATT_OK, nullptr);

  // Result: 0x01 Success for the requests we honor, 0x02 Not Supported otherwise.
  uint8_t result;
  switch (op) {
    case 0x00:  // Request Control
    case 0x01:  // Reset
    case 0x07:  // Start / Resume
    case 0x08:  // Stop / Pause
      result = 0x01;
      break;
    default:
      result = 0x02;
      break;
  }
  uint8_t resp[3] = {0x80, op, result};  // 0x80 = Response Code
  esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, this->handle_table_[IDX_CP_VAL],
                              sizeof(resp), resp, true);
}

void BleFtms::publish(const RowingMetrics &m) {
  if (!this->connected() || !this->rower_subscribed_)
    return;
  uint8_t buf[ftms::ROWER_DATA_MAX];
  size_t len = ftms::encode_rower_data(m, buf);
  esp_ble_gatts_send_indicate(this->gatts_if_, this->conn_id_, this->handle_table_[IDX_ROWER_VAL],
                              len, buf, false);
}

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
