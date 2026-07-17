#include "pm_usb_host.h"

#ifdef USE_ESP_IDF

#include <cstring>

#include "esphome/core/log.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace esphome {
namespace concept2 {

static const char *const TAG = "concept2.usb";

// Scratch buffer sizes (rounded to endpoint max-packet-size at submit time).
static constexpr size_t IN_BUF_SIZE = 512;
static constexpr size_t OUT_BUF_SIZE = 128;

bool PmUsbHost::begin(int task_core) {
  const usb_host_config_t host_cfg = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  esp_err_t err = usb_host_install(&host_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_install failed: %s", esp_err_to_name(err));
    return false;
  }

  // Explicitly assert root-port power. On boards that gate the native port's
  // VBUS via the host controller this turns VBUS on; on boards with VBUS
  // hard-wired to the 5V rail it is a harmless no-op. If VBUS never reaches the
  // PM the device won't wake / won't enumerate (CHECK_SHORT_DEV_DESC FAILED).
  err = usb_host_lib_set_root_port_power(true);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "set_root_port_power(true) not supported here: %s", esp_err_to_name(err));

  esp_log_level_set("USBH", ESP_LOG_VERBOSE);
  esp_log_level_set("HUB", ESP_LOG_VERBOSE);
  esp_log_level_set("ENUM", ESP_LOG_VERBOSE);
  esp_log_level_set("HCD", ESP_LOG_VERBOSE);
  esp_log_level_set("USB-OTG", ESP_LOG_VERBOSE);
  esp_log_level_set("CDC_ACM", ESP_LOG_VERBOSE);

  const usb_host_client_config_t client_cfg = {
      .is_synchronous = false,
      .max_num_event_msg = 5,
      .async =
          {
              .client_event_callback = &PmUsbHost::client_event_cb,
              .callback_arg = this,
          },
  };
  err = usb_host_client_register(&client_cfg, &this->client_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "usb_host_client_register failed: %s", esp_err_to_name(err));
    return false;
  }

  BaseType_t ok = xTaskCreatePinnedToCore(&PmUsbHost::task_trampoline, "c2_usb", 4096, this,
                                          5, nullptr, task_core);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "failed to create USB task");
    return false;
  }
  ESP_LOGI(TAG, "USB host started (waiting for a Concept2 PM on VID 0x%04X)", C2_VENDOR_ID);
  return true;
}

void PmUsbHost::task_trampoline(void *arg) { static_cast<PmUsbHost *>(arg)->task_loop(); }

void PmUsbHost::task_loop() {
  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(pdMS_TO_TICKS(20), &event_flags);
    usb_host_client_handle_events(this->client_, pdMS_TO_TICKS(20));

    if (this->dev_gone_) {
      this->close_device_();
      this->dev_gone_ = false;
    }
  }
}

void PmUsbHost::client_event_cb(const usb_host_client_event_msg_t *msg, void *arg) {
  static_cast<PmUsbHost *>(arg)->on_client_event(msg);
}

void PmUsbHost::on_client_event(const usb_host_client_event_msg_t *msg) {
  switch (msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
      ESP_LOGD(TAG, "client event NEW_DEV: device enumerated at address %u",
               msg->new_dev.address);
      if (this->dev_ == nullptr)
        this->open_device_(msg->new_dev.address);
      break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
      ESP_LOGD(TAG, "client event DEV_GONE");
      if (msg->dev_gone.dev_hdl == this->dev_)
        this->dev_gone_ = true;
      break;
    default:
      ESP_LOGD(TAG, "client event %d", (int) msg->event);
      break;
  }
}

bool PmUsbHost::open_device_(uint8_t addr) {
  usb_device_handle_t dev;
  esp_err_t err = usb_host_device_open(this->client_, addr, &dev);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "device_open(%u) failed: %s", addr, esp_err_to_name(err));
    return false;
  }

  const usb_device_desc_t *dev_desc = nullptr;
  usb_host_get_device_descriptor(dev, &dev_desc);
  if (dev_desc == nullptr || dev_desc->idVendor != C2_VENDOR_ID) {
    ESP_LOGD(TAG, "ignoring non-Concept2 device (VID 0x%04X)",
             dev_desc ? dev_desc->idVendor : 0);
    usb_host_device_close(this->client_, dev);
    return false;
  }

  this->dev_ = dev;
  this->dev_addr_ = addr;
  this->pid_ = dev_desc->idProduct;
  ESP_LOGI(TAG, "Concept2 PM attached: VID 0x%04X PID 0x%04X", dev_desc->idVendor, this->pid_);

  if (!this->claim_interface_()) {
    this->close_device_();
    return false;
  }
  return true;
}

bool PmUsbHost::claim_interface_() {
  const usb_config_desc_t *cfg = nullptr;
  esp_err_t err = usb_host_get_active_config_descriptor(this->dev_, &cfg);
  if (err != ESP_OK || cfg == nullptr) {
    ESP_LOGE(TAG, "get_active_config_descriptor failed");
    return false;
  }

  // Walk the configuration descriptor: find the HID interface (class 0x03) and
  // its interrupt IN/OUT endpoints.
  bool in_hid_itf = false;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(cfg);
  const uint8_t *end = p + cfg->wTotalLength;
  bool found_itf = false, found_in = false, found_out = false;
  for (const uint8_t *d = p; d + 2 <= end; d += d[0]) {
    if (d[0] == 0)
      break;  // malformed; avoid infinite loop
    uint8_t b_len = d[0];
    uint8_t b_type = d[1];
    if (b_type == USB_B_DESCRIPTOR_TYPE_INTERFACE && b_len >= sizeof(usb_intf_desc_t)) {
      const auto *itf = reinterpret_cast<const usb_intf_desc_t *>(d);
      in_hid_itf = (itf->bInterfaceClass == USB_CLASS_HID);
      if (in_hid_itf && !found_itf) {
        this->itf_num_ = itf->bInterfaceNumber;
        found_itf = true;
      }
    } else if (b_type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && b_len >= sizeof(usb_ep_desc_t) &&
               in_hid_itf) {
      const auto *ep = reinterpret_cast<const usb_ep_desc_t *>(d);
      bool is_intr = (ep->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) ==
                     USB_BM_ATTRIBUTES_XFER_INT;
      if (!is_intr)
        continue;
      if (ep->bEndpointAddress & 0x80) {
        this->ep_in_addr_ = ep->bEndpointAddress;
        this->ep_in_mps_ = ep->wMaxPacketSize;
        found_in = true;
      } else {
        this->ep_out_addr_ = ep->bEndpointAddress;
        this->ep_out_mps_ = ep->wMaxPacketSize;
        found_out = true;
      }
    }
  }

  if (!found_itf || !found_in || !found_out) {
    ESP_LOGE(TAG, "HID interface/endpoints not found (itf=%d in=%d out=%d)", found_itf,
             found_in, found_out);
    return false;
  }

  err = usb_host_interface_claim(this->client_, this->dev_, this->itf_num_, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "interface_claim(%u) failed: %s", this->itf_num_, esp_err_to_name(err));
    return false;
  }
  ESP_LOGI(TAG, "claimed HID itf %u  ep_in=0x%02X(mps %u)  ep_out=0x%02X(mps %u)",
           this->itf_num_, this->ep_in_addr_, this->ep_in_mps_, this->ep_out_addr_,
           this->ep_out_mps_);

  // Allocate IN/OUT transfers (buffer rounded up to a multiple of MPS).
  size_t in_size = ((IN_BUF_SIZE + this->ep_in_mps_ - 1) / this->ep_in_mps_) * this->ep_in_mps_;
  if (usb_host_transfer_alloc(in_size, 0, &this->in_xfer_) != ESP_OK ||
      usb_host_transfer_alloc(OUT_BUF_SIZE, 0, &this->out_xfer_) != ESP_OK) {
    ESP_LOGE(TAG, "transfer_alloc failed");
    return false;
  }

  this->dev_ready_ = true;
  this->submit_in_();
  return true;
}

void PmUsbHost::submit_in_() {
  if (this->in_xfer_ == nullptr || !this->dev_ready_)
    return;
  size_t rounded = (this->in_xfer_->data_buffer_size / this->ep_in_mps_) * this->ep_in_mps_;
  this->in_xfer_->device_handle = this->dev_;
  this->in_xfer_->bEndpointAddress = this->ep_in_addr_;
  this->in_xfer_->callback = &PmUsbHost::in_transfer_cb;
  this->in_xfer_->context = this;
  this->in_xfer_->num_bytes = rounded;
  esp_err_t err = usb_host_transfer_submit(this->in_xfer_);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "IN submit failed: %s", esp_err_to_name(err));
}

void PmUsbHost::in_transfer_cb(usb_transfer_t *xfer) {
  auto *self = static_cast<PmUsbHost *>(xfer->context);
  if (xfer->status == USB_TRANSFER_STATUS_COMPLETED && xfer->actual_num_bytes >= 1) {
    // First byte is the HID report ID; the CSAFE frame follows.
    if (self->frame_cb_)
      self->frame_cb_(xfer->data_buffer + 1, xfer->actual_num_bytes - 1);
  }
  if (self->dev_ready_)
    self->submit_in_();  // keep the IN pipe continuously armed
}

bool PmUsbHost::write_frame(const uint8_t *frame, size_t len) {
  if (!this->dev_ready_ || this->out_xfer_ == nullptr)
    return false;
  if (this->out_busy_)
    return false;  // previous OUT still in flight
  // Choose the smallest HID report ID whose payload fits the frame.
  uint8_t report_id = (len <= 20) ? 0x01 : (len <= 120) ? 0x02 : 0x04;
  if (len + 1 > this->out_xfer_->data_buffer_size)
    return false;

  this->out_xfer_->data_buffer[0] = report_id;
  memcpy(this->out_xfer_->data_buffer + 1, frame, len);
  this->out_xfer_->device_handle = this->dev_;
  this->out_xfer_->bEndpointAddress = this->ep_out_addr_;
  this->out_xfer_->callback = &PmUsbHost::out_transfer_cb;
  this->out_xfer_->context = this;
  this->out_xfer_->num_bytes = len + 1;

  this->out_busy_ = true;
  esp_err_t err = usb_host_transfer_submit(this->out_xfer_);
  if (err != ESP_OK) {
    this->out_busy_ = false;
    ESP_LOGW(TAG, "OUT submit failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

void PmUsbHost::out_transfer_cb(usb_transfer_t *xfer) {
  static_cast<PmUsbHost *>(xfer->context)->out_busy_ = false;
}

void PmUsbHost::close_device_() {
  this->dev_ready_ = false;
  if (this->in_xfer_ != nullptr) {
    usb_host_transfer_free(this->in_xfer_);
    this->in_xfer_ = nullptr;
  }
  if (this->out_xfer_ != nullptr) {
    usb_host_transfer_free(this->out_xfer_);
    this->out_xfer_ = nullptr;
  }
  if (this->dev_ != nullptr) {
    usb_host_interface_release(this->client_, this->dev_, this->itf_num_);
    usb_host_device_close(this->client_, this->dev_);
    this->dev_ = nullptr;
  }
  this->out_busy_ = false;
  this->pid_ = 0;
  ESP_LOGI(TAG, "Concept2 PM detached");
}

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
