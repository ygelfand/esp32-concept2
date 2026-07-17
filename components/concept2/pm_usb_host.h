#pragma once

#ifdef USE_ESP_IDF

#include <cstddef>
#include <cstdint>
#include <functional>

#include "usb/usb_host.h"

namespace esphome {
namespace concept2 {

// Concept2 USB vendor ID and the product IDs for each PM generation.
constexpr uint16_t C2_VENDOR_ID = 0x17A4;
constexpr uint16_t C2_PID_PM3 = 0x0001;
constexpr uint16_t C2_PID_PM4 = 0x0002;
constexpr uint16_t C2_PID_PM5 = 0x0003;

// USB HID host driver for the Concept2 Performance Monitor.
//
// The PM enumerates as a HID-class device with two interrupt endpoints (IN/OUT).
// CSAFE frames are exchanged over those endpoints, each prefixed by a 1-byte HID
// report ID. We deliberately use the raw usb_host client API (not the HID class
// driver) so we own both the interrupt-IN and interrupt-OUT endpoints directly.
//
// All USB event handling and transfer completion runs on a dedicated FreeRTOS
// task. Received frames are delivered via the frame callback (invoked from that
// task) with the report-ID byte already stripped.
class PmUsbHost {
 public:
  using FrameCallback = std::function<void(const uint8_t *data, size_t len)>;

  // Install the USB host stack and start the event task. Returns false on
  // failure (already logged).
  bool begin(int task_core = 0);

  void set_frame_callback(FrameCallback cb) { this->frame_cb_ = std::move(cb); }

  bool connected() const { return this->dev_ready_; }
  uint16_t product_id() const { return this->pid_; }

  // Wrap a CSAFE frame in the smallest fitting HID report and submit it on the
  // interrupt-OUT endpoint. Thread-safe; returns false if not connected, an OUT
  // transfer is already in flight, or the frame is too large.
  bool write_frame(const uint8_t *frame, size_t len);

 private:
  // FreeRTOS entry point (static trampoline -> task_loop).
  static void task_trampoline(void *arg);
  void task_loop();

  // usb_host client event callback (device attach/detach).
  static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg);
  void on_client_event(const usb_host_client_event_msg_t *msg);

  // Transfer completion callbacks.
  static void in_transfer_cb(usb_transfer_t *xfer);
  static void out_transfer_cb(usb_transfer_t *xfer);

  bool open_device_(uint8_t addr);
  void close_device_();
  bool claim_interface_();
  void submit_in_();

  usb_host_client_handle_t client_{nullptr};
  usb_device_handle_t dev_{nullptr};
  uint8_t dev_addr_{0};
  uint16_t pid_{0};

  uint8_t itf_num_{0};
  uint8_t ep_in_addr_{0};
  uint8_t ep_out_addr_{0};
  uint16_t ep_in_mps_{64};
  uint16_t ep_out_mps_{64};

  usb_transfer_t *in_xfer_{nullptr};
  usb_transfer_t *out_xfer_{nullptr};
  volatile bool out_busy_{false};

  volatile bool dev_ready_{false};
  volatile bool dev_gone_{false};

  FrameCallback frame_cb_{};
};

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
