#pragma once

#ifdef USE_ESP_IDF

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ftms_encode.h"
#include "rowing_metrics.h"

namespace esphome {
namespace concept2 {

// Wahoo "DirCon" (Direct Connect / WFTNP) server: BLE-GATT-over-TCP.
//
// Advertises `_wahoo-fitness-tnp._tcp` over mDNS and serves the same FTMS Rower
// Data payloads the BLE transport uses, wrapped in DirCon's 6-byte framed
// messages. UUIDs and the header length field are big-endian; the FTMS payload
// itself stays little-endian.
class DirCon {
 public:
  // DirCon message identifiers.
  enum MsgType : uint8_t {
    MSG_DISCOVER_SERVICES = 0x01,
    MSG_DISCOVER_CHARS = 0x02,
    MSG_READ_CHAR = 0x03,
    MSG_WRITE_CHAR = 0x04,
    MSG_ENABLE_NOTIFY = 0x05,
    MSG_CHAR_NOTIFICATION = 0x06,
  };
  enum RespCode : uint8_t {
    RC_SUCCESS = 0x00,
    RC_UNKNOWN_MSG = 0x01,
    RC_ERROR = 0x02,
    RC_SVC_NOT_FOUND = 0x03,
    RC_CHAR_NOT_FOUND = 0x04,
    RC_OP_NOT_SUPPORTED = 0x05,
    RC_WRITE_FAILED = 0x06,
  };

  bool begin(uint16_t port, const std::string &name, int task_core = 1);

  // Broadcast the latest metrics as a Rower Data notification to subscribers.
  void publish(const RowingMetrics &m);

 private:
  struct Client {
    int fd{-1};
    bool sub_rower{false};
    std::vector<uint8_t> rx;
  };

  static void task_trampoline(void *arg);
  void task_loop();
  void register_mdns_();
  void accept_client_(int listen_fd);
  void service_client_(Client &c);
  void process_message_(Client &c, const uint8_t *msg, size_t len);
  void send_message_(int fd, uint8_t type, uint8_t seq, uint8_t rc, const uint8_t *payload,
                     uint16_t payload_len);
  void drop_client_(Client &c);

  uint16_t port_{36866};
  std::string name_{"Concept2 Rower"};
  static constexpr int MAX_CLIENTS = 3;
  Client clients_[MAX_CLIENTS];
  RowingMetrics latest_{};
  volatile bool notify_pending_{false};
};

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
