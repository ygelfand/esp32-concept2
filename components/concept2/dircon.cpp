#include "dircon.h"

#ifdef USE_ESP_IDF

#include <cstdio>
#include <cstring>

#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_mac.h"
#include "lwip/sockets.h"
#include "mdns.h"

namespace esphome {
namespace concept2 {

static const char *const TAG = "concept2.dircon";

namespace {
// Expand a 16-bit UUID into its 128-bit form, big-endian (DirCon wire order).
void uuid128(uint16_t u, uint8_t out[16]) {
  static const uint8_t base[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
                                   0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
  memcpy(out, base, 16);
  out[2] = static_cast<uint8_t>(u >> 8);
  out[3] = static_cast<uint8_t>(u & 0xFF);
}
// Extract the 16-bit UUID from an incoming 128-bit big-endian UUID.
uint16_t uuid16_from(const uint8_t *p) {
  return static_cast<uint16_t>((p[2] << 8) | p[3]);
}

constexpr uint8_t DC_PROP_READ = 0x01;
constexpr uint8_t DC_PROP_WRITE = 0x02;
constexpr uint8_t DC_PROP_NOTIFY = 0x04;
}  // namespace

bool DirCon::begin(uint16_t port, const std::string &name, int task_core) {
  this->port_ = port;
  this->name_ = name;
  this->register_mdns_();
  BaseType_t ok = xTaskCreatePinnedToCore(&DirCon::task_trampoline, "c2_dircon", 5120, this, 4,
                                          nullptr, task_core);
  if (ok != pdPASS) {
    ESP_LOGE(TAG, "failed to create DirCon task");
    return false;
  }
  ESP_LOGI(TAG, "DirCon server on tcp/%u", this->port_);
  return true;
}

void DirCon::register_mdns_() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2],
           mac[3], mac[4], mac[5]);
  char serial[16];
  snprintf(serial, sizeof(serial), "C2-%02X%02X%02X", mac[3], mac[4], mac[5]);

  mdns_txt_item_t txt[3] = {
      {"serial-number", serial},
      {"mac-address", mac_str},
      {"ble-service-uuids", "1826"},
  };
  // Assumes ESPHome's mdns component already ran mdns_init().
  esp_err_t err = mdns_service_add(nullptr, "_wahoo-fitness-tnp", "_tcp", this->port_, txt, 3);
  if (err != ESP_OK)
    ESP_LOGW(TAG, "mdns_service_add failed: %s (is the mdns component enabled?)",
             esp_err_to_name(err));
  else
    ESP_LOGI(TAG, "advertising _wahoo-fitness-tnp._tcp (serial %s)", serial);
}

void DirCon::task_trampoline(void *arg) { static_cast<DirCon *>(arg)->task_loop(); }

void DirCon::task_loop() {
  int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed");
    vTaskDelete(nullptr);
    return;
  }
  int one = 1;
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->port_);
  if (::bind(listen_fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 ||
      ::listen(listen_fd, 2) < 0) {
    ESP_LOGE(TAG, "bind/listen on %u failed", this->port_);
    ::close(listen_fd);
    vTaskDelete(nullptr);
    return;
  }

  while (true) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(listen_fd, &rfds);
    int maxfd = listen_fd;
    for (auto &c : this->clients_) {
      if (c.fd >= 0) {
        FD_SET(c.fd, &rfds);
        if (c.fd > maxfd)
          maxfd = c.fd;
      }
    }

    timeval tv = {.tv_sec = 0, .tv_usec = 100000};  // 100 ms
    int n = ::select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
    if (n > 0) {
      if (FD_ISSET(listen_fd, &rfds))
        this->accept_client_(listen_fd);
      for (auto &c : this->clients_)
        if (c.fd >= 0 && FD_ISSET(c.fd, &rfds))
          this->service_client_(c);
    }

    // Flush a pending Rower Data notification to all subscribers.
    if (this->notify_pending_) {
      this->notify_pending_ = false;
      uint8_t payload[16 + ftms::ROWER_DATA_MAX];
      uuid128(ftms::UUID_ROWER_DATA, payload);
      size_t dl = ftms::encode_rower_data(this->latest_, payload + 16);
      for (auto &c : this->clients_)
        if (c.fd >= 0 && c.sub_rower)
          this->send_message_(c.fd, MSG_CHAR_NOTIFICATION, 0, RC_SUCCESS, payload,
                              static_cast<uint16_t>(16 + dl));
    }
  }
}

void DirCon::accept_client_(int listen_fd) {
  int fd = ::accept(listen_fd, nullptr, nullptr);
  if (fd < 0)
    return;
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  for (auto &c : this->clients_) {
    if (c.fd < 0) {
      c.fd = fd;
      c.sub_rower = false;
      c.rx.clear();
      ESP_LOGI(TAG, "DirCon client connected (fd %d)", fd);
      return;
    }
  }
  ESP_LOGW(TAG, "no free client slot, rejecting");
  ::close(fd);
}

void DirCon::service_client_(Client &c) {
  uint8_t tmp[512];
  int r = ::recv(c.fd, tmp, sizeof(tmp), 0);
  if (r <= 0) {
    this->drop_client_(c);
    return;
  }
  c.rx.insert(c.rx.end(), tmp, tmp + r);
  if (c.rx.size() > 2048) {  // runaway guard
    ESP_LOGW(TAG, "client rx overflow, dropping");
    this->drop_client_(c);
    return;
  }
  // Extract complete messages (6-byte header + big-endian length).
  while (c.rx.size() >= 6) {
    uint16_t len = static_cast<uint16_t>((c.rx[4] << 8) | c.rx[5]);
    size_t total = 6u + len;
    if (c.rx.size() < total)
      break;
    this->process_message_(c, c.rx.data(), total);
    c.rx.erase(c.rx.begin(), c.rx.begin() + total);
  }
}

void DirCon::process_message_(Client &c, const uint8_t *msg, size_t len) {
  uint8_t type = msg[1];
  uint8_t seq = msg[2];
  const uint8_t *payload = msg + 6;
  uint16_t plen = static_cast<uint16_t>(len - 6);

  switch (type) {
    case MSG_DISCOVER_SERVICES: {
      uint8_t out[16];
      uuid128(ftms::UUID_SERVICE, out);
      this->send_message_(c.fd, type, seq, RC_SUCCESS, out, sizeof(out));
      break;
    }
    case MSG_DISCOVER_CHARS: {
      if (plen < 16 || uuid16_from(payload) != ftms::UUID_SERVICE) {
        this->send_message_(c.fd, type, seq, RC_SVC_NOT_FOUND, nullptr, 0);
        break;
      }
      // service UUID (16) + per char { UUID(16), property(1) }.
      struct {
        uint16_t uuid;
        uint8_t prop;
      } chars[] = {{ftms::UUID_ROWER_DATA, DC_PROP_NOTIFY},
                   {ftms::UUID_FEATURE, DC_PROP_READ},
                   {ftms::UUID_CONTROL_POINT, DC_PROP_WRITE}};
      uint8_t out[16 + 3 * 17];
      uint8_t *p = out;
      uuid128(ftms::UUID_SERVICE, p);
      p += 16;
      for (auto &ch : chars) {
        uuid128(ch.uuid, p);
        p += 16;
        *p++ = ch.prop;
      }
      this->send_message_(c.fd, type, seq, RC_SUCCESS, out, static_cast<uint16_t>(p - out));
      break;
    }
    case MSG_READ_CHAR: {
      if (plen < 16) {
        this->send_message_(c.fd, type, seq, RC_CHAR_NOT_FOUND, nullptr, 0);
        break;
      }
      uint16_t u = uuid16_from(payload);
      uint8_t out[16 + ftms::ROWER_DATA_MAX];
      uuid128(u, out);
      if (u == ftms::UUID_FEATURE) {
        ftms::encode_feature(out + 16);
        this->send_message_(c.fd, type, seq, RC_SUCCESS, out, 16 + ftms::FEATURE_LEN);
      } else if (u == ftms::UUID_ROWER_DATA) {
        size_t dl = ftms::encode_rower_data(this->latest_, out + 16);
        this->send_message_(c.fd, type, seq, RC_SUCCESS, out, static_cast<uint16_t>(16 + dl));
      } else {
        this->send_message_(c.fd, type, seq, RC_CHAR_NOT_FOUND, nullptr, 0);
      }
      break;
    }
    case MSG_WRITE_CHAR: {
      uint16_t u = plen >= 16 ? uuid16_from(payload) : 0;
      uint8_t rc = (u == ftms::UUID_CONTROL_POINT) ? RC_SUCCESS : RC_OP_NOT_SUPPORTED;
      this->send_message_(c.fd, type, seq, rc, nullptr, 0);
      break;
    }
    case MSG_ENABLE_NOTIFY: {
      uint16_t u = plen >= 16 ? uuid16_from(payload) : 0;
      if (u == ftms::UUID_ROWER_DATA) {
        c.sub_rower = true;
        ESP_LOGD(TAG, "client subscribed to rower data");
        this->send_message_(c.fd, type, seq, RC_SUCCESS, nullptr, 0);
      } else {
        this->send_message_(c.fd, type, seq, RC_CHAR_NOT_FOUND, nullptr, 0);
      }
      break;
    }
    default:
      this->send_message_(c.fd, type, seq, RC_UNKNOWN_MSG, nullptr, 0);
      break;
  }
}

void DirCon::send_message_(int fd, uint8_t type, uint8_t seq, uint8_t rc, const uint8_t *payload,
                           uint16_t payload_len) {
  uint8_t header[6] = {0x01, type, seq, rc, static_cast<uint8_t>(payload_len >> 8),
                       static_cast<uint8_t>(payload_len & 0xFF)};
  if (::send(fd, header, sizeof(header), 0) < 0)
    return;
  if (payload_len > 0)
    ::send(fd, payload, payload_len, 0);
}

void DirCon::drop_client_(Client &c) {
  if (c.fd >= 0) {
    ESP_LOGI(TAG, "DirCon client disconnected (fd %d)", c.fd);
    ::close(c.fd);
  }
  c.fd = -1;
  c.sub_rower = false;
  c.rx.clear();
}

void DirCon::publish(const RowingMetrics &m) {
  this->latest_ = m;
  this->notify_pending_ = true;
}

}  // namespace concept2
}  // namespace esphome

#endif  // USE_ESP_IDF
