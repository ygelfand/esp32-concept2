#include "csafe.h"

namespace esphome {
namespace concept2 {
namespace csafe {

namespace {

// Rotating poll blocks, each a small group of proprietary getters wrapped in
// 0x1A. The caller cycles through them one per poll.
// Proprietary getters (wrapped in 0x1A) that the PM4 answers reliably.
const uint8_t BLK_TIME_DIST[] = {PM_GET_WORKTIME, PM_GET_WORKDISTANCE};
const uint8_t BLK_STATE_DRAG[] = {PM_GET_STROKESTATE, PM_GET_DRAGFACTOR, PM_GET_WORKOUTSTATE};
// Public getters (sent bare) - the PM returns empty for the proprietary
// pace/power/stroke-rate getters, so use the standard CSAFE ones.
const uint8_t BLK_PUB_POWER_PACE_RATE[] = {CMD_GETPOWER, CMD_GETPACE, CMD_GETCADENCE};
const uint8_t BLK_PUB_HR_CAL[] = {CMD_GETHRCUR, CMD_GETCALORIES};

struct PollBlock {
  const uint8_t *cmds;
  uint8_t count;
  bool proprietary;  // wrap in 0x1A when true, else send bare public commands
};
const PollBlock POLL_BLOCKS[] = {
    {BLK_TIME_DIST, 2, true},
    {BLK_PUB_POWER_PACE_RATE, 3, false},
    {BLK_STATE_DRAG, 3, true},
    {BLK_PUB_HR_CAL, 2, false},
};
const size_t NUM_POLL_BLOCKS = sizeof(POLL_BLOCKS) / sizeof(POLL_BLOCKS[0]);

inline uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
inline uint16_t le16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

// Append one content byte to `out`, byte-stuffing if it collides with a control
// byte. Returns false if it would overflow.
inline bool stuff_byte(uint8_t b, uint8_t *out, size_t out_cap, size_t &n) {
  if (b >= FRAME_START_EXT && b <= FRAME_STUFF) {  // 0xF0..0xF3
    if (n + 2 > out_cap)
      return false;
    out[n++] = FRAME_STUFF;
    out[n++] = b & 0x03;
  } else {
    if (n + 1 > out_cap)
      return false;
    out[n++] = b;
  }
  return true;
}

// Apply a decoded public-command response block to the metrics.
void apply_public(uint8_t id, const uint8_t *data, uint8_t len, RowingMetrics &m) {
  switch (id) {
    case CMD_GETPOWER:  // [watts LSB, MSB, units]
      if (len >= 2)
        m.inst_power_w = static_cast<int16_t>(le16(data));
      break;
    case CMD_GETCADENCE:  // [spm LSB, MSB, units]
      if (len >= 2)
        m.stroke_rate_spm = static_cast<float>(le16(data));
      break;
    case CMD_GETPACE:  // [sec/km LSB, MSB, units] -> FTMS wants sec/500m
      if (len >= 2)
        m.inst_pace_s500 = static_cast<uint16_t>(le16(data) / 2);
      break;
    case CMD_GETHORIZONTAL:  // [distance LSB, MSB, units], meters
      if (len >= 2)
        m.total_distance_m = static_cast<float>(le16(data));
      break;
    case CMD_GETCALORIES:  // [kcal LSB, MSB]
      if (len >= 2)
        m.total_energy_kcal = le16(data);
      break;
    case CMD_GETHRCUR:  // [bpm]
      if (len >= 1)
        m.heart_rate_bpm = data[0];
      break;
    case CMD_GETTWORK:  // [hours, minutes, seconds]
      if (len >= 3)
        m.elapsed_time_s =
            static_cast<uint16_t>(data[0] * 3600 + data[1] * 60 + data[2]);
      break;
    default:
      break;
  }
}

// Apply a decoded proprietary (0x1A-wrapped) response block to the metrics.
//
// NOTE on scaling: the fixed-point scales below match the CSAFE Communication
// Definition as cross-referenced from Py3Row/ErgometerJS, but the exact
// resolution of a few PM getters varies by firmware. Sanity-check WORKTIME,
// WORKDISTANCE and 500m pace against the PM console during bring-up.
void apply_proprietary(uint8_t id, const uint8_t *data, uint8_t len, RowingMetrics &m) {
  switch (id) {
    case PM_GET_WORKTIME:  // 4-byte value @0.01 s (+ 1 fractional byte)
      if (len >= 4)
        m.elapsed_time_s = static_cast<uint16_t>(le32(data) / 100);
      break;
    case PM_GET_WORKDISTANCE:  // 4-byte value @0.1 m (+ 1 fractional byte)
      if (len >= 4)
        m.total_distance_m = static_cast<float>(le32(data)) / 10.0f;
      break;
    case PM_GET_STROKERATE:  // strokes/min
      if (len >= 1)
        m.stroke_rate_spm = static_cast<float>(data[0]);
      break;
    case PM_GET_STROKE_500MPACE:  // 4-byte value, sec/500m @0.01 s
      if (len >= 4)
        m.inst_pace_s500 = static_cast<uint16_t>(le32(data) / 100);
      break;
    case PM_GET_STROKE_POWER:  // 4-byte value, watts
      if (len >= 4)
        m.inst_power_w = static_cast<int16_t>(le32(data));
      break;
    case PM_GET_STROKESTATE:
      if (len >= 1)
        m.stroke_state = static_cast<StrokeState>(data[0]);
      break;
    case PM_GET_WORKOUTSTATE:
      if (len >= 1)
        m.workout_state = static_cast<WorkoutState>(data[0]);
      break;
    case PM_GET_DRAGFACTOR:
      if (len >= 1)
        m.drag_factor = data[0];
      break;
    default:
      break;
  }
}

}  // namespace

size_t build_frame(const uint8_t *contents, size_t contents_len, uint8_t *out, size_t out_cap) {
  if (out_cap < 3)
    return 0;
  size_t n = 0;
  out[n++] = FRAME_START_STD;

  uint8_t checksum = 0;
  for (size_t i = 0; i < contents_len; i++) {
    checksum ^= contents[i];
    if (!stuff_byte(contents[i], out, out_cap, n))
      return 0;
  }
  if (!stuff_byte(checksum, out, out_cap, n))
    return 0;

  if (n + 1 > out_cap)
    return 0;
  out[n++] = FRAME_STOP;
  return n;
}

size_t poll_block_count() { return NUM_POLL_BLOCKS; }

size_t build_poll_frame(size_t block_index, uint8_t *out, size_t out_cap) {
  const PollBlock &b = POLL_BLOCKS[block_index % NUM_POLL_BLOCKS];
  uint8_t contents[16];
  size_t n = 0;
  if (b.proprietary) {
    contents[n++] = CMD_PROP_WRAPPER;
    contents[n++] = b.count;
  }
  for (uint8_t i = 0; i < b.count; i++)
    contents[n++] = b.cmds[i];
  return build_frame(contents, n, out, out_cap);
}

bool parse_response(const uint8_t *frame, size_t len, RowingMetrics &m) {
  // Locate the start flag.
  size_t start = 0;
  while (start < len && frame[start] != FRAME_START_STD && frame[start] != FRAME_START_EXT)
    start++;
  if (start >= len)
    return false;
  bool extended = frame[start] == FRAME_START_EXT;
  size_t i = start + 1;

  // Unstuff into a scratch buffer, stopping at the stop flag.
  uint8_t buf[256];
  size_t n = 0;
  bool got_stop = false;
  for (; i < len; i++) {
    uint8_t b = frame[i];
    if (b == FRAME_STOP) {
      got_stop = true;
      break;
    }
    if (b == FRAME_STUFF) {
      if (i + 1 >= len)
        return false;
      b = 0xF0 | (frame[++i] & 0x03);
    }
    if (n >= sizeof(buf))
      return false;
    buf[n++] = b;
  }
  if (!got_stop || n < 2)
    return false;

  // Last unstuffed byte is the checksum over the preceding contents.
  size_t content_len = n - 1;
  uint8_t checksum = 0;
  for (size_t k = 0; k < content_len; k++)
    checksum ^= buf[k];
  if (checksum != buf[content_len])
    return false;

  // Extended frames prefix contents with dest+src address bytes; skip them.
  size_t p = 0;
  if (extended) {
    if (content_len < 2)
      return false;
    p = 2;
  }
  if (p >= content_len)
    return false;

  // First content byte is the status byte (state machine + prev-frame status).
  // We don't gate on it here; the orchestrator inspects workout_state instead.
  p++;  // skip status

  // Walk [identifier][byteCount][data] blocks (proprietary nested in 0x1A).
  while (p < content_len) {
    uint8_t id = buf[p++];
    if (id == CMD_PROP_WRAPPER) {
      if (p >= content_len)
        break;
      uint8_t wlen = buf[p++];
      size_t wend = p + wlen;
      if (wend > content_len)
        wend = content_len;
      while (p < wend) {
        uint8_t pid = buf[p++];
        if (p >= wend)
          break;
        uint8_t plen = buf[p++];
        if (p + plen > wend)
          plen = static_cast<uint8_t>(wend - p);
        apply_proprietary(pid, &buf[p], plen, m);
        p += plen;
      }
    } else {
      if (p >= content_len)
        break;
      uint8_t dlen = buf[p++];
      if (p + dlen > content_len)
        dlen = static_cast<uint8_t>(content_len - p);
      apply_public(id, &buf[p], dlen, m);
      p += dlen;
    }
  }

  m.valid = true;
  return true;
}

}  // namespace csafe
}  // namespace concept2
}  // namespace esphome
