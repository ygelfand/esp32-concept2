#pragma once

#include <cstddef>
#include <cstdint>

#include "rowing_metrics.h"

// CSAFE protocol codec for the Concept2 Performance Monitor.
//
// Framing (standard frame):
//   0xF1 | <contents> | <checksum> | 0xF2
//   checksum = byte-by-byte XOR of <contents>
//   byte-stuffing: any content byte in 0xF0..0xF3 is emitted as 0xF3 followed
//   by (byte & 0x03). The checksum is computed over the UNSTUFFED contents and
//   is itself stuffed if needed.
//
// Commands come in two forms inside <contents>:
//   * short command  - single byte with MSB set (0x80..0xFF), no data
//   * long command   - byte with MSB clear (0x00..0x7F), followed by [len][data]
//
// Concept2 proprietary "get" commands are short commands carried as the data of
// the long wrapper command SETUSERCFG1 (0x1A). e.g. reading work distance:
//   contents = 0x1A 0x01 0xA3        (wrapper, len=1, PM_GET_WORKDISTANCE)
//
// Responses always use [identifier][byteCount][data...] blocks preceded by a
// single status byte. Proprietary responses are nested inside a 0x1A block.
//
// All multi-byte values are little-endian.

namespace esphome {
namespace concept2 {
namespace csafe {

// Frame delimiters / control bytes.
constexpr uint8_t FRAME_START_STD = 0xF1;
constexpr uint8_t FRAME_START_EXT = 0xF0;
constexpr uint8_t FRAME_STOP = 0xF2;
constexpr uint8_t FRAME_STUFF = 0xF3;

// Public CSAFE commands we send (all short commands).
constexpr uint8_t CMD_GETSTATUS = 0x80;
constexpr uint8_t CMD_GETTWORK = 0xA0;       // elapsed time, H:M:S
constexpr uint8_t CMD_GETHORIZONTAL = 0xA1;  // distance, meters
constexpr uint8_t CMD_GETCALORIES = 0xA3;    // total kcal
constexpr uint8_t CMD_GETPACE = 0xA6;        // sec/km
constexpr uint8_t CMD_GETCADENCE = 0xA7;     // strokes/min
constexpr uint8_t CMD_GETHRCUR = 0xB0;       // bpm
constexpr uint8_t CMD_GETPOWER = 0xB4;       // watts

// Long wrapper command carrying Concept2 proprietary short commands.
constexpr uint8_t CMD_PROP_WRAPPER = 0x1A;  // CSAFE_SETUSERCFG1

// Concept2 proprietary short "get" commands (carried inside CMD_PROP_WRAPPER).
constexpr uint8_t PM_GET_WORKTIME = 0xA0;         // 4-byte @0.01s + 1 frac
constexpr uint8_t PM_GET_WORKDISTANCE = 0xA3;     // 4-byte @0.1m + 1 frac
constexpr uint8_t PM_GET_WORKOUTTYPE = 0x89;      // enum
constexpr uint8_t PM_GET_WORKOUTSTATE = 0x8D;     // enum 0..13
constexpr uint8_t PM_GET_STROKESTATE = 0xBF;      // enum 0..4
constexpr uint8_t PM_GET_DRAGFACTOR = 0xC1;       // unitless
constexpr uint8_t PM_GET_STROKERATE = 0xB3;       // strokes/min
constexpr uint8_t PM_GET_STROKE_500MPACE = 0xA8;  // sec/500m
constexpr uint8_t PM_GET_STROKE_POWER = 0xA9;     // watts

// Number of rotating poll blocks. The caller cycles `block_index` 0..count-1 so
// each poll requests only a few proprietary getters (the PM's reply wrapper
// mangles/over-runs if too many are batched at once).
size_t poll_block_count();

// Build poll frame for `block_index` (F1 .. F2, stuffed, WITHOUT the leading HID
// report-ID byte). Returns bytes written to `out`, or 0 if `out_cap` too small.
size_t build_poll_frame(size_t block_index, uint8_t *out, size_t out_cap);

// Build an arbitrary standard frame from raw command `contents`. Handles
// checksum + byte-stuffing + start/stop flags. Returns bytes written, or 0.
size_t build_frame(const uint8_t *contents, size_t contents_len, uint8_t *out, size_t out_cap);

// Parse a raw HID payload (a CSAFE frame, report-ID byte already stripped) and
// update `m` in place. Returns true if a well-formed, checksum-valid frame was
// decoded. Unknown command blocks are skipped, not treated as errors.
bool parse_response(const uint8_t *frame, size_t len, RowingMetrics &m);

}  // namespace csafe
}  // namespace concept2
}  // namespace esphome
