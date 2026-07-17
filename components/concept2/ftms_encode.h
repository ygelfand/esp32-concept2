#pragma once

#include <cstddef>
#include <cstdint>

#include "rowing_metrics.h"

// FTMS (Fitness Machine Service) encoding, shared by the BLE and DirCon
// transports. Both carry byte-identical payloads; only the framing differs.
// All FTMS multi-byte values are little-endian.

namespace esphome {
namespace concept2 {
namespace ftms {

// 16-bit UUIDs (assigned numbers).
constexpr uint16_t UUID_SERVICE = 0x1826;        // Fitness Machine Service
constexpr uint16_t UUID_ROWER_DATA = 0x2AD1;     // Rower Data (Notify)
constexpr uint16_t UUID_FEATURE = 0x2ACC;        // Fitness Machine Feature (Read)
constexpr uint16_t UUID_CONTROL_POINT = 0x2AD9;  // Control Point (Write/Indicate)
constexpr uint16_t UUID_STATUS = 0x2ADA;         // Fitness Machine Status (Notify)

// Max size of an encoded Rower Data record with the field set we emit.
constexpr size_t ROWER_DATA_MAX = 24;
// Fitness Machine Feature is a fixed 8-byte value.
constexpr size_t FEATURE_LEN = 8;

// Encode a Rower Data (0x2AD1) notification payload from `m`. Returns the number
// of bytes written to `out` (`out` must be >= ROWER_DATA_MAX). The leading
// uint16 flags field advertises exactly the fields that follow.
size_t encode_rower_data(const RowingMetrics &m, uint8_t *out);

// Write the fixed 8-byte Fitness Machine Feature (0x2ACC) value to `out`
// (must be >= FEATURE_LEN). Advertises the metrics we actually provide so apps
// classify the device as a valid rower.
void encode_feature(uint8_t *out);

}  // namespace ftms
}  // namespace concept2
}  // namespace esphome
