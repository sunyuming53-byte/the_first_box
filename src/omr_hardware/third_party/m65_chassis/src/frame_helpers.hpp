// SPDX-License-Identifier: MIT
// Internal frame construction/validation helpers for M65 chassis protocol.
// Shared between chassis.cpp and test_serial_framing.cpp.
// NOT part of the public API — do NOT include from outside this package.
#pragma once

#include <cstdint>
#include <cstring>

#include <algorithm>
#include <array>

namespace m65 {
namespace internal {

// ── Protocol constants ─────────────────────────────────────────────────

constexpr uint8_t kFrameLen = 40;
constexpr uint8_t kFrameHead0 = 0x7F;
constexpr uint8_t kFrameHead1 = 0x7F;
constexpr uint8_t kFrameTail0 = 0x0D;  // CR
constexpr uint8_t kFrameTail1 = 0x0A;  // LF
constexpr uint8_t kChksOffset = 37;    // checksum byte position
constexpr uint8_t kChksRangeEnd = 37;  // sum bytes [0..36] for checksum
constexpr uint8_t kHeadOffset = 0;
constexpr uint8_t kLenOffset = 2;
constexpr uint8_t kCmdOffset = 3;
constexpr uint8_t kResultOffset = 4;
constexpr uint8_t kDataOffset = 5;

// ── Big-endian int16 helpers ──────────────────────────────────────────

inline void write_int16_be(uint8_t* buf, int16_t val) {
    buf[0] = static_cast<uint8_t>((val >> 8) & 0xFF);
    buf[1] = static_cast<uint8_t>(val & 0xFF);
}

inline int16_t read_int16_be(const uint8_t* buf) {
    return static_cast<int16_t>((static_cast<uint16_t>(buf[0]) << 8) | buf[1]);
}

// ── Checksum ──────────────────────────────────────────────────────────

// Computes checksum: (~sum(bytes[0..chksRangeEnd-1]) + 1) & 0xFF
inline uint8_t compute_checksum(const std::array<uint8_t, kFrameLen>& frame) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < kChksRangeEnd; ++i) {
        sum += frame[i];
    }
    return static_cast<uint8_t>((~sum + 1) & 0xFF);
}

// Validates checksum on a received frame
inline bool verify_checksum(const std::array<uint8_t, kFrameLen>& frame) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < kChksRangeEnd; ++i) {
        sum += frame[i];
    }
    uint8_t expected = static_cast<uint8_t>((~sum + 1) & 0xFF);
    return frame[kChksOffset] == expected;
}

// ── Frame construction ────────────────────────────────────────────────

// Builds a full 40-byte frame with command and data payload.
// cmd: MSG_ID byte placed at byte 3.
// data: pointer to payload bytes to place starting at byte 4.
// data_len: number of payload bytes (≤33).
inline std::array<uint8_t, kFrameLen> build_frame(uint8_t cmd, const uint8_t* data,
                                                  size_t data_len) {
    std::array<uint8_t, kFrameLen> frame{};
    frame[kHeadOffset] = kFrameHead0;
    frame[kHeadOffset + 1] = kFrameHead1;
    frame[kLenOffset] = kFrameLen;
    frame[kCmdOffset] = cmd;

    if (data != nullptr && data_len > 0) {
        size_t copy_len = std::min(data_len, size_t(kChksRangeEnd - kDataOffset));
        std::memcpy(&frame[kDataOffset], data, copy_len);
    }

    frame[kChksOffset] = compute_checksum(frame);
    frame[kFrameLen - 2] = kFrameTail0;
    frame[kFrameLen - 1] = kFrameTail1;
    return frame;
}

}  // namespace internal
}  // namespace m65
