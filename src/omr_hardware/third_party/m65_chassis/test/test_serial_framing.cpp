// SPDX-License-Identifier: MIT
// Unit tests for M65 chassis serial framing logic.
// No hardware dependency — tests only build_frame / checksum / verify.

#include <gtest/gtest.h>

#include "../src/frame_helpers.hpp"

namespace {

using m65::internal::build_frame;
using m65::internal::compute_checksum;
using m65::internal::verify_checksum;
using m65::internal::kFrameLen;
using m65::internal::kFrameHead0;
using m65::internal::kFrameHead1;
using m65::internal::kFrameTail0;
using m65::internal::kFrameTail1;
using m65::internal::kChksOffset;

TEST(SerialFramingTest, BuildFrameProducesValid40ByteOutput) {
    // Build a frame with known command and data payload.
    const uint8_t cmd = 0x42;
    const uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    const size_t data_len = sizeof(data);

    auto frame = build_frame(cmd, data, data_len);

    // Frame must be exactly 40 bytes.
    EXPECT_EQ(frame.size(), kFrameLen);

    // Header bytes.
    EXPECT_EQ(frame[0], kFrameHead0);
    EXPECT_EQ(frame[1], kFrameHead1);

    // Length byte.
    EXPECT_EQ(frame[2], kFrameLen);

    // Command byte.
    EXPECT_EQ(frame[3], cmd);

    // Data payload (bytes 5-8 = indices 5, 6, 7, 8).
    EXPECT_EQ(frame[5], data[0]);
    EXPECT_EQ(frame[6], data[1]);
    EXPECT_EQ(frame[7], data[2]);
    EXPECT_EQ(frame[8], data[3]);

    // Tail bytes (indices 38 and 39).
    EXPECT_EQ(frame[38], kFrameTail0);
    EXPECT_EQ(frame[39], kFrameTail1);

    // Checksum must be valid.
    EXPECT_TRUE(verify_checksum(frame));
}

TEST(SerialFramingTest, BuildFrameWithNullDataProducesValidOutput) {
    // null-pointer data is valid (zero-data command like baud check).
    auto frame = build_frame(0x01, nullptr, 0);

    EXPECT_EQ(frame.size(), kFrameLen);
    EXPECT_EQ(frame[0], kFrameHead0);
    EXPECT_EQ(frame[1], kFrameHead1);
    EXPECT_EQ(frame[2], kFrameLen);
    EXPECT_EQ(frame[3], 0x01);
    EXPECT_EQ(frame[38], kFrameTail0);
    EXPECT_EQ(frame[39], kFrameTail1);

    // All data bytes (5-36) must be zero.
    for (size_t i = 5; i <= kChksOffset - 1; ++i) {
        EXPECT_EQ(frame[i], 0) << "byte " << i << " should be zero";
    }

    // Checksum must be valid.
    EXPECT_TRUE(verify_checksum(frame));
}

TEST(SerialFramingTest, VerifyChecksumAcceptsGoodFrame) {
    auto frame = build_frame(0x03, nullptr, 0);
    EXPECT_TRUE(verify_checksum(frame));
}

TEST(SerialFramingTest, VerifyChecksumRejectsCorruptedFrame) {
    auto frame = build_frame(0x01, nullptr, 0);

    // Flip a bit in the data region (byte 10).
    frame[10] ^= 0xFF;
    EXPECT_FALSE(verify_checksum(frame));
}

TEST(SerialFramingTest, VerifyChecksumRejectsCorruptedChecksumByte) {
    auto frame = build_frame(0x02, nullptr, 0);

    // Corrupt the checksum byte itself.
    frame[kChksOffset] ^= 0x01;
    EXPECT_FALSE(verify_checksum(frame));
}

TEST(SerialFramingTest, ComputeChecksumIsDeterministic) {
    auto frame1 = build_frame(0x10, nullptr, 0);
    auto frame2 = build_frame(0x10, nullptr, 0);

    EXPECT_EQ(compute_checksum(frame1), compute_checksum(frame2));
    EXPECT_EQ(frame1[kChksOffset], frame2[kChksOffset]);
}

TEST(SerialFramingTest, DifferentPayloadsProduceDifferentChecksums) {
    const uint8_t data_a[] = {0x00, 0x00, 0x00, 0x00};
    const uint8_t data_b[] = {0xFF, 0xFF, 0xFF, 0xFF};

    auto frame_a = build_frame(0x01, data_a, sizeof(data_a));
    auto frame_b = build_frame(0x01, data_b, sizeof(data_b));

    // Checksums should differ (extremely unlikely to collide).
    EXPECT_NE(frame_a[kChksOffset], frame_b[kChksOffset]);
}

TEST(SerialFramingTest, AllDataBytesUpToChecksumAreIncluded) {
    // Verify that changing any byte in [0..36] changes the checksum.
    auto base = build_frame(0x01, nullptr, 0);
    const uint8_t base_cks = base[kChksOffset];

    int changes_detected = 0;
    for (size_t i = 0; i < kChksOffset; ++i) {
        auto modified = base;
        modified[i] ^= 0x01;
        if (compute_checksum(modified) != base_cks) {
            ++changes_detected;
        }
    }
    EXPECT_EQ(changes_detected, static_cast<int>(kChksOffset))
        << "Every byte [0..36] must affect the checksum";
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
