#include <iostream>

#include "gtest/gtest.h"

int16_t decodeRPM(const uint8_t &lowbyte, const uint8_t &highbyte) {
    uint8_t lower = lowbyte;
    uint8_t higher = highbyte;
    int16_t rpm_motor = 0;
    rpm_motor = static_cast<int16_t>((highbyte << 8) | lowbyte);
    rpm_motor = abs(rpm_motor) <= 5 ? 0 : rpm_motor;
    return rpm_motor;
}

TEST(TestCanReceive, decodeRPM) {
    const uint8_t lowbyte = 0x34;
    const uint8_t highbyte = 0x12;
    EXPECT_TRUE(decodeRPM(lowbyte, highbyte) > 0);
    EXPECT_NEAR(decodeRPM(lowbyte, highbyte), 4660, 10);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}