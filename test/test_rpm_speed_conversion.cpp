#include <cmath>
#include <cstdint>
#include <iostream>

#include "gtest/gtest.h"

float rpm_motor_to_wheel = 21.1;
float wheel_diameter = 0.6096;

float rpmToSpeed(const int16_t rpm) {
    float rpm_wheel = static_cast<float>(rpm) / rpm_motor_to_wheel;
    float vel_wheel = (rpm_wheel / 60.0) * (M_PI * wheel_diameter);
    vel_wheel = abs(vel_wheel) <= 0.01 ? 0 : vel_wheel;
    return vel_wheel;
}

TEST(TestCanReceive, rpmToSpeed1) {
    int16_t rpm = 100;
    EXPECT_TRUE(rpmToSpeed(rpm) > 0);
    EXPECT_NEAR(rpmToSpeed(rpm), 0.15, 0.01);
}

TEST(TestCanReceive, rpmToSpeed2) {
    int16_t rpm2 = 200;
    EXPECT_TRUE(rpmToSpeed(rpm2) > 0);
    EXPECT_NEAR(rpmToSpeed(rpm2), 0.30, 0.01);
}

TEST(TestCanReceive, rpmToSpeed3) {
    int16_t rpm3 = 300;
    EXPECT_TRUE(rpmToSpeed(rpm3) > 0);
    EXPECT_NEAR(rpmToSpeed(rpm3), 0.45, 0.01);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}