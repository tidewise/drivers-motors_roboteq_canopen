#include <gtest/gtest.h>
#include <motors_roboteq_canopen/Factors.hpp>

using namespace motors_roboteq_canopen;

struct FactorsTest : public ::testing::Test {
    Factors factors;
    FactorsTest() {
        factors.speed_min = -10;
        factors.speed_max = 42;
        factors.position_min = -100;
        factors.position_max = 84;
        factors.encoder_position_factor = 0.5;
    }
};

TEST_F(FactorsTest, it_returns_the_center_of_the_min_max_range_at_speed_0) {
    ASSERT_FLOAT_EQ(16, factors.relativeSpeedToSI(0));
}

TEST_F(FactorsTest, it_returns_SI_speed_min_at_relative_speed_minus_1000) {
    ASSERT_FLOAT_EQ(-10, factors.relativeSpeedToSI(-1000));
}

TEST_F(FactorsTest, it_returns_SI_speed_max_at_relative_speed_1000) {
    ASSERT_FLOAT_EQ(42, factors.relativeSpeedToSI(1000));
}

TEST_F(FactorsTest, it_returns_zero_at_the_center_of_the_min_max_range) {
    ASSERT_FLOAT_EQ(0, factors.relativeSpeedFromSI(16));
}

TEST_F(FactorsTest, it_returns_speed_minus_1000_at_SI_speed_min) {
    ASSERT_FLOAT_EQ(-1000, factors.relativeSpeedFromSI(-10));
}
TEST_F(FactorsTest, it_clamps_the_relative_speed_at_minus_1000) {
    ASSERT_FLOAT_EQ(-1000, factors.relativeSpeedFromSI(-20));
}

TEST_F(FactorsTest, it_returns_speed_1000_at_SI_speed_max) {
    ASSERT_FLOAT_EQ(1000, factors.relativeSpeedFromSI(42));
}

TEST_F(FactorsTest, it_clamps_the_relative_speed_at_plus_1000) {
    ASSERT_FLOAT_EQ(1000, factors.relativeSpeedFromSI(50));
}

TEST_F(FactorsTest, it_returns_the_SI_center_of_the_min_max_range_at_position_zero) {
    ASSERT_FLOAT_EQ(-8, factors.relativePositionToSI(0));
}

TEST_F(FactorsTest, it_returns_SI_position_min_at_relative_position_minus_1000) {
    ASSERT_FLOAT_EQ(-100, factors.relativePositionToSI(-1000));
}

TEST_F(FactorsTest, it_returns_SI_position_max_at_relative_position_1000) {
    ASSERT_FLOAT_EQ(84, factors.relativePositionToSI(1000));
}

TEST_F(FactorsTest, it_converts_encoder_ticks_to_position) {
    ASSERT_FLOAT_EQ(0.5, factors.encoderToSI(1));
}

TEST_F(FactorsTest, it_returns_position_zero_at_the_SI_center_of_the_min_max_range) {
    ASSERT_FLOAT_EQ(0, factors.relativePositionFromSI(-8));
}

TEST_F(FactorsTest, it_returns_position_minus_1000_at_SI_position_min) {
    ASSERT_FLOAT_EQ(-1000, factors.relativePositionFromSI(-100));
}

TEST_F(FactorsTest, it_clamps_the_relative_position_at_minus_1000) {
    ASSERT_FLOAT_EQ(-1000, factors.relativePositionFromSI(-200));
}

TEST_F(FactorsTest, it_returns_position_1000_at_SI_position_max) {
    ASSERT_FLOAT_EQ(1000, factors.relativePositionFromSI(84));
}

TEST_F(FactorsTest, it_clamps_the_relative_position_at_plus_1000) {
    ASSERT_FLOAT_EQ(1000, factors.relativePositionFromSI(85));
}
