#include <gtest/gtest.h>
#include <motors_roboteq_canopen/Factors.hpp>

using namespace motors_roboteq_canopen;

struct FactorsTest : public ::testing::Test {
    Factors factors;
    FactorsTest() {
        factors.speed_zero = 100;
        factors.speed_min = -10;
        factors.speed_max = 42;
        factors.position_zero = 250;
        factors.position_min = -100;
        factors.position_max = 84;
    }
};

TEST_F(FactorsTest, it_returns_SI_zero_at_speed_zero) {
    ASSERT_FLOAT_EQ(0, factors.speedToSI(100));
}

TEST_F(FactorsTest, it_returns_SI_speed_min_at_relative_speed_minus_1000) {
    ASSERT_FLOAT_EQ(-10, factors.speedToSI(-1000));
}

TEST_F(FactorsTest, it_returns_SI_speed_max_at_relative_speed_1000) {
    ASSERT_FLOAT_EQ(42, factors.speedToSI(1000));
}

TEST_F(FactorsTest, it_returns_speed_zero_at_SI_zero) {
    ASSERT_FLOAT_EQ(100, factors.speedFromSI(0));
}

TEST_F(FactorsTest, it_returns_speed_minus_1000_at_SI_speed_min) {
    ASSERT_FLOAT_EQ(-1000, factors.speedFromSI(-10));
}

TEST_F(FactorsTest, it_returns_speed_1000_at_SI_speed_max) {
    ASSERT_FLOAT_EQ(1000, factors.speedFromSI(42));
}




TEST_F(FactorsTest, it_returns_SI_zero_at_position_zero) {
    ASSERT_FLOAT_EQ(0, factors.positionToSI(250));
}

TEST_F(FactorsTest, it_returns_SI_position_min_at_relative_position_minus_1000) {
    ASSERT_FLOAT_EQ(-100, factors.positionToSI(-1000));
}

TEST_F(FactorsTest, it_returns_SI_position_max_at_relative_position_1000) {
    ASSERT_FLOAT_EQ(84, factors.positionToSI(1000));
}

TEST_F(FactorsTest, it_returns_position_zero_at_SI_zero) {
    ASSERT_FLOAT_EQ(250, factors.positionFromSI(0));
}

TEST_F(FactorsTest, it_returns_position_minus_1000_at_SI_position_min) {
    ASSERT_FLOAT_EQ(-1000, factors.positionFromSI(-100));
}

TEST_F(FactorsTest, it_returns_position_1000_at_SI_position_max) {
    ASSERT_FLOAT_EQ(1000, factors.positionFromSI(84));
}