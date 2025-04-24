#include <gtest/gtest.h>
#include <motors_roboteq_canopen/Driver.hpp>

using namespace motors_roboteq_canopen;

struct DriverTest : public ::testing::Test {
    DriverTest()
    {
    }
};

TEST_F(DriverTest, it_parses_managed_digital_outputs)
{
    std::vector<uint8_t> managed_outputs = {1, 3};
    std::uint16_t raw_reading = 0b0100;
    auto reading = Driver::parseDigitalOutput(raw_reading, managed_outputs);

    ASSERT_EQ(0, reading[0].data);
    ASSERT_EQ(1, reading[1].data);
}

TEST_F(DriverTest, it_sets_digital_output_reading_time)
{
    auto now = base::Time::now();

    std::vector<uint8_t> managed_outputs{1, 4, 5};
    auto reading = Driver::parseDigitalOutput(0, managed_outputs);

    for (auto& r : reading) {
        ASSERT_GE(r.time, now);
    }
}
