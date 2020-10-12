#include <gtest/gtest.h>
#include <motors_roboteq_canopen/Driver.hpp>
#include <motors_roboteq_canopen/Channel.hpp>

using namespace motors_roboteq_canopen;
using canopen_master::PDOMapping;

struct ChannelTest : public ::testing::Test {
    canopen_master::StateMachine canopen;
    Driver driver;
    Channel& channel;

    ChannelTest()
        : canopen(0x1)
        , driver(canopen, 2) // use not-the-first channel to make sure we apply the subid properly
        , channel(driver.getChannel(1)) {}
};

TEST_F(ChannelTest, it_does_not_query_anything_in_TPDO_while_ignored) {
    channel.setControlMode(CONTROL_IGNORED);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_TRUE(queries.empty());
}

TEST_F(ChannelTest, it_queries_amp_pwm_and_status_in_TPDO_while_open_loop) {
    channel.setControlMode(CONTROL_OPEN_LOOP);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(1, queries.size());
    auto& mapping = queries[0];
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));
}

TEST_F(ChannelTest, it_queries_amp_pwm_feedback_and_status_in_TPDO_while_closed_loop) {
    channel.setControlMode(CONTROL_TORQUE);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(1, queries.size());
    auto& mapping = queries[0];
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2110, 2, 2 }));
    ASSERT_EQ(mapping.mappings[3],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));
}
