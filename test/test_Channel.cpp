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
    ASSERT_EQ(3, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));
}

TEST_F(ChannelTest, it_allows_to_explicitly_require_the_encoder_as_position_feedback_in_open_loop) {
    channel.setControlMode(CONTROL_OPEN_LOOP);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_ENCODER);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(2, queries.size());

    auto mapping = queries[0];
    ASSERT_EQ(3, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));

    mapping = queries[1];
    ASSERT_EQ(1, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2104, 2, 4 }));
}

TEST_F(ChannelTest, it_queries_amp_pwm_feedback_and_status_in_TPDO_while_closed_loop) {
    channel.setControlMode(CONTROL_POSITION);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(2, queries.size());

    auto mapping = queries[0];
    ASSERT_EQ(3, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));

    mapping = queries[1];
    ASSERT_EQ(1, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2110, 2, 4 }));
}

TEST_F(ChannelTest, it_allows_selecting_the_encoder_for_feedback_in_closed_loop) {
    channel.setControlMode(CONTROL_POSITION);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_ENCODER);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(2, queries.size());

    auto mapping = queries[0];
    ASSERT_EQ(3, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));

    mapping = queries[1];
    ASSERT_EQ(1, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2104, 2, 4 }));
}

TEST_F(ChannelTest, it_allows_disabling_position_feedback_in_closed_loop) {
    channel.setControlMode(CONTROL_POSITION);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_NONE);
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_EQ(1, queries.size());

    auto mapping = queries[0];
    ASSERT_EQ(3, mapping.mappings.size());
    ASSERT_EQ(mapping.mappings[0],
              (PDOMapping::MappedObject{ 0x2100, 2, 2 }));
    ASSERT_EQ(mapping.mappings[1],
              (PDOMapping::MappedObject{ 0x2102, 2, 2 }));
    ASSERT_EQ(mapping.mappings[2],
              (PDOMapping::MappedObject{ 0x2122, 2, 2 }));
}

TEST_F(ChannelTest, it_reports_a_full_joint_state_with_no_position_feedback) {
    channel.setControlMode(CONTROL_OPEN_LOOP);
    canopen_master::StateMachine::Update update;

    update.addUpdate(0x2100, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2102, 2);
    channel.updateJointStateTracking(update);
    ASSERT_TRUE(channel.hasJointStateUpdate());

    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

TEST_F(ChannelTest, it_fills_the_joint_state_with_no_position_feedback) {
    channel.setControlMode(CONTROL_OPEN_LOOP);

    Factors factors;
    factors.torque_constant = 2.0;
    channel.setFactors(factors);
    driver.set<MotorAmps>(10, 0, 1);
    driver.set<AppliedPowerLevel>(20, 0, 1);

    auto joint = channel.getJointState();
    ASSERT_FLOAT_EQ(0.5, joint.effort);
    ASSERT_FLOAT_EQ(0.02, joint.raw);
    ASSERT_TRUE(base::isUnknown(joint.position));
    ASSERT_TRUE(base::isUnknown(joint.speed));
}

TEST_F(ChannelTest, it_reports_a_full_joint_state_with_feedback_as_position) {
    channel.setControlMode(CONTROL_POSITION);
    canopen_master::StateMachine::Update update;

    update.addUpdate(0x2100, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2102, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2110, 2);
    channel.updateJointStateTracking(update);
    ASSERT_TRUE(channel.hasJointStateUpdate());
}

TEST_F(ChannelTest, it_fills_the_joint_state_with_feedback_as_position) {
    channel.setControlMode(CONTROL_POSITION);

    Factors factors;
    factors.torque_constant = 2.0;
    channel.setFactors(factors);
    driver.set<MotorAmps>(10, 0, 1);
    driver.set<AppliedPowerLevel>(20, 0, 1);
    driver.set<Feedback>(42, 0, 1);

    auto joint = channel.getJointState();
    ASSERT_FLOAT_EQ(0.5, joint.effort);
    ASSERT_FLOAT_EQ(0.02, joint.raw);
    ASSERT_NEAR(0.042, joint.position, 1e-4);
    ASSERT_TRUE(base::isUnknown(joint.speed));
}

TEST_F(ChannelTest, it_allows_disabling_position_feedback_in_position_control_mode) {
    channel.setControlMode(CONTROL_POSITION);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_NONE);
    canopen_master::StateMachine::Update update;

    update.addUpdate(0x2100, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2102, 2);
    channel.updateJointStateTracking(update);
    ASSERT_TRUE(channel.hasJointStateUpdate());
}

TEST_F(ChannelTest, it_fills_the_joint_state_without_position_if_explicitly_disabled) {
    channel.setControlMode(CONTROL_POSITION);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_NONE);

    Factors factors;
    factors.torque_constant = 2.0;
    channel.setFactors(factors);
    driver.set<MotorAmps>(10, 0, 1);
    driver.set<AppliedPowerLevel>(20, 0, 1);

    auto joint = channel.getJointState();
    ASSERT_FLOAT_EQ(0.5, joint.effort);
    ASSERT_FLOAT_EQ(0.02, joint.raw);
    ASSERT_TRUE(base::isUnknown(joint.position));
    ASSERT_TRUE(base::isUnknown(joint.speed));
}

TEST_F(ChannelTest, it_reports_a_full_joint_state_with_encoder_as_position) {
    channel.setControlMode(CONTROL_OPEN_LOOP);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_ENCODER);
    canopen_master::StateMachine::Update update;

    update.addUpdate(0x2100, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2102, 2);
    channel.updateJointStateTracking(update);
    ASSERT_FALSE(channel.hasJointStateUpdate());

    update.addUpdate(0x2104, 2);
    channel.updateJointStateTracking(update);
    ASSERT_TRUE(channel.hasJointStateUpdate());
}

TEST_F(ChannelTest, it_fills_the_joint_state_with_encoder_as_position) {
    channel.setControlMode(CONTROL_POSITION);
    channel.setJointStatePositionSource(JOINT_STATE_POSITION_SOURCE_ENCODER);

    Factors factors;
    factors.torque_constant = 2.0;
    factors.encoder_position_factor = 0.1;
    channel.setFactors(factors);
    driver.set<MotorAmps>(10, 0, 1);
    driver.set<AppliedPowerLevel>(20, 0, 1);
    driver.set<EncoderCounter>(42, 0, 1);

    auto joint = channel.getJointState();
    ASSERT_FLOAT_EQ(0.5, joint.effort);
    ASSERT_FLOAT_EQ(0.02, joint.raw);
    ASSERT_NEAR(4.2, joint.position, 1e-4);
    ASSERT_TRUE(base::isUnknown(joint.speed));
}

TEST_F(ChannelTest, it_does_not_report_a_full_joint_state_after_reset) {
    channel.setControlMode(CONTROL_OPEN_LOOP);

    canopen_master::StateMachine::Update update;
    update.addUpdate(0x2100, 2);
    update.addUpdate(0x2102, 2);
    channel.updateJointStateTracking(update);
    ASSERT_TRUE(channel.hasJointStateUpdate());

    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}