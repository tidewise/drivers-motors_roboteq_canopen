#include <gtest/gtest.h>
#include "Helpers.hpp"
#include <motors_roboteq_canopen/DS402Driver.hpp>
#include <motors_roboteq_canopen/DS402Channel.hpp>

using namespace std;
using namespace base;
using namespace motors_roboteq_canopen;

typedef canopen_master::StateMachine StateMachine;
typedef canopen_master::StateMachine::Update Update;

struct ChannelTestBase : public Helpers {
    static const int NODE_ID = 2;
    static const int CHANNEL_COUNT = 3;
    static const int CHANNEL_ID = 1;

    canopen_master::StateMachine can_open;
    DS402Driver driver;
    DS402Channel& channel;

    ChannelTestBase()
        : can_open(NODE_ID)
        , driver(can_open, CHANNEL_COUNT)
        , channel(driver.getChannel(CHANNEL_ID)) {

        Factors factors;
        factors.speed_min = -1;
        factors.speed_max = 2.5;
        factors.position_min = -3;
        factors.position_max = 4;
        factors.torque_constant = 0.3;
        channel.setFactors(factors);
    }

    template<typename T>
    void ASSERT_JOINT_STATE_UPDATE(bool state) {
        Update update;
        update.mode = StateMachine::PROCESSED_SDO_INITIATE_DOWNLOAD;
        auto offsets = driver.getChannel(CHANNEL_ID).getObjectOffsets<T>();
        update.addUpdate<T>(offsets.first, offsets.second);
        channel.updateJointStateTracking(update);
        return Helpers::ASSERT_JOINT_STATE_UPDATE<T>(driver, NODE_ID, true, state, true);
    }
};

TEST_F(ChannelTestBase, it_sends_a_ds402_transition_request) {
    vector<canbus::Message> messages = channel.sendDS402Transition(
        ControlWord::ENABLE_OPERATION, true
    );
    ASSERT_QUERIES_SDO_DOWNLOAD(
        messages,
        { 0x6840, 0 }
    );

    ASSERT_EQ(messages[0].data[4] & 0xF, 0xF);
}

TEST_F(ChannelTestBase, it_returns_the_ds402_status) {
    can_open.set<uint16_t>(0x6841, 0, 1 << 11 | 1 << 7 | 0x27);
    StatusWord status = channel.getDS402Status();
    ASSERT_TRUE(status.warning);
    ASSERT_TRUE(status.internal_limit_active);
    ASSERT_EQ(StatusWord::OPERATION_ENABLED, status.state);
}

TEST_F(ChannelTestBase, it_queries_the_ds402_status) {
    auto queries = channel.queryDS402Status();
    ASSERT_QUERIES_SDO_UPLOAD(
        queries,
        { 0x6841, 0 }
    );
}

TEST_F(ChannelTestBase, it_reports_having_a_joint_state_update) {
    ASSERT_TRUE(channel.hasJointStateUpdate());
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(true);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(true);
    ASSERT_JOINT_STATE_UPDATE<Position>(true);
    channel.resetJointStateTracking();
    ASSERT_TRUE(channel.hasJointStateUpdate());
}

TEST_F(ChannelTestBase, it_returns_a_unknown_joint_state) {
    auto state = channel.getJointState();
    ASSERT_TRUE(base::isUnknown(state.position));
    ASSERT_TRUE(base::isUnknown(state.speed));
    ASSERT_TRUE(base::isUnknown(state.effort));
    ASSERT_TRUE(base::isUnknown(state.raw));
}

struct DirectVelocityModes : public ChannelTestBase,
                             public testing::WithParamInterface<DS402OperationModes> {
    DirectVelocityModes() {
        channel.setOperationMode(GetParam());
    }
};
struct DirectVelocityModesNotAnalog : public DirectVelocityModes {
};
struct DirectVelocityModesAnalog : public DirectVelocityModes {
};

TEST_P(DirectVelocityModes, it_queries_the_necessary_fields) {
    auto queries = channel.queryJointState();
    ASSERT_QUERIES_SDO_UPLOAD(
        queries,
        { 0x6844, 0,
          0x2100, 2,
          0x2102, 2 }
    );
}

TEST_P(DirectVelocityModes, it_creates_a_TPDO_mappings) {
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x6844, 0, 2 } });
}

TEST_P(DirectVelocityModes, it_sets_the_necessary_fields) {
    channel.setJointCommand(JointState::Speed(-0.5));
    auto queries = channel.queryJointCommandDownload();
    ASSERT_QUERIES_SDO_DOWNLOAD(
        queries,
        { 0x6842, 0 }
    );
}

TEST_P(DirectVelocityModes, it_creates_a_RPDO_mappings) {
    auto queries = channel.getJointCommandRPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x6842, 0, 2 } }
    );
}

TEST_P(DirectVelocityModes, it_throws_if_the_speed_field_is_not_set) {
    ASSERT_THROW(channel.setJointCommand(JointState::Position(-0.5)),
                 InvalidJointCommand);
}

TEST_P(DirectVelocityModes, it_handle_multiple_updates_at_the_same_time) {
    driver.process(make_sdo_ack<MotorAmps>(driver, NODE_ID, CHANNEL_ID));
    driver.process(make_sdo_ack<AppliedPowerLevel>(driver, NODE_ID, CHANNEL_ID));
    driver.process(make_sdo_ack<ActualVelocity>(driver, NODE_ID, CHANNEL_ID));
    ASSERT_TRUE(channel.hasJointStateUpdate());
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

TEST_P(DirectVelocityModes, it_tracks_joint_state_updates) {
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(false);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false);
    ASSERT_JOINT_STATE_UPDATE<ActualVelocity>(true);
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

TEST_P(DirectVelocityModes, it_enables_all_ramps) {
    vector<canbus::Message> messages = channel.sendDS402Transition(
        ControlWord::ENABLE_OPERATION, true
    );
    uint8_t lsb = messages[0].data[4];
    ASSERT_EQ((lsb >> 4) & 0x7, 0x7);
}

TEST_P(DirectVelocityModesNotAnalog, it_writes_the_target_velocity) {
    channel.setJointCommand(JointState::Speed(-0.5));
    int16_t v = can_open.get<int16_t>(0x6842, 0);
    ASSERT_EQ(v, -5);
}

TEST_P(DirectVelocityModesNotAnalog, it_reports_joint_effort_pwm_and_speed) {
    can_open.set<int16_t>(0x6844, 0, 5);
    can_open.set<int16_t>(0x2100, 2, 12);
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_NEAR(0.5236, state.speed, 1e-3);
    ASSERT_FLOAT_EQ(1.2 * 0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

TEST_P(DirectVelocityModesAnalog, it_writes_the_target_velocity) {
    channel.setJointCommand(JointState::Speed(-0.5));
    int16_t v = can_open.get<int16_t>(0x6842, 0);
    ASSERT_EQ(v, -525);
}

TEST_P(DirectVelocityModesAnalog, it_reports_joint_effort_pwm_and_speed) {
    can_open.set<int16_t>(0x6844, 0, 5);
    can_open.set<int16_t>(0x2100, 2, 12);
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_NEAR(0.13, state.speed, 1e-3);
    ASSERT_FLOAT_EQ(1.2 * 0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

INSTANTIATE_TEST_CASE_P(
    ChannelTestDirectVelocityModesNotAnalog,
    DirectVelocityModes,
    testing::Values(DS402_OPERATION_MODE_VELOCITY,
                    DS402_OPERATION_MODE_VELOCITY_POSITION)
);

INSTANTIATE_TEST_CASE_P(
    ChannelTestDirectVelocityModesAnalog,
    DirectVelocityModes,
    testing::Values(DS402_OPERATION_MODE_ANALOG_VELOCITY)
);

struct ProfileVelocityModes : public testing::WithParamInterface<DS402OperationModes>,
                              public ChannelTestBase {
    JointState cmd;

    ProfileVelocityModes() {
        channel.setOperationMode(GetParam());

        cmd.speed = 0.5;
        cmd.acceleration = 0.3;
        cmd.effort = 0.42;
    }
};

TEST_P(ProfileVelocityModes, it_queries_the_necessary_fields) {
    auto queries = channel.queryJointState();
    ASSERT_QUERIES_SDO_UPLOAD(queries,
        { 0x686C, 0,
          0x2100, 2,
          0x2102, 2 }
    );
}

TEST_P(ProfileVelocityModes, it_creates_a_TPDO_mappings) {
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x686C, 0, 4 } });
}

TEST_P(ProfileVelocityModes, it_sets_the_necessary_fields) {
    channel.setJointCommand(cmd);
    auto queries = channel.queryJointCommandDownload();
    ASSERT_QUERIES_SDO_DOWNLOAD(
        queries,
        { 0x68ff, 0,
          0x6871, 0,
          0x6883, 0,
          0x6884, 0 }
    );
}

TEST_P(ProfileVelocityModes, it_creates_a_RPDO_mappings) {
    auto queries = channel.getJointCommandRPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x6871, 0, 2,
            0x68ff, 0, 4 },
          { 0x6883, 0, 4,
            0x6884, 0, 4 } }
    );
}

TEST_P(ProfileVelocityModes, it_writes_the_target_velocity_and_desired_accelerations_and_efforts) {
    channel.setJointCommand(cmd);
    int32_t speed = can_open.get<int32_t>(0x68ff, 0);
    int16_t torque = can_open.get<int16_t>(0x6871, 0);
    int16_t acceleration = can_open.get<uint32_t>(0x6883, 0);
    int16_t deceleration = can_open.get<uint32_t>(0x6884, 0);
    ASSERT_EQ(speed, 4);
    ASSERT_EQ(torque, 12);
    ASSERT_EQ(acceleration, 28);
    ASSERT_EQ(deceleration, 28);
}

TEST_P(ProfileVelocityModes, it_throws_if_the_speed_field_is_not_set) {
    cmd.speed = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileVelocityModes, it_throws_if_the_acceleration_field_is_not_set) {
    cmd.acceleration = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileVelocityModes, it_throws_if_the_effort_field_is_not_set) {
    cmd.effort = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileVelocityModes, it_reports_joint_effort_pwm_and_speed) {
    can_open.set<int32_t>(0x686C, 0, 5);
    can_open.set<int16_t>(0x2100, 2, 12); // A
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_NEAR(0.5236, state.speed, 1e-3);
    ASSERT_FLOAT_EQ(1.2 / 0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

TEST_P(ProfileVelocityModes, it_tracks_joint_state_updates) {
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(false);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false);
    ASSERT_JOINT_STATE_UPDATE<ActualProfileVelocity>(true);
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

INSTANTIATE_TEST_CASE_P(
    ChannelTestProfileVelocityModes,
    ProfileVelocityModes,
    testing::Values(DS402_OPERATION_MODE_VELOCITY_POSITION_PROFILE,
                    DS402_OPERATION_MODE_VELOCITY_PROFILE)
);

struct ProfileRelativePositionModes : public testing::WithParamInterface<DS402OperationModes>,
                                      public ChannelTestBase {
    JointState cmd;

    ProfileRelativePositionModes() {
        channel.setOperationMode(GetParam());

        cmd.position = 0.3;
        cmd.speed = 0.5;
        cmd.acceleration = 0.3;
    }
};

TEST_P(ProfileRelativePositionModes, it_queries_the_necessary_fields) {
    auto queries = channel.queryJointState();
    ASSERT_QUERIES_SDO_UPLOAD(queries,
        { 0x6864, 0,
          0x2100, 2,
          0x2102, 2 }
    );
}

TEST_P(ProfileRelativePositionModes, it_creates_a_TPDO_mappings) {
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x6864, 0, 4 } });
}

TEST_P(ProfileRelativePositionModes, it_sets_the_necessary_fields) {
    channel.setJointCommand(cmd);
    auto queries = channel.queryJointCommandDownload();
    ASSERT_QUERIES_SDO_DOWNLOAD(
        queries,
        { 0x687a, 0,
          0x6881, 0,
          0x6883, 0,
          0x6884, 0 }
    );
}

TEST_P(ProfileRelativePositionModes, it_creates_a_RPDO_mappings) {
    auto queries = channel.getJointCommandRPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x687a, 0, 4,
            0x6881, 0, 4 },
          { 0x6883, 0, 4,
            0x6884, 0, 4 } }
    );
}

TEST_P(ProfileRelativePositionModes, it_writes_the_target_position_and_desired_velocity_and_accelerations) {
    channel.setJointCommand(cmd);
    int32_t position = can_open.get<int32_t>(0x687a, 0);
    int32_t rpm = can_open.get<int32_t>(0x6881, 0);
    int16_t acceleration = can_open.get<uint32_t>(0x6883, 0);
    int16_t deceleration = can_open.get<uint32_t>(0x6884, 0);
    ASSERT_EQ(position, -57);
    ASSERT_EQ(rpm, 4);
    ASSERT_EQ(acceleration, 28);
    ASSERT_EQ(deceleration, 28);
}

TEST_P(ProfileRelativePositionModes, it_throws_if_the_speed_field_is_not_set) {
    cmd.speed = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileRelativePositionModes, it_throws_if_the_acceleration_field_is_not_set) {
    cmd.acceleration = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileRelativePositionModes, it_throws_if_the_position_field_is_not_set) {
    cmd.position = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileRelativePositionModes, it_reports_joint_effort_pwm_and_position) {
    can_open.set<int32_t>(0x6864, 0, 120);
    can_open.set<int16_t>(0x2100, 2, 12);
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_NEAR(0.92, state.position, 1e-4);
    ASSERT_FLOAT_EQ(1.2 / 0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

TEST_P(ProfileRelativePositionModes, it_controls_next_setpoint_immediately) {
    vector<canbus::Message> messages = channel.sendDS402Transition(
        ControlWord::ENABLE_OPERATION, true
    );
    uint8_t lsb = messages[0].data[4];
    ASSERT_EQ((lsb >> 5) & 0x1, 0x1);
}

TEST_P(ProfileRelativePositionModes, it_uses_absolute_positions) {
    vector<canbus::Message> messages = channel.sendDS402Transition(
        ControlWord::ENABLE_OPERATION, true
    );
    uint8_t lsb = messages[0].data[4];
    ASSERT_EQ((lsb >> 6) & 0x1, 0x0);
}

TEST_P(ProfileRelativePositionModes, it_tracks_joint_state_updates) {
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(false);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false);
    ASSERT_JOINT_STATE_UPDATE<Position>(true);
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

INSTANTIATE_TEST_CASE_P(
    ChannelTestProfileRelativePositionModes,
    ProfileRelativePositionModes,
    testing::Values(DS402_OPERATION_MODE_RELATIVE_POSITION_PROFILE)
);

struct DirectRelativePositionModes : public testing::WithParamInterface<DS402OperationModes>,
                                     public ChannelTestBase {
    JointState cmd;

    DirectRelativePositionModes() {
        channel.setOperationMode(GetParam());

        cmd.position = 0.3;
    }
};

TEST_P(DirectRelativePositionModes, it_queries_the_necessary_fields) {
    auto queries = channel.queryJointState();
    ASSERT_QUERIES_SDO_UPLOAD(queries,
        { 0x6864, 0,
          0x2100, 2,
          0x2102, 2 }
    );
}

TEST_P(DirectRelativePositionModes, it_creates_a_TPDO_mappings) {
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x6864, 0, 4 } });
}

TEST_P(DirectRelativePositionModes, it_sets_the_necessary_fields) {
    channel.setJointCommand(cmd);
    auto queries = channel.queryJointCommandDownload();
    ASSERT_QUERIES_SDO_DOWNLOAD(
        queries,
        { 0x687a, 0 }
    );
}

TEST_P(DirectRelativePositionModes, it_creates_a_RPDO_mappings) {
    auto queries = channel.getJointCommandRPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x687a, 0, 4 } }
    );
}

TEST_P(DirectRelativePositionModes, it_writes_the_target_position) {
    channel.setJointCommand(cmd);
    int32_t position = can_open.get<int32_t>(0x687a, 0);
    ASSERT_EQ(position, -57);
}

TEST_P(DirectRelativePositionModes, it_throws_if_the_position_field_is_not_set) {
    cmd.position = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(DirectRelativePositionModes, it_reports_joint_effort_pwm_and_position) {
    can_open.set<int32_t>(0x6864, 0, 120);
    can_open.set<int16_t>(0x2100, 2, 12);
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_NEAR(0.92, state.position, 1e-4);
    ASSERT_FLOAT_EQ(1.2 / 0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

TEST_P(DirectRelativePositionModes, it_tracks_joint_state_updates) {
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(false);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false);
    ASSERT_JOINT_STATE_UPDATE<Position>(true);
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

INSTANTIATE_TEST_CASE_P(
    ChannelTestDirectRelativePositionModes,
    DirectRelativePositionModes,
    testing::Values(DS402_OPERATION_MODE_RELATIVE_POSITION)
);

struct ProfileTorqueModes : public testing::WithParamInterface<DS402OperationModes>,
                            public ChannelTestBase {
    JointState cmd;

    ProfileTorqueModes() {
        channel.setOperationMode(GetParam());
        cmd.effort = 0.3;
        cmd.raw = 0.5;
    }
};

TEST_P(ProfileTorqueModes, it_queries_the_necessary_fields) {
    auto queries = channel.queryJointState();
    ASSERT_QUERIES_SDO_UPLOAD(queries,
        { 0x2100, 2,
          0x2102, 2 }
    );
}

TEST_P(ProfileTorqueModes, it_creates_a_TPDO_mappings) {
    auto queries = channel.getJointStateTPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x2100, 2, 2,
            0x2102, 2, 2 } });
}

TEST_P(ProfileTorqueModes, it_sets_the_necessary_fields) {
    channel.setJointCommand(cmd);
    auto queries = channel.queryJointCommandDownload();
    ASSERT_QUERIES_SDO_DOWNLOAD(
        queries,
        { 0x6871, 0,
          0x6887, 0 }
    );
}

TEST_P(ProfileTorqueModes, it_creates_a_RPDO_mappings) {
    auto queries = channel.getJointCommandRPDOMapping();
    ASSERT_PDO_MAPPINGS(
        queries,
        { { 0x6871, 0, 2,
            0x6887, 0, 4 } }
    );
}

TEST_P(ProfileTorqueModes, it_writes_the_target_torque_and_torque_slope) {
    channel.setJointCommand(cmd);
    int16_t torque = can_open.get<int16_t>(0x6871, 0);
    uint32_t torque_slope = can_open.get<uint32_t>(0x6887, 0);
    ASSERT_EQ(torque, 9);
    ASSERT_EQ(torque_slope, 1500);
}

TEST_P(ProfileTorqueModes, it_throws_if_the_torque_field_is_not_set) {
    cmd.effort = base::unknown<float>();
    ASSERT_THROW(channel.setJointCommand(cmd), InvalidJointCommand);
}

TEST_P(ProfileTorqueModes, it_reports_joint_effort_and_pwm) {
    can_open.set<int16_t>(0x2100, 2, 120);
    can_open.set<int16_t>(0x2102, 2, 400);
    auto state = channel.getJointState();
    ASSERT_FLOAT_EQ(12/0.3, state.effort);
    ASSERT_FLOAT_EQ(0.4, state.raw);
}

TEST_P(ProfileTorqueModes, it_tracks_joint_state_updates) {
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false);
    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(true);
    channel.resetJointStateTracking();
    ASSERT_FALSE(channel.hasJointStateUpdate());
}

INSTANTIATE_TEST_CASE_P(
    ChannelTestProfileTorqueModes,
    ProfileTorqueModes,
    testing::Values(DS402_OPERATION_MODE_TORQUE_PROFILE)
);
