#include <gtest/gtest.h>
#include "Helpers.hpp"
#include <motors_roboteq_canopen/DS402Driver.hpp>

using namespace motors_roboteq_canopen;

struct DS402DriverTest : public Helpers {
    static const int NODE_ID = 2;
    static const int CHANNEL_COUNT = 3;

    canopen_master::StateMachine can_open;
    DS402Driver driver;

    DS402DriverTest()
        : can_open(NODE_ID)
        , driver(can_open, CHANNEL_COUNT) {
        Factors factors;
        factors.position_zero = 0.3;
        factors.position_min = -3;
        factors.position_max = 4;
        factors.torque_constant = 0.3;
        driver.getChannel(0).setFactors(factors);
        driver.getChannel(1).setFactors(factors);
        driver.getChannel(2).setFactors(factors);
    }

    template<typename T>
    canbus::Message make_sdo_ack(int channel_id) {
        return Helpers::make_sdo_ack<T>(driver, NODE_ID, channel_id);
    }

    template<typename T>
    void ASSERT_JOINT_STATE_UPDATE(bool channel0, bool channel1, bool channel2) {
        return Helpers::ASSERT_JOINT_STATE_UPDATE<T>(
            driver, NODE_ID, channel0, channel1, channel2
        );
    }
};

TEST_F(DS402DriverTest, it_queries_controller_status) {
    auto queries = driver.queryControllerStatus();
    ASSERT_QUERIES_SDO_UPLOAD(
        queries,
        { 0x210D, 1,
          0x210D, 2,
          0x210D, 3,
          0x2111, 0,
          0x2112, 0,
          0x210F, 1,
          0x210F, 2,
          0x2122, 1,
          0x210F, 3,
          0x2122, 2,
          0x210F, 4,
          0x2122, 3 }
    );
}

TEST_F(DS402DriverTest, it_returns_controller_status) {
    can_open.set<uint16_t>(0x210D, 1, 20);
    can_open.set<uint16_t>(0x210D, 2, 25);
    can_open.set<uint16_t>(0x210D, 3, 500);
    can_open.set<uint16_t>(0x2111, 0, 0X1234);
    can_open.set<uint16_t>(0x2112, 0, 0xabcd);
    can_open.set<int8_t>(0x210F, 1, 50);
    can_open.set<int8_t>(0x210F, 2, 100);
    can_open.set<uint16_t>(0x2122, 1, 16);
    can_open.set<int8_t>(0x210F, 3, 120);
    can_open.set<uint16_t>(0x2122, 2, 8);
    can_open.set<int8_t>(0x210F, 4, -20);
    can_open.set<uint16_t>(0x2122, 3, 24);
    ControllerStatus status = driver.getControllerStatus();
    ASSERT_FLOAT_EQ(2, status.voltage_internal);
    ASSERT_FLOAT_EQ(2.5, status.voltage_battery);
    ASSERT_FLOAT_EQ(0.5, status.voltage_5v);
    ASSERT_EQ(0x1234, status.status_flags);
    ASSERT_EQ(0xabcd, status.fault_flags);
    ASSERT_FLOAT_EQ(50, status.temperature_mcu.getCelsius());
    ASSERT_FLOAT_EQ(100, status.temperature_sensors.at(0).getCelsius());
    ASSERT_EQ(16, status.channel_status_flags[0]);
    ASSERT_FLOAT_EQ(120, status.temperature_sensors.at(1).getCelsius());
    ASSERT_EQ(8, status.channel_status_flags[1]);
    ASSERT_FLOAT_EQ(-20, status.temperature_sensors.at(2).getCelsius());
    ASSERT_EQ(24, status.channel_status_flags[2]);
}

TEST_F(DS402DriverTest, it_sets_up_joint_state_TPDOs) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);

    std::vector<canbus::Message> messages;
    int nextPDO = driver.setupJointStateTPDOs(
        messages, 3, canopen_master::PDOCommunicationParameters::Async()
    );

    // Initial PDO plus size of expected PDO below
    ASSERT_EQ(nextPDO, 6);
    ASSERT_PDO_MAPPING_MESSAGES(messages, true, 3,
        { { 0x2102, 2, 2,
            0x6877, 0, 2 },
          { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x6844, 0, 2 },
          { 0x2100, 2, 2,
            0x2102, 2, 2,
            0x6864, 0, 4 } }
    );
}

TEST_F(DS402DriverTest, it_sets_up_joint_command_RPDOs) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);

    std::vector<canbus::Message> messages;
    int nextPDO = driver.setupJointCommandRPDOs(
        messages, 3, canopen_master::PDOCommunicationParameters::Async()
    );
    // Initial PDO plus size of expected PDO below
    ASSERT_EQ(nextPDO, 6);

    ASSERT_PDO_MAPPING_MESSAGES(messages, false, 3,
        { { 0x6871, 0, 2,
            0x6887, 0, 4 },
          { 0x6842, 0, 2 },
          { 0x687a, 0, 4 } }
    );
}

TEST_F(DS402DriverTest, it_sets_up_status_TPDOs) {
    std::vector<canbus::Message> messages;

    canopen_master::StateMachine can_open(NODE_ID);
    DS402Driver driver(can_open, 2);
    driver.setupStatusTPDOs(
        messages, 3, canopen_master::PDOCommunicationParameters::Async()
    );

    can_open.set<uint16_t>(0x210D, 1, 20);
    can_open.set<uint16_t>(0x210D, 2, 25);
    can_open.set<uint16_t>(0x210D, 3, 500);
    can_open.set<uint16_t>(0x2111, 0, 0x1234);
    can_open.set<uint16_t>(0x2112, 0, 0xabcd);
    can_open.set<int16_t>(0x210F, 1, 50);
    can_open.set<int16_t>(0x210F, 2, 100);
    can_open.set<int16_t>(0x210F, 3, 120);
    ASSERT_PDO_MAPPING_MESSAGES(messages, false, 3,
        { { 0x2111, 0, 2,
            0x2112, 0, 2,
            0x210D, 1, 2,
            0x210D, 2, 2 },
          { 0x210D, 3, 2,
            0x210F, 1, 1,
            0x210F, 2, 1,
            0x210F, 3, 1 } }
    );
}

TEST_F(DS402DriverTest, it_updates_the_channels_joint_state_tracking_on_process) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_VELOCITY_PROFILE);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);

    ASSERT_JOINT_STATE_UPDATE<MotorAmps>(false, false, false);
    ASSERT_JOINT_STATE_UPDATE<AppliedPowerLevel>(false, true, false);
    ASSERT_JOINT_STATE_UPDATE<ActualProfileVelocity>(true, true, false);
    ASSERT_JOINT_STATE_UPDATE<ActualVelocity>(true, true, false);
    ASSERT_JOINT_STATE_UPDATE<Position>(true, true, true);
}

base::samples::Joints getJointCommand() {
    base::samples::Joints cmd;
    cmd.elements.push_back(base::JointState::Speed(2));
    cmd.elements.push_back(base::JointState::Position(3));
    auto torque_cmd = base::JointState::Effort(1);
    torque_cmd.raw = 1;
    cmd.elements.push_back(torque_cmd);
    return cmd;
}

TEST_F(DS402DriverTest, it_sets_the_joint_commands_on_all_joints) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);

    base::samples::Joints cmd = getJointCommand();
    driver.setJointCommand(cmd);

    ASSERT_EQ(2, driver.getChannel(0).getJointCommand().speed);
    ASSERT_EQ(3, driver.getChannel(1).getJointCommand().position);
    ASSERT_EQ(1, driver.getChannel(2).getJointCommand().effort);
}

TEST_F(DS402DriverTest, it_raises_if_there_are_too_few_commands) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);

    base::samples::Joints cmd = getJointCommand();
    cmd.elements.pop_back();
    ASSERT_THROW(driver.setJointCommand(cmd), std::invalid_argument);
}

TEST_F(DS402DriverTest, it_raises_if_there_are_too_many_commands) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_TORQUE_PROFILE);

    base::samples::Joints cmd = getJointCommand();
    cmd.elements.push_back(base::JointState::Position(3));
    ASSERT_THROW(driver.setJointCommand(cmd), std::invalid_argument);
}

TEST_F(DS402DriverTest, it_skips_the_joints_that_are_ignored) {
    driver.getChannel(0).setOperationMode(DS402_OPERATION_MODE_VELOCITY);
    driver.getChannel(1).setOperationMode(DS402_OPERATION_MODE_NONE);
    driver.getChannel(2).setOperationMode(DS402_OPERATION_MODE_RELATIVE_POSITION);

    base::samples::Joints cmd;
    cmd.elements.push_back(base::JointState::Speed(1));
    cmd.elements.push_back(base::JointState::Position(3));
    driver.setJointCommand(cmd);

    ASSERT_EQ(1, driver.getChannel(0).getJointCommand().speed);
    ASSERT_EQ(3, driver.getChannel(2).getJointCommand().position);
}
