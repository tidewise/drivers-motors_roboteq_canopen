#include <gtest/gtest.h>
#include "Helpers.hpp"
#include <motors_roboteq_canopen/Driver.hpp>

using namespace motors_roboteq_canopen;

struct DriverTest : public Helpers {
    Driver driver;
    canopen_master::StateMachine& can_open;

    DriverTest()
        : driver(2, 3)
        , can_open(driver.getStateMachine()) {
        Factors factors;
        factors.position_zero = 0.3;
        factors.position_min = -3;
        factors.position_max = 4;
        factors.torque_constant = 0.3;
        driver.getChannel(0).setFactors(factors);
        driver.getChannel(1).setFactors(factors);
        driver.getChannel(2).setFactors(factors);
    }
};

TEST_F(DriverTest, it_queries_controller_status) {
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
          0x210F, 3,
          0x210F, 4 }
    );
}

TEST_F(DriverTest, it_returns_controller_status) {
    can_open.set<uint16_t>(0x210D, 1, 20);
    can_open.set<uint16_t>(0x210D, 2, 25);
    can_open.set<uint16_t>(0x210D, 3, 500);
    can_open.set<uint16_t>(0x2111, 0, 0X1234);
    can_open.set<uint16_t>(0x2112, 0, 0xabcd);
    can_open.set<int8_t>(0x210F, 1, 50);
    can_open.set<int8_t>(0x210F, 2, 100);
    can_open.set<int8_t>(0x210F, 3, 120);
    can_open.set<int8_t>(0x210F, 4, -20);
    ControllerStatus status = driver.getControllerStatus();
    ASSERT_FLOAT_EQ(2, status.voltage_internal);
    ASSERT_FLOAT_EQ(2.5, status.voltage_battery);
    ASSERT_FLOAT_EQ(0.5, status.voltage_5v);
    ASSERT_EQ(0x1234, status.status_flags);
    ASSERT_EQ(0xabcd, status.fault_flags);
    ASSERT_FLOAT_EQ(50, status.temperature_mcu.getCelsius());
    ASSERT_FLOAT_EQ(100, status.temperature_sensors.at(0).getCelsius());
    ASSERT_FLOAT_EQ(120, status.temperature_sensors.at(1).getCelsius());
    ASSERT_FLOAT_EQ(-20, status.temperature_sensors.at(2).getCelsius());
}

TEST_F(DriverTest, it_sets_up_joint_state_TPDOs) {
    driver.getChannel(0).setOperationMode(OPERATION_MODE_TORQUE_PROFILE);
    driver.getChannel(1).setOperationMode(OPERATION_MODE_VELOCITY);
    driver.getChannel(2).setOperationMode(OPERATION_MODE_RELATIVE_POSITION);

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

TEST_F(DriverTest, it_sets_up_joint_command_RPDOs) {
    driver.getChannel(0).setOperationMode(OPERATION_MODE_TORQUE_PROFILE);
    driver.getChannel(1).setOperationMode(OPERATION_MODE_VELOCITY);
    driver.getChannel(2).setOperationMode(OPERATION_MODE_RELATIVE_POSITION);

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

TEST_F(DriverTest, it_sets_up_status_TPDOs) {
    std::vector<canbus::Message> messages;
    driver.setupStatusTPDOs(
        messages, 3, canopen_master::PDOCommunicationParameters::Async()
    );

    can_open.set<uint16_t>(0x210D, 1, 20);
    can_open.set<uint16_t>(0x210D, 2, 25);
    can_open.set<uint16_t>(0x210D, 3, 500);
    can_open.set<uint16_t>(0x2111, 0, 0x1234);
    can_open.set<uint16_t>(0x2112, 0, 0xabcd);
    can_open.set<int8_t>(0x210F, 1, 50);
    can_open.set<int8_t>(0x210F, 2, 100);
    can_open.set<int8_t>(0x210F, 3, 120);
    can_open.set<int8_t>(0x210F, 4, -20);
    ASSERT_PDO_MAPPING_MESSAGES(messages, false, 3,
        { { 0x2111, 0, 2,
            0x2112, 0, 2,
            0x210D, 1, 2,
            0x210D, 2, 2 },
          { 0x210D, 3, 2,
            0x210F, 1, 1,
            0x210F, 2, 1,
            0x210F, 3, 1,
            0x210F, 4, 1 } }
    );
}