#ifndef MOTORS_ROBOTEQ_CANOPEN_TEST_HELPERS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_TEST_HELPERS_HPP

#include <vector>
#include <gtest/gtest.h>
#include <canbus/Message.hpp>
#include <canopen_master/SDO.hpp>
#include <canopen_master/PDOMapping.hpp>
#include <motors_roboteq_canopen/DS402Driver.hpp>

struct Helpers : testing::Test {
    void ASSERT_QUERIES_SDO_UPLOAD(std::vector<canbus::Message> const& queries,
                                   std::vector<int> const& objects);

    void ASSERT_QUERIES_SDO_DOWNLOAD(std::vector<canbus::Message> const& queries,
                                     std::vector<int> const& objects);

    void ASSERT_QUERIES_SDO(canopen_master::SDO_COMMANDS command,
                            std::vector<canbus::Message> const& queries,
                            std::vector<int> const& objects);

    void ASSERT_PDO_MAPPINGS(
        std::vector<canopen_master::PDOMapping> const& mappings,
        std::vector<std::vector<int>> const& expected);

    void ASSERT_PDO_MAPPING(
        canopen_master::PDOMapping const& mappings,
        std::vector<int> const& expected
    );

    void ASSERT_PDO_MAPPING_MESSAGES(
        std::vector<canbus::Message> const& messages,
        int pdoIndex, bool transmit,
        std::vector<std::vector<int>> const& mappings);

    template<typename T>
    canbus::Message make_sdo_ack(
        motors_roboteq_canopen::DS402Driver& driver, int node_id, int channel_id
    ) {
        canbus::Message msg;
        msg.can_id = node_id | canopen_master::FUNCTION_SDO_TRANSMIT;
        msg.data[0] = (
            canopen_master::SDO_INITIATE_DOMAIN_DOWNLOAD_REPLY << 5
        ) | 2;

        auto offsets = driver.getChannel(channel_id).getObjectOffsets<T>();
        uint16_t object_id = T::OBJECT_ID + offsets.first;
        msg.data[1] = (object_id & 0xFF);
        msg.data[2] = (object_id >> 8) & 0xFF;
        msg.data[3] = T::OBJECT_SUB_ID + offsets.second;
        return msg;
    }

    template<typename T>
    void ASSERT_JOINT_STATE_UPDATE(
        motors_roboteq_canopen::DS402Driver& driver, int node_id,
        bool channel0, bool channel1, bool channel2
    ) {
        bool channels_value[3] = { channel0, channel1, channel2 };
        for (int i = 0; i < 3; ++i) {
            driver.process(make_sdo_ack<T>(driver, node_id, i));
            bool hasUpdate = driver.getChannel(i).hasJointStateUpdate();
            if (channels_value[i] != hasUpdate) {
                FAIL() << "channel " << i << " was expected to have hasJointStateUpdate() == "
                       << channels_value[i] << " but it is " << hasUpdate << " after receiving "
                       << typeid(T).name();
            }
        }
    }
};

#endif
