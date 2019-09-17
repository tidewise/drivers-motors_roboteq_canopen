#ifndef MOTORS_ROBOTEQ_CANOPEN_TEST_HELPERS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_TEST_HELPERS_HPP

#include <vector>
#include <gtest/gtest.h>
#include <canbus/Message.hpp>
#include <canopen_master/SDO.hpp>
#include <canopen_master/PDOMapping.hpp>

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
};

#endif