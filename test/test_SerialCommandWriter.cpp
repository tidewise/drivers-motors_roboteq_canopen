#include <gtest/gtest.h>
#include <fstream>

#include <iodrivers_base/FixtureGTest.hpp>

#include <motors_roboteq_canopen/SerialCommandWriter.hpp>

using namespace std;
using namespace motors_roboteq_canopen;

struct SerialCommandWriterTest : public ::testing::Test,
                                 public iodrivers_base::Fixture<SerialCommandWriter> {
    SerialCommandWriterTest() {
        driver.openURI("test://");
    }

    void pushStringToDriver(string const& cmd) {
        pushDataToDriver(cmd.begin(), cmd.end());
    }

    void EXPECT_COMMAND(std::string command, std::string reply) {
        uint8_t const* command_u8 = reinterpret_cast<uint8_t const*>(command.c_str());
        vector<uint8_t> command_v(command_u8, command_u8 + command.size());
        uint8_t const* reply_u8 = reinterpret_cast<uint8_t const*>(reply.c_str());
        vector<uint8_t> reply_v(reply_u8, reply_u8 + reply.size());

        EXPECT_REPLY(command_v, reply_v);
    }

    string getDataFile(string name) {
        string current = __FILE__;
        size_t dirname = current.find_last_of("/");
        return current.substr(0, dirname) + "/data/SerialCommandWriter/" + name;
    }

    void sendFile(string name) {
        string path = getDataFile(name);
        ifstream file(path);
        if (!file) {
            throw std::invalid_argument("no test file " + name);
        }
        driver.executeCommands(file);
    }
};

TEST_F(SerialCommandWriterTest, it_sends_a_command_with_parameters_and_validates_the_reply) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1 4\r\n", "^MMOD 1 4+");
    driver.executeCommand("^MMOD 1 4");
}

TEST_F(SerialCommandWriterTest, it_ignores_garbage_in_the_received_stream) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1 4\r\n", "BR\n\r^MR^MMOD 1 4+");
    driver.executeCommand("^MMOD 1 4");
}

TEST_F(SerialCommandWriterTest, it_throws_if_the_command_is_rejected) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1 4\r\n", "^MMOD 1 4-");
    ASSERT_THROW(driver.executeCommand("^MMOD 1 4"),
                 SerialCommandWriter::CommandFailed);
}

TEST_F(SerialCommandWriterTest, it_sends_a_command_file_that_contains_only_commands) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    EXPECT_COMMAND("^KD 1 100\r\n", "^KD 1 100+");
    EXPECT_COMMAND("^CLERD 1 0\r\n", "^CLERD 1 0+");
    sendFile("commands_only.txt");
}

TEST_F(SerialCommandWriterTest, it_ignores_comments_at_the_end_of_line) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    EXPECT_COMMAND("^KD 1 100\r\n", "^KD 1 100+");
    EXPECT_COMMAND("^CLERD 1 0\r\n", "^CLERD 1 0+");
    sendFile("with_inline_comments.txt");
}

TEST_F(SerialCommandWriterTest, it_ignores_comment_lines) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    EXPECT_COMMAND("^CLERD 1 0\r\n", "^CLERD 1 0+");
    sendFile("with_comment_lines.txt");
}

TEST_F(SerialCommandWriterTest, it_ignores_empty_lines) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    EXPECT_COMMAND("^CLERD 1 0\r\n", "^CLERD 1 0+");
    sendFile("with_empty_lines.txt");
}

TEST_F(SerialCommandWriterTest, it_rejects_files_containing_runtime_queries) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    ASSERT_THROW(sendFile("with_runtime_queries.txt"),
                 std::invalid_argument);
}

TEST_F(SerialCommandWriterTest, it_rejects_files_containing_configuration_queries) {
    IODRIVERS_BASE_MOCK();
    EXPECT_COMMAND("^MMOD 1\r\n", "^MMOD 1+");
    ASSERT_THROW(sendFile("with_configuration_queries.txt"),
                 std::invalid_argument);
}