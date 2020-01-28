#include <motors_roboteq_canopen/SerialCommandWriter.hpp>
#include <fstream>
#include <iostream>

using namespace std;
using namespace motors_roboteq_canopen;

SerialCommandWriter::CommandFailed::CommandFailed(string const& command_line)
    : runtime_error(command_line + ": command rejected") {

}

int SerialCommandWriter::extractPacket(uint8_t const* buffer, size_t buffer_size) const {
    if (m_current_command.empty()) {
        return -buffer_size;
    }

    size_t expected_reply_size = m_current_command.size() + 2;

    size_t prefix = std::min(buffer_size, m_current_command.size());
    for (size_t i = 0; i < prefix; ++i) {
        if (m_current_command[i] != buffer[i]) {
            return -1;
        }
    }

    if (buffer_size < expected_reply_size) {
        return 0;
    }

    char lf = buffer[expected_reply_size - 2];
    char ack = buffer[expected_reply_size - 1];
    if (lf != '\r' || (ack != '+' && ack != '-')) {
        return -1;
    }
    return expected_reply_size;
}

SerialCommandWriter::SerialCommandWriter()
    : iodrivers_base::Driver(INTERNAL_BUFFER_SIZE) {
    setReadTimeout(base::Time::fromSeconds(1));
    setWriteTimeout(base::Time::fromSeconds(1));
}

void SerialCommandWriter::setLogStream(std::ostream& stream) {
    m_log_stream = &stream;
}

void SerialCommandWriter::log(std::string const& msg) {
    if (m_log_stream) {
        *m_log_stream << msg;
    }
}

void SerialCommandWriter::sendCommand(string const& command_line) {
    m_current_command = command_line;

    writePacket(
        reinterpret_cast<uint8_t const*>((command_line + "\r\n").c_str()),
        command_line.size() + 1
    );
}

void SerialCommandWriter::executeCommand(string const& command_line) {
    sendCommand(command_line);

    log ("> " + command_line);
    char read_buffer[INTERNAL_BUFFER_SIZE];
    int length = readPacket(
        reinterpret_cast<uint8_t*>(read_buffer),
        INTERNAL_BUFFER_SIZE
    );

    if (read_buffer[length - 1] != '+') {
        throw CommandFailed(command_line);
    }
    else {
        log(": OK\n");
    }
}

void SerialCommandWriter::executeCommands(ifstream& stream) {
    string line;
    while (getline(stream, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        else if (line[0] != '!' && line[0] != '^' && line[0] != '%') {
            throw invalid_argument(
                "unexpected command line '" + line + "', expected a line "
                "starting with '^' or '!'"
            );
        }

        string without_comments = line.substr(0, line.find_first_of("#"));
        size_t not_trailing_space = without_comments.find_last_not_of(" ");

        string command = line;
        if (not_trailing_space != string::npos) {
            executeCommand(without_comments.substr(0, not_trailing_space + 1));
        }
        else {
            executeCommand(line);
        }
    }
}
