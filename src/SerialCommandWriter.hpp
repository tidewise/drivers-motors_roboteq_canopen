#ifndef MOTORS_ROBOTEQ_CANOPEN_SERIALCOMMANDWRITER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_SERIALCOMMANDWRITER_HPP

#include <cstdint>
#include <iosfwd>
#include <stdexcept>

#include <iodrivers_base/Driver.hpp>

namespace motors_roboteq_canopen {
    /**
     * Driver class that can send and validate commands through the serial line protocol
     *
     * The objective of this class is to provide a way to send configuration
     * files to the roboteq
     */
    class SerialCommandWriter : public iodrivers_base::Driver {
    public:
        struct CommandFailed : std::runtime_error {
            CommandFailed(std::string const& command_line);
        };

    private:
        static const int INTERNAL_BUFFER_SIZE = 1024;
        int extractPacket(uint8_t const* buffer, size_t buffer_size) const;

        std::string m_current_command;
        std::ostream* m_log_stream = nullptr;

        void log(std::string const& msg);

    public:
        SerialCommandWriter();

        /** Set a stream to which the class should display information */
        void setLogStream(std::ostream& stream);

        /** Send a command line, not waiting for the controller's reply */
        void sendCommand(std::string const& command_line);

        /** Send a command line and wait the controller's reply
         *
         * @throw InvalidCommand if the controller NACKed the command
         */
        void executeCommand(std::string const& command_line);

        /** Send multiple commands written in e.g. a file */
        void executeCommands(std::ifstream& stream);
    };
}

#endif