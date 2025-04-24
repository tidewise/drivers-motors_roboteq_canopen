#ifndef MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP

#include <motors_roboteq_canopen/DriverBase.hpp>
#include <motors_roboteq_canopen/Channel.hpp>
#include <raw_io/Digital.hpp>

namespace motors_roboteq_canopen {
    /**
     * Controller driver using the CANOpen roboteq-specific objects
     */
    class Driver : public DriverBase {
    public:
        Driver(canopen_master::StateMachine& state_machine, int channel_count);

        Channel& getChannel(int i);

        std::vector<canbus::Message> queryMotorStop();

        /**
         * Writes a digital output
         *
         * @param index is the ouput index, it ranges from 1 - MAX. Where MAX is the
         * controller's digital output quantity. Roboteq's SDC2160 is 2, e.g. This method
         * does not validate the index range, it is the caller's responsibility
         */
        canbus::Message queryWriteDigitalOutput(std::uint8_t index,
            bool out);

        canbus::Message queryReadDigitalOutput();

        std::uint16_t readDigitalOutputRaw();

        std::vector<raw_io::Digital> readDigitalOutput(
            std::vector<std::uint8_t> const& managed_outputs);

        static std::vector<raw_io::Digital> parseDigitalOutput(std::uint16_t reading,
            std::vector<std::uint8_t> const& managed_outputs);
    };
}

#endif