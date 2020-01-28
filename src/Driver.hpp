#ifndef MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP

#include <motors_roboteq_canopen/DriverBase.hpp>
#include <motors_roboteq_canopen/Channel.hpp>

namespace motors_roboteq_canopen {
    /**
     * Controller driver using the CANOpen roboteq-specific objects
     */
    class Driver : public DriverBase {
    public:
        Driver(canopen_master::StateMachine& state_machine, int channel_count);

        Channel& getChannel(int i);

        std::vector<canbus::Message> queryMotorStop();
    };
}

#endif