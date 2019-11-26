#ifndef MOTORS_ROBOTEQ_CANOPEN_DS402DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DS402DRIVER_HPP

#include <canopen_master/Slave.hpp>
#include <canopen_master/PDOCommunicationParameters.hpp>
#include <motors_roboteq_canopen/ControllerStatus.hpp>
#include <motors_roboteq_canopen/DS402Channel.hpp>
#include <motors_roboteq_canopen/Driver.hpp>
#include <base/JointState.hpp>
#include <base/samples/Joints.hpp>

namespace motors_roboteq_canopen {
    /**
     * DS402Driver implementation for the CANOpen interface to Roboteq controllers
     */
    class DS402Driver : public Driver {
    public:
        DS402Driver(canopen_master::StateMachine& state_machine, int channel_count);

        DS402Channel& getChannel(int i);
    };
}

#endif