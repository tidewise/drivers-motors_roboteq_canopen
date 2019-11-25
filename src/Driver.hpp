#ifndef MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP

#include <canopen_master/Slave.hpp>
#include <canopen_master/PDOCommunicationParameters.hpp>
#include <motors_roboteq_canopen/ControllerStatus.hpp>
#include <motors_roboteq_canopen/Channel.hpp>
#include <base/JointState.hpp>
#include <base/samples/Joints.hpp>

namespace motors_roboteq_canopen {
    /**
     * Driver implementation for the CANOpen interface to Roboteq controllers
     */
    class Driver : public canopen_master::Slave {
    public:
        static const int MAX_CHANNEL_COUNT = 4;

    private:
        std::vector<Channel> m_channels;

        int m_joint_state_sync_period;
        base::Time m_joint_state_period;

        int m_rpdo_begin = 0;
        int m_rpdo_end = 0;

        canopen_master::PDOCommunicationParameters
            getJointStateTPDOParameters();

    public:
        Driver(canopen_master::StateMachine& state_machine, int channel_count);

        canopen_master::StateMachine::Update process(
            canbus::Message const& message
        );

        /** Return the SDO queries to update the controller status */
        std::vector<canbus::Message> queryControllerStatus();

        /** Return the last known controller status value
         *
         * This only reads from the internal object database. It is up to the
         * caller to ensure the existence and freshness of the information
         */
        ControllerStatus getControllerStatus() const;

        /** Returns the object controlling this object's N-th channel
         */
        Channel& getChannel(int i);

        /** Return the SDO messages that will setup the PDOs for joint
         * control and feedback
         */
        int setupJointStateTPDOs(
            std::vector<canbus::Message>& messages, int pdoStartIndex,
            canopen_master::PDOCommunicationParameters const& parameters
        );

        int setupJointCommandRPDOs(
            std::vector<canbus::Message>& messages, int pdoStartIndex,
            canopen_master::PDOCommunicationParameters const& parameters
        );

        int setupStatusTPDOs(
            std::vector<canbus::Message>& messages, int pdoStartIndex,
            canopen_master::PDOCommunicationParameters const& parameters
        );

        /** Update the object dictionary to reflect the given command
         */
        void setJointCommand(base::samples::Joints const& command);

        /** Get the last set joint command */
        base::samples::Joints getJointCommand() const;

        /** Generate the RPDO messages to be sent on the bus to
         * apply the last set command
         *
         * @see setupJointCommandRPDOs setJointCommand
         */
        std::vector<canbus::Message> getRPDOMessages() const;
    };
}

#endif