#ifndef MOTORS_ROBOTEQ_CANOPEN_CHANNEL_HPP
#define MOTORS_ROBOTEQ_CANOPEN_CHANNEL_HPP

#include <base/JointState.hpp>
#include <canopen_master/PDOMapping.hpp>
#include <canopen_master/StateMachine.hpp>
#include <motors_roboteq_canopen/ChannelBase.hpp>
#include <motors_roboteq_canopen/Objects.hpp>
#include <motors_roboteq_canopen/Factors.hpp>
#include <motors_roboteq_canopen/Exceptions.hpp>

namespace motors_roboteq_canopen {
    class Driver;

    /**
     * Control of a single controller channel
     */
    class Channel : public ChannelBase {
    private:
        friend class Driver;

        Driver& m_driver;
        int m_channel;

        ControlModes m_control_mode = CONTROL_NONE;
        base::JointState m_current_command;

        Channel(Driver& driver, int channel);
        double validateField(base::JointState::MODE i, base::JointState const& cmd);

        template<typename T>
        void set(typename T::OBJECT_TYPE value);

        template<typename T>
        typename T::OBJECT_TYPE get() const;

        template<typename T>
        canbus::Message queryDownload() const;

        template<typename T>
        canbus::Message queryDownload(typename T::OBJECT_TYPE value) const;

        template<typename T>
        canbus::Message queryUpload() const;


        template<typename T>
        bool hasUpdatedObject(canopen_master::StateMachine::Update const& update) const;

        enum JointStateTracking {
            UPDATED_MOTOR_AMPS = 0x1,
            UPDATED_POWER_LEVEL = 0x2,
            UPDATED_FEEDBACK = 0x4
        };

        uint8_t m_joint_state_tracking = 0;
        uint8_t m_joint_state_mask = 0;
        uint32_t getJointStateMask() const;

    public:
        bool isIgnored() const;

        /** Return a SDO download query to stop the motor */
        canbus::Message queryMotorStop() const;

        /** Get the channel's joint state */
        std::vector<canbus::Message> queryJointState() const;

        /** Get the channel's joint state */
        base::JointState getJointState() const;

        /** Add the joint state object to the PDO mapping */
        std::vector<canopen_master::PDOMapping> getJointStateTPDOMapping() const;

        /**
         * Set the channel's operating mode
         *
         * Note that the CANOpen interface cannot change the mode. This is set to
         * tell the driver in which mode the channel has been configured
         */
        void setControlMode(ControlModes mode);

        /**
         * Return the channel's operation mode
         */
        ControlModes getControlMode() const;

        /** Set command objects in the object dictionary */
        void setJointCommand(base::JointState const& cmd);

        /** Returns the last joint command set */
        base::JointState getJointCommand() const;

        /** Return the SDO messages that would update the current command
         */
        std::vector<canbus::Message> queryJointCommandDownload() const;

        /** Add the joint command object to the PDO mapping */
        std::vector<canopen_master::PDOMapping> getJointCommandRPDOMapping() const;

        /** Track which fields of the JointState have been received
         *
         * @return true if all expected fields (given the operation mode) have been
         *         received. Call resetJointStateTracking() to reset the flag
         * @see hasJointStateUpdate
         */
        bool updateJointStateTracking(canopen_master::StateMachine::Update const& update);

        /** Whether all fields necessary for getJointState have been updated since
         * the last call to resetJointStateTracking
         *
         * This assumes that you call updateJointStateTracking with the update
         * call
         */
        bool hasJointStateUpdate() const;

        /** Reset the internal tracking state of updateJointStateTracking */
        void resetJointStateTracking();
    };
}
#endif