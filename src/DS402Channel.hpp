#ifndef MOTORS_ROBOTEQ_CANOPEN_DS402CHANNEL_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DS402CHANNEL_HPP

#include <vector>

#include <base/JointState.hpp>
#include <canopen_master/PDOMapping.hpp>
#include <canopen_master/StateMachine.hpp>
#include <motors_roboteq_canopen/Objects.hpp>
#include <motors_roboteq_canopen/Factors.hpp>
#include <motors_roboteq_canopen/Exceptions.hpp>
#include <motors_roboteq_canopen/ChannelBase.hpp>

namespace motors_roboteq_canopen {
    class DS402Driver;

    /**
     * Control of a single controller channel
     */
    class DS402Channel : public ChannelBase {
    public:
        /** Return the object id and sub-id needed to access the given dictionary object
         * for this channel
         */
        template<typename T>
        std::pair<int, int> getObjectOffsets() const;

    private:
        friend class DS402Driver;

        static const int CHANNEL_OBJECT_ID_OFFSET = 0x800;

        DS402Driver& m_driver;
        int m_channel;
        int m_object_id_offset;

        DS402OperationModes m_operation_mode = DS402_OPERATION_MODE_NONE;
        base::JointState m_current_command;
        bool m_command_fields[base::JointState::UNSET];

        DS402Channel(DS402Driver& driver, int channel);
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
            UPDATED_ACTUAL_PROFILE_VELOCITY = 0x4,
            UPDATED_ACTUAL_VELOCITY = 0x8,
            UPDATED_POSITION = 0x10,
            UPDATED_TORQUE = 0x20
        };

        uint8_t m_joint_state_tracking = 0;
        uint8_t m_joint_state_mask = 0;
        uint32_t getJointStateMask() const;

    public:
        bool isIgnored() const;

        /** Send a DS402 transition */
        std::vector<canbus::Message> sendDS402Transition(
            ControlWord::Transition transition, bool enable_halt
        ) const;

        /** Get the DS402 state machine status */
        std::vector<canbus::Message> queryDS402Status() const;

        /** Get the DS402 state machine status */
        StatusWord getDS402Status() const;

        /** Get the channel's joint state */
        std::vector<canbus::Message> queryJointState() const;

        /** Get the channel's joint state */
        base::JointState getJointState() const;

        /** Add the joint state object to the PDO mapping */
        std::vector<canopen_master::PDOMapping> getJointStateTPDOMapping() const;

        /** Return the SDO messages that would update the drive's operation mode
         * for this channel
         */
        std::vector<canbus::Message> queryOperationModeDownload(
            DS402OperationModes mode
        );

        /**
         * Change the DS402Channel's internal state to reflect a change of operation
         * mode
         */
        void setOperationMode(DS402OperationModes mode);

        /**
         * Return the channel's operation mode
         */
        DS402OperationModes getOperationMode() const;

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
