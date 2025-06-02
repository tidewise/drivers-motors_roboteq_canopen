#ifndef MOTORS_ROBOTEQ_CANOPEN_CHANNELBASE_HPP
#define MOTORS_ROBOTEQ_CANOPEN_CHANNELBASE_HPP

#include <vector>

#include <base/JointState.hpp>
#include <canopen_master/PDOMapping.hpp>
#include <canopen_master/StateMachine.hpp>
#include <motors_roboteq_canopen/Objects.hpp>
#include <motors_roboteq_canopen/Factors.hpp>
#include <motors_roboteq_canopen/JointStateSources.hpp>
#include <motors_roboteq_canopen/Exceptions.hpp>

namespace motors_roboteq_canopen {
    class DS402Driver;

    /**
     * Control of a single controller channel
     */
    class ChannelBase {
    protected:
        Factors m_factors;
        JointStatePositionSources m_joint_state_position_source;

    public:
        virtual ~ChannelBase();

        /** Set conversion factors for the given channel
         *
         * Conversion factors are used to convert from Roboteq's internal
         * representations to Rock's (i.e. SI)
         */
        virtual void setFactors(Factors const& factors);

        /** Whether this channel should simply be ignored by the rest of the system
         */
        virtual bool isIgnored() const = 0;

        /** Get the channel's joint state */
        virtual std::vector<canbus::Message> queryJointState() const = 0;

        /** Get the channel's joint state */
        virtual base::JointState getJointState() const = 0;

        /** Add the joint state object to the PDO mapping */
        virtual std::vector<canopen_master::PDOMapping>
            getJointStateTPDOMapping() const = 0;

        /** Set command objects in the object dictionary */
        virtual void setJointCommand(base::JointState const& cmd) = 0;

        /** Returns the last joint command set */
        virtual base::JointState getJointCommand() const = 0;

        /** Return the SDO messages that would update the current command
         */
        virtual std::vector<canbus::Message> queryJointCommandDownload() const = 0;

        /** Add the joint command object to the PDO mapping */
        virtual std::vector<canopen_master::PDOMapping>
            getJointCommandRPDOMapping() const = 0;

        /** Track which fields of the JointState have been received
         *
         * @return true if all expected fields (given the operation mode) have been
         *         received. Call resetJointStateTracking() to reset the flag
         * @see hasJointStateUpdate
         */
        virtual bool updateJointStateTracking(
            canopen_master::StateMachine::Update const& update
        ) = 0;

        /** Whether all fields necessary for getJointState have been updated since
         * the last call to resetJointStateTracking
         *
         * This assumes that you call updateJointStateTracking with the update
         * call
         */
        virtual bool hasJointStateUpdate() const = 0;

        /** Reset the internal tracking state of updateJointStateTracking */
        virtual void resetJointStateTracking() = 0;
    };
}

#endif