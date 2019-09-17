#ifndef MOTORS_ROBOTEQ_CANOPEN_CHANNEL_HPP
#define MOTORS_ROBOTEQ_CANOPEN_CHANNEL_HPP

#include <vector>

#include <base/JointState.hpp>
#include <canopen_master/PDOMapping.hpp>
#include <motors_roboteq_canopen/Objects.hpp>
#include <motors_roboteq_canopen/Factors.hpp>
#include <motors_roboteq_canopen/Exceptions.hpp>

namespace motors_roboteq_canopen {
    class Driver;

    /**
     * Control of a single controller channel
     */
    class Channel {
        friend class Driver;

        static const int CHANNEL_OBJECT_ID_OFFSET = 0x800;

        Driver& m_driver;
        int m_channel;
        int m_object_id_offset;

        Factors m_factors;
        OperationModes m_operation_mode = OPERATION_MODE_NONE;
        bool m_command_fields[base::JointState::UNSET];

        Channel(Driver& driver, int channel);
        double validateField(base::JointState::MODE i, base::JointState const& cmd);

        template<typename T>
        void setRaw(typename T::OBJECT_TYPE value);

        template<typename T>
        typename T::OBJECT_TYPE getRaw() const;

        template<typename T>
        canbus::Message queryDownload() const;

        template<typename T>
        canbus::Message queryUpload() const;

    public:
        /** Set conversion factors for the given channel
         *
         * Conversion factors are used to convert from Roboteq's internal
         * representations to Rock's (i.e. SI)
         */
        void setFactors(Factors const& factors);

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
            OperationModes mode
        );

        /**
         * Change the Channel's internal state to reflect a change of operation
         * mode
         */
        void setOperationMode(OperationModes mode);

        /** Set command objects in the object dictionary */
        void setJointCommand(base::JointState const& cmd);

        /** Return the SDO messages that would update the current command
         */
        std::vector<canbus::Message> queryJointCommandDownload() const;

        /** Add the joint command object to the PDO mapping */
        std::vector<canopen_master::PDOMapping> getJointCommandRPDOMapping() const;
    };
}

#endif