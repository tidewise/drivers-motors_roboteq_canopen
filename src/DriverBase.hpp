#ifndef MOTORS_ROBOTEQ_CANOPEN_BASE_DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_BASE_DRIVER_HPP

#include <canopen_master/Slave.hpp>
#include <canopen_master/PDOCommunicationParameters.hpp>
#include <motors_roboteq_canopen/ControllerStatus.hpp>
#include <motors_roboteq_canopen/ChannelBase.hpp>
#include <base/JointState.hpp>
#include <base/samples/Joints.hpp>

namespace motors_roboteq_canopen {
    /**
     * Common CANOpen-related functionality for DS402 and direct CANOpen protocols
     *
     * See the Documentation of Driver for a high level description Roboteq's direct
     * CANOpen interface and DS402Driver for the DS402-based protocol. The latter
     * is most probably broken, use it at your own risk
     */
    class DriverBase : public canopen_master::Slave {
    private:
        std::vector<ChannelBase*> m_channels;

        int m_joint_state_sync_period;
        base::Time m_joint_state_period;


        uint32_t m_received_encoder_counter_mask = 0;
        uint32_t m_expected_encoder_counter_mask = 0;

        uint32_t m_received_analog_inputs_mask = 0;
        uint32_t m_expected_analog_inputs_mask = 0;
        uint32_t m_received_converted_analog_inputs_mask = 0;
        uint32_t m_expected_converted_analog_inputs_mask = 0;

        int m_rpdo_begin = 0;
        int m_rpdo_end = 0;

        canopen_master::PDOCommunicationParameters
            getJointStateTPDOParameters();

        int setupTPDO(
            canopen_master::PDOMapping mapping, std::vector<canbus::Message>& messages,
            int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters
        );

        int setupAnalogTPDOsInternal(
            uint32_t const mask, int objectOffset, std::vector<canbus::Message>& messages,
            int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters
        );

    protected:
        void addChannel(ChannelBase* channel);

    public:
        DriverBase(canopen_master::StateMachine& state_machine);
        virtual ~DriverBase();

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

        /** Return how many channels have been declared on this driver
         */
        size_t getChannelCount() const;

        /** Returns the object controlling this object's N-th channel
         */
        ChannelBase& getChannel(int i);

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

        /** Insert SDO messages to configure the TPDOs needed to receive analog inputs
         *
         * The inputs must have been configured first with @c setAnalogInputEnableInTPDO
         * or @c setAnalogInputConvertedEnableInTPDO
         */
        int setupAnalogTPDOs(
            std::vector<canbus::Message>& messages, int pdoStartIndex,
            canopen_master::PDOCommunicationParameters const& parameters
        );

        /** Setup TPDOs to receive encoder updates
         *
         * The primary mean to get encoder value is to tie it to a channel
         * and setup the \c Channel object to read the encoder in the channel's
         * joint state position field using \c Channel.setJointStatePositionSource.
         *
         * Alternatively, one can read encoder values independently. Choose which
         * encoders to read with \c setEncoderCounterEnableInTPDO, setup the TPDO
         * using this method and then use the state tracking pattern
         * \c hasEncoderCounterUpdate / \c resetEncoderCounterTracking to track
         * updates.
         *
         * Each encoder pair consumes a PDO.
         *
         * @param messages the CAN message array in which the method will add SDO
         *    setup messages that configure the given TPDOs
         * @param pdoStartIndex the PDO index of the first PDO that should be used
         * @return the next usable PDO index (meaning the method used
         *    [pdoStartIndex, returnValue[)
         */
        int setupEncoderTPDOs(
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

        /** Get the SDO write messages that update the joint command */
        std::vector<canbus::Message> queryJointCommandDownload() const;

        /** Whether we received a whole update of the analog inputs we are
         * expecting
         */
        bool hasAnalogInputUpdate() const;

        /** Whether we received a whole update of the encoders we are
         * expecting
         *
         * @see setupEncoderTPDOs
         */
        bool hasEncoderCounterUpdate() const;

        /** Enable or disable a given analog input to be received by TPDOs
         */
        void setAnalogInputEnableInTPDO(int index, bool enable);

        /** Enable or disable specific encoder channels to receive and track via PDOs
         *
         * @see setupEncoderTPDOs
         */
        void setEncoderCounterEnableInTPDO(int index, bool enable);

        /** Reset the state tracking needed by hasAnalogInputUpdate */
        void resetAnalogInputTracking();

        /** Reset tracking of encoder updates
         *
         * @see setupEncoderTPDOs
         */
        void resetEncoderCounterTracking();

        /**
         * Get query messages to receive the current value of the given analog
         * input
         *
         * This should be processed as a SDO query, that you can send it but have
         * to wait for a reply before another SDO query can be sent
         */
        canbus::Message queryAnalogInput(int index) const;

        /** Get query message to receive the current value of the given encoder
         *
         * This should be processed as a SDO query, that you can send it but have
         * to wait for a reply before another SDO query can be sent
         */
        canbus::Message queryEncoderCounter(int index) const;

        /**
         * Whether we received a whole update of the converted analog inputs we
         * are expecting
         */
        bool hasConvertedAnalogInputUpdate() const;

        /** Enable or disable a given analog input to be received by TPDOs
         */
        void setConvertedAnalogInputEnableInTPDO(int index, bool enable);

        /** Reset the state tracking needed by hasConvertedAnalogInputUpdate */
        void resetConvertedAnalogInputTracking();

        /**
         * Get query messages to receive the current value of the given analog
         * input in converted form
         */
        canbus::Message queryAnalogInputConverted(int index) const;
    };
}

#endif
