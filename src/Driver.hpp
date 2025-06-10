#ifndef MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP
#define MOTORS_ROBOTEQ_CANOPEN_DRIVER_HPP

#include <motors_roboteq_canopen/DriverBase.hpp>
#include <motors_roboteq_canopen/Channel.hpp>
#include <raw_io/Digital.hpp>

namespace motors_roboteq_canopen {
    /**
     * Controller driver using Roboteq's direct (non-DS402) CANOpen interface
     *
     * ## CAN communication
     *
     * The general API philosophy is to *not* deal with CAN communication. On the one
     * hand, you need to externally instanciate a CAN driver (for instance one of the
     * drivers/canbus package) and push packages to the driver's \c process method.
     * On the other hand, you will call `query` method that return CAN messages,
     * and you are responsible to push these messages to the CAN driver.
     *
     * The method nomenclature is to use "query" as a prefix that returns CAN messages
     * that should be sent to the device, and "read" for things that interact directly
     * with the driver's internal representation of CANOpen objects. These read methods,
     * for instance, will throw if trying to read an object that has never been received
     * from the driver.
     *
     * ## Interaction with the motor control itself: Channel
     *
     * Some driver functionality is split into Channels, each channel being one motor
     * controlled by the roboteq device itself. The number of channels has to be declared
     * in the constructor.
     *
     * Once configured, motor control happens through Rock's standard type (JointState).
     * The different motor control modes expect different control inputs:
     * - CONTROL_OPEN_LOOP: control in raw (PWM), in [-1, 1]
     * - CONTROL_TORQUE: control in effort (torque). In effect, this is a control in
     *   current inside the roboteq controller. The scaling factor applied between
     *   the JointState (Nm) and the current setpoint (A) sent to the roboteq controller
     *   has to be given via the Factors.torque_constant field
     * - CONTROL_SPEED and CONTROL_POSITION: The driver assumes that the roboteq
     *   controller expects a value in [-1000, 1000] and the conversion parameters are
     *   given in the Factors structure
     *
     * In the default configuration, the feedback JointState always includes the torque
     * and pwm (resp. in the effort and raw fields). The torque is converted from the
     * device's reported current via the \c torque_constant field of \c Factors.
     *
     * When in position resp. speed control modes, using the default configuration, the
     * \c position (resp. \c speed) field contains the feedback position used by the
     * roboteq's internal controller. It is assumed that the internal feedback value is
     * in [-1000, 1000], which gets normalized to SI using the parameters from \c Factors
     *
     * Alternatively, the \c JointState can be made to contain a position derived
     * from an encoder channel. This is done by calling
     * \c Channel#setJointStatePositionSource with the JOINT_STATE_POSITION_SOURCE_ENCODER
     * This setting can be used in any control mode (including OPEN_LOOP or TORQUE)
     *
     * A full JointState update requires more than one reading. Getting an update
     * can be done via SDOs (explicit query) or PDOs.
     *
     * To get a full update using SDOs, call \c Channel.queryJointState(). Each returned
     * message is a single SDO query, and you must wait for the query answer before
     * sending the next one.
     *
     * In the case of PDOs, one must first setup the PDOs themselves.
     * \c setupJointSTateTPDOs will return the SDO queries necessary to do the PDO
     * setup for joint state updates. Then, at runtime, one calls
     * \c Channel.hasJointStateUpdate to check whether a full update was received. Once it
     * returns true, read the joint state and call \c Channel.resetJointStateTracking
     * to be able to wait for a new update.
     *
     * ## Analog readings
     *
     * Periodic analog readings via PDOs are managed the same way than the joint state
     * updates:
     * - choose and configure the desired analog readings using
     *   \c setAnalogInputEnableInTPDO, resp. \c setConvertedAnalogInputEnableInTPDO if
     *   getting converted values (see Roboteq manual for the difference between raw
     *   and converted analog values)
     * - setup the PDOs using \c setupAnalogPDOs
     * - at runtime, after passing all pending CAN messages to \c process, check if
     *   the analog readings have been updated using \c hasAnalogInputUpdate
     *   (resp. \c hasConvertedAnalogInputUpdate) and reset the tracking with
     *   \c resetAnalogInputTracking (resp. \c resetConvertedAnalogInputTracking)
     *   to wait for the next update cycle
     *
     * ## Digital IO
     *
     * Digital IOs are read in bulk. A single queryReadDigitalOutput will update all of
     * them. Writing, however, is done per-output using \c queryWriteDigitalOutput
     *
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
