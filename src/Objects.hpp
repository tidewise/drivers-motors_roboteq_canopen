#ifndef MOTORS_ROBOTEQ_CANOPEN_OBJECTS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_OBJECTS_HPP

#include <canopen_master/Objects.hpp>

namespace motors_roboteq_canopen {
    CANOPEN_DEFINE_OBJECT(0x2000, 1, SetCommand,                    std::int32_t);
    CANOPEN_DEFINE_OBJECT(0x2002, 1, SetSpeedTarget,                std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x200C, 0, EmergencyShutdown,             std::uint8_t);
    CANOPEN_DEFINE_OBJECT(0x200D, 0, ReleaseShutdown,               std::uint8_t);
    CANOPEN_DEFINE_OBJECT(0x200E, 0, MotorStop,                     std::uint8_t);

    CANOPEN_DEFINE_OBJECT(0x2100, 1, MotorAmps,                     std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x2102, 1, AppliedPowerLevel,             std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x210C, 1, BatteryAmps,                   std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x210D, 1, VoltageInternal,               std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x210D, 2, VoltageBattery,                std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x210D, 3, Voltage5V,                     std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x210F, 1, TemperatureMCU,                std::int8_t);
    CANOPEN_DEFINE_OBJECT(0x210F, 2, TemperatureSensor0,            std::int8_t);

    CANOPEN_DEFINE_OBJECT(0x2110, 0, Feedback,                      std::int32_t);
    CANOPEN_DEFINE_OBJECT(0x2111, 0, StatusFlagsRaw,                std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x2112, 0, FaultFlagsRaw,                 std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x2114, 1, ClosedLoopError,               std::int32_t);

    CANOPEN_DEFINE_OBJECT(0x2119, 0, Time,                          std::uint32_t);

    CANOPEN_DEFINE_OBJECT(0x6040, 0, ControlWordRaw,                std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x6041, 0, StatusWordRaw,                 std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x6042, 0, TargetVelocity,                std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x6043, 0, ActualTargetVelocity,          std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x6044, 0, ActualVelocity,                std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x6046, 1, MinVelocity,                   std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6046, 2, MaxVelocity,                   std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6048, 1, VelocityAccelerationDelta,     std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6048, 2, VelocityAccelerationTime,      std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x6049, 1, VelocityDecelerationDelta,     std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6049, 2, VelocityDecelerationTime,      std::uint16_t);
    CANOPEN_DEFINE_OBJECT(0x6060, 0, OperationMode,                 std::int8_t);
    CANOPEN_DEFINE_OBJECT(0x6064, 0, Position,                      std::int32_t);
    CANOPEN_DEFINE_OBJECT(0x606C, 0, ActualProfileVelocity,         std::int32_t);
    CANOPEN_DEFINE_OBJECT(0x6071, 0, TargetTorque,                  std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x6077, 0, Torque,                        std::int16_t);
    CANOPEN_DEFINE_OBJECT(0x607A, 0, TargetPosition,                std::int32_t);
    CANOPEN_DEFINE_OBJECT(0x6081, 0, ProfileVelocity,               std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6083, 0, ProfileAcceleration,           std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6084, 0, ProfileDeceleration,           std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x6087, 0, TorqueSlope,                   std::uint32_t);
    CANOPEN_DEFINE_OBJECT(0x60FF, 0, TargetProfileVelocity,         std::int32_t);

    enum ControlModes {
        /** Completely ignore this channel */
        CONTROL_IGNORED,

        /** Report the channel state but do not control it */
        CONTROL_NONE,

        /** Direct PWM control */
        CONTROL_OPEN_LOOP,

        /** Speed control */
        CONTROL_SPEED,

        /** PIV control: position is computed as integral of speed command,
         * and the motor is controlled to reach this generated position
         */
        CONTROL_SPEED_POSITION,

        /**
         * Position control with acceleration and speed ramps
         */
        CONTROL_PROFILED_POSITION,

        /**
         * Direct position control. This is called "Position Tracking" in
         * Roboteq's documentation
         */
        CONTROL_POSITION,

        /** PIV control: position is computed as integral of speed command,
         * and the motor is controlled to reach this generated position
         */
        CONTROL_TORQUE
    };

    enum DS402OperationModes {
        /** Velocity-Position control loop, no profile
         *
         * The controller generates a position trajectory from a velocity
         * command, and runs the PID on the target position
         */
        DS402_OPERATION_MODE_VELOCITY_POSITION = -4,

        /** Velocity-Position control loop, profile
         *
         * The controller generates a position trajectory from a velocity
         * command, and runs the PID on the target position
         *
         * The velocity trajectory is controlled with profile parameters
         */
        DS402_OPERATION_MODE_VELOCITY_POSITION_PROFILE = -3,

        /** Direct PID control on relative position with velocity profile.
         *
         * Command and feedback is expressed in ratio to min/max value [-1000,
         * 1000]
         *
         * Direct position PID
         */
        DS402_OPERATION_MODE_RELATIVE_POSITION_PROFILE = -2,

        /** Direct PID control on relative position, no profile.
         *
         * Command and feedback is expressed in ratio to min/max value [-1000,
         * 1000]
         *
         * Direct position PID
         */
        DS402_OPERATION_MODE_RELATIVE_POSITION = -1,

        /** Open loop control
         */
        DS402_OPERATION_MODE_NONE = 0,

        /** Direct PID control on absolute position with velocity profile
         */
        DS402_OPERATION_MODE_POSITION_PROFILE = 1,

        /** Direct PID control on velocity, no profile
         */
        DS402_OPERATION_MODE_VELOCITY = 2,

        /** Direct PID control on velocity with acceleration profile
         */
        DS402_OPERATION_MODE_VELOCITY_PROFILE = 3,

        /** Direct PID control on torque with ramps
         */
        DS402_OPERATION_MODE_TORQUE_PROFILE = 4,

        /** Direct PID control on position for feedback based on analog inputs */
        DS402_OPERATION_MODE_ANALOG_POSITION = 10,

        /** Direct PID control on velocity for feedback based on analog inputs */
        DS402_OPERATION_MODE_ANALOG_VELOCITY = 11
    };

    enum Updates {
        UPDATE_JOINT_POSITION = 0x0001,
        UPDATE_JOINT_VELOCITY = 0x0002,
        UPDATE_JOINT_TORQUE   = 0x0004,
        UPDATE_JOINT_PWM      = 0x0008,
        UPDATE_JOINT_ALL      = 0x000F
    };

    /** Representation of the control word
     *
     * The control word changes the drive's operational state
     */
    struct ControlWord
    {
        enum Transition
        {
            SHUTDOWN,
            SWITCH_ON,
            ENABLE_OPERATION,
            DISABLE_VOLTAGE,
            QUICK_STOP,
            DISABLE_OPERATION,
            FAULT_RESET
        };

        ControlWord(DS402OperationModes mode, Transition transition, bool enable_halt)
            : operation_mode(mode)
            , transition(transition)
            , enable_halt(enable_halt) {}

        DS402OperationModes operation_mode;
        Transition transition;
        bool enable_halt;

        ControlWordRaw::OBJECT_TYPE toRaw() const;
    };

    /** Representation of the status word
     *
     * The status word is the main representation of the drive's state
     */
    struct StatusWord
    {
        enum State
        {
            NOT_READY_TO_SWITCH_ON,
            SWITCH_ON_DISABLED,
            READY_TO_SWITCH_ON,
            SWITCH_ON,
            OPERATION_ENABLED,
            QUICK_STOP_ACTIVE,
            FAULT_REACTION_ACTIVE,
            FAULT
        };

        struct UnknownState : public std::runtime_error
        {
            using std::runtime_error::runtime_error;
        };

        uint16_t raw;
        State state;
        bool voltage_enabled;
        bool warning;
        bool target_reached;
        bool internal_limit_active;

        StatusWord(uint16_t raw, State state, bool voltageEnabled, bool warning,
                   bool targetReached, bool internalLimitActive)
            : raw(raw)
            , state(state)
            , voltage_enabled(voltageEnabled)
            , warning(warning)
            , target_reached(targetReached)
            , internal_limit_active(internalLimitActive) {}

        static StatusWord fromRaw(StatusWordRaw::OBJECT_TYPE raw);
    };

}

#endif