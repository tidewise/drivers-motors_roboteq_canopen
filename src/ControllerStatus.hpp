#ifndef MOTORS_ROBOTEQ_CANOPEN_CONTROLLERSTATUS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_CONTROLLERSTATUS_HPP

#include <base/Float.hpp>
#include <base/Temperature.hpp>

namespace motors_roboteq_canopen {
    /** Bitfield values for Roboteq's status flags */
    enum StatusFlags {
        STATUS_SERIAL_MODE = 0x01,
        STATUS_PULSE_MODE = 0x02,
        STATUS_ANALOG_MODE = 0x04,
        STATUS_POWER_STAGE_OFF = 0x08,
        STATUS_STALL = 0x10,
        STATUS_AT_LIMIT = 0x20,
        STATUS_UNUSED = 0x40,
        STATUS_MICROBASIC_SCRIPT_RUNNING = 0x80,
        STATUS_TUNING_MODE = 0x100
    };

    /** Bitfield values for Roboteq's fault flags */
    enum FaultFlags {
        FAULT_OVERHEAT = 0x01,
        FAULT_OVERVOLTAGE = 0x02,
        FAULT_UNDERVOLTAGE = 0x04,
        FAULT_SHORT_CIRCUIT = 0x08,
        FAULT_EMERGENCY_STOP = 0x10,
        FAULT_SETUP_FAULT = 0x20,
        FAULT_MOSFET_FAILURE = 0x40,
        FAULT_UNCONFIGURED = 0x80
    };

    /** Single controller channel status
     */
    struct ControllerStatus {
        float voltage_internal = base::unknown<float>();
        float voltage_battery = base::unknown<float>();
        float voltage_5v = base::unknown<float>();
        base::Temperature temperature_mcu;
        std::vector<base::Temperature> temperature_sensors;

        /** @meta bitfield /motors_roboteq_canopen/StatusFlags
         */
        uint16_t status_flags = 0xFFFF;

        /** @meta bitfield /motors_roboteq_canopen/FaultFlags
         */
        uint16_t fault_flags = 0xFFFF;
    };
}

#endif