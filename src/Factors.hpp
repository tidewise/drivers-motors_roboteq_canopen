#ifndef MOTORS_ROBOTEQ_CANOPEN_FACTORS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_FACTORS_HPP

#include <cstdint>

namespace motors_roboteq_canopen {
    /** Conversion factors between the internal units and SI units
     */
    struct Factors {
        /** Relative speed at which the SI speed is zero
         */
        int32_t speed_zero = 0;
        /** SI speed when relative speed is -1000
         */
        double speed_min = -1;
        /** SI speed when relative speed is +1000
         */
        double speed_max = 1;

        /** Relative position at which the SI position is zero
         */
        int32_t position_zero = 0;
        /** SI position at which the relative position is -1000
         */
        double position_min = -1;
        /** SI position at which the relative position is +1000
         */
        double position_max = 1;

        /** Torque constant in A/Nm */
        double torque_constant = 1;

        float speedToSI(int32_t position) const;
        float positionToSI(int32_t position) const;
        float torqueToSI(int16_t value) const;
        float currentToTorqueSI(int16_t value) const;
        float pwmToFloat(int16_t value) const;

        int32_t positionFromSI(float position) const;
        int32_t speedFromSI(float value) const;
        int32_t accelerationFromSI(float value) const;
        int16_t torqueFromSI(float value) const;
        int16_t torqueSlopeFromSI(float value) const;
    };
}

#endif
