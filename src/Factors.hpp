#ifndef MOTORS_ROBOTEQ_CANOPEN_FACTORS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_FACTORS_HPP

#include <cstdint>

namespace motors_roboteq_canopen {
    /** Conversion factors between the internal units and SI units
     */
    struct Factors {
        double position_zero = 0;
        double position_min = -1;
        double position_max = 1;
        /** Torque constant in A/Nm */
        double torque_constant = 1;

        double relativePositionToSI(int32_t position) const;
        float velocityToSI(int32_t value) const;
        float torqueToSI(int16_t value) const;
        float currentToTorqueSI(int16_t value) const;
        float pwmToFloat(int16_t value) const;

        int32_t relativePositionFromSI(double position) const;
        int32_t velocityFromSI(float value) const;
        int32_t accelerationFromSI(float acceleration) const;
        int16_t torqueFromSI(float value) const;
        int16_t torqueSlopeFromSI(float value) const;
    };
}

#endif
