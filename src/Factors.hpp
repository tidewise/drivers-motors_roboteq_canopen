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

        float relativeSpeedToSI(int32_t position) const;
        float relativePositionToSI(int32_t position) const;
        float pwmToFloat(int16_t value) const;

        int32_t relativePositionFromSI(float position) const;
        int32_t relativeSpeedFromSI(float value) const;
        float rpmFromSI(float speed) const;
        float rpmToSI(float speed) const;

        /** Torque from current in A * 10 */
        float currentToTorqueSI(int16_t value) const;

        /** Current A * 100 from torque */
        int16_t currentFromTorqueSI(float value) const;

        /** Current slope in A * 1e4 / seconds from torque slope */
        int16_t currentSlopeFromTorqueSlopeSI(float value) const;
    };
}

#endif
