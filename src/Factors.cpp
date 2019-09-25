#include <motors_roboteq_canopen/Factors.hpp>
#include <cmath>

using namespace motors_roboteq_canopen;

double Factors::relativePositionToSI(int32_t position) const {
    return (position + 1000) * (position_max - position_min) / 2000
           + position_min - position_zero;
}

float Factors::velocityToSI(int32_t value) const {
    return static_cast<float>(value) * 2 * M_PI / 60;
}

float Factors::torqueToSI(int16_t value) const {
    return static_cast<float>(value) / 100;
}

float Factors::currentToTorqueSI(int16_t value) const {
    return static_cast<float>(value) / 10 * torque_constant;
}

float Factors::pwmToFloat(int16_t value) const {
    return static_cast<float>(value) / 1000;
}

int32_t Factors::relativePositionFromSI(double position) const {
    return (position + position_zero - position_min) / (position_max - position_min)
           * 1000
           - 1000;
}

int32_t Factors::accelerationFromSI(float value) const {
    return value / (2 * M_PI) * 60 * 10;
}

int32_t Factors::velocityFromSI(float value) const {
    return value / (2 * M_PI) * 60;
}

int16_t Factors::torqueFromSI(float value) const {
    return value * 100;
}
int16_t Factors::torqueSlopeFromSI(float value) const {
    return value * 10 * 1000;
}
