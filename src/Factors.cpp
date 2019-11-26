#include <motors_roboteq_canopen/Factors.hpp>
#include <cmath>

using namespace motors_roboteq_canopen;

static float toSI(int32_t roboteq, float max, int32_t zero, float min) {
    if (roboteq > zero) {
        return static_cast<float>(roboteq - zero) / (1000.0 - zero) * max;
    }
    else {
        return static_cast<float>(zero - roboteq) / (1000.0 + zero) * min;
    }
}

static int32_t fromSI(float si, float max, int32_t zero, float min) {
    if (si > 0) {
        return (si / max) * (1000.0 - zero) + zero;
    }
    else {
        return (si / min) * (-1000.0 - zero) + zero;
    }
}

float Factors::relativePositionToSI(int32_t position) const {
    return toSI(position, position_max, position_zero, position_min);
}

int32_t Factors::relativePositionFromSI(float position) const {
    return fromSI(position, position_max, position_zero, position_min);
}

float Factors::relativeSpeedToSI(int32_t speed) const {
    return toSI(speed, speed_max, speed_zero, speed_min);
}

int32_t Factors::relativeSpeedFromSI(float speed) const {
    return fromSI(speed, speed_max, speed_zero, speed_min);
}

float Factors::rpmToSI(float speed) const {
    return speed * 2 * M_PI / 60;
}

float Factors::rpmFromSI(float speed) const {
    return speed / (2 * M_PI) * 60;
}

float Factors::pwmToFloat(int16_t value) const {
    return static_cast<float>(value) / 1000;
}

float Factors::currentToTorqueSI(int16_t value) const {
    return static_cast<float>(value) / 10.0 / torque_constant;
}

int16_t Factors::currentFromTorqueSI(float value) const {
    return value * torque_constant * 100;
}

int16_t Factors::currentSlopeFromTorqueSlopeSI(float value) const {
    return value * torque_constant * 10000;
}
