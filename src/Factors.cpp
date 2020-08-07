#include <motors_roboteq_canopen/Factors.hpp>
#include <cmath>

using namespace motors_roboteq_canopen;

int32_t Factors::clamp1000(float value) {
    int32_t v = std::round(value);
    if (v > 1000) {
        return 1000;
    }
    else if (v < -1000) {
        return -1000;
    }
    else {
        return v;
    }
}


static float toSI(int32_t roboteq, float max, float min) {
    return (roboteq + 1000) * (max - min) / 2000 + min;
}

static int32_t fromSI(float si, float max, float min) {
    float v = 2000 * (si - min) / (max - min) - 1000;
    return Factors::clamp1000(std::round(v));
}

float Factors::relativePositionToSI(int32_t position) const {
    return toSI(position, position_max, position_min);
}

int32_t Factors::relativePositionFromSI(float position) const {
    return fromSI(position, position_max, position_min);
}

float Factors::relativeSpeedToSI(int32_t speed) const {
    return toSI(speed, speed_max, speed_min);
}

int32_t Factors::relativeSpeedFromSI(float speed) const {
    return clamp1000(fromSI(speed, speed_max, speed_min));
}

int32_t Factors::relativeTorqueFromSI(float torque) const {
    float current = currentFromTorqueSI(torque);
    return clamp1000(fromSI(current, max_current, -max_current));
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
