#include <motors_roboteq_canopen/Channel.hpp>
#include <motors_roboteq_canopen/Driver.hpp>

using namespace std;
using namespace base;
using namespace motors_roboteq_canopen;

using canopen_master::PDOMapping;

Channel::Channel(Driver& driver, int channel)
    : m_driver(driver)
    , m_channel(channel)
    , m_object_id_offset(channel * CHANNEL_OBJECT_ID_OFFSET) {

    for (auto& f : m_command_fields) {
        f = false;
    }
}

std::vector<canbus::Message> Channel::sendDS402Transition(
    ControlWord::Transition transition, bool enable_halt
) const {
    return vector<canbus::Message> {
        queryDownload<ControlWordRaw>(
            ControlWord(m_operation_mode, transition, enable_halt).toRaw()
        )
    };
}

std::vector<canbus::Message> Channel::queryDS402Status() const {
    return vector<canbus::Message> {
        queryUpload<StatusWordRaw>()
    };
}

StatusWord Channel::getDS402Status() const {
    return StatusWord::fromRaw(get<StatusWordRaw>());
}

void Channel::setFactors(Factors const& factors) {
    m_factors = factors;
}

bool Channel::updateJointStateTracking(canopen_master::StateMachine::Update const& update) {
    if (hasUpdatedObject<MotorAmps>(update)) {
        m_joint_state_tracking |= UPDATED_MOTOR_AMPS;
    }
    if (hasUpdatedObject<AppliedPowerLevel>(update)) {
        m_joint_state_tracking |= UPDATED_POWER_LEVEL;
    }
    if (hasUpdatedObject<ActualProfileVelocity>(update)) {
        m_joint_state_tracking |= UPDATED_ACTUAL_PROFILE_VELOCITY;
    }
    if (hasUpdatedObject<ActualVelocity>(update)) {
        m_joint_state_tracking |= UPDATED_ACTUAL_VELOCITY;
    }
    if (hasUpdatedObject<Position>(update)) {
        m_joint_state_tracking |= UPDATED_POSITION;
    }
    if (hasUpdatedObject<Torque>(update)) {
        m_joint_state_tracking |= UPDATED_TORQUE;
    }
    return hasJointStateUpdate();
}

bool Channel::hasJointStateUpdate() const {
    return (m_joint_state_tracking & m_joint_state_mask) == m_joint_state_mask;
}

void Channel::resetJointStateTracking() {
    m_joint_state_tracking = 0;
}

vector<PDOMapping> Channel::getJointStateTPDOMapping() const {
    PDOMapping mapping;
    mapping.add<MotorAmps>(0, m_channel);
    mapping.add<AppliedPowerLevel>(0, m_channel);

    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE:
            mapping.add<ActualProfileVelocity>(m_object_id_offset);
            return vector<PDOMapping> { mapping };
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY:
        case OPERATION_MODE_ANALOG_POSITION:
        case OPERATION_MODE_ANALOG_VELOCITY:
            mapping.add<ActualVelocity>(m_object_id_offset);
            return vector<PDOMapping> { mapping };
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE:
        case OPERATION_MODE_RELATIVE_POSITION:
            mapping.add<Position>(m_object_id_offset);
            return vector<PDOMapping> { mapping };
        case OPERATION_MODE_TORQUE_PROFILE: {
            return vector<PDOMapping> { mapping };
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }
}

vector<canbus::Message> Channel::queryJointState() const {
    vector<canbus::Message> messages;
    messages.push_back(queryUpload<MotorAmps>());
    messages.push_back(queryUpload<AppliedPowerLevel>());

    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE:
            messages.push_back(queryUpload<ActualProfileVelocity>());
            return messages;
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY:
        case OPERATION_MODE_ANALOG_POSITION:
        case OPERATION_MODE_ANALOG_VELOCITY:
            messages.push_back(queryUpload<ActualVelocity>());
            return messages;
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE:
        case OPERATION_MODE_RELATIVE_POSITION:
            messages.push_back(queryUpload<Position>());
            return messages;
        case OPERATION_MODE_TORQUE_PROFILE: {
            return messages;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }
}

JointState Channel::getJointState() const {
    JointState state;
    if (m_operation_mode == OPERATION_MODE_NONE) {
        return state;
    }

    state.effort = m_factors.currentToTorqueSI(get<MotorAmps>());
    state.raw = m_factors.pwmToFloat(get<AppliedPowerLevel>());

    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE:
            state.speed = m_factors.rpmToSI(get<ActualProfileVelocity>());
            return state;
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY:
            state.speed = m_factors.rpmToSI(get<ActualVelocity>());
            return state;
        case OPERATION_MODE_ANALOG_VELOCITY:
            state.speed = m_factors.relativeSpeedToSI(get<ActualVelocity>());
            return state;
        case OPERATION_MODE_ANALOG_POSITION:
            state.position = m_factors.relativePositionToSI(get<ActualVelocity>());
            return state;
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE:
        case OPERATION_MODE_RELATIVE_POSITION:
            state.position = m_factors.relativePositionToSI(get<Position>());
            return state;
        case OPERATION_MODE_TORQUE_PROFILE: {
            return state;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }
}

uint32_t Channel::getJointStateMask() const {
    if (m_operation_mode == OPERATION_MODE_NONE) {
        return 0;
    }

    uint32_t mask = UPDATED_POWER_LEVEL;
    mask |= UPDATED_MOTOR_AMPS;

    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE:
            return mask | UPDATED_ACTUAL_PROFILE_VELOCITY;
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY:
        case OPERATION_MODE_ANALOG_POSITION:
        case OPERATION_MODE_ANALOG_VELOCITY:
            return mask | UPDATED_ACTUAL_VELOCITY;
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE:
        case OPERATION_MODE_RELATIVE_POSITION:
            return mask | UPDATED_POSITION;
        case OPERATION_MODE_TORQUE_PROFILE: {
            return mask;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }
}

vector<canbus::Message> Channel::queryOperationModeDownload(
    OperationModes mode
) {
    int roboteq_mode = mode;
    if (mode == OPERATION_MODE_ANALOG_POSITION ||
        mode == OPERATION_MODE_ANALOG_VELOCITY) {
        roboteq_mode = OPERATION_MODE_VELOCITY;
    }

    return vector<canbus::Message>{
        m_driver.queryDownload<OperationMode>(roboteq_mode, m_object_id_offset)
    };
}

void Channel::setOperationMode(OperationModes mode) {
    m_operation_mode = mode;
    m_joint_state_tracking = 0;
    m_joint_state_mask = getJointStateMask();
}

OperationModes Channel::getOperationMode() const {
    return m_operation_mode;
}

double Channel::validateField(JointState::MODE i, base::JointState const& cmd) {
    double v = cmd.getField(i);
    if (base::isUnknown(v)) {
        throw InvalidJointCommand(
            "expected field is not set in joint command for channel "
            + to_string(m_channel)
        );
    }
    return v;
}

template<typename T> std::pair<int, int> Channel::getObjectOffsets() const {
    return std::make_pair(m_object_id_offset, 0);
}

template<> std::pair<int, int> Channel::getObjectOffsets<MotorAmps>() const {
    return std::make_pair(0, m_channel);
}

template<> std::pair<int, int> Channel::getObjectOffsets<SetSpeedTarget>() const {
    return std::make_pair(0, m_channel);
}

template<> std::pair<int, int> Channel::getObjectOffsets<AppliedPowerLevel>() const {
    return std::make_pair(0, m_channel);
}

template<typename T>
typename T::OBJECT_TYPE Channel::get() const {
    auto offsets = getObjectOffsets<T>();
    return m_driver.get<T>(offsets.first, offsets.second);
}

template<typename T>
void Channel::set(typename T::OBJECT_TYPE value) {
    auto offsets = getObjectOffsets<T>();
    return m_driver.set<T>(value, offsets.first, offsets.second);
}

template<typename T>
canbus::Message Channel::queryDownload(typename T::OBJECT_TYPE value) const {
    auto offsets = getObjectOffsets<T>();
    return m_driver.queryDownload<T>(value, offsets.first, offsets.second);
}

template<typename T>
canbus::Message Channel::queryUpload() const {
    auto offsets = getObjectOffsets<T>();
    return m_driver.queryUpload<T>(offsets.first, offsets.second);
}

template<typename T>
canbus::Message Channel::queryDownload() const {
    auto offsets = getObjectOffsets<T>();
    return m_driver.queryDownload<T>(m_driver.get<T>(offsets.first, offsets.second),
                                     offsets.first, offsets.second);
}

template<typename T>
bool Channel::hasUpdatedObject(canopen_master::StateMachine::Update const& update) const {
    auto offsets = getObjectOffsets<T>();
    return update.hasUpdatedObject<T>(offsets.first, offsets.second);
}

vector<PDOMapping> Channel::getJointCommandRPDOMapping() const {
    vector<PDOMapping> mappings;
    mappings.push_back(PDOMapping());

    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE: {
            mappings.back().add<TargetTorque>(m_object_id_offset);
            mappings.back().add<TargetProfileVelocity>(m_object_id_offset);
            mappings.push_back(PDOMapping());
            mappings.back().add<ProfileAcceleration>(m_object_id_offset);
            mappings.back().add<ProfileDeceleration>(m_object_id_offset);
            return mappings;
        }
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_ANALOG_POSITION:
        case OPERATION_MODE_ANALOG_VELOCITY:
        case OPERATION_MODE_VELOCITY: {
            mappings.back().add<TargetVelocity>(m_object_id_offset);
            return mappings;
        }
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE: {
            mappings.back().add<TargetPosition>(m_object_id_offset);
            mappings.back().add<ProfileVelocity>(m_object_id_offset);
            mappings.push_back(PDOMapping());
            mappings.back().add<ProfileAcceleration>(m_object_id_offset);
            mappings.back().add<ProfileDeceleration>(m_object_id_offset);
            return mappings;
        }
        case OPERATION_MODE_RELATIVE_POSITION: {
            mappings.back().add<TargetPosition>(m_object_id_offset);
            return mappings;
        }
        case OPERATION_MODE_TORQUE_PROFILE: {
            mappings.back().add<TargetTorque>(m_object_id_offset);
            mappings.back().add<TorqueSlope>(m_object_id_offset);
            return mappings;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }
}

base::JointState Channel::getJointCommand() const {
    return m_current_command;
}

void Channel::setJointCommand(base::JointState const& cmd) {
    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE: {
            double acceleration = validateField(JointState::ACCELERATION, cmd);
            uint32_t acceleration_roboteq = m_factors.rpmFromSI(acceleration) * 10;
            double effort = validateField(JointState::EFFORT, cmd);
            double velocity = validateField(JointState::SPEED, cmd);

            set<TargetTorque>(m_factors.currentFromTorqueSI(effort));
            set<ProfileAcceleration>(acceleration_roboteq);
            set<ProfileDeceleration>(acceleration_roboteq);
            set<TargetProfileVelocity>(m_factors.rpmFromSI(velocity));
            break;
        }
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY: {
            double velocity = validateField(JointState::SPEED, cmd);
            set<TargetVelocity>(m_factors.rpmFromSI(velocity));
            break;
        }
        case OPERATION_MODE_ANALOG_VELOCITY: {
            double velocity = validateField(JointState::SPEED, cmd);
            set<TargetVelocity>(m_factors.relativeSpeedFromSI(velocity));
            break;
        }
        case OPERATION_MODE_ANALOG_POSITION: {
            double position = validateField(JointState::POSITION, cmd);
            set<TargetVelocity>(m_factors.relativePositionFromSI(position));
            break;
        }
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE: {
            double velocity = validateField(JointState::SPEED, cmd);
            double acceleration = validateField(JointState::ACCELERATION, cmd);
            uint32_t acceleration_roboteq = m_factors.rpmFromSI(acceleration) * 10;
            double position = validateField(JointState::POSITION, cmd);

            set<TargetPosition>(m_factors.relativePositionFromSI(position));
            set<ProfileVelocity>(m_factors.rpmFromSI(velocity));
            set<ProfileAcceleration>(acceleration_roboteq);
            set<ProfileDeceleration>(acceleration_roboteq);
            break;
        }
        case OPERATION_MODE_RELATIVE_POSITION: {
            double position = validateField(JointState::POSITION, cmd);
            set<TargetPosition>(m_factors.relativePositionFromSI(position));
            break;
        }
        case OPERATION_MODE_TORQUE_PROFILE: {
            double effort = validateField(JointState::EFFORT, cmd);
            double effort_slope = validateField(JointState::RAW, cmd);
            set<TargetTorque>(m_factors.currentFromTorqueSI(effort));
            set<TorqueSlope>(m_factors.currentSlopeFromTorqueSlopeSI(effort_slope));
            break;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }

    m_current_command = cmd;
}

vector<canbus::Message> Channel::queryJointCommandDownload() const {
    switch (m_operation_mode) {
        case OPERATION_MODE_VELOCITY_POSITION_PROFILE:
        case OPERATION_MODE_VELOCITY_PROFILE:
            return vector<canbus::Message> {
                queryDownload<TargetTorque>(),
                queryDownload<ProfileAcceleration>(),
                queryDownload<ProfileDeceleration>(),
                queryDownload<TargetProfileVelocity>()
            };
        case OPERATION_MODE_VELOCITY_POSITION:
        case OPERATION_MODE_VELOCITY:
            return vector<canbus::Message> {
                queryDownload<TargetVelocity>()
            };
        case OPERATION_MODE_ANALOG_POSITION:
        case OPERATION_MODE_ANALOG_VELOCITY:
            return vector<canbus::Message> {
                queryDownload<TargetVelocity>()
            };
        case OPERATION_MODE_RELATIVE_POSITION_PROFILE:
            return vector<canbus::Message> {
                queryDownload<TargetPosition>(),
                queryDownload<ProfileVelocity>(),
                queryDownload<ProfileAcceleration>(),
                queryDownload<ProfileDeceleration>()
            };
        case OPERATION_MODE_RELATIVE_POSITION:
            return vector<canbus::Message> {
                queryDownload<TargetPosition>()
            };
        case OPERATION_MODE_TORQUE_PROFILE:
            return vector<canbus::Message> {
                queryDownload<TargetTorque>(),
                queryDownload<TorqueSlope>()
            };
        default:
            throw invalid_argument("unsupported operation mode");
    }
}