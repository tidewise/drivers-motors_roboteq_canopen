#include <motors_roboteq_canopen/Channel.hpp>
#include <motors_roboteq_canopen/Driver.hpp>

using namespace std;
using namespace base;
using namespace motors_roboteq_canopen;

using canopen_master::PDOMapping;

Channel::Channel(Driver& driver, int channel)
    : m_driver(driver)
    , m_channel(channel) {
}

bool Channel::isIgnored() const {
    return m_control_mode == CONTROL_IGNORED;
}

canbus::Message Channel::queryMotorStop() const {
    return m_driver.queryDownload<MotorStop>(m_channel + 1);
}

bool Channel::updateJointStateTracking(canopen_master::StateMachine::Update const& update) {
    if (hasUpdatedObject<MotorAmps>(update)) {
        m_joint_state_tracking |= UPDATED_MOTOR_AMPS;
    }
    if (hasUpdatedObject<AppliedPowerLevel>(update)) {
        m_joint_state_tracking |= UPDATED_POWER_LEVEL;
    }
    if (hasUpdatedObject<Feedback>(update)) {
        m_joint_state_tracking |= UPDATED_FEEDBACK;
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
    if (isIgnored()) {
        return vector<PDOMapping>();
    }

    PDOMapping mapping;
    mapping.add<MotorAmps>(0, m_channel);
    mapping.add<AppliedPowerLevel>(0, m_channel);
    if (m_control_mode != CONTROL_OPEN_LOOP) {
        mapping.add<Feedback>(0, m_channel);
    }
    mapping.add<ChannelStatusFlagsRaw>(0, m_channel);
    return vector<PDOMapping> { mapping };
}

vector<canbus::Message> Channel::queryJointState() const {
    vector<canbus::Message> messages;
    if (isIgnored()) {
        return messages;
    }

    messages.push_back(queryUpload<MotorAmps>());
    messages.push_back(queryUpload<AppliedPowerLevel>());
    if (m_control_mode != CONTROL_OPEN_LOOP) {
        messages.push_back(queryUpload<Feedback>());
    }
    return messages;
}

JointState Channel::getJointState() const {
    JointState state;
    if (isIgnored()) {
        return state;
    }

    state.effort = m_factors.currentToTorqueSI(get<MotorAmps>());
    state.raw = m_factors.pwmToFloat(get<AppliedPowerLevel>());

    switch (m_control_mode) {
        case CONTROL_NONE:
        case CONTROL_OPEN_LOOP:
            return state;
        case CONTROL_SPEED:
        case CONTROL_SPEED_POSITION: {
            int32_t feedback = get<Feedback>();
            state.speed = m_factors.relativeSpeedToSI(feedback);
            return state;
        }
        case CONTROL_PROFILED_POSITION:
        case CONTROL_POSITION: {
            int32_t feedback = get<Feedback>();
            state.position = m_factors.relativePositionToSI(feedback);
            return state;
        }
        case CONTROL_TORQUE:
            return state;
        default:
            throw std::invalid_argument("unexpected mode");
    }
}

uint32_t Channel::getJointStateMask() const {
    if (isIgnored()) {
        return 0;
    }

    uint32_t mask = UPDATED_POWER_LEVEL | UPDATED_MOTOR_AMPS;

    switch (m_control_mode) {
        case CONTROL_NONE:
        case CONTROL_OPEN_LOOP:
            return mask;
        default:
            return mask | UPDATED_FEEDBACK;
    }
}

void Channel::setControlMode(ControlModes mode) {
    m_control_mode = mode;
    m_joint_state_tracking = 0;
    m_joint_state_mask = getJointStateMask();
}

ControlModes Channel::getControlMode() const {
    return m_control_mode;
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

template<typename T>
typename T::OBJECT_TYPE Channel::get() const {
    return m_driver.get<T>(0, m_channel);
}

template<typename T>
void Channel::set(typename T::OBJECT_TYPE value) {
    return m_driver.set<T>(value, 0, m_channel);
}

template<typename T>
canbus::Message Channel::queryDownload(typename T::OBJECT_TYPE value) const {
    return m_driver.queryDownload<T>(value, 0, m_channel);
}

template<typename T>
canbus::Message Channel::queryUpload() const {
    return m_driver.queryUpload<T>(0, m_channel);
}

template<typename T>
canbus::Message Channel::queryDownload() const {
    return m_driver.queryDownload<T>(m_driver.get<T>(0, m_channel),
                                     0, m_channel);
}

template<typename T>
bool Channel::hasUpdatedObject(canopen_master::StateMachine::Update const& update) const {
    return update.hasUpdatedObject<T>(0, m_channel);
}

vector<PDOMapping> Channel::getJointCommandRPDOMapping() const {
    vector<PDOMapping> mappings;
    if (isIgnored() || m_control_mode == CONTROL_NONE) {
        return mappings;
    }

    PDOMapping mapping;
    mapping.add<SetCommand>(0, m_channel);
    mappings.push_back(mapping);
    return mappings;
}

base::JointState Channel::getJointCommand() const {
    return m_current_command;
}

void Channel::setJointCommand(base::JointState const& cmd) {
    switch (m_control_mode) {
        case CONTROL_IGNORED:
        case CONTROL_NONE:
            break;

        case CONTROL_OPEN_LOOP: {
            float raw = validateField(JointState::RAW, cmd);
            set<SetCommand>(Factors::clamp1000(raw * 1000));
            break;
        }

        case CONTROL_SPEED:
        case CONTROL_SPEED_POSITION: {
            double speed = validateField(JointState::SPEED, cmd);
            set<SetCommand>(m_factors.relativeSpeedFromSI(speed));
            break;
        }

        case CONTROL_PROFILED_POSITION:
        case CONTROL_POSITION: {
            double position = validateField(JointState::POSITION, cmd);
            set<SetCommand>(m_factors.relativePositionFromSI(position));
            break;
        }

        case CONTROL_TORQUE: {
            double torque = validateField(JointState::EFFORT, cmd);
            set<SetCommand>(m_factors.relativeTorqueFromSI(torque));
            break;
        }
        default:
            throw invalid_argument("unsupported operation mode");
    }

    m_current_command = cmd;
}

vector<canbus::Message> Channel::queryJointCommandDownload() const {
    if (isIgnored() || m_control_mode == CONTROL_NONE) {
        return vector<canbus::Message>();
    }

    return vector<canbus::Message> { queryDownload<SetCommand>() };
}