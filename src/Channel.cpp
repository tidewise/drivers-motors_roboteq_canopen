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
    if (hasUpdatedObject<EncoderCounter>(update)) {
        m_joint_state_tracking |= UPDATED_ENCODER;
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

    vector<PDOMapping> mappings;

    PDOMapping mapping0;
    mapping0.add<MotorAmps>(0, m_channel);
    mapping0.add<AppliedPowerLevel>(0, m_channel);
    mapping0.add<ChannelStatusFlagsRaw>(0, m_channel);
    mappings.push_back(mapping0);

    PDOMapping mapping1;
    if (jointStateNeedsFeedback()) {
        mapping1.add<Feedback>(0, m_channel);
    }
    if (jointStateNeedsEncoder()) {
        mapping1.add<EncoderCounter>(0, m_channel);
    }
    if (!mapping1.empty()) {
        mappings.push_back(mapping1);
    }

    return mappings;
}

bool Channel::jointStateNeedsFeedback() const
{
    return (getSpeedObject() == SPEED_OBJECT_FEEDBACK ||
            getPositionObject() == POSITION_OBJECT_FEEDBACK);
}

bool Channel::jointStateNeedsEncoder() const
{
    return (getPositionObject() == POSITION_OBJECT_ENCODER);
}

Channel::SpeedObject Channel::getSpeedObject() const
{
    switch (m_control_mode) {
        case CONTROL_SPEED:
        case CONTROL_SPEED_POSITION:
            return SPEED_OBJECT_FEEDBACK;
        default:
            return SPEED_OBJECT_NONE;
    }
}

Channel::PositionObject Channel::getPositionObject() const
{
    switch (m_joint_state_position_source) {
        case JOINT_STATE_POSITION_SOURCE_NONE:
            return POSITION_OBJECT_NONE;
        case JOINT_STATE_POSITION_SOURCE_ENCODER:
            return POSITION_OBJECT_ENCODER;
        case JOINT_STATE_POSITION_SOURCE_AUTO:
            if (m_control_mode == CONTROL_POSITION ||
                m_control_mode == CONTROL_PROFILED_POSITION) {
                return POSITION_OBJECT_FEEDBACK;
            }
            return POSITION_OBJECT_NONE;
        default:
            throw std::invalid_argument(
                "joint state position source unsupported by getPositionObject");
    }
}

vector<canbus::Message> Channel::queryJointState() const
{
    vector<canbus::Message> messages;
    if (isIgnored()) {
        return messages;
    }

    messages.push_back(queryUpload<MotorAmps>());
    messages.push_back(queryUpload<AppliedPowerLevel>());

    if (jointStateNeedsFeedback()) {
        messages.push_back(queryUpload<Feedback>());
    }
    if (jointStateNeedsEncoder()) {
        messages.push_back(queryUpload<EncoderCounter>());
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

    auto position_object = getPositionObject();
    if (position_object == POSITION_OBJECT_FEEDBACK) {
        state.position = m_factors.relativePositionToSI(get<Feedback>());
    }
    else if (position_object == POSITION_OBJECT_ENCODER) {
        state.position = m_factors.encoderToSI(get<EncoderCounter>());
    }

    auto speed_object = getSpeedObject();
    if (speed_object == SPEED_OBJECT_FEEDBACK) {
        state.speed = m_factors.relativeSpeedToSI(get<Feedback>());
    }

    return state;
}

uint32_t Channel::getJointStateMask() const {
    if (isIgnored()) {
        return 0;
    }

    uint32_t mask = UPDATED_POWER_LEVEL | UPDATED_MOTOR_AMPS;

    if (jointStateNeedsFeedback()) {
        mask |= UPDATED_FEEDBACK;
    }
    if (jointStateNeedsEncoder()) {
        mask |= UPDATED_ENCODER;
    }

    return mask;
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

    return vector<canbus::Message>{queryDownload<SetCommand>()};
}

void Channel::setJointStatePositionSource(JointStatePositionSources source) {
    m_joint_state_position_source = source;
    m_joint_state_tracking = 0;
    m_joint_state_mask = getJointStateMask();
}