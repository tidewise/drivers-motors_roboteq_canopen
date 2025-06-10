#include <motors_roboteq_canopen/DriverBase.hpp>
#include <motors_roboteq_canopen/Objects.hpp>

using namespace std;
using namespace base;
using canopen_master::PDOMapping;
using canopen_master::PDOCommunicationParameters;
using namespace motors_roboteq_canopen;

DriverBase::DriverBase(canopen_master::StateMachine& state_machine)
    : canopen_master::Slave(state_machine) {

    state_machine.setQuirks(
        canopen_master::StateMachine::PDO_COBID_MESSAGE_RESERVED_BIT_QUIRK
    );
}

DriverBase::~DriverBase() {
    for (auto c : m_channels) {
        delete c;
    }
}

size_t DriverBase::getChannelCount() const {
    return m_channels.size();
}

void DriverBase::addChannel(ChannelBase* channel) {
    m_channels.push_back(channel);
}

canopen_master::StateMachine::Update DriverBase::process(canbus::Message const& message) {
    auto update = canopen_master::Slave::process(message);
    for (auto c : m_channels) {
        c->updateJointStateTracking(update);
    }
    for (auto single_update : update) {
        if (single_update.first == AnalogInput::OBJECT_ID) {
            m_received_analog_inputs_mask |= 1 << (single_update.second - 1);
        }
        else if (single_update.first == ConvertedAnalogInput::OBJECT_ID) {
            m_received_converted_analog_inputs_mask |= 1 << (single_update.second - 1);
        }
        else if (single_update.first == EncoderCounter::OBJECT_ID) {
            m_received_encoder_counter_mask |= 1 << (single_update.second - 1);
        }
    }
    return update;
}

bool DriverBase::hasAnalogInputUpdate() const {
    return m_expected_analog_inputs_mask == m_received_analog_inputs_mask;
}

bool DriverBase::hasEncoderCounterUpdate() const {
    return m_expected_encoder_counter_mask == m_received_encoder_counter_mask;
}

void DriverBase::setAnalogInputEnableInTPDO(int index, bool enable) {
    if (enable) {
        m_expected_analog_inputs_mask |= 1 << index;
    }
    else {
        m_expected_analog_inputs_mask &= ~(1 << index);
    }
}

void DriverBase::setEncoderCounterEnableInTPDO(int index, bool enable) {
    if (enable) {
        m_expected_encoder_counter_mask |= 1 << index;
    }
    else {
        m_expected_encoder_counter_mask &= ~(1 << index);
    }
}

void DriverBase::resetAnalogInputTracking() {
    m_received_analog_inputs_mask = 0;
}

void DriverBase::resetEncoderCounterTracking() {
    m_received_encoder_counter_mask = 0;
}

bool DriverBase::hasConvertedAnalogInputUpdate() const {
    return m_expected_converted_analog_inputs_mask ==
           m_received_converted_analog_inputs_mask;
}

void DriverBase::setConvertedAnalogInputEnableInTPDO(int index, bool enable) {
    if (enable) {
        m_expected_converted_analog_inputs_mask |= 1 << index;
    }
    else {
        m_expected_converted_analog_inputs_mask &= ~(1 << index);
    }
}

void DriverBase::resetConvertedAnalogInputTracking() {
    m_received_converted_analog_inputs_mask = 0;
}

canbus::Message DriverBase::queryAnalogInput(int index) const {
    return queryUpload<AnalogInput>(0, index + 1);
}

canbus::Message DriverBase::queryEncoderCounter(int index) const {
    return queryUpload<EncoderCounter>(0, index + 1);
}

canbus::Message DriverBase::queryAnalogInputConverted(int index) const {
    return queryUpload<ConvertedAnalogInput>(0, index + 1);
}

vector<canbus::Message> DriverBase::queryControllerStatus() {
    vector<canbus::Message> queries{
        queryUpload<VoltageInternal>(),
        queryUpload<VoltageBattery>(),
        queryUpload<Voltage5V>(),
        queryUpload<StatusFlagsRaw>(),
        queryUpload<FaultFlagsRaw>(),
        queryUpload<TemperatureMCU>()
    };
    for (size_t i = 0; i < m_channels.size(); ++i) {
        queries.push_back(queryUpload<TemperatureSensor0>(0, i));
        queries.push_back(queryUpload<ChannelStatusFlagsRaw>(0, i));
    }
    return queries;
}

ControllerStatus DriverBase::getControllerStatus() const {
    ControllerStatus status;
    status.voltage_internal = static_cast<float>(get<VoltageInternal>()) / 10;
    status.voltage_battery = static_cast<float>(get<VoltageBattery>()) / 10;
    status.voltage_5v = static_cast<float>(get<Voltage5V>()) / 1000;
    status.temperature_mcu = Temperature::fromCelsius(get<TemperatureMCU>());
    for (size_t i = 0; i < m_channels.size(); ++i) {
        status.temperature_sensors.push_back(
            Temperature::fromCelsius(get<TemperatureSensor0>(0, i))
        );
    }
    status.status_flags = get<StatusFlagsRaw>();
    status.fault_flags = get<FaultFlagsRaw>();

    status.channel_status_flags.resize(m_channels.size());
    for (size_t i = 0; i < m_channels.size(); ++i) {
        status.channel_status_flags[i] = get<ChannelStatusFlagsRaw>(0, i);
    }
    return status;
}

ChannelBase& DriverBase::getChannel(int i) {
    return *m_channels.at(i);
}

int DriverBase::setupJointStateTPDOs(vector<canbus::Message>& messages,
    int pdoStartIndex, PDOCommunicationParameters const& parameters
) {
    int pdoIndex = pdoStartIndex;
    for (auto channel : m_channels) {
        vector<PDOMapping> mappings = channel->getJointStateTPDOMapping();
        for (size_t i = 0; i < mappings.size(); ++i, ++pdoIndex) {
            auto msgs = mCANOpen.configurePDO(
                true, pdoIndex, parameters, mappings[i]
            );
            mCANOpen.declareTPDOMapping(pdoIndex, mappings[i]);
            messages.insert(messages.end(), msgs.begin(), msgs.end());
        }
    }
    return pdoIndex;
}

int DriverBase::setupJointCommandRPDOs(vector<canbus::Message>& messages,
    int pdoStartIndex, PDOCommunicationParameters const& parameters
) {
    int pdoIndex = pdoStartIndex;
    for (auto const channel : m_channels) {
        vector<PDOMapping> mappings = channel->getJointCommandRPDOMapping();
        for (size_t i = 0; i < mappings.size(); ++i, ++pdoIndex) {
            auto msgs = mCANOpen.configurePDO(
                false, pdoIndex, parameters, mappings[i]
            );
            mCANOpen.declareRPDOMapping(pdoIndex, mappings[i]);
            messages.insert(messages.end(), msgs.begin(), msgs.end());
        }
    }
    m_rpdo_begin = pdoStartIndex;
    m_rpdo_end = pdoIndex;
    return pdoIndex;
}

int DriverBase::setupTPDO(
    PDOMapping mapping, vector<canbus::Message>& messages, int pdoIndex,
    PDOCommunicationParameters const& parameters
) {
    auto msgs = mCANOpen.configurePDO(true, pdoIndex, parameters, mapping);
    messages.insert(messages.end(), msgs.begin(), msgs.end());
    mCANOpen.declareTPDOMapping(pdoIndex, mapping);
    return pdoIndex + 1;
}

int DriverBase::setupAnalogTPDOsInternal(
    uint32_t const mask, int objectOffset, std::vector<canbus::Message>& messages,
    int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters
) {
    bool first = true;
    PDOMapping mapping;
    for (int i = 0; i < 32; ++i) {
        if (!(mask & (1 << i))) {
            continue;
        }

        mapping.add<AnalogInput>(objectOffset, i + 1);
        if (!first) {
            pdoIndex = setupTPDO(mapping, messages, pdoIndex, parameters);
            mapping = PDOMapping();
        }

        first = !first;
    }

    if (!first) {
        pdoIndex = setupTPDO(mapping, messages, pdoIndex, parameters);
    }

    return pdoIndex;
}

int DriverBase::setupAnalogTPDOs(std::vector<canbus::Message>& messages,
    int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters) {
    pdoIndex = setupAnalogTPDOsInternal(
        m_expected_analog_inputs_mask, 0, messages,
        pdoIndex, parameters
    );
    pdoIndex = setupAnalogTPDOsInternal(
        m_expected_converted_analog_inputs_mask, 1, messages,
        pdoIndex, parameters
    );
    return pdoIndex;
}

int DriverBase::setupEncoderTPDOs(std::vector<canbus::Message>& messages,
    int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters) {
    bool first = true;
    PDOMapping mapping;
    for (int i = 0; i < 32; ++i) {
        if (!(m_expected_encoder_counter_mask & (1 << i))) {
            continue;
        }

        mapping.add<EncoderCounter>(0, i + 1);
        if (!first) {
            pdoIndex = setupTPDO(mapping, messages, pdoIndex, parameters);
            mapping = PDOMapping();
        }

        first = !first;
    }

    if (!first) {
        pdoIndex = setupTPDO(mapping, messages, pdoIndex, parameters);
    }

    return pdoIndex;
}

int DriverBase::setupStatusTPDOs(std::vector<canbus::Message>& messages,
    int pdoIndex, canopen_master::PDOCommunicationParameters const& parameters
) {
    {
        PDOMapping mapping;
        mapping.add<StatusFlagsRaw>();
        mapping.add<FaultFlagsRaw>();
        mapping.add<VoltageInternal>();
        mapping.add<VoltageBattery>();
        auto msgs = mCANOpen.configurePDO(true, pdoIndex, parameters, mapping);
        messages.insert(messages.end(), msgs.begin(), msgs.end());
        mCANOpen.declareTPDOMapping(pdoIndex, mapping);
    }

    {
        PDOMapping mapping;
        mapping.add<Voltage5V>();
        mapping.add<TemperatureMCU>();
        for (size_t i = 0; i < m_channels.size(); ++i) {
            mapping.add<TemperatureSensor0>(0, i);
        }

        auto msgs = mCANOpen.configurePDO(true, pdoIndex + 1, parameters, mapping);
        messages.insert(messages.end(), msgs.begin(), msgs.end());
        mCANOpen.declareTPDOMapping(pdoIndex + 1, mapping);
    }
    return pdoIndex + 2;
}

void DriverBase::setJointCommand(base::samples::Joints const& command) {
    size_t i = 0;
    for (auto channel : m_channels) {
        if (channel->isIgnored()) {
            continue;
        }
        else if (command.elements.size() <= i) {
            throw std::invalid_argument("too few elements in joint command");
        }

        channel->setJointCommand(command.elements[i]);
        ++i;
    }
    if (i != command.elements.size()) {
        throw std::invalid_argument("too many elements in joint command");
    }
}

base::samples::Joints DriverBase::getJointCommand() const {
    base::samples::Joints command;

    for (auto channel : m_channels) {
        if (channel->isIgnored()) {
            continue;
        }

        command.elements.push_back(channel->getJointCommand());
    }
    return command;
}

std::vector<canbus::Message> DriverBase::getRPDOMessages() const {
    std::vector<canbus::Message> messages;
    for (int i = m_rpdo_begin; i != m_rpdo_end; ++i) {
        messages.push_back(mCANOpen.getRPDOMessage(i));
    }
    return messages;
}

std::vector<canbus::Message> DriverBase::queryJointCommandDownload() const {
    std::vector<canbus::Message> messages;
    for (auto channel : m_channels) {
        auto const& channel_msgs = channel->queryJointCommandDownload();
        messages.insert(messages.end(), channel_msgs.begin(), channel_msgs.end());
    }
    return messages;
}
