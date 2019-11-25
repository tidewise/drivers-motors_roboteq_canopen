#include <motors_roboteq_canopen/Driver.hpp>
#include <motors_roboteq_canopen/Objects.hpp>

using namespace std;
using namespace base;
using canopen_master::PDOMapping;
using namespace motors_roboteq_canopen;

Driver::Driver(canopen_master::StateMachine& state_machine, int channel_count)
    : canopen_master::Slave(state_machine) {

    if (channel_count > MAX_CHANNEL_COUNT) {
        throw std::invalid_argument("this driver only supports up to 4 channels");
    }

    for (int i = 0; i < channel_count; ++i) {
        m_channels.push_back(Channel(*this, i));
    }

    state_machine.setQuirks(
        canopen_master::StateMachine::PDO_COBID_MESSAGE_RESERVED_BIT_QUIRK
    );
}

canopen_master::StateMachine::Update Driver::process(canbus::Message const& message) {
    auto update = canopen_master::Slave::process(message);
    for (auto& c : m_channels) {
        c.updateJointStateTracking(update);
    }
    return update;
}

vector<canbus::Message> Driver::queryControllerStatus() {
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
    }
    return queries;
}

ControllerStatus Driver::getControllerStatus() const {
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
    return status;
}

Channel& Driver::getChannel(int i) {
    return m_channels.at(i);
}

int Driver::setupJointStateTPDOs(std::vector<canbus::Message>& messages,
    int pdoStartIndex, canopen_master::PDOCommunicationParameters const& parameters
) {
    int pdoIndex = pdoStartIndex;
    for (auto const& channel : m_channels) {
        vector<PDOMapping> mappings = channel.getJointStateTPDOMapping();
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

int Driver::setupJointCommandRPDOs(std::vector<canbus::Message>& messages,
    int pdoStartIndex, canopen_master::PDOCommunicationParameters const& parameters
) {
    int pdoIndex = pdoStartIndex;
    for (auto const& channel : m_channels) {
        vector<PDOMapping> mappings = channel.getJointCommandRPDOMapping();
        for (size_t i = 0; i < mappings.size(); ++i, ++pdoIndex) {
            auto msgs = mCANOpen.configurePDO(
                false, pdoIndex, parameters, mappings[i]
            );
            mCANOpen.declareRPDOMapping(pdoIndex, mappings[i]);
            messages.insert(messages.end(), msgs.begin(), msgs.end());
        }
    }
    return pdoIndex;
}

int Driver::setupStatusTPDOs(std::vector<canbus::Message>& messages,
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
void Driver::setJointCommand(base::samples::Joints const& command) {
    size_t i = 0;
    for (auto& channel : m_channels) {
        if (channel.getOperationMode() == OPERATION_MODE_NONE) {
            continue;
        }
        else if (command.elements.size() <= i) {
            throw std::invalid_argument("too few elements in joint command");
        }

        channel.setJointCommand(command.elements[i]);
        ++i;
    }
    if (i != command.elements.size()) {
        throw std::invalid_argument("too many elements in joint command");
    }
}

base::samples::Joints Driver::getJointCommand() const {
    base::samples::Joints command;

    for (auto& channel : m_channels) {
        if (channel.getOperationMode() == OPERATION_MODE_NONE) {
            continue;
        }

        command.elements.push_back(channel.getJointCommand());
    }
    return command;
}

