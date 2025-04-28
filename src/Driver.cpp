#include <motors_roboteq_canopen/Driver.hpp>
#include <motors_roboteq_canopen/Channel.hpp>

using namespace std;
using namespace motors_roboteq_canopen;

Driver::Driver(canopen_master::StateMachine& state_machine, int channel_count)
    : DriverBase(state_machine) {
    for (int i = 0; i < channel_count; ++i) {
        addChannel(new Channel(*this, i));
    }
}

Channel& Driver::getChannel(int i) {
    return static_cast<Channel&>(DriverBase::getChannel(i));
}

vector<canbus::Message> Driver::queryMotorStop() {
    vector<canbus::Message> messages;
    for (size_t i = 0; i < getChannelCount(); ++i) {
        messages.push_back(getChannel(i).queryMotorStop());
    }
    return messages;
}

canbus::Message Driver::queryWriteDigitalOutput(std::uint8_t index,
    bool out)
{
    if (out) {
        return queryDownload<ActivateDigitalOutput>(index);
    }

    return queryDownload<ResetDigitalOutput>(index);
}

canbus::Message Driver::queryReadDigitalOutput()
{
    return queryUpload<ReadAllDigitalOutput>();
}

std::uint16_t Driver::readDigitalOutputRaw()
{
    return get<ReadAllDigitalOutput>();
}

std::vector<raw_io::Digital> Driver::readDigitalOutput(
    std::vector<std::uint8_t> const& managed_outputs)
{
    std::uint16_t reading = readDigitalOutputRaw();
    return parseDigitalOutput(reading, managed_outputs);
}

std::vector<raw_io::Digital> Driver::parseDigitalOutput(std::uint16_t reading,
    std::vector<std::uint8_t> const& managed_outputs)
{
    std::vector<raw_io::Digital> out;
    out.reserve(managed_outputs.size());

    base::Time now = base::Time::now();

    for (std::uint8_t output : managed_outputs) {
        std::uint8_t index = output - 1;
        out.emplace_back(now, reading & (1 << index));
    }

    return out;
}