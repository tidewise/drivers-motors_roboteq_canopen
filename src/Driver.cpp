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