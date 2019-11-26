#include <motors_roboteq_canopen/DS402Driver.hpp>
#include <motors_roboteq_canopen/Objects.hpp>

using namespace std;
using namespace base;
using canopen_master::PDOMapping;
using namespace motors_roboteq_canopen;

DS402Driver::DS402Driver(canopen_master::StateMachine& state_machine, int channel_count)
    : Driver(state_machine, channel_count) {

    for (int i = 0; i < channel_count; ++i) {
        addChannel(new DS402Channel(*this, i));
    }
}

DS402Channel& DS402Driver::getChannel(int i) {
    return static_cast<DS402Channel&>(Driver::getChannel(i));
}