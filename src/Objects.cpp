#include <motors_roboteq_canopen/Objects.hpp>

using namespace motors_roboteq_canopen;

uint16_t ControlWord::toRaw() const {
    uint16_t word = 0;
    switch(transition)
    {
        case SHUTDOWN:
            word = 0x06;
            break;
        case SWITCH_ON:
            word = 0x07;
            break;
        case ENABLE_OPERATION:
            word = 0x0F;
            break;
        case DISABLE_VOLTAGE:
            word = 0x00;
            break;
        case QUICK_STOP:
            word = 0x02;
            break;
        case DISABLE_OPERATION:
            word = 0x07;
            break;
        case FAULT_RESET:
            word = 0x80;
            break;
    }

    switch(operation_mode) {
        case DS402_OPERATION_MODE_POSITION_PROFILE:
        case DS402_OPERATION_MODE_RELATIVE_POSITION_PROFILE:
            word |= (1 << 5);
            break;
        case DS402_OPERATION_MODE_ANALOG_VELOCITY:
        case DS402_OPERATION_MODE_ANALOG_POSITION:
        case DS402_OPERATION_MODE_VELOCITY:
        case DS402_OPERATION_MODE_VELOCITY_POSITION:
            word |= 0x70; // enable all ramps
            break;
        default:
            break;
    }

    if (enable_halt)
        word |= 0x100;

    return word;
}

StatusWord::State parseState(uint8_t byte)
{
    switch(byte & 0x4F)
    {
        case 0x00: return StatusWord::NOT_READY_TO_SWITCH_ON;
        case 0x40: return StatusWord::SWITCH_ON_DISABLED;
        case 0x0F: return StatusWord::FAULT_REACTION_ACTIVE;
        case 0x08: return StatusWord::FAULT;
    }

    switch(byte & 0x6F)
    {
        case 0x21: return StatusWord::READY_TO_SWITCH_ON;
        case 0x23: return StatusWord::SWITCH_ON;
        case 0x27: return StatusWord::OPERATION_ENABLED;
        case 0x07: return StatusWord::QUICK_STOP_ACTIVE;
    }

    throw StatusWord::UnknownState("received an unknown value for the state");
}


StatusWord StatusWord::fromRaw(uint16_t raw) {
    StatusWord::State state = parseState(raw & 0x6F);
    bool voltageEnabled = (raw & 0x0010);
    bool warning        = (raw & 0x0080);
    bool targetReached  = (raw & 0x0400);
    bool internalLimitActive = (raw & 0x0800);
    return StatusWord { raw, state, voltageEnabled, warning,
        targetReached, internalLimitActive };
}

