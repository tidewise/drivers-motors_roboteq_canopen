#ifndef MOTORS_ROBOTEQ_CANOPEN_EXCEPTIONS_HPP
#define MOTORS_ROBOTEQ_CANOPEN_EXCEPTIONS_HPP

#include <stdexcept>

namespace motors_roboteq_canopen {
    /** Exception thrown when receiving a joint command that is not
     * valid for the current operation mode
     */
    struct InvalidJointCommand : std::runtime_error {
        using std::runtime_error::runtime_error;
    };
}

#endif