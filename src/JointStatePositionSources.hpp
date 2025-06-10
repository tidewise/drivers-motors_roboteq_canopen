#ifndef MOTORS_ROBOTEQ_CANOPEN_JOINTSTATESOURCES_HPP
#define MOTORS_ROBOTEQ_CANOPEN_JOINTSTATESOURCES_HPP

#include <cstdint>

namespace motors_roboteq_canopen {
    /** Sources for the joint position state */
    enum JointStatePositionSources {
        /**  */
        JOINT_STATE_POSITION_SOURCE_NONE,

        /** Use the Feedback as a joint position source */
        JOINT_STATE_POSITION_SOURCE_AUTO,

        /** Use the EncoderCounter as a joint position source */
        JOINT_STATE_POSITION_SOURCE_ENCODER
    };
}

#endif
