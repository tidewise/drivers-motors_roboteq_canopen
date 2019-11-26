#include <motors_roboteq_canopen/ChannelBase.hpp>

using namespace motors_roboteq_canopen;

ChannelBase::~ChannelBase() {
}

void ChannelBase::setFactors(Factors const& factors) {
    m_factors = factors;
}