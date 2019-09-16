#include <boost/test/unit_test.hpp>
#include <motors_roboteq_canopen/Dummy.hpp>

using namespace motors_roboteq_canopen;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motors_roboteq_canopen::DummyClass dummy;
    dummy.welcome();
}
