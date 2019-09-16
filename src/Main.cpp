#include <iostream>
#include <motors_roboteq_canopen/Dummy.hpp>

int main(int argc, char** argv)
{
    motors_roboteq_canopen::DummyClass dummyClass;
    dummyClass.welcome();

    return 0;
}
