rock_library(motors_roboteq_canopen
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motors_roboteq_canopen_bin Main.cpp
    DEPS motors_roboteq_canopen)

