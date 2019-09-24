rock_library(motors_roboteq_canopen
    SOURCES Driver.cpp Factors.cpp Driver.cpp Channel.cpp Objects.cpp
    HEADERS Driver.hpp Factors.hpp Driver.hpp Channel.hpp Objects.hpp
            ControllerStatus.hpp Exceptions.hpp
    DEPS_PKGCONFIG base-types canopen_master)

rock_executable(motors_roboteq_canopen_bin Main.cpp
    DEPS motors_roboteq_canopen)

