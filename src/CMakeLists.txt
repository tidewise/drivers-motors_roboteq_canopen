rock_library(motors_roboteq_canopen
    SOURCES DS402Driver.cpp Factors.cpp DS402Channel.cpp Objects.cpp
    HEADERS DS402Driver.hpp Factors.hpp DS402Channel.hpp Objects.hpp
            ControllerStatus.hpp Exceptions.hpp
    DEPS_PKGCONFIG base-types canopen_master)

rock_executable(motors_roboteq_canopen_bin Main.cpp
    DEPS motors_roboteq_canopen)

