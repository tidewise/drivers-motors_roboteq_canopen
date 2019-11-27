rock_library(motors_roboteq_canopen
    SOURCES DriverBase.cpp Driver.cpp DS402Driver.cpp
            ChannelBase.cpp Channel.cpp DS402Channel.cpp
            Factors.cpp Objects.cpp
    HEADERS DriverBase.hpp Driver.hpp DS402Driver.hpp
            ChannelBase.hpp Channel.hpp DS402Channel.hpp
            Factors.hpp Objects.hpp
            ControllerStatus.hpp Exceptions.hpp
    DEPS_PKGCONFIG base-types canopen_master)

rock_executable(motors_roboteq_canopen_bin Main.cpp
    DEPS motors_roboteq_canopen)

