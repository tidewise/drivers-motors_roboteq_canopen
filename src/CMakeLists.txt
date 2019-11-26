rock_library(motors_roboteq_canopen
    SOURCES Driver.cpp DS402Driver.cpp Factors.cpp ChannelBase.cpp DS402Channel.cpp Objects.cpp
    HEADERS Driver.hpp DS402Driver.hpp Factors.hpp ChannelBase.hpp DS402Channel.hpp Objects.hpp
            ControllerStatus.hpp Exceptions.hpp
    DEPS_PKGCONFIG base-types canopen_master)

rock_executable(motors_roboteq_canopen_bin Main.cpp
    DEPS motors_roboteq_canopen)

