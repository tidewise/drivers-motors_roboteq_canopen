rock_library(motors_roboteq_canopen
    SOURCES DriverBase.cpp Driver.cpp DS402Driver.cpp
            ChannelBase.cpp Channel.cpp DS402Channel.cpp
            Factors.cpp Objects.cpp SerialCommandWriter.cpp
    HEADERS DriverBase.hpp Driver.hpp DS402Driver.hpp
            ChannelBase.hpp Channel.hpp DS402Channel.hpp
            Factors.hpp Objects.hpp JointStatePositionSources.hpp
            ControllerStatus.hpp Exceptions.hpp
    DEPS_PKGCONFIG
        base-types
        canopen_master
        raw_io
)

rock_executable(motors_roboteq_canopen_cfg MainCfg.cpp
    DEPS motors_roboteq_canopen)

