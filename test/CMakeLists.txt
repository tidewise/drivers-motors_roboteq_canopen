rock_gtest(test_suite suite.cpp Helpers.cpp
    test_Factors.cpp test_DS402Channel.cpp test_DS402Driver.cpp
    test_SerialCommandWriter.cpp
    DEPS motors_roboteq_canopen)
