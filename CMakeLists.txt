cmake_minimum_required(VERSION 3.20)
project(Simulation_Prediction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Simulation_Prediction
        main.cpp head.h data_preparation.cpp simulation.cpp data_cleaning.cpp
)

include(CTest)
if(BUILD_TESTING)
    add_executable(MovementDispatchRegressionTest
            test_movement_dispatch.cpp head.h data_preparation.cpp simulation.cpp data_cleaning.cpp
    )
    add_test(NAME movement_dispatch_regression COMMAND MovementDispatchRegressionTest)
endif()
