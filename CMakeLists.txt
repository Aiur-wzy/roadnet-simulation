cmake_minimum_required(VERSION 3.20)
project(Simulation_Prediction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(Boost QUIET COMPONENTS filesystem thread system)

add_executable(Simulation_Prediction
        main.cpp head.h data_preparation.cpp simulation.cpp data_cleaning.cpp
)

if (Boost_FOUND)
    target_link_libraries(Simulation_Prediction PRIVATE
            Boost::filesystem Boost::thread Boost::system Threads::Threads
    )
else()
    message(WARNING "Boost was not found; using the std::thread fallback in head.h for this build.")
    target_link_libraries(Simulation_Prediction PRIVATE Threads::Threads)
endif()
