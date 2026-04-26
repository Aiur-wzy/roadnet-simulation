cmake_minimum_required(VERSION 3.20)
project(Simulation_Prediction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()
set(Boost_NO_BOOST_CMAKE ON)
set(Boost_NO_SYSTEM_PATHS ON)

if(DEFINED ENV{BOOST_ROOT})
    set(Boost_ROOT "$ENV{BOOST_ROOT}")
endif()

# ✅ 不要请求 system
find_package(Boost REQUIRED COMPONENTS filesystem thread)

add_executable(Simulation_Prediction
        main.cpp head.h data_preparation.cpp simulation.cpp data_cleaning.cpp
        update.cpp deletion_operation.cpp eta_update.cpp
)

target_link_libraries(Simulation_Prediction PRIVATE
        Boost::filesystem Boost::thread
)





#[[
cmake_minimum_required(VERSION 3.20)
project(Simulation_Prediction)

# 不再硬编码Boost路径，而是让CMake自动查找
find_package(Boost 1.71.0 COMPONENTS thread system filesystem REQUIRED)
if (Boost_FOUND)
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost_VERSION: ${Boost_VERSION}")
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
else()
    message(FATAL_ERROR "Boost not found")
endif ()

set(CMAKE_CXX_STANDARD 14)

# 添加源文件
add_executable(Simulation_Prediction main.cpp head.h data_preparation.cpp simulation.cpp data_cleaning.cpp update.cpp deletion_operation.cpp eta_update.cpp data_cleaning.cpp)

# 链接Boost库
target_link_libraries(Simulation_Prediction Boost::thread Boost::system Boost::filesystem)
]]
