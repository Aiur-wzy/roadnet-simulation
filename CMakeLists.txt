cmake_minimum_required(VERSION 3.20)
project(Simulation_Prediction)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/simulator")
set(CAMS_CORE_SOURCES simulator/head.h simulator/data_preparation.cpp simulator/simulation.cpp simulator/lane_borrowing.cpp simulator/roadpair_runtime.cpp simulator/data_cleaning.cpp
        simulator/sf_travel_time_table.cpp simulator/cap_context.cpp simulator/cap_context_table.cpp
        simulator/signal_rg.cpp simulator/signal_rg_graph.cpp simulator/cap_context_abc_rg_table.cpp
        simulator/native_road_time_table.cpp)

# Reuse identical core objects across regression executables; no per-test core
# definitions are used. This avoids recompiling the full scheduler for each test.
add_library(CamsCore OBJECT ${CAMS_CORE_SOURCES})
set(CAMS_CORE_SOURCES $<TARGET_OBJECTS:CamsCore>)

add_executable(Simulation_Prediction
        simulator/main.cpp ${CAMS_CORE_SOURCES}
)

include(CTest)
if(BUILD_TESTING)
    add_executable(NativeRoadTimeTableTest tests/cpp/test_native_road_time_table.cpp simulator/native_road_time_table.cpp simulator/signal_rg.cpp)
    add_test(NAME native_road_time_table COMMAND NativeRoadTimeTableTest)
    add_executable(NativeDemandIntegrationTest tests/cpp/test_native_demand_integration.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME native_demand_integration COMMAND NativeDemandIntegrationTest)
    add_executable(RoadPairGraphTest tests/cpp/test_roadpair_graph.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME roadpair_graph COMMAND RoadPairGraphTest)
    add_executable(RoadPairRuntimeTest tests/cpp/test_roadpair_runtime.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME roadpair_runtime COMMAND RoadPairRuntimeTest)
    add_executable(LaneEntryReselectionTest tests/cpp/test_lane_entry_reselection.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME lane_entry_reselection COMMAND LaneEntryReselectionTest)
    add_executable(SignalRGTest tests/cpp/test_signal_rg.cpp simulator/signal_rg.cpp)
    add_test(NAME signal_rg COMMAND SignalRGTest)
    add_executable(SignalRGGraphTest tests/cpp/test_signal_rg_graph.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME signal_rg_graph COMMAND SignalRGGraphTest)
    add_executable(SumoNetworkTypeTest tests/cpp/test_sumo_network_type.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME sumo_network_type COMMAND SumoNetworkTypeTest)
    add_executable(ContinuousSpeedTest tests/cpp/test_continuous_speed.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME continuous_speed COMMAND ContinuousSpeedTest)
    add_executable(CapContextABCRGTableTest tests/cpp/test_cap_context_abc_rg_table.cpp simulator/cap_context_abc_rg_table.cpp simulator/signal_rg.cpp)
    add_test(NAME cap_context_abc_rg_table COMMAND CapContextABCRGTableTest "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/cap_context_abc_rg_mock.rle" "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/cap_context_abc_rg_mock_queries.txt")
    add_executable(DemandTableReaderTest tests/cpp/test_demand_table_reader.cpp simulator/cap_context_abc_rg_table.cpp simulator/signal_rg.cpp)
    add_test(NAME demand_table_reader COMMAND DemandTableReaderTest "${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/cap_context_abc_rg_mock.rle")
    add_executable(TurnContextTest tests/cpp/test_turn_context.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME turn_context COMMAND TurnContextTest)
    add_executable(MovementDispatchRegressionTest
            tests/cpp/test_movement_dispatch.cpp ${CAMS_CORE_SOURCES}
    )
    add_test(NAME movement_dispatch_regression COMMAND MovementDispatchRegressionTest)
    add_executable(UpstreamReleaseIndexTest tests/cpp/test_upstream_release_index.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME upstream_release_index COMMAND UpstreamReleaseIndexTest)
    add_executable(PersistentDispatchTest tests/cpp/test_persistent_dispatch.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME persistent_dispatch COMMAND PersistentDispatchTest)
    add_executable(LaneBorrowingTest tests/cpp/test_lane_borrowing.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME lane_borrowing COMMAND LaneBorrowingTest)
    add_executable(QueueLengthTrimmingTest
            tests/cpp/test_queue_length_trimming.cpp ${CAMS_CORE_SOURCES}
    )
    add_test(NAME queue_length_trimming COMMAND QueueLengthTrimmingTest)
    add_executable(VehicleSpeedFactorTest
            tests/cpp/test_vehicle_speed_factor.cpp ${CAMS_CORE_SOURCES}
    )
    add_test(NAME vehicle_speed_factor COMMAND VehicleSpeedFactorTest)
    add_executable(DeparturePositionTest
            tests/cpp/test_departure_position.cpp ${CAMS_CORE_SOURCES}
    )
    add_test(NAME departure_position COMMAND DeparturePositionTest)
    add_executable(SfTravelTimeTableTest tests/cpp/test_sf_travel_time_table.cpp simulator/sf_travel_time_table.cpp)
    add_test(NAME sf_travel_time_table COMMAND SfTravelTimeTableTest)
    add_executable(CapContextTest tests/cpp/test_cap_context.cpp ${CAMS_CORE_SOURCES})
    add_test(NAME cap_context COMMAND CapContextTest)
    add_executable(CapContextTableTest tests/cpp/test_cap_context_table.cpp simulator/cap_context_table.cpp)
    add_test(NAME cap_context_table COMMAND CapContextTableTest)
    add_executable(CapContextABCTest tests/cpp/cap_context_abc_test.cpp simulator/cap_context.cpp)
    add_test(NAME cap_context_abc COMMAND CapContextABCTest)
    add_executable(CapContextABCTableTest tests/cpp/test_cap_context_abc_table.cpp simulator/cap_context_table.cpp)
    add_test(NAME cap_context_abc_table COMMAND CapContextABCTableTest)
    get_property(CAMS_TESTS DIRECTORY PROPERTY TESTS)
    set_tests_properties(${CAMS_TESTS} PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()
