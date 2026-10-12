#include "head.h"

#include <cmath>
#include <iostream>

namespace {

void check(bool condition, const string& message) {
    if (!condition) throw runtime_error(message);
}

void close(double actual, double expected, const string& message) {
    check(std::abs(actual - expected) <= 1e-8, message);
}

void prepare(Graph& graph) {
    graph.travelTimeMode = TravelTimeMode::SPEED_NET;
    graph.speedNetUseAcceleration = false;
    graph.speedNetUseFollowing = false;
    VehicleType car;
    car.length = 5.0;
    car.minGap = 2.5;
    car.maxSpeed = 10.0;
    car.accel = 2.0;
    graph.sumoVehicleTypes["car"] = car;
    VehicleType large = car;
    large.length = 10.0;
    large.minGap = 2.0;
    graph.sumoVehicleTypes["large"] = large;
    graph.roads.resize(2);
    for (int roadID = 0; roadID < 2; ++roadID) {
        RoadSegment& road = graph.roads[roadID];
        road.roadID = roadID;
        road.length = 100.0;
        road.speedLimit = 10.0;
        road.laneNum = roadID == 0 ? 2 : 1;
    }
    for (int movementID = 0; movementID < 3; ++movementID) {
        const int roadID = movementID < 2 ? 0 : 1;
        Movement movement;
        movement.movementID = movementID;
        movement.fromRoadID = roadID;
        movement.toRoadID = 1-roadID;
        movement.alwaysOpen = true;
        movement.fromLanes.push_back("0");
        movement.toLanes.push_back("0");
        graph.movements.push_back(movement);
        WaitingBuffer buffer;
        buffer.bufferID = movementID;
        buffer.movementID = movementID;
        buffer.roadID = roadID;
        buffer.laneCount = 2;
        graph.waitingBuffers.push_back(buffer);
        graph.roads[roadID].movementIDToWaitingBufferID[movementID] = movementID;
    }
    graph.initializeRoadLaneStorage();
    graph.vehicles.resize(6);
    graph.vehicleTypeIDs.assign(6, "car");
    graph.vehicleTypeIDs[1] = "large";
    graph.ETA_result_cycle_aware.resize(6);
    graph.initializeMovementDischargeTimes(0.0);
}

void enter(Graph& graph, int id, int road, int lane, int buffer, double arrival, bool enqueue) {
    VehicleLabel& vehicle = graph.vehicles[id];
    vehicle.vehicleID = id;
    vehicle.routeID = id;
    vehicle.currentRoadID = road;
    vehicle.routeRoadIDs = {road, 1-road};
    vehicle.routeMovementIDs = {buffer};
    vehicle.currentMovementID = buffer;
    vehicle.currentBufferID = buffer;
    vehicle.state = VehicleState::WaitingAtIntersection;
    vehicle.arrivalTime = arrival;
    graph.reserveLaneOccupancy(id, road, lane);
    if (enqueue) graph.insertVehicleToBufferOrdered(buffer, id);
}

void test_snapshot_and_switch(MovementQueueOrder order) {
    Graph graph;
    prepare(graph);
    graph.movementQueueOrder = order;
    // Both future arrivals must count, including a different movement sharing
    // the physical lane. Another lane and another road must not count.
    enter(graph, 0, 0, 0, 0, 100.0, true);
    enter(graph, 1, 0, 0, 1, 200.0, true);
    enter(graph, 2, 0, 1, 0, 300.0, true);
    enter(graph, 4, 1, 0, 2, 400.0, true);
    enter(graph, 3, 0, 0, 0, 50.0, false);
    const BasicRoadModelFeatures before = graph.buildBasicRoadModelFeatures(0, 3, 0, 0.0, 0);
    close(before.road_length, 100.0, "physical road length changed");
    close(before.lane_waiting_buffer_length, 19.5, "future-arrival logical buffer length is wrong");
    close(before.lane_occupied_length, 27.0, "physical occupancy includes the newly reserved vehicle");
    check(!graph.speedNetQueueTrim, "queue trimming must default off");
    close(graph.predictRoadTravelTimeSpeedNet(before), 10.0, "off prediction changed");
    graph.speedNetQueueTrim = true;
    close(graph.predictRoadTravelTimeSpeedNet(before), 8.05, "on prediction did not use trimmed length");
    close(graph.roads[0].length, 100.0, "trimming mutated road geometry");
    close(graph.buildBasicRoadModelFeatures(0, 3, 0, 999.0, 0).lane_waiting_buffer_length,
          19.5, "logical queue length must not depend on arrival-time eligibility");
    close(graph.buildBasicRoadModelFeatures(0, 3, 0, 0.0, 1).lane_waiting_buffer_length,
          7.5, "selected-lane isolation failed");
    close(graph.buildBasicRoadModelFeatures(0, -1, -1, 0.0).lane_waiting_buffer_length,
          0.0, "unknown lane must not trim by a road-wide queue");

    graph.insertVehicleToBufferOrdered(0, 3);
    close(graph.roads[0].laneWaitingBufferLength[0], 27.0, "buffer insertion failed to add length");
    const BasicRoadModelFeatures after = graph.buildBasicRoadModelFeatures(0, 3, 0, 0.0, 0);
    close(after.lane_waiting_buffer_length, 19.5, "self was counted after insertion");
    graph.registerWaitingBufferLength(0, 3);
    close(graph.roads[0].laneWaitingBufferLength[0], 27.0, "duplicate registration double counted");
    const double assignedArrival = graph.vehicles[3].arrivalTime;
    graph.speedNetQueueTrim = false;
    close(graph.predictRoadTravelTimeSpeedNet(after), 10.0, "runtime switch off did not restore full length");
    graph.speedNetQueueTrim = true;
    close(graph.predictRoadTravelTimeSpeedNet(after), 8.05, "runtime switch on needs a rebuild");
    close(graph.vehicles[3].arrivalTime, assignedArrival, "switching must not rewrite assigned arrivals");

    // A successful queue removal reduces only later snapshots. No repairing of
    // already assigned travel times or cached feature snapshots is performed.
    auto& queue = graph.waitingBuffers[0].vehicleQueue;
    queue.erase(std::find(queue.begin(), queue.end(), 0));
    graph.releaseLaneOccupancy(0);
    close(graph.roads[0].laneWaitingBufferLength[0], 19.5, "release failed to subtract queued length");
    close(graph.buildBasicRoadModelFeatures(0, 3, 0, 1.0, 0).lane_waiting_buffer_length,
          12.0, "next snapshot did not see released queue space");
    close(graph.vehicles[3].arrivalTime, assignedArrival, "queue shrink triggered forbidden compensation");
    close(graph.predictRoadTravelTimeSpeedNet(after), 8.05, "previous entry snapshot changed retroactively");
    graph.releaseLaneOccupancy(0);
    close(graph.roads[0].laneWaitingBufferLength[0], 19.5, "repeated release double subtracted");
    graph.initializeRoadLaneStorage();
    close(graph.roads[0].laneWaitingBufferLength[0], 0.0, "new-run initialization did not reset lengths");
    check(!graph.vehicles[3].countedInWaitingBufferLength, "new-run initialization left a stale registration");
}

void test_formula_boundaries() {
    Graph graph;
    prepare(graph);
    BasicRoadModelFeatures features;
    features.roadID = 0;
    features.vehicleID = 0;
    features.road_length = 100.0;
    features.speed_limit = 10.0;
    features.lane_flow = 4;
    features.lane_waiting_buffer_length = 19.5;
    graph.speedNetQueueTrim = true;
    graph.speedNetUseAcceleration = true;
    close(graph.predictRoadTravelTimeSpeedNet(features), 10.55, "acceleration did not use effective length");
    graph.speedNetUseFollowing = true;
    graph.speedNetHeadway = 0.2;
    graph.speedNetDelayMax = 3.0;
    close(graph.predictRoadTravelTimeSpeedNet(features), 11.15, "following delay changed with trimming");
    features.lane_waiting_buffer_length = 90.0;
    close(graph.predictRoadTravelTimeSpeedNet(features), std::sqrt(10.0)+0.6,
          "short effective road must use acceleration-only branch");
    features.lane_waiting_buffer_length = 1000.0;
    close(graph.predictRoadTravelTimeSpeedNet(features), 1.0, "negative length or time escaped clamping");
    features.lane_flow = 50;
    close(graph.predictRoadTravelTimeSpeedNet(features), 3.0, "following cap was lost at zero effective length");
    graph.speedNetQueueTrim = false;
    close(graph.predictRoadTravelTimeSpeedNet(features), 15.5, "off no longer matches original formula");
    features.lane_waiting_buffer_length = -10.0;
    graph.speedNetQueueTrim = true;
    close(graph.predictRoadTravelTimeSpeedNet(features), 15.5, "negative queue increased road length");
    features.lane_waiting_buffer_length = 19.5;
    const auto tableKeyOn = graph.buildSumoV1TravelTimeKeyForPrediction(features);
    const double kinematicOn = graph.predictRoadTravelTimeKinematic(features);
    graph.speedNetQueueTrim = false;
    check(tableKeyOn == graph.buildSumoV1TravelTimeKeyForPrediction(features), "Table lookup key changed");
    close(kinematicOn, graph.predictRoadTravelTimeKinematic(features), "kinematic mode changed");
}

void test_discharge_lifecycle() {
    Graph graph;
    prepare(graph);
    graph.speedNetQueueTrim = true;
    graph.intersectionCrossingTime = 3.0;
    enter(graph, 0, 0, 0, 0, 5.0, true);
    enter(graph, 5, 1, 0, 2, 500.0, false);
    graph.roads[1].laneCapacity[0] = 1;
    const DischargeResult blocked = graph.dischargeOneVehicle(0, 6.0);
    check(blocked.vehicleID < 0, "full downstream lane unexpectedly admitted a vehicle");
    close(graph.roads[0].laneWaitingBufferLength[0], 7.5, "failed discharge removed queue length");
    check(graph.waitingBuffers[0].vehicleCount() == 1, "failed discharge changed queue membership");
    graph.releaseLaneOccupancy(5);
    const DischargeResult released = graph.dischargeOneVehicle(0, 6.0);
    check(released.vehicleID == 0, "successful discharge failed");
    close(graph.roads[0].laneWaitingBufferLength[0], 0.0, "successful discharge leaked old queue length");
    close(graph.roads[1].laneWaitingBufferLength[0], 0.0, "final-road storage was counted as a buffer");
    close(graph.vehicles[0].arrivalTime, 19.0, "final road without a buffer did not retain full length");
    check(graph.processFinalRoadExitsUntil(19.0), "final exit did not finish");
    close(graph.roads[1].laneWaitingBufferLength[0], 0.0, "final exit corrupted buffer length");
}

}  // namespace

int main() {
    try {
        test_snapshot_and_switch(MovementQueueOrder::ArrivalTime);
        test_snapshot_and_switch(MovementQueueOrder::StrictFIFO);
        test_formula_boundaries();
        test_discharge_lifecycle();
        std::cout << "queue length trimming regression passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "queue length trimming regression failed: " << error.what() << '\n';
        return 1;
    }
}
