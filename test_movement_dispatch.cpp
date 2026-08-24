#include "head.h"

#include <cmath>
#include <iostream>

namespace {

bool nearlyEqual(double a, double b) {
    return std::abs(a - b) <= TIME_EPSILON;
}

VehicleLabel makeWaitingVehicle(int vehicleID, double arrivalTime) {
    VehicleLabel vehicle;
    vehicle.vehicleID = vehicleID;
    vehicle.valid = true;
    vehicle.finished = false;
    vehicle.state = VehicleState::WaitingAtIntersection;
    vehicle.currentRoadID = 0;
    vehicle.currentMovementID = 0;
    vehicle.currentBufferID = 0;
    vehicle.arrivalTime = arrivalTime;
    return vehicle;
}

}  // namespace

int main() {
    Graph graph;
    graph.roads.resize(2);
    graph.roads[0].roadID = 0;
    graph.roads[0].movementIDToWaitingBufferID[0] = 0;

    Movement movement;
    movement.movementID = 0;
    movement.fromRoadID = 0;
    movement.toRoadID = 1;
    movement.alwaysOpen = true;
    graph.movements.push_back(movement);

    WaitingBuffer buffer;
    buffer.bufferID = 0;
    buffer.roadID = 0;
    buffer.movementID = 0;
    graph.waitingBuffers.push_back(buffer);

    graph.vehicles.push_back(makeWaitingVehicle(0, 10.0));
    graph.vehicles.push_back(makeWaitingVehicle(1, 20.0));
    graph.vehicles.push_back(makeWaitingVehicle(2, 5.0));
    graph.movementTimeLabel.assign(1, 0.0);
    graph.movementPQVersion.assign(1, 0);
    graph.movementBlockedByDownstream.assign(1, false);
    graph.movementInDispatchPQ.assign(1, false);
    graph.movementLaneNextAvailableTime.assign(1, std::vector<double>(1, 0.0));

    graph.insertVehicleToBufferOrdered(0, 0);
    if (!graph.scheduleInsertedVehicleIfMovementFront(0, 0, 0.0)) {
        std::cerr << "initial front was not scheduled\n";
        return 1;
    }
    const double originalLabel = graph.movementTimeLabel[0];
    const int originalVersion = graph.movementPQVersion[0];

    graph.insertVehicleToBufferOrdered(0, 1);
    if (graph.scheduleInsertedVehicleIfMovementFront(0, 1, 1.0)) {
        std::cerr << "follower unexpectedly changed the movement candidate\n";
        return 1;
    }
    if (!nearlyEqual(graph.movementTimeLabel[0], originalLabel)
            || graph.movementPQVersion[0] != originalVersion) {
        std::cerr << "follower changed the first label or candidate version\n";
        return 1;
    }

    graph.insertVehicleToBufferOrdered(0, 2);
    if (!graph.scheduleInsertedVehicleIfMovementFront(0, 2, 2.0)) {
        std::cerr << "new ordered front was not scheduled\n";
        return 1;
    }
    if (!nearlyEqual(graph.movementTimeLabel[0], 5.0)) {
        std::cerr << "new front label was not recomputed from its own arrival\n";
        return 1;
    }

    // Reset the same movement and verify that strict FIFO preserves insertion
    // order even when a later vehicle has a slightly earlier predicted arrival.
    while (!graph.dispatchPQ.empty()) graph.dispatchPQ.pop();
    graph.waitingBuffers[0].vehicleQueue.clear();
    graph.movementTimeLabel.assign(1, 0.0);
    graph.movementPQVersion.assign(1, 0);
    graph.movementBlockedByDownstream.assign(1, false);
    graph.movementInDispatchPQ.assign(1, false);
    graph.movementQueueOrder = MovementQueueOrder::StrictFIFO;
    graph.vehicles[0].arrivalTime = 10.0;
    graph.vehicles[1].arrivalTime = 9.9;
    graph.vehicles[2].arrivalTime = 9.0;

    graph.insertVehicleToBufferOrdered(0, 0);
    if (!graph.scheduleInsertedVehicleIfMovementFront(0, 0, 0.0)) {
        std::cerr << "FIFO initial front was not scheduled\n";
        return 1;
    }
    const double fifoFrontLabel = graph.movementTimeLabel[0];
    const int fifoFrontVersion = graph.movementPQVersion[0];

    graph.insertVehicleToBufferOrdered(0, 1);
    graph.insertVehicleToBufferOrdered(0, 2);
    const auto &fifoQueue = graph.waitingBuffers[0].vehicleQueue;
    if (fifoQueue.size() != 3
            || fifoQueue[0] != 0
            || fifoQueue[1] != 1
            || fifoQueue[2] != 2) {
        std::cerr << "strict FIFO did not preserve insertion order\n";
        return 1;
    }
    if (graph.scheduleInsertedVehicleIfMovementFront(0, 1, 1.0)
            || graph.scheduleInsertedVehicleIfMovementFront(0, 2, 2.0)) {
        std::cerr << "FIFO follower unexpectedly changed the movement candidate\n";
        return 1;
    }
    if (!nearlyEqual(graph.movementTimeLabel[0], fifoFrontLabel)
            || !nearlyEqual(fifoFrontLabel, 10.0)
            || graph.movementPQVersion[0] != fifoFrontVersion) {
        std::cerr << "FIFO follower changed the front-derived outer label\n";
        return 1;
    }

    // A one-road route must keep contributing to road/lane flow until its
    // floating-point final ETA is reached by the final-road exit event queue.
    Graph exitGraph;
    RoadSegment finalRoad;
    finalRoad.roadID = 0;
    finalRoad.length = 100.0;
    finalRoad.speedLimit = 10.0;
    finalRoad.laneNum = 1;
    exitGraph.roads.push_back(finalRoad);
    exitGraph.initializeRoadLaneStorage();

    VehicleLabel finalVehicle;
    finalVehicle.vehicleID = 0;
    finalVehicle.routeID = 0;
    finalVehicle.routeRoadIDs.push_back(0);
    finalVehicle.currentRoadID = 0;
    finalVehicle.state = VehicleState::NotDeparted;
    exitGraph.vehicles.push_back(finalVehicle);
    exitGraph.ETA_result_cycle_aware.resize(1);

    if (!exitGraph.tryStartVehicleFromEntryQueue(0, 2.25, 0, 0)) {
        std::cerr << "single-road vehicle did not enter its final road\n";
        return 1;
    }
    const double finalExitTime = exitGraph.vehicles[0].arrivalTime;
    if (exitGraph.roads[0].roadFlow != 1
            || exitGraph.roads[0].laneFlow[0] != 1
            || exitGraph.vehicles[0].occupiedRoadID != 0
            || exitGraph.vehicles[0].occupiedLaneIndex != 0
            || !exitGraph.vehicles[0].queuedForFinalExit
            || exitGraph.vehicles[0].finished
            || exitGraph.finishedVehicleCount != 0
            || exitGraph.finalRoadExitPQ.size() != 1) {
        std::cerr << "final-road occupancy was not retained until exit\n";
        return 1;
    }

    if (exitGraph.processFinalRoadExitsUntil(finalExitTime - 0.25)) {
        std::cerr << "final-road exit was processed before its exact ETA\n";
        return 1;
    }
    if (exitGraph.roads[0].roadFlow != 1
            || exitGraph.roads[0].laneFlow[0] != 1
            || exitGraph.finishedVehicleCount != 0) {
        std::cerr << "early exit check changed final-road occupancy\n";
        return 1;
    }

    if (!exitGraph.processFinalRoadExitsUntil(finalExitTime)) {
        std::cerr << "final-road exit was not processed at its exact ETA\n";
        return 1;
    }
    if (exitGraph.roads[0].roadFlow != 0
            || exitGraph.roads[0].laneFlow[0] != 0
            || exitGraph.vehicles[0].occupiedRoadID != -1
            || exitGraph.vehicles[0].occupiedLaneIndex != -1
            || exitGraph.vehicles[0].queuedForFinalExit
            || !exitGraph.vehicles[0].finished
            || exitGraph.vehicles[0].state != VehicleState::Finished
            || exitGraph.finishedVehicleCount != 1
            || !exitGraph.finalRoadExitPQ.empty()
            || exitGraph.ETA_result_cycle_aware[0].size() != 2
            || !nearlyEqual(exitGraph.ETA_result_cycle_aware[0].back().second, finalExitTime)) {
        std::cerr << "final-road exit did not release flow and finalize the vehicle\n";
        return 1;
    }

    // The normal multi-road discharge path must reserve the destination road
    // before prediction and defer completion to the same exit queue.
    Graph routeExitGraph;
    RoadSegment upstreamRoad;
    upstreamRoad.roadID = 0;
    upstreamRoad.length = 80.0;
    upstreamRoad.speedLimit = 10.0;
    upstreamRoad.laneNum = 1;
    upstreamRoad.movementIDToWaitingBufferID[0] = 0;
    RoadSegment destinationRoad;
    destinationRoad.roadID = 1;
    destinationRoad.length = 100.0;
    destinationRoad.speedLimit = 10.0;
    destinationRoad.laneNum = 1;
    routeExitGraph.roads.push_back(upstreamRoad);
    routeExitGraph.roads.push_back(destinationRoad);
    routeExitGraph.initializeRoadLaneStorage();

    Movement finalMovement;
    finalMovement.movementID = 0;
    finalMovement.fromRoadID = 0;
    finalMovement.toRoadID = 1;
    finalMovement.alwaysOpen = true;
    finalMovement.fromLanes.push_back("0");
    finalMovement.toLanes.push_back("0");
    finalMovement.laneDischargeCapacity = 1;
    routeExitGraph.movements.push_back(finalMovement);

    WaitingBuffer finalBuffer;
    finalBuffer.bufferID = 0;
    finalBuffer.roadID = 0;
    finalBuffer.movementID = 0;
    finalBuffer.vehicleQueue.push_back(0);
    routeExitGraph.waitingBuffers.push_back(finalBuffer);

    VehicleLabel routedVehicle;
    routedVehicle.vehicleID = 0;
    routedVehicle.routeID = 0;
    routedVehicle.routeRoadIDs.push_back(0);
    routedVehicle.routeRoadIDs.push_back(1);
    routedVehicle.routeMovementIDs.push_back(0);
    routedVehicle.currentRoadID = 0;
    routedVehicle.currentMovementID = 0;
    routedVehicle.currentBufferID = 0;
    routedVehicle.state = VehicleState::WaitingAtIntersection;
    routedVehicle.arrivalTime = 3.0;
    routeExitGraph.vehicles.push_back(routedVehicle);
    routeExitGraph.ETA_result_cycle_aware.resize(1);
    routeExitGraph.ETA_result_cycle_aware[0].push_back({0, 0.0});
    routeExitGraph.movementTimeLabel.assign(1, 0.0);
    routeExitGraph.movementPQVersion.assign(1, 0);
    routeExitGraph.movementBlockedByDownstream.assign(1, false);
    routeExitGraph.movementInDispatchPQ.assign(1, false);
    routeExitGraph.initializeMovementDischargeTimes(0.0);
    routeExitGraph.enableBasicFeatureLogging = true;
    routeExitGraph.reserveLaneOccupancy(0, 0, 0);

    const DischargeResult finalDischarge = routeExitGraph.dischargeOneVehicle(0, 4.5);
    if (finalDischarge.vehicleID != 0
            || finalDischarge.finished
            || routeExitGraph.roads[0].roadFlow != 0
            || routeExitGraph.roads[1].roadFlow != 1
            || routeExitGraph.roads[1].laneFlow[0] != 1
            || routeExitGraph.vehicles[0].occupiedRoadID != 1
            || routeExitGraph.vehicles[0].occupiedLaneIndex != 0
            || !routeExitGraph.vehicles[0].queuedForFinalExit
            || routeExitGraph.vehicles[0].finished
            || routeExitGraph.finishedVehicleCount != 0
            || routeExitGraph.finalRoadExitPQ.size() != 1) {
        std::cerr << "multi-road final discharge did not retain destination occupancy\n";
        return 1;
    }
    if (routeExitGraph.basicFeatureSnapshots.empty()
            || routeExitGraph.basicFeatureSnapshots.back().road_flow != 1
            || routeExitGraph.basicFeatureSnapshots.back().lane_flow != 1) {
        std::cerr << "final-road prediction did not include its reserved vehicle\n";
        return 1;
    }

    const double routedExitTime = routeExitGraph.vehicles[0].arrivalTime;
    if (!routeExitGraph.processFinalRoadExitsUntil(routedExitTime)
            || routeExitGraph.roads[1].roadFlow != 0
            || routeExitGraph.roads[1].laneFlow[0] != 0
            || !routeExitGraph.vehicles[0].finished
            || routeExitGraph.finishedVehicleCount != 1
            || routeExitGraph.ETA_result_cycle_aware[0].size() != 2
            || !nearlyEqual(routeExitGraph.ETA_result_cycle_aware[0].back().second, routedExitTime)) {
        std::cerr << "multi-road final exit did not release destination occupancy\n";
        return 1;
    }

    std::cout << "movement dispatch regression passed\n";
    return 0;
}
