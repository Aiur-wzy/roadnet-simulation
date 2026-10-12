#include "head.h"
#include "test_cap_context_abc_fixture.h"

#include <iostream>
#include <numeric>
#include <sstream>

namespace {

int checks = 0;
void check(bool ok, const string& reason) {
    ++checks;
    if (!ok) throw runtime_error(reason);
}
void near(double actual, double expected, const string& reason) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= 1e-9, reason);
}

void add_movement(Graph& g, int from, int to, int fromLane, int toLane) {
    const int id = static_cast<int>(g.movements.size());
    Movement movement;
    movement.movementID = id; movement.fromRoadID = from; movement.toRoadID = to;
    movement.fromLanes = {to_string(fromLane)}; movement.toLanes = {to_string(toLane)};
    movement.alwaysOpen = true;
    g.movements.push_back(movement);
    WaitingBuffer buffer;
    buffer.bufferID = id; buffer.roadID = from; buffer.movementID = id;
    g.waitingBuffers.push_back(buffer);
    g.roads[from].movementIDToWaitingBufferID[id] = id;
    g.fromToRoadToMovementIDs[make_pair(from, to)].push_back(id);
}

// The receiving road has two lanes: its entrance is lane 1, its exit is lane 0.
// This is the smallest road arrangement where borrowing differs from the old
// one-step logical lane assignment. Synthetic graphs have no SUMO restrictions.
void fixture(Graph& g, bool borrowing = true) {
    g.laneBorrowing = borrowing;
    g.persistentDispatch = true;
    g.travelTimeMode = TravelTimeMode::MIN_TIME;
    g.intersectionCrossingTime = 0;
    g.defaultDischargeInterval = 2.15;
    g.roads.resize(3);
    for (int i = 0; i < 3; ++i) {
        auto& road = g.roads[i];
        road.roadID = i; road.fromNode = i; road.toNode = (i + 1) % 3;
        road.length = 100; road.speedLimit = 10; road.minTravelTime = 10;
        road.laneNum = i == 1 ? 2 : 1;
    }
    add_movement(g, 0, 1, 0, 1);
    add_movement(g, 1, 2, 0, 0);
    add_movement(g, 2, 0, 0, 0);
    g.initializeRoadLaneStorage();
    g.roads[0].laneCapacity[0] = 8;
    g.roads[1].laneCapacity = {1, 1};
    g.roads[2].laneCapacity[0] = 8;
    g.initializeMovementLaneDischargeCapacity();
    g.initializeMovementDischargeTimes(0);
    g.reset_dispatch_state();
    g.rebuild_incoming_movement_index();
}

int add_vehicle(Graph& g, const vector<int>& roads, const vector<int>& movements,
                int logicalLane, int storageLane, double ready, int routeIndex = 0) {
    VehicleLabel vehicle;
    vehicle.vehicleID = static_cast<int>(g.vehicles.size());
    vehicle.routeID = vehicle.vehicleID;
    vehicle.routeRoadIDs = roads; vehicle.routeMovementIDs = movements;
    vehicle.roadIndex = routeIndex; vehicle.currentRoadID = roads[routeIndex];
    vehicle.currentMovementID = routeIndex < static_cast<int>(movements.size()) ? movements[routeIndex] : -1;
    vehicle.currentBufferID = vehicle.currentMovementID < 0 ? -1 : g.getMovementBufferID(vehicle.currentMovementID);
    vehicle.arrivalTime = ready;
    vehicle.state = vehicle.currentMovementID < 0 ? VehicleState::RunningOnRoad : VehicleState::WaitingAtIntersection;
    g.vehicles.push_back(vehicle);
    g.ETA_result_cycle_aware.resize(g.vehicles.size());
    g.reserveLaneOccupancy(vehicle.vehicleID, vehicle.currentRoadID, logicalLane, storageLane);
    if (vehicle.currentBufferID >= 0) g.insertVehicleToBufferOrdered(vehicle.currentBufferID, vehicle.vehicleID);
    return vehicle.vehicleID;
}

void conservation(const Graph& g, const string& where) {
    for (const auto& road : g.roads) {
        check(accumulate(road.laneFlow.begin(), road.laneFlow.end(), 0) == road.roadFlow,
              where + ": logical counts conserve vehicles");
        if (g.laneBorrowing) {
            check(accumulate(road.storageFlow.begin(), road.storageFlow.end(), 0) == road.roadFlow,
                  where + ": storage counts conserve vehicles");
        }
        const auto& storage = g.laneBorrowing ? road.storageFlow : road.laneFlow;
        for (size_t lane = 0; lane < storage.size(); ++lane) {
            check(storage[lane] >= 0 && storage[lane] <= road.laneCapacity[lane],
                  where + ": physical storage remains within lane capacity");
        }
    }
    check(g.validateRoadLaneFlows(), where + ": production vehicle/count invariant");
}

string state(const Graph& g) {
    ostringstream out;
    out << setprecision(17);
    for (const auto& road : g.roads) {
        out << road.roadFlow << ':';
        for (int value : road.laneFlow) out << value << ',';
        for (int value : road.storageFlow) out << value << ',';
        for (double value : road.laneOccupiedLength) out << value << ',';
        for (double value : road.storageOccupiedLength) out << value << ',';
        for (double value : road.laneWaitingBufferLength) out << value << ',';
    }
    for (const auto& vehicle : g.vehicles) {
        out << vehicle.roadIndex << ',' << vehicle.currentRoadID << ',' << vehicle.currentMovementID << ','
            << vehicle.occupiedRoadID << ',' << vehicle.occupiedLaneIndex << ',' << vehicle.storageLaneIndex << ','
            << vehicle.arrivalTime << ',' << vehicle.lastWaitingDuration << ',' << vehicle.queuedForFinalExit << ';';
    }
    for (const auto& buffer : g.waitingBuffers) {
        out << '|'; for (int id : buffer.vehicleQueue) out << id << ',';
    }
    for (const auto& lanes : g.movementLaneNextAvailableTime)
        for (double value : lanes) out << value << ',';
    out << g.finalRoadExitPQ.size();
    return out.str();
}

void switch_and_original_path_checks() {
    Graph defaultGraph;
    check(!defaultGraph.laneBorrowing, "borrowing is opt-in by default");
    for (bool enabled : {false, true}) {
        Graph g; fixture(g, enabled);
        const int ego = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 0, 0);
        int logical = -1, storage = -1;
        const string before = state(g);
        check(g.hasDownstreamLaneStorage(0, ego, logical, &storage), "normal path has receiving space");
        check(logical == 0 && storage == 0, "available original lane keeps one-step assignment");
        check(before == state(g), "admission preview is pure");
        const auto transfer = g.dischargeOneVehicle(0, 0);
        check(transfer.vehicleID == ego && g.vehicles[ego].occupiedLaneIndex == 0 &&
              g.vehicles[ego].storageLaneIndex == 0, "normal discharge reserves original lane");
        check(g.laneBorrowingStats.admissions == 0, "normal discharge is not counted as borrowing");
        conservation(g, "normal branch");
    }
    Graph off; fixture(off, false);
    add_vehicle(off, {1, 2}, {1}, 0, 0, 100);
    const int ego = add_vehicle(off, {0, 1, 2}, {0, 1}, 0, 0, 0);
    const string before = state(off);
    int lane = -1;
    check(!off.hasDownstreamLaneStorage(0, ego, lane), "off preserves target-lane-full blockage");
    check(off.dischargeOneVehicle(0, 0).vehicleID == -1, "off cannot use free entrance lane");
    check(before == state(off), "off blocked attempt preserves original state");
}

void borrowing_lifecycle_checks() {
    Graph g; fixture(g);
    const int native = add_vehicle(g, {1, 2}, {1}, 0, 0, 100);
    const int ego = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 0, 0);
    int logical = -1, storage = -1;
    check(g.hasDownstreamLaneStorage(0, ego, logical, &storage), "full logical target may borrow free entrance");
    check(logical == 0 && storage == 1, "borrowed admission separates exit intent and entrance storage");
    const auto entry = g.dischargeOneVehicle(0, 0);
    check(entry.vehicleID == ego && g.vehicles[ego].currentMovementID == 1,
          "borrowed vehicle joins its route's original outgoing movement");
    check(g.roads[1].laneFlow == vector<int>({2, 0}) && g.roads[1].storageFlow == vector<int>({1, 1}),
          "logical flow stays on target while physical storage uses donor");
    check(g.roads[1].roadFlow == 2 && g.roads[0].roadFlow == 0, "one transfer counts the vehicle once");
    check(g.getFrontVehicleForMovement(1) == ego, "earlier ready borrower is real movement head");
    check(g.laneBorrowingStats.admissions == 1 && g.laneBorrowingStats.active == 1, "admission counters are lifecycle based");
    conservation(g, "borrowed entry");

    const int follower = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 0, 1);
    const string before = state(g);
    check(!g.hasDownstreamLaneStorage(0, follower, logical, &storage), "full donor and target reject another vehicle");
    check(g.dischargeOneVehicle(0, 3).vehicleID == -1, "blocked retry returns no successful transfer");
    check(before == state(g), "failed borrowing does not pop queue, alter storage, or consume service clock");

    add_vehicle(g, {2, 0}, {2}, 0, 0, 100);
    g.deactivateMovementForDownstreamBlock(0);
    g.deactivateMovementForDownstreamBlock(2);
    const double exitAt = g.vehicles[ego].arrivalTime;
    int front = -1;
    check(g.getDischargeBlockReason(1, exitAt, front, logical) == DischargeBlockReason::None,
          "borrowed head may exit while target lane is still physically full");
    check(g.movements[1].laneDischargeCapacity == 1 && g.movementLaneNextAvailableTime[1].size() == 1,
          "extra storage creates no extra exit service channel");
    const auto exit = g.dischargeOneVehicle(1, exitAt);
    check(exit.vehicleID == ego && exit.releasedRoadID == 1 && exit.releasedLaneIndex == 1,
          "successful movement reports the actual freed donor lane");
    check(g.roads[1].laneFlow == vector<int>({1, 0}) && g.roads[1].storageFlow == vector<int>({1, 0}),
          "borrower release preserves the native target occupant");
    check(g.vehicles[native].occupiedRoadID == 1, "borrower cannot release another vehicle's slot");
    near(g.nextAvailableCapacityTime(1, exitAt), exitAt + 2.15, "original movement headway is consumed once");
    check(!g.hasDischargeCapacity(1, exitAt), "borrower cannot grant simultaneous second discharge");
    g.handleSuccessfulDischargePostUpdate(1, exitAt, 0, g.getMovementBufferID(1), exit);
    check(!g.movementBlockedByDownstream[0] && g.movementInDispatchPQ[0], "donor release wakes incoming blocked movement");
    check(g.movementBlockedByDownstream[2] && !g.movementInDispatchPQ[2], "release leaves unrelated road blockage untouched");
    check(g.hasDownstreamLaneStorage(0, follower, logical, &storage) && storage == 1,
          "woken upstream front can use freed donor capacity");
    check(g.laneBorrowingStats.releases == 1 && g.laneBorrowingStats.active == 0,
          "borrowed residence ends exactly when source road is left");
    conservation(g, "borrowed exit");

    const double finalAt = g.vehicles[ego].arrivalTime;
    const int finalFlow = g.roads[2].roadFlow;
    check(!g.processFinalRoadExitsUntil(finalAt - .25), "final occupancy persists before exact ETA");
    check(g.processFinalRoadExitsUntil(finalAt) && g.vehicles[ego].finished, "final event completes borrowed route");
    check(g.roads[2].roadFlow == finalFlow - 1 && g.vehicles[ego].storageLaneIndex == -1,
          "terminal release clears one actual storage record");
    conservation(g, "terminal exit");
}

void feature_contract_checks() {
    Graph g; fixture(g);
    g.intersectionCrossingTime = 3;
    g.travelTimeMode = TravelTimeMode::TABLE;
    g.travelTimeTableFormat = TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RLE_V1;
    istringstream table(cap_context_abc_test::fixture(), ios::binary);
    g.capContextTable.load(table, true);
    g.enableBasicFeatureLogging = true;
    g.sumoVehicleTypes["car"] = VehicleType();
    g.sumoVehicleTypes["car"].maxSpeed = 10;
    g.vehicleTypeIDs.assign(2, "car");
    g.vehicleSpeedFactors.assign(2, 1);
    g.vehicleHasExplicitSpeedFactor.assign(2, true);
    const int native = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 0, 100, 1);
    g.predictRoadTravelTime(1, native, 0, 3, 0, 0);
    const int borrower = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 1, 10, 1);
    const double prediction = g.predictRoadTravelTime(1, borrower, 0, 4, 0, 1);
    const auto features = g.basicFeatureSnapshots.back();
    check(features.laneIndex == 0 && features.lane_flow == 1 && features.road_flow == 2,
          "table receives original lane and lane-only clamp, with true road count");
    check(g.roads[1].laneFlow[0] == 2, "feature clamp never truncates logical accounting");
    // The predecessor's planned entry is 3. Query at decision 4 to include it.
    const CapContextRecord observer{99, 0, 4, 100, 10};
    check(g.capContextLedger.queryABC(1, 0, observer, 4).eligible_count == 2 &&
          g.capContextLedger.queryABC(1, 1, observer, 4).eligible_count == 0,
          "F13 shadow membership remains on original lane, never donor lane");
    near(prediction, g.capContextTable.lookup(100, 2, 1, 10, features.platoon_context),
         "existing table lookup consumes bounded original-lane feature without interface change");
    check(g.laneBorrowingStats.clampedPredictions > 0, "clamp is observable without altering raw count");
    g.releaseLaneOccupancy(borrower);
    check(g.capContextLedger.size() == 1 && g.roads[1].storageFlow[1] == 0,
          "release removes logical F13 record and donor storage together");
    g.vehicles[borrower].roadIndex = 4;
    g.vehicles[borrower].routeRoadIDs = {0, 1, 2, 0, 1, 2};
    g.vehicles[borrower].routeMovementIDs = {0, 1, 2, 0, 1};
    g.reserveLaneOccupancy(borrower, 1, 0, 1);
    g.predictRoadTravelTime(1, borrower, 0, 14, 0, 11);
    check(g.capContextLedger.size() == 2, "same road revisit installs new route-occurrence ledger record");
    g.releaseLaneOccupancy(borrower);
    g.releaseLaneOccupancy(native);
    check(g.capContextLedger.size() == 0 && g.roads[1].roadFlow == 0 &&
          g.roads[1].laneFlow == vector<int>({0, 0}) && g.roads[1].storageFlow == vector<int>({0, 0}),
          "loop occurrence cleanup leaves no clamped residual count");

    // Only logical lane 0 has an exit connection. Selecting the donor for RG
    // would fail; use exactly the lane produced by the prediction feature path.
    g.roadIDToSumoEdgeStr[1] = "b"; g.roadIDToSumoEdgeStr[2] = "c";
    SumoConnectionRaw connection;
    connection.fromEdge = "b"; connection.toEdge = "c";
    connection.fromLane = 0; connection.toLane = 0;
    connection.tl = "T"; connection.linkIndex = 0;
    g.sumoConnectionsRaw.push_back(connection);
    SumoSignalProgram program;
    program.tlID = "T"; program.programID = "0"; program.type = "static";
    program.cycleLength = 70;
    SumoSignalPhase green; green.duration = 60; green.state = "G";
    SumoSignalPhase red; red.duration = 10; red.state = "r";
    program.phases = {green, red}; g.sumoSignalPrograms.push_back(program);
    const auto signal = g.outgoingSignalRG(1, 2, features.laneIndex, 4);
    check(signal.entry_type == 'G' && signal.entry_remaining_seconds == 56,
          "unchanged logical lane resolves RG even when donor has no exit link");
}

void static_eligibility_checks() {
    Graph g; fixture(g);
    g.roads[0].laneNum = 2;
    g.movements[0].fromLanes = {"0", "1"};
    g.movements[0].toLanes = {"0", "1"};
    g.initializeRoadLaneStorage();
    g.roads[1].laneCapacity = {1, 1};
    for (int road = 0; road < 3; ++road) {
        const string name(1, static_cast<char>('a' + road));
        g.roadIDToSumoEdgeStr[road] = name; g.sumoEdgeStrToRoadID[name] = road;
        SumoEdgeRaw edge; edge.id = name;
        for (int lane = 0; lane < g.roads[road].laneNum; ++lane) {
            SumoLaneRaw metadata;
            metadata.id = name + "_" + to_string(lane); metadata.index = lane;
            metadata.length = 100; metadata.speed = 10;
            edge.lanes.push_back(metadata);
        }
        g.sumoEdgesRaw.push_back(edge);
    }
    SumoConnectionRaw entrance;
    entrance.fromEdge = "a"; entrance.toEdge = "b";
    entrance.fromLane = 0; entrance.toLane = 1;
    SumoConnectionRaw otherEntrance = entrance;
    otherEntrance.fromLane = 1; otherEntrance.toLane = 0;
    SumoConnectionRaw exit;
    exit.fromEdge = "b"; exit.toEdge = "c"; exit.fromLane = exit.toLane = 0;
    g.sumoConnectionsRaw = {entrance, otherEntrance, exit};
    const int native = add_vehicle(g, {1, 2}, {1}, 0, 0, 100);
    // This vehicle itself already borrows on its upstream road. Incoming
    // connection eligibility must use its virtual service lane 0, not storage 1.
    const int ego = add_vehicle(g, {0, 1, 2}, {0, 1}, 0, 1, 0);
    auto available = [&]() {
        g.initialize_lane_borrowing();
        int logical = -1, storage = -1;
        const bool result = g.hasDownstreamLaneStorage(0, ego, logical, &storage);
        if (result) check(logical == 0 && storage == 1, "metadata plan preserves intended lane and matching entrance");
        return result;
    };
    check(available(), "matching lane-pair entrance and permitted lateral path allow borrowing");
    g.sumoConnectionsRaw[0].fromLane = 1;
    check(!available(), "aggregated toLanes cannot invent connection from wrong upstream logical lane");
    g.sumoConnectionsRaw[0].fromLane = 0;
    g.sumoConnectionsRaw[0].disallow = "passenger";
    check(!available(), "entrance connection vehicle-class restriction is enforced");
    g.sumoConnectionsRaw[0].disallow.clear();
    g.sumoEdgesRaw[1].lanes[1].allow = "bus";
    check(!available(), "donor lane class restriction is enforced");
    g.sumoEdgesRaw[1].lanes[1].allow = "all";
    g.sumoEdgesRaw[1].lanes[1].changeRight.clear();
    check(!available(), "forbidden rightward lane change prevents virtual merge");
    g.sumoEdgesRaw[1].lanes[1].changeRight = "all";
    g.sumoEdgesRaw[1].lanes[0].changeLeft.clear();
    check(available(), "opposite-direction lane-change restriction does not block permitted merge");
    g.sumoConnectionsRaw[2].disallow = "passenger";
    check(!available(), "exit connection restriction also excludes borrowing plan");
    g.sumoConnectionsRaw[2].disallow.clear();
    g.sumoEdgesRaw[1].lanes[0].index = -1;
    check(!available(), "missing loaded SUMO lane metadata fails closed");

    g.sumoEdgesRaw[1].lanes[0].index = 0;
    g.sumoEdgesRaw[1].lanes[0].changeLeft = "all";
    g.sumoEdgesRaw[1].lanes[1].allow = "passenger";
    g.sumoVehicleTypes["passenger_car"] = VehicleType();
    g.sumoVehicleTypes["bus"] = VehicleType();
    g.sumoVehicleTypes["bus"].vehicleClass = "bus";
    g.vehicleTypeIDs.assign(g.vehicles.size(), "passenger_car");
    check(available(), "passenger plan is cached with its class");
    g.vehicleTypeIDs[ego] = "bus";
    int logical = -1, storage = -1;
    check(!g.hasDownstreamLaneStorage(0, ego, logical, &storage),
          "cached passenger permission cannot leak into a bus lookup");
    g.vehicleTypeIDs[ego] = "passenger_car";
    g.vehicles[ego].occupiedLaneIndex = 1;
    check(!g.hasDownstreamLaneStorage(0, ego, logical, &storage),
          "cached entrance plan is distinct for a different upstream logical lane");
    g.vehicles[ego].occupiedLaneIndex = 0;

    g.releaseLaneOccupancy(native);
    g.reserveLaneOccupancy(native, 1, 1, 1);
    g.movements[1].fromLanes = {"1"};
    g.sumoConnectionsRaw[0].toLane = 0;
    g.sumoConnectionsRaw[2].fromLane = 1;
    g.initialize_lane_borrowing();
    check(g.hasDownstreamLaneStorage(0, ego, logical, &storage) && logical == 1 && storage == 0,
          "index-increasing leftward merge is supported");
    g.sumoEdgesRaw[1].lanes[0].changeLeft.clear();
    g.initialize_lane_borrowing();
    check(!g.hasDownstreamLaneStorage(0, ego, logical, &storage),
          "forbidden leftward merge is enforced independently from rightward permission");
}

void complete_run_and_reset_checks() {
    vector<double> offTimes;
    for (bool enabled : {false, true}) {
        Graph g; fixture(g, enabled);
        g.evalTrajectoryDetails = true;
        // A sparse repeated-road route cannot borrow. It provides an end-to-end
        // off/on regression through initialization, dispatch and final events.
        vector<vector<int>> routes(2, {0, 1, 2, 0, 1, 2});
        vector<vector<int>> queries(2, {0, 0, 0});
        g.vehicleDepartureTimes = {.25, 150.25};
        g.vehicleDeparturePositions.assign(2, 0);
        g.routeRoadID = routes;
        g.route_roadID_2_movementID();
        vector<double> first;
        for (int run = 0; run < 2; ++run) {
            g.cycle_aware_signal_driven_records(queries, routes);
            check(g.finishedVehicleCount == 2 && g.invalidVehicleCount == 0, "complete repeated-road run finishes");
            check(g.laneBorrowingStats.active == 0 && g.laneBorrowingStats.admissions == 0,
                  "sparse run creates no borrowing and rerun clears statistics");
            for (int id = 0; id < 2; ++id) {
                check(g.vehicles[id].storageLaneIndex == -1 && g.vehicles[id].occupiedLaneIndex == -1,
                      "complete run clears logical and storage identities");
                check(g.vehicleRoadTimeline[id].size() == 6 && g.vehicleMovementTimeline[id].size() == 5,
                      "each loop occurrence recorded once, including after reset");
                if (!run) first.push_back(g.vehicles[id].arrivalTime);
                else near(g.vehicles[id].arrivalTime, first[id], "same Graph rerun reproduces exact terminal time");
                if (enabled) near(g.vehicles[id].arrivalTime, offTimes[id], "no-borrow on path reproduces off trajectory");
            }
            conservation(g, "complete simulation reset");
            check(g.finalRoadExitPQ.empty() && !g.hasPendingEntryDepartureQueues(), "complete run drains boundary events");
        }
        if (!enabled) offTimes = first;
    }
}

void congested_run_and_reset_checks() {
    Graph g; fixture(g);
    g.roads[1].length = 4; // Capacity formula gives one slot per physical lane.
    g.evalTrajectoryDetails = true;
    SignalProgram program;
    program.tlID = "T"; program.cycleLength = 200;
    program.phases = {{0, 30, "r"}, {30, 200, "G"}};
    g.signalPrograms.push_back(program); g.tlIDToSignalProgramID["T"] = 0;
    g.movements[1].alwaysOpen = false; g.movements[1].tlID = "T"; g.movements[1].linkIndex = 0;
    SignalController signal; signal.signalID = 0; signal.movementID = 1;
    g.movements[1].signalID = 0; g.signals.push_back(signal);
    vector<vector<int>> routes(6, {0, 1, 2});
    vector<vector<int>> queries(6, {0, 0, 0});
    g.vehicleDepartureTimes.assign(6, .25); g.vehicleDeparturePositions.assign(6, 0);
    g.routeRoadID = routes;
    g.route_roadID_2_movementID();
    vector<double> first;
    long long admissions = 0;
    for (int run = 0; run < 2; ++run) {
        g.cycle_aware_signal_driven_records(queries, routes);
        check(g.finishedVehicleCount == 6 && g.invalidVehicleCount == 0,
              "congested event-driven run completes all vehicles");
        check(g.laneBorrowingStats.admissions > 0 &&
              g.laneBorrowingStats.admissions == g.laneBorrowingStats.releases &&
              g.laneBorrowingStats.active == 0 && g.laneBorrowingStats.clampedPredictions > 0,
              "real scheduler exercises borrowing, feature clamp and every release");
        check(g.laneBorrowingStats.storageReservations == 18 && g.laneBorrowingStats.storageReleases == 18,
              "each route occurrence reserves/releases once; rerun counters do not accumulate");
        conservation(g, "congested scheduler completion");
        for (int id = 0; id < 6; ++id) {
            if (!run) first.push_back(g.vehicles[id].arrivalTime);
            else near(g.vehicles[id].arrivalTime, first[id], "borrowed congested run repeats exactly after reset");
            if (id) {
                check(g.vehicleMovementTimeline[id][1].dischargeTime - g.vehicleMovementTimeline[id - 1][1].dischargeTime
                      >= g.defaultDischargeInterval - TIME_EPSILON,
                      "congested borrowed queue retains original exit service headway");
            }
        }
        if (!run) admissions = g.laneBorrowingStats.admissions;
        else check(g.laneBorrowingStats.admissions == admissions, "rerun discards old borrow counters/cache state");
    }
}

}  // namespace

int main() {
    try {
        switch_and_original_path_checks();
        borrowing_lifecycle_checks();
        feature_contract_checks();
        static_eligibility_checks();
        complete_run_and_reset_checks();
        congested_run_and_reset_checks();
        cout << "Lane borrowing checks passed: " << checks << endl;
        return 0;
    } catch (const exception& error) {
        cerr << "Lane borrowing test failed after " << checks << " checks: " << error.what() << endl;
        return 1;
    }
}
