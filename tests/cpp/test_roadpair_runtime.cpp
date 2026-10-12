#include "head.h"

namespace {
int checks = 0;
void require(bool value, const string& message) {
    ++checks;
    if (!value) throw runtime_error(message);
}
void roads(Graph& g, const vector<int>& lanes) {
    g.mergeRoadPairMovements = true;
    g.persistentDispatch = true;
    g.laneBorrowing = false;
    g.travelTimeMode = TravelTimeMode::SPEED_NET;
    g.speedNetUseAcceleration = false;
    g.speedNetUseFollowing = false;
    g.speedNetDepartPosition = false;
    g.defaultDischargeInterval = 2.0;
    g.intersectionCrossingTime = 0.0;
    g.sumoNetworkWithoutInternalLinks = true;
    g.evalTrajectoryDetails = true;
    g.enableBasicFeatureLogging = true;
    for (int count : lanes) {
        RoadSegment road;
        road.roadID = static_cast<int>(g.roads.size());
        road.laneNum = count;
        road.length = 100.0;
        road.speedLimit = 10.0;
        g.roads.push_back(road);
    }
}
int movement(Graph& g, int from, int to, const vector<pair<int,int>>& pairs) {
    Movement m;
    m.movementID = static_cast<int>(g.movements.size());
    m.fromRoadID = from;
    m.toRoadID = to;
    m.alwaysOpen = true;
    for (const auto& pair : pairs) {
        SumoConnectionRaw c;
        c.fromEdge = "r" + to_string(from);
        c.toEdge = "r" + to_string(to);
        c.fromLane = pair.first;
        c.toLane = pair.second;
        m.connectionIndices.push_back(static_cast<int>(g.sumoConnectionsRaw.size()));
        g.sumoConnectionsRaw.push_back(c);
        m.fromLanes.push_back(to_string(pair.first));
        m.toLanes.push_back(to_string(pair.second));
    }
    sort(m.fromLanes.begin(), m.fromLanes.end());
    m.fromLanes.erase(unique(m.fromLanes.begin(), m.fromLanes.end()), m.fromLanes.end());
    sort(m.toLanes.begin(), m.toLanes.end());
    m.toLanes.erase(unique(m.toLanes.begin(), m.toLanes.end()), m.toLanes.end());
    g.movements.push_back(m);
    WaitingBuffer b;
    b.bufferID = static_cast<int>(g.waitingBuffers.size());
    b.roadID = from;
    b.movementID = m.movementID;
    g.waitingBuffers.push_back(b);
    g.roads[from].movementIDToWaitingBufferID[m.movementID] = b.bufferID;
    g.fromToRoadToMovementIDs[{from,to}].push_back(m.movementID);
    g.roadPairToMovementID[{from,to}] = m.movementID;
    return m.movementID;
}
void initialize(Graph& g) {
    g.initializeMovementLaneDischargeCapacity();
    g.initializeRoadLaneStorage();
    g.initializeMovementDischargeTimes(0.0);
    g.reset_dispatch_state();
    g.rebuild_incoming_movement_index();
    g.initializeEntryLaneDepartureQueues();
}
int vehicle(Graph& g, int m, int lane, double at, const vector<int>& route = {}, const vector<int>& movements = {}) {
    VehicleLabel v;
    v.vehicleID = static_cast<int>(g.vehicles.size());
    v.routeID = v.vehicleID;
    v.currentMovementID = m;
    v.currentBufferID = g.getMovementBufferID(m);
    v.currentRoadID = g.movements[m].fromRoadID;
    v.state = VehicleState::WaitingAtIntersection;
    v.arrivalTime = at;
    v.routeRoadIDs = route.empty() ? vector<int>{g.movements[m].fromRoadID,g.movements[m].toRoadID} : route;
    v.routeMovementIDs = movements.empty() ? vector<int>{m} : movements;
    g.vehicles.push_back(v);
    g.vehicleRoadTimeline.resize(g.vehicles.size());
    g.vehicleMovementTimeline.resize(g.vehicles.size());
    g.ETA_result_cycle_aware.resize(g.vehicles.size());
    g.reserveLaneOccupancy(v.vehicleID, v.currentRoadID, lane);
    g.insertVehicleToBufferOrdered(v.currentBufferID, v.vehicleID);
    g.scheduleInsertedVehicleIfMovementFront(m, v.vehicleID, 0.0);
    return v.vehicleID;
}
void independent_lanes() {
    Graph g; roads(g,{2,2}); movement(g,0,1,{{0,0},{1,1}}); initialize(g);
    g.roads[1].laneCapacity[0] = 1;
    VehicleLabel blocker; blocker.vehicleID = 0; blocker.currentRoadID = 1; blocker.state = VehicleState::RunningOnRoad;
    g.vehicles.push_back(blocker); g.reserveLaneOccupancy(0,1,0);
    int blocked = vehicle(g,0,0,0.0), free = vehicle(g,0,1,1.0);
    require(g.roadPairChoices[0].vehicleID == free,"blocked first lane froze independent lane");
    g.process_discharge_window(0.0,2.0);
    require(g.vehicles[free].roadIndex == 1 && g.vehicles[blocked].roadIndex == 0,"wrong independent lane discharged");
    const auto trace = g.vehicleMovementTimeline[free].front();
    require(trace.serviceLane == 1 && trace.entryLane == 1,"connection Cartesian product or anonymous source lane");
    g.releaseLaneOccupancy(0); g.vehicles[0].finished = true;
    g.reactivateMovementsBlockedByRoad(1,1.5,0);
    require(g.movementInDispatchPQ[0] && g.roadPairChoices[0].vehicleID == blocked,"alternate released lane did not wake");
    g.process_discharge_window(1.5,2.0);
    require(g.vehicles[blocked].roadIndex == 1,"released independent lane not served");
    require(g.road_pair_validate_runtime(false),"lane index conservation after independent releases");
}
void shared_source() {
    Graph g; roads(g,{1,1,1}); movement(g,0,1,{{0,0}}); movement(g,0,2,{{0,0}}); initialize(g);
    g.movements[0].alwaysOpen = false; g.movements[0].signalID = 0;
    SignalController s; s.signalID = 0; s.movementID = 0; s.cycleLength = 30; s.greenStart = 10; s.greenEnd = 20;
    g.signals.push_back(s);
    int first = vehicle(g,0,0,1), second = vehicle(g,1,0,2);
    require(!g.movementInDispatchPQ[1],"different movement overtook same lane head");
    g.process_discharge_window(0,10);
    require(g.vehicles[first].roadIndex == 0 && g.vehicles[second].roadIndex == 0,"red head did not block shared lane");
    g.process_discharge_window(10,15);
    require(g.vehicleMovementTimeline[first][0].dischargeTime == 10,"first green release wrong");
    require(g.vehicleMovementTimeline[second][0].dischargeTime == 12,"shared source clock duplicated across movements");
    require(g.road_pair_validate_runtime(false),"shared source queue bookkeeping failed");
}
void capacity_and_pairs() {
    Graph g; roads(g,{5,1}); movement(g,0,1,{{0,0},{1,0},{2,0},{3,0},{4,0}}); initialize(g);
    for (int lane=0; lane<5; ++lane) vehicle(g,0,lane,0);
    g.process_discharge_window(0,9);
    for (int lane=0; lane<5; ++lane)
        require(g.vehicleMovementTimeline[lane][0].dischargeTime == lane*2.0,"5-to-1 multiplied original movement pool");
    Graph fanout; roads(fanout,{1,2}); movement(fanout,0,1,{{0,0},{0,1}}); initialize(fanout);
    vehicle(fanout,0,0,0); vehicle(fanout,0,0,0);
    fanout.process_discharge_window(0,3);
    require(fanout.vehicleMovementTimeline[1][0].dischargeTime == 2,"1-to-many duplicated source capacity");
    Graph crossed; roads(crossed,{2,2}); movement(crossed,0,1,{{0,1},{1,0}}); initialize(crossed);
    vehicle(crossed,0,0,0); vehicle(crossed,0,1,0);
    crossed.process_discharge_window(0,1);
    require(crossed.vehicleMovementTimeline[0][0].entryLane == 1 && crossed.vehicleMovementTimeline[1][0].entryLane == 0,
        "original crossed lane pairs were replaced with Cartesian selection");
}
void projection_and_permissions() {
    Graph g; roads(g,{1,2,1}); movement(g,0,1,{{0,0}}); movement(g,1,2,{{1,0}}); initialize(g);
    vehicle(g,0,0,0,{0,1,2},{0,1});
    g.process_discharge_window(0,1);
    const auto& trace = g.vehicleMovementTimeline[0][0];
    require(trace.entryLane == 0 && trace.storageLane == 1 && trace.chosenLane == 1,"legacy projection contract changed");
    require(g.roadPairVirtualProjections == 1 && g.roads[1].laneFlow[0] == 0 && g.roads[1].laneFlow[1] == 1,"projection occupancy wrong");
    require(g.basicFeatureSnapshots.size() == 1,"downstream road predicted more than once");
    require(g.road_pair_validate_runtime(false),"projected lane membership invalid");
    Graph permissions; roads(permissions,{2,2}); movement(permissions,0,1,{{0,0},{1,1}});
    permissions.sumoConnectionsRaw[0].allow = "bus"; initialize(permissions);
    require(permissions.road_pair_entry_lanes(0,0,0) == vector<int>{1},"connection class permission ignored at entry");
    vehicle(permissions,0,0,0);
    require(!permissions.movementInDispatchPQ[0],"disallowed connection became dispatchable");
}
void head_insertion_and_stale_candidate() {
    for (bool fifo : {false,true}) {
        Graph g; roads(g,{1,1,1}); movement(g,0,1,{{0,0}}); movement(g,0,2,{{0,0}}); initialize(g);
        g.movementQueueOrder = fifo ? MovementQueueOrder::StrictFIFO : MovementQueueOrder::ArrivalTime;
        vehicle(g,0,0,10); const auto old = g.dispatchPQ.top(); vehicle(g,1,0,5);
        require(g.isDispatchCandidateValid(old) == fifo,"cross-movement insertion did not respect queue mode/invalidate old head");
        g.process_discharge_window(0,15);
        const double t0 = g.vehicleMovementTimeline[0][0].dischargeTime, t1 = g.vehicleMovementTimeline[1][0].dischargeTime;
        require(fifo ? (t0 == 10 && t1 == 12) : (t1 == 5 && t0 == 10),"cross-movement lane ordering incorrect");
    }
}
void internal_permissions_and_route_guards() {
    Graph g; roads(g,{1,1}); movement(g,0,1,{{0,0}});
    for (int road = 0; road < 2; ++road) {
        SumoEdgeRaw edge; edge.id = "r" + to_string(road);
        SumoLaneRaw lane; lane.id = edge.id + "_0"; lane.index = 0; edge.lanes.push_back(lane);
        g.sumoEdgeStrToRoadID[edge.id] = road; g.sumoEdgesRaw.push_back(edge);
    }
    SumoEdgeRaw internal; internal.id = ":i"; internal.isInternal = true;
    SumoLaneRaw lane; lane.id = ":i_0"; lane.index = 0; lane.allow = "bus";
    internal.lanes.push_back(lane); g.sumoEdgesRaw.push_back(internal);
    g.sumoConnectionsRaw[0].via = ":i_0";
    SumoConnectionRaw continuation; continuation.fromEdge = ":i"; continuation.fromLane = 0;
    continuation.toEdge = "r1"; continuation.toLane = 0; g.sumoConnectionsRaw.push_back(continuation);
    initialize(g);
    require(g.road_pair_entry_lanes(0,0,0).empty(),"internal lane class permission ignored");
    g.sumoEdgesRaw.back().lanes[0].allow = "all"; initialize(g);
    require(g.road_pair_entry_lanes(0,0,0) == vector<int>{0},"legal complete internal path rejected");
    g.sumoConnectionsRaw.back().allow = "bus"; initialize(g);
    require(g.road_pair_entry_lanes(0,0,0).empty(),"internal continuation class permission ignored");
    g.sumoConnectionsRaw.back().allow = "all"; g.sumoConnectionsRaw.push_back(continuation); initialize(g);
    require(g.road_pair_entry_lanes(0,0,0).empty(),"ambiguous internal path accepted");
    Graph bad; roads(bad,{1,1,1}); movement(bad,0,1,{{0,0}}); movement(bad,1,2,{{0,0}});
    initialize(bad);
    vector<vector<int>> q{{0,0,0}}, routes{{0,1,2}};
    bad.routeMovementID = {{0}};
    bool threw = false;
    try { bad.initialize_cycle_aware_vehicles(q,routes); } catch (const runtime_error& error) { threw = string(error.what()).find("exactly one movement") != string::npos; }
    require(threw,"truncated route movement sequence silently accepted");
    bad.routeMovementID = {{0,0}}; threw = false;
    try { bad.initialize_cycle_aware_vehicles(q,routes); } catch (const runtime_error& error) { threw = string(error.what()).find("does not match") != string::npos; }
    require(threw,"wrong downstream route movement silently accepted");
    bad.routeMovementID = {{0,1}}; bad.sumoConnectionsRaw[1].allow = "bus"; threw = false;
    try { bad.initialize_cycle_aware_vehicles(q,routes); } catch (const runtime_error& error) { threw = string(error.what()).find("no class-permitted") != string::npos; }
    require(threw,"impossible class on later route occurrence was allowed to strand a vehicle");
    g.sumoEdgesRaw[0].lanes[0].allow = "bus";
    g.routeMovementID = {{}}; routes = {{0}}; threw = false;
    try { g.initialize_cycle_aware_vehicles(q,routes); } catch (const runtime_error& error) { threw = string(error.what()).find("single-road route") != string::npos; }
    require(threw,"single-road route accepted forbidden lane");
}
void occupancy_transaction_guards() {
    Graph g; roads(g,{1,1}); movement(g,0,1,{{0,0}}); initialize(g);
    g.vehicles.resize(2); g.vehicles[0].vehicleID = 0; g.vehicles[1].vehicleID = 1;
    g.roads[0].laneCapacity[0] = 1; g.reserveLaneOccupancy(0,0,0);
    bool duplicate = false, full = false, doubleRelease = false;
    try { g.reserveLaneOccupancy(0,0,0); } catch (const runtime_error&) { duplicate = true; }
    try { g.reserveLaneOccupancy(1,0,0); } catch (const runtime_error&) { full = true; }
    require(duplicate && full && g.roads[0].roadFlow == 1 && g.roadPairReservations == 1,"failed admission mutated occupancy");
    g.releaseLaneOccupancy(0);
    try { g.releaseLaneOccupancy(0); } catch (const runtime_error&) { doubleRelease = true; }
    require(doubleRelease && g.roads[0].roadFlow == 0 && g.roadPairReleases == 1,"double release was silently repaired");
}
vector<vector<pair<int,double>>> run_reference(bool persistent) {
    Graph g; roads(g,{2,2,1,1}); movement(g,0,1,{{0,0},{1,1}}); movement(g,0,2,{{0,0}});
    movement(g,1,0,{{0,0},{1,1}}); movement(g,1,3,{{0,0},{1,0}});
    g.persistentDispatch = persistent;
    vector<vector<int>> routes{{0,1,0,2},{0,1,3},{0,2},{0,1,3},{3},{0,1,0,2}};
    vector<vector<int>> q(routes.size(),vector<int>{0,0,0});
    g.vehicleDepartureTimes = {0,0,1,2,0,3};
    auto result = g.cycle_aware_signal_driven_records(q,routes);
    require(g.finishedVehicleCount == 6 && g.invalidVehicleCount == 0,"reference run did not finish every route");
    require(g.road_pair_validate_runtime(true),"run-end lane conservation failed");
    require(g.basicFeatureSnapshots.size() == 17,"first/final/repeated edge prediction count=" + to_string(g.basicFeatureSnapshots.size()) + " expected=17");
    if (persistent) require(g.dispatchQueueStats.globalMovementVisits == 0,"incremental runtime scanned all movements");
    else require(g.dispatchQueueStats.globalMovementVisits > 0,"reference runtime did not rebuild");
    return result;
}
}

int main() {
    try {
        independent_lanes(); shared_source(); capacity_and_pairs(); projection_and_permissions(); head_insertion_and_stale_candidate();
        internal_permissions_and_route_guards(); occupancy_transaction_guards();
        const auto rebuilt = run_reference(false);
        const auto incremental = run_reference(true);
        require(rebuilt == incremental,"incremental and full-rebuild runtime ETA differ");
        cout << "RoadPairRuntimeTest checks=" << checks << " passed\n";
    } catch (const exception& error) { cerr << "RoadPairRuntimeTest failed: " << error.what() << '\n'; return 1; }
    return 0;
}
