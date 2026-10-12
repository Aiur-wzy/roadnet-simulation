#include "head.h"
#include <iomanip>

namespace {
int checks = 0;
void require(bool ok, const string& message) {
    ++checks;
    if (!ok) throw runtime_error(message);
}
void setup_roads(Graph& g, const vector<int>& lanes, bool enabled = true) {
    g.mergeRoadPairMovements = true;
    g.laneEntryReselection = enabled;
    g.laneBorrowing = false;
    g.persistentDispatch = true;
    g.travelTimeMode = TravelTimeMode::SPEED_NET;
    g.speedNetUseAcceleration = false;
    g.speedNetUseFollowing = false;
    g.speedNetDepartPosition = false;
    g.defaultDischargeInterval = 2.0;
    g.intersectionCrossingTime = 0.0;
    g.sumoNetworkWithoutInternalLinks = true;
    g.evalTrajectoryDetails = true;
    g.enableBasicFeatureLogging = true;
    VehicleType type; type.id = "DEFAULT_VEHTYPE";
    g.sumoVehicleTypes[type.id] = type;
    for (int count : lanes) {
        RoadSegment r;
        r.roadID = static_cast<int>(g.roads.size());
        r.laneNum = count; r.length = 100.0; r.speedLimit = 10.0;
        g.roads.push_back(r);
    }
}
int add_movement(Graph& g, int from, int to, const vector<pair<int,int>>& pairs) {
    Movement m;
    m.movementID = static_cast<int>(g.movements.size());
    m.fromRoadID = from; m.toRoadID = to; m.alwaysOpen = true;
    for (const auto& p : pairs) {
        SumoConnectionRaw c;
        c.fromEdge = "r" + to_string(from); c.toEdge = "r" + to_string(to);
        c.fromLane = p.first; c.toLane = p.second;
        m.connectionIndices.push_back(static_cast<int>(g.sumoConnectionsRaw.size()));
        g.sumoConnectionsRaw.push_back(c);
        m.fromLanes.push_back(to_string(p.first)); m.toLanes.push_back(to_string(p.second));
    }
    sort(m.fromLanes.begin(), m.fromLanes.end());
    m.fromLanes.erase(unique(m.fromLanes.begin(),m.fromLanes.end()),m.fromLanes.end());
    sort(m.toLanes.begin(), m.toLanes.end());
    m.toLanes.erase(unique(m.toLanes.begin(),m.toLanes.end()),m.toLanes.end());
    g.movements.push_back(m);
    WaitingBuffer b;
    b.bufferID = static_cast<int>(g.waitingBuffers.size()); b.roadID = from; b.movementID = m.movementID;
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
int append_vehicle(Graph& g, VehicleLabel v) {
    v.vehicleID = v.routeID = static_cast<int>(g.vehicles.size());
    g.vehicles.push_back(v);
    g.vehicleTypeIDs.push_back("DEFAULT_VEHTYPE");
    g.vehicleRoadTimeline.resize(g.vehicles.size());
    g.vehicleMovementTimeline.resize(g.vehicles.size());
    g.ETA_result_cycle_aware.resize(g.vehicles.size());
    return v.vehicleID;
}
// A held capacity fixture, not a dispatched road vehicle. Only explicit test
// release can remove it; full-run lifecycle is tested separately below.
int occupant(Graph& g, int road, int lane, double space = 7.5) {
    VehicleLabel v; v.currentRoadID = road; v.state = VehicleState::RunningOnRoad;
    v.routeRoadIDs = {road}; v.arrivalTime = 1000.0;
    const int id = append_vehicle(g,v);
    VehicleType type; type.id = "held_" + to_string(id); type.length = space; type.minGap = 0.0;
    g.sumoVehicleTypes[type.id] = type; g.vehicleTypeIDs[id] = type.id;
    g.reserveLaneOccupancy(id,road,lane);
    return id;
}
void release_occupant(Graph& g, int id) {
    g.releaseLaneOccupancy(id);
    g.vehicles[id].finished = true; g.vehicles[id].state = VehicleState::Finished;
}
int waiting_vehicle(Graph& g, int m, int lane, const vector<int>& route,
                    const vector<int>& routeMovements, double ready = 0.0) {
    VehicleLabel v;
    v.currentMovementID = m; v.currentBufferID = g.getMovementBufferID(m);
    v.currentRoadID = g.movements[m].fromRoadID;
    v.routeRoadIDs = route; v.routeMovementIDs = routeMovements;
    v.state = VehicleState::WaitingAtIntersection; v.arrivalTime = ready;
    const int id = append_vehicle(g,v);
    g.reserveLaneOccupancy(id,v.currentRoadID,lane);
    g.insertVehicleToBufferOrdered(v.currentBufferID,id);
    g.scheduleInsertedVehicleIfMovementFront(m,id,0.0);
    return id;
}
void three_roads(Graph& g, const vector<pair<int,int>>& incoming = {{0,0}},
                 const vector<pair<int,int>>& outgoing = {{0,0},{1,0}}, bool enabled = true) {
    setup_roads(g,{1,2,1},enabled);
    add_movement(g,0,1,incoming); add_movement(g,1,2,outgoing); initialize(g);
}
string committed_state(const Graph& g) {
    ostringstream out; out << setprecision(17);
    out << g.roadPairReservations << ':' << g.roadPairReleases << ':' << g.basicFeatureSnapshots.size();
    for (const auto& r : g.roads) {
        out << '/' << r.roadFlow;
        for (int f : r.laneFlow) out << ',' << f;
        for (double x : r.laneOccupiedLength) out << ',' << x;
        for (double x : r.laneWaitingBufferLength) out << ',' << x;
    }
    for (const auto& v : g.vehicles)
        out << ';' << v.roadIndex << ',' << v.currentRoadID << ',' << v.currentMovementID << ',' << v.currentBufferID
            << ',' << v.occupiedRoadID << ',' << v.occupiedLaneIndex << ',' << v.storageLaneIndex << ',' << v.occupiedLength
            << ',' << v.arrivalTime << ',' << static_cast<int>(v.state) << ',' << v.finished << ',' << v.valid
            << ',' << v.lastWaitingDuration << ',' << v.lastDischargeHadWaiting << ',' << v.hasWaitingBeforeCurrentRoad
            << ',' << v.entrySpeedProxy << ',' << v.estimatedRoadExitSpeed << ',' << v.estimatedExitRoadIndex;
    for (const auto& b : g.waitingBuffers) { out << 'b'; for (int v : b.vehicleQueue) out << ',' << v; }
    for (const auto& road : g.roadPairLaneQueues) for (const auto& q : road) {
        out << 'q'; for (int v : q) out << ',' << v;
    }
    for (const auto& road : g.roadPairLaneNextAvailable) for (double x : road) out << 's' << x;
    for (const auto& pool : g.movementLaneNextAvailableTime) for (double x : pool) out << 'p' << x;
    for (const auto& rows : g.vehicleRoadTimeline) out << 'r' << rows.size();
    for (const auto& rows : g.vehicleMovementTimeline) out << 'm' << rows.size();
    for (const auto& rows : g.ETA_result_cycle_aware) out << 'e' << rows.size();
    out << 'f' << g.finalRoadExitPQ.size();
    return out.str();
}
void default_and_configuration() {
    Graph defaults;
    require(!defaults.laneEntryReselection,"new strategy must default off");
    for (bool borrowing : {false,true}) {
        Graph g; g.laneEntryReselection = true;
        g.mergeRoadPairMovements = borrowing; g.laneBorrowing = borrowing;
        bool rejected = false;
        try { g.initialize_road_pair_runtime(0.0); }
        catch (const runtime_error& e) { rejected = string(e.what()).find("requires") != string::npos; }
        require(rejected,"incompatible road-pair/borrowing configuration accepted");
    }
}
void expanded_intersection_and_mode_cache() {
    Graph g; three_roads(g);
    occupant(g,1,0); occupant(g,1,0);
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    const auto expanded = g.road_pair_lane_options(0,id);
    require(expanded.size() == 2,"nonempty E intersection S did not expand to all S");
    RoadPairChoice choice;
    require(g.road_pair_select_lane(0,id,choice) && choice.entryLane == 0 && choice.logicalLane == 1,
            "expanded selection did not choose less-loaded service lane");
    g.laneEntryReselection = false;
    require(g.road_pair_lane_options(0,id).size() == 1,"on-mode options leaked through off-mode cache");
    require(g.road_pair_select_lane(0,id,choice) && choice.logicalLane == 0,"off intersection contract changed");
    g.laneEntryReselection = true;
    require(g.road_pair_lane_options(0,id).size() == 2,"off-mode cache contaminated restored on mode");
    require(g.basicFeatureSnapshots.empty(),"pure selector evaluated travel time");
}
void dynamic_entry_and_storage_guards() {
    Graph entry; three_roads(entry,{{0,0}},{{1,0}});
    entry.roads[1].laneCapacity[0] = 1;
    occupant(entry,1,0);
    const int id = waiting_vehicle(entry,0,0,{0,1,2},{0,1});
    RoadPairChoice choice;
    require(!entry.road_pair_select_lane(0,id,choice),"full real entry was bypassed using free logical lane");
    entry.laneEntryReselection = false;
    require(entry.road_pair_select_lane(0,id,choice) && choice.logicalLane == 1,"off old projection behavior changed");
    Graph storage; three_roads(storage,{{0,0}},{{1,0}});
    storage.roads[1].laneCapacity[1] = 1; occupant(storage,1,1);
    const int other = waiting_vehicle(storage,0,0,{0,1,2},{0,1});
    require(!storage.road_pair_select_lane(0,other,choice),"free entry became unbounded waiting-for-lane pool");
    const string before = committed_state(storage);
    require(storage.dischargeOneVehicle(0,0).vehicleID < 0,"blocked downstream commit unexpectedly succeeded");
    require(before == committed_state(storage),"failed storage commit changed queue/flow/clock/features");
}
void all_actual_entries_survive_cache() {
    Graph g; setup_roads(g,{1,3,1});
    add_movement(g,0,1,{{0,0},{0,2}}); add_movement(g,1,2,{{1,0}}); initialize(g);
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    require(g.road_pair_lane_options(0,id).size() == 2,"cache discarded a real entry alternative");
    g.roads[1].laneCapacity[0] = 1; const int held = occupant(g,1,0);
    RoadPairChoice choice;
    require(g.road_pair_select_lane(0,id,choice) && choice.entryLane == 2 && choice.logicalLane == 1 && choice.connectionIndex == 1,
            "cached first entry hid available alternative raw connection");
    release_occupant(g,held);
    g.roads[1].laneCapacity[2] = 1; occupant(g,1,2);
    require(g.road_pair_select_lane(0,id,choice) && choice.entryLane == 0 && choice.connectionIndex == 0,
            "dynamic entry availability was frozen in static cache");
}
void deterministic_load_rank() {
    Graph g; setup_roads(g,{1,3,1});
    add_movement(g,0,1,{{0,1},{0,1}}); add_movement(g,1,2,{{0,0},{1,0},{2,0}}); initialize(g);
    occupant(g,1,0,8); const int middle = occupant(g,1,1,8); occupant(g,1,2,6);
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    RoadPairChoice choice;
    require(g.road_pair_select_lane(0,id,choice) && choice.logicalLane == 2,"occupied length did not precede lateral distance");
    // Keep accounting coherent while making the three loads identical.
    g.roads[1].laneOccupiedLength[2] = 8; g.vehicles[2].occupiedLength = 8;
    require(g.road_pair_select_lane(0,id,choice) && choice.logicalLane == 1,"equal loads failed prefer no lateral move");
    require(choice.connectionIndex == 0,"equal endpoints did not use stable raw connection tie-break");
    g.roads[1].laneCapacity[1] = 1;
    require(!g.road_pair_select_lane(0,id,choice),"distance tie rule bypassed full shared real entry");
    release_occupant(g,middle);
    require(g.road_pair_select_lane(0,id,choice) && choice.logicalLane == 1,"released entry/storage lane failed lowest-flow priority");
}
void actual_source_and_permissions() {
    Graph source; setup_roads(source,{2,2,1});
    add_movement(source,0,1,{{0,0},{1,1}}); add_movement(source,1,2,{{0,0},{1,0}}); initialize(source);
    source.roads[1].laneCapacity[0] = 1; occupant(source,1,0);
    const int id = waiting_vehicle(source,0,0,{0,1,2},{0,1});
    RoadPairChoice choice;
    require(!source.road_pair_select_lane(0,id,choice),"vehicle used another actual source lane's entry connection");
    Graph permissions; setup_roads(permissions,{1,3,1});
    add_movement(permissions,0,1,{{0,0},{0,1}}); add_movement(permissions,1,2,{{0,0},{1,0},{2,0}});
    permissions.sumoConnectionsRaw[0].allow = "bus";
    permissions.sumoConnectionsRaw[3].disallow = "passenger";
    permissions.sumoConnectionsRaw[4].allow = "bus";
    initialize(permissions);
    const int p = waiting_vehicle(permissions,0,0,{0,1,2},{0,1});
    const auto opts = permissions.road_pair_lane_options(0,p);
    require(opts.size() == 1 && opts[0].connectionIndex == 1 && opts[0].entryLane == 1 && opts[0].logicalLane == 0,
            "entry or next-movement vehicle-class permission ignored");
}
void successful_transaction_and_crossing() {
    Graph g; three_roads(g);
    occupant(g,1,0); occupant(g,1,0);
    g.sumoNetworkWithoutInternalLinks = false; g.intersectionCrossingTime = 3.4;
    g.movements[0].tlID = "fixture_tls"; // known controlled crossing, always-open service fixture
    g.roadPairLaneNextAvailable[1][1] = 73.0;
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    const uint64_t reservations = g.roadPairReservations, releases = g.roadPairReleases;
    g.process_discharge_window(0,1);
    const auto& v = g.vehicles[id];
    require(v.roadIndex == 1 && v.currentRoadID == 1 && v.currentMovementID == 1 && v.occupiedLaneIndex == 1 && v.storageLaneIndex == 1,
            "successful transfer did not commit final logical/storage lane once");
    require(g.roads[0].roadFlow == 0 && g.roads[1].roadFlow == 3 && g.roads[1].laneFlow == vector<int>({2,1}),
            "entry lane was reserved or road occupancy duplicated");
    require(g.roadPairReservations == reservations + 1 && g.roadPairReleases == releases + 1,"transfer did not reserve/release exactly once");
    require(g.waitingBuffers[0].vehicleQueue.empty() && g.waitingBuffers[1].vehicleQueue == deque<int>({id}),"WB membership duplicated/lost");
    require(g.roadPairLaneQueues[0][0].empty() && g.roadPairLaneQueues[1][1] == deque<int>({id}),"source lane index not moved exactly once");
    require(g.roadPairLaneNextAvailable[0][0] == 2 && g.roadPairLaneNextAvailable[1][1] == 73,
            "source resource not consumed or receiving service clock changed");
    require(g.movementLaneNextAvailableTime[1][0] == 0,"entry consumed next movement pool");
    require(g.basicFeatureSnapshots.size() == 1 && g.vehicleRoadTimeline[id].size() == 1 && g.vehicleMovementTimeline[id].size() == 1,
            "transfer predicted or recorded more than once");
    const auto& f = g.basicFeatureSnapshots[0];
    require(f.roadID == 1 && f.route_index == 1 && f.laneIndex == 1 && f.road_flow == 3 && f.lane_flow == 1,
            "prediction features not taken on selected s after self reservation");
    require(f.lane_waiting_buffer_length == 0,"prediction counted self as already-enqueued ahead member");
    require(abs(f.time - 3.4) < 1e-12 && g.vehicleRoadTimeline[id][0].intersectionCrossingTime == 3.4,
            "nonzero crossing t versus t+crossing contract changed");
    const auto& t = g.vehicleMovementTimeline[id][0];
    require(t.entryLane == 0 && t.storageLane == 1 && t.serviceLane == 0 && t.connectionIndex == 0,"trace lost real entry connection");
    require(g.road_pair_validate_runtime(false),"successful transfer broke runtime ledger");
}
void stale_entry_candidate_is_atomic() {
    Graph g; three_roads(g,{{0,0}},{{1,0}});
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    require(g.movementInDispatchPQ[0] && g.roadPairChoices[0].logicalLane == 1,"fixture lacks preview candidate");
    g.roads[1].laneCapacity[0] = 1; occupant(g,1,0);
    const string before = committed_state(g);
    require(g.dischargeOneVehicle(0,0).vehicleID < 0,"stale entry admission bypassed fresh capacity check");
    require(before == committed_state(g),"stale-entry failure mutated committed queue/flow/speed/clock/prediction state");
    g.process_discharge_window(0,1);
    require(g.vehicles[id].roadIndex == 0 && !g.movementInDispatchPQ[0],"blocked stale candidate was committed or left live");
}
void commit_reselects_current_least_load() {
    Graph g; three_roads(g);
    const int held0 = occupant(g,1,0), held1 = occupant(g,1,1);
    occupant(g,1,1);
    const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
    require(g.roadPairChoices[0].logicalLane == 0,"fixture old lane should be least loaded");
    release_occupant(g,held1); release_occupant(g,held0);
    // B0=0,B1=1: add a fresh B0 occupant so old s remains feasible but B1 can
    // become strictly lighter after its final held car is removed.
    occupant(g,1,0); release_occupant(g,2);
    require(g.roads[1].laneFlow == vector<int>({1,0}),"fixture current load mismatch");
    const auto result = g.dischargeOneVehicle(0,0);
    require(result.vehicleID == id && g.vehicles[id].occupiedLaneIndex == 1,"commit retained old feasible but no-longer-best s");
    require(g.basicFeatureSnapshots.size() == 1 && g.basicFeatureSnapshots[0].laneIndex == 1,"fresh chosen s not used by exactly one prediction");
}
void competing_upstreams_last_slot() {
    Graph g; setup_roads(g,{1,1,2,1});
    add_movement(g,0,2,{{0,0}}); add_movement(g,1,2,{{0,0}}); add_movement(g,2,3,{{1,0}}); initialize(g);
    g.roads[2].laneCapacity[1] = 1;
    const int a = waiting_vehicle(g,0,0,{0,2,3},{0,2});
    const int b = waiting_vehicle(g,1,0,{1,2,3},{1,2});
    require(g.movementInDispatchPQ[0] && g.movementInDispatchPQ[1],"both competitors must preview last slot");
    g.process_discharge_window(0,1);
    require((g.vehicles[a].roadIndex == 1) + (g.vehicles[b].roadIndex == 1) == 1,"two upstreams committed one final slot");
    require(g.roads[2].roadFlow == 1 && g.roads[2].laneFlow == vector<int>({0,1}),"competition duplicated entry/storage occupancy");
    require(g.basicFeatureSnapshots.size() == 1 && g.roadPairReservations == 3 && g.roadPairReleases == 1,"losing candidate consumed transaction/prediction");
    require(g.road_pair_validate_runtime(false),"last-slot competition broke lane membership");
}
void either_capacity_release_wakes() {
    for (int blockedLane : {0,1}) {
        Graph g; three_roads(g,{{0,0}},{{1,0}});
        g.roads[1].laneCapacity[blockedLane] = 1;
        const int held = occupant(g,1,blockedLane);
        const int id = waiting_vehicle(g,0,0,{0,1,2},{0,1});
        require(!g.movementInDispatchPQ[0],"fixture must be blocked by entry or storage");
        release_occupant(g,held);
        g.reactivateMovementsBlockedByRoad(1,5,blockedLane,held,-1);
        require(g.movementInDispatchPQ[0] && g.roadPairChoices[0].vehicleID == id && g.roadPairChoices[0].time == 5,
                "release of entry or storage lane failed local incoming wake");
        g.process_discharge_window(5,6);
        require(g.vehicles[id].roadIndex == 1,"awakened source did not transfer");
        require(g.dispatchQueueStats.globalMovementVisits == 0,"capacity wake scanned all movements");
    }
}
void equal_entry_storage_and_final_boundary() {
    Graph middle; three_roads(middle,{{0,0}},{{0,0},{1,0}});
    middle.roads[1].laneCapacity = {1,1}; occupant(middle,1,1);
    const int through = waiting_vehicle(middle,0,0,{0,1,2},{0,1});
    RoadPairChoice same;
    require(middle.road_pair_select_lane(0,through,same) && same.entryLane == 0 && same.logicalLane == 0,
            "nonfinal e=s single free slot required two capacity units");
    const auto reservationBefore = middle.roadPairReservations;
    middle.process_discharge_window(0,1);
    require(middle.vehicles[through].roadIndex == 1 && middle.roads[1].laneFlow == vector<int>({1,1}) &&
            middle.roadPairReservations == reservationBefore + 1 && middle.basicFeatureSnapshots.size() == 1,
            "nonfinal e=s transfer counted capacity or prediction twice");
    Graph g; setup_roads(g,{1,2}); add_movement(g,0,1,{{0,1}}); initialize(g);
    g.roads[1].laneCapacity[1] = 1;
    const int id = waiting_vehicle(g,0,0,{0,1},{0});
    const auto opts = g.road_pair_lane_options(0,id);
    require(opts.size() == 1 && opts[0].entryLane == 1 && opts[0].logicalLane == 1,"final road invented next-movement service lane");
    g.process_discharge_window(0,1);
    require(g.vehicles[id].roadIndex == 1 && g.vehicles[id].queuedForFinalExit && g.finalRoadExitPQ.size() == 1,
            "e=s single free slot required two spaces or lost final exit");
    require(g.roads[1].laneFlow == vector<int>({0,1}) && g.roadPairReservations == 2 && g.roadPairReleases == 1,
            "final e=s occupancy was counted twice");
    require(g.waitingBuffers[0].vehicleQueue.empty() && g.roadPairLaneQueues[1][1].empty(),"final road fabricated exit WB/source membership");
    const double eta = g.vehicles[id].arrivalTime;
    g.processFinalRoadExitsUntil(eta);
    require(g.vehicles[id].finished && g.roadPairReservations == g.roadPairReleases && g.road_pair_validate_runtime(true),
            "final boundary did not release only its single slot");
}
void first_road_assignment_is_not_reselected_while_waiting() {
    Graph g; setup_roads(g,{2,1}); add_movement(g,0,1,{{0,0},{1,0}}); initialize(g);
    g.roads[0].laneCapacity = {1,1};
    const int held0 = occupant(g,0,0);
    VehicleLabel v; v.currentRoadID = 0; v.currentMovementID = 0; v.currentBufferID = 0;
    v.routeRoadIDs = {0,1}; v.routeMovementIDs = {0}; v.state = VehicleState::NotDeparted;
    const int id = append_vehicle(g,v);
    require(g.enqueueDepartingVehicle(id,0) && g.vehicles[id].assignedEntryLaneIndex == 1,"first road ignored existing assignment policy");
    const int held1 = occupant(g,0,1); release_occupant(g,held0);
    g.processEntryDepartureQueuesAtTime(0);
    require(g.vehicles[id].queuedForEntry && g.vehicles[id].occupiedRoadID < 0 && g.basicFeatureSnapshots.empty(),
            "waiting first-road vehicle was silently reassigned to another lane");
    release_occupant(g,held1); g.processEntryDepartureQueuesAtTime(1);
    require(!g.vehicles[id].queuedForEntry && g.vehicles[id].occupiedLaneIndex == 1 && g.basicFeatureSnapshots.size() == 1,
            "fixed first-road assignment failed ordinary release retry");
}
void missing_next_buffer_fails_before_pop() {
    Graph g; three_roads(g);
    waiting_vehicle(g,0,0,{0,1,2},{0,1});
    g.roads[1].movementIDToWaitingBufferID.erase(1);
    const string before = committed_state(g);
    bool rejected = false;
    try { rejected = g.dischargeOneVehicle(0,0).vehicleID < 0; }
    catch (const runtime_error& e) {
        const string message = e.what();
        rejected = message.find("buffer") != string::npos || message.find("Buffer") != string::npos;
    }
    require(rejected,"missing next buffer was not rejected before mutation");
    require(before == committed_state(g),"missing next buffer left a half-committed transfer");
}
vector<vector<pair<int,double>>> reference_run(bool persistent) {
    Graph g; setup_roads(g,{2,3,1,1});
    add_movement(g,0,1,{{0,0},{1,0},{1,2}}); add_movement(g,0,2,{{0,0}});
    add_movement(g,1,0,{{0,0},{1,1},{2,0}}); add_movement(g,1,3,{{0,0},{1,0},{2,0}});
    g.persistentDispatch = persistent;
    vector<vector<int>> routes{{0,1,0,2},{0,1,3},{0,2},{0,1,3},{3},{0,1,0,2}};
    vector<vector<int>> query(routes.size(),vector<int>{0,0,0});
    g.vehicleDepartureTimes = {0,0,1,2,0,3};
    const auto result = g.cycle_aware_signal_driven_records(query,routes);
    require(g.finishedVehicleCount == 6 && g.invalidVehicleCount == 0,"bounded reference routes did not finish");
    require(g.basicFeatureSnapshots.size() == 17 && g.roadPairReservations == 17 && g.roadPairReleases == 17,
            "first/repeated/final/single-road visits not predicted/reserved/released exactly once");
    require(g.road_pair_validate_runtime(true),"reference end-state not empty and conserved");
    for (const auto& f : g.basicFeatureSnapshots) require(f.lane_flow >= 1 && f.road_flow >= f.lane_flow,"feature snapshot excluded reserved self");
    require(persistent ? g.dispatchQueueStats.globalMovementVisits == 0 : g.dispatchQueueStats.globalMovementVisits > 0,
            "incremental/rebuild paths not actually exercised");
    return result;
}
}

int main() {
    try {
        default_and_configuration(); expanded_intersection_and_mode_cache(); dynamic_entry_and_storage_guards();
        all_actual_entries_survive_cache(); deterministic_load_rank(); actual_source_and_permissions();
        successful_transaction_and_crossing(); stale_entry_candidate_is_atomic(); commit_reselects_current_least_load();
        competing_upstreams_last_slot(); either_capacity_release_wakes(); equal_entry_storage_and_final_boundary();
        first_road_assignment_is_not_reselected_while_waiting(); missing_next_buffer_fails_before_pop();
        const auto rebuilt = reference_run(false), incremental = reference_run(true);
        require(rebuilt == incremental,"on-mode incremental and rebuild per-vehicle ETA differ");
        cout << "LaneEntryReselectionTest checks=" << checks << " passed\n";
    } catch (const exception& e) {
        cerr << "LaneEntryReselectionTest failed: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
