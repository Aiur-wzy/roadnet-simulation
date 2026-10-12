#include "head.h"

#include <iostream>
#include <sstream>

namespace {

int checks = 0;
void check(bool ok, const string& why) {
    ++checks;
    if (!ok) throw runtime_error(why);
}

// Frozen reference of the pre-index implementation. Diagnostics are omitted,
// but every state transition, candidate calculation and trace field is retained.
void full_scan_reference(Graph& g, int freedRoadID, double currentTime,
                         int freedLaneIndex, int releasedByVehicleID,
                         int releasedByMovementID) {
    if (freedRoadID < 0 || g.movementBlockedByDownstream.size() != g.movements.size()) return;
    const long long releaseSequence = g.evalTrajectoryDetails ? ++g.downstreamReleaseSequence : 0;
    for (const auto& m : g.movements) {
        int movementID = m.movementID;
        if (movementID < 0 || movementID >= static_cast<int>(g.movements.size())) continue;
        if (m.toRoadID != freedRoadID) continue;
        const int frontVehicleID = g.getFrontVehicleForMovement(movementID);
        const bool hasFrontVehicle = frontVehicleID >= 0;
        const bool wasBlocked = g.movementBlockedByDownstream[movementID];
        double candidateFirstLabel = INF;
        bool reactivated = false;
        DownstreamReleaseCompetitionRecord trace;
        if (g.evalTrajectoryDetails && hasFrontVehicle) {
            const int bufferID = g.getMovementBufferID(movementID);
            trace.releaseSequence = releaseSequence;
            trace.releaseTime = currentTime;
            trace.releasedByVehicleID = releasedByVehicleID;
            trace.releasedByMovementID = releasedByMovementID;
            trace.freedRoadID = freedRoadID;
            trace.freedLaneIndex = freedLaneIndex;
            trace.movementID = movementID;
            trace.frontVehicleID = frontVehicleID;
            trace.queueSize = (bufferID >= 0 && bufferID < static_cast<int>(g.waitingBuffers.size()))
                ? static_cast<int>(g.waitingBuffers[bufferID].vehicleQueue.size()) : 0;
            trace.movementLabelBefore = g.movementTimeLabel[movementID];
            trace.frontArrivalSecondLabel = g.vehicles[frontVehicleID].arrivalTime;
            trace.signalActiveAtRelease = g.isMovementActive(movementID, currentTime);
            trace.nextGreenTime = g.nextGreenTimeForMovement(movementID, currentTime);
            trace.nextCapacityTime = g.nextAvailableCapacityTime(movementID, currentTime);
            trace.candidateFirstLabel = g.computeMovementAttemptTime(movementID, currentTime);
            trace.downstreamStorageAvailable = g.hasDownstreamLaneStorage(movementID, frontVehicleID, trace.candidateLane);
            trace.wasBlockedByDownstream = wasBlocked;
            trace.wasInDispatchQueue = movementID < static_cast<int>(g.movementInDispatchPQ.size())
                && g.movementInDispatchPQ[movementID];
        }
        if (wasBlocked && hasFrontVehicle) {
            candidateFirstLabel = g.computeMovementAttemptTime(movementID, currentTime);
            if (candidateFirstLabel < INF) {
                g.movementBlockedByDownstream[movementID] = false;
                reactivated = true;
                g.scheduleMovementCandidate(movementID, candidateFirstLabel);
            }
        }
        if (g.evalTrajectoryDetails && hasFrontVehicle) {
            trace.reactivated = reactivated;
            g.downstreamReleaseCompetitionTimeline.push_back(trace);
        }
    }
}

string trace_text(const DownstreamReleaseCompetitionRecord& r) {
    ostringstream out;
    out << setprecision(17) << r.releaseSequence << ',' << r.releaseTime << ','
        << r.releasedByVehicleID << ',' << r.releasedByMovementID << ','
        << r.freedRoadID << ',' << r.freedLaneIndex << ',' << r.movementID << ','
        << r.frontVehicleID << ',' << r.queueSize << ',' << r.movementLabelBefore << ','
        << r.frontArrivalSecondLabel << ',' << r.signalActiveAtRelease << ','
        << r.nextGreenTime << ',' << r.nextCapacityTime << ',' << r.candidateFirstLabel << ','
        << r.downstreamStorageAvailable << ',' << r.candidateLane << ','
        << r.wasBlockedByDownstream << ',' << r.wasInDispatchQueue << ',' << r.reactivated;
    return out.str();
}

void equal_state(const Graph& indexed, const Graph& reference, const string& where) {
    check(indexed.movementTimeLabel == reference.movementTimeLabel, where + ": labels");
    check(indexed.movementPQVersion == reference.movementPQVersion, where + ": versions");
    check(indexed.movementBlockedByDownstream == reference.movementBlockedByDownstream, where + ": blocked flags");
    check(indexed.movementInDispatchPQ == reference.movementInDispatchPQ, where + ": in-PQ flags");
    check(indexed.downstreamReleaseSequence == reference.downstreamReleaseSequence, where + ": release sequence");
    check(indexed.downstreamReleaseCompetitionTimeline.size() == reference.downstreamReleaseCompetitionTimeline.size(),
          where + ": trace length");
    for (size_t i = 0; i < indexed.downstreamReleaseCompetitionTimeline.size(); ++i) {
        check(trace_text(indexed.downstreamReleaseCompetitionTimeline[i]) ==
              trace_text(reference.downstreamReleaseCompetitionTimeline[i]), where + ": trace fields/order");
    }
    auto a = indexed.dispatchPQ;
    auto b = reference.dispatchPQ;
    check(a.size() == b.size(), where + ": raw PQ length (including stale items)");
    while (!a.empty()) {
        const auto x = a.top(), y = b.top();
        check(x.timeLabel == y.timeLabel && x.firstCarArriveTime == y.firstCarArriveTime &&
              x.movementID == y.movementID && x.frontVehicleID == y.frontVehicleID && x.version == y.version,
              where + ": raw PQ pop order");
        a.pop(); b.pop();
    }
    for (size_t i = 0; i < indexed.waitingBuffers.size(); ++i)
        check(indexed.waitingBuffers[i].vehicleQueue == reference.waitingBuffers[i].vehicleQueue,
              where + ": no direct queue discharge");
    for (size_t i = 0; i < indexed.roads.size(); ++i) {
        check(indexed.roads[i].roadFlow == reference.roads[i].roadFlow, where + ": road flow");
        check(indexed.roads[i].laneFlow == reference.roads[i].laneFlow, where + ": lane flow");
    }
}

void add_movement(Graph& g, int from, int to, int lane = 0) {
    const int id = static_cast<int>(g.movements.size());
    Movement m;
    m.movementID = id; m.fromRoadID = from; m.toRoadID = to;
    m.alwaysOpen = true; m.fromLanes = {to_string(lane)}; m.toLanes = {to_string(lane)};
    g.movements.push_back(m);
    WaitingBuffer b;
    b.bufferID = id; b.roadID = from; b.movementID = id;
    g.waitingBuffers.push_back(b);
    g.roads[from].movementIDToWaitingBufferID[id] = id;
}

void roads(Graph& g, int count, int lanes = 1) {
    g.roads.resize(count);
    for (int i = 0; i < count; ++i) {
        g.roads[i].roadID = i;
        g.roads[i].fromNode = i; g.roads[i].toNode = i + 1;
        g.roads[i].length = 100; g.roads[i].speedLimit = 10;
        g.roads[i].minTravelTime = 1; g.roads[i].laneNum = lanes;
    }
}

void enqueue_front(Graph& g, int movementID, double arrival) {
    VehicleLabel v;
    v.vehicleID = static_cast<int>(g.vehicles.size()); v.routeID = v.vehicleID;
    v.currentMovementID = movementID; v.currentBufferID = movementID;
    v.currentRoadID = g.movements[movementID].fromRoadID;
    v.state = VehicleState::WaitingAtIntersection; v.arrivalTime = arrival;
    v.routeRoadIDs = {v.currentRoadID, g.movements[movementID].toRoadID};
    v.routeMovementIDs = {movementID};
    g.vehicles.push_back(v);
    g.waitingBuffers[movementID].vehicleQueue.push_back(v.vehicleID);
    g.reserveLaneOccupancy(v.vehicleID, v.currentRoadID, 0);
}

void fixture(Graph& g, bool details) {
    g.evalTrajectoryDetails = details;
    roads(g, 7, 2);
    add_movement(g, 0, 3);       // Several upstream roads, including two movements from road 0.
    add_movement(g, 1, 3, 1);    // Different lane from freedLaneIndex must NOT be filtered out.
    add_movement(g, 0, 3);
    add_movement(g, 3, 3);       // Self-loop is still an incoming movement of road 3.
    add_movement(g, 2, 4);       // Unrelated destination must remain blocked.
    add_movement(g, 2, 3);       // Empty buffer must remain blocked and produce no trace.
    add_movement(g, 5, 3);       // Not blocked: trace retained but no duplicate scheduling.
    add_movement(g, 6, 3);       // Red now, future green exists.
    add_movement(g, 6, 3);       // Red forever: INF candidate remains blocked.
    g.initializeRoadLaneStorage();
    g.initializeMovementDischargeTimes(0);
    g.movementPQVersion.assign(g.movements.size(), 3);
    g.movementInDispatchPQ.assign(g.movements.size(), false);
    g.movementBlockedByDownstream.assign(g.movements.size(), true);
    for (int id : {0, 1, 2, 3, 4, 6, 7, 8}) enqueue_front(g, id, id == 3 ? 0.0 : 1.0);
    VehicleLabel laneBlocker;
    laneBlocker.vehicleID = static_cast<int>(g.vehicles.size());
    laneBlocker.currentRoadID = 3; laneBlocker.routeRoadIDs = {3};
    laneBlocker.state = VehicleState::RunningOnRoad;
    g.vehicles.push_back(laneBlocker);
    g.roads[3].laneCapacity[1] = 1;
    g.reserveLaneOccupancy(laneBlocker.vehicleID, 3, 1);
    g.movementBlockedByDownstream[6] = false;
    g.scheduleMovementCandidate(6, 7); // Existing queue item must survive unchanged.
    g.movementLaneNextAvailableTime[2][0] = 12.5;
    g.movementTimeLabel[2] = 100;      // Cached stale label is not a scheduling lower bound.
    SignalProgram program;
    program.tlID = "later"; program.cycleLength = 30;
    program.phases = {{0, 15, "r"}, {15, 30, "G"}};
    g.tlIDToSignalProgramID[program.tlID] = 0; g.signalPrograms.push_back(program);
    g.movements[7].alwaysOpen = false; g.movements[7].tlID = "later"; g.movements[7].linkIndex = 0;
    program.tlID = "never"; program.phases = {{0, 30, "r"}};
    g.tlIDToSignalProgramID[program.tlID] = 1; g.signalPrograms.push_back(program);
    g.movements[8].alwaysOpen = false; g.movements[8].tlID = "never"; g.movements[8].linkIndex = 0;
    g.rebuild_incoming_movement_index();
}

void equivalence_checks(bool details) {
    Graph indexed, reference;
    fixture(indexed, details); fixture(reference, details);
    const auto expected = vector<int>{0, 1, 2, 3, 5, 6, 7, 8};
    check(indexed.incomingMovementsByRoad.at(3) == expected, "index preserves original movement order");
    const auto run = [&](int road, double t, int lane, const string& description) {
        indexed.reactivateMovementsBlockedByRoad(road, t, lane, 111, 222);
        full_scan_reference(reference, road, t, lane, 111, 222);
        equal_state(indexed, reference, description);
    };
    run(3, 10, 0, "first release");
    check(!indexed.movementBlockedByDownstream[1], "different destination lane still reactivated");
    check(!indexed.movementBlockedByDownstream[3], "self-loop reactivated without recursion");
    check(indexed.movementBlockedByDownstream[4], "unrelated road not reactivated");
    check(indexed.movementBlockedByDownstream[5], "no-front movement not reactivated");
    check(indexed.movementBlockedByDownstream[8], "no future green remains blocked");
    check(indexed.movementTimeLabel[7] == 15, "red movement schedules at future green");
    check(indexed.movementTimeLabel[2] == 12.5, "capacity used, not stale label");
    check(indexed.movementPQVersion[6] == 4, "unblocked movement is not rescheduled");
    check(indexed.vehicles[0].roadIndex == 0 && indexed.vehicles[3].roadIndex == 0,
          "reactivation does not move vehicles, including self-loop");
    if (details) {
        check(indexed.downstreamReleaseCompetitionTimeline.size() == 7, "all seven fronted matching movements traced");
        check(indexed.downstreamReleaseCompetitionTimeline[5].movementID == 7, "trace keeps source order");
        check(!indexed.downstreamReleaseCompetitionTimeline[1].downstreamStorageAvailable &&
              indexed.downstreamReleaseCompetitionTimeline[1].reactivated,
              "wake-up retains original candidate semantics even when another allowed lane is still full");
    }
    run(3, 10, 1, "duplicate release does not duplicate active candidates");
    const auto mapSize = indexed.incomingMovementsByRoad.size();
    const auto sequence = indexed.downstreamReleaseSequence;
    run(999, 10, -1, "empty incoming bucket");
    check(indexed.incomingMovementsByRoad.size() == mapSize, "lookup does not insert an empty map bucket");
    check(indexed.downstreamReleaseSequence == sequence + (details ? 1 : 0), "empty bucket preserves old release sequence");
    run(-1, 10, -1, "negative road rejected before sequence change");
    indexed.movementBlockedByDownstream.pop_back(); reference.movementBlockedByDownstream.pop_back();
    run(3, 10, -1, "incompatible state size rejected before sequence change");
}

void topology_rebuild_checks() {
    Graph g; fixture(g, false);
    g.incomingMovementsByRoad[999] = {4};
    g.movements[0].toRoadID = 5;
    g.rebuild_incoming_movement_index();
    check(g.incomingMovementsByRoad.count(999) == 0, "explicit rebuild clears stale buckets");
    check(g.incomingMovementsByRoad.at(5) == vector<int>{0}, "same-size topology change gets new destination");
    check(g.incomingMovementsByRoad.at(3) == vector<int>({1, 2, 3, 5, 6, 7, 8}), "old destination loses changed movement");
    // A run-level reset must rebuild even when the number of movements is unchanged.
    g.movements[1].toRoadID = 6;
    g.incomingMovementsByRoad[999] = {1};
    vector<vector<int>> q, routes;
    g.initialize_cycle_aware_vehicles(q, routes);
    check(g.incomingMovementsByRoad.count(999) == 0, "run initialization clears stale index");
    check(g.incomingMovementsByRoad.at(6) == vector<int>{1}, "run initialization rebuilds same-size topology");
    check(g.incomingMovementsByRoad.at(3) == vector<int>({2, 3, 5, 6, 7, 8}), "run initialization does not retain old destination");
    g.movements.clear();
    g.rebuild_incoming_movement_index();
    check(g.incomingMovementsByRoad.empty(), "empty topology leaves empty index");
    for (int invalidID : {-1, 1, 99}) {
        Movement invalid; invalid.movementID = invalidID; invalid.fromRoadID = 0; invalid.toRoadID = 1;
        g.movements = {invalid};
        bool rejected = false;
        try { g.rebuild_incoming_movement_index(); }
        catch (const runtime_error&) { rejected = true; }
        check(rejected, "invalid movementID/vector-index contract fails closed");
    }
}

void chain_release_checks() {
    Graph g;
    g.evalTrajectoryDetails = true; g.travelTimeMode = TravelTimeMode::MIN_TIME;
    g.intersectionCrossingTime = 0; g.defaultDischargeInterval = 2.15;
    roads(g, 3);
    add_movement(g, 0, 1); add_movement(g, 1, 2);
    g.initializeRoadLaneStorage();
    for (auto& road : g.roads) road.laneCapacity[0] = 1;
    g.initializeMovementDischargeTimes(0);
    g.movementPQVersion.assign(2, 0); g.movementInDispatchPQ.assign(2, false);
    g.movementBlockedByDownstream.assign(2, true);
    enqueue_front(g, 0, 1); enqueue_front(g, 1, 1);
    g.vehicles[0].routeRoadIDs = {0, 1, 2}; g.vehicles[0].routeMovementIDs = {0, 1};
    VehicleLabel terminal;
    terminal.vehicleID = 2; terminal.routeID = 2; terminal.currentRoadID = 2;
    terminal.routeRoadIDs = {2}; terminal.state = VehicleState::RunningOnRoad;
    g.vehicles.push_back(terminal);
    g.reserveLaneOccupancy(2, 2, 0);
    g.ETA_result_cycle_aware = {{{0, 0}}, {{1, 0}}, {{2, 0}}};
    g.vehicleRoadTimeline.resize(3); g.vehicleMovementTimeline.resize(3);
    g.rebuild_incoming_movement_index();
    g.scheduleFinalRoadExit(2, 10);
    check(!g.processFinalRoadExitsUntil(9), "C occupancy does not release before final exit event");
    check(g.processFinalRoadExitsUntil(10), "C final exit event fires");
    check(!g.movementBlockedByDownstream[1] && g.movementBlockedByDownstream[0],
          "C release wakes only B-to-C, never recursively A-to-B");
    check(g.roads[0].roadFlow == 1 && g.roads[1].roadFlow == 1 && g.roads[2].roadFlow == 0,
          "reactivation alone leaves A and B occupancy unchanged");
    check(g.vehicles[0].roadIndex == 0 && g.vehicles[1].roadIndex == 0, "wake-up alone advances no route");
    check(g.dispatchPQ.size() == 1 && g.dispatchPQ.top().movementID == 1, "only B-to-C candidate enqueued");
    const auto firstCandidate = g.dispatchPQ.top(); g.dispatchPQ.pop();
    check(g.isDispatchCandidateValid(firstCandidate), "actual B-to-C candidate valid");
    int front = -1, lane = -1;
    check(g.getDischargeBlockReason(1, 10, front, lane) == DischargeBlockReason::None,
          "B-to-C discharge passes actual signal/capacity/storage checks");
    const double previousLabel = g.movementTimeLabel[1];
    const auto result = g.dischargeOneVehicle(1, 10);
    check(result.vehicleID == 1 && result.releasedRoadID == 1, "actual discharge frees B");
    check(g.movementBlockedByDownstream[0], "discharge mutation alone does not recursively wake A");
    g.handleSuccessfulDischargePostUpdate(1, 10, previousLabel, 1, result);
    check(!g.movementBlockedByDownstream[0], "successful event post-update wakes A-to-B");
    check(g.roads[0].roadFlow == 1 && g.roads[1].roadFlow == 0 && g.roads[2].roadFlow == 1,
          "B-to-C changes only its actual storage");
    check(g.vehicles[0].roadIndex == 0, "A vehicle still waits until its own discharge event");
    check(g.dispatchPQ.size() == 1 && g.dispatchPQ.top().movementID == 0, "A candidate now independently queued");
    check(g.downstreamReleaseCompetitionTimeline.size() == 2, "one immediate upstream trace per actual release");
    check(g.downstreamReleaseCompetitionTimeline[0].freedRoadID == 2 &&
          g.downstreamReleaseCompetitionTimeline[0].movementID == 1 &&
          g.downstreamReleaseCompetitionTimeline[1].freedRoadID == 1 &&
          g.downstreamReleaseCompetitionTimeline[1].movementID == 0,
          "release trace follows C exit then B discharge, not transitive recursion");
    check(g.getDischargeBlockReason(0, 10, front, lane) == DischargeBlockReason::None,
          "A-to-B independently passes discharge checks");
    const auto next = g.dischargeOneVehicle(0, 10);
    check(next.vehicleID == 0 && g.vehicles[0].roadIndex == 1, "A moves only on its own discharge");
    check(g.roads[0].roadFlow == 0 && g.roads[1].roadFlow == 1 && g.roads[2].roadFlow == 1,
          "capacity-one chain never overfills B or C");
}

} // namespace

int main() {
    try {
        equivalence_checks(true);
        equivalence_checks(false);
        topology_rebuild_checks();
        chain_release_checks();
        cout << "PASS upstream_release_index checks=" << checks << '\n';
        return 0;
    } catch (const exception& e) {
        cerr << "FAIL upstream_release_index checks=" << checks << ": " << e.what() << '\n';
        return 1;
    }
}
