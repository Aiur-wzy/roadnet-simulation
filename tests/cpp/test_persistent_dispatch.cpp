#include "head.h"

#include <chrono>
#include <cstdio>
#include <sstream>

namespace {

int checks = 0;
void check(bool condition, const string& message) {
    ++checks;
    if (!condition) throw runtime_error(message);
}
void near(double actual, double expected, const string& message) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= 1e-12, message);
}

string candidate_text(const DispatchCandidate& c) {
    ostringstream out;
    out << setprecision(17) << c.timeLabel << ',' << c.firstCarArriveTime << ','
        << c.movementID << ',' << c.frontVehicleID << ',' << c.version;
    return out.str();
}

vector<string> valid_keys(Graph& g) {
    vector<string> result;
    auto heap = g.dispatchPQ;
    while (!heap.empty()) {
        const auto c = heap.top(); heap.pop();
        if (g.isDispatchCandidateValid(c)) result.push_back(candidate_text(c));
    }
    return result;
}

void check_live_index(Graph& g, const string& where) {
    vector<int> count(g.movements.size(), 0);
    for (size_t i = 0; i < g.activeDispatchMovements.size(); ++i) {
        const int m = g.activeDispatchMovements[i];
        check(m >= 0 && m < static_cast<int>(count.size()), where + ": active ID valid");
        ++count[m];
        check(g.dispatchActivePosition[m] == static_cast<int>(i), where + ": active reverse position");
        check(g.movementInDispatchPQ[m], where + ": active implies live");
        check(g.isDispatchCandidateValid(g.latestDispatchCandidates[m]), where + ": latest active token valid");
    }
    auto heap = g.dispatchPQ;
    vector<int> physicalLive(g.movements.size(), 0);
    while (!heap.empty()) {
        const auto c = heap.top(); heap.pop();
        if (g.isDispatchCandidateValid(c)) ++physicalLive[c.movementID];
    }
    for (size_t m = 0; m < count.size(); ++m) {
        check(count[m] == (g.movementInDispatchPQ[m] ? 1 : 0), where + ": no duplicate active ID");
        check(physicalLive[m] == count[m], where + ": exactly one physical live token per active movement");
        if (!count[m]) check(g.dispatchActivePosition[m] == -1, where + ": inactive position sentinel");
    }
}

void base_graph(Graph& g, int movementCount = 3) {
    g.persistentDispatch = true;
    g.travelTimeMode = TravelTimeMode::MIN_TIME;
    g.intersectionCrossingTime = 0;
    g.defaultDischargeInterval = 2.15;
    g.roads.resize(movementCount + 1);
    for (int i = 0; i <= movementCount; ++i) {
        auto& r = g.roads[i];
        r.roadID = i; r.fromNode = i; r.toNode = i + 1;
        r.length = 100; r.speedLimit = 10; r.minTravelTime = 10; r.laneNum = 1;
    }
    for (int i = 0; i < movementCount; ++i) {
        Movement m;
        m.movementID = i; m.fromRoadID = i; m.toRoadID = movementCount;
        m.alwaysOpen = true; m.fromLanes = {"0"}; m.toLanes = {"0"};
        g.movements.push_back(m);
        WaitingBuffer b;
        b.bufferID = i; b.roadID = i; b.movementID = i;
        g.waitingBuffers.push_back(b);
        g.roads[i].movementIDToWaitingBufferID[i] = i;
    }
    g.initializeRoadLaneStorage();
    g.initializeMovementDischargeTimes(0);
    g.reset_dispatch_state();
    g.rebuild_incoming_movement_index();
}

int add_vehicle(Graph& g, int movementID, double arrival) {
    VehicleLabel v;
    v.vehicleID = static_cast<int>(g.vehicles.size()); v.routeID = v.vehicleID;
    v.currentRoadID = g.movements[movementID].fromRoadID;
    v.currentMovementID = movementID; v.currentBufferID = movementID;
    v.arrivalTime = arrival; v.state = VehicleState::WaitingAtIntersection;
    v.routeRoadIDs = {v.currentRoadID, g.movements[movementID].toRoadID};
    v.routeMovementIDs = {movementID};
    g.vehicles.push_back(v);
    g.reserveLaneOccupancy(v.vehicleID, v.currentRoadID, 0);
    g.insertVehicleToBufferOrdered(movementID, v.vehicleID);
    return v.vehicleID;
}

void token_lifecycle_checks() {
    Graph g; base_graph(g);
    add_vehicle(g, 0, 1);
    g.scheduleMovementCandidate(0, 5);
    const auto old = g.latestDispatchCandidates[0];
    g.scheduleMovementCandidate(0, 5);
    check(g.dispatchPQ.size() == 1 && g.movementPQVersion[0] == old.version,
          "identical appointment is idempotent");
    g.scheduleMovementCandidate(0, 10);
    const auto current = g.latestDispatchCandidates[0];
    check(current.version != old.version, "replacement gets new generation");
    check(!g.isDispatchCandidateValid(old) && g.isDispatchCandidateValid(current), "only latest generation valid");
    check(g.dispatchPQ.size() == 2 && g.activeDispatchMovements.size() == 1,
          "two physical versions but one logical candidate");
    g.prepare_dispatch_window(0, 6);
    g.process_discharge_window(0, 6);
    check(g.dispatchQueueStats.stalePops > 0, "old early version was actually popped");
    check(g.movementInDispatchPQ[0] && g.isDispatchCandidateValid(current),
          "discarding stale early item never clears newer live appointment");
    check(valid_keys(g) == vector<string>{candidate_text(current)}, "future appointment survives window");
    check_live_index(g, "replacement");
    g.scheduleMovementCandidate(0, INF);
    check(!g.movementInDispatchPQ[0] && !g.isDispatchCandidateValid(current), "INF cancels old appointment");
    check_live_index(g, "INF cancellation");
    g.refresh_dispatch_candidate(0, 0);
    check(g.movementInDispatchPQ[0], "refresh can reinstate valid front");
    g.waitingBuffers[0].vehicleQueue.clear();
    g.refresh_dispatch_candidate(0, 0);
    check(!g.movementInDispatchPQ[0] && valid_keys(g).empty(), "empty front cancels rather than strands old token");
    check_live_index(g, "empty cancellation");
    g.invalidate_dispatch_candidate(0);
    check(!g.movementInDispatchPQ[0], "repeated invalidate leaves inactive");
}

void defensive_contract_checks() {
    Graph exhausted; base_graph(exhausted, 1); add_vehicle(exhausted, 0, 1);
    exhausted.movementPQVersion[0] = numeric_limits<uint64_t>::max();
    bool rejected = false;
    try { exhausted.refresh_dispatch_candidate(0, 0); }
    catch (const runtime_error&) { rejected = true; }
    check(rejected, "generation overflow fails closed instead of reviving old versions");
    Graph changed; base_graph(changed, 1); add_vehicle(changed, 0, 1);
    changed.refresh_dispatch_candidate(0, 0);
    Movement extra; extra.movementID = 1; changed.movements.push_back(extra);
    rejected = false;
    try { changed.refresh_dispatch_candidate(0, 0); }
    catch (const runtime_error&) { rejected = true; }
    check(rejected, "live topology size change cannot trigger hidden global repair");
    Graph missing; base_graph(missing, 1); add_vehicle(missing, 0, 1);
    missing.refresh_dispatch_candidate(0, 0);
    missing.waitingBuffers[0].vehicleQueue.clear(); // Deliberately omit mutation notification.
    rejected = false;
    try { missing.prepare_dispatch_window(0, 2); }
    catch (const runtime_error&) { rejected = true; }
    check(rejected, "missing current-head notification is diagnosed rather than silently lost");
    Graph retry; base_graph(retry, 1); add_vehicle(retry, 0, 1);
    retry.defer_dispatch_retry(0); retry.defer_dispatch_retry(0);
    check(retry.deferredDispatchRetries.size() == 1, "deferred retries deduplicate per movement");
    retry.prepare_dispatch_window(3, 4);
    near(retry.latestDispatchCandidates[0].timeLabel, 3, "deferred retry uses next window lower bound");
    check(retry.dispatchQueueStats.deferredRetries == 1 && retry.deferredDispatchRetries.empty(),
          "next-window retry is consumed once");
    check_live_index(retry, "deferred retry");
}

void front_update_checks(MovementQueueOrder order) {
    Graph g; base_graph(g, 1); g.movementQueueOrder = order;
    const int first = add_vehicle(g, 0, 10);
    check(g.scheduleInsertedVehicleIfMovementFront(0, first, 0), "initial head scheduled");
    const auto version = g.movementPQVersion[0];
    const int follower = add_vehicle(g, 0, 20);
    check(!g.scheduleInsertedVehicleIfMovementFront(0, follower, 0), "follower does not replace head candidate");
    check(g.movementPQVersion[0] == version && g.dispatchPQ.size() == 1,
          "follower does not inflate generation or physical heap");
    const int earlier = add_vehicle(g, 0, 2);
    const bool replaced = g.scheduleInsertedVehicleIfMovementFront(0, earlier, 0);
    if (order == MovementQueueOrder::ArrivalTime) {
        check(replaced && g.latestDispatchCandidates[0].frontVehicleID == earlier, "arrival-order earlier head replaces token");
        near(g.latestDispatchCandidates[0].timeLabel, 2, "earlier head lowers appointment");
        check(g.movementPQVersion[0] > version, "head replacement advances version");
    } else {
        check(!replaced && g.latestDispatchCandidates[0].frontVehicleID == first, "strict FIFO ignores follower earlier time");
        check(g.movementPQVersion[0] == version, "FIFO follower keeps generation");
    }
    check_live_index(g, "head update");
}

void epsilon_prefix_checks() {
    Graph g; base_graph(g);
    add_vehicle(g, 0, 0.8); add_vehicle(g, 1, 0.2); add_vehicle(g, 2, 10);
    g.scheduleMovementCandidate(0, 1 - TIME_EPSILON * .8);
    g.scheduleMovementCandidate(1, 1 - TIME_EPSILON * .2);
    g.refresh_dispatch_candidate(2, 0);
    const auto far = g.latestDispatchCandidates[2];
    check(g.dispatchPQ.top().movementID == 0, "old near-boundary keys put movement0 first");
    g.process_discharge_window(0, 1);
    check(g.vehicles[0].roadIndex == 0 && g.vehicles[1].roadIndex == 0,
          "EPS exclusion retains both appointments for next window");
    g.prepare_dispatch_window(1, 2);
    near(g.latestDispatchCandidates[0].timeLabel, 1, "first near-boundary key normalized");
    near(g.latestDispatchCandidates[1].timeLabel, 1, "all near-boundary keys normalized before release");
    check(g.dispatchPQ.top().movementID == 1, "normalized tie uses earlier front arrival, not old heap order");
    check(candidate_text(g.latestDispatchCandidates[2]) == candidate_text(far), "unaffected future token retained exactly");
    check_live_index(g, "EPS prefix");
    for (double delta : {-TIME_EPSILON, -TIME_EPSILON * .5, 0.0, TIME_EPSILON * .5}) {
        Graph b; base_graph(b, 1);
        add_vehicle(b, 0, 0);
        b.scheduleMovementCandidate(0, 1 + delta);
        b.process_discharge_window(0, 1);
        check(b.vehicles[0].roadIndex == 0, "boundary appointment is never discharged in prior window");
        b.prepare_dispatch_window(1, 2);
        check(b.latestDispatchCandidates[0].timeLabel >= 1, "new-window appointment cannot remain in past");
    }
}

void signal_local_checks() {
    Graph g; base_graph(g);
    add_vehicle(g, 0, 5); add_vehicle(g, 1, 20); add_vehicle(g, 2, 5);
    SignalProgram p;
    p.tlID = "tls"; p.cycleLength = 20;
    p.phases = {{0, 4, "G"}, {4, 10, "r"}, {10, 20, "G"}};
    g.tlIDToSignalProgramID[p.tlID] = 0; g.signalPrograms.push_back(p);
    for (int m : {0, 2}) {
        g.movements[m].alwaysOpen = false; g.movements[m].tlID = "tls"; g.movements[m].linkIndex = 0;
        SignalController s; s.signalID = static_cast<int>(g.signals.size()); s.movementID = m;
        g.movements[m].signalID = s.signalID; g.signals.push_back(s);
    }
    for (int m = 0; m < 3; ++m) g.refresh_dispatch_candidate(m, 0);
    const auto reservation = g.latestDispatchCandidates[0];
    const auto unrelated = g.latestDispatchCandidates[1];
    near(reservation.timeLabel, 10, "static calendar already reserves future green");
    g.deactivateMovementForDownstreamBlock(2);
    g.handle_signal_change_event({4, 0});
    g.handle_signal_change_event({4, 1});
    check(candidate_text(g.latestDispatchCandidates[0]) == candidate_text(reservation),
          "red transition preserves identical future green appointment");
    check(candidate_text(g.latestDispatchCandidates[1]) == candidate_text(unrelated), "signal event does not update unrelated movement");
    check(g.movementBlockedByDownstream[2] && !g.movementInDispatchPQ[2], "red event does not unlock storage blockage");
    g.handle_signal_change_event({10, 0});
    g.handle_signal_change_event({10, 1});
    check(candidate_text(g.latestDispatchCandidates[0]) == candidate_text(reservation), "green transition is idempotent for reserved current time");
    check(g.movementBlockedByDownstream[2] && !g.movementInDispatchPQ[2], "green event does not unlock storage blockage");
    check(candidate_text(g.latestDispatchCandidates[1]) == candidate_text(unrelated), "green event stays local");
    // Explicitly refresh a now-impossible signal program; an old finite key must not survive INF.
    g.signalPrograms[0].phases = {{0, 20, "r"}};
    g.refresh_dispatch_candidate(0, 10);
    check(!g.movementInDispatchPQ[0] && !g.isDispatchCandidateValid(reservation), "permanent red invalidates prior finite key");
    check_live_index(g, "signal updates");

    Graph edge; base_graph(edge, 1); add_vehicle(edge, 0, 0);
    const double cursor = 4;
    const double eventTime = cursor + TIME_EPSILON * .5;
    SignalProgram nearBoundary;
    nearBoundary.tlID = "near"; nearBoundary.cycleLength = 20;
    nearBoundary.phases = {{0, eventTime, "G"}, {eventTime, 10, "r"}, {10, 20, "G"}};
    edge.signalPrograms.push_back(nearBoundary); edge.tlIDToSignalProgramID["near"] = 0;
    edge.movements[0].alwaysOpen = false; edge.movements[0].tlID = "near"; edge.movements[0].linkIndex = 0;
    SignalController controller; controller.signalID = 0; controller.movementID = 0;
    controller.currentState = SignalState::Green; edge.signals.push_back(controller);
    edge.scheduleMovementCandidate(0, cursor);
    const auto nearKey = edge.latestDispatchCandidates[0];
    edge.handle_signal_change_event({eventTime, 0}, cursor);
    check(edge.signals[0].currentState == SignalState::Red, "near-boundary event updates its own event-time state");
    near(edge.latestDispatchCandidates[0].timeLabel, cursor,
         "event consumed within EPS uses dispatch cursor, not slightly later red time");
    check(candidate_text(edge.latestDispatchCandidates[0]) == candidate_text(nearKey),
          "near-boundary signal refresh preserves identical current-cursor key");
}

void compaction_checks() {
    Graph g; base_graph(g);
    for (int m = 0; m < 3; ++m) add_vehicle(g, m, .5 - .1 * m);
    // Up to three live keys, many obsolete future appointments; automatic cleanup
    // is allowed, so assert semantics rather than an arbitrary physical threshold.
    for (int i = 0; i < 12000; ++i) {
        g.scheduleMovementCandidate(i % 3, 50000 - i);
        // Production performs maintenance at stable event boundaries, not in
        // the middle of a successful discharge's multi-movement state update.
        if (i % 64 == 63) g.compact_dispatch_queue(false);
    }
    for (int m = 0; m < 3; ++m) g.scheduleMovementCandidate(m, 50);
    const auto before = valid_keys(g);
    const auto versions = g.movementPQVersion;
    check(before.size() == 3, "three logical keys despite repeated replacement");
    check(g.dispatchQueueStats.compactions > 0, "replacement stress triggers automatic heap compaction");
    check(g.latestDispatchCandidates[2].firstCarArriveTime < g.latestDispatchCandidates[1].firstCarArriveTime,
          "compaction fixture contains secondary-key ordering");
    g.compact_dispatch_queue(true);
    check(g.dispatchPQ.size() == 3, "forced compaction retains exactly live keys");
    check(valid_keys(g) == before, "compaction preserves complete key and pop order");
    check(g.movementPQVersion == versions, "compaction does not invent generations");
    check_live_index(g, "compacted");
    g.invalidate_dispatch_candidate(1);
    g.compact_dispatch_queue(true);
    check(g.dispatchPQ.size() == 2, "compaction does not revive explicitly cancelled key");
    check_live_index(g, "cancelled then compacted");
    g.reset_dispatch_state();
    check(g.dispatchPQ.empty() && g.activeDispatchMovements.empty(), "new run clears physical and active queues");
    for (size_t m = 0; m < g.movements.size(); ++m)
        check(!g.movementInDispatchPQ[m] && g.dispatchActivePosition[m] == -1, "new run clears ownership markers");
}

void same_time_competition_checks() {
    Graph g; base_graph(g, 2);
    g.evalTrajectoryDetails = true;
    add_vehicle(g, 0, .2); add_vehicle(g, 1, .1);
    g.roads[2].laneCapacity[0] = 1;
    g.ETA_result_cycle_aware = {{{0, 0}}, {{1, 0}}};
    g.vehicleMovementTimeline.resize(2); g.vehicleRoadTimeline.resize(2);
    g.movementLaneNextAvailableTime[0][0] = 1;
    g.movementLaneNextAvailableTime[1][0] = 1;
    g.refresh_dispatch_candidate(0, 0); g.refresh_dispatch_candidate(1, 0);
    g.process_discharge_window(1, 1.5);
    check(g.vehicles[1].roadIndex == 1 && g.vehicles[0].roadIndex == 0,
          "same-time competition chooses earlier head, not lower movement ID");
    check(g.roads[2].roadFlow == 1 && g.movementBlockedByDownstream[0],
          "losing same-time movement sees storage consumed by winner");
    check(!g.movementInDispatchPQ[0] && !g.movementInDispatchPQ[1], "winner empty and loser blocked have no live key");
    const double exitTime = g.vehicles[1].arrivalTime;
    check(!g.processFinalRoadExitsUntil(exitTime - .1), "final road keeps occupancy before exact exit");
    check(g.processFinalRoadExitsUntil(exitTime), "actual final exit releases winning vehicle storage");
    check(!g.movementBlockedByDownstream[0] && g.movementInDispatchPQ[0], "loser wakes only at actual release");
    g.process_discharge_window(exitTime, exitTime + .1);
    check(g.vehicles[0].roadIndex == 1, "reactivated loser executes its own candidate");
    check_live_index(g, "same-time competition");
    Graph tie; base_graph(tie, 2);
    add_vehicle(tie, 0, .1); add_vehicle(tie, 1, .1);
    tie.roads[2].laneCapacity[0] = 1;
    tie.ETA_result_cycle_aware = {{{0, 0}}, {{1, 0}}};
    tie.movementLaneNextAvailableTime[0][0] = 1;
    tie.movementLaneNextAvailableTime[1][0] = 1;
    tie.refresh_dispatch_candidate(1, 0); tie.refresh_dispatch_candidate(0, 0);
    tie.process_discharge_window(1, 1.5);
    check(tie.vehicles[0].roadIndex == 1 && tie.vehicles[1].roadIndex == 0,
          "identical time and head-arrival tie uses movement ID, not insertion order");
}

void multiple_capacity_lane_checks() {
    vector<double> reference;
    for (bool persistent : {false, true}) {
        Graph g; base_graph(g, 1); g.persistentDispatch = persistent; g.evalTrajectoryDetails = true;
        for (auto& road : g.roads) road.laneNum = 2;
        g.initializeRoadLaneStorage();
        g.movements[0].fromLanes = {"0", "1"}; g.movements[0].toLanes = {"0", "1"};
        g.movements[0].laneDischargeCapacity = 2;
        g.initializeMovementDischargeTimes(0);
        g.movementLaneNextAvailableTime[0] = {1, 1};
        for (int i = 0; i < 6; ++i) add_vehicle(g, 0, 0);
        g.ETA_result_cycle_aware.assign(6, vector<pair<int, double>>{{0, 0}});
        g.vehicleMovementTimeline.resize(6); g.vehicleRoadTimeline.resize(6);
        g.refresh_dispatch_candidate(0, 0);
        g.process_discharge_window(1, 1.1);
        check(g.vehicles[0].roadIndex == 1 && g.vehicles[1].roadIndex == 1 && g.vehicles[2].roadIndex == 0,
              "two capacity lanes permit exactly two initial same-time departures");
        near(g.movementLaneNextAvailableTime[0][0], 3.15, "first channel floating headway carried");
        near(g.movementLaneNextAvailableTime[0][1], 3.15, "second channel floating headway carried");
        g.process_discharge_window(1.1, 3.15);
        check(g.vehicles[2].roadIndex == 0, "next pair not discharged at previous window right boundary");
        g.process_discharge_window(3.15, 3.2);
        check(g.vehicles[2].roadIndex == 1 && g.vehicles[3].roadIndex == 1 && g.vehicles[4].roadIndex == 0,
              "both capacity channels reopen at exact fractional headway");
        vector<double> times;
        for (int i = 0; i < 4; ++i) times.push_back(g.vehicleMovementTimeline[i].front().dischargeTime);
        check(times == vector<double>({1, 1, 3.15, 3.15}), "multi-channel timestamps exact");
        if (!persistent) reference = times;
        else {
            check(times == reference, "multi-channel persistent and rebuild event times equal");
            check_live_index(g, "multiple capacity lanes");
        }
    }
}

struct FixtureFile {
    string path;
    explicit FixtureFile(const string& xml) {
        static int serial = 0;
        path = ".persistent_dispatch_" + to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count())
            + "_" + to_string(++serial) + ".xml";
        ofstream out(path);
        if (!out) throw runtime_error("cannot create small network fixture");
        out << xml;
    }
    ~FixtureFile() { std::remove(path.c_str()); }
};

string small_network() {
    return "<net>"
        "<edge id=\"a\" from=\"A\" to=\"J\"><lane id=\"a_0\" index=\"0\" length=\"18\" speed=\"10\"/></edge>"
        "<edge id=\"b\" from=\"B\" to=\"J\"><lane id=\"b_0\" index=\"0\" length=\"18\" speed=\"10\"/></edge>"
        "<edge id=\"c\" from=\"J\" to=\"K\"><lane id=\"c_0\" index=\"0\" length=\"8\" speed=\"10\"/></edge>"
        "<edge id=\"d\" from=\"K\" to=\"Z\"><lane id=\"d_0\" index=\"0\" length=\"8\" speed=\"10\"/></edge>"
        "<junction id=\"A\" type=\"dead_end\"/><junction id=\"B\" type=\"dead_end\"/>"
        "<junction id=\"J\" type=\"traffic_light\"/><junction id=\"K\" type=\"priority\"/>"
        "<junction id=\"Z\" type=\"dead_end\"/>"
        "<tlLogic id=\"J\" type=\"static\" programID=\"0\" offset=\"0.25\">"
        "<phase duration=\"4\" state=\"GG\"/><phase duration=\"3\" state=\"rr\"/></tlLogic>"
        "<connection from=\"a\" to=\"c\" fromLane=\"0\" toLane=\"0\" tl=\"J\" linkIndex=\"0\" dir=\"s\"/>"
        "<connection from=\"b\" to=\"c\" fromLane=\"0\" toLane=\"0\" tl=\"J\" linkIndex=\"1\" dir=\"l\"/>"
        "<connection from=\"c\" to=\"d\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>"
        "</net>";
}

string scientific_state(Graph& g) {
    ostringstream out;
    out << setprecision(17) << g.finishedVehicleCount << ',' << g.invalidVehicleCount << '\n';
    for (const auto& vehicle : g.vehicles)
        out << vehicle.vehicleID << ',' << vehicle.arrivalTime << ',' << vehicle.roadIndex << ','
            << vehicle.finished << ',' << vehicle.valid << ',' << vehicle.occupiedRoadID << ',' << vehicle.occupiedLaneIndex << '\n';
    for (const auto& row : g.ETA_result_cycle_aware) {
        for (const auto& item : row) out << item.first << ':' << item.second << ',';
        out << '\n';
    }
    for (const auto& row : g.vehicleRoadTimeline)
        for (const auto& r : row)
            out << "R," << r.vehicleID << ',' << r.routeRoadIndex << ',' << r.roadID << ',' << r.enteringMovementID << ','
                << r.nextMovementID << ',' << r.laneIndex << ',' << r.intersectionCrossingTime << ',' << r.enterTime << ','
                << r.arrivalAtRoadEndTime << ',' << r.roadTravelTime << '\n';
    for (const auto& row : g.vehicleMovementTimeline)
        for (const auto& r : row)
            out << "M," << r.vehicleID << ',' << r.routeMovementIndex << ',' << r.movementID << ',' << r.fromRoadID << ','
                << r.toRoadID << ',' << r.chosenLane << ',' << r.arrivalAtWaitingBufferTime << ','
                << r.dischargeTime << ',' << r.movementWaitingTime << '\n';
    // Cache label, version and in-PQ diagnostics intentionally are not trajectory output.
    for (const auto& r : g.downstreamReleaseCompetitionTimeline)
        out << "D," << r.releaseSequence << ',' << r.releaseTime << ',' << r.releasedByVehicleID << ','
            << r.releasedByMovementID << ',' << r.freedRoadID << ',' << r.freedLaneIndex << ',' << r.movementID << ','
            << r.frontVehicleID << ',' << r.queueSize << ',' << r.frontArrivalSecondLabel << ','
            << r.signalActiveAtRelease << ',' << r.nextGreenTime << ',' << r.nextCapacityTime << ','
            << r.candidateFirstLabel << ',' << r.downstreamStorageAvailable << ',' << r.candidateLane << ','
            << r.wasBlockedByDownstream << ',' << r.reactivated << '\n';
    for (const auto& r : g.roads) {
        out << "F," << r.roadFlow;
        for (int flow : r.laneFlow) out << ',' << flow;
        out << '\n';
    }
    return out.str();
}

void loop_equivalence_checks(MovementQueueOrder order, bool continuous) {
    FixtureFile file(small_network());
    string reference;
    for (bool persistent : {false, true}) {
        Graph g;
        g.persistentDispatch = persistent; g.movementQueueOrder = order;
        g.travelTimeMode = TravelTimeMode::SPEED_NET; g.speedNetUseAcceleration = true;
        g.speedNetUseFollowing = true; g.speedNetHeadway = .12; g.speedNetDelayMax = 3;
        g.speedNetContinuousSpeed = continuous;
        g.defaultDischargeInterval = 2.15; g.intersectionCrossingTime = 3.4;
        g.evalTrajectoryDetails = true;
        g.read_sumo_net_xml(file.path); g.build_new_graph_structures_from_sumo();
        vector<vector<int>> routes, q;
        for (int i = 0; i < 16; ++i) {
            const int first = i % 2;
            routes.push_back(i == 15 ? vector<int>{first} : vector<int>{first, 2, 3});
            q.push_back({g.roads[first].fromNode, g.roads[routes.back().back()].toNode, 0});
            g.vehicleDepartureTimes.push_back(.25 + .125 * i);
            g.vehicleDeparturePositions.push_back(0);
        }
        g.cycle_aware_signal_driven_records(q, routes);
        check(g.finishedVehicleCount == 16 && g.invalidVehicleCount == 0, "small graph all vehicles finish");
        check(g.finalRoadExitPQ.empty(), "small graph consumes all terminal events");
        for (const auto& road : g.roads) check(road.roadFlow == 0, "small graph drains road occupancy");
        const string firstRun = scientific_state(g);
        if (!persistent) reference = firstRun;
        else check(firstRun == reference, "persistent/rebuild complete scientific trajectories exactly equal");
        g.cycle_aware_signal_driven_records(q, routes);
        check(scientific_state(g) == firstRun, "same Graph second run resets all candidate state");
        if (persistent) {
            check_live_index(g, "finished small graph");
            check(g.dispatchQueueStats.fullRebuilds == 0 && g.dispatchQueueStats.globalMovementVisits == 0,
                  "persistent small-graph loop does not hide a global movement rebuild");
            check(g.dispatchQueueStats.deferredRetries == 0, "valid small-graph run needs no unexpected failure recovery");
        }
    }
}

void self_loop_run_checks() {
    FixtureFile file(
        "<net><edge id=\"a\" from=\"J\" to=\"J\"><lane id=\"a_0\" index=\"0\" length=\"100\" speed=\"10\"/></edge>"
        "<edge id=\"b\" from=\"J\" to=\"Z\"><lane id=\"b_0\" index=\"0\" length=\"20\" speed=\"10\"/></edge>"
        "<junction id=\"J\" type=\"priority\"/><junction id=\"Z\" type=\"dead_end\"/>"
        "<connection from=\"a\" to=\"a\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>"
        "<connection from=\"a\" to=\"b\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/></net>");
    string reference;
    for (bool persistent : {false, true}) {
        Graph g; g.persistentDispatch = persistent; g.evalTrajectoryDetails = true;
        g.travelTimeMode = TravelTimeMode::SPEED_NET; g.speedNetUseAcceleration = true;
        g.speedNetUseFollowing = false; g.speedNetContinuousSpeed = true;
        g.defaultDischargeInterval = 2.15;
        g.read_sumo_net_xml(file.path); g.build_new_graph_structures_from_sumo();
        vector<vector<int>> routes(3, {0, 0, 0, 1});
        vector<vector<int>> q(3, {g.roads[0].fromNode, g.roads[1].toNode, 0});
        g.vehicleDepartureTimes = {.25, .5, .75}; g.vehicleDeparturePositions.assign(3, 0);
        g.cycle_aware_signal_driven_records(q, routes);
        check(g.finishedVehicleCount == 3 && g.invalidVehicleCount == 0, "actual self-loop routes complete");
        for (int i = 0; i < 3; ++i) {
            check(g.vehicleRoadTimeline[i].size() == 4 && g.vehicleMovementTimeline[i].size() == 3,
                  "self-loop preserves every repeated-road occurrence and actual discharge");
            check(g.vehicleMovementTimeline[i][0].fromRoadID == 0 && g.vehicleMovementTimeline[i][0].toRoadID == 0,
                  "self-loop transition actually executes, not merely an indexed topology fixture");
        }
        const string result = scientific_state(g);
        if (!persistent) reference = result;
        else {
            check(result == reference, "actual self-loop run matches rebuild scientific trajectory exactly");
            check_live_index(g, "finished self-loop run");
            check(g.dispatchQueueStats.deferredRetries == 0, "valid self-loop needs no unexpected retry path");
        }
    }
}

} // namespace

int main() {
    try {
        token_lifecycle_checks();
        defensive_contract_checks();
        front_update_checks(MovementQueueOrder::ArrivalTime);
        front_update_checks(MovementQueueOrder::StrictFIFO);
        epsilon_prefix_checks();
        signal_local_checks();
        compaction_checks();
        same_time_competition_checks();
        multiple_capacity_lane_checks();
        for (auto order : {MovementQueueOrder::ArrivalTime, MovementQueueOrder::StrictFIFO})
            for (bool continuous : {false, true}) loop_equivalence_checks(order, continuous);
        self_loop_run_checks();
        cout << "PASS persistent_dispatch checks=" << checks << '\n';
        return 0;
    } catch (const exception& error) {
        cerr << "FAIL persistent_dispatch checks=" << checks << ": " << error.what() << '\n';
        return 1;
    }
}
