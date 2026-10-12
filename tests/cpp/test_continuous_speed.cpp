#include "head.h"
#include <chrono>
#include <cstdio>

namespace {
int checks = 0;
void check(bool ok, const string& why) { ++checks; if (!ok) throw runtime_error(why); }
void near(double actual, double expected, const string& why) {
    check(std::isfinite(actual) && abs(actual - expected) < 1e-9, why);
}
void rejects(const function<void()>& action, const string& why) {
    bool rejected = false;
    try { action(); } catch (const exception&) { rejected = true; }
    check(rejected, why);
}
struct Fixture {
    string path;
    explicit Fixture(const string& xml) {
        static int serial = 0;
        path = ".continuous_speed_" + to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count())
            + "_" + to_string(++serial) + ".xml";
        ofstream out(path); if (!out) throw runtime_error("fixture failure"); out << xml;
    }
    ~Fixture() { std::remove(path.c_str()); }
};
void configure(Graph& g) {
    g.travelTimeMode = TravelTimeMode::SPEED_NET;
    g.speedNetContinuousSpeed = true;
    g.speedNetUseAcceleration = true;
    g.speedNetUseFollowing = false;
    VehicleType type; type.accel = 2; type.decel = 4; type.maxSpeed = 30;
    g.sumoVehicleTypes["car"] = type;
}
void pure_motion_checks() {
    Graph g; configure(g); g.roads.resize(1);
    BasicRoadModelFeatures f; f.roadID = 0; f.speed_limit = 10; f.road_length = 10;
    f.entry_speed_proxy = 4;
    auto p = g.evaluateSpeedNetMotion(f);
    near(p.motion_time, (sqrt(56.0) - 4) / 2, "accelerate from nonzero v0");
    near(p.exit_speed, sqrt(56.0), "short road preserves sub-cap end speed");
    f.road_length = 100;
    p = g.evaluateSpeedNetMotion(f);
    near(p.motion_time, 3 + 79.0 / 10, "long road reaches cap and cruises");
    near(p.exit_speed, 10, "cap bound");
    f.entry_speed_proxy = 20;
    p = g.evaluateSpeedNetMotion(f);
    near(p.entry_speed, 10, "downstream cap clamps high upstream speed");
    near(p.motion_time, 10, "clamped cap does not produce negative acceleration time");
    f.entry_speed_proxy = 4; f.speed_limit = 25; f.road_length = 1;
    p = g.evaluateSpeedNetMotion(f);
    near(p.exit_speed, sqrt(20.0), "higher downstream limit cannot instantly jump speed");
    near(p.predicted_time, 1, "existing minimum second retained");
    const double exit = p.exit_speed;
    g.speedNetUseFollowing = true; g.speedNetHeadway = 1; g.speedNetDelayMax = 3; f.lane_flow = 10;
    p = g.evaluateSpeedNetMotion(f);
    near(p.exit_speed, exit, "following delay not treated as acceleration time");
    near(p.predicted_time, p.motion_time + 3, "following delay retained separately");
    f.road_length = 0; p = g.evaluateSpeedNetMotion(f);
    near(p.exit_speed, 4, "zero effective distance does not create speed from floor");
    g.speedNetQueueTrim = true; f.road_length = 8; f.lane_waiting_buffer_length = 20;
    near(g.evaluateSpeedNetMotion(f).effective_distance, 0, "existing queue trim composes once");
    g.speedNetQueueTrim = false;
    f.is_route_start = true; f.departure_position = 5.1; f.road_length = 100;
    near(g.evaluateSpeedNetMotion(f).effective_distance, 94.9, "first-road geometry remains independent");
    f.is_route_start = false; f.entry_speed_proxy = 15;
    for (double length : {0.0, 0.01, 4.0, 25.0, 100.0}) {
        f.road_length = length; f.speed_limit = 10;
        g.speedNetContinuousSpeed = false;
        const double old = g.predictRoadTravelTimeSpeedNet(f);
        g.speedNetContinuousSpeed = true; f.entry_speed_proxy = 0;
        check(g.predictRoadTravelTimeSpeedNet(f) == old, "v0=0 exactly preserves old arithmetic");
        f.entry_speed_proxy = 15;
        for (auto mode : {TravelTimeMode::TABLE, TravelTimeMode::MODEL, TravelTimeMode::MIN_TIME, TravelTimeMode::KINEMATIC}) {
            g.travelTimeMode = mode;
            check(g.predictRoadTravelTimeSpeedNet(f) == old, "non-speed-net fallback cannot inherit proxy");
            rejects([&] { g.requireContinuousSpeedConfiguration(); }, "explicit on rejected in other modes");
        }
        g.travelTimeMode = TravelTimeMode::SPEED_NET;
    }
    g.speedNetUseAcceleration = false;
    rejects([&] { g.requireContinuousSpeedConfiguration(); }, "on requires acceleration");
    g.speedNetUseAcceleration = true;
    for (double bad : vector<double>{-1.0, NAN, INFINITY}) {
        g.speedNetContinuousWaitThreshold = bad;
        rejects([&] { g.requireContinuousSpeedConfiguration(); }, "invalid threshold");
    }
    g.speedNetContinuousWaitThreshold = 0;
    f.entry_speed_proxy = NAN;
    rejects([&] { g.evaluateSpeedNetMotion(f); }, "invalid proxy fails without mutating state");
}
string net(const string& via = "", const string& internals = "", const string& continuation = "") {
    return string("<net>") + internals +
        "<edge id=\"a\" from=\"A\" to=\"T\"><lane id=\"a_0\" index=\"0\" length=\"100\" speed=\"10\"/></edge>"
        "<edge id=\"b\" from=\"T\" to=\"U\"><lane id=\"b_0\" index=\"0\" length=\"4\" speed=\"20\"/></edge>"
        "<edge id=\"c\" from=\"U\" to=\"Z\"><lane id=\"c_0\" index=\"0\" length=\"100\" speed=\"20\"/></edge>"
        "<junction id=\"A\" type=\"dead_end\"/><junction id=\"T\" type=\"traffic_light\"/>"
        "<junction id=\"U\" type=\"priority\"/><junction id=\"Z\" type=\"dead_end\"/>"
        "<tlLogic id=\"T\" type=\"static\" programID=\"0\" offset=\"0\"><phase duration=\"100\" state=\"G\"/>"
        "<phase duration=\"5\" state=\"r\"/></tlLogic>"
        "<connection from=\"a\" to=\"b\" fromLane=\"0\" toLane=\"0\" tl=\"T\" linkIndex=\"0\" dir=\"r\" "
        + via + "/><connection from=\"b\" to=\"c\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>" + continuation + "</net>";
}
string lane(const string& id, double speed) {
    return "<edge id=\"" + id + "\" function=\"internal\"><lane id=\"" + id +
        "_0\" index=\"0\" speed=\"" + to_string(speed) + "\" length=\"2\"/></edge>";
}
string link(const string& from, const string& via = "") {
    return "<connection from=\"" + from + "\" to=\"b\" fromLane=\"0\" toLane=\"0\" " + via + "/>";
}
Graph read_net(const string& xml) {
    Fixture file(xml); Graph g; configure(g); g.read_sumo_net_xml(file.path);
    g.build_new_graph_structures_from_sumo(); g.intersectionCrossingTime = 3.4;
    return g;
}
VehicleLabel previous() {
    VehicleLabel v; v.vehicleID = 0; v.currentRoadID = 0; v.roadIndex = 2;
    v.arrivalTime = 10; v.estimatedRoadExitSpeed = 10; v.estimatedExitRoadIndex = 2;
    return v;
}
void transfer_checks() {
    auto g = read_net(net()); auto v = previous();
    near(g.continuousSpeedTransfer(v, 0, 10).entry_speed, 10, "zero-wait direct carry");
    near(g.continuousSpeedTransfer(v, 0, 10.1).entry_speed, 0, "long wait resets at default threshold0");
    g.speedNetContinuousWaitThreshold = 1;
    near(g.continuousSpeedTransfer(v, 0, 10.5).entry_speed, 8, "short wait applies explicit deceleration");
    near(g.continuousSpeedTransfer(v, 0, 11).entry_speed, 6, "inclusive threshold boundary");
    check(g.continuousSpeedTransfer(v, 0, 11.1).reason == "wait_exceeds_threshold", "threshold excess reason");
    g.speedNetContinuousWaitThreshold = 3;
    check(g.continuousSpeedTransfer(v, 0, 12.5).reason == "braked_to_zero", "allowed wait can consume all speed");
    g.speedNetContinuousWaitThreshold = 0;
    near(g.continuousSpeedTransfer(v, 0, 10 + TIME_EPSILON / 2).entry_speed,
         10 - 4 * TIME_EPSILON / 2, "strict-zero threshold permits only numeric epsilon");
    near(v.estimatedRoadExitSpeed, 10, "transfer helper has no side effects");
    rejects([&] { g.continuousSpeedTransfer(v, 0, 9); }, "early release is not zero wait");
    --v.estimatedExitRoadIndex;
    check(g.continuousSpeedTransfer(v, 0, 10).reason == "unknown_previous_speed", "stale occurrence cannot carry");
    v = previous();
    auto chained = read_net(net("via=\":x_0\"", lane(":x", 8) + lane(":y", 5), link(":x", "via=\":y_0\"") + link(":y")));
    check(chained.movements[0].continuousSpeedPathKnown, "complete multi-via chain resolved");
    near(chained.movements[0].continuousSpeedPathLimit, 5, "whole path min not just first via");
    near(chained.continuousSpeedTransfer(v, 0, 10).entry_speed, 5, "internal turn bound applied");
    near(chained.movementCrossingTime(0), 3.4, "fixed crossing unchanged");
    chained.speedNetUseSpeedFactor = true; chained.vehicleSpeedFactors = {.5}; chained.vehicleHasExplicitSpeedFactor = {true};
    near(chained.continuousSpeedTransfer(v, 0, 10).entry_speed, 2.5, "internal bound applies same vehicle SF rule");
    for (const string& follow : {string(""), link(":x", "via=\":x_0\""), link(":x") + link(":x")}) {
        auto broken = read_net(net("via=\":x_0\"", lane(":x", 8), follow));
        check(!broken.movements[0].continuousSpeedPathKnown, "missing/cyclic/ambiguous continuation unknown");
        check(broken.continuousSpeedTransfer(v, 0, 10).reason == "unknown_internal_path", "unknown path resets rather than assuming cap");
    }
    const string alternatives = "<connection from=\"a\" to=\"b\" fromLane=\"0\" toLane=\"0\" tl=\"T\" linkIndex=\"0\" via=\":y_0\"/>";
    auto multiple = read_net(net("via=\":x_0\"", lane(":x", 8) + lane(":y", 6), link(":x") + link(":y") + alternatives));
    near(multiple.movements[0].continuousSpeedPathLimit, 6, "aggregated complete lane paths use conservative min");
}
void lifecycle_checks() {
    auto g = read_net(net()); g.evalTrajectoryDetails = true; g.defaultDischargeInterval = 2.15;
    vector<vector<int>> routes(4, {0, 1, 2});
    vector<vector<int>> queries(4, {g.roads[0].fromNode, g.roads[2].toNode, 0});
    g.vehicleDepartureTimes.assign(4, 0.25); g.vehicleDeparturePositions.assign(4, 0.0);
    g.cycle_aware_signal_driven_records(queries, routes);
    check(g.finishedVehicleCount == 4 && g.invalidVehicleCount == 0, "concurrent routes complete");
    for (int i = 0; i < 4; ++i) {
        check(g.vehicles[i].estimatedExitRoadIndex == 2, "final road gets its own speed proxy");
        check(g.vehicleRoadTimeline[i].size() == 3, "no skipped road occurrence");
        check(g.vehicleMovementTimeline[i].size() == 2, "one successful discharge per transfer");
        if (i) check(g.vehicleMovementTimeline[i][0].dischargeTime - g.vehicleMovementTimeline[i - 1][0].dischargeTime
                     >= 2.15 - TIME_EPSILON, "service headway unchanged");
    }
    for (const auto& road : g.roads) check(road.roadFlow == 0, "all storage released");
    for (const auto& buffer : g.waitingBuffers) check(buffer.vehicleQueue.empty(), "all buffers drained");
    check(g.finalRoadExitPQ.empty(), "terminal events consumed");
    vector<double> firstFinish;
    for (const auto& vehicle : g.vehicles) firstFinish.push_back(vehicle.arrivalTime);
    // Same Graph, same inputs: initialize must discard prior occurrence speed and queues.
    g.cycle_aware_signal_driven_records(queries, routes);
    check(g.finishedVehicleCount == 4 && g.invalidVehicleCount == 0, "same Graph rerun completes");
    for (int i = 0; i < 4; ++i) {
        near(g.vehicles[i].arrivalTime, firstFinish[i], "same Graph rerun is deterministic");
        check(g.vehicles[i].estimatedExitRoadIndex == 2, "rerun final occurrence tag reset and recommitted");
        check(g.vehicleRoadTimeline[i].size() == 3 && g.vehicleMovementTimeline[i].size() == 2,
              "rerun timelines do not accumulate");
    }
    for (const auto& road : g.roads) check(road.roadFlow == 0, "rerun storage drained");
    for (const auto& buffer : g.waitingBuffers) check(buffer.vehicleQueue.empty(), "rerun buffers drained");
    auto once = read_net(net()); once.initializeRoadLaneStorage(); once.initializeMovementDischargeTimes(0);
    once.vehicles.resize(2);
    auto& first = once.vehicles[0]; first = previous(); first.roadIndex = 0; first.estimatedExitRoadIndex = 0;
    first.routeRoadIDs = {0,1,2}; first.routeMovementIDs = {0,1}; first.currentMovementID = 0;
    first.currentBufferID = once.movements[0].movementID; first.state = VehicleState::WaitingAtIntersection;
    once.reserveLaneOccupancy(0, 0, 0); once.waitingBuffers[0].vehicleQueue.push_back(0);
    once.vehicles[1].vehicleID = 1; once.reserveLaneOccupancy(1, 1, 0);
    const double sourceSpeed = first.estimatedRoadExitSpeed;
    const auto blocked = once.dischargeOneVehicle(0, 10);
    check(blocked.vehicleID == -1 && first.roadIndex == 0, "full downstream release fails cleanly");
    near(first.estimatedRoadExitSpeed, sourceSpeed, "failed release cannot consume speed state");
    near(first.entrySpeedProxy, 0, "failed release cannot install next entry speed");
    once.releaseLaneOccupancy(1);
    const auto success = once.dischargeOneVehicle(0, 10);
    check(success.vehicleID == 0 && first.roadIndex == 1, "retry can succeed once");
    near(first.entrySpeedProxy, 10, "successful retry carries once");
    near(first.estimatedRoadExitSpeed, sqrt(116.0), "short faster downstream road continues accelerating");
    const auto snapshot = once.buildBasicRoadModelFeatures(1, 0, 0, 10, 0);
    const double frozen = first.estimatedRoadExitSpeed;
    once.predictRoadTravelTimeSpeedNet(snapshot); once.computeMovementAttemptTime(1, 10);
    near(first.estimatedRoadExitSpeed, frozen, "snapshot/pure prediction/candidate retry leave state untouched");
    rejects([&] { once.predictRoadTravelTime(1, 0, 0, 10, 0); }, "same occurrence cannot be committed twice");
    auto single = read_net(net());
    vector<vector<int>> oneRoute{{0}}, oneQuery{{single.roads[0].fromNode, single.roads[0].toNode, 0}};
    single.vehicleDepartureTimes = {0}; single.vehicleDeparturePositions = {0};
    single.cycle_aware_signal_driven_records(oneQuery, oneRoute);
    near(single.vehicles[0].entrySpeedProxy, 0, "single-road first entry remains zero");
    check(single.finishedVehicleCount == 1 && single.vehicles[0].estimatedExitRoadIndex == 0, "single final road predicted once");
    string cyclicXml = net();
    const string oldEnd = "from=\"U\" to=\"Z\"";
    cyclicXml.replace(cyclicXml.find(oldEnd), oldEnd.size(), "from=\"U\" to=\"A\"");
    cyclicXml.insert(cyclicXml.find("</net>"),
        "<connection from=\"c\" to=\"a\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>");
    auto cyclic = read_net(cyclicXml); cyclic.enableBasicFeatureLogging = true;
    vector<vector<int>> repeatedRoute{{0, 1, 2, 0, 1, 2}};
    vector<vector<int>> repeatedQuery{{cyclic.roads[0].fromNode, cyclic.roads[2].toNode, 0}};
    cyclic.vehicleDepartureTimes = {0}; cyclic.vehicleDeparturePositions = {0};
    cyclic.cycle_aware_signal_driven_records(repeatedQuery, repeatedRoute);
    check(cyclic.finishedVehicleCount == 1 && cyclic.invalidVehicleCount == 0, "repeated road IDs complete");
    check(cyclic.vehicles[0].estimatedExitRoadIndex == 5, "speed state belongs to occurrence not road ID");
    check(cyclic.basicFeatureSnapshots.size() == 6, "one commit for each repeated occurrence");
    near(cyclic.basicFeatureSnapshots[0].entry_speed_proxy, 0, "first occurrence starts at rest");
    check(cyclic.basicFeatureSnapshots[3].roadID == 0 && cyclic.basicFeatureSnapshots[3].entry_speed_proxy > 0,
          "second occurrence of first road carries its actual predecessor speed");
}
}
int main() {
    try { pure_motion_checks(); transfer_checks(); lifecycle_checks();
        cout << "PASS continuous_speed checks=" << checks << '\n'; return 0;
    } catch (const exception& error) { cerr << "FAIL continuous_speed checks=" << checks << ": " << error.what() << '\n'; return 1; }
}
