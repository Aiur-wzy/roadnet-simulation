#include "head.h"

#include <cstdio>
#include <functional>

namespace {

int checks = 0;

void check(bool condition, const string& message) {
    ++checks;
    if (!condition) throw runtime_error(message);
}

void near(double actual, double expected, const string& message) {
    check(std::isfinite(actual) && std::abs(actual - expected) <= 1e-9,
          message + " (actual=" + to_string(actual) + ", expected=" + to_string(expected) + ")");
}

void rejects(const function<void()>& action, const string& message) {
    bool rejected = false;
    try { action(); }
    catch (const runtime_error& error) {
        rejected = string(error.what()).find("depart") != string::npos;
    }
    check(rejected, message);
}

struct Fixture {
    string path;
    explicit Fixture(const string& content) {
        static int counter = 0;
        path = "departure_position_test_" + to_string(++counter) + ".rou.xml";
        ifstream existing(path);
        check(!existing.good(), "refusing to overwrite fixture: " + path);
        ofstream output(path);
        check(output.good(), "cannot create fixture: " + path);
        output << content;
    }
    ~Fixture() { std::remove(path.c_str()); }
};

string document(const string& vehicles, const string& extraTypes = "") {
    string typedVehicles = vehicles;
    size_t pos = 0;
    while ((pos = typedVehicles.find("<vehicle ", pos)) != string::npos) {
        const size_t end = typedVehicles.find('>', pos);
        if (typedVehicles.substr(pos, end - pos).find("type=") == string::npos) {
            typedVehicles.insert(pos + 9, "type=\"car\" ");
            pos += 20;
        } else {
            pos = end + 1;
        }
    }
    return "<routes><vType id=\"car\" length=\"5\" minGap=\"2.5\" maxSpeed=\"10\" accel=\"2\"/>" +
           extraTypes + "<route id=\"r\" edges=\"a b\"/>" + typedVehicles + "</routes>";
}

void prepare(Graph& graph) {
    graph.roads.resize(2);
    for (int index = 0; index < 2; ++index) {
        RoadSegment& road = graph.roads[index];
        road.roadID = index;
        road.fromNode = index;
        road.toNode = index + 1;
        road.length = 100.0;
        road.speedLimit = 10.0;
        road.laneNum = 1;
        road.laneFlow.assign(1, 0);
        graph.sumoEdgeStrToRoadID[index == 0 ? "a" : "b"] = index;
    }
    graph.speedNetUseAcceleration = false;
    graph.speedNetUseFollowing = false;
}

// For prediction-only tests, provide runtime route identity without requiring
// intersection topology. Lifecycle coverage below uses the actual initializer.
void bindVehicles(Graph& graph) {
    graph.vehicles.assign(graph.routeRoadID.size(), VehicleLabel());
    for (int id = 0; id < static_cast<int>(graph.vehicles.size()); ++id) {
        VehicleLabel& vehicle = graph.vehicles[id];
        vehicle.vehicleID = id;
        vehicle.routeID = id;
        vehicle.routeRoadIDs = graph.routeRoadID[id];
        vehicle.roadIndex = 0;
        vehicle.currentRoadID = vehicle.routeRoadIDs.front();
    }
    graph.initializeRoadLaneStorage();
}

void test_parse_base_types_and_reload() {
    Graph graph;
    prepare(graph);
    check(graph.speedNetDepartPosition, "departure-position correction must default on");
    Fixture input(document(
        "<vehicle id=\"base\" route=\"r\" depart=\"0\" departPos=\"base\"/>"
        "<vehicle id=\"default\" route=\"r\" depart=\"1\"/>"
        "<vehicle id=\"long\" type=\"long\" depart=\"2\"><route edges=\"b\"/></vehicle>",
        "<vType id=\"long\" length=\"8\" minGap=\"20\" maxSpeed=\"10\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    check(graph.sumoVehicleIDs == vector<string>({"base", "default", "long"}), "route ordering changed");
    check(graph.vehicleDeparturePositions.size() == 3 && graph.vehicleDeparturePositionIssues.size() == 3,
          "departure metadata is not aligned with imported vehicles");
    near(graph.vehicleDeparturePositions[0], 5.1, "explicit base does not use front position length + 0.1");
    near(graph.vehicleDeparturePositions[1], 5.1, "missing departPos is not SUMO base default");
    near(graph.vehicleDeparturePositions[2], 8.1, "base used minGap or car length instead of actual type length");
    for (const string& issue : graph.vehicleDeparturePositionIssues)
        check(issue.empty(), "supported base input incorrectly marked unsupported");

    Fixture replacement(document(
        "<vehicle id=\"skip\" depart=\"0\"><route edges=\"unknown\"/></vehicle>"
        "<vehicle id=\"keep\" route=\"r\" depart=\"1\"/>"));
    graph.read_sumo_route_xml(replacement.path, -1);
    check(graph.sumoVehicleIDs == vector<string>({"keep"}), "unknown-edge route was not skipped");
    check(graph.vehicleDeparturePositions.size() == 1 && graph.vehicleDeparturePositionIssues.size() == 1,
          "reload or invalid-route skip left stale/misaligned departure metadata");
    near(graph.vehicleDeparturePositions[0], 5.1, "reloaded metadata used old type/vehicle position");
}

void test_first_only_and_physical_geometry() {
    Graph graph;
    prepare(graph);
    Fixture input(document("<vehicle id=\"v\" route=\"r\" departPos=\"base\" speedFactor=\"0.5\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    // Imported metadata alone is not enough to identify the first occurrence.
    const auto beforeRuntime = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
    check(!beforeRuntime.is_route_start, "missing runtime route was guessed to be first road");
    near(graph.predictRoadTravelTimeSpeedNet(beforeRuntime), 10.0, "legacy/no-runtime API changed");
    bindVehicles(graph);
    const int capacity = graph.roads[0].laneCapacity[0];
    const double storage = graph.roads[0].laneStorageLength[0];
    const auto first = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
    check(first.is_route_start, "actual first-road runtime state was not recognized");
    near(first.departure_position, 5.1, "first-road feature lost resolved front position");
    near(first.road_length, 100.0, "feature changed physical road length");
    near(graph.predictRoadTravelTimeSpeedNet(first), 9.49, "first-road remaining distance incorrect");
    graph.speedNetDepartPosition = false;
    near(graph.predictRoadTravelTimeSpeedNet(first), 10.0, "off does not reproduce full-road formula");
    graph.speedNetDepartPosition = true;
    near(graph.predictRoadTravelTimeSpeedNet(first), 9.49, "runtime switch does not restore correction");
    near(graph.roads[0].length, 100.0, "predictor mutated shared road geometry");
    near(graph.roads[0].laneStorageLength[0], storage, "predictor changed lane storage length");
    check(graph.roads[0].laneCapacity[0] == capacity && graph.roads[0].laneFlow[0] == 0 &&
          graph.roads[0].roadFlow == 0, "prediction changed physical capacity or vehicle counts");
    const auto differentRoad = graph.buildBasicRoadModelFeatures(1, 0, -1, 0, 0);
    check(!differentRoad.is_route_start, "route index zero corrected a different road");
    near(graph.predictRoadTravelTimeSpeedNet(differentRoad), 10.0, "different road was shortened");
    graph.vehicles[0].roadIndex = 1;
    graph.vehicles[0].currentRoadID = 1;
    const auto final = graph.buildBasicRoadModelFeatures(1, 0, -1, 0, 0);
    check(!final.is_route_start, "ordinary final road was shortened");
    near(graph.predictRoadTravelTimeSpeedNet(final), 10.0, "final-road prediction changed");
    graph.vehicles[0].routeRoadIDs = {0, 1, 0};
    graph.vehicles[0].roadIndex = 2;
    graph.vehicles[0].currentRoadID = 0;
    const auto revisit = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
    check(!revisit.is_route_start, "starting edge revisit reapplied departure position");
    near(graph.predictRoadTravelTimeSpeedNet(revisit), 10.0, "revisited edge lost departure offset twice");
}

void test_sf_acceleration_queue_and_table_contract() {
    Graph graph;
    prepare(graph);
    Fixture input(document("<vehicle id=\"v\" route=\"r\" speedFactor=\"0.5\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    bindVehicles(graph);
    auto features = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
    graph.speedNetUseSpeedFactor = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 18.98, "position correction failed with per-vehicle SF cap");
    graph.speedNetUseAcceleration = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 20.23, "acceleration/cruise did not use remaining distance");
    graph.speedNetUseFollowing = true;
    graph.speedNetHeadway = 0.2;
    graph.speedNetDelayMax = 3.0;
    features.lane_flow = 4;
    features.road_flow = 4;
    near(graph.predictRoadTravelTimeSpeedNet(features), 20.83, "departure correction incorrectly scaled following delay");
    features.lane_waiting_buffer_length = 19.5;
    graph.speedNetQueueTrim = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 16.93, "departure and queue offsets did not compose once each");
    graph.speedNetDepartPosition = false;
    near(graph.predictRoadTravelTimeSpeedNet(features), 17.95, "off did not retain existing SF/acceleration/queue formula");
    graph.speedNetDepartPosition = true;

    graph.travelTimeTableFormat = TravelTimeTableFormat::SUMO_V1;
    const auto key = graph.buildSumoV1TravelTimeKeyForPrediction(features);
    graph.sumoV1TravelTimeTable[key] = 23.125;
    near(graph.predictRoadTravelTimeTable(features), 23.125, "Table hit was scaled by departure distance");
    graph.speedNetDepartPosition = false;
    check(key == graph.buildSumoV1TravelTimeKeyForPrediction(features), "departure option changed legacy Table key");
    near(graph.predictRoadTravelTimeTable(features), 23.125, "off/on Table hit differs");
    graph.speedNetDepartPosition = true;
    // A hypothetical shortened-key lookup must be absent: production features
    // keep the original full length and must continue to hit the stored key.
    auto shortenedKeyFeatures = features;
    shortenedKeyFeatures.road_length -= shortenedKeyFeatures.departure_position;
    check(!(key == graph.buildSumoV1TravelTimeKeyForPrediction(shortenedKeyFeatures)),
          "test fixture failed to distinguish physical and effective Table keys");

    features.road_length = 6.1;
    features.lane_waiting_buffer_length = 0.0;
    near(graph.predictRoadTravelTimeSpeedNet(features), 1.6, "short remaining distance missed acceleration-only branch");
    features.lane_waiting_buffer_length = 1000.0;
    near(graph.predictRoadTravelTimeSpeedNet(features), 1.0, "combined offsets escaped nonnegative length/time floor");
    features.lane_flow = 100;
    near(graph.predictRoadTravelTimeSpeedNet(features), 3.0, "zero remaining distance lost unchanged following cap");
}

void test_unsupported_and_invalid_positions() {
    for (const string& attributes : {
             string("departPos=\"random\""), string("departPos=\"0\""),
             string("departPos=\"5.1\""), string("departPos=\"base\" departEdge=\"1\"")}) {
        Graph graph;
        prepare(graph);
        Fixture input(document("<vehicle id=\"bad\" route=\"r\" " + attributes + "/>"));
        // MIN_TIME/Table may preserve metadata; only consuming the correction
        // should fail. This makes unsupported inputs explicit, not approximate.
        graph.read_sumo_route_xml(input.path, -1);
        check(!graph.vehicleDeparturePositionIssues[0].empty(), "unsupported position policy silently resolved");
        bindVehicles(graph);
        const auto features = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
        rejects([&]() { graph.predictRoadTravelTimeSpeedNet(features); }, "unsupported runtime correction not rejected");
        graph.speedNetDepartPosition = false;
        near(graph.predictRoadTravelTimeSpeedNet(features), 10.0, "off must retain legacy behavior for unsupported policies");
        graph.travelTimeMode = TravelTimeMode::SPEED_NET;
        graph.read_sumo_route_xml(input.path, -1);
        graph.speedNetDepartPosition = true;
        rejects([&]() { graph.read_sumo_route_xml(input.path, -1); }, "active speed-net route load did not fail fast");
    }

    Graph graph;
    prepare(graph);
    Fixture input(document("<vehicle id=\"v\" route=\"r\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    bindVehicles(graph);
    for (double bad : {-0.1, 100.0, 101.0, numeric_limits<double>::quiet_NaN(), numeric_limits<double>::infinity()}) {
        graph.vehicleDeparturePositions[0] = bad;
        const auto features = graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0);
        rejects([&]() { graph.predictRoadTravelTimeSpeedNet(features); }, "invalid in-memory departure position accepted");
    }
    graph.roads[0].length = 5.1;
    graph.travelTimeMode = TravelTimeMode::SPEED_NET;
    rejects([&]() { graph.read_sumo_route_xml(input.path, -1); }, "unsupported short-road base was not rejected");
}

void test_single_road_entry_and_final_exit() {
    for (bool enabled : {false, true}) {
        Graph graph;
        prepare(graph);
        graph.travelTimeMode = TravelTimeMode::SPEED_NET;
        graph.speedNetDepartPosition = enabled;
        Fixture input(document("<vehicle id=\"single\" depart=\"2.25\" departPos=\"base\"><route edges=\"a\"/></vehicle>"));
        graph.read_sumo_route_xml(input.path, -1);
        graph.initialize_cycle_aware_vehicles(graph.queryDataRaw, graph.routeRoadID);
        check(graph.vehicles.size() == 1 && graph.vehicles[0].valid, "single-road initialization failed");
        check(graph.tryStartVehicleFromEntryQueue(0, 2.25, 0, 0), "single-road admission failed");
        const double expected = 2.25 + (enabled ? 9.49 : 10.0);
        near(graph.vehicles[0].arrivalTime, expected, "single-road departure offset applied incorrectly");
        check(graph.vehicles[0].queuedForFinalExit && graph.roads[0].laneFlow[0] == 1,
              "single-road car failed to retain occupancy until exit");
        near(graph.roads[0].laneWaitingBufferLength[0], 0.0, "single-road car entered intersection waiting buffer");
        check(!graph.processFinalRoadExitsUntil(expected - 0.01), "single-road car exited before traversal completed");
        check(graph.processFinalRoadExitsUntil(expected), "single-road final event did not fire");
        check(graph.vehicles[0].finished && graph.finishedVehicleCount == 1 && graph.roads[0].laneFlow[0] == 0,
              "single-road final event did not release occupancy/finish exactly once");
        check(!graph.processFinalRoadExitsUntil(expected + 1.0), "single-road duplicate final exit remained");
    }
}

void test_unresolved_type_geometry() {
    for (const string& length : {string("bad"), string(""), string("5junk"), string("0"), string("-5")}) {
        Graph graph;
        prepare(graph);
        Fixture input(document("<vehicle id=\"typed\" type=\"bad\" route=\"r\"/>",
            "<vType id=\"bad\" length=\"" + length + "\"/>"));
        graph.travelTimeMode = TravelTimeMode::SPEED_NET;
        graph.speedNetDepartPosition = false;
        bool rejected = false;
        try { graph.read_sumo_route_xml(input.path, -1); }
        catch (const runtime_error&) { rejected = true; }
        check(rejected, "malformed vType length must fail even when position correction is off");
    }
    for (const string& typeDeclaration : {string(""), string("<vType id=\"truck\" vClass=\"truck\"/>")}) {
        Graph graph;
        prepare(graph);
        Fixture input(document("<vehicle id=\"typed\" type=\"truck\" route=\"r\"/>", typeDeclaration));
        graph.travelTimeMode = TravelTimeMode::SPEED_NET;
        graph.speedNetDepartPosition = false;
        if (typeDeclaration.empty()) {
            bool rejected = false;
            try { graph.read_sumo_route_xml(input.path, -1); }
            catch (const runtime_error&) { rejected = true; }
            check(rejected, "unknown type must fail instead of using car geometry");
        } else {
            graph.read_sumo_route_xml(input.path, -1);
            near(graph.vehicleDeparturePositions[0], 7.2, "truck class uses SUMO length 7.1, not passenger length");
        }
    }
}

}  // namespace

int main() {
    try {
        test_parse_base_types_and_reload();
        test_first_only_and_physical_geometry();
        test_sf_acceleration_queue_and_table_contract();
        test_unsupported_and_invalid_positions();
        test_single_road_entry_and_final_exit();
        test_unresolved_type_geometry();
        cout << "departure position regression passed (" << checks << " checks)\n";
        return 0;
    } catch (const exception& error) {
        cerr << "departure position regression failed: " << error.what() << '\n';
        return 1;
    }
}
