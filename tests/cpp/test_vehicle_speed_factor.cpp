#include "head.h"

#include <cstdio>

namespace {

void check(bool condition, const string& message) {
    if (!condition) throw runtime_error(message);
}

void near(double actual, double expected, const string& message) {
    check(std::isfinite(actual) && abs(actual - expected) <= 1e-10, message);
}

struct ScopedFixture {
    string path;
    explicit ScopedFixture(const string& content, const string& suffix = ".rou.xml") {
        static int counter = 0;
        path = "vehicle_speed_factor_test_" + to_string(++counter) + suffix;
        ifstream existing(path);
        check(!existing.good(), "refusing to overwrite existing fixture: " + path);
        ofstream output(path);
        check(output.good(), "cannot create fixture: " + path);
        output << content;
    }
    ~ScopedFixture() { std::remove(path.c_str()); }
};

void prepare(Graph& graph) {
    graph.roads.resize(2);
    for (int index = 0; index < 2; ++index) {
        RoadSegment& road = graph.roads[index];
        road.roadID = index;
        road.fromNode = index;
        road.toNode = index + 1;
        road.length = 100.0;
        road.speedLimit = 13.89;
        road.laneNum = 1;
        road.laneFlow.assign(1, 0);
        graph.sumoEdgeStrToRoadID[index == 0 ? "a" : "b"] = index;
    }
    graph.speedNetUseAcceleration = false;
    graph.speedNetUseFollowing = false;
}

string route_document(const string& vehicles, const string& type = "<vType id=\"car\" maxSpeed=\"13.89\"/>") {
    string typedVehicles = vehicles;
    size_t pos = 0;
    while ((pos = typedVehicles.find("<vehicle ", pos)) != string::npos) {
        typedVehicles.insert(pos + 9, "type=\"car\" ");
        pos += 20;
    }
    return "<routes>" + type + "<route id=\"r\" edges=\"a b\"/>" + typedVehicles + "</routes>";
}

void test_route_reference_inline_and_unknown() {
    Graph graph;
    prepare(graph);
    const double exactFactor = 0.81234567890123456;
    ScopedFixture input(route_document(
        "<vehicle id=\"slow\" route=\"r\" depart=\"0\" speedFactor=\"0.81234567890123456\"/>"
        "<vehicle id=\"fast\" depart=\"1\" speedFactor=\"1.2\"><route edges=\"b\"/></vehicle>"
        "<vehicle id=\"missing\" route=\"r\" depart=\"2\"/>",
        "<vType id=\"car\" maxSpeed=\"13.89\" speedFactor=\"1.7\" speedDev=\"0.1\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    check(graph.sumoVehicleIDs == vector<string>({"slow", "fast", "missing"}), "vehicle ordering changed");
    check(graph.routeRoadID[0] == vector<int>({0, 1}), "external route reference not parsed");
    check(graph.routeRoadID[1] == vector<int>({1}), "inline route not parsed");
    near(graph.getVehicleSpeedFactor(0), exactFactor, "full-precision per-vehicle SF lost");
    near(graph.getVehicleSpeedFactor(1), 1.2, "second vehicle SF not independent");
    check(graph.hasExplicitVehicleSpeedFactor(0), "explicit flag lost");
    check(!graph.hasExplicitVehicleSpeedFactor(2), "missing SF marked explicit");
    check(std::isnan(graph.getVehicleSpeedFactor(2)), "vType distribution mean substituted for missing SF");
    check(std::isnan(graph.getVehicleSpeedFactor(-1)), "invalid vehicle should be unknown");
    check(std::isnan(graph.getVehicleSpeedFactor(3)), "out-of-range vehicle should be unknown");
    near(graph.getVehicleFreeSpeedCap(0, 13.89), exactFactor * 13.89, "slow cap incorrect");
    near(graph.getVehicleFreeSpeedCap(1, 13.89), 13.89, "maxSpeed failed to cap SF > 1");
    near(graph.getVehicleFreeSpeedCap(1, 6.51), 7.812, "lower lane limit failed to preserve SF > 1");
    check(std::isnan(graph.getVehicleFreeSpeedCap(2, 13.89)), "unknown SF yielded known free cap");
    check(std::isnan(graph.getVehicleFreeSpeedCap(0, 0.0)), "invalid lane limit yielded known cap");

    const auto slowFeatures = graph.buildBasicRoadModelFeatures(0, 0, -1, 1.23456789, 0);
    const auto missingFeatures = graph.buildBasicRoadModelFeatures(0, 2, -1, 1.23456789, 0);
    near(slowFeatures.speed_factor, exactFactor, "snapshot did not retain SF");
    near(slowFeatures.free_speed_cap, exactFactor * 13.89, "snapshot cap incorrect");
    check(!missingFeatures.has_explicit_speed_factor && std::isnan(missingFeatures.speed_factor),
          "snapshot did not preserve missing SF");
    near(graph.predictRoadTravelTimeSpeedNet(slowFeatures),
         graph.predictRoadTravelTimeSpeedNet(missingFeatures), "SF changed existing speed-net predictions");
    near(graph.predictRoadTravelTimeKinematic(slowFeatures),
         graph.predictRoadTravelTimeKinematic(missingFeatures), "SF changed existing kinematic predictions");

    graph.basicFeatureSnapshots = {slowFeatures, missingFeatures};
    ScopedFixture output("", ".csv");
    graph.exportBasicFeatureSnapshots(output.path);
    ifstream trace(output.path);
    string header, first, second;
    getline(trace, header);
    getline(trace, first);
    getline(trace, second);
    const string sfColumns = ",speed_factor,has_explicit_speed_factor,free_speed_cap,";
    const string turnColumns = "turn_context_schema,route_index,route_context_resolved,incoming_movement_id,"
        "outgoing_movement_id,incoming_turn_present,outgoing_turn_present,incoming_turn_type,"
        "outgoing_turn_type,legacy_context_turn_type";
    check(header.size() >= sfColumns.size() + turnColumns.size() &&
          header.substr(header.size() - sfColumns.size() - turnColumns.size()) == sfColumns + turnColumns,
          "trace changed SF metadata order or omitted versioned turn append columns");
    vector<string> firstCells;
    istringstream firstRow(first);
    string cell;
    while (getline(firstRow, cell, ',')) firstCells.push_back(cell);
    check(firstCells.size() == 30 && stod(firstCells[17]) == exactFactor && firstCells[18] == "1" &&
          stod(firstCells[19]) == slowFeatures.free_speed_cap,
          "trace did not round-trip full SF precision");
    vector<string> secondCells;
    istringstream secondRow(second);
    while (getline(secondRow, cell, ',')) secondCells.push_back(cell);
    check(secondCells.size() == 30 && secondCells[17].empty() && secondCells[18] == "0" &&
          secondCells[19].empty(), "unknown trace metadata must use blank cells");
    check(firstCells[20] == "road-turn-context-v1" && secondCells[20] == "road-turn-context-v1",
          "turn metadata schema missing from appended columns");
    check(first.find("1.23457,") == 0 && second.find("1.23457,") == 0,
          "new metadata changed precision of existing trace columns");

    ScopedFixture legacy(route_document("<vehicle id=\"legacy\" route=\"r\" depart=\"0\"/>"));
    graph.read_sumo_route_xml(legacy.path, -1);
    check(graph.vehicleSpeedFactors.size() == 1 && graph.vehicleHasExplicitSpeedFactor.size() == 1,
          "reloading route retained stale SF metadata");
    check(!graph.hasExplicitVehicleSpeedFactor(0) && std::isnan(graph.getVehicleSpeedFactor(0)),
          "legacy reload inherited previous SF");
    near(graph.predictRoadTravelTimeSpeedNet(graph.buildBasicRoadModelFeatures(0, 0, -1, 0, 0)),
         100.0 / 13.89, "legacy route prediction changed");
}

void test_strict_numbers_and_alignment() {
    for (const string& bad : {"0", "-1", "NaN", "nan", "inf", "-inf", "1.2junk", "", " ", "1e999"}) {
        Graph graph;
        prepare(graph);
        ScopedFixture input(route_document("<vehicle id=\"bad\" route=\"r\" speedFactor=\"" + bad + "\"/>"));
        bool rejected = false;
        try { graph.read_sumo_route_xml(input.path, -1); }
        catch (const runtime_error& error) {
            rejected = string(error.what()).find("vehicle=bad speedFactor") != string::npos;
        }
        check(rejected, "invalid SF accepted or insufficient diagnostic: '" + bad + "'");
    }
    Graph graph;
    prepare(graph);
    ScopedFixture input(route_document(
        "<vehicle id=\"skip\" depart=\"0\" speedFactor=\"0.7\"><route edges=\"unknown\"/></vehicle>"
        "<vehicle id=\"keep\" route=\"r\" depart=\"1\" speedFactor=\" 1.125 \"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    check(graph.sumoVehicleIDs == vector<string>({"keep"}), "invalid route skipping changed");
    near(graph.getVehicleSpeedFactor(0), 1.125, "skipped route misaligned SF");
}

void test_desired_speed_cap() {
    Graph graph;
    prepare(graph);
    ScopedFixture input(route_document(
        "<vehicle id=\"v\" route=\"r\" speedFactor=\"1.2\"/>",
        "<vType id=\"car\" maxSpeed=\"13.89\" desiredMaxSpeed=\"8\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    near(graph.getVehicleFreeSpeedCap(0, 13.89), 9.6, "desiredMaxSpeed not included in cap");
    ScopedFixture invalid(route_document("", "<vType id=\"car\" desiredMaxSpeed=\"nan\"/>"));
    bool rejected = false;
    try { graph.read_sumo_route_xml(invalid.path, -1); }
    catch (const runtime_error&) { rejected = true; }
    check(rejected, "invalid desiredMaxSpeed accepted");
}

void test_speed_net_sf_cruise_and_caps() {
    Graph graph;
    prepare(graph);
    graph.roads[0].speedLimit = 10.0;
    check(!graph.speedNetUseSpeedFactor, "SF speed-net cap must default off");
    ScopedFixture input(route_document(
        "<vehicle id=\"slow\" route=\"r\" speedFactor=\"0.5\"/>"
        "<vehicle id=\"fast\" route=\"r\" speedFactor=\"1.2\"/>"
        "<vehicle id=\"capped\" route=\"r\" speedFactor=\"3\"/>"
        "<vehicle id=\"missing\" route=\"r\"/>",
        "<vType id=\"car\" maxSpeed=\"20\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    for (int vehicle = 0; vehicle < 4; ++vehicle) {
        near(graph.predictRoadTravelTimeSpeedNet(
                 graph.buildBasicRoadModelFeatures(0, vehicle, -1, 0.0, 0)),
             10.0, "SF off changed the original cruise formula");
    }
    graph.speedNetUseSpeedFactor = true;
    const double expected[] = {20.0, 100.0 / 12.0, 5.0};
    for (int vehicle = 0; vehicle < 3; ++vehicle) {
        const auto features = graph.buildBasicRoadModelFeatures(0, vehicle, -1, 0.0, 0);
        near(graph.predictRoadTravelTimeSpeedNet(features), expected[vehicle],
             "SF on did not apply the per-vehicle road/maxSpeed cap");
        graph.speedNetUseSpeedFactor = false;
        const double oldKinematic = graph.predictRoadTravelTimeKinematic(features);
        graph.speedNetUseSpeedFactor = true;
        near(graph.predictRoadTravelTimeKinematic(features), oldKinematic,
             "SF speed-net option leaked into another predictor");
    }
    bool missingRejected = false;
    try {
        graph.predictRoadTravelTimeSpeedNet(graph.buildBasicRoadModelFeatures(0, 3, -1, 0.0, 0));
    } catch (const runtime_error& error) {
        missingRejected = string(error.what()).find("speedFactor") != string::npos;
    }
    check(missingRejected, "SF on silently accepted a vehicle without explicit SF");

    Graph desiredGraph;
    prepare(desiredGraph);
    desiredGraph.roads[0].speedLimit = 10.0;
    ScopedFixture desiredInput(route_document(
        "<vehicle id=\"desired\" route=\"r\" speedFactor=\"1.5\"/>",
        "<vType id=\"car\" maxSpeed=\"20\" desiredMaxSpeed=\"8\"/>"));
    desiredGraph.read_sumo_route_xml(desiredInput.path, -1);
    const auto desiredFeatures = desiredGraph.buildBasicRoadModelFeatures(0, 0, -1, 0.0, 0);
    near(desiredGraph.predictRoadTravelTimeSpeedNet(desiredFeatures), 10.0,
         "SF off unexpectedly applied desiredMaxSpeed");
    desiredGraph.speedNetUseSpeedFactor = true;
    near(desiredGraph.predictRoadTravelTimeSpeedNet(desiredFeatures), 100.0 / 12.0,
         "SF on did not scale desiredMaxSpeed before taking the minimum");
}

void test_speed_net_sf_acceleration_and_following() {
    Graph graph;
    prepare(graph);
    graph.roads[0].speedLimit = 10.0;
    ScopedFixture input(route_document(
        "<vehicle id=\"v\" route=\"r\" speedFactor=\"0.5\"/>",
        "<vType id=\"car\" maxSpeed=\"20\" accel=\"2\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    graph.speedNetUseAcceleration = true;
    auto features = graph.buildBasicRoadModelFeatures(0, 0, -1, 0.0, 0);
    features.road_length = 4.0;
    graph.speedNetUseSpeedFactor = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 2.0,
         "SF changed a triangular acceleration segment before reaching the cap");
    graph.speedNetUseSpeedFactor = false;
    near(graph.predictRoadTravelTimeSpeedNet(features), 2.0,
         "SF off triangular acceleration changed");

    features.road_length = 6.25;
    graph.speedNetUseSpeedFactor = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 2.5,
         "SF acceleration/cruise boundary is discontinuous");
    features.road_length = 9.0;
    near(graph.predictRoadTravelTimeSpeedNet(features), 3.05,
         "SF target speed not applied to both acceleration and cruise segments");
    graph.speedNetUseSpeedFactor = false;
    near(graph.predictRoadTravelTimeSpeedNet(features), 3.0,
         "SF off changed the uncapped triangular segment");

    features.road_length = 100.0;
    near(graph.predictRoadTravelTimeSpeedNet(features), 12.5,
         "SF off changed long-road acceleration/cruise time");
    graph.speedNetUseSpeedFactor = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 21.25,
         "SF on changed the acceleration model beyond its target speed");
    graph.speedNetUseFollowing = true;
    graph.speedNetHeadway = 0.8;
    graph.speedNetDelayMax = 3.0;
    features.lane_flow = 4;
    near(graph.predictRoadTravelTimeSpeedNet(features), 23.65,
         "SF incorrectly scaled same-lane following delay");
    features.lane_flow = 100;
    near(graph.predictRoadTravelTimeSpeedNet(features), 24.25,
         "SF incorrectly changed the following-delay cap");
    graph.speedNetQueueTrim = true;
    features.lane_waiting_buffer_length = 24.0;
    near(graph.predictRoadTravelTimeSpeedNet(features), 19.45,
         "SF cap and entry-only queue-length trim do not compose correctly");
    graph.speedNetUseSpeedFactor = false;
    near(graph.predictRoadTravelTimeSpeedNet(features), 13.1,
         "SF off no longer preserves the old queue-trim/following formula");
    graph.speedNetQueueTrim = false;
    graph.speedNetUseFollowing = false;
    graph.speedNetUseAcceleration = false;
    graph.speedNetUseSpeedFactor = true;
    near(graph.predictRoadTravelTimeSpeedNet(features), 20.0,
         "disabling acceleration/following disabled the SF cruise cap");
}

void test_speed_net_sf_rejects_missing_or_invalid() {
    Graph missing;
    prepare(missing);
    missing.speedNetUseSpeedFactor = true;
    ScopedFixture missingInput(route_document(
        "<vehicle id=\"explicit\" route=\"r\" speedFactor=\"1\"/>"
        "<vehicle id=\"missing\" route=\"r\"/>"));
    bool rejected = false;
    try { missing.read_sumo_route_xml(missingInput.path, -1); }
    catch (const runtime_error& error) {
        const string message(error.what());
        rejected = message.find("speedFactor") != string::npos && message.find("missing") != string::npos;
    }
    check(rejected, "SF-enabled route loading did not reject incomplete SF coverage");

    Graph graph;
    prepare(graph);
    ScopedFixture input(route_document("<vehicle id=\"v\" route=\"r\" speedFactor=\"1\"/>"));
    graph.read_sumo_route_xml(input.path, -1);
    graph.speedNetUseSpeedFactor = true;
    // XML input validation is tested separately. This also protects direct API
    // callers or corrupt in-memory metadata from silently obtaining a fallback.
    for (double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                       std::numeric_limits<double>::infinity()}) {
        graph.vehicleSpeedFactors[0] = bad;
        bool invalidRejected = false;
        try {
            graph.predictRoadTravelTimeSpeedNet(graph.buildBasicRoadModelFeatures(0, 0, -1, 0.0, 0));
        } catch (const runtime_error& error) {
            invalidRejected = string(error.what()).find("speedFactor") != string::npos;
        }
        check(invalidRejected, "SF-enabled prediction accepted invalid in-memory SF");
        graph.speedNetUseSpeedFactor = false;
        near(graph.predictRoadTravelTimeSpeedNet(graph.buildBasicRoadModelFeatures(0, 0, -1, 0.0, 0)),
             100.0 / 13.89, "SF off no longer preserves legacy behavior with unused SF metadata");
        graph.speedNetUseSpeedFactor = true;
    }
}

}  // namespace

int main() {
    try {
        test_route_reference_inline_and_unknown();
        test_strict_numbers_and_alignment();
        test_desired_speed_cap();
        test_speed_net_sf_cruise_and_caps();
        test_speed_net_sf_acceleration_and_following();
        test_speed_net_sf_rejects_missing_or_invalid();
        cout << "vehicle speed factor tests passed" << endl;
        return 0;
    } catch (const std::exception& error) {
        cerr << "vehicle speed factor test failed: " << error.what() << endl;
        return 1;
    }
}
