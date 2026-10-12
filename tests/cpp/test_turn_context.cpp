#include "head.h"
#include "test_cap_context_fixture.h"

#include <cstdio>

namespace {
int checks = 0;
void check(bool value, const string& message) {
    ++checks;
    if (!value) throw runtime_error(message);
}

void prepare(Graph& graph) {
    graph.roads.resize(4);
    for (int index = 0; index < 4; ++index) {
        auto& road = graph.roads[index];
        road.roadID = index;
        road.length = 100;
        road.speedLimit = 10;
        road.laneNum = 1;
        road.laneFlow = {1};
        road.roadFlow = 1;
    }
    const int from[] = {0, 1, 2, 1};
    const int to[] = {1, 2, 1, 3};
    const TurnDir turns[] = {TurnDir::Left, TurnDir::Right, TurnDir::UTurn, TurnDir::Straight};
    for (int index = 0; index < 4; ++index) {
        Movement movement;
        movement.movementID = index;
        movement.fromRoadID = from[index];
        movement.toRoadID = to[index];
        movement.turn = turns[index];
        graph.movements.push_back(movement);
    }
    graph.vehicles.resize(1);
    graph.vehicles[0].routeRoadIDs = {0, 1, 2, 1, 3};
    graph.vehicles[0].routeMovementIDs = {0, 1, 2, 3};
    graph.speedNetUseAcceleration = false;
    graph.speedNetUseFollowing = false;
}

BasicRoadModelFeatures at(Graph& graph, int index, int legacyMovement) {
    auto& vehicle = graph.vehicles[0];
    vehicle.roadIndex = index;
    // Deliberately stale: explicit context must not rely on currentMovementID.
    vehicle.currentMovementID = 0;
    return graph.buildBasicRoadModelFeatures(vehicle.routeRoadIDs[index], 0, legacyMovement, 12.5, 0);
}

vector<pair<string, BasicRoadModelFeatures>> contract_cases() {
    Graph graph;
    prepare(graph);
    vector<pair<string, BasicRoadModelFeatures>> cases;
    const string names[] = {"first", "incoming_left_outgoing_right", "incoming_right_outgoing_uturn",
                            "repeated_edge", "final"};
    for (int index = 0; index < 5; ++index)
        cases.push_back({names[index], at(graph, index, index ? index - 1 : 0)});
    graph.vehicles[0].routeRoadIDs = {1};
    graph.vehicles[0].routeMovementIDs.clear();
    cases.push_back({"single", at(graph, 0, -1)});
    graph.vehicles[0].routeRoadIDs = {0, 1, 2};
    graph.vehicles[0].routeMovementIDs = {-1, -1};
    cases.push_back({"missing_connections", at(graph, 1, -1)});
    return cases;
}

void test_encoding() {
    Graph graph;
    const string chars = "lLsSrRtU?ux";
    const int expected[] = {1, 1, 2, 2, 3, 3, 4, 4, 0, 0, 0};
    for (size_t index = 0; index < chars.size(); ++index)
        check(graph.encodeTurnDirForModel(graph.parseTurnDir(chars[index])) == expected[index],
              "turn parser/encoding mismatch");
    check(static_cast<int>(TurnDir::Left) == 0 && static_cast<int>(TurnDir::Straight) == 1 &&
          static_cast<int>(TurnDir::Right) == 2 && static_cast<int>(TurnDir::UTurn) == 3 &&
          static_cast<int>(TurnDir::Unknown) == 4, "control enum ordinals changed");
}

void test_route_occurrences_and_legacy_keys() {
    Graph graph;
    prepare(graph);
    const int incoming[] = {0, 1, 3, 4, 2};
    const int outgoing[] = {1, 3, 4, 2, 0};
    const int legacy[] = {1, 1, 3, 4, 2};
    graph.travelTimeTableFormat = TravelTimeTableFormat::SUMO_V1;
    for (int index = 0; index < 5; ++index) {
        const auto features = at(graph, index, index ? index - 1 : 0);
        check(features.route_context_resolved && features.route_index == index, "occurrence unresolved");
        check(features.incoming_turn_present == (index > 0), "incoming presence incorrect");
        check(features.outgoing_turn_present == (index < 4), "outgoing presence incorrect");
        check(features.incoming_movement_id == (index ? index - 1 : -1), "incoming movement incorrect");
        check(features.outgoing_movement_id == (index < 4 ? index : -1), "outgoing movement incorrect");
        check(features.incoming_turn_type == incoming[index], "incoming code incorrect");
        check(features.outgoing_turn_type == outgoing[index] && features.turn_type == outgoing[index],
              "canonical outgoing alias incorrect");
        check(features.legacy_context_turn_type == legacy[index], "legacy mixed context changed");
        const auto key = graph.buildSumoV1TravelTimeKeyForPrediction(features);
        check(key.turn_type == legacy[index], "SUMO_V1 key no longer uses legacy context");
        auto oldManualFeatures = features;
        oldManualFeatures.legacy_context_turn_type = -1;
        oldManualFeatures.turn_type = legacy[index];
        check(key == graph.buildSumoV1TravelTimeKeyForPrediction(oldManualFeatures), "legacy key changed");
        check(SumoV1TravelTimeKeyHash{}(key) ==
              SumoV1TravelTimeKeyHash{}(graph.buildSumoV1TravelTimeKeyForPrediction(oldManualFeatures)),
              "legacy key hash changed");
        graph.sumoV1TravelTimeTable[key] = 20.125 + index;
        check(graph.predictRoadTravelTimeTable(features) == 20.125 + index, "legacy hit/value changed");
        auto metadataOnly = features;
        metadataOnly.turn_type = 99;
        metadataOnly.incoming_turn_type = 98;
        metadataOnly.outgoing_turn_type = 97;
        check(graph.predictRoadTravelTimeTable(metadataOnly) == 20.125 + index,
              "canonical metadata contaminated legacy query");
        check(graph.predictRoadTravelTimeSpeedNet(features) == graph.predictRoadTravelTimeSpeedNet(metadataOnly),
              "turn metadata changed speed-net");
        check(graph.predictRoadTravelTimeKinematic(features) == graph.predictRoadTravelTimeKinematic(metadataOnly),
              "turn metadata changed kinematic mode");
        check(graph.buildRoadKeyForPrediction(features) == graph.buildRoadKeyForPrediction(metadataOnly),
              "turn metadata changed old RoadKey");
    }
    const auto noContext = at(graph, 1, -1);
    check(noContext.turn_type == 3 && noContext.legacy_context_turn_type == 0,
          "missing legacy argument must not overwrite outgoing metadata");
    check(graph.buildSumoV1TravelTimeKeyForPrediction(noContext).turn_type == 0,
          "unknown constructed legacy context incorrectly activated fallback");
    const auto manual = BasicRoadModelFeatures{};
    check(manual.legacy_context_turn_type == -1, "hand-built fallback sentinel lost");
}

void test_missing_and_boundary_metadata() {
    const auto cases = contract_cases();
    const auto& single = cases[5].second;
    check(single.route_context_resolved && single.route_index == 0, "single occurrence unresolved");
    check(!single.incoming_turn_present && !single.outgoing_turn_present && single.turn_type == 0 &&
          single.incoming_turn_type == 0 && single.outgoing_turn_type == 0, "single boundary incorrect");
    const auto& missing = cases[6].second;
    check(missing.route_context_resolved && missing.incoming_turn_present && missing.outgoing_turn_present,
          "missing connection confused with absent adjacent road");
    check(missing.incoming_movement_id == -1 && missing.outgoing_movement_id == -1 &&
          missing.incoming_turn_type == 0 && missing.outgoing_turn_type == 0, "missing turn guessed");
    Graph graph;
    prepare(graph);
    graph.vehicles[0].routeMovementIDs = {0};
    const auto shortened = at(graph, 1, 0);
    check(shortened.route_context_resolved && shortened.incoming_turn_present && shortened.outgoing_turn_present &&
          shortened.incoming_movement_id == -1 && shortened.outgoing_movement_id == -1,
          "shortened movement vector treated as aligned");
    graph.vehicles[0].routeMovementIDs = {0, 0, 2, 3};
    const auto mismatch = at(graph, 1, 0);
    check(mismatch.incoming_turn_type == 1 && mismatch.outgoing_turn_present &&
          mismatch.outgoing_turn_type == 0 && mismatch.outgoing_movement_id == -1,
          "wrong-endpoint movement accepted");
    graph.vehicles[0].routeMovementIDs = {0, 1, 2, 3};
    graph.movements[1].turn = TurnDir::Unknown;
    const auto unknown = at(graph, 1, 0);
    check(unknown.outgoing_turn_present && unknown.outgoing_turn_type == 0 && unknown.outgoing_movement_id == 1,
          "known movement with unknown direction lost presence/identity");
    const auto wrongRoad = graph.buildBasicRoadModelFeatures(2, 0, 0, 0, 0);
    check(!wrongRoad.route_context_resolved && wrongRoad.route_index == -1 &&
          !wrongRoad.incoming_turn_present && !wrongRoad.outgoing_turn_present && wrongRoad.turn_type == 0,
          "wrong queried road inferred an occurrence");
    check(wrongRoad.legacy_context_turn_type == 1, "unresolved route changed old argument semantics");
    graph.vehicles[0].roadIndex = -1;
    check(!graph.buildBasicRoadModelFeatures(0, 0, 0, 0, 0).route_context_resolved, "negative index accepted");
    graph.vehicles[0].roadIndex = 5;
    check(!graph.buildBasicRoadModelFeatures(0, 0, 0, 0, 0).route_context_resolved, "past-end index accepted");
    const auto noVehicle = graph.buildBasicRoadModelFeatures(0, -1, 0, 0, 0);
    check(!noVehicle.route_context_resolved && noVehicle.turn_type == 0 && noVehicle.legacy_context_turn_type == 1,
          "vehicle-less legacy query no longer compatible");
    const auto invalidRoad = graph.buildBasicRoadModelFeatures(-1, 0, 0, 0, 0);
    check(!invalidRoad.route_context_resolved && invalidRoad.turn_type == 0 && invalidRoad.legacy_context_turn_type == 0,
          "invalid road early return no longer compatible");
}

void test_f13_turn_independence() {
    Graph graph;
    prepare(graph);
    graph.travelTimeTableFormat = TravelTimeTableFormat::CAMS_CAP_CONTEXT_RLE_V1;
    istringstream input(cap_context_test::fixture());
    graph.capContextTable.load(input);
    graph.intersectionCrossingTime = 3;
    auto features = at(graph, 1, 0);
    features.free_speed_cap = 10;
    features.platoon_context = 2;
    const double reference = graph.predictRoadTravelTimeTable(features);
    for (int code = 0; code <= 4; ++code) {
        features.turn_type = code;
        features.incoming_turn_type = code;
        features.outgoing_turn_type = code;
        features.legacy_context_turn_type = code;
        check(graph.predictRoadTravelTimeTable(features) == reference, "turn contaminated F13 five-axis lookup");
    }
}

void test_snapshot_contract() {
    Graph graph;
    prepare(graph);
    graph.basicFeatureSnapshots.push_back(at(graph, 1, 0));
    const string path = "turn_context_test_snapshot.csv";
    check(!ifstream(path).good(), "refuse to overwrite existing snapshot fixture");
    struct Cleanup { string path; ~Cleanup() { std::remove(path.c_str()); } } cleanup{path};
    graph.exportBasicFeatureSnapshots(path);
    ifstream input(path);
    string header, row;
    getline(input, header);
    getline(input, row);
    check(header.find("road_length,turn_type,road_flow,lane_flow,") != string::npos,
          "snapshot original column order changed");
    check(header.find("turn_context_schema,route_index,route_context_resolved,incoming_movement_id,outgoing_movement_id,")
          != string::npos, "snapshot missing versioned occurrence fields");
    check(row.find(",road-turn-context-v1,1,1,0,1,1,1,1,3,1") != string::npos,
          "snapshot context columns incorrectly aligned");
    check(count(header.begin(), header.end(), ',') == count(row.begin(), row.end(), ','),
          "snapshot header/row width mismatch");
}

void emit_fixture() {
    cout << "{\"turn_context_schema\":\"road-turn-context-v1\",\"cases\":[";
    bool first = true;
    for (const auto& entry : contract_cases()) {
        if (!first) cout << ',';
        first = false;
        const auto& f = entry.second;
        cout << "{\"name\":\"" << entry.first << "\",\"route_index\":" << f.route_index
             << ",\"route_context_resolved\":" << (f.route_context_resolved ? "true" : "false")
             << ",\"incoming_turn_present\":" << (f.incoming_turn_present ? "true" : "false")
             << ",\"outgoing_turn_present\":" << (f.outgoing_turn_present ? "true" : "false")
             << ",\"incoming_turn_type\":" << f.incoming_turn_type
             << ",\"outgoing_turn_type\":" << f.outgoing_turn_type << ",\"turn_type\":" << f.turn_type
             << ",\"incoming_movement_id\":" << f.incoming_movement_id
             << ",\"outgoing_movement_id\":" << f.outgoing_movement_id
             << ",\"legacy_context_turn_type\":" << f.legacy_context_turn_type << '}';
    }
    cout << "]}\n";
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && string(argv[1]) == "--fixture") { emit_fixture(); return 0; }
        if (argc != 1) throw runtime_error("usage: TurnContextTest [--fixture]");
        test_encoding();
        test_route_occurrences_and_legacy_keys();
        test_missing_and_boundary_metadata();
        test_f13_turn_independence();
        test_snapshot_contract();
        cout << "Turn context checks passed: " << checks << '\n';
        return 0;
    } catch (const exception& error) {
        cerr << "Turn context test failed: " << error.what() << '\n';
        return 1;
    }
}
