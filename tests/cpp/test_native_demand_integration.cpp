#include "head.h"
#include "native_table_fixture.h"
#include <chrono>
#include <cstdio>

using namespace native_test;

struct NativeFixture {
    std::string table_path, requests_path, route_path;
    NativeFixture() : NativeFixture(header(300000000), {{1, 1, 1.0}}) {}
    NativeFixture(const Json& meta, const std::vector<Entry>& entries) {
        const auto identity = std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
        table_path = ".native_demand_" + identity + ".bin";
        requests_path = ".native_requests_" + identity + ".csv";
        route_path = ".native_route_" + identity + ".rou.xml";
        std::ofstream out(table_path, std::ios::binary);
        out << table(meta, entries);
        check(bool(out), "cannot write independent test table");
    }
    ~NativeFixture() { std::remove(table_path.c_str()); std::remove(requests_path.c_str()); std::remove(route_path.c_str()); }
};

void first_road_policy_tests() {
    // Same Table prediction, independent on/off ABC distance. Actual cap is 3,
    // above the two-node table max 2: only the lookup must clamp, not free clocks.
    for (const double length : {100., 4.46}) {
        for (const bool correction : {false, true}) {
            auto meta = query_policy(header(), correction);
            meta.object["road_lengths"].array[0] = JsonParser(length == 100. ? "100.0" : "4.46").parse();
            NativeFixture fixture(meta, {{19, 1, 7.0}});
            {
                std::ofstream route(fixture.route_path);
                route << "<routes><vType id=\"car\" length=\"5\" minGap=\"2.5\" maxSpeed=\"3\"/>"
                      << "<vehicle id=\"v\" type=\"car\" speedFactor=\"1\" depart=\"2.25\" departPos=\"base\">"
                      << "<route edges=\"a\"/></vehicle></routes>";
            }
            Graph graph;
            graph.travelTimeMode = TravelTimeMode::TABLE;
            graph.speedNetDepartPosition = correction;
            graph.enableBasicFeatureLogging = true;
            graph.buildDictionary(fixture.table_path);
            graph.roads.resize(1);
            auto& road = graph.roads[0];
            road.roadID = 0; road.fromNode = 0; road.toNode = 1;
            road.length = length; road.speedLimit = 3; road.laneNum = 1;
            road.laneFlow.assign(1, 0);
            graph.sumoEdgeStrToRoadID["a"] = 0;
            if (correction && length < 5.1) {
                rejects([&] { graph.read_sumo_route_xml(fixture.route_path, -1); }, "short base first road rejected with correction on");
                continue;
            }
            graph.read_sumo_route_xml(fixture.route_path, -1);
            check(graph.sumoVehicleIDs.size() == 1, "off does not drop the short-first-road vehicle");
            graph.initialize_cycle_aware_vehicles(graph.queryDataRaw, graph.routeRoadID);
            const auto capacity = road.laneCapacity;
            check(graph.tryStartVehicleFromEntryQueue(0, 2.25, 0, 0), "native first-road admission");
            check(graph.basicFeatureSnapshots.size() == 1, "native first-road snapshot");
            const auto& features = graph.basicFeatureSnapshots.back();
            check(std::abs(features.context_distance - (correction ? length - 5.1 : length)) < 1e-9,
                  "native ABC clock honors first-road switch");
            check(features.free_speed_cap == 3 && features.road_length == length,
                  "clamp must not mutate actual cap or physical length");
            check(road.length == length && road.laneCapacity == capacity && road.laneFlow[0] == 1,
                  "first-road policy preserves geometry, capacity and occupancy");
            check(graph.vehicles[0].arrivalTime == 9.25, "Table result is not distance rescaled");
            check(!graph.processFinalRoadExitsUntil(9.24), "no early final exit");
            check(graph.processFinalRoadExitsUntil(9.25) && graph.vehicles[0].finished
                  && graph.finishedVehicleCount == 1 && road.laneFlow[0] == 0 && graph.capContextLedger.size() == 0,
                  "native final exit and ABC/occupancy cleanup");
            Graph mismatch;
            mismatch.travelTimeMode = TravelTimeMode::TABLE;
            mismatch.speedNetDepartPosition = !correction;
            rejects([&] { mismatch.buildDictionary(fixture.table_path); }, "wrong runtime/table policy fails before route load");
        }
    }
}

int main() {
    try {
        first_road_policy_tests();
        NativeFixture fixture;
        Graph graph;
        graph.travelTimeMode = TravelTimeMode::TABLE;
        graph.intersectionCrossingTime = 3.0;
        graph.buildDictionary(fixture.table_path);
        check(graph.usesNativeRoadTimeTable() && graph.usesDemandCapContextTable()
              && graph.usesRGCapContextTable() && graph.usesABCCapContextTable(), "native dispatch capability flags");
        check(graph.capContextOverdueThreshold() == 0, "native ABC existing threshold");
        graph.requireCapContextCrossingTime();
        graph.requireRGUnsignalizedContract();
        graph.requireRGNoInternalLinksContract();
        BasicRoadModelFeatures features;
        features.road_length = 100; features.road_flow = 1; features.lane_flow = 1;
        features.free_speed_cap = 1; features.platoon_context = 0;
        features.signal_rg = NoSignalRGFeatures(); features.signal_rg_available = true;
        check(graph.predictRoadTravelTimeTable(features) == 1.0 && graph.travelTimeTableHit == 1,
              "native hit dispatch preserves floor1 value");
        features.road_flow = 2; features.lane_flow = 2;
        features.free_speed_cap = 300000000; features.platoon_context = 5;
        rejects([&] { graph.predictRoadTravelTimeTable(features); }, "strict native miss must fail");
        graph.demandRequestsPath = fixture.requests_path;
        graph.demandMaxMissingKeys = 1;
        graph.demandRequests.open(fixture.requests_path);
        graph.demandRequests << "group_index,road_flow\n";
        features.speed_limit = 10;
        // Formula fallback only fills discovery, not final results. Its numeric
        // value is deliberately not an assertion about native training truth.
        graph.predictRoadTravelTimeTable(features);
        graph.predictRoadTravelTimeTable(features);
        graph.demandRequests.close();
        check(graph.demandMissingCount() == 1, "discovery missing-key dedup");
        std::ifstream in(fixture.requests_path);
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        check(text == "group_index,road_flow\n5399999999,2\n", "CSV group must retain upper 32 bits");
        check(graph.travelTimeTableMiss == 3, "every provisional query counted");
        features.road_flow = 5;
        rejects([&] { graph.predictRoadTravelTimeTable(features); }, "native discovery must fail before exceeding unique-key memory limit");
        check(graph.demandMissingCount() == 1, "overflowing discovery key was not inserted");
        features.signal_rg_available = false;
        rejects([&] { graph.predictRoadTravelTimeTable(features); }, "missing signal must not become demand request");
        check(graph.demandMissingCount() == 1, "invalid-feature query cannot extend cache");
        std::cout << "native demand Graph integration tests passed\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
