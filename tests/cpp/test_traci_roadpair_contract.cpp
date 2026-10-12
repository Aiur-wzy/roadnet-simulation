#include "head.h"
#include <iomanip>

// Independent, bounded production-kernel probe. No network files, SUMO, model,
// scheduler replay, or production changes are used. The explicitly supplied
// aggregate loads are selector inputs, not a claim of a complete vehicle state.
namespace {
struct Fixture {
    string name;
    bool enabled = true;
    int source_lanes = 1, receiving_lanes = 2, source_lane = 0;
    bool final_road = false;
    vector<pair<int,int>> incoming{{0,0}}, outgoing{{0,0},{1,0}};
    vector<int> flow{0,0}, capacity;
    vector<double> occupied{0.0,0.0};
    vector<int> bus_only_incoming, bus_only_outgoing, bus_only_receiving;
    bool bus_only_source = false;
    bool missing_internal_path = false;
    bool expected_feasible = true;
    int expected_entry = 0, expected_logical = 0, expected_connection = 0;
};
int checks = 0;
void require(bool condition, const string& message) {
    ++checks;
    if (!condition) throw runtime_error(message);
}
template<class T> void array_json(ostream& out, const vector<T>& values) {
    out << '[';
    for (size_t i=0; i<values.size(); ++i) { if (i) out << ','; out << values[i]; }
    out << ']';
}
void pairs_json(ostream& out, const vector<pair<int,int>>& values) {
    out << '[';
    for (size_t i=0; i<values.size(); ++i) {
        if (i) out << ',';
        out << '[' << values[i].first << ',' << values[i].second << ']';
    }
    out << ']';
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
    g.movements.push_back(m);
    return m.movementID;
}
string run_case(const Fixture& f) {
    Graph g;
    g.mergeRoadPairMovements = true;
    g.laneEntryReselection = f.enabled;
    g.laneBorrowing = false;
    g.sumoNetworkWithoutInternalLinks = true;
    VehicleType type;
    type.id = "DEFAULT_VEHTYPE"; type.vehicleClass = "passenger";
    type.length = 5.0; type.minGap = 2.5;
    g.sumoVehicleTypes[type.id] = type;
    for (int count : vector<int>{f.source_lanes,f.receiving_lanes,1}) {
        RoadSegment r;
        r.roadID = static_cast<int>(g.roads.size());
        r.laneNum = count; r.length = 100.0; r.speedLimit = 10.0;
        g.roads.push_back(r);
    }
    add_movement(g,0,1,f.incoming);
    if (!f.final_road) add_movement(g,1,2,f.outgoing);
    for (int i : f.bus_only_incoming) g.sumoConnectionsRaw.at(i).allow = "bus";
    for (int i : f.bus_only_outgoing)
        g.sumoConnectionsRaw.at(f.incoming.size()+i).allow = "bus";
    if (f.missing_internal_path) g.sumoConnectionsRaw.at(0).via = ":missing_0";
    g.initializeRoadLaneStorage();
    for (int i : f.bus_only_receiving) g.roadPairLaneMetadata.at(1).at(i).allow = "bus";
    if (f.bus_only_source) g.roadPairLaneMetadata.at(0).at(f.source_lane).allow = "bus";
    const vector<int> default_capacity = g.roads[1].laneCapacity;
    require(default_capacity == vector<int>(f.receiving_lanes,14), f.name+": default count capacity");
    if (!f.capacity.empty()) g.roads[1].laneCapacity = f.capacity;
    g.roads[1].laneFlow = f.flow;
    g.roads[1].laneOccupiedLength = f.occupied;
    g.roads[1].roadFlow = accumulate(f.flow.begin(),f.flow.end(),0);
    VehicleLabel v;
    v.vehicleID = 0; v.routeID = 0; v.roadIndex = 0;
    v.currentRoadID = 0; v.currentMovementID = 0;
    v.occupiedRoadID = 0; v.occupiedLaneIndex = f.source_lane;
    v.storageLaneIndex = f.source_lane; v.occupiedLength = 7.5;
    v.routeRoadIDs = f.final_road ? vector<int>{0,1} : vector<int>{0,1,2};
    v.routeMovementIDs = f.final_road ? vector<int>{0} : vector<int>{0,1};
    g.vehicles.push_back(v); g.vehicleTypeIDs.push_back(type.id);
    g.roads[0].roadFlow = 1; g.roads[0].laneFlow[f.source_lane] = 1;
    g.roads[0].laneOccupiedLength[f.source_lane] = 7.5;

    const auto options = g.road_pair_lane_options(0,0);
    RoadPairChoice selected;
    const bool feasible = g.road_pair_select_lane(0,0,selected);
    require(feasible == f.expected_feasible, f.name+": feasibility");
    if (feasible) {
        require(selected.entryLane == f.expected_entry, f.name+": entry lane");
        require(selected.logicalLane == f.expected_logical, f.name+": logical lane");
        require(selected.connectionIndex == f.expected_connection, f.name+": raw connection tie");
    }
    require(g.roads[1].laneFlow == f.flow && g.roads[1].laneOccupiedLength == f.occupied,
            f.name+": preview must not reserve");

    ostringstream out; out << setprecision(17);
    out << "{\"name\":\"" << f.name << "\",\"enabled\":" << (f.enabled?"true":"false")
        << ",\"source_lanes\":" << f.source_lanes << ",\"receiving_lanes\":" << f.receiving_lanes
        << ",\"source_lane\":" << f.source_lane << ",\"final_road\":" << (f.final_road?"true":"false")
        << ",\"road_length\":100,\"representative_length\":5,\"representative_min_gap\":2.5,\"incoming\":";
    pairs_json(out,f.incoming); out << ",\"outgoing\":"; pairs_json(out,f.outgoing);
    out << ",\"flow\":"; array_json(out,f.flow);
    out << ",\"occupied_length\":"; array_json(out,f.occupied);
    out << ",\"capacity\":"; array_json(out,g.roads[1].laneCapacity);
    out << ",\"default_capacity\":"; array_json(out,default_capacity);
    out << ",\"bus_only_incoming\":"; array_json(out,f.bus_only_incoming);
    out << ",\"bus_only_outgoing\":"; array_json(out,f.bus_only_outgoing);
    out << ",\"bus_only_receiving\":"; array_json(out,f.bus_only_receiving);
    out << ",\"bus_only_source\":" << (f.bus_only_source?"true":"false")
        << ",\"missing_internal_path\":" << (f.missing_internal_path?"true":"false")
        << ",\"options\":[";
    for (size_t i=0; i<options.size(); ++i) {
        if (i) out << ',';
        out << '[' << options[i].connectionIndex << ',' << options[i].entryLane << ','
            << options[i].logicalLane << ']';
    }
    out << "],\"feasible\":" << (feasible?"true":"false") << ",\"selected\":";
    if (feasible) out << '[' << selected.connectionIndex << ',' << selected.entryLane << ',' << selected.logicalLane << ']';
    else out << "null";

    // Separately check reserve->feature ordering through the production API.
    // This is deliberately not a dispatch replay or a P/E prediction call.
    if (feasible) {
        g.releaseLaneOccupancy(0);
        g.vehicles[0].roadIndex = 1; g.vehicles[0].currentRoadID = 1;
        g.reserveLaneOccupancy(0,1,selected.logicalLane);
        const auto features = g.buildBasicRoadModelFeatures(1,0,0,120.0,selected.logicalLane);
        require(features.road_flow == accumulate(f.flow.begin(),f.flow.end(),0)+1,
                f.name+": road flow must include ego once");
        require(features.lane_flow == f.flow[selected.logicalLane]+1,
                f.name+": selected lane flow must include ego once");
        require(features.laneIndex == selected.logicalLane && features.road_length == 100.0,
                f.name+": physical length and final lane feature");
        vector<int> expected = f.flow; ++expected[selected.logicalLane];
        require(g.roads[1].laneFlow == expected, f.name+": real entry not separately reserved");
        out << ",\"post_reserve_lane_flow\":"; array_json(out,g.roads[1].laneFlow);
        out << ",\"post_reserve_road_flow\":" << features.road_flow
            << ",\"feature_lane_flow\":" << features.lane_flow
            << ",\"feature_road_length\":" << features.road_length;
    }
    out << '}';
    return out.str();
}
vector<Fixture> fixtures() {
    vector<Fixture> cases;
    Fixture f;
    f.name="expand_nonempty_intersection"; f.flow={2,0}; f.occupied={15,0}; f.expected_logical=1; cases.push_back(f);
    f.name="legacy_nonempty_intersection"; f.enabled=false; f.expected_logical=0; cases.push_back(f);
    f=Fixture(); f.name="entry_full_target_free"; f.outgoing={{1,0}};
    f.flow={1,0}; f.occupied={7.5,0}; f.capacity={1,14}; f.expected_feasible=false; cases.push_back(f);
    f.name="legacy_projection_bypasses_entry"; f.enabled=false; f.expected_feasible=true; f.expected_logical=1; cases.push_back(f);
    f=Fixture(); f.name="target_full_entry_free"; f.outgoing={{1,0}};
    f.flow={0,1}; f.occupied={0,7.5}; f.capacity={14,1}; f.expected_feasible=false; cases.push_back(f);
    f=Fixture(); f.name="alternative_real_entry"; f.receiving_lanes=3; f.incoming={{0,0},{0,2}};
    f.outgoing={{1,0}}; f.flow={1,0,0}; f.occupied={7.5,0,0}; f.capacity={1,14,14};
    f.expected_entry=2; f.expected_logical=1; f.expected_connection=1; cases.push_back(f);
    f=Fixture(); f.name="cannot_borrow_another_source_lane"; f.source_lanes=2; f.incoming={{0,0},{1,1}};
    f.flow={1,0}; f.occupied={7.5,0}; f.capacity={1,14}; f.expected_feasible=false; cases.push_back(f);
    f=Fixture(); f.name="length_before_lateral_distance"; f.receiving_lanes=3; f.incoming={{0,1},{0,1}};
    f.outgoing={{0,0},{1,0},{2,0}}; f.flow={1,1,1}; f.occupied={8,8,6};
    f.expected_entry=1; f.expected_logical=2; cases.push_back(f);
    f.name="lateral_distance_before_lane_id"; f.occupied={8,8,8}; f.expected_logical=1; cases.push_back(f);
    f.name="lane_id_before_connection_id"; f.incoming={{0,1}}; f.outgoing={{0,0},{2,0}};
    f.expected_logical=0; cases.push_back(f);
    f=Fixture(); f.name="duplicate_connection_stable_tie"; f.incoming={{0,1},{0,1}};
    f.outgoing={{1,0}}; f.expected_entry=1; f.expected_logical=1; cases.push_back(f);
    f=Fixture(); f.name="connection_vehicle_class_permissions"; f.receiving_lanes=3; f.incoming={{0,0},{0,1}};
    f.outgoing={{0,0},{1,0},{2,0}}; f.flow={0,0,0}; f.occupied={0,0,0};
    f.bus_only_incoming={0}; f.bus_only_outgoing={1,2};
    f.expected_entry=1; f.expected_logical=0; f.expected_connection=1; cases.push_back(f);
    f=Fixture(); f.name="receiving_lane_vehicle_class_permission"; f.incoming={{0,0},{0,1}};
    f.bus_only_receiving={0}; f.expected_entry=1; f.expected_logical=1; f.expected_connection=1; cases.push_back(f);
    f=Fixture(); f.name="source_lane_vehicle_class_permission"; f.bus_only_source=true;
    f.expected_feasible=false; cases.push_back(f);
    f=Fixture(); f.name="missing_internal_path_fails_closed"; f.missing_internal_path=true;
    f.expected_feasible=false; cases.push_back(f);
    f=Fixture(); f.name="final_road_keeps_actual_entry"; f.final_road=true; f.incoming={{0,1}}; f.outgoing.clear();
    f.flow={0,1}; f.occupied={0,7.5}; f.expected_entry=1; f.expected_logical=1; cases.push_back(f);
    f=Fixture(); f.name="same_entry_target_one_slot"; f.flow={0,1}; f.occupied={0,7.5}; f.capacity={1,1}; cases.push_back(f);
    f=Fixture(); f.name="count_capacity_not_length_capacity"; f.flow={1,1}; f.occupied={500,600}; cases.push_back(f);
    return cases;
}
}
int main() {
    try {
        vector<string> records;
        for (const auto& f : fixtures()) records.push_back(run_case(f));
        cout << "{\"schema\":\"traci-roadpair-production-probe-v1\",\"checks\":" << checks << ",\"cases\":[";
        for (size_t i=0; i<records.size(); ++i) { if (i) cout << ','; cout << records[i]; }
        cout << "]}\n";
    } catch (const exception& e) {
        cerr << "TraciRoadPairContractProbe failed: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
