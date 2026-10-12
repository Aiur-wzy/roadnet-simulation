#include "head.h"
#include <chrono>
#include <cstdio>

namespace {
int checks = 0;
void check(bool condition, const string& message) {
    ++checks;
    if (!condition) throw runtime_error(message);
}
void rejects(const function<void()>& action, const string& message) {
    bool failed = false;
    try { action(); } catch (const exception&) { failed = true; }
    check(failed, message);
}
SumoSignalProgram program(const string& tls, const vector<pair<double, string>>& phases,
                          double offset = 0.0) {
    SumoSignalProgram result;
    result.tlID = tls;
    result.type = "static";
    result.programID = "0";
    result.offset = offset;
    for (const auto& item : phases) {
        SumoSignalPhase phase;
        phase.duration = item.first;
        phase.state = item.second;
        result.phases.push_back(phase);
        result.cycleLength += phase.duration;
    }
    return result;
}
SumoConnectionRaw connection(const string& from, const string& to, int fromLane,
                             int toLane, const string& tls = "", int link = -1) {
    SumoConnectionRaw result;
    result.fromEdge = from;
    result.toEdge = to;
    result.fromLane = fromLane;
    result.toLane = toLane;
    result.tl = tls;
    result.linkIndex = link;
    result.dir = TurnDir::Straight;
    return result;
}
Graph empty_graph() {
    Graph graph;
    graph.roads.resize(4);
    for (int i = 0; i < 4; ++i) {
        const string id(1, static_cast<char>('A' + i));
        graph.sumoEdgeStrToRoadID[id] = i;
        graph.roadIDToSumoEdgeStr[i] = id;
        graph.roads[i].roadID = i;
        graph.roads[i].toNode = -1;
        graph.roads[i].laneNum = 5;
    }
    graph.sumoNetworkWithoutInternalLinks = true;
    return graph;
}
Graph parallel_graph() {
    auto graph = empty_graph();
    graph.sumoSignalPrograms.push_back(program("T", {{10, "GgGGGr"}, {3, "yyyyyy"}, {7, "rrrrrG"}}));
    for (int lane = 0; lane < 5; ++lane) {
        auto item = connection("A", "B", lane, lane, "T", lane);
        item.state = lane == 1 ? 'g' : 'G';
        item.allow = "passenger taxi";
        item.disallow = "bus";
        graph.sumoConnectionsRaw.push_back(item);
    }
    graph.sumoConnectionsRaw.push_back(connection("A", "C", 0, 0, "T", 5));
    graph.sumoConnectionsRaw.push_back(connection("D", "B", 0, 0));
    graph.sumoConnectionsRaw.push_back(connection("B", "A", 0, 0));
    return graph;
}
void pair_and_route_checks() {
    auto legacy = parallel_graph();
    check(!legacy.mergeRoadPairMovements, "legacy grouping remains the default");
    legacy.build_movements_from_sumo_connections();
    check(legacy.movements.size() == 8, "legacy retains five distinct AB links");
    check(legacy.fromToRoadToMovementIDs.at({0, 1}).size() == 5, "legacy road-pair multiplicity retained");
    for (const auto& movement : legacy.movements)
        check(movement.connectionIndices.size() == 1, "legacy movements retain raw index too");

    auto graph = parallel_graph();
    graph.mergeRoadPairMovements = true;
    graph.build_movements_from_sumo_connections();
    check(graph.movements.size() == 4, "five AB links merge without AC, DB or BA");
    const int ab = graph.roadPairToMovementID.at({0, 1});
    const int ac = graph.roadPairToMovementID.at({0, 2});
    const int db = graph.roadPairToMovementID.at({3, 1});
    const int ba = graph.roadPairToMovementID.at({1, 0});
    check(ab != ac && ab != db && ab != ba && ac != db && ac != ba && db != ba,
          "ordered road identities alone define merge boundaries");
    check(graph.fromToRoadToMovementIDs.at({0, 1}) == vector<int>{ab}, "one route mapping per road pair");
    const auto& movement = graph.movements[ab];
    check(movement.connectionIndices == vector<int>({0, 1, 2, 3, 4}), "raw connection order and indexes retained");
    check(movement.fromLanes.size() == 5 && movement.toLanes.size() == 5, "lane summaries retain all five lanes");
    check(movement.connectionIndices.size() == 5, "five original pairs never become 25 Cartesian connections");
    for (int index : movement.connectionIndices) {
        const auto& raw = graph.sumoConnectionsRaw[index];
        check(raw.fromLane == raw.toLane, "only original diagonal lane pair exists");
        check(raw.allow == "passenger taxi" && raw.disallow == "bus", "connection permissions unchanged");
        check(graph.movementKeyToID.at("0|1|T|" + to_string(index)) == ab, "former link key maps to merged movement");
    }
    check(graph.sumoConnectionsRaw[1].state == 'g' && graph.sumoSignalPrograms[0].phases[0].state[1] == 'g',
          "permissive g raw metadata is not rewritten");
    check(movement.connectionContinuousSpeedPathKnown == vector<bool>(5, true), "no-internal path flags align");
    for (double limit : movement.connectionContinuousSpeedPathLimit)
        check(std::isinf(limit), "no-internal individual paths have no internal speed restriction");
    graph.routeRoadID = {{0, 1, 0, 1}, {0, 2}, {3, 1}, {0}};
    graph.route_roadID_2_movementID();
    check(graph.routeMovementID[0] == vector<int>({ab, ba, ab}), "repeated road occurrences bind directly by pair");
    check(graph.routeMovementID[1] == vector<int>{ac} && graph.routeMovementID[2] == vector<int>{db},
          "same turn and different target/source route identities retained");
    check(graph.routeMovementID[3].empty(), "single-road route has no invented movement");
    graph.fromToRoadToMovementIDs[{0, 1}].push_back(ac);
    rejects([&] { graph.route_roadID_2_movementID(); }, "ambiguous road-pair route map fails instead of choosing first link");
}
Graph two_controller_graph() {
    auto graph = empty_graph();
    graph.mergeRoadPairMovements = true;
    graph.sumoConnectionsRaw = {connection("A", "B", 0, 0, "T", 0), connection("A", "B", 1, 1, "U", 0)};
    graph.sumoSignalPrograms = {program("T", {{10, "G"}, {10, "r"}}), program("U", {{10, "g"}, {10, "r"}})};
    return graph;
}
void calendar_checks() {
    auto same = two_controller_graph();
    same.build_movements_from_sumo_connections();
    check(same.movements.size() == 1, "different controllers merge only after complete calendar equivalence");
    auto shifted = two_controller_graph();
    shifted.sumoSignalPrograms[1].offset = 1;
    rejects([&] { shifted.build_movements_from_sumo_connections(); }, "different absolute offsets reject identical local states");
    shifted.mergeRoadPairMovements = false;
    shifted.build_movements_from_sumo_connections();
    check(shifted.movements.size() == 2, "legacy permits separately controlled movements");
    auto rotated = two_controller_graph();
    rotated.sumoSignalPrograms[1] = program("U", {{10, "r"}, {10, "g"}}, 10);
    rotated.build_movements_from_sumo_connections();
    check(rotated.movements.size() == 1, "rotated phases plus offset preserve identical absolute calendar");
    auto segmented = two_controller_graph();
    segmented.sumoSignalPrograms[1] = program("U", {{4, "g"}, {6, "G"}, {10, "r"}}, 20);
    segmented.build_movements_from_sumo_connections();
    check(segmented.movements.size() == 1, "phase segmentation and full-cycle offsets do not invent disagreement");
    auto different = two_controller_graph();
    different.sumoSignalPrograms[1] = program("U", {{10, "g"}, {3, "y"}, {7, "r"}});
    rejects([&] { different.build_movements_from_sumo_connections(); }, "same green window but differing yellow/red calendar rejects");
    auto sameTLS = parallel_graph();
    sameTLS.mergeRoadPairMovements = true;
    sameTLS.sumoSignalPrograms[0].phases[0].state[1] = 'r';
    rejects([&] { sameTLS.build_movements_from_sumo_connections(); }, "same-pair differing link calendar rejects");
    auto missing = two_controller_graph();
    missing.sumoSignalPrograms.pop_back();
    rejects([&] { missing.build_movements_from_sumo_connections(); }, "missing program rejects");
    auto duplicate = two_controller_graph();
    duplicate.sumoSignalPrograms.push_back(duplicate.sumoSignalPrograms[0]);
    rejects([&] { duplicate.build_movements_from_sumo_connections(); }, "ambiguous active TLS program rejects");
    auto actuated = two_controller_graph();
    actuated.sumoSignalPrograms[0].type = "actuated";
    rejects([&] { actuated.build_movements_from_sumo_connections(); }, "non-static signal program rejects");
    auto switching = two_controller_graph();
    switching.sumoHasSignalProgramSwitching = true;
    rejects([&] { switching.build_movements_from_sumo_connections(); }, "program switching rejects");
    auto badLink = two_controller_graph();
    badLink.sumoConnectionsRaw[1].linkIndex = 9;
    rejects([&] { badLink.build_movements_from_sumo_connections(); }, "invalid signal link rejects");
    auto branching = two_controller_graph();
    branching.sumoSignalPrograms[0].phases[0].nextPhaseIndices = "0 1";
    rejects([&] { branching.build_movements_from_sumo_connections(); }, "branching phase successor rejects");
    auto badLane = two_controller_graph();
    badLane.sumoConnectionsRaw[1].fromLane = 9;
    rejects([&] { badLane.build_movements_from_sumo_connections(); }, "invalid raw lane pair rejects");
}
struct XmlFixture {
    string path;
    explicit XmlFixture(const string& xml) {
        static int serial = 0;
        path = ".roadpair_graph_" + to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count())
            + "_" + to_string(++serial) + ".xml";
        ofstream output(path);
        if (!output) throw runtime_error("Cannot create road-pair XML fixture");
        output << xml;
    }
    ~XmlFixture() { std::remove(path.c_str()); }
};
string calendar_xml(const string& offset, const string& duration, const string& link) {
    return "<net>"
        "<edge id=\"A\" from=\"source\" to=\"T\"><lane id=\"A_0\" index=\"0\" speed=\"10\" length=\"100\"/></edge>"
        "<edge id=\"B\" from=\"T\" to=\"sink\"><lane id=\"B_0\" index=\"0\" speed=\"10\" length=\"100\"/></edge>"
        "<junction id=\"source\" type=\"dead_end\"/><junction id=\"T\" type=\"traffic_light\"/>"
        "<junction id=\"sink\" type=\"dead_end\"/>"
        "<tlLogic id=\"T\" type=\"static\" programID=\"0\" offset=\"" + offset +
        "\"><phase duration=\"" + duration + "\" state=\"G\"/></tlLogic>"
        "<connection from=\"A\" to=\"B\" fromLane=\"0\" toLane=\"0\" tl=\"T\" linkIndex=\"" + link + "\"/></net>";
}
void strict_calendar_parse_checks() {
    for (const auto& values : vector<vector<string>>{{"bad", "10", "0"}, {"0junk", "10", "0"},
            {"0", "10junk", "0"}, {"0", "10", "0x"}, {"nan", "10", "0"}}) {
        XmlFixture fixture(calendar_xml(values[0], values[1], values[2]));
        Graph graph;
        graph.mergeRoadPairMovements = true;
        rejects([&] { graph.read_sumo_net_xml(fixture.path); }, "road-pair signal metadata never silently accepts numeric prefixes/defaults");
    }
    XmlFixture fixture(calendar_xml("bad", "10junk", "0x"));
    Graph legacy;
    legacy.read_sumo_net_xml(fixture.path);
    check(legacy.sumoSignalPrograms[0].offset == 0 && legacy.sumoSignalPrograms[0].phases[0].duration == 10 &&
          legacy.sumoConnectionsRaw[0].linkIndex == 0, "legacy parser compatibility remains unchanged");
}
void connection_path_checks() {
    auto graph = empty_graph();
    graph.mergeRoadPairMovements = true;
    graph.sumoNetworkWithoutInternalLinks = false;
    graph.sumoConnectionsRaw = {connection("A", "B", 0, 0), connection("A", "B", 1, 1)};
    for (int i = 0; i < 2; ++i) {
        SumoEdgeRaw internal;
        internal.id = ":path" + to_string(i);
        internal.isInternal = true;
        SumoLaneRaw lane;
        lane.id = internal.id + "_0";
        lane.index = 0;
        lane.speed = i == 0 ? 8.0 : 4.0;
        internal.lanes.push_back(lane);
        graph.sumoEdgesRaw.push_back(internal);
        graph.sumoConnectionsRaw[i].via = lane.id;
        graph.sumoConnectionsRaw.push_back(connection(internal.id, "B", 0, i));
    }
    graph.build_movements_from_sumo_connections();
    check(graph.movements.size() == 1, "internal continuations do not become external movements");
    const auto& movement = graph.movements[0];
    check(movement.connectionIndices == vector<int>({0, 1}), "external raw indexes exclude internal continuations");
    check(movement.connectionContinuousSpeedPathKnown == vector<bool>({true, true}), "both complete original paths resolve");
    check(movement.connectionContinuousSpeedPathLimit == vector<double>({8.0, 4.0}), "individual path limits remain distinct");
    check(movement.continuousSpeedPathKnown && movement.continuousSpeedPathLimit == 4.0,
          "legacy aggregate remains conservative minimum");
    check(graph.sumoConnectionsRaw[0].via == ":path0_0" && graph.sumoConnectionsRaw[1].via == ":path1_0",
          "raw internal paths remain intact");
    graph.sumoConnectionsRaw.pop_back();
    graph.build_movements_from_sumo_connections();
    check(!graph.movements[0].continuousSpeedPathKnown, "incomplete option invalidates only legacy aggregate assurance");
    check(graph.movements[0].connectionContinuousSpeedPathKnown == vector<bool>({true, false}),
          "complete selected option remains usable despite another incomplete path");
}
}

int main() {
    try {
        pair_and_route_checks();
        calendar_checks();
        strict_calendar_parse_checks();
        connection_path_checks();
        cout << "Road-pair graph checks passed: " << checks << endl;
        return 0;
    } catch (const exception& error) {
        cerr << "Road-pair graph check failed: " << error.what() << endl;
        return 1;
    }
}
