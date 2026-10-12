#include "head.h"
#include <chrono>
#include <cstdio>

namespace {
int checks = 0;
void check(bool condition, const string& reason) {
    ++checks;
    if (!condition) throw runtime_error(reason);
}
void rejects(const std::function<void()>& action, const string& reason) {
    ++checks;
    try { action(); } catch (const std::exception&) { return; }
    throw runtime_error("expected rejection: " + reason);
}
Graph graph_fixture() {
    Graph g;
    g.roads.resize(3);
    for (int i = 0; i < 3; ++i) {
        g.roads[i].laneNum = 1;
        g.roadIDToSumoEdgeStr[i] = string(1, static_cast<char>('a' + i));
        g.sumoEdgeStrToRoadID[g.roadIDToSumoEdgeStr[i]] = i;
    }
    SumoConnectionRaw ab;
    ab.fromEdge = "a"; ab.toEdge = "b"; ab.fromLane = ab.toLane = 0;
    ab.tl = "T"; ab.linkIndex = 0;
    g.sumoConnectionsRaw.push_back(ab);
    SumoConnectionRaw bc = ab;
    bc.fromEdge = "b"; bc.toEdge = "c"; bc.linkIndex = 1;
    g.sumoConnectionsRaw.push_back(bc);
    SumoSignalProgram p;
    p.tlID = "T"; p.programID = "0"; p.type = "static";
    const double durations[] = {24, 3, 15, 3, 24, 3, 15, 3};
    const string states[] = {"Gr", "yr", "rG", "ry", "rr", "rr", "rr", "rr"};
    for (int i = 0; i < 8; ++i) {
        SumoSignalPhase phase;
        phase.duration = durations[i]; phase.state = states[i];
        phase.startTime = p.cycleLength; p.cycleLength += durations[i]; phase.endTime = p.cycleLength;
        p.phases.push_back(phase);
    }
    g.sumoSignalPrograms.push_back(p);
    return g;
}
void expect(const SignalRGFeatures& v, char a, double b, char c, double d) {
    check(v.entry_type == a && v.next_type == c, "wrong outgoing colors");
    check(std::abs(v.entry_remaining_seconds - b) < 1e-10 &&
          std::abs(v.next_duration_seconds - d) < 1e-10, "wrong outgoing durations");
}

struct TemporaryNet {
    string path;
    explicit TemporaryNet(const string& xml) {
        static unsigned serial = 0;
        path = ".signal_rg_graph_" + std::to_string(
            std::chrono::high_resolution_clock::now().time_since_epoch().count())
            + "_" + std::to_string(++serial) + ".net.xml";
        ofstream out(path);
        if (!out) throw runtime_error("cannot create owned signal test fixture");
        out << xml;
        if (!out) throw runtime_error("cannot write owned signal test fixture");
    }
    ~TemporaryNet() { std::remove(path.c_str()); }
};

string tiny_net(const string& offset = "offset=\"0\"", const string& duration = "duration=\"24\"",
                const string& from_lane = "fromLane=\"0\"", const string& to_lane = "toLane=\"0\"",
                const string& link = "linkIndex=\"0\"", const string& extra = "") {
    return "<net>"
        "<edge id=\":T_0\" function=\"internal\"><lane id=\":T_0_0\" index=\"0\" length=\"3\" speed=\"10\"/></edge>"
        "<edge id=\"a\" from=\"A\" to=\"T\"><lane id=\"a_0\" index=\"0\" length=\"100\" speed=\"10\"/></edge>"
        "<edge id=\"b\" from=\"T\" to=\"B\"><lane id=\"b_0\" index=\"0\" length=\"100\" speed=\"10\"/></edge>"
        "<edge id=\"c\" from=\"B\" to=\"C\"><lane id=\"c_0\" index=\"0\" length=\"100\" speed=\"10\"/></edge>"
        "<junction id=\"A\" type=\"dead_end\"/><junction id=\"T\" type=\"traffic_light\"/>"
        "<junction id=\"B\" type=\"priority\"/><junction id=\"C\" type=\"dead_end\"/>"
        "<tlLogic id=\"T\" type=\"static\" programID=\"0\" " + offset + ">"
        "<phase " + duration + " state=\"G\"/><phase duration=\"3\" state=\"y\"/>"
        "<phase duration=\"63\" state=\"r\"/></tlLogic>"
        "<connection from=\"a\" to=\"b\" " + from_lane + " " + to_lane
        + " tl=\"T\" via=\":T_0_0\" " + link + " dir=\"s\"/>"
        "<connection from=\"b\" to=\"c\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>" + extra + "</net>";
}

Graph read_tiny(const string& xml, bool rg = true, bool old_abc = false) {
    TemporaryNet fixture(xml);
    Graph g;
    if (rg) {
        g.travelTimeMode = TravelTimeMode::TABLE;
        g.travelTimeTableFormat = TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_RLE_V1;
    } else if (old_abc) {
        g.travelTimeMode = TravelTimeMode::TABLE;
        g.travelTimeTableFormat = TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RLE_V1;
    } else {
        g.travelTimeMode = TravelTimeMode::SPEED_NET;
    }
    g.read_sumo_net_xml(fixture.path);
    return g;
}

void parser_checks() {
    auto normal = read_tiny(tiny_net());
    expect(normal.outgoingSignalRG(0, 1, 0, 28), 'R', 62, 'G', 24);
    check(normal.sumoConnectionsRaw[1].linkIndex == -1, "no-TLS missing linkIndex remains legal");
    auto absent_offset = read_tiny(tiny_net(""));
    check(absent_offset.sumoSignalPrograms[0].offset == 0, "omitted offset has defined zero default");
    auto fractional = read_tiny(tiny_net("offset=\" -1.25 \"", "duration=\" 24.5 \""));
    check(fractional.sumoSignalPrograms[0].offset == -1.25, "valid finite negative offset retained");
    check(fractional.sumoSignalPrograms[0].phases[0].duration == 24.5, "fractional duration retained");
    for (const string& value : {"bad", "40junk", "", "nan", "inf", "1e999"}) {
        rejects([&] { read_tiny(tiny_net("offset=\"" + value + "\"")); }, "malformed/nonfinite offset " + value);
    }
    for (const string& value : {"24junk", "bad", "", "0", "-1", "nan", "inf", "1e999"}) {
        rejects([&] { read_tiny(tiny_net("offset=\"0\"", "duration=\"" + value + "\"")); }, "invalid duration " + value);
    }
    rejects([&] { read_tiny(tiny_net("offset=\"0\"", "")); }, "missing required duration");
    for (const string& name : {"fromLane", "toLane", "linkIndex"}) {
        for (const string& value : {"0junk", "bad", "", "-1", "0.5", "2147483648"}) {
            const string attribute = name + "=\"" + value + "\"";
            rejects([&] { read_tiny(tiny_net("offset=\"0\"", "duration=\"24\"",
                        name == "fromLane" ? attribute : "fromLane=\"0\"",
                        name == "toLane" ? attribute : "toLane=\"0\"",
                        name == "linkIndex" ? attribute : "linkIndex=\"0\"")); }, "invalid " + attribute);
        }
    }
    rejects([&] { read_tiny(tiny_net("offset=\"0\"", "duration=\"24\"", "")); }, "missing fromLane");
    rejects([&] { read_tiny(tiny_net("offset=\"0\"", "duration=\"24\"", "fromLane=\"0\"", "")); }, "missing toLane");
    rejects([&] { read_tiny(tiny_net("offset=\"0\"", "duration=\"24\"", "fromLane=\"0\"", "toLane=\"0\"", "")); }, "missing controlled linkIndex");
    for (const string& tag : {"WAUT", "wautJunction", "wautSwitch"}) {
        const string xml = tiny_net("offset=\"0\"", "duration=\"24\"", "fromLane=\"0\"", "toLane=\"0\"",
                                    "linkIndex=\"0\"", "<" + tag + "/>");
        rejects([&] { read_tiny(xml); }, "RG net globally rejects " + tag);
        auto legacy = read_tiny(xml, false);
        check(legacy.sumoHasSignalProgramSwitching, "legacy load records WAUT without changed dispatch rules");
    }
    auto legacy = read_tiny(tiny_net("offset=\"bad\"", "duration=\"24junk\"",
                                    "fromLane=\"0junk\"", "toLane=\"0junk\"", "linkIndex=\"0junk\""), false);
    check(legacy.sumoSignalPrograms[0].offset == 0, "legacy malformed offset fallback unchanged");
    check(legacy.sumoSignalPrograms[0].phases[0].duration == 24, "legacy numeric-prefix behavior unchanged");
    check(legacy.sumoConnectionsRaw[0].fromLane == 0 && legacy.sumoConnectionsRaw[0].toLane == 0
          && legacy.sumoConnectionsRaw[0].linkIndex == 0, "legacy integer-prefix behavior unchanged");
    auto old_abc = read_tiny(tiny_net("offset=\"bad\"", "duration=\"24junk\"",
                                    "fromLane=\"0junk\"", "toLane=\"0junk\"", "linkIndex=\"0junk\""), false, true);
    check(old_abc.sumoSignalPrograms[0].offset == 0
          && old_abc.sumoSignalPrograms[0].phases[0].duration == 24,
          "old ABC parser remains unchanged, not only speed-net");
}

void always_open_runtime_checks() {
    const string xml = "<net>"
        "<edge id=\"a\" from=\"A\" to=\"U\"><lane id=\"a_0\" index=\"0\" length=\"40\" speed=\"10\"/></edge>"
        "<edge id=\"b\" from=\"B\" to=\"U\"><lane id=\"b_0\" index=\"0\" length=\"40\" speed=\"10\"/></edge>"
        "<edge id=\"c\" from=\"U\" to=\"V\"><lane id=\"c_0\" index=\"0\" length=\"4\" speed=\"1\"/></edge>"
        "<edge id=\"d\" from=\"V\" to=\"Z\"><lane id=\"d_0\" index=\"0\" length=\"30\" speed=\"10\"/></edge>"
        "<junction id=\"A\" type=\"dead_end\"/><junction id=\"B\" type=\"dead_end\"/>"
        "<junction id=\"U\" type=\"priority\"/><junction id=\"V\" type=\"right_before_left\"/>"
        "<junction id=\"Z\" type=\"dead_end\"/>"
        "<connection from=\"a\" to=\"c\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>"
        "<connection from=\"b\" to=\"c\" fromLane=\"0\" toLane=\"0\" dir=\"r\"/>"
        "<connection from=\"c\" to=\"d\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/></net>";
    auto g = read_tiny(xml, false);
    g.build_new_graph_structures_from_sumo();
    check(g.intersections.size() == 2 && g.waitingBuffers.size() == 3,
          "all uncontrolled legal movements receive dispatch groups and buffers");
    for (const auto& m : g.movements) {
        check(m.intersectionID >= 0 && m.alwaysOpen, "uncontrolled movement group is complete");
        check(!g.roads[m.fromRoadID].hasSignalizedDownstream, "do not falsify imported physical signal flag");
        check(g.isMovementActive(m.movementID, 12.125), "uncontrolled movement stays active");
        check(g.nextGreenTimeForMovement(m.movementID, 12.125) == 12.125, "no synthetic green boundary");
        check(g.nextSignalChangeTime(m.signalID, 12.125) == INF, "no uncontrolled switch event");
        check(g.isUnsignalizedMovement(m.movementID) && g.movementCrossingTime(m.movementID) == 0.0,
              "known uncontrolled junction has zero crossing, not zero service headway");
    }
    g.initialize_signal_event_queue(0.0);
    check(g.signalEventPQ.empty(), "all-uncontrolled graph has no signal events");
    rejects([&] { g.unsignalizedGreenHorizon(); }, "no finite H available for RG features");
    // No TLS/H needed by the speed-net runtime. A one-vehicle middle road forces
    // downstream deactivation and release-driven reactivation at a merge.
    g.defaultDischargeInterval = 2.15;
    g.evalTrajectoryDetails = true;
    vector<vector<int>> routes, queries;
    for (int i = 0; i < 12; ++i) {
        routes.push_back({i % 2, 2, 3});
        queries.push_back({g.roads[i % 2].fromNode, g.roads[3].toNode, 0});
    }
    g.vehicleDepartureTimes.assign(routes.size(), 0.25);
    g.vehicleDeparturePositions.assign(routes.size(), 0.0);
    auto eta = g.cycle_aware_signal_driven_records(queries, routes);
    check(g.invalidVehicleCount == 0 && g.finishedVehicleCount == 12, "uncontrolled merge routes finish valid");
    check(g.roads[2].laneCapacity[0] == 1, "test actually exercises a one-car choke road");
    check(g.signalEventPQ.empty(), "runtime did not invent signal events");
    check(!std::isfinite(g.unsignalizedGreenHorizonCache), "speed-net never requested/guessed H");
    bool reactivated = false;
    for (const auto& trace : g.downstreamReleaseCompetitionTimeline)
        reactivated = reactivated || (trace.wasBlockedByDownstream && trace.reactivated);
    check(reactivated, "downstream storage release reactivates uncontrolled blocked movement");
    vector<vector<double>> served(g.movements.size());
    for (size_t i = 0; i < routes.size(); ++i) {
        check(eta[i].size() == 2 && eta[i].back().first == -1, "departure and final exit recorded");
        check(g.vehicleRoadTimeline[i].size() == 3, "all three route occurrences recorded in road timeline");
        check(g.vehicleMovementTimeline[i].size() == 2, "both uncontrolled movements discharged");
        check(g.vehicleRoadTimeline[i][0].intersectionCrossingTime == 0.0,
              "initial departure has no incoming crossing");
        for (size_t road = 1; road < g.vehicleRoadTimeline[i].size(); ++road) {
            const auto& entry = g.vehicleRoadTimeline[i][road];
            const auto& discharge = g.vehicleMovementTimeline[i][road - 1];
            check(entry.intersectionCrossingTime == 0.0,
                  "middle and final road timelines report zero unsignalized crossing");
            check(std::abs(entry.enterTime - discharge.dischargeTime) < TIME_EPSILON,
                  "unsignalized entry and upstream discharge have the same timestamp");
        }
        for (const auto& event : g.vehicleMovementTimeline[i]) served[event.movementID].push_back(event.dischargeTime);
    }
    for (auto& times : served) {
        sort(times.begin(), times.end());
        for (size_t i = 1; i < times.size(); ++i)
            check(times[i] - times[i - 1] >= 2.15 - TIME_EPSILON, "uncontrolled service headway retained across windows");
    }
    for (const auto& road : g.roads) check(road.roadFlow == 0 && road.laneFlow[0] == 0, "final exits conserve occupancy");
    check(g.validateRoadLaneFlows(), "all-uncontrolled completed flow balance");

    // Existing TLS keeps ID zero and its calendar; the unsignalized successor
    // appends a group instead of perturbing original controlled group IDs.
    auto mixed = read_tiny(tiny_net(), false);
    mixed.build_new_graph_structures_from_sumo();
    check(mixed.movements[0].intersectionID == 0 && !mixed.movements[0].alwaysOpen,
          "existing controlled group identity unchanged");
    check(mixed.movements[1].intersectionID == 1 && mixed.movements[1].alwaysOpen,
          "uncontrolled successor gets appended dispatch group");
    mixed.initialize_signal_event_queue(0);
    check(mixed.signalEventPQ.size() == 1, "only real controlled movement schedules phase events");
    check(mixed.signalStateAtMovement(0, 0) == SignalState::Green &&
          mixed.signalStateAtMovement(0, 28) == SignalState::Red, "real TLS calendar unchanged");
    check(mixed.unsignalizedGreenHorizon() == 24, "mixed net H is observed finite green length");
    expect(mixed.outgoingSignalRG(1, 2, 0, 900), 'G', 24, 'G', 24);

    mixed.intersectionCrossingTime = 3.4;
    mixed.defaultDischargeInterval = 2.15;
    mixed.evalTrajectoryDetails = true;
    check(mixed.movementCrossingTime(0) == 3.4 && mixed.movementCrossingTime(1) == 0.0,
          "controlled and uncontrolled crossings share one physical-node selector");
    vector<vector<int>> mixedRoutes{{0, 1, 2}};
    vector<vector<int>> mixedQueries{{mixed.roads[0].fromNode, mixed.roads[2].toNode, 0}};
    mixed.vehicleDepartureTimes = {0.25};
    mixed.vehicleDeparturePositions = {0.0};
    const auto mixedEta = mixed.cycle_aware_signal_driven_records(mixedQueries, mixedRoutes);
    check(mixed.finishedVehicleCount == 1 && mixed.invalidVehicleCount == 0,
          "mixed controlled/uncontrolled route finishes valid");
    check(mixed.vehicleRoadTimeline[0].size() == 3 && mixed.vehicleMovementTimeline[0].size() == 2,
          "mixed crossing test records every road and discharge");
    const auto& controlledEntry = mixed.vehicleRoadTimeline[0][1];
    const auto& finalEntry = mixed.vehicleRoadTimeline[0][2];
    check(controlledEntry.intersectionCrossingTime == 3.4 &&
          std::abs(controlledEntry.enterTime - mixed.vehicleMovementTimeline[0][0].dischargeTime - 3.4) < TIME_EPSILON,
          "controlled crossing configuration is retained in entry time and timeline");
    check(finalEntry.intersectionCrossingTime == 0.0 &&
          finalEntry.enterTime == mixed.vehicleMovementTimeline[0][1].dischargeTime,
          "final road uses incoming zero crossing, independently of its no-exit sentinel");
    check(mixedEta[0].back().first == -1 &&
          std::abs(mixedEta[0].back().second - finalEntry.arrivalAtRoadEndTime) < TIME_EPSILON,
          "mixed route still runs and completes its entire final road");
}

void physical_crossing_identity_checks() {
    auto graph = read_tiny(tiny_net(), false);
    graph.build_new_graph_structures_from_sumo();
    graph.intersectionCrossingTime = 3.4;
    graph.movements[0].alwaysOpen = true;
    graph.signals[graph.movements[0].signalID].alwaysOpen = true;
    check(!graph.isUnsignalizedMovement(0) && graph.movementCrossingTime(0) == 3.4,
          "a TLS movement does not become zero crossing merely because it is always open");
    graph.movements[0].tlID.clear();
    check(graph.movementCrossingTime(0) == 3.4,
          "uncontrolled bypass at a signalized node retains crossing");
    graph.roads[graph.movements[0].fromRoadID].hasSignalizedDownstream = false;
    check(graph.movementCrossingTime(0) == 3.4,
          "physical signalized node is sufficient even without the redundant road flag");
    graph.movements[1].tlID = "EXPLICIT_TLS";
    check(graph.movementCrossingTime(1) == 3.4,
          "explicit TLS binding is sufficient even at a non-signalized node type");
    graph.movements[1].tlID.clear();
    const int fromRoad = graph.movements[1].fromRoadID;
    graph.roads[fromRoad].hasSignalizedDownstream = true;
    check(graph.movementCrossingTime(1) == 3.4, "legacy road signal flag conservatively retains crossing");
    graph.roads[fromRoad].hasSignalizedDownstream = false;
    graph.nodeIDToSumoNodeStr.clear();
    check(graph.movementCrossingTime(1) == 3.4,
          "legacy-only or unknown physical node is not silently classified as zero crossing");
    graph.movements[1].fromRoadID = -1;
    check(graph.movementCrossingTime(1) == 3.4, "missing incoming geometry retains configured crossing");
    rejects([&] { graph.movementCrossingTime(-1); }, "first road must explicitly use zero, not an invalid movement");

    const string original = "<junction id=\"B\" type=\"priority\"/>";
    for (const string& type : {"priority", "priority_stop", "right_before_left", "allway_stop",
                               "zipper", "unregulated", "dead_end", "district", "", "custom",
                               "traffic_light_unregulated", "traffic_light_right_on_red"}) {
        string xml = tiny_net();
        xml.replace(xml.find(original), original.size(), "<junction id=\"B\" type=\"" + type + "\"/>");
        auto typed = read_tiny(xml, false);
        typed.build_new_graph_structures_from_sumo();
        typed.intersectionCrossingTime = 3.4;
        const bool unsupported = type.empty() || type == "custom" || type.find("traffic_light") == 0;
        check(typed.nodes[typed.roads[1].toNode].sumoJunctionType == type,
              "raw SUMO junction type remains available after coarse classification");
        check(typed.isUnsignalizedMovement(1) == !unsupported &&
              typed.movementCrossingTime(1) == (unsupported ? 3.4 : 0.0),
              "only explicit supported nonsignal types receive zero crossing: " + type);
    }
    string absentType = tiny_net();
    absentType.replace(absentType.find(original), original.size(), "<junction id=\"B\"/>");
    auto missingType = read_tiny(absentType, false);
    missingType.build_new_graph_structures_from_sumo();
    check(!missingType.isUnsignalizedMovement(1) && missingType.movementCrossingTime(1) == 3.0,
          "missing SUMO junction type retains configured crossing rather than guessing zero");
}

void run() {
    auto g = graph_fixture();
    string tls; int link = -1;
    expect(g.outgoingSignalRG(0, 1, 0, 28, &tls, &link), 'R', 62, 'G', 24);
    check(tls == "T" && link == 0, "signal provenance");
    expect(g.outgoingSignalRG(0, 1, 0, 25), 'R', 63, 'G', 24);
    expect(g.outgoingSignalRG(1, 2, 0, 28), 'G', 14, 'R', 72);
    check(g.outgoingSignalRGCache.size() == 2, "mapping cached once");
    expect(g.outgoingSignalRG(2, -1, 0, 28), 'G', 0, 'G', 0);
    rejects([&] { g.outgoingSignalRG(0, 2, 0, 28); }, "missing route connection");
    rejects([&] { g.outgoingSignalRG(0, 1, 2, 28); }, "invalid logical lane");
    rejects([&] { g.outgoingSignalRG(0, 1, 0, NAN); }, "invalid time");
    auto offset = graph_fixture(); offset.sumoSignalPrograms[0].offset = 40;
    expect(offset.outgoingSignalRG(0, 1, 0, 68), 'R', 62, 'G', 24);
    auto unsignalized = graph_fixture(); unsignalized.sumoConnectionsRaw[0].tl.clear();
    // Only b->c/link1 remains an actual controlled external connection: H=15,
    // not 24 from the now unused link0 and not a time-dependent countdown.
    expect(unsignalized.outgoingSignalRG(0, 1, 0, 28), 'G', 15, 'G', 15);
    expect(unsignalized.outgoingSignalRG(0, 1, 0, 104.75), 'G', 15, 'G', 15);
    check(unsignalized.unsignalizedGreenHorizon() == 15, "only actual active links contribute H");
    expect(unsignalized.outgoingSignalRG(2, -1, 0, 28), 'G', 0, 'G', 0);
    auto nofinite = graph_fixture();
    for (auto& c : nofinite.sumoConnectionsRaw) c.tl.clear();
    rejects([&] { nofinite.outgoingSignalRG(0, 1, 0, 28); }, "no active finite horizon must fail");
    expect(nofinite.outgoingSignalRG(2, -1, 0, 28), 'G', 0, 'G', 0);
    auto unused = graph_fixture();
    auto unusedProgram = unused.sumoSignalPrograms.front();
    unusedProgram.tlID = "UNUSED"; unusedProgram.type = "actuated";
    unused.sumoSignalPrograms.push_back(unusedProgram);
    check(unused.unsignalizedGreenHorizon() == 24, "unconnected program does not affect H");
    auto constant = graph_fixture();
    for (auto& phase : constant.sumoSignalPrograms.front().phases) phase.state = "GG";
    rejects([&] { constant.unsignalizedGreenHorizon(); }, "all-green active network has no finite H");
    rejects([&] { constant.outgoingSignalRG(0, 1, 0, 0); }, "controlled constant program not newly supported");
    auto wrapped = graph_fixture();
    auto& wp = wrapped.sumoSignalPrograms.front(); wp.phases.clear();
    const double hd[] = {2, 3, 4, 5}; const string hs[] = {"Gr", "gr", "rr", "Gr"};
    for (int i = 0; i < 4; ++i) { SumoSignalPhase phase; phase.duration = hd[i]; phase.state = hs[i]; wp.phases.push_back(phase); }
    check(wrapped.unsignalizedGreenHorizon() == 10, "H merges cross-phase and cross-cycle green");
    auto identical = graph_fixture(); identical.sumoConnectionsRaw.push_back(identical.sumoConnectionsRaw[0]);
    expect(identical.outgoingSignalRG(0, 1, 0, 28), 'R', 62, 'G', 24);
    auto ambiguous = graph_fixture(); auto other = ambiguous.sumoConnectionsRaw[0]; other.linkIndex = 1;
    ambiguous.sumoConnectionsRaw.push_back(other);
    rejects([&] { ambiguous.outgoingSignalRG(0, 1, 0, 28); }, "different full calendars");
    auto controller = graph_fixture(); other = controller.sumoConnectionsRaw[0]; other.tl.clear();
    controller.sumoConnectionsRaw.push_back(other);
    rejects([&] { controller.outgoingSignalRG(0, 1, 0, 28); }, "mixed controlled/uncontrolled");
    auto missing = graph_fixture(); missing.sumoSignalPrograms.clear();
    rejects([&] { missing.outgoingSignalRG(0, 1, 0, 28); }, "missing program");
    auto multiple = graph_fixture(); multiple.sumoSignalPrograms.push_back(multiple.sumoSignalPrograms[0]);
    rejects([&] { multiple.outgoingSignalRG(0, 1, 0, 28); }, "ambiguous active program");
    rejects([&] { multiple.unsignalizedGreenHorizon(); }, "H rejects multiple active programs");
    auto dynamic = graph_fixture(); dynamic.sumoSignalPrograms[0].type = "actuated";
    rejects([&] { dynamic.outgoingSignalRG(0, 1, 0, 28); }, "actuated");
    rejects([&] { dynamic.unsignalizedGreenHorizon(); }, "H rejects actuated programs");
    auto waut = graph_fixture(); waut.sumoHasSignalProgramSwitching = true;
    rejects([&] { waut.outgoingSignalRG(0, 1, 0, 28); }, "WAUT");
    rejects([&] { waut.outgoingSignalRG(2, -1, 0, 28); }, "WAUT cannot bypass via final-road sentinel");
    auto branching = graph_fixture(); branching.sumoSignalPrograms[0].phases[0].nextPhaseIndices = "1 2";
    rejects([&] { branching.outgoingSignalRG(0, 1, 0, 28); }, "branching");
    auto nonsequence = graph_fixture(); nonsequence.sumoSignalPrograms[0].phases[0].nextPhaseIndices = "2";
    rejects([&] { nonsequence.outgoingSignalRG(0, 1, 0, 28); }, "nonsequential");
    auto validnext = graph_fixture(); validnext.sumoSignalPrograms[0].phases[0].nextPhaseIndices = "1";
    expect(validnext.outgoingSignalRG(0, 1, 0, 28), 'R', 62, 'G', 24);
    parser_checks();
    always_open_runtime_checks();
    physical_crossing_identity_checks();
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 3 && string(argv[1]) == "--network-horizon") {
            Graph g;
            g.travelTimeMode = TravelTimeMode::TABLE;
            g.travelTimeTableFormat = TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_RLE_V1;
            g.read_sumo_net_xml(argv[2]);
            std::cout << "NETWORK_GREEN_HORIZON=" << setprecision(17) << g.unsignalizedGreenHorizon() << '\n';
            return 0;
        }
        run(); std::cout << "SignalRGGraphTest " << checks << " checks passed\n"; return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
