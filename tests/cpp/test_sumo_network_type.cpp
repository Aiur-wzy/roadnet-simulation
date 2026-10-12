#include "head.h"
#include <chrono>
#include <cstdio>

namespace {
int checks = 0;
void check(bool value, const string& message) {
    ++checks;
    if (!value) throw runtime_error(message);
}
void near(double value, double expected, const string& message) {
    check(std::isfinite(value) && std::abs(value - expected) < 1e-10, message);
}
void rejects(const function<void()>& action, const string& message) {
    bool failed = false;
    try { action(); } catch (const exception&) { failed = true; }
    check(failed, message);
}
struct Fixture {
    string path;
    explicit Fixture(const string& content) {
        static int serial = 0;
        path = ".sumo_network_type_" + to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count())
             + "_" + to_string(++serial) + ".xml";
        ofstream output(path);
        if (!output) throw runtime_error("fixture creation failed");
        output << content;
    }
    ~Fixture() { std::remove(path.c_str()); }
};
string network(bool internal) {
    return string("<net><!-- <edge id=\":comment_only\" function=\"internal\"/> -->") +
        "<edge id=\"a\" from=\"A\" to=\"T\"><lane id=\"a_0\" length=\"100\" speed=\"20\"/></edge>"
        "<edge id=\"b\" from=\"T\" to=\"U\"><lane id=\"b_0\" length=\"80\" speed=\"20\"/></edge>"
        "<edge id=\"c\" from=\"U\" to=\"Z\"><lane id=\"c_0\" length=\"100\" speed=\"20\"/></edge>"
        "<junction id=\"A\" type=\"dead_end\"/><junction id=\"T\" type=\"traffic_light\"/>"
        "<junction id=\"U\" type=\"priority\"/><junction id=\"Z\" type=\"dead_end\"/>"
        "<tlLogic id=\"T\" type=\"static\" programID=\"0\" offset=\"0\">"
        "<phase duration=\"100\" state=\"G\"/><phase duration=\"5\" state=\"r\"/></tlLogic>"
        "<connection from=\"a\" to=\"b\" fromLane=\"0\" toLane=\"0\" tl=\"T\" linkIndex=\"0\" dir=\"s\" "
        + (internal ? "via=\":T_0_0\"" : "") + "/>"
        "<connection from=\"b\" to=\"c\" fromLane=\"0\" toLane=\"0\" dir=\"s\"/>"
        + (internal ? "<edge id=\":T_0\" function=\"internal\"><lane id=\":T_0_0\" length=\"3\" speed=\"10\"/></edge>" : "")
        + "</net>";
}
Graph load_net(bool internal) {
    Fixture file(network(internal));
    Graph graph;
    graph.read_sumo_net_xml(file.path);
    graph.build_new_graph_structures_from_sumo();
    return graph;
}
void net_checks() {
    Graph legacy;
    check(!legacy.sumoNetworkWithoutInternalLinks, "legacy unknown graph must retain crossing policy");
    for (bool internal : {false, true}) {
        auto graph = load_net(internal);
        check(graph.sumoNetworkWithoutInternalLinks == !internal, "structural net flag");
        graph.intersectionCrossingTime = 3.4;
        near(graph.movementCrossingTime(0), internal ? 3.4 : 0.0, "TLS entry uses network clock");
        near(graph.movementCrossingTime(1), 0.0, "unsignalized entry remains zero");
        rejects([&] { graph.movementCrossingTime(-1); }, "invalid movement cannot masquerade as first entry");
        graph.defaultDischargeInterval = 2.15;
        graph.travelTimeMode = TravelTimeMode::SPEED_NET;
        graph.evalTrajectoryDetails = true;
        graph.vehicleDepartureTimes = {0.25, 0.25, 0.25};
        graph.vehicleDeparturePositions = {0.0, 0.0, 0.0};
        vector<vector<int>> routes(3, {0, 1, 2});
        vector<vector<int>> queries(3, {graph.roads[0].fromNode, graph.roads[2].toNode, 0});
        graph.cycle_aware_signal_driven_records(queries, routes);
        check(graph.invalidVehicleCount == 0 && graph.finishedVehicleCount == 3, "runtime completion");
        for (size_t v = 0; v < routes.size(); ++v) {
            const auto& roads = graph.vehicleRoadTimeline[v];
            const auto& moves = graph.vehicleMovementTimeline[v];
            check(roads.size() == 3 && moves.size() == 2, "all first/middle/final occurrences retained");
            near(roads[0].intersectionCrossingTime, 0, "first has no incoming crossing");
            near(roads[1].intersectionCrossingTime, internal ? 3.4 : 0, "timeline clock");
            near(roads[1].enterTime - moves[0].dischargeTime, internal ? 3.4 : 0, "actual downstream entry clock");
            near(roads[2].enterTime - moves[1].dischargeTime, 0, "final incoming unsignalized clock");
            check(roads[2].arrivalAtRoadEndTime > roads[2].enterTime, "final road travel not skipped");
            if (v) check(moves[0].dischargeTime - graph.vehicleMovementTimeline[v - 1][0].dischargeTime
                         >= 2.15 - TIME_EPSILON, "service headway retained");
        }
        check(graph.validateRoadLaneFlows(), "capacity accounting conserved");
    }
    Graph reload = load_net(false);
    Fixture withInternal(network(true));
    reload.read_sumo_net_xml(withInternal.path);
    check(!reload.sumoNetworkWithoutInternalLinks, "reload clears prior no-internal flag");
    for (const string& broken : {
            string("<routes></routes>"), string("<net></net>"),
            network(false).insert(5, "<connection from=\"a\" to=\"b\" via=\":missing\"/>"),
            network(false).insert(5, "<junction id=\"bad\" intLanes=\":missing\"/>"),
            network(false).substr(0, network(false).size() - 6)}) {
        Fixture file(broken);
        rejects([&] { reload.read_sumo_net_xml(file.path); }, "invalid/contradictory net must not establish zero crossing");
        check(!reload.sumoNetworkWithoutInternalLinks, "failed load leaves conservative network flag");
    }
}
void read_route(Graph& graph, const string& types, const string& vehicles) {
    Fixture file("<routes>" + types + "<route id=\"r\" edges=\"a b c\"/>" + vehicles + "</routes>");
    graph.read_sumo_route_xml(file.path, -1);
}
void type_checks() {
    Graph legacy;
    near(legacy.getVehicleTypeForVehicle(-1).maxSpeed, 13.89, "legacy struct default is unchanged");
    auto graph = load_net(false);
    read_route(graph, "<vType id=\"car\" maxSpeed=\"13.89\" length=\"6\"/><vType id=\"plain\"/>",
        "<vehicle id=\"default\" route=\"r\"/><vehicle id=\"car\" type=\"car\" route=\"r\"/>"
        "<vehicle id=\"plain\" type=\"plain\" route=\"r\"/>");
    check(graph.vehicleTypeIDs == vector<string>({"DEFAULT_VEHTYPE", "car", "plain"}), "actual type assignment");
    near(graph.getVehicleTypeForVehicle(0).maxSpeed, 200.0 / 3.6, "untyped is default SUMO passenger, not declared car");
    near(graph.getVehicleTypeForVehicle(1).maxSpeed, 13.89, "explicit original car limit retained");
    near(graph.getVehicleTypeForVehicle(2).maxSpeed, 200.0 / 3.6, "explicit passenger missing maxSpeed default");
    near(graph.getVehicleTypeForVehicle(0).length, 5.0, "untyped geometry does not borrow named car");
    near(graph.getVehicleTypeForVehicle(-1).length, 6.0, "legacy representative capacity still uses named car");
    read_route(graph, "<vType id=\"DEFAULT_VEHTYPE\" maxSpeed=\"23\" length=\"6\"/>",
        "<vehicle id=\"v\" route=\"r\"/>");
    near(graph.getVehicleTypeForVehicle(0).maxSpeed, 23, "DEFAULT_VEHTYPE explicit override");
    near(graph.vehicleDeparturePositions[0], 6.1, "overridden default geometry");
    for (const string& bad : {"nan", "inf", "1e999", "23junk", "", "0", "-1"}) {
        rejects([&] { read_route(graph, "<vType id=\"bad\" maxSpeed=\"" + bad + "\"/>", ""); }, "invalid maxSpeed rejected " + bad);
    }
    for (const string& key : {"accel", "decel", "length", "sigma", "minGap", "desiredMaxSpeed"}) {
        rejects([&] { read_route(graph, "<vType id=\"bad\" " + key + "=\"bad\"/>", ""); }, "consumed malformed numeric fails " + key);
    }
    rejects([&] { read_route(graph, "", "<vehicle id=\"v\" type=\"car\" route=\"r\"/>"); }, "undefined car is not implicit");
    rejects([&] { read_route(graph, "", "<vehicle id=\"v\" type=\"\" route=\"r\"/>"); }, "explicit empty type rejected");
    rejects([&] { read_route(graph, "<vType id=\"x\" vClass=\"rail\"/>", ""); }, "unsupported class fails closed");
    rejects([&] { read_route(graph, "<vType id=\"x\"/><vType id=\"x\"/>", ""); }, "duplicate type rejected");
    rejects([&] { read_route(graph, "<vTypeDistribution id=\"x\"><vType id=\"c\"/></vTypeDistribution>", ""); }, "random type distribution not flattened");
    const vector<string> classes = {"passenger", "private", "vip", "hov", "taxi", "evehicle", "custom1", "custom2",
        "delivery", "emergency", "truck", "trailer", "bus", "coach", "motorcycle", "moped", "bicycle"};
    const double speeds[] = {200,200,200,200,200,200,200,200,200,200,130,130,100,100,200,60,50};
    const double lengths[] = {5,5,5,5,5,5,5,5,6.5,6.5,7.1,16.5,12,14,2.2,2.1,1.6};
    const double accels[] = {2.6,2.6,2.6,2.6,2.6,2.6,2.6,2.6,2.6,2.6,1.3,1.1,1.2,2,6,1.1,1.2};
    const double decels[] = {4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4.5,4,4,4,4,10,7,3};
    for (size_t i = 0; i < classes.size(); ++i) {
        read_route(graph, "<vType id=\"typed\" vClass=\"" + classes[i] + "\"/>", "<vehicle id=\"v\" type=\"typed\" route=\"r\"/>");
        const auto& type = graph.getVehicleTypeForVehicle(0);
        near(type.maxSpeed, speeds[i] / 3.6, "class speed " + classes[i]);
        near(type.length, lengths[i], "class length " + classes[i]);
        near(type.accel, accels[i], "class accel " + classes[i]);
        near(type.decel, decels[i], "class decel " + classes[i]);
        near(type.sigma, .5, "class sigma " + classes[i]);
        near(type.minGap, classes[i] == "bicycle" ? .5 : 2.5, "class minGap " + classes[i]);
        near(type.desiredMaxSpeed, (classes[i] == "bicycle" ? 20.0 : 10000.0) / 3.6, "class desired cap " + classes[i]);
    }
}
}
int main() {
    try { net_checks(); type_checks(); std::cout << "PASS sumo_network_type checks=" << checks << '\n'; return 0; }
    catch (const exception& error) { std::cerr << "FAIL sumo_network_type checks=" << checks << ": " << error.what() << '\n'; return 1; }
}
