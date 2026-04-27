#include "head.h"

int main() {
    Graph g;

    cout << "\nStep 1: Data Cleaning" << endl;
    cout << "-------------------------------------" << endl;
    cout << "Data Cleaning Done." << endl;

    cout << "\nStep 2: Data Preparation" << endl;
    cout << "-------------------------------------" << endl;

    g.read_graph();
    g.read_road_info();

    int readNum = 192484;
    g.percent = 1;
    g.small = 0;
    g.big = 0;

    g.queryDataRaw = g.read_query(g.queryPath, readNum);
    cout << "Length of query is: " << g.queryDataRaw.size() << endl;

    g.routeDataRaw = g.read_route(g.route_path, readNum);
    cout << "Length of route is: " << g.routeDataRaw.size() << endl;

    g.timeDataRaw = g.read_time(g.time_path, readNum, g.queryDataRaw);
    cout << "Length of time is: " << g.timeDataRaw.size() << endl;

    g.check_size();

    int avg_length = 30;
    bool cut = false;

    vector<vector<int>> queryData;
    vector<vector<int>> routeData;
    vector<vector<int>> timeData;

    if (cut) {
        routeData = g.cut_route_data(g.routeDataRaw, avg_length);
        queryData = g.cut_query_data(g.queryDataRaw, routeData, avg_length);
        timeData = g.cut_time_data(g.timeDataRaw, avg_length);
    } else {
        queryData = g.queryDataRaw;
        routeData = g.routeDataRaw;
        timeData = g.timeDataRaw;
    }

    g.min_depar_time(queryData);

    bool runLegacy = false;
    bool runCycleAware = true;

    if (runLegacy) {
        cout << "\nStep 3(Legacy): ALGORITHM I SIMULATION" << endl;
        cout << "-------------------------------------" << endl;

        g.route_nodeID_2_roadID(routeData);
        g.classify_latency_function();
        g.minRange = 20;
        g.flowIni = 0;
        g.flow_base_ini(g.minRange, g.flowIni);

        bool range = true;
        bool server = false;
        bool catching = false;
        bool write = false;
        bool latency = true;
        string te_choose = "catching";

        vector<vector<pair<int, float>>> ETA_old = g.alg1Records(
                queryData, routeData, range, server, catching, write, latency, te_choose);
        g.MSE_estimation(timeData, ETA_old);
    }

    if (runCycleAware) {
        cout << "\nStep 3: Build Cycle-Aware Graph" << endl;
        cout << "-------------------------------------" << endl;

        g.build_new_graph_structures(routeData);

        cout << "Cycle-aware graph build done." << endl;
        cout << "Roads: " << g.roads.size() << endl;
        cout << "Nodes: " << g.nodes.size() << endl;
        cout << "Intersections: " << g.intersections.size() << endl;
        cout << "Movements: " << g.movements.size() << endl;
        cout << "Signals: " << g.signals.size() << endl;
        cout << "Waiting buffers: " << g.waitingBuffers.size() << endl;
        cout << "Route road sequences: " << g.routeRoadID.size() << endl;
        cout << "Route movement sequences: " << g.routeMovementID.size() << endl;

        cout << "\nStep 4: Cycle-Aware Signal-Driven Simulation" << endl;
        cout << "-------------------------------------" << endl;

        vector<vector<pair<int, float>>> ETA =
                g.cycle_aware_signal_driven_records(queryData, g.routeRoadID);

        cout << "Cycle-aware simulation done." << endl;
        cout << "Finished vehicles: " << g.finishedVehicleCount
             << " / " << g.vehicles.size() << endl;
        cout << "Invalid vehicles: " << g.invalidVehicleCount << endl;

        cout << "\nStep 5: Evaluation" << endl;
        cout << "-------------------------------------" << endl;

        float MSE = g.MSE_estimation_cycle_aware_total(timeData, ETA);
        cout << "Cycle-aware total travel-time MSE: " << MSE << endl;
    }

    return 0;
}
