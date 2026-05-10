#include "head.h"

struct RunConfig {
    bool useSumoNet = false;
    bool smokeTest = false;
    bool runEvaluation = true;
    bool cut = false;
    bool showHelp = false;

    int readNum = 192484;
    int avgLength = 30;

    string baseDir;
    string sumoNetPath;
    string bjPath;
    string bjMinTravelTimePath;
    string roadInfoPath;
    string queryPath;
    string routePath;
    string timePath;
    string timeNoWaitPath;

    bool useSumoSetByCli = false;
    bool sumoNetSetByCli = false;
    string envSumoNetPath;
};

namespace {
bool env_enabled(const char *value) {
    return value != nullptr && (string(value) == "1" || string(value) == "true" || string(value) == "TRUE");
}

string require_value(int argc, char **argv, int &i, const string &option) {
    if (i + 1 >= argc) {
        throw runtime_error("Missing value for " + option);
    }
    return argv[++i];
}

int parse_int_value(const string &value, const string &option) {
    try {
        size_t pos = 0;
        int parsed = stoi(value, &pos);
        if (pos != value.size()) throw invalid_argument("trailing characters");
        return parsed;
    } catch (const exception&) {
        throw runtime_error("Invalid integer for " + option + ": " + value);
    }
}

void print_sumo_sample_signal_states(Graph &g) {
    if (g.movements.empty()) return;

    int sampleMovementID = -1;
    for (const auto &m : g.movements) {
        if (!m.tlID.empty() && m.linkIndex >= 0) {
            sampleMovementID = m.movementID;
            break;
        }
    }
    if (sampleMovementID < 0) sampleMovementID = g.movements.front().movementID;

    const vector<int> sampleTimes = {0, 10, 42, 45, 90};
    cout << "[SUMO] sample movement signal states movementID=" << sampleMovementID
         << " tlID=" << g.movements[sampleMovementID].tlID
         << " linkIndex=" << g.movements[sampleMovementID].linkIndex << endl;
    for (int t : sampleTimes) {
        SignalState state = g.signalStateAtMovement(sampleMovementID, t);
        string stateText = "Red";
        if (state == SignalState::Green) stateText = "Green";
        else if (state == SignalState::Yellow) stateText = "Yellow";
        else if (state == SignalState::AlwaysOpen) stateText = "AlwaysOpen";
        cout << "[SUMO] t=" << t << " state=" << stateText << endl;
    }
}
}

void print_usage(const char* programName) {
    cout << "Usage: " << programName << " [options]\n\n"
         << "Workflow options:\n"
         << "  --help                 Show this help message and exit.\n"
         << "  --use-sumo             Use the SUMO .net.xml workflow instead of legacy BJ.\n"
         << "  --smoke-test           In SUMO mode, prepare/validate/print sample signals and exit.\n"
         << "  --no-eval              Skip final MSE/evaluation in legacy workflow.\n\n"
         << "Path options:\n"
         << "  --base <dir>           Derive standard input paths from this directory.\n"
         << "  --sumo-net <path>      SUMO .net.xml path.\n"
         << "  --bj <path>            Legacy BJ graph path.\n"
         << "  --bj-min-time <path>   Legacy BJ min travel-time path.\n"
         << "  --road-info <path>     Legacy road-info path.\n"
         << "  --query <path>         Query data path.\n"
         << "  --route <path>         Route data path.\n"
         << "  --time <path>          Time data path.\n"
         << "  --time-no-wait <path>  Time-no-wait data path.\n\n"
         << "Data options:\n"
         << "  --read-num <n>         Number of query/route/time records to read (default: 192484).\n"
         << "  --cut                  Cut route/query/time data before simulation (default: false).\n"
         << "  --avg-length <n>       Average length used when --cut is enabled (default: 30).\n\n"
         << "Environment fallback (lower priority than command-line args):\n"
         << "  USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml " << programName << "\n";
}

RunConfig parse_args(int argc, char** argv) {
    RunConfig cfg;

    const char *useSumoEnv = getenv("USE_SUMO_NET");
    const char *sumoNetEnv = getenv("SUMO_NET_PATH");
    if (sumoNetEnv != nullptr && string(sumoNetEnv).size() > 0) {
        cfg.envSumoNetPath = sumoNetEnv;
    }

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--help" || arg == "-h") cfg.showHelp = true;
        else if (arg == "--use-sumo") { cfg.useSumoNet = true; cfg.useSumoSetByCli = true; }
        else if (arg == "--smoke-test") cfg.smokeTest = true;
        else if (arg == "--base") cfg.baseDir = require_value(argc, argv, i, arg);
        else if (arg == "--sumo-net") { cfg.sumoNetPath = require_value(argc, argv, i, arg); cfg.sumoNetSetByCli = true; }
        else if (arg == "--bj") cfg.bjPath = require_value(argc, argv, i, arg);
        else if (arg == "--bj-min-time") cfg.bjMinTravelTimePath = require_value(argc, argv, i, arg);
        else if (arg == "--road-info") cfg.roadInfoPath = require_value(argc, argv, i, arg);
        else if (arg == "--query") cfg.queryPath = require_value(argc, argv, i, arg);
        else if (arg == "--route") cfg.routePath = require_value(argc, argv, i, arg);
        else if (arg == "--time") cfg.timePath = require_value(argc, argv, i, arg);
        else if (arg == "--time-no-wait") cfg.timeNoWaitPath = require_value(argc, argv, i, arg);
        else if (arg == "--read-num") cfg.readNum = parse_int_value(require_value(argc, argv, i, arg), arg);
        else if (arg == "--cut") cfg.cut = true;
        else if (arg == "--avg-length") cfg.avgLength = parse_int_value(require_value(argc, argv, i, arg), arg);
        else if (arg == "--no-eval") cfg.runEvaluation = false;
        else throw runtime_error("Unknown option: " + arg);
    }

    if (!cfg.useSumoSetByCli && env_enabled(useSumoEnv)) cfg.useSumoNet = true;
    return cfg;
}

void apply_config_to_graph(Graph& g, const RunConfig& cfg) {
    if (!cfg.baseDir.empty()) g.set_base_path(cfg.baseDir);

    if (!cfg.bjPath.empty()) g.BJ = cfg.bjPath;
    if (!cfg.bjMinTravelTimePath.empty()) g.BJ_minTravleTime = cfg.bjMinTravelTimePath;
    if (!cfg.roadInfoPath.empty()) g.beijingMoreRoadInfo = cfg.roadInfoPath;
    if (!cfg.queryPath.empty()) g.queryPath = cfg.queryPath;
    if (!cfg.routePath.empty()) g.route_path = cfg.routePath;
    if (!cfg.timePath.empty()) g.time_path = cfg.timePath;
    if (!cfg.timeNoWaitPath.empty()) g.time_path_no_wait = cfg.timeNoWaitPath;

    if (cfg.sumoNetSetByCli) g.sumoNetPath = cfg.sumoNetPath;
    else if (cfg.baseDir.empty() && !cfg.envSumoNetPath.empty()) g.sumoNetPath = cfg.envSumoNetPath;
}

void print_resolved_config(const Graph& g, const RunConfig& cfg) {
    cout << boolalpha;
    cout << "[Config] Mode: " << (cfg.useSumoNet ? "SUMO" : "Legacy BJ") << endl;
    cout << "[Config] Smoke test: " << cfg.smokeTest << endl;
    if (!g.Base.empty()) cout << "[Config] Base: " << g.Base << endl;
    if (cfg.useSumoNet) cout << "[Config] SUMO net: " << g.sumoNetPath << endl;
    else {
        cout << "[Config] BJ graph: " << g.BJ << endl;
        cout << "[Config] BJ min travel time: " << g.BJ_minTravleTime << endl;
        cout << "[Config] Road info: " << g.beijingMoreRoadInfo << endl;
    }
    cout << "[Config] Query: " << g.queryPath << endl;
    cout << "[Config] Route: " << g.route_path << endl;
    cout << "[Config] Time: " << g.time_path << endl;
    cout << "[Config] Time no wait: " << g.time_path_no_wait << endl;
    cout << "[Config] Read num: " << cfg.readNum << endl;
    cout << "[Config] Cut: " << cfg.cut << endl;
    cout << "[Config] Avg length: " << cfg.avgLength << endl;
    cout << "[Config] Evaluation: " << cfg.runEvaluation << endl;
    cout << noboolalpha;
}

int main(int argc, char** argv) {
    try {
        RunConfig cfg = parse_args(argc, argv);
        if (cfg.showHelp) {
            print_usage(argv[0]);
            return 0;
        }

        Graph g;
        apply_config_to_graph(g, cfg);
        print_resolved_config(g, cfg);

        if (cfg.useSumoNet) {
            cout << "\nSUMO Network Preparation Mode" << endl;
            cout << "-------------------------------------" << endl;
            g.read_sumo_net_xml(g.sumoNetPath);
            g.build_new_graph_structures_from_sumo();
            g.validate_sumo_network();
            g.validate_sumo_connections();
            g.validate_sumo_signal_programs();
            g.validate_sumo_routes();
            print_sumo_sample_signal_states(g);

            if (cfg.smokeTest) {
                cout << "[SUMO] Smoke test complete." << endl;
            } else {
                cout << "[SUMO] Preparation complete. Provide SUMO-compatible route/query/time data before full simulation." << endl;
            }
            return 0;
        }

        cout << "\nStep 1: Data Cleaning" << endl;
        cout << "-------------------------------------" << endl;
        cout << "Data Cleaning Done." << endl;

        cout << "\nStep 2: Data Preparation" << endl;
        cout << "-------------------------------------" << endl;

        g.read_graph();
        g.read_road_info();
        g.percent = 1;
        g.small = 0;
        g.big = 0;

        g.queryDataRaw = g.read_query(g.queryPath, cfg.readNum);
        cout << "Length of query is: " << g.queryDataRaw.size() << endl;

        g.routeDataRaw = g.read_route(g.route_path, cfg.readNum);
        cout << "Length of route is: " << g.routeDataRaw.size() << endl;

        g.timeDataRaw = g.read_time(g.time_path, cfg.readNum, g.queryDataRaw);
        cout << "Length of time is: " << g.timeDataRaw.size() << endl;

        g.check_size();

        vector<vector<int>> queryData;
        vector<vector<int>> routeData;
        vector<vector<int>> timeData;

        if (cfg.cut) {
            routeData = g.cut_route_data(g.routeDataRaw, cfg.avgLength);
            queryData = g.cut_query_data(g.queryDataRaw, routeData, cfg.avgLength);
            timeData = g.cut_time_data(g.timeDataRaw, cfg.avgLength);
        } else {
            queryData = g.queryDataRaw;
            routeData = g.routeDataRaw;
            timeData = g.timeDataRaw;
        }

        g.min_depar_time(queryData);

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
        cout << "Valid simulated vehicles: " << (g.vehicles.size() - g.invalidVehicleCount) << endl;

        if (cfg.runEvaluation) {
            cout << "\nStep 5: Evaluation" << endl;
            cout << "-------------------------------------" << endl;
            float MSE = g.MSE_estimation_cycle_aware_total(timeData, ETA);
            cout << "Cycle-aware total travel-time MSE: " << MSE << endl;
        } else {
            cout << "\nStep 5: Evaluation skipped (--no-eval)" << endl;
        }

        return 0;
    } catch (const exception& ex) {
        cerr << "[Fatal] " << ex.what() << endl;
        return 1;
    }
}
