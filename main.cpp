#include "head.h"

struct RunConfig {
    bool useSumoNet = false;
    bool smokeTest = false;
    bool runEvaluation = true;
    bool cut = false;
    bool showHelp = false;

    int readNum = 192484;
    int avgLength = 30;
    double laneDischargeInterval = 1.0;

    string baseDir;
    string sumoNetPath;
    string sumoRoutePath;
    string sumoTripinfoPath;
    string evalOutputPath;
    string predictionTripinfoOutputPath;
    bool evalSplitExtreme = false;
    bool evalExtremeTimeline = false;
    bool evalTrajectoryDetails = false;
    double evalExtremeRelativeDurationThreshold = 3.0;
    string bjPath;
    string bjMinTravelTimePath;
    string roadInfoPath;
    string queryPath;
    string routePath;
    string timePath;

    TravelTimeMode travelTimeMode = TravelTimeMode::MIN_TIME;
    string travelTimeTablePath;
    string modelHost = "127.0.0.1";
    int modelPort = 9000;
    bool fallbackToSpeedNet = true;
    bool verboseTravelTimePrediction = false;
    double speedNetHeadway = 0.8;
    double speedNetDelayMax = 6.0;
    bool speedNetUseAcceleration = true;
    bool speedNetUseFollowing = true;
    double kinematicCongestionAlpha = 1.0;

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

double parse_double_value(const string &value, const string &option) {
    try {
        size_t pos = 0;
        double parsed = stod(value, &pos);
        if (pos != value.size()) throw invalid_argument("trailing characters");
        return parsed;
    } catch (const exception&) {
        throw runtime_error("Invalid number for " + option + ": " + value);
    }
}

bool parse_tt_fallback(const string& value) {
    if (value == "speed-net") return true;
    if (value == "min-time") return false;
    throw runtime_error("Invalid travel time fallback: " + value + " (expected speed-net or min-time)");
}

string tt_fallback_to_string(bool fallbackToSpeedNet) {
    return fallbackToSpeedNet ? "speed-net" : "min-time";
}

void parse_speed_net_components(
        const string& value,
        bool& useAcceleration,
        bool& useFollowing) {
    if (value == "none") {
        useAcceleration = false;
        useFollowing = false;
        return;
    }
    if (value == "acceleration") {
        useAcceleration = true;
        useFollowing = false;
        return;
    }
    if (value == "following") {
        useAcceleration = false;
        useFollowing = true;
        return;
    }
    if (value == "both") {
        useAcceleration = true;
        useFollowing = true;
        return;
    }
    throw runtime_error(
            "Invalid speed-net components: " + value +
            " (expected none, acceleration, following, or both)");
}

string speed_net_components_to_string(
        bool useAcceleration,
        bool useFollowing) {
    if (useAcceleration && useFollowing) return "both";
    if (useAcceleration) return "acceleration";
    if (useFollowing) return "following";
    return "none";
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
         << "  --no-eval              Skip final MSE/evaluation, including SUMO tripinfo evaluation.\n\n"
         << "Path options:\n"
         << "  --base <dir>           Derive standard input paths from this directory.\n"
         << "  --sumo-net <path>      SUMO .net.xml path.\n"
         << "  --sumo-route <path>    SUMO .rou.xml route file used for full SUMO simulation.\n"
         << "  --sumo-tripinfo <path>    SUMO tripinfo.xml ground-truth output used for evaluation.\n"
         << "  --eval-output <path>   CSV path for per-vehicle SUMO evaluation comparison.\n"
         << "  --prediction-tripinfo-output <path>\n"
         << "                         Optional SUMO-like XML with CAMS predicted tripinfo records.\n"
         << "  --eval-split-extreme  Write all/non-extreme/extreme-only SUMO evaluation metrics.\n"
         << "  --eval-extreme-timeline\n"
         << "                         Record and write per-road/per-movement diagnostics for extreme vehicles.\n"
         << "  --eval-trajectory-details\n"
         << "                         Write full vehicle trajectories and downstream-release competition labels.\n"
         << "  --eval-extreme-relative-threshold <value>\n"
         << "                         Relative duration error threshold for extreme vehicles (default: 3.0).\n"
         << "  --bj <path>            Legacy BJ graph path.\n"
         << "  --bj-min-time <path>   Legacy BJ min travel-time path.\n"
         << "  --road-info <path>     Legacy road-info path.\n"
         << "  --query <path>         Query data path.\n"
         << "  --route <path>         Route data path.\n"
         << "  --time <path>          Time data path.\n"
         << "\n"
         << "Travel-time prediction options:\n"
         << "  --travel-time-mode <speed-net|min-time|table|model|kinematic>\n"
         << "                         Single-road travel-time predictor (default: min-time).\n"
         << "  --travel-time-table <path>\n"
         << "                         Table dictionary path for --travel-time-mode table.\n"
         << "  --speed-net-components <none|acceleration|following|both>\n"
         << "                         Optional speed-net corrections (default: both).\n"
         << "                         none reproduces length / cruise-speed behavior.\n"
         << "  --speed-net-headway <seconds>\n"
         << "                         Per-vehicle same-lane following delay for speed-net (default: 0.8; must be >= 0).\n"
         << "  --speed-net-delay-max <seconds>\n"
         << "                         Maximum same-lane following delay for speed-net (default: 6.0; must be >= 0).\n"
         << "  --kinematic-congestion-alpha <value>\n"
         << "                         Congestion multiplier for --travel-time-mode kinematic (default: 1.0; negative clamps to 0).\n"
         << "  --model-host <host>    External model host for future model mode (default: 127.0.0.1).\n"
         << "  --model-port <port>    External model port for future model mode (default: 9000).\n"
         << "  --tt-fallback <speed-net|min-time>\n"
         << "                         Fallback for table/model misses (default: speed-net).\n"
         << "  --verbose-travel-time  Print verbose travel-time prediction messages.\n\n"
         << "Data options:\n"
         << "  --read-num <n>         Number of query/route/time records to read (default: 192484).\n"
         << "  --cut                  Cut route/query/time data before simulation (default: false).\n"
         << "  --avg-length <n>       Average length used when --cut is enabled (default: 30).\n"
         << "  --lane-discharge-interval <seconds>\n"
         << "                         Minimum floating-point headway per movement capacity lane (default: 1.0; must be > 0).\n\n"
         << "Environment fallback (lower priority than command-line args):\n"
         << "  USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml " << programName << "\n";
}

// Parse CLI/config: this only resolves runtime options and paths.
// --travel-time-mode selects the single-road travel-time predictor, while
// --lane-discharge-interval controls the per-capacity-lane discharge headway.
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
        else if (arg == "--sumo-route") cfg.sumoRoutePath = require_value(argc, argv, i, arg);
        else if (arg == "--sumo-tripinfo") cfg.sumoTripinfoPath = require_value(argc, argv, i, arg);
        else if (arg == "--eval-output") cfg.evalOutputPath = require_value(argc, argv, i, arg);
        else if (arg == "--prediction-tripinfo-output") cfg.predictionTripinfoOutputPath = require_value(argc, argv, i, arg);
        else if (arg == "--eval-split-extreme") cfg.evalSplitExtreme = true;
        else if (arg == "--eval-extreme-timeline") cfg.evalExtremeTimeline = true;
        else if (arg == "--eval-trajectory-details") cfg.evalTrajectoryDetails = true;
        else if (arg == "--eval-extreme-relative-threshold") {
            cfg.evalExtremeRelativeDurationThreshold = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (cfg.evalExtremeRelativeDurationThreshold <= 0.0) {
                throw runtime_error("--eval-extreme-relative-threshold must be > 0");
            }
        }
        else if (arg == "--bj") cfg.bjPath = require_value(argc, argv, i, arg);
        else if (arg == "--bj-min-time") cfg.bjMinTravelTimePath = require_value(argc, argv, i, arg);
        else if (arg == "--road-info") cfg.roadInfoPath = require_value(argc, argv, i, arg);
        else if (arg == "--query") cfg.queryPath = require_value(argc, argv, i, arg);
        else if (arg == "--route") cfg.routePath = require_value(argc, argv, i, arg);
        else if (arg == "--time") cfg.timePath = require_value(argc, argv, i, arg);
        else if (arg == "--travel-time-mode") {
            string value = require_value(argc, argv, i, arg);
            if (value == "speed-net-legacy") {
                cfg.travelTimeMode = TravelTimeMode::SPEED_NET;
                cfg.speedNetUseAcceleration = false;
                cfg.speedNetUseFollowing = false;
                cout << "[Config Warning] speed-net-legacy is deprecated; using "
                     << "--travel-time-mode speed-net "
                     << "--speed-net-components none." << endl;
            } else {
                cfg.travelTimeMode = parseTravelTimeMode(value);
            }
        }
        else if (arg == "--travel-time-table") cfg.travelTimeTablePath = require_value(argc, argv, i, arg);
        else if (arg == "--speed-net-components") {
            parse_speed_net_components(
                    require_value(argc, argv, i, arg),
                    cfg.speedNetUseAcceleration,
                    cfg.speedNetUseFollowing);
        }
        else if (arg == "--speed-net-headway") {
            cfg.speedNetHeadway = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(cfg.speedNetHeadway) || cfg.speedNetHeadway < 0.0) {
                throw runtime_error("--speed-net-headway must be finite and >= 0");
            }
        }
        else if (arg == "--speed-net-delay-max") {
            cfg.speedNetDelayMax = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(cfg.speedNetDelayMax) || cfg.speedNetDelayMax < 0.0) {
                throw runtime_error("--speed-net-delay-max must be finite and >= 0");
            }
        }
        else if (arg == "--kinematic-congestion-alpha") {
            cfg.kinematicCongestionAlpha = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (cfg.kinematicCongestionAlpha < 0.0) cfg.kinematicCongestionAlpha = 0.0;
        }
        else if (arg == "--model-host") cfg.modelHost = require_value(argc, argv, i, arg);
        else if (arg == "--model-port") cfg.modelPort = parse_int_value(require_value(argc, argv, i, arg), arg);
        else if (arg == "--tt-fallback") cfg.fallbackToSpeedNet = parse_tt_fallback(require_value(argc, argv, i, arg));
        else if (arg == "--verbose-travel-time") cfg.verboseTravelTimePrediction = true;
        else if (arg == "--read-num") cfg.readNum = parse_int_value(require_value(argc, argv, i, arg), arg);
        else if (arg == "--cut") cfg.cut = true;
        else if (arg == "--avg-length") cfg.avgLength = parse_int_value(require_value(argc, argv, i, arg), arg);
        else if (arg == "--lane-discharge-interval") {
            double parsedInterval = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(parsedInterval) || parsedInterval <= 0.0) {
                throw runtime_error("--lane-discharge-interval must be finite and > 0");
            }
            cfg.laneDischargeInterval = parsedInterval;
        }
        else if (arg == "--no-eval") cfg.runEvaluation = false;
        else throw runtime_error("Unknown option: " + arg);
    }

    if (!cfg.useSumoSetByCli && env_enabled(useSumoEnv)) cfg.useSumoNet = true;
    return cfg;
}

// Apply parsed config to Graph without building data structures yet.
// Travel-time settings affect predictRoadTravelTime; dispatch scheduling still uses
// movement labels, signal state, FIFO buffers, and discharge capacity.
void apply_config_to_graph(Graph& g, const RunConfig& cfg) {
    if (!cfg.baseDir.empty()) g.set_base_path(cfg.baseDir);

    if (!cfg.bjPath.empty()) g.BJ = cfg.bjPath;
    if (!cfg.bjMinTravelTimePath.empty()) g.BJ_minTravleTime = cfg.bjMinTravelTimePath;
    if (!cfg.roadInfoPath.empty()) g.beijingMoreRoadInfo = cfg.roadInfoPath;
    if (!cfg.queryPath.empty()) g.queryPath = cfg.queryPath;
    if (!cfg.routePath.empty()) g.route_path = cfg.routePath;
    if (!cfg.timePath.empty()) g.time_path = cfg.timePath;

    if (cfg.sumoNetSetByCli) g.sumoNetPath = cfg.sumoNetPath;
    else if (cfg.baseDir.empty() && !cfg.envSumoNetPath.empty()) g.sumoNetPath = cfg.envSumoNetPath;
    if (!cfg.sumoRoutePath.empty()) g.sumoRoutePath = cfg.sumoRoutePath;
    if (!cfg.sumoTripinfoPath.empty()) g.sumoTripinfoPath = cfg.sumoTripinfoPath;
    if (!cfg.evalOutputPath.empty()) g.evalOutputPath = cfg.evalOutputPath;
    if (!cfg.predictionTripinfoOutputPath.empty()) g.predictionTripinfoOutputPath = cfg.predictionTripinfoOutputPath;
    g.evalSplitExtreme = cfg.evalSplitExtreme;
    g.evalExtremeTimeline = cfg.evalExtremeTimeline;
    g.evalTrajectoryDetails = cfg.evalTrajectoryDetails;
    g.evalExtremeRelativeDurationThreshold = cfg.evalExtremeRelativeDurationThreshold;

    g.travelTimeMode = cfg.travelTimeMode;
    if (!cfg.travelTimeTablePath.empty()) g.travelTimeTablePath = cfg.travelTimeTablePath;
    g.modelHost = cfg.modelHost;
    g.modelPort = cfg.modelPort;
    g.fallbackToSpeedNet = cfg.fallbackToSpeedNet;
    g.verboseTravelTimePrediction = cfg.verboseTravelTimePrediction;
    g.speedNetHeadway = cfg.speedNetHeadway;
    g.speedNetDelayMax = cfg.speedNetDelayMax;
    g.speedNetUseAcceleration = cfg.speedNetUseAcceleration;
    g.speedNetUseFollowing = cfg.speedNetUseFollowing;
    g.kinematicCongestionAlpha = max(0.0, cfg.kinematicCongestionAlpha);
    g.defaultDischargeInterval = cfg.laneDischargeInterval;
    g.modelWarningPrinted = false;
    g.travelTimeTableHit = 0;
    g.travelTimeTableMiss = 0;

    if (g.travelTimeMode == TravelTimeMode::TABLE) {
        if (g.travelTimeTablePath.empty()) {
            throw runtime_error("--travel-time-mode table requires --travel-time-table or a non-empty default table path");
        }
        g.buildDictionary(g.travelTimeTablePath);
        if (g.travelTimeTableFormat == TravelTimeTableFormat::SUMO_V1) {
            cout << "[TravelTime] Loaded SUMO v1 table entries: " << g.sumoV1TravelTimeTable.size() << endl;
        } else {
            cout << "[TravelTime] Loaded legacy table dictionary entries: " << g.dictionary.size() << endl;
        }
    }
}

void print_resolved_config(const Graph& g, const RunConfig& cfg) {
    cout << boolalpha;
    cout << "[Config] Mode: " << (cfg.useSumoNet ? "SUMO" : "Legacy BJ") << endl;
    cout << "[Config] Smoke test: " << cfg.smokeTest << endl;
    if (!g.Base.empty()) cout << "[Config] Base: " << g.Base << endl;
    if (cfg.useSumoNet) {
        cout << "[Config] SUMO net: " << g.sumoNetPath << endl;
        if (!g.sumoRoutePath.empty()) cout << "[Config] SUMO route: " << g.sumoRoutePath << endl;
        if (!g.sumoTripinfoPath.empty()) cout << "[Config] SUMO tripinfo: " << g.sumoTripinfoPath << endl;
        if (!g.evalOutputPath.empty()) cout << "[Config] Eval output: " << g.evalOutputPath << endl;
        if (!g.predictionTripinfoOutputPath.empty()) cout << "[Config] Prediction tripinfo output: " << g.predictionTripinfoOutputPath << endl;
        cout << "[Config] Eval split extreme: " << g.evalSplitExtreme << endl;
        cout << "[Config] Eval extreme timeline: " << g.evalExtremeTimeline << endl;
        cout << "[Config] Eval trajectory details: " << g.evalTrajectoryDetails << endl;
        cout << "[Config] Eval extreme relative threshold: " << g.evalExtremeRelativeDurationThreshold << endl;
    }
    else {
        cout << "[Config] BJ graph: " << g.BJ << endl;
        cout << "[Config] BJ min travel time: " << g.BJ_minTravleTime << endl;
        cout << "[Config] Road info: " << g.beijingMoreRoadInfo << endl;
    }
    cout << "[Config] Query: " << g.queryPath << endl;
    cout << "[Config] Route: " << g.route_path << endl;
    cout << "[Config] Time: " << g.time_path << endl;
    cout << "[Config] Read num: " << cfg.readNum << endl;
    cout << "[Config] Cut: " << cfg.cut << endl;
    cout << "[Config] Avg length: " << cfg.avgLength << endl;
    cout << "[Config] Evaluation: " << cfg.runEvaluation << endl;
    cout << "[Config] Lane discharge interval: " << g.defaultDischargeInterval << "s" << endl;
    cout << "[Config] Travel time mode: " << travelTimeModeToString(g.travelTimeMode) << endl;
    cout << "[Config] Travel time table: " << g.travelTimeTablePath << endl;
    cout << "[Config] Speed-net components: "
         << speed_net_components_to_string(
                    g.speedNetUseAcceleration,
                    g.speedNetUseFollowing)
         << endl;
    cout << "[Config] Speed-net headway: " << g.speedNetHeadway << "s" << endl;
    cout << "[Config] Speed-net delay max: " << g.speedNetDelayMax << "s" << endl;
    cout << "[Config] Kinematic congestion alpha: " << g.kinematicCongestionAlpha << endl;
    cout << "[Config] TT fallback: " << tt_fallback_to_string(g.fallbackToSpeedNet) << endl;
    cout << "[Config] Model host/port: " << g.modelHost << ":" << g.modelPort << endl;
    cout << "[Config] Verbose travel time: " << g.verboseTravelTimePrediction << endl;
    cout << noboolalpha;
}

int main(int argc, char** argv) {
    try {
        // Main workflow section 1: parse CLI/config.
        RunConfig cfg = parse_args(argc, argv);
        if (cfg.showHelp) {
            print_usage(argv[0]);
            return 0;
        }

        Graph g;
        // Main workflow section 2: configure travel-time prediction and discharge interval.
        apply_config_to_graph(g, cfg);
        print_resolved_config(g, cfg);

        // Main workflow section 3: select SUMO vs legacy BJ data-preparation workflow.
        if (cfg.useSumoNet) {
            cout << "\nSUMO Network Preparation Mode" << endl;
            cout << "-------------------------------------" << endl;
            // Read/build road network: SUMO net.xml is converted into Road/Movement/Signal/Buffer structures.
            g.read_sumo_net_xml(g.sumoNetPath);
            g.build_new_graph_structures_from_sumo();
            g.validate_sumo_network();
            g.validate_sumo_connections();
            g.validate_sumo_signal_programs();

            // --smoke-test prepares and validates SUMO structures, then exits before full simulation.
            if (cfg.smokeTest) {
                print_sumo_sample_signal_states(g);
                cout << "[SUMO] Smoke test complete." << endl;
                return 0;
            }

            if (cfg.sumoRoutePath.empty()) {
                cerr << "[Fatal] SUMO full simulation requires --sumo-route <path>. "
                     << "Use --smoke-test if you only want network validation." << endl;
                return 1;
            }

            // Validate network/routes before running the runtime event simulation.
            g.read_sumo_route_xml(cfg.sumoRoutePath, cfg.readNum);
            g.route_roadID_2_movementID();
            g.validate_sumo_routes();

            // Run cycle-aware simulation: movement dispatch is independent from travel-time-mode selection.
            cout << "\nStep: Cycle-Aware SUMO Simulation" << endl;
            cout << "-------------------------------------" << endl;
            vector<vector<pair<int, double>>> ETA =
                    g.cycle_aware_signal_driven_records(g.queryDataRaw, g.routeRoadID);

            cout << "SUMO cycle-aware simulation done." << endl;
            cout << "Finished vehicles: " << g.finishedVehicleCount
                 << " / " << g.vehicles.size() << endl;
            cout << "Invalid vehicles: " << g.invalidVehicleCount << endl;
            cout << "Valid simulated vehicles: "
                 << (g.vehicles.size() - g.invalidVehicleCount) << endl;

            cout << "[TravelTime] mode: " << travelTimeModeToString(g.travelTimeMode) << endl;
            if (g.travelTimeMode == TravelTimeMode::TABLE) {
                cout << "[TravelTime] table hits: " << g.travelTimeTableHit << endl;
                cout << "[TravelTime] table misses: " << g.travelTimeTableMiss << endl;
            }
            if (g.travelTimeMode == TravelTimeMode::TABLE || g.travelTimeMode == TravelTimeMode::MODEL) {
                cout << "[TravelTime] fallback: " << tt_fallback_to_string(g.fallbackToSpeedNet) << endl;
            }

            // Run evaluation/output after the SUMO simulation has produced ETA records.
            if (cfg.runEvaluation) {
                if (!cfg.sumoTripinfoPath.empty()) {
                    g.read_sumo_tripinfo_xml(cfg.sumoTripinfoPath);
                    g.evaluate_sumo_tripinfo_truth(ETA);
                } else {
                    cout << "[SUMO Eval] No --sumo-tripinfo provided; skipping SUMO truth evaluation." << endl;
                }
            } else {
                cout << "[SUMO Eval] Skipped because --no-eval was set." << endl;
            }

            (void)ETA;
            return 0;
        }

        cout << "\nStep 1: Data Cleaning" << endl;
        cout << "-------------------------------------" << endl;
        cout << "Data Cleaning Done." << endl;

        // Legacy BJ workflow: read/build static graph inputs consumed by the same core simulator.
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
        vector<vector<double>> timeData;

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

        // Build structured road/node/movement/signal/waiting-buffer graph for legacy BJ data.
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

        // Run cycle-aware simulation over the prepared legacy graph.
        vector<vector<pair<int, double>>> ETA =
                g.cycle_aware_signal_driven_records(queryData, g.routeRoadID);

        cout << "Cycle-aware simulation done." << endl;
        cout << "Finished vehicles: " << g.finishedVehicleCount
             << " / " << g.vehicles.size() << endl;
        cout << "Invalid vehicles: " << g.invalidVehicleCount << endl;
        cout << "Valid simulated vehicles: " << (g.vehicles.size() - g.invalidVehicleCount) << endl;

        cout << "[TravelTime] mode: " << travelTimeModeToString(g.travelTimeMode) << endl;
        if (g.travelTimeMode == TravelTimeMode::TABLE) {
            cout << "[TravelTime] table hits: " << g.travelTimeTableHit << endl;
            cout << "[TravelTime] table misses: " << g.travelTimeTableMiss << endl;
        }
        if (g.travelTimeMode == TravelTimeMode::TABLE || g.travelTimeMode == TravelTimeMode::MODEL) {
            cout << "[TravelTime] fallback: " << tt_fallback_to_string(g.fallbackToSpeedNet) << endl;
        }

        // Run evaluation/output for legacy BJ data.
        if (cfg.runEvaluation) {
            cout << "\nStep 5: Evaluation" << endl;
            cout << "-------------------------------------" << endl;
            double MSE = g.MSE_estimation_cycle_aware_total(timeData, ETA);
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
