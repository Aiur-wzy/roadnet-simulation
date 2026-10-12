#include "head.h"
#include "native_replay_guard.h"

struct RunConfig {
    bool useSumoNet = false;
    bool smokeTest = false;
    bool runEvaluation = true;
    bool cut = false;
    bool showHelp = false;

    int readNum = 192484;
    int avgLength = 30;
    double laneDischargeInterval = 2.0;
    double intersectionCrossingTime = 3.0;
    MovementQueueOrder movementQueueOrder = MovementQueueOrder::ArrivalTime;
    string dispatchQueueMode = "auto";
    bool laneBorrowing = false;
    bool mergeRoadPairMovements = false;
    bool laneEntryReselection = false;

    string baseDir;
    string sumoNetPath;
    string sumoRoutePath;
    string sumoTripinfoPath;
    string evalOutputPath;
    string predictionTripinfoOutputPath;
    string capContextTracePath;
    string demandRequestsPath;
    string demandStatusPath;
    int demandMaxMissingKeys = 2000000;
    string demandRunGuard;
    string demandRunToken;
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
    double expectedSignalRemainingStep = 0.0; // Absent: use the table's declared step.
    string modelHost = "127.0.0.1";
    int modelPort = 9000;
    bool fallbackToSpeedNet = true;
    bool verboseTravelTimePrediction = false;
    double speedNetHeadway = 0.6;
    double speedNetDelayMax = 3.0;
    bool speedNetUseAcceleration = true;
    bool speedNetUseFollowing = true;
    bool speedNetQueueTrim = false;
    bool speedNetDepartPosition = true;
    bool speedNetUseSpeedFactor = false;
    bool speedNetContinuousSpeed = false;
    double speedNetContinuousWaitThreshold = 0.0;
    string speedNetContinuousTracePath;
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

bool parse_on_off(const string& value, const string& option) {
    if (value == "on") return true;
    if (value == "off") return false;
    throw runtime_error("Invalid value for " + option + ": " + value + " (expected on or off)");
}

MovementQueueOrder parse_movement_queue_order(const string& value) {
    if (value == "arrival-time") return MovementQueueOrder::ArrivalTime;
    if (value == "fifo") return MovementQueueOrder::StrictFIFO;
    throw runtime_error(
            "Invalid movement queue order: " + value +
            " (expected arrival-time or fifo)");
}

string movement_queue_order_to_string(MovementQueueOrder order) {
    return order == MovementQueueOrder::StrictFIFO ? "fifo" : "arrival-time";
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
         << "                         Supports CAMS_CAP_CONTEXT_RLE_V1 (cap + 4 contexts, strict).\n"
         << "                         Supports CAMS_CAP_CONTEXT_ABC_RLE_V1 (cap + 6 ABC contexts, strict).\n"
         << "                         Supports CAMS_CAP_CONTEXT_ABC_RG_RLE_V1 (ABC + two R/G segments, compressed lookup).\n"
         << "                         Supports CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 (sparse, strict by default).\n"
         << "                         Supports CAMS_NATIVE_ABC_RG_DEMAND_V2 (native-trained, 64-bit sparse keys; C++ proxy inputs).\n"
         << "  --demand-requests-output <path>\n"
         << "                         Explicit provisional discovery: record missing demand keys, use SF formula temporarily.\n"
         << "                         Requires --demand-status-output; any misses suppress evaluation.\n"
         << "  --demand-status-output <path>\n"
         << "                         Write coverage/miss status for a demand-table run; never reuse an existing file.\n"
         << "  --demand-max-missing-keys <nonnegative-int> (0: no key ceiling)\n"
         << "  --demand-run-guard <lock-path> --demand-run-token <token>\n"
         << "                         Bound native sparse discovery memory (default 2000000); abort before exceeding it.\n"
         << "  --expected-signal-remaining-step <seconds>\n"
         << "                         Optional finite positive expectation for the RG table header (exact match).\n"
         << "                         Requires --use-sumo --travel-time-mode table and an RG table; never overrides its grid.\n"
         << "                         Default: auto from table; change the grid by re-exporting the table.\n"
         << "  --cap-context-trace <path>\n"
         << "                         Optional per-query F13 causal feature CSV (new table only).\n"
         << "  --speed-net-components <none|acceleration|following|both>\n"
         << "                         Optional speed-net corrections (default: both).\n"
         << "                         none reproduces length / cruise-speed behavior.\n"
         << "  --speed-net-headway <seconds>\n"
         << "                         Per-vehicle same-lane following delay for speed-net (default: 0.6; must be >= 0).\n"
         << "  --speed-net-continuous-speed <on|off> (default: off; speed-net with acceleration only)\n"
         << "  --speed-net-continuous-wait-threshold <seconds> (finite nonnegative; default: 0)\n"
         << "                         Carry estimated upstream tail speed only after sufficiently short actual release wait.\n"
         << "  --speed-net-continuous-trace <path> (requires on; new file, per-road speed proxy audit)\n"
         << "  --speed-net-delay-max <seconds>\n"
         << "                         Maximum same-lane following delay for speed-net (default: 3.0; must be >= 0).\n"
         << "  --speed-net-queue-trim <on|off>\n"
         << "                         Subtract same-lane logical waiting-buffer length at road entry (default: off).\n"
         << "                         Includes not-yet-arrived buffer vehicles; no later distance compensation.\n"
         << "                         Requires --travel-time-mode speed-net; other predictors are unchanged.\n"
         << "  --first-road-position-correction <on|off> (alias: --speed-net-depart-position)\n"
         << "                         Use SUMO base/default departure position on the first road (default: on).\n"
         << "                         Formula distance = full length - (vehicle length + 0.1 m); off restores full length.\n"
         << "                         Also controls native Table ABC clocks and formula fallbacks; use off for Manhattan.\n"
         << "                         Must match native table header. Table output and physical capacity are not rescaled.\n"
         << "                         Historical cap/context tables keep their original geometry contract.\n"
         << "  --speed-net-speed-factor <on|off>\n"
         << "                         Apply explicit per-vehicle SF to the speed-net target speed (default: off).\n"
         << "                         Cap = min(maxSpeed, SF * desiredMaxSpeed, SF * roadSpeed).\n"
         << "                         Requires SUMO speed-net and explicit SF for every loaded vehicle.\n"
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
         << "                         Minimum floating-point headway per movement capacity lane (default: 2.0; must be > 0).\n"
         << "  --intersection-crossing-time <seconds>\n"
         << "                         Time from movement discharge to downstream-road entry (default: 3.0; must be >= 0).\n"
         << "  --movement-queue-order <arrival-time|fifo>\n"
         << "                         Within-movement queue policy (default: arrival-time).\n"
         << "                         fifo preserves road-entry order despite prediction fluctuations.\n"
         << "  --movement-grouping <legacy-link|road-pair>\n"
         << "                         Merge only identical ordered from/to roads (default: legacy-link).\n"
         << "                         road-pair uses lane-level source queues; requires SUMO and lane-borrowing off.\n"
         << "  --lane-entry-reselection <on|off>\n"
         << "                         Rechoose next-movement lane at road entry (default: off).\n"
         << "                         Requires road-pair and borrowing off; real entry AND final lane must have space.\n"
         << "                         Only the final lane is occupied; no continuous lane changes or receiver clock.\n"
         << "  --lane-borrowing <on|off>\n"
         << "                         Borrow same-road entry-lane storage when original target lanes are full (default: off).\n"
         << "                         Requires --use-sumo; preserves logical lane features and movement service capacity.\n"
         << "                         Prediction lane_flow is capped at the original lane capacity; raw counts remain exact.\n"
         << "  --dispatch-queue-mode <incremental|rebuild>\n"
         << "                         Candidate maintenance (default: incremental for SUMO, rebuild for legacy).\n\n"
         << "Environment fallback (lower priority than command-line args):\n"
         << "  USE_SUMO_NET=1 SUMO_NET_PATH=test.net.xml " << programName << "\n";
}

// Parse CLI/config: this only resolves runtime options and paths.
// --travel-time-mode selects the single-road travel-time predictor, while
// --lane-discharge-interval controls the per-capacity-lane discharge headway;
// --intersection-crossing-time controls one vehicle's travel through the junction.
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
        else if (arg == "--expected-signal-remaining-step") {
            cfg.expectedSignalRemainingStep = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(cfg.expectedSignalRemainingStep) || cfg.expectedSignalRemainingStep <= 0.0) {
                throw runtime_error("--expected-signal-remaining-step must be finite and > 0");
            }
        }
        else if (arg == "--cap-context-trace") cfg.capContextTracePath = require_value(argc, argv, i, arg);
        else if (arg == "--demand-requests-output") cfg.demandRequestsPath = require_value(argc, argv, i, arg);
        else if (arg == "--demand-status-output") cfg.demandStatusPath = require_value(argc, argv, i, arg);
        else if (arg == "--demand-max-missing-keys") {
            cfg.demandMaxMissingKeys = parse_int_value(require_value(argc, argv, i, arg), arg);
            if (cfg.demandMaxMissingKeys < 0) throw runtime_error("--demand-max-missing-keys must be nonnegative");
        }
        else if (arg == "--demand-run-guard") cfg.demandRunGuard = require_value(argc, argv, i, arg);
        else if (arg == "--demand-run-token") cfg.demandRunToken = require_value(argc, argv, i, arg);
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
        else if (arg == "--speed-net-continuous-speed") {
            cfg.speedNetContinuousSpeed = parse_on_off(require_value(argc, argv, i, arg), arg);
        }
        else if (arg == "--speed-net-continuous-wait-threshold") {
            cfg.speedNetContinuousWaitThreshold = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(cfg.speedNetContinuousWaitThreshold) || cfg.speedNetContinuousWaitThreshold < 0.0)
                throw runtime_error("--speed-net-continuous-wait-threshold must be finite and >= 0");
        }
        else if (arg == "--speed-net-continuous-trace") {
            cfg.speedNetContinuousTracePath = require_value(argc, argv, i, arg);
        }
        else if (arg == "--speed-net-delay-max") {
            cfg.speedNetDelayMax = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(cfg.speedNetDelayMax) || cfg.speedNetDelayMax < 0.0) {
                throw runtime_error("--speed-net-delay-max must be finite and >= 0");
            }
        }
        else if (arg == "--speed-net-queue-trim") {
            cfg.speedNetQueueTrim = parse_on_off(require_value(argc, argv, i, arg), arg);
        }
        else if (arg == "--speed-net-depart-position" || arg == "--first-road-position-correction") {
            cfg.speedNetDepartPosition = parse_on_off(require_value(argc, argv, i, arg), arg);
        }
        else if (arg == "--speed-net-speed-factor") {
            cfg.speedNetUseSpeedFactor = parse_on_off(require_value(argc, argv, i, arg), arg);
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
        else if (arg == "--intersection-crossing-time") {
            double parsedCrossingTime = parse_double_value(require_value(argc, argv, i, arg), arg);
            if (!std::isfinite(parsedCrossingTime) || parsedCrossingTime < 0.0) {
                throw runtime_error("--intersection-crossing-time must be finite and >= 0");
            }
            cfg.intersectionCrossingTime = parsedCrossingTime;
        }
        else if (arg == "--movement-queue-order") {
            cfg.movementQueueOrder = parse_movement_queue_order(
                    require_value(argc, argv, i, arg));
        }
        else if (arg == "--lane-borrowing") {
            cfg.laneBorrowing = parse_on_off(require_value(argc, argv, i, arg), arg);
        }
        else if (arg == "--lane-entry-reselection") {
            cfg.laneEntryReselection = parse_on_off(require_value(argc, argv, i, arg), arg);
        }
        else if (arg == "--movement-grouping") {
            const string value = require_value(argc, argv, i, arg);
            if (value != "legacy-link" && value != "road-pair")
                throw runtime_error("--movement-grouping must be legacy-link or road-pair");
            cfg.mergeRoadPairMovements = value == "road-pair";
        }
        else if (arg == "--dispatch-queue-mode") {
            cfg.dispatchQueueMode = require_value(argc, argv, i, arg);
            if (cfg.dispatchQueueMode != "incremental" && cfg.dispatchQueueMode != "rebuild")
                throw runtime_error("--dispatch-queue-mode must be incremental or rebuild");
        }
        else if (arg == "--no-eval") cfg.runEvaluation = false;
        else throw runtime_error("Unknown option: " + arg);
    }

    if (!cfg.useSumoSetByCli && env_enabled(useSumoEnv)) cfg.useSumoNet = true;
    if (!cfg.showHelp && cfg.laneBorrowing && !cfg.useSumoNet)
        throw runtime_error("--lane-borrowing on requires --use-sumo");
    if (!cfg.showHelp && cfg.mergeRoadPairMovements && !cfg.useSumoNet)
        throw runtime_error("--movement-grouping road-pair requires --use-sumo");
    if (!cfg.showHelp && cfg.mergeRoadPairMovements && cfg.laneBorrowing)
        throw runtime_error("--movement-grouping road-pair currently requires --lane-borrowing off");
    if (!cfg.showHelp && cfg.laneEntryReselection && (!cfg.mergeRoadPairMovements || cfg.laneBorrowing))
        throw runtime_error("--lane-entry-reselection on requires --movement-grouping road-pair and --lane-borrowing off");
    if (cfg.dispatchQueueMode == "incremental" && !cfg.useSumoNet)
        throw runtime_error("Incremental dispatch currently requires --use-sumo; legacy retains rebuild mode");
    if (!cfg.showHelp && cfg.speedNetContinuousSpeed &&
        (cfg.travelTimeMode != TravelTimeMode::SPEED_NET || !cfg.speedNetUseAcceleration))
        throw runtime_error("--speed-net-continuous-speed on requires speed-net with acceleration enabled");
    if (!cfg.showHelp && !cfg.speedNetContinuousTracePath.empty()) {
        if (!cfg.speedNetContinuousSpeed)
            throw runtime_error("--speed-net-continuous-trace requires continuous-speed on");
        if (ifstream(cfg.speedNetContinuousTracePath.c_str()).good())
            throw runtime_error("Continuous speed trace already exists: " + cfg.speedNetContinuousTracePath);
    }
    if (!cfg.showHelp && cfg.speedNetQueueTrim && cfg.travelTimeMode != TravelTimeMode::SPEED_NET) {
        throw runtime_error("--speed-net-queue-trim on requires --travel-time-mode speed-net");
    }
    if (!cfg.showHelp && cfg.speedNetUseSpeedFactor
        && (cfg.travelTimeMode != TravelTimeMode::SPEED_NET || !cfg.useSumoNet)) {
        throw runtime_error("--speed-net-speed-factor on requires --use-sumo --travel-time-mode speed-net");
    }
    if (!cfg.showHelp && cfg.expectedSignalRemainingStep > 0.0
        && (cfg.travelTimeMode != TravelTimeMode::TABLE || !cfg.useSumoNet)) {
        throw runtime_error("--expected-signal-remaining-step requires --use-sumo --travel-time-mode table");
    }
    if (!cfg.showHelp && (!cfg.demandRequestsPath.empty() || !cfg.demandStatusPath.empty())) {
        if (!cfg.useSumoNet || cfg.travelTimeMode != TravelTimeMode::TABLE || cfg.smokeTest)
            throw runtime_error("Demand outputs require a full --use-sumo --travel-time-mode table run");
        if (!cfg.demandRequestsPath.empty() && cfg.demandStatusPath.empty())
            throw runtime_error("Discovery requires --demand-status-output; provisional runs are not final results");
        // Avoid replacing an earlier result, the table, or either input.
        for (const string& path : {cfg.demandRequestsPath, cfg.demandStatusPath}) {
            if (!path.empty() && ifstream(path.c_str()).good())
                throw runtime_error("Demand output already exists: " + path);
        }
        if (!cfg.demandRequestsPath.empty() && cfg.demandRequestsPath == cfg.demandStatusPath)
            throw runtime_error("Demand requests/status paths must differ");
    }
    return cfg;
}

// Apply parsed config to Graph without building data structures yet.
// Travel-time settings affect predictRoadTravelTime; dispatch scheduling still uses
// movement labels, signal state, configured waiting-buffer order, and discharge capacity.
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
    g.capContextTracePath = cfg.capContextTracePath;
    g.demandRequestsPath = cfg.demandRequestsPath;
    g.demandMaxMissingKeys = static_cast<std::size_t>(cfg.demandMaxMissingKeys);
    if (!cfg.travelTimeTablePath.empty()) g.travelTimeTablePath = cfg.travelTimeTablePath;
    g.modelHost = cfg.modelHost;
    g.modelPort = cfg.modelPort;
    g.fallbackToSpeedNet = cfg.fallbackToSpeedNet;
    g.verboseTravelTimePrediction = cfg.verboseTravelTimePrediction;
    g.speedNetHeadway = cfg.speedNetHeadway;
    g.speedNetDelayMax = cfg.speedNetDelayMax;
    g.speedNetUseAcceleration = cfg.speedNetUseAcceleration;
    g.speedNetUseFollowing = cfg.speedNetUseFollowing;
    g.speedNetQueueTrim = cfg.speedNetQueueTrim;
    g.speedNetDepartPosition = cfg.speedNetDepartPosition;
    g.speedNetUseSpeedFactor = cfg.speedNetUseSpeedFactor;
    g.speedNetContinuousSpeed = cfg.speedNetContinuousSpeed;
    g.speedNetContinuousWaitThreshold = cfg.speedNetContinuousWaitThreshold;
    g.speedNetContinuousTracePath = cfg.speedNetContinuousTracePath;
    g.kinematicCongestionAlpha = max(0.0, cfg.kinematicCongestionAlpha);
    g.defaultDischargeInterval = cfg.laneDischargeInterval;
    g.intersectionCrossingTime = cfg.intersectionCrossingTime;
    g.movementQueueOrder = cfg.movementQueueOrder;
    g.laneBorrowing = cfg.laneBorrowing;
    g.mergeRoadPairMovements = cfg.mergeRoadPairMovements;
    g.laneEntryReselection = cfg.laneEntryReselection;
    g.persistentDispatch = cfg.dispatchQueueMode == "incremental"
        || (cfg.dispatchQueueMode == "auto" && cfg.useSumoNet);
    g.modelWarningPrinted = false;
    g.travelTimeTableHit = 0;
    g.travelTimeTableMiss = 0;

    if (g.travelTimeMode == TravelTimeMode::TABLE) {
        if (g.travelTimeTablePath.empty()) {
            throw runtime_error("--travel-time-mode table requires --travel-time-table or a non-empty default table path");
        }
        g.buildDictionary(g.travelTimeTablePath);
        if (g.usesNativeRoadTimeTable()) {
            if (!cfg.useSumoNet) throw runtime_error("Native road-time demand tables require --use-sumo");
            g.nativeRoadTimeTable.requireNetworkFile(g.sumoNetPath);
        }
        if ((!cfg.demandRequestsPath.empty() || !cfg.demandStatusPath.empty()) && !g.usesDemandCapContextTable())
            throw runtime_error("Demand outputs require a versioned sparse demand table; full tables remain strict");
        if (!cfg.demandRequestsPath.empty()) {
            g.demandRequests.open(cfg.demandRequestsPath.c_str());
            if (!g.demandRequests) throw runtime_error("Cannot open demand requests: " + cfg.demandRequestsPath);
            g.demandRequests << "group_index,road_flow\n";
        }
        if (cfg.expectedSignalRemainingStep > 0.0) {
            if (!g.usesRGCapContextTable()) {
                throw runtime_error("--expected-signal-remaining-step requires an RG demand/RLE table");
            }
            g.requireRGSignalRemainingStep(cfg.expectedSignalRemainingStep);
        }
        if (g.usesCapContextTable()) {
            if (!cfg.useSumoNet) throw runtime_error("CAMS cap/context tables require --use-sumo");
            cout << "[TravelTime] table format: " << (g.usesNativeRoadTimeTable() ? "CAMS_NATIVE_ABC_RG_DEMAND_V2" : g.usesRGCapContextTable()
                ? (g.usesDemandCapContextTable() ? "CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1" : "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1") : g.usesABCCapContextTable()
                    ? "CAMS_CAP_CONTEXT_ABC_RLE_V1" : "CAMS_CAP_CONTEXT_RLE_V1") << endl;
            cout << "[TravelTime] table entries: " << (g.usesRGCapContextTable()
                ? g.rgTableSize() : g.capContextTable.size()) << endl;
            cout << "[TravelTime] source label: " << (g.usesRGCapContextTable()
                ? g.rgTableLabel() : g.capContextTable.label()) << endl;
            if (g.usesNativeRoadTimeTable())
                cout << "[TravelTime] Training: SUMO first-visible snapshot; deployment: unchanged C++ entry proxies (not microscopic equivalence)." << endl;
            if (g.usesRGCapContextTable()) {
                ostringstream signalStep;
                signalStep << setprecision(numeric_limits<double>::max_digits10)
                           << g.rgSignalRemainingStep();
                cout << "[TravelTime] RG signal remaining step: " << signalStep.str() << "s (auto from table)" << endl;
                cout << "[TravelTime] RG signal points: " << g.rgSignalPointCount() << endl;
                cout << "[TravelTime] RG table compressed bytes: " << g.rgTableBytes() << endl;
                if (g.usesDemandCapContextTable()) {
                    cout << "[Demand] cached nodes: " << g.demandEntryCount() << endl;
                    cout << "[Demand] scope: " << g.demandScope() << endl;
                    cout << "[Demand] mode: " << (cfg.demandRequestsPath.empty() ? "strict" : "provisional discovery") << endl;
                } else {
                    cout << "[TravelTime] RG table: exact output RLE; declared nearest time nodes; no dense expansion or fallback." << endl;
                }
            } else {
                cout << "[TravelTime] cap/context table: exact RLE expanded to float64; strict keys, no fallback." << endl;
            }
        } else if (g.travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1) {
            if (!cfg.useSumoNet) {
                throw runtime_error("CAMS_SF_TABLE_V1 requires --use-sumo");
            }
            cout << "[TravelTime] table format: CAMS_SF_TABLE_V1" << endl;
            cout << "[TravelTime] table entries: " << g.sfTravelTimeTable.size() << endl;
            cout << "[TravelTime] source label: " << g.sfTravelTimeTable.label() << endl;
            cout << "[TravelTime] SF table requires explicit route SF; out-of-range/missing keys are fatal." << endl;
        } else if (g.travelTimeTableFormat == TravelTimeTableFormat::SUMO_V1) {
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
    cout << "[Config] Intersection crossing time: " << g.intersectionCrossingTime << "s" << endl;
    cout << "[Config] Movement queue order: "
         << movement_queue_order_to_string(g.movementQueueOrder) << endl;
    cout << "[Config] Lane borrowing: " << (g.laneBorrowing ? "on" : "off") << endl;
    cout << "[Config] Movement grouping: " << (g.mergeRoadPairMovements ? "road-pair" : "legacy-link") << endl;
    cout << "[Config] Lane entry reselection: " << (g.laneEntryReselection ? "on (entry constrained)" : "off") << endl;
    cout << "[Config] Travel time mode: " << travelTimeModeToString(g.travelTimeMode) << endl;
    cout << "[Config] Travel time table: " << g.travelTimeTablePath << endl;
    cout << "[Config] Speed-net components: "
         << speed_net_components_to_string(
                    g.speedNetUseAcceleration,
                    g.speedNetUseFollowing)
         << endl;
    cout << "[Config] Speed-net headway: " << g.speedNetHeadway << "s" << endl;
    cout << "[Config] Speed-net delay max: " << g.speedNetDelayMax << "s" << endl;
    cout << "[Config] Speed-net queue trim: " << (g.speedNetQueueTrim ? "on" : "off") << endl;
    cout << "[Config] First-road position correction: " << (g.speedNetDepartPosition ? "on" : "off")
         << " (formula distance + native ABC clocks; physical length and table output unchanged)" << endl;
    if (g.usesNativeRoadTimeTable())
        cout << "[Config] Native table cap out-of-range: " << (g.nativeRoadTimeTable.clampsCap() ? "clamp" : "error")
             << "; flow, signal and categorical inputs remain strict" << endl;
    cout << "[Config] Speed-net speed factor: " << (g.speedNetUseSpeedFactor ? "on" : "off") << endl;
    cout << "[Config] Speed-net continuous speed: " << (g.speedNetContinuousSpeed ? "on" : "off")
         << " wait_threshold=" << g.speedNetContinuousWaitThreshold
         << " trace=" << g.speedNetContinuousTracePath << endl;
    cout << "[Config] Kinematic congestion alpha: " << g.kinematicCongestionAlpha << endl;
    cout << "[Config] TT fallback: " << tt_fallback_to_string(g.fallbackToSpeedNet) << endl;
    if (g.travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1) {
        cout << "[Config] SF table fallback: disabled (strict errors)" << endl;
    }
    if (g.usesCapContextTable()) {
        cout << "[Config] Cap/context table fallback: "
             << (g.demandRequestsPath.empty() ? "disabled" : "explicit provisional discovery only (no final metrics on misses)")
             << "; P/E=0.05/2, epsilon=1e-9s" << endl;
        if (g.usesABCCapContextTable()) {
            cout << "[Config] ABC contexts: 0=A,1/2/3=legacy,4=B,5=C; overdue threshold: "
                 << g.capContextOverdueThreshold() << "s (table contract)" << endl;
        }
    }
    cout << "[Config] Model host/port: " << g.modelHost << ":" << g.modelPort << endl;
    cout << "[Config] Verbose travel time: " << g.verboseTravelTimePrediction << endl;
    cout << noboolalpha;
}

bool demand_run_complete(const Graph& g) {
    std::uint64_t expectedQueries = 0;
    for (const auto& route : g.routeRoadID) expectedQueries += route.size();
    const std::uint64_t queries = static_cast<std::uint64_t>(g.travelTimeTableHit) + g.travelTimeTableMiss;
    return !g.vehicles.empty() && g.invalidVehicleCount == 0 &&
        g.finishedVehicleCount == static_cast<int>(g.vehicles.size()) && queries == expectedQueries;
}

// Demand status is emitted only after the current run (and any exact evaluation)
// has returned normally. An exception cannot leave a successful status marker.
bool write_demand_status(Graph& g, const RunConfig& cfg) {
    if (!g.usesDemandCapContextTable()) return true;
    if (g.demandRequests.is_open()) {
        g.demandRequests.close();
        if (g.demandRequests.fail()) throw runtime_error("Cannot finalize demand requests: " + g.demandRequestsPath);
    }
    std::uint64_t expectedQueries = 0;
    for (const auto& route : g.routeRoadID) expectedQueries += route.size();
    const std::uint64_t queries = static_cast<std::uint64_t>(g.travelTimeTableHit) + g.travelTimeTableMiss;
    const bool complete = demand_run_complete(g);
    const bool exact = complete && g.travelTimeTableMiss == 0;
    if (!cfg.demandStatusPath.empty()) {
        ofstream out(cfg.demandStatusPath.c_str());
        if (!out) throw runtime_error("Cannot open demand status: " + cfg.demandStatusPath);
        out << "{\n  \"schema\": \"" << (g.usesNativeRoadTimeTable() ? "native-road-time-demand-status-v2" : "abc-rg-demand-status-v1")
            << "\",\n  \"scope_sha256\": \"" << g.demandScope() << "\",\n"
            << "  \"complete\": " << (complete ? "true" : "false") << ",\n"
            << "  \"exact\": " << (exact ? "true" : "false") << ",\n"
            << "  \"discovery_enabled\": " << (!g.demandRequestsPath.empty() ? "true" : "false") << ",\n"
            << "  \"vehicle_count\": " << g.vehicles.size() << ",\n"
            << "  \"finished_count\": " << g.finishedVehicleCount << ",\n"
            << "  \"invalid_count\": " << g.invalidVehicleCount << ",\n"
            << "  \"expected_queries\": " << expectedQueries << ",\n"
            << "  \"query_count\": " << queries << ",\n"
            << "  \"missing_queries\": " << g.travelTimeTableMiss << ",\n"
            << "  \"missing_unique\": " << g.demandMissingCount() << "\n}\n";
        out.close();
        if (out.fail()) throw runtime_error("Cannot finalize demand status: " + cfg.demandStatusPath);
    }
    cout << "[Demand] complete=" << complete << " exact=" << exact
         << " missing_queries=" << g.travelTimeTableMiss << " missing_unique=" << g.demandMissingCount() << endl;
    return complete;
}

int main(int argc, char** argv) {
    try {
        // Main workflow section 1: parse CLI/config.
        RunConfig cfg = parse_args(argc, argv);
        if (cfg.showHelp) {
            print_usage(argv[0]);
            return 0;
        }

        NativeReplayGuard replayGuard(cfg.demandRunGuard, cfg.demandRunToken);
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
                if (g.travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1) {
                    cout << "[SFTable] queries=" << g.travelTimeTableHit
                         << " road_flow_outside_training=" << g.sfTableRoadFlowOutsideTraining
                         << " lane_flow_outside_training=" << g.sfTableLaneFlowOutsideTraining
                         << " road_flow_max_seen=" << g.sfTableMaxRoadFlowSeen
                         << " lane_flow_max_seen=" << g.sfTableMaxLaneFlowSeen << endl;
                }
            }
            if (g.travelTimeMode == TravelTimeMode::TABLE || g.travelTimeMode == TravelTimeMode::MODEL) {
                cout << "[TravelTime] fallback: "
                     << (!g.demandRequestsPath.empty() ? "explicit provisional discovery (not a final result)"
                         : g.usesCapContextTable() ? "disabled (strict cap/context table)"
                         : g.travelTimeTableFormat == TravelTimeTableFormat::CAMS_SF_V1
                             ? "disabled (strict SF table)" : tt_fallback_to_string(g.fallbackToSpeedNet)) << endl;
            }

            // Run evaluation/output after the SUMO simulation has produced ETA records.
            if (g.usesDemandCapContextTable() && (g.travelTimeTableMiss > 0 || !demand_run_complete(g))) {
                cout << "[Demand] Evaluation suppressed: provisional or incomplete run. Batch missing keys and rerun." << endl;
            } else if (cfg.runEvaluation) {
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
            return write_demand_status(g, cfg) ? 0 : 2;
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
