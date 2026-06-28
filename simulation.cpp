
#include "head.h"

TravelTimeMode parseTravelTimeMode(const string& s) {
    if (s == "speed-net") return TravelTimeMode::SPEED_NET;
    if (s == "min-time") return TravelTimeMode::MIN_TIME;
    if (s == "table") return TravelTimeMode::TABLE;
    if (s == "model") return TravelTimeMode::MODEL;
    if (s == "kinematic") return TravelTimeMode::KINEMATIC;
    throw runtime_error("Invalid travel time mode: " + s + " (expected speed-net, min-time, table, model, or kinematic)");
}

string travelTimeModeToString(TravelTimeMode mode) {
    switch (mode) {
        case TravelTimeMode::SPEED_NET: return "speed-net";
        case TravelTimeMode::MIN_TIME: return "min-time";
        case TravelTimeMode::TABLE: return "table";
        case TravelTimeMode::MODEL: return "model";
        case TravelTimeMode::KINEMATIC: return "kinematic";
    }
    return "speed-net";
}

// 函数用于读取CSV文件并构建map
void Graph::readConnectionsToDirections(const string& filename) {

    ifstream file(filename);
    string line;

    // 跳过标题行
    getline(file, line);

    while (getline(file, line)) {
        stringstream linestream(line);
        string from_edge_str, to_edge_str, direction_str;
        int from_edge, to_edge;
        char direction;

        // 使用逗号分隔值
        getline(linestream, from_edge_str, ',');
        getline(linestream, to_edge_str, ',');
        getline(linestream, direction_str);

        // 将字符串转换为适当的类型
        from_edge = std::stoi(from_edge_str);
        to_edge = std::stoi(to_edge_str);
        direction = direction_str[0]; // 假设direction总是一个字符

        // 将数据添加到map中
        connections_to_direction[make_pair(from_edge, to_edge)] = direction;
    }
}

// 定义函数，返回给定边的下一边的方向
// 读取文件并构建词典的函数
static long long quantizeRoadLength(double roadLength) {
    return static_cast<long long>(std::llround(roadLength * 10000.0));
}

static long long quantizeLaneFlow(double laneFlow) {
    return static_cast<long long>(std::llround(laneFlow * 1000000.0));
}

static double sumoV1LaneFlowForModel(const BasicRoadModelFeatures& features) {
    return static_cast<double>(std::max(0, features.lane_flow));
}

static bool isSumoV1TravelTimeHeader(const string& line) {
    stringstream ss(line);
    vector<string> cols;
    string col;
    while (ss >> col) cols.push_back(col);
    if (cols.size() < 6) return false;
    return cols[0] == "has_waiting" &&
           cols[1] == "road_length" &&
           cols[2] == "turn_type" &&
           cols[3] == "road_flow" &&
           cols[4] == "lane_flow" &&
           cols[5] == "travel_time_no_waiting";
}

static bool parseHasWaitingValue(const string& value, int& hasWaiting) {
    string normalized = value;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "true" || normalized == "1") {
        hasWaiting = 1;
        return true;
    }
    if (normalized == "false" || normalized == "0") {
        hasWaiting = 0;
        return true;
    }
    return false;
}

// 读取文件并构建词典的函数
void Graph::buildDictionary(const std::string& filename) {

    dictionary.clear();
    sumoV1TravelTimeTable.clear();
    travelTimeTableFormat = TravelTimeTableFormat::LEGACY;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    string firstLine;
    if (!getline(file, firstLine)) {
        return;
    }

    if (isSumoV1TravelTimeHeader(firstLine)) {
        travelTimeTableFormat = TravelTimeTableFormat::SUMO_V1;
        string hasWaitingRaw;
        double roadLength = 0.0;
        int turnType = 0;
        int roadFlow = 0;
        double laneFlow = 0.0;
        double travelTimeNoWaiting = 0.0;

        while (file >> hasWaitingRaw >> roadLength >> turnType >> roadFlow >> laneFlow >> travelTimeNoWaiting) {
            int hasWaiting = 0;
            if (!parseHasWaitingValue(hasWaitingRaw, hasWaiting)) {
                continue;
            }
            SumoV1TravelTimeKey key{};
            key.has_waiting = hasWaiting;
            key.road_length_q = quantizeRoadLength(roadLength);
            key.turn_type = turnType;
            key.road_flow = roadFlow;
            key.lane_flow_q = quantizeLaneFlow(laneFlow);
            sumoV1TravelTimeTable[key] = travelTimeNoWaiting;
        }
        return;
    }

    RoadKey key;
    double travel_time_predict;
    stringstream firstRow(firstLine);
    if (firstRow >> key.lane_num >> key.speed_limit >> key.edge_length >> key.driving_number >> key.delay_time >> key.lowSpee_time >> key.wait_time >> key.ratio >> key.length_square >> travel_time_predict) {
        dictionary[key] = travel_time_predict;
    }

    while (file >> key.lane_num >> key.speed_limit >> key.edge_length >> key.driving_number >> key.delay_time >> key.lowSpee_time >> key.wait_time >> key.ratio >> key.length_square >> travel_time_predict) {
        dictionary[key] = travel_time_predict;
    }
}

float Graph::MSE_estimation_cycle_aware_total(
        vector<vector<int>> &timeData,
        vector<vector<pair<int, float>>> &ETA)
{
    double mse = 0.0;
    double mae = 0.0;
    double rmse = 0.0;
    double mape = 0.0;
    int validCnt = 0;

    int n = min(static_cast<int>(ETA.size()), static_cast<int>(timeData.size()));

    for (int i = 0; i < n; ++i) {
        if (i < static_cast<int>(vehicles.size()) && !vehicles[i].valid) {
            continue;
        }
        if (ETA[i].size() < 2) {
            continue;
        }

        double gt = 0.0;
        for (int j = 1; j < static_cast<int>(timeData[i].size()); ++j) {
            gt += timeData[i][j];
        }

        double pred = ETA[i].back().second - ETA[i].front().second;
        double diff = pred - gt;

        mae += abs(diff);
        mse += diff * diff;

        if (gt > 0) {
            mape += abs(diff) / gt;
        }

        validCnt++;
    }

    if (validCnt == 0) {
        cout << "[CycleAware Eval] No valid ETA result for evaluation." << endl;
        return 0.0f;
    }

    mse /= validCnt;
    mae /= validCnt;
    rmse = sqrt(mse);
    mape = (mape / validCnt) * 100.0;

    cout << "[CycleAware Eval] valid evaluated routes: " << validCnt << endl;
    cout << "[CycleAware Eval] invalid vehicles skipped: " << invalidVehicleCount << endl;
    cout << "[CycleAware Eval] MSE: " << mse << endl;
    cout << "[CycleAware Eval] MAE: " << mae << endl;
    cout << "[CycleAware Eval] RMSE: " << rmse << endl;
    cout << "[CycleAware Eval] MAPE: " << mape << "%" << endl;

    return static_cast<float>(mse);
}



namespace {

struct SumoEvalRecord {
    double predDuration = 0.0;
    double truthDuration = 0.0;
    double predArrival = 0.0;
    double truthArrival = 0.0;
    double truthWaitingTime = 0.0;
    double truthTimeLoss = 0.0;
    double truthRouteLength = 0.0;
    double predAvgSpeed = 0.0;
    double truthAvgSpeed = 0.0;
    double durationError = 0.0;
    double absDurationError = 0.0;
    double arrivalError = 0.0;
    double absArrivalError = 0.0;
    double speedError = 0.0;
    int numMovements = 0;
};

struct DistributionMetrics {
    double sortedMAE = 0.0;
    double sortedMSE = 0.0;
    double sortedRMSE = 0.0;
    double predP10 = 0.0;
    double truthP10 = 0.0;
    double diffP10 = 0.0;
    double predP50 = 0.0;
    double truthP50 = 0.0;
    double diffP50 = 0.0;
    double predP90 = 0.0;
    double truthP90 = 0.0;
    double diffP90 = 0.0;
    double predMean = 0.0;
    double truthMean = 0.0;
    double diffMean = 0.0;
};

double safe_divide(double numerator, double denominator)
{
    return denominator != 0.0 ? numerator / denominator : 0.0;
}

double percentile_sorted(vector<double> values, double percentile)
{
    if (values.empty()) return 0.0;
    sort(values.begin(), values.end());
    if (values.size() == 1) return values.front();

    const double rank = (percentile / 100.0) * static_cast<double>(values.size() - 1);
    const int lower = static_cast<int>(floor(rank));
    const int upper = static_cast<int>(ceil(rank));
    const double fraction = rank - static_cast<double>(lower);
    return values[lower] + (values[upper] - values[lower]) * fraction;
}

double mean_of(const vector<double>& values)
{
    if (values.empty()) return 0.0;
    double sum = 0.0;
    for (double v : values) sum += v;
    return sum / static_cast<double>(values.size());
}

string eval_sibling_path(const string& evalOutputPath, const string& filename)
{
    const size_t slash = evalOutputPath.find_last_of("/\\");
    if (slash == string::npos) return filename;
    return evalOutputPath.substr(0, slash + 1) + filename;
}

string csv_escape(const string& value)
{
    bool needsQuotes = false;
    string escaped;
    escaped.reserve(value.size());
    for (char c : value) {
        if (c == '"') {
            escaped += "\"\"";
            needsQuotes = true;
        } else {
            if (c == ',' || c == '\n' || c == '\r') needsQuotes = true;
            escaped += c;
        }
    }
    if (!needsQuotes) return escaped;
    return "\"" + escaped + "\"";
}

string join_int_vector(const vector<int>& values, const string& sep = "|")
{
    ostringstream out;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out << sep;
        out << values[i];
    }
    return out.str();
}

string join_string_vector(const vector<string>& values, const string& sep = "|")
{
    ostringstream out;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) out << sep;
        out << values[i];
    }
    return out.str();
}

string road_debug_name(const Graph& g, int roadID)
{
    auto it = g.roadIDToSumoEdgeStr.find(roadID);
    if (it != g.roadIDToSumoEdgeStr.end() && !it->second.empty()) return it->second;
    return "road_" + to_string(roadID);
}

string join_road_debug_names(const Graph& g, const vector<int>& roadIDs)
{
    vector<string> names;
    names.reserve(roadIDs.size());
    for (int roadID : roadIDs) names.push_back(road_debug_name(g, roadID));
    return join_string_vector(names, "|");
}

string turn_dir_to_string(TurnDir turn)
{
    switch (turn) {
        case TurnDir::Left: return "Left";
        case TurnDir::Straight: return "Straight";
        case TurnDir::Right: return "Right";
        case TurnDir::UTurn: return "UTurn";
        case TurnDir::Unknown: return "Unknown";
    }
    return "Unknown";
}

string movement_debug_name(const Graph& g, int movementID)
{
    if (movementID < 0 || movementID >= static_cast<int>(g.movements.size())) {
        return "movement_" + to_string(movementID) + ":invalid";
    }
    const Movement& movement = g.movements[movementID];
    const string fromEdge = road_debug_name(g, movement.fromRoadID);
    const string toEdge = road_debug_name(g, movement.toRoadID);
    return to_string(movementID) + ":" + fromEdge + "->" + toEdge + ":" + turn_dir_to_string(movement.turn);
}

string join_movement_debug_names(const Graph& g, const vector<int>& movementIDs)
{
    vector<string> names;
    names.reserve(movementIDs.size());
    for (int movementID : movementIDs) names.push_back(movement_debug_name(g, movementID));
    return join_string_vector(names, "|");
}

struct ExtremeRouteStats {
    int count = 0;
    double sumRelativeDurationError = 0.0;
    double maxRelativeDurationError = 0.0;
    double sumAbsDurationError = 0.0;
    double maxAbsDurationError = 0.0;

    vector<string> sampleVehicleIDs;
    vector<int> routeRoadIDs;
    vector<int> routeMovementIDs;
};

vector<pair<string, function<bool(const SumoEvalRecord&)>>> sumo_duration_bins()
{
    return {
        {"[0,300)", [](const SumoEvalRecord& r) { return r.truthDuration >= 0.0 && r.truthDuration < 300.0; }},
        {"[300,600)", [](const SumoEvalRecord& r) { return r.truthDuration >= 300.0 && r.truthDuration < 600.0; }},
        {"[600,900)", [](const SumoEvalRecord& r) { return r.truthDuration >= 600.0 && r.truthDuration < 900.0; }},
        {"[900,+inf)", [](const SumoEvalRecord& r) { return r.truthDuration >= 900.0; }}
    };
}

vector<pair<string, function<bool(const SumoEvalRecord&)>>> sumo_waiting_time_bins()
{
    return {
        {"0", [](const SumoEvalRecord& r) { return r.truthWaitingTime == 0.0; }},
        {"(0,30)", [](const SumoEvalRecord& r) { return r.truthWaitingTime > 0.0 && r.truthWaitingTime < 30.0; }},
        {"[30,60)", [](const SumoEvalRecord& r) { return r.truthWaitingTime >= 30.0 && r.truthWaitingTime < 60.0; }},
        {"[60,120)", [](const SumoEvalRecord& r) { return r.truthWaitingTime >= 60.0 && r.truthWaitingTime < 120.0; }},
        {"[120,+inf)", [](const SumoEvalRecord& r) { return r.truthWaitingTime >= 120.0; }}
    };
}

vector<pair<string, function<bool(const SumoEvalRecord&)>>> sumo_time_loss_bins()
{
    return {
        {"[0,30)", [](const SumoEvalRecord& r) { return r.truthTimeLoss >= 0.0 && r.truthTimeLoss < 30.0; }},
        {"[30,60)", [](const SumoEvalRecord& r) { return r.truthTimeLoss >= 30.0 && r.truthTimeLoss < 60.0; }},
        {"[60,120)", [](const SumoEvalRecord& r) { return r.truthTimeLoss >= 60.0 && r.truthTimeLoss < 120.0; }},
        {"[120,300)", [](const SumoEvalRecord& r) { return r.truthTimeLoss >= 120.0 && r.truthTimeLoss < 300.0; }},
        {"[300,+inf)", [](const SumoEvalRecord& r) { return r.truthTimeLoss >= 300.0; }}
    };
}

vector<pair<string, function<bool(const SumoEvalRecord&)>>> sumo_route_length_bins()
{
    return {
        {"[0,500)", [](const SumoEvalRecord& r) { return r.truthRouteLength >= 0.0 && r.truthRouteLength < 500.0; }},
        {"[500,1000)", [](const SumoEvalRecord& r) { return r.truthRouteLength >= 500.0 && r.truthRouteLength < 1000.0; }},
        {"[1000,3000)", [](const SumoEvalRecord& r) { return r.truthRouteLength >= 1000.0 && r.truthRouteLength < 3000.0; }},
        {"[3000,5000)", [](const SumoEvalRecord& r) { return r.truthRouteLength >= 3000.0 && r.truthRouteLength < 5000.0; }},
        {"[5000,+inf)", [](const SumoEvalRecord& r) { return r.truthRouteLength >= 5000.0; }}
    };
}

vector<pair<string, function<bool(const SumoEvalRecord&)>>> sumo_num_movements_bins()
{
    return {
        {"[0,2]", [](const SumoEvalRecord& r) { return r.numMovements >= 0 && r.numMovements <= 2; }},
        {"[3,5]", [](const SumoEvalRecord& r) { return r.numMovements >= 3 && r.numMovements <= 5; }},
        {"[6,10]", [](const SumoEvalRecord& r) { return r.numMovements >= 6 && r.numMovements <= 10; }},
        {"[11,+inf)", [](const SumoEvalRecord& r) { return r.numMovements >= 11; }}
    };
}

DistributionMetrics compute_distribution_metrics(const vector<SumoEvalRecord>& records)
{
    DistributionMetrics m;
    vector<double> pred;
    vector<double> truth;
    pred.reserve(records.size());
    truth.reserve(records.size());
    for (const auto& r : records) {
        pred.push_back(r.predDuration);
        truth.push_back(r.truthDuration);
    }
    sort(pred.begin(), pred.end());
    sort(truth.begin(), truth.end());
    if (pred.empty()) return m;

    double sumAbs = 0.0;
    double sumSq = 0.0;
    for (size_t i = 0; i < pred.size(); ++i) {
        const double diff = pred[i] - truth[i];
        sumAbs += abs(diff);
        sumSq += diff * diff;
    }
    m.sortedMAE = sumAbs / static_cast<double>(pred.size());
    m.sortedMSE = sumSq / static_cast<double>(pred.size());
    m.sortedRMSE = sqrt(m.sortedMSE);
    m.predP10 = percentile_sorted(pred, 10.0);
    m.truthP10 = percentile_sorted(truth, 10.0);
    m.diffP10 = m.predP10 - m.truthP10;
    m.predP50 = percentile_sorted(pred, 50.0);
    m.truthP50 = percentile_sorted(truth, 50.0);
    m.diffP50 = m.predP50 - m.truthP50;
    m.predP90 = percentile_sorted(pred, 90.0);
    m.truthP90 = percentile_sorted(truth, 90.0);
    m.diffP90 = m.predP90 - m.truthP90;
    m.predMean = mean_of(pred);
    m.truthMean = mean_of(truth);
    m.diffMean = m.predMean - m.truthMean;
    return m;
}

void write_distribution_metrics_csv(const string& path, const DistributionMetrics& m)
{
    ofstream out(path.c_str());
    if (!out) throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open distribution metrics output '" + path + "'");
    out << "metric,value\n";
    out << "sortedMAE," << m.sortedMAE << '\n';
    out << "sortedMSE," << m.sortedMSE << '\n';
    out << "sortedRMSE," << m.sortedRMSE << '\n';
    out << "predP10," << m.predP10 << '\n';
    out << "truthP10," << m.truthP10 << '\n';
    out << "diffP10," << m.diffP10 << '\n';
    out << "predP50," << m.predP50 << '\n';
    out << "truthP50," << m.truthP50 << '\n';
    out << "diffP50," << m.diffP50 << '\n';
    out << "predP90," << m.predP90 << '\n';
    out << "truthP90," << m.truthP90 << '\n';
    out << "diffP90," << m.diffP90 << '\n';
    out << "predMean," << m.predMean << '\n';
    out << "truthMean," << m.truthMean << '\n';
    out << "diffMean," << m.diffMean << '\n';
}

void append_group_metrics_csv(ostream& out,
        const string& groupType,
        const vector<pair<string, function<bool(const SumoEvalRecord&)>>>& bins,
        const vector<SumoEvalRecord>& records)
{
    for (const auto& bin : bins) {
        vector<double> absErrors;
        int n = 0;
        double sumTruth = 0.0, sumPred = 0.0, sumError = 0.0, sumAbs = 0.0, sumSq = 0.0, sumPct = 0.0;
        int pctN = 0;
        for (const auto& r : records) {
            if (!bin.second(r)) continue;
            ++n;
            sumTruth += r.truthDuration;
            sumPred += r.predDuration;
            sumError += r.durationError;
            sumAbs += r.absDurationError;
            sumSq += r.durationError * r.durationError;
            if (r.truthDuration > 0.0) { sumPct += r.absDurationError / r.truthDuration; ++pctN; }
            absErrors.push_back(r.absDurationError);
        }
        if (n == 0) continue;
        const double mse = sumSq / n;
        out << groupType << ',' << bin.first << ',' << n << ','
            << (sumTruth / n) << ',' << (sumPred / n) << ',' << (sumError / n) << ','
            << (sumAbs / n) << ',' << mse << ',' << sqrt(mse) << ','
            << ((pctN > 0) ? (sumPct / pctN) * 100.0 : 0.0) << ','
            << percentile_sorted(absErrors, 50.0) << ',' << percentile_sorted(absErrors, 90.0) << '\n';
    }
}

void write_grouped_metrics_csv(const string& path, const vector<SumoEvalRecord>& records)
{
    ofstream out(path.c_str());
    if (!out) throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open grouped metrics output '" + path + "'");
    out << "groupType,groupName,n,meanTruth,meanPred,bias,mae,mse,rmse,mape,medAE,p90AE\n";
    append_group_metrics_csv(out, "truthDuration", sumo_duration_bins(), records);
    append_group_metrics_csv(out, "truthWaitingTime", sumo_waiting_time_bins(), records);
    append_group_metrics_csv(out, "truthTimeLoss", sumo_time_loss_bins(), records);
    append_group_metrics_csv(out, "truthRouteLength", sumo_route_length_bins(), records);
    append_group_metrics_csv(out, "numMovements", sumo_num_movements_bins(), records);
}

} // namespace

float Graph::evaluate_sumo_tripinfo_truth(
        const vector<vector<pair<int, float>>>& ETA)
{
    const double epsilon = 1e-9;
    const double extremeRelativeDurationThreshold = 3.0;

    if (sumoTruthByVehicleID.empty()) {
        cout << "[SUMO Eval] No SUMO tripinfo truth loaded; skipping evaluation." << endl;
        return 0.0f;
    }

    const string summaryPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_summary.txt");
    const string groupedPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_grouped_metrics.csv");
    const string distributionPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_distribution_metrics.csv");
    const string extremePath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_extreme_duration_errors.csv");
    const string extremeRouteSummaryPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_extreme_route_summary.csv");

    ofstream csv;
    if (!evalOutputPath.empty()) {
        csv.open(evalOutputPath.c_str());
        if (!csv) {
            throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open CSV output '" + evalOutputPath + "'");
        }
        // predDepart is the actual first-road entry time recorded after the
        // first lane occupancy reservation succeeds, not the scheduled/input
        // departure time.
        csv << "vehicleID,predDepart,truthDepart,predArrival,truthArrival,"
            << "predDuration,truthDuration,durationError,absDurationError,"
            << "arrivalError,truthWaitingTime,truthTimeLoss,truthRouteLength,"
            << "durationErrorSigned,arrivalErrorSigned,absArrivalError,relativeDurationError,"
            << "absDurationErrorPerKm,predAvgSpeed,truthAvgSpeed,speedError,"
            << "numRoads,numMovements,validVehicle\n";
    }

    ofstream extremeCsv;
    if (!extremePath.empty()) {
        extremeCsv.open(extremePath.c_str());
        if (!extremeCsv) {
            throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open extreme duration output '" + extremePath + "'");
        }

        extremeCsv << "vehicleID,predDepart,truthDepart,predArrival,truthArrival,"
                   << "predDuration,truthDuration,durationError,absDurationError,"
                   << "relativeDurationError,relativeDurationErrorPercent,"
                   << "arrivalError,absArrivalError,truthWaitingTime,truthTimeLoss,truthRouteLength,"
                   << "predAvgSpeed,truthAvgSpeed,speedError,numRoads,numMovements,"
                   << "routeRoadIDs,routeRoadEdges,routeMovementIDs,routeMovementEdges\n";
    }

    unordered_map<string, ExtremeRouteStats> extremeRouteStatsByRoute;
    int extremeDurationErrorCount = 0;

    int comparedCount = 0;
    int missingTruthCount = 0;
    int invalidVehicleSkipped = 0;
    int etaMissingSkipped = 0;
    vector<SumoEvalRecord> evalRecords;
    vector<double> absDurationErrors;
    vector<double> absArrivalErrors;
    vector<double> durationErrors;
    vector<double> arrivalErrors;
    vector<double> predDurations;
    vector<double> truthDurations;
    vector<double> predArrivals;
    vector<double> truthArrivals;
    vector<double> predSpeeds;
    vector<double> truthSpeeds;
    vector<double> speedErrors;

    int n = static_cast<int>(ETA.size());
    n = max(n, static_cast<int>(vehicles.size()));
    n = max(n, static_cast<int>(queryDataRaw.size()));
    n = max(n, static_cast<int>(routeRoadID.size()));
    n = max(n, static_cast<int>(sumoVehicleIDs.size()));

    unordered_set<string> simulatedVehicleIDs;

    for (int i = 0; i < n; ++i) {
        string vehicleID = (i < static_cast<int>(sumoVehicleIDs.size()))
                         ? sumoVehicleIDs[i]
                         : ("veh_" + to_string(i));
        simulatedVehicleIDs.insert(vehicleID);

        auto truthIt = sumoTruthByVehicleID.find(vehicleID);
        if (truthIt == sumoTruthByVehicleID.end()) {
            ++missingTruthCount;
            continue;
        }

        bool validVehicle = true;
        if (i < static_cast<int>(vehicles.size())) validVehicle = vehicles[i].valid;
        if (!validVehicle) {
            ++invalidVehicleSkipped;
            continue;
        }

        if (i >= static_cast<int>(ETA.size()) || ETA[i].size() < 2) {
            ++etaMissingSkipped;
            continue;
        }

        const SumoTripInfoTruth &truth = truthIt->second;
        // ETA.front() is inserted only after actual first-road entry, so
        // predDuration excludes any initial storage-blocked departure delay.
        double predDepart = ETA[i].front().second;
        double predArrival = ETA[i].back().second;
        double predDuration = predArrival - predDepart;
        double truthDuration = truth.duration;
        double durationError = predDuration - truthDuration;
        double absDurationError = abs(durationError);
        double arrivalError = predArrival - truth.arrival;
        double absArrivalError = abs(arrivalError);
        double relativeDurationError = (truthDuration > 0.0) ? (absDurationError / truthDuration) : 0.0;
        double routeLengthKm = max(truth.routeLength / 1000.0, 1e-6);
        double absDurationErrorPerKm = absDurationError / routeLengthKm;
        double truthAvgSpeed = (truthDuration > epsilon) ? (truth.routeLength / truthDuration) : 0.0;
        double predAvgSpeed = (predDuration > epsilon) ? (truth.routeLength / predDuration) : 0.0;
        double speedError = predAvgSpeed - truthAvgSpeed;
        int numRoads = (i < static_cast<int>(routeRoadID.size()))
                     ? static_cast<int>(routeRoadID[i].size())
                     : 0;
        int numMovements = (i < static_cast<int>(routeMovementID.size()))
                         ? static_cast<int>(routeMovementID[i].size())
                         : 0;
        vector<int> vehicleRouteRoadIDs = (i < static_cast<int>(routeRoadID.size()))
                                        ? routeRoadID[i]
                                        : vector<int>{};
        vector<int> vehicleRouteMovementIDs = (i < static_cast<int>(routeMovementID.size()))
                                            ? routeMovementID[i]
                                            : vector<int>{};

        if (truthDuration > epsilon && relativeDurationError >= extremeRelativeDurationThreshold) {
            ++extremeDurationErrorCount;
            const string routeRoadIDsValue = join_int_vector(vehicleRouteRoadIDs, "|");
            const string routeRoadEdgesValue = join_road_debug_names(*this, vehicleRouteRoadIDs);
            const string routeMovementIDsValue = join_int_vector(vehicleRouteMovementIDs, "|");
            const string routeMovementEdgesValue = join_movement_debug_names(*this, vehicleRouteMovementIDs);
            const double relativeDurationErrorPercent = relativeDurationError * 100.0;

            if (extremeCsv) {
                extremeCsv << csv_escape(vehicleID) << ','
                           << predDepart << ','
                           << truth.depart << ','
                           << predArrival << ','
                           << truth.arrival << ','
                           << predDuration << ','
                           << truthDuration << ','
                           << durationError << ','
                           << absDurationError << ','
                           << relativeDurationError << ','
                           << relativeDurationErrorPercent << ','
                           << arrivalError << ','
                           << absArrivalError << ','
                           << truth.waitingTime << ','
                           << truth.timeLoss << ','
                           << truth.routeLength << ','
                           << predAvgSpeed << ','
                           << truthAvgSpeed << ','
                           << speedError << ','
                           << numRoads << ','
                           << numMovements << ','
                           << csv_escape(routeRoadIDsValue) << ','
                           << csv_escape(routeRoadEdgesValue) << ','
                           << csv_escape(routeMovementIDsValue) << ','
                           << csv_escape(routeMovementEdgesValue) << '\n';
            }

            ExtremeRouteStats& stats = extremeRouteStatsByRoute[routeRoadIDsValue];
            ++stats.count;
            stats.sumRelativeDurationError += relativeDurationError;
            stats.maxRelativeDurationError = max(stats.maxRelativeDurationError, relativeDurationError);
            stats.sumAbsDurationError += absDurationError;
            stats.maxAbsDurationError = max(stats.maxAbsDurationError, absDurationError);
            if (stats.sampleVehicleIDs.size() < 10) stats.sampleVehicleIDs.push_back(vehicleID);
            if (stats.routeRoadIDs.empty()) stats.routeRoadIDs = vehicleRouteRoadIDs;
            if (stats.routeMovementIDs.empty()) stats.routeMovementIDs = vehicleRouteMovementIDs;
        }

        comparedCount++;
        absDurationErrors.push_back(absDurationError);
        absArrivalErrors.push_back(absArrivalError);
        durationErrors.push_back(durationError);
        arrivalErrors.push_back(arrivalError);
        predDurations.push_back(predDuration);
        truthDurations.push_back(truthDuration);
        predArrivals.push_back(predArrival);
        truthArrivals.push_back(truth.arrival);
        predSpeeds.push_back(predAvgSpeed);
        truthSpeeds.push_back(truthAvgSpeed);
        speedErrors.push_back(speedError);
        evalRecords.push_back({predDuration, truthDuration, predArrival, truth.arrival,
                               truth.waitingTime, truth.timeLoss, truth.routeLength,
                               predAvgSpeed, truthAvgSpeed, durationError, absDurationError,
                               arrivalError, absArrivalError, speedError, numMovements});

        if (csv) {
            csv << vehicleID << ','
                << predDepart << ','
                << truth.depart << ','
                << predArrival << ','
                << truth.arrival << ','
                << predDuration << ','
                << truthDuration << ','
                << durationError << ','
                << absDurationError << ','
                << arrivalError << ','
                << truth.waitingTime << ','
                << truth.timeLoss << ','
                << truth.routeLength << ','
                << durationError << ','
                << arrivalError << ','
                << absArrivalError << ','
                << relativeDurationError << ','
                << absDurationErrorPerKm << ','
                << predAvgSpeed << ','
                << truthAvgSpeed << ','
                << speedError << ','
                << numRoads << ','
                << numMovements << ','
                << (validVehicle ? "true" : "false") << '\n';
        }
    }

    int truthNotSimulated = 0;
    for (const auto &kv : sumoTruthByVehicleID) {
        if (simulatedVehicleIDs.find(kv.first) == simulatedVehicleIDs.end()) {
            ++truthNotSimulated;
        }
    }

    const int totalTruthVehicles = static_cast<int>(sumoTruthByVehicleID.size());
    const int totalSimulatedVehicles = n;
    const double coverageByTruth = safe_divide(comparedCount, totalTruthVehicles);
    const double coverageBySimulation = safe_divide(comparedCount, totalSimulatedVehicles);

    double mse = 0.0, mae = 0.0, rmse = 0.0, mape = 0.0, biasDuration = 0.0;
    double medianAbsDurationError = 0.0, p90AbsDurationError = 0.0, p95AbsDurationError = 0.0;
    double mseArrival = 0.0, maeArrival = 0.0, rmseArrival = 0.0, biasArrival = 0.0;
    double medianAbsArrivalError = 0.0, p90AbsArrivalError = 0.0, p95AbsArrivalError = 0.0;
    double sumSqDuration = 0.0, sumAbsDuration = 0.0, sumPctDuration = 0.0;
    double sumSqArrival = 0.0, sumAbsArrival = 0.0;
    int pctCount = 0;
    for (size_t i = 0; i < durationErrors.size(); ++i) {
        sumSqDuration += durationErrors[i] * durationErrors[i];
        sumAbsDuration += absDurationErrors[i];
        if (truthDurations[i] > 0.0) { sumPctDuration += absDurationErrors[i] / truthDurations[i]; ++pctCount; }
        sumSqArrival += arrivalErrors[i] * arrivalErrors[i];
        sumAbsArrival += absArrivalErrors[i];
    }
    if (comparedCount > 0) {
        mse = sumSqDuration / comparedCount;
        mae = sumAbsDuration / comparedCount;
        rmse = sqrt(mse);
        mape = (pctCount > 0) ? (sumPctDuration / pctCount) * 100.0 : 0.0;
        biasDuration = mean_of(durationErrors);
        medianAbsDurationError = percentile_sorted(absDurationErrors, 50.0);
        p90AbsDurationError = percentile_sorted(absDurationErrors, 90.0);
        p95AbsDurationError = percentile_sorted(absDurationErrors, 95.0);
        mseArrival = sumSqArrival / comparedCount;
        maeArrival = sumAbsArrival / comparedCount;
        rmseArrival = sqrt(mseArrival);
        biasArrival = mean_of(arrivalErrors);
        medianAbsArrivalError = percentile_sorted(absArrivalErrors, 50.0);
        p90AbsArrivalError = percentile_sorted(absArrivalErrors, 90.0);
        p95AbsArrivalError = percentile_sorted(absArrivalErrors, 95.0);
    }

    const double meanPredDuration = mean_of(predDurations);
    const double meanTruthDuration = mean_of(truthDurations);
    const double meanDurationDiff = meanPredDuration - meanTruthDuration;
    const double relativeMeanDurationDiff = safe_divide(meanDurationDiff, meanTruthDuration);
    double sumPredDuration = 0.0, sumTruthDuration = 0.0;
    for (double v : predDurations) sumPredDuration += v;
    for (double v : truthDurations) sumTruthDuration += v;
    const double sumDurationDiff = sumPredDuration - sumTruthDuration;
    const double relativeSumDurationDiff = safe_divide(sumDurationDiff, sumTruthDuration);
    const double meanPredArrival = mean_of(predArrivals);
    const double meanTruthArrival = mean_of(truthArrivals);
    const double meanArrivalDiff = meanPredArrival - meanTruthArrival;
    const double meanPredSpeed = mean_of(predSpeeds);
    const double meanTruthSpeed = mean_of(truthSpeeds);
    const double speedBias = mean_of(speedErrors);
    double speedMAE = 0.0;
    for (double v : speedErrors) speedMAE += abs(v);
    speedMAE = safe_divide(speedMAE, static_cast<double>(speedErrors.size()));
    const double biasThreshold = 1.0;
    const string durationBiasDirection = (meanDurationDiff > biasThreshold) ? "slower" : ((meanDurationDiff < -biasThreshold) ? "faster" : "balanced");
    const string arrivalBiasDirection = (meanArrivalDiff > biasThreshold) ? "later" : ((meanArrivalDiff < -biasThreshold) ? "earlier" : "balanced");
    const DistributionMetrics distribution = compute_distribution_metrics(evalRecords);

    ostringstream summary;
    summary << "[Coverage]\n";
    summary << "simulatedVehicles=" << totalSimulatedVehicles << '\n';
    summary << "truthVehicles=" << totalTruthVehicles << '\n';
    summary << "comparedVehicles=" << comparedCount << '\n';
    summary << "missingTruth=" << missingTruthCount << '\n';
    summary << "invalidSkipped=" << invalidVehicleSkipped << '\n';
    summary << "etaMissingSkipped=" << etaMissingSkipped << '\n';
    summary << "truthNotSimulated=" << truthNotSimulated << '\n';
    summary << "coverageByTruth=" << coverageByTruth << '\n';
    summary << "coverageBySimulation=" << coverageBySimulation << "\n\n";

    if (comparedCount == 0) {
        summary << "[SUMO Eval Warning]\ncomparedCount == 0; no valid vehicles compared and accuracy metrics are unavailable.\n\n";
    }

    summary << "[Per-Vehicle Duration Error]\n";
    summary << "MAE=" << mae << '\n' << "MSE=" << mse << '\n' << "RMSE=" << rmse << '\n' << "MAPE=" << mape << '\n';
    summary << "Bias=" << biasDuration << '\n' << "Median absolute error=" << medianAbsDurationError << '\n';
    summary << "P90 absolute error=" << p90AbsDurationError << '\n' << "P95 absolute error=" << p95AbsDurationError << "\n\n";

    summary << "[Per-Vehicle Arrival Error]\n";
    summary << "MAE=" << maeArrival << '\n' << "MSE=" << mseArrival << '\n' << "RMSE=" << rmseArrival << '\n';
    summary << "Bias=" << biasArrival << '\n' << "Median absolute error=" << medianAbsArrivalError << '\n';
    summary << "P90 absolute error=" << p90AbsArrivalError << '\n' << "P95 absolute error=" << p95AbsArrivalError << "\n\n";

    summary << "[Aggregate Travel-Time Error]\n";
    summary << "meanPredDuration=" << meanPredDuration << '\n' << "meanTruthDuration=" << meanTruthDuration << '\n';
    summary << "meanDurationDiff=" << meanDurationDiff << '\n' << "relativeMeanDurationDiff=" << relativeMeanDurationDiff << '\n';
    summary << "sumPredDuration=" << sumPredDuration << '\n' << "sumTruthDuration=" << sumTruthDuration << '\n';
    summary << "sumDurationDiff=" << sumDurationDiff << '\n' << "relativeSumDurationDiff=" << relativeSumDurationDiff << '\n';
    summary << "meanPredArrival=" << meanPredArrival << '\n' << "meanTruthArrival=" << meanTruthArrival << '\n';
    summary << "meanArrivalDiff=" << meanArrivalDiff << "\n\n";

    summary << "[Speed Error]\n";
    summary << "meanPredSpeed=" << meanPredSpeed << '\n' << "meanTruthSpeed=" << meanTruthSpeed << '\n';
    summary << "speedBias=" << speedBias << '\n' << "speedMAE=" << speedMAE << "\n\n";

    summary << "[Bias Direction]\n";
    summary << "durationBiasDirection=" << durationBiasDirection << '\n';
    summary << "arrivalBiasDirection=" << arrivalBiasDirection << "\n\n";

    summary << "[Distribution-Level Duration Error]\n";
    summary << "sortedMAE=" << distribution.sortedMAE << '\n';
    summary << "sortedMSE=" << distribution.sortedMSE << '\n';
    summary << "sortedRMSE=" << distribution.sortedRMSE << '\n';
    summary << "predP10=" << distribution.predP10 << '\n' << "truthP10=" << distribution.truthP10 << '\n' << "diffP10=" << distribution.diffP10 << '\n';
    summary << "predP50=" << distribution.predP50 << '\n' << "truthP50=" << distribution.truthP50 << '\n' << "diffP50=" << distribution.diffP50 << '\n';
    summary << "predP90=" << distribution.predP90 << '\n' << "truthP90=" << distribution.truthP90 << '\n' << "diffP90=" << distribution.diffP90 << '\n';
    summary << "predMean=" << distribution.predMean << '\n' << "truthMean=" << distribution.truthMean << '\n' << "diffMean=" << distribution.diffMean << "\n\n";

    summary << "[Extreme Duration Error]\n";
    summary << "threshold=absDurationError/truthDuration >= " << extremeRelativeDurationThreshold << '\n';
    summary << "thresholdPercent=" << (extremeRelativeDurationThreshold * 100.0) << '\n';
    summary << "vehicleCount=" << extremeDurationErrorCount << '\n';
    summary << "routePatternCount=" << extremeRouteStatsByRoute.size() << '\n';
    summary << "extremeVehicleOutput=" << extremePath << '\n';
    summary << "extremeRouteSummaryOutput=" << extremeRouteSummaryPath << "\n";

    if (!extremeRouteSummaryPath.empty()) {
        vector<pair<string, ExtremeRouteStats>> routeRows(extremeRouteStatsByRoute.begin(), extremeRouteStatsByRoute.end());
        sort(routeRows.begin(), routeRows.end(), [](const auto& a, const auto& b) {
            if (a.second.count != b.second.count) return a.second.count > b.second.count;
            return a.second.maxRelativeDurationError > b.second.maxRelativeDurationError;
        });

        ofstream routeSummaryCsv(extremeRouteSummaryPath.c_str());
        if (!routeSummaryCsv) {
            throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open extreme route summary output '" + extremeRouteSummaryPath + "'");
        }
        routeSummaryCsv << "routeKey,count,meanRelativeDurationError,meanRelativeDurationErrorPercent,"
                        << "maxRelativeDurationError,maxRelativeDurationErrorPercent,"
                        << "meanAbsDurationError,maxAbsDurationError,sampleVehicleIDs,"
                        << "routeRoadIDs,routeRoadEdges,routeMovementIDs,routeMovementEdges\n";
        for (const auto& row : routeRows) {
            const string& routeKey = row.first;
            const ExtremeRouteStats& stats = row.second;
            const double count = static_cast<double>(stats.count);
            const double meanRelativeDurationError = safe_divide(stats.sumRelativeDurationError, count);
            const double meanAbsDurationError = safe_divide(stats.sumAbsDurationError, count);
            const string routeRoadIDsValue = join_int_vector(stats.routeRoadIDs, "|");
            const string routeRoadEdgesValue = join_road_debug_names(*this, stats.routeRoadIDs);
            const string routeMovementIDsValue = join_int_vector(stats.routeMovementIDs, "|");
            const string routeMovementEdgesValue = join_movement_debug_names(*this, stats.routeMovementIDs);
            routeSummaryCsv << csv_escape(routeKey) << ','
                            << stats.count << ','
                            << meanRelativeDurationError << ','
                            << (meanRelativeDurationError * 100.0) << ','
                            << stats.maxRelativeDurationError << ','
                            << (stats.maxRelativeDurationError * 100.0) << ','
                            << meanAbsDurationError << ','
                            << stats.maxAbsDurationError << ','
                            << csv_escape(join_string_vector(stats.sampleVehicleIDs, "|")) << ','
                            << csv_escape(routeRoadIDsValue) << ','
                            << csv_escape(routeRoadEdgesValue) << ','
                            << csv_escape(routeMovementIDsValue) << ','
                            << csv_escape(routeMovementEdgesValue) << '\n';
        }
    }

    cout << "[SUMO Eval] Overall Accuracy" << endl;
    cout << "[SUMO Eval] comparedCount: " << comparedCount << endl;
    cout << "[SUMO Eval] totalTruthVehicles: " << totalTruthVehicles << endl;
    cout << "[SUMO Eval] totalSimulatedVehicles: " << totalSimulatedVehicles << endl;
    cout << "[SUMO Eval] Coverage" << endl;
    cout << "[SUMO Eval] comparisonCoverage: " << coverageByTruth << endl;
    cout << "[SUMO Eval] invalidVehicleSkipped: " << invalidVehicleSkipped << endl;
    cout << "[SUMO Eval] missingTruthCount: " << missingTruthCount << endl;
    cout << "[SUMO Eval] etaMissingSkipped: " << etaMissingSkipped << endl;
    cout << "[SUMO Eval] truthNotSimulated: " << truthNotSimulated << endl;
    cout << "[SUMO Eval] compared vehicles: " << comparedCount << endl;
    cout << "[SUMO Eval] missing truth: " << missingTruthCount << endl;
    cout << "[SUMO Eval] invalid skipped: " << invalidVehicleSkipped << endl;
    cout << "[SUMO Eval] ETA missing skipped: " << etaMissingSkipped << endl;
    cout << "[SUMO Eval] truth not simulated: " << truthNotSimulated << endl;
    cout << "[SUMO Eval] MSE duration: " << mse << endl;
    cout << "[SUMO Eval] MAE duration: " << mae << endl;
    cout << "[SUMO Eval] RMSE duration: " << rmse << endl;
    cout << "[SUMO Eval] MAPE duration: " << mape << "%" << endl;
    cout << "[SUMO Eval] Bias duration: " << biasDuration << endl;
    cout << "[SUMO Eval] Median absolute duration error: " << medianAbsDurationError << endl;
    cout << "[SUMO Eval] P90 absolute duration error: " << p90AbsDurationError << endl;
    cout << "[SUMO Eval] P95 absolute duration error: " << p95AbsDurationError << endl;
    cout << "[SUMO Eval] MAE arrival: " << maeArrival << endl;
    cout << "[SUMO Eval] RMSE arrival: " << rmseArrival << endl;
    cout << "[SUMO Eval] Bias arrival: " << biasArrival << endl;
    cout << "[SUMO Eval] Extreme duration error threshold: absDurationError/truthDuration >= "
         << extremeRelativeDurationThreshold << " (" << extremeRelativeDurationThreshold * 100.0 << "%)" << endl;
    cout << "[SUMO Eval] Extreme duration error vehicles: "
         << extremeDurationErrorCount << endl;
    cout << "[SUMO Eval] Extreme duration error route patterns: "
         << extremeRouteStatsByRoute.size() << endl;
    if (!extremePath.empty()) {
        cout << "[SUMO Eval] Extreme vehicle output: " << extremePath << endl;
        cout << "[SUMO Eval] Extreme route summary output: " << extremeRouteSummaryPath << endl;
    }
    cout << summary.str();

    if (!evalOutputPath.empty()) {
        ofstream summaryOut(summaryPath.c_str());
        if (!summaryOut) throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open summary output '" + summaryPath + "'");
        summaryOut << summary.str();
        write_grouped_metrics_csv(groupedPath, evalRecords);
        write_distribution_metrics_csv(distributionPath, distribution);
        cout << "[SUMO Eval] CSV written to " << evalOutputPath << endl;
        cout << "[SUMO Eval] Summary written to " << summaryPath << endl;
        cout << "[SUMO Eval] Grouped metrics written to " << groupedPath << endl;
        cout << "[SUMO Eval] Distribution metrics written to " << distributionPath << endl;
        cout << "[SUMO Eval] Extreme vehicle output written to " << extremePath << endl;
        cout << "[SUMO Eval] Extreme route summary written to " << extremeRouteSummaryPath << endl;
    }

    return static_cast<float>(mse);
}


const VehicleType& Graph::getVehicleTypeForVehicle(int vehicleID) const {
    if (vehicleID >= 0 && vehicleID < static_cast<int>(vehicleTypeIDs.size())) {
        auto it = sumoVehicleTypes.find(vehicleTypeIDs[vehicleID]);
        if (it != sumoVehicleTypes.end()) return it->second;
    }
    auto carIt = sumoVehicleTypes.find("car");
    if (carIt != sumoVehicleTypes.end()) return carIt->second;

    static const VehicleType defaultVehicleType;
    return defaultVehicleType;
}

vector<int> Graph::parseLaneIndices(const vector<string>& lanes, int roadID) const {
    vector<int> parsed;
    int laneNum = 0;
    if (roadID >= 0 && roadID < static_cast<int>(roads.size())) {
        laneNum = max(1, roads[roadID].laneNum);
        if (!roads[roadID].laneFlow.empty()) laneNum = static_cast<int>(roads[roadID].laneFlow.size());
    }

    for (const string& lane : lanes) {
        if (lane.empty()) continue;
        char *end = nullptr;
        long value = strtol(lane.c_str(), &end, 10);
        if (end == lane.c_str() || *end != '\0') continue;
        int laneIndex = static_cast<int>(value);
        if (laneNum > 0) laneIndex = max(0, min(laneIndex, laneNum - 1));
        if (find(parsed.begin(), parsed.end(), laneIndex) == parsed.end()) {
            parsed.push_back(laneIndex);
        }
    }
    return parsed;
}

vector<int> Graph::laneIntersection(const vector<int>& a, const vector<int>& b) const {
    vector<int> result;
    for (int lane : a) {
        if (find(b.begin(), b.end(), lane) != b.end() &&
            find(result.begin(), result.end(), lane) == result.end()) {
            result.push_back(lane);
        }
    }
    return result;
}

void Graph::initializeRoadLaneStorage() {
    const VehicleType& representative = getVehicleTypeForVehicle(-1);
    const double vehicleSpace = max(1e-6, representative.length + representative.minGap);
    for (auto &road : roads) {
        int laneNum = max(1, road.laneNum);
        int capacityPerLane = max(1, static_cast<int>(floor(max(0.0, road.length) / vehicleSpace)));
        road.roadFlow = 0;
        road.laneFlow.assign(laneNum, 0);
        road.laneCapacity.assign(laneNum, capacityPerLane);
        road.laneOccupiedLength.assign(laneNum, 0.0);
        road.laneStorageLength.assign(laneNum, max(vehicleSpace, max(0.0, road.length)));
    }
}

// Lane-level storage selector, not signal logic: choose an allowed downstream lane
// with available count/length capacity, preferring the least occupied lane.
int Graph::chooseLeastOccupiedAvailableLane(int roadID, const vector<int>& candidateLanes, int vehicleID) const {
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return -1;
    const RoadSegment& road = roads[roadID];
    if (road.laneFlow.empty()) return -1;

    const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);
    const double requiredLength = max(1e-6, vt.length + vt.minGap);
    int bestLane = -1;
    int bestFlow = numeric_limits<int>::max();
    double bestOccupiedLength = numeric_limits<double>::max();

    for (int rawLane : candidateLanes) {
        if (rawLane < 0) continue;
        int lane = min(rawLane, static_cast<int>(road.laneFlow.size()) - 1);
        if (lane < 0) continue;
        int capacity = (lane < static_cast<int>(road.laneCapacity.size())) ? road.laneCapacity[lane] : 1;
        double occupiedLength = (lane < static_cast<int>(road.laneOccupiedLength.size())) ? road.laneOccupiedLength[lane] : 0.0;
        double storageLength = (lane < static_cast<int>(road.laneStorageLength.size()))
            ? road.laneStorageLength[lane]
            : max(0.0, road.length);
        bool hasCountCapacity = road.laneFlow[lane] < max(1, capacity);
        bool hasLengthCapacity = occupiedLength + requiredLength <= storageLength + 1e-9;
        if (!hasCountCapacity || !hasLengthCapacity) continue;
        if (road.laneFlow[lane] < bestFlow ||
            (road.laneFlow[lane] == bestFlow && occupiedLength < bestOccupiedLength)) {
            bestLane = lane;
            bestFlow = road.laneFlow[lane];
            bestOccupiedLength = occupiedLength;
        }
    }
    return bestLane;
}

// Flow accounting entry point: reserve increments roadFlow/laneFlow and occupied length.
// Failed discharge paths must not call this; it is the single source of truth for
// online road/lane flow features together with releaseLaneOccupancy.
void Graph::reserveLaneOccupancy(int vehicleID, int roadID, int laneIndex) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return;
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return;
    RoadSegment& road = roads[roadID];
    if (laneIndex < 0 || laneIndex >= static_cast<int>(road.laneFlow.size())) return;

    VehicleLabel& vehicle = vehicles[vehicleID];
    if (vehicle.occupiedRoadID >= 0 || vehicle.occupiedLaneIndex >= 0) {
        cout << "[LaneFlow Warning] reserveLaneOccupancy called for vehicle=" << vehicleID
             << " while it already occupies road=" << vehicle.occupiedRoadID
             << " lane=" << vehicle.occupiedLaneIndex
             << "; releasing old occupancy before reserving road=" << roadID
             << " lane=" << laneIndex << endl;
        releaseLaneOccupancy(vehicleID);
    }

    const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);
    const double occupiedLength = max(1e-6, vt.length + vt.minGap);
    road.roadFlow++;
    road.laneFlow[laneIndex]++;
    if (laneIndex < static_cast<int>(road.laneOccupiedLength.size())) {
        road.laneOccupiedLength[laneIndex] += occupiedLength;
    }
    vehicle.occupiedRoadID = roadID;
    vehicle.occupiedLaneIndex = laneIndex;
    vehicle.occupiedLength = occupiedLength;
}

// Flow accounting entry point: release decrements roadFlow/laneFlow and occupied length.
// Defensive warnings/checks here protect against invalid release or double-reserve
// without changing the dispatch rule itself.
void Graph::releaseLaneOccupancy(int vehicleID) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return;
    VehicleLabel& vehicle = vehicles[vehicleID];
    int roadID = vehicle.occupiedRoadID;
    int laneIndex = vehicle.occupiedLaneIndex;
    if (roadID >= 0 && roadID < static_cast<int>(roads.size())) {
        RoadSegment& road = roads[roadID];
        if (road.roadFlow > 0) road.roadFlow--;
        if (laneIndex >= 0 && laneIndex < static_cast<int>(road.laneFlow.size()) && road.laneFlow[laneIndex] > 0) {
            road.laneFlow[laneIndex]--;
        }
        if (laneIndex >= 0 && laneIndex < static_cast<int>(road.laneOccupiedLength.size())) {
            road.laneOccupiedLength[laneIndex] = max(0.0, road.laneOccupiedLength[laneIndex] - vehicle.occupiedLength);
        }
    }
    vehicle.occupiedRoadID = -1;
    vehicle.occupiedLaneIndex = -1;
    vehicle.occupiedLength = 0.0;
}

// Lane-level downstream storage check, not signal eligibility.
// Uses movement toRoad lanes and, when available, the next movement's from lanes so
// vehicles do not enter full or incompatible downstream lane storage.
bool Graph::hasDownstreamLaneStorage(int movementID, int vehicleID, int &chosenLane) {
    chosenLane = -1;
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return false;
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return false;

    const Movement& m = movements[movementID];
    int toRoadID = m.toRoadID;
    if (toRoadID < 0 || toRoadID >= static_cast<int>(roads.size())) return false;

    vector<int> toLanes = parseLaneIndices(m.toLanes, toRoadID);
    vector<int> candidateLanes = toLanes;
    const VehicleLabel& vehicle = vehicles[vehicleID];
    int nextMovementIndex = vehicle.roadIndex + 1;
    if (nextMovementIndex >= 0 && nextMovementIndex < static_cast<int>(vehicle.routeMovementIDs.size())) {
        int nextMovementID = vehicle.routeMovementIDs[nextMovementIndex];
        if (nextMovementID >= 0 && nextMovementID < static_cast<int>(movements.size())) {
            vector<int> nextFromLanes = parseLaneIndices(movements[nextMovementID].fromLanes, toRoadID);
            vector<int> intersected = laneIntersection(toLanes, nextFromLanes);
            candidateLanes = !intersected.empty() ? intersected : nextFromLanes;
        }
    }
    if (candidateLanes.empty()) candidateLanes = toLanes;
    if (candidateLanes.empty()) candidateLanes.push_back(0);

    chosenLane = chooseLeastOccupiedAvailableLane(toRoadID, candidateLanes, vehicleID);
    return chosenLane >= 0;
}

bool Graph::validateRoadLaneFlows() const {
    vector<int> expectedRoadFlow(roads.size(), 0);
    vector<vector<int>> expectedLaneFlow;
    expectedLaneFlow.reserve(roads.size());
    for (const auto& road : roads) {
        expectedLaneFlow.push_back(vector<int>(road.laneFlow.size(), 0));
    }

    for (const auto& vehicle : vehicles) {
        if (!vehicle.valid || vehicle.finished || vehicle.state == VehicleState::NotDeparted) continue;
        int roadID = vehicle.occupiedRoadID;
        int laneIndex = vehicle.occupiedLaneIndex;
        if (roadID < 0 || roadID >= static_cast<int>(roads.size())) {
            cout << "[LaneFlow Warning] active vehicle " << vehicle.vehicleID << " has no occupied road/lane." << endl;
            return false;
        }
        if (laneIndex < 0 || laneIndex >= static_cast<int>(expectedLaneFlow[roadID].size())) {
            cout << "[LaneFlow Warning] vehicle " << vehicle.vehicleID << " has invalid lane " << laneIndex
                 << " on road " << roadID << endl;
            return false;
        }
        expectedRoadFlow[roadID]++;
        expectedLaneFlow[roadID][laneIndex]++;
    }

    bool ok = true;
    vector<tuple<double, int, int, int, int>> congested;
    for (int roadID = 0; roadID < static_cast<int>(roads.size()); ++roadID) {
        const RoadSegment& road = roads[roadID];
        int laneSum = 0;
        for (int lane = 0; lane < static_cast<int>(road.laneFlow.size()); ++lane) {
            laneSum += road.laneFlow[lane];
            int expected = (roadID < static_cast<int>(expectedLaneFlow.size()) && lane < static_cast<int>(expectedLaneFlow[roadID].size()))
                ? expectedLaneFlow[roadID][lane]
                : 0;
            if (road.laneFlow[lane] != expected) {
                cout << "[LaneFlow Warning] laneFlow mismatch road=" << roadID << " lane=" << lane
                     << " actual=" << road.laneFlow[lane] << " expected=" << expected << endl;
                ok = false;
            }
            int capacity = (lane < static_cast<int>(road.laneCapacity.size())) ? road.laneCapacity[lane] : 0;
            if (capacity > 0) congested.emplace_back(safe_divide(static_cast<double>(road.laneFlow[lane]), static_cast<double>(capacity)), roadID, lane, road.laneFlow[lane], capacity);
            if (capacity > 0 && road.laneFlow[lane] > capacity) {
                cout << "[LaneFlow Warning] lane exceeds capacity road=" << roadID << " lane=" << lane
                     << " flow=" << road.laneFlow[lane] << " capacity=" << capacity << endl;
                ok = false;
            }
            double occupiedLength = (lane < static_cast<int>(road.laneOccupiedLength.size())) ? road.laneOccupiedLength[lane] : 0.0;
            double storageLength = (lane < static_cast<int>(road.laneStorageLength.size())) ? road.laneStorageLength[lane] : 0.0;
            if (storageLength > 0.0 && occupiedLength > storageLength + 1e-6) {
                cout << "[LaneFlow Warning] lane exceeds storage length road=" << roadID << " lane=" << lane
                     << " occupiedLength=" << occupiedLength << " storageLength=" << storageLength << endl;
                ok = false;
            }
        }
        if (road.roadFlow != laneSum || road.roadFlow != expectedRoadFlow[roadID]) {
            cout << "[LaneFlow Warning] roadFlow mismatch road=" << roadID
                 << " actual=" << road.roadFlow << " laneSum=" << laneSum
                 << " expected=" << expectedRoadFlow[roadID] << endl;
            ok = false;
        }
    }

    sort(congested.begin(), congested.end(), greater<tuple<double, int, int, int, int>>());
    int printed = 0;
    for (const auto& item : congested) {
        if (printed >= 5) break;
        double ratio;
        int roadID, lane, flow, capacity;
        tie(ratio, roadID, lane, flow, capacity) = item;
        if (flow <= 0) continue;
        cout << "[LaneFlow Debug] congested lane road=" << roadID << " lane=" << lane
             << " flow=" << flow << " capacity=" << capacity
             << " ratio=" << ratio << endl;
        ++printed;
    }
    return ok;
}

// Core cycle-aware signal-driven algorithm:
// 1) initialize vehicles and event queues;
// 2) advance time through signal/departure windows;
// 3) inside each stable signal window, dispatch movement candidates from the PQ;
// 4) keep real vehicle FIFO order in WaitingBuffer;
// 5) preserve movementTimeLabel across windows so discharge interval constraints survive.
vector<vector<pair<int, float>>> Graph::cycle_aware_signal_driven_records(
        vector<vector<int>> &Q, vector<vector<int>> &routeRoadIDInput) {
    // Initialization: clear per-run runtime state; static graph structures stay intact.
    vehicles.clear();
    ETA_result_cycle_aware.clear();
    finishedVehicleCount = 0;
    invalidVehicleCount = 0;
    usedMovementDischargeCapacity.clear();
    movementTimeLabel.assign(movements.size(), 0);
    movementPQVersion.assign(movements.size(), 0);
    movementBlockedByDownstream.assign(movements.size(), false);
    movementInDispatchPQ.assign(movements.size(), false);
    while (!signalEventPQ.empty()) signalEventPQ.pop();
    while (!departurePQ.empty()) departurePQ.pop();
    while (!dispatchPQ.empty()) dispatchPQ.pop();
    entryLaneDepartureQueues.clear();

    // Static route/movement setup for this run.
    this->routeRoadID = routeRoadIDInput;
    route_roadID_2_movementID();
    initializeMovementLaneDischargeCapacity();
    initializeRoadLaneStorage();
    initializeEntryLaneDepartureQueues();

    int simStartTime = 0;
    if (!Q.empty()) {
        simStartTime = Q[0][2];
        for (const auto &q : Q) simStartTime = min(simStartTime, q[2]);
    }

    // Event queue setup: departures and signal phase changes are independent streams.
    initialize_cycle_aware_vehicles(Q, routeRoadIDInput);
    initialize_signal_event_queue(simStartTime);

    int currentTime = simStartTime;
    int idleWindows = 0;
    const int maxIdleWindows = max(1000, static_cast<int>(vehicles.size()) * 10000);
    // Main event loop: advance from one departure/signal boundary to the next.
    while (!allVehiclesFinished()) {
        int nextTime = INF;
        if (!signalEventPQ.empty()) nextTime = min(nextTime, signalEventPQ.top().time);
        if (!departurePQ.empty()) nextTime = min(nextTime, departurePQ.top().time);
        if (hasPendingEntryDepartureQueues()) nextTime = min(nextTime, currentTime + 1);

        if (nextTime == INF) {
            // No external events remain. Keep always-open movements moving in small windows
            // rather than stopping early while queued vehicles are still active.
            nextTime = currentTime + 1;
            if (++idleWindows > maxIdleWindows) {
                cout << "[Error] no signal/departure events remain before all vehicles finished." << endl;
                break;
            }
        } else {
            idleWindows = 0;
        }

        if (nextTime < currentTime) nextTime = currentTime;
        if (currentTime < nextTime) {
            // Process the normal movement-dispatch window while signal state is stable.
            process_discharge_window(currentTime, nextTime);
        }

        currentTime = nextTime;
        processScheduledDeparturesUntil(currentTime);
        processEntryDepartureQueuesAtTime(currentTime);

        // Handle signal changes at the boundary after the just-closed dispatch window.
        while (!signalEventPQ.empty() && signalEventPQ.top().time == currentTime) {
            SignalEvent e = signalEventPQ.top();
            signalEventPQ.pop();
            handle_signal_change_event(e);
            int nextT = nextSignalChangeTime(e.signalID, currentTime + 1);
            if (nextT < INF) {
                signalEventPQ.push({nextT, e.signalID});
            }
        }
    }
    // Final ETA/flow consistency recording path.
    validateRoadLaneFlows();
    return ETA_result_cycle_aware;
}

// Initialization step: convert routeRoadID/routeMovementID into runtime VehicleLabel state.
// Invalid route/movement/buffer mappings are marked and skipped from normal evaluation;
// valid vehicles start as NotDeparted and enter through DepartureEvent processing.
void Graph::initialize_cycle_aware_vehicles(vector<vector<int>>& Q, vector<vector<int>>& routeRoadIDInput) {
    vehicles.clear();
    vehicles.resize(routeRoadIDInput.size());
    ETA_result_cycle_aware.assign(routeRoadIDInput.size(), {});
    finishedVehicleCount = 0;
    invalidVehicleCount = 0;
    usedMovementDischargeCapacity.clear();
    movementTimeLabel.assign(movements.size(), 0);
    movementPQVersion.assign(movements.size(), 0);
    movementBlockedByDownstream.assign(movements.size(), false);
    movementInDispatchPQ.assign(movements.size(), false);
    while (!departurePQ.empty()) departurePQ.pop();
    while (!dispatchPQ.empty()) dispatchPQ.pop();

    for (auto &b : waitingBuffers) {
        b.vehicleQueue.clear();
    }
    initializeRoadLaneStorage();
    initializeEntryLaneDepartureQueues();

    // Only true route/mapping errors are invalid. Vehicles with no lane storage
    // remain NotDeparted in FIFO entry-lane departure queues.
    auto mark_invalid = [&](VehicleLabel &vehicle) {
        vehicle.valid = false;
        vehicle.finished = true;
        vehicle.state = VehicleState::Finished;
        vehicles[vehicle.vehicleID] = vehicle;
        finishedVehicleCount++;
        invalidVehicleCount++;
    };

    for (int i = 0; i < static_cast<int>(routeRoadIDInput.size()); ++i) {
        VehicleLabel v;
        v.vehicleID = i;
        v.routeID = i;
        v.routeRoadIDs = routeRoadIDInput[i];
        if (i < static_cast<int>(routeMovementID.size())) v.routeMovementIDs = routeMovementID[i];
        v.roadIndex = 0;
        v.state = VehicleState::NotDeparted;

        if (v.routeRoadIDs.empty()) {
            mark_invalid(v);
            continue;
        }

        int departTime = (i < static_cast<int>(Q.size()) && Q[i].size() > 2) ? Q[i][2] : 0;
        v.scheduledDepartTime = departTime;
        v.currentRoadID = v.routeRoadIDs[0];
        v.arrivalTime = departTime;
        // First-road entry is an outside-system departure, not a signal-buffer
        // discharge. Delayed departure retries due to initial storage limits are
        // therefore not counted as vehicle-level signal waiting for has_waiting.
        v.hasWaitingBeforeCurrentRoad = false;
        v.lastDischargeHadWaiting = false;
        v.lastWaitingDuration = 0;

        if (v.currentRoadID < 0 || v.currentRoadID >= static_cast<int>(roads.size())) {
            mark_invalid(v);
            continue;
        }

        if (v.routeRoadIDs.size() > 1) {
            if (v.routeMovementIDs.empty()) {
                mark_invalid(v);
                continue;
            }
            int movementID = v.routeMovementIDs[0];
            if (movementID < 0 || movementID >= static_cast<int>(movements.size())) {
                mark_invalid(v);
                continue;
            }
            auto itRoad = roads[v.currentRoadID].movementIDToWaitingBufferID.find(movementID);
            if (itRoad == roads[v.currentRoadID].movementIDToWaitingBufferID.end()) {
                mark_invalid(v);
                continue;
            }
            v.currentMovementID = movementID;
            v.currentBufferID = itRoad->second;
            if (v.currentBufferID < 0 || v.currentBufferID >= static_cast<int>(waitingBuffers.size())) {
                mark_invalid(v);
                continue;
            }
        }

        vehicles[i] = v;
        departurePQ.push({departTime, i});
    }
}

// Event queue setup: schedule signal phase-change events only.
// Vehicle discharge attempts are handled by DispatchCandidate, not this queue.
void Graph::initialize_signal_event_queue(int simStartTime) {
    // 为每个受控信号注册“下一次状态切换”事件。
    while (!signalEventPQ.empty()) signalEventPQ.pop();
    for (const auto &signal : signals) {
        if (signal.alwaysOpen) continue;
        int nextT = nextSignalChangeTime(signal.signalID, simStartTime);
        if (nextT < INF) {
            signalEventPQ.push({nextT, signal.signalID});
        }
    }
}

// Heap hygiene for a new stable signal window.
// Rebuilds active movement candidates without resetting persistent movement labels
// or usedMovementDischargeCapacity; downstream-blocked movements remain inactive.
void Graph::rebuildActiveDispatchPQ(int currentTime, int windowEnd) {
    (void)windowEnd;
    // Rebuild only the heap contents; persistent movement labels/capacity history remain intact.
    while (!dispatchPQ.empty()) dispatchPQ.pop();
    if (movementTimeLabel.size() != movements.size()) movementTimeLabel.assign(movements.size(), currentTime);
    if (movementPQVersion.size() != movements.size()) movementPQVersion.assign(movements.size(), 0);
    if (movementBlockedByDownstream.size() != movements.size()) movementBlockedByDownstream.assign(movements.size(), false);
    movementInDispatchPQ.assign(movements.size(), false);

    for (const auto &m : movements) {
        int movementID = m.movementID;
        if (movementID < 0 || movementID >= static_cast<int>(movements.size())) continue;
        // Defensive guard: a stale candidate may remain after downstream-full
        // deactivation. It must not retry until storage is freed.
        if (movementBlockedByDownstream[movementID]) continue;
        int frontVehicleID = getFrontVehicleForMovement(movementID);
        if (frontVehicleID < 0) continue;
        int t = computeMovementAttemptTime(movementID, currentTime);
        scheduleMovementCandidate(movementID, t);
    }
}

void Graph::initializeEntryLaneDepartureQueues() {
    entryLaneDepartureQueues.clear();
    entryLaneDepartureQueues.resize(roads.size());
    for (int roadID = 0; roadID < static_cast<int>(roads.size()); ++roadID) {
        int laneCount = max(1, static_cast<int>(roads[roadID].laneFlow.size()));
        entryLaneDepartureQueues[roadID].resize(laneCount);
    }
}

bool Graph::hasPendingEntryDepartureQueues() const {
    for (const auto& roadQueues : entryLaneDepartureQueues) {
        for (const auto& laneQueue : roadQueues) {
            if (!laneQueue.empty()) return true;
        }
    }
    return false;
}

void Graph::processScheduledDeparturesUntil(int t) {
    while (!departurePQ.empty() && departurePQ.top().time <= t) {
        DepartureEvent e = departurePQ.top();
        departurePQ.pop();
        enqueueDepartingVehicle(e.vehicleID, e.time);
    }
}

void Graph::process_departures_until(int windowStart, int windowEnd) {
    (void)windowStart;
    processScheduledDeparturesUntil(windowEnd - 1);
}

bool Graph::enqueueDepartingVehicle(int vehicleID, int scheduledTime) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return false;
    VehicleLabel &v = vehicles[vehicleID];
    if (!v.valid || v.finished || v.state != VehicleState::NotDeparted) return false;
    if (v.queuedForEntry) {
        cout << "[EntryQueue Warning] vehicle=" << vehicleID << " already queued for entry road="
             << v.assignedEntryRoadID << " lane=" << v.assignedEntryLaneIndex << endl;
        return false;
    }
    if (v.routeRoadIDs.empty()) return false;

    int firstRoad = v.routeRoadIDs[0];
    if (firstRoad < 0 || firstRoad >= static_cast<int>(roads.size())) return false;
    if (entryLaneDepartureQueues.size() != roads.size()) initializeEntryLaneDepartureQueues();
    if (entryLaneDepartureQueues[firstRoad].empty()) entryLaneDepartureQueues[firstRoad].resize(max(1, static_cast<int>(roads[firstRoad].laneFlow.size())));

    vector<int> candidateLanes;
    int firstMovement = v.currentMovementID;
    if (v.routeRoadIDs.size() > 1) {
        if (firstMovement < 0 || firstMovement >= static_cast<int>(movements.size())) return false;
        candidateLanes = parseLaneIndices(movements[firstMovement].fromLanes, firstRoad);
    }
    if (candidateLanes.empty()) {
        int laneCount = max(1, static_cast<int>(entryLaneDepartureQueues[firstRoad].size()));
        for (int lane = 0; lane < laneCount; ++lane) candidateLanes.push_back(lane);
    }

    int laneCount = max(1, static_cast<int>(entryLaneDepartureQueues[firstRoad].size()));
    int chosenLane = 0;
    size_t bestQueueSize = numeric_limits<size_t>::max();
    int bestFlow = numeric_limits<int>::max();
    for (int rawLane : candidateLanes) {
        int lane = max(0, min(rawLane, laneCount - 1));
        size_t queueSize = entryLaneDepartureQueues[firstRoad][lane].size();
        int flow = (lane < static_cast<int>(roads[firstRoad].laneFlow.size())) ? roads[firstRoad].laneFlow[lane] : 0;
        if (queueSize < bestQueueSize || (queueSize == bestQueueSize && flow < bestFlow) ||
            (queueSize == bestQueueSize && flow == bestFlow && lane < chosenLane)) {
            chosenLane = lane;
            bestQueueSize = queueSize;
            bestFlow = flow;
        }
    }

    entryLaneDepartureQueues[firstRoad][chosenLane].push_back(vehicleID);
    v.scheduledDepartTime = scheduledTime;
    v.assignedEntryRoadID = firstRoad;
    v.assignedEntryLaneIndex = chosenLane;
    v.queuedForEntry = true;
    if (verboseTravelTimePrediction) {
        cout << "[EntryQueue] enqueue vehicle=" << vehicleID
             << " scheduledDepart=" << scheduledTime
             << " road=" << firstRoad
             << " lane=" << chosenLane
             << " queueSize=" << entryLaneDepartureQueues[firstRoad][chosenLane].size() << endl;
    }
    return true;
}

bool Graph::tryStartVehicleFromEntryQueue(int vehicleID, int actualDepartTime, int firstRoad, int laneIndex) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return false;
    VehicleLabel &v = vehicles[vehicleID];
    if (!v.valid || v.finished || v.state != VehicleState::NotDeparted) return false;
    if (firstRoad < 0 || firstRoad >= static_cast<int>(roads.size())) return false;
    RoadSegment& road = roads[firstRoad];
    if (laneIndex < 0 || laneIndex >= static_cast<int>(road.laneFlow.size())) return false;

    const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);
    const double requiredLength = max(1e-6, vt.length + vt.minGap);
    int capacity = (laneIndex < static_cast<int>(road.laneCapacity.size())) ? road.laneCapacity[laneIndex] : 1;
    double occupiedLength = (laneIndex < static_cast<int>(road.laneOccupiedLength.size())) ? road.laneOccupiedLength[laneIndex] : 0.0;
    double storageLength = (laneIndex < static_cast<int>(road.laneStorageLength.size())) ? road.laneStorageLength[laneIndex] : max(0.0, road.length);
    if (road.laneFlow[laneIndex] >= max(1, capacity) || occupiedLength + requiredLength > storageLength + 1e-9) {
        return false;
    }

    int firstMovement = v.currentMovementID;
    int firstBuffer = v.currentBufferID;
    if (v.routeRoadIDs.size() > 1) {
        if (firstMovement < 0 || firstMovement >= static_cast<int>(movements.size())) return false;
        if (firstBuffer < 0 || firstBuffer >= static_cast<int>(waitingBuffers.size())) return false;
    }

    reserveLaneOccupancy(vehicleID, firstRoad, laneIndex);
    if (ETA_result_cycle_aware[vehicleID].empty()) {
        ETA_result_cycle_aware[vehicleID].push_back({firstRoad, static_cast<float>(actualDepartTime)});
    } else if (verboseTravelTimePrediction) {
        cout << "[EntryQueue] duplicate first ETA ignored vehicle=" << vehicleID
             << " existingRoad=" << ETA_result_cycle_aware[vehicleID].front().first
             << " existingDepart=" << ETA_result_cycle_aware[vehicleID].front().second
             << " attemptedDepart=" << actualDepartTime << endl;
    }
    if (verboseTravelTimePrediction) {
        cout << "[EntryQueue] depart vehicle=" << vehicleID
             << " actualDepart=" << actualDepartTime
             << " road=" << firstRoad
             << " lane=" << laneIndex << endl;
    }

    v.queuedForEntry = false;
    v.hasWaitingBeforeCurrentRoad = false;
    v.lastDischargeHadWaiting = false;
    v.lastWaitingDuration = 0;
    v.arrivalTime = actualDepartTime + predictRoadTravelTime(firstRoad, vehicleID, firstMovement, actualDepartTime, laneIndex);

    if (v.routeRoadIDs.size() == 1) {
        releaseLaneOccupancy(vehicleID);
        recordFinalETA(vehicleID, v.arrivalTime);
        reactivateMovementsBlockedByRoad(firstRoad, actualDepartTime);
        return true;
    }

    v.state = VehicleState::WaitingAtIntersection;
    insertVehicleToBufferOrdered(firstBuffer, vehicleID);
    return true;
}

void Graph::processEntryDepartureQueuesAtTime(int t) {
    if (entryLaneDepartureQueues.size() != roads.size()) initializeEntryLaneDepartureQueues();
    for (int roadID = 0; roadID < static_cast<int>(entryLaneDepartureQueues.size()); ++roadID) {
        for (int laneIndex = 0; laneIndex < static_cast<int>(entryLaneDepartureQueues[roadID].size()); ++laneIndex) {
            deque<int>& q = entryLaneDepartureQueues[roadID][laneIndex];
            if (q.empty()) continue;
            int vehicleID = q.front();
            if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) {
                cout << "[EntryQueue Warning] dropping invalid vehicle=" << vehicleID
                     << " road=" << roadID << " lane=" << laneIndex << endl;
                q.pop_front();
                continue;
            }
            VehicleLabel& v = vehicles[vehicleID];
            if (!v.valid || v.finished || v.state != VehicleState::NotDeparted) {
                cout << "[EntryQueue Warning] dropping non-waiting vehicle=" << vehicleID
                     << " state=" << static_cast<int>(v.state)
                     << " road=" << roadID << " lane=" << laneIndex << endl;
                v.queuedForEntry = false;
                q.pop_front();
                continue;
            }
            if (tryStartVehicleFromEntryQueue(vehicleID, t, roadID, laneIndex)) {
                q.pop_front();
            }
        }
    }
}

bool Graph::departVehicle(int vehicleID, int departTime) {
    return enqueueDepartingVehicle(vehicleID, departTime);
}

// Core movement-dispatch window.
// Pop movement candidates, read the current FIFO front only after pop, compute the
// actual attempt time, classify blocks, and mutate queues/flows only on success.
namespace {
string joinMovementIDsForLog(const vector<int>& movementIDs) {
    ostringstream oss;
    for (size_t i = 0; i < movementIDs.size(); ++i) {
        if (i > 0) oss << ",";
        oss << movementIDs[i];
    }
    return oss.str();
}
}

// Core movement-dispatch window.
// Pop movement candidates, read the current FIFO front only after pop, compute the
// actual attempt time, classify blocks, and mutate queues/flows only on success.
void Graph::process_discharge_window(int windowStart, int windowEnd) {
    (void)windowStart;
    rebuildActiveDispatchPQ(windowStart, windowEnd);

    auto handleBlockedCandidate = [&](int movementID, int t, DischargeBlockReason reason, int frontVehicleID) {
        // Red signal: jump directly to next green instead of retrying every second.
        if (reason == DischargeBlockReason::RedSignal) {
            int nextT = nextGreenTimeForMovement(movementID, t);
            if (nextT < INF) {
                movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
                scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            }
            return;
        }

        // Capacity exhausted: move to the next discharge-capacity slot.
        if (reason == DischargeBlockReason::Capacity) {
            int nextT = nextAvailableCapacityTime(movementID, t);
            movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
            scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            return;
        }

        // Front vehicle has not reached the waiting-buffer label time yet.
        if (reason == DischargeBlockReason::NotArrived) {
            int nextT = (frontVehicleID >= 0 && frontVehicleID < static_cast<int>(vehicles.size()))
                ? vehicles[frontVehicleID].arrivalTime
                : t + 1;
            movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
            scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            return;
        }

        // Downstream lane storage is full: deactivate until a vehicle releases that road.
        if (reason == DischargeBlockReason::DownstreamFull) {
            deactivateMovementForDownstreamBlock(movementID);
        }
    };

    // Core loop: process one movement candidate at a time. The priority queue
    // orders ready movements by (timeLabel, firstCarArriveTime, movementID, version).
    while (!dispatchPQ.empty()) {
        DispatchCandidate c = dispatchPQ.top();

        if (c.timeLabel >= windowEnd) return;

        dispatchPQ.pop();
        if (!isDispatchCandidateValid(c)) continue;

        int movementID = c.movementID;
        movementInDispatchPQ[movementID] = false;

        int bufferID = getMovementBufferID(movementID);
        if (bufferID < 0 || waitingBuffers[bufferID].vehicleQueue.empty()) continue;

        int vehicleID = waitingBuffers[bufferID].vehicleQueue.front();
        if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) continue;

        int t = max(max(movementTimeLabel[movementID],
                        vehicles[vehicleID].arrivalTime),
                    c.timeLabel);

        if (t >= windowEnd) {
            scheduleMovementCandidate(movementID, t);
            continue;
        }

        if (t > c.timeLabel) {
            scheduleMovementCandidate(movementID, t);
            continue;
        }

        int frontVehicleID = -1;
        int chosenLane = -1;
        DischargeBlockReason reason =
            getDischargeBlockReason(movementID, t, frontVehicleID, chosenLane);

        if (reason != DischargeBlockReason::None) {
            handleBlockedCandidate(movementID, t, reason, frontVehicleID);
            continue;
        }

        int oldLabel = movementTimeLabel[movementID];
        DischargeResult result = dischargeOneVehicle(movementID, t);

        if (result.vehicleID >= 0) {
            handleSuccessfulDischargePostUpdate(
                movementID,
                t,
                oldLabel,
                bufferID,
                result
            );
        }
    }
}

vector<int> Graph::orderMovementsByDownstreamRoundRobin(
        int intersectionID,
        int toRoadID,
        const vector<int>& movementIDs
) {
    vector<int> ordered = movementIDs;
    if (ordered.empty()) return ordered;
    sort(ordered.begin(), ordered.end());
    ordered.erase(unique(ordered.begin(), ordered.end()), ordered.end());

    if (intersectionID < 0 || intersectionID >= static_cast<int>(intersections.size())) return ordered;
    auto it = intersections[intersectionID].roundRobinPointerByToRoad.find(toRoadID);
    if (it == intersections[intersectionID].roundRobinPointerByToRoad.end()) return ordered;

    auto lastServedIt = find(ordered.begin(), ordered.end(), it->second);
    if (lastServedIt == ordered.end()) return ordered;
    rotate(ordered.begin(), next(lastServedIt), ordered.end());
    return ordered;
}

void Graph::markDownstreamRoundRobinServed(int movementID) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    const Movement &m = movements[movementID];
    if (m.intersectionID < 0 || m.intersectionID >= static_cast<int>(intersections.size())) return;
    intersections[m.intersectionID].roundRobinPointerByToRoad[m.toRoadID] = movementID;
}

void Graph::handleSuccessfulDischargePostUpdate(
        int movementID,
        int t,
        int oldLabel,
        int bufferID,
        const DischargeResult& result
) {
    int nextLabel = nextAvailableCapacityTime(movementID, t);
    if (nextLabel < oldLabel) {
        cout << "[Dispatch Sanity Warning] movementTimeLabel would decrease movement=" << movementID
             << " old=" << oldLabel << " new=" << nextLabel << endl;
        nextLabel = oldLabel;
    }
    movementTimeLabel[movementID] = nextLabel;

    if (verboseTravelTimePrediction) {
        cout << "[Dispatch Success] movement=" << movementID
             << " vehicle=" << result.vehicleID
             << " dischargeTime=" << t
             << " oldLabel=" << oldLabel
             << " newLabel=" << movementTimeLabel[movementID]
             << " releasedRoad=" << result.releasedRoadID
             << " toRoad=" << result.toRoadID << endl;
    }

    // Same movement may still have a FIFO front; schedule its next capacity-safe attempt.
    if (bufferID >= 0 && bufferID < static_cast<int>(waitingBuffers.size())
            && !waitingBuffers[bufferID].vehicleQueue.empty()) {
        int nextFront = waitingBuffers[bufferID].vehicleQueue.front();
        int nextT = max(movementTimeLabel[movementID], vehicles[nextFront].arrivalTime);
        scheduleMovementCandidate(movementID, computeMovementAttemptTime(movementID, nextT));
    }

    // Vehicle entered a downstream road and will next wait on its next movement.
    if (!result.finished && result.nextMovementID >= 0) {
        int nextT = max(movementTimeLabel[result.nextMovementID], result.newArrivalTime);
        scheduleMovementCandidate(result.nextMovementID,
                                  computeMovementAttemptTime(result.nextMovementID, nextT));
    }

    // Released upstream storage can unblock movements that were deactivated by DownstreamFull.
    reactivateMovementsBlockedByRoad(result.releasedRoadID, t);
}

void Graph::initializeMovementLaneDischargeCapacity() {
    int missingLaneSideCount = 0;
    for (auto &movement : movements) {
        int upstreamLaneCount = static_cast<int>(movement.fromLanes.size());
        int downstreamLaneCount = static_cast<int>(movement.toLanes.size());
        if (upstreamLaneCount <= 0 || downstreamLaneCount <= 0) {
            movement.laneDischargeCapacity = 1;
            ++missingLaneSideCount;
            cout << "[Discharge Warning] movementID=" << movement.movementID
                 << " has incomplete lane-level connection data"
                 << " (fromLanes=" << upstreamLaneCount
                 << ", toLanes=" << downstreamLaneCount
                 << "); using laneDischargeCapacity=1." << endl;
            continue;
        }
        movement.laneDischargeCapacity = max(1, min(upstreamLaneCount, downstreamLaneCount));
    }
    cout << "[Discharge] lane discharge interval: " << max(1, defaultDischargeInterval) << "s" << endl;
    cout << "[Discharge] initialized movement lane capacities for " << movements.size()
         << " movements";
    if (missingLaneSideCount > 0) {
        cout << " (fallback capacity used for " << missingLaneSideCount << " movements)";
    }
    cout << endl;
}

bool Graph::isMovementActive(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return false;
    SignalState state = signalStateAtMovement(movementID, t);
    return state == SignalState::Green || state == SignalState::AlwaysOpen;
}

// Successful-discharge state transition.
// This is the only place that pops the real WaitingBuffer. It captures released
// road/lane storage, updates waiting features, moves flow accounting, predicts the
// downstream arrival label, and returns data for follow-up scheduling/reactivation.
DischargeResult Graph::dischargeOneVehicle(int movementID, int dischargeTime) {
    DischargeResult result;
    // Defensive checks: invalid movement/buffer/no storage means no state mutation. Failed
    // discharge attempts must not pop WaitingBuffer or update flow/capacity.
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return result;
    const Movement &m = movements[movementID];
    int bufferID = getMovementBufferID(movementID);
    if (bufferID < 0) return result;
    auto &b = waitingBuffers[bufferID];
    if (b.vehicleQueue.empty()) return result;

    int vehicleID = b.vehicleQueue.front();
    int chosenLane = -1;
    if (!hasDownstreamLaneStorage(movementID, vehicleID, chosenLane)) return result;

    VehicleLabel &v = vehicles[vehicleID];
    // Capture released storage before releasing occupancy so blocked upstream
    // movements can be reactivated after this successful discharge.
    result.releasedRoadID = v.occupiedRoadID;
    result.releasedLaneIndex = v.occupiedLaneIndex;

    // Feature update section: has_waiting describes signal-buffer waiting before
    // predicting travel time on the downstream road.
    const int waitingDuration = max(0, dischargeTime - v.arrivalTime);
    v.lastWaitingDuration = waitingDuration;
    v.lastDischargeHadWaiting = waitingDuration > 0;
    v.hasWaitingBeforeCurrentRoad = v.lastDischargeHadWaiting;

    if (verboseTravelTimePrediction) {
        cout << "[TravelTime] waiting feature"
             << " vehicleID=" << vehicleID
             << " movementID=" << movementID
             << " dischargeTime=" << dischargeTime
             << " arrivalTime=" << v.arrivalTime
             << " waitingDuration=" << waitingDuration
             << " has_waiting=" << (v.lastDischargeHadWaiting ? 1 : 0)
             << endl;
    }

    // Core state mutation section: successful discharge is the only path that pops
    // the real FIFO queue and consumes movement capacity.
    b.vehicleQueue.pop_front();
    // Flow accounting section: leave old road/lane before entering downstream storage.
    releaseLaneOccupancy(vehicleID);
    consumeDischargeCapacity(movementID, dischargeTime);

    v.roadIndex += 1;
    v.currentRoadID = m.toRoadID;

    result.vehicleID = vehicleID;
    result.movementID = movementID;
    result.intersectionID = m.intersectionID;
    result.fromRoadID = m.fromRoadID;
    result.toRoadID = m.toRoadID;
    result.dischargeTime = dischargeTime;

    // The model turn_type for an entered road is the movement that led into that road.
    // A future model can add next-movement features without changing this call shape.
    if (v.roadIndex >= static_cast<int>(v.routeRoadIDs.size()) - 1) {
        // ETA update section: final road prediction completes the route.
        v.arrivalTime = dischargeTime + predictRoadTravelTimeWithEnteringVehicle(
                m.toRoadID, vehicleID, movementID, dischargeTime, chosenLane);
        result.newArrivalTime = v.arrivalTime;
        v.currentMovementID = -1;
        v.currentBufferID = -1;
        result.finished = true;
        recordFinalETA(vehicleID, v.arrivalTime);
        return result;
    }

    if (v.roadIndex < static_cast<int>(v.routeMovementIDs.size())) {
        int nextMovementID = v.routeMovementIDs[v.roadIndex];
        auto itBuffer = roads[m.toRoadID].movementIDToWaitingBufferID.find(nextMovementID);
        if (itBuffer != roads[m.toRoadID].movementIDToWaitingBufferID.end()) {
            int nextBufferID = itBuffer->second;
            v.currentMovementID = nextMovementID;
            v.currentBufferID = nextBufferID;
            v.state = VehicleState::WaitingAtIntersection;
            // Choose/reserve downstream lane storage before computing the next arrival label.
            reserveLaneOccupancy(vehicleID, m.toRoadID, chosenLane);
            // ETA update section for the next waiting buffer.
            v.arrivalTime = dischargeTime + predictRoadTravelTime(m.toRoadID, vehicleID, movementID, dischargeTime, chosenLane);
            result.newArrivalTime = v.arrivalTime;
            insertVehicleToBufferOrdered(nextBufferID, vehicleID);
            result.nextMovementID = nextMovementID;
            result.nextBufferID = nextBufferID;
        } else {
            // Defensive boundary check: route references a missing next waiting buffer.
            v.valid = false;
            invalidVehicleCount++;
            v.state = VehicleState::Finished;
            result.finished = true;
            v.finished = true;
            finishedVehicleCount++;
        }
    } else {
        // Defensive boundary check: road/movement route indices are inconsistent.
        v.valid = false;
        invalidVehicleCount++;
        v.state = VehicleState::Finished;
        result.finished = true;
        v.finished = true;
        finishedVehicleCount++;
    }
    return result;
}

void Graph::pushCandidateIfPossible(int movementID, int currentTime, int windowEnd) {
    (void)windowEnd;
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    if (movementBlockedByDownstream.size() == movements.size() && movementBlockedByDownstream[movementID]) return;
    if (getFrontVehicleForMovement(movementID) < 0) return;
    int t = computeMovementAttemptTime(movementID, currentTime);
    scheduleMovementCandidate(movementID, t);
}

// Defensive stale-item guard for lazy PQ invalidation; not part of the dispatch priority rule.
bool Graph::isDispatchCandidateValid(const DispatchCandidate& c) {
    if (c.movementID < 0 || c.movementID >= static_cast<int>(movements.size())) return false;
    if (movementPQVersion.size() != movements.size()) return false;
    if (c.version != movementPQVersion[c.movementID]) {
        if (verboseTravelTimePrediction) {
            cout << "[Dispatch Stale] movement=" << c.movementID
                 << " candidateVersion=" << c.version
                 << " currentVersion=" << movementPQVersion[c.movementID]
                 << " timeLabel=" << c.timeLabel << endl;
        }
        return false;
    }
    if (movementBlockedByDownstream.size() == movements.size() && movementBlockedByDownstream[c.movementID]) return false;
    int currentFrontVehicleID = getFrontVehicleForMovement(c.movementID);
    if (currentFrontVehicleID < 0) return false;
    if (c.frontVehicleID != currentFrontVehicleID) return false;
    if (c.firstCarArriveTime != vehicles[currentFrontVehicleID].arrivalTime) return false;
    return true;
}

int Graph::getMovementBufferID(int movementID) const {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return -1;
    const Movement &m = movements[movementID];
    if (m.fromRoadID < 0 || m.fromRoadID >= static_cast<int>(roads.size())) return -1;
    auto it = roads[m.fromRoadID].movementIDToWaitingBufferID.find(movementID);
    if (it == roads[m.fromRoadID].movementIDToWaitingBufferID.end()) return -1;
    int bufferID = it->second;
    if (bufferID < 0 || bufferID >= static_cast<int>(waitingBuffers.size())) return -1;
    return bufferID;
}

int Graph::getFrontVehicleForMovement(int movementID) const {
    int bufferID = getMovementBufferID(movementID);
    if (bufferID < 0) return -1;
    const auto &q = waitingBuffers[bufferID].vehicleQueue;
    if (q.empty()) return -1;
    int vehicleID = q.front();
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return -1;
    const auto &v = vehicles[vehicleID];
    if (!v.valid || v.finished || v.state == VehicleState::Finished) return -1;
    if (v.currentMovementID != movementID || v.currentBufferID != bufferID) return -1;
    return vehicleID;
}

// Compute the earliest useful retry time by combining movement label, front-vehicle
// arrival, next green, and next capacity slot.
int Graph::computeMovementAttemptTime(int movementID, int lowerBoundTime) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return INF;
    if (movementTimeLabel.size() != movements.size()) movementTimeLabel.assign(movements.size(), 0);
    int t = max(movementTimeLabel[movementID], lowerBoundTime);
    int vehicleID = getFrontVehicleForMovement(movementID);
    if (vehicleID >= 0) t = max(t, vehicles[vehicleID].arrivalTime);
    int greenT = nextGreenTimeForMovement(movementID, t);
    if (greenT >= INF) return INF;
    t = max(t, greenT);
    t = nextAvailableCapacityTime(movementID, t);
    return t;
}

// Signal-state helper: jump to next green rather than repeatedly retrying each second.
// SignalEventPQ handles phase transitions; dispatchPQ handles movement discharge attempts.
int Graph::nextGreenTimeForMovement(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return INF;
    if (isMovementActive(movementID, t)) return t;
    const Movement &m = movements[movementID];
    if (m.alwaysOpen) return t;

    if (!m.tlID.empty()) {
        auto programIt = tlIDToSignalProgramID.find(m.tlID);
        if (programIt == tlIDToSignalProgramID.end()) return INF;
        const SignalProgram &p = signalPrograms[programIt->second];
        if (p.cycleLength <= 0 || p.phases.empty()) return INF;
        int baseK = (t - p.offset) / p.cycleLength;
        if (p.offset + baseK * p.cycleLength > t) --baseK;
        int best = INF;
        for (int k = baseK; k <= baseK + 2; ++k) {
            for (const auto &phase : p.phases) {
                if (m.linkIndex < 0 || m.linkIndex >= static_cast<int>(phase.state.size())) continue;
                char state = phase.state[m.linkIndex];
                if (!(state == 'G' || state == 'g' || state == 'O')) continue;
                int start = p.offset + k * p.cycleLength + phase.startTime;
                int end = p.offset + k * p.cycleLength + phase.endTime;
                if (end <= t) continue;
                best = min(best, max(t, start));
            }
        }
        return best;
    }

    if (m.signalID < 0 || m.signalID >= static_cast<int>(signals.size())) return t;
    const SignalController &s = signals[m.signalID];
    if (s.alwaysOpen) return t;
    if (s.cycleLength <= 0) return INF;
    int local = (t - s.offset) % s.cycleLength;
    if (local < 0) local += s.cycleLength;
    int delta = 0;
    if (s.greenStart <= s.greenEnd) {
        delta = (local < s.greenStart) ? (s.greenStart - local) : (s.cycleLength - local + s.greenStart);
    } else {
        // Wrapped green interval; if we reached this branch, local is in the red gap [greenEnd, greenStart).
        delta = s.greenStart - local;
    }
    return t + max(0, delta);
}

// Pure decision/classification helper: no FIFO or road-flow mutation here.
// Separates can/cannot-discharge checks from the rescheduling action chosen by
// process_discharge_window.
DischargeBlockReason Graph::getDischargeBlockReason(int movementID, int t, int &frontVehicleID, int &chosenLane) {
    frontVehicleID = -1;
    chosenLane = -1;
    // Invalid: defensive boundary check, no candidate can be evaluated.
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return DischargeBlockReason::Invalid;
    int bufferID = getMovementBufferID(movementID);
    if (bufferID < 0) return DischargeBlockReason::Invalid;
    // EmptyBuffer: robustness check; no movement candidate is possible now.
    if (waitingBuffers[bufferID].vehicleQueue.empty()) return DischargeBlockReason::EmptyBuffer;
    frontVehicleID = getFrontVehicleForMovement(movementID);
    if (frontVehicleID < 0) return DischargeBlockReason::Invalid;
    // NotArrived: the FIFO front has not reached its waiting-buffer label time.
    if (vehicles[frontVehicleID].arrivalTime > t) return DischargeBlockReason::NotArrived;
    // RedSignal: signal state blocks this movement at time t.
    if (!isMovementActive(movementID, t)) return DischargeBlockReason::RedSignal;
    // Capacity: this movement's per-slot discharge capacity is exhausted.
    if (!hasDischargeCapacity(movementID, t)) return DischargeBlockReason::Capacity;
    // DownstreamFull: lane storage prevents entering the next road.
    if (!hasDownstreamLaneStorage(movementID, frontVehicleID, chosenLane)) return DischargeBlockReason::DownstreamFull;
    return DischargeBlockReason::None;
}

// Movement-based scheduler: update movementTimeLabel and push a candidate label.
// version implements lazy invalidation for std::priority_queue. No vehicle is bound
// here and no WaitingBuffer is mutated.
void Graph::scheduleMovementCandidate(int movementID, int time) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    if (time >= INF) return;
    if (movementPQVersion.size() != movements.size()) movementPQVersion.assign(movements.size(), 0);
    if (movementTimeLabel.size() != movements.size()) movementTimeLabel.assign(movements.size(), 0);
    if (movementBlockedByDownstream.size() != movements.size()) movementBlockedByDownstream.assign(movements.size(), false);
    if (movementInDispatchPQ.size() != movements.size()) movementInDispatchPQ.assign(movements.size(), false);
    if (movementBlockedByDownstream[movementID]) {
        if (verboseTravelTimePrediction) {
            cout << "[Dispatch Sanity Warning] not scheduling downstream-blocked movement=" << movementID << endl;
        }
        return;
    }
    int frontVehicleID = getFrontVehicleForMovement(movementID);
    if (frontVehicleID < 0) return;

    int label = max(movementTimeLabel[movementID], time);
    movementTimeLabel[movementID] = label;
    int firstCarArriveTime = vehicles[frontVehicleID].arrivalTime;
    int version = ++movementPQVersion[movementID];
    dispatchPQ.push({label, firstCarArriveTime, movementID, frontVehicleID, version});
    movementInDispatchPQ[movementID] = true;

    if (verboseTravelTimePrediction) {
        cout << "[Dispatch Schedule] movement=" << movementID
             << " timeLabel=" << label
             << " firstCarArriveTime=" << firstCarArriveTime
             << " frontVehicle=" << frontVehicleID
             << " version=" << version << endl;
    }
}

void Graph::deactivateMovementForDownstreamBlock(int movementID) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    if (movementBlockedByDownstream.size() != movements.size()) movementBlockedByDownstream.assign(movements.size(), false);
    if (movementPQVersion.size() != movements.size()) movementPQVersion.assign(movements.size(), 0);
    if (movementInDispatchPQ.size() != movements.size()) movementInDispatchPQ.assign(movements.size(), false);
    movementBlockedByDownstream[movementID] = true;
    movementInDispatchPQ[movementID] = false;
    ++movementPQVersion[movementID];
    if (verboseTravelTimePrediction) {
        cout << "[Dispatch DownstreamBlock] movement=" << movementID
             << " timeLabel=" << movementTimeLabel[movementID] << endl;
    }
}

void Graph::reactivateMovementsBlockedByRoad(int freedRoadID, int currentTime) {
    if (freedRoadID < 0 || movementBlockedByDownstream.size() != movements.size()) return;
    for (const auto &m : movements) {
        int movementID = m.movementID;
        if (movementID < 0 || movementID >= static_cast<int>(movements.size())) continue;
        if (m.toRoadID != freedRoadID) continue;
        if (!movementBlockedByDownstream[movementID]) continue;
        if (getFrontVehicleForMovement(movementID) < 0) continue;

        int t = computeMovementAttemptTime(movementID, currentTime);
        if (t >= INF) continue;
        movementBlockedByDownstream[movementID] = false;
        if (verboseTravelTimePrediction) {
            cout << "[Dispatch Reactivate] movement=" << movementID
                 << " freedRoad=" << freedRoadID
                 << " timeLabel=" << movementTimeLabel[movementID]
                 << " scheduledAt=" << t << endl;
        }
        scheduleMovementCandidate(movementID, t);
    }
}

// Per-movement per-slot discharge capacity check. Capacity usage persists across
// signal windows through usedMovementDischargeCapacity.
bool Graph::hasDischargeCapacity(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return false;
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    int used = usedMovementDischargeCapacity[make_tuple(movementID, slot)];
    int cap = max(1, movements[movementID].laneDischargeCapacity);
    return used < cap;
}

// Capacity accounting for a successful discharge in this movement/time slot.
void Graph::consumeDischargeCapacity(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    usedMovementDischargeCapacity[make_tuple(movementID, slot)]++;
}

// Capacity rescheduling helper: advance to the next slot that still has capacity.
int Graph::nextAvailableCapacityTime(int movementID, int t) {
    int interval = max(1, defaultDischargeInterval);
    int cur = t;
    while (!hasDischargeCapacity(movementID, cur)) {
        cur = ((cur / interval) + 1) * interval;
    }
    return cur;
}

int Graph::encodeTurnDirForModel(TurnDir turn) const {
    switch (turn) {
        case TurnDir::Left: return 1;
        case TurnDir::Straight: return 2;
        case TurnDir::Right: return 3;
        case TurnDir::UTurn: return 4;
        case TurnDir::Unknown: return 0;
    }
    return 0;
}

// Future model-extension boundary: construct the intentionally basic feature vector
// used by TABLE/MODEL/KINEMATIC travel-time predictors. Avoid adding advanced graph
// neighbor features here unless intentionally extending the model contract.
BasicRoadModelFeatures Graph::buildBasicRoadModelFeatures(
        int roadID,
        int vehicleID,
        int movementID,
        int currentTime,
        int preferredLaneIndex) const {
    BasicRoadModelFeatures features;
    features.time = currentTime;
    features.vehicleID = vehicleID;
    features.roadID = roadID;
    features.movementID = movementID;

    if (vehicleID >= 0 && vehicleID < static_cast<int>(vehicles.size())) {
        const VehicleLabel& vehicle = vehicles[vehicleID];
        features.has_waiting = vehicle.lastDischargeHadWaiting ? 1 : 0;
        features.waiting_duration = vehicle.lastWaitingDuration;
    } else {
        features.has_waiting = 0;
        features.waiting_duration = 0;
    }

    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) {
        const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);
        features.vehicle_length = vt.length;
        features.vehicle_min_gap = vt.minGap;
        return features;
    }

    const RoadSegment& road = roads[roadID];
    features.road_length = max(0.0, road.length);
    features.lane_num = max(1, road.laneNum);
    if (!road.laneFlow.empty()) features.lane_num = max(1, static_cast<int>(road.laneFlow.size()));
    features.speed_limit = road.speedLimit;
    features.road_flow = road.roadFlow;

    int laneIndex = -1;
    if (preferredLaneIndex >= 0 && preferredLaneIndex < static_cast<int>(road.laneFlow.size())) {
        laneIndex = preferredLaneIndex;
    } else if (vehicleID >= 0 && vehicleID < static_cast<int>(vehicles.size())) {
        const VehicleLabel& vehicle = vehicles[vehicleID];
        if (vehicle.occupiedRoadID == roadID &&
            vehicle.occupiedLaneIndex >= 0 &&
            vehicle.occupiedLaneIndex < static_cast<int>(road.laneFlow.size())) {
            laneIndex = vehicle.occupiedLaneIndex;
        }
    }
    features.laneIndex = laneIndex;

    if (laneIndex >= 0) {
        features.lane_flow = road.laneFlow[laneIndex];
        if (laneIndex < static_cast<int>(road.laneCapacity.size())) {
            features.lane_capacity = road.laneCapacity[laneIndex];
        }
        if (laneIndex < static_cast<int>(road.laneOccupiedLength.size())) {
            features.lane_occupied_length = road.laneOccupiedLength[laneIndex];
        }
    } else {
        features.lane_flow = road.laneFlow.empty() ? road.roadFlow : *max_element(road.laneFlow.begin(), road.laneFlow.end());
        features.lane_capacity = 0;
        for (int capacity : road.laneCapacity) features.lane_capacity += capacity;
        features.lane_occupied_length = 0.0;
        for (double occupiedLength : road.laneOccupiedLength) features.lane_occupied_length += occupiedLength;
    }

    if (movementID >= 0 && movementID < static_cast<int>(movements.size())) {
        features.turn_type = encodeTurnDirForModel(movements[movementID].turn);
    }

    const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);
    features.vehicle_length = vt.length;
    features.vehicle_min_gap = vt.minGap;
    return features;
}

BasicRoadModelFeatures Graph::buildBasicRoadModelFeaturesWithEnteringVehicle(
        int roadID,
        int vehicleID,
        int movementID,
        int currentTime,
        int preferredLaneIndex) const {
    BasicRoadModelFeatures features = buildBasicRoadModelFeatures(
            roadID, vehicleID, movementID, currentTime, preferredLaneIndex);

    features.road_flow += 1;
    features.lane_flow += 1;

    if (verboseTravelTimePrediction) {
        cout << "[TravelTime FinalRoad] vehicle=" << vehicleID
             << " road=" << roadID
             << " lane=" << preferredLaneIndex
             << " adjustedRoadFlow=" << features.road_flow
             << " adjustedLaneFlow=" << features.lane_flow
             << endl;
    }

    return features;
}

int Graph::predictRoadTravelTimeFromFeatures(
        int roadID,
        const BasicRoadModelFeatures& features) {
    if (enableBasicFeatureLogging) {
        basicFeatureSnapshots.push_back(features);
    }

    switch (travelTimeMode) {
        case TravelTimeMode::SPEED_NET:
            return predictRoadTravelTimeSpeedNet(roadID);
        case TravelTimeMode::MIN_TIME:
            return predictRoadTravelTimeMinTime(roadID);
        case TravelTimeMode::TABLE:
            return predictRoadTravelTimeTable(features);
        case TravelTimeMode::MODEL:
            return predictRoadTravelTimeModel(features);
        case TravelTimeMode::KINEMATIC:
            return predictRoadTravelTimeKinematic(features);
    }
    return predictRoadTravelTimeSpeedNet(roadID);
}

// Single-road travel-time prediction selector.
// --travel-time-mode affects only this road-time estimate, not movement dispatch
// scheduling, signal handling, FIFO order, or capacity rules.
int Graph::predictRoadTravelTime(
        int roadID,
        int vehicleID,
        int movementID,
        int currentTime,
        int preferredLaneIndex) {
    BasicRoadModelFeatures features = buildBasicRoadModelFeatures(
            roadID, vehicleID, movementID, currentTime, preferredLaneIndex);
    return predictRoadTravelTimeFromFeatures(roadID, features);
}

int Graph::predictRoadTravelTimeWithEnteringVehicle(
        int roadID,
        int vehicleID,
        int movementID,
        int currentTime,
        int preferredLaneIndex) {
    BasicRoadModelFeatures features = buildBasicRoadModelFeaturesWithEnteringVehicle(
            roadID, vehicleID, movementID, currentTime, preferredLaneIndex);
    return predictRoadTravelTimeFromFeatures(roadID, features);
}

int Graph::predictRoadTravelTime(int roadID, int vehicleID) {
    return predictRoadTravelTime(roadID, vehicleID, -1, 0, -1);
}

int Graph::predictRoadTravelTimeSpeedNet(int roadID) const {
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return 1;
    const RoadSegment& r = roads[roadID];
    double speed = std::max(0.1, r.speedLimit);
    double length = std::max(0.0, r.length);
    return std::max(1, static_cast<int>(std::round(length / speed)));
}

int Graph::predictRoadTravelTimeMinTime(int roadID) const {
    if (roadID >= 0 &&
        roadID < static_cast<int>(roads.size()) &&
        roads[roadID].minTravelTime > 0) {
        return std::max(1, static_cast<int>(std::round(roads[roadID].minTravelTime)));
    }
    return predictRoadTravelTimeSpeedNet(roadID);
}

RoadKey Graph::buildRoadKeyForPrediction(const BasicRoadModelFeatures& features) const {
    RoadKey key{};
    const double speed = std::max(0.1, features.speed_limit);
    const double length = std::max(0.0, features.road_length);

    key.lane_num = max(1, features.lane_num);
    key.speed_limit = static_cast<float>(std::round(speed));
    key.edge_length = static_cast<float>(std::round(length));
    key.driving_number = features.road_flow;
    key.delay_time = 0;
    key.lowSpee_time = 0;
    key.wait_time = 0;
    // Conservative lane-level congestion proxy: use the selected lane's flow, or the
    // max lane flow supplied by buildBasicRoadModelFeatures when no lane is selected.
    key.ratio = features.lane_flow;
    key.length_square = static_cast<int>(std::round(length * length));
    return key;
}

RoadKey Graph::buildRoadKeyForPrediction(int roadID, int vehicleID) const {
    return buildRoadKeyForPrediction(buildBasicRoadModelFeatures(roadID, vehicleID, -1, 0, -1));
}

SumoV1TravelTimeKey Graph::buildSumoV1TravelTimeKeyForPrediction(const BasicRoadModelFeatures& features) const {
    SumoV1TravelTimeKey key{};
    key.has_waiting = features.has_waiting ? 1 : 0;
    key.road_length_q = quantizeRoadLength(features.road_length);
    key.turn_type = features.turn_type;
    key.road_flow = max(0, features.road_flow);

    // SUMO_V1 table feature contract:
    // lane_flow is selected-lane vehicle count, matching TraCI slim output and
    // model_catching_sumo_v1.py. Do not derive it as road_flow / lane_num.
    double lane_flow_for_model = sumoV1LaneFlowForModel(features);
    key.lane_flow_q = quantizeLaneFlow(lane_flow_for_model);
    return key;
}

int Graph::predictRoadTravelTimeTable(const BasicRoadModelFeatures& features) {
    if (features.roadID < 0 || features.roadID >= static_cast<int>(roads.size())) return 1;

    if (travelTimeTableFormat == TravelTimeTableFormat::SUMO_V1) {
        SumoV1TravelTimeKey key = buildSumoV1TravelTimeKeyForPrediction(features);
        auto it = sumoV1TravelTimeTable.find(key);
        if (it != sumoV1TravelTimeTable.end()) {
            ++travelTimeTableHit;
            return std::max(1, static_cast<int>(std::round(it->second)));
        }

        ++travelTimeTableMiss;
        if (verboseTravelTimePrediction) {
            double lane_flow_for_model = sumoV1LaneFlowForModel(features);
            cout << "[TravelTime] SUMO_V1 TABLE miss roadID=" << features.roadID
                 << " key={has_waiting=" << key.has_waiting
                 << ", road_length_q=" << key.road_length_q
                 << ", turn_type=" << key.turn_type
                 << ", road_flow=" << key.road_flow
                 << ", lane_flow_q=" << key.lane_flow_q
                 << "} raw={has_waiting=" << features.has_waiting
                 << ", road_length=" << features.road_length
                 << ", turn_type=" << features.turn_type
                 << ", road_flow=" << features.road_flow
                 << ", lane_flow_for_model=" << lane_flow_for_model
                 << ", lane_flow=" << features.lane_flow
                 << ", lane_num=" << features.lane_num
                 << ", laneIndex=" << features.laneIndex
                 << "}" << endl;
        }

        return fallbackToSpeedNet
            ? predictRoadTravelTimeSpeedNet(features.roadID)
            : predictRoadTravelTimeMinTime(features.roadID);
    }

    RoadKey key = buildRoadKeyForPrediction(features);
    auto it = dictionary.find(key);
    if (it != dictionary.end()) {
        ++travelTimeTableHit;
        return std::max(1, static_cast<int>(std::round(it->second)));
    }

    ++travelTimeTableMiss;
    if (verboseTravelTimePrediction) {
        cout << "[TravelTime] TABLE miss roadID=" << features.roadID << endl;
    }

    return fallbackToSpeedNet
        ? predictRoadTravelTimeSpeedNet(features.roadID)
        : predictRoadTravelTimeMinTime(features.roadID);
}

int Graph::predictRoadTravelTimeTable(int roadID, int vehicleID) {
    return predictRoadTravelTimeTable(buildBasicRoadModelFeatures(roadID, vehicleID, -1, 0, -1));
}

int Graph::predictRoadTravelTimeKinematic(const BasicRoadModelFeatures& features) const {
    if (features.roadID < 0 || features.roadID >= static_cast<int>(roads.size())) return 1;

    const double L = max(0.0, features.road_length);
    const double vRoad = max(0.1, features.speed_limit);
    const double vVeh = max(0.1, getVehicleTypeForVehicle(features.vehicleID).maxSpeed);
    double v = min(vRoad, vVeh);
    v = v * max(0.7, 1.0 - 0.1 * getVehicleTypeForVehicle(features.vehicleID).sigma);
    const double a = max(0.1, getVehicleTypeForVehicle(features.vehicleID).accel);
    const double dAcc = v * v / (2.0 * a);

    double tFree = 0.0;
    if (L <= dAcc) {
        tFree = sqrt(2.0 * L / a);
    } else {
        tFree = v / a + (L - dAcc) / v;
    }

    const int occupancy = features.road_flow;
    const double vehicleSpace = max(1e-6, features.vehicle_length + features.vehicle_min_gap);
    int capacity = static_cast<int>(floor(L * max(1, features.lane_num) / vehicleSpace));
    capacity = max(1, capacity);
    double rho = safe_divide(static_cast<double>(occupancy), static_cast<double>(capacity));
    rho = max(0.0, min(0.95, rho));
    const double alpha = max(0.0, kinematicCongestionAlpha);
    const double t = tFree * (1.0 + alpha * rho);
    return max(1, static_cast<int>(round(t)));
}

int Graph::predictRoadTravelTimeKinematic(int roadID, int vehicleID) const {
    return predictRoadTravelTimeKinematic(buildBasicRoadModelFeatures(roadID, vehicleID, -1, 0, -1));
}

// Future MODEL extension stub/fallback boundary.
// Currently only exposes/logs BasicRoadModelFeatures; no external predictor is wired.
bool Graph::queryExternalTravelTimeModel(const BasicRoadModelFeatures& features, double& predictedTime) {
    predictedTime = -1.0;
    if (verboseTravelTimePrediction) {
        cout << "[TravelTime] MODEL features"
             << " road_length=" << features.road_length
             << " turn_type=" << features.turn_type
             << " road_flow=" << features.road_flow
             << " lane_flow=" << features.lane_flow
             << " lane_num=" << features.lane_num
             << " speed_limit=" << features.speed_limit
             << " lane_capacity=" << features.lane_capacity
             << " lane_occupied_length=" << features.lane_occupied_length
             << " has_waiting=" << features.has_waiting
             << " waiting_duration=" << features.waiting_duration
             << endl;
    }
    // TODO: future implementation. This can later use socket / Python service / embedded model.
    return false;
}

int Graph::predictRoadTravelTimeModel(const BasicRoadModelFeatures& features) {
    double pred = -1.0;
    if (queryExternalTravelTimeModel(features, pred) && pred > 0) {
        return std::max(1, static_cast<int>(std::round(pred)));
    }

    if (!modelWarningPrinted) {
        cout << "[TravelTime] MODEL mode is selected but no external model is implemented; using fallback." << endl;
        modelWarningPrinted = true;
    }

    return fallbackToSpeedNet
        ? predictRoadTravelTimeSpeedNet(features.roadID)
        : predictRoadTravelTimeMinTime(features.roadID);
}

int Graph::predictRoadTravelTimeModel(int roadID, int vehicleID) {
    return predictRoadTravelTimeModel(buildBasicRoadModelFeatures(roadID, vehicleID, -1, 0, -1));
}

// Debug/export path only: writes captured BasicRoadModelFeatures snapshots.
// This is not part of the core simulation semantics.
void Graph::exportBasicFeatureSnapshots(const string& path) const {
    ofstream out(path);
    if (!out.is_open()) {
        cout << "[TravelTime] failed to open basic feature snapshot CSV: " << path << endl;
        return;
    }
    out << "time,vehicleID,roadID,movementID,laneIndex,"
        << "road_length,turn_type,road_flow,lane_flow,"
        << "lane_num,speed_limit,vehicle_length,vehicle_min_gap,lane_capacity,lane_occupied_length,"
        << "has_waiting,waiting_duration\n";
    for (const auto& f : basicFeatureSnapshots) {
        out << f.time << ','
            << f.vehicleID << ','
            << f.roadID << ','
            << f.movementID << ','
            << f.laneIndex << ','
            << f.road_length << ','
            << f.turn_type << ','
            << f.road_flow << ','
            << f.lane_flow << ','
            << f.lane_num << ','
            << f.speed_limit << ','
            << f.vehicle_length << ','
            << f.vehicle_min_gap << ','
            << f.lane_capacity << ','
            << f.lane_occupied_length << ','
            << f.has_waiting << ','
            << f.waiting_duration << '\n';
    }
}

void Graph::insertVehicleToBufferOrdered(int bufferID, int vehicleID) {
    if (bufferID < 0 || bufferID >= waitingBuffers.size()) return;
    if (vehicleID < 0 || vehicleID >= vehicles.size()) return;
    auto &q = waitingBuffers[bufferID].vehicleQueue;
    int at = vehicles[vehicleID].arrivalTime;
    auto pos = q.end();
    for (auto it = q.begin(); it != q.end(); ++it) {
        if (vehicles[*it].arrivalTime > at) {
            pos = it;
            break;
        }
    }
    q.insert(pos, vehicleID);
}

void Graph::handle_signal_change_event(const SignalEvent& e) {
    if (e.signalID < 0 || e.signalID >= static_cast<int>(signals.size())) return;
    int movementID = signals[e.signalID].movementID;
    signals[e.signalID].currentState = signalStateAtMovement(movementID, e.time);
}

// Signal helper: find the next phase boundary for SignalEvent scheduling.
// SignalEventPQ controls phase transitions; dispatchPQ controls vehicle discharge attempts.
int Graph::nextSignalChangeTime(int signalID, int afterTime) {
    if (signalID < 0 || signalID >= static_cast<int>(signals.size())) return INF;
    const SignalController &s = signals[signalID];
    int movementID = s.movementID;
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return INF;
    const Movement &m = movements[movementID];
    if (m.alwaysOpen || m.tlID.empty()) return INF;

    auto programIt = tlIDToSignalProgramID.find(m.tlID);
    if (programIt == tlIDToSignalProgramID.end()) return INF;
    const SignalProgram &p = signalPrograms[programIt->second];
    if (p.cycleLength <= 0 || p.phases.empty()) return INF;

    int best = INF;
    int baseK = (afterTime - p.offset) / p.cycleLength;
    for (int k = baseK - 1; k <= baseK + 2; ++k) {
        for (const auto &phase : p.phases) {
            int start = p.offset + k * p.cycleLength + phase.startTime;
            int end = p.offset + k * p.cycleLength + phase.endTime;
            if (start >= afterTime) best = min(best, start);
            if (end >= afterTime) best = min(best, end);
        }
    }
    return best;
}

bool Graph::allVehiclesFinished() const {
    return finishedVehicleCount >= static_cast<int>(vehicles.size());
}

void Graph::recordFinalETA(int vehicleID, int finalTime) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return;
    if (vehicleID >= static_cast<int>(ETA_result_cycle_aware.size())) return;

    VehicleLabel &v = vehicles[vehicleID];
    if (v.finished) return;

    ETA_result_cycle_aware[vehicleID].push_back({-1, static_cast<float>(finalTime)});
    v.arrivalTime = finalTime;
    v.finished = true;
    v.state = VehicleState::Finished;
    finishedVehicleCount++;
}
