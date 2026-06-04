
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

vector<int> Graph::Dij_vetex(int ID1, int ID2){

    /*
     * Description: Dijkstra’s algorithm to find the shortest path between two nodes.
     *
     * Parameters:
     * int ID1 -> source node.
     * int ID2 -> destination node.
     *
     * Return:
     * int d -> the shortest path between ID1 and ID2.
     */

    vector<int> vPath;

    vPath.clear();

    if(ID1==ID2) return vPath;
    benchmark::heap<2, int, int> pqueue(nodenum);
    pqueue.update(ID1,0);

    vector<bool> closed(nodenum, false);
    vector<int> distance(nodenum, INF);
    vector<int> vPrevious(nodenum, -1);
    vector<int> vPreviousEdge(nodenum, -1);

    distance[ID1]=0;
    int topNodeID, topNodeDis;
    int NNodeID,NWeigh;

    int d=INF;//initialize d to infinite for the unreachable case

    while(!pqueue.empty()){
        pqueue.extract_min(topNodeID, topNodeDis);
        if(topNodeID==ID2){
            d=distance[ID2];
            break;
        }
        closed[topNodeID]=true;

        for(auto it=graphLength[topNodeID].begin();it!=graphLength[topNodeID].end();it++){
            NNodeID=(*it).first;
            NWeigh=(*it).second+topNodeDis;
            if(!closed[NNodeID]){
                if(distance[NNodeID]>NWeigh){
                    distance[NNodeID]=NWeigh;
                    pqueue.update(NNodeID, NWeigh);
                    auto itr = nodeID2RoadID.find(make_pair(topNodeID, NNodeID));
                    if(itr == nodeID2RoadID.end())
                        cout << "No Road from " << topNodeID << " to " << NNodeID <<endl;
                    else
                    {
                        vPreviousEdge[NNodeID] = (*itr).second;
                    }
                    vPrevious[NNodeID] = topNodeID;
                }
            }
        }
    }

    vPath.push_back(ID2);
    int p = vPrevious[ID2];
    while(p != -1)
    {
        vPath.push_back(p);
        p = vPrevious[p];
    }

    reverse(vPath.begin(), vPath.end());
    // Print the shortest path
    /*
    for (int i = 0; i < vPath.size(); ++i){
        cout << vPath[i] << " ";
    }
    cout << endl;
    */
    return vPath;
}

// flow -> travel time range -> travel time
int Graph::flow2time_by_range(int &ID1index, int &ID2index, int &flow)
{
    // Find Range for Specific Time Range
    vector<pair<int,int>> range = timeRange[ID1index][ID2index].second;
    // Correctness Check
    if (range.size() == 0)
        cout << "Do not find travel time range or its size is zero." << endl;
    // Variable Initialization
    int bound, travelTime;
    if(graphLength[ID1index][ID2index].second <= 20)
        return range[0].second;
    // Comparison
    for (int j=1;j<range.size();j++){
        bound = range[j].first;
        travelTime = range[j-1].second;
        if (flow < bound){
            /*
            // Print
            cout << flow << " Travel time is: " << travelTime << endl;
            */
            if (travelTime < 0)
                cout << "!!!!!Travel Time < 0" << endl;
            return travelTime;
        }
    }
    // Travel Time Equals to The Biggest One
    travelTime = range[range.size()-1].second;
    /*
    // Print
    cout << " Travel time is: " << travelTime << endl;
    */
    if (travelTime < 0)
        cout << "!!!!!!Tavel Time < 0" << endl;
    return travelTime;
}

// 读取CSV文件并返回一个 map，其中 key 是 edge_id，value 是 EdgeInfo 结构体
void Graph::read_edge_feature_2_map(const string& filePath) {


    ifstream file(filePath);
    if (!file.is_open()) {
        cerr << "Error: Could not open file " << filePath << endl;
        return;
    }

    string line;
    // Skip the first title row
    getline(file, line);

    while (getline(file, line)) {
        stringstream ss(line);
        string item;
        vector<string> rowData;

        // 分割每一行的数据，并存储到 rowData
        while (getline(ss, item, ',')) {
            rowData.push_back(item);
        }
        // 将 rowData 转换为适当的数据类型并存储到 map
        int edge_id = stoi(rowData[0]);
        int lane_num = static_cast<int>(round(stod(rowData[1])));  // 对 lane_num 进行四舍五入并转换为 int
        float speed = round(stod(rowData[2]) * 100) / 100;   // 对 speed 进行四舍五入并转换为 int
        float length = round(stod(rowData[3]) * 100) / 100;   // 对 length 进行四舍五入并转换为 int
        string edge_str = rowData[4];

        // cout << edge_id << " " << lane_num << " " << speed << " " << length << " " << edge_str << endl;

        edge_id_to_features[edge_id] = {lane_num, speed, length, edge_str};
    }

    /*
    // 打印读取的数据，验证结果
    for (const auto& edge : edge_id_to_features) {
        std::cout << "Edge ID: " << edge.first
                  << ", Lane Num: " << edge.second.lane_num
                  << ", Speed: " << edge.second.speed
                  << ", Length: " << edge.second.length
                  << ", Edge Str: " << edge.second.edge_str
                  << std::endl;
    }
    */

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
char Graph::findNextEdgeDirection(int currentEdgeID, int route_id) {
    char Turn = 'N';
    for (int i = 0; i < routeRoadID[route_id].size(); ++i) {
        if (routeRoadID[route_id][i] == currentEdgeID) {
            // 检查当前边是否是路由中的最后一条边
            if (i + 1 < routeRoadID[route_id].size()) {
                int nextEdgeID = routeRoadID[route_id][i + 1];
                auto it = connections_to_direction.find(make_pair(currentEdgeID, nextEdgeID));
                if (it != connections_to_direction.end()) {
                    Turn = it->second; // 将方向字符转换为字符串
                } else {
                    Turn = 'N'; // 在映射中找不到对应的方向
                }
                // cout << "edge_1 " << currentEdgeID << " and edge_2 " << nextEdgeID << "'s turn is: " << Turn << endl;
            } else {
                Turn = 'e'; // 当前边是路由中的最后一条边
                // cout << "edge_1 " << currentEdgeID << " is: " << Turn << endl;
            }
            return Turn; // 一旦找到当前边，立即返回结果
        }
    }
    cout << "Error. currentEdgeID is: " << currentEdgeID << endl;
    return Turn; // 如果未找到当前边，返回"unknown"
}

set<char> Graph::processVehicleDirections(const std::vector<int>& vehicleIDs, int& currentEdgeID) {
    set<char> Turn_Stat;
    for (int vehicleID : vehicleIDs) {
        // 假设我们可以通过 vehicleID 获取其当前边的 ID
        char Turn = findNextEdgeDirection(currentEdgeID, vehicleID);
        Turn_Stat.insert(Turn); // 将转向状态插入集合
    }
    return Turn_Stat;
}

bool sendDataToPython(int sock, int edge_id, const char& Turn, const set<char>& Turn_Stat, int flow) {
    // 将数据序列化为字符串
    std::stringstream ss;
    ss << edge_id << "|"; // 分隔符为 '|'
    ss << flow << "|";
    ss << Turn << "|";
    for (const auto& item : Turn_Stat) {
        ss << item << ","; // 使用 ',' 分隔集合中的元素
    }
    ss << "|" << "END_OF_MESSAGE";

    std::string data_to_send = ss.str();

    // 发送序列化后的数据
    if(send(sock, data_to_send.c_str(), data_to_send.size(), 0) < 0) {
        std::cerr << "Failed to send data." << std::endl;
        return false;
    }
    return true;
}

// 读取文件并构建词典的函数
void Graph::buildDictionary(const std::string& filename) {

    dictionary.clear();
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    RoadKey key;
    double travel_time_predict;

    while (file >> key.lane_num >> key.speed_limit >> key.edge_length >> key.driving_number >> key.delay_time >> key.lowSpee_time >> key.wait_time >> key.ratio >> key.length_square >> travel_time_predict) {

        dictionary[key] = travel_time_predict;
        //cout << "key.edge_str:" << key.edge_str << "." << endl;

        // std::cout << "Read In" << std::endl;
        /* cout << " " << key.lane_num << " " << key.speed_limit << " " << key.edge_length << " ";
        cout << key.driving_number << " " << key.delay_time << " " << key.lowSpee_time << " " << key.wait_time << endl; */
    }

    /*// 整个 list check 一下
    RoadKey queryKey = {"162041588#1", 2, 11, 56, 1, 0, 0, 0};  // 这是我们想要查找的键

    // 查找是否能在词典中找到对应的数据
    if (dictionary.find(queryKey) != dictionary.end()) {
        std::cout << "Found key! The value is: " << dictionary[queryKey] << std::endl;
    } else {
        std::cout << "Key not found!" << std::endl;
    }*/

    file.close();
}

// Estimate average travel time of ground truth
float Graph::AVG_estimation(vector<vector<int>> routeData, vector<vector<int>> timeData) {
    float totalTime = 0;
    int routeNum = routeData.size();
    for (int i = 0; i < timeData.size(); i++) {
        int travelTime = 0;
        for (int j = 1; j < timeData[i].size(); j++) {
            travelTime += timeData[i][j];
        }
        // cout << travelTime << endl;
        totalTime += travelTime;
    }
    int AVGTime = totalTime / routeNum;
    cout << "ground truth travel time avg is: " << AVGTime << "s."<< endl;

    return AVGTime;
}

// Estimate travel time MSE between simulated results and truth
float Graph::MSE_estimation(vector<vector<int>> time, vector<vector<pair<int, float>>> ETA) {

    float MSE_diff = 0;
    float MAE_diff = 0;
    float MAPE_diff = 0;
    int timeSize = 0;

    for (int i = 0; i < ETA.size(); i++) {

        float time_data_diff = 0;
        for (int j = 1; j < time[i].size(); j++) {
            time_data_diff += time[i][j];
        }

        // float time_data_diff = time[i][time[i].size()-1] - time[i][0];
        // float time_data_diff = time_no_wait[i];
        float eta_diff = ETA[i][ETA[i].size() - 1].second - ETA[i][0].second;
        float diff = time_data_diff - eta_diff;

        // MAE
        MAE_diff += abs(diff);

        // MAPE
        if (time_data_diff != 0) {  // To avoid division by zero
            MAPE_diff += abs(diff) / time_data_diff;
        }

        // MSE
        MSE_diff += diff * diff;

        timeSize += 1;
    }

    // MAE_diff = abs(MAE_diff);
    // MSE_diff += MAE_diff * MAE_diff;

    // MAE
    float MSE = MSE_diff / timeSize;
    cout << "MSE is: " << abs(MSE) << endl;

    // MAE
    float MAE = MAE_diff / timeSize;
    cout << "MAE is: " << abs(MAE) << endl;

    // RMSE
    float RMSE = sqrt(MSE_diff / timeSize);
    cout << "RMSE is: " << abs(RMSE) << endl;

    // MAPE
    float MAPE = (MAPE_diff / timeSize) * 100;  // Convert to percentage
    cout << "MAPE is: " << abs(MAPE) << "%" << endl;

    return MSE;
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

void Graph::Traffic_Prediction(vector<vector<pair<int, float>>> ETA_result) {

    // Initialization
    vector<vector<pair<int, float>>> traffic_prediction_structure(ETA_result.size());
    cout << "Number of route is: " << ETA_result.size() << endl;

    // Filter traffic prediction data
    for (int i = 0; i < ETA_result.size(); i++) {
        int route_ID = i;

        for (int j = 1; j < ETA_result[i].size(); j++) {
            int node_1 = ETA_result[route_ID][j - 1].first;
            float time_1 = ETA_result[route_ID][j - 1].second;

            int node_2 = ETA_result[route_ID][j].first;
            float time_2 = ETA_result[route_ID][j].second;

            auto roadIt = nodeID2RoadID.find(make_pair(node_1, node_2));
            if (roadIt == nodeID2RoadID.end()) {
                cout << "Warning. Missing road mapping for node pair: " << node_1 << " " << node_2 << endl;
                continue;
            }
            int edge_ID = roadIt->second;
            float travel_time = time_2 - time_1;

            traffic_prediction_structure[route_ID].push_back(make_pair(edge_ID, travel_time));
        }
    }

    // Print
    for (int i = 0; i < 1; i++) {
        cout << "Route ID: " << i << ": ";

        for (int j = 0; j < traffic_prediction_structure[i].size(); j++) {
            int edge_ID = traffic_prediction_structure[i][j].first;
            float travel_time = traffic_prediction_structure[i][j].second;
            cout << "(" << edge_ID << ", " << travel_time << ")";
        }
        cout << endl;
    }

    // Write to file
    ofstream outfile(join_path(Base, "traffic_prediction_structure_1.txt"));
    if (outfile.is_open()) {
        for (int i = 0; i < traffic_prediction_structure.size(); i++) {

            for (int j = 0; j < traffic_prediction_structure[i].size(); j++) {
                int edge_ID = traffic_prediction_structure[i][j].first;
                float travel_time = traffic_prediction_structure[i][j].second;
                outfile << edge_ID << " " << travel_time << " ";
            }
            outfile << endl;
        }
        outfile.close();
        cout << "Data written to traffic_prediction_structure_1.txt" << endl;
    } else {
        cout << "Unable to open file for writing!" << endl;
    }
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

    if (sumoTruthByVehicleID.empty()) {
        cout << "[SUMO Eval] No SUMO tripinfo truth loaded; skipping evaluation." << endl;
        return 0.0f;
    }

    const string summaryPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_summary.txt");
    const string groupedPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_grouped_metrics.csv");
    const string distributionPath = evalOutputPath.empty() ? "" : eval_sibling_path(evalOutputPath, "eval_distribution_metrics.csv");

    ofstream csv;
    if (!evalOutputPath.empty()) {
        csv.open(evalOutputPath.c_str());
        if (!csv) {
            throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open CSV output '" + evalOutputPath + "'");
        }
        csv << "vehicleID,predDepart,truthDepart,predArrival,truthArrival,"
            << "predDuration,truthDuration,durationError,absDurationError,"
            << "arrivalError,truthWaitingTime,truthTimeLoss,truthRouteLength,"
            << "durationErrorSigned,arrivalErrorSigned,absArrivalError,relativeDurationError,"
            << "absDurationErrorPerKm,predAvgSpeed,truthAvgSpeed,speedError,"
            << "numRoads,numMovements,validVehicle\n";
    }

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
    summary << "predMean=" << distribution.predMean << '\n' << "truthMean=" << distribution.truthMean << '\n' << "diffMean=" << distribution.diffMean << "\n";

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

void Graph::reserveLaneOccupancy(int vehicleID, int roadID, int laneIndex) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return;
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return;
    RoadSegment& road = roads[roadID];
    if (laneIndex < 0 || laneIndex >= static_cast<int>(road.laneFlow.size())) return;

    VehicleLabel& vehicle = vehicles[vehicleID];
    if (vehicle.occupiedRoadID >= 0 || vehicle.occupiedLaneIndex >= 0) {
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

vector<vector<pair<int, float>>> Graph::cycle_aware_signal_driven_records(
        vector<vector<int>> &Q, vector<vector<int>> &routeRoadIDInput) {
    // 新算法核心：事件驱动（signal change）+ 候选放行队列（dispatchPQ）+ 缓冲区排队
    // 每个信号事件触发一个 [currentTime, nextTime) 放行窗口，窗口内反复选择可放行候选车辆。
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
    delayedDepartureLogged.clear();

    this->routeRoadID = routeRoadIDInput;
    route_roadID_2_movementID();
    initializeMovementLaneDischargeCapacity();
    initializeRoadLaneStorage();

    int simStartTime = 0;
    if (!Q.empty()) {
        simStartTime = Q[0][2];
        for (const auto &q : Q) simStartTime = min(simStartTime, q[2]);
    }

    initialize_cycle_aware_vehicles(Q, routeRoadIDInput);
    initialize_signal_event_queue(simStartTime);

    int currentTime = simStartTime;
    int idleWindows = 0;
    const int maxIdleWindows = max(1000, static_cast<int>(vehicles.size()) * 10000);
    while (!allVehiclesFinished()) {
        int nextTime = INF;
        if (!signalEventPQ.empty()) nextTime = min(nextTime, signalEventPQ.top().time);
        if (!departurePQ.empty()) nextTime = min(nextTime, departurePQ.top().time);

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
            process_departures_until(currentTime, nextTime);
            process_discharge_window(currentTime, nextTime);
        }

        currentTime = nextTime;
        process_departures_until(currentTime, currentTime + 1);

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
    validateRoadLaneFlows();
    return ETA_result_cycle_aware;
}

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
    delayedDepartureLogged.clear();

    for (auto &b : waitingBuffers) {
        b.vehicleQueue.clear();
    }
    initializeRoadLaneStorage();

    // Only true route/mapping errors are invalid. Vehicles with no lane storage
    // remain NotDeparted and retry departure later.
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
        v.currentRoadID = v.routeRoadIDs[0];
        v.arrivalTime = departTime;

        if (v.currentRoadID < 0 || v.currentRoadID >= static_cast<int>(roads.size())) {
            mark_invalid(v);
            continue;
        }

        ETA_result_cycle_aware[i].push_back({v.currentRoadID, static_cast<float>(departTime)});

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
        if (movementBlockedByDownstream[movementID]) continue;
        int frontVehicleID = getFrontVehicleForMovement(movementID);
        if (frontVehicleID < 0) continue;
        int t = computeMovementAttemptTime(movementID, currentTime);
        scheduleMovementCandidate(movementID, t);
    }
}

void Graph::process_departures_until(int windowStart, int windowEnd) {
    (void)windowStart;
    while (!departurePQ.empty() && departurePQ.top().time < windowEnd) {
        DepartureEvent e = departurePQ.top();
        departurePQ.pop();
        departVehicle(e.vehicleID, e.time);
    }
}

bool Graph::departVehicle(int vehicleID, int departTime) {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return false;
    VehicleLabel &v = vehicles[vehicleID];
    if (!v.valid || v.finished || v.state != VehicleState::NotDeparted) return false;
    if (v.routeRoadIDs.empty()) return false;

    int firstRoad = v.routeRoadIDs[0];
    if (firstRoad < 0 || firstRoad >= static_cast<int>(roads.size())) return false;

    vector<int> candidateLanes;
    int firstMovement = v.currentMovementID;
    int firstBuffer = v.currentBufferID;
    if (v.routeRoadIDs.size() > 1) {
        if (firstMovement < 0 || firstMovement >= static_cast<int>(movements.size())) return false;
        if (firstBuffer < 0 || firstBuffer >= static_cast<int>(waitingBuffers.size())) return false;
        candidateLanes = parseLaneIndices(movements[firstMovement].fromLanes, firstRoad);
    }
    if (candidateLanes.empty()) {
        int laneCount = (firstRoad >= 0 && firstRoad < static_cast<int>(roads.size()))
            ? max(1, static_cast<int>(roads[firstRoad].laneFlow.size()))
            : 1;
        for (int lane = 0; lane < laneCount; ++lane) candidateLanes.push_back(lane);
    }

    int chosenLane = chooseLeastOccupiedAvailableLane(firstRoad, candidateLanes, vehicleID);
    if (chosenLane < 0) {
        int retryAt = departTime + 1;
        departurePQ.push({retryAt, vehicleID});
        if (verboseTravelTimePrediction || delayedDepartureLogged.insert(vehicleID).second) {
            cout << "[LaneFlow] delayed departure vehicle=" << vehicleID
                 << " road=" << firstRoad << " retryAt=" << retryAt << endl;
        }
        return false;
    }

    reserveLaneOccupancy(vehicleID, firstRoad, chosenLane);
    v.arrivalTime = departTime + predictRoadTravelTime(firstRoad, vehicleID);

    if (v.routeRoadIDs.size() == 1) {
        releaseLaneOccupancy(vehicleID);
        recordFinalETA(vehicleID, v.arrivalTime);
        reactivateMovementsBlockedByRoad(firstRoad, departTime);
        return true;
    }

    v.state = VehicleState::WaitingAtIntersection;
    insertVehicleToBufferOrdered(firstBuffer, vehicleID);
    return true;
}

void Graph::process_discharge_window(int windowStart, int windowEnd) {
    (void)windowStart;
    rebuildActiveDispatchPQ(windowStart, windowEnd);

    while (!dispatchPQ.empty()) {
        DispatchCandidate c = dispatchPQ.top();
        if (c.timeLabel >= windowEnd) break;
        dispatchPQ.pop();

        if (!isDispatchCandidateValid(c)) continue;
        int movementID = c.movementID;
        movementInDispatchPQ[movementID] = false;

        if (movementBlockedByDownstream[movementID]) continue;

        int bufferID = getMovementBufferID(movementID);
        if (bufferID < 0 || waitingBuffers[bufferID].vehicleQueue.empty()) continue;

        int vehicleID = waitingBuffers[bufferID].vehicleQueue.front();
        if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) continue;
        int t = max(max(movementTimeLabel[movementID], vehicles[vehicleID].arrivalTime), c.timeLabel);

        if (t >= windowEnd) {
            scheduleMovementCandidate(movementID, t);
            continue;
        }

        int frontVehicleID = -1;
        int chosenLane = -1;
        DischargeBlockReason reason = getDischargeBlockReason(movementID, t, frontVehicleID, chosenLane);

        if (reason == DischargeBlockReason::RedSignal) {
            int nextT = nextGreenTimeForMovement(movementID, t);
            if (nextT < INF) {
                movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
                scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            }
            continue;
        }

        if (reason == DischargeBlockReason::Capacity) {
            int nextT = nextAvailableCapacityTime(movementID, t);
            movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
            scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            continue;
        }

        if (reason == DischargeBlockReason::NotArrived) {
            int nextT = (frontVehicleID >= 0 && frontVehicleID < static_cast<int>(vehicles.size()))
                ? vehicles[frontVehicleID].arrivalTime
                : t + 1;
            movementTimeLabel[movementID] = max(movementTimeLabel[movementID], nextT);
            scheduleMovementCandidate(movementID, movementTimeLabel[movementID]);
            continue;
        }

        if (reason == DischargeBlockReason::DownstreamFull) {
            deactivateMovementForDownstreamBlock(movementID);
            continue;
        }

        if (reason != DischargeBlockReason::None) continue;

        int oldLabel = movementTimeLabel[movementID];
        int fifoBefore = waitingBuffers[bufferID].vehicleQueue.empty() ? -1 : waitingBuffers[bufferID].vehicleQueue.front();
        DischargeResult result = dischargeOneVehicle(movementID, t);
        if (result.vehicleID < 0) {
            if (fifoBefore >= 0 && !waitingBuffers[bufferID].vehicleQueue.empty()
                    && waitingBuffers[bufferID].vehicleQueue.front() != fifoBefore) {
                cout << "[Dispatch Sanity Warning] failed discharge changed FIFO movement=" << movementID << endl;
            }
            continue;
        }

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

        if (!waitingBuffers[bufferID].vehicleQueue.empty()) {
            int nextFront = waitingBuffers[bufferID].vehicleQueue.front();
            int nextT = max(movementTimeLabel[movementID], vehicles[nextFront].arrivalTime);
            scheduleMovementCandidate(movementID, computeMovementAttemptTime(movementID, nextT));
        }

        if (!result.finished && result.nextMovementID >= 0) {
            int nextT = max(movementTimeLabel[result.nextMovementID], result.newArrivalTime);
            scheduleMovementCandidate(result.nextMovementID,
                                      computeMovementAttemptTime(result.nextMovementID, nextT));
        }

        reactivateMovementsBlockedByRoad(result.releasedRoadID, t);
    }
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

bool Graph::canDischarge(int movementID, int dischargeTime, int windowEnd) {
    (void)windowEnd;
    int frontVehicleID = -1;
    int chosenLane = -1;
    return getDischargeBlockReason(movementID, dischargeTime, frontVehicleID, chosenLane)
        == DischargeBlockReason::None;
}

DischargeResult Graph::dischargeOneVehicle(int movementID, int dischargeTime) {
    DischargeResult result;
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
    result.releasedRoadID = v.occupiedRoadID;
    result.releasedLaneIndex = v.occupiedLaneIndex;

    b.vehicleQueue.pop_front();
    releaseLaneOccupancy(vehicleID);
    consumeDischargeCapacity(movementID, dischargeTime);

    v.roadIndex += 1;
    v.currentRoadID = m.toRoadID;
    v.arrivalTime = dischargeTime + predictRoadTravelTime(m.toRoadID, vehicleID);

    result.vehicleID = vehicleID;
    result.movementID = movementID;
    result.intersectionID = m.intersectionID;
    result.fromRoadID = m.fromRoadID;
    result.toRoadID = m.toRoadID;
    result.dischargeTime = dischargeTime;
    result.newArrivalTime = v.arrivalTime;

    if (v.roadIndex >= static_cast<int>(v.routeRoadIDs.size()) - 1) {
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
            insertVehicleToBufferOrdered(nextBufferID, vehicleID);
            reserveLaneOccupancy(vehicleID, m.toRoadID, chosenLane);
            result.nextMovementID = nextMovementID;
            result.nextBufferID = nextBufferID;
        } else {
            v.valid = false;
            invalidVehicleCount++;
            v.state = VehicleState::Finished;
            result.finished = true;
            v.finished = true;
            finishedVehicleCount++;
        }
    } else {
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
    return getFrontVehicleForMovement(c.movementID) >= 0;
}

int Graph::computeEarliestDischargeTime(int movementID, int readyTime, int currentTime) {
    return computeMovementAttemptTime(movementID, max(readyTime, currentTime));
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

bool Graph::hasReadyFrontVehicle(int movementID, int t) {
    int vehicleID = getFrontVehicleForMovement(movementID);
    return vehicleID >= 0 && vehicles[vehicleID].arrivalTime <= t;
}

DischargeBlockReason Graph::getDischargeBlockReason(int movementID, int t, int &frontVehicleID, int &chosenLane) {
    frontVehicleID = -1;
    chosenLane = -1;
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return DischargeBlockReason::Invalid;
    int bufferID = getMovementBufferID(movementID);
    if (bufferID < 0) return DischargeBlockReason::Invalid;
    if (waitingBuffers[bufferID].vehicleQueue.empty()) return DischargeBlockReason::EmptyBuffer;
    frontVehicleID = getFrontVehicleForMovement(movementID);
    if (frontVehicleID < 0) return DischargeBlockReason::Invalid;
    if (vehicles[frontVehicleID].arrivalTime > t) return DischargeBlockReason::NotArrived;
    if (!isMovementActive(movementID, t)) return DischargeBlockReason::RedSignal;
    if (!hasDischargeCapacity(movementID, t)) return DischargeBlockReason::Capacity;
    if (!hasDownstreamLaneStorage(movementID, frontVehicleID, chosenLane)) return DischargeBlockReason::DownstreamFull;
    return DischargeBlockReason::None;
}

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
    if (getFrontVehicleForMovement(movementID) < 0) return;

    int label = max(movementTimeLabel[movementID], time);
    movementTimeLabel[movementID] = label;
    int version = ++movementPQVersion[movementID];
    dispatchPQ.push({label, movementID, version});
    movementInDispatchPQ[movementID] = true;

    if (verboseTravelTimePrediction) {
        cout << "[Dispatch Schedule] movement=" << movementID
             << " timeLabel=" << label
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

bool Graph::hasDischargeCapacity(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return false;
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    int used = usedMovementDischargeCapacity[make_tuple(movementID, slot)];
    int cap = max(1, movements[movementID].laneDischargeCapacity);
    return used < cap;
}

void Graph::consumeDischargeCapacity(int movementID, int t) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    usedMovementDischargeCapacity[make_tuple(movementID, slot)]++;
}

int Graph::nextAvailableCapacityTime(int movementID, int t) {
    int interval = max(1, defaultDischargeInterval);
    int cur = t;
    while (!hasDischargeCapacity(movementID, cur)) {
        cur = ((cur / interval) + 1) * interval;
    }
    return cur;
}

bool Graph::hasDownstreamStorage(int roadID) {
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return false;
    const RoadSegment &road = roads[roadID];
    double vehicleLength = 5.0;
    double gap = 1.0;
    int capacity = road.storageCapacityVehicles;
    if (capacity <= 0) {
        capacity = static_cast<int>(floor(max(0.0, road.length) * max(1, road.laneNum) / (vehicleLength + gap)));
    }
    if (capacity <= 0) capacity = 1;

    int waitingOnRoad = 0;
    for (const auto &kv : road.movementIDToWaitingBufferID) {
        int bufferID = kv.second;
        if (bufferID >= 0 && bufferID < static_cast<int>(waitingBuffers.size())) {
            waitingOnRoad += waitingBuffers[bufferID].vehicleCount();
        }
    }
    int currentOccupancy = road.runningCount + waitingOnRoad;
    return currentOccupancy < capacity;
}

int Graph::predictRoadTravelTime(int roadID, int vehicleID) {
    switch (travelTimeMode) {
        case TravelTimeMode::SPEED_NET:
            return predictRoadTravelTimeSpeedNet(roadID);
        case TravelTimeMode::MIN_TIME:
            return predictRoadTravelTimeMinTime(roadID);
        case TravelTimeMode::TABLE:
            return predictRoadTravelTimeTable(roadID, vehicleID);
        case TravelTimeMode::MODEL:
            return predictRoadTravelTimeModel(roadID, vehicleID);
        case TravelTimeMode::KINEMATIC:
            return predictRoadTravelTimeKinematic(roadID, vehicleID);
    }
    return predictRoadTravelTimeSpeedNet(roadID);
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

RoadKey Graph::buildRoadKeyForPrediction(int roadID, int vehicleID) const {
    (void)vehicleID;
    RoadKey key{};
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) {
        return key;
    }

    const RoadSegment& r = roads[roadID];
    const double speed = std::max(0.1, r.speedLimit);
    const double length = std::max(0.0, r.length);

    key.lane_num = r.laneNum;
    key.speed_limit = static_cast<float>(std::round(speed));
    key.edge_length = static_cast<float>(std::round(length));
    key.driving_number = r.roadFlow;
    key.delay_time = 0;
    key.lowSpee_time = 0;
    key.wait_time = r.roadFlow;
    key.ratio = static_cast<int>(std::round(length / speed));
    key.length_square = static_cast<int>(std::round(length * length));
    return key;
}

int Graph::predictRoadTravelTimeTable(int roadID, int vehicleID) {
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return 1;

    RoadKey key = buildRoadKeyForPrediction(roadID, vehicleID);
    auto it = dictionary.find(key);
    if (it != dictionary.end()) {
        ++travelTimeTableHit;
        return std::max(1, static_cast<int>(std::round(it->second)));
    }

    ++travelTimeTableMiss;
    if (verboseTravelTimePrediction) {
        cout << "[TravelTime] TABLE miss roadID=" << roadID << endl;
    }

    return fallbackToSpeedNet
        ? predictRoadTravelTimeSpeedNet(roadID)
        : predictRoadTravelTimeMinTime(roadID);
}


int Graph::predictRoadTravelTimeKinematic(int roadID, int vehicleID) const {
    if (roadID < 0 || roadID >= static_cast<int>(roads.size())) return 1;
    const RoadSegment& r = roads[roadID];

    const VehicleType& vt = getVehicleTypeForVehicle(vehicleID);

    const double L = max(0.0, r.length);
    const double vRoad = max(0.1, r.speedLimit);
    const double vVeh = max(0.1, vt.maxSpeed);
    double v = min(vRoad, vVeh);
    v = v * max(0.7, 1.0 - 0.1 * vt.sigma);
    const double a = max(0.1, vt.accel);
    const double dAcc = v * v / (2.0 * a);

    double tFree = 0.0;
    if (L <= dAcc) {
        tFree = sqrt(2.0 * L / a);
    } else {
        tFree = v / a + (L - dAcc) / v;
    }

    const int occupancy = r.roadFlow;
    const double vehicleSpace = max(1e-6, vt.length + vt.minGap);
    int capacity = static_cast<int>(floor(L * max(1, r.laneNum) / vehicleSpace));
    capacity = max(1, capacity);
    double rho = safe_divide(static_cast<double>(occupancy), static_cast<double>(capacity));
    rho = max(0.0, min(0.95, rho));
    const double alpha = max(0.0, kinematicCongestionAlpha);
    const double t = tFree * (1.0 + alpha * rho);
    return max(1, static_cast<int>(round(t)));
}

bool Graph::queryExternalTravelTimeModel(int roadID, int vehicleID, double& predictedTime) {
    (void)roadID;
    (void)vehicleID;
    predictedTime = -1.0;
    // TODO: future implementation. This can later use socket / Python service / embedded model.
    return false;
}

int Graph::predictRoadTravelTimeModel(int roadID, int vehicleID) {
    double pred = -1.0;
    if (queryExternalTravelTimeModel(roadID, vehicleID, pred) && pred > 0) {
        return std::max(1, static_cast<int>(std::round(pred)));
    }

    if (!modelWarningPrinted) {
        cout << "[TravelTime] MODEL mode is selected but no external model is implemented; using fallback." << endl;
        modelWarningPrinted = true;
    }

    return fallbackToSpeedNet
        ? predictRoadTravelTimeSpeedNet(roadID)
        : predictRoadTravelTimeMinTime(roadID);
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

SignalState Graph::signalStateAt(int signalID, int t) {
    if (signalID < 0 || signalID >= static_cast<int>(signals.size())) return SignalState::Red;
    int movementID = signals[signalID].movementID;
    if (movementID >= 0 && movementID < static_cast<int>(movements.size())) {
        return signalStateAtMovement(movementID, t);
    }

    // Legacy fallback for old BJ workflow controllers.
    const SignalController &s = signals[signalID];
    if (s.alwaysOpen) return SignalState::AlwaysOpen;
    if (s.cycleLength <= 0) return SignalState::Red;
    int local = (t - s.offset) % s.cycleLength;
    if (local < 0) local += s.cycleLength;
    if (s.greenStart <= s.greenEnd) {
        return (local >= s.greenStart && local < s.greenEnd) ? SignalState::Green : SignalState::Red;
    }
    return (local >= s.greenStart || local < s.greenEnd) ? SignalState::Green : SignalState::Red;
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
