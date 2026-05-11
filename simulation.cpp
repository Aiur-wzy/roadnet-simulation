
#include "head.h"

TravelTimeMode parseTravelTimeMode(const string& s) {
    if (s == "speed-net") return TravelTimeMode::SPEED_NET;
    if (s == "min-time") return TravelTimeMode::MIN_TIME;
    if (s == "table") return TravelTimeMode::TABLE;
    if (s == "model") return TravelTimeMode::MODEL;
    throw runtime_error("Invalid travel time mode: " + s + " (expected speed-net, min-time, table, or model)");
}

string travelTimeModeToString(TravelTimeMode mode) {
    switch (mode) {
        case TravelTimeMode::SPEED_NET: return "speed-net";
        case TravelTimeMode::MIN_TIME: return "min-time";
        case TravelTimeMode::TABLE: return "table";
        case TravelTimeMode::MODEL: return "model";
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


float Graph::evaluate_sumo_tripinfo_truth(
        const vector<vector<pair<int, float>>>& ETA)
{
    if (sumoTruthByVehicleID.empty()) {
        cout << "[SUMO Eval] No SUMO tripinfo truth loaded; skipping evaluation." << endl;
        return 0.0f;
    }

    ofstream csv;
    if (!evalOutputPath.empty()) {
        csv.open(evalOutputPath.c_str());
        if (!csv) {
            throw runtime_error("evaluate_sumo_tripinfo_truth: cannot open CSV output '" + evalOutputPath + "'");
        }
        csv << "vehicleID,predDepart,truthDepart,predArrival,truthArrival,"
            << "predDuration,truthDuration,durationError,absDurationError,"
            << "arrivalError,truthWaitingTime,truthTimeLoss,truthRouteLength\n";
    }

    int comparedCount = 0;
    int missingTruthCount = 0;
    int invalidVehicleSkipped = 0;
    int etaMissingSkipped = 0;
    double squaredDurationError = 0.0;
    double absoluteDurationError = 0.0;
    double percentageDurationError = 0.0;
    int percentageCount = 0;
    double squaredArrivalError = 0.0;
    double absoluteArrivalError = 0.0;

    int n = static_cast<int>(ETA.size());
    n = max(n, static_cast<int>(vehicles.size()));
    n = max(n, static_cast<int>(queryDataRaw.size()));
    n = max(n, static_cast<int>(routeRoadID.size()));

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

        if (i < static_cast<int>(vehicles.size()) && !vehicles[i].valid) {
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

        squaredDurationError += durationError * durationError;
        absoluteDurationError += absDurationError;
        if (truthDuration > 0.0) {
            percentageDurationError += absDurationError / truthDuration;
            ++percentageCount;
        }
        squaredArrivalError += arrivalError * arrivalError;
        absoluteArrivalError += abs(arrivalError);
        ++comparedCount;

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
                << truth.routeLength << '\n';
        }
    }

    int truthNotSimulated = 0;
    for (const auto &kv : sumoTruthByVehicleID) {
        if (simulatedVehicleIDs.find(kv.first) == simulatedVehicleIDs.end()) {
            ++truthNotSimulated;
        }
    }

    cout << "[SUMO Eval] compared vehicles: " << comparedCount << endl;
    cout << "[SUMO Eval] missing truth: " << missingTruthCount << endl;
    cout << "[SUMO Eval] invalid skipped: " << invalidVehicleSkipped << endl;
    cout << "[SUMO Eval] ETA missing skipped: " << etaMissingSkipped << endl;
    cout << "[SUMO Eval] truth not simulated: " << truthNotSimulated << endl;

    if (comparedCount == 0) {
        cout << "[SUMO Eval] No valid vehicles compared." << endl;
        if (csv) cout << "[SUMO Eval] CSV written to " << evalOutputPath << endl;
        return 0.0f;
    }

    double mse = squaredDurationError / comparedCount;
    double mae = absoluteDurationError / comparedCount;
    double rmse = sqrt(mse);
    double mape = (percentageCount > 0)
                ? (percentageDurationError / percentageCount) * 100.0
                : 0.0;
    double maeArrival = absoluteArrivalError / comparedCount;
    double rmseArrival = sqrt(squaredArrivalError / comparedCount);

    cout << "[SUMO Eval] MSE duration: " << mse << endl;
    cout << "[SUMO Eval] MAE duration: " << mae << endl;
    cout << "[SUMO Eval] RMSE duration: " << rmse << endl;
    cout << "[SUMO Eval] MAPE duration: " << mape << "%" << endl;
    cout << "[SUMO Eval] MAE arrival: " << maeArrival << endl;
    cout << "[SUMO Eval] RMSE arrival: " << rmseArrival << endl;
    if (csv) cout << "[SUMO Eval] CSV written to " << evalOutputPath << endl;

    return static_cast<float>(mse);
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
    while (!signalEventPQ.empty()) signalEventPQ.pop();
    while (!dispatchPQ.empty()) dispatchPQ.pop();

    this->routeRoadID = routeRoadIDInput;
    route_roadID_2_movementID();
    initializeMovementLaneDischargeCapacity();

    int simStartTime = 0;
    if (!Q.empty()) {
        simStartTime = Q[0][2];
        for (const auto &q : Q) simStartTime = min(simStartTime, q[2]);
    }

    initialize_cycle_aware_vehicles(Q, routeRoadIDInput);
    initialize_signal_event_queue(simStartTime);

    int currentTime = simStartTime;
    while (!allVehiclesFinished()) {
        if (signalEventPQ.empty()) {
            cout << "[Error] signalEventPQ is empty before all vehicles finished." << endl;
            break;
        }

        int nextTime = signalEventPQ.top().time;
        if (currentTime < nextTime) {
            process_discharge_window(currentTime, nextTime);
        }

        currentTime = nextTime;

        while (!signalEventPQ.empty() && signalEventPQ.top().time == currentTime) {
            SignalEvent e = signalEventPQ.top();
            signalEventPQ.pop();
            handle_signal_change_event(e);
            int nextT = nextSignalChangeTime(e.signalID, currentTime + 1);
            if (nextT < INF) {
                signalEventPQ.push({nextT, e.signalID});
            }
        }

        if (signalEventPQ.empty()) break;
    }
    return ETA_result_cycle_aware;
}

void Graph::initialize_cycle_aware_vehicles(vector<vector<int>>& Q, vector<vector<int>>& routeRoadIDInput) {
    vehicles.clear();
    vehicles.resize(routeRoadIDInput.size());
    ETA_result_cycle_aware.assign(routeRoadIDInput.size(), {});
    finishedVehicleCount = 0;
    invalidVehicleCount = 0;
    usedMovementDischargeCapacity.clear();

    for (auto &b : waitingBuffers) {
        b.vehicleQueue.clear();
    }

    // 路径无法映射到 movement/buffer 的车辆标记为 invalid，并从评估集中排除。
    auto mark_invalid = [&](VehicleLabel &vehicle) {
        vehicle.valid = false;
        vehicle.finished = true;
        vehicle.state = VehicleState::Finished;
        vehicles[vehicle.vehicleID] = vehicle;
        finishedVehicleCount++;
        invalidVehicleCount++;
    };

    for (int i = 0; i < routeRoadIDInput.size(); ++i) {
        VehicleLabel v;
        v.vehicleID = i;
        v.routeID = i;
        v.routeRoadIDs = routeRoadIDInput[i];
        if (i < routeMovementID.size()) v.routeMovementIDs = routeMovementID[i];
        v.roadIndex = 0;

        if (v.routeRoadIDs.empty()) {
            mark_invalid(v);
            continue;
        }

        int departTime = (i < Q.size() && Q[i].size() > 2) ? Q[i][2] : 0;
        v.currentRoadID = v.routeRoadIDs[0];
        v.arrivalTime = departTime + predictRoadTravelTime(v.currentRoadID, i);

        ETA_result_cycle_aware[i].push_back({v.currentRoadID, static_cast<float>(departTime)});

        if (v.routeRoadIDs.size() == 1) {
            vehicles[i] = v;
            recordFinalETA(i, v.arrivalTime);
            continue;
        } else if (!v.routeMovementIDs.empty()) {
            int movementID = v.routeMovementIDs[0];
            if (movementID < 0 || movementID >= static_cast<int>(movements.size())) {
                mark_invalid(v);
                continue;
            }
            if (v.currentRoadID < 0 || v.currentRoadID >= static_cast<int>(roads.size())) {
                mark_invalid(v);
                continue;
            }
            auto itRoad = roads[v.currentRoadID].movementIDToWaitingBufferID.find(movementID);
            if (itRoad != roads[v.currentRoadID].movementIDToWaitingBufferID.end()) {
                v.currentMovementID = movementID;
                v.currentBufferID = itRoad->second;
                if (v.currentBufferID < 0 || v.currentBufferID >= static_cast<int>(waitingBuffers.size())) {
                    mark_invalid(v);
                    continue;
                }
                v.state = VehicleState::WaitingAtIntersection;
                vehicles[i] = v;
                insertVehicleToBufferOrdered(v.currentBufferID, i);
                continue;
            } else {
                mark_invalid(v);
                continue;
            }
        } else {
            mark_invalid(v);
            continue;
        }
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
    // 在当前放行窗口内，收集所有 active movement 的队首车辆作为候选。
    while (!dispatchPQ.empty()) dispatchPQ.pop();
    for (const auto &m : movements) {
        if (m.movementID < 0) continue;
        if (!isMovementActive(m.movementID, currentTime)) continue;
        auto it = roads[m.fromRoadID].movementIDToWaitingBufferID.find(m.movementID);
        if (it == roads[m.fromRoadID].movementIDToWaitingBufferID.end()) continue;
        int bufferID = it->second;
        if (bufferID < 0 || bufferID >= waitingBuffers.size()) continue;
        if (waitingBuffers[bufferID].vehicleQueue.empty()) continue;
        int vehicleID = waitingBuffers[bufferID].vehicleQueue.front();
        int readyTime = vehicles[vehicleID].arrivalTime;
        int earliest = computeEarliestDischargeTime(m.movementID, readyTime, currentTime);
        if (earliest < windowEnd) {
            dispatchPQ.push({earliest, readyTime, m.movementID, bufferID, vehicleID});
        }
    }
}

void Graph::process_discharge_window(int windowStart, int windowEnd) {
    // 窗口内持续弹出“最早可放行”候选，直到无合法候选或越过窗口右边界。
    rebuildActiveDispatchPQ(windowStart, windowEnd);
    while (!dispatchPQ.empty()) {
        DispatchCandidate c = dispatchPQ.top();
        dispatchPQ.pop();
        if (!isDispatchCandidateValid(c)) continue;
        if (c.earliestDischargeTime >= windowEnd) continue;
        if (!canDischarge(c.movementID, c.earliestDischargeTime, windowEnd)) continue;

        DischargeResult result = dischargeOneVehicle(c.movementID, c.earliestDischargeTime);
        pushCandidateIfPossible(c.movementID, c.earliestDischargeTime, windowEnd);
        if (!result.finished) {
            pushCandidateIfPossible(result.nextMovementID, c.earliestDischargeTime, windowEnd);
        }
        for (const auto &m : movements) {
            if (m.intersectionID == result.intersectionID && m.toRoadID == result.toRoadID) {
                pushCandidateIfPossible(m.movementID, c.earliestDischargeTime, windowEnd);
            }
        }
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
    // 放行条件：相位有效 + 队首车辆已到达 + 下游存储可用 + 断面放行能力可用。
    if (!isMovementActive(movementID, dischargeTime)) return false;
    if (movementID < 0 || movementID >= movements.size()) return false;
    const Movement &m = movements[movementID];
    auto it = roads[m.fromRoadID].movementIDToWaitingBufferID.find(movementID);
    if (it == roads[m.fromRoadID].movementIDToWaitingBufferID.end()) return false;
    int bufferID = it->second;
    if (bufferID < 0 || bufferID >= waitingBuffers.size()) return false;
    auto &b = waitingBuffers[bufferID];
    if (b.vehicleQueue.empty()) return false;
    int vehicleID = b.vehicleQueue.front();
    if (vehicleID < 0 || vehicleID >= vehicles.size()) return false;
    if (vehicles[vehicleID].arrivalTime > dischargeTime) return false;
    if (!hasDownstreamStorage(m.toRoadID)) return false;
    if (!hasDischargeCapacity(movementID, dischargeTime)) return false;
    return true;
}

DischargeResult Graph::dischargeOneVehicle(int movementID, int dischargeTime) {
    DischargeResult result;
    if (movementID < 0 || movementID >= movements.size()) return result;
    const Movement &m = movements[movementID];
    auto it = roads[m.fromRoadID].movementIDToWaitingBufferID.find(movementID);
    if (it == roads[m.fromRoadID].movementIDToWaitingBufferID.end()) return result;
    int bufferID = it->second;
    auto &b = waitingBuffers[bufferID];
    if (b.vehicleQueue.empty()) return result;

    int vehicleID = b.vehicleQueue.front();
    b.vehicleQueue.pop_front();
    consumeDischargeCapacity(movementID, dischargeTime);

    VehicleLabel &v = vehicles[vehicleID];
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

    if (v.roadIndex < v.routeMovementIDs.size()) {
        int nextMovementID = v.routeMovementIDs[v.roadIndex];
        auto itBuffer = roads[m.toRoadID].movementIDToWaitingBufferID.find(nextMovementID);
        if (itBuffer != roads[m.toRoadID].movementIDToWaitingBufferID.end()) {
            int nextBufferID = itBuffer->second;
            v.currentMovementID = nextMovementID;
            v.currentBufferID = nextBufferID;
            v.state = VehicleState::WaitingAtIntersection;
            insertVehicleToBufferOrdered(nextBufferID, vehicleID);
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
    if (movementID < 0 || movementID >= movements.size()) return;
    if (!isMovementActive(movementID, currentTime)) return;
    const Movement &m = movements[movementID];
    auto it = roads[m.fromRoadID].movementIDToWaitingBufferID.find(movementID);
    if (it == roads[m.fromRoadID].movementIDToWaitingBufferID.end()) return;
    int bufferID = it->second;
    if (bufferID < 0 || bufferID >= waitingBuffers.size()) return;
    auto &b = waitingBuffers[bufferID];
    if (b.vehicleQueue.empty()) return;
    int vehicleID = b.vehicleQueue.front();
    int readyTime = vehicles[vehicleID].arrivalTime;
    int earliest = computeEarliestDischargeTime(movementID, readyTime, currentTime);
    if (earliest >= windowEnd) return;
    dispatchPQ.push({earliest, readyTime, movementID, bufferID, vehicleID});
}

bool Graph::isDispatchCandidateValid(const DispatchCandidate& c) {
    if (c.bufferID < 0 || c.bufferID >= waitingBuffers.size()) return false;
    if (c.vehicleID < 0 || c.vehicleID >= vehicles.size()) return false;
    auto &b = waitingBuffers[c.bufferID];
    if (b.vehicleQueue.empty()) return false;
    if (b.vehicleQueue.front() != c.vehicleID) return false;
    const auto &v = vehicles[c.vehicleID];
    if (v.currentBufferID != c.bufferID) return false;
    if (v.currentMovementID != c.movementID) return false;
    return true;
}

int Graph::computeEarliestDischargeTime(int movementID, int readyTime, int currentTime) {
    if (movementID < 0 || movementID >= movements.size()) return INF;
    int t = max(readyTime, currentTime);
    int capT = nextAvailableCapacityTime(movementID, t);
    return max(t, capT);
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

    int waitingOnRoad = 0;
    for (const auto& kv : r.movementIDToWaitingBufferID) {
        int bufferID = kv.second;
        if (bufferID >= 0 && bufferID < static_cast<int>(waitingBuffers.size())) {
            waitingOnRoad += waitingBuffers[bufferID].vehicleCount();
        }
    }

    key.lane_num = r.laneNum;
    key.speed_limit = static_cast<float>(std::round(speed));
    key.edge_length = static_cast<float>(std::round(length));
    key.driving_number = r.runningCount;
    key.delay_time = 0;
    key.lowSpee_time = 0;
    key.wait_time = waitingOnRoad;
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
