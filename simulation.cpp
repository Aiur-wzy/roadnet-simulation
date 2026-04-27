
#include "head.h"

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

// Simulation
vector<vector<pair<int,float>>> Graph::alg1Records(
        vector<vector<int>> &Q, vector<vector<int>> &Pi,
        bool range, bool server, bool catching, bool write, bool latency, string te_choose) {

    // 本次重构中，server/catching/write 暂不参与主流程，保留入参兼容旧调用
    (void)server;
    (void)catching;
    (void)write;
    (void)te_choose;

    // 车辆状态：当前所在路径位置、当前事件时间、累计等待时间、是否完成
    struct VehicleState {
        int route_id = -1;
        int node_index = 0;
        float time = 0;
        bool finished = false;
        float wait_time = 0;
    };

    // 通用最小事件结构（车辆/节点局部队列）
    struct MinEvent {
        float t;
        int id;
        bool operator>(const MinEvent& other) const {
            if (t == other.t) return id > other.id;
            return t > other.t;
        }
    };

    // 信号相位变化事件（全局信号事件堆）
    struct SignalEvent {
        float t;
        int node;
        bool operator>(const SignalEvent& other) const {
            if (t == other.t) return node > other.node;
            return t > other.t;
        }
    };

    // 在邻接表中找到 from->to 边对应的索引
    auto get_edge_index = [&](int from, int to) {
        for (int i = 0; i < graphLength[from].size(); ++i) {
            if (graphLength[from][i].first == to) return i;
        }
        return -1;
    };

    vector<int> indegree(nodenum, 0);
    for (int u = 0; u < graphLength.size(); ++u) {
        for (auto &e : graphLength[u]) indegree[e.first]++;
    }
    vector<bool> signal_node(nodenum, false);
    for (int n = 0; n < nodenum; ++n) {
        signal_node[n] = indegree[n] > 1;
    }

    // 基础参数：车辆等效长度、相位数、相位时长、等待重试步长
    const float vehicle_len = 5.0f;
    const float vehicle_gap = 1.0f;
    const int phase_num = 3;
    const float phase_duration = 20.0f;
    const float release_step = 1.0f;

    auto turn_to_phase = [&](char d) {
        if (d == 'l' || d == 'L') return 1;
        if (d == 'r' || d == 'R') return 2;
        return 0;
    };

    // 初始化 ETA 输出结构：每辆车在每个路径节点上的到达时刻
    ETA_result.assign(Pi.size(), {});
    vector<VehicleState> vehicles(Pi.size());
    for (int i = 0; i < Pi.size(); ++i) {
        ETA_result[i].resize(Pi[i].size());
        for (int j = 0; j < Pi[i].size(); ++j) {
            ETA_result[i][j] = {Pi[i][j], INF};
        }
        vehicles[i].route_id = i;
        vehicles[i].time = Q[i][2];
        ETA_result[i][0].second = Q[i][2];
    }

    // driving_flow：道路行驶区流量；waiting_queue：路口前等待区队列（FIFO）
    vector<vector<pair<int, int>>> driving_flow(graphLength.size());
    vector<vector<deque<int>>> waiting_queue(graphLength.size());
    for (int i = 0; i < graphLength.size(); ++i) {
        driving_flow[i].resize(graphLength[i].size());
        waiting_queue[i].resize(graphLength[i].size());
        for (int j = 0; j < graphLength[i].size(); ++j) {
            driving_flow[i][j] = {graphLength[i][j].first, 0};
        }
    }

    timeFlowChange.clear();
    timeFlowChange.resize(graphLength.size());
    for (int i = 0; i < graphLength.size(); i++) {
        timeFlowChange[i].resize(graphLength[i].size());
        for (int j = 0; j < graphLength[i].size(); j++) {
            timeFlowChange[i][j].first = graphLength[i][j].first;
        }
    }

    // 分层调度结构：
    // local_queues[node] 维护“到达该 node 的局部最小事件”，top_queue 只维护各局部队列最小值
    vector<priority_queue<MinEvent, vector<MinEvent>, greater<MinEvent>>> local_queues(nodenum);
    priority_queue<MinEvent, vector<MinEvent>, greater<MinEvent>> top_queue;

    auto push_local = [&](int node, float t, int vid) {
        local_queues[node].push({t, vid});
        // 若该事件成为此 node 的当前最小值，则同步推入顶层队列
        if (!local_queues[node].empty() && local_queues[node].top().id == vid && local_queues[node].top().t == t) {
            top_queue.push({t, node});
        }
    };

    // 全局相位变化事件堆：每个信号节点始终保持“下一个相位变化事件”
    priority_queue<SignalEvent, vector<SignalEvent>, greater<SignalEvent>> signal_events;
    for (int n = 0; n < nodenum; ++n) {
        if (signal_node[n]) {
            signal_events.push({phase_duration, n});
        }
    }

    // 兼容旧路网定义：优先用 edge_id_to_features，其次回退到 roadInfor，最后使用保底值
    auto get_edge_info = [&](int edge_id, float fallback_len) {
        EdgeInfo info{1, 10.0f, fallback_len, ""};
        auto f_it = edge_id_to_features.find(edge_id);
        if (f_it != edge_id_to_features.end()) {
            if (f_it->second.lane_num > 0) info.lane_num = f_it->second.lane_num;
            if (f_it->second.speed > 0) info.speed = f_it->second.speed;
            if (f_it->second.length > 0) info.length = f_it->second.length;
            info.edge_str = f_it->second.edge_str;
            return info;
        }
        for (const auto &r : roadInfor) {
            if (r.roadID == edge_id) {
                if (r.laneNum > 0) info.lane_num = r.laneNum;
                if (r.speedLimit > 0) info.speed = (float)r.speedLimit;
                if (r.length > 0) info.length = (float)r.length;
                break;
            }
        }
        return info;
    };

    // 下游道路存储约束：
    // 等待区长度 = ceil(waiting/lane) * (vehicle_len + gap)
    // 与行驶区占用长度共同约束，决定是否还能接纳新车
    auto edge_storage_ok = [&](int from, int idx) {
        int to = graphLength[from][idx].first;
        int edge_id = nodeID2RoadID[make_pair(from, to)];
        EdgeInfo features = get_edge_info(edge_id, graphLength[from][idx].second);
        int lanes = max(1, features.lane_num);
        float length = max(features.length, 1.0f);
        int waiting = (int)waiting_queue[from][idx].size();
        int waiting_rows = (waiting + lanes - 1) / lanes;
        float waiting_len = waiting_rows * (vehicle_len + vehicle_gap);
        float moving_len = ((float)driving_flow[from][idx].second / lanes) * vehicle_len;
        return moving_len + waiting_len + vehicle_len <= length;
    };

    // 安排一辆车进入 from->to 行驶区，并基于 latency/range 计算到达下游节点时刻
    auto schedule_travel = [&](int vid, int from, int to, float start_t) {
        int idx = get_edge_index(from, to);
        if (idx < 0) return;
        driving_flow[from][idx].second += 1;
        int tm = nodeID2minTime[make_pair(from, to)];
        float te = tm;
        if (latency) {
            int cflow = driving_flow[from][idx].second;
            te = range ? flow2time_by_range(from, idx, cflow) : tm * (1 + sigma * pow(cflow / varphi, beta));
        }
        vehicles[vid].time = start_t + te;
        vehicles[vid].node_index += 1;
        int downstream = Pi[vid][vehicles[vid].node_index];
        ETA_result[vid][vehicles[vid].node_index].second = vehicles[vid].time;
        push_local(downstream, vehicles[vid].time, vid);
    };

    // 路口放行判定：相位是否匹配 + 下游是否有可用存储
    auto can_release = [&](int vid, int node, float now) {
        int idx = vehicles[vid].node_index;
        if (idx >= (int)Pi[vid].size() - 1) return true;
        int prev = Pi[vid][idx - 1];
        int cur = Pi[vid][idx];
        int nxt = Pi[vid][idx + 1];
        int in_edge_id = nodeID2RoadID[make_pair(prev, cur)];
        char turn = findNextEdgeDirection(in_edge_id, vid);
        int phase = (int)floor((now / phase_duration)) % phase_num;
        if (turn_to_phase(turn) != phase) return false;
        int out_idx = get_edge_index(cur, nxt);
        if (out_idx < 0) return false;
        return edge_storage_ok(cur, out_idx);
    };

    // 在一次信号事件时刻，按上游顺序尝试放行（FIFO），并受“每下游边按车道数限流”约束
    auto release_at_signal = [&](int node, float now) {
        map<int, int> lane_release_count;
        for (int up = 0; up < graphLength.size(); ++up) {
            int in_idx = get_edge_index(up, node);
            if (in_idx < 0) continue;
            if (waiting_queue[up][in_idx].empty()) continue;
            int vid = waiting_queue[up][in_idx].front();
            int idx = vehicles[vid].node_index;
            if (idx >= (int)Pi[vid].size() - 1) continue;
            int nxt = Pi[vid][idx + 1];
            int out_idx = get_edge_index(node, nxt);
            if (out_idx < 0) continue;
            int out_edge_id = nodeID2RoadID[make_pair(node, nxt)];
            EdgeInfo out_features = get_edge_info(out_edge_id, graphLength[node][out_idx].second);
            int lanes = max(1, out_features.lane_num);
            if (lane_release_count[out_edge_id] >= lanes) continue;
            if (!can_release(vid, node, now)) continue;

            waiting_queue[up][in_idx].pop_front();
            lane_release_count[out_edge_id] += 1;
            schedule_travel(vid, node, nxt, now);
        }
    };

    // 初始发车：把每条路径第一段路程入队，形成第一批车辆到达事件
    for (int i = 0; i < vehicles.size(); ++i) {
        if (Pi[i].size() > 1) {
            schedule_travel(i, Pi[i][0], Pi[i][1], vehicles[i].time);
        } else {
            vehicles[i].finished = true;
        }
    }

    int unfinished = 0;
    for (const auto &v : vehicles) {
        if (!v.finished) unfinished += 1;
    }

    // 主循环：比较“下一车辆事件”和“下一信号事件”，始终处理全局最早事件
    while (unfinished > 0 && (!top_queue.empty() || !signal_events.empty())) {
        float next_vehicle_t = top_queue.empty() ? INF : top_queue.top().t;
        float next_signal_t = signal_events.empty() ? INF : signal_events.top().t;

        // 优先处理更早（或同时间）的信号事件，随后回填该信号下一次相位事件
        if (next_signal_t <= next_vehicle_t) {
            auto ev = signal_events.top();
            signal_events.pop();
            release_at_signal(ev.node, ev.t);
            signal_events.push({ev.t + phase_duration, ev.node});
            continue;
        }

        auto top = top_queue.top();
        top_queue.pop();
        int node = top.id;
        if (local_queues[node].empty()) continue;
        auto ve = local_queues[node].top();
        local_queues[node].pop();
        if (ve.t != top.t) continue;

        // 处理车辆到达节点事件
        int vid = ve.id;
        if (vehicles[vid].finished) continue;
        int idx = vehicles[vid].node_index;
        int cur = Pi[vid][idx];

        // 车辆离开上一条边：更新行驶区流量
        if (idx > 0) {
            int prev = Pi[vid][idx - 1];
            int in_idx = get_edge_index(prev, cur);
            if (in_idx >= 0) {
                driving_flow[prev][in_idx].second = max(0, driving_flow[prev][in_idx].second - 1);
            }
        }

        if (idx == (int)Pi[vid].size() - 1) {
            vehicles[vid].finished = true;
            unfinished -= 1;
            continue;
        }

        int prev = Pi[vid][idx - 1];
        int in_idx = get_edge_index(prev, cur);
        // 信号节点：先进入等待区，按 release_step 重试（体现多周期等待）
        if (signal_node[cur]) {
            waiting_queue[prev][in_idx].push_back(vid);
            vehicles[vid].wait_time += release_step;
            push_local(cur, vehicles[vid].time + release_step, vid);
        } else {
            // 非信号节点：若下游可接纳则直接进入下一段，否则同样进入等待区重试
            int nxt = Pi[vid][idx + 1];
            int out_idx = get_edge_index(cur, nxt);
            if (out_idx >= 0 && edge_storage_ok(cur, out_idx)) {
                schedule_travel(vid, cur, nxt, vehicles[vid].time);
            } else {
                waiting_queue[prev][in_idx].push_back(vid);
                vehicles[vid].wait_time += release_step;
                push_local(cur, vehicles[vid].time + release_step, vid);
            }
        }

        if (!local_queues[node].empty()) {
            top_queue.push({local_queues[node].top().t, node});
        }
    }

    cout <<  "Algorithm I Simulation Done (new intersection waiting model)." << endl;
    return ETA_result;
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

            int edge_ID = nodeID2RoadID[make_pair(node_1, node_2)];
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
    ofstream outfile(Base + "traffic_prediction_structure_1.txt");
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

vector<vector<pair<int, float>>> Graph::cycle_aware_signal_driven_records(
        vector<vector<int>> &Q, vector<vector<int>> &routeRoadIDInput) {
    this->routeRoadID = routeRoadIDInput;
    route_roadID_2_movementID();

    int simStartTime = 0;
    if (!Q.empty()) {
        simStartTime = Q[0][2];
        for (const auto &q : Q) simStartTime = min(simStartTime, q[2]);
    }

    initialize_cycle_aware_vehicles(Q, routeRoadIDInput);
    initialize_signal_event_queue(simStartTime);

    int currentTime = simStartTime;
    while (!allVehiclesFinished()) {
        while (!signalEventPQ.empty() && signalEventPQ.top().time == currentTime) {
            SignalEvent e = signalEventPQ.top();
            signalEventPQ.pop();
            handle_signal_change_event(e);
            int nextT = nextSignalChangeTime(e.signalID, currentTime + 1);
            if (nextT < INF) {
                signalEventPQ.push({nextT, e.signalID});
            }
        }

        int nextTime = signalEventPQ.empty() ? INF : signalEventPQ.top().time;
        process_discharge_window(currentTime, nextTime);

        if (nextTime == INF) {
            break;
        }
        currentTime = nextTime;
    }
    return ETA_result_cycle_aware;
}

void Graph::initialize_cycle_aware_vehicles(vector<vector<int>>& Q, vector<vector<int>>& routeRoadIDInput) {
    vehicles.clear();
    vehicles.resize(routeRoadIDInput.size());
    ETA_result_cycle_aware.assign(routeRoadIDInput.size(), {});
    finishedVehicleCount = 0;
    usedDischargeCapacity.clear();

    for (auto &b : waitingBuffers) {
        b.vehicleQueue.clear();
    }

    for (int i = 0; i < routeRoadIDInput.size(); ++i) {
        VehicleLabel v;
        v.vehicleID = i;
        v.routeID = i;
        v.routeRoadIDs = routeRoadIDInput[i];
        if (i < routeMovementID.size()) v.routeMovementIDs = routeMovementID[i];
        v.roadIndex = 0;

        if (v.routeRoadIDs.empty()) {
            v.finished = true;
            v.state = VehicleState::Finished;
            vehicles[i] = v;
            finishedVehicleCount++;
            continue;
        }

        int departTime = (i < Q.size() && Q[i].size() > 2) ? Q[i][2] : 0;
        v.currentRoadID = v.routeRoadIDs[0];
        v.arrivalTime = departTime + predictRoadTravelTime(v.currentRoadID, i);

        ETA_result_cycle_aware[i].push_back({v.currentRoadID, static_cast<float>(departTime)});

        if (v.routeRoadIDs.size() == 1) {
            v.finished = true;
            v.state = VehicleState::Finished;
            recordFinalETA(i, v.arrivalTime);
        } else if (!v.routeMovementIDs.empty()) {
            int movementID = v.routeMovementIDs[0];
            auto itRoad = roads[v.currentRoadID].movementIDToWaitingBufferID.find(movementID);
            if (itRoad != roads[v.currentRoadID].movementIDToWaitingBufferID.end()) {
                v.currentMovementID = movementID;
                v.currentBufferID = itRoad->second;
                v.state = VehicleState::WaitingAtIntersection;
                vehicles[i] = v;
                insertVehicleToBufferOrdered(v.currentBufferID, i);
                continue;
            } else {
                v.finished = true;
                v.state = VehicleState::Finished;
                recordFinalETA(i, v.arrivalTime);
            }
        } else {
            v.finished = true;
            v.state = VehicleState::Finished;
            recordFinalETA(i, v.arrivalTime);
        }

        vehicles[i] = v;
    }
}

void Graph::initialize_signal_event_queue(int simStartTime) {
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

bool Graph::isMovementActive(int movementID, int t) {
    if (movementID < 0 || movementID >= movements.size()) return false;
    const Movement &m = movements[movementID];
    if (m.alwaysOpen) return true;
    if (m.signalID < 0 || m.signalID >= signals.size()) return false;
    return signalStateAt(m.signalID, t) == SignalState::Green ||
           signalStateAt(m.signalID, t) == SignalState::AlwaysOpen;
}

bool Graph::canDischarge(int movementID, int dischargeTime, int windowEnd) {
    (void)windowEnd;
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
    if (!hasDischargeCapacity(m.intersectionID, m.toRoadID, dischargeTime)) return false;
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
    consumeDischargeCapacity(m.intersectionID, m.toRoadID, dischargeTime);

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
        v.finished = true;
        v.state = VehicleState::Finished;
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
            v.finished = true;
            v.state = VehicleState::Finished;
            result.finished = true;
            recordFinalETA(vehicleID, v.arrivalTime);
        }
    } else {
        v.finished = true;
        v.state = VehicleState::Finished;
        result.finished = true;
        recordFinalETA(vehicleID, v.arrivalTime);
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
    const Movement &m = movements[movementID];
    int t = max(readyTime, currentTime);
    int capT = nextAvailableCapacityTime(m.intersectionID, m.toRoadID, t);
    return max(t, capT);
}

bool Graph::hasDischargeCapacity(int intersectionID, int toRoadID, int t) {
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    int used = usedDischargeCapacity[make_tuple(intersectionID, toRoadID, slot)];
    int cap = 1;
    if (toRoadID >= 0 && toRoadID < roads.size()) {
        cap = max(1, roads[toRoadID].laneNum);
    }
    return used < cap;
}

void Graph::consumeDischargeCapacity(int intersectionID, int toRoadID, int t) {
    int interval = max(1, defaultDischargeInterval);
    int slot = t / interval;
    usedDischargeCapacity[make_tuple(intersectionID, toRoadID, slot)]++;
}

int Graph::nextAvailableCapacityTime(int intersectionID, int toRoadID, int t) {
    int interval = max(1, defaultDischargeInterval);
    int cur = t;
    while (!hasDischargeCapacity(intersectionID, toRoadID, cur)) {
        cur = ((cur / interval) + 1) * interval;
    }
    return cur;
}

bool Graph::hasDownstreamStorage(int roadID) {
    (void)roadID;
    return true;
}

int Graph::predictRoadTravelTime(int roadID, int vehicleID) {
    (void)vehicleID;
    if (roadID >= 0 && roadID < roads.size() && roads[roadID].minTravelTime > 0) {
        return max(1, static_cast<int>(round(roads[roadID].minTravelTime)));
    }
    if (roadID >= 0 && roadID < roads.size()) {
        double speed = max(1.0, roads[roadID].speedLimit);
        return max(1, static_cast<int>(round(roads[roadID].length / speed)));
    }
    return 1;
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
    if (e.signalID < 0 || e.signalID >= signals.size()) return;
    signals[e.signalID].currentState = signalStateAt(e.signalID, e.time);
}

int Graph::nextSignalChangeTime(int signalID, int afterTime) {
    if (signalID < 0 || signalID >= signals.size()) return INF;
    const SignalController &s = signals[signalID];
    if (s.alwaysOpen || s.cycleLength <= 0) return INF;

    int cycle = s.cycleLength;
    int best = INF;
    int baseK = (afterTime - s.offset) / cycle;
    for (int k = baseK - 1; k <= baseK + 2; ++k) {
        int t1 = s.offset + k * cycle + s.greenStart;
        int t2 = s.offset + k * cycle + s.greenEnd;
        if (t1 >= afterTime) best = min(best, t1);
        if (t2 >= afterTime) best = min(best, t2);
    }
    return best;
}

SignalState Graph::signalStateAt(int signalID, int t) {
    if (signalID < 0 || signalID >= signals.size()) return SignalState::Red;
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
    if (vehicleID < 0 || vehicleID >= ETA_result_cycle_aware.size()) return;
    if (vehicleID >= 0 && vehicleID < vehicles.size() && vehicles[vehicleID].finished) return;
    ETA_result_cycle_aware[vehicleID].push_back({-1, static_cast<float>(finalTime)});
    if (vehicleID >= 0 && vehicleID < vehicles.size()) {
        vehicles[vehicleID].finished = true;
    }
    finishedVehicleCount++;
}
