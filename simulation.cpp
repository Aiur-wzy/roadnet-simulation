
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

    // Step 1: Variable Initialization
    // -------------------------------------------------------
    benchmark::heap<2, int, int> H(Q.size());
    // Initialize travel time for routes with 'INF' as temporal information
    ETA_result.resize(Pi.size());
    for (int i = 0; i < Pi.size(); ++i) {
        ETA_result[i].resize(Pi[i].size());
        ETA_result[i][0].first = Pi[i][0];
        ETA_result[i][0].second = Q[i][2];
        for (int k = 1; k < Pi[i].size(); ++k) {
            ETA_result[i][k].first = Pi[i][k];
            ETA_result[i][k].second = INF;
        }
    }
    // Initialize Road Segment's Traffic Flow
    vector<vector<pair<int, int>>> F;
    F.resize(graphLength.size());
    for (int i = 0; i < F.size(); i++) {
        F[i].resize(graphLength[i].size());
        for (int j = 0; j < F[i].size(); j++) {
            F[i][j].first = graphLength[i][j].first;
            F[i][j].second = 0;
        }
    }
    // Initialize driving vehicles' ID on edges
    // node_1: (node_2, (vehicle_1, vehicle_2,...)), (node_3, (vehicle_3, vehicle_4,...))
    vector<vector<pair<int, vector<int>>>> traveling_vehicles_on_edge;
    traveling_vehicles_on_edge.reserve(graphLength.size());
    for (int i = 0; i < graphLength.size(); ++i) {
        traveling_vehicles_on_edge[i].resize(graphLength[i].size());
        for (int j = 0; j < graphLength[i].size(); ++j) {
            traveling_vehicles_on_edge[i][j].first = graphLength[i][j].first;
        }
    }
    // Initialize time records when traffic flow Change
    timeFlowChange.clear();
    timeFlowChange.resize(F.size());
    for (int i = 0; i < timeFlowChange.size(); i++) {
        timeFlowChange[i].resize(F[i].size()); // num of nei
        for (int j = 0; j < F[i].size(); j++) {
            timeFlowChange[i][j].first = F[i][j].first;
        }
    }
    // Initialize route label
    vector<tuple<int, int, float>> label(Q.size());
    int ini_node_index = 0;
    for (int i = 0; i < Q.size(); i++) {
        label[i] = std::make_tuple(i, ini_node_index, Q[i][2]);
        float current_time = get<2>(label[i]);
        H.update(i, current_time); // query index, departure time
    }
    // Store input features to construct model catching dictionary
    std::ofstream outFile;
    if (write) {
        outFile.open(model_catching);
        if (!outFile.is_open()) {
            std::cerr << "Unable to open file: " << model_catching << std::endl;
            return ETA_result;
        }
    }

    // Step 2: Simulation
    // -------------------------------------------------------
    std::chrono::high_resolution_clock::time_point t0_1, t0_2;
    std::chrono::duration<double> time_span;
    t0_1=std::chrono::high_resolution_clock::now();

    int current_label_index, current_time, current_node_index;
    while (!H.empty()){
        // 提取 vehicle_ID 和 时间
        H.extract_min(current_label_index, current_time);
        current_node_index = get<1>(label[current_label_index]);
        int current_node = Pi[current_label_index][current_node_index];

        int sock;
        if (server == true){
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock == -1) {
                std::cerr << "Could not create socket." << std::endl;
                return {};
            }
            sockaddr_in server_addr;
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(65432);
            server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
            // 连接服务器
            if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                std::cerr << "Connection failed with error: " << strerror(errno) << std::endl;
                close(sock);
                return {};
            }
        }

        // Step 3: If Current Node is The Last One of The Path
        // -------------------------------------------------------
        if (current_node_index == (Pi[current_label_index].size() - 1)) {
            int previous_node = Pi[current_label_index][current_node_index - 1];

            for (int i = 0; i < F[previous_node].size(); i++) {
                if(F[previous_node][i].first == current_node) {
                    // 更新对应的 edge 上的 traffic flow 的值
                    F[previous_node][i].second = F[previous_node][i].second - 1;
                    // 更新对应的 edge 上的 vehicle_ID 的值
                    auto& vehicle_vector = traveling_vehicles_on_edge[previous_node][i].second;
                    vehicle_vector.erase(std::remove(vehicle_vector.begin(), vehicle_vector.end(), current_label_index), vehicle_vector.end());

                    if (server == true) {
                        // 占位置的，没有实际作用
                        // 1）根据 node_1 和 node_2 拿到对应该 edge_ID
                        int edge_id = 0;
                        // 2）读取车辆的转向信息
                        char Turn = 's';
                        // 3）读取 edge 上其他车辆的转向信息
                        set<char> Turn_Stat = {'s'};
                        // 通过封装好的函数发送数据
                        if(!sendDataToPython(sock, edge_id, Turn, Turn_Stat, F[previous_node][i].second)) {
                            close(sock); // 发送失败时关闭socket
                            return {};
                        }
                        // Shutdown the connection for sending
                        shutdown(sock, SHUT_WR);
                        // cout << "send "  << edge_id << " " << Turn << " " << Turn_Stat.size() << " " << endl;
                        // 从服务器接收处理后的数据
                        char buffer[1024] = {0};
                        if(recv(sock, buffer, sizeof(buffer), 0) < 0) {
                            std::cerr << "Failed to receive data." << std::endl;
                            close(sock);
                            return {};
                        }
                        // std::cout << "Received from server: " << buffer << std::endl;
                        // 关闭socket
                        close(sock);
                    }

                    // 由于 flow 改变，记录这个时间下，该 edge 上的，vehicle ID/驶入驶出状态/flow 的值
                    /*int record_time = label[current_label_index][2];*/
                    int record_time = get<2>(label[current_label_index]);
                    if (timeFlowChange[previous_node][i].second.find(record_time) ==
                        timeFlowChange[previous_node][i].second.end())
                    {
                        timeFlowChange[previous_node][i].second.insert(
                                pair<int, vector<vector<int>>>(
                                        get<2>(label[current_label_index]),
                                        {{current_label_index, 0, F[previous_node][i].second}}));
                    }
                    else {
                        timeFlowChange[previous_node][i].second[record_time].push_back(
                                {current_label_index, 0, F[previous_node][i].second});
                    }
                }
                continue;
            }
        }
            // Step 4: If Current Node is The First One of The Path
            // -------------------------------------------------------
        else {
            int next_node = Pi[current_label_index][current_node_index + 1];
            if (current_node_index == 0) {
                // for 循环找到当前的 edge，也就是 node pair，i 是 next node index
                for (int i = 0; i < F[current_node].size(); i++) {
                    if(F[current_node][i].first == next_node) {
                        // 更新 edge 上的 traffic flow
                        F[current_node][i].second = F[current_node][i].second + 1;
                        // 更新对应的 edge 上的 vehicle_ID 的值
                        auto& vehicle_vector = traveling_vehicles_on_edge[current_node][i].second;
                        vehicle_vector.push_back(current_label_index);

                        // 由于 flow 改变，记录这个时间下，该 edge 上的，vehicle ID/驶入驶出状态/flow 的值
                        int record_time = get<2>(label[current_label_index]);
                        if (timeFlowChange[current_node][i].second.find(record_time) ==
                            timeFlowChange[current_node][i].second.end())
                        {
                            timeFlowChange[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[current_label_index]),
                                    {{current_label_index, 1, F[current_node][i].second}}));
                        }
                        else {
                            timeFlowChange[current_node][i].second[record_time].push_back(
                                    {current_label_index, 1, F[current_node][i].second});
                        }

                        // edit：应该把这部分代码替换掉，换成以下步骤
                        // 1）根据 node_1 和 node_2 拿到对应该 edge_ID
                        int node_2 = F[current_node][i].first;
                        int edge_id = nodeID2RoadID[make_pair(current_node, node_2)];
                        // 2）读取车辆的转向信息
                        char Turn = findNextEdgeDirection(edge_id, current_label_index);
                        if (Turn == 'e') {
                            Turn = 's';
                        }
                        // 3）读取 edge 上其他车辆的转向信息
                        set<char> Turn_Stat = processVehicleDirections(vehicle_vector, edge_id);

                        float te_latency, te_server, te_catching;
                        if (server) {
                            // 通过封装好的函数发送数据
                            if(!sendDataToPython(sock, edge_id, Turn, Turn_Stat, F[current_node][i].second)) {
                                close(sock); // 发送失败时关闭socket
                                return {};
                            }
                            // Shutdown the connection for sending
                            shutdown(sock, SHUT_WR);
                            int node_first = roadID2NodeID[edge_id].first;
                            int node_second = roadID2NodeID[edge_id].second;
                            int edge_len = graphLength[node_first][node_second].second;
                            // cout << "send "  << edge_id << " " << edge_len << " " << Turn << " " << Turn_Stat.size() << " " << F[current_node][i].second << endl;
                            // 从服务器接收处理后的数据
                            char buffer[1024] = {0};
                            if(recv(sock, buffer, sizeof(buffer), 0) < 0) {
                                std::cerr << "Failed to receive data." << std::endl;
                                close(sock);
                                return {};
                            }
                            // cout << "Received from server: " << buffer << endl;
                            float travel_time_predict = std::stof(buffer);
                            te_server = travel_time_predict;
                            // 关闭socket
                            close(sock);
                        }

                        if (catching) {

                            int delay_time = route_time_Dict[current_label_index][current_node][0];
                            int low_time = route_time_Dict[current_label_index][current_node][1];
                            int wait_time = route_time_Dict[current_label_index][current_node][2];
                            int driving_num = route_time_Dict[current_label_index][current_node][3];

                            if (driving_num <= 1) {
                                small += 1;
                            } else {
                                big += 1;
                            }

                            std::random_device rd;       // 用于生成随机数种子
                            std::mt19937 gen(rd());      // 标准的梅森旋转算法随机数生成器
                            std::uniform_real_distribution<> dis(0.0, 1.0);  // 生成 [0, 1] 之间的随机数

                            // 生成一个随机数，如果它小于 0.1 (10%)，执行 driving_num 计算
                            if (dis(gen) < (1 - percent)) {
                                driving_num = 0;
                            }

                            /*for (const auto& edge : edge_id_to_features) {
                                std::cout << "Edge ID: " << edge.first
                                          << ", Lane Num: " << edge.second.lane_num
                                          << ", Speed: " << edge.second.speed
                                          << ", Length: " << edge.second.length
                                          << ", Edge Str: " << edge.second.edge_str
                                          << std::endl;
                            }*/

                            EdgeInfo features = edge_id_to_features[edge_id];
                            int lane_num = features.lane_num;
                            float speed = features.speed;
                            float length = features.length;
                            int ratio = static_cast<int>(std::round(length / speed));  // 假设 length 和 speed 是浮点数
                            int log_length = static_cast<int>(std::round(log(length)));  // 先取对数，再四舍五入
                            int length_square = static_cast<int>(std::round(pow(length, 2)));  // 先平方，再四舍五入


                            // string edge_str = features.edge_str;
                            // edge_str.pop_back();

                            /*cout << "edge_id:" << edge_id << endl;
                            cout << edge_id_to_features[edge_id].edge_str << endl;
                            cout << edge_id_to_features[edge_id].edge_str.size() << endl;
                            cout << edge_str[0] << endl;
                            cout << edge_str[7] << endl;
                            cout << edge_str[8] << endl;
                            cout << edge_str << ":" << endl;
                            cout << "----------" << endl;*/

                            // exit(0);

                            // RoadKey queryKey = {lane_num, speed, length, F[current_node][i].second, delay_time, low_time, wait_time, ratio, length_square};
                            RoadKey queryKey = {lane_num, speed, length, driving_num, delay_time, low_time, wait_time, ratio, length_square};

                            /*cout << queryKey.edge_str << endl;*/

                            /*cout << queryKey.edge_str << " " << queryKey.lane_num << " " << queryKey.speed_limit << " " << queryKey.edge_length << " " << queryKey.driving_number << " " << queryKey.delay_time << " ";
                            cout << queryKey.lowSpee_time << " " << queryKey.wait_time << endl;*/

                            // cout << edge_str << " " << lane_num << " " << speed << " " << length << " ";
                            // cout << F[current_node][i].second << " " << delay_time << " " << low_time << " " << wait_time << endl;

                            if (dictionary.find({lane_num, speed, length, F[current_node][i].second, delay_time, low_time, wait_time, ratio, length_square}) != dictionary.end()) {
                                te_catching = dictionary[queryKey];
                                catching_found += 1;
                            } else {
                                te_catching = 0;
                                catching_no_found += 1;
                                /*
                                cout << "No travel time found for query key " << dictionary[queryKey] << endl;
                                std::cout << "No travel time found for query key" << std::endl;
                                cout << "lane_num: " << lane_num << endl;
                                cout << "speed: " << speed << endl;
                                cout << "length: " << length << endl;
                                cout << "F[current_node][i].second: " << F[current_node][i].second << endl;
                                cout << "delay_time: " << delay_time << endl;
                                cout << "low_time: " << low_time << endl;
                                cout << "wait_time: " << wait_time << endl;
                                cout << "ratio: " << ratio << endl;
                                cout << "log_length: " << log_length << endl;
                                cout << "length_square: " << length_square << endl;
                                cout << "------------------------" << endl;
                                */
                            }
                        }

                        // if (round(te_catching * 10000) / 10000 != round(te_server * 10000) / 10000)
                        //    cout << round(te_catching * 10000) / 10000 << " " << round(te_catching * 10000) / 10000 << endl;

                        if (write) {
                            outFile << edge_id << " " << Turn << " " << F[current_node][i].second << endl;
                        }

                        if (latency) {
                            //Estimate Travel Time Based on Latency Function
                            int tm = nodeID2minTime[make_pair(current_node, next_node)];
                            int Cflow = F[current_node][i].second;
                            if (range == false){
                                te_latency = tm * (1 + sigma * pow(Cflow/varphi, beta));
                                minTravel += tm;
                                realTravel += te_latency;
                            }
                            else{
                                te_latency = flow2time_by_range(current_node, i, Cflow);
                                minTravel += tm;
                                realTravel += te_latency;
                            }
                        }

                        float te;
                        // 根据 te_choose 的值决定 te 的值
                        if (te_choose == "server") {
                            te = te_server;
                        } else if (te_choose == "catching") {
                            te = te_catching;
                        } else if (te_choose == "latency") {
                            te = te_latency;
                        } else {
                            std::cerr << "Error: Invalid option '" << te_choose << "'." << std::endl;
                        }

                        //Update Label, Travel Time, Nodes Label, and Priority Queue
                        get<2>(label[current_label_index]) = get<2>(label[current_label_index]) + te;
                        get<1>(label[current_label_index]) = get<1>(label[current_label_index]) + 1;
                        ETA_result[current_label_index][current_node_index+1].second = get<2>(label[current_label_index]);
                        H.update(get<0>(label[current_label_index]), get<2>(label[current_label_index]));
                    }
                }
            }
                // Step 5: If Current Node is Not The First One of The Path
                // -------------------------------------------------------
            else {
                int previous_node = Pi[current_label_index][current_node_index-1];
                for (int i = 0; i < F[previous_node].size(); i++) {
                    if(F[previous_node][i].first == current_node) {
                        F[previous_node][i].second = F[previous_node][i].second - 1;
                        // 更新对应的 edge 上的 vehicle_ID 的值
                        auto& vehicle_vector = traveling_vehicles_on_edge[previous_node][i].second;
                        vehicle_vector.erase(std::remove(vehicle_vector.begin(), vehicle_vector.end(), current_label_index), vehicle_vector.end());

                        int record_time = get<2>(label[current_label_index]);
                        if (timeFlowChange[previous_node][i].second.find(record_time) ==
                            timeFlowChange[previous_node][i].second.end())
                        {
                            timeFlowChange[previous_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[current_label_index]),
                                    {{current_label_index, 0, F[previous_node][i].second}}));
                        }
                        else {
                            timeFlowChange[previous_node][i].second[record_time].push_back(
                                    {current_label_index, 0, F[previous_node][i].second});
                        }
                    }
                }
                for (int i = 0; i < F[current_node].size(); i++) {
                    if(F[current_node][i].first == next_node) {
                        F[current_node][i].second = F[current_node][i].second + 1;
                        // 更新对应的 edge 上的 vehicle_ID 的值
                        auto& vehicle_vector = traveling_vehicles_on_edge[current_node][i].second;
                        vehicle_vector.push_back(current_label_index);

                        int record_time = get<2>(label[current_label_index]);
                        if (timeFlowChange[current_node][i].second.find(record_time) ==
                            timeFlowChange[current_node][i].second.end())
                        {
                            timeFlowChange[current_node][i].second.insert(pair<int, vector<vector<int>>>(
                                    get<2>(label[current_label_index]),
                                    {{current_label_index, 1, F[current_node][i].second}}));
                        }
                        else {
                            timeFlowChange[current_node][i].second[record_time].push_back({current_label_index, 1, F[current_node][i].second});
                        }

                        // 应该把这部分代码替换掉，换成以下步骤
                        // 1）根据 node_1 和 node_2 拿到对应该 edge_ID
                        int node_2 = F[current_node][i].first;
                        int edge_id = nodeID2RoadID[make_pair(current_node, node_2)];
                        // 2）读取车辆的转向信息
                        char Turn = findNextEdgeDirection(edge_id, current_label_index);
                        if (Turn == 'e') {
                            Turn = 's';
                        }
                        // 3）读取 edge 上其他车辆的转向信息
                        set<char> Turn_Stat = processVehicleDirections(vehicle_vector, edge_id);

                        float te_latency, te_server, te_catching;

                        if (server) {
                            // 通过封装好的函数发送数据
                            if(!sendDataToPython(sock, edge_id, Turn, Turn_Stat, F[current_node][i].second)) {
                                close(sock); // 发送失败时关闭socket
                                return {};
                            }
                            // Shutdown the connection for sending
                            // shutdown(sock, SHUT_WR);
                            int node_first = roadID2NodeID[edge_id].first;
                            int node_second = roadID2NodeID[edge_id].second;
                            int edge_len = graphLength[node_first][node_second].second;
                            // cout << "send "  << edge_id << " " << edge_len << " " << Turn << " " << Turn_Stat.size() << " " << F[current_node][i].second << endl;
                            // 从服务器接收处理后的数据
                            char buffer[1024] = {0};
                            if(recv(sock, buffer, sizeof(buffer), 0) < 0) {
                                std::cerr << "Failed to receive data." << std::endl;
                                close(sock);
                                return {};
                            }
                            // cout << "Received from server: " << buffer << endl;
                            float travel_time_predict = std::stof(buffer);
                            te_server = travel_time_predict;
                            // 关闭socket
                            close(sock);
                        }

                        if (catching) {

                            int delay_time = route_time_Dict[current_label_index][current_node][0];
                            int low_time = route_time_Dict[current_label_index][current_node][1];
                            int wait_time = route_time_Dict[current_label_index][current_node][2];
                            int driving_num = route_time_Dict[current_label_index][current_node][3];

                            if (driving_num <= 1) {
                                small += 1;
                            } else {
                                big += 1;
                            }

                            std::random_device rd;       // 用于生成随机数种子
                            std::mt19937 gen(rd());      // 标准的梅森旋转算法随机数生成器
                            std::uniform_real_distribution<> dis(0.0, 1.0);  // 生成 [0, 1] 之间的随机数

                            // 生成一个随机数，如果它小于 0.1 (10%)，执行 driving_num 计算
                            if (dis(gen) < (1 - percent)) {
                                driving_num = 0;
                            }

                            EdgeInfo features = edge_id_to_features[edge_id];
                            int lane_num = features.lane_num;
                            float speed = features.speed;
                            float length = features.length;
                            int ratio = static_cast<int>(std::round(length / speed));  // 假设 length 和 speed 是浮点数
                            // int log_length = static_cast<int>(std::round(log(length)));  // 先取对数，再四舍五入
                            int length_square = static_cast<int>(std::round(pow(length, 2)));  // 先平方，再四舍五入
                            // string edge_str = features.edge_str;
                            // edge_str.pop_back();

                            /*cout << "edge_str:" << endl;
                            cout << lane_num << "," << speed << "," << length<<endl;
                            cout << edge_str.size() << endl;
                            cout << features.edge_str << "::" << endl;
                            cout << "----------" << endl;*/

                            /*cout << "edge_str:" << edge_id
                            cout << edge_id_to_features[edge_id].edge_str << endl;
                            cout << edge_str << ":" << endl;
                            cout << "----------" << endl;*/

                            // RoadKey queryKey = {"162041588#1", 2, 11, 56, 1, 0, 0, 0};  // 这是我们想要查找的键

                            // RoadKey queryKey = {lane_num, speed, length, F[current_node][i].second, delay_time, low_time, wait_time, ratio, length_square};
                            RoadKey queryKey = {lane_num, speed, length, driving_num, delay_time, low_time, wait_time, ratio, length_square};

                            /*cout << queryKey.edge_str << endl;*/

                            /*cout << queryKey.edge_str << " " << queryKey.lane_num << " " << queryKey.speed_limit << " " << queryKey.edge_length << " " << queryKey.driving_number << " " << queryKey.delay_time << " ";
                            cout << queryKey.lowSpee_time << " " << queryKey.wait_time << endl;*/


                            // cout << edge_str << endl;
                            // cout << edge_str << " " << lane_num << " " << speed << " " << length << " ";
                            // cout << F[current_node][i].second << " " << delay_time << " " << low_time << " " << wait_time << endl;

                            if (dictionary.find({lane_num, speed, length, F[current_node][i].second, delay_time, low_time, wait_time, ratio, length_square}) != dictionary.end()) {
                                te_catching = dictionary[queryKey];
                                catching_found += 1;
                            } else {
                                // cout << "No travel time found for query key " << dictionary[queryKey] << endl;
                                te_catching = 0;
                                catching_no_found += 1;
                                /*
                                std::cout << "No travel time found for query key" << std::endl;
                                cout << "lane_num: " << lane_num << endl;
                                cout << "speed: " << speed << endl;
                                cout << "length: " << length << endl;
                                cout << "F[current_node][i].second: " << F[current_node][i].second << endl;
                                cout << "delay_time: " << delay_time << endl;
                                cout << "low_time: " << low_time << endl;
                                cout << "wait_time: " << wait_time << endl;
                                cout << "ratio: " << ratio << endl;
                                cout << "length_square: " << length_square << endl;
                                cout << "------------------------" << endl;
                                */
                            }
                            // cout << "catching travel time is: " << te_catching << endl;
                        }

                        // if (round(te_catching * 10000) / 10000 != round(te_server * 10000) / 10000)
                        //    cout << round(te_catching * 10000) / 10000 << " " << round(te_catching * 10000) / 10000 << endl;

                        if (write) {
                            outFile << edge_id << " " << Turn << " " << F[current_node][i].second << endl;
                        }

                        if (latency) {
                            // Estimate Travel Time Based on Latency Function
                            int tm = nodeID2minTime[make_pair(current_node, next_node)];
                            int Cflow = F[current_node][i].second;
                            if (range == false){
                                te_latency = tm * (1 + sigma * pow(Cflow/varphi, beta));
                                minTravel += tm;
                                realTravel += te_latency;
                            }
                            else{
                                te_latency = flow2time_by_range(current_node, i, Cflow);
                                minTravel += tm;
                                realTravel += te_latency;
                            }
                        }

                        float te;
                        // 根据 te_choose 的值决定 te 的值
                        if (te_choose == "server") {
                            te = te_server;
                        } else if (te_choose == "catching") {
                            te = te_catching;
                        } else if (te_choose == "latency") {
                            te = te_latency;
                        } else {
                            std::cerr << "Error: Invalid option '" << te_choose << "'." << std::endl;
                        }

                        // Update Label, Travel Time, Nodes Label, and Priority Queue
                        get<2>(label[current_label_index]) = get<2>(label[current_label_index]) + te;
                        get<1>(label[current_label_index]) = get<1>(label[current_label_index]) + 1;
                        ETA_result[current_label_index][current_node_index+1].second = get<2>(label[current_label_index]);
                        H.update(get<0>(label[current_label_index]), get<2>(label[current_label_index]));
                    }
                }
            }
        }
    }

    /*
    // Step 6: Print nodes_label
    // -------------------------------------------------------
    for (int i=0;i<nodes_labsocket doneel.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<nodes_label[i].size();j++){
            cout << nodes_label[i][j][1] << " " << nodes_label[i][j][2] << " ";
        }
        cout << endl;
    }
    */

    /*
    // Step 7: Print timeFlowChange
    // -------------------------------------------------------
    for (int i=0;i<timeFlowChange.size();i++){
        cout << "node1: " << i << endl;
        for (int j=0;j<F[i].size();j++){
            cout << " node2: " << timeFlowChange[i][j].first;
            cout << " with size: " << timeFlowChange[i][j].second.size() << " ";
            map<int, vector<vector<int>>>::iterator itr;
            for (itr = timeFlowChange[i][j].second.begin(); itr != timeFlowChange[i][j].second.end(); ++itr) {
                for (int i=0;i<itr->second.size();i++){
                    cout << " time " << itr->first << " routeID " << itr->second[i][0];
                    cout << " status " << itr->second[i][1] << " flow " << itr->second[i][2] << "||";
                }
            }
            cout << "\n" << endl;
        }
        cout << "\n" << endl;
    }
    */

    /*
    // Step 8: Print ETA_result
    // -------------------------------------------------------
    for (int i=0;i<ETA_result.size();i++){
        cout << "path " << i << ": ";
        for (int j=0;j<ETA_result[i].size();j++){
            cout << "node " << ETA_result[i][j].first << " with ";
            cout << "time " << ETA_result[i][j].second << " ";
        }
        cout << endl;
    }
    */

    cout <<  "Algorithm I Simulation Done." << endl;

    t0_2=std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t0_2-t0_1);

    // Close opened file
    if (outFile.is_open())
        outFile.close();
    if (range == false)
        cout << "Algorithm 1 simulation time consumption without range is: " << time_span.count() <<endl;
    else
        cout << "Algorithm 1 simulation time consumption is: " << time_span.count() << "s."<< endl;

    // Step 9: return ETA_result
    // -------------------------------------------------------

    /*
    int timeAll = 0;
    int timeTemp = 0;
    for (int i = 0; i < ETA_result.size(); i++){
        timeTemp = ETA_result[i][ETA_result[i].size() - 1].second - ETA_result[i][0].second;
        if (timeTemp < 0){
            cout << "Error. Simulation Error." << endl;
        }
        timeAll += timeTemp;
    }
    cout << "timeAll is: " << timeAll << endl;
    cout << "average timeAll is: " << timeAll / Q.size() << endl;
    */

    for (int i = 0; i < ETA_result.size(); ++i) {
        float travelTime = ETA_result[i][ETA_result[i].size()-1].second - ETA_result[i][0].second;
        // cout << "predicted travel time of route " << i << " is: " << travelTime << endl;
    }
    cout << endl;

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

