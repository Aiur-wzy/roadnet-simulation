#include "head.h"


// Read Graph
void Graph::read_graph()
{

    // 使用当前主数据源 "BJ" 构建基础路网（节点、边、RoadID 映射）
    ifstream IF(BJ);    //ID1, ID2, Weight (Length in Meters)
    if(!IF)
        cout<<"Cannot open Graph BJ"<<endl;
    //Read Node Number and Edge Number
    IF>>nodenum>>edgenum; // 296710 774660

    // Variable Initialization
    nodeID2RoadID.clear();
    roadID2NodeID.clear();
    vector<pair<int,int>> vecp;
    vecp.clear();
    vector<pair<int, float>> vecp_float;
    vecp_float.clear();
    graphLength.assign(nodenum, vecp_float);         //ID1, ID2, Length
    graphRoadID.assign(nodenum, vecp);         //ID1, ID2, RoadID
    roadInfor.reserve(edgenum);                //Road Info (ID1, ID2, RoadID, Length, Time)
    set<int> setp; setp.clear();
    adjNodes.assign(nodenum, setp);            //ID1, ID2
    int ID1, ID2, edgeID;
    float length;
    // Avoid Redundant Edge
    set<pair<int,int>> edgeRedun;              //ID1, ID2
    // Read Graph Length as Weight
    for(int i=0;i<edgenum;i++){
        IF >> ID1 >> ID2 >> edgeID >> length;
        //Store Road Information
        Road r;
        r.ID1 = ID1;
        r.ID2 = ID2;
        r.roadID = edgeID;
        r.length = length;
        r.travelTime = -1;
        roadInfor.push_back(r);
        //Construct Maps
        nodeID2RoadID.insert(make_pair(make_pair(ID1, ID2), edgeID));    //<ID1, ID2> -> RoadID
        roadID2NodeID.insert(make_pair(edgeID, make_pair(ID1, ID2)));    //RoadID -> <ID1, ID2>
        //Check Redundant
        if(edgeRedun.find(make_pair(ID1,ID2))==edgeRedun.end())
        {
            graphLength[ID1].push_back(make_pair(ID2, length));
            graphRoadID[ID1].push_back(make_pair(ID2, edgeID));
            adjNodes[ID1].insert(ID2);
        }
        edgeRedun.insert(make_pair(ID1,ID2));
    }
    // Open File "BJ_minTravleTime"
    ifstream IFTime(BJ_minTravleTime);
    if(!IFTime){
        cout<<"Cannot open Map BJ_minTravleTime"<<endl;
    }
    // Read Node Number
    int nodenum, edgenum;
    IFTime>>nodenum>>edgenum; // 296710
    //Variable Initialization
    vector<pair<int,int>> vecpTIme; vecpTIme.clear();
    graphTime.assign(nodenum, vecp);
    int ID1Time, ID2Time;
    double minTime;
    // Avoid Redundant Information
    set<pair<int,int>> edgeRedunTime;
    // Read Graph Min Travel Time as Weight
    for(int i=0;i<edgenum;i++){
        IFTime>>ID1>>ID2>>minTime;
        minTime = (int) minTime;
        // Construct Maps
        nodeID2minTime.insert(make_pair(make_pair(ID1, ID2), minTime));  //<ID1, ID2> -> Min Travel Time
        // Check Redundant
        if(edgeRedunTime.find(make_pair(ID1,ID2))==edgeRedunTime.end()){
            graphTime[ID1].push_back(make_pair(ID2, minTime));
        }
        edgeRedunTime.insert(make_pair(ID1,ID2));
    }
}

// Read Road Information from Path "beijingMoreRoadInfo"
// More Road Information Contains in Another File
void Graph::read_road_info()
{
    // Read File
    ifstream IFNodeRoadID(beijingMoreRoadInfo);
    if(!IFNodeRoadID)
        cout<<"Cannot open beijingMoreRoadInfo"<<endl;
    // Variable Initialization
    int nodeID1;
    int nodeID2;
    int routeIDLei;
    int length;
    int direction;
    int speedLimit;
    int laneNum;
    int width;
    int kindNumber;
    string kind;
    int routeIDGen;
    int i = 0;
    roadInforMore.resize(387587);
    // Read In Info
    while(IFNodeRoadID >> nodeID1){
        IFNodeRoadID >> nodeID2 >> routeIDLei >> length >> direction;
        IFNodeRoadID >> speedLimit >> laneNum >> width >> kindNumber >> kind;
        routeIDGen = nodeID2RoadID[make_pair(nodeID1, nodeID2)];
        roadInforMore[i].nodeID1 = nodeID1;
        roadInforMore[i].nodeID2 = nodeID2;
        roadInforMore[i].routeID = routeIDGen;
        roadInforMore[i].length = length;
        roadInforMore[i].direction = direction;
        roadInforMore[i].speedLimit = speedLimit;
        roadInforMore[i].laneNum = laneNum;
        roadInforMore[i].width = width;
        roadInforMore[i].kindNumber = kindNumber;
        roadInforMore[i].kind = kind;
        i++;
    }
    // 将补充属性回填到 roadInfor[roadID]，供后续新算法结构化阶段直接读取
    for (int i=0;i<roadInforMore.size();i++)
    {
        roadInfor[roadInforMore[i].routeID].length = roadInforMore[i].length;
        roadInfor[roadInforMore[i].routeID].direction = roadInforMore[i].direction;
        roadInfor[roadInforMore[i].routeID].speedLimit = roadInforMore[i].speedLimit;
        roadInfor[roadInforMore[i].routeID].laneNum = roadInforMore[i].laneNum;
        roadInfor[roadInforMore[i].routeID].width = roadInforMore[i].width;
        roadInfor[roadInforMore[i].routeID].kindNumber = roadInforMore[i].kindNumber;
        roadInfor[roadInforMore[i].routeID].kind = roadInforMore[i].kind;
    }
}

TurnDir Graph::parseTurnDir(char c)
{
    switch (c) {
        case 'l':
        case 'L':
            return TurnDir::Left;
        case 's':
        case 'S':
            return TurnDir::Straight;
        case 'r':
        case 'R':
            return TurnDir::Right;
        case 'U':
            return TurnDir::UTurn;
        default:
            return TurnDir::Unknown;
    }
}

void Graph::build_new_graph_structures(vector<vector<int>>& routeDataForSimulation)
{
    // 现行新算法预处理流水线：
    // 1) 由传统数据映射生成结构化 road/node/intersection
    // 2) 构建 movement/lane-group/signal/waiting-buffer
    // 3) 将 route 转为 roadID + movementID，供 cycle-aware 调度直接使用
    build_road_segments_from_legacy_roads();
    initialize_nodes_from_roads();
    classify_node_types_assume_all_junctions_signalized();
    build_intersections_from_signalized_nodes();
    attach_min_travel_time_to_roads();

    if (connections_to_direction.empty()) {
        readConnectionsToDirections(connections_to_direction_path);
    }
    build_movements_from_connections();
    build_default_lane_groups();
    attach_lane_groups_to_movements();
    build_signal_controllers_assume_default();
    build_waiting_buffers();
    route_nodeID_2_roadID(routeDataForSimulation);
    route_roadID_2_movementID();
    validate_cycle_aware_graph();
}

void Graph::build_road_segments_from_legacy_roads()
{
    roads.clear();
    roadIDToRoadIndex.clear();
    if (roadID2NodeID.empty()) {
        return;
    }

    int maxRoadID = -1;
    for (const auto &kv : roadID2NodeID) {
        maxRoadID = max(maxRoadID, kv.first);
    }
    roads.resize(maxRoadID + 1);

    for (const auto &kv : roadID2NodeID) {
        int roadID = kv.first;
        int fromNode = kv.second.first;
        int toNode = kv.second.second;

        RoadSegment segment;
        segment.roadID = roadID;
        segment.fromNode = fromNode;
        segment.toNode = toNode;
        if (roadID >= 0 && roadID < roadInfor.size()) {
            const Road &legacyRoad = roadInfor[roadID];
            segment.length = legacyRoad.length;
            segment.speedLimit = legacyRoad.speedLimit;
            segment.laneNum = max(1, legacyRoad.laneNum);
            segment.width = legacyRoad.width;
            segment.direction = legacyRoad.direction;
            segment.kindNumber = legacyRoad.kindNumber;
            segment.kind = legacyRoad.kind;
        }
        roads[roadID] = segment;
        roadIDToRoadIndex[roadID] = roadID;
    }
}

void Graph::initialize_nodes_from_roads()
{
    nodes.clear();
    nodeIDToNodeIndex.clear();
    incomingRoadsByNode.clear();
    outgoingRoadsByNode.clear();

    if (nodenum <= 0) {
        return;
    }
    nodes.resize(nodenum);
    for (int nodeID = 0; nodeID < nodenum; ++nodeID) {
        nodes[nodeID].nodeID = nodeID;
        nodes[nodeID].type = NodeType::NormalSplit;
        nodeIDToNodeIndex[nodeID] = nodeID;
    }

    for (const auto &road : roads) {
        if (road.roadID < 0 || road.fromNode < 0 || road.toNode < 0) {
            continue;
        }
        if (road.fromNode >= nodenum || road.toNode >= nodenum) {
            continue;
        }
        nodes[road.fromNode].outgoingRoads.push_back(road.roadID);
        nodes[road.toNode].incomingRoads.push_back(road.roadID);
        outgoingRoadsByNode[road.fromNode].push_back(road.roadID);
        incomingRoadsByNode[road.toNode].push_back(road.roadID);
    }
}

void Graph::classify_node_types_assume_all_junctions_signalized()
{
    for (auto &node : nodes) {
        // Temporary assumption. Replace with real signalized node input later.
        if (node.incomingRoads.size() >= 2 && node.outgoingRoads.size() >= 2) {
            node.type = NodeType::SignalizedJunction;
        } else if (!node.incomingRoads.empty() || !node.outgoingRoads.empty()) {
            node.type = NodeType::NormalSplit;
        } else {
            node.type = NodeType::NormalSplit;
        }
    }
}

void Graph::build_intersections_from_signalized_nodes()
{
    intersections.clear();
    nodeToIntersectionID.clear();

    int intersectionID = 0;
    for (auto &road : roads) {
        road.hasSignalizedDownstream = false;
        road.downstreamIntersectionID = -1;
    }

    for (auto &node : nodes) {
        if (node.type != NodeType::SignalizedJunction) {
            node.intersectionID = -1;
            continue;
        }
        node.intersectionID = intersectionID;
        Intersection inter;
        inter.intersectionID = intersectionID;
        inter.nodeID = node.nodeID;
        inter.incomingRoads = node.incomingRoads;
        inter.outgoingRoads = node.outgoingRoads;
        intersections.push_back(inter);
        nodeToIntersectionID[node.nodeID] = intersectionID;

        for (int incomingRoadID : node.incomingRoads) {
            if (incomingRoadID >= 0 && incomingRoadID < roads.size()) {
                roads[incomingRoadID].hasSignalizedDownstream = true;
                roads[incomingRoadID].downstreamIntersectionID = intersectionID;
            }
        }
        ++intersectionID;
    }
}

void Graph::attach_min_travel_time_to_roads()
{
    for (auto &road : roads) {
        if (road.roadID < 0) continue;
        auto it = nodeID2minTime.find(make_pair(road.fromNode, road.toNode));
        if (it != nodeID2minTime.end()) {
            road.minTravelTime = it->second;
        }
    }
}

void Graph::build_movements_from_connections()
{
    movements.clear();
    roadPairToMovementID.clear();
    outgoingMovementsByRoad.clear();
    incomingMovementsByRoad.clear();
    movementIDsByIntersection.clear();

    int movementID = 0;
    for (const auto &kv : connections_to_direction) {
        int fromRoad = kv.first.first;
        int toRoad = kv.first.second;
        if (fromRoad < 0 || toRoad < 0 || fromRoad >= roads.size() || toRoad >= roads.size()) {
            continue;
        }
        if (roads[fromRoad].roadID < 0 || roads[toRoad].roadID < 0) {
            continue;
        }

        Movement movement;
        movement.movementID = movementID;
        movement.fromRoadID = fromRoad;
        movement.toRoadID = toRoad;
        movement.turn = parseTurnDir(kv.second);
        movement.alwaysOpen = (movement.turn == TurnDir::Right);

        int downstreamNode = roads[fromRoad].toNode;
        if (downstreamNode >= 0 && downstreamNode < nodes.size() &&
            nodes[downstreamNode].type == NodeType::SignalizedJunction) {
            movement.intersectionID = nodes[downstreamNode].intersectionID;
        }

        movements.push_back(movement);
        roadPairToMovementID[make_pair(fromRoad, toRoad)] = movementID;
        outgoingMovementsByRoad[fromRoad].push_back(movementID);
        incomingMovementsByRoad[toRoad].push_back(movementID);

        if (movement.intersectionID >= 0 && movement.intersectionID < intersections.size()) {
            intersections[movement.intersectionID].movementIDs.push_back(movementID);
            movementIDsByIntersection[movement.intersectionID].push_back(movementID);
        }
        ++movementID;
    }
}

void Graph::build_default_lane_groups()
{
    laneGroups.clear();
    laneGroupsByRoad.clear();
    roadTurnToLaneGroupID.clear();

    auto build_lane_indices = [](int laneNum, TurnDir turn) -> vector<int> {
        int normalizedLaneNum = max(1, laneNum);
        if (normalizedLaneNum <= 1) {
            return {0};
        }
        if (normalizedLaneNum == 2) {
            if (turn == TurnDir::Left) return {0};
            return {1};
        }
        if (turn == TurnDir::Left) return {0};
        if (turn == TurnDir::Right) return {normalizedLaneNum - 1};
        vector<int> middle;
        for (int i = 1; i < normalizedLaneNum - 1; ++i) {
            middle.push_back(i);
        }
        if (middle.empty()) middle.push_back(1);
        return middle;
    };

    int laneGroupID = 0;
    for (const auto &road : roads) {
        if (road.roadID < 0) continue;
        const vector<TurnDir> turns = {TurnDir::Left, TurnDir::Straight, TurnDir::Right};
        for (TurnDir turn : turns) {
            LaneGroup group;
            group.laneGroupID = laneGroupID;
            group.roadID = road.roadID;
            group.turn = turn;
            group.laneIndices = build_lane_indices(road.laneNum, turn);
            laneGroups.push_back(group);

            laneGroupsByRoad[road.roadID].push_back(laneGroupID);
            roadTurnToLaneGroupID[make_pair(road.roadID, turn)] = laneGroupID;
            ++laneGroupID;
        }
    }
}

void Graph::attach_lane_groups_to_movements()
{
    for (auto &movement : movements) {
        auto it = roadTurnToLaneGroupID.find(make_pair(movement.fromRoadID, movement.turn));
        if (it != roadTurnToLaneGroupID.end()) {
            movement.laneGroupID = it->second;
            continue;
        }
        auto fallback = roadTurnToLaneGroupID.find(make_pair(movement.fromRoadID, TurnDir::Straight));
        movement.laneGroupID = (fallback != roadTurnToLaneGroupID.end()) ? fallback->second : -1;
    }
}

void Graph::build_signal_controllers_assume_default()
{
    signals.clear();
    signalIDsByIntersection.clear();
    int signalID = 0;
    constexpr int kDefaultCycleLength = 60;
    constexpr int kDefaultGreenStart = 0;
    constexpr int kDefaultGreenEnd = 30;

    for (auto &movement : movements) {
        if (movement.intersectionID < 0 || movement.intersectionID >= intersections.size()) {
            continue;
        }
        SignalController signal;
        signal.signalID = signalID;
        signal.intersectionID = movement.intersectionID;
        signal.movementID = movement.movementID;
        if (movement.turn == TurnDir::Right) {
            signal.alwaysOpen = true;
            signal.currentState = SignalState::AlwaysOpen;
        } else {
            signal.alwaysOpen = false;
            signal.cycleLength = kDefaultCycleLength;
            signal.greenStart = kDefaultGreenStart;
            signal.greenEnd = kDefaultGreenEnd;
            signal.offset = 0;
            signal.currentState = SignalState::Red;
        }
        signals.push_back(signal);
        movement.signalID = signalID;
        intersections[signal.intersectionID].signalIDs.push_back(signalID);
        signalIDsByIntersection[signal.intersectionID].push_back(signalID);
        ++signalID;
    }
}

void Graph::build_waiting_buffers()
{
    waitingBuffers.clear();
    int bufferID = 0;
    for (auto &road : roads) {
        road.movementIDToWaitingBufferID.clear();
    }

    for (const auto &movement : movements) {
        if (movement.intersectionID < 0) {
            continue;
        }
        WaitingBuffer buffer;
        buffer.bufferID = bufferID;
        buffer.roadID = movement.fromRoadID;
        buffer.movementID = movement.movementID;
        buffer.laneGroupID = movement.laneGroupID;

        if (movement.laneGroupID >= 0 && movement.laneGroupID < laneGroups.size()) {
            buffer.laneCount = max(1, laneGroups[movement.laneGroupID].laneCount());
        } else {
            buffer.laneCount = 1;
        }

        waitingBuffers.push_back(buffer);
        if (movement.fromRoadID >= 0 && movement.fromRoadID < roads.size()) {
            roads[movement.fromRoadID].movementIDToWaitingBufferID[movement.movementID] = bufferID;
        }
        ++bufferID;
    }
}

void Graph::route_roadID_2_movementID()
{
    routeMovementID.clear();
    routeMovementID.resize(routeRoadID.size());

    int missingMovements = 0;
    for (int i = 0; i < routeRoadID.size(); ++i) {
        if (routeRoadID[i].size() <= 1) {
            continue;
        }
        for (int j = 0; j + 1 < routeRoadID[i].size(); ++j) {
            int fromRoad = routeRoadID[i][j];
            int toRoad = routeRoadID[i][j + 1];
            auto it = roadPairToMovementID.find(make_pair(fromRoad, toRoad));
            if (it != roadPairToMovementID.end()) {
                routeMovementID[i].push_back(it->second);
            } else {
                ++missingMovements;
            }
        }
    }
    if (missingMovements > 0) {
        cout << "[Validation] missing movements from route conversion: " << missingMovements << endl;
    }
}

void Graph::validate_cycle_aware_graph()
{
    int invalidRoadEndpoints = 0;
    int missingIntersectionID = 0;
    int missingSignalizedRoadFlag = 0;
    int invalidMovements = 0;
    int missingSignal = 0;
    int missingBuffer = 0;
    int missingLaneGroup = 0;
    int missingRouteMovement = 0;

    for (const auto &road : roads) {
        if (road.roadID < 0) continue;
        if (road.fromNode < 0 || road.toNode < 0 || road.fromNode >= nodenum || road.toNode >= nodenum) {
            ++invalidRoadEndpoints;
        }
    }
    for (const auto &node : nodes) {
        if (node.type == NodeType::SignalizedJunction && node.intersectionID < 0) {
            ++missingIntersectionID;
        }
        if (node.type == NodeType::SignalizedJunction) {
            for (int incomingRoadID : node.incomingRoads) {
                if (incomingRoadID < 0 || incomingRoadID >= roads.size() ||
                    !roads[incomingRoadID].hasSignalizedDownstream) {
                    ++missingSignalizedRoadFlag;
                }
            }
        }
    }
    for (const auto &movement : movements) {
        if (movement.fromRoadID < 0 || movement.toRoadID < 0 ||
            movement.fromRoadID >= roads.size() || movement.toRoadID >= roads.size()) {
            ++invalidMovements;
        }
        if (movement.laneGroupID < 0) {
            ++missingLaneGroup;
        }
        if (movement.intersectionID >= 0) {
            if (movement.signalID < 0) {
                ++missingSignal;
            }
            if (movement.fromRoadID >= 0 && movement.fromRoadID < roads.size()) {
                auto bufferIt = roads[movement.fromRoadID].movementIDToWaitingBufferID.find(movement.movementID);
                if (bufferIt == roads[movement.fromRoadID].movementIDToWaitingBufferID.end()) {
                    ++missingBuffer;
                }
            }
        }
    }
    for (const auto &route : routeRoadID) {
        for (int j = 0; j + 1 < route.size(); ++j) {
            if (roadPairToMovementID.find(make_pair(route[j], route[j + 1])) == roadPairToMovementID.end()) {
                ++missingRouteMovement;
            }
        }
    }

    cout << "[Validation] invalid road endpoints: " << invalidRoadEndpoints << endl;
    cout << "[Validation] missing intersection ids: " << missingIntersectionID << endl;
    cout << "[Validation] missing signalized road flags: " << missingSignalizedRoadFlag << endl;
    cout << "[Validation] invalid movements: " << invalidMovements << endl;
    cout << "[Validation] missing lane groups: " << missingLaneGroup << endl;
    cout << "[Validation] missing signals: " << missingSignal << endl;
    cout << "[Validation] missing buffers: " << missingBuffer << endl;
    cout << "[Validation] missing movements in routes: " << missingRouteMovement << endl;
}

// Read Query with Defined Number
vector<vector<int>> Graph::read_query(string filename, int num)
{
    // Variable Initialization
    int input_num;
    vector<vector<int>> Q;
    int DepartureID;
    int DestinationID;
    int DepartureTime;
    // Read Query Data
    ifstream file_name(filename);
    if(!file_name)
        cout<<"Cannot open Query Data"<<endl;
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than query data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0)
        input_num = lines;
    else
        input_num = num;
    // Read Query Data
    for (int i=0;i<input_num;i++){
        file_name >> DepartureID >> DestinationID >> DepartureTime;
        Q.push_back({DepartureID,DestinationID,DepartureTime});
    }
    // Close File
    file_name.close();
    /*
    // Print Query Data
    for (int i=0;i<Q.size();i++)
    {
        cout << "query " << i << ": ";
        cout << Q[i][0] << " " << Q[i][1] << " " << Q[i][2] << endl;
    }
    */
    return Q;
}

// Read Route Data with Defined Number
vector<vector<int>> Graph::read_route(string filename, int num)
{
    // Variable Initialization
    vector<vector<int>> Pi;
    int input_num;
    int vertexNum, vertex;
    vector<int> pi;

    int delay_time, low_time, wait_time, driving_num;

    // Read Route Data
    ifstream file_name(filename);
    if(!file_name)
        cout<<"Cannot open Route Data" <<endl;
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than route data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0){
        input_num = lines;
    }else{
        input_num = num;
    }
    // Read Route Data
    for (int i=0;i<input_num;i++){
        // Read One Route
        pi.clear();
        file_name >> vertexNum;

        // Correctness Check
        if (vertexNum == 0)
            cout << "route num is 0 when read in." << endl;
        for (int j=0;j<vertexNum;j++){
            file_name >> vertex;

            file_name >> delay_time >> low_time >> wait_time >> driving_num;
            route_time_Dict[i][vertex] = {delay_time, low_time, wait_time, driving_num};

            pi.push_back(vertex);
        }
        // Add Route into Route Data
        Pi.push_back(pi);
    }
    // Close File
    file_name.close();

    /*
    // Print Route Data
    for (int i=0;i<Pi.size();i++)
    {
        cout << "route " << i << ": ";
        for (int j=0;j<Pi[i].size();j++)
        {
            cout << Pi[i][j] << " ";
        }
        cout << endl;
    }
    */
    return Pi;
}

// Read time data with defined number
vector<vector<int>> Graph::read_time(string filename, int num, vector<vector<int>> query)
{
    // Variable Initialization
    vector<vector<int>> T;
    int input_num;
    int timeNum, time;
    vector<int> t;
    // Read Route Data
    ifstream file_name(filename);
    if(!file_name)
        cout<<"Cannot open time data" <<endl;
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than time data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0){
        input_num = lines;
    }else{
        input_num = num;
    }
    // Read Route Data
    for (int i=0;i<input_num;i++){
        // Read One Route
        t.clear();
        file_name >> timeNum;
        t.push_back(query[i][2]);
        // Correctness Check
        if (timeNum == 0)
            cout << "route num is 0 when read in." << endl;
        for (int j=0;j<timeNum;j++){
            file_name >> time;
            t.push_back(time);
        }
        // Add Route into Route Data
        T.push_back(t);
    }
    // Close File
    file_name.close();
    /*
    // Print Route Data
    for (int i=0;i<T.size();i++)
    {
        cout << "route " << i << ": ";
        for (int j=0;j<T[i].size();j++)
        {
            cout << T[i][j] << " ";
        }
        cout << endl;
    }
    */
    return T;
}


// Read time data with defined number
tuple<vector<int>, int> Graph::read_time_no_wait(string filename, int num) {
    // Variable Initialization
    vector<int> T(num);
    int input_num;
    // Read Route Data
    ifstream file_name(filename);
    if(!file_name)
        cout<<"Cannot open time data" <<endl;
    // Count Number of Lines
    int lines = CountLines(filename);
    // if defined number is greater than time data size
    // or smaller than 0, count defined number into lines
    if (num > lines or num < 0) {
        input_num = lines;
    }else {
        input_num = num;
    }
    // Read Data
    int unique_route_id, route_avg_length, route_id, travel_time_no_wait;
    file_name >> unique_route_id >> route_avg_length;

    int check = 0;

    for (int i = 0; i < input_num; i++) {
        file_name >> route_id >> travel_time_no_wait;
        T[i] = travel_time_no_wait;
    }

    // Close File
    file_name.close();

    return make_tuple(T, route_avg_length);
}

// Remove data with duplicate values
void Graph::removeDuplicates()
{

    vector<int> indicesToRemove;

    for (int i=0;i<routeDataRaw.size(); ++i) {
        unordered_set<int> seenElements;
        bool isDuplicateFound = false;

        // 检查 queryDataRaw 中的每个 vector 是否有重复元素
        for (int elem : routeDataRaw[i]) {
            if (seenElements.find(elem) != seenElements.end()) {
                isDuplicateFound = true;
                break;
            }
            seenElements.insert(elem);
        }

        if (isDuplicateFound) {
            // 如果找到重复，记录要删除的索引
            indicesToRemove.push_back(i);
        }
    }

    // 逆序删除元素，以免改变未处理的元素的索引
    for (auto it = indicesToRemove.rbegin(); it != indicesToRemove.rend(); ++it) {
        // cout << "Removing duplicate at index: " << *it << endl;

        // 从所有三个 vector 中删除对应的元素
        queryDataRaw.erase(queryDataRaw.begin() + *it);
        routeDataRaw.erase(routeDataRaw.begin() + *it);
        // timeDataRaw.erase(timeDataRaw.begin() + *it);
    }
    // Check if route data, query data, and time data size are same
    check_size();
}

// Check if route data, query data, and time data size are same
void Graph::check_size(){
    // if (!(queryDataRaw.size() == routeDataRaw.size() && routeDataRaw.size() == timeDataRaw.size())) {
    if (!(queryDataRaw.size() == routeDataRaw.size()) and queryDataRaw.size() == timeDataRaw.size() ) {
        cout << "Error. Data sizes are different." << endl;
    }
    else{
        cout << "Size of route, query, and time data currently is : " << routeDataRaw.size() << endl;
    }
}

// Split Route and Query Data as Average Length
pair<vector<vector<int>>, vector<vector<int>>> Graph::data_length_modify(vector<vector<int>> &queryDataRaw, vector<vector<int>> &routeDataRaw, int avg_length)
{
    // Variable Initialization
    vector<vector<int>> routeData;
    routeData.resize(routeDataRaw.size());
    for (int i=0;i<routeDataRaw.size();i++){
        // Set Length as Raw Data Size
        // In Case of Raw Data Original Size Is Smaller Than Average Length
        routeData[i].resize(routeDataRaw[i].size());
    }
    vector<int> routeTemp;
    // Go Through Route Data and Select Parts
    for (int i=0;i<routeDataRaw.size();i++){
        routeTemp.clear();

        if (routeDataRaw[i].size() <= avg_length){
            routeData[i] = routeDataRaw[i];
        }
        else{
            // Set Size as Average Length
            routeData[i].resize(avg_length);
            vector<int>::const_iterator first1 = routeDataRaw[i].begin();
            vector<int>::const_iterator last1  = routeDataRaw[i].begin() + avg_length;
            vector<int> cut1_vector(first1, last1);
            routeData[i] = cut1_vector;
        }
    }
    // Variable Initialization
    vector<vector<int>> queryData = queryDataRaw;
    // Go Through Query Data and Modify Its Destination One
    for (int i=0;i<queryData.size();i++){
        queryData[i][1] = routeData[i][routeData[i].size()-1];
    }
    // Correctness Check
    for (int i=0;i<routeData.size();i++){
        if (routeData[i][0] != queryData[i][0])
            cout << "Error! First nodes are not match." << endl;
        if (routeData[i][routeData[i].size()-1] != queryData[i][1])
            cout << "Error! Last nodes are not match." << endl;
    }
    /*
    // Print
    for (int i=0;i<routeData.size();i++)
    {
        cout << "query data are: " << queryData[i][0] << " " << queryData[i][1] << " " << queryData[i][2] << endl;
        cout << "routeData[i] size: " << routeDataRaw[i].size() << endl;
        cout << "route data are: ";
        for (int j=0;j<routeData[i].size();j++)
        {
            cout << routeData[i][j] << " ";
        }
        cout << endl;
    }
    */
    pair<vector<vector<int>>, vector<vector<int>>> dataCombine = make_pair(queryData, routeData);
    cout << "Data Cut Done." << endl;
    return dataCombine;
}


// Convert Route from "Node ID Pair" to "Road ID"
void Graph::route_nodeID_2_roadID(vector<vector<int>> &routeData)
{
    // Variable Initialization
    routeRoadID.resize(routeData.size());
    int node01, node02, roadID;
    // Route Data Transfer
    for (int i=0;i<routeRoadID.size();i++){
        if (routeData[i].size() <= 1)
            continue;
        // Resize Route Data
        routeRoadID[i].resize(routeData[i].size()-1);
        for (int j=0;j<routeRoadID[i].size();j++){
            node01 = routeData[i][j];
            node02 = routeData[i][j+1];
            // If Node Pairs Contained, Convert
            if (nodeID2RoadID.find(make_pair(node01,node02)) != nodeID2RoadID.end()){
                roadID = nodeID2RoadID[make_pair(node01,node02)];
                routeRoadID[i][j] = roadID;
            }
            else{
                // If Node Pairs Are not Contained, Not Achieve Error,
                // But Do not Consider How to Solve Them Yet.
                // We Consider to Use "BJ" Data Instead of "BJ_NodeWeight" Now.
                cout << "Warning. Unfounded node pairs are: " << node01 << " " << node02 << endl;
            }
        }
    }
    /*
    // Print Transfer Route Data
    for (int i=0;i<routeRoadID.size();i++){
        cout << "Route ID: " << i << endl;
        for (int j=0;j<routeRoadID[i].size();j++){
            cout << "Road Segment ID: " << routeRoadID[i][j] << " ";
        }
        cout << endl;
    }
    */
}

// Convert Route from "Node ID Pair" to "Road ID"
void Graph::route_nodeID_2_roadID_single(vector<int> &routeData)
{
    // Variable Initialization
    vector<int> route_roadID_single;
    route_roadID_single.resize(routeData.size() - 1);
    int node01, node02, roadID;

    for (int i = 0; i < route_roadID_single.size(); ++i) {
        node01 = routeData[i];
        node02 = routeData[i + 1];
        if (nodeID2RoadID.find(make_pair(node01, node02)) != nodeID2RoadID.end()){
            roadID = nodeID2RoadID[make_pair(node01, node02)];
            route_roadID_single[i] = roadID;
        }
        else{
            cout << "Warning+ Unfounded node pairs are: " << node01 << " " << node02 << endl;
        }
    }
    for (int i = 0; i < route_roadID_single.size(); ++i) {
        cout << route_roadID_single[i] << " ";
    }
    cout << endl;
}



//Classify Traffic Flow's Range
//Define a function to find "x" values when "y" equal to integer
//"constant" and "power" are predefined parameters
//e.g. y = 10 (min travel time) + 0.00375 (constant) * x ^ 2 (power)
//"maxTime" is the max travel time we defined
void Graph::flow_range_classification(float constant, int power, int maxTime)
{
    //Variable Initialization
    float travelTime;
    int rangeIndex = 0;
    vector<int> rangeRaw; rangeRaw.resize(maxTime);

    //Value of "x"
    for (int x=0;travelTime<maxTime;x++){
        if (abs(travelTime - constant * pow(x, power)) > 1)
            break;

        travelTime = constant * pow(x, power);
        if (travelTime > (rangeIndex + 1)){
            rangeIndex++;
            rangeRaw[rangeIndex] = x;
        }
    }
    //Cut rangeRaw
    //By the limitation of max travel time
    //"rangeRaw" size is bigger than what we defined
    //modified "range" does not have any empty position
    vector<int>::const_iterator first = rangeRaw.begin();
    vector<int>::const_iterator last;
    //Start from Second Position
    //because the first value equal to 0 (0 flow with min travel time)
    for (int i=1;i<rangeRaw.size();i++){
        if (rangeRaw[i] == 0){
            last = rangeRaw.begin() + i;
            break;
        }
    }
    //Cut
    vector<int> rangeCut(first, last);
    //"rangeCut" to Global Variable "range"
    range = rangeCut;

    //Print range
    cout << "Travel Time || Flow to Change Travel Time" << endl;
    for (int i=0;i<range.size();i++){
        cout << i << ": " << range[i] << endl;
    }

}

// Classify Each Road with A Unique Latency Function
// E.g. roadID: <[0,20) -> v1>, <[20,40) -> v2>, ..., <[80,100) -> v5>
// in simple:
// E.g. roadID: <20,v1>, <40,v2>, ..., <100,v5>
void Graph::classify_latency_function()
{
    // Initialize Variables Define Size
    // vector<vector<pair<int,vector<pair<int,int>>>>>
    // NodeID1: <NodeID2,<Flow1, TravelTime1>,...,<Flow5, TravelTime5>>
    timeRange.clear();
    timeRange.resize(nodenum);
    for (int i=0;i<timeRange.size();i++){
        timeRange[i].resize(graphLength[i].size());
        // Define Five Travel Time Ranges for Each Road
        for (int j=0;j<timeRange[i].size();j++){
            timeRange[i][j].second.resize(6);
            // timeRange[i][j].second.resize(1);
        }
    }
    // Define Values
    for (int i=0;i<timeRange.size();i++){
        int ID1 = i;
        for (int j=0;j<timeRange[i].size();j++){
            // Initialization Before Each Loop
            int ID2 = graphLength[i][j].first;
            vector<pair<int,int>> roadRange;
            roadRange.clear();
            roadRange.resize(6);
            // roadRange.clear();
            // roadRange.resize(1);
            int flow = 0;
            int minTravelTime = nodeID2minTime[make_pair(ID1, ID2)];
            int roadID = nodeID2RoadID[make_pair(ID1,ID2)];
            // area = length * laneNum
            // cap = area / 2
            int length = roadInfor[roadID].length;
            int laneNum = roadInfor[roadID].laneNum;
            int cap = length * laneNum / 5;
            int oneRange = cap / 5;
            // For Some Short Road
            if (oneRange <= 0){
                oneRange = 1;
            }
            for (int k=0;k<roadRange.size();k++){
                // Under This Setting
                // All Roads Share Same Latency Function
                /*flow += 20;
                int travelTime = minTravelTime * (1 + sigma * pow(flow/varphi, beta));*/

                // Under This Setting
                // Each Road Has Unique Latency Function
                // Based on Its Road Length & Number of Lan
                // flow += oneRange;
                int travelTime = minTravelTime * (1 + sigma * pow(flow/oneRange, beta));
                roadRange[k] = make_pair(flow,travelTime);
                flow += oneRange;
            }
            // Define Values
            timeRange[i][j].first = ID2;
            timeRange[i][j].second = roadRange;
            flow += oneRange;
        }
    }

    /*
    // Print
    for (int i=0;i<timeRange.size();i++){
        int ID1 = i;
        cout << ID1 << ": ";
        for (int j=0;j<timeRange[i].size();j++){
            int ID2 = timeRange[i][j].first;
            cout << ID2 << " ";
            vector<pair<int,int>> roadRange; roadRange.clear();
            roadRange = timeRange[i][j].second;
            for (int k=0;k<roadRange.size();k++){
                int flow = roadRange[k].first;
                int travelTime = roadRange[k].second;
                cout << flow << " -> " << travelTime << " ";
            }
        }
        cout << endl;
    }
    */
}

// Generate Hour Index and Its Related Minutes Index
// Each Road Has 24 Hour Index and Each Hour Has 6 or 12 Index (10/5 minutes)
// E.g. 3950s -> 1 Hour Index -> 1 Minutes Index (5 Minutes as Time Range)
pair<int, int> Graph::time_to_base_index(int seconds, int minRange)
{
    // Define Time Belonged Hours and Minutes
    int minute = seconds / 60;   // minutes
    int hour = minute / 60;      // hour
    // Define Hour Index
    int hourIndex = hour;

    // Define Minutes Index
    int minRest = minute % 60;
    int minIndex = minRest / minRange;
    // Correctness Check
    if (hourIndex > 24 or hourIndex < 0){
        // cout << "Error! Hour index is greater than 24 or smaller than 0" << endl;
        // cout << "Hour Index is " << hourIndex << " with its Minutes Index " << minIndex << endl;
        hourIndex = 23;
        minIndex = 0;
    }
    /*
    // Print
    cout << "Hour Index is " << hourIndex << " with its Minutes Index " << minIndex << endl;
    */
    return make_pair(hourIndex, minIndex);
}

// Define Flow Base
void Graph::flow_base_ini(int minRange, int flowValue)
{
    // Variable Initialization
    // ID1, ID2, Hour Index, Minutes Index
    flowBaseList.clear();
    // Flow Base Size Initialization
    flowBaseList.resize(graphLength.size());
    for (int i=0;i<flowBaseList.size();i++){    // ID1
        flowBaseList[i].resize(graphLength[i].size());  // Number of Neighbours (ID2)
        for (int j=0;j<graphLength[i].size();j++){
            // Assign Values for ID2
            flowBaseList[i][j].first = graphLength[i][j].first;     // ID2
            // Define Size for Time Range Index -> 24 Indicates Hours
            flowBaseList[i][j].second.resize(24);
            // Define Size for Time Slices for Each Hour -> minRange
            for (int k=0;k<flowBaseList[i][j].second.size();k++){
                int minSize = 60 / minRange;
                flowBaseList[i][j].second[k].resize(minSize);
            }
        }
    }
    // Assign Values
    // Now Only Assign Them to A Specific Value
    // Can Further Read and Assign Predicted Flow Data
    for (int i=0;i<flowBaseList.size();i++){
        for (int j=0;j<graphLength[i].size();j++){
            for (int k=0;k<flowBaseList[i][j].second.size();k++){
                for (int l=0;l<flowBaseList[i][j].second[k].size();l++){
                    flowBaseList[i][j].second[k][l] = flowValue;
                }
            }
        }
    }
    // Print
    /* for (int i=0;i<flowBaseList.size();i++){
        cout << "ID1 " << i << ": " << endl;
        for (int j=0;j<graphLength[i].size();j++){
            cout << " with ID2 " << graphLength[i][j].first << ": " << endl;
            for (int k=0;k<flowBaseList[i][j].second.size();k++){
                cout << " with hour index " << k << ": ";
                for (int l=0;l<flowBaseList[i][j].second[k].size();l++){
                    cout << flowBaseList[i][j].second[k][l] << " ";
                }
                cout << endl;
            }
        }
        cout << endl;
    } */
}

// Cut route data
vector<vector<int>> Graph::cut_route_data(vector<vector<int>> &routeDataRaw, int avg_length)
{
    // Variable Initialization
    vector<vector<int>> routeData;
    vector<int> placeHolder;
    routeData.assign(routeDataRaw.size(), placeHolder);

    // Go Through Route Data and Select Parts
    for (int i=0;i<routeDataRaw.size();i++){
        // If route length is smaller than defined length, keep original one
        if (routeDataRaw[i].size() <= avg_length){
            routeData[i] = routeDataRaw[i];
        }
        else{
            vector<int>::const_iterator start = routeDataRaw[i].begin();
            vector<int>::const_iterator end = routeDataRaw[i].begin() + avg_length;
            vector<int> cutVector(start, end);
            routeData[i] = cutVector;
        }
    }
    /*
    // Print route data out
    for (int i=0;i<routeData.size();i++){
        cout << "route " << i << " size: " << routeData[i].size() << endl;
        cout << "route data are: ";
        for (int j=0;j<routeData[i].size();j++){
            cout << routeData[i][j] << " ";
        }
        cout << endl;
    }
    */
    return routeData;
}

// Cut query data
vector<vector<int>> Graph::cut_query_data(vector<vector<int>> &queryDataRaw, vector<vector<int>> &routeData, int avg_length)
{
    // Variable initialization
    vector<vector<int>> queryData = queryDataRaw;
    // Go through query data and modify its destination one
    for (int i=0;i<queryData.size();i++){
        queryData[i][1] = routeData[i][routeData[i].size()-1];
    }

    // Correctness Check
    for (int i=0;i<routeData.size();i++){
        if (routeData[i][0] != queryData[i][0])
            cout << "Error! First nodes are not match." << endl;
        if (routeData[i][routeData[i].size()-1] != queryData[i][1])
            cout << "Error! Last nodes are not match." << endl;
    }

    return queryData;
}

// Cut time data
vector<vector<int>> Graph::cut_time_data(vector<vector<int>> &timeDataRaw, int avg_length)
{
    // Variable Initialization
    vector<vector<int>> timeData;
    vector<int> placeHolder;
    timeData.assign(timeDataRaw.size(), placeHolder);

    // Go Through Route Data and Select Parts
    for (int i=0;i<timeDataRaw.size();i++){
        // If route length is smaller than defined length, keep original one
        if (timeDataRaw[i].size() <= avg_length){
            timeData[i] = timeDataRaw[i];
        }
        else{
            vector<int>::const_iterator start = timeDataRaw[i].begin();
            vector<int>::const_iterator end = timeDataRaw[i].begin() + avg_length;
            vector<int> cutVector(start, end);
            timeData[i] = cutVector;
        }
    }
    /*
    // Print route data out
    for (int i=0;i<timeData.size();i++){
        cout << "time data " << i << " size: " << timeData[i].size() << endl;
        cout << "time data are: ";
        for (int j=0;j<timeData[i].size();j++){
            cout << timeData[i][j] << " ";
        }
        cout << endl;
    }
    */
    return timeData;
}
