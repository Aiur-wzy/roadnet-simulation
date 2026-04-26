#include "head.h"


// Read Graph
void Graph::read_graph()
{

    // Both "BJ" and "BJ_NodeWeight" Contains Length Information on Node Pairs
    // "BJ" Consider All Roads as Two-way & "BJ_NodeWeight" Contain some One-way
    // A Few Node Pairs Are not Contained in "BJ_NodeWeight"
    // Consider for The Suitable One

    // Choice 1: Open File "BJ"
    ifstream IF(BJ);    //ID1, ID2, Weight (Length in Meters)
    if(!IF)
        cout<<"Cannot open Graph BJ"<<endl;
    //Read Node Number and Edge Number
    IF>>nodenum>>edgenum; // 296710 774660

    /*
    // Choice 2: Open File "BJ_NodeWeight"
    ifstream IF(BJ_NodeWeight);    //ID1, ID2, Weight (Length in Meters)
    if(!IF)
        cout<<"Cannot open Graph BJ"<<endl;
    // Read Node Number and Edge Number
    IF>>nodenum; // 296710
    edgenum = CountLines(BJ_NodeWeight); // 651748
    */

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
    /*
    // Print Graph Length as Weight
    for (int i=0;i<graphLength.size();i++)
    {
        cout << i << ": ";
        for (int j=0;j<graphLength[i].size();j++)
        {
            cout << graphLength[i][j].first << " " << graphLength[i][j].second << " ";
        }
        cout << endl;
    }
    */

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
    /*
    // Print Graph Min Travel Time as Weight
    for (int i=0;i<graphTime.size();i++)
    {
        cout << i << ": ";
        for (int j=0;j<graphTime[i].size();j++)
        {
            cout << graphTime[i][j].first << " " << graphTime[i][j].second << " ";
        }
        cout << endl;
    }
    */
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
    // Count Number of Lines in This File
    int i = 0;
    roadInforMore.resize(387587);       // 387587 comes from
    // Read In Info
    while(IFNodeRoadID >> nodeID1){
        IFNodeRoadID >> nodeID2 >> routeIDLei >> length >> direction;
        IFNodeRoadID >> speedLimit >> laneNum >> width >> kindNumber >> kind;
        // Assign Read Info to Variables
        // However, "i" Do Not Contain Any Info
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
    // Add Road Info into "roadInfor"
    // Index "i" of "roadInforMore" Do Not Contain Any Info
    // Index "i" of "roadInfor" Inidicate Road ID
    // Have to Move Info in "roadInforMore" to "roadInfor" for Further Application
    // Otherwise, Road Info Cannot Be Index
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

    /*
    // Correctness Check
    // "ID1 == 0" Should Appear Twice
    // "ID2 == 0" Should Appear Twice
    // "roadID == 0" Should Appear Once
    for (int i=0;i<roadInfor.size();i++)
    {
        if (roadInfor[i].ID1 == 0)
            cout << "ID1 is equal to 0" << endl;
        if (roadInfor[i].ID2 == 0)
            cout << "ID2 is equal to 0" << endl;
        if (roadInfor[i].roadID == 0)
            cout << "roadID is equal to 0" << endl;
    }
    */

    /*
    // Print
    for (int i=0;i<roadInfor.size();i++)
    {
        if (roadInfor[i].ID1 == )
        cout << "node pair: " << roadInfor[i].ID1 << " " << roadInfor[i].ID2;
        cout << " with road ID: " << roadInfor[i].roadID << endl;
    }
    */
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

