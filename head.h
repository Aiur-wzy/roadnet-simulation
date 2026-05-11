
#ifndef INC_1ST_WORK_HEAD_H
#define INC_1ST_WORK_HEAD_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <set>

#include<iostream>
#include<fstream>
#include<math.h>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <functional>
#include <utility>
#include <string>
#include <algorithm>
#include <map>
#include <deque>
#include <thread>
#include <future>
#include <memory>
#if defined(__has_include)
#  if __has_include(<boost/thread/thread.hpp>) && __has_include(<boost/thread.hpp>)
#    include <boost/thread/thread.hpp>
#    include <boost/thread.hpp>
#    define ROADNET_HAS_BOOST_THREAD 1
#  endif
#endif
#ifndef ROADNET_HAS_BOOST_THREAD
namespace boost {
    using std::ref;
    using thread = std::thread;
    class thread_group {
    public:
        ~thread_group() { join_all(); }
        void add_thread(std::thread *t) { threads.emplace_back(t); }
        void join_all() {
            for (auto &t : threads) {
                if (t && t->joinable()) t->join();
            }
            threads.clear();
        }
    private:
        std::vector<std::unique_ptr<std::thread>> threads;
    };
}
#endif
#include <semaphore.h>
#include <time.h>
#include <cstdlib>
#include <random>
#include <queue>
#include <tuple>
#include <cctype>
#include <cassert>
#include <stdexcept>
#include "config_defaults.h"

#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <set>
#include <sstream>
#define INF 999999999

using namespace std;

// Parallel Computation

class Semaphore
{
public:
    Semaphore (int count_ = 0)
            : count(count_) {}

    inline void Signal()
    {
        unique_lock<std::mutex> lock(mtx);
        count++;
        cv.notify_one();
    }

    inline void Wait()
    {
        unique_lock<std::mutex> lock(mtx);

        while(count == 0)
        {
            cv.wait(lock);

        }
        count--;
    }


    mutex mtx;
    condition_variable cv;
    int count;
};

// 定义一个用作词典键的结构体
struct RoadKey {
    int lane_num;
    float speed_limit;
    float edge_length;
    // char turn_direction;
    int driving_number;
    int delay_time;
    int lowSpee_time;
    int wait_time;
    int ratio;
    int length_square;

    // 定义比较操作符，以便在词典中使用这个结构体作为键
    bool operator==(const RoadKey &other) const {
        return std::tie(lane_num, speed_limit, edge_length, driving_number, delay_time, lowSpee_time, wait_time, ratio, length_square) ==
               std::tie(other.lane_num, other.speed_limit, other.edge_length, other.driving_number, other.delay_time, other.lowSpee_time,other.wait_time, other.ratio, other.length_square);
    }
};

#include <functional>

namespace std {
    template <>
    struct hash<RoadKey> {
        std::size_t operator()(const RoadKey& k) const {
            size_t res = 17;  // 初始化一个基值
            // res = res * 31 + hash<string>()(k.edge_str);       // 对 edge_str 进行哈希
            res = res * 31 + hash<int>()(k.lane_num);            // 对 lane_num 进行哈希
            res = res * 31 + hash<float>()(k.speed_limit);       // 对 speed_limit 进行哈希 (float)
            res = res * 31 + hash<float>()(k.edge_length);       // 对 edge_length 进行哈希 (float)
            res = res * 31 + hash<int>()(k.driving_number);      // 对 driving_number 进行哈希
            res = res * 31 + hash<int>()(k.delay_time);          // 对 delay_time 进行哈希
            res = res * 31 + hash<int>()(k.lowSpee_time);        // 对 lowSpee_time 进行哈希
            res = res * 31 + hash<int>()(k.wait_time);           // 对 wait_time 进行哈希
            res = res * 31 + hash<int>()(k.ratio);               // 对 ratio 进行哈希
            res = res * 31 + hash<int>()(k.length_square);       // 对 length_square 进行哈希
            return res;  // 返回组合后的哈希值
        }
    };
}




//Contain Road Information
typedef struct ROAD
{
    int roadID;
    int ID1, ID2;
    int length;
    int travelTime;
    int direction;
    int speedLimit;
    int laneNum;
    int width;
    int kindNumber;
    string kind;
}Road;

//Contain Road Information
typedef struct ROADMORE
{
    int nodeID1;
    int nodeID2;
    int routeID;
    int length;
    int direction;
    int speedLimit;
    int laneNum;
    int width;
    int kindNumber;
    string kind;
}roadMore;

// Structure to store edge's static infor
struct EdgeInfo {
    int lane_num;
    float speed;
    float length;
    string edge_str;
};

enum class NodeType {
    NormalSplit,
    UnsignalizedJunction,
    SignalizedJunction
};

enum class TurnDir {
    Left,
    Straight,
    Right,
    UTurn,
    Unknown
};

enum class SignalState {
    Red,
    Green,
    Yellow,
    AlwaysOpen
};

enum class VehicleState {
    NotDeparted,
    RunningOnRoad,
    WaitingAtIntersection,
    Finished
};

enum class TravelTimeMode {
    SPEED_NET,
    MIN_TIME,
    TABLE,
    MODEL
};

TravelTimeMode parseTravelTimeMode(const string& s);
string travelTimeModeToString(TravelTimeMode mode);


struct SumoLaneRaw {
    string id;
    int index = -1;
    double speed = 0.0;
    double length = 0.0;
    string shape;
};

struct SumoEdgeRaw {
    string id;
    string from;
    string to;
    string function;
    int priority = 0;
    vector<SumoLaneRaw> lanes;
    bool isInternal = false;
};

struct SumoJunctionRaw {
    string id;
    string type;
    double x = 0.0;
    double y = 0.0;
    vector<string> incLanes;
    vector<string> intLanes;
};

struct SumoConnectionRaw {
    string fromEdge;
    string toEdge;
    int fromLane = -1;
    int toLane = -1;
    string via;
    string tl;
    int linkIndex = -1;
    TurnDir dir = TurnDir::Unknown;
    char state = 'O';
};

struct SumoSignalPhase {
    int duration = 0;
    int startTime = 0;
    int endTime = 0;
    string state;
};

struct SumoSignalProgram {
    string tlID;
    string type;
    string programID;
    int offset = 0;
    int cycleLength = 0;
    vector<SumoSignalPhase> phases;
};

struct SumoTripInfoTruth {
    string vehicleID;
    double depart = 0.0;
    double arrival = 0.0;
    double duration = 0.0;
    double routeLength = 0.0;
    double waitingTime = 0.0;
    double timeLoss = 0.0;
    double departDelay = 0.0;
};

struct SignalPhase {
    int startTime = 0;
    int endTime = 0;
    string state;
};

struct SignalProgram {
    string tlID;
    int offset = 0;
    int cycleLength = 0;
    vector<SignalPhase> phases;
};

struct NodeInfo {
    int nodeID = -1;
    NodeType type = NodeType::NormalSplit;
    vector<int> incomingRoads;
    vector<int> outgoingRoads;
    int intersectionID = -1;
};

struct RoadSegment {
    int roadID = -1;
    int fromNode = -1;
    int toNode = -1;
    double length = 0.0;
    double speedLimit = 0.0;
    int laneNum = 1;
    int width = 0;
    int direction = 0;
    int kindNumber = 0;
    string kind;
    double minTravelTime = -1.0;
    bool hasSignalizedDownstream = false;
    int downstreamIntersectionID = -1;
    int storageCapacityVehicles = 0;
    int runningCount = 0;
    vector<int> runningVehicles;
    unordered_map<int, int> movementIDToWaitingBufferID;
};

struct LaneGroup {
    int laneGroupID = -1;
    int roadID = -1;
    TurnDir turn = TurnDir::Unknown;
    vector<int> laneIndices;
    vector<int> allowedToRoads;

    int laneCount() const {
        return static_cast<int>(laneIndices.size());
    }
};

struct Movement {
    int movementID = -1;
    int fromRoadID = -1;
    int toRoadID = -1;
    int intersectionID = -1;
    TurnDir turn = TurnDir::Unknown;
    int laneGroupID = -1;
    int signalID = -1;
    bool alwaysOpen = false;
    int priorityOrder = 0;
    string tlID;
    int linkIndex = -1;
    vector<string> fromLanes;
    vector<string> toLanes;
    int laneDischargeCapacity = 1;
    char defaultConnectionState = 'O';
};

struct WaitingBuffer {
    int bufferID = -1;
    int roadID = -1;
    int movementID = -1;
    int laneGroupID = -1;
    int laneCount = 1;
    double vehicleLength = 5.0;
    double gap = 1.0;
    deque<int> vehicleQueue;

    int vehicleCount() const {
        return static_cast<int>(vehicleQueue.size());
    }

    double occupiedLength() const {
        if (laneCount <= 0) return 0.0;
        return ceil(static_cast<double>(vehicleQueue.size()) / laneCount) * (vehicleLength + gap);
    }
};

struct SignalController {
    int signalID = -1;
    int intersectionID = -1;
    int movementID = -1;
    bool alwaysOpen = false;
    int cycleLength = 0;
    int greenStart = 0;
    int greenEnd = 0;
    int offset = 0;
    SignalState currentState = SignalState::Red;
};

struct Intersection {
    int intersectionID = -1;
    int nodeID = -1;
    vector<int> incomingRoads;
    vector<int> outgoingRoads;
    vector<int> movementIDs;
    vector<int> signalIDs;
    int dischargeInterval = 1;
    unordered_map<int, int> roundRobinPointerByToRoad;
};

struct VehicleLabel {
    int vehicleID = -1;
    int routeID = -1;

    vector<int> routeRoadIDs;
    vector<int> routeMovementIDs;

    int roadIndex = 0;
    int currentRoadID = -1;
    int arrivalTime = 0;

    int currentMovementID = -1;
    int currentBufferID = -1;

    VehicleState state = VehicleState::NotDeparted;

    bool finished = false;

    // invalid=true means route conversion failed or required movement/buffer is missing.
    // invalid vehicles should be excluded from normal cycle-aware evaluation.
    bool valid = true;
};

struct SignalEvent {
    int time = 0;
    int signalID = -1;
};

struct DispatchCandidate {
    int earliestDischargeTime = 0;
    int readyTime = 0;
    int movementID = -1;
    int bufferID = -1;
    int vehicleID = -1;
};

struct DischargeResult {
    int vehicleID = -1;
    int movementID = -1;
    int intersectionID = -1;
    int fromRoadID = -1;
    int toRoadID = -1;
    int dischargeTime = 0;
    int newArrivalTime = 0;
    bool finished = false;
    int nextMovementID = -1;
    int nextBufferID = -1;
};

struct SignalEventCompare {
    bool operator()(const SignalEvent& a, const SignalEvent& b) const {
        if (a.time != b.time) return a.time > b.time;
        return a.signalID > b.signalID;
    }
};

struct DispatchCandidateCompare {
    bool operator()(const DispatchCandidate& a, const DispatchCandidate& b) const {
        if (a.earliestDischargeTime != b.earliestDischargeTime) {
            return a.earliestDischargeTime > b.earliestDischargeTime;
        }
        if (a.readyTime != b.readyTime) return a.readyTime > b.readyTime;
        if (a.movementID != b.movementID) return a.movementID > b.movementID;
        return a.vehicleID > b.vehicleID;
    }
};

class Graph {
public:

    // Step 1: Data Cleaning
    // -----------------------------------------------------------------------------

    // Check Why "BJ" had different edge number with "Minimal Travel Time"
    void edge_num_check();
    Semaphore* smGlobal = new Semaphore(1);

    // Step 2: Data Preparation
    // -----------------------------------------------------------------------------

    // Define Data Path
    // -----------------------------------------------------------------------------
    // Default path section:
    // - Prefer command-line arguments for reproducible experiments.
    // - If you do not want to pass command-line arguments, edit
    //   roadnet_defaults::DEFAULT_BASE_DIR in config_defaults.h.
    // - Graph::set_base_path() derives all standard input paths from Base.
    // - Explicit command-line path arguments override these defaults.
    string Base = roadnet_defaults::DEFAULT_BASE_DIR;
    static string normalize_base_dir(const string& baseDir) {
        if (baseDir.empty()) return baseDir;
        size_t end = baseDir.size();
        while (end > 1 && (baseDir[end - 1] == '/' || baseDir[end - 1] == '\\')) --end;
        return baseDir.substr(0, end);
    }
    static string join_path(const string& baseDir, const string& child) {
        if (baseDir.empty()) return child;
        string b = normalize_base_dir(baseDir);
        return b + "/" + child;
    }
    void set_base_path(const string& baseDir) {
        string b = normalize_base_dir(baseDir);
        Base = b;
        BJ = join_path(b, "Manhattan_network_BJ.txt");
        BJ_minTravleTime = join_path(b, "Manhattan_network_min_Travel_Time.txt");
        beijingMoreRoadInfo = join_path(b, "beijingMoreRoadInfo");
        queryPath = join_path(b, "query.txt");
        route_path = join_path(b, "route.txt");
        time_path = join_path(b, "time.txt");
        time_path_no_wait = join_path(b, "time_no_wait.txt");
        travelTimeTablePath = join_path(b, "model_catching_with_travel_time_1.txt");
        model_catch_dic_path = travelTimeTablePath;
        sumoNetPath = join_path(b, "test.net.xml");
    }
    // Read Road Network
    void read_graph();       // Can Be "one-way" (edge:651749) or "two-way" (edge:774660)
    // string BJ_NodeWeight = join_path(Base, "BJ_NodeWeight");
    string BJ = join_path(Base, "Manhattan_network_BJ.txt");                // ID1, ID2, Weight (Length in Meters)
    int nodenum;                                    // Node Number (296710)
    int edgenum;                                    // Edge Number (774660 or 651748) 387587
    vector<vector<pair<int, float>>> graphLength;      // ID1, ID2, Length
    vector<vector<pair<int, int>>> graphRoadID;     // ID1, ID2, RoadID
    vector<Road> roadInfor;                         // Road Info (ID1, ID2, RoadID, Length, Time
    vector<set<int>> adjNodes;                      // ID1, ID2
    map<pair<int, int>, int> nodeID2RoadID;         // <ID1, ID2> -> RoadID
    map<int, pair<int, int>> roadID2NodeID;         // RoadID -> <ID1, ID2>
    map<pair<int, int>, int> nodeID2minTime;        // <ID1, ID2> -> Min Travel Time
    string BJ_minTravleTime = join_path(Base,                // ID1, ID2, Weight (Min Travel Time in s)
                              "Manhattan_network_min_Travel_Time.txt");
    vector<vector<pair<int,int>>> graphTime;        // ID1, ID2, Weight (Min Travel Time)
    // Read Road Information
    void read_road_info();
    vector<roadMore> roadInforMore;
    string beijingMoreRoadInfo = join_path(Base, "beijingMoreRoadInfo");

    // Read query, route, time data
    vector<vector<int>> read_query(string filename, int num);
    string queryPath = join_path(Base, "query.txt");
    vector<vector<int>> queryDataRaw;
    vector<vector<int>> read_route(string filename, int num);
    string route_path = join_path(Base, "route.txt");
    vector<vector<int>> routeDataRaw;
    unordered_map<int, unordered_map<int, vector<int>>> route_time_Dict;
    vector<vector<int>> read_time(string filename, int num, vector<vector<int>> query);
    string time_path = join_path(Base, "time.txt");
    vector<vector<int>> timeDataRaw;
    tuple<vector<int>, int> read_time_no_wait(string filename, int num);
    string time_path_no_wait = join_path(Base, "time_no_wait.txt");
    vector<int> time_no_wait;

    // SUMO .net.xml input and adaptation
    string sumoNetPath = roadnet_defaults::DEFAULT_SUMO_NET_PATH;
    string sumoRoutePath;
    string sumoTripinfoPath;
    string evalOutputPath;
    void read_sumo_net_xml(const string& netXmlPath);
    void read_sumo_route_xml(const string& routeXmlPath, int maxVehicles);
    void read_sumo_tripinfo_xml(const string& tripinfoPath);
    float evaluate_sumo_tripinfo_truth(const vector<vector<pair<int, float>>>& ETA);
    void classify_node_types_from_sumo_junctions();
    void build_movements_from_sumo_connections();
    void build_lane_groups_from_sumo_connections();
    void build_signal_programs_from_sumo_tllogic();
    void build_new_graph_structures_from_sumo();
    SignalState signalStateAtMovement(int movementID, int t);
    void validate_sumo_network();
    void validate_sumo_connections();
    void validate_sumo_signal_programs();
    void validate_sumo_routes();

    unordered_map<string, int> sumoNodeStrToID;
    unordered_map<int, string> nodeIDToSumoNodeStr;
    unordered_map<string, int> sumoEdgeStrToRoadID;
    unordered_map<int, string> roadIDToSumoEdgeStr;
    unordered_map<string, int> tlIDToSignalProgramID;
    vector<SumoEdgeRaw> sumoEdgesRaw;
    vector<SumoJunctionRaw> sumoJunctionsRaw;
    vector<SumoConnectionRaw> sumoConnectionsRaw;
    vector<SumoSignalProgram> sumoSignalPrograms;
    vector<SumoTripInfoTruth> sumoTruthAligned;
    unordered_map<string, SumoTripInfoTruth> sumoTruthByVehicleID;
    vector<string> sumoVehicleIDs;
    vector<SignalProgram> signalPrograms;
    unordered_map<string, int> movementKeyToID;
    // Remove data with duplicate values
    void removeDuplicates();
    // Check if route data, query data, and time data size are same
    void check_size();
    // Cut route data
    vector<vector<int>> cut_route_data(vector<vector<int>> &routeDataRaw, int avg_length);
    // Cut query data
    vector<vector<int>> cut_query_data(vector<vector<int>> &queryDataRaw, vector<vector<int>> &routeData, int avg_length);
    // Cut time data
    vector<vector<int>> cut_time_data(vector<vector<int>> &timeDataRaw, int avg_length);

    // Split Route and Query Data as Average Length
    pair<vector<vector<int>>, vector<vector<int>>> data_length_modify(vector<vector<int>> &queryDataRaw,
                                                                      vector<vector<int>> &routeDataRaw, int avg_length);
    // Convert Route from "Node ID Pair" to "Road ID"
    void route_nodeID_2_roadID(vector<vector<int>> &routeData);
    // Convert Single Route from "Node ID Pair" to "Road ID"
    void route_nodeID_2_roadID_single(vector<int> &routeData);
    vector<vector<int>> routeRoadID;
    vector<vector<int>> routeMovementID;

    // Cycle-aware graph preparation（现行新算法依赖的核心结构构建）
    void build_new_graph_structures(vector<vector<int>>& routeDataForSimulation);
    void build_road_segments_from_legacy_roads();
    void initialize_nodes_from_roads();
    void classify_node_types_assume_all_junctions_signalized();
    void build_intersections_from_signalized_nodes();
    void attach_min_travel_time_to_roads();
    void build_movements_from_connections();
    void build_default_lane_groups();
    void attach_lane_groups_to_movements();
    void build_signal_controllers_assume_default();
    void build_waiting_buffers();
    void route_roadID_2_movementID();
    void initializeMovementLaneDischargeCapacity();
    void validate_cycle_aware_graph();
    TurnDir parseTurnDir(char c);

    vector<NodeInfo> nodes;
    vector<RoadSegment> roads;
    vector<LaneGroup> laneGroups;
    vector<Movement> movements;
    vector<WaitingBuffer> waitingBuffers;
    vector<SignalController> signals;
    vector<Intersection> intersections;

    unordered_map<int, int> roadIDToRoadIndex;
    unordered_map<int, int> nodeIDToNodeIndex;
    map<int, vector<int>> incomingRoadsByNode;
    map<int, vector<int>> outgoingRoadsByNode;
    map<pair<int, int>, int> roadPairToMovementID;
    map<pair<int, int>, vector<int>> fromToRoadToMovementIDs;
    map<int, vector<int>> outgoingMovementsByRoad;
    map<int, vector<int>> incomingMovementsByRoad;
    map<int, vector<int>> laneGroupsByRoad;
    map<pair<int, TurnDir>, int> roadTurnToLaneGroupID;
    map<int, int> nodeToIntersectionID;
    map<int, vector<int>> movementIDsByIntersection;
    map<int, vector<int>> signalIDsByIntersection;

    vector<VehicleLabel> vehicles;
    priority_queue<SignalEvent, vector<SignalEvent>, SignalEventCompare> signalEventPQ;
    priority_queue<DispatchCandidate, vector<DispatchCandidate>, DispatchCandidateCompare> dispatchPQ;
    vector<vector<pair<int, float>>> ETA_result_cycle_aware;
    int finishedVehicleCount = 0;
    int invalidVehicleCount = 0;
    int defaultDischargeInterval = 1;
    map<tuple<int, int>, int> usedMovementDischargeCapacity;
    // Classify Each Road with A Unique Latency Function
    vector<vector<pair<int,vector<pair<int,int>>>>> timeRange;
    // int realPercent;
    void classify_latency_function();
    // Generate Hour Index and Its Related Minutes Index
    pair<int, int> time_to_base_index(int seconds, int minRange);
    // Define Flow Base
    void flow_base_ini(int minRange, int flowValue);
    vector<vector<pair<int, vector<vector<int>>>>> flowBaseList;
    int minRange;   // minRange -> Min Time Range
    int flowIni;    // flowIni -> Initialized Flow Base Value

    // Step 3: ALGORITHM I SIMULATION
    // -----------------------------------------------------------------------------

    vector<int> Dij_vetex(int ID1, int ID2);
    // flow -> travel time range -> travel time
    int flow2time_by_range(int &ID1index, int &ID2index, int &flow);
    //
    void read_edge_feature_2_map(const string& filePath);
    string edge_id_to_features_path = join_path(Base, "edge_id_to_features.csv");

    map<int, EdgeInfo> edge_id_to_features;

    //
    void readConnectionsToDirections(const std::string& filename);
    map<pair<int, int>, char> connections_to_direction;
    string connections_to_direction_path = join_path(Base, "connections_to_directions.csv");

    // 定义函数，返回给定边的下一边的方向
    char findNextEdgeDirection(int currentEdgeID, int route_id);
    //
    set<char> processVehicleDirections(const std::vector<int>& vehicleIDs, int& currentEdgeID);
    //
    void buildDictionary(const std::string& filename);
    unordered_map<RoadKey, double> dictionary;
    string model_catch_dic_path = join_path(Base, "model_catching_with_travel_time_1.txt");

    TravelTimeMode travelTimeMode = TravelTimeMode::MIN_TIME;
    string travelTimeTablePath = join_path(Base, "model_catching_with_travel_time_1.txt");
    bool fallbackToSpeedNet = true;
    bool verboseTravelTimePrediction = false;
    string modelHost = "127.0.0.1";
    int modelPort = 9000;
    bool modelWarningPrinted = false;
    int travelTimeTableHit = 0;
    int travelTimeTableMiss = 0;
    // 现行新算法主入口：基于 signal event + waiting buffer + discharge capacity 的仿真
    vector<vector<pair<int, float>>> cycle_aware_signal_driven_records(
            vector<vector<int>> &Q, vector<vector<int>> &routeRoadID);
    void initialize_cycle_aware_vehicles(vector<vector<int>>& Q, vector<vector<int>>& routeRoadID);
    void initialize_signal_event_queue(int simStartTime);
    void process_discharge_window(int windowStart, int windowEnd);
    void rebuildActiveDispatchPQ(int currentTime, int windowEnd);
    bool isMovementActive(int movementID, int t);
    bool canDischarge(int movementID, int dischargeTime, int windowEnd);
    DischargeResult dischargeOneVehicle(int movementID, int dischargeTime);
    void pushCandidateIfPossible(int movementID, int currentTime, int windowEnd);
    bool isDispatchCandidateValid(const DispatchCandidate& c);
    int computeEarliestDischargeTime(int movementID, int readyTime, int currentTime);
    bool hasDischargeCapacity(int movementID, int t);
    void consumeDischargeCapacity(int movementID, int t);
    int nextAvailableCapacityTime(int movementID, int t);
    bool hasDownstreamStorage(int roadID);
    int predictRoadTravelTime(int roadID, int vehicleID);
    int predictRoadTravelTimeSpeedNet(int roadID) const;
    int predictRoadTravelTimeMinTime(int roadID) const;
    int predictRoadTravelTimeTable(int roadID, int vehicleID);
    int predictRoadTravelTimeModel(int roadID, int vehicleID);
    RoadKey buildRoadKeyForPrediction(int roadID, int vehicleID) const;
    bool queryExternalTravelTimeModel(int roadID, int vehicleID, double& predictedTime);
    void insertVehicleToBufferOrdered(int bufferID, int vehicleID);
    void handle_signal_change_event(const SignalEvent& e);
    int nextSignalChangeTime(int signalID, int afterTime);
    SignalState signalStateAt(int signalID, int t);
    bool allVehiclesFinished() const;
    void recordFinalETA(int vehicleID, int finalTime);
    vector<vector<pair<int, float>>> ETA_result;
    vector<vector<pair<int, map<float, vector<vector<int>>>>>> timeFlowChange;
    float sigma = 0.15; float varphi = 20; float beta = 2;
    float percent;
    int small, big;

    // Estimate average travel time of ground truth
    float AVG_estimation(vector<vector<int>> routeData, vector<vector<int>> timeData);
    // Estimate travel time MSE between simulated results and truth
    float MSE_estimation(vector<vector<int>> time, vector<vector<pair<int, float>>> ETA);
    float MSE_estimation_cycle_aware_total(
            vector<vector<int>> &timeData,
            vector<vector<pair<int, float>>> &ETA);
    // Find previous or next edge of current one.
    int find_direction(vector<vector<int>> &routeData, vector<vector<int>> &routeDataEdge,
                       int routeID, int nodeStart, int nodeEnd);
    // Export record information on each edge for further analysis
    void export_time_records(string route_num, vector<vector<int>> &routeData,
                             vector<vector<int>> &routeDataEdge, vector<vector<int>> &timeData,
                             vector<vector<pair<int,int>>> &ETA);
    string file_path_time_records = join_path(Base, "time_records_out.txt");

    void Traffic_Prediction(vector<vector<pair<int, float>>> ETA_result);

    // Step 4: ROUTE DATA UPDATE OPERATIONS PREPARATION
    // -----------------------------------------------------------------------------

    // Check Simulation
    void check_simulation(int simulation);
    // Change NodeID to RoadID
    void nodeID_2_roadID_in_records(vector<vector<pair<int,map<float, vector<vector<int>>>>>> &timeFlowChange);
    vector<map<float, vector<vector<int>>>> route_timeFlowChange;
    // Check Correctness of Time Records in Slices
    void time_record_correct_check();
    // Find Min Departure Time from Queries
    void min_depar_time(vector<vector<int>> &Q);
    int minDeparture; int minHour;
    // Convert int Time into UTC Hour
    int time_2_hour(int intTime);
    // Convert Hour to Its Belonged Index
    int hour_2_index(int hour);
    // Split Time Flow Change into Time Slices
    void split_2_time_slices(vector<map<float, vector<vector<int>>>> &route_timeFlowChange);
    vector<vector<map<int, vector<vector<int>>>>> timeSlice; // RoadSegmentID: TimeSlice: TimeRecord
    // Convert Node Constructed Route to Road Constructed Route
    vector<int> route_node_2_route_road(vector<int> &routeNode);
    // Generate New Data for Insertion
    void data_generation(string route_file, string depar_file, string Pi_file, int newNum, int avg_length, bool cut);
    string routeRoadPath = join_path(Base, "new_data_update/test_route");
    string deparTimePath = join_path(Base, "new_data_update/test_depar");
    string routeNodePath = join_path(Base, "new_data_update/test_Pi");
    // Generate Data Same as Simulation Operation for Comparison
    void data_generation_same_simulation(
            string route_file, string depar_file, string Pi_file, vector<vector<int>> &query_data, vector<vector<int>> &route_data);
    // Read Generated Route Data
    void read_new_data(string routeRoadPath, string deparTimePath, string routeNodePath);
    vector<vector<int>> Pi;
    vector<pair<int, vector<int>>> routeRoadInput;
    vector<int> departTimeList;
    int routeDataSize;
    // "ETA_result" Initialization
    void ETA_initialization(bool simulation, bool print);

    // Step 5: ROUTE DATA INSERTION OPERATIONS
    // -----------------------------------------------------------------------------

    // Insertion Operation (Main)
    void update_operation_insertion(bool paralle, bool terminal, bool range, bool print);
    vector<vector<map<int, vector<vector<int>>>>> timeSliceInsert;
    //  Insertion Operation by Inserting New Route Data One by One
    vector<int> one_route_update_insertion(pair<int, vector<int>> &newRoute, int inTime, bool parallel, bool terminal, bool range, bool print);
    //  Insertion Operation for Different Road Has Different Operation
    //  For a Road ID Constructed Route, Find Next Road ID
    int find_next_roadID(vector<int> route, int roadSegmentID);
    //  Check If Two Traffic Flows Are In Same Range
    bool flow_same_range_check(int &ID1index, int &ID2index, int &flow1, int &flow2);
    //  This Is Insertion Operation for The First Road of New Route
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> update_operation_1st(
            int &RoadSegmentID, int &inTime, pair<int, vector<int>> &newRoutePair, bool &parallel, bool &terminal, bool &range, bool print);
    //  This Is Insertion Operation for The Further Road of New Route or Propagated Road
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> updateOperationFurther(
            pair<int,int> &RoadSegmentID, int &inTime, pair<int, vector<int>> &newRoutePair, pair<int, vector<int>> &routeProp,
            map<int, vector<vector<int>>> &InsertPre, vector<int> &DeletionPre, bool &parallel, bool &terminal, bool &range, bool print);
    // Insertion Operation Parallel
    void update_operation_parallel(vector<pair<pair<int, vector<int>>, int>> &newDataMulti, bool &parallel, bool terminal,
                                   bool &range, bool print, int &threadNum);
    //  Split New Route Data into Groups
    void multi_new_data_initial(void);
    vector<pair<pair<int, vector<int>>, int>> newDataMulti;
    //  Insertion Operation by Inserting New Route Data One by One (Parallel Version)
    void one_route_parallel_update(vector<pair<pair<int, vector<int>>, int>> &multi_test,
                                   int i, int j, vector<Semaphore*>& vLock, bool parallel, bool terminal, bool range, bool print);
    vector<Semaphore*> vLock;
    vector<int> affectedRoadParallel;

    // Step 6: TRAVEL TIME ESTIMATION (ETA) UPDATE
    // -----------------------------------------------------------------------------
    void ETA_update();

    // Step 7: ROUTE DATA DELETION OPERATIONS
    // -----------------------------------------------------------------------------

    // Generate New Data for Deletion
    // Main Idea: Capturing Existing Route as New Deletion Data
    void dele_data_generation(string route_file, string Pi_file, string route_index_file, int newNum);
    string routeRoadPathD = join_path(Base, "new_data_update/deletion_routeNode");
    string routeNodePathD = join_path(Base, "new_data_update/deletion_routeRoad");
    string routeIndexpath = join_path(Base, "new_data_update/deletion_routeRoadIndex");
    // Read Selected Deleted Route Data
    void read_deletion_data(string routeRoadPath, string routeRoadIndex);
    vector<pair<int, vector<int>>> routeRoadDelInput;
    // Deletion Operation
    void update_operation_deletion(bool parallel, bool terminal, bool range, bool print);
    vector<vector<map<int, vector<vector<int>>>>> timeSliceDel;
    vector<int> one_route_update_deletion(pair<int, vector<int>> delRoute, bool parallel, bool terminal, bool range, bool print);
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> deletion_operation_1st(
            int &RoadSegmentID, int &inTime, pair<int, vector<int>> &newRoutePair, bool &parallel, bool &terminal, bool &range, bool print);
    //  This Is Deletion Operation for The Further Road of New Route or Propagated Road
    vector<pair<pair<int,int>, tuple<int, map<int, vector<vector<int>>>, vector<int>>>> update_operation_deletion_further(
            pair<int,int> &RoadSegmentID, int &inTime, pair<int, vector<int>> &newRoutePair, pair<int, vector<int>> &routeProp,
            map<int, vector<vector<int>>> &InsertPre, vector<int> &DeletionPre, bool &parallel, bool &terminal, bool &range, bool print);
    //  Split New Route Data into Groups
    void multi_del_data_initial(void);
    vector<pair<int, vector<int>>> delDataMulti;
    //  Deletion Operation by Deleting Target Route Data One by One (Parallel Version)
    void del_operation_parallel(vector<pair<int, vector<int>>> &delDataMulti,
                                bool &parallel, bool terminal, bool &range,  bool print, int &threadNum);
    //  Deletion Operation by Inserting New Route Data One by One (Parallel Version)
    void one_route_parallel_update_del(vector<pair<int, vector<int>>> &multi_test,
                                       int i, int j, vector<Semaphore*>& vLock, bool parallel, bool terminal, bool range, bool print);
    vector<int> affectedRoadParallelDel;
    vector<Semaphore*> vLockDel;
    void stat();

    // Appendix
    // -----------------------------------------------------------------------------
    // Count Number of Lines in ".txt" File
    int CountLines(string filename){
        ifstream ReadFile;
        int n=0;
        char line[512];
        string temp;
        ReadFile.open(filename,ios::in);
        if(ReadFile.fail())
            return 0;
        else
        {
            while(getline(ReadFile,temp))
            {
                n++;
            }
            ReadFile.close();
            return n;
        }
    }
    // Randomly Generate Unordered Integer
    vector<int> randperm(int Num)
    {
        vector<int> temp;
        for (int i = 0; i < Num; ++i)
        {
            temp.push_back(i);
        }
        std::random_device rd;
        std::mt19937 g(rd());
        shuffle(temp.begin(), temp.end(),g);
        return temp;
    }

    // Abundant Functions
    // -----------------------------------------------------------------------------

    //Classify Traffic Flow's Range
    void flow_range_classification(float constant, int power, int maxTime);
    vector<int> range;
};


namespace benchmark {

#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heap {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };


    public:

        // Constructor of the heap.
        heap(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {
        }

        heap() {

        }

        // Size of the heap.
        inline node_t size() const {
            return n;
        }

        // Heap empty?
        inline bool empty() const {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(node_t &element, key_t &key) {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty()) {
                front = elements[n];
                position[front.element] = 0;
                sift_down(0);
            }
        }

        inline key_t top() {
            assert(!empty());

            element_t &front = elements[0];

            return front.key;

        }

        inline node_t top_value() {

            assert(!empty());

            element_t &front = elements[0];

            return front.element;
        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key) {
            if (position[element] == NULLINDEX) {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                sift_up(n++);
            } else {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key > el.key) {
                    el.key = key;
                    sift_down(el_pos);
                } else {
                    el.key = key;
                    sift_up(el_pos);
                }
            }
        }


        // Clear the heap.
        inline void clear() {
            for (node_t i = 0; i < n; ++i) {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v) {
            position[v] = NULLINDEX;
        }

        inline void clear_n() {
            n = 0;
        }


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const {
            return position[element] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void sift_up(node_t i) {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0) {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key > elements[cur_i].key)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void sift_down(node_t i) {
            assert(i < n);

            while (true) {
                node_t min_ind = i;
                key_t min_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j) {
                    if (elements[j].key < min_key) {
                        min_ind = j;
                        min_key = elements[j].key;
                    }
                }

                // Exchange?
                if (min_ind != i) {
                    swap(i, min_ind);
                    i = min_ind;
                } else {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const node_t i, const node_t j) {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.element] = j;
            position[el_j.element] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heap.
        node_t n;

        // Number of maximal elements.
        node_t max_n;

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };
}

#endif //INC_1ST_WORK_HEAD_H
