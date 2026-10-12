
#ifndef INC_1ST_WORK_HEAD_H
#define INC_1ST_WORK_HEAD_H

#include <stdio.h>
#include <string.h>
#include <vector>
#include <map>
#include <set>

#include<iostream>
#include<fstream>
#include <cmath>
#include<math.h>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <ctime>
#include <functional>
#include <iomanip>
#include <utility>
#include <string>
#include <algorithm>
#include <map>
#include <deque>
#include <limits>
#include <cstdlib>
#include <time.h>
#include <cstdlib>
#include <random>
#include <queue>
#include <tuple>
#include <cctype>
#include <cassert>
#include <stdexcept>
#include <memory>
#include <cstdint>
#include "config_defaults.h"
#include "sf_travel_time_table.h"
#include "cap_context.h"
#include "cap_context_table.h"
#include "signal_rg.h"
#include "cap_context_abc_rg_table.h"
#include "native_road_time_table.h"

#include <iostream>
#include <sys/types.h>
#include <string.h>
#include <set>
#include <sstream>
constexpr double INF = 1.0e15;
constexpr double TIME_EPSILON = 1.0e-9;

using namespace std;

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
    double length;
    double travelTime;
    int direction;
    double speedLimit;
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
    double length;
    int direction;
    double speedLimit;
    int laneNum;
    int width;
    int kindNumber;
    string kind;
}roadMore;

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

enum class MovementQueueOrder {
    ArrivalTime,
    StrictFIFO
};

enum class TravelTimeMode {
    SPEED_NET,
    MIN_TIME,
    TABLE,
    MODEL,
    KINEMATIC
};

enum class TravelTimeTableFormat {
    LEGACY,
    SUMO_V1,
    CAMS_SF_V1,
    CAMS_CAP_CONTEXT_RLE_V1,
    CAMS_CAP_CONTEXT_ABC_RLE_V1,
    CAMS_CAP_CONTEXT_ABC_RG_RLE_V1,
    CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1,
    CAMS_NATIVE_ABC_RG_DEMAND_V2
};

TravelTimeMode parseTravelTimeMode(const string& s);
string travelTimeModeToString(TravelTimeMode mode);


struct SumoLaneRaw {
    string id;
    int index = -1;
    double speed = 0.0;
    double length = 0.0;
    string shape;
    string allow = "all";
    string disallow;
    string changeLeft = "all";
    string changeRight = "all";
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
    string allow = "all";
    string disallow;
};

struct SumoSignalPhase {
    double duration = 0.0;
    double startTime = 0.0;
    double endTime = 0.0;
    string state;
    // Retained for the versioned feature calendar; dispatch behavior is unchanged.
    string nextPhaseIndices;
};

struct SumoSignalProgram {
    string tlID;
    string type;
    string programID;
    double offset = 0.0;
    double cycleLength = 0.0;
    vector<SumoSignalPhase> phases;
};

struct OutgoingSignalRGContext {
    std::shared_ptr<SignalRGProgram> program;
    double offset = 0.0;
    string tls;
    int link = -1;
};

struct VehicleType {
    string id = "car";
    double accel = 2.6;
    double decel = 4.5;
    double sigma = 0.5;
    double length = 5.0;
    double minGap = 2.5;
    double maxSpeed = 13.89;
    // Used by the opt-in speed-net SF cap as well as diagnostic features.
    double desiredMaxSpeed = 10000.0 / 3.6;
    string vehicleClass = "passenger";
};


// Future model-extension interface:
// Lightweight feature vector for model-based single-road travel-time prediction.
// turn_type is the outgoing alias in road-turn-context-v1. Incoming turn and
// the legacy mixed prediction context are separate, explicit metadata.
// extra is reserved for deliberate future advanced features. Do not confuse this
// feature path with the legacy table-only RoadKey lookup key.
struct BasicRoadModelFeatures {
    double time = 0.0;
    int vehicleID = -1;
    int roadID = -1;
    int movementID = -1;
    int laneIndex = -1;

    double road_length = 0.0;
    // Boundary metadata, NOT additional inputs or altered keys for existing tables.
    bool is_route_start = false;
    double departure_position = 0.0;
    string departure_position_issue;
    // Canonical route-occurrence contract: 0 unknown/absent, l=1, s=2, r=3, t=4.
    // Present means an adjacent route road exists, even if its turn is unknown.
    int turn_type = 0; // Alias of outgoing_turn_type for constructed features.
    int route_index = -1;
    bool route_context_resolved = false;
    int incoming_movement_id = -1;
    int outgoing_movement_id = -1;
    bool incoming_turn_present = false;
    bool outgoing_turn_present = false;
    int incoming_turn_type = 0;
    int outgoing_turn_type = 0;
    // Old predictor argument: first-road outgoing, otherwise incoming. -1 is
    // reserved for hand-built features whose SUMO_V1 key falls back to turn_type.
    int legacy_context_turn_type = -1;

    int road_flow = 0;
    int lane_flow = 0;

    int lane_num = 1;
    double speed_limit = 0.0;
    double vehicle_length = 5.0;
    double vehicle_min_gap = 2.5;
    int lane_capacity = 0;
    double lane_occupied_length = 0.0;
    // Logical waiting-buffer occupancy in the selected physical lane, excluding
    // this vehicle. Includes buffer members whose arrivalTime is still future.
    double lane_waiting_buffer_length = 0.0;

    // Vehicle-level waiting state carried into the road being predicted.
    // 0/1 integer encoding is kept to match Python model feature columns.
    int has_waiting = 0;
    double waiting_duration = 0.0;

    // Route-resolved vehicle metadata, not additions to existing table keys.
    // Missing per-vehicle SF remains unknown (NaN), never a sampled-type mean.
    double speed_factor = std::numeric_limits<double>::quiet_NaN();
    bool has_explicit_speed_factor = false;
    double free_speed_cap = std::numeric_limits<double>::quiet_NaN();
    // Diagnostic/proxy only, never a new input to an existing learned table.
    double entry_speed_proxy = 0.0;
    // New-table inputs plus diagnostic-only causal snapshot metadata.
    int platoon_context = -1;
    double context_decision_time = std::numeric_limits<double>::quiet_NaN();
    double context_distance = std::numeric_limits<double>::quiet_NaN();
    CapContextResult context_details;
    SignalRGFeatures signal_rg = NoSignalRGFeatures();
    SignalRGFeatures signal_rg_table_node = NoSignalRGFeatures();
    bool signal_rg_available = false;
    string signal_rg_tls;
    int signal_rg_link = -1;

    unordered_map<string, double> extra;
};


struct SpeedNetMotionResult {
    double effective_distance = 0.0;
    double target_speed = 0.0;
    double entry_speed = 0.0;
    double exit_speed = 0.0;
    double motion_time = 0.0;
    double following_delay = 0.0;
    double predicted_time = 1.0;
};

struct ContinuousSpeedTransfer {
    double entry_speed = 0.0;
    double previous_exit_speed = std::numeric_limits<double>::quiet_NaN();
    double waiting_seconds = 0.0;
    double connection_cap = std::numeric_limits<double>::quiet_NaN();
    string reason = "unknown_previous_speed";
};

struct SumoV1TravelTimeKey {
    int has_waiting = 0;
    long long road_length_q = 0;
    int turn_type = 0;
    int road_flow = 0;
    long long lane_flow_q = 0;

    bool operator==(const SumoV1TravelTimeKey& other) const {
        return std::tie(has_waiting, road_length_q, turn_type, road_flow, lane_flow_q) ==
               std::tie(other.has_waiting, other.road_length_q, other.turn_type, other.road_flow, other.lane_flow_q);
    }
};

struct SumoV1TravelTimeKeyHash {
    std::size_t operator()(const SumoV1TravelTimeKey& k) const {
        size_t res = 17;
        res = res * 31 + std::hash<int>()(k.has_waiting);
        res = res * 31 + std::hash<long long>()(k.road_length_q);
        res = res * 31 + std::hash<int>()(k.turn_type);
        res = res * 31 + std::hash<int>()(k.road_flow);
        res = res * 31 + std::hash<long long>()(k.lane_flow_q);
        return res;
    }
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
    double startTime = 0.0;
    double endTime = 0.0;
    string state;
};

struct SignalProgram {
    string tlID;
    double offset = 0.0;
    double cycleLength = 0.0;
    vector<SignalPhase> phases;
};

struct NodeInfo {
    int nodeID = -1;
    NodeType type = NodeType::NormalSplit;
    vector<int> incomingRoads;
    vector<int> outgoingRoads;
    int intersectionID = -1;
    // Original physical SUMO type; empty for legacy/unknown nodes. Do not infer
    // absence of signals from the coarse NormalSplit fallback classification.
    string sumoJunctionType;
};

// Core graph data structure: one physical road/edge in the simulation.
// Static attributes describe geometry/capacity inputs (length, speedLimit, laneNum);
// Logical laneFlow/lengths retain the model's original lane assignment. When
// borrowing is enabled, separate storageFlow/lengths enforce physical capacity.
// movementIDToWaitingBufferID links each outgoing movement to its real FIFO queue.
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
    int roadFlow = 0;
    vector<int> laneFlow;
    vector<int> laneCapacity;
    vector<double> laneOccupiedLength;
    vector<double> laneStorageLength;
    // Incremental logical-buffer length; not a measured stopped-queue length.
    vector<double> laneWaitingBufferLength;
    // Opt-in borrowing: physical storage is separate from logical model lanes.
    vector<int> storageFlow;
    vector<double> storageOccupiedLength;
    int runningCount = 0;
    vector<int> runningVehicles;
    unordered_map<int, int> movementIDToWaitingBufferID;
};

// Core graph data structure: a group of lanes that serve the same turn/movement.
// This bridges movement-level waiting queues to lane-level road storage constraints.
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

// Core dispatch unit: a legal transition from fromRoadID to toRoadID through an intersection.
// Movement candidates, not vehicles, are ordered in the dispatch priority queue;
// signal eligibility is bound by tlID/linkIndex or alwaysOpen. laneDischargeCapacity
// is the number of independent headway channels. This is not a vehicle; FIFO
// vehicle order stays in WaitingBuffer.
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
    // Exact original SUMO lane pairs, never the Cartesian product of lane sets.
    // tlID/linkIndex above are a representative validated to share one calendar
    // when mergeRoadPairMovements is enabled; raw metadata stays untouched.
    vector<int> connectionIndices;
    // Per-connection complete internal-path bounds, aligned with connectionIndices.
    vector<bool> connectionContinuousSpeedPathKnown;
    vector<double> connectionContinuousSpeedPathLimit;
    int laneDischargeCapacity = 1;
    char defaultConnectionState = 'O';
    // Static conservative bound across complete internal-via paths. No runtime
    // path scans. Unknown/incomplete paths require a zero-speed reset.
    bool continuousSpeedPathKnown = false;
    double continuousSpeedPathLimit = std::numeric_limits<double>::quiet_NaN();
};

// Core queue state: the real FIFO queue for vehicles waiting to pass one movement.
// Depending on the current design the queue is inserted by predicted arrival label,
// but vehicles must only be removed here after a successful discharge. Failed
// attempts must leave this queue unchanged.
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
    double cycleLength = 0.0;
    double greenStart = 0.0;
    double greenEnd = 0.0;
    double offset = 0.0;
    SignalState currentState = SignalState::Red;
};

struct Intersection {
    int intersectionID = -1;
    int nodeID = -1;
    vector<int> incomingRoads;
    vector<int> outgoingRoads;
    vector<int> movementIDs;
    vector<int> signalIDs;
    double dischargeInterval = 1.0;
    unordered_map<int, int> roundRobinPointerByToRoad;
};

// Runtime vehicle state for cycle-aware simulation.
// routeRoadIDs/routeMovementIDs define the path; arrivalTime is the label time for
// reaching the current waiting buffer. occupiedRoadID/occupiedLaneIndex connect this
// vehicle to roadFlow/laneFlow accounting. Waiting fields carry vehicle-level model
// features after signal waiting.
struct VehicleLabel {
    int vehicleID = -1;
    int routeID = -1;

    vector<int> routeRoadIDs;
    vector<int> routeMovementIDs;

    int roadIndex = 0;
    int currentRoadID = -1;
    double arrivalTime = 0.0;
    double scheduledDepartTime = 0.0;
    int assignedEntryRoadID = -1;
    int assignedEntryLaneIndex = -1;
    bool queuedForEntry = false;
    bool queuedForFinalExit = false;

    // Waiting state from the most recent movement discharge. These values describe
    // whether the vehicle waited before entering its current road, so downstream
    // travel-time prediction can use them as vehicle-level model inputs.
    bool hasWaitingBeforeCurrentRoad = false;
    bool lastDischargeHadWaiting = false;
    double lastWaitingDuration = 0.0;
    double entrySpeedProxy = 0.0;
    double estimatedRoadExitSpeed = std::numeric_limits<double>::quiet_NaN();
    int estimatedExitRoadIndex = -1;
    ContinuousSpeedTransfer entrySpeedTransfer;

    int currentMovementID = -1;
    int currentBufferID = -1;

    VehicleState state = VehicleState::NotDeparted;

    bool finished = false;

    int occupiedRoadID = -1;
    int occupiedLaneIndex = -1;
    // occupiedLaneIndex remains the logical/model lane even while borrowing.
    int storageLaneIndex = -1;
    double occupiedLength = 0.0;
    bool countedInWaitingBufferLength = false;

    // invalid=true means route conversion failed or required movement/buffer is missing.
    // invalid vehicles should be excluded from normal cycle-aware evaluation.
    bool valid = true;
};

// Event stream 1/4: signal phase transitions.
// Separate from vehicle entry/exit and movement-dispatch events.
struct SignalEvent {
    double time = 0.0;
    int signalID = -1;
};

// Event stream 2/4: vehicle departures into the simulated road network.
struct DepartureEvent {
    double time = 0.0;
    int vehicleID = -1;
};

// Event stream 3/4: a vehicle physically leaves its final road and the network.
// roadID/laneIndex bind the event to the occupancy reserved when the vehicle
// entered that final road, so stale or inconsistent events can be detected.
struct FinalRoadExitEvent {
    double time = 0.0;
    int vehicleID = -1;
    int roadID = -1;
    int laneIndex = -1;
    int storageLaneIndex = -1;
};

struct LaneBorrowingCandidates {
    vector<int> toLanes, nextFromLanes, intersected, logicalLanes;
    vector<vector<int>> donorsByLogicalLane;
};

struct LaneBorrowingStats {
    long long admissions = 0, releases = 0, active = 0, peakActive = 0;
    long long storageReservations = 0, storageReleases = 0, clampedPredictions = 0;
};

// Event stream 4/4: movement-based dispatch attempt, not vehicle-based.
// timeLabel is the next attempt time for movementID. firstCarArriveTime and
// frontVehicleID bind the candidate to the current FIFO front for fair ordering
// and stale-candidate detection. version supports lazy invalidation because
// std::priority_queue cannot update keys in place.
struct VehicleRoadTimelineRecord {
    int vehicleID = -1;
    int routeRoadIndex = -1;
    int roadID = -1;
    int enteringMovementID = -1;
    int nextMovementID = -1;
    int laneIndex = -1;
    double intersectionCrossingTime = 0.0;
    double enterTime = 0.0;
    double arrivalAtRoadEndTime = 0.0;
    double roadTravelTime = 0.0;
};

struct VehicleMovementTimelineRecord {
    int vehicleID = -1;
    int routeMovementIndex = -1;
    int movementID = -1;
    int fromRoadID = -1;
    int toRoadID = -1;
    int chosenLane = -1;
    double arrivalAtWaitingBufferTime = 0.0;
    double dischargeTime = 0.0;
    double movementWaitingTime = 0.0;
    int connectionIndex = -1;
    int serviceLane = -1;
    int entryLane = -1;
    int storageLane = -1;
    uint64_t candidateVersion = 0;
};

// Optional evaluation trace captured whenever a vehicle frees lane storage.
// The dispatch queue compares candidateFirstLabel before frontArrivalSecondLabel,
// so recording both values makes cross-movement priority decisions auditable.
struct DownstreamReleaseCompetitionRecord {
    long long releaseSequence = 0;
    double releaseTime = 0.0;
    int releasedByVehicleID = -1;
    int releasedByMovementID = -1;
    int freedRoadID = -1;
    int freedLaneIndex = -1;
    int movementID = -1;
    int frontVehicleID = -1;
    int queueSize = 0;
    double movementLabelBefore = INF;
    double frontArrivalSecondLabel = INF;
    bool signalActiveAtRelease = false;
    double nextGreenTime = INF;
    double nextCapacityTime = INF;
    double candidateFirstLabel = INF;
    bool downstreamStorageAvailable = false;
    int candidateLane = -1;
    bool wasBlockedByDownstream = false;
    bool wasInDispatchQueue = false;
    bool reactivated = false;
};

struct DispatchCandidate {
    double timeLabel = 0.0;
    double firstCarArriveTime = INF;
    int movementID = -1;
    int frontVehicleID = -1;
    uint64_t version = 0;
    // Road-pair mode binds a real lane head and original connection. Legacy
    // candidates leave these fields unset.
    int serviceLane = -1;
    int connectionIndex = -1;
    int entryLane = -1;
    int storageLane = -1;
    int routeIndex = -1;
};

struct RoadPairLaneOption {
    int connectionIndex = -1;
    int entryLane = -1;
    int logicalLane = -1;
};

struct RoadPairChoice {
    int vehicleID = -1;
    int serviceLane = -1;
    int connectionIndex = -1;
    int entryLane = -1;
    int logicalLane = -1;
    int routeIndex = -1;
    double time = INF;
};

struct DispatchQueueStats {
    uint64_t windows = 0, fullRebuilds = 0, globalMovementVisits = 0;
    uint64_t pushes = 0, identicalSkips = 0, invalidations = 0, stalePops = 0;
    uint64_t timeNormalizations = 0, compactions = 0, compactedCandidates = 0;
    uint64_t signalEvents = 0, signalRefreshes = 0, deferredRetries = 0;
    size_t peakHeap = 0, peakActive = 0;
};

// Classification result for why a movement cannot discharge at candidate time.
// Reasons map to different rescheduling actions: RedSignal -> next green,
// Capacity -> earliest capacity-lane timestamp, NotArrived -> front vehicle label,
// DownstreamFull -> deactivate until downstream storage is freed.
enum class DischargeBlockReason {
    None,
    EmptyBuffer,
    NotArrived,
    RedSignal,
    Capacity,
    DownstreamFull,
    Invalid
};

// Runtime-only candidate data for movements that share the same dispatch time.
// These records let process_discharge_window classify a same-time batch before
// applying downstream-road round-robin ordering within each ready group.
struct ReadyDispatchCandidate {
    int movementID = -1;
    double time = 0.0;
    int bufferID = -1;
    int frontVehicleID = -1;
    int chosenLane = -1;
    DischargeBlockReason reason = DischargeBlockReason::Invalid;
};

// Output of a successful discharge.
// releasedRoadID/releasedLaneIndex identify freed storage so upstream movements
// blocked by downstream fullness can be reactivated.
struct DischargeResult {
    int vehicleID = -1;
    int movementID = -1;
    int intersectionID = -1;
    int fromRoadID = -1;
    int toRoadID = -1;
    double dischargeTime = 0.0;
    double newArrivalTime = 0.0;
    bool finished = false;
    int nextMovementID = -1;
    int nextBufferID = -1;
    int releasedRoadID = -1;
    int releasedLaneIndex = -1;
};

struct SignalEventCompare {
    bool operator()(const SignalEvent& a, const SignalEvent& b) const {
        if (a.time != b.time) return a.time > b.time;
        return a.signalID > b.signalID;
    }
};

struct DepartureEventCompare {
    bool operator()(const DepartureEvent& a, const DepartureEvent& b) const {
        if (a.time != b.time) return a.time > b.time;
        return a.vehicleID > b.vehicleID;
    }
};

struct FinalRoadExitEventCompare {
    bool operator()(const FinalRoadExitEvent& a, const FinalRoadExitEvent& b) const {
        if (a.time != b.time) return a.time > b.time;
        return a.vehicleID > b.vehicleID;
    }
};

struct DispatchCandidateCompare {
    bool operator()(const DispatchCandidate& a, const DispatchCandidate& b) const {
        if (a.timeLabel != b.timeLabel) {
            return a.timeLabel > b.timeLabel;
        }
        if (a.firstCarArriveTime != b.firstCarArriveTime) {
            return a.firstCarArriveTime > b.firstCarArriveTime;
        }
        if (a.movementID != b.movementID) {
            return a.movementID > b.movementID;
        }
        return a.version > b.version;
    }
};

class Graph {
public:

    // Step 1: Data Cleaning
    // -----------------------------------------------------------------------------

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
    vector<vector<pair<int, double>>> graphLength;   // ID1, ID2, Length
    vector<vector<pair<int, int>>> graphRoadID;     // ID1, ID2, RoadID
    vector<Road> roadInfor;                         // Road Info (ID1, ID2, RoadID, Length, Time
    vector<set<int>> adjNodes;                      // ID1, ID2
    map<pair<int, int>, int> nodeID2RoadID;         // <ID1, ID2> -> RoadID
    map<int, pair<int, int>> roadID2NodeID;         // RoadID -> <ID1, ID2>
    map<pair<int, int>, double> nodeID2minTime;     // <ID1, ID2> -> Min Travel Time
    string BJ_minTravleTime = join_path(Base,                // ID1, ID2, Weight (Min Travel Time in s)
                              "Manhattan_network_min_Travel_Time.txt");
    vector<vector<pair<int, double>>> graphTime;    // ID1, ID2, Weight (Min Travel Time)
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
    vector<vector<double>> read_time(string filename, int num, const vector<vector<int>>& query);
    string time_path = join_path(Base, "time.txt");
    vector<vector<double>> timeDataRaw;

    // SUMO .net.xml input and adaptation
    string sumoNetPath = roadnet_defaults::DEFAULT_SUMO_NET_PATH;
    string sumoRoutePath;
    string sumoTripinfoPath;
    string evalOutputPath;
    string predictionTripinfoOutputPath;
    bool evalSplitExtreme = false;
    bool evalExtremeTimeline = false;
    bool evalTrajectoryDetails = false;
    double evalExtremeRelativeDurationThreshold = 3.0;
    void read_sumo_net_xml(const string& netXmlPath);
    void read_sumo_route_xml(const string& routeXmlPath, int maxVehicles);
    void read_sumo_tripinfo_xml(const string& tripinfoPath);
    double evaluate_sumo_tripinfo_truth(const vector<vector<pair<int, double>>>& ETA);
    void classify_node_types_from_sumo_junctions();
    void build_movements_from_sumo_connections();
    void build_continuous_speed_path_limits();
    void build_lane_groups_from_sumo_connections();
    void build_signal_programs_from_sumo_tllogic();
    void build_new_graph_structures_from_sumo();
    void append_unsignalized_dispatch_intersections();
    SignalState signalStateAtMovement(int movementID, double t);
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
    // False for legacy/unloaded graphs. Set only from a successfully loaded
    // SUMO network's actual internal edges / connection via references.
    bool sumoNetworkWithoutInternalLinks = false;
    bool sumoHasSignalProgramSwitching = false;
    map<tuple<int, int, int>, OutgoingSignalRGContext> outgoingSignalRGCache;
    // Feature-only constant for uncontrolled exits, never a signal timer.
    // Lazy so speed-net and other non-RG modes need no finite green program.
    double unsignalizedGreenHorizonCache = numeric_limits<double>::quiet_NaN();
    double unsignalizedGreenHorizon();
    SignalRGFeatures outgoingSignalRG(int roadID, int nextRoadID, int logicalLane,
                                     double entryTime, string* tls = nullptr, int* link = nullptr);
    vector<SumoTripInfoTruth> sumoTruthAligned;
    unordered_map<string, SumoTripInfoTruth> sumoTruthByVehicleID;
    unordered_map<string, VehicleType> sumoVehicleTypes;
    vector<string> sumoVehicleIDs;
    vector<string> vehicleTypeIDs;
    vector<double> vehicleSpeedFactors;
    vector<bool> vehicleHasExplicitSpeedFactor;
    vector<double> vehicleDepartureTimes;
    // SUMO front-bumper coordinate. Unknown policies remain unresolved, never 0.
    vector<double> vehicleDeparturePositions;
    vector<string> vehicleDeparturePositionIssues;
    vector<SignalProgram> signalPrograms;
    unordered_map<string, int> movementKeyToID;
    // Check if route data, query data, and time data size are same
    void check_size();
    // Cut route data
    vector<vector<int>> cut_route_data(vector<vector<int>> &routeDataRaw, int avg_length);
    // Cut query data
    vector<vector<int>> cut_query_data(vector<vector<int>> &queryDataRaw, vector<vector<int>> &routeData, int avg_length);
    // Cut time data
    vector<vector<double>> cut_time_data(const vector<vector<double>>& timeDataRaw, int avg_length);

    // Convert Route from "Node ID Pair" to "Road ID"
    void route_nodeID_2_roadID(vector<vector<int>> &routeData);
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
    void initializeRoadLaneStorage();
    void validate_cycle_aware_graph();
    bool validateRoadLaneFlows() const;
    const VehicleType& getVehicleTypeForVehicle(int vehicleID) const;
    bool hasExplicitVehicleSpeedFactor(int vehicleID) const;
    // Both getters return NaN when the per-vehicle SF is unavailable. This
    // exposes metadata without changing the legacy prediction fallback rules.
    double getVehicleSpeedFactor(int vehicleID) const;
    double getVehicleFreeSpeedCap(int vehicleID, double laneSpeedLimit) const;
    vector<int> parseLaneIndices(const vector<string>& lanes, int roadID = -1) const;
    vector<int> laneIntersection(const vector<int>& a, const vector<int>& b) const;
    bool hasLaneStorageCapacity(int roadID, int laneIndex, int vehicleID = -1) const;
    int chooseLeastOccupiedAvailableLane(int roadID, const vector<int>& candidateLanes, int vehicleID = -1) const;
    void reserveLaneOccupancy(int vehicleID, int roadID, int laneIndex, int storageLaneIndex = -1);
    void releaseLaneOccupancy(int vehicleID);
    bool hasDownstreamLaneStorage(int movementID, int vehicleID, int &chosenLane, int* chosenStorageLane = nullptr);
    void initialize_lane_borrowing();
    const LaneBorrowingCandidates& lane_borrowing_candidates(int movementID, int vehicleID);
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
    // Static during a run; rebuild after manually changing movement topology.
    void rebuild_incoming_movement_index();
    map<int, vector<int>> laneGroupsByRoad;
    map<pair<int, TurnDir>, int> roadTurnToLaneGroupID;
    map<int, int> nodeToIntersectionID;
    map<int, vector<int>> movementIDsByIntersection;
    map<int, vector<int>> signalIDsByIntersection;

    vector<VehicleLabel> vehicles;
    priority_queue<SignalEvent, vector<SignalEvent>, SignalEventCompare> signalEventPQ;
    priority_queue<DepartureEvent, vector<DepartureEvent>, DepartureEventCompare> departurePQ;
    priority_queue<FinalRoadExitEvent, vector<FinalRoadExitEvent>, FinalRoadExitEventCompare> finalRoadExitPQ;
    priority_queue<DispatchCandidate, vector<DispatchCandidate>, DispatchCandidateCompare> dispatchPQ;
    vector<vector<pair<int, double>>> ETA_result_cycle_aware;
    vector<vector<VehicleRoadTimelineRecord>> vehicleRoadTimeline;
    vector<vector<VehicleMovementTimelineRecord>> vehicleMovementTimeline;
    vector<DownstreamReleaseCompetitionRecord> downstreamReleaseCompetitionTimeline;
    long long downstreamReleaseSequence = 0;
    int finishedVehicleCount = 0;
    int invalidVehicleCount = 0;
    double defaultDischargeInterval = 2.0;
    double intersectionCrossingTime = 3.0;
    MovementQueueOrder movementQueueOrder = MovementQueueOrder::ArrivalTime;
    vector<vector<double>> movementLaneNextAvailableTime;
    // Road-pair mode: WB remains unique membership; this index orders the same
    // IDs across all movements using one physical/logical source lane.
    vector<vector<deque<int>>> roadPairLaneQueues;
    vector<vector<double>> roadPairLaneNextAvailable;
    vector<vector<vector<int>>> roadPairLaneSubscribers;
    vector<vector<int>> roadPairSourceLanes;
    vector<vector<SumoLaneRaw>> roadPairLaneMetadata;
    vector<bool> roadPairInternalPathComplete;
    vector<vector<pair<string, string>>> roadPairInternalPathPermissions;
    vector<RoadPairChoice> roadPairChoices;
    map<tuple<int, int, int, string, bool>, vector<RoadPairLaneOption>> roadPairOptionCache;
    mutable map<tuple<int, int, string>, vector<int>> roadPairPermittedEntryCache;
    uint64_t roadPairVirtualProjections = 0;
    uint64_t roadPairReservations = 0, roadPairReleases = 0;
    uint64_t roadPairCandidateLaneVisits = 0;
    bool roadPairRuntimeInitialized = false;
    void initialize_road_pair_runtime(double initialTime);
    bool road_pair_lane_permitted(int roadID, int lane, const string& vehicleClass) const;
    bool road_pair_connection_permitted(int movementID, int connectionIndex, const string& vehicleClass) const;
    const vector<int>& road_pair_entry_lanes(int movementID, int roadID, int vehicleID) const;
    const vector<RoadPairLaneOption>& road_pair_lane_options(int movementID, int vehicleID);
    bool road_pair_option_has_capacity(int movementID, int vehicleID, const RoadPairLaneOption& option) const;
    bool road_pair_select_lane(int movementID, int vehicleID, RoadPairChoice& choice);
    bool road_pair_is_head(int movementID, int vehicleID) const;
    int road_pair_front(int movementID) const;
    RoadPairChoice road_pair_compute_choice(int movementID, double lowerBound);
    void road_pair_schedule(int movementID, double lowerBound);
    void road_pair_refresh_lane(int roadID, int lane, double currentTime);
    void road_pair_insert_lane(int vehicleID);
    void road_pair_pop_and_consume(int movementID, int vehicleID, double time);
    bool road_pair_validate_runtime(bool requireEmpty) const;
    vector<double> movementTimeLabel;
    vector<uint64_t> movementPQVersion;
    vector<bool> movementBlockedByDownstream;
    vector<bool> movementInDispatchPQ;
    // CLI enables this by default for SUMO. Legacy/embedded callers opt in.
    bool persistentDispatch = false;
    // Opt in to one movement per ordered (fromRoadID, toRoadID), with exact
    // original connections and shared source-lane queues/service clocks.
    bool mergeRoadPairMovements = false;
    // Fixed per run. Real entry must be available; choose the downstream road's
    // service/storage lane once at admission, before prediction and WB insertion.
    // Requires road-pair mode and borrowing off; not an in-road microscopic LC.
    bool laneEntryReselection = false;
    // Fixed per run; no change to model schema, service capacity, or queue order.
    bool laneBorrowing = false;
    LaneBorrowingStats laneBorrowingStats;
    vector<vector<SumoLaneRaw>> laneBorrowingLaneMetadata;
    vector<vector<int>> laneBorrowingConnections;
    map<tuple<int, int, int, string>, LaneBorrowingCandidates> laneBorrowingCandidateCache;
    vector<DispatchCandidate> latestDispatchCandidates;
    vector<int> activeDispatchMovements, dispatchActivePosition;
    vector<int> deferredDispatchRetries;
    vector<bool> dispatchRetryPending;
    size_t dispatchCompactMinSize = 4096;
    DispatchQueueStats dispatchQueueStats;
    vector<vector<deque<int>>> entryLaneDepartureQueues;
    // Step 3: ALGORITHM I SIMULATION
    // -----------------------------------------------------------------------------

    //
    void readConnectionsToDirections(const std::string& filename);
    map<pair<int, int>, char> connections_to_direction;
    string connections_to_direction_path = join_path(Base, "connections_to_directions.csv");

    void buildDictionary(const std::string& filename);
    unordered_map<RoadKey, double> dictionary;
    TravelTimeTableFormat travelTimeTableFormat = TravelTimeTableFormat::LEGACY;
    unordered_map<SumoV1TravelTimeKey, double, SumoV1TravelTimeKeyHash> sumoV1TravelTimeTable;
    SfTravelTimeTable sfTravelTimeTable;
    CapContextTable capContextTable;
    CapContextABCRGTable capContextABCRGTable;
    NativeRoadTimeTable nativeRoadTimeTable;
    CapContextLedger capContextLedger;
    string capContextTracePath;
    ofstream capContextTrace;
    // Explicit discovery only: provisional misses are never final predictions.
    string demandRequestsPath;
    ofstream demandRequests;
    unordered_set<std::uint64_t> demandMissingKeys;
    unordered_set<NativeDemandKey, NativeDemandKeyHash> nativeDemandMissingKeys;
    std::size_t demandMaxMissingKeys = 2000000;
    bool usesNativeRoadTimeTable() const {
        return travelTimeMode == TravelTimeMode::TABLE &&
               travelTimeTableFormat == TravelTimeTableFormat::CAMS_NATIVE_ABC_RG_DEMAND_V2;
    }
    bool capContextUsesDeparturePositionCorrection() const {
        // Frozen historical cap/context tables keep their original geometry.
        // Current native tables explicitly bind this switch in their header.
        return !usesNativeRoadTimeTable() || speedNetDepartPosition;
    }
    bool usesDemandCapContextTable() const {
        return travelTimeMode == TravelTimeMode::TABLE &&
               (travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_NATIVE_ABC_RG_DEMAND_V2);
    }
    bool usesRGCapContextTable() const {
        return travelTimeMode == TravelTimeMode::TABLE &&
               (travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_NATIVE_ABC_RG_DEMAND_V2);
    }
    double capContextOverdueThreshold() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.overdueThreshold()
             : usesRGCapContextTable() ? capContextABCRGTable.overdueThreshold()
                                       : capContextTable.overdueThreshold();
    }
    void requireCapContextCrossingTime() const {
        if (usesNativeRoadTimeTable()) nativeRoadTimeTable.requireCrossingTime(intersectionCrossingTime);
        else if (usesRGCapContextTable()) capContextABCRGTable.requireCrossingTime(intersectionCrossingTime);
        else capContextTable.requireCrossingTime(intersectionCrossingTime);
    }
    bool usesCapContextTable() const {
        return travelTimeMode == TravelTimeMode::TABLE &&
               (travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_NATIVE_ABC_RG_DEMAND_V2);
    }
    bool usesABCCapContextTable() const {
        return travelTimeMode == TravelTimeMode::TABLE &&
               (travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_RLE_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 ||
                travelTimeTableFormat == TravelTimeTableFormat::CAMS_NATIVE_ABC_RG_DEMAND_V2);
    }
    // Dispatch format contracts only; traffic clocks/ABC/lane logic are shared
    // unchanged. Native-trained inputs and C++ proxies are explicitly distinct.
    void requireRGUnsignalizedContract() const {
        if (usesNativeRoadTimeTable()) nativeRoadTimeTable.requireUnsignalizedContract();
        else capContextABCRGTable.requireUnsignalizedContract();
    }
    void requireRGNoInternalLinksContract() const {
        if (usesNativeRoadTimeTable()) nativeRoadTimeTable.requireNoInternalLinksContract();
        else capContextABCRGTable.requireNoInternalLinksContract();
    }
    void requireRGSignalRemainingStep(double value) const {
        if (usesNativeRoadTimeTable()) nativeRoadTimeTable.requireSignalRemainingStep(value);
        else capContextABCRGTable.requireSignalRemainingStep(value);
    }
    SignalRGFeatures quantizeRGSignal(const SignalRGFeatures& value) const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.quantizeSignal(value)
                                       : capContextABCRGTable.quantizeSignal(value);
    }
    double rgSignalRemainingStep() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.signalRemainingStep() : capContextABCRGTable.signalRemainingStep();
    }
    std::size_t rgSignalPointCount() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.signalPointCount() : capContextABCRGTable.signalPointCount();
    }
    std::uint64_t rgTableSize() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.size() : capContextABCRGTable.size();
    }
    const string& rgTableLabel() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.label() : capContextABCRGTable.label();
    }
    std::size_t rgTableBytes() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.compressedBytes() : capContextABCRGTable.compressedBytes();
    }
    std::size_t demandEntryCount() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.demandEntryCount() : capContextABCRGTable.demandEntryCount();
    }
    const string& demandScope() const {
        return usesNativeRoadTimeTable() ? nativeRoadTimeTable.demandScope() : capContextABCRGTable.demandScope();
    }
    std::size_t demandMissingCount() const {
        return usesNativeRoadTimeTable() ? nativeDemandMissingKeys.size() : demandMissingKeys.size();
    }
    string model_catch_dic_path = join_path(Base, "model_catching_with_travel_time_1.txt");

    TravelTimeMode travelTimeMode = TravelTimeMode::MIN_TIME;
    string travelTimeTablePath = join_path(Base, "model_catching_with_travel_time_1.txt");
    bool fallbackToSpeedNet = true;
    bool verboseTravelTimePrediction = false;
    string modelHost = "127.0.0.1";
    int modelPort = 9000;
    bool modelWarningPrinted = false;
    bool debugMovement53Enabled = false;
    double debugCurrentTime = -1.0;
    mutable ofstream debugMovement53Timeline;
    mutable ofstream debugE13Occupancy;
    mutable ofstream debugLane53;
    void initMovement53DebugIfNeeded();
    void debugMovement53Event(double time, const string& eventType, int movementID, int frontVehicleID, int chosenLane, const string& blockReason, const string& note);
    void debugE13OccupancyEvent(double time, const string& eventType, int vehicleID, int roadID, int laneIndex, int movementID, const string& note, int laneFlowBefore=-1, int roadFlowBefore=-1, double occupiedBefore=-1.0);
    void debugMovement53LaneCheck(double time, int movementID, int vehicleID, const vector<int>& toLanes, const vector<int>& nextFromLanes, const vector<int>& intersected, const vector<int>& candidateLanes, int chosenLane, bool result);
    int travelTimeTableHit = 0;
    int travelTimeTableMiss = 0;
    // Constant-time audit counters; no new simulation events or queue rules.
    long long sfTableRoadFlowOutsideTraining = 0;
    long long sfTableLaneFlowOutsideTraining = 0;
    int sfTableMaxRoadFlowSeen = 0;
    int sfTableMaxLaneFlowSeen = 0;
    double speedNetHeadway = 0.6;
    double speedNetDelayMax = 3.0;
    bool speedNetUseAcceleration = true;
    bool speedNetUseFollowing = true;
    bool speedNetQueueTrim = false;
    // CLI: --first-road-position-correction; old name remains an alias.
    // Native table headers bind this policy; historical cap/context stays fixed.
    bool speedNetDepartPosition = true;
    // Opt-in: preserve previous predictions and reject missing SF when enabled.
    bool speedNetUseSpeedFactor = false;
    bool speedNetContinuousSpeed = false;
    double speedNetContinuousWaitThreshold = 0.0;
    string speedNetContinuousTracePath;
    ofstream speedNetContinuousTrace;
    void requireContinuousSpeedConfiguration() const;
    ContinuousSpeedTransfer continuousSpeedTransfer(const VehicleLabel& vehicle, int movementID,
                                                    double dischargeTime) const;
    void commitContinuousSpeedPrediction(int vehicleID, const BasicRoadModelFeatures& features,
                                         const SpeedNetMotionResult& prediction);
    double kinematicCongestionAlpha = 1.0;
    bool enableBasicFeatureLogging = false;
    vector<BasicRoadModelFeatures> basicFeatureSnapshots;
    void exportBasicFeatureSnapshots(const string& path) const;
    // 现行新算法主入口：基于 signal event + waiting buffer + discharge capacity 的仿真
    vector<vector<pair<int, double>>> cycle_aware_signal_driven_records(
            vector<vector<int>> &Q, vector<vector<int>> &routeRoadID);
    void initialize_cycle_aware_vehicles(vector<vector<int>>& Q, vector<vector<int>>& routeRoadID);
    void initialize_signal_event_queue(double simStartTime);
    void initializeMovementDischargeTimes(double initialTime);
    void initializeEntryLaneDepartureQueues();
    bool hasPendingEntryDepartureQueues() const;
    bool processScheduledDeparturesUntil(double t);
    void scheduleFinalRoadExit(int vehicleID, double exitTime);
    bool processFinalRoadExitsUntil(double t);
    bool enqueueDepartingVehicle(int vehicleID, double scheduledTime);
    bool tryStartVehicleFromEntryQueue(int vehicleID, double actualDepartTime, int firstRoad, int laneIndex);
    void processEntryDepartureQueueForRoadAtTime(int roadID, double t);
    void processEntryDepartureQueuesAtTime(double t);
    bool departVehicle(int vehicleID, double departTime);
    void process_discharge_window(double windowStart, double windowEnd);
    void rebuildActiveDispatchPQ(double currentTime, double windowEnd);
    void reset_dispatch_state();
    void ensure_dispatch_state();
    uint64_t next_dispatch_version(int movementID);
    void retire_dispatch_candidate(int movementID);
    void invalidate_dispatch_candidate(int movementID);
    void refresh_dispatch_candidate(int movementID, double currentTime);
    void defer_dispatch_retry(int movementID);
    void discard_stale_dispatch_tops();
    void prepare_dispatch_window(double windowStart, double windowEnd);
    void compact_dispatch_queue(bool force = false);
    bool isMovementActive(int movementID, double t);
    // Physical signal identity, not the alwaysOpen dispatch flag. Only known
    // unsignalized SUMO junctions use zero crossing; unknown/legacy nodes keep
    // the configured signalized crossing time.
    bool isUnsignalizedMovement(int movementID) const;
    double movementCrossingTime(int movementID) const;
    DischargeResult dischargeOneVehicle(int movementID, double dischargeTime);
    void pushCandidateIfPossible(int movementID, double currentTime, double windowEnd);
    bool isDispatchCandidateValid(const DispatchCandidate& c);
    int getMovementBufferID(int movementID) const;
    int getFrontVehicleForMovement(int movementID) const;
    double computeMovementAttemptTime(int movementID, double lowerBoundTime);
    double nextGreenTimeForMovement(int movementID, double t);
    DischargeBlockReason getDischargeBlockReason(int movementID, double t, int &frontVehicleID, int &chosenLane);
    void scheduleMovementCandidate(int movementID, double time);
    bool scheduleInsertedVehicleIfMovementFront(
            int movementID,
            int insertedVehicleID,
            double currentTime);
    void deactivateMovementForDownstreamBlock(int movementID);
    void reactivateMovementsBlockedByRoad(
            int freedRoadID,
            double currentTime,
            int freedLaneIndex = -1,
            int releasedByVehicleID = -1,
            int releasedByMovementID = -1);
    vector<int> orderMovementsByDownstreamRoundRobin(
            int intersectionID,
            int toRoadID,
            const vector<int>& movementIDs
    );
    void markDownstreamRoundRobinServed(int movementID);
    void handleSuccessfulDischargePostUpdate(
            int movementID,
            double t,
            double oldLabel,
            int bufferID,
            const DischargeResult& result
    );
    bool hasDischargeCapacity(int movementID, double t);
    void consumeDischargeCapacity(int movementID, double t);
    double nextAvailableCapacityTime(int movementID, double t);
    BasicRoadModelFeatures buildBasicRoadModelFeatures(
            int roadID,
            int vehicleID,
            int movementID,
            double currentTime,
            int preferredLaneIndex = -1
    ) const;
    int encodeTurnDirForModel(TurnDir turn) const;
    double predictRoadTravelTimeFromFeatures(
            int roadID,
            const BasicRoadModelFeatures& features
    );
    double predictRoadTravelTime(
            int roadID,
            int vehicleID,
            int movementID,
            double currentTime,
            int preferredLaneIndex = -1,
            double decisionTime = std::numeric_limits<double>::quiet_NaN()
    );
    double predictRoadTravelTime(int roadID, int vehicleID);
    double predictRoadTravelTimeSpeedNet(const BasicRoadModelFeatures& features, bool forceExplicitSF = false) const;
    SpeedNetMotionResult evaluateSpeedNetMotion(const BasicRoadModelFeatures& features, bool forceExplicitSF = false) const;
    double predictRoadTravelTimeSpeedNet(int roadID) const;
    double predictRoadTravelTimeMinTime(int roadID) const;
    double predictRoadTravelTimeTable(const BasicRoadModelFeatures& features);
    double predictRoadTravelTimeTable(int roadID, int vehicleID);
    double predictRoadTravelTimeModel(const BasicRoadModelFeatures& features);
    double predictRoadTravelTimeModel(int roadID, int vehicleID);
    double predictRoadTravelTimeKinematic(const BasicRoadModelFeatures& features) const;
    double predictRoadTravelTimeKinematic(int roadID, int vehicleID) const;
    RoadKey buildRoadKeyForPrediction(const BasicRoadModelFeatures& features) const;
    SumoV1TravelTimeKey buildSumoV1TravelTimeKeyForPrediction(const BasicRoadModelFeatures& features) const;
    RoadKey buildRoadKeyForPrediction(int roadID, int vehicleID) const;
    bool queryExternalTravelTimeModel(const BasicRoadModelFeatures& features, double& predictedTime);
    void registerWaitingBufferLength(int bufferID, int vehicleID);
    void insertVehicleToBufferOrdered(int bufferID, int vehicleID);
    void handle_signal_change_event(const SignalEvent& e, double dispatchTime = -INF);
    double nextSignalChangeTime(int signalID, double afterTime);
    bool allVehiclesFinished() const;
    void recordFinalETA(int vehicleID, double finalTime);
    float percent;
    int small, big;

    double MSE_estimation_cycle_aware_total(
            vector<vector<double>> &timeData,
            vector<vector<pair<int, double>>> &ETA);
    // Find previous or next edge of current one.
    int find_direction(vector<vector<int>> &routeData, vector<vector<int>> &routeDataEdge,
                       int routeID, int nodeStart, int nodeEnd);
    // Export record information on each edge for further analysis
    void export_time_records(string route_num, vector<vector<int>> &routeData,
                             vector<vector<int>> &routeDataEdge, vector<vector<double>> &timeData,
                             vector<vector<pair<int,double>>> &ETA);
    string file_path_time_records = join_path(Base, "time_records_out.txt");

    // Legacy BJ setup helper used by the cycle-aware workflow.
    void min_depar_time(vector<vector<int>> &Q);
    int minDeparture = 0;
    int minHour = 0;

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
