#include "head.h"

namespace {
bool contains_class(const string& classes, const string& vehicleClass) {
    istringstream tokens(classes);
    string token;
    while (tokens >> token) {
        if (token == "all" || token == vehicleClass) return true;
    }
    return false;
}

bool permits_class(const string& allow, const string& disallow, const string& vehicleClass) {
    return vehicleClass == "ignoring" ||
           (contains_class(allow, vehicleClass) && !contains_class(disallow, vehicleClass));
}
}

void Graph::initialize_lane_borrowing() {
    laneBorrowingStats = LaneBorrowingStats();
    laneBorrowingCandidateCache.clear();
    laneBorrowingLaneMetadata.clear();
    laneBorrowingConnections.clear();
    if (!laneBorrowing) return;

    laneBorrowingLaneMetadata.resize(roads.size());
    for (size_t road = 0; road < roads.size(); ++road) {
        auto& lanes = laneBorrowingLaneMetadata[road];
        lanes.resize(roads[road].laneFlow.size());
        // Hand-built graphs have no XML permissions. Loaded SUMO graphs fail
        // closed for missing lane metadata, rather than inventing access.
        if (sumoEdgesRaw.empty()) {
            for (size_t lane = 0; lane < lanes.size(); ++lane) lanes[lane].index = static_cast<int>(lane);
        }
    }
    for (const auto& edge : sumoEdgesRaw) {
        auto road = sumoEdgeStrToRoadID.find(edge.id);
        if (road == sumoEdgeStrToRoadID.end() || road->second < 0 ||
            road->second >= static_cast<int>(roads.size())) continue;
        auto& lanes = laneBorrowingLaneMetadata[road->second];
        for (const auto& lane : edge.lanes) {
            if (lane.index >= 0 && lane.index < static_cast<int>(lanes.size())) lanes[lane.index] = lane;
        }
    }

    laneBorrowingConnections.resize(movements.size());
    map<tuple<int, int, string, int>, int> movementByConnection;
    for (size_t i = 0; i < movements.size(); ++i) {
        const auto& m = movements[i];
        if (!m.connectionIndices.empty()) {
            laneBorrowingConnections[i] = m.connectionIndices;
        } else {
            // Retain the legacy fallback for hand-built graphs without indexes.
            movementByConnection[make_tuple(m.fromRoadID, m.toRoadID, m.tlID, m.linkIndex)] = static_cast<int>(i);
        }
    }
    for (size_t i = 0; i < sumoConnectionsRaw.size(); ++i) {
        const auto& c = sumoConnectionsRaw[i];
        auto from = sumoEdgeStrToRoadID.find(c.fromEdge);
        auto to = sumoEdgeStrToRoadID.find(c.toEdge);
        if (from == sumoEdgeStrToRoadID.end() || to == sumoEdgeStrToRoadID.end()) continue;
        auto movement = movementByConnection.find(make_tuple(from->second, to->second, c.tl, c.linkIndex));
        if (movement != movementByConnection.end()) laneBorrowingConnections[movement->second].push_back(static_cast<int>(i));
    }
}

const LaneBorrowingCandidates& Graph::lane_borrowing_candidates(int movementID, int vehicleID) {
    const auto& vehicle = vehicles.at(vehicleID);
    const auto& movement = movements.at(movementID);
    const int routeIndex = vehicle.roadIndex + 1;
    const int nextMovement = routeIndex >= 0 && routeIndex < static_cast<int>(vehicle.routeMovementIDs.size())
        ? vehicle.routeMovementIDs[routeIndex] : -1;
    const int incomingLane = vehicle.occupiedLaneIndex;
    const string& vehicleClass = getVehicleTypeForVehicle(vehicleID).vehicleClass;
    const auto key = make_tuple(movementID, nextMovement, incomingLane, vehicleClass);
    auto cached = laneBorrowingCandidateCache.find(key);
    if (cached != laneBorrowingCandidateCache.end()) return cached->second;

    LaneBorrowingCandidates plan;
    const int roadID = movement.toRoadID;
    const int laneCount = static_cast<int>(roads.at(roadID).laneFlow.size());
    plan.toLanes = parseLaneIndices(movement.toLanes, roadID);
    plan.logicalLanes = plan.toLanes;
    if (nextMovement >= 0 && nextMovement < static_cast<int>(movements.size())) {
        plan.nextFromLanes = parseLaneIndices(movements[nextMovement].fromLanes, roadID);
        plan.intersected = laneIntersection(plan.toLanes, plan.nextFromLanes);
        plan.logicalLanes = plan.intersected.empty() ? plan.nextFromLanes : plan.intersected;
    }
    if (plan.logicalLanes.empty()) plan.logicalLanes = plan.toLanes;
    if (plan.logicalLanes.empty()) plan.logicalLanes.push_back(0);
    plan.donorsByLogicalLane.resize(laneCount);
    // Final roads already use all entrance lanes; there is no next turn to borrow for.
    if (nextMovement < 0 || nextMovement >= static_cast<int>(movements.size())) {
        return laneBorrowingCandidateCache.emplace(key, std::move(plan)).first->second;
    }

    const bool synthetic = sumoEdgesRaw.empty() && sumoConnectionsRaw.empty();
    vector<bool> entrance(laneCount, false), exit(laneCount, false), permitted(laneCount, false);
    if (synthetic) {
        for (int lane : plan.toLanes) if (lane >= 0 && lane < laneCount) entrance[lane] = true;
        for (int lane : plan.nextFromLanes) if (lane >= 0 && lane < laneCount) exit[lane] = true;
    } else {
        if (movementID < static_cast<int>(laneBorrowingConnections.size())) {
            for (int index : laneBorrowingConnections[movementID]) {
                const auto& c = sumoConnectionsRaw[index];
                if (c.fromLane == incomingLane && c.toLane >= 0 && c.toLane < laneCount &&
                    permits_class(c.allow, c.disallow, vehicleClass)) entrance[c.toLane] = true;
            }
        }
        if (nextMovement < static_cast<int>(laneBorrowingConnections.size())) {
            for (int index : laneBorrowingConnections[nextMovement]) {
                const auto& c = sumoConnectionsRaw[index];
                if (c.fromLane >= 0 && c.fromLane < laneCount &&
                    permits_class(c.allow, c.disallow, vehicleClass)) exit[c.fromLane] = true;
            }
        }
    }
    const auto& metadata = laneBorrowingLaneMetadata.at(roadID);
    for (int lane = 0; lane < laneCount; ++lane) {
        permitted[lane] = metadata[lane].index == lane &&
            permits_class(metadata[lane].allow, metadata[lane].disallow, vehicleClass);
    }
    // O(L^2) static reachability, with SUMO index+1 meaning a change left.
    // Dynamic gap acceptance is intentionally replaced by virtual merging.
    vector<vector<bool>> reachable(laneCount, vector<bool>(laneCount, false));
    for (int start = 0; start < laneCount; ++start) {
        if (!permitted[start]) continue;
        reachable[start][start] = true;
        for (int direction : {-1, 1}) {
            int lane = start;
            while (lane + direction >= 0 && lane + direction < laneCount) {
                const string& classes = direction > 0 ? metadata[lane].changeLeft : metadata[lane].changeRight;
                if ((vehicleClass != "ignoring" && !contains_class(classes, vehicleClass)) || !permitted[lane + direction]) break;
                lane += direction;
                reachable[start][lane] = true;
            }
        }
    }
    for (int logical : plan.logicalLanes) {
        if (logical < 0 || logical >= laneCount || !exit[logical]) continue;
        for (int storage : plan.toLanes) {
            if (storage >= 0 && storage < laneCount && storage != logical &&
                entrance[storage] && reachable[storage][logical]) plan.donorsByLogicalLane[logical].push_back(storage);
        }
    }
    return laneBorrowingCandidateCache.emplace(key, std::move(plan)).first->second;
}
