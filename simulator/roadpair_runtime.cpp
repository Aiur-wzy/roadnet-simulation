#include "head.h"

namespace {
bool class_list_contains(const string& list, const string& vehicleClass) {
    istringstream tokens(list);
    string token;
    while (tokens >> token) if (token == "all" || token == vehicleClass) return true;
    return false;
}

bool class_permitted(const string& allow, const string& disallow, const string& vehicleClass) {
    return vehicleClass == "ignoring" ||
        (class_list_contains(allow, vehicleClass) && !class_list_contains(disallow, vehicleClass));
}
}

void Graph::initialize_road_pair_runtime(double initialTime) {
    if (laneEntryReselection && (!mergeRoadPairMovements || laneBorrowing))
        throw runtime_error("Lane entry reselection requires road-pair movements and lane borrowing off");
    if (!mergeRoadPairMovements) return;
    if (laneBorrowing) throw runtime_error("Road-pair runtime currently requires lane borrowing off");
    roadPairLaneQueues.assign(roads.size(), {});
    roadPairLaneNextAvailable.assign(roads.size(), {});
    roadPairLaneSubscribers.assign(roads.size(), {});
    roadPairLaneMetadata.assign(roads.size(), {});
    roadPairInternalPathComplete.assign(sumoConnectionsRaw.size(), false);
    roadPairInternalPathPermissions.assign(sumoConnectionsRaw.size(), {});
    roadPairSourceLanes.assign(movements.size(), {});
    roadPairChoices.assign(movements.size(), RoadPairChoice());
    roadPairOptionCache.clear();
    roadPairPermittedEntryCache.clear();
    roadPairVirtualProjections = 0;
    roadPairReservations = roadPairReleases = 0;
    roadPairCandidateLaneVisits = 0;
    for (size_t r = 0; r < roads.size(); ++r) {
        const int lanes = max(1, roads[r].laneNum);
        roadPairLaneQueues[r].resize(lanes);
        roadPairLaneNextAvailable[r].assign(lanes, initialTime);
        roadPairLaneSubscribers[r].resize(lanes);
        roadPairLaneMetadata[r].resize(lanes);
        if (sumoEdgesRaw.empty()) {
            for (int lane = 0; lane < lanes; ++lane) roadPairLaneMetadata[r][lane].index = lane;
        }
    }
    for (const auto& edge : sumoEdgesRaw) {
        const auto found = sumoEdgeStrToRoadID.find(edge.id);
        if (found == sumoEdgeStrToRoadID.end()) continue;
        auto& lanes = roadPairLaneMetadata.at(found->second);
        for (const auto& lane : edge.lanes) {
            if (lane.index >= 0 && lane.index < static_cast<int>(lanes.size())) lanes[lane.index] = lane;
        }
    }
    // Compile original internal paths once. Missing/branching/cyclic paths
    // cannot be claimed vehicle-class legal, even when their speed cap is off.
    struct InternalLaneRef { const SumoEdgeRaw* edge; const SumoLaneRaw* lane; };
    unordered_map<string, InternalLaneRef> internalLanes;
    set<string> duplicateInternalLanes;
    map<pair<string,int>, vector<int>> continuations;
    for (const auto& edge : sumoEdgesRaw) if (edge.isInternal) {
        for (const auto& lane : edge.lanes) {
            if (!internalLanes.emplace(lane.id, InternalLaneRef{&edge,&lane}).second) duplicateInternalLanes.insert(lane.id);
        }
    }
    for (size_t index = 0; index < sumoConnectionsRaw.size(); ++index) {
        const auto& c = sumoConnectionsRaw[index];
        continuations[{c.fromEdge,c.fromLane}].push_back(static_cast<int>(index));
    }
    for (const auto& movement : movements) {
        if (movement.connectionIndices.empty())
            throw runtime_error("Road-pair runtime requires original connections for every movement");
        auto& sourceLanes = roadPairSourceLanes.at(movement.movementID);
        for (int index : movement.connectionIndices) {
            const auto& connection = sumoConnectionsRaw.at(index);
            if (connection.fromLane < 0 || connection.fromLane >= static_cast<int>(roadPairLaneQueues.at(movement.fromRoadID).size()) ||
                connection.toLane < 0 || connection.toLane >= static_cast<int>(roadPairLaneQueues.at(movement.toRoadID).size()))
                throw runtime_error("Road-pair connection has an invalid physical lane index");
            sourceLanes.push_back(connection.fromLane);
            if (connection.via.empty()) {
                roadPairInternalPathComplete[index] = true;
            } else {
                string via = connection.via;
                set<string> visited;
                auto& permissions = roadPairInternalPathPermissions[index];
                while (!via.empty()) {
                    const auto lane = internalLanes.find(via);
                    if (lane == internalLanes.end() || duplicateInternalLanes.count(via) || !visited.insert(via).second) break;
                    permissions.push_back({lane->second.lane->allow,lane->second.lane->disallow});
                    const auto next = continuations.find({lane->second.edge->id,lane->second.lane->index});
                    if (next == continuations.end() || next->second.size() != 1) break;
                    const auto& continuation = sumoConnectionsRaw[next->second.front()];
                    if (continuation.toEdge != connection.toEdge || continuation.toLane != connection.toLane) break;
                    permissions.push_back({continuation.allow,continuation.disallow});
                    if (continuation.via.empty()) { roadPairInternalPathComplete[index] = true; break; }
                    via = continuation.via;
                }
            }
        }
        sort(sourceLanes.begin(), sourceLanes.end());
        sourceLanes.erase(unique(sourceLanes.begin(), sourceLanes.end()), sourceLanes.end());
        for (int lane : sourceLanes) roadPairLaneSubscribers[movement.fromRoadID][lane].push_back(movement.movementID);
    }
    roadPairRuntimeInitialized = true;
}

bool Graph::road_pair_lane_permitted(int roadID, int lane, const string& vehicleClass) const {
    if (roadID < 0 || roadID >= static_cast<int>(roadPairLaneMetadata.size()) || lane < 0 ||
        lane >= static_cast<int>(roadPairLaneMetadata[roadID].size())) return false;
    const auto& metadata = roadPairLaneMetadata[roadID][lane];
    return metadata.index == lane && class_permitted(metadata.allow, metadata.disallow, vehicleClass);
}

bool Graph::road_pair_connection_permitted(int movementID, int connectionIndex, const string& vehicleClass) const {
    if (connectionIndex < 0 || connectionIndex >= static_cast<int>(sumoConnectionsRaw.size())) return false;
    const auto& m = movements.at(movementID);
    const auto& c = sumoConnectionsRaw[connectionIndex];
    if (connectionIndex >= static_cast<int>(roadPairInternalPathComplete.size()) || !roadPairInternalPathComplete[connectionIndex]) return false;
    for (const auto& permission : roadPairInternalPathPermissions[connectionIndex]) {
        if (!class_permitted(permission.first, permission.second, vehicleClass)) return false;
    }
    return class_permitted(c.allow, c.disallow, vehicleClass) &&
        road_pair_lane_permitted(m.fromRoadID, c.fromLane, vehicleClass) &&
        road_pair_lane_permitted(m.toRoadID, c.toLane, vehicleClass);
}

const vector<int>& Graph::road_pair_entry_lanes(int movementID, int roadID, int vehicleID) const {
    const auto& vehicleClass = getVehicleTypeForVehicle(vehicleID).vehicleClass;
    const auto key = make_tuple(movementID, roadID, vehicleClass);
    const auto cached = roadPairPermittedEntryCache.find(key);
    if (cached != roadPairPermittedEntryCache.end()) return cached->second;
    vector<int> result;
    if (movementID >= 0) {
        for (int index : movements.at(movementID).connectionIndices) {
            if (road_pair_connection_permitted(movementID, index, vehicleClass))
                result.push_back(sumoConnectionsRaw[index].fromLane);
        }
    } else {
        for (int lane = 0; lane < static_cast<int>(roadPairLaneMetadata.at(roadID).size()); ++lane)
            if (road_pair_lane_permitted(roadID, lane, vehicleClass)) result.push_back(lane);
    }
    sort(result.begin(), result.end());
    result.erase(unique(result.begin(), result.end()), result.end());
    return roadPairPermittedEntryCache.emplace(key, std::move(result)).first->second;
}

const vector<RoadPairLaneOption>& Graph::road_pair_lane_options(int movementID, int vehicleID) {
    const auto& v = vehicles.at(vehicleID);
    const auto& m = movements.at(movementID);
    const int nextIndex = v.roadIndex + 1;
    const int nextMovement = nextIndex >= 0 && nextIndex < static_cast<int>(v.routeMovementIDs.size())
        ? v.routeMovementIDs[nextIndex] : -1;
    const string& vehicleClass = getVehicleTypeForVehicle(vehicleID).vehicleClass;
    const auto key = make_tuple(movementID, nextMovement, v.occupiedLaneIndex, vehicleClass, laneEntryReselection);
    const auto found = roadPairOptionCache.find(key);
    if (found != roadPairOptionCache.end()) return found->second;
    vector<int> connections;
    vector<int> entryLanes;
    for (int index : m.connectionIndices) {
        const auto& c = sumoConnectionsRaw.at(index);
        if (c.fromLane == v.occupiedLaneIndex && road_pair_connection_permitted(movementID, index, vehicleClass)) {
            connections.push_back(index);
            entryLanes.push_back(c.toLane);
        }
    }
    vector<int> nextLanes;
    if (nextMovement >= 0) nextLanes = road_pair_entry_lanes(nextMovement, m.toRoadID, vehicleID);
    const auto common = laneIntersection(entryLanes, nextLanes);
    vector<RoadPairLaneOption> options;
    for (int index : connections) {
        const int entry = sumoConnectionsRaw[index].toLane;
        if (nextMovement < 0) {
            options.push_back({index, entry, entry});
        } else if (laneEntryReselection) {
            // One coarse-grained reassignment early in the receiving road.
            // Preserve every actual entry connection: entry availability is
            // dynamic, so no single entry may be cached for a logical lane.
            // This is a K x L static plan, not a fabricated junction link or
            // a model of lateral gaps/duration. Only the final lane is stored.
            for (int logical : nextLanes) options.push_back({index, entry, logical});
        } else if (!common.empty()) {
            if (find(common.begin(), common.end(), entry) != common.end()) options.push_back({index, entry, entry});
        } else {
            // Explicit compatibility approximation inherited from borrowing=off:
            // real entry remains the original connection.toLane, whereas the
            // logical AND storage lane project onto the next movement's lanes.
            // This is not a fabricated source->destination connection, a lane
            // change model, or storage recorded on the real entry lane.
            for (int logical : nextLanes) options.push_back({index, entry, logical});
        }
    }
    return roadPairOptionCache.emplace(key, std::move(options)).first->second;
}

bool Graph::road_pair_option_has_capacity(int movementID, int vehicleID, const RoadPairLaneOption& option) const {
    const int roadID = movements.at(movementID).toRoadID;
    if (!hasLaneStorageCapacity(roadID, option.logicalLane, vehicleID)) return false;
    // Entry is an admission check, not a second occupancy reservation. The
    // current count-capacity rule is only a proxy for a physical entry gap.
    // Final-road options keep entry==logical, hence retain their old check.
    return !laneEntryReselection || option.entryLane == option.logicalLane ||
        hasLaneStorageCapacity(roadID, option.entryLane, vehicleID);
}

bool Graph::road_pair_select_lane(int movementID, int vehicleID, RoadPairChoice& choice) {
    const auto& m = movements.at(movementID);
    const auto& road = roads.at(m.toRoadID);
    const auto& options = road_pair_lane_options(movementID, vehicleID);
    const RoadPairLaneOption* best = nullptr;
    for (const auto& option : options) {
        if (!road_pair_option_has_capacity(movementID, vehicleID, option)) continue;
        if (!best) {
            best = &option;
        } else if (laneEntryReselection) {
            if (make_tuple(road.laneFlow[option.logicalLane], road.laneOccupiedLength[option.logicalLane],
                           abs(option.entryLane - option.logicalLane), option.logicalLane, option.connectionIndex) <
                make_tuple(road.laneFlow[best->logicalLane], road.laneOccupiedLength[best->logicalLane],
                           abs(best->entryLane - best->logicalLane), best->logicalLane, best->connectionIndex)) best = &option;
        } else {
            // Keep the disabled policy's ordering byte-for-byte equivalent.
            if (make_tuple(road.laneFlow[option.logicalLane], road.laneOccupiedLength[option.logicalLane], option.logicalLane, option.connectionIndex) <
                make_tuple(road.laneFlow[best->logicalLane], road.laneOccupiedLength[best->logicalLane], best->logicalLane, best->connectionIndex)) best = &option;
        }
    }
    if (!best) return false;
    choice.vehicleID = vehicleID;
    choice.serviceLane = vehicles[vehicleID].occupiedLaneIndex;
    choice.routeIndex = vehicles[vehicleID].roadIndex;
    choice.connectionIndex = best->connectionIndex;
    choice.entryLane = best->entryLane;
    choice.logicalLane = best->logicalLane;
    return true;
}

bool Graph::road_pair_is_head(int movementID, int vehicleID) const {
    if (vehicleID < 0 || vehicleID >= static_cast<int>(vehicles.size())) return false;
    const auto& v = vehicles[vehicleID];
    if (!v.valid || v.finished || v.currentMovementID != movementID || v.currentBufferID != getMovementBufferID(movementID)) return false;
    const int road = v.occupiedRoadID, lane = v.occupiedLaneIndex;
    if (road != movements[movementID].fromRoadID || road < 0 || road >= static_cast<int>(roadPairLaneQueues.size()) ||
        lane < 0 || lane >= static_cast<int>(roadPairLaneQueues[road].size())) return false;
    const auto& queue = roadPairLaneQueues[road][lane];
    return !queue.empty() && queue.front() == vehicleID;
}

int Graph::road_pair_front(int movementID) const {
    if (movementID < 0 || movementID >= static_cast<int>(roadPairChoices.size())) return -1;
    const int selected = roadPairChoices[movementID].vehicleID;
    if (road_pair_is_head(movementID, selected)) return selected;
    int first = -1;
    const int road = movements[movementID].fromRoadID;
    for (int lane : roadPairSourceLanes[movementID]) {
        const auto& queue = roadPairLaneQueues[road][lane];
        if (queue.empty() || !road_pair_is_head(movementID, queue.front())) continue;
        const int id = queue.front();
        if (first < 0 || make_pair(vehicles[id].arrivalTime, id) < make_pair(vehicles[first].arrivalTime, first)) first = id;
    }
    return first;
}

RoadPairChoice Graph::road_pair_compute_choice(int movementID, double lowerBound) {
    RoadPairChoice best;
    const auto& movement = movements.at(movementID);
    for (int lane : roadPairSourceLanes.at(movementID)) {
        ++roadPairCandidateLaneVisits;
        const auto& queue = roadPairLaneQueues[movement.fromRoadID][lane];
        if (queue.empty() || !road_pair_is_head(movementID, queue.front())) continue;
        const int vehicleID = queue.front();
        const auto& v = vehicles[vehicleID];
        if (v.roadIndex < 0 || v.roadIndex + 1 >= static_cast<int>(v.routeRoadIDs.size()) ||
            v.routeRoadIDs[v.roadIndex] != movement.fromRoadID || v.routeRoadIDs[v.roadIndex + 1] != movement.toRoadID)
            throw runtime_error("Road-pair lane head disagrees with its route occurrence");
        RoadPairChoice candidate;
        if (!road_pair_select_lane(movementID, vehicleID, candidate)) continue;
        double time = max(max(lowerBound, v.arrivalTime), roadPairLaneNextAvailable[movement.fromRoadID][lane]);
        time = nextAvailableCapacityTime(movementID, time); // retain the original movement pool bound
        candidate.time = nextGreenTimeForMovement(movementID, time);
        if (candidate.time >= INF) continue;
        if (best.vehicleID < 0 || make_tuple(candidate.time, v.arrivalTime, vehicleID, lane) <
            make_tuple(best.time, vehicles[best.vehicleID].arrivalTime, best.vehicleID, best.serviceLane)) best = candidate;
    }
    return best;
}

void Graph::road_pair_schedule(int movementID, double lowerBound) {
    if (movementID < 0 || movementID >= static_cast<int>(movements.size())) return;
    ensure_dispatch_state();
    if (!roadPairRuntimeInitialized) throw runtime_error("Road-pair runtime was not initialized");
    if (!std::isfinite(lowerBound)) throw runtime_error("Non-finite road-pair candidate time");
    RoadPairChoice choice = road_pair_compute_choice(movementID, lowerBound);
    roadPairChoices[movementID] = choice;
    movementBlockedByDownstream[movementID] = false;
    if (choice.vehicleID < 0) {
        // Blocked means no independently usable lane head, not WB.front full.
        movementBlockedByDownstream[movementID] = road_pair_front(movementID) >= 0;
        invalidate_dispatch_candidate(movementID);
        return;
    }
    const auto& previous = latestDispatchCandidates[movementID];
    const auto& v = vehicles[choice.vehicleID];
    if (movementInDispatchPQ[movementID] && previous.version == movementPQVersion[movementID] &&
        previous.frontVehicleID == choice.vehicleID && previous.timeLabel == choice.time &&
        previous.firstCarArriveTime == v.arrivalTime && previous.connectionIndex == choice.connectionIndex &&
        previous.storageLane == choice.logicalLane && previous.routeIndex == choice.routeIndex) {
        ++dispatchQueueStats.identicalSkips;
        return;
    }
    DispatchCandidate candidate{choice.time, v.arrivalTime, movementID, choice.vehicleID, next_dispatch_version(movementID)};
    candidate.serviceLane = choice.serviceLane;
    candidate.connectionIndex = choice.connectionIndex;
    candidate.entryLane = choice.entryLane;
    candidate.storageLane = choice.logicalLane;
    candidate.routeIndex = choice.routeIndex;
    movementTimeLabel[movementID] = choice.time;
    latestDispatchCandidates[movementID] = candidate;
    dispatchPQ.push(candidate);
    movementInDispatchPQ[movementID] = true;
    ++dispatchQueueStats.pushes;
    dispatchQueueStats.peakHeap = max(dispatchQueueStats.peakHeap, dispatchPQ.size());
    if (persistentDispatch && dispatchActivePosition[movementID] < 0) {
        dispatchActivePosition[movementID] = static_cast<int>(activeDispatchMovements.size());
        activeDispatchMovements.push_back(movementID);
    }
    dispatchQueueStats.peakActive = max(dispatchQueueStats.peakActive, activeDispatchMovements.size());
}

void Graph::road_pair_refresh_lane(int roadID, int lane, double currentTime) {
    if (roadID < 0 || roadID >= static_cast<int>(roadPairLaneSubscribers.size()) || lane < 0 ||
        lane >= static_cast<int>(roadPairLaneSubscribers[roadID].size())) return;
    for (int movementID : roadPairLaneSubscribers[roadID][lane]) road_pair_schedule(movementID, currentTime);
}

void Graph::road_pair_insert_lane(int vehicleID) {
    const auto& v = vehicles.at(vehicleID);
    auto& queue = roadPairLaneQueues.at(v.occupiedRoadID).at(v.occupiedLaneIndex);
    const int previous = queue.empty() ? -1 : queue.front();
    if (find(queue.begin(), queue.end(), vehicleID) != queue.end())
        throw runtime_error("Duplicate road-pair lane queue membership");
    auto position = queue.end();
    if (movementQueueOrder == MovementQueueOrder::ArrivalTime) {
        for (auto it = queue.begin(); it != queue.end(); ++it) {
            if (vehicles[*it].arrivalTime > v.arrivalTime) { position = it; break; }
        }
    }
    queue.insert(position, vehicleID);
    if (previous >= 0 && previous != queue.front()) invalidate_dispatch_candidate(vehicles[previous].currentMovementID);
}

void Graph::road_pair_pop_and_consume(int movementID, int vehicleID, double time) {
    if (!road_pair_is_head(movementID, vehicleID)) throw runtime_error("Road-pair commit lost its lane head");
    const auto& v = vehicles[vehicleID];
    double& nextAvailable = roadPairLaneNextAvailable.at(v.occupiedRoadID).at(v.occupiedLaneIndex);
    if (nextAvailable > time + TIME_EPSILON) throw runtime_error("Road-pair source lane headway consumed early");
    auto& buffer = waitingBuffers.at(v.currentBufferID).vehicleQueue;
    const auto found = find(buffer.begin(), buffer.end(), vehicleID);
    if (found == buffer.end()) throw runtime_error("Road-pair lane index lost unique waiting-buffer membership");
    buffer.erase(found);
    roadPairLaneQueues[v.occupiedRoadID][v.occupiedLaneIndex].pop_front();
    nextAvailable = time + defaultDischargeInterval;
}

bool Graph::road_pair_validate_runtime(bool requireEmpty) const {
    vector<int> laneMembership(vehicles.size(), 0), bufferMembership(vehicles.size(), 0);
    size_t laneQueued = 0, bufferQueued = 0;
    for (size_t road = 0; road < roadPairLaneQueues.size(); ++road) {
        for (size_t lane = 0; lane < roadPairLaneQueues[road].size(); ++lane) {
            if (!std::isfinite(roadPairLaneNextAvailable[road][lane])) return false;
            for (int id : roadPairLaneQueues[road][lane]) {
                if (id < 0 || id >= static_cast<int>(vehicles.size()) || ++laneMembership[id] != 1) return false;
                const auto& v = vehicles[id];
                if (v.finished || !v.valid || v.occupiedRoadID != static_cast<int>(road) ||
                    v.occupiedLaneIndex != static_cast<int>(lane) || v.storageLaneIndex != static_cast<int>(lane)) return false;
                ++laneQueued;
            }
        }
        for (size_t lane = 0; lane < roads[road].laneFlow.size(); ++lane) {
            if (roads[road].laneFlow[lane] < 0 || roads[road].laneFlow[lane] > roads[road].laneCapacity[lane] ||
                !std::isfinite(roads[road].laneOccupiedLength[lane]) || roads[road].laneOccupiedLength[lane] < -TIME_EPSILON)
                return false;
            if (requireEmpty && roads[road].laneFlow[lane] != 0) return false;
        }
    }
    for (const auto& buffer : waitingBuffers) {
        for (int id : buffer.vehicleQueue) {
            if (id < 0 || id >= static_cast<int>(vehicles.size()) || ++bufferMembership[id] != 1 ||
                vehicles[id].currentBufferID != buffer.bufferID || vehicles[id].currentMovementID != buffer.movementID) return false;
            ++bufferQueued;
        }
    }
    if (laneMembership != bufferMembership || laneQueued != bufferQueued || (requireEmpty && laneQueued != 0)) return false;
    uint64_t activeOccupancy = 0;
    for (const auto& vehicle : vehicles) if (vehicle.occupiedRoadID >= 0) ++activeOccupancy;
    if (roadPairReleases > roadPairReservations || roadPairReservations - roadPairReleases != activeOccupancy ||
        (requireEmpty && roadPairReservations != roadPairReleases)) return false;
    for (const auto& traces : vehicleMovementTimeline) for (const auto& trace : traces) {
        if (trace.connectionIndex < 0 || trace.connectionIndex >= static_cast<int>(sumoConnectionsRaw.size())) return false;
        const auto& connection = sumoConnectionsRaw[trace.connectionIndex];
        const auto& movement = movements.at(trace.movementID);
        if (connection.fromLane != trace.serviceLane || connection.toLane != trace.entryLane ||
            trace.storageLane != trace.chosenLane || find(movement.connectionIndices.begin(), movement.connectionIndices.end(), trace.connectionIndex) == movement.connectionIndices.end()) return false;
    }
    return true;
}
