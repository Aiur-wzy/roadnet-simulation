
#include "head.h"

// Check Why "BJ" had different edge number with "Minimal Travel Time"
void Graph::edge_num_check()
{
    // Variable Initialization
    int ID1, ID2;
    vector<int> IDs;
    // Each Node ID
    for (int i=0;i<graphLength.size();i++){
        ID1 = i;
        // Node ID's Neighbors
        for (int j=0;j<graphLength[i].size();j++){
            ID2 = graphLength[i][j].first;
            IDs.clear();
            // Store Neighbors ID from Minimal Travel Time Data in a List
            for (int k=0;k<graphTime[i].size();k++){
                IDs.push_back(graphTime[i][k].first);
            }
            // Check if Node Pairs Appear in Minimal Travel Time Data
            if (find(IDs.begin(), IDs.end(), ID2) == IDs.end()){
                cout << "ID1 " << ID1 << " ID2 " << ID2 << endl;
			}
        }
    }
}

void Graph::validate_sumo_network()
{
    int internalEdges = 0;
    int missingEndpoints = 0;
    int missingLanes = 0;
    int invalidLaneSpeed = 0;
    int invalidLaneLength = 0;
    int trafficLights = 0;
    int trafficLightsMissingProgram = 0;

    for (const auto &edge : sumoEdgesRaw) {
        if (edge.isInternal) {
            ++internalEdges;
            continue;
        }
        if (sumoNodeStrToID.find(edge.from) == sumoNodeStrToID.end() ||
            sumoNodeStrToID.find(edge.to) == sumoNodeStrToID.end()) {
            ++missingEndpoints;
        }
        if (edge.lanes.empty()) ++missingLanes;
        for (const auto &lane : edge.lanes) {
            if (lane.speed <= 0) ++invalidLaneSpeed;
            if (lane.length <= 0) ++invalidLaneLength;
        }
    }

    for (const auto &j : sumoJunctionsRaw) {
        if (j.type == "traffic_light") {
            ++trafficLights;
            if (tlIDToSignalProgramID.find(j.id) == tlIDToSignalProgramID.end()) {
                bool existsRaw = false;
                for (const auto &p : sumoSignalPrograms) {
                    if (p.tlID == j.id) {
                        existsRaw = true;
                        break;
                    }
                }
                if (!existsRaw) ++trafficLightsMissingProgram;
            }
        }
    }

    cout << "[SUMO Validation] total edges: " << sumoEdgesRaw.size() << endl;
    cout << "[SUMO Validation] normal edges: " << sumoEdgeStrToRoadID.size() << endl;
    cout << "[SUMO Validation] skipped internal edges: " << internalEdges << endl;
    cout << "[SUMO Validation] junctions: " << sumoJunctionsRaw.size() << endl;
    cout << "[SUMO Validation] traffic lights: " << trafficLights << endl;
    cout << "[SUMO Validation] edges with missing endpoints: " << missingEndpoints << endl;
    cout << "[SUMO Validation] edges with no lanes: " << missingLanes << endl;
    cout << "[SUMO Validation] lanes with speed <= 0: " << invalidLaneSpeed << endl;
    cout << "[SUMO Validation] lanes with length <= 0: " << invalidLaneLength << endl;
    cout << "[SUMO Validation] traffic_light junctions missing tlLogic: " << trafficLightsMissingProgram << endl;
    validate_sumo_connections();
    validate_sumo_signal_programs();
}

void Graph::validate_sumo_connections()
{
    int invalidEdgeRefs = 0;
    int invalidTLLinkIndex = 0;
    int linkIndexOutOfState = 0;
    for (const auto &c : sumoConnectionsRaw) {
        bool fromOk = c.fromEdge.empty() || c.fromEdge[0] == ':' || sumoEdgeStrToRoadID.find(c.fromEdge) != sumoEdgeStrToRoadID.end();
        bool toOk = c.toEdge.empty() || c.toEdge[0] == ':' || sumoEdgeStrToRoadID.find(c.toEdge) != sumoEdgeStrToRoadID.end();
        if (!fromOk || !toOk) ++invalidEdgeRefs;
        if (!c.tl.empty()) {
            if (c.linkIndex < 0) ++invalidTLLinkIndex;
            auto pIt = tlIDToSignalProgramID.find(c.tl);
            if (pIt != tlIDToSignalProgramID.end()) {
                const SignalProgram &p = signalPrograms[pIt->second];
                for (const auto &phase : p.phases) {
                    if (c.linkIndex < 0 || c.linkIndex >= static_cast<int>(phase.state.size())) {
                        ++linkIndexOutOfState;
                        break;
                    }
                }
            } else {
                bool existsRaw = false;
                for (const auto &p : sumoSignalPrograms) {
                    if (p.tlID == c.tl) {
                        existsRaw = true;
                        for (const auto &phase : p.phases) {
                            if (c.linkIndex < 0 || c.linkIndex >= static_cast<int>(phase.state.size())) {
                                ++linkIndexOutOfState;
                                break;
                            }
                        }
                        break;
                    }
                }
                if (!existsRaw) ++linkIndexOutOfState;
            }
        }
    }
    cout << "[SUMO Validation] connections: " << sumoConnectionsRaw.size() << endl;
    cout << "[SUMO Validation] movements: " << movements.size() << endl;
    cout << "[SUMO Validation] invalid connection edge refs: " << invalidEdgeRefs << endl;
    cout << "[SUMO Validation] tl connections with linkIndex < 0: " << invalidTLLinkIndex << endl;
    cout << "[SUMO Validation] tl connections with linkIndex outside phase.state: " << linkIndexOutOfState << endl;
}

void Graph::validate_sumo_signal_programs()
{
    int invalidPhaseDurations = 0;
    int invalidCycles = 0;
    size_t phaseCount = 0;
    const bool builtPrograms = !signalPrograms.empty();
    if (builtPrograms) {
        for (const auto &p : signalPrograms) {
            if (p.cycleLength <= 0) ++invalidCycles;
            phaseCount += p.phases.size();
            for (const auto &phase : p.phases) {
                if (phase.endTime <= phase.startTime) ++invalidPhaseDurations;
            }
        }
    } else {
        for (const auto &p : sumoSignalPrograms) {
            if (p.cycleLength <= 0) ++invalidCycles;
            phaseCount += p.phases.size();
            for (const auto &phase : p.phases) {
                if (phase.duration <= 0) ++invalidPhaseDurations;
            }
        }
    }
    cout << "[SUMO Validation] tlLogics: " << sumoSignalPrograms.size() << endl;
    cout << "[SUMO Validation] phases: " << phaseCount << endl;
    cout << "[SUMO Validation] invalid phase durations: " << invalidPhaseDurations << endl;
    cout << "[SUMO Validation] invalid cycle lengths: " << invalidCycles << endl;
}

void Graph::validate_sumo_routes()
{
    map<string, int> reasons;
    int invalidRoutes = 0;
    int missingMovements = 0;
    for (const auto &route : routeRoadID) {
        bool invalid = false;
        for (int roadID : route) {
            if (roadID < 0 || roadID >= static_cast<int>(roads.size())) {
                reasons["roadID outside roads vector"]++;
                invalid = true;
            } else {
                auto strIt = roadIDToSumoEdgeStr.find(roadID);
                if (strIt != roadIDToSumoEdgeStr.end() && !strIt->second.empty() && strIt->second[0] == ':') {
                    reasons["route includes internal edge"]++;
                    invalid = true;
                }
            }
        }
        for (int i = 0; i + 1 < static_cast<int>(route.size()); ++i) {
            bool hasMovement = fromToRoadToMovementIDs.find(make_pair(route[i], route[i + 1])) != fromToRoadToMovementIDs.end() ||
                               roadPairToMovementID.find(make_pair(route[i], route[i + 1])) != roadPairToMovementID.end();
            if (!hasMovement) {
                reasons["adjacent road pair has no movement"]++;
                ++missingMovements;
                invalid = true;
            }
        }
        if (invalid) ++invalidRoutes;
    }
    cout << "[SUMO Validation] route sequences: " << routeRoadID.size() << endl;
    cout << "[SUMO Validation] invalid routes: " << invalidRoutes << endl;
    cout << "[SUMO Validation] route missing movements: " << missingMovements << endl;
    for (const auto &kv : reasons) {
        cout << "[SUMO Validation] invalid route reason: " << kv.first << " count=" << kv.second << endl;
    }
}
