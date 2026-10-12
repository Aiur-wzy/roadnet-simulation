#include "head.h"

namespace {
vector<double> staticSignalDurations(const SumoSignalProgram* program) {
    if (!program || program->type != "static" || program->phases.empty() || !std::isfinite(program->offset))
        throw runtime_error("R/G requires one finite sequential static program");
    vector<double> durations;
    for (size_t p = 0; p < program->phases.size(); ++p) {
        const auto& phase = program->phases[p];
        if (!phase.nextPhaseIndices.empty()) {
            std::istringstream input(phase.nextPhaseIndices);
            string token, extra;
            if (!(input >> token) || (input >> extra))
                throw runtime_error("R/G branching phase sequence is unsupported");
            size_t consumed = 0;
            const int successor = std::stoi(token, &consumed);
            if (consumed != token.size() || successor != static_cast<int>((p + 1) % program->phases.size()))
                throw runtime_error("R/G requires sequential static phases");
        }
        if (phase.state.size() != program->phases.front().state.size())
            throw runtime_error("R/G inconsistent movement signal widths");
        if (!std::isfinite(phase.duration) || phase.duration <= 0.0)
            throw runtime_error("R/G phase duration must be finite and positive");
        durations.push_back(phase.duration);
    }
    return durations;
}
}

double Graph::unsignalizedGreenHorizon() {
    if (std::isfinite(unsignalizedGreenHorizonCache)) return unsignalizedGreenHorizonCache;
    if (sumoHasSignalProgramSwitching)
        throw runtime_error("R/G horizon requires a fixed static signal program; WAUT unsupported");
    map<string, set<int>> activeLinks;
    for (const auto& connection : sumoConnectionsRaw) {
        if (connection.tl.empty() || !sumoEdgeStrToRoadID.count(connection.fromEdge) ||
            !sumoEdgeStrToRoadID.count(connection.toEdge)) continue;
        activeLinks[connection.tl].insert(connection.linkIndex);
    }
    map<string, vector<const SumoSignalProgram*>> programs;
    for (const auto& program : sumoSignalPrograms)
        if (activeLinks.count(program.tlID)) programs[program.tlID].push_back(&program);
    double horizon = 0.0;
    for (const auto& active : activeLinks) {
        const auto& candidates = programs[active.first];
        if (candidates.size() != 1)
            throw runtime_error("R/G horizon requires exactly one active static program per connected TLS");
        const auto* program = candidates.front();
        const auto durations = staticSignalDurations(program);
        for (int link : active.second) {
            string colors;
            for (const auto& phase : program->phases) {
                if (link < 0 || link >= static_cast<int>(phase.state.size()))
                    throw runtime_error("R/G horizon active linkIndex outside static program");
                colors += phase.state[link];
            }
            horizon = max(horizon, LongestFiniteGreenSegment(durations, colors));
        }
    }
    if (!std::isfinite(horizon) || horizon <= 0.0)
        throw runtime_error("Unsignalized R/G features require a finite positive green segment in an active network program; no fallback configured");
    unsignalizedGreenHorizonCache = horizon;
    return horizon;
}

// Feature-only static calendar. Never modifies signal controllers, capacity,
// queue order or the existing movement service rules. Unknown mapping is not
// equivalent to an uncontrolled exit. Each route/lane mapping is cached once.
SignalRGFeatures Graph::outgoingSignalRG(int roadID, int nextRoadID, int logicalLane,
                                        double entryTime, string* tls, int* link) {
    if (tls) tls->clear();
    if (link) *link = -1;
    if (!std::isfinite(entryTime)) throw runtime_error("R/G nonfinite planned entry time");
    if (roadID < 0 || roadID >= static_cast<int>(roads.size()) || logicalLane < 0 ||
        logicalLane >= roads[roadID].laneNum)
        throw runtime_error("R/G invalid current road/logical lane");
    if (sumoHasSignalProgramSwitching)
        throw runtime_error("R/G v1 requires a fixed static signal program; WAUT unsupported");
    if (nextRoadID == -1) return NoSignalRGFeatures(); // Known final route occurrence.
    if (nextRoadID < 0 || nextRoadID >= static_cast<int>(roads.size()))
        throw runtime_error("R/G invalid next route road");
    const auto key = make_tuple(roadID, nextRoadID, logicalLane);
    auto cached = outgoingSignalRGCache.find(key);
    if (cached == outgoingSignalRGCache.end()) {
        const auto from = roadIDToSumoEdgeStr.find(roadID);
        const auto to = roadIDToSumoEdgeStr.find(nextRoadID);
        if (from == roadIDToSumoEdgeStr.end() || to == roadIDToSumoEdgeStr.end())
            throw runtime_error("R/G requires SUMO external-road identities");
        vector<const SumoConnectionRaw*> connections;
        for (const auto& connection : sumoConnectionsRaw)
            if (connection.fromEdge == from->second && connection.toEdge == to->second &&
                connection.fromLane == logicalLane)
                connections.push_back(&connection);
        if (connections.empty())
            throw runtime_error("R/G missing outgoing route/logical-lane connection");
        const string control = connections.front()->tl;
        for (const auto* connection : connections)
            if (connection->tl != control)
                throw runtime_error("R/G ambiguous outgoing signal controllers");
        OutgoingSignalRGContext context;
        if (!control.empty()) {
            const SumoSignalProgram* selected = nullptr;
            for (const auto& program : sumoSignalPrograms) {
                if (program.tlID != control) continue;
                if (selected) throw runtime_error("R/G multiple programs for one TLS require explicit active-program support");
                selected = &program;
            }
            const vector<double> durations = staticSignalDurations(selected);
            string movementStates;
            for (const auto* connection : connections) {
                string candidate;
                for (const auto& phase : selected->phases) {
                    if (connection->linkIndex < 0 || connection->linkIndex >= static_cast<int>(phase.state.size()))
                        throw runtime_error("R/G outgoing linkIndex outside static program");
                    candidate += phase.state[connection->linkIndex];
                }
                if (!movementStates.empty() && movementStates != candidate)
                    throw runtime_error("R/G ambiguous complete outgoing movement calendars");
                movementStates = candidate;
            }
            context.program = std::make_shared<SignalRGProgram>(durations, movementStates);
            context.offset = selected->offset;
            context.tls = control;
            context.link = connections.front()->linkIndex;
        }
        cached = outgoingSignalRGCache.emplace(key, std::move(context)).first;
    }
    if (tls) *tls = cached->second.tls;
    if (link) *link = cached->second.link;
    if (!cached->second.program && usesRGCapContextTable())
        requireRGUnsignalizedContract();
    return cached->second.program
        ? cached->second.program->queryAtTime(entryTime, cached->second.offset)
        : AlwaysGreenSignalRGFeatures(unsignalizedGreenHorizon());
}
