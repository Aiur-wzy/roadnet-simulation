#ifndef ROADNET_NATIVE_ROAD_TIME_TABLE_H
#define ROADNET_NATIVE_ROAD_TIME_TABLE_H

#include "signal_rg.h"
#include <cstdint>
#include <istream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

struct NativeDemandKey {
    std::uint64_t group;
    std::uint32_t road;
    bool operator==(const NativeDemandKey& other) const {
        return group == other.group && road == other.road;
    }
};

struct NativeDemandKeyHash {
    std::size_t operator()(const NativeDemandKey& key) const {
        const auto first = std::hash<std::uint64_t>()(key.group);
        return first ^ (std::hash<std::uint32_t>()(key.road) + 0x9e3779b9U + (first << 6) + (first >> 2));
    }
};

// Independent native-trained sparse format. Training uses actual SUMO snapshot
// features; deployment deliberately retains the existing C++ entry proxies.
// This reader does not assert their microscopic equivalence or alter dynamics.
class NativeRoadTimeTable {
public:
    // Input starts AFTER CAMS_NATIVE_ABC_RG_DEMAND_V2 and its newline.
    void load(std::istream& input);
    void clear();
    bool lookupDemand(double road_length, int road_flow, int lane_flow, double raw_cap,
                      int context, const SignalRGFeatures& signal, double& value,
                      std::uint64_t& group, std::uint32_t& representative) const;
    SignalRGFeatures quantizeSignal(const SignalRGFeatures& signal) const;
    void requireCrossingTime(double seconds) const;
    void requireDeparturePositionCorrection(bool enabled) const;
    bool clampsCap() const { return clamp_cap_; }
    void requireSignalRemainingStep(double seconds) const;
    void requireUnsignalizedContract() const;
    void requireNoInternalLinksContract() const;
    void requireNetworkFile(const std::string& path) const;
    double overdueThreshold() const { return overdue_threshold_; }
    double signalRemainingStep() const { return signal_step_; }
    std::size_t signalPointCount() const { return signal_points_; }
    std::uint64_t size() const { return expanded_rows_; }
    std::uint64_t groupCount() const { return group_count_; }
    std::size_t compressedBytes() const { return payload_.size(); }
    std::size_t demandEntryCount() const { return entries_; }
    const std::string& demandScope() const { return scope_; }
    const std::string& label() const { return label_; }

private:
    struct SignalGroup {
        char first, second;
        double next_duration, maximum;
        bool sentinel;
        std::size_t begin;
        std::vector<double> nodes;
    };
    void load_impl(std::istream& input);
    std::size_t signal_point(const SignalRGFeatures& raw, SignalRGFeatures* quantized) const;
    std::uint32_t representative_road_flow(int road, int lane) const;
    std::uint64_t query_group(double length, int road, int lane, double cap,
                              int context, const SignalRGFeatures& signal) const;
    bool loaded_ = false;
    bool departure_position_correction_ = true, clamp_cap_ = false;
    std::vector<unsigned char> payload_;
    std::unordered_map<std::int64_t, std::size_t> lengths_;
    std::vector<SignalGroup> signals_;
    std::map<std::tuple<char, char, std::int64_t>, std::size_t> signal_groups_;
    std::vector<int> road_nodes_;
    std::uint64_t expanded_rows_ = 0, group_count_ = 0, road_step_ = 1, dense_until_ = 10;
    std::size_t entries_ = 0, signal_points_ = 0, cap_count_ = 0;
    int road_max_ = 0, lane_max_ = 0, cap_index_min_ = 0, cap_index_max_ = 0;
    double cap_step_ = 0, cap_min_ = 0, cap_max_ = 0, signal_step_ = 0;
    double crossing_time_ = 0, overdue_threshold_ = 0;
    std::string label_, scope_, network_sha_;
};

#endif
