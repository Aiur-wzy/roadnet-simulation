#ifndef ROADNET_CAP_CONTEXT_ABC_RG_TABLE_H
#define ROADNET_CAP_CONTEXT_ABC_RG_TABLE_H

#include "signal_rg.h"

#include <cstddef>
#include <cstdint>
#include <istream>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

// Independently versioned ABC + R/G table. Keep exact RLE bytes resident and
// search runs directly; size() is the logical expanded size, not RAM use.
class CapContextABCRGTable {
public:
    // Input begins immediately AFTER CAMS_CAP_CONTEXT_ABC_RG_RLE_V1 newline.
    // Every failure clears the object; no fallback to an older table format.
    void load(std::istream& input);
    // Input begins AFTER CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1 newline. The
    // catalog/domain is complete; only explicitly stored representative keys hit.
    void loadDemand(std::istream& input);
    bool isDemand() const { return demand_mode_; }
    std::size_t demandEntryCount() const { return demand_entry_count_; }
    const std::string& demandScope() const { return demand_scope_; }
    // Invalid inputs throw. A legal missing key returns false with group/road
    // filled, and leaves value unchanged; this class never supplies a fallback.
    bool lookupDemand(double road_length, int road_flow, int lane_flow,
                      double raw_cap, int context, const SignalRGFeatures& signal,
                      double& value, std::uint32_t& group, std::uint32_t& representative) const;
    void clear();
    double lookup(double road_length, int road_flow, int lane_flow,
                  double raw_cap, int context, const SignalRGFeatures& signal) const;
    SignalRGFeatures quantizeSignal(const SignalRGFeatures& signal) const;
    void requireCrossingTime(double seconds) const;
    // Expectation only: the table header remains the source of the node grid.
    void requireSignalRemainingStep(double expected) const;
    // Legacy G0G0-no-TLS models must not accidentally hit a positive GG key.
    void requireUnsignalizedContract() const;
    // A pre-v4 table is compatible only if its trained crossing was already zero.
    void requireNoInternalLinksContract() const;
    double signalRemainingStep() const { return signal_remaining_step_; }
    std::size_t signalPointCount() const { return signal_point_count_; }
    double overdueThreshold() const { return overdue_threshold_; }
    std::uint64_t size() const { return expanded_rows_; }
    const std::string& label() const { return label_; }
    std::size_t compressedBytes() const { return payload_.size(); }

private:
    struct SignalGroup {
        char entry_type = 0;
        char next_type = 0;
        double next_duration = 0;
        double entry_max = 0;
        bool sentinel = false;
        std::vector<double> nodes;
        std::size_t point_begin = 0;
    };
    void load_impl(std::istream& input, bool demand = false);
    std::size_t signal_point(const SignalRGFeatures& raw, SignalRGFeatures* quantized) const;
    std::uint32_t query_group(double road_length, int road_flow, int lane_flow,
                              double raw_cap, int context, const SignalRGFeatures& signal) const;
    std::uint32_t representative_road_flow(int road_flow, int lane_flow) const;

    std::vector<unsigned char> payload_;
    std::unordered_map<std::int64_t, std::size_t> lengths_;
    std::vector<SignalGroup> signals_;
    std::map<std::tuple<char, char, std::int64_t>, std::size_t> signal_groups_;
    std::string label_;
    bool demand_mode_ = false;
    bool constant_green_contract_ = false;
    bool no_internal_links_contract_ = false;
    std::size_t demand_entry_count_ = 0;
    std::string demand_scope_;
    std::uint64_t road_flow_step_ = 1, road_flow_dense_until_ = 10;
    // Empty means the legacy uniform/prefix policy. Explicit nodes include
    // both global endpoints; each lane also keeps its own lower bound.
    std::vector<int> road_flow_nodes_;
    std::uint64_t expanded_rows_ = 0;
    std::size_t groups_ = 0, run_count_ = 0, records_start_ = 0;
    std::size_t cap_count_ = 0, signal_point_count_ = 0;
    int road_flow_max_ = 0, lane_flow_max_ = 0, cap_index_min_ = 0, cap_index_max_ = 0;
    double cap_step_ = 0, cap_min_ = 0, cap_max_ = 0, crossing_time_ = 0;
    double overdue_threshold_ = 0, signal_remaining_step_ = 0;
};

#endif
