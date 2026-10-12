#ifndef CAMS_CAP_CONTEXT_H
#define CAMS_CAP_CONTEXT_H

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

// Frozen F13 v1 feature contract. These values must match the trained table.
constexpr double CAP_CONTEXT_P_THRESHOLD = 0.05;
constexpr double CAP_CONTEXT_E_THRESHOLD = 2.0;
constexpr double CAP_CONTEXT_TIME_EPSILON = 1e-9;

struct CapContextRecord {
    int vehicle_id = -1;
    int route_index = -1;
    double planned_enter_time = 0;
    double distance = 0;
    double free_speed_cap = 0;
};

struct CapContextResult {
    int context = 0;
    std::size_t eligible_count = 0;
    std::size_t pending_count = 0;
    std::size_t positive_count = 0;
    double maximum_gap = 0;
    double gap_sum = 0;
    double potential = 0;
    double equivalent_count = 0;
    // ABC v1 audit: only populated by the ABC query. These are logical
    // free-clock overruns, not measured physical waiting times.
    int legacy_context = 0;
    double max_overdue_seconds = 0;
    double sum_overdue_seconds = 0;
    std::size_t overdue_count = 0;
    double overdue_fraction = 0;
    double mean_overdue_seconds = 0;
};

double cap_context_free_speed(double max_speed, double desired_max_speed,
                              double speed_factor, double speed_limit);
double cap_context_distance(double physical_length, bool first_road,
                            double base_front_position);
CapContextResult scan_cap_context(const std::vector<CapContextRecord>& records,
                                 const CapContextRecord& ego, double decision_time);
CapContextResult scan_cap_context_abc(const std::vector<CapContextRecord>& records,
                                     const CapContextRecord& ego, double decision_time,
                                     double overdue_threshold_seconds = 0.0);

// Shadow state only: it neither changes logical flow nor schedules an event.
// Only one active/reserved route record per vehicle is allowed. Per-lane scans
// are O(n_lane); swap deletion and reservation are expected O(1).
class CapContextLedger {
public:
    void clear();
    void reserve(int road_id, int lane_index, const CapContextRecord& record);
    void erase(int vehicle_id, int route_index);
    CapContextResult query(int road_id, int lane_index,
                           const CapContextRecord& ego, double decision_time) const;
    CapContextResult queryABC(int road_id, int lane_index,
                             const CapContextRecord& ego, double decision_time,
                             double overdue_threshold_seconds = 0.0) const;
    std::size_t size() const { return positions_.size(); }
private:
    struct Position { std::uint64_t lane_key; std::size_t index; int route_index; };
    std::unordered_map<std::uint64_t, std::vector<CapContextRecord>> lanes_;
    std::unordered_map<int, Position> positions_;
    static std::uint64_t key(int road_id, int lane_index);
};
#endif
