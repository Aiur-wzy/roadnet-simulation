#include "cap_context.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
void require_positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0)
        throw std::runtime_error(std::string("F13 requires positive finite ") + name);
}
double finish_time(const CapContextRecord& record) {
    if (record.vehicle_id < 0 || record.route_index < 0 ||
        !std::isfinite(record.planned_enter_time) || record.planned_enter_time < 0)
        throw std::runtime_error("F13 invalid vehicle identity or planned enter time");
    require_positive(record.distance, "distance");
    require_positive(record.free_speed_cap, "free speed cap");
    const double horizon = record.distance / record.free_speed_cap;
    require_positive(horizon, "free travel horizon");
    const double finish = record.planned_enter_time + horizon;
    if (!std::isfinite(finish)) throw std::runtime_error("F13 free completion time overflow");
    return finish;
}
}

double cap_context_free_speed(double max_speed, double desired_max_speed,
                              double speed_factor, double speed_limit) {
    require_positive(max_speed, "maxSpeed");
    require_positive(desired_max_speed, "desiredMaxSpeed");
    require_positive(speed_factor, "speedFactor");
    require_positive(speed_limit, "speed limit");
    const double result = std::min(max_speed, std::min(speed_factor * desired_max_speed,
                                                     speed_factor * speed_limit));
    require_positive(result, "derived free speed cap");
    return result;
}

double cap_context_distance(double physical_length, bool first_road,
                            double base_front_position) {
    require_positive(physical_length, "physical road length");
    if (!first_road) return physical_length;
    if (!std::isfinite(base_front_position) || base_front_position < 0 ||
        base_front_position >= physical_length)
        throw std::runtime_error("F13 unsupported or invalid first-road base position");
    return physical_length - base_front_position;
}

CapContextResult scan_cap_context(const std::vector<CapContextRecord>& records,
                                 const CapContextRecord& ego, double decision_time) {
    const double ego_finish = finish_time(ego);
    if (!std::isfinite(decision_time) || decision_time < 0 ||
        decision_time > ego.planned_enter_time)
        throw std::runtime_error("F13 decision time must be finite and no later than planned entry");
    CapContextResult result;
    double sum = 0, compensation = 0;
    for (const auto& candidate : records) {
        if (candidate.vehicle_id == ego.vehicle_id && candidate.route_index == ego.route_index) continue;
        if (candidate.planned_enter_time > decision_time) { ++result.pending_count; continue; }
        const double gap = std::max(0.0, finish_time(candidate) - ego_finish);
        ++result.eligible_count;
        if (gap <= 0) continue;
        ++result.positive_count;
        result.maximum_gap = std::max(result.maximum_gap, gap);
        const double next = sum + gap;
        compensation += std::abs(sum) >= std::abs(gap) ? (sum - next) + gap : (gap - next) + sum;
        sum = next;
    }
    result.gap_sum = sum + compensation;
    if (!std::isfinite(result.gap_sum)) throw std::runtime_error("F13 gap sum overflow");
    if (result.maximum_gap <= CAP_CONTEXT_TIME_EPSILON) return result;
    result.potential = result.maximum_gap / (ego.distance / ego.free_speed_cap);
    result.equivalent_count = result.gap_sum / result.maximum_gap;
    result.context = result.potential < CAP_CONTEXT_P_THRESHOLD ? 1
        : result.equivalent_count < CAP_CONTEXT_E_THRESHOLD ? 2 : 3;
    return result;
}

CapContextResult scan_cap_context_abc(const std::vector<CapContextRecord>& records,
                                     const CapContextRecord& ego, double decision_time,
                                     double overdue_threshold_seconds) {
    if (!std::isfinite(overdue_threshold_seconds) || overdue_threshold_seconds < 0)
        throw std::runtime_error("F13 ABC overdue threshold must be finite and nonnegative");
    const double ego_finish = finish_time(ego);
    // Match the collection-side ABC decision boundary; this tolerance does
    // not activate candidate reservations before their exact entry time.
    if (!std::isfinite(decision_time) || decision_time < 0 ||
        ego.planned_enter_time < decision_time - CAP_CONTEXT_TIME_EPSILON)
        throw std::runtime_error("F13 ABC decision time must not exceed planned entry beyond epsilon");
    CapContextResult result;
    double sum = 0, compensation = 0;
    double overdue_sum = 0, overdue_compensation = 0;
    for (const auto& candidate : records) {
        if (candidate.vehicle_id == ego.vehicle_id && candidate.route_index == ego.route_index) continue;
        if (candidate.planned_enter_time > decision_time) { ++result.pending_count; continue; }
        const double candidate_finish = finish_time(candidate);
        const double gap = std::max(0.0, candidate_finish - ego_finish);
        ++result.eligible_count;
        if (gap > 0) ++result.positive_count;
        result.maximum_gap = std::max(result.maximum_gap, gap);
        const double next = sum + gap;
        compensation += std::abs(sum) >= std::abs(gap) ? (sum - next) + gap : (gap - next) + sum;
        sum = next;

        // Include every eligible record, also gap == 0. Such records supply
        // precisely the overdue information that distinguishes B from C.
        const double overdue = std::max(0.0, decision_time - candidate_finish);
        result.max_overdue_seconds = std::max(result.max_overdue_seconds, overdue);
        if (overdue > CAP_CONTEXT_TIME_EPSILON) ++result.overdue_count;
        const double next_overdue = overdue_sum + overdue;
        overdue_compensation += std::abs(overdue_sum) >= std::abs(overdue)
            ? (overdue_sum - next_overdue) + overdue : (overdue - next_overdue) + overdue_sum;
        overdue_sum = next_overdue;
    }
    result.gap_sum = sum + compensation;
    result.sum_overdue_seconds = overdue_sum + overdue_compensation;
    if (!std::isfinite(result.gap_sum) || !std::isfinite(result.sum_overdue_seconds))
        throw std::runtime_error("F13 ABC accumulated free-clock value overflow");
    if (result.maximum_gap > CAP_CONTEXT_TIME_EPSILON) {
        result.potential = result.maximum_gap / (ego.distance / ego.free_speed_cap);
        result.equivalent_count = result.gap_sum / result.maximum_gap;
        result.legacy_context = result.potential < CAP_CONTEXT_P_THRESHOLD ? 1
            : result.equivalent_count < CAP_CONTEXT_E_THRESHOLD ? 2 : 3;
    }
    result.context = result.legacy_context;
    if (result.legacy_context == 0 && result.eligible_count > 0)
        result.context = result.max_overdue_seconds <= overdue_threshold_seconds + CAP_CONTEXT_TIME_EPSILON ? 4 : 5;
    if (result.eligible_count > 0) {
        result.overdue_fraction = static_cast<double>(result.overdue_count) / result.eligible_count;
        result.mean_overdue_seconds = result.sum_overdue_seconds / result.eligible_count;
    }
    return result;
}

std::uint64_t CapContextLedger::key(int road_id, int lane_index) {
    if (road_id < 0 || lane_index < 0) throw std::runtime_error("F13 requires a resolved road/lane");
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(road_id)) << 32)
         | static_cast<std::uint32_t>(lane_index);
}
void CapContextLedger::clear() { lanes_.clear(); positions_.clear(); }
void CapContextLedger::reserve(int road_id, int lane_index, const CapContextRecord& record) {
    finish_time(record);
    const auto lane_key = key(road_id, lane_index);
    if (positions_.count(record.vehicle_id))
        throw std::runtime_error("F13 duplicate vehicle reservation without road release");
    auto& lane = lanes_[lane_key];
    positions_.emplace(record.vehicle_id, Position{lane_key, lane.size(), record.route_index});
    lane.push_back(record);
}
void CapContextLedger::erase(int vehicle_id, int route_index) {
    const auto position = positions_.find(vehicle_id);
    if (position == positions_.end()) return;
    if (position->second.route_index != route_index)
        throw std::runtime_error("F13 stale route identity on release");
    const auto lane_it = lanes_.find(position->second.lane_key);
    auto& lane = lane_it->second;
    const std::size_t index = position->second.index;
    if (index + 1 != lane.size()) {
        lane[index] = lane.back();
        positions_.at(lane[index].vehicle_id).index = index;
    }
    lane.pop_back();
    positions_.erase(position);
    if (lane.empty()) lanes_.erase(lane_it);
}
CapContextResult CapContextLedger::query(int road_id, int lane_index,
                                        const CapContextRecord& ego, double decision_time) const {
    const auto lane = lanes_.find(key(road_id, lane_index));
    static const std::vector<CapContextRecord> empty;
    return scan_cap_context(lane == lanes_.end() ? empty : lane->second, ego, decision_time);
}
CapContextResult CapContextLedger::queryABC(int road_id, int lane_index,
                                           const CapContextRecord& ego, double decision_time,
                                           double overdue_threshold_seconds) const {
    const auto lane = lanes_.find(key(road_id, lane_index));
    static const std::vector<CapContextRecord> empty;
    return scan_cap_context_abc(lane == lanes_.end() ? empty : lane->second, ego,
                                decision_time, overdue_threshold_seconds);
}
