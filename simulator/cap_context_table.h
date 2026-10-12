#ifndef CAMS_CAP_CONTEXT_TABLE_H
#define CAMS_CAP_CONTEXT_TABLE_H

#include <cstddef>
#include <cstdint>
#include <istream>
#include <string>
#include <unordered_map>
#include <vector>

class CapContextTable {
public:
    // Input starts immediately after the magic line. Failure empties the object.
    void load(std::istream& input, bool abc = false);
    void clear();
    double lookup(double road_length, int road_flow, int lane_flow,
                  double raw_cap, int context) const;
    void requireCrossingTime(double seconds) const;
    std::size_t size() const { return values_.size(); }
    const std::string& label() const { return label_; }
    const std::vector<double>& denseValues() const { return values_; }
    bool isABC() const { return abc_; }
    int contextCount() const { return context_count_; }
    double overdueThreshold() const { return overdue_threshold_; }
private:
    void load_impl(std::istream& input, bool abc);
    std::vector<double> values_;
    std::unordered_map<std::int64_t, std::size_t> lengths_;
    std::string label_;
    bool abc_ = false;
    int context_count_ = 0;
    int road_flow_max_ = 0, lane_flow_max_ = 0;
    int cap_index_min_ = 0, cap_index_max_ = 0;
    std::size_t flow_count_ = 0, cap_count_ = 0;
    double cap_step_ = 0, cap_min_ = 0, cap_max_ = 0, crossing_time_ = 0;
    double overdue_threshold_ = 0;
};
#endif
