#ifndef SF_TRAVEL_TIME_TABLE_H
#define SF_TRAVEL_TIME_TABLE_H

#include <cstddef>
#include <cstdint>
#include <istream>
#include <map>
#include <string>
#include <utility>
#include <vector>

// Versioned five-feature external-road table. SF is already an input to the
// prediction; callers must not divide the returned time by SF a second time.
class SfTravelTimeTable {
public:
    void clear();
    // The caller has consumed the CAMS_SF_TABLE_V1 magic line. Any failure
    // leaves this object empty, including a failure while replacing a table.
    void load(std::istream& input);
    std::size_t size() const { return values_.size(); }
    const std::string& label() const { return label_; }
    double lookup(double roadLength, int roadFlow, int laneFlow,
                  double speedFactor, double speedLimit) const;
    int roadFlowMax() const { return road_flow_max_; }
    int laneFlowMax() const { return lane_flow_max_; }
    double sfMin() const { return sf_min_; }
    double sfMax() const { return sf_max_; }
    double trainingRoadFlowMin() const { return training_road_min_; }
    double trainingRoadFlowMax() const { return training_road_max_; }
    double trainingLaneFlowMin() const { return training_lane_min_; }
    double trainingLaneFlowMax() const { return training_lane_max_; }

private:
    using StaticKey = std::pair<std::int64_t, std::int64_t>;
    std::map<StaticKey, std::size_t> static_indices_;
    std::vector<double> values_;
    std::string label_;
    int road_flow_max_ = 0;
    int lane_flow_max_ = 0;
    int sf_scale_ = 0;
    int sf_index_min_ = 0;
    int sf_index_max_ = 0;
    std::size_t flow_count_ = 0;
    std::size_t sf_count_ = 0;
    double sf_min_ = 0;
    double sf_max_ = 0;
    double training_road_min_ = 0;
    double training_road_max_ = 0;
    double training_lane_min_ = 0;
    double training_lane_max_ = 0;

    std::size_t flow_offset(int roadFlow, int laneFlow) const;
    void load_checked(std::istream& input);
};

#endif
