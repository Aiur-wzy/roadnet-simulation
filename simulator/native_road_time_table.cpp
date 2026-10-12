#include "native_road_time_table.h"
#include "cap_context_json.h"
#include "cap_context_sha256.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

namespace {
using namespace cap_context_json;
constexpr std::uint64_t MAX_JSON = 9007199254740991ULL;
constexpr std::uint64_t MAX_PAYLOAD = 512ULL * 1024 * 1024;
constexpr std::size_t MAX_HEADER = 1024 * 1024;
constexpr double RAW_EPS = 1e-12, SCALED_EPS = 1e-9;

std::uint64_t integer64(const Json& object, const std::string& key,
                        std::uint64_t low = 0, std::uint64_t high = MAX_JSON) {
    const Json& node = object.at(key);
    const double value = number(node, key);
    if (!node.integer_token || value < low || value > high || value != std::floor(value))
        fail("native sparse integer outside exact domain: " + key);
    return static_cast<std::uint64_t>(value);
}

std::uint64_t multiply(std::uint64_t left, std::uint64_t right) {
    if (!left || !right || left > MAX_JSON / right)
        fail("native sparse virtual dimension overflow");
    return left * right;
}

void boolean(const Json& object, const std::string& key, bool expected) {
    const auto& node = object.at(key);
    if (node.kind != Json::Boolean || node.boolean != expected) fail("native sparse boolean mismatch: " + key);
}

std::string sha(const Json& object, const std::string& key) {
    const auto value = string_value(object.at(key), key);
    if (value.size() != 64 || value.find_first_not_of("0123456789abcdef") != std::string::npos)
        fail("native sparse invalid SHA256: " + key);
    return value;
}

std::uint64_t u64(const unsigned char* data) {
    std::uint64_t result = 0;
    for (unsigned i = 0; i < 8; ++i) result |= std::uint64_t(data[i]) << (8 * i);
    return result;
}

std::uint32_t u32(const unsigned char* data) {
    std::uint32_t result = 0;
    for (unsigned i = 0; i < 4; ++i) result |= std::uint32_t(data[i]) << (8 * i);
    return result;
}

double f64(const unsigned char* data) {
    const std::uint64_t bits = u64(data);
    double result;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
}

bool near(double a, double b) {
    return std::abs(a - b) <= 1e-10 * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

std::int64_t length_key(double value) {
    if (!std::isfinite(value) || value <= 0 || value * 10000 > 9e15)
        fail("native sparse invalid physical road length");
    return static_cast<std::int64_t>(std::floor(value * 10000 + 0.5 + SCALED_EPS));
}

std::int64_t duration_key(double value) {
    if (!std::isfinite(value) || value < 0 || !std::isfinite(value * 1e9) || value * 1e9 > 9e15)
        fail("native sparse invalid signal duration");
    return static_cast<std::int64_t>(std::floor(value * 1e9 + 0.5));
}

std::vector<double> remaining_nodes(double maximum, double step) {
    if (!std::isfinite(maximum) || maximum <= 0 || maximum / step > 99998)
        fail("native sparse invalid bounded signal grid");
    const double minimum = std::min(0.001, maximum);
    std::vector<double> nodes{minimum, maximum};
    for (std::size_t i = 1; i <= static_cast<std::size_t>(std::floor(maximum / step)); ++i) {
        const double value = i * step;
        if (minimum < value && value < maximum) nodes.push_back(value);
    }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    return nodes;
}

void strings(const Json& values, const std::vector<std::string>& expected) {
    if (values.kind != Json::Array || values.array.size() != expected.size())
        fail("native sparse feature/category order mismatch");
    for (std::size_t i = 0; i < expected.size(); ++i)
        if (string_value(values.array[i], "column") != expected[i]) fail("native sparse feature/category order mismatch");
}
} // namespace

void NativeRoadTimeTable::clear() { *this = NativeRoadTimeTable(); }

void NativeRoadTimeTable::load(std::istream& input) {
    clear();
    try { load_impl(input); loaded_ = true; }
    catch (...) { clear(); throw; }
}

void NativeRoadTimeTable::load_impl(std::istream& input) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "native table requires binary64");
    std::string line;
    char character;
    bool ended = false;
    while (input.get(character)) {
        if (character == '\n') { ended = true; break; }
        if (line.size() >= MAX_HEADER) fail("native sparse header exceeds 1MiB");
        line.push_back(character);
    }
    if (!ended) fail("native sparse missing header newline");
    const Json meta = JsonParser(line).parse();
    require_string(meta, "format", "CAMS_NATIVE_ABC_RG_DEMAND_V2");
    require_string(meta, "payload_layout", "sorted-u64-u32-f64-v2");
    require_string(meta, "training_schema", "road-time-native-model-v2");
    require_string(meta, "source_schema", "road-time-sumo-native-euler-streaming-quality-v2");
    strings(meta.at("columns"), {"road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context",
            "entry_signal_type", "entry_signal_remaining_seconds", "next_signal_type", "next_signal_duration_seconds", "road_time_seconds"});
    label_ = string_value(meta.at("source_label"), "source_label");
    const std::vector<std::string> labels{"travel_time_label_gt_0p1", "travel_time_label_gt_0p5", "travel_time_label_gt_1",
        "travel_time_label_gt_1p5", "travel_time_label_gt_2", "travel_time_label_gt_3"};
    if (std::find(labels.begin(), labels.end(), label_) == labels.end()) fail("native sparse unsupported label");
    const Json& deployment = meta.at("deployment_contract");
    const auto deployment_version = string_value(deployment.at("version"), "deployment version");
    if (deployment_version == "sumo-native-training-to-cpp-proxy-v3") {
        const auto correction = string_value(deployment.at("first_road_position_correction"), "first-road correction");
        const auto cap_policy = string_value(deployment.at("cap_out_of_range"), "cap out-of-range policy");
        if ((correction != "on" && correction != "off") || (cap_policy != "clamp" && cap_policy != "error"))
            fail("native sparse unsupported query policy");
        departure_position_correction_ = correction == "on";
        clamp_cap_ = cap_policy == "clamp";
    } else if (deployment_version != "sumo-native-training-to-cpp-proxy-v2") {
        fail("native sparse unsupported deployment version");
    }
    // Historical v2 tables retain strict cap bounds and corrected first-road
    // clocks. New policies require v3, so old binaries reject rather than ignore them.
    require_string(deployment, "training_inputs", "sumo-native-first-visible-snapshot");
    require_string(deployment, "query_inputs", "cpp-entry-decision-proxy");
    require_string(deployment, "abc_algorithm", "entry-frozen-free-clock-six-contexts");
    require_string(deployment, "crossing_policy", "first-or-known-unsignalized-or-no-internal-zero");
    boolean(deployment, "equivalent", false);
    crossing_time_ = number(deployment.at("crossing_time_seconds"), "deployment crossing time");
    overdue_threshold_ = number(deployment.at("overdue_threshold_seconds"), "deployment overdue threshold");
    if (crossing_time_ < 0 || overdue_threshold_ != 0) fail("native sparse unsupported deployment timing");
    const auto& model_contract = meta.at("model_contract");
    require_string(model_contract, "schema", "road-time-native-model-v2");
    require_string(model_contract, "status", "COMPLETE");
    require_string(model_contract, "source_schema", "road-time-sumo-native-euler-streaming-quality-v2");
    require_string(model_contract, "source_label_column", label_);
    require_string(model_contract, "model_label", "road_time_seconds");
    require_string(model_contract, "predictor_directory", "predictor");
    strings(model_contract.at("feature_columns"), {"road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context",
            "entry_signal_type", "entry_signal_remaining_seconds", "next_signal_type", "next_signal_duration_seconds"});
    require_string(model_contract.at("collection_contract"), "schema", "road-time-sumo-native-euler-streaming-quality-v2");
    require_string(model_contract.at("collection_contract"), "clock_schema", "sumo-euler-last-completed-snapshot-v2");
    sha(meta.at("model"), "contract_sha256");
    network_sha_ = sha(meta.at("network"), "sha256");
    integer64(meta.at("network"), "external_edges", 1);
    if (meta.at("network").at("without_internal_links").kind != Json::Boolean)
        fail("native sparse network topology flag must be boolean");
    integer64(meta, "context_count", 6, 6);
    number(meta, "static_key_scale", 10000.0);
    number(meta, "prediction_floor_seconds", 1.0);
    boolean(meta, "categorical_ordered", false);
    const Json& categorical = meta.at("categorical_features");
    strings(categorical.at("entry_signal_type"), {"R", "G"});
    strings(categorical.at("next_signal_type"), {"R", "G"});
    const Json& contexts = categorical.at("platoon_context");
    if (contexts.kind != Json::Array || contexts.array.size() != 6) fail("native sparse context categories mismatch");
    for (int i = 0; i < 6; ++i)
        if (!contexts.array[i].integer_token || number(contexts.array[i], "context") != i)
            fail("native sparse context categories mismatch");
    const auto& model_categories = model_contract.at("categorical");
    strings(model_categories.at("entry_signal_type"), {"R", "G"});
    strings(model_categories.at("next_signal_type"), {"R", "G"});
    const auto& model_contexts = model_categories.at("platoon_context");
    if (model_contexts.kind != Json::Array || model_contexts.array.size() != 6)
        fail("native sparse embedded model context categories mismatch");
    for (int i = 0; i < 6; ++i)
        if (!model_contexts.array[i].integer_token || number(model_contexts.array[i], "model context") != i)
            fail("native sparse embedded model context categories mismatch");
    const Json& quantization = meta.at("signal_quantization");
    require_string(quantization, "name", "nearest-positive-node-v1");
    require_string(quantization, "tie", "higher");
    number(quantization, "minimum_node_seconds", 0.001);
    number(quantization, "next_duration_key_scale", 1e9);
    number(quantization, "next_duration_tolerance_seconds", 1e-9);
    number(quantization, "entry_upper_tolerance_ulps", 8);
    signal_step_ = number(meta.at("signal_remaining_step"), "signal_remaining_step");
    if (signal_step_ <= 0) fail("native sparse nonpositive signal step");
    const Json& catalog = meta.at("signal_catalog");
    if (catalog.kind != Json::Array || catalog.array.empty() || catalog.array.size() > 100000)
        fail("native sparse invalid signal catalog");
    std::tuple<char, char, std::int64_t> previous;
    std::map<std::pair<char, char>, double> durations;
    for (std::size_t i = 0; i < catalog.array.size(); ++i) {
        const auto& item = catalog.array[i];
        const auto first = string_value(item.at("entry_type"), "entry_type");
        const auto second = string_value(item.at("next_type"), "next_type");
        if ((first != "R" && first != "G") || (second != "R" && second != "G")) fail("native sparse signal type mismatch");
        SignalGroup group;
        group.first = first[0]; group.second = second[0];
        group.next_duration = number(item.at("next_duration_seconds"), "next duration");
        group.maximum = number(item.at("entry_max_seconds"), "entry maximum");
        const auto key = duration_key(group.next_duration);
        if (integer64(item, "duration_key") != static_cast<std::uint64_t>(key)) fail("native sparse duration key mismatch");
        const auto order = std::make_tuple(group.first, group.second, key);
        if (i && order <= previous) fail("native sparse unordered/duplicate signal catalog");
        previous = order;
        const auto pair = std::make_pair(group.first, group.second);
        const auto seen = durations.find(pair);
        if (seen != durations.end() && group.next_duration - seen->second <= 2e-9)
            fail("native sparse ambiguous signal duration");
        durations[pair] = group.next_duration;
        group.begin = static_cast<std::size_t>(integer64(item, "point_start"));
        if (group.begin != signal_points_ || item.at("sentinel").kind != Json::Boolean)
            fail("native sparse invalid signal point offset");
        group.sentinel = item.at("sentinel").boolean;
        const Json& nodes = item.at("nodes");
        if (nodes.kind != Json::Array || nodes.array.empty() || nodes.array.size() > 100000)
            fail("native sparse invalid signal nodes");
        for (const auto& value : nodes.array) group.nodes.push_back(number(value, "signal node"));
        if (group.sentinel) {
            if (i != 0 || order != std::make_tuple('G', 'G', std::int64_t(0)) || group.next_duration != 0
                    || group.maximum != 0 || group.nodes != std::vector<double>{0})
                fail("native sparse invalid final-road G0G0 sentinel");
        } else if (!i || group.next_duration <= 0 || group.nodes != remaining_nodes(group.maximum, signal_step_)) {
            fail("native sparse noncanonical positive signal nodes");
        }
        signal_points_ += group.nodes.size();
        signal_groups_.emplace(order, i);
        signals_.push_back(std::move(group));
    }
    if (!signals_.front().sentinel || integer64(meta, "signal_point_count", 1) != signal_points_)
        fail("native sparse signal point count/sentinel mismatch");
    const auto declared_payload_limit = integer64(meta.at("resource_limits"), "max_payload_bytes");
    if (declared_payload_limit != 0 && declared_payload_limit != MAX_PAYLOAD)
        fail("native sparse unsupported payload limit policy");
    const auto payload_limit = declared_payload_limit ? declared_payload_limit : MAX_JSON;
    integer64(meta.at("resource_limits"), "max_json_integer", MAX_JSON, MAX_JSON);
    const Json& lengths = meta.at("road_lengths");
    if (lengths.kind != Json::Array || lengths.array.empty()) fail("native sparse missing road lengths");
    std::int64_t last_length = -1;
    for (std::size_t i = 0; i < lengths.array.size(); ++i) {
        const double value = number(lengths.array[i], "road length");
        const auto key = length_key(value);
        if (key <= last_length || !near(value, key / 10000.0)) fail("native sparse road length ordering/precision mismatch");
        lengths_.emplace(key, i); last_length = key;
    }
    const auto& grid = meta.at("free_speed_cap_grid");
    cap_index_min_ = static_cast<int>(integer64(grid, "index_min", 1, std::numeric_limits<int>::max()));
    cap_index_max_ = static_cast<int>(integer64(grid, "index_max", cap_index_min_, std::numeric_limits<int>::max()));
    cap_count_ = static_cast<std::size_t>(cap_index_max_) - cap_index_min_ + 1;
    cap_step_ = number(grid.at("step"), "cap step");
    cap_min_ = number(grid.at("min"), "cap minimum"); cap_max_ = number(grid.at("max"), "cap maximum");
    number(grid, "raw_boundary_tolerance", RAW_EPS); number(grid, "scaled_boundary_tolerance", SCALED_EPS);
    if (cap_step_ <= 0 || cap_min_ <= 0 || cap_max_ < cap_min_ || !std::isfinite(cap_index_max_ * cap_step_)
            || !near(cap_min_, cap_index_min_ * cap_step_) || !near(cap_max_, cap_index_max_ * cap_step_))
        fail("native sparse inconsistent cap lattice");
    // The new record has a uint32 road key; the simulator query API is signed
    // int. Do not inherit the historical RLE uint16 flow ceiling.
    road_max_ = static_cast<int>(integer64(meta, "road_flow_max", 0, std::numeric_limits<int>::max()));
    lane_max_ = static_cast<int>(integer64(meta, "lane_flow_max", 0, road_max_));
    const std::uint64_t lanes = static_cast<std::uint64_t>(lane_max_) + 1;
    const std::uint64_t flows = lanes * (static_cast<std::uint64_t>(road_max_) + 1) - lanes * lane_max_ / 2;
    if (integer64(meta, "flow_count", 1) != flows) fail("native sparse triangular flow count mismatch");
    const auto blocks = multiply(multiply(multiply(signal_points_, lengths_.size()), cap_count_), 6);
    group_count_ = integer64(meta, "group_count", 1);
    expanded_rows_ = integer64(meta, "expanded_rows", 1);
    if (group_count_ != multiply(blocks, lanes) || expanded_rows_ != multiply(blocks, flows))
        fail("native sparse inconsistent virtual dimensions");
    const auto& road_policy = meta.at("road_flow_quantization");
    require_string(road_policy, "unit", "vehicles"); require_string(road_policy, "tie", "higher");
    number(road_policy, "grid_origin", 0);
    boolean(road_policy, "preserve_lane_lower_bound", true); boolean(road_policy, "preserve_road_upper_bound", true);
    require_string(road_policy, "output_coverage", "all-legal-integer-road-flow");
    const auto policy = string_value(road_policy.at("name"), "road-flow policy");
    if (policy == "nearest-explicit-integer-road-flow-v1") {
        const auto& nodes = road_policy.at("nodes");
        if (nodes.kind != Json::Array || nodes.array.empty() || nodes.array.size() > static_cast<std::size_t>(road_max_) + 1)
            fail("native sparse invalid road nodes");
        for (const auto& node : nodes.array) {
            const double value = number(node, "road node");
            if (!node.integer_token || value < 0 || value > road_max_ || value != std::floor(value)
                    || (!road_nodes_.empty() && value <= road_nodes_.back())) fail("native sparse unordered/invalid road nodes");
            road_nodes_.push_back(static_cast<int>(value));
        }
        if (road_nodes_.front() != 0 || road_nodes_.back() != road_max_) fail("native sparse missing road endpoint nodes");
    } else if (policy == "nearest-integer-road-flow-v1") {
        road_step_ = integer64(road_policy, "step", 1);
        dense_until_ = integer64(road_policy, "dense_until");
    } else fail("native sparse unsupported road-flow quantization");
    scope_ = sha(meta, "scope_sha256");
    const auto entry_count = integer64(meta, "entry_count", 0, payload_limit / 20);
    const auto bytes = integer64(meta, "payload_bytes", 0, payload_limit);
    if (entry_count > std::numeric_limits<std::size_t>::max() || bytes > payload_.max_size())
        fail("native sparse payload exceeds host address range");
    entries_ = static_cast<std::size_t>(entry_count);
    if (bytes != 20ULL * entries_ || entries_ > expanded_rows_) fail("native sparse payload size/count mismatch");
    const auto expected_sha = sha(meta, "payload_sha256");
    // Reject truncated/forged headers before allocating potentially large data.
    const auto payload_start = input.tellg();
    input.seekg(0, std::ios::end);
    const auto file_end = input.tellg();
    if (payload_start < 0 || file_end < payload_start ||
            static_cast<std::uint64_t>(file_end - payload_start) != bytes)
        fail("native sparse actual payload size differs from header");
    input.seekg(payload_start);
    payload_.resize(static_cast<std::size_t>(bytes));
    CapContextSha256 hash;
    for (std::size_t offset = 0; offset < payload_.size();) {
        const auto count = std::min<std::size_t>(1024 * 1024, payload_.size() - offset);
        if (!input.read(reinterpret_cast<char*>(payload_.data() + offset), static_cast<std::streamsize>(count)))
            fail("native sparse truncated payload");
        hash.update(payload_.data() + offset, count); offset += count;
    }
    if (input.peek() != std::char_traits<char>::eof() || input.bad()) fail("native sparse trailing payload/stream error");
    if (hash.finish() != expected_sha) fail("native sparse payload SHA256 mismatch");
    std::pair<std::uint64_t, std::uint32_t> previous_key;
    for (std::size_t i = 0; i < entries_; ++i) {
        const auto* record = payload_.data() + 20 * i;
        const auto group = u64(record);
        const auto road = u32(record + 8);
        const auto lane = static_cast<int>(group % lanes);
        const auto key = std::make_pair(group, road);
        const double value = f64(record + 12);
        if (group >= group_count_ || road > static_cast<unsigned>(road_max_) || road < static_cast<unsigned>(lane)
                || representative_road_flow(static_cast<int>(road), lane) != road)
            fail("native sparse nonrepresentative/out-of-domain payload key");
        if ((i && key <= previous_key) || !std::isfinite(value) || value < 1)
            fail("native sparse duplicate/unsorted key or invalid prediction");
        previous_key = key;
    }
}

std::size_t NativeRoadTimeTable::signal_point(const SignalRGFeatures& raw, SignalRGFeatures* quantized) const {
    if (!loaded_) fail("native sparse table is not loaded");
    if ((raw.entry_type != 'R' && raw.entry_type != 'G') || (raw.next_type != 'R' && raw.next_type != 'G')
            || !std::isfinite(raw.entry_remaining_seconds) || raw.entry_remaining_seconds < 0)
        fail("native sparse invalid raw signal");
    const auto key = std::make_tuple(raw.entry_type, raw.next_type, duration_key(raw.next_duration_seconds));
    const auto found = signal_groups_.find(key);
    if (found == signal_groups_.end()) fail("native sparse unknown signal combination");
    const auto& group = signals_[found->second];
    if (std::abs(raw.next_duration_seconds - group.next_duration) > 1e-9) fail("native sparse next duration mismatch");
    std::size_t index = 0;
    if (group.sentinel) {
        if (raw.entry_remaining_seconds != 0 || raw.next_duration_seconds != 0) fail("native sparse G0G0 mismatch");
    } else {
        double value = raw.entry_remaining_seconds;
        const double ulp = std::nextafter(group.maximum, std::numeric_limits<double>::infinity()) - group.maximum;
        if (value > group.maximum && value - group.maximum <= 8 * ulp) value = group.maximum;
        if (value <= 0 || value > group.maximum) fail("native sparse signal time outside catalog");
        const auto high = std::lower_bound(group.nodes.begin(), group.nodes.end(), value);
        if (high == group.nodes.end()) fail("native sparse signal time exceeds last node");
        index = static_cast<std::size_t>(high - group.nodes.begin());
        if (index && value - group.nodes[index - 1] < group.nodes[index] - value) --index;
    }
    if (quantized) {
        *quantized = raw;
        quantized->entry_remaining_seconds = group.nodes[index];
        quantized->next_duration_seconds = group.next_duration;
    }
    return group.begin + index;
}

SignalRGFeatures NativeRoadTimeTable::quantizeSignal(const SignalRGFeatures& raw) const {
    SignalRGFeatures result;
    signal_point(raw, &result);
    return result;
}

std::uint64_t NativeRoadTimeTable::query_group(double length, int road, int lane, double cap,
                                               int context, const SignalRGFeatures& signal) const {
    const auto point = signal_point(signal, nullptr);
    if (road < 0 || road > road_max_ || lane < 0 || lane > lane_max_ || lane > road || context < 0 || context >= 6)
        fail("native sparse flow/context outside domain");
    if (!std::isfinite(cap) || cap <= 0) fail("native sparse invalid cap");
    if (clamp_cap_) cap = std::min(cap_max_, std::max(cap_min_, cap));
    else if (cap < cap_min_ - RAW_EPS || cap > cap_max_ + RAW_EPS)
        fail("native sparse cap outside domain");
    const double rounded = std::floor(cap / cap_step_ + 0.5 + SCALED_EPS);
    if (!std::isfinite(rounded) || rounded < cap_index_min_ || rounded > cap_index_max_) fail("native sparse cap index outside domain");
    const auto found = lengths_.find(length_key(length));
    if (found == lengths_.end()) fail("native sparse road length missing");
    const std::uint64_t cap_index = static_cast<std::uint64_t>(rounded - cap_index_min_);
    const std::uint64_t group = ((((std::uint64_t(point) * lengths_.size() + found->second) * cap_count_
                                   + cap_index) * 6 + context) * (std::uint64_t(lane_max_) + 1)) + lane;
    if (group >= group_count_) fail("native sparse computed group outside domain");
    return group;
}

std::uint32_t NativeRoadTimeTable::representative_road_flow(int road, int lane) const {
    if (!road_nodes_.empty()) {
        const auto high = std::lower_bound(road_nodes_.begin(), road_nodes_.end(), road);
        int low = lane;
        if (high != road_nodes_.begin()) low = std::max(low, *(high - 1));
        return static_cast<std::uint32_t>(road - low < *high - road ? low : *high);
    }
    const std::uint64_t value = road, lower_bound = lane, maximum = road_max_;
    if (value <= dense_until_) return static_cast<std::uint32_t>(value);
    const auto lower = (value / road_step_) * road_step_;
    const std::uint64_t candidates[]{lower_bound, maximum, std::min(dense_until_, maximum), lower, lower + road_step_};
    auto best = lower_bound, distance = value - lower_bound;
    for (auto node : candidates) {
        if (node < lower_bound || node > maximum) continue;
        const auto delta = node > value ? node - value : value - node;
        if (delta < distance || (delta == distance && node > best)) { best = node; distance = delta; }
    }
    return static_cast<std::uint32_t>(best);
}

bool NativeRoadTimeTable::lookupDemand(double length, int road, int lane, double cap,
                                       int context, const SignalRGFeatures& signal, double& value,
                                       std::uint64_t& group, std::uint32_t& representative) const {
    group = query_group(length, road, lane, cap, context, signal);
    representative = representative_road_flow(road, lane);
    const auto wanted = std::make_pair(group, representative);
    std::size_t low = 0, high = entries_;
    while (low < high) {
        const auto middle = low + (high - low) / 2;
        const auto* record = payload_.data() + 20 * middle;
        if (std::make_pair(u64(record), u32(record + 8)) < wanted) low = middle + 1;
        else high = middle;
    }
    if (low == entries_) return false;
    const auto* record = payload_.data() + 20 * low;
    if (u64(record) != group || u32(record + 8) != representative) return false;
    value = f64(record + 12);
    return true;
}

void NativeRoadTimeTable::requireCrossingTime(double seconds) const {
    if (!loaded_ || !std::isfinite(seconds) || seconds != crossing_time_) fail("native sparse deployment crossing-time mismatch");
}

void NativeRoadTimeTable::requireDeparturePositionCorrection(bool enabled) const {
    if (!loaded_ || enabled != departure_position_correction_)
        fail("native sparse first-road position correction mismatch; use the table's recorded on/off setting");
}

void NativeRoadTimeTable::requireSignalRemainingStep(double seconds) const {
    if (!loaded_ || !std::isfinite(seconds) || seconds <= 0 || seconds != signal_step_)
        fail("native sparse signal node-step expectation mismatch");
}

void NativeRoadTimeTable::requireUnsignalizedContract() const {
    if (!loaded_) fail("native sparse deployment contract not loaded");
}

void NativeRoadTimeTable::requireNoInternalLinksContract() const {
    if (!loaded_) fail("native sparse deployment contract not loaded");
}

void NativeRoadTimeTable::requireNetworkFile(const std::string& path) const {
    if (!loaded_) fail("native sparse table is not loaded");
    std::ifstream input(path, std::ios::binary);
    if (!input) fail("native sparse cannot verify deployment network");
    CapContextSha256 hash;
    char block[65536];
    while (input.read(block, sizeof(block)) || input.gcount())
        hash.update(reinterpret_cast<const unsigned char*>(block), static_cast<std::size_t>(input.gcount()));
    if (input.bad() || hash.finish() != network_sha_) fail("native sparse deployment network SHA256 mismatch");
}
