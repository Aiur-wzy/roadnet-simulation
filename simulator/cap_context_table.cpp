#include "cap_context_table.h"
#include "cap_context.h"
#include "cap_context_json.h"
#include "cap_context_sha256.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using namespace cap_context_json;
constexpr std::size_t MAX_ROWS = 50000000;
constexpr std::size_t MAX_HEADER = 1024 * 1024;
constexpr double RAW_EPS = 1e-12;
constexpr double SCALED_EPS = 1e-9;
std::int64_t length_key(double length) {
    if (!std::isfinite(length) || length <= 0 || length * 10000 > 9e15)
        fail("invalid physical road length");
    return static_cast<std::int64_t>(std::floor(length * 10000 + 0.5 + SCALED_EPS));
}
bool nearly_equal(double a, double b) {
    return std::abs(a - b) <= 1e-10 * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}
std::size_t bounded_product(std::size_t left, std::size_t right) {
    if (left == 0 || right == 0 || left > MAX_ROWS / right) fail("expanded-row safety limit exceeded");
    return left * right;
}
void read_bytes(std::istream& input, unsigned char* target, std::size_t count, CapContextSha256& sha) {
    if (!input.read(reinterpret_cast<char*>(target), static_cast<std::streamsize>(count)))
        fail("truncated binary payload");
    sha.update(target, count);
}
std::uint32_t little_u32(const unsigned char* b) {
    return std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) |
        (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24);
}

// Frozen ABC v1 contract, mirrored from abc_context_contract(). Only the two
// explicitly configurable seconds fields may vary. Do not silently accept a
// changed population, mapping, classification boundary, or audit definition.
const char* ABC_CONTRACT = R"JSON({
"version":"free-clock-context-abc-v1",
"base_context_contract":{"version":"free-clock-context-v1","p_threshold":0.05,"e_threshold":2.0,"epsilon_seconds":1e-9,"crossing_time_seconds":3.0,"sampling":"upstream-discharge-known-reservations-v1","geometry":"base-first-v1","candidate_lane":"incoming-logical-lane-v1"},
"context_ids":[0,1,2,3,4,5],
"context_mapping":{"0":"A","1":"legacy_1","2":"legacy_2","3":"legacy_3","4":"B","5":"C"},
"context0_overdue_threshold_seconds":0.0,
"overdue_epsilon_seconds":1e-9,
"overdue_formula":"max(0, decision_time - candidate.free_finish_time)",
"overdue_aggregation_population":"same eligible non-ego logical-lane records as base query; planned_enter_time <= decision_time",
"context0_subclass_rules":{"A":"legacy_context == 0 and eligible_count == 0","B":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds <= threshold + epsilon","C":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds > threshold + epsilon","not_context0":"legacy_context in (1, 2, 3); preserve original context ID"},
"overdue_count_rule":"count(overdue_seconds > epsilon); independent of classification threshold",
"overdue_fraction_denominator":"eligible_count; zero if eligible_count == 0",
"mean_overdue_denominator":"eligible_count; zero if eligible_count == 0",
"continuous_audit_scope":"record all contexts, including legacy 1/2/3; do not clip at threshold",
"interpretation":"logical free-clock overrun proxy, not physical waiting or measured queue time"
})JSON";

void exact_abc_contract(const Json& actual, const Json& expected, const std::string& path) {
    if (actual.kind != expected.kind) fail("wrong ABC JSON type: " + path);
    if (expected.kind == Json::Object) {
        if (actual.object.size() != expected.object.size()) fail("wrong ABC contract keys: " + path);
        for (const auto& entry : expected.object)
            exact_abc_contract(actual.at(entry.first), entry.second, path + "." + entry.first);
    } else if (expected.kind == Json::Array) {
        if (actual.array.size() != expected.array.size()) fail("wrong ABC array length: " + path);
        for (std::size_t i = 0; i < expected.array.size(); ++i)
            exact_abc_contract(actual.array[i], expected.array[i], path + "[" + std::to_string(i) + "]");
    } else if (expected.kind == Json::Number) {
        if (expected.integer_token && !actual.integer_token) fail("ABC JSON integer required: " + path);
        if (!std::isfinite(actual.number)) fail("nonfinite ABC number: " + path);
        if (path == "context_contract.context0_overdue_threshold_seconds" ||
            path == "context_contract.base_context_contract.crossing_time_seconds") {
            if (actual.number < 0) fail("negative ABC seconds: " + path);
        } else if (actual.number != expected.number) fail("unsupported ABC number: " + path);
    } else if (expected.kind == Json::String) {
        if (actual.string != expected.string) fail("unsupported ABC value: " + path);
    } else if (expected.kind == Json::Boolean && actual.boolean != expected.boolean) {
        fail("unsupported ABC boolean: " + path);
    }
}
}

void CapContextTable::clear() {
    values_.clear(); lengths_.clear(); label_.clear();
    road_flow_max_ = lane_flow_max_ = cap_index_min_ = cap_index_max_ = 0;
    flow_count_ = cap_count_ = 0;
    cap_step_ = cap_min_ = cap_max_ = crossing_time_ = 0;
    abc_ = false; context_count_ = 0; overdue_threshold_ = 0;
}
void CapContextTable::load(std::istream& input, bool abc) {
    clear();
    try { load_impl(input, abc); }
    catch (...) { clear(); throw; }
}
void CapContextTable::load_impl(std::istream& input, bool abc) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                  "CAMS cap/context requires IEEE-754 binary64");
    std::string header;
    char ch;
    bool ended = false;
    while (input.get(ch)) {
        if (ch == '\n') { ended = true; break; }
        if (header.size() >= MAX_HEADER) fail("metadata exceeds 1MiB");
        header += ch;
    }
    if (!ended) fail("missing metadata newline");
    const Json meta = JsonParser(header).parse();
    require_string(meta, "format", abc ? "CAMS_CAP_CONTEXT_ABC_RLE_V1" : "CAMS_CAP_CONTEXT_RLE_V1");
    require_string(meta, "training_schema", abc ? "road-time-cap-context-abc-v1" : "road-time-cap-context-v1");
    abc_ = abc; context_count_ = abc ? 6 : 4;
    require_string(meta, "payload_layout", "offsets_u32_le+records_u16_u16_f64_le-v1");
    const Json& columns = meta.at("columns");
    const std::vector<std::string> expected_columns = {"road_length", "road_flow", "lane_flow",
        "free_speed_cap", "platoon_context", "travel_time_no_waiting"};
    if (columns.kind != Json::Array || columns.array.size() != expected_columns.size())
        fail("wrong feature columns");
    for (std::size_t i = 0; i < expected_columns.size(); ++i)
        if (string_value(columns.array[i], "columns") != expected_columns[i]) fail("wrong feature order");
    label_ = string_value(meta.at("source_label"), "source_label");
    const std::vector<std::string> labels = {"travel_time_label_gt_0p1", "travel_time_label_gt_0p5",
        "travel_time_label_gt_1", "travel_time_label_gt_1p5", "travel_time_label_gt_2", "travel_time_label_gt_3"};
    if (std::find(labels.begin(), labels.end(), label_) == labels.end()) fail("unsupported source label");
    number(meta, "static_key_scale", 10000);
    number(meta, "context_count", context_count_);
    number(meta, "prediction_floor_seconds", 1.0);
    const Json& outer_contract = meta.at("context_contract");
    if (abc) {
        if (!meta.at("context_count").integer_token) fail("ABC context_count must be JSON integer 6");
        exact_abc_contract(outer_contract, JsonParser(ABC_CONTRACT).parse(), "context_contract");
        const std::string categories = R"JSON({"platoon_context":[0,1,2,3,4,5]})JSON";
        exact_abc_contract(meta.at("categorical_features"), JsonParser(categories).parse(), "categorical_features");
        if (meta.at("categorical_ordered").kind != Json::Boolean || meta.at("categorical_ordered").boolean)
            fail("ABC categorical_ordered must be explicitly false");
        overdue_threshold_ = number(outer_contract.at("context0_overdue_threshold_seconds"),
                                    "context0_overdue_threshold_seconds");
    }
    const Json& contract = abc ? outer_contract.at("base_context_contract") : outer_contract;
    require_string(contract, "version", "free-clock-context-v1");
    require_string(contract, "sampling", "upstream-discharge-known-reservations-v1");
    require_string(contract, "geometry", "base-first-v1");
    require_string(contract, "candidate_lane", "incoming-logical-lane-v1");
    number(contract, "p_threshold", CAP_CONTEXT_P_THRESHOLD);
    number(contract, "e_threshold", CAP_CONTEXT_E_THRESHOLD);
    number(contract, "epsilon_seconds", CAP_CONTEXT_TIME_EPSILON);
    crossing_time_ = number(contract.at("crossing_time_seconds"), "crossing_time_seconds");
    if (crossing_time_ < 0) fail("negative crossing time");
    road_flow_max_ = integer(meta, "road_flow_max", 0, 65535);
    lane_flow_max_ = integer(meta, "lane_flow_max", 0, road_flow_max_);
    const std::uint64_t lane_count = std::uint64_t(lane_flow_max_) + 1;
    const std::uint64_t computed_flow = lane_count * (std::uint64_t(road_flow_max_) + 1)
                                    - lane_count * lane_flow_max_ / 2;
    flow_count_ = integer(meta, "flow_count", 1, MAX_ROWS);
    if (flow_count_ != computed_flow) fail("flow_count does not match legal triangular flow grid");
    const Json& grid = meta.at("free_speed_cap_grid");
    cap_index_min_ = integer(grid, "index_min", 1, std::numeric_limits<int>::max());
    cap_index_max_ = integer(grid, "index_max", cap_index_min_, std::numeric_limits<int>::max());
    cap_count_ = static_cast<std::size_t>(cap_index_max_) - cap_index_min_ + 1;
    cap_step_ = number(grid.at("step"), "step");
    cap_min_ = number(grid.at("min"), "min");
    cap_max_ = number(grid.at("max"), "max");
    number(grid, "scaled_boundary_tolerance", SCALED_EPS);
    number(grid, "raw_boundary_tolerance", RAW_EPS);
    if (cap_step_ <= 0 || cap_min_ <= 0 || cap_max_ < cap_min_ ||
        !std::isfinite(cap_index_max_ * cap_step_) ||
        !nearly_equal(cap_min_, cap_index_min_ * cap_step_) ||
        !nearly_equal(cap_max_, cap_index_max_ * cap_step_)) fail("invalid uniform cap lattice");
    const Json& lengths = meta.at("road_lengths");
    if (lengths.kind != Json::Array || lengths.array.empty()) fail("road_lengths must be a nonempty array");
    std::int64_t previous_key = -1;
    for (std::size_t i = 0; i < lengths.array.size(); ++i) {
        const double value = number(lengths.array[i], "road_length");
        const auto key = length_key(value);
        if (key <= previous_key || !nearly_equal(value, key / 10000.0))
            fail("duplicate, unordered or non-four-decimal road length");
        lengths_.emplace(key, i); previous_key = key;
    }
    const auto blocks = bounded_product(bounded_product(lengths_.size(), cap_count_), context_count_);
    const auto expected_rows = bounded_product(blocks, flow_count_);
    const auto expected_groups = bounded_product(blocks, static_cast<std::size_t>(lane_count));
    const auto rows = static_cast<std::size_t>(integer(meta, "expanded_rows", 1, MAX_ROWS));
    const auto groups = static_cast<std::size_t>(integer(meta, "group_count", 1, MAX_ROWS));
    const auto runs = static_cast<std::size_t>(integer(meta, "run_count", 1, MAX_ROWS));
    if (rows != expected_rows || groups != expected_groups || runs < groups || runs > rows)
        fail("inconsistent expanded/group/run dimensions");
    const std::uint64_t expected_bytes = 4ULL * (groups + 1) + 12ULL * runs;
    if (number(meta.at("payload_bytes"), "payload_bytes") != static_cast<double>(expected_bytes))
        fail("payload byte count mismatch");
    const std::string expected_sha = string_value(meta.at("payload_sha256"), "payload_sha256");
    if (expected_sha.size() != 64 || expected_sha.find_first_not_of("0123456789abcdef") != std::string::npos)
        fail("invalid lowercase payload SHA256");
    std::vector<std::uint32_t> offsets(groups + 1);
    CapContextSha256 sha;
    unsigned char bytes[12];
    for (std::size_t i = 0; i <= groups; ++i) {
        read_bytes(input, bytes, 4, sha); offsets[i] = little_u32(bytes);
        if ((i == 0 && offsets[i] != 0) || offsets[i] > runs ||
            (i > 0 && offsets[i] <= offsets[i - 1])) fail("invalid group offsets");
    }
    if (offsets.back() != runs) fail("last offset does not match run_count");
    values_.resize(rows);
    std::size_t cursor = 0;
    for (std::size_t group = 0; group < groups; ++group) {
        unsigned expected_low = static_cast<unsigned>(group % lane_count);
        double previous_value = 0;
        for (std::size_t run = offsets[group]; run < offsets[group + 1]; ++run) {
            read_bytes(input, bytes, 12, sha);
            const unsigned low = unsigned(bytes[0]) | (unsigned(bytes[1]) << 8);
            const unsigned high = unsigned(bytes[2]) | (unsigned(bytes[3]) << 8);
            std::uint64_t bits = 0;
            for (unsigned i = 0; i < 8; ++i) bits |= std::uint64_t(bytes[4 + i]) << (i * 8);
            double value;
            std::memcpy(&value, &bits, 8);
            if (low != expected_low || high < low || high > static_cast<unsigned>(road_flow_max_))
                fail("run bounds overlap, leave a gap, or escape legal flow range");
            if (!std::isfinite(value) || value < 1.0) fail("prediction is not finite and >=1 second");
            if (run > offsets[group] && value == previous_value) fail("adjacent equal runs are not canonical");
            const std::size_t count = high - low + 1;
            if (count > rows - cursor) fail("decoded row overflow");
            std::fill_n(values_.begin() + cursor, count, value);
            cursor += count; expected_low = high + 1; previous_value = value;
        }
        if (expected_low != static_cast<unsigned>(road_flow_max_) + 1) fail("incomplete group coverage");
    }
    if (cursor != rows) fail("decoded row count mismatch");
    if (input.peek() != std::char_traits<char>::eof()) fail("trailing payload data");
    if (input.bad()) fail("payload input error");
    if (sha.finish() != expected_sha) fail("payload SHA256 mismatch");
}

void CapContextTable::requireCrossingTime(double seconds) const {
    if (values_.empty()) fail("table is not loaded");
    if (!std::isfinite(seconds) || seconds != crossing_time_)
        fail("runtime crossing time differs from frozen context contract");
}
double CapContextTable::lookup(double road_length, int road_flow, int lane_flow,
                               double raw_cap, int context) const {
    if (values_.empty()) fail("table is not loaded");
    if (road_flow < 0 || road_flow > road_flow_max_ || lane_flow < 0 ||
        lane_flow > lane_flow_max_ || lane_flow > road_flow) fail("flow outside legal table domain");
    if (context < 0 || context >= context_count_)
        fail(abc_ ? "ABC context must be an integer from 0 through 5" : "context must be 0, 1, 2 or 3");
    if (!std::isfinite(raw_cap) || raw_cap <= 0 || raw_cap < cap_min_ - RAW_EPS || raw_cap > cap_max_ + RAW_EPS)
        fail("free speed cap outside table domain");
    const double rounded_index = std::floor(raw_cap / cap_step_ + 0.5 + SCALED_EPS);
    if (!std::isfinite(rounded_index) || rounded_index < cap_index_min_ || rounded_index > cap_index_max_)
        fail("cap grid lookup outside domain");
    const auto cap_index = static_cast<std::int64_t>(rounded_index);
    const auto found = lengths_.find(length_key(road_length));
    if (found == lengths_.end()) fail("physical road length missing from table");
    const std::size_t lane = static_cast<std::size_t>(lane_flow);
    const std::size_t flow_offset = lane * (static_cast<std::size_t>(road_flow_max_) + 1)
        - lane * (lane ? lane - 1 : 0) / 2 + static_cast<std::size_t>(road_flow - lane_flow);
    const std::size_t block = (found->second * cap_count_ + static_cast<std::size_t>(cap_index - cap_index_min_)) * context_count_
                           + static_cast<std::size_t>(context);
    return values_.at(block * flow_count_ + flow_offset);
}
