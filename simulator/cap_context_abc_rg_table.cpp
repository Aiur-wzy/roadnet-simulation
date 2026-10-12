#include "cap_context_abc_rg_table.h"
#include "cap_context_json.h"
#include "cap_context_sha256.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
using namespace cap_context_json;
constexpr std::uint64_t MAX_ROWS = 4000000000ULL;
constexpr std::uint64_t MAX_PAYLOAD = 512ULL * 1024 * 1024;
constexpr std::size_t MAX_HEADER = 1024 * 1024;
constexpr double RAW_EPS = 1e-12;
constexpr double SCALED_EPS = 1e-9;
constexpr double NEXT_SCALE = 1e9;
constexpr double NEXT_EPS = 1e-9;
constexpr double MIN_REMAINING = 0.001;

std::uint64_t unsigned_integer(const Json& object, const std::string& key,
                               std::uint64_t minimum, std::uint64_t maximum) {
    const Json& entry = object.at(key);
    const double value = number(entry, key);
    if (!entry.integer_token || value < minimum || value > maximum || value != std::floor(value)) {
        fail("ABC/RG JSON integer required for " + key);
    }
    return static_cast<std::uint64_t>(value);
}

std::uint64_t checked_product(std::uint64_t first, std::uint64_t second,
                              std::uint64_t maximum, const char* name) {
    if (!first || !second || first > maximum / second) {
        fail(std::string("ABC/RG checked-product limit exceeded: ") + name);
    }
    return first * second;
}

bool close(double first, double second) {
    return std::abs(first - second) <= 1e-10 * std::max(1.0, std::max(std::abs(first), std::abs(second)));
}

std::int64_t length_key(double value) {
    if (!std::isfinite(value) || value <= 0 || value * 10000 > 9e15) {
        fail("ABC/RG invalid physical road length");
    }
    return static_cast<std::int64_t>(std::floor(value * 10000 + 0.5 + SCALED_EPS));
}

std::uint32_t little_u32(const unsigned char* bytes) {
    return std::uint32_t(bytes[0]) | (std::uint32_t(bytes[1]) << 8)
        | (std::uint32_t(bytes[2]) << 16) | (std::uint32_t(bytes[3]) << 24);
}

unsigned little_u16(const unsigned char* bytes) {
    return unsigned(bytes[0]) | (unsigned(bytes[1]) << 8);
}

double little_f64(const unsigned char* bytes) {
    std::uint64_t bits = 0;
    for (unsigned index = 0; index < 8; ++index) {
        bits |= std::uint64_t(bytes[index]) << (8 * index);
    }
    double result;
    std::memcpy(&result, &bits, 8);
    return result;
}

std::int64_t duration_key(double value) {
    if (!std::isfinite(value) || value < 0 || !std::isfinite(value * NEXT_SCALE) || value * NEXT_SCALE > 9e15) {
        fail("ABC/RG next duration exceeds finite exact-key domain");
    }
    return static_cast<std::int64_t>(std::floor(value * NEXT_SCALE + 0.5));
}

std::vector<double> remaining_nodes(double maximum, double step) {
    if (!std::isfinite(maximum) || maximum <= 0 || !std::isfinite(step) || step <= 0
            || !std::isfinite(maximum / step) || maximum / step > 99998.0) {
        fail("ABC/RG invalid remaining-time grid or too many nodes");
    }
    const double minimum = std::min(MIN_REMAINING, maximum);
    std::vector<double> nodes = {minimum, maximum};
    const std::size_t last = static_cast<std::size_t>(std::floor(maximum / step));
    for (std::size_t index = 1; index <= last; ++index) {
        const double value = static_cast<double>(index) * step;
        if (minimum < value && value < maximum) { nodes.push_back(value); }
    }
    std::sort(nodes.begin(), nodes.end());
    nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
    return nodes;
}

const char* ABC_CONTRACT = R"JSON({
"version":"free-clock-context-abc-v1",
"base_context_contract":{"version":"free-clock-context-v1","p_threshold":0.05,"e_threshold":2.0,"epsilon_seconds":1e-9,"crossing_time_seconds":3.0,"sampling":"upstream-discharge-known-reservations-v1","geometry":"base-first-v1","candidate_lane":"incoming-logical-lane-v1"},
"context_ids":[0,1,2,3,4,5],
"context_mapping":{"0":"A","1":"legacy_1","2":"legacy_2","3":"legacy_3","4":"B","5":"C"},
"context0_overdue_threshold_seconds":0.0,"overdue_epsilon_seconds":1e-9,
"overdue_formula":"max(0, decision_time - candidate.free_finish_time)",
"overdue_aggregation_population":"same eligible non-ego logical-lane records as base query; planned_enter_time <= decision_time",
"context0_subclass_rules":{"A":"legacy_context == 0 and eligible_count == 0","B":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds <= threshold + epsilon","C":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds > threshold + epsilon","not_context0":"legacy_context in (1, 2, 3); preserve original context ID"},
"overdue_count_rule":"count(overdue_seconds > epsilon); independent of classification threshold",
"overdue_fraction_denominator":"eligible_count; zero if eligible_count == 0",
"mean_overdue_denominator":"eligible_count; zero if eligible_count == 0",
"continuous_audit_scope":"record all contexts, including legacy 1/2/3; do not clip at threshold",
"interpretation":"logical free-clock overrun proxy, not physical waiting or measured queue time"
})JSON";

const char* SIGNAL_CONTRACT = R"JSON({
"version":"static-movement-rg-segments-v1",
"feature_schema":"road-time-cap-context-abc-rg-v1",
"feature_columns":["entry_signal_type","entry_signal_remaining_seconds","next_signal_type","next_signal_duration_seconds"],
"audit_columns":["signal_state_at_reference","signal_yellow_skipped"],
"signal_types":["R","G"],
"state_normalization":{"r":"R","R":"R","g":"G","G":"G","y":"Y","Y":"Y"},
"supported_program":"finite positive-duration phases; static sequential cyclic program; one outgoing movement",
"reference_time":"planned_enter_time: first=decision_time; later=decision_time+context crossing seconds",
"phase_clock_input":"current phase index and its remaining seconds at decision d",
"phase_boundary":"half-open phase intervals; remaining == 0 advances to the next phase",
"same_color_merge":"merge consecutive normalized same-color phases, including across the cycle boundary; never merge across yellow",
"reference_rg":"entry is current R/G segment remainder; next is full next non-yellow segment after current segment ends",
"reference_yellow":"skip current yellow; entry is full first following R/G segment; next is full following non-yellow segment",
"yellow_duration_policy":"advance clock through yellow, but exclude yellow from both formal durations; no start_delta or yellow-duration feature",
"second_segment_color":"may equal entry color when yellow separates two segments",
"duration_unit":"seconds; no fixed cycle length or green duration",
"lookahead":"nonnegative finite seconds; any number of full cycles supported",
"unavailable_finite_segments":"raise ValueError for constant-color or all-yellow cycles; never emit infinite or invented finite durations",
"no_tls_placeholder":{"entry_signal_type":"G","entry_signal_remaining_seconds":0.0,"next_signal_type":"G","next_signal_duration_seconds":0.0,"interpretation":"G0G0 is an explicit no-TLS placeholder, not measured green or an always-green timed program","signal_state_at_reference":"G","signal_yellow_skipped":0},
"complexity":"precompute O(phases log phases), O(phases) memory; query O(log phases), no per-vehicle phase scan",
"movement":"current_external_road -> next_route_road; ABC logical fromLane; live linkIndex checked",
"anchor":"observed active STATIC program/phase/absolute nextSwitch; half-open phase boundaries",
"causality":"known fixed plan at decision; no future actual entry, trajectory or release information",
"supported_programs":"fixed sequential cyclic STATIC; reject WAUT, switching, calendar edits and unknown mappings"
})JSON";

void exact_contract(const Json& actual, const Json& expected, const std::string& path) {
    if (actual.kind != expected.kind) { fail("ABC/RG wrong contract JSON type: " + path); }
    if (expected.kind == Json::Object) {
        if (actual.object.size() != expected.object.size()) { fail("ABC/RG wrong contract keys: " + path); }
        for (const auto& entry : expected.object) { exact_contract(actual.at(entry.first), entry.second, path + "." + entry.first); }
    } else if (expected.kind == Json::Array) {
        if (actual.array.size() != expected.array.size()) { fail("ABC/RG wrong array size: " + path); }
        for (std::size_t index = 0; index < expected.array.size(); ++index) {
            exact_contract(actual.array[index], expected.array[index], path + "[" + std::to_string(index) + "]");
        }
    } else if (expected.kind == Json::Number) {
        if (expected.integer_token && !actual.integer_token) { fail("ABC/RG integer contract required: " + path); }
        if (path == "context_contract.context0_overdue_threshold_seconds" || path == "context_contract.base_context_contract.crossing_time_seconds") {
            if (!std::isfinite(actual.number) || actual.number < 0) { fail("ABC/RG invalid contract seconds: " + path); }
        } else if (actual.number != expected.number) { fail("ABC/RG unsupported contract number: " + path); }
    } else if (expected.kind == Json::String) {
        if (actual.string != expected.string) { fail("ABC/RG unsupported contract string: " + path); }
    } else if (expected.kind == Json::Boolean && actual.boolean != expected.boolean) {
        fail("ABC/RG unsupported contract boolean: " + path);
    }
}

}  // namespace

void CapContextABCRGTable::clear() {
    std::vector<unsigned char>().swap(payload_);
    lengths_.clear(); signals_.clear(); signal_groups_.clear(); label_.clear();
    expanded_rows_ = 0; groups_ = run_count_ = records_start_ = 0;
    cap_count_ = signal_point_count_ = 0;
    road_flow_max_ = lane_flow_max_ = cap_index_min_ = cap_index_max_ = 0;
    cap_step_ = cap_min_ = cap_max_ = crossing_time_ = overdue_threshold_ = 0;
    signal_remaining_step_ = 0;
    demand_mode_ = false; demand_entry_count_ = 0; demand_scope_.clear();
    constant_green_contract_ = false;
    no_internal_links_contract_ = false;
    road_flow_step_ = 1; road_flow_dense_until_ = 10;
    road_flow_nodes_.clear();
}

void CapContextABCRGTable::load(std::istream& input) {
    clear();
    try { load_impl(input); }
    catch (...) { clear(); throw; }
}

void CapContextABCRGTable::loadDemand(std::istream& input) {
    clear();
    try { load_impl(input, true); }
    catch (...) { clear(); throw; }
}

void CapContextABCRGTable::requireCrossingTime(double seconds) const {
    if (!expanded_rows_) { fail("ABC/RG table is not loaded"); }
    if (!std::isfinite(seconds) || seconds != crossing_time_) {
        fail("ABC/RG runtime crossing time differs from frozen contract");
    }
}

void CapContextABCRGTable::requireSignalRemainingStep(double expected) const {
    if (!expanded_rows_) { fail("ABC/RG table is not loaded"); }
    if (!std::isfinite(expected) || expected <= 0) {
        fail("ABC/RG expected signal remaining step must be finite and positive");
    }
    if (expected != signal_remaining_step_) {
        fail("ABC/RG expected signal remaining step differs from table header");
    }
}

void CapContextABCRGTable::requireUnsignalizedContract() const {
    if (!expanded_rows_ || !constant_green_contract_) {
        fail("ABC/RG unsignalized entries/exits require the zero-crossing G,H,G,H v3 model contract; legacy model cannot be silently reused");
    }
}

void CapContextABCRGTable::requireNoInternalLinksContract() const {
    if (!expanded_rows_ || (!no_internal_links_contract_ && crossing_time_ != 0.0)) {
        fail("ABC/RG no-internal network entries require the v4 zero-crossing contract or an already zero trained crossing; legacy nonzero-crossing model cannot be silently reused");
    }
}

void CapContextABCRGTable::load_impl(std::istream& input, bool demand) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559,
                  "ABC/RG tables require IEEE-754 binary64");
    std::string line;
    bool ended = false;
    char character;
    while (input.get(character)) {
        if (character == '\n') { ended = true; break; }
        if (line.size() >= MAX_HEADER) { fail("ABC/RG header exceeds 1MiB"); }
        line.push_back(character);
    }
    if (!ended) { fail("ABC/RG missing header newline"); }
    const Json meta = JsonParser(line).parse();
    require_string(meta, "format", demand ? "CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1" : "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1");
    require_string(meta, "training_schema", "road-time-cap-context-abc-rg-v1");
    require_string(meta, "payload_layout", demand ? "sorted-u32-u32-f64-v1" : "offsets_u32_le+records_u16_u16_f64_le-v1");
    demand_mode_ = demand;
    const std::vector<std::string> expected_columns = {"road_length", "road_flow", "lane_flow", "free_speed_cap", "platoon_context",
        "entry_signal_type", "entry_signal_remaining_seconds", "next_signal_type", "next_signal_duration_seconds", "travel_time_no_waiting"};
    const Json& columns = meta.at("columns");
    if (columns.kind != Json::Array || columns.array.size() != expected_columns.size()) { fail("ABC/RG wrong columns"); }
    for (std::size_t index = 0; index < expected_columns.size(); ++index) {
        if (string_value(columns.array[index], "column") != expected_columns[index]) { fail("ABC/RG wrong feature order"); }
    }
    label_ = string_value(meta.at("source_label"), "source_label");
    const std::vector<std::string> labels = {"travel_time_label_gt_0p1", "travel_time_label_gt_0p5", "travel_time_label_gt_1",
        "travel_time_label_gt_1p5", "travel_time_label_gt_2", "travel_time_label_gt_3"};
    if (std::find(labels.begin(), labels.end(), label_) == labels.end()) { fail("ABC/RG unsupported label"); }
    unsigned_integer(meta, "context_count", 6, 6);
    number(meta, "static_key_scale", 10000.0);
    number(meta, "prediction_floor_seconds", 1.0);
    exact_contract(meta.at("context_contract"), JsonParser(ABC_CONTRACT).parse(), "context_contract");
    Json expected_signal = JsonParser(SIGNAL_CONTRACT).parse();
    const std::string signal_version = string_value(meta.at("signal_contract").at("version"), "signal version");
    if (signal_version == "static-movement-rg-segments-v2" || signal_version == "static-movement-rg-segments-v3" ||
        signal_version == "static-movement-rg-segments-v4") {
        expected_signal.object["version"] = JsonParser(R"JSON("static-movement-rg-segments-v2")JSON").parse();
        expected_signal.object["supported_programs"] = JsonParser(R"JSON("fixed sequential cyclic STATIC; reject WAUT and queried-controller calendar changes; network green horizon frozen after initialization; external signal overrides unsupported")JSON").parse();
        expected_signal.object["no_tls_placeholder"].object["interpretation"] = JsonParser(
            R"JSON("G0G0 identifies a final route occurrence with no outgoing movement only")JSON").parse();
        expected_signal.object["unsignalized_connection"] = JsonParser(R"JSON({
            "types":["G","G"],
            "durations":"both equal network longest finite continuous movement green segment H",
            "green_merge":"normalize G/g; merge adjacent phases and across cycle boundary; exclude yellow",
            "network_scope":"external controlled connections; one fixed sequential static program per TLS",
            "clock":"constant H at every reference time; no countdown or artificial phase event",
            "missing_finite_green":"error when an unsignalized movement needs H; never invent a duration"
        })JSON").parse();
    }
    if (signal_version == "static-movement-rg-segments-v3" || signal_version == "static-movement-rg-segments-v4") {
        constant_green_contract_ = true;
        expected_signal.object["version"] = JsonParser(R"JSON("static-movement-rg-segments-v3")JSON").parse();
        expected_signal.object["reference_time"] = JsonParser(R"JSON("planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized=0; otherwise context crossing seconds")JSON").parse();
        expected_signal.object["crossing_policy"] = JsonParser(R"JSON({
            "first_road":"zero",
            "unsignalized_entry":"zero only for known nonsignalized incoming junction without TLS connection",
            "other_entry":"base context crossing_time_seconds; controlled junction or TLS connection retains crossing",
            "capacity":"unchanged waiting buffers, movement service interval and downstream storage"
        })JSON").parse();
    }
    if (signal_version == "static-movement-rg-segments-v4") {
        no_internal_links_contract_ = true;
        expected_signal.object["version"] = JsonParser(R"JSON("static-movement-rg-segments-v4")JSON").parse();
        expected_signal.object["reference_time"] = JsonParser(R"JSON("planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized/no-internal network=0; otherwise context crossing seconds")JSON").parse();
        expected_signal.object["crossing_policy"].object["network_without_internal_links"] = JsonParser(R"JSON("zero for every incoming junction when loaded SUMO network has no internal edges or via references")JSON").parse();
    }
    exact_contract(meta.at("signal_contract"), expected_signal, "signal_contract");
    const std::string categories = R"JSON({"platoon_context":[0,1,2,3,4,5],"entry_signal_type":["R","G"],"next_signal_type":["R","G"]})JSON";
    exact_contract(meta.at("categorical_features"), JsonParser(categories).parse(), "categorical_features");
    if (meta.at("categorical_ordered").kind != Json::Boolean || meta.at("categorical_ordered").boolean) {
        fail("ABC/RG categories must be explicitly unordered");
    }
    crossing_time_ = number(meta.at("context_contract").at("base_context_contract").at("crossing_time_seconds"), "crossing time");
    overdue_threshold_ = number(meta.at("context_contract").at("context0_overdue_threshold_seconds"), "overdue threshold");
    const std::string quantization = R"JSON({"name":"nearest-positive-node-v1","tie":"higher","minimum_node_seconds":0.001,"next_duration_key_scale":1000000000,"next_duration_tolerance_seconds":1e-9,"entry_upper_tolerance_ulps":8})JSON";
    exact_contract(meta.at("signal_quantization"), JsonParser(quantization).parse(), "signal_quantization");
    const double signal_step = number(meta.at("signal_remaining_step"), "signal_remaining_step");
    if (signal_step <= 0) { fail("ABC/RG remaining step must be positive"); }
    signal_remaining_step_ = signal_step;

    const Json& catalog = meta.at("signal_catalog");
    if (catalog.kind != Json::Array || catalog.array.empty() || catalog.array.size() > 100000) { fail("ABC/RG invalid signal catalog"); }
    const std::vector<std::string> catalog_fields = {"entry_type", "next_type", "next_duration_seconds", "duration_key", "entry_max_seconds", "nodes", "point_start", "sentinel"};
    std::tuple<char, char, std::int64_t> previous_key;
    std::map<std::pair<char, char>, double> previous_duration;
    std::uint64_t total_points = 0;
    for (std::size_t position = 0; position < catalog.array.size(); ++position) {
        const Json& item = catalog.array[position];
        if (item.kind != Json::Object || item.object.size() != catalog_fields.size()) { fail("ABC/RG wrong catalog fields"); }
        for (const std::string& field : catalog_fields) { item.at(field); }
        const std::string first = string_value(item.at("entry_type"), "entry_type");
        const std::string second = string_value(item.at("next_type"), "next_type");
        if ((first != "R" && first != "G") || (second != "R" && second != "G")) { fail("ABC/RG signal types must be R/G"); }
        SignalGroup group;
        group.entry_type = first[0]; group.next_type = second[0];
        group.next_duration = number(item.at("next_duration_seconds"), "next duration");
        group.entry_max = number(item.at("entry_max_seconds"), "entry maximum");
        if (group.next_duration < 0 || group.entry_max < 0) { fail("ABC/RG negative signal duration"); }
        const std::int64_t key = static_cast<std::int64_t>(unsigned_integer(item, "duration_key", 0, 9000000000000000ULL));
        if (key != duration_key(group.next_duration)) { fail("ABC/RG next duration key mismatch"); }
        const auto order = std::make_tuple(group.entry_type, group.next_type, key);
        if (position && order <= previous_key) { fail("ABC/RG duplicate or unordered signal key"); }
        previous_key = order;
        const auto pair = std::make_pair(group.entry_type, group.next_type);
        const auto seen = previous_duration.find(pair);
        if (seen != previous_duration.end() && group.next_duration - seen->second <= 2e-9) { fail("ABC/RG ambiguous next duration keys"); }
        previous_duration[pair] = group.next_duration;
        group.point_begin = static_cast<std::size_t>(unsigned_integer(item, "point_start", 0, MAX_ROWS));
        if (group.point_begin != total_points || item.at("sentinel").kind != Json::Boolean) { fail("ABC/RG invalid catalog point offset/sentinel"); }
        group.sentinel = item.at("sentinel").boolean;
        const Json& nodes = item.at("nodes");
        if (nodes.kind != Json::Array || nodes.array.empty() || nodes.array.size() > 100000) { fail("ABC/RG invalid remaining nodes"); }
        for (const Json& node : nodes.array) {
            const double value = number(node, "remaining node");
            if (value < 0) { fail("ABC/RG negative remaining node"); }
            group.nodes.push_back(value);
        }
        if (group.sentinel) {
            if (position != 0 || order != std::make_tuple('G', 'G', std::int64_t(0)) || group.next_duration != 0
                    || group.entry_max != 0 || group.nodes != std::vector<double>{0.0}) { fail("ABC/RG invalid G0G0 sentinel"); }
        } else if (position == 0 || group.next_duration <= 0 || group.entry_max <= 0
                   || group.nodes != remaining_nodes(group.entry_max, signal_step)) {
            fail("ABC/RG noncanonical positive remaining nodes");
        }
        if (total_points > MAX_ROWS - group.nodes.size()) { fail("ABC/RG signal-point limit exceeded"); }
        total_points += group.nodes.size();
        signal_groups_.emplace(order, position);
        signals_.push_back(std::move(group));
    }
    if (!signals_.front().sentinel) { fail("ABC/RG missing explicit no-TLS sentinel"); }
    signal_point_count_ = static_cast<std::size_t>(unsigned_integer(meta, "signal_point_count", 1, MAX_ROWS));
    if (signal_point_count_ != total_points) { fail("ABC/RG signal point count mismatch"); }

    const Json& limits = meta.at("resource_limits");
    if (limits.kind != Json::Object || limits.object.size() != 3) { fail("ABC/RG invalid resource contract"); }
    unsigned_integer(limits, "expanded_rows", MAX_ROWS, MAX_ROWS);
    unsigned_integer(limits, "payload_bytes", MAX_PAYLOAD, MAX_PAYLOAD);
    const std::uint64_t requested_max_rows = unsigned_integer(limits, "requested_max_rows", 1, MAX_ROWS);
    const Json& lengths = meta.at("road_lengths");
    if (lengths.kind != Json::Array || lengths.array.empty()) { fail("ABC/RG no road lengths"); }
    std::int64_t previous_length = -1;
    for (std::size_t index = 0; index < lengths.array.size(); ++index) {
        const double value = number(lengths.array[index], "road_length");
        const auto key = length_key(value);
        if (key <= previous_length || !close(value, key / 10000.0)) { fail("ABC/RG unordered/duplicate/non-four-decimal road length"); }
        lengths_.emplace(key, index); previous_length = key;
    }
    const Json& grid = meta.at("free_speed_cap_grid");
    cap_index_min_ = static_cast<int>(unsigned_integer(grid, "index_min", 1, std::numeric_limits<int>::max()));
    cap_index_max_ = static_cast<int>(unsigned_integer(grid, "index_max", cap_index_min_, std::numeric_limits<int>::max()));
    cap_count_ = static_cast<std::size_t>(cap_index_max_) - cap_index_min_ + 1;
    cap_step_ = number(grid.at("step"), "cap step");
    cap_min_ = number(grid.at("min"), "cap min"); cap_max_ = number(grid.at("max"), "cap max");
    number(grid, "scaled_boundary_tolerance", SCALED_EPS); number(grid, "raw_boundary_tolerance", RAW_EPS);
    if (cap_step_ <= 0 || cap_min_ <= 0 || cap_max_ < cap_min_ || !std::isfinite(cap_index_max_ * cap_step_)
            || !close(cap_min_, cap_index_min_ * cap_step_) || !close(cap_max_, cap_index_max_ * cap_step_)) { fail("ABC/RG invalid cap lattice"); }
    road_flow_max_ = static_cast<int>(unsigned_integer(meta, "road_flow_max", 0, 65535));
    lane_flow_max_ = static_cast<int>(unsigned_integer(meta, "lane_flow_max", 0, road_flow_max_));
    const std::uint64_t lane_count = static_cast<std::uint64_t>(lane_flow_max_) + 1;
    const std::uint64_t flow_count = lane_count * (static_cast<std::uint64_t>(road_flow_max_) + 1) - lane_count * lane_flow_max_ / 2;
    if (unsigned_integer(meta, "flow_count", 1, MAX_ROWS) != flow_count) { fail("ABC/RG triangular flow-count mismatch"); }
    const std::uint64_t blocks = checked_product(checked_product(checked_product(signal_point_count_, lengths_.size(), MAX_ROWS, "signal*length"),
        cap_count_, MAX_ROWS, "*cap"), 6, MAX_ROWS, "*context");
    const std::uint64_t expected_rows = checked_product(blocks, flow_count, MAX_ROWS, "rows");
    const std::uint64_t expected_groups = checked_product(blocks, lane_count, MAX_ROWS, "groups");
    expanded_rows_ = unsigned_integer(meta, "expanded_rows", 1, MAX_ROWS);
    groups_ = static_cast<std::size_t>(unsigned_integer(meta, "group_count", 1, MAX_ROWS));
    if (expanded_rows_ != expected_rows || groups_ != expected_groups
            || expanded_rows_ > requested_max_rows) { fail("ABC/RG inconsistent dimensions or explicit row budget exceeded"); }
    const bool has_policy = meta.object.count("road_flow_quantization") != 0;
    const bool has_prediction_rows = meta.object.count("prediction_rows") != 0;
    if (has_policy != has_prediction_rows || (demand && !has_policy)) {
        fail("ABC/RG incomplete road-flow quantization/prediction_rows contract");
    }
    std::uint64_t prediction_rows = expanded_rows_;
    if (has_policy) {
        const Json& policy = meta.at("road_flow_quantization");
        const std::uint64_t maximum = static_cast<std::uint64_t>(road_flow_max_);
        std::uint64_t prediction_flows = 0;
        const std::string name = string_value(policy.at("name"), "road-flow policy name");
        const std::string common_policy = "\"unit\":\"vehicles\",\"tie\":\"higher\","
            "\"grid_origin\":0,\"preserve_lane_lower_bound\":true,\"preserve_road_upper_bound\":true,"
            "\"output_coverage\":\"all-legal-integer-road-flow\",";
        if (name == "nearest-explicit-integer-road-flow-v1") {
            const Json& nodes = policy.at("nodes");
            if (nodes.kind != Json::Array || nodes.array.empty() || nodes.array.size() > maximum + 1) {
                fail("ABC/RG explicit road-flow nodes must be a bounded nonempty array");
            }
            std::string node_json = "[";
            for (const Json& entry : nodes.array) {
                const double value = number(entry, "road-flow node");
                if (!entry.integer_token || value < 0 || value > maximum || value != std::floor(value)
                        || (!road_flow_nodes_.empty() && value <= road_flow_nodes_.back())) {
                    fail("ABC/RG road-flow nodes must be strictly increasing in-domain integers");
                }
                if (!road_flow_nodes_.empty()) { node_json += ','; }
                road_flow_nodes_.push_back(static_cast<int>(value));
                node_json += std::to_string(road_flow_nodes_.back());
            }
            node_json += ']';
            if (road_flow_nodes_.front() != 0 || road_flow_nodes_.back() != road_flow_max_) {
                fail("ABC/RG explicit road-flow nodes must preserve both global endpoints");
            }
            exact_contract(policy, JsonParser("{\"name\":\"nearest-explicit-integer-road-flow-v1\","
                + common_policy + "\"nodes\":" + node_json + "}").parse(), "road_flow_quantization");
            for (int lane = 0; lane <= lane_flow_max_; ++lane) {
                const auto first = std::lower_bound(road_flow_nodes_.begin(), road_flow_nodes_.end(), lane);
                prediction_flows += static_cast<std::uint64_t>(road_flow_nodes_.end() - first);
                if (*first != lane) { ++prediction_flows; }
            }
        } else if (name == "nearest-integer-road-flow-v1") {
            road_flow_step_ = unsigned_integer(policy, "step", 1, MAX_ROWS);
            road_flow_dense_until_ = unsigned_integer(policy, "dense_until", 0, MAX_ROWS);
            exact_contract(policy, JsonParser("{\"name\":\"nearest-integer-road-flow-v1\","
                + common_policy + "\"step\":" + std::to_string(road_flow_step_)
                + ",\"dense_until\":" + std::to_string(road_flow_dense_until_) + "}").parse(), "road_flow_quantization");
            const std::uint64_t dense = std::min(road_flow_dense_until_, maximum);
            for (std::uint64_t lane = 0; lane < lane_count; ++lane) {
                if (lane <= dense) { prediction_flows += dense - lane + 1; }
                const std::uint64_t lower = std::max(lane, dense + 1);
                if (lower <= maximum) {
                    prediction_flows += maximum / road_flow_step_ - (lower - 1) / road_flow_step_;
                }
                if (lane > dense && lane % road_flow_step_ != 0) { ++prediction_flows; }
                if (maximum != lane && maximum > dense && maximum % road_flow_step_ != 0) { ++prediction_flows; }
            }
        } else {
            fail("ABC/RG unsupported road-flow quantization policy");
        }
        prediction_rows = checked_product(blocks, prediction_flows, MAX_ROWS, "prediction rows");
        if (unsigned_integer(meta, "prediction_rows", 1, MAX_ROWS) != prediction_rows
                || prediction_rows > expanded_rows_) { fail("ABC/RG prediction-row count mismatch"); }
    }
    std::uint64_t expected_bytes = 0;
    if (demand) {
        if (meta.object.count("run_count")) { fail("ABC/RG demand payload cannot declare RLE run_count"); }
        demand_entry_count_ = static_cast<std::size_t>(unsigned_integer(meta, "entry_count", 0,
                                                    std::min(prediction_rows, MAX_PAYLOAD / 16)));
        demand_scope_ = string_value(meta.at("scope_sha256"), "scope_sha256");
        if (demand_scope_.size() != 64 || demand_scope_.find_first_not_of("0123456789abcdef") != std::string::npos) {
            fail("ABC/RG invalid demand scope SHA256");
        }
        expected_bytes = 16ULL * demand_entry_count_;
    } else {
        run_count_ = static_cast<std::size_t>(unsigned_integer(meta, "run_count", groups_, expanded_rows_));
        expected_bytes = 4ULL * (groups_ + 1ULL) + 12ULL * run_count_;
        records_start_ = static_cast<std::size_t>(4ULL * (groups_ + 1ULL));
    }
    const std::uint64_t declared_bytes = unsigned_integer(meta, "payload_bytes", demand ? 0 : 1, MAX_PAYLOAD);
    if (expected_bytes != declared_bytes || expected_bytes > MAX_PAYLOAD) { fail("ABC/RG payload size or 512MiB limit mismatch"); }
    const std::string expected_sha = string_value(meta.at("payload_sha256"), "payload_sha256");
    if (expected_sha.size() != 64 || expected_sha.find_first_not_of("0123456789abcdef") != std::string::npos) { fail("ABC/RG invalid payload SHA256"); }
    payload_.resize(static_cast<std::size_t>(declared_bytes));
    CapContextSha256 hash;
    for (std::size_t offset = 0; offset < payload_.size();) {
        const std::size_t count = std::min<std::size_t>(1024 * 1024, payload_.size() - offset);
        if (!input.read(reinterpret_cast<char*>(payload_.data() + offset), static_cast<std::streamsize>(count))) { fail("ABC/RG truncated payload"); }
        hash.update(payload_.data() + offset, count);
        offset += count;
    }
    if (input.peek() != std::char_traits<char>::eof() || input.bad()) { fail("ABC/RG trailing payload or stream error"); }
    if (hash.finish() != expected_sha) { fail("ABC/RG payload SHA256 mismatch"); }
    if (demand) {
        std::uint64_t previous_key = 0;
        for (std::size_t index = 0; index < demand_entry_count_; ++index) {
            const unsigned char* record = payload_.data() + 16ULL * index;
            const std::uint32_t group = little_u32(record), road = little_u32(record + 4);
            const std::uint64_t key = (std::uint64_t(group) << 32) | road;
            const double value = little_f64(record + 8);
            if (group >= groups_ || road > static_cast<unsigned>(road_flow_max_)) { fail("ABC/RG demand key outside domain"); }
            const int lane = static_cast<int>(group % lane_count);
            if (road < static_cast<unsigned>(lane)
                    || representative_road_flow(static_cast<int>(road), lane) != road) {
                fail("ABC/RG demand road key is not a legal representative node");
            }
            if ((index && key <= previous_key) || !std::isfinite(value) || value < 1) {
                fail("ABC/RG duplicate/unsorted demand key or invalid prediction");
            }
            previous_key = key;
        }
        return;
    }
    std::uint32_t last_offset = little_u32(payload_.data());
    if (last_offset != 0) { fail("ABC/RG first group offset is not zero"); }
    std::uint64_t decoded_rows = 0;
    for (std::size_t group = 0; group < groups_; ++group) {
        const std::uint32_t end = little_u32(payload_.data() + 4 * (group + 1));
        if (end <= last_offset || end > run_count_) { fail("ABC/RG invalid group offset"); }
        unsigned expected_low = static_cast<unsigned>(group % lane_count);
        double previous_value = 0;
        for (std::uint32_t run = last_offset; run < end; ++run) {
            const unsigned char* record = payload_.data() + records_start_ + 12ULL * run;
            const unsigned low = little_u16(record), high = little_u16(record + 2);
            const double value = little_f64(record + 4);
            if (low != expected_low || high < low || high > static_cast<unsigned>(road_flow_max_)) { fail("ABC/RG run bounds gap/overlap/outside flow range"); }
            if (!std::isfinite(value) || value < 1 || (run > last_offset && value == previous_value)) { fail("ABC/RG invalid/noncanonical prediction run"); }
            const std::uint64_t count = high - low + 1ULL;
            if (count > expanded_rows_ - decoded_rows) { fail("ABC/RG decoded row count overflow"); }
            decoded_rows += count; expected_low = high + 1; previous_value = value;
        }
        if (expected_low != static_cast<unsigned>(road_flow_max_) + 1) { fail("ABC/RG incomplete group coverage"); }
        last_offset = end;
    }
    if (last_offset != run_count_ || decoded_rows != expanded_rows_) { fail("ABC/RG final offset/decoded count mismatch"); }
}

std::size_t CapContextABCRGTable::signal_point(const SignalRGFeatures& raw, SignalRGFeatures* quantized) const {
    if (!expanded_rows_ || (!demand_mode_ && payload_.empty())) { fail("ABC/RG table is not loaded"); }
    if ((raw.entry_type != 'G' && raw.entry_type != 'R') || (raw.next_type != 'G' && raw.next_type != 'R')
            || !std::isfinite(raw.entry_remaining_seconds) || raw.entry_remaining_seconds < 0) { fail("ABC/RG invalid raw signal"); }
    const auto key = std::make_tuple(raw.entry_type, raw.next_type, duration_key(raw.next_duration_seconds));
    const auto found = signal_groups_.find(key);
    if (found == signal_groups_.end()) { fail("ABC/RG unknown signal combination; no fallback"); }
    const SignalGroup& group = signals_[found->second];
    if (std::abs(raw.next_duration_seconds - group.next_duration) > NEXT_EPS) { fail("ABC/RG next duration mismatch"); }
    std::size_t index = 0;
    if (group.sentinel) {
        if (raw.entry_remaining_seconds != 0 || raw.next_duration_seconds != 0) { fail("ABC/RG G0G0 sentinel mismatch"); }
    } else {
        double value = raw.entry_remaining_seconds;
        if (value > group.entry_max) {
            const double ulp = std::nextafter(group.entry_max, std::numeric_limits<double>::infinity()) - group.entry_max;
            // Only absorb floating-point summation noise, never a fraction of
            // the catch step. Difference comparison avoids rounding max+8ULP.
            if (value - group.entry_max <= 8.0 * ulp) { value = group.entry_max; }
        }
        if (value <= 0 || value > group.entry_max) { fail("ABC/RG remaining outside catalog domain; no extrapolation"); }
        const auto high = std::lower_bound(group.nodes.begin(), group.nodes.end(), value);
        if (high == group.nodes.end()) { fail("ABC/RG remaining exceeds last node"); }
        index = static_cast<std::size_t>(high - group.nodes.begin());
        if (index && value - group.nodes[index - 1] < group.nodes[index] - value) { --index; }
    }
    if (quantized) {
        *quantized = raw;
        quantized->entry_remaining_seconds = group.nodes[index];
        quantized->next_duration_seconds = group.next_duration;
    }
    return group.point_begin + index;
}

SignalRGFeatures CapContextABCRGTable::quantizeSignal(const SignalRGFeatures& signal) const {
    SignalRGFeatures result;
    signal_point(signal, &result);
    return result;
}

std::uint32_t CapContextABCRGTable::query_group(double road_length, int road_flow, int lane_flow,
                                              double raw_cap, int context, const SignalRGFeatures& signal) const {
    const std::size_t point = signal_point(signal, nullptr);
    if (road_flow < 0 || road_flow > road_flow_max_ || lane_flow < 0 || lane_flow > lane_flow_max_
            || lane_flow > road_flow || context < 0 || context >= 6) { fail("ABC/RG flow/context outside legal domain"); }
    if (!std::isfinite(raw_cap) || raw_cap <= 0 || raw_cap < cap_min_ - RAW_EPS || raw_cap > cap_max_ + RAW_EPS) { fail("ABC/RG cap outside table domain"); }
    const double rounded = std::floor(raw_cap / cap_step_ + 0.5 + SCALED_EPS);
    if (!std::isfinite(rounded) || rounded < cap_index_min_ || rounded > cap_index_max_) { fail("ABC/RG cap index outside table domain"); }
    const auto length = lengths_.find(length_key(road_length));
    if (length == lengths_.end()) { fail("ABC/RG missing physical road length"); }
    const std::size_t cap = static_cast<std::size_t>(rounded - cap_index_min_);
    const std::size_t group = ((((point * lengths_.size() + length->second) * cap_count_ + cap) * 6
                                + static_cast<std::size_t>(context)) * (static_cast<std::size_t>(lane_flow_max_) + 1))
                              + static_cast<std::size_t>(lane_flow);
    if (group >= groups_) { fail("ABC/RG internal group index out of bounds"); }
    return static_cast<std::uint32_t>(group);
}

std::uint32_t CapContextABCRGTable::representative_road_flow(int road_flow, int lane_flow) const {
    if (!road_flow_nodes_.empty()) {
        // query_group already checked lane <= road <= maximum. The global
        // maximum is a node, so lower_bound always yields a valid upper node.
        const auto high = std::lower_bound(road_flow_nodes_.begin(), road_flow_nodes_.end(), road_flow);
        int low = lane_flow;
        if (high != road_flow_nodes_.begin()) { low = std::max(low, *(high - 1)); }
        return static_cast<std::uint32_t>(road_flow - low < *high - road_flow ? low : *high);
    }
    const std::uint64_t road = static_cast<std::uint64_t>(road_flow);
    const std::uint64_t lane = static_cast<std::uint64_t>(lane_flow);
    const std::uint64_t maximum = static_cast<std::uint64_t>(road_flow_max_);
    if (road <= road_flow_dense_until_) { return static_cast<std::uint32_t>(road); }
    // Only endpoint/prefix boundary and neighbouring lattice nodes can win.
    // Arithmetic is uint64 because configurable step may exceed INT_MAX.
    const std::uint64_t lower = (road / road_flow_step_) * road_flow_step_;
    const std::uint64_t candidates[] = {lane, maximum, std::min(road_flow_dense_until_, maximum),
                                        lower, lower + road_flow_step_};
    std::uint64_t best = lane, best_distance = road - lane;
    for (const std::uint64_t node : candidates) {
        if (node < lane || node > maximum) { continue; }
        const std::uint64_t distance = node > road ? node - road : road - node;
        if (distance < best_distance || (distance == best_distance && node > best)) {
            best = node; best_distance = distance;
        }
    }
    return static_cast<std::uint32_t>(best);
}

bool CapContextABCRGTable::lookupDemand(double road_length, int road_flow, int lane_flow,
                                       double raw_cap, int context, const SignalRGFeatures& signal,
                                       double& value, std::uint32_t& group, std::uint32_t& representative) const {
    if (!demand_mode_) { fail("ABC/RG demand lookup requires a loaded demand cache"); }
    group = query_group(road_length, road_flow, lane_flow, raw_cap, context, signal);
    representative = representative_road_flow(road_flow, lane_flow);
    const std::uint64_t wanted = (std::uint64_t(group) << 32) | representative;
    std::size_t low = 0, high = demand_entry_count_;
    while (low < high) {
        const std::size_t middle = low + (high - low) / 2;
        const unsigned char* record = payload_.data() + 16ULL * middle;
        const std::uint64_t key = (std::uint64_t(little_u32(record)) << 32) | little_u32(record + 4);
        if (key < wanted) { low = middle + 1; }
        else { high = middle; }
    }
    if (low == demand_entry_count_) { return false; }
    const unsigned char* record = payload_.data() + 16ULL * low;
    if (little_u32(record) != group || little_u32(record + 4) != representative) { return false; }
    value = little_f64(record + 8);
    return true;
}

double CapContextABCRGTable::lookup(double road_length, int road_flow, int lane_flow,
                                   double raw_cap, int context, const SignalRGFeatures& signal) const {
    if (demand_mode_) {
        double value = 0;
        std::uint32_t group = 0, representative = 0;
        if (!lookupDemand(road_length, road_flow, lane_flow, raw_cap, context, signal, value, group, representative)) {
            fail("ABC/RG demand cache missing key group=" + std::to_string(group)
                 + " road=" + std::to_string(representative) + "; strict lookup has no fallback");
        }
        return value;
    }
    const std::uint32_t group = query_group(road_length, road_flow, lane_flow, raw_cap, context, signal);
    std::uint32_t low = little_u32(payload_.data() + 4 * group);
    const std::uint32_t end = little_u32(payload_.data() + 4 * (group + 1));
    std::uint32_t high = end;
    while (low < high) {
        const std::uint32_t middle = low + (high - low) / 2;
        const unsigned char* record = payload_.data() + records_start_ + 12ULL * middle;
        if (little_u16(record + 2) < static_cast<unsigned>(road_flow)) { low = middle + 1; }
        else { high = middle; }
    }
    if (low == end) { fail("ABC/RG internal uncovered flow query"); }
    const unsigned char* record = payload_.data() + records_start_ + 12ULL * low;
    if (little_u16(record) > static_cast<unsigned>(road_flow)) { fail("ABC/RG internal flow gap"); }
    return little_f64(record + 4);
}
