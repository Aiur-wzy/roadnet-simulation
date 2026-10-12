#ifndef ROADNET_TEST_NATIVE_TABLE_FIXTURE_H
#define ROADNET_TEST_NATIVE_TABLE_FIXTURE_H
#include "native_road_time_table.h"
#include "cap_context_json.h"
#include "cap_context_sha256.h"
#include <cstring>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace native_test {
using cap_context_json::Json;
using cap_context_json::JsonParser;
inline void check(bool okay, const std::string& why) { if (!okay) throw std::runtime_error(why); }
inline void rejects(const std::function<void()>& action, const std::string& why) {
    bool rejected = false;
    try { action(); } catch (const std::exception&) { rejected = true; }
    check(rejected, why);
}
inline std::string quote(const std::string& value) {
    std::string text = "\"";
    for (char character : value) {
        if (character == '\\' || character == '"') text += '\\';
        text += character;
    }
    return text + '"';
}
inline std::string serialize(const Json& value) {
    if (value.kind == Json::Null) return "null";
    if (value.kind == Json::Boolean) return value.boolean ? "true" : "false";
    if (value.kind == Json::String) return quote(value.string);
    if (value.kind == Json::Number) {
        std::ostringstream out;
        if (value.integer_token) out << std::fixed << std::setprecision(0) << value.number;
        else out << std::setprecision(17) << value.number;
        auto text = out.str();
        if (!value.integer_token && text.find_first_of(".eE") == std::string::npos) text += ".0";
        return text;
    }
    std::string text;
    if (value.kind == Json::Array) {
        text = "[";
        for (const auto& item : value.array) { if (text.size() > 1) text += ','; text += serialize(item); }
        return text + ']';
    }
    text = "{";
    for (const auto& item : value.object) { if (text.size() > 1) text += ','; text += quote(item.first) + ':' + serialize(item.second); }
    return text + '}';
}
inline void set_number(Json& value, const std::string& key, std::uint64_t number) {
    value.object[key] = JsonParser(std::to_string(number)).parse();
}
inline void set_string(Json& value, const std::string& key, const std::string& text) {
    value.object[key] = JsonParser(quote(text)).parse();
}
inline Json header(std::uint64_t caps = 2, bool signals = false) {
    Json meta = JsonParser(R"JSON({
"format":"CAMS_NATIVE_ABC_RG_DEMAND_V2","payload_layout":"sorted-u64-u32-f64-v2",
"training_schema":"road-time-native-model-v2","source_schema":"road-time-sumo-native-euler-streaming-quality-v2",
"columns":["road_length","road_flow","lane_flow","free_speed_cap","platoon_context","entry_signal_type","entry_signal_remaining_seconds","next_signal_type","next_signal_duration_seconds","road_time_seconds"],
"source_label":"travel_time_label_gt_1",
"model_contract":{"schema":"road-time-native-model-v2","status":"COMPLETE","source_schema":"road-time-sumo-native-euler-streaming-quality-v2","source_label_column":"travel_time_label_gt_1","model_label":"road_time_seconds","predictor_directory":"predictor","feature_columns":["road_length","road_flow","lane_flow","free_speed_cap","platoon_context","entry_signal_type","entry_signal_remaining_seconds","next_signal_type","next_signal_duration_seconds"],"categorical":{"platoon_context":[0,1,2,3,4,5],"entry_signal_type":["R","G"],"next_signal_type":["R","G"]},"collection_contract":{"schema":"road-time-sumo-native-euler-streaming-quality-v2","clock_schema":"sumo-euler-last-completed-snapshot-v2"}},
"deployment_contract":{"version":"sumo-native-training-to-cpp-proxy-v2","training_inputs":"sumo-native-first-visible-snapshot","query_inputs":"cpp-entry-decision-proxy","equivalent":false,"abc_algorithm":"entry-frozen-free-clock-six-contexts","crossing_time_seconds":3.0,"overdue_threshold_seconds":0.0,"crossing_policy":"first-or-known-unsignalized-or-no-internal-zero"},
"model":{"contract_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"},
"network":{"sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","external_edges":1,"without_internal_links":true},
"context_count":6,"static_key_scale":10000,"prediction_floor_seconds":1.0,
"categorical_features":{"platoon_context":[0,1,2,3,4,5],"entry_signal_type":["R","G"],"next_signal_type":["R","G"]},"categorical_ordered":false,
"signal_quantization":{"name":"nearest-positive-node-v1","tie":"higher","minimum_node_seconds":0.001,"next_duration_key_scale":1000000000,"next_duration_tolerance_seconds":1e-9,"entry_upper_tolerance_ulps":8},
"signal_remaining_step":5,"signal_catalog":[{"entry_type":"G","next_type":"G","next_duration_seconds":0,"duration_key":0,"entry_max_seconds":0,"nodes":[0],"point_start":0,"sentinel":true}],
"resource_limits":{"max_payload_bytes":536870912,"max_json_integer":9007199254740991},
"road_lengths":[100],"free_speed_cap_grid":{"index_min":1,"index_max":2,"step":1.0,"min":1.0,"max":2.0,"raw_boundary_tolerance":1e-12,"scaled_boundary_tolerance":1e-9},
"road_flow_max":10,"lane_flow_max":2,"flow_count":30,
"road_flow_quantization":{"name":"nearest-integer-road-flow-v1","unit":"vehicles","tie":"higher","grid_origin":0,"preserve_lane_lower_bound":true,"preserve_road_upper_bound":true,"output_coverage":"all-legal-integer-road-flow","step":5,"dense_until":0},
"scope_sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
})JSON").parse();
    if (signals) meta.object["signal_catalog"].array.push_back(JsonParser(R"JSON({"entry_type":"G","next_type":"R","next_duration_seconds":30,"duration_key":30000000000,"entry_max_seconds":10,"nodes":[0.001,5,10],"point_start":1,"sentinel":false})JSON").parse());
    const auto points = signals ? 4ULL : 1ULL;
    set_number(meta, "signal_point_count", points);
    set_number(meta, "group_count", points * caps * 6 * 3);
    set_number(meta, "expanded_rows", points * caps * 6 * 30);
    set_number(meta.object["free_speed_cap_grid"], "index_max", caps);
    set_number(meta.object["free_speed_cap_grid"], "max", caps);
    return meta;
}
using Entry = std::tuple<std::uint64_t, std::uint32_t, double>;
inline Json query_policy(Json meta, bool correction = true, bool clamp = true) {
    auto& policy = meta.object["deployment_contract"];
    set_string(policy, "version", "sumo-native-training-to-cpp-proxy-v3");
    set_string(policy, "first_road_position_correction", correction ? "on" : "off");
    set_string(policy, "cap_out_of_range", clamp ? "clamp" : "error");
    return meta;
}
inline std::string table(Json meta, const std::vector<Entry>& entries) {
    std::string payload;
    for (const auto& item : entries) {
        const auto group = std::get<0>(item);
        const auto road = std::get<1>(item);
        const auto value = std::get<2>(item);
        std::uint64_t bits;
        std::memcpy(&bits, &value, 8);
        for (int i = 0; i < 8; ++i) payload += static_cast<char>(group >> (8 * i));
        for (int i = 0; i < 4; ++i) payload += static_cast<char>(road >> (8 * i));
        for (int i = 0; i < 8; ++i) payload += static_cast<char>(bits >> (8 * i));
    }
    CapContextSha256 hash;
    hash.update(reinterpret_cast<const unsigned char*>(payload.data()), payload.size());
    set_number(meta, "entry_count", entries.size());
    set_number(meta, "payload_bytes", payload.size());
    set_string(meta, "payload_sha256", hash.finish());
    return "CAMS_NATIVE_ABC_RG_DEMAND_V2\n" + serialize(meta) + '\n' + payload;
}
inline void load(NativeRoadTimeTable& reader, const std::string& bytes) {
    std::istringstream in(bytes, std::ios::binary);
    std::string magic;
    std::getline(in, magic);
    check(magic == "CAMS_NATIVE_ABC_RG_DEMAND_V2", "fixture magic");
    reader.load(in);
}
} // namespace native_test
#endif
