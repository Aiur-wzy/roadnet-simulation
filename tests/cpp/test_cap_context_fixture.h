#ifndef CAMS_TEST_CAP_CONTEXT_FIXTURE_H
#define CAMS_TEST_CAP_CONTEXT_FIXTURE_H
#include "cap_context_sha256.h"
#include <cstring>
#include <stdexcept>
#include <string>

namespace cap_context_test {
inline std::string replace_once(std::string text, const std::string& from, const std::string& to) {
    const auto where = text.find(from);
    if (where == std::string::npos) throw std::runtime_error("bad fixture replacement: " + from);
    text.replace(where, from.size(), to); return text;
}
inline void append_integer(std::string& bytes, std::uint64_t value, unsigned size) {
    for (unsigned i=0; i<size; ++i) bytes += static_cast<char>(value >> (i*8));
}
inline void append_double(std::string& bytes, double value) {
    std::uint64_t bits; std::memcpy(&bits, &value, 8); append_integer(bytes, bits, 8);
}
inline std::string digest(const std::string& bytes) {
    CapContextSha256 sha;
    sha.update(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size());
    return sha.finish();
}
inline std::string payload() {
    std::string bytes;
    for (unsigned group=0; group<=48; ++group) append_integer(bytes, group, 4);
    for (unsigned group=0; group<48; ++group) {
        append_integer(bytes, group % 3, 2); append_integer(bytes, 3, 2);
        append_double(bytes, group + 1.0);
    }
    return bytes;
}
inline std::string metadata(const std::string& bytes = payload()) {
    return std::string(R"JSON({"format":"CAMS_CAP_CONTEXT_RLE_V1","training_schema":"road-time-cap-context-v1","columns":["road_length","road_flow","lane_flow","free_speed_cap","platoon_context","travel_time_no_waiting"],"source_label":"travel_time_label_gt_0p1","road_lengths":[100,320],"static_key_scale":10000,"free_speed_cap_grid":{"index_min":1,"index_max":2,"step":5,"min":5,"max":10,"scaled_boundary_tolerance":1e-9,"raw_boundary_tolerance":1e-12},"context_contract":{"version":"free-clock-context-v1","p_threshold":0.05,"e_threshold":2.0,"epsilon_seconds":1e-9,"crossing_time_seconds":3,"sampling":"upstream-discharge-known-reservations-v1","geometry":"base-first-v1","candidate_lane":"incoming-logical-lane-v1"},"context_count":4,"road_flow_max":3,"lane_flow_max":2,"flow_count":9,"expanded_rows":144,"group_count":48,"run_count":48,"prediction_floor_seconds":1,"payload_layout":"offsets_u32_le+records_u16_u16_f64_le-v1","payload_bytes":772,"payload_sha256":")JSON")
        + digest(bytes) + "\"}";
}
inline std::string fixture() { const auto bytes = payload(); return metadata(bytes) + "\n" + bytes; }
}
#endif
