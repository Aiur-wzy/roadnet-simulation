#ifndef CAMS_TEST_CAP_CONTEXT_ABC_FIXTURE_H
#define CAMS_TEST_CAP_CONTEXT_ABC_FIXTURE_H
#include "test_cap_context_fixture.h"

namespace cap_context_abc_test {
using cap_context_test::append_integer;
using cap_context_test::append_double;
using cap_context_test::digest;
using cap_context_test::replace_once;

inline std::string contract() {
    return R"JSON({"version":"free-clock-context-abc-v1","base_context_contract":{"version":"free-clock-context-v1","p_threshold":0.05,"e_threshold":2.0,"epsilon_seconds":1e-9,"crossing_time_seconds":3,"sampling":"upstream-discharge-known-reservations-v1","geometry":"base-first-v1","candidate_lane":"incoming-logical-lane-v1"},"context_ids":[0,1,2,3,4,5],"context_mapping":{"0":"A","1":"legacy_1","2":"legacy_2","3":"legacy_3","4":"B","5":"C"},"context0_overdue_threshold_seconds":0.0,"overdue_epsilon_seconds":1e-9,"overdue_formula":"max(0, decision_time - candidate.free_finish_time)","overdue_aggregation_population":"same eligible non-ego logical-lane records as base query; planned_enter_time <= decision_time","context0_subclass_rules":{"A":"legacy_context == 0 and eligible_count == 0","B":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds <= threshold + epsilon","C":"legacy_context == 0 and eligible_count > 0 and max_overdue_seconds > threshold + epsilon","not_context0":"legacy_context in (1, 2, 3); preserve original context ID"},"overdue_count_rule":"count(overdue_seconds > epsilon); independent of classification threshold","overdue_fraction_denominator":"eligible_count; zero if eligible_count == 0","mean_overdue_denominator":"eligible_count; zero if eligible_count == 0","continuous_audit_scope":"record all contexts, including legacy 1/2/3; do not clip at threshold","interpretation":"logical free-clock overrun proxy, not physical waiting or measured queue time"})JSON";
}
inline std::string payload() {
    std::string bytes;
    for (unsigned group = 0; group <= 96; ++group) append_integer(bytes, group, 4);
    for (unsigned group = 0; group < 96; ++group) {
        append_integer(bytes, group % 4, 2); append_integer(bytes, 5, 2);
        append_double(bytes, group + 1.0);
    }
    return bytes;
}
inline std::string metadata(const std::string& bytes = payload()) {
    return std::string(R"JSON({"format":"CAMS_CAP_CONTEXT_ABC_RLE_V1","training_schema":"road-time-cap-context-abc-v1","columns":["road_length","road_flow","lane_flow","free_speed_cap","platoon_context","travel_time_no_waiting"],"source_label":"travel_time_label_gt_0p1","road_lengths":[100,320],"static_key_scale":10000,"free_speed_cap_grid":{"index_min":1,"index_max":2,"step":5,"min":5,"max":10,"scaled_boundary_tolerance":1e-9,"raw_boundary_tolerance":1e-12},"context_contract":)JSON")
        + contract()
        + R"JSON(,"context_count":6,"categorical_features":{"platoon_context":[0,1,2,3,4,5]},"categorical_ordered":false,"road_flow_max":5,"lane_flow_max":3,"flow_count":18,"expanded_rows":432,"group_count":96,"run_count":96,"prediction_floor_seconds":1,"payload_layout":"offsets_u32_le+records_u16_u16_f64_le-v1","payload_bytes":1540,"payload_sha256":")JSON"
        + digest(bytes) + "\"}";
}
// Exactly like the legacy fixture, the stream begins AFTER the magic line.
inline std::string fixture() { const auto bytes = payload(); return metadata(bytes) + "\n" + bytes; }
}
#endif
