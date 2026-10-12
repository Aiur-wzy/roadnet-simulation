#include "cap_context_table.h"
#include "cap_context_json.h"
#include "test_cap_context_abc_fixture.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>

namespace {
using namespace cap_context_abc_test;
int checks = 0;
void check(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& action, const std::string& message) {
    ++checks;
    try { action(); } catch (const std::runtime_error&) { return; }
    throw std::runtime_error("expected rejection: " + message);
}
void load(CapContextTable& table, const std::string& contents, bool abc = true) {
    std::istringstream input(contents, std::ios::binary); table.load(input, abc);
}
void bad(const std::string& contents, const std::string& message) {
    CapContextTable table; load(table, fixture());
    rejects([&] { load(table, contents); }, message);
    check(table.size() == 0 && table.label().empty() && !table.isABC() &&
          table.contextCount() == 0 && table.overdueThreshold() == 0,
          message + " retained partial state");
}
void unit_tests() {
    CapContextTable table; load(table, fixture());
    check(table.isABC() && table.contextCount() == 6 && table.overdueThreshold() == 0,
          "ABC runtime contract");
    check(table.size() == 432, "expanded ABC count");
    std::size_t dense = 0;
    for (int length = 0; length < 2; ++length) for (int cap = 0; cap < 2; ++cap)
        for (int context = 0; context < 6; ++context) for (int lane = 0; lane <= 3; ++lane)
            for (int road = lane; road <= 5; ++road) {
                const double expected = (((length * 2 + cap) * 6 + context) * 4 + lane) + 1;
                check(table.lookup(length ? 320 : 100, road, lane, (cap + 1) * 5, context) == expected,
                      "ABC lookup coordinate");
                check(table.denseValues().at(dense++) == expected, "ABC dense order");
            }
    check(table.lookup(100, 0, 0, 7.5, 5) == 45, "ABC cap midpoint ties upward");
    check(table.lookup(100, 0, 0, 7.499999, 5) == 21, "ABC below cap midpoint");
    for (int context : {-1, 6, 7})
        rejects([&] { table.lookup(100, 0, 0, 5, context); }, "illegal ABC ID");
    rejects([&] { table.lookup(100, 2, 3, 5, 5); }, "lane exceeds road");
    rejects([&] { table.lookup(100, 6, 0, 5, 5); }, "road out of domain");
    rejects([&] { table.lookup(100, 5, 4, 5, 5); }, "lane out of domain");
    rejects([&] { table.lookup(101, 0, 0, 5, 5); }, "missing length");
    for (double cap : {0.0, -1.0, 4.9, 10.1, std::numeric_limits<double>::quiet_NaN()})
        rejects([&] { table.lookup(100, 0, 0, cap, 5); }, "cap out of domain");
    table.requireCrossingTime(3);
    rejects([&] { table.requireCrossingTime(2); }, "crossing mismatch");

    // Explicit magic dispatch cannot reinterpret either model as the other.
    rejects([&] { load(table, fixture(), false); }, "old reader rejects ABC");
    rejects([&] { load(table, cap_context_test::fixture(), true); }, "ABC rejects old reader");
    load(table, cap_context_test::fixture(), false);
    check(!table.isABC() && table.contextCount() == 4 && table.overdueThreshold() == 0,
          "legacy runtime contract after reload");
    rejects([&] { table.lookup(100, 0, 0, 5, 4); }, "legacy still rejects ABC B");

    auto configurable = replace_once(fixture(), "\"context0_overdue_threshold_seconds\":0.0",
                                    "\"context0_overdue_threshold_seconds\":2.5");
    configurable = replace_once(configurable, "\"crossing_time_seconds\":3", "\"crossing_time_seconds\":2.5");
    load(table, configurable);
    check(table.overdueThreshold() == 2.5, "configurable threshold retained");
    table.requireCrossingTime(2.5);
    // Floats are allowed for float-valued contract fields, but not categorical IDs.
    load(table, replace_once(fixture(), "\"context0_overdue_threshold_seconds\":0.0",
                             "\"context0_overdue_threshold_seconds\":0"));
    const std::vector<std::pair<std::string, std::string>> mutations = {
        {"CAMS_CAP_CONTEXT_ABC_RLE_V1", "CAMS_CAP_CONTEXT_RLE_V1"},
        {"road-time-cap-context-abc-v1", "road-time-cap-context-v1"},
        {"\"context_count\":6", "\"context_count\":4"},
        {"\"context_count\":6", "\"context_count\":6.0"},
        {"\"context_count\":6", "\"context_count\":6e0"},
        {"\"context_count\":6", "\"context_count\":true"},
        {"\"context_count\":6", "\"context_count\":\"6\""},
        {"\"categorical_ordered\":false", "\"categorical_ordered\":true"},
        {"\"categorical_ordered\":false", "\"categorical_ordered\":0"},
        {"\"categorical_features\":{\"platoon_context\":[0,1,2,3,4,5]}",
         "\"categorical_features\":{\"platoon_context\":[0,1,2,3,5,4]}"},
        {"\"categorical_features\":{\"platoon_context\":[0,1,2,3,4,5]}",
         "\"categorical_features\":{\"platoon_context\":[0.0,1,2,3,4,5]}"},
        {"\"categorical_features\":{\"platoon_context\":[0,1,2,3,4,5]}",
         "\"categorical_features\":{\"platoon_context\":[false,1,2,3,4,5]}"},
        {"free-clock-context-abc-v1", "free-clock-context-abc-v2"},
        {"free-clock-context-v1", "free-clock-context-v2"},
        {"\"context_ids\":[0,1,2,3,4,5]", "\"context_ids\":[0,1,2,3,4]"},
        {"\"context_ids\":[0,1,2,3,4,5]", "\"context_ids\":[0,1,2,3,5,4]"},
        {"\"context_ids\":[0,1,2,3,4,5]", "\"context_ids\":[0.0,1,2,3,4,5]"},
        {"\"context_ids\":[0,1,2,3,4,5]", "\"context_ids\":[false,1,2,3,4,5]"},
        {"\"4\":\"B\",\"5\":\"C\"", "\"4\":\"C\",\"5\":\"B\""},
        {"\"0\":\"A\"", "\"0\":0"},
        {"\"context0_overdue_threshold_seconds\":0.0", "\"context0_overdue_threshold_seconds\":-1"},
        {"\"context0_overdue_threshold_seconds\":0.0", "\"context0_overdue_threshold_seconds\":false"},
        {"\"context0_overdue_threshold_seconds\":0.0", "\"context0_overdue_threshold_seconds\":\"0\""},
        {"\"overdue_epsilon_seconds\":1e-9", "\"overdue_epsilon_seconds\":1e-8"},
        {"max(0, decision_time - candidate.free_finish_time)", "max(0, planned_enter_time - candidate.free_finish_time)"},
        {"planned_enter_time <= decision_time", "planned_enter_time <= decision_time + 3"},
        {"legacy_context == 0 and eligible_count == 0", "legacy_context == 0"},
        {"max_overdue_seconds <= threshold + epsilon", "max_overdue_seconds < threshold + epsilon"},
        {"max_overdue_seconds > threshold + epsilon", "max_overdue_seconds >= threshold + epsilon"},
        {"legacy_context in (1, 2, 3); preserve original context ID", "renumber all classes"},
        {"count(overdue_seconds > epsilon); independent of classification threshold", "count(overdue_seconds > threshold)"},
        {"\"overdue_fraction_denominator\":\"eligible_count; zero if eligible_count == 0\"", "\"overdue_fraction_denominator\":\"positive_count\""},
        {"\"mean_overdue_denominator\":\"eligible_count; zero if eligible_count == 0\"", "\"mean_overdue_denominator\":\"overdue_count\""},
        {"record all contexts, including legacy 1/2/3; do not clip at threshold", "record only context zero"},
        {"logical free-clock overrun proxy, not physical waiting or measured queue time", "physical waiting time"},
        {"\"p_threshold\":0.05", "\"p_threshold\":0.1"},
        {"\"e_threshold\":2.0", "\"e_threshold\":3.0"},
        {"\"epsilon_seconds\":1e-9", "\"epsilon_seconds\":1e-8"},
        {"\"crossing_time_seconds\":3", "\"crossing_time_seconds\":-1"},
        {"\"crossing_time_seconds\":3", "\"crossing_time_seconds\":true"},
        {"base-first-v1", "full-first"},
        {"incoming-logical-lane-v1", "physical-lane"},
        {"upstream-discharge-known-reservations-v1", "future-enter"},
        {"\"context_contract\":{", "\"context_contract\":{\"extra\":1,"},
        {"\"base_context_contract\":{", "\"base_context_contract\":{\"extra\":1,"},
        {"\"context_mapping\":{", "\"context_mapping\":{\"extra\":1,"},
        {"\"context0_subclass_rules\":{", "\"context0_subclass_rules\":{\"extra\":1,"},
        {"\"context_ids\":[0,1,2,3,4,5],", ""},
        {"\"road_flow_max\":5", "\"road_flow_max\":65536"},
        {"\"lane_flow_max\":3", "\"lane_flow_max\":6"},
        {"\"flow_count\":18", "\"flow_count\":17"},
        {"\"expanded_rows\":432", "\"expanded_rows\":431"},
        {"\"group_count\":96", "\"group_count\":95"},
        {"\"run_count\":96", "\"run_count\":95"},
        {"\"payload_bytes\":1540", "\"payload_bytes\":1539"}
    };
    for (const auto& mutation : mutations)
        bad(replace_once(fixture(), mutation.first, mutation.second), mutation.first);
    bad(fixture() + "x", "trailing payload");
    bad(fixture().substr(0, fixture().size() - 1), "truncated payload");
    std::string corrupt = fixture(); corrupt.back() ^= 1; bad(corrupt, "payload SHA");
    std::string bytes = payload(); bytes[4] = 0;
    bad(metadata(bytes) + "\n" + bytes, "empty group");
    bytes = payload(); bytes[97 * 4] = 1;
    bad(metadata(bytes) + "\n" + bytes, "run coverage gap");
    bytes = payload(); bytes[97 * 4 + 2] = 6;
    bad(metadata(bytes) + "\n" + bytes, "run beyond flow bound");
    bytes = payload(); bytes.replace(97 * 4 + 4, 8, std::string(8, '\0'));
    bad(metadata(bytes) + "\n" + bytes, "zero prediction");
    load(table, fixture()); table.clear();
    rejects([&] { table.lookup(100, 0, 0, 5, 0); }, "cleared lookup");
}

void audit_file(const std::string& path, const std::string& reference_path) {
    std::ifstream input(path, std::ios::binary);
    std::string magic, header;
    if (!std::getline(input, magic) || magic != "CAMS_CAP_CONTEXT_ABC_RLE_V1")
        throw std::runtime_error("bad ABC magic");
    const auto position = input.tellg(); std::getline(input, header); input.seekg(position);
    CapContextTable table; table.load(input, true);
    const auto meta = cap_context_json::JsonParser(header).parse();
    const auto& grid = meta.at("free_speed_cap_grid");
    std::ifstream reference;
    if (!reference_path.empty()) {
        reference.open(reference_path, std::ios::binary);
        if (!reference) throw std::runtime_error("cannot open dense reference");
    }
    CapContextSha256 dense_sha, lookup_sha;
    std::size_t count = 0;
    const int lo = static_cast<int>(grid.at("index_min").number);
    const int hi = static_cast<int>(grid.at("index_max").number);
    for (const auto& length : meta.at("road_lengths").array) for (std::int64_t cap = lo; cap <= hi; ++cap)
        for (int context = 0; context < table.contextCount(); ++context)
            for (int lane = 0; lane <= meta.at("lane_flow_max").number; ++lane)
                for (int road = lane; road <= meta.at("road_flow_max").number; ++road) {
                    const double value = table.lookup(length.number, road, lane, cap * grid.at("step").number, context);
                    const double dense = table.denseValues().at(count++);
                    if (std::memcmp(&value, &dense, 8) != 0) throw std::runtime_error("lookup/dense bit mismatch");
                    std::string bytes; append_double(bytes, value);
                    lookup_sha.update(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size());
                    std::string dense_bytes; append_double(dense_bytes, dense);
                    dense_sha.update(reinterpret_cast<const unsigned char*>(dense_bytes.data()), dense_bytes.size());
                    if (!reference_path.empty()) {
                        char expected[8];
                        if (!reference.read(expected, 8)) throw std::runtime_error("truncated dense reference");
                        if (std::memcmp(expected, bytes.data(), 8) != 0)
                            throw std::runtime_error("reference bit mismatch at row " + std::to_string(count - 1));
                    }
                }
    if (!reference_path.empty() && (reference.peek() != std::char_traits<char>::eof() || reference.bad()))
        throw std::runtime_error("dense reference trailing bytes or read error");
    std::cout << "{\"rows\":" << count << ",\"contexts\":" << table.contextCount()
              << ",\"overdue_threshold_seconds\":" << table.overdueThreshold()
              << ",\"dense_sha256\":\"" << dense_sha.finish()
              << "\",\"lookup_sha256\":\"" << lookup_sha.finish()
              << "\",\"all_lookup_values_bitwise_equal\":true,\"reference_bitwise_equal\":"
              << (reference_path.empty() ? "null" : "true") << "}\n";
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 || argc == 3) { audit_file(argv[1], argc == 3 ? argv[2] : ""); return 0; }
        if (argc != 1) throw std::runtime_error("usage: test [table.rle [reference.f64]]");
        unit_tests(); std::cout << "ABC cap/context table checks passed: " << checks << "\n"; return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << "\n"; return 1; }
}
