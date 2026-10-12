#include "sf_travel_time_table.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
int checks = 0;
void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error("TEST FAILED: " + message);
}
void throws(const std::function<void()>& action, const std::string& message) {
    ++checks;
    try { action(); }
    catch (const std::runtime_error&) { return; }
    throw std::runtime_error("TEST FAILED, expected runtime_error: " + message);
}
std::string replace_once(std::string text, const std::string& old, const std::string& replacement) {
    const auto position = text.find(old);
    if (position == std::string::npos) throw std::runtime_error("bad replacement fixture: " + old);
    text.replace(position, old.size(), replacement);
    return text;
}
std::string metadata() {
    return R"JSON({"format":"CAMS_SF_TABLE_V1","training_schema":"road-time-core3-sf-speed-limit-v1","columns":["road_length","road_flow","lane_flow","speed_factor","speed_limit","travel_time_no_waiting"],"source_label":"travel_time_label_gt_0p1","static_pairs":[{"road_length":320,"speed_limit":13.89},{"road_length":545.6,"speed_limit":10}],"static_key_scale":10000,"static_key_rounding":"floor(value * scale + 0.5 + 1e-9)","speed_factor_grid":{"scale":100,"index_min":99,"index_max":101,"min":0.99,"max":1.01,"step":0.01,"rounding":"nearest, positive half-up: floor(SF * scale + 0.5 + 1e-9)","scaled_boundary_tolerance":1e-9,"raw_boundary_tolerance":1e-12,"out_of_range":"error; no clamp and no implicit formula fallback"},"road_flow_max":2,"lane_flow_max":1,"flow_min":0,"flow_step":1,"flow_constraint":"lane_flow <= road_flow","planned_rows":30,"prediction_floor_seconds":1.0,"lookup_policy":"exact static/flow keys and nearest SF node; no interpolation or clamping","scope":"external-road time only; no junction/wait addition and no division by SF","legacy_cpp_compatible":false,"training_ranges":{"road_flow":{"min":1,"max":2},"lane_flow":{"min":0,"max":1}},"audit":{"nested":{"min":123,"max":456},"unicode":"\u4e2d\u6587\ud83d\ude00"}})JSON";
}
std::string body() {
    std::ostringstream output;
    output << std::setprecision(17);
    int i = 0;
    for (int pair = 0; pair < 2; ++pair)
        for (int sf = 99; sf <= 101; ++sf)
            for (int road = 0; road <= 2; ++road)
                for (int lane = 0; lane <= std::min(road, 1); ++lane)
                    output << (pair ? 545.6 : 320.0) << ' ' << road << ' ' << lane << ' '
                           << sf / 100.0 << ' ' << (pair ? 10.0 : 13.89) << ' ' << ++i << '\n';
    return output.str();
}
std::string fixture(const std::string& meta = metadata(), const std::string& rows = body()) {
    return meta + "\nroad_length road_flow lane_flow speed_factor speed_limit travel_time_no_waiting\n" + rows;
}
void load_text(SfTravelTimeTable& table, const std::string& text) {
    std::istringstream input(text);
    table.load(input);
}
void reject_text(const std::string& text, const std::string& name) {
    SfTravelTimeTable table;
    load_text(table, fixture());
    throws([&]() { load_text(table, text); }, name);
    require(table.size() == 0 && table.label().empty(), name + " clears replaced table");
    throws([&]() { table.lookup(320, 0, 0, 1, 13.89); }, name + " disallows stale lookup");
}

void test_lookup() {
    SfTravelTimeTable table;
    throws([&]() { table.lookup(320, 0, 0, 1, 13.89); }, "empty lookup");
    load_text(table, fixture());
    require(table.size() == 30, "dense count");
    require(table.label() == "travel_time_label_gt_0p1", "threshold label");
    require(table.roadFlowMax() == 2 && table.laneFlowMax() == 1, "grid flow ranges");
    require(table.sfMin() == .99 && table.sfMax() == 1.01, "grid SF ranges");
    require(table.trainingRoadFlowMin() == 1 && table.trainingRoadFlowMax() == 2 &&
            table.trainingLaneFlowMin() == 0 && table.trainingLaneFlowMax() == 1, "scoped training ranges");
    int expected = 0;
    for (int pair = 0; pair < 2; ++pair)
        for (int sf = 99; sf <= 101; ++sf)
            for (int road = 0; road <= 2; ++road)
                for (int lane = 0; lane <= std::min(road, 1); ++lane)
                    require(table.lookup(pair ? 545.6 : 320, road, lane, sf / 100.0, pair ? 10 : 13.89) == ++expected,
                            "every compressed flow coordinate");
    require(table.lookup(320, 0, 0, .995, 13.89) == 6, "lower positive half rounds up");
    require(table.lookup(320, 0, 0, 1.005, 13.89) == 11, "binary-sensitive upper positive half rounds up");
    require(table.lookup(320, 0, 0, .994999, 13.89) == 1, "below half rounds down");
    require(table.lookup(320, 0, 0, 1.01, 13.89) == 11, "SF above one not capped or divided");
    require(table.lookup(320, 0, 0, .99 - 5e-13, 13.89) == 1, "lower raw tolerance accepted");
    require(table.lookup(320, 0, 0, 1.01 + 5e-13, 13.89) == 11, "upper raw tolerance accepted");
    require(table.lookup(320.00000001, 0, 0, 1, 13.890000001) == 6, "static quantization representation noise");
    for (double sf : {0.0, -.1, .99 - 2e-12, 1.01 + 2e-12,
                      std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
        throws([&]() { table.lookup(320, 0, 0, sf, 13.89); }, "invalid/outside SF");
    for (const auto& flows : std::vector<std::pair<int, int>>{{-1, 0}, {0, -1}, {3, 0}, {2, 2}, {0, 1}})
        throws([&]() { table.lookup(320, flows.first, flows.second, 1, 13.89); }, "outside/invalid flow");
    throws([&]() { table.lookup(321, 0, 0, 1, 13.89); }, "missing length");
    throws([&]() { table.lookup(320, 0, 0, 1, 10); }, "crossed pair rejected");
    throws([&]() { table.lookup(0, 0, 0, 1, 13.89); }, "zero length");
    throws([&]() { table.lookup(320, 0, 0, 1, std::numeric_limits<double>::quiet_NaN()); }, "NaN speed limit");
    table.clear();
    require(table.size() == 0 && table.label().empty() && table.sfMax() == 0, "clear resets state");
    load_text(table, fixture());
    require(table.size() == 30, "reload after clear");
    std::string crlf;
    for (char c : fixture()) { if (c == '\n') crlf += '\r'; crlf += c; }
    load_text(table, crlf);
    require(table.size() == 30, "Windows CRLF supported");
    std::string final_no_newline = fixture();
    final_no_newline.pop_back();
    load_text(table, final_no_newline);
    require(table.size() == 30, "valid final row without newline");
    for (const std::string& label : {"travel_time_label_gt_0p1", "travel_time_label_gt_0p5", "travel_time_label_gt_1",
                                   "travel_time_label_gt_1p5", "travel_time_label_gt_2", "travel_time_label_gt_3", "travel_time_label"}) {
        load_text(table, fixture(replace_once(metadata(), "travel_time_label_gt_0p1", label)));
        require(table.label() == label, "all exporter source labels supported");
    }
    std::string zero_meta = replace_once(metadata(), "\"road_flow_max\":2", "\"road_flow_max\":0");
    zero_meta = replace_once(zero_meta, "\"lane_flow_max\":1", "\"lane_flow_max\":0");
    zero_meta = replace_once(zero_meta, "\"planned_rows\":30", "\"planned_rows\":6");
    std::ostringstream zero_body;
    zero_body << std::setprecision(17);
    for (int pair = 0; pair < 2; ++pair)
        for (int sf = 99; sf <= 101; ++sf)
            zero_body << (pair ? 545.6 : 320) << " 0 0 " << sf / 100.0 << ' ' << (pair ? 10 : 13.89) << " 1\n";
    load_text(table, fixture(zero_meta, zero_body.str()));
    require(table.size() == 6 && table.lookup(545.6, 0, 0, 1.005, 10) == 1, "degenerate zero-flow grid");
}

void test_metadata_failures() {
    const std::vector<std::pair<std::string, std::string>> mutations = {
        {"CAMS_SF_TABLE_V1", "CAMS_SF_TABLE_V2"},
        {"road-time-core3-sf-speed-limit-v1", "old-schema"},
        {"\"road_length\",\"road_flow\"", "\"speed_limit\",\"road_flow\""},
        {"travel_time_label_gt_0p1", "arbitrary_label"},
        {"travel_time_label_gt_0p1", "travel_time_no_waiting"},
        {"\"static_key_scale\":10000", "\"static_key_scale\":1000"},
        {"\"scale\":100", "\"scale\":100.5"},
        {"\"index_min\":99", "\"index_min\":98"},
        {"\"index_max\":101", "\"index_max\":98"},
        {"\"min\":0.99", "\"min\":0.98"},
        {"\"step\":0.01", "\"step\":0.02"},
        {"\"scaled_boundary_tolerance\":1e-9", "\"scaled_boundary_tolerance\":1e-8"},
        {"\"raw_boundary_tolerance\":1e-12", "\"raw_boundary_tolerance\":1e-8"},
        {"error; no clamp and no implicit formula fallback", "clamp"},
        {"nearest, positive half-up: floor(SF * scale + 0.5 + 1e-9)", "bankers"},
        {"\"flow_min\":0", "\"flow_min\":1"},
        {"\"flow_step\":1", "\"flow_step\":2"},
        {"lane_flow <= road_flow", "unconstrained"},
        {"\"road_flow_max\":2", "\"road_flow_max\":-1"},
        {"\"lane_flow_max\":1", "\"lane_flow_max\":3"},
        {"\"planned_rows\":30", "\"planned_rows\":29"},
        {"\"planned_rows\":30", "\"planned_rows\":50000001"},
        {"\"prediction_floor_seconds\":1.0", "\"prediction_floor_seconds\":0"},
        {"\"legacy_cpp_compatible\":false", "\"legacy_cpp_compatible\":true"},
        {"\"legacy_cpp_compatible\":false", "\"legacy_cpp_compatible\":0"},
        {"no junction/wait addition and no division by SF", "includes waiting"},
        {"no interpolation or clamping", "with interpolation"},
        {"\"road_length\":545.6,\"speed_limit\":10", "\"road_length\":320,\"speed_limit\":13.89"},
        {"\"road_length\":320", "\"road_length\":320.00001"},
        {"\"road_length\":320", "\"road_length\":0"},
        {"\"road_flow\":{\"min\":1,\"max\":2}", "\"road_flow\":{\"min\":3,\"max\":2}"},
        {"\"road_flow\":{\"min\":1,\"max\":2}", "\"road_flow\":{\"min\":1.1,\"max\":2}"},
        {"\"source_label\":", "\"missing_source_label\":"},
        {"\"min\":0.99", "\"min\":0.99,\"min\":0.99"},
        {"\"format\":", "\"format\":\"CAMS_SF_TABLE_V1\",\"format\":"},
        {"\"nested\":{\"min\":123", "\"nested\":{\"min\":123,\"min\":124"},
        {"\"min\":123", "\"min\":01"},
        {"\"min\":123", "\"min\":1e999"},
        {"\"min\":123", "\"min\":NaN"},
        {"\\u4e2d", "\\ud800"},
        {"\\u4e2d", "\\uZZZZ"},
        {"\\u4e2d", "\\x12"}
    };
    for (const auto& mutation : mutations)
        reject_text(fixture(replace_once(metadata(), mutation.first, mutation.second)), mutation.first);
    reject_text(fixture(metadata() + " garbage"), "trailing JSON");
    reject_text(fixture(metadata().substr(0, metadata().size() - 1)), "truncated JSON");
    reject_text(fixture("[" + metadata() + "]"), "non-object root");
    reject_text(fixture(std::string(1024 * 1024 + 1, ' ')), "metadata size limit");
    std::string huge = replace_once(metadata(), "\"road_flow_max\":2", "\"road_flow_max\":1000000");
    huge = replace_once(huge, "\"lane_flow_max\":1", "\"lane_flow_max\":1000000");
    reject_text(fixture(huge), "derived count memory guard before reserve");
    reject_text(fixture(std::string(34, '[') + metadata() + std::string(34, ']')), "JSON depth limit");
    reject_text("", "missing metadata");
    reject_text(metadata() + "\n", "missing header");
    reject_text(replace_once(fixture(), "\nroad_length road_flow", "\nspeed_limit road_flow"), "wrong header");
    reject_text(replace_once(fixture(), "travel_time_no_waiting\n", "travel_time_no_waiting extra\n"), "extra header");
}

void test_body_failures() {
    const std::string rows = body();
    const auto first_end = rows.find('\n') + 1;
    const std::string first = rows.substr(0, first_end);
    reject_text(fixture(metadata(), ""), "empty body");
    reject_text(fixture(metadata(), rows.substr(first_end)), "missing first row");
    reject_text(fixture(metadata(), rows.substr(0, rows.rfind('\n', rows.size() - 2) + 1)), "missing final row");
    reject_text(fixture(metadata(), first + rows), "duplicate row");
    reject_text(fixture(metadata(), rows + first), "extra row");
    reject_text(fixture(metadata(), rows + "\n"), "extra blank row");
    reject_text(fixture(metadata(), "\n" + rows), "blank body row");
    reject_text(fixture(metadata(), replace_once(rows, "320 0 0", "320 0 1")), "flow constraint/order mismatch");
    reject_text(fixture(metadata(), replace_once(rows, "320 0 0", "320 0.1 0")), "noninteger flow");
    reject_text(fixture(metadata(), replace_once(rows, "320 0 0", "321 0 0")), "wrong static value");
    reject_text(fixture(metadata(), replace_once(rows, "320 0 0", "320x 0 0")), "invalid numeric token");
    reject_text(fixture(metadata(), replace_once(rows, "320 0 0", "320 0+0")), "missing whitespace");
    for (const std::string& prediction : {"0", "-1", "0.999", "nan", "inf", "1e999", "1 trailing"})
        reject_text(fixture(metadata(), replace_once(rows, " 1\n", " " + prediction + "\n")), "invalid prediction " + prediction);
    reject_text(fixture(metadata(), replace_once(rows, " 1\n", "\n")), "missing prediction");
    reject_text(fixture(metadata(), replace_once(rows, " 1\n", std::string(" 1\0hidden\n", 10))), "embedded NUL tail");
    reject_text(fixture(metadata(), std::string(4097, ' ') + rows), "body line size limit");
    SfTravelTimeTable table;
    std::istringstream broken(fixture());
    broken.setstate(std::ios::badbit);
    throws([&]() { table.load(broken); }, "bad input stream");
    require(table.size() == 0, "bad stream leaves empty table");
}
}

int main() {
    try {
        test_lookup();
        test_metadata_failures();
        test_body_failures();
        std::cout << "SF table tests passed: " << checks << " checks\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
