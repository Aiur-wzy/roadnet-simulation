// Reuse only the established fixture JSON/byte helpers and its full-table
// regression entry point. The sparse expected node mapping below is brute force.
#define main original_rg_table_test_main
#include "test_cap_context_abc_rg_table.cpp"
#undef main

#include <algorithm>
#include <cstdint>
#include <tuple>
#include <vector>

namespace {
using Entry = std::tuple<std::uint32_t, std::uint32_t, double>;
const std::string DEMAND_MAGIC = "CAMS_CAP_CONTEXT_ABC_RG_DEMAND_V1";

void set_integer(Json& object, const std::string& key, std::uint64_t value) {
    object.object[key] = JsonParser(std::to_string(value)).parse();
}

void set_string(Json& object, const std::string& key, const std::string& value) {
    object.object[key] = JsonParser(quote(value)).parse();
}

std::vector<int> brute_nodes(int lane, int maximum, std::uint64_t step, std::uint64_t dense) {
    std::vector<int> result;
    for (int value = lane; value <= maximum; ++value) {
        if (static_cast<std::uint64_t>(value) <= dense || value % step == 0 || value == lane || value == maximum) {
            result.push_back(value);
        }
    }
    return result;
}

int brute_nearest(int road, int lane, std::uint64_t step, std::uint64_t dense) {
    int best = lane;
    for (int value : brute_nodes(lane, 27, step, dense)) {
        if (std::abs(value - road) <= std::abs(best - road)) { best = value; }
    }
    return best;
}

Json demand_header(const Json& source, std::uint64_t step, std::uint64_t dense) {
    Json header = source;
    set_string(header, "format", DEMAND_MAGIC);
    set_string(header, "payload_layout", "sorted-u32-u32-f64-v1");
    set_string(header, "scope_sha256", std::string(64, 'a'));
    header.object.erase("run_count");
    const std::string policy = "{\"name\":\"nearest-integer-road-flow-v1\",\"unit\":\"vehicles\",\"tie\":\"higher\","
        "\"grid_origin\":0,\"preserve_lane_lower_bound\":true,\"preserve_road_upper_bound\":true,"
        "\"output_coverage\":\"all-legal-integer-road-flow\",\"step\":" + std::to_string(step)
        + ",\"dense_until\":" + std::to_string(dense) + "}";
    header.object["road_flow_quantization"] = JsonParser(policy).parse();
    const Json& grid = header.at("free_speed_cap_grid");
    const std::uint64_t blocks = static_cast<std::uint64_t>(header.at("signal_point_count").number)
        * header.at("road_lengths").array.size()
        * static_cast<std::uint64_t>(grid.at("index_max").number - grid.at("index_min").number + 1) * 6;
    std::uint64_t flows = 0, predictions = 0;
    for (int lane = 0; lane <= 16; ++lane) {
        flows += 28 - lane;
        predictions += brute_nodes(lane, 27, step, dense).size();
    }
    set_integer(header, "road_flow_max", 27);
    set_integer(header, "lane_flow_max", 16);
    set_integer(header, "flow_count", flows);
    set_integer(header, "group_count", blocks * 17);
    set_integer(header, "expanded_rows", blocks * flows);
    set_integer(header, "prediction_rows", blocks * predictions);
    set_integer(header.object.at("resource_limits"), "requested_max_rows", 4000000000ULL);
    return header;
}

void append_u32(std::string& payload, std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index) { payload += static_cast<char>((value >> (index * 8)) & 255); }
}

std::string demand_bytes(Json header, const std::vector<Entry>& entries) {
    std::string payload;
    for (const auto& entry : entries) {
        append_u32(payload, std::get<0>(entry)); append_u32(payload, std::get<1>(entry));
        const double value = std::get<2>(entry);
        std::uint64_t bits;
        std::memcpy(&bits, &value, 8);
        for (unsigned index = 0; index < 8; ++index) { payload += static_cast<char>((bits >> (index * 8)) & 255); }
    }
    CapContextSha256 hash;
    hash.update(reinterpret_cast<const unsigned char*>(payload.data()), payload.size());
    set_integer(header, "entry_count", entries.size());
    set_integer(header, "payload_bytes", payload.size());
    set_string(header, "payload_sha256", hash.finish());
    return DEMAND_MAGIC + '\n' + serialize(header) + '\n' + payload;
}

void load_demand(CapContextABCRGTable& table, const std::string& bytes) {
    std::istringstream input(bytes, std::ios::binary);
    std::string magic;
    std::getline(input, magic);
    require(magic == DEMAND_MAGIC, "Demand test magic");
    table.loadDemand(input);
}

void reject_demand(CapContextABCRGTable& table, const std::string& good, const std::string& bad, const std::string& name) {
    load_demand(table, good);
    rejects_query([&] { load_demand(table, bad); }, name);
    require(table.size() == 0 && table.compressedBytes() == 0 && !table.isDemand()
            && table.demandEntryCount() == 0 && table.demandScope().empty(), "Failed demand load clears " + name);
}

void explicit_demand_regression(const Json& source) {
    const double length = source.at("road_lengths").array.front().number;
    const double cap = source.at("free_speed_cap_grid").at("min").number;
    CapContextABCRGTable table;
    for (const std::vector<int>& nodes : std::vector<std::vector<int>>{
            {0, 27}, {0, 1, 2, 3, 4, 5, 10, 15, 20, 27}, {0, 2, 9, 18, 23, 27}}) {
        Json header = demand_header(source, 1, 10);
        set_explicit_flow_policy(header, nodes);
        std::vector<Entry> entries;
        std::vector<std::vector<int>> expected_nodes;
        for (int lane = 0; lane <= 16; ++lane) {
            expected_nodes.emplace_back();
            for (int road = lane; road <= 27; ++road) {
                if (road == lane || std::find(nodes.begin(), nodes.end(), road) != nodes.end()) {
                    expected_nodes.back().push_back(road);
                    entries.emplace_back(lane, road, 1. + lane * .25 + road * .125);
                }
            }
        }
        const std::string good = demand_bytes(header, entries);
        load_demand(table, good);
        require(table.isDemand() && table.demandEntryCount() == entries.size(), "explicit demand mode and entry count");
        for (int lane = 0; lane <= 16; ++lane) {
            for (int road = lane; road <= 27; ++road) {
                int expected = lane;
                for (int node : expected_nodes[lane]) {
                    if (std::abs(node - road) <= std::abs(expected - road)) { expected = node; }
                }
                double value = -123.;
                std::uint32_t group = 999, representative = 999;
                require(table.lookupDemand(length, road, lane, cap, 0, NoSignalRGFeatures(), value, group, representative),
                        "explicit demand all legal flow inputs hit");
                require(group == static_cast<unsigned>(lane) && representative == static_cast<unsigned>(expected),
                        "explicit nearest node agrees with independent exhaustive mapping including tie and lane lower bound");
                require(value == 1. + lane * .25 + expected * .125, "explicit sparse prediction is exact");
            }
        }
        for (const auto& change : explicit_flow_corruptions()) {
            reject_demand(table, good, replace_header(good, change.second), change.first);
        }
        reject_demand(table, good, demand_bytes(header, {{0, 26, 1.}}), "explicit key not representative");
        // A subsequent legacy load must discard the explicit-node grid.
        const Json old_header = demand_header(source, 5, 10);
        load_demand(table, demand_bytes(old_header, {{0, 15, 2.}}));
        double value = -123.;
        std::uint32_t group = 999, representative = 999;
        require(table.lookupDemand(length, 14, 0, cap, 0, NoSignalRGFeatures(), value, group, representative)
                && representative == 15 && value == 2., "explicit-to-legacy reload clears nodes");
    }
    // Single-node domain is valid and must not dereference a missing neighbour.
    Json zero_header = demand_header(source, 1, 10);
    const auto blocks = static_cast<std::uint64_t>(zero_header.at("group_count").number) / 17;
    set_integer(zero_header, "road_flow_max", 0); set_integer(zero_header, "lane_flow_max", 0);
    set_integer(zero_header, "flow_count", 1); set_integer(zero_header, "group_count", blocks);
    set_integer(zero_header, "expanded_rows", blocks);
    set_explicit_flow_policy(zero_header, {0});
    load_demand(table, demand_bytes(zero_header, {{0, 0, 1.}}));
    double value = -123.; std::uint32_t group = 999, representative = 999;
    require(table.lookupDemand(length, 0, 0, cap, 0, NoSignalRGFeatures(), value, group, representative)
            && representative == 0 && value == 1., "explicit singleton road-flow domain");
}
}

int main(int argc, char** argv) {
    try {
        if (argc != 2) { throw std::runtime_error("Usage: test_demand_table_reader existing-full-rle-fixture"); }
        const std::string original = read_file(argv[1]);
        const Json source = header_of(original);
        explicit_demand_regression(source);
        const double length = source.at("road_lengths").array.front().number;
        const double cap = source.at("free_speed_cap_grid").at("min").number;
        CapContextABCRGTable table;
        for (std::uint64_t step : {1ULL, 2ULL, 5ULL, 7ULL, 4000000000ULL}) {
            for (std::uint64_t dense : {0ULL, 10ULL, 40ULL}) {
                Json header = demand_header(source, step, dense);
                std::vector<Entry> entries;
                for (int lane = 0; lane <= 16; ++lane) {
                    for (int road : brute_nodes(lane, 27, step, dense)) {
                        entries.emplace_back(lane, road, 1. + lane * .25 + road * .125);
                    }
                }
                const auto last_group = static_cast<std::uint32_t>(header.at("group_count").number - 1);
                entries.emplace_back(last_group, 27, 42.);
                const std::string good = demand_bytes(header, entries);
                load_demand(table, good);
                require(table.isDemand() && table.demandEntryCount() == entries.size(), "Demand mode/count");
                require(table.demandScope() == std::string(64, 'a'), "Demand scope");
                require(table.size() == static_cast<std::uint64_t>(header.at("expanded_rows").number), "Logical coverage unchanged");
                const Json& last_signal = header.at("signal_catalog").array.back();
                const SignalRGFeatures signal = {last_signal.at("entry_type").string[0], last_signal.at("nodes").array.back().number,
                    last_signal.at("next_type").string[0], last_signal.at("next_duration_seconds").number, 'G', false};
                double last_value = 0.;
                std::uint32_t last_found_group = 0, last_road = 0;
                require(table.lookupDemand(header.at("road_lengths").array.back().number, 27, 16,
                        header.at("free_speed_cap_grid").at("max").number, 5, signal,
                        last_value, last_found_group, last_road), "Last joint group hit");
                require(last_found_group == last_group && last_road == 27 && last_value == 42., "Signal/length/cap/context/lane group ordering");
                for (int lane = 0; lane <= 16; ++lane) {
                    for (int road = lane; road <= 27; ++road) {
                        double value = -123.;
                        std::uint32_t group = 999, representative = 999;
                        require(table.lookupDemand(length, road, lane, cap, 0, NoSignalRGFeatures(), value, group, representative), "Known key hit");
                        const int expected = brute_nearest(road, lane, step, dense);
                        require(group == static_cast<unsigned>(lane) && representative == static_cast<unsigned>(expected), "Independent quantized key");
                        require(value == 1. + lane * .25 + expected * .125, "Exact sparse prediction");
                        require(table.lookup(length, road, lane, cap, 0, NoSignalRGFeatures()) == value, "Strict known lookup");
                    }
                }
                double absent_value = -123.;
                std::uint32_t group = 0, representative = 0;
                require(!table.lookupDemand(length, 13, 11, cap, 1, NoSignalRGFeatures(), absent_value, group, representative), "Legal missing key");
                require(absent_value == -123. && group == 28 && representative == static_cast<unsigned>(brute_nearest(13, 11, step, dense)), "Missing has key not fallback");
                rejects_query([&] { table.lookup(length, 13, 11, cap, 1, NoSignalRGFeatures()); }, "Strict missing");
                const std::string empty = demand_bytes(header, {});
                load_demand(table, empty);
                require(table.isDemand() && table.compressedBytes() == 0 && table.demandEntryCount() == 0, "Empty demand is loaded");
                table.requireCrossingTime(3.);
                table.requireSignalRemainingStep(source.at("signal_remaining_step").number);
                require(table.quantizeSignal(NoSignalRGFeatures()).entry_remaining_seconds == 0., "Empty cache catalog available");
                require(!table.lookupDemand(length, 13, 11, cap, 0, NoSignalRGFeatures(), absent_value, group, representative), "Empty cache legal miss");
                rejects_query([&] { table.lookupDemand(length, 10, 11, cap, 0, NoSignalRGFeatures(), absent_value, group, representative); }, "Illegal triangular flow");
                rejects_query([&] { table.lookupDemand(length, 28, 11, cap, 0, NoSignalRGFeatures(), absent_value, group, representative); }, "Outside maximum flow");
                rejects_query([&] { table.lookupDemand(length, 13, 11, 0., 0, NoSignalRGFeatures(), absent_value, group, representative); }, "Bad cap");
            }
        }
        const Json header = demand_header(source, 5, 10);
        const std::string good = demand_bytes(header, {{0, 0, 1.}, {0, 15, 2.}, {12, 12, 3.}});
        const std::vector<std::pair<std::string, std::function<void(Json&)>>> changes = {
            {"missing flow policy", [](Json& h) { h.object.erase("road_flow_quantization"); }},
            {"missing prediction count", [](Json& h) { h.object.erase("prediction_rows"); }},
            {"both flow fields absent", [](Json& h) { h.object.erase("road_flow_quantization"); h.object.erase("prediction_rows"); }},
            {"wrong prediction count", [](Json& h) { h.object.at("prediction_rows").number += 1; }},
            {"zero step", [](Json& h) { h.object.at("road_flow_quantization").object.at("step").number = 0; }},
            {"float step", [](Json& h) { h.object.at("road_flow_quantization").object.at("step").integer_token = false; }},
            {"wrong tie", [](Json& h) { h.object.at("road_flow_quantization").object.at("tie").string = "lower"; }},
            {"extra policy key", [](Json& h) { set_integer(h.object.at("road_flow_quantization"), "unknown", 1); }},
            {"wrong scope", [](Json& h) { h.object.at("scope_sha256").string = "a"; }},
            {"bad scope uppercase", [](Json& h) { h.object.at("scope_sha256").string = std::string(64, 'A'); }},
            {"RLE run_count", [](Json& h) { set_integer(h, "run_count", 3); }},
            {"entry count mismatch", [](Json& h) { h.object.at("entry_count").number += 1; }},
            {"entry limit", [](Json& h) { h.object.at("entry_count").number = 4000000000.; }},
            {"payload mismatch", [](Json& h) { h.object.at("payload_bytes").number += 1; }},
            {"wrong payload SHA", [](Json& h) { h.object.at("payload_sha256").string = std::string(64, '0'); }},
        };
        for (const auto& change : changes) { reject_demand(table, good, replace_header(good, change.second), change.first); }
        reject_demand(table, good, good + "x", "Trailing payload");
        reject_demand(table, good, good.substr(0, good.size() - 1), "Truncated payload");
        reject_demand(table, good, demand_bytes(header, {{0, 0, 1.}, {0, 0, 2.}}), "Duplicate keys");
        reject_demand(table, good, demand_bytes(header, {{1, 1, 1.}, {0, 0, 2.}}), "Unsorted keys");
        reject_demand(table, good, demand_bytes(header, {{12, 11, 1.}}), "Road below group lane");
        reject_demand(table, good, demand_bytes(header, {{0, 13, 1.}}), "Not a representative node");
        reject_demand(table, good, demand_bytes(header, {{0, 28, 1.}}), "Road out of domain");
        reject_demand(table, good, demand_bytes(header, {{static_cast<std::uint32_t>(header.at("group_count").number), 27, 1.}}), "Group out of domain");
        reject_demand(table, good, demand_bytes(header, {{0, 0, std::numeric_limits<double>::quiet_NaN()}}), "NaN prediction");
        reject_demand(table, good, demand_bytes(header, {{0, 0, .5}}), "Prediction below floor");
        load_bytes(table, original);
        require(!table.isDemand() && table.demandEntryCount() == 0 && table.demandScope().empty(), "Switch back to full RLE");
        double value = 0.; std::uint32_t group = 0, representative = 0;
        rejects_query([&] { table.lookupDemand(length, 0, 0, cap, 0, NoSignalRGFeatures(), value, group, representative); }, "Demand API on RLE");
        std::cout << "PASS demand reader checks=" << checks << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL demand reader checks=" << checks << ": " << error.what() << std::endl;
        return 1;
    }
}
