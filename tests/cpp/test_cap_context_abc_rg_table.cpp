#include "cap_context_abc_rg_table.h"
#include "cap_context_json.h"
#include "cap_context_sha256.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using cap_context_json::Json;
using cap_context_json::JsonParser;
std::size_t checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) { throw std::runtime_error(message); }
}

std::string read_file(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) { throw std::runtime_error("Cannot open " + path); }
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void load_bytes(CapContextABCRGTable& table, const std::string& bytes) {
    std::istringstream input(bytes, std::ios::binary);
    std::string magic;
    std::getline(input, magic);
    if (magic != "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1") { throw std::runtime_error("Wrong ABC/RG magic"); }
    table.load(input);
}

std::string quote(const std::string& value) {
    std::string output = "\"";
    for (char c : value) {
        if (c == '\\' || c == '"') { output += '\\'; output += c; }
        else if (c == '\n') { output += "\\n"; }
        else if (c == '\r') { output += "\\r"; }
        else if (c == '\t') { output += "\\t"; }
        else { output += c; }
    }
    return output + "\"";
}

std::string serialize(const Json& value) {
    switch (value.kind) {
        case Json::Null: return "null";
        case Json::Boolean: return value.boolean ? "true" : "false";
        case Json::String: return quote(value.string);
        case Json::Number: {
            std::ostringstream output;
            output << std::setprecision(17) << value.number;
            std::string text = output.str();
            if (!value.integer_token && text.find_first_of(".eE") == std::string::npos) { text += ".0"; }
            return text;
        }
        case Json::Array: {
            std::string text = "[";
            for (std::size_t i = 0; i < value.array.size(); ++i) { if (i) { text += ','; } text += serialize(value.array[i]); }
            return text + ']';
        }
        case Json::Object: {
            std::string text = "{";
            bool first = true;
            for (const auto& entry : value.object) { if (!first) { text += ','; } first = false; text += quote(entry.first) + ':' + serialize(entry.second); }
            return text + '}';
        }
    }
    throw std::runtime_error("Unexpected test JSON kind");
}

Json header_of(const std::string& bytes) {
    const std::size_t first = bytes.find('\n');
    const std::size_t second = bytes.find('\n', first + 1);
    return JsonParser(bytes.substr(first + 1, second - first - 1)).parse();
}

std::string replace_header(const std::string& bytes, const std::function<void(Json&)>& edit) {
    const std::size_t first = bytes.find('\n');
    const std::size_t second = bytes.find('\n', first + 1);
    Json header = header_of(bytes);
    edit(header);
    return bytes.substr(0, first + 1) + serialize(header) + '\n' + bytes.substr(second + 1);
}

std::string rehashed_payload(const std::string& bytes) {
    const std::size_t begin = bytes.find('\n', bytes.find('\n') + 1) + 1;
    CapContextSha256 hash;
    hash.update(reinterpret_cast<const unsigned char*>(bytes.data() + begin), bytes.size() - begin);
    const std::string digest = hash.finish();
    return replace_header(bytes, [&](Json& header) { header.object.at("payload_sha256").string = digest; });
}

void require_rejected_and_cleared(CapContextABCRGTable& table, const std::string& good,
                                 const std::string& bad, const std::string& name) {
    load_bytes(table, good);
    bool rejected = false;
    try { load_bytes(table, bad); } catch (const std::exception&) { rejected = true; }
    require(rejected, "Should reject " + name);
    require(table.size() == 0 && table.compressedBytes() == 0 && table.label().empty()
            && table.signalRemainingStep() == 0 && table.signalPointCount() == 0, "Failure should clear " + name);
    bool guard_rejected = false;
    try { table.requireSignalRemainingStep(header_of(good).at("signal_remaining_step").number); }
    catch (const std::exception&) { guard_rejected = true; }
    require(guard_rejected, "Failed load must leave step expectation unavailable: " + name);
}

void rejects_query(const std::function<void()>& operation, const std::string& name) {
    bool rejected = false;
    try { operation(); } catch (const std::exception&) { rejected = true; }
    require(rejected, "Should reject query: " + name);
}

void corruption_checks(CapContextABCRGTable& table, const std::string& good) {
    const std::vector<std::pair<std::string, std::function<void(Json&)>>> changes = {
        {"old schema", [](Json& h) { h.object.at("training_schema").string = "road-time-cap-context-abc-v1"; }},
        {"old format", [](Json& h) { h.object.at("format").string = "CAMS_CAP_CONTEXT_ABC_RLE_V1"; }},
        {"five categories", [](Json& h) { h.object.at("context_count").number = 5; }},
        {"float context count", [](Json& h) { h.object.at("context_count").integer_token = false; }},
        {"missing column", [](Json& h) { h.object.at("columns").array.pop_back(); }},
        {"ordered categories", [](Json& h) { h.object.at("categorical_ordered").boolean = true; }},
        {"wrong signal semantics", [](Json& h) { h.object.at("signal_contract").object.at("phase_boundary").string = "closed"; }},
        {"wrong ABC mapping", [](Json& h) { h.object.at("context_contract").object.at("context_mapping").object.at("4").string = "C"; }},
        {"wrong quantization tie", [](Json& h) { h.object.at("signal_quantization").object.at("tie").string = "lower"; }},
        {"wrong upper-bound ULP tolerance", [](Json& h) { h.object.at("signal_quantization").object.at("entry_upper_tolerance_ulps").number = 9; }},
        {"missing signal step", [](Json& h) { h.object.erase("signal_remaining_step"); }},
        {"zero signal step", [](Json& h) { h.object.at("signal_remaining_step").number = 0; }},
        {"negative signal step", [](Json& h) { h.object.at("signal_remaining_step").number = -1; }},
        {"nonfinite signal step", [](Json& h) { h.object.at("signal_remaining_step").number = std::numeric_limits<double>::infinity(); }},
        {"row mismatch", [](Json& h) { h.object.at("expanded_rows").number += 1; }},
        {"expanded-row hard guard", [](Json& h) { h.object.at("expanded_rows").number = 4000000001.0; }},
        {"payload hard guard", [](Json& h) { h.object.at("payload_bytes").number = 536870913.0; }},
        {"invalid cap", [](Json& h) { h.object.at("free_speed_cap_grid").object.at("min").number = 0; }},
    };
    for (const auto& change : changes) {
        require_rejected_and_cleared(table, good, replace_header(good, change.second), change.first);
    }
    require_rejected_and_cleared(table, good, good.substr(0, good.size() - 1), "truncated payload");
    require_rejected_and_cleared(table, good, good + 'x', "trailing payload");
    const std::size_t payload_begin = good.find('\n', good.find('\n') + 1) + 1;
    std::string offsets_bad = good;
    offsets_bad[payload_begin] = 1;
    require_rejected_and_cleared(table, good, rehashed_payload(offsets_bad), "bad first offset despite valid hash");
    std::string hash_bad = good;
    hash_bad.back() ^= 1;
    require_rejected_and_cleared(table, good, hash_bad, "changed payload/hash");
    const Json header = header_of(good);
    const std::size_t record_begin = payload_begin + 4 * (static_cast<std::size_t>(header.at("group_count").number) + 1);
    std::string gap = good;
    gap[record_begin] = 1;
    require_rejected_and_cleared(table, good, rehashed_payload(gap), "run starts after lane flow despite valid hash");
    std::string nonfinite = good;
    for (std::size_t index = 0; index < 8; ++index) { nonfinite[record_begin + 4 + index] = 0; }
    nonfinite[record_begin + 10] = static_cast<char>(0xf8);
    nonfinite[record_begin + 11] = static_cast<char>(0x7f);
    require_rejected_and_cleared(table, good, rehashed_payload(nonfinite), "NaN prediction despite valid hash");
    const std::string root_bad = good.substr(0, good.find('\n') + 1) + "[]\n" + good.substr(payload_begin);
    require_rejected_and_cleared(table, good, root_bad, "non-object header");
    const std::size_t header_begin = good.find('\n') + 1;
    const std::string duplicate = good.substr(0, header_begin + 1) + "\"context_count\":6," + good.substr(header_begin + 1);
    require_rejected_and_cleared(table, good, duplicate, "duplicate header field");
}

void query_checks(CapContextABCRGTable& table, const Json& header) {
    const double length = header.at("road_lengths").array.front().number;
    const double cap = header.at("free_speed_cap_grid").at("min").number;
    const double crossing = header.at("context_contract").at("base_context_contract").at("crossing_time_seconds").number;
    table.requireCrossingTime(crossing);
    rejects_query([&] { table.requireCrossingTime(crossing + 1); }, "crossing mismatch");
    const double signal_step = header.at("signal_remaining_step").number;
    const std::size_t signal_points = static_cast<std::size_t>(header.at("signal_point_count").number);
    require(table.signalRemainingStep() == signal_step, "signal step is read from header");
    require(table.signalPointCount() == signal_points, "signal point count is read from header");
    table.requireSignalRemainingStep(signal_step);
    rejects_query([&] { table.requireSignalRemainingStep(signal_step + 1); }, "signal step mismatch");
    rejects_query([&] { table.requireSignalRemainingStep(std::nextafter(signal_step, std::numeric_limits<double>::infinity())); },
                  "signal step one-ULP mismatch is not approximately accepted");
    for (double invalid : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        rejects_query([&] { table.requireSignalRemainingStep(invalid); }, "signal step expectation must be finite and positive");
    }
    require(table.signalRemainingStep() == signal_step && table.signalPointCount() == signal_points,
            "expectation validation never modifies table metadata");
    require(table.overdueThreshold() == header.at("context_contract").at("context0_overdue_threshold_seconds").number, "overdue contract getter");
    auto query = [&](double query_length, int road, int lane, double query_cap, int context, const SignalRGFeatures& signal) {
        return table.lookup(query_length, road, lane, query_cap, context, signal);
    };
    const SignalRGFeatures sentinel = NoSignalRGFeatures();
    require(std::isfinite(query(length, 0, 0, cap, 0, sentinel)), "sentinel lookup exists");
    rejects_query([&] { query(length, -1, 0, cap, 0, sentinel); }, "negative flow");
    rejects_query([&] { query(length, 0, 1, cap, 0, sentinel); }, "lane exceeds road");
    rejects_query([&] { query(length, 0, 0, cap, 6, sentinel); }, "context 6");
    rejects_query([&] { query(length, 0, 0, 0, 0, sentinel); }, "zero cap");
    rejects_query([&] { query(-1, 0, 0, cap, 0, sentinel); }, "negative length");
    rejects_query([&] { query(length + 0.1234, 0, 0, cap, 0, sentinel); }, "unknown physical length");
    rejects_query([&] { table.quantizeSignal({'Y', 1, 'G', 1, 'Y', true}); }, "yellow is not formal signal type");
    rejects_query([&] { table.quantizeSignal({'G', -1, 'R', 1, 'G', false}); }, "negative remaining");
    rejects_query([&] { table.quantizeSignal({'G', 0, 'R', 0, 'G', false}); }, "non-sentinel zero combination");
    rejects_query([&] { table.quantizeSignal({'G', std::numeric_limits<double>::quiet_NaN(), 'R', 1, 'G', false}); }, "nonfinite remaining");
    const SignalRGFeatures quantized = table.quantizeSignal(sentinel);
    require(quantized.entry_type == 'G' && quantized.next_type == 'G'
            && quantized.entry_remaining_seconds == 0 && quantized.next_duration_seconds == 0, "sentinel stays exact G0G0");
    for (const Json& item : header.at("signal_catalog").array) {
        if (item.at("sentinel").boolean) { continue; }
        const char first = item.at("entry_type").string[0], second = item.at("next_type").string[0];
        const double duration = item.at("next_duration_seconds").number;
        const double maximum = item.at("entry_max_seconds").number;
        const auto& nodes = item.at("nodes").array;
        for (const Json& node : nodes) {
            const auto result = table.quantizeSignal({first, node.number, second, duration, 'Y', true});
            require(result.entry_remaining_seconds == node.number && result.next_duration_seconds == duration,
                    "all declared signal nodes quantize identically");
            require(result.source_state == 'Y' && result.yellow_skipped, "quantization preserves non-feature audit values");
        }
        require(table.quantizeSignal({first, nodes.front().number / 4, second, duration, first, false}).entry_remaining_seconds == nodes.front().number,
                "small positive duration maps to first positive node, not G0G0");
        rejects_query([&] { table.quantizeSignal({first, 0, second, duration, first, false}); }, "normal signal cannot use zero remaining");
        const double ulp = std::nextafter(maximum, std::numeric_limits<double>::infinity()) - maximum;
        const double within = maximum + 8.0 * ulp;
        require(table.quantizeSignal({first, within, second, duration, first, false}).entry_remaining_seconds == maximum,
                "at most eight ULP summation noise normalizes to exact endpoint");
        const double outside = std::nextafter(within, std::numeric_limits<double>::infinity());
        rejects_query([&] { table.quantizeSignal({first, outside, second, duration, first, false}); }, "beyond eight ULP remains strictly out of domain");
        if (nodes.size() >= 3) {
            const double lower = nodes[nodes.size() - 2].number, higher = nodes.back().number;
            const double midpoint = (lower + higher) / 2;
            const double wanted = midpoint - lower < higher - midpoint ? lower : higher;
            require(table.quantizeSignal({first, midpoint, second, duration, first, false}).entry_remaining_seconds == wanted,
                    "nearest-node midpoint uses higher on exact tie");
        }
    }
}

Json numeric(double value, bool integer = false) {
    Json result; result.kind = Json::Number; result.number = value; result.integer_token = integer; return result;
}

Json explicit_flow_policy(const std::vector<int>& nodes) {
    Json policy = JsonParser(R"JSON({"name":"nearest-explicit-integer-road-flow-v1","unit":"vehicles","tie":"higher","grid_origin":0,"preserve_lane_lower_bound":true,"preserve_road_upper_bound":true,"output_coverage":"all-legal-integer-road-flow","nodes":[]})JSON").parse();
    for (int node : nodes) { policy.object.at("nodes").array.push_back(numeric(node, true)); }
    return policy;
}

void set_explicit_flow_policy(Json& header, const std::vector<int>& nodes) {
    header.object["road_flow_quantization"] = explicit_flow_policy(nodes);
    const int lane_max = static_cast<int>(header.at("lane_flow_max").number);
    std::uint64_t count = 0;
    // Deliberately brute-force fixture count, independent of the reader's
    // lower_bound implementation and of the Python exporter's count helper.
    for (int lane = 0; lane <= lane_max; ++lane) {
        bool includes_lane = false;
        for (int node : nodes) {
            if (node >= lane) { ++count; }
            if (node == lane) { includes_lane = true; }
        }
        if (!includes_lane) { ++count; }
    }
    const std::uint64_t blocks = static_cast<std::uint64_t>(header.at("expanded_rows").number)
        / static_cast<std::uint64_t>(header.at("flow_count").number);
    header.object["prediction_rows"] = numeric(static_cast<double>(blocks * count), true);
}

std::vector<std::pair<std::string, std::function<void(Json&)>>> explicit_flow_corruptions() {
    return {
        {"unknown node policy", [](Json& h) { h.object.at("road_flow_quantization").object.at("name").string = "unknown"; }},
        {"explicit extra step", [](Json& h) { h.object.at("road_flow_quantization").object["step"] = numeric(1, true); }},
        {"explicit wrong tie", [](Json& h) { h.object.at("road_flow_quantization").object.at("tie").string = "lower"; }},
        {"explicit nodes absent", [](Json& h) { h.object.at("road_flow_quantization").object.erase("nodes"); }},
        {"explicit nodes not array", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes") = numeric(0, true); }},
        {"explicit empty nodes", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.clear(); }},
        {"explicit duplicate nodes", [](Json& h) { auto& n = h.object.at("road_flow_quantization").object.at("nodes").array; n.insert(n.begin(), n.front()); }},
        {"explicit unordered nodes", [](Json& h) { auto& n = h.object.at("road_flow_quantization").object.at("nodes").array; std::swap(n.front(), n.back()); }},
        {"explicit boolean node", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.front() = JsonParser("false").parse(); }},
        {"explicit floating token", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.front().integer_token = false; }},
        {"explicit fractional node", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.front() = numeric(.5); }},
        {"explicit negative node", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.front() = numeric(-1, true); }},
        {"explicit node above domain", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.back().number += 1; }},
        {"explicit missing zero", [](Json& h) { auto& n = h.object.at("road_flow_quantization").object.at("nodes").array; n.erase(n.begin()); }},
        {"explicit missing maximum", [](Json& h) { h.object.at("road_flow_quantization").object.at("nodes").array.pop_back(); }},
        {"explicit prediction mismatch", [](Json& h) { h.object.at("prediction_rows").number += 1; }},
        {"explicit missing prediction", [](Json& h) { h.object.erase("prediction_rows"); }},
    };
}

void explicit_full_rle_regression(const std::string& original) {
    const Json source = header_of(original);
    const int maximum = static_cast<int>(source.at("road_flow_max").number);
    require(maximum > 0, "explicit regression fixture has positive flow range");
    const std::string explicit_bytes = replace_header(original, [&](Json& h) { set_explicit_flow_policy(h, {0, maximum}); });
    CapContextABCRGTable reference, table;
    load_bytes(reference, original);
    load_bytes(table, explicit_bytes);
    const double length = source.at("road_lengths").array.front().number;
    const double cap = source.at("free_speed_cap_grid").at("min").number;
    for (int lane = 0; lane <= source.at("lane_flow_max").number; ++lane) {
        for (int road = lane; road <= maximum; ++road) {
            require(table.lookup(length, road, lane, cap, 0, NoSignalRGFeatures())
                == reference.lookup(length, road, lane, cap, 0, NoSignalRGFeatures()),
                "full RLE continues to use raw road flow; payload already includes mapping");
        }
    }
    for (const auto& change : explicit_flow_corruptions()) {
        require_rejected_and_cleared(table, explicit_bytes, replace_header(explicit_bytes, change.second), change.first);
    }
    // Old full-RLE headers without either optional field remain accepted.
    load_bytes(table, replace_header(original, [](Json& h) {
        h.object.erase("road_flow_quantization"); h.object.erase("prediction_rows");
    }));
    require(table.size() == reference.size(), "old full table without flow policy still loads");
}

void fractional_endpoint_regression(const std::string& template_bytes, double signal_step,
                                    const std::vector<double>& first_nodes, const std::vector<double>& second_nodes) {
    // Synthetic constant-prediction payload isolates the signal quantizer; its
    // values are deliberately not presented as model accuracy evidence.
    Json header = header_of(template_bytes);
    auto& catalog = header.object.at("signal_catalog").array;
    require(catalog.size() >= 3, "fixture supplies non-yellow catalog templates");
    catalog.resize(3);
    catalog[1].object.at("entry_max_seconds") = numeric(1.3);
    catalog[1].object.at("next_duration_seconds") = numeric(2.0);
    catalog[1].object.at("duration_key") = numeric(2000000000.0, true);
    catalog[1].object.at("nodes").array.clear();
    for (double node : first_nodes) { catalog[1].object.at("nodes").array.push_back(numeric(node)); }
    catalog[1].object.at("point_start") = numeric(1, true);
    catalog[2].object.at("entry_max_seconds") = numeric(2.0);
    catalog[2].object.at("next_duration_seconds") = numeric(1.3);
    catalog[2].object.at("duration_key") = numeric(1300000000.0, true);
    catalog[2].object.at("nodes").array.clear();
    for (double node : second_nodes) { catalog[2].object.at("nodes").array.push_back(numeric(node)); }
    catalog[2].object.at("point_start") = numeric(1 + first_nodes.size(), true);
    const unsigned points = static_cast<unsigned>(1 + first_nodes.size() + second_nodes.size());
    const unsigned rows = 6 * points;
    header.object.at("signal_remaining_step") = numeric(signal_step);
    header.object.at("signal_point_count") = numeric(points, true);
    header.object.at("road_lengths").array.resize(1);
    auto& grid = header.object.at("free_speed_cap_grid").object;
    grid.at("index_max") = grid.at("index_min");
    grid.at("max") = grid.at("min");
    header.object.at("road_flow_max") = numeric(0, true);
    header.object.at("lane_flow_max") = numeric(0, true);
    header.object.at("flow_count") = numeric(1, true);
    header.object.at("expanded_rows") = numeric(rows, true);
    header.object.at("group_count") = numeric(rows, true);
    header.object.at("run_count") = numeric(rows, true);
    std::string payload;
    for (unsigned index = 0; index <= rows; ++index) {
        for (unsigned shift = 0; shift < 32; shift += 8) { payload.push_back(static_cast<char>((index >> shift) & 255)); }
    }
    const double one = 1.0;
    std::uint64_t one_bits;
    std::memcpy(&one_bits, &one, 8);
    for (unsigned index = 0; index < rows; ++index) {
        payload.append(4, '\0');
        for (unsigned shift = 0; shift < 64; shift += 8) { payload.push_back(static_cast<char>((one_bits >> shift) & 255)); }
    }
    header.object.at("payload_bytes") = numeric(static_cast<double>(payload.size()), true);
    const std::string raw = "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1\n" + serialize(header) + '\n' + payload;
    CapContextABCRGTable table;
    load_bytes(table, rehashed_payload(raw));
    query_checks(table, header);
    SignalRGProgram movement({.1, .1, 1.1, 1, 2}, "GGGYR");
    const auto exact_clock = movement.query(0, .1);
    require(exact_clock.entry_remaining_seconds > 1.3, "fractional helper reproduces one-ULP endpoint overshoot");
    const auto quantized = table.quantizeSignal(exact_clock);
    require(quantized.entry_remaining_seconds == 1.3 && quantized.next_duration_seconds == 2,
            "legitimate fractional clock survives strict catalog lookup");
    auto invalid = exact_clock;
    invalid.entry_remaining_seconds = 1.300001;
    rejects_query([&] { table.quantizeSignal(invalid); }, "ordinary above-domain seconds are not clamped");
}

void no_internal_contract_checks(const std::string& seed) {
    const Json original = header_of(seed).at("signal_contract");
    require(original.at("version").string == "static-movement-rg-segments-v1", "version regression uses legacy fixture");
    for (int version = 1; version <= 4; ++version) {
        for (double crossing : {0.0, 3.4}) {
            const std::string bytes = replace_header(seed, [&](Json& h) {
                Json signal = original;
                signal.object["version"].string = "static-movement-rg-segments-v" + std::to_string(version);
                if (version >= 2) {
                    signal.object["supported_programs"].string = "fixed sequential cyclic STATIC; reject WAUT and queried-controller calendar changes; network green horizon frozen after initialization; external signal overrides unsupported";
                    signal.object["no_tls_placeholder"].object["interpretation"].string = "G0G0 identifies a final route occurrence with no outgoing movement only";
                    signal.object["unsignalized_connection"] = JsonParser(R"JSON({
                        "types":["G","G"],
                        "durations":"both equal network longest finite continuous movement green segment H",
                        "green_merge":"normalize G/g; merge adjacent phases and across cycle boundary; exclude yellow",
                        "network_scope":"external controlled connections; one fixed sequential static program per TLS",
                        "clock":"constant H at every reference time; no countdown or artificial phase event",
                        "missing_finite_green":"error when an unsignalized movement needs H; never invent a duration"
                    })JSON").parse();
                }
                if (version >= 3) {
                    signal.object["reference_time"].string = "planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized=0; otherwise context crossing seconds";
                    signal.object["crossing_policy"] = JsonParser(R"JSON({
                        "first_road":"zero",
                        "unsignalized_entry":"zero only for known nonsignalized incoming junction without TLS connection",
                        "other_entry":"base context crossing_time_seconds; controlled junction or TLS connection retains crossing",
                        "capacity":"unchanged waiting buffers, movement service interval and downstream storage"
                    })JSON").parse();
                }
                if (version == 4) {
                    signal.object["reference_time"].string = "planned_enter_time: decision_time plus incoming junction crossing; first/known unsignalized/no-internal network=0; otherwise context crossing seconds";
                    signal.object["crossing_policy"].object["network_without_internal_links"] = JsonParser(R"JSON("zero for every incoming junction when loaded SUMO network has no internal edges or via references")JSON").parse();
                }
                h.object["signal_contract"] = signal;
                h.object["context_contract"].object["base_context_contract"].object["crossing_time_seconds"].number = crossing;
            });
            CapContextABCRGTable table;
            load_bytes(table, bytes);
            table.requireCrossingTime(crossing);
            require(table.size() != 0, "all v1-v4 remain loadable for original controlled networks");
            if (version >= 3) table.requireUnsignalizedContract();
            else rejects_query([&] { table.requireUnsignalizedContract(); }, "v1/v2 unsignalized clock remains protected");
            if (version == 4 || crossing == 0) table.requireNoInternalLinksContract();
            else rejects_query([&] { table.requireNoInternalLinksContract(); }, "v1-v3 nonzero trained crossing rejected on no-internal entry");
            if (version == 4) {
                const std::string wrong = replace_header(bytes, [](Json& h) {
                    h.object["signal_contract"].object["crossing_policy"].object["network_without_internal_links"].string = "guess zero";
                });
                require_rejected_and_cleared(table, bytes, wrong, "incorrect v4 network policy");
                rejects_query([&] { table.requireNoInternalLinksContract(); }, "failed v4 load cleared no-internal permission");
            }
            load_bytes(table, bytes);
            table.clear();
            rejects_query([&] { table.requireNoInternalLinksContract(); }, "clear resets no-internal contract state");
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2 || argc > 3) { throw std::runtime_error("Usage: test_cap_context_abc_rg_table table.rle [queries.txt]"); }
        const std::string good = read_file(argv[1]);
        const Json header = header_of(good);
        CapContextABCRGTable table;
        require(table.signalRemainingStep() == 0 && table.signalPointCount() == 0, "empty table has no signal metadata");
        rejects_query([&] { table.requireSignalRemainingStep(1); }, "unloaded step expectation");
        load_bytes(table, good);
        require(table.size() == static_cast<std::uint64_t>(header.at("expanded_rows").number), "expanded size matches header");
        require(table.compressedBytes() == static_cast<std::size_t>(header.at("payload_bytes").number), "resident memory uses compressed payload exactly");
        std::size_t queries = 0;
        if (argc == 3) {
            std::ifstream input(argv[2]);
            if (!input) { throw std::runtime_error("Cannot open queries"); }
            std::string line;
            while (std::getline(input, line)) {
                if (line.empty() || line[0] == '#' || line.find("road_length ") == 0) { continue; }
                std::istringstream row(line);
                double length, cap, remaining, next_duration, expected;
                int road, lane, context;
                char first, second;
                if (!(row >> length >> road >> lane >> cap >> context >> first >> remaining >> second >> next_duration >> expected)) {
                    throw std::runtime_error("Invalid fixture query line");
                }
                const double actual = table.lookup(length, road, lane, cap, context, {first, remaining, second, next_duration, first, false});
                require(actual == expected, "Exact fixture prediction at query " + std::to_string(queries));
                ++queries;
            }
        }
        query_checks(table, header);
        no_internal_contract_checks(good);
        explicit_full_rle_regression(good);
        fractional_endpoint_regression(good, 1, {.001, 1, 1.3}, {.001, 1, 2});
        fractional_endpoint_regression(good, .5, {.001, .5, 1, 1.3}, {.001, .5, 1, 1.5, 2});
        fractional_endpoint_regression(good, 2, {.001, 1.3}, {.001, 2});
        fractional_endpoint_regression(good, 5, {.001, 1.3}, {.001, 2});
        fractional_endpoint_regression(good, 120, {.001, 1.3}, {.001, 2});
        corruption_checks(table, good);
        load_bytes(table, good);
        table.clear();
        require(table.size() == 0 && table.compressedBytes() == 0
                && table.signalRemainingStep() == 0 && table.signalPointCount() == 0, "explicit clear frees payload and metadata");
        rejects_query([&] { table.requireSignalRemainingStep(header.at("signal_remaining_step").number); }, "cleared step expectation");
        rejects_query([&] { table.quantizeSignal(NoSignalRGFeatures()); }, "empty quantizer");
        std::cout << "PASS cap_context_abc_rg_table checks=" << checks << " exact_queries=" << queries << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL cap_context_abc_rg_table checks=" << checks << ": " << error.what() << std::endl;
        return 1;
    }
}
