#include "native_table_fixture.h"
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_set>

using namespace native_test;

int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string(argv[1]) == "--probe") {
            std::ifstream in(argv[2], std::ios::binary);
            std::string magic;
            std::getline(in, magic);
            check(magic == "CAMS_NATIVE_ABC_RG_DEMAND_V2", "wrong probe table magic");
            NativeRoadTimeTable reader;
            reader.load(in);
            double length, cap;
            int road, lane, context;
            SignalRGFeatures signal{};
            while (std::cin >> length >> road >> lane >> cap >> context >> signal.entry_type
                            >> signal.entry_remaining_seconds >> signal.next_type >> signal.next_duration_seconds) {
                double value = -1;
                std::uint64_t group;
                std::uint32_t representative;
                const bool hit = reader.lookupDemand(length, road, lane, cap, context, signal, value, group, representative);
                std::cout << group << ',' << representative << ',';
                if (hit) std::cout << std::setprecision(17) << value;
                else std::cout << "MISSING";
                std::cout << '\n';
            }
            if (!std::cin.eof()) throw std::runtime_error("malformed probe query");
            return 0;
        }
        NativeRoadTimeTable reader;
        auto meta = header(300000000);
        const std::uint64_t high = 5399999999ULL;
        const auto bytes = table(meta, {{1, 1, 1.0}, {high, 2, 12.25}});
        load(reader, bytes);
        auto unlimited = meta;
        set_number(unlimited.object["resource_limits"], "max_payload_bytes", 0);
        load(reader, table(unlimited, {{1, 1, 1.0}, {high, 2, 12.25}}));
        check(reader.demandEntryCount() == 2, "unlimited policy remains sparse and exact");
        auto unsupported_limit = meta;
        set_number(unsupported_limit.object["resource_limits"], "max_payload_bytes", 123);
        rejects([&] { load(reader, table(unsupported_limit, {})); }, "unknown resource policy rejected");
        load(reader, bytes);
        check(reader.groupCount() == 5400000000ULL && reader.size() == 54000000000ULL,
              "virtual dimensions above uint32 must load");
        check(reader.compressedBytes() == 40 && reader.demandEntryCount() == 2, "sparse RAM must not expand Cartesian grid");
        double value = -123;
        std::uint64_t group;
        std::uint32_t road;
        check(reader.lookupDemand(100, 2, 2, 300000000, 5, NoSignalRGFeatures(), value, group, road), "high u64 hit");
        check(group == high && road == 2 && value == 12.25, "high group preserved without truncation");
        value = -123;
        check(!reader.lookupDemand(100, 5, 2, 300000000, 5, NoSignalRGFeatures(), value, group, road)
              && group == high && road == 5 && value == -123, "legal miss returns exact key, no fallback");
        reader.requireCrossingTime(3);
        reader.requireDeparturePositionCorrection(true);
        rejects([&] { reader.requireDeparturePositionCorrection(false); }, "historical v2 first-road policy retained");
        rejects([&] { reader.lookupDemand(100, 1, 1, .5, 0, NoSignalRGFeatures(), value, group, road); }, "historical cap domain remains strict");
        reader.requireSignalRemainingStep(5);
        reader.requireUnsignalizedContract();
        reader.requireNoInternalLinksContract();
        rejects([&] { reader.requireNetworkFile("definitely_missing_native_network.xml"); }, "unverified network must fail");
        rejects([&] { reader.requireCrossingTime(2); }, "crossing mismatch must fail");
        rejects([&] { reader.requireSignalRemainingStep(1); }, "grid expectation mismatch must fail");
        rejects([&] { reader.lookupDemand(100, 1, 2, 1, 0, NoSignalRGFeatures(), value, group, road); }, "lane above road must fail");
        rejects([&] { reader.lookupDemand(100, 1, 1, 0, 0, NoSignalRGFeatures(), value, group, road); }, "invalid cap must fail");
        rejects([&] { reader.lookupDemand(101, 1, 1, 1, 0, NoSignalRGFeatures(), value, group, road); }, "missing length must fail");
        std::unordered_set<NativeDemandKey, NativeDemandKeyHash> keys;
        keys.insert({high, 2}); keys.insert({high - (1ULL << 32), 2}); keys.insert({high, 2});
        check(keys.size() == 2, "missing-key dedup must distinguish upper 32 bits");
        auto signal_meta = header(2, true);
        load(reader, table(signal_meta, {{139, 5, 9.0}}));
        SignalRGFeatures signal{'G', 7.5, 'R', 30, 'G', false};
        check(reader.lookupDemand(100, 4, 1, 1.5, 4, signal, value, group, road)
              && group == 139 && road == 5 && value == 9, "cap/signal/road nearest tie-higher preserved");
        check(reader.quantizeSignal(signal).entry_remaining_seconds == 10, "signal quantization tie");
        auto bounded = query_policy(header(2, true), false);
        load(reader, table(bounded, {{1, 1, 3.0}, {19, 1, 7.0}}));
        reader.requireDeparturePositionCorrection(false);
        rejects([&] { reader.requireDeparturePositionCorrection(true); }, "first-road switch/header mismatch fails");
        check(reader.clampsCap(), "v3 default cap clamp");
        check(reader.lookupDemand(100, 1, 1, .5, 0, NoSignalRGFeatures(), value, group, road)
              && group == 1 && value == 3, "cap below minimum hits minimum node");
        check(reader.lookupDemand(100, 1, 1, 1e100, 0, NoSignalRGFeatures(), value, group, road)
              && group == 19 && value == 7, "huge finite cap clamps before index conversion");
        check(!reader.lookupDemand(100, 5, 1, .5, 0, NoSignalRGFeatures(), value, group, road)
              && group == 1 && road == 5, "missing clamped node stays missing, never formula fallback");
        for (double cap : {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()})
            rejects([&] { reader.lookupDemand(100, 1, 1, cap, 0, NoSignalRGFeatures(), value, group, road); }, "invalid cap is not clamped");
        rejects([&] { reader.lookupDemand(100, 11, 1, .5, 0, NoSignalRGFeatures(), value, group, road); }, "road flow is not clamped");
        rejects([&] { reader.lookupDemand(100, 3, 3, .5, 0, NoSignalRGFeatures(), value, group, road); }, "lane flow is not clamped");
        rejects([&] { reader.lookupDemand(101, 1, 1, .5, 0, NoSignalRGFeatures(), value, group, road); }, "length is not clamped");
        signal.entry_remaining_seconds = 11;
        rejects([&] { reader.lookupDemand(100, 1, 1, .5, 0, signal, value, group, road); }, "signal time is not clamped");
        set_string(bounded.object["deployment_contract"], "cap_out_of_range", "error");
        load(reader, table(bounded, {}));
        rejects([&] { reader.lookupDemand(100, 1, 1, .5, 0, NoSignalRGFeatures(), value, group, road); }, "explicit strict cap policy");
        set_string(bounded.object["deployment_contract"], "cap_out_of_range", "anything");
        rejects([&] { load(reader, table(bounded, {})); }, "unknown policy rejected");
        bounded = query_policy(header());
        bounded.object["deployment_contract"].object.erase("first_road_position_correction");
        rejects([&] { load(reader, table(bounded, {})); }, "missing v3 policy rejected");
        for (const auto& corrupt : std::vector<std::pair<std::string, std::string>>{
                {"training_schema", "road-time-cap-context-abc-rg-v1"},
                {"source_schema", "old-shadow"}, {"payload_layout", "sorted-u32-u32-f64-v1"}}) {
            auto invalid = meta; set_string(invalid, corrupt.first, corrupt.second);
            rejects([&] { load(reader, table(invalid, {})); }, "old schema/layout must fail");
            check(reader.compressedBytes() == 0 && reader.size() == 0, "failed load clears object");
        }
        auto invalid = meta;
        set_string(invalid.object["model_contract"], "source_schema", "old-shadow");
        rejects([&] { load(reader, table(invalid, {})); }, "embedded old model must fail");
        invalid = meta;
        set_number(invalid, "group_count", 9007199254740992ULL);
        rejects([&] { load(reader, table(invalid, {})); }, "inexact JSON integer must fail");
        rejects([&] { load(reader, table(meta, {{high, 2, 12}, {high, 2, 12}})); }, "duplicate keys must fail");
        rejects([&] { load(reader, table(meta, {{high, 2, 12}, {1, 1, 1}})); }, "unsorted keys must fail");
        rejects([&] { load(reader, table(meta, {{high, 3, 12}})); }, "nonrepresentative road node must fail");
        rejects([&] { load(reader, table(meta, {{high, 2, 0.5}})); }, "current floor1 contract must remain");
        rejects([&] { load(reader, bytes.substr(0, bytes.size() - 1)); }, "truncation must fail");
        rejects([&] { load(reader, bytes + "x"); }, "trailing bytes must fail");
        auto bad_digest = bytes; bad_digest.back() ^= 1;
        rejects([&] { load(reader, bad_digest); }, "SHA mismatch must fail");
        load(reader, table(meta, {}));
        check(!reader.lookupDemand(100, 1, 1, 1, 0, NoSignalRGFeatures(), value, group, road), "empty sparse discovery cache is valid");
        std::cout << "native sparse reader tests passed (64-bit, integrity, strict missing, grids, semantic contracts)\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
