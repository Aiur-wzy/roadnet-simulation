#include "cap_context_table.h"
#include "cap_context_json.h"
#include "test_cap_context_fixture.h"

#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>

namespace {
using namespace cap_context_test;
int checks = 0;
void check(bool value, const std::string& message) {
    ++checks; if (!value) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& call, const std::string& message) {
    ++checks;
    try { call(); } catch (const std::runtime_error&) { return; }
    throw std::runtime_error("expected rejection: " + message);
}
void load(CapContextTable& table, const std::string& contents) {
    std::istringstream input(contents, std::ios::binary); table.load(input);
}
void bad(const std::string& contents, const std::string& message) {
    CapContextTable table; load(table, fixture());
    rejects([&] { load(table, contents); }, message);
    check(table.size() == 0 && table.label().empty(), message + " retained partially loaded table");
}
void unit_tests() {
    check(digest("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", "SHA empty");
    check(digest("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", "SHA abc");
    check(digest(std::string(1000000, 'a')) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0", "SHA multiblock");
    CapContextTable table; load(table, fixture());
    check(table.size() == 144, "expanded count");
    std::size_t dense = 0;
    for (int s=0; s<2; ++s) for (int cap=0; cap<2; ++cap) for (int context=0; context<4; ++context)
        for (int lane=0; lane<=2; ++lane) for (int road=lane; road<=3; ++road) {
            const double expected = (((s*2+cap)*4+context)*3+lane)+1;
            check(table.lookup(s ? 320 : 100, road, lane, (cap+1)*5, context) == expected, "lookup coordinate");
            check(table.denseValues().at(dense++) == expected, "expanded order");
        }
    check(table.lookup(100, 0, 0, 7.5, 0) == 13, "cap midpoint ties upward");
    check(table.lookup(100, 0, 0, 7.499999, 0) == 1, "cap below midpoint");
    check(table.lookup(100, 0, 0, 5-5e-13, 0) == 1, "lower raw tolerance");
    check(table.lookup(100, 0, 0, 10+5e-13, 0) == 13, "upper raw tolerance");
    check(table.lookup(100.000000001, 0, 0, 5, 0) == 1, "physical length quantization");
    for (double cap : {0.0,-1.0,5-2e-12,10+2e-12,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
        rejects([&] { table.lookup(100,0,0,cap,0); }, "cap outside range");
    rejects([&] { table.lookup(101,0,0,5,0); }, "missing length");
    rejects([&] { table.lookup(100,0,1,5,0); }, "lane exceeds road");
    rejects([&] { table.lookup(100,4,1,5,0); }, "road outside domain");
    rejects([&] { table.lookup(100,3,3,5,0); }, "lane outside domain");
    rejects([&] { table.lookup(100,0,0,5,-1); }, "negative context");
    rejects([&] { table.lookup(100,0,0,5,4); }, "fifth context");
    table.requireCrossingTime(3);
    rejects([&] { table.requireCrossingTime(2); }, "clock mismatch");
    const std::vector<std::pair<std::string,std::string>> mutations = {
        {"CAMS_CAP_CONTEXT_RLE_V1","CAMS_CAP_CONTEXT_RLE_V2"},
        {"road-time-cap-context-v1","old-schema"},
        {"\"road_length\",\"road_flow\"","\"road_flow\",\"road_length\""},
        {"travel_time_label_gt_0p1","travel_time_label"},
        {"\"context_count\":4","\"context_count\":3"},
        {"\"p_threshold\":0.05","\"p_threshold\":0.1"},
        {"\"e_threshold\":2.0","\"e_threshold\":1.5"},
        {"\"epsilon_seconds\":1e-9","\"epsilon_seconds\":1e-8"},
        {"\"crossing_time_seconds\":3","\"crossing_time_seconds\":-1"},
        {"incoming-logical-lane-v1","physical-order"},
        {"base-first-v1","full-first"},
        {"upstream-discharge-known-reservations-v1","future-enter"},
        {"\"static_key_scale\":10000","\"static_key_scale\":1"},
        {"[100,320]","[320,100]"},{"[100,320]","[100,100]"},
        {"[100,320]","[100.00001,320]"},
        {"\"step\":5","\"step\":0"},{"\"min\":5","\"min\":4"},
        {"\"max\":10","\"max\":11"},{"\"index_min\":1","\"index_min\":0"},
        {"\"index_max\":2","\"index_max\":1"},
        {"\"road_flow_max\":3","\"road_flow_max\":65536"},
        {"\"lane_flow_max\":2","\"lane_flow_max\":4"},
        {"\"flow_count\":9","\"flow_count\":8"},
        {"\"expanded_rows\":144","\"expanded_rows\":50000001"},
        {"\"group_count\":48","\"group_count\":47"},
        {"\"run_count\":48","\"run_count\":47"},
        {"\"run_count\":48","\"run_count\":145"},
        {"\"payload_bytes\":772","\"payload_bytes\":771"},
        {"\"context_count\":4","\"context_count\":4,\"context_count\":4"},
        {"\"road_flow_max\":3","\"road_flow_max\":true"},
        {"\"source_label\":","\"source_label\":null,\"other\":"},
        {"\"raw_boundary_tolerance\":1e-12","\"raw_boundary_tolerance\":1e-9"},
        {"\"scaled_boundary_tolerance\":1e-9","\"scaled_boundary_tolerance\":0"},
        {"records_u16_u16_f64_le-v1","records_u16_u16_f32_le-v1"}
    };
    for (const auto& change : mutations) bad(replace_once(fixture(),change.first,change.second),change.first);
    bad(fixture()+"x", "trailing bytes");
    bad(fixture().substr(0,fixture().size()-1), "truncated payload");
    bad(std::string(1024*1024+1,' ') + "\n", "header size bound");
    std::string corrupt=fixture(); corrupt.back() ^= 1; bad(corrupt,"SHA corruption");
    auto malformed_payload = [&](std::string bytes, const std::string& reason) {
        bad(metadata(bytes)+"\n"+bytes, reason); // recompute SHA to exercise structural checks
    };
    std::string bytes=payload(); bytes[0]=1; malformed_payload(bytes,"first offset");
    bytes=payload(); bytes[4]=0; malformed_payload(bytes,"empty group");
    bytes=payload(); bytes[48*4]=47; malformed_payload(bytes,"last offset");
    bytes=payload(); bytes[49*4]=1; malformed_payload(bytes,"first run gap");
    bytes=payload(); bytes[49*4+2]=4; malformed_payload(bytes,"run past max");
    bytes=payload(); bytes[49*4+2]=2; malformed_payload(bytes,"incomplete coverage");
    bytes=payload(); bytes.replace(49*4+4,8,std::string(8,'\0')); malformed_payload(bytes,"zero prediction");
    bytes=payload(); std::string nan; append_double(nan,std::numeric_limits<double>::quiet_NaN());
    bytes.replace(49*4+4,8,nan); malformed_payload(bytes,"NaN prediction");
    // Split the first run into adjacent identical values; valid coverage is noncanonical.
    bytes.clear(); append_integer(bytes,0,4);
    for(unsigned group=1;group<=48;++group) append_integer(bytes,group+1,4);
    append_integer(bytes,0,2);append_integer(bytes,0,2);append_double(bytes,1);
    append_integer(bytes,1,2);append_integer(bytes,3,2);append_double(bytes,1);
    bytes += payload().substr(49*4+12);
    auto meta=replace_once(metadata(bytes),"\"run_count\":48","\"run_count\":49");
    meta=replace_once(meta,"\"payload_bytes\":772","\"payload_bytes\":784");
    bad(meta+"\n"+bytes,"adjacent identical runs");
    // Distinct split values must be accepted and preserve all decoded cells.
    std::string two;append_double(two,2);bytes.replace(49*4+12+4,8,two);
    meta=replace_once(metadata(bytes),"\"run_count\":48","\"run_count\":49");
    meta=replace_once(meta,"\"payload_bytes\":772","\"payload_bytes\":784");
    load(table,meta+"\n"+bytes);
    check(table.lookup(100,0,0,5,0)==1 && table.lookup(100,1,0,5,0)==2,
          "distinct adjacent runs decode exactly");
}

void audit_file(const std::string& path) {
    std::ifstream input(path,std::ios::binary);
    std::string magic, header;
    if (!std::getline(input,magic) || magic!="CAMS_CAP_CONTEXT_RLE_V1") throw std::runtime_error("bad magic");
    const auto position=input.tellg();std::getline(input,header);input.seekg(position);
    CapContextTable table; table.load(input);
    const auto meta=cap_context_json::JsonParser(header).parse();
    const auto& grid=meta.at("free_speed_cap_grid");
    CapContextSha256 dense_sha,lookup_sha;
    for(const auto value:table.denseValues()) {
        std::string bytes;append_double(bytes,value);
        dense_sha.update(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size());
    }
    std::size_t count=0;
    const int lo=static_cast<int>(grid.at("index_min").number),hi=static_cast<int>(grid.at("index_max").number);
    for(const auto& length:meta.at("road_lengths").array) for(std::int64_t cap=lo;cap<=hi;++cap)
        for(int context=0;context<4;++context) for(int lane=0;lane<=meta.at("lane_flow_max").number;++lane)
            for(int road=lane;road<=meta.at("road_flow_max").number;++road) {
                const double value=table.lookup(length.number,road,lane,cap*grid.at("step").number,context);
                if(value!=table.denseValues().at(count++))throw std::runtime_error("lookup differs from dense");
                std::string bytes;append_double(bytes,value);
                lookup_sha.update(reinterpret_cast<const unsigned char*>(bytes.data()),bytes.size());
            }
    std::cout<<"{\"rows\":"<<count<<",\"dense_sha256\":\""<<dense_sha.finish()
             <<"\",\"lookup_sha256\":\""<<lookup_sha.finish()<<"\",\"all_lookup_values_equal\":true}\n";
}
}
int main(int argc,char**argv) {
    try {
        if(argc==2) { audit_file(argv[1]);return 0; }
        unit_tests();std::cout<<"Cap/context table checks passed: "<<checks<<"\n";return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<"\n";return 1; }
}
