#include "cap_context_abc_rg_table.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

// This probe deliberately uses the existing reader without road-flow mapping.
// Expected values are independently constructed by verify_roadflow_catch.py.
namespace {
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
std::uint64_t bits(double value) {
    std::uint64_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}
}

int main(int argc, char** argv) {
    try {
        require(argc == 5, "Usage: test_roadflow_catch_reader table queries expanded_rows signal_step");
        std::ifstream input(argv[1], std::ios::binary);
        std::string magic;
        std::getline(input, magic);
        require(magic == "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1", "wrong magic");
        CapContextABCRGTable table;
        table.load(input);
        require(table.size() == std::stoull(argv[3]), "expanded size changed");
        table.requireSignalRemainingStep(std::stod(argv[4]));
        std::ifstream queries(argv[2]);
        require(static_cast<bool>(queries), "missing queries");
        std::string line;
        std::size_t valid = 0, rejected = 0, row_number = 0;
        while (std::getline(queries, line)) {
            ++row_number;
            if (line.empty() || line[0] == '#') continue;
            std::istringstream row(line);
            double length, cap, remaining, duration;
            int road, lane, context, accept;
            char entry, next;
            std::uint64_t expected;
            require(static_cast<bool>(row >> accept >> length >> road >> lane >> cap >> context
                >> entry >> remaining >> next >> duration >> expected), "bad query");
            bool failed = false;
            double actual = 0;
            const SignalRGFeatures signal{entry, remaining, next, duration, 'Y', true};
            try { actual = table.lookup(length, road, lane, cap, context, signal); }
            catch (const std::exception&) { failed = true; }
            if (accept) {
                require(!failed && bits(actual) == expected,
                        "value mismatch at row " + std::to_string(row_number));
                ++valid;
            } else {
                require(failed, "invalid query accepted at row " + std::to_string(row_number));
                ++rejected;
            }
        }
        require(valid > 0, "no valid queries");
        std::cout << "PASS valid=" << valid << " rejected=" << rejected
                  << " expanded_rows=" << table.size() << " bytes=" << table.compressedBytes() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
