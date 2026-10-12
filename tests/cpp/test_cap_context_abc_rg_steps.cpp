#include "cap_context_abc_rg_table.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

// Deliberately simple runner for independently generated expected query values.
// Synthetic predictions are mechanism tests, never accuracy evidence.
namespace {
void require(bool condition, const std::string& message) {
    if (!condition) { throw std::runtime_error(message); }
}

std::uint64_t bits(double value) {
    std::uint64_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

void clock_checks() {
    SignalRGProgram clock({2.0, 1.0, 3.5, 1.0, 5.25}, "GYGYR");
    const auto first = clock.query(0, 2.0);
    require(first.entry_type == 'G' && first.entry_remaining_seconds == 2.0
            && first.next_type == 'G' && first.next_duration_seconds == 3.5,
            "G-Y-G keeps two separate green segments");
    const auto yellow = clock.query(1, 0.5);
    require(yellow.entry_type == 'G' && yellow.entry_remaining_seconds == 3.5
            && yellow.next_type == 'R' && yellow.next_duration_seconds == 5.25
            && yellow.source_state == 'Y' && yellow.yellow_skipped,
            "yellow skips to full next segment without adding yellow duration");
    const auto boundary = clock.query(0, 0.0);
    require(boundary.entry_type == yellow.entry_type
            && boundary.entry_remaining_seconds == yellow.entry_remaining_seconds
            && boundary.next_type == yellow.next_type
            && boundary.next_duration_seconds == yellow.next_duration_seconds,
            "half-open phase boundary enters yellow skip semantics");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 6, "Usage: test_cap_context_abc_rg_steps table queries step points rows");
        std::ifstream table_input(argv[1], std::ios::binary);
        std::string magic;
        std::getline(table_input, magic);
        require(magic == "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1", "table magic");
        CapContextABCRGTable table;
        table.load(table_input);
        const double expected_step = std::stod(argv[3]);
        const std::size_t expected_points = static_cast<std::size_t>(std::stoull(argv[4]));
        require(bits(table.signalRemainingStep()) == bits(expected_step), "loaded step binary64");
        require(table.signalPointCount() == expected_points, "loaded point count");
        require(table.size() == std::stoull(argv[5]), "loaded expanded rows");
        table.requireSignalRemainingStep(expected_step);
        bool mismatch_rejected = false;
        try { table.requireSignalRemainingStep(expected_step + 0.125); }
        catch (const std::exception&) { mismatch_rejected = true; }
        require(mismatch_rejected, "mismatched expected step must reject");
        std::ifstream query_input(argv[2]);
        require(static_cast<bool>(query_input), "query file");
        std::string line;
        std::size_t valid = 0, rejected = 0;
        while (std::getline(query_input, line)) {
            if (line.empty() || line[0] == '#') { continue; }
            std::istringstream row(line);
            std::string id;
            int accept, road, lane, context;
            char entry, next;
            double length, cap, remaining, duration, wanted_node;
            std::uint64_t wanted_bits;
            require(static_cast<bool>(row >> id >> accept >> length >> road >> lane >> cap >> context
                    >> entry >> remaining >> next >> duration >> wanted_node >> wanted_bits),
                    "malformed query row");
            const SignalRGFeatures signal{entry, remaining, next, duration, 'Y', true};
            bool failed = false;
            double value = 0;
            SignalRGFeatures node = NoSignalRGFeatures();
            try {
                node = table.quantizeSignal(signal);
                value = table.lookup(length, road, lane, cap, context, signal);
            } catch (const std::exception&) { failed = true; }
            if (!accept) {
                require(failed, id + ": invalid query accepted");
                ++rejected;
                continue;
            }
            require(!failed, id + ": valid query rejected");
            require(bits(node.entry_remaining_seconds) == bits(wanted_node), id + ": wrong nearest node");
            require(node.entry_type == entry && node.next_type == next, id + ": signal types changed");
            require(node.source_state == 'Y' && node.yellow_skipped, id + ": diagnostics changed");
            require(bits(value) == wanted_bits, id + ": prediction binary64 mismatch");
            ++valid;
        }
        clock_checks();
        table.clear();
        require(table.signalRemainingStep() == 0 && table.signalPointCount() == 0, "clear step metadata");
        std::cout << "PASS valid=" << valid << " rejected=" << rejected
                  << " step=" << expected_step << " points=" << expected_points << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << std::endl;
        return 1;
    }
}
