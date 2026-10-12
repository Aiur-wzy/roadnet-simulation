#include "cap_context_abc_rg_table.h"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

void load(CapContextABCRGTable& table, const char* path) {
    std::ifstream input(path, std::ios::binary);
    std::string magic;
    if (!std::getline(input, magic) || magic != "CAMS_CAP_CONTEXT_ABC_RG_RLE_V1")
        throw std::runtime_error("invalid fixture");
    table.load(input);
}

int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) return 2;
    try {
        CapContextABCRGTable table;
        load(table, argv[1]);
        bool refused = false;
        try { table.requireUnsignalizedContract(); } catch (const std::exception&) { refused = true; }
        if (!refused) throw std::runtime_error("legacy contract silently accepted uncontrolled exit");
        if (argc == 4) {
            load(table, argv[3]);
            refused = false;
            try { table.requireUnsignalizedContract(); } catch (const std::exception&) { refused = true; }
            if (!refused) throw std::runtime_error("v2 fixed crossing silently accepted the v3 zero-crossing clock");
        }
        load(table, argv[2]);
        table.requireUnsignalizedContract();
        auto point = table.quantizeSignal(AlwaysGreenSignalRGFeatures(6.0));
        if (point.entry_type != 'G' || point.next_type != 'G' || point.entry_remaining_seconds != 6.0
                || point.next_duration_seconds != 6.0) throw std::runtime_error("wrong constant point");
        auto terminal = table.quantizeSignal(NoSignalRGFeatures());
        if (terminal.entry_remaining_seconds != 0.0) throw std::runtime_error("terminal conflated with uncontrolled exit");
        table.clear();
        refused = false;
        try { table.requireUnsignalizedContract(); } catch (const std::exception&) { refused = true; }
        if (!refused) throw std::runtime_error("clear retained contract permission");
        std::cout << "unsignalized contract checks passed\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
