#include "signal_rg.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::size_t checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool close(double first, double second) {
    return std::abs(first - second) <= 1e-12 * std::max(1.0, std::abs(second));
}

void same(const SignalRGFeatures& actual, const SignalRGFeatures& expected, const std::string& label) {
    require(actual.entry_type == expected.entry_type && close(actual.entry_remaining_seconds, expected.entry_remaining_seconds)
            && actual.next_type == expected.next_type && close(actual.next_duration_seconds, expected.next_duration_seconds)
            && actual.source_state == expected.source_state && actual.yellow_skipped == expected.yellow_skipped, label);
}

void rejects(const std::function<void()>& operation, const std::string& label) {
    bool rejected = false;
    try {
        operation();
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, label);
}

void fixed_cases() {
    require(LongestFiniteGreenSegment({2, 3, 4, 5}, "GgrG") == 10.0,
            "finite horizon joins G/g and cycle boundary, not max individual phase");
    require(LongestFiniteGreenSegment({2, 3, 4, 5}, "GGyG") == 10.0,
            "yellow bounds the green run");
    require(LongestFiniteGreenSegment({2, 3}, "Gg") == 0.0, "all-green is unbounded");
    require(LongestFiniteGreenSegment({2, 3}, "rR") == 0.0, "no green contributes zero");
    require(LongestFiniteGreenSegment({2, 3}, "yY") == 0.0, "all-yellow contributes zero");
    require(close(LongestFiniteGreenSegment({0.1, 0.2, 1, 0.3}, "GGrG"), 0.6),
            "fractional continuous green duration");
    same(AlwaysGreenSignalRGFeatures(10), {'G', 10, 'G', 10, 'G', false}, "constant H features");
    rejects([] { AlwaysGreenSignalRGFeatures(0); }, "no guessed zero horizon");
    rejects([] { AlwaysGreenSignalRGFeatures(INFINITY); }, "no infinite feature horizon");
    rejects([] { LongestFiniteGreenSegment({1, 2}, "Go"); }, "unsupported signal state in H");
    rejects([] { LongestFiniteGreenSegment({1, 0}, "Gr"); }, "invalid duration in H");
    SignalRGProgram program({24, 3, 15, 3, 24, 3, 15, 3}, "Gyrrrrrr");
    require(program.cycleSeconds() == 90.0 && program.phaseCount() == 8, "J17 dimensions");
    same(program.queryAtTime(20), {'G', 4, 'R', 63, 'G', false}, "e20 G4 R63");
    same(program.queryAtTime(25), {'R', 63, 'G', 24, 'Y', true}, "e25 yellow is not added");
    same(program.queryAtTime(28), {'R', 62, 'G', 24, 'R', false}, "e28 red62 not phase14");
    same(program.query(2, 14), program.queryAtTime(28), "runtime phase anchor equals calendar");
    same(program.query(0, 0), {'R', 63, 'G', 24, 'Y', true}, "phase remaining zero moves forward");
    same(program.queryAtTime(24), program.query(0, 0), "calendar half-open at yellow start");
    same(program.queryAtTime(27), {'R', 63, 'G', 24, 'R', false}, "calendar half-open at red start");
    same(program.queryAtTime(90), {'G', 24, 'R', 63, 'G', false}, "calendar wraps at exact cycle");
    same(program.query(7, 0), program.queryAtTime(90), "last phase zero wraps");
    same(program.query(7, 2, 3), program.queryAtTime(91), "lookahead through cycle");
    same(program.query(7, 2, 90.0 * 1e9 + 3), program.queryAtTime(91), "large lookahead");
    same(program.queryAtTime(68, 40), program.queryAtTime(28), "positive SUMO offset sign");
    same(program.queryAtTime(18, -10), program.queryAtTime(28), "negative offset");
    same(program.queryAtTime(28 - 90 * 2), program.queryAtTime(28), "negative periodic calendar time");
    same(program.queryAtTime(28 + 90.0 * 1e9), program.queryAtTime(28), "large calendar time");

    SignalRGProgram merged({2, 3, 4, 5}, "rGyr");
    same(merged.query(3, 4), {'R', 6, 'G', 3, 'R', false}, "same red across cycle");
    same(merged.query(2, 2), {'R', 7, 'G', 3, 'Y', true}, "yellow picks merged red");
    same(merged.query(0, 1), {'R', 1, 'G', 3, 'R', false}, "early cycle red remainder excludes previous cycle");
    SignalRGProgram green_merged({2, 3, 4, 5}, "gRyg");
    same(green_merged.query(3, 4), {'G', 6, 'R', 3, 'G', false}, "same green across cycle");
    SignalRGProgram case_merged({2, 5, 3, 4, 6}, "rRyGg");
    same(case_merged.query(0, 1), {'R', 6, 'G', 10, 'R', false}, "case-normalized red merge");
    same(case_merged.query(3, 2), {'G', 8, 'R', 7, 'G', false}, "case-normalized green merge");
    SignalRGProgram separated({5, 2, 7, 3}, "GyGr");
    same(separated.query(0, 4), {'G', 4, 'G', 7, 'G', false}, "yellow separates two green runs");
    same(separated.query(1, 1), {'G', 7, 'R', 3, 'Y', true}, "yellow G then R");
    SignalRGProgram repeating({5, 2}, "Gy");
    same(repeating.query(1, 1), {'G', 5, 'G', 5, 'Y', true}, "finite same-color run in next cycle");
    SignalRGProgram fractional({1.25, 0.5, 2.75, 0.25}, "rGyr");
    same(fractional.query(3, 0.125), {'R', 1.375, 'G', 0.5, 'R', false}, "fractional durations");
    same(fractional.query(2, 1.5), {'R', 1.5, 'G', 0.5, 'Y', true}, "fractional cross-cycle full run");
    SignalRGProgram decimal({0.1, 0.2, 0.3, 0.4}, "rGyr");
    same(decimal.query(3, 0.2), {'R', 0.3, 'G', 0.2, 'R', false}, "nonbinary decimal times");
    same(decimal.query(0, 0), decimal.query(1, 0.2), "nonbinary phase boundary");

    const double before = std::nextafter(24.0, 0.0);
    const double after = std::nextafter(24.0, 25.0);
    require(program.queryAtTime(before).source_state == 'G'
            && program.queryAtTime(before).entry_remaining_seconds > 0, "one ulp before boundary remains green");
    require(program.queryAtTime(after).source_state == 'Y', "one ulp after boundary is yellow");
    require(program.queryAtTime(-std::numeric_limits<double>::denorm_min()).source_state == 'R',
            "tiny negative calendar belongs to preceding cycle interval");
    same(NoSignalRGFeatures(), {'G', 0, 'G', 0, 'G', false}, "explicit no-TLS sentinel");
}

void invalid_cases() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    rejects([] { SignalRGProgram({}, ""); }, "empty program");
    rejects([] { SignalRGProgram({1, 2}, "R"); }, "length mismatch");
    for (const std::string states : {"R", "rr", "GGg", "yY"}) {
        rejects([states] { SignalRGProgram(std::vector<double>(states.size(), 1), states); }, "constant program");
    }
    for (const std::string states : {"Ru", "Go", "RO", "R-", "Rs"}) {
        rejects([states] { SignalRGProgram({1, 2}, states); }, "unknown state");
    }
    for (double invalid : {0.0, -1.0, nan, inf}) {
        rejects([invalid] { SignalRGProgram({invalid, 1}, "RG"); }, "invalid phase duration");
    }
    rejects([] { SignalRGProgram({1e308, 1e308}, "RG"); }, "overflow cycle");
    rejects([] { SignalRGProgram({1e308, 1}, "RG"); }, "unrepresentable phase boundary");
    SignalRGProgram program({24, 3, 63}, "Gyr");
    rejects([&] { program.query(3, 1); }, "bad phase index");
    rejects([&] { program.query(static_cast<std::size_t>(-1), 1); }, "negative phase cast");
    rejects([&] { program.query(0, 25); }, "remainder above duration");
    for (double invalid : {-1.0, nan, inf}) {
        rejects([&] { program.query(0, invalid); }, "invalid phase remaining");
        rejects([&] { program.query(0, 1, invalid); }, "invalid lookahead");
    }
    for (double invalid : {nan, inf, -inf}) {
        rejects([&] { program.queryAtTime(invalid); }, "invalid absolute time");
        rejects([&] { program.queryAtTime(1, invalid); }, "invalid offset");
    }
}

SignalRGFeatures tick_oracle(const std::string& ticks, std::size_t position) {
    const char source = ticks[position % ticks.size()];
    if (source == 'Y') {
        while (ticks[position % ticks.size()] == 'Y') { ++position; }
    }
    const char first = ticks[position % ticks.size()];
    double first_duration = 0;
    while (ticks[position % ticks.size()] == first) { ++position; ++first_duration; }
    while (ticks[position % ticks.size()] == 'Y') { ++position; }
    const char second = ticks[position % ticks.size()];
    double second_duration = 0;
    while (ticks[position % ticks.size()] == second) { ++position; ++second_duration; }
    return {first, first_duration, second, second_duration, source, source == 'Y'};
}

void exhaustive_programs() {
    std::mt19937 generator(27419);
    const std::string alphabet = "RGY";
    std::size_t programs = 0;
    for (std::size_t phase_count = 2; phase_count <= 5; ++phase_count) {
        std::size_t combinations = 1;
        for (std::size_t index = 0; index < phase_count; ++index) { combinations *= 3; }
        for (std::size_t combination = 0; combination < combinations; ++combination) {
            std::size_t code = combination;
            std::string states, ticks;
            std::vector<double> durations;
            bool different = false;
            for (std::size_t index = 0; index < phase_count; ++index) {
                const char state = alphabet[code % 3];
                code /= 3;
                different = different || (!states.empty() && state != states.front());
                states.push_back(state);
                const std::size_t duration = 1 + generator() % 5;
                durations.push_back(static_cast<double>(duration));
                ticks.append(duration, state);
            }
            if (!different) { continue; }
            ++programs;
            SignalRGProgram program(durations, states);
            for (std::size_t time = 0; time < ticks.size() * 3; ++time) {
                const SignalRGFeatures expected = tick_oracle(ticks, time);
                same(program.queryAtTime(static_cast<double>(time)), expected, "independent per-second calendar oracle");
                same(program.queryAtTime(static_cast<double>(time + 40), 40), expected, "independent positive-offset calendar oracle");
            }
            for (std::size_t phase = 0; phase < durations.size(); ++phase) {
                std::size_t start = 0;
                for (std::size_t index = 0; index < phase; ++index) { start += static_cast<std::size_t>(durations[index]); }
                for (std::size_t remaining = 0; remaining <= static_cast<std::size_t>(durations[phase]); ++remaining) {
                    const std::size_t time = start + static_cast<std::size_t>(durations[phase]) - remaining;
                    same(program.query(phase, static_cast<double>(remaining), 3), tick_oracle(ticks, time + 3),
                         "independent remaining/zero/three-second-lookahead oracle");
                }
            }
        }
    }
    require(programs == 348, "all 348 nonconstant small signal programs covered");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--batch-horizon") {
            std::string line;
            while (std::getline(std::cin, line)) {
                std::istringstream input(line);
                std::string durations_text, states;
                if (!(input >> durations_text >> states)) throw std::invalid_argument("Invalid horizon input");
                std::vector<double> durations;
                std::istringstream values(durations_text);
                std::string value;
                while (std::getline(values, value, ',')) durations.push_back(std::stod(value));
                std::cout << std::setprecision(17) << LongestFiniteGreenSegment(durations, states) << '\n';
            }
            return 0;
        }
        // Optional line-oriented bridge for independent Python/C++ comparisons.
        // Each stdin line: durations_csv movement_states absolute_time offset.
        if (argc == 2 && std::string(argv[1]) == "--batch-calendar") {
            std::string line;
            while (std::getline(std::cin, line)) {
                std::istringstream input(line);
                std::string durations_text, states;
                double absolute_time, offset;
                if (!(input >> durations_text >> states >> absolute_time >> offset)) {
                    throw std::invalid_argument("Invalid batch calendar input");
                }
                std::vector<double> durations;
                std::istringstream values(durations_text);
                std::string value;
                while (std::getline(values, value, ',')) { durations.push_back(std::stod(value)); }
                const SignalRGFeatures features = SignalRGProgram(durations, states).queryAtTime(absolute_time, offset);
                std::cout << std::setprecision(17) << features.entry_type << ' ' << features.entry_remaining_seconds << ' '
                          << features.next_type << ' ' << features.next_duration_seconds << ' '
                          << features.source_state << ' ' << static_cast<int>(features.yellow_skipped) << '\n';
            }
            return 0;
        }
        fixed_cases();
        invalid_cases();
        exhaustive_programs();
        std::cout << "PASS signal_rg checks=" << checks << " static_programs=348" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL signal_rg after checks=" << checks << ": " << error.what() << std::endl;
        return 1;
    }
}
