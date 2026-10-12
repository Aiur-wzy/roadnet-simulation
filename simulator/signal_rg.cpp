#include "signal_rg.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

double finite_number(double value, const char* name, bool nonnegative) {
    if (!std::isfinite(value) || (nonnegative && value < 0.0)) {
        throw std::invalid_argument(std::string(name) + " must be finite" +
                                    (nonnegative ? " and nonnegative" : ""));
    }
    return value;
}

char normalized_color(char value) {
    switch (value) {
        case 'r': case 'R': return 'R';
        case 'g': case 'G': return 'G';
        case 'y': case 'Y': return 'Y';
        default: throw std::invalid_argument("Movement signal state must be r/R/g/G/y/Y");
    }
}

double positive_mod(double value, double cycle) {
    double result = std::fmod(value, cycle);
    if (result < 0.0) {
        result += cycle;
        // A tiny negative time can round to cycle when translated. Keep the
        // preceding half-open interval rather than index past the partition.
        if (result >= cycle) {
            result = std::nextafter(cycle, 0.0);
        }
    }
    return result == 0.0 ? 0.0 : result;
}

double modular_add(double first, double second, double cycle) {
    const double sum = first + second;
    if (std::isfinite(sum)) {
        return positive_mod(sum, cycle);
    }
    // Both operands are bounded by one cycle; this branch avoids overflow for
    // valid extremely long programs without changing ordinary-clock arithmetic.
    return first >= cycle - second ? first - (cycle - second) : first + second;
}

// Error-free transforms keep the segment total aligned with Python math.fsum
// without changing its separate, sequential phase-boundary partition.
double accurate_sum(const std::vector<double>& values) {
    std::vector<double> partials;
    for (double value : values) {
        std::size_t keep = 0;
        for (double partial : partials) {
            if (std::abs(value) < std::abs(partial)) {
                std::swap(value, partial);
            }
            const double high = value + partial;
            const double low = partial - (high - value);
            if (low != 0.0) {
                partials[keep++] = low;
            }
            value = high;
        }
        partials.resize(keep);
        partials.push_back(value);
    }
    double high = 0.0;
    double low = 0.0;
    std::size_t count = partials.size();
    if (count) {
        high = partials[--count];
        while (count) {
            const double value = high;
            const double partial = partials[--count];
            high = value + partial;
            low = partial - (high - value);
            if (low != 0.0) {
                break;
            }
        }
        // Half-even correction when the remaining nonzero partials have the
        // same sign. No epsilon snapping is applied to signal boundaries.
        if (count && ((low < 0.0 && partials[count - 1] < 0.0) ||
                      (low > 0.0 && partials[count - 1] > 0.0))) {
            const double doubled = low * 2.0;
            const double corrected = high + doubled;
            if (corrected - high == doubled) {
                high = corrected;
            }
        }
    }
    if (!std::isfinite(high)) {
        throw std::invalid_argument("Signal segment duration must be finite");
    }
    return high;
}

}  // namespace

SignalRGFeatures NoSignalRGFeatures() {
    return {'G', 0.0, 'G', 0.0, 'G', false};
}

SignalRGFeatures AlwaysGreenSignalRGFeatures(double horizon) {
    if (!std::isfinite(horizon) || horizon <= 0.0)
        throw std::invalid_argument("Unsignalized R/G horizon requires a finite positive network green segment");
    return {'G', horizon, 'G', horizon, 'G', false};
}

double LongestFiniteGreenSegment(const std::vector<double>& durations,
                                const std::string& movement_states) {
    if (durations.empty() || durations.size() != movement_states.size())
        throw std::invalid_argument("Durations and movement states require the same positive length");
    std::string colors;
    double cycle = 0.0;
    for (std::size_t i = 0; i < durations.size(); ++i) {
        if (!std::isfinite(durations[i]) || durations[i] <= 0.0 ||
            !std::isfinite(cycle + durations[i]) || cycle + durations[i] <= cycle)
            throw std::invalid_argument("Every phase needs finite increasing positive duration");
        cycle += durations[i];
        colors.push_back(normalized_color(movement_states[i]));
    }
    const auto nongreen = colors.find_first_not_of('G');
    if (nongreen == std::string::npos) return 0.0; // unbounded green, not one cycle
    double longest = 0.0;
    std::vector<double> run;
    for (std::size_t offset = 1; offset <= colors.size(); ++offset) {
        const std::size_t phase = (nongreen + offset) % colors.size();
        if (colors[phase] == 'G') {
            run.push_back(durations[phase]);
        } else if (!run.empty()) {
            longest = std::max(longest, accurate_sum(run));
            run.clear();
        }
    }
    return longest;
}

SignalRGProgram::SignalRGProgram(const std::vector<double>& durations,
                                 const std::string& movement_states)
    : durations_(durations), cycle_seconds_(0.0) {
    if (durations.empty() || durations.size() != movement_states.size()) {
        throw std::invalid_argument("Durations and movement states require the same positive length");
    }
    states_.reserve(movement_states.size());
    for (char color : movement_states) {
        states_.push_back(normalized_color(color));
    }
    if (std::all_of(states_.begin(), states_.end(), [this](char color) { return color == states_[0]; })) {
        throw std::invalid_argument("Constant-color/all-yellow program has no two finite non-yellow segments");
    }
    for (double duration : durations_) {
        if (!std::isfinite(duration) || duration <= 0.0) {
            throw std::invalid_argument("Every phase duration must be finite and positive");
        }
        phase_starts_.push_back(cycle_seconds_);
        const double end = cycle_seconds_ + duration;
        if (!std::isfinite(end) || end <= cycle_seconds_) {
            throw std::invalid_argument("Signal phase boundaries are not representable as finite increasing doubles");
        }
        phase_ends_.push_back(end);
        cycle_seconds_ = end;
    }

    const std::size_t count = durations_.size();
    std::size_t start = 0;
    while (states_[start] == states_[(start + count - 1) % count]) {
        ++start;
    }
    std::vector<std::vector<std::size_t>> segment_phases;
    for (std::size_t advance = 0; advance < count; ++advance) {
        const std::size_t phase = (start + advance) % count;
        if (segment_states_.empty() || states_[phase] != segment_states_.back()) {
            segment_states_.push_back(states_[phase]);
            segment_phases.emplace_back();
        }
        segment_phases.back().push_back(phase);
    }
    phase_segments_.resize(count);
    following_in_segment_.resize(count);
    for (std::size_t segment = 0; segment < segment_phases.size(); ++segment) {
        const std::vector<std::size_t>& phases = segment_phases[segment];
        std::vector<double> values;
        values.reserve(phases.size());
        for (std::size_t phase : phases) {
            values.push_back(durations_[phase]);
        }
        segment_durations_.push_back(accurate_sum(values));
        double tail = 0.0;
        for (auto iterator = phases.rbegin(); iterator != phases.rend(); ++iterator) {
            phase_segments_[*iterator] = segment;
            following_in_segment_[*iterator] = tail;
            tail += durations_[*iterator];
        }
    }
    std::vector<std::size_t> nonyellow;
    for (std::size_t segment = 0; segment < segment_states_.size(); ++segment) {
        if (segment_states_[segment] != 'Y') {
            nonyellow.push_back(segment);
        }
    }
    if (nonyellow.empty()) {
        throw std::invalid_argument("Program has no finite non-yellow segment");
    }
    for (std::size_t segment = 0; segment < segment_states_.size(); ++segment) {
        const auto next = std::upper_bound(nonyellow.begin(), nonyellow.end(), segment);
        next_nonyellow_.push_back(next == nonyellow.end() ? nonyellow.front() : *next);
    }
}

SignalRGFeatures SignalRGProgram::queryAtTime(double absolute_time, double offset) const {
    finite_number(absolute_time, "absolute_time", false);
    finite_number(offset, "offset", false);
    double difference = absolute_time - offset;
    if (!std::isfinite(difference)) {
        difference = positive_mod(absolute_time, cycle_seconds_) - positive_mod(offset, cycle_seconds_);
    }
    return query_local(positive_mod(difference, cycle_seconds_));
}

SignalRGFeatures SignalRGProgram::query(std::size_t phase_index, double phase_remaining_seconds,
                                       double lookahead_seconds) const {
    if (phase_index >= durations_.size()) {
        throw std::invalid_argument("phase_index outside signal program");
    }
    const double remaining = finite_number(phase_remaining_seconds, "phase_remaining_seconds", true);
    if (remaining > durations_[phase_index]) {
        throw std::invalid_argument("phase_remaining_seconds exceeds the phase duration");
    }
    const double lookahead = finite_number(lookahead_seconds, "lookahead_seconds", true);
    const double base = positive_mod(phase_starts_[phase_index] + (durations_[phase_index] - remaining), cycle_seconds_);
    return query_local(modular_add(base, positive_mod(lookahead, cycle_seconds_), cycle_seconds_));
}

SignalRGFeatures SignalRGProgram::query_local(double position) const {
    const auto phase_it = std::upper_bound(phase_ends_.begin(), phase_ends_.end(), position);
    if (phase_it == phase_ends_.end()) {
        throw std::logic_error("Internal signal position outside half-open cycle");
    }
    const std::size_t phase = static_cast<std::size_t>(phase_it - phase_ends_.begin());
    const char source_state = states_[phase];
    const bool skipped = source_state == 'Y';
    const std::size_t original_segment = phase_segments_[phase];
    const std::size_t entry = skipped ? next_nonyellow_[original_segment] : original_segment;
    const double entry_remaining = skipped ? segment_durations_[entry]
        : phase_ends_[phase] - position + following_in_segment_[phase];
    const std::size_t next = next_nonyellow_[entry];
    return {segment_states_[entry], entry_remaining, segment_states_[next],
            segment_durations_[next], source_state, skipped};
}
