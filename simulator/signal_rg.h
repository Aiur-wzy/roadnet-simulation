#ifndef ROADNET_SIGNAL_RG_H
#define ROADNET_SIGNAL_RG_H

#include <cstddef>
#include <string>
#include <vector>

// Four formal R/G values and two diagnostics. Yellow duration is not a feature.
struct SignalRGFeatures {
    char entry_type;
    double entry_remaining_seconds;
    char next_type;
    double next_duration_seconds;
    char source_state;
    bool yellow_skipped;
};

// Only for a positively established final route occurrence (no outgoing road).
// It must not mask an unknown movement or an invalid constant-color program.
SignalRGFeatures NoSignalRGFeatures();

// A feature horizon, NOT remaining time or a synthetic switch calendar.
SignalRGFeatures AlwaysGreenSignalRGFeatures(double horizon);

// Longest bounded continuous G/g segment, merging phase/cycle boundaries.
// All-green (unbounded), all-red and all-yellow programs contribute zero.
double LongestFiniteGreenSegment(const std::vector<double>& durations,
                                const std::string& movement_states);

class SignalRGProgram {
public:
    // One character per TLS phase for ONE movement. Caller must establish that
    // the actual program is STATIC with sequential cyclic phase succession.
    SignalRGProgram(const std::vector<double>& durations, const std::string& movement_states);

    // Static SUMO calendar local time is (absolute_time - offset) mod cycle.
    // Finite negative calendar times/offsets are valid periodic extrapolations.
    SignalRGFeatures queryAtTime(double absolute_time, double offset = 0.0) const;

    // Same interface/half-open boundary as signal_rg_features.RGSignalProgram:
    // remaining == 0 means the next phase, not the old color with zero duration.
    SignalRGFeatures query(std::size_t phase_index, double phase_remaining_seconds,
                           double lookahead_seconds = 0.0) const;

    double cycleSeconds() const noexcept { return cycle_seconds_; }
    std::size_t phaseCount() const noexcept { return durations_.size(); }

private:
    SignalRGFeatures query_local(double position) const;

    std::vector<double> durations_;
    std::string states_;
    double cycle_seconds_;
    std::vector<double> phase_starts_;
    std::vector<double> phase_ends_;
    std::vector<std::size_t> phase_segments_;
    std::vector<double> following_in_segment_;
    std::string segment_states_;
    std::vector<double> segment_durations_;
    std::vector<std::size_t> next_nonyellow_;
};

#endif
