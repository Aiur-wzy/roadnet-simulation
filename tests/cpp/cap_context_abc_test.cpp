#include "cap_context.h"

#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace {
std::size_t checks = 0;
void expect(bool condition, const char* message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& operation, const char* message) {
    bool rejected = false;
    try { operation(); } catch (const std::runtime_error&) { rejected = true; }
    expect(rejected, message);
}
bool identical(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }
CapContextRecord record(int id, double enter = 10, double distance = 100,
                        double cap = 10, int route = 0) {
    return CapContextRecord{id, route, enter, distance, cap};
}
void base_identical(const CapContextResult& old, const CapContextResult& abc) {
    expect(old.context == abc.legacy_context, "legacy class changed");
    expect(old.eligible_count == abc.eligible_count, "eligible membership changed");
    expect(old.pending_count == abc.pending_count, "pending membership changed");
    expect(old.positive_count == abc.positive_count, "positive membership changed");
    expect(identical(old.maximum_gap, abc.maximum_gap), "maximum gap changed");
    expect(identical(old.gap_sum, abc.gap_sum), "compensated gap sum changed");
    expect(identical(old.potential, abc.potential), "P changed");
    expect(identical(old.equivalent_count, abc.equivalent_count), "E changed");
}
void test_states() {
    const auto ego = record(99);
    const std::vector<std::vector<CapContextRecord>> candidates = {
        {}, {record(0, 0, 150)}, {record(0, 0, 50)},
        {record(0, 10, 104)}, {record(0, 10, 105)},
        {record(0, 10, 105), record(1, 10, 105)}
    };
    const int expected[] = {0, 4, 5, 1, 2, 3};
    for (std::size_t i = 0; i != candidates.size(); ++i) {
        auto abc = scan_cap_context_abc(candidates[i], ego, 10);
        expect(abc.context == expected[i], "incorrect A/B/C or preserved legacy class");
        base_identical(scan_cap_context(candidates[i], ego, 10), abc);
    }
    auto ego_only = scan_cap_context_abc({ego}, ego, 10);
    expect(ego_only.context == 0 && ego_only.eligible_count == 0, "ego must not count as a candidate");
    expect(ego_only.max_overdue_seconds == 0 && ego_only.sum_overdue_seconds == 0 &&
           ego_only.overdue_count == 0 && ego_only.overdue_fraction == 0 &&
           ego_only.mean_overdue_seconds == 0, "empty audit must be zero");

    // Audit continuous values in nonzero classes too; never replace class 2
    // with C merely because a different eligible record is overdue.
    auto mixed = scan_cap_context_abc({record(0, 0, 50), record(1, 10, 120)}, ego, 10);
    expect(mixed.context == 2 && mixed.legacy_context == 2, "overdue changed a nonzero legacy class");
    expect(mixed.max_overdue_seconds == 5 && mixed.sum_overdue_seconds == 5 &&
           mixed.overdue_count == 1 && mixed.overdue_fraction == .5 &&
           mixed.mean_overdue_seconds == 2.5, "audit denominator must include every eligible record");
    auto maximum_not_sum = scan_cap_context_abc(
        {record(0, 0, 80), record(1, 0, 80), record(2, 0, 80)}, ego, 10, 3);
    expect(maximum_not_sum.context == 4 && maximum_not_sum.sum_overdue_seconds == 6 &&
           maximum_not_sum.max_overdue_seconds == 2 && maximum_not_sum.overdue_fraction == 1,
           "ABC threshold must apply to maximum, not sum or fraction");
}
void test_boundaries() {
    // At decision 2*epsilon and candidate finish epsilon the subtraction is
    // exactly epsilon, without the large-time cancellation of a shifted test.
    const double eps = CAP_CONTEXT_TIME_EPSILON;
    const double decision = 2 * eps;
    const auto ego = record(99, decision, 1, 1);
    const auto exact = record(0, 0, eps, 1);
    auto at = scan_cap_context_abc({exact}, ego, decision);
    expect(at.max_overdue_seconds == eps && at.context == 4 && at.overdue_count == 0,
           "overdue epsilon must be inclusive for B and exclusive for count");
    const auto above = record(0, 0, std::nextafter(eps, 0.0), 1);
    auto after = scan_cap_context_abc({above}, ego, decision);
    expect(after.max_overdue_seconds > eps && after.context == 5 && after.overdue_count == 1,
           "next overdue above epsilon must be C");
    // Positive threshold separate from the overdue_count epsilon.
    const double threshold = 2;
    for (const double finish : {2 - eps, std::nextafter(2 - eps, 0.0), 2.0}) {
        auto value = scan_cap_context_abc({record(0, 0, finish, 1)}, record(99, 4), 4, threshold);
        const double overdue = 4 - finish;
        expect(value.context == (overdue <= threshold + eps ? 4 : 5), "positive threshold boundary mismatch");
        expect(value.max_overdue_seconds == overdue && value.overdue_count == 1,
               "positive classification threshold changed the count threshold");
    }
    auto pending = scan_cap_context_abc(
        {record(0, std::nextafter(10.0, 11.0), 1000)}, record(99), 10);
    expect(pending.context == 0 && pending.pending_count == 1 && pending.eligible_count == 0,
           "epsilon must never activate a future reservation");
    auto active = scan_cap_context_abc({record(0, 10, 50)}, record(99), 10);
    expect(active.context == 4 && active.eligible_count == 1 && active.pending_count == 0,
           "exact planned-entry boundary must be eligible");
    // Use upstream decision=10, not the ego's future entry=13, for overdue.
    auto future_ego = scan_cap_context_abc({record(0, 0, 110)}, record(99, 13), 10);
    expect(future_ego.context == 4 && future_ego.max_overdue_seconds == 0,
           "overdue used future ego entry instead of decision time");
    const double tiny_late = 0.5 * eps;
    auto tolerant = scan_cap_context_abc({}, record(99, 0), tiny_late);
    expect(tolerant.context == 0, "ABC decision tolerance must match collection");
    rejects([&] { scan_cap_context({}, record(99, 0), tiny_late); }, "legacy decision boundary was weakened");
    rejects([&] { scan_cap_context_abc({}, record(99, 0), 2 * eps); }, "ABC accepted entry before decision beyond epsilon");
}
void test_ledger() {
    CapContextLedger ledger;
    const auto ego = record(99);
    ledger.reserve(1, 0, ego);
    ledger.reserve(1, 0, record(0, 0, 50, 10, 2));
    ledger.reserve(1, 1, record(1, 0, 10));
    ledger.reserve(2, 0, record(2, 0, 10));
    ledger.reserve(1, 0, record(3, 11, 1000));
    auto value = ledger.queryABC(1, 0, ego, 10);
    expect(value.context == 5 && value.eligible_count == 1 && value.pending_count == 1,
           "queryABC mixed lanes, roads, ego, or pending entries");
    base_identical(ledger.query(1, 0, ego, 10), value);
    rejects([&] { ledger.erase(0, 1); }, "stale occurrence release must fail");
    ledger.erase(0, 2); // Swap-delete moves the pending record; state remains valid.
    expect(ledger.queryABC(1, 0, ego, 10).context == 0, "released overdue record remained in query");
    ledger.erase(0, 2);
    ledger.reserve(1, 0, record(0, 10, 105, 10, 3));
    expect(ledger.queryABC(1, 0, ego, 10).context == 2, "revisit used stale occurrence");
    rejects([&] { ledger.reserve(1, 0, record(0)); }, "duplicate live vehicle must fail");
    expect(ledger.queryABC(3, 0, ego, 10).context == 0, "empty lane query not A");
    ledger.clear();
    expect(ledger.size() == 0 && ledger.queryABC(1, 0, ego, 10).context == 0, "clear left feature state");
}
void test_invalid() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const auto ego = record(99);
    for (double threshold : {-1.0, -1e-300, nan, inf, -inf})
        rejects([&] { scan_cap_context_abc({}, ego, 10, threshold); }, "invalid threshold accepted");
    for (double decision : {-1.0, 11.0, nan, inf, -inf})
        rejects([&] { scan_cap_context_abc({}, ego, decision); }, "invalid decision accepted");
    for (double distance : {0.0, -1.0, nan, inf})
        rejects([&] { scan_cap_context_abc({}, record(99, 10, distance), 10); }, "invalid ego geometry accepted");
    rejects([&] { scan_cap_context_abc({record(0, 0, 0)}, ego, 10); }, "invalid eligible geometry accepted");
    rejects([&] { scan_cap_context_abc({record(0, 0, 1e308, 1), record(1, 0, 1e308, 1)}, ego, 10); },
            "overflowing compensated sum accepted");
}
void test_randomized_legacy() {
    std::mt19937_64 random(20260914);
    std::uniform_real_distribution<double> unit(0, 1);
    for (int i = 0; i != 2000; ++i) {
        const double decision = 100 + 10000 * unit(random);
        const auto ego = record(999, decision + 3 * unit(random), 1 + 600 * unit(random), 3 + 17 * unit(random));
        std::vector<CapContextRecord> candidates;
        const int n = random() % 75;
        for (int j = 0; j != n; ++j)
            candidates.push_back(record(j, decision - 100 + 110 * unit(random),
                                        .1 + 1000 * unit(random), 3 + 17 * unit(random)));
        if (i % 2 == 0) candidates.push_back(ego);
        const auto abc = scan_cap_context_abc(candidates, ego, decision, (i % 4) * .5);
        base_identical(scan_cap_context(candidates, ego, decision), abc);
        expect(abc.context == abc.legacy_context || (abc.legacy_context == 0 && (abc.context == 4 || abc.context == 5)),
               "ABC modified a nonzero class in randomized check");
    }
}
bool read_record(std::istream& input, CapContextRecord& result) {
    return static_cast<bool>(input >> result.vehicle_id >> result.route_index >>
                             result.planned_enter_time >> result.distance >> result.free_speed_cap);
}
void run_stream() {
    // Per case: candidate_count decision threshold; ego's 5 fields; then N
    // records' 5 fields. IDs are integer, and all times nonnegative in C++.
    // Output has 14 tab-separated result fields, all floating values at 17 digits.
    std::size_t count;
    double decision, threshold;
    while (std::cin >> count >> decision >> threshold) {
        CapContextRecord ego;
        if (!read_record(std::cin, ego)) throw std::runtime_error("incomplete stream ego");
        std::vector<CapContextRecord> records(count);
        for (auto& item : records)
            if (!read_record(std::cin, item)) throw std::runtime_error("incomplete stream candidate");
        const auto result = scan_cap_context_abc(records, ego, decision, threshold);
        std::cout << std::setprecision(17) << result.context << '\t' << result.legacy_context
                  << '\t' << result.eligible_count << '\t' << result.pending_count << '\t' << result.positive_count
                  << '\t' << result.maximum_gap << '\t' << result.gap_sum << '\t' << result.potential
                  << '\t' << result.equivalent_count << '\t' << result.max_overdue_seconds
                  << '\t' << result.sum_overdue_seconds << '\t' << result.overdue_count
                  << '\t' << result.overdue_fraction << '\t' << result.mean_overdue_seconds << '\n';
    }
    if (!std::cin.eof()) throw std::runtime_error("invalid stream case header");
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--stream") {
            run_stream();
        } else if (argc == 1) {
            test_states();
            test_boundaries();
            test_ledger();
            test_invalid();
            test_randomized_legacy();
            std::cout << "ABC kernel checks passed: " << checks << '\n';
        } else {
            throw std::runtime_error("usage: CapContextABCTest [--stream]");
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ABC kernel failure: " << error.what() << '\n';
        return 1;
    }
}
