"""Historical shadow-context ABC/RG collector, retained for old experiments.

New SUMO-grounded training uses collection.TraCI_Python_SUMO_Native. This
historical entry is not the recommended Manhattan training-data collector.

Four signal inputs only. Yellow is skipped, not a third model category and
not added to either R/G duration. Z/M remain audit/future work, not new inputs.
"""
from __future__ import annotations

import argparse
from functools import partial
import sys

from roadnet_paths import source_path

import collection._shared.TraCI_Python_Cap_Context as shared
import collection._shared.TraCI_Python_Cap_Context_ABC as abc
from features.cap_context_features import ContextConfig, FEATURE_COLS
from features.cap_context_abc_features import ABC_AUDIT_FIELDS, abc_context_contract
from features.signal_rg_features import SIGNAL_FEATURE_COLS, SIGNAL_FEATURE_SCHEMA
from collection.traci_signal_rg import SIGNAL_AUDIT_FIELDS, TraCISignalRG


class ABCRGCapContextObserver(abc.ABCCapContextObserver):
    feature_schema = SIGNAL_FEATURE_SCHEMA
    feature_columns = tuple(FEATURE_COLS) + tuple(SIGNAL_FEATURE_COLS)
    extra_audit_fields = tuple(ABC_AUDIT_FIELDS) + tuple(SIGNAL_AUDIT_FIELDS) + (
        "incoming_junction_status", "incoming_crossing_seconds")

    def __init__(self, api, sumo_config, net_file, config=ContextConfig(), *, overdue_threshold_seconds=0.0,
                 network_factory=None):
        super().__init__(api, sumo_config, net_file, config,
                         overdue_threshold_seconds=overdue_threshold_seconds, network_factory=network_factory)
        self.signals = TraCISignalRG(api, net_file, self.metadata.source_paths)

    def entry_crossing_seconds(self, route, route_index):
        status = self.signals.incoming_junction_status(route, route_index)
        return 0.0 if (self.signals.network_without_internal_links or status in {"route_start", "unsignalized_node"}) else self.context_config.crossing_time_seconds

    def reserve(self, vehicle_id, route_index, decision_time):
        super().reserve(vehicle_id, route_index, decision_time)
        values = self.frozen[(vehicle_id, route_index)]
        values["incoming_junction_status"] = self.signals.incoming_junction_status(self.routes[vehicle_id], route_index)
        values["incoming_crossing_seconds"] = self.entry_crossing_seconds(self.routes[vehicle_id], route_index)
        values.update(self.signals.features(
            self.routes[vehicle_id], route_index, values["logical_lane_index"],
            decision_time, values["planned_enter_time"]))

    def feature_signal_contract(self):
        return self.signals.contract()


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--context0-overdue-threshold", type=float, default=0.0, metavar="SECONDS")
    parser.add_argument("--context-lane-policy", choices=("legacy", "road-pair-reselection-v1"),
                        default="legacy", help="Explicit opt-in to new C++ lane/flow rules; requires a new training sampling contract.")
    custom, remaining = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    abc_context_contract(ContextConfig(), custom.context0_overdue_threshold)
    if "--help" in remaining or "-h" in remaining:
        print("ABC+R/G: 9 formal features; existing five-feature trainers are incompatible.")
        parser.print_help()
    print("[cap_context_abc_rg_v1] ABC unchanged; two R/G segments; skip yellow without adding its time")
    observer_class = ABCRGCapContextObserver
    extra_sources = {}
    if custom.context_lane_policy != "legacy":
        raise ValueError("road-pair shadow admission collection was withdrawn: SUMO truth must not be gated by C++ capacity. Use python -m collection.TraCI_Python_SUMO_Native for new data; historical source snapshots remain in experiment evidence.")
    shared.main(
        remaining, observer_factory=partial(observer_class,
                                           overdue_threshold_seconds=custom.context0_overdue_threshold),
        collector_path=__file__, default_output_prefix="cap_context_abc_rg",
        extra_source_paths={
            "shared_cap_context_collector": shared.__file__,
            "abc_collector": abc.__file__,
            "abc_features": source_path("cap_context_abc_features.py"),
            "signal_features": source_path("signal_rg_features.py"),
            "traci_signal_reader": source_path("traci_signal_rg.py"),
            **extra_sources,
        },
    )


if __name__ == "__main__":
    main()
