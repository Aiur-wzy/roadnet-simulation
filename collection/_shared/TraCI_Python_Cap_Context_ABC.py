"""Collect six-state ABC F13 features without changing road-time labels.

Only legacy context 0 is split: A=0, B=4, C=5. Legacy 1/2/3 are
unchanged. Overdue free-clock completion is a logical proxy, not measured
waiting time, and is never added to any of the six travel-time labels.
"""
from __future__ import annotations

import argparse
from functools import partial
import sys

from roadnet_paths import source_path

import collection._shared.TraCI_Python_Cap_Context as shared
from features.cap_context_features import ContextConfig
from features.cap_context_abc_features import (
    ABC_AUDIT_FIELDS, ABC_CONTEXT_IDS, ABC_FEATURE_SCHEMA,
    ABCShadowLedger, abc_context_contract,
)


class ABCCapContextObserver(shared.CapContextObserver):
    feature_schema = ABC_FEATURE_SCHEMA
    context_ids = ABC_CONTEXT_IDS
    extra_audit_fields = ABC_AUDIT_FIELDS

    def __init__(self, api, sumo_config, net_file, config=ContextConfig(), *,
                 overdue_threshold_seconds=0.0, network_factory=None):
        super().__init__(api, sumo_config, net_file, config, network_factory=network_factory)
        self.ledger = ABCShadowLedger(config, overdue_threshold_seconds=overdue_threshold_seconds)
        self.overdue_threshold_seconds = self.ledger.overdue_threshold_seconds

    def feature_context_contract(self):
        return abc_context_contract(self.context_config, self.overdue_threshold_seconds)


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--context0-overdue-threshold", type=float, default=0.0, metavar="SECONDS",
        help="ABC C requires max logical overdue > SECONDS + 1e-9; default 0. "
             "Not a waiting-time label or calibrated optimum. Raw summaries are always saved.",
    )
    custom, remaining = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    # Validate before opening SUMO or creating outputs, including on --help.
    abc_context_contract(ContextConfig(), custom.context0_overdue_threshold)
    if "--help" in remaining or "-h" in remaining:
        print("ABC-specific options (new six-state schema; old trainer is incompatible):")
        parser.print_help()
    observer_factory = partial(
        ABCCapContextObserver, overdue_threshold_seconds=custom.context0_overdue_threshold,
    )
    print(f"[cap_context_abc_v1] A=0 B=4 C=5; legacy 1/2/3 unchanged; "
          f"overdue threshold={custom.context0_overdue_threshold:g} s + 1e-9 s")
    shared.main(
        remaining, observer_factory=observer_factory, collector_path=__file__,
        extra_source_paths={
            "shared_cap_context_collector": shared.__file__,
            "abc_features": source_path("cap_context_abc_features.py"),
        },
        default_output_prefix="cap_context_abc",
    )


if __name__ == "__main__":
    main()
