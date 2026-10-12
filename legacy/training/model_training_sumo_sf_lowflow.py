#!/usr/bin/env python3
"""Train five-feature road-time models with strict road_flow < 5 weighted 3:1.

Uses the shared SF trainer, file split and label contract. No samples are
duplicated, no extra predictive features are added, and external metrics are
unweighted. The original model_training_sumo_sf.py still defaults to equal weights.
"""
from __future__ import annotations

import training._shared.model_training_sumo_sf as training


def build_parser():
    parser = training.build_parser()
    parser.description = __doc__
    parser.set_defaults(sample_weighting="low-flow", low_flow_cutoff=5, low_flow_weight=3.0)
    return parser


def main(argv=None):
    return training.main(argv, parser=build_parser(), entrypoint_file=__file__)


if __name__ == "__main__":
    main()
