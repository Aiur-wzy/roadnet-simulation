# Random-Offset SUMO Network Summary

## Purpose

This change generates a new SUMO network with randomized traffic-light offsets. The network is intended for future SUMO tripinfo regeneration and later CAMS evaluation to test whether different signal offsets can reduce false-blocking / overprediction issues.

No evaluation is run here because changing traffic-light offsets invalidates existing tripinfo.

## Source and output

- Source network: `data/test.net.xml`
- Generated network: `data/test_random_offsets_seed20260706.net.xml`
- Seed: `20260706`
- Offset CSV: `docs/random_offset_network_offsets.csv`

## Method

For each `<tlLogic>` element, the script computes the cycle length as the sum of child `<phase duration="...">` values. It then generates a deterministic random offset in `[0, cycleLength)` using seed `20260706` and updates only the `offset` attribute.

The generated network preserves traffic-light phases, phase states, connections, edges, and lanes. No route files, tripinfo files, or evaluation outputs are generated or modified.

## Offset summary

| tlID | programID | cycleLength | oldOffset | newOffset | status |
| --- | --- | ---: | ---: | ---: | --- |
| J17 | 0 | 90 | 45 | 2 | randomized |
| J20 | 0 | 90 | 0 | 53 | randomized |
| J21 | 0 | 90 | 36 | 16 | randomized |
| J22 | 0 | 90 | 72 | 16 | randomized |
| J24 | 0 | 90 | 0 | 2 | randomized |
| J25 | 0 | 90 | 0 | 11 | randomized |

## Validation

- Original XML parsed successfully.
- New XML parsed successfully.
- `tlLogic` count unchanged: 6 original, 6 generated.
- `tlLogic` keys unchanged.
- `tlLogic` state length validation passed for both original and generated networks.
- Original `data/test.net.xml` was not overwritten.

## Important note for future experiments

Existing `data/tripinfo_full_no_change.xml` and other tripinfo files are no longer valid for this new random-offset network. Regenerate SUMO tripinfo using the new network before running CAMS evaluation.

Future CAMS commands should use:

```sh
--sumo-net data/test_random_offsets_seed20260706.net.xml
```

together with a newly generated matching tripinfo.

## PR safety

- No SUMO run.
- No CAMS eval run.
- No tripinfo generated.
- No diag/log files committed.
