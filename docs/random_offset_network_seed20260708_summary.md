# Random-offset SUMO network candidate (seed 20260708)

## 1. Purpose

- Generate a fresh randomized-offset network candidate because a previous random-offset candidate caused SUMO to get stuck under high-flow conditions.
- This PR does not run SUMO or CAMS evaluation.

## 2. Source and output

- Source network: `data/test.net.xml`
- Output network: `data/test_random_offsets_seed20260708.net.xml`
- Seed: `20260708`
- Offset CSV: `docs/random_offset_network_seed20260708_offsets.csv`

## 3. Method

- For each `<tlLogic>`, compute the cycle length from the sum of child `<phase duration="...">` values.
- Generate a deterministic random offset in `[0, cycleLength)` using seed `20260708`.
- Use integer-second offsets by default.
- Update only the `offset` attribute.
- Preserve phase durations, phase states, connections, edges, lanes, and junctions.

## 4. Previous candidate note

Previous random-offset files found before generation:

- `data/test_random_offsets_seed20260706.net.xml`
- `docs/random_offset_network_offsets.csv`
- `docs/random_offset_network_summary.md`

These previous files were not modified and were not reused as input. The only source network used was `data/test.net.xml`.

## 5. Offset table

| tlID | programID | cycleLength | oldOffset | newOffset | status |
| --- | --- | ---: | ---: | ---: | --- |
| J17 | 0 | 90 | 45 | 4 | randomized |
| J20 | 0 | 90 | 0 | 68 | randomized |
| J21 | 0 | 90 | 36 | 4 | randomized |
| J22 | 0 | 90 | 72 | 54 | randomized |
| J24 | 0 | 90 | 0 | 7 | randomized |
| J25 | 0 | 90 | 0 | 47 | randomized |

## 6. Validation

- Original XML parsed successfully.
- New XML parsed successfully.
- `tlLogic` count unchanged: 6 original, 6 new.
- `tlLogic` keys unchanged.
- `tlLogic` state length validation passed for the original and new network.
- Original `data/test.net.xml` unchanged.

## 7. Important future-experiment note

Existing tripinfo files are not valid for this new offset network. Regenerate SUMO tripinfo using the new network before CAMS evaluation.

Future SUMO/CAMS runs should use:

```text
--sumo-net data/test_random_offsets_seed20260708.net.xml
```

with a matching newly generated tripinfo.

## 8. Limitation

- This random offset candidate is not guaranteed to avoid high-flow SUMO gridlock.
- It must be validated by a fresh SUMO run.
- If it still deadlocks, generate another candidate with a different seed or use a constrained offset search.

## 9. PR safety

- No SUMO run.
- No tripinfo generated.
- No CAMS eval run.
- No raw diag/log/eval files committed.
