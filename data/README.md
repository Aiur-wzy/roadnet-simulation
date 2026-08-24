# Versioned SUMO Baselines

Git retains three canonical scenarios on `test_new.net.xml`:

| Scenario | Vehicles | Demand | SUMO configuration | Ground truth |
| --- | ---: | --- | --- | --- |
| `free` | 7,778 | `route/free.rou.xml` | `sumocfg/free.sumocfg` | `tripinfo/tripinfo_free.xml` |
| `tight` | 17,778 | `route/tight.rou.xml` | `sumocfg/tight.sumocfg` | `tripinfo/tripinfo_tight.xml` |
| `congested` | 19,444 | `route/congested.rou.xml` | `sumocfg/congested.sumocfg` | `tripinfo/tripinfo_congested.xml` |

`route/experiment_profiles_free_tight_congested.csv` records the generation
profile. Each configuration references `net/test_new.net.xml`, its matching
route, and its matching SUMO `tripinfo` output.

Other networks, routes, TraCI exports, and tripinfo files are local experiment
artifacts. `.gitignore` excludes them without deleting them from the working
directory. Reproduce a baseline with, for example:

```bash
sumo -c data/sumocfg/free.sumocfg
```
