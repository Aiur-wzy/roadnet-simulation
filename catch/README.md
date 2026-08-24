# Lookup-Table Artifacts

This directory intentionally keeps all exported catch results in Git.

- `*.txt` files contain simulator lookup tables.
- `unique_feature_combinations_*.csv` files record enumerated feature keys.
- `turn_type_mapping_*.json` files preserve categorical encoding metadata.

Treat these files as immutable model artifacts. When replacing a table, retain
its feature-combination and turn-mapping metadata under the same threshold name.
