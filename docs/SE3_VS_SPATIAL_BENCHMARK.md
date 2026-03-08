# se3 vs Spatial `ST_Affine` Benchmark

This benchmark compares:
- `se3_apply` from this extension
- DuckDB Spatial `ST_Affine` (3D overload)

on equivalent 3D rotation+translation workloads.

## Fairness Criteria

- Same DuckDB binary (`./build/release/duckdb`, v1.4.4).
- Same machine and same thread setting (`PRAGMA threads=8`).
- Same number of points and transform rows.
- Same transformation semantics:
  - full 3D rotation
  - 3D translation
- Same aggregation shape (transform every point, then aggregate transformed coordinates).
- Warm-up run before timed runs.
- Timings taken from `EXPLAIN ANALYZE` "Total Time".

Important semantic alignment:
- `ST_Affine` applies `R * p + o`.
- `se3_apply` applies `R * (p + t)`.
- For fairness, `se3` uses `t = R^{-1} * o` so both sides represent the same transform.

## Scenarios

1. Constant transform:
- One fixed rotation+translation applied to all random points.

2. Joined transform:
- A `transforms` table stores per-`tid` transforms.
- Points are joined to transforms via `tid`.
- Joined transform is applied to each point.

## Data Size

- Points: `5,000,000`
- Transforms: `10,000`
- Measured runs per query: `3` (after one warm-up)

## How To Run

```sh
./scripts/benchmark_se3_vs_spatial.sh
```

Environment overrides are supported:
- `THREADS`
- `N_POINTS`
- `N_TRANSFORMS`
- `RUNS`
- `DB_PATH`
- `DUCKDB_BIN`

## Results (measured on 2026-03-03)

Average query time (seconds):

| case | avg_s |
|---|---:|
| se3_constant_transform | 0.032100 |
| spatial_constant_transform | 0.125333 |
| se3_joined_transform | 0.063033 |
| spatial_joined_transform | 0.151000 |

Relative speedup from these runs:
- Constant transform: `se3` ~3.9x faster than `ST_Affine`
- Joined transform: `se3` ~2.4x faster than `ST_Affine`

## Parallelism Check

Joined-transform scenario, same dataset:

- `se3` query:
  - `threads=1`: `0.568s`
  - `threads=8`: `0.110s`
- `ST_Affine` query:
  - `threads=1`: `1.34s`
  - `threads=8`: `0.153s`

Both workloads show strong speedup with more threads, indicating the dataset is large enough to trigger parallel work.
