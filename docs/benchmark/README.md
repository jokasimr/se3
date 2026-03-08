# Benchmarking

## Parallel benchmark
Use the benchmark script to run `se3` scalar kernels on a dataset large enough to trigger parallel work.

Run:
```sh
./build/release/duckdb < scripts/parallel_benchmark.sql
```

The script:
- sets `PRAGMA threads=8`
- generates a 20M-row input table
- materializes `W` transforms once
- runs a warm-up query
- times:
  - `se3_apply(W, p)`
  - `se3_compose(W, W)` followed by `se3_apply(...)`

Notes:
- This benchmark is intentionally heavy; adjust row count in `scripts/parallel_benchmark.sql` if needed.
- For consistent results, run on an otherwise idle machine and repeat measurements.

## Vector vs macro benchmark
Compare vector extension functions (`vadd`, `vsub`, `vscale`, `vdot`, `vnorm2`, `vnorm`, `vnormalize`) against equivalent SQL macros.

Run:
```sh
./scripts/benchmark_vector_vs_macros.sh
```

Environment overrides:
- `DUCKDB_BIN`
- `DB_PATH`
- `THREADS`
- `N_ROWS`
- `RUNS`
- `RESULTS_CSV`
- `CHECK_CORRECTNESS` (`1` by default)
- `ABS_TOL` (default `1e-9`)
- `REL_TOL` (default `1e-12`)

Methodology notes:
- Uses a warm-up for both extension and macro implementations.
- Uses paired interleaved runs (alternating execution order each run) to reduce order/thermal/cache bias.
- Validates extension and macro outputs before timing with absolute+relative tolerance.
- Measures full query execution time via `EXPLAIN ANALYZE` on identical SQL shapes for both alternatives.

## Scale+add pipeline benchmark
Benchmark a practical opaque pipeline where vector ops feed into `se3_apply`:
- extension pipeline: `se3_apply(q_const, vadd(vscale(a, s), b))`
- macro pipeline: `se3_apply(q_const, m_vadd(m_vscale(a, s), b))`
- baseline: `se3_apply(q_const, a)` (to estimate `se3_apply + aggregation` overhead)

Run:
```sh
./scripts/benchmark_vscale_vadd_se3_apply.sh
```

Environment overrides:
- `DUCKDB_BIN`
- `DB_PATH`
- `THREADS`
- `N_ROWS`
- `RUNS`
- `RESULTS_CSV`
- `CHECK_CORRECTNESS` (`1` by default)
- `ABS_TOL` (default `1e-9`)
- `REL_TOL` (default `1e-12`)
- `CONST_Q_W`, `CONST_Q_X`, `CONST_Q_Y`, `CONST_Q_Z` (default identity quaternion)

Output includes:
- raw pipeline timings (`extension`, `macro`)
- baseline timing
- overall macro/extension speedup
- incremental estimate: `(pipeline - baseline)` for each implementation

## Benchmark Report
For consolidated results and interpretation from the latest benchmark campaign, see:
- [`docs/BENCHMARK_REPORT.md`](/home/johannes/personal/se3-extension/se3/docs/BENCHMARK_REPORT.md)
