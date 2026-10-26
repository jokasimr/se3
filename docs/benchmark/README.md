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

## Comparing extension builds with the CLI

The SQL runners cover different workloads:

- [`benchmark_cli.sql`](../../scripts/benchmark_cli.sql): general vector and transform operations, constant and NULL inputs, and chained calculations.
- [`benchmark_chained.sql`](../../scripts/benchmark_chained.sql): projection, scaling, addition, subtraction and angles, optionally followed by rotation or transformation.
- [`benchmark_projection.sql`](../../scripts/benchmark_projection.sql): projection and rejection with ordinary, extreme, mixed and zero inputs.

Run from the repository root with a Release CLI and `se3` installed or linked in:

```sh
./build/release/duckdb -batch -init /dev/null -csv -noheader -bail -unsigned \
  < scripts/benchmark_chained.sql > /tmp/se3-chained.log
```

Each runner generates data once and reuses it for a warm-up and five measured
passes. The general and chained suites cover 1, 4, 16 and 64 threads; the
projection suite uses one thread. Counts and component sums accompany timings.

## Interpreting vector comparisons

See [Comparing vector functions with SQL macros](../BENCHMARK_REPORT.md) for
the workload definitions and the meaning of the reported speedup and baseline
estimates.
