# Development

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
