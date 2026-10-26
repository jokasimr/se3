# Benchmarking

The SQL benchmarks run through the DuckDB CLI with `se3` already installed or
linked into the executable. Use Release builds when comparing implementations.
Run from the repository root so the CLI can resolve `.read` paths.

| Script | Workload | Default rows | Threads |
|---|---|---|---|
| [`benchmark_cli.sql`](../../scripts/benchmark_cli.sql) | Vector arithmetic, rotations, transforms, composition, NULLs and chained operations | 10M, 50M and 200M | 1, 4, 16, 64 |
| [`benchmark_chained.sql`](../../scripts/benchmark_chained.sql) | Projection, subtraction, scaling, addition and angles, with optional rotation or transformation | 10M and 50M | 1, 4, 16, 64 |
| [`benchmark_projection.sql`](../../scripts/benchmark_projection.sql) | 3D/4D projection and rejection with ordinary, mixed, extreme and zero inputs | 10M | 1 |

Each script generates its data once per CLI process and reuses it across queries
and repetitions. Each thread configuration has one warm-up and five measured
passes. Queries aggregate every result component to consume the complete output.
Dependent calculations in the chained workloads use successive CTEs.

For example, run the focused chained workloads with the locally built CLI:

```sh
./build/release/duckdb -batch -init /dev/null -csv -noheader -bail -unsigned \
  < scripts/benchmark_chained.sql > /tmp/se3-chained.log
```

Replace the executable to compare another build, and save its output separately.
The general suite targets machines with about 128GB of RAM; its `rows_200m`,
`rows_50m` and `rows_10m` variables can be reduced for smaller machines. The
focused suites use substantially less memory. Spilling is disabled, so the
generated tables must fit within each script's memory limit.

Compare medians and sample ranges at the same row count and thread count, and
check that counts and sums agree within floating-point rounding. Timings include
scanning and aggregation as well as extension execution. When DuckDB versions
differ, the comparison includes engine changes. Run binaries sequentially on an
otherwise idle machine, with no builds running alongside the measurements.
