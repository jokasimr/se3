# Benchmark Report: Vector UDFs vs Macros

Date: 2026-03-08  
Repository: `se3` extension

## Motivation
The vector API (`vadd`, `vsub`, `vscale`, `vdot`, `vnorm*`, `vangle`, `vproj`, `vrej`, etc.) was added partly to avoid deep macro expression trees that can increase planning cost and sometimes produce suboptimal execution for more complex expressions (notably in `se3_*` composition/use cases).

The benchmark goals were:
- compare extension scalar UDFs vs equivalent SQL macros for vector operations,
- check scaling with threads,
- test a more practical opaque pipeline where vector output feeds into `se3_apply`,
- investigate whether end-to-end timings hide true operation cost.

## Benchmark Methodology
Primary scripts:
- [`scripts/benchmark_vector_vs_macros.sh`](/home/johannes/personal/se3-extension/se3/scripts/benchmark_vector_vs_macros.sh)
- [`scripts/benchmark_vscale_vadd_se3_apply.sh`](/home/johannes/personal/se3-extension/se3/scripts/benchmark_vscale_vadd_se3_apply.sh)

Environment note:
- Thread-scaling benchmarks were run on a laptop CPU. Even with 6 logical/physical cores available, core performance is typically heterogeneous (e.g., performance vs efficiency cores), so scaling figures should be interpreted as machine-specific.

Shared methodology:
- fixed synthetic dataset generated in DuckDB,
- warm-up runs before measurement,
- multiple measured runs and average reporting,
- paired interleaved execution order for extension vs macro to reduce ordering/thermal bias,
- extension and macro run in the same DuckDB process per paired run,
- correctness checks before timing (absolute + relative tolerance).

Important caveat:
- end-to-end query time includes scan/aggregation/framework overhead; it is not pure kernel time.

## Results

## 1) Core vector functions (5M rows, RUNS=3)
Speedup definition: `macro_avg / extension_avg`  
Interpretation: `< 1.0x` means macro is faster.

| Function | Threads=1 | Threads=2 | Threads=6 |
|---|---:|---:|---:|
| vadd | 0.812x | 0.788x | 0.783x |
| vsub | 0.808x | 0.804x | 0.776x |
| vscale | 0.811x | 0.825x | 0.814x |
| vdot | 0.816x | 0.821x | 0.783x |
| vnorm2 | 0.865x | 0.822x | 0.824x |
| vnorm | 0.841x | 0.888x | 0.822x |
| vnormalize | 0.900x | 0.900x | 0.924x |

Signal:
- For these simpler kernels, macros were consistently faster in this workload.

## 2) Additional functions (5M rows, RUNS=3)
Speedup definition: `macro_avg / extension_avg`  
Interpretation: `> 1.0x` means extension is faster.

| Function | Threads=1 | Threads=2 | Threads=6 |
|---|---:|---:|---:|
| vcross | 1.304x | 1.306x | 1.253x |
| vcos_angle | 1.940x | 1.872x | 1.862x |
| vangle | 1.774x | 1.870x | 1.737x |
| vproj | 1.544x | 1.549x | 1.490x |
| vrej | 1.797x | 1.782x | 1.625x |

Signal:
- For these heavier/composed operations, extension UDFs were clearly faster.

## 3) Thread scaling (5M rows, all measured functions)
Representative category-level `1 -> 6` scaling:

| Category | Extension | Macro |
|---|---:|---:|
| basic_arith (`vadd`,`vsub`,`vscale`) | 3.888x | 3.983x |
| norm_dot (`vdot`,`vnorm2`,`vnorm`,`vnormalize`) | 3.918x | 4.004x |
| angle_ops (`vcos_angle`,`vangle`) | 3.803x | 3.923x |
| geo_ops (`vcross`,`vproj`,`vrej`) | 3.628x | 3.844x |

Signal:
- Both implementations scale similarly.
- Macros show slightly better high-thread scaling on average.

## 4) `vadd` at larger row counts
Measured averages (`RUNS=3`), extension:

| Rows | Threads=1 | Threads=2 | Threads=6 |
|---|---:|---:|---:|
| 1,000,000 | 0.02290s | 0.01367s | 0.00877s |
| 100,000,000 | 2.38000s | 1.13200s | 0.53467s |

Corresponding `macro/extension` speedups ranged roughly `0.749x` to `0.864x` (macro faster).

## 5) Opaque pipeline benchmark
Pipeline query:
- extension: `se3_apply(q_const, vadd(vscale(a, s), b))`
- macro: `se3_apply(q_const, m_vadd(m_vscale(a, s), b))`
- baseline: `se3_apply(q_const, a)` for `se3_apply + aggregation` overhead estimate

### 5M rows (THREADS=8, RUNS=5)
- extension pipeline: `0.058660s`
- macro pipeline: `0.045560s`
- baseline: `0.022600s`
- pipeline speedup (`macro/extension`): `0.777x`
- incremental speedup (`(macro-baseline)/(extension-baseline)`): `0.637x`

### 20M rows (THREADS=8, RUNS=5)
- extension pipeline: `0.235800s`
- macro pipeline: `0.188200s`
- baseline: `0.092240s`
- pipeline speedup (`macro/extension`): `0.798x`
- incremental speedup: `0.668x`

Signal:
- In this practical opaque pipeline, macros still outperform extension `vscale+vadd`.

## 6) `vangle` macro rewrite experiment
Hypothesis: old macro form used two square roots (`sqrt(na2)*sqrt(nb2)`), while UDF uses one (`sqrt(na2*nb2)`).

Optimized macro benchmark (5M rows, RUNS=5):

| Threads | Old macro/udf | Optimized macro/udf |
|---|---:|---:|
| 1 | 1.871x | 1.788x |
| 2 | 1.939x | 1.798x |
| 6 | 1.878x | 1.829x |

Signal:
- Optimization helps, but UDF remains significantly faster for `vangle`.

## 7) Profiling-based comparison (`vadd` vs `vangle`)
Using DuckDB JSON profiling (5M rows, 6 threads, 5 runs):
- `vadd` total latency avg: `0.03316s`
- `vangle` total latency avg: `0.04415s`
- total-latency ratio (`vangle/vadd`): `1.33x`

Projection-node timing (proxy for expression/UDF work):
- `vadd` projection timing avg: `0.01640s`
- `vangle` projection timing avg: `0.06518s`
- projection ratio (`vangle/vadd`): `3.98x`

Interpretation:
- End-to-end latency compresses differences due to shared overheads.
- Profiling suggests true expression compute gap is much larger than wall-clock query gap.

Profiler caveat:
- DuckDB exposes operator-level timing, not a guaranteed pure per-UDF timer.

## Conclusions
1. There is no single winner across all vector API functions.
2. Simple vector arithmetic/dot/norm operations often favor macros in measured execution time.
3. Heavier composed operations (`vangle`, `vcos_angle`, `vproj`, `vrej`, `vcross`) can favor UDFs.
4. Opaque-pipeline measurements still showed macro advantage for `vscale+vadd` in tested workloads.
5. Whole-query benchmarks can understate operation-level differences; differential baselines and profiling are needed for interpretation.
6. Keeping UDFs remains justified for complex operations and for controlling deep-expression behavior; macro alternatives are attractive for simple operations where execution speed is dominant.

## Artifacts
Primary run outputs used in this report (local machine):
- `/tmp/se3_bench_runs/results_t1.csv`
- `/tmp/se3_bench_runs/results_t2.csv`
- `/tmp/se3_bench_runs/results_t6.csv`
- `/tmp/se3_bench_runs/extra_funcs_speedup.csv`
- `/tmp/se3_bench_runs/vadd_rows_compare.csv`
- `/tmp/se3_bench_runs/vscale_vadd_se3_apply_5000000.csv`
- `/tmp/se3_bench_runs/vscale_vadd_se3_apply_20000000.csv`
- `/tmp/se3_bench_runs/vangle_macro_opt_compare.csv`
- `/tmp/se3_bench_runs/profile_udf_compare.csv`
