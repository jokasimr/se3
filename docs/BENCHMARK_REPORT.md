# Comparing vector functions with SQL macros

The vector benchmarks compare `se3` functions with equivalent SQL macros on the
same input table. The [benchmark guide](benchmark/README.md) lists commands and
configuration options.

## Workloads

[`benchmark_vector_vs_macros.sh`](../scripts/benchmark_vector_vs_macros.sh)
compares `vadd`, `vsub`, `vscale`, `vdot`, `vnorm2`, `vnorm` and `vnormalize`
individually. Each query aggregates the complete scalar or vector result.

[`benchmark_vscale_vadd_se3_apply.sh`](../scripts/benchmark_vscale_vadd_se3_apply.sh)
compares three expressions:

| Case | Expression |
|---|---|
| Extension pipeline | `se3_apply(q_const, vadd(vscale(a, s), b))` |
| Macro pipeline | `se3_apply(q_const, m_vadd(m_vscale(a, s), b))` |
| Baseline | `se3_apply(q_const, a)` |

The baseline measures rotation, scanning and aggregation without the scale/add
operations. The constant quaternion is configurable and defaults to the identity.

Both scripts materialize the inputs once and, by default, check aggregate results
before measurement. Each measured extension/macro pair runs in the same CLI process;
the execution order alternates between repetitions. Reported times are averages
of `EXPLAIN ANALYZE` total query times.

## Interpreting the output

Let `E` be the extension time, `M` the macro time and `B` the baseline time.

- `M / E` is the reported speedup: above 1 means the extension query is faster.
- `E - B` and `M - B` estimate the extra query cost of the scale/add pipeline.
- `(M - B) / (E - B)` compares those estimates. It becomes sensitive to timing
  variation when either pipeline time is close to the baseline.

Scanning, aggregation and other shared work can make whole-query differences
smaller than differences in expression execution. Subtracting the baseline
provides an estimate, but changing an expression can also change its execution
plan. Operator profiling can help locate the cost; a projection operator's time
covers all expressions it executes, rather than one function call.
