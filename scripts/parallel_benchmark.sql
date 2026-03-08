-- Benchmark se3 scalar functions on a dataset large enough to trigger parallel work.
-- Run with:
--   ./build/release/duckdb < scripts/parallel_benchmark.sql
-- or from inside DuckDB:
--   .read scripts/parallel_benchmark.sql

PRAGMA threads=8;
PRAGMA enable_progress_bar;

.timer on

-- Use enough rows to produce many scan/expression chunks.
-- Adjust row count if your machine has less memory.
CREATE OR REPLACE TABLE bench_points AS
SELECT
  i AS id,
  struct_pack(
    x := (i % 1000)::DOUBLE * 0.001,
    y := (i % 2000)::DOUBLE * 0.001,
    z := (i % 3000)::DOUBLE * 0.001
  ) AS p,
  struct_pack(
    x := (i % 97)::DOUBLE * 0.01,
    y := (i % 89)::DOUBLE * 0.01,
    z := (i % 83)::DOUBLE * 0.01
  ) AS t,
  struct_pack(
    x := 0.0::DOUBLE,
    y := 0.0::DOUBLE,
    z := 1.0::DOUBLE
  ) AS axis,
  ((i % 360)::DOUBLE * pi() / 180.0) AS th
FROM range(0, 20000000) AS r(i);

-- Materialize transforms once so benchmark focuses on apply/compose kernels.
CREATE OR REPLACE TABLE bench_w AS
SELECT
  id,
  p,
  t,
  se3_from_axis_angle(t, axis, th) AS w
FROM bench_points;

-- Warm-up run.
SELECT sum((se3_apply(w, p)).x) AS warmup_sum_x
FROM bench_w;

-- Benchmark 1: se3_apply(W, p) over 20M rows.
SELECT
  sum((se3_apply(w, p)).x) AS sum_x,
  sum((se3_apply(w, p)).y) AS sum_y,
  sum((se3_apply(w, p)).z) AS sum_z
FROM bench_w;

-- Benchmark 2: compose + apply on 20M rows.
SELECT
  sum((se3_apply(se3_compose(se3_compose(w, w), w), p)).x) AS sum_x,
  sum((se3_apply(se3_compose(se3_compose(w, w), w), p)).y) AS sum_y,
  sum((se3_apply(se3_compose(se3_compose(w, w), w), p)).z) AS sum_z
FROM bench_w, range(3);

.timer off
