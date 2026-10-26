.bail on
.timer off
LOAD se3;
SET threads = 16;
SET enable_progress_bar = false;
SET memory_limit = '16GB';
SET max_temp_directory_size = '0B';

CREATE TEMP TABLE bench_projection AS
SELECT vvec(x, y, z) AS a3, vvec(x, y, z, u) AS a4,
       vvec(u, v, w) AS fast3, vvec(u, v, w, y) AS fast4,
       vvec(u*s, v*s, w*s) AS slow3, vvec(u*s, v*s, w*s, y*s) AS slow4,
       vvec(u*m1, v*m1, w*m1) AS mixed1_3, vvec(u*m1, v*m1, w*m1, y*m1) AS mixed1_4,
       vvec(u*m50, v*m50, w*m50) AS mixed50_3, vvec(u*m50, v*m50, w*m50, y*m50) AS mixed50_4,
       vvec(0, 0, 0) AS zero3, vvec(0, 0, 0, 0) AS zero4,
       vvec(x*1e-170, y*1e200, 0) AS wide_a3, vvec(x*1e-170, y*1e200, 0, 0) AS wide_a4,
       vvec(u*1e150, 0, 0) AS wide_b3, vvec(u*1e150, 0, 0, 0) AS wide_b4,
       bucket
FROM (
    SELECT *, CASE WHEN bucket % 2 = 0 THEN 1e200 ELSE 1e-200 END AS s,
           CASE WHEN bucket < 2 THEN CASE WHEN bucket = 0 THEN 1e200 ELSE 1e-200 END ELSE 1 END AS m1,
           CASE WHEN bucket < 100 THEN CASE WHEN bucket % 2 = 0 THEN 1e200 ELSE 1e-200 END ELSE 1 END AS m50
    FROM (
        SELECT (i % 997 + 1)::DOUBLE AS x, (i % 991 + 1)::DOUBLE AS y,
               (i % 983 + 1)::DOUBLE AS z, (i % 977 + 1)::DOUBLE AS u,
               (i % 971 + 1)::DOUBLE AS v, (i % 967 + 1)::DOUBLE AS w,
               hash(i) % 200 AS bucket
        FROM range(10000000) r(i)
    ) values_
) scales;

SELECT version(), count(*), count(*) FILTER (WHERE bucket < 2) AS mixed1_slow_rows,
       count(*) FILTER (WHERE bucket < 100) AS mixed50_slow_rows FROM bench_projection;
SET threads = 1;
.print WARMUP
.read scripts/benchmark_projection_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_projection_queries.sql
.print RUN 2
.read scripts/benchmark_projection_queries.sql
.print RUN 3
.read scripts/benchmark_projection_queries.sql
.print RUN 4
.read scripts/benchmark_projection_queries.sql
.print RUN 5
.read scripts/benchmark_projection_queries.sql
.timer off
