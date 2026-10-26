-- Run from the repository root: duckdb -batch -init /dev/null -json -bail -unsigned < scripts/benchmark_cli.sql
-- LOAD se3 uses the installed extension or the extension built into the CLI.
.bail on
.timer off
LOAD se3;
SET threads = 16;
SET memory_limit = '110GB';
SET max_temp_directory_size = '0B';
SET enable_progress_bar = false;

-- Overrides are only for checking the script with small inputs.
SET VARIABLE rows_200m = coalesce(getvariable('rows_200m'), 200000000);
SET VARIABLE rows_50m = coalesce(getvariable('rows_50m'), 50000000);
SET VARIABLE rows_10m = coalesce(getvariable('rows_10m'), 10000000);

.print Generating data once
CREATE TEMP TABLE bench_200m AS
SELECT i, a, b, q,
       struct_pack(t := a, q := q) AS w,
       struct_pack(t := b, q := q2) AS w2,
       CASE WHEN i % 17 = 0 THEN NULL
            WHEN i % 17 = 1 THEN struct_pack(t := NULL, q := q)
            WHEN i % 17 = 2 THEN struct_pack(
                t := a, q := struct_pack(w := q.w, x := q.x, y := NULL::DOUBLE, z := q.z))
            ELSE struct_pack(t := a, q := q) END AS nullable_w
FROM (
    SELECT i,
           struct_pack(x := (i % 997 - 400) * 0.001::DOUBLE,
                       y := (i % 991 - 450) * 0.002::DOUBLE,
                       z := (i % 983 - 500) * 0.003::DOUBLE) AS a,
           struct_pack(x := (i % 977 - 550) * 0.003::DOUBLE,
                       y := (i % 971 - 500) * 0.002::DOUBLE,
                       z := (i % 967 - 450) * 0.001::DOUBLE) AS b,
           struct_pack(w := cos(i % 1009 * 0.003),
                       x := sin(i % 1009 * 0.003) / 3,
                       y := sin(i % 1009 * 0.003) * 2 / 3,
                       z := sin(i % 1009 * 0.003) * 2 / 3) AS q,
           struct_pack(w := cos(i % 1013 * 0.002),
                       x := sin(i % 1013 * 0.002) * 2 / 3,
                       y := -sin(i % 1013 * 0.002) / 3,
                       z := sin(i % 1013 * 0.002) * 2 / 3) AS q2
    FROM range(getvariable('rows_200m')) AS r(i)
);

.print Materializing 50M table from generated data
CREATE TEMP TABLE bench_50m AS SELECT * FROM bench_200m WHERE i < getvariable('rows_50m');
.print Materializing 10M table from generated data
CREATE TEMP TABLE bench_10m AS SELECT * FROM bench_50m WHERE i < getvariable('rows_10m');

SELECT version() AS duckdb_version, extension_version, install_mode
FROM duckdb_extensions() WHERE extension_name = 'se3';
SELECT 'setup' AS phase,
       (SELECT count(*) FROM bench_200m) AS large_rows,
       (SELECT count(*) FROM bench_50m) AS medium_rows,
       (SELECT count(*) FROM bench_10m) AS small_rows,
       (SELECT sum(memory_usage_bytes) FROM duckdb_memory()) AS memory_bytes;

SET threads = 1;
CREATE TEMP VIEW bench_input AS SELECT * FROM bench_10m;
CREATE TEMP VIEW bench_heavy AS SELECT * FROM bench_10m;
.print THREADS 1: normal=10M heavy=10M
.timer off
.print WARMUP
.read scripts/benchmark_cli_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_cli_queries.sql
.print RUN 2
.read scripts/benchmark_cli_queries.sql
.print RUN 3
.read scripts/benchmark_cli_queries.sql
.print RUN 4
.read scripts/benchmark_cli_queries.sql
.print RUN 5
.read scripts/benchmark_cli_queries.sql
.timer off

SET threads = 4;
CREATE OR REPLACE TEMP VIEW bench_input AS SELECT * FROM bench_50m;
CREATE OR REPLACE TEMP VIEW bench_heavy AS SELECT * FROM bench_10m;
.print THREADS 4: normal=50M heavy=10M
.timer off
.print WARMUP
.read scripts/benchmark_cli_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_cli_queries.sql
.print RUN 2
.read scripts/benchmark_cli_queries.sql
.print RUN 3
.read scripts/benchmark_cli_queries.sql
.print RUN 4
.read scripts/benchmark_cli_queries.sql
.print RUN 5
.read scripts/benchmark_cli_queries.sql
.timer off

SET threads = 16;
CREATE OR REPLACE TEMP VIEW bench_input AS SELECT * FROM bench_200m;
CREATE OR REPLACE TEMP VIEW bench_heavy AS SELECT * FROM bench_50m;
.print THREADS 16: normal=200M heavy=50M
.timer off
.print WARMUP
.read scripts/benchmark_cli_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_cli_queries.sql
.print RUN 2
.read scripts/benchmark_cli_queries.sql
.print RUN 3
.read scripts/benchmark_cli_queries.sql
.print RUN 4
.read scripts/benchmark_cli_queries.sql
.print RUN 5
.read scripts/benchmark_cli_queries.sql
.timer off

SET threads = 64;
.print THREADS 64: normal=200M heavy=50M
.timer off
.print WARMUP
.read scripts/benchmark_cli_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_cli_queries.sql
.print RUN 2
.read scripts/benchmark_cli_queries.sql
.print RUN 3
.read scripts/benchmark_cli_queries.sql
.print RUN 4
.read scripts/benchmark_cli_queries.sql
.print RUN 5
.read scripts/benchmark_cli_queries.sql
.timer off
