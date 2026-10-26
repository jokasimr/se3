.bail on
.timer off
LOAD se3;
SET threads = 16;
SET memory_limit = '24GB';
SET max_temp_directory_size = '0B';
SET enable_progress_bar = false;

CREATE TEMP TABLE chain_50m AS
SELECT i, a, b, q, struct_pack(t := a, q := q) AS w
FROM (
    SELECT i, struct_pack(x := (i % 997 - 400) * 0.001::DOUBLE,
                       y := (i % 991 - 450) * 0.002::DOUBLE,
                       z := (i % 983 - 500) * 0.003::DOUBLE) AS a,
           struct_pack(x := (i % 977 - 550) * 0.003::DOUBLE,
                       y := (i % 971 - 500) * 0.002::DOUBLE,
                       z := (i % 967 - 450) * 0.001::DOUBLE) AS b,
           struct_pack(w := cos(i % 1009 * 0.003),
                       x := sin(i % 1009 * 0.003) / 3,
                       y := sin(i % 1009 * 0.003) * 2 / 3,
                       z := sin(i % 1009 * 0.003) * 2 / 3) AS q
    FROM range(50000000) r(i)
);
CREATE TEMP TABLE chain_10m AS SELECT * FROM chain_50m WHERE i < 10000000;

SELECT version() AS duckdb_version, extension_version, install_mode
FROM duckdb_extensions() WHERE extension_name = 'se3';

SET threads = 1;
CREATE TEMP VIEW bench_heavy AS SELECT * FROM chain_10m;
.print THREADS 1 ROWS 10000000
.print WARMUP
.read scripts/benchmark_chained_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_chained_queries.sql
.print RUN 2
.read scripts/benchmark_chained_queries.sql
.print RUN 3
.read scripts/benchmark_chained_queries.sql
.print RUN 4
.read scripts/benchmark_chained_queries.sql
.print RUN 5
.read scripts/benchmark_chained_queries.sql
.timer off

SET threads = 4;
.print THREADS 4 ROWS 10000000
.print WARMUP
.read scripts/benchmark_chained_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_chained_queries.sql
.print RUN 2
.read scripts/benchmark_chained_queries.sql
.print RUN 3
.read scripts/benchmark_chained_queries.sql
.print RUN 4
.read scripts/benchmark_chained_queries.sql
.print RUN 5
.read scripts/benchmark_chained_queries.sql
.timer off

SET threads = 16;
CREATE OR REPLACE TEMP VIEW bench_heavy AS SELECT * FROM chain_50m;
.print THREADS 16 ROWS 50000000
.print WARMUP
.read scripts/benchmark_chained_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_chained_queries.sql
.print RUN 2
.read scripts/benchmark_chained_queries.sql
.print RUN 3
.read scripts/benchmark_chained_queries.sql
.print RUN 4
.read scripts/benchmark_chained_queries.sql
.print RUN 5
.read scripts/benchmark_chained_queries.sql
.timer off

SET threads = 64;
.print THREADS 64 ROWS 50000000
.print WARMUP
.read scripts/benchmark_chained_queries.sql
.timer on
.print RUN 1
.read scripts/benchmark_chained_queries.sql
.print RUN 2
.read scripts/benchmark_chained_queries.sql
.print RUN 3
.read scripts/benchmark_chained_queries.sql
.print RUN 4
.read scripts/benchmark_chained_queries.sql
.print RUN 5
.read scripts/benchmark_chained_queries.sql
.timer off
