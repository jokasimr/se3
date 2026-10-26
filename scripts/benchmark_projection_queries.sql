SELECT 'fast_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a3, fast3) AS r FROM bench_projection);
SELECT 'fast_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a4, fast4) AS r FROM bench_projection);
SELECT 'fast_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a3, fast3) AS r FROM bench_projection);
SELECT 'fast_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a4, fast4) AS r FROM bench_projection);

SELECT 'mixed1_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a3, mixed1_3) AS r FROM bench_projection);
SELECT 'mixed1_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a4, mixed1_4) AS r FROM bench_projection);
SELECT 'mixed1_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a3, mixed1_3) AS r FROM bench_projection);
SELECT 'mixed1_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a4, mixed1_4) AS r FROM bench_projection);

SELECT 'mixed50_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a3, mixed50_3) AS r FROM bench_projection);
SELECT 'mixed50_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a4, mixed50_4) AS r FROM bench_projection);
SELECT 'mixed50_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a3, mixed50_3) AS r FROM bench_projection);
SELECT 'mixed50_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a4, mixed50_4) AS r FROM bench_projection);

SELECT 'slow_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a3, slow3) AS r FROM bench_projection);
SELECT 'slow_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(a4, slow4) AS r FROM bench_projection);
SELECT 'slow_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a3, slow3) AS r FROM bench_projection);
SELECT 'slow_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(a4, slow4) AS r FROM bench_projection);

SELECT 'zero_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(zero3, slow3) AS r FROM bench_projection);
SELECT 'zero_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(zero4, slow4) AS r FROM bench_projection);
SELECT 'zero_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(zero3, slow3) AS r FROM bench_projection);
SELECT 'zero_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(zero4, slow4) AS r FROM bench_projection);

SELECT 'wide_proj3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(wide_a3, wide_b3) AS r FROM bench_projection);
SELECT 'wide_proj4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vproj(wide_a4, wide_b4) AS r FROM bench_projection);
SELECT 'wide_rej3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(wide_a3, wide_b3) AS r FROM bench_projection);
SELECT 'wide_rej4' AS name, count(r), sum(r.w), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vrej(wide_a4, wide_b4) AS r FROM bench_projection);
