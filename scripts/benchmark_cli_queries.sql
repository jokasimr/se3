SELECT 'scan' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT a AS r FROM bench_input);

SELECT 'constructor3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vvec(a.x, a.y, a.z) AS r FROM bench_input);

SELECT 'vadd3' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vadd(a, b) AS r FROM bench_input);

SELECT 'vscale' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT vscale(a, 0.7) AS r FROM bench_input);

SELECT 'rotation' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(q, b) AS r FROM bench_input);

SELECT 'transform' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(w, b) AS r FROM bench_input);

SELECT 'constant_rotation' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(vvec(0.5, 0.5, 0.5, 0.5), b) AS r FROM bench_input);

SELECT 'constant_transform' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(se3_make(vvec(0.1, 0.2, 0.3), vvec(0.5, 0.5, 0.5, 0.5)), b) AS r
      FROM bench_input);

SELECT 'constant_point_rotation' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(q, vvec(0.1, 0.2, 0.3)) AS r FROM bench_input);

SELECT 'mixed_point_rotation' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(q, struct_pack(x := b.x, y := b.y, z := 0.0::DOUBLE)) AS r
      FROM bench_input);

SELECT 'pipeline' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(vvec(0.5, 0.5, 0.5, 0.5), vadd(vscale(a, 0.7), b)) AS r
      FROM bench_input);

.read scripts/benchmark_chained_queries.sql

SELECT 'inverse' AS name, count(r),
       sum(r.t.x), sum(r.t.y), sum(r.t.z), sum(r.q.w), sum(r.q.x), sum(r.q.y), sum(r.q.z)
FROM (SELECT se3_inv(w) AS r FROM bench_input);

SELECT 'compose' AS name, count(r),
       sum(r.t.x), sum(r.t.y), sum(r.t.z), sum(r.q.w), sum(r.q.x), sum(r.q.y), sum(r.q.z)
FROM (SELECT se3_compose(w, w2) AS r FROM bench_heavy);

SELECT 'selected_null_transform' AS name, count(r), sum(r.x), sum(r.y), sum(r.z)
FROM (SELECT se3_apply(nullable_w, b) AS r FROM bench_heavy WHERE i % 3 <> 1);
