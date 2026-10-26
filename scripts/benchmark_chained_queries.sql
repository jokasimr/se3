WITH differences AS (
    SELECT a, b, vsub(a, b) AS delta, vangle(a, b) AS weight
    FROM bench_heavy
), projections AS (
    SELECT a, b, delta, weight, vproj(delta, b) AS parallel
    FROM differences
), rejections AS (
    SELECT a, b, parallel, weight, vsub(delta, parallel) AS perpendicular
    FROM projections
), blends AS (
    SELECT a, b, vadd(vscale(parallel, 0.7), vscale(perpendicular, weight)) AS blended
    FROM rejections
), chain AS (
    SELECT b, vadd(a, blended) AS adjusted
    FROM blends
)
SELECT 'chain_project_angle' AS name, count(angle), sum(angle)
FROM (SELECT vangle(adjusted, b) AS angle FROM chain);

WITH differences AS (
    SELECT a, b, q, vsub(a, b) AS delta, vangle(a, b) AS weight
    FROM bench_heavy
), projections AS (
    SELECT a, b, q, delta, weight, vproj(delta, b) AS parallel
    FROM differences
), rejections AS (
    SELECT a, b, q, parallel, weight, vsub(delta, parallel) AS perpendicular
    FROM projections
), blends AS (
    SELECT a, b, q, vadd(vscale(parallel, 0.7), vscale(perpendicular, weight)) AS blended
    FROM rejections
), chain AS (
    SELECT b, q, vadd(a, blended) AS adjusted
    FROM blends
)
SELECT 'chain_project_rotate_angle' AS name, count(angle), sum(angle)
FROM (SELECT vangle(se3_apply(q, adjusted), b) AS angle FROM chain);

WITH differences AS (
    SELECT a, b, w, vsub(a, b) AS delta, vangle(a, b) AS weight
    FROM bench_heavy
), projections AS (
    SELECT a, b, w, delta, weight, vproj(delta, b) AS parallel
    FROM differences
), rejections AS (
    SELECT a, b, w, parallel, weight, vsub(delta, parallel) AS perpendicular
    FROM projections
), blends AS (
    SELECT a, b, w, vadd(vscale(parallel, 0.7), vscale(perpendicular, weight)) AS blended
    FROM rejections
), chain AS (
    SELECT b, w, vadd(a, blended) AS adjusted
    FROM blends
)
SELECT 'chain_project_transform_angle' AS name, count(angle), sum(angle)
FROM (SELECT vangle(se3_apply(w, adjusted), b) AS angle FROM chain);
