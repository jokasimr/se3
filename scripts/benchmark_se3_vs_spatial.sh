#!/usr/bin/env bash
set -euo pipefail

DUCKDB_BIN="${DUCKDB_BIN:-./build/release/duckdb}"
DB_PATH="${DB_PATH:-/tmp/se3_vs_spatial_benchmark.duckdb}"
THREADS="${THREADS:-8}"
N_POINTS="${N_POINTS:-5000000}"
N_TRANSFORMS="${N_TRANSFORMS:-10000}"
RUNS="${RUNS:-3}"

if ! command -v rg >/dev/null 2>&1; then
  echo "This script requires 'rg' (ripgrep) on PATH." >&2
  exit 1
fi

echo "Using DuckDB binary: ${DUCKDB_BIN}"
echo "Database path: ${DB_PATH}"
echo "Threads: ${THREADS}"
echo "Points: ${N_POINTS}"
echo "Transforms: ${N_TRANSFORMS}"
echo "Measured runs per query: ${RUNS}"
echo

echo "Preparing benchmark data..."
"${DUCKDB_BIN}" -unsigned "${DB_PATH}" <<SQL
LOAD spatial;
PRAGMA threads=${THREADS};
SELECT setseed(0.42);

CREATE OR REPLACE TABLE transforms AS
SELECT
  tid,
  theta,
  ax,
  ay,
  az,
  ox AS tx,
  oy AS ty,
  oz AS tz,
  cos(theta) + ax * ax * (1.0 - cos(theta)) AS a,
  ax * ay * (1.0 - cos(theta)) - az * sin(theta) AS b,
  ax * az * (1.0 - cos(theta)) + ay * sin(theta) AS c,
  ay * ax * (1.0 - cos(theta)) + az * sin(theta) AS d,
  cos(theta) + ay * ay * (1.0 - cos(theta)) AS e,
  ay * az * (1.0 - cos(theta)) - ax * sin(theta) AS f,
  az * ax * (1.0 - cos(theta)) - ay * sin(theta) AS g,
  az * ay * (1.0 - cos(theta)) + ax * sin(theta) AS h,
  cos(theta) + az * az * (1.0 - cos(theta)) AS i,
  se3_from_axis_angle(
    struct_pack(
      x:= (cos(theta) + ax * ax * (1.0 - cos(theta))) * ox +
          (ay * ax * (1.0 - cos(theta)) + az * sin(theta)) * oy +
          (az * ax * (1.0 - cos(theta)) - ay * sin(theta)) * oz,
      y:= (ax * ay * (1.0 - cos(theta)) - az * sin(theta)) * ox +
          (cos(theta) + ay * ay * (1.0 - cos(theta))) * oy +
          (az * ay * (1.0 - cos(theta)) + ax * sin(theta)) * oz,
      z:= (ax * az * (1.0 - cos(theta)) + ay * sin(theta)) * ox +
          (ay * az * (1.0 - cos(theta)) - ax * sin(theta)) * oy +
          (cos(theta) + az * az * (1.0 - cos(theta))) * oz
    ),
    struct_pack(x:=ax, y:=ay, z:=az),
    theta
  ) AS w
FROM (
  SELECT
    tid,
    theta,
    axr / axis_norm AS ax,
    ayr / axis_norm AS ay,
    azr / axis_norm AS az,
    ox,
    oy,
    oz
  FROM (
    SELECT
      i::INTEGER AS tid,
      ((i % 3600)::DOUBLE * pi() / 1800.0) AS theta,
      ((i % 1000)::DOUBLE / 100.0 - 5.0) AS ox,
      (((i * 7) % 1000)::DOUBLE / 100.0 - 5.0) AS oy,
      (((i * 13) % 1000)::DOUBLE / 100.0 - 5.0) AS oz,
      (((i * 17) % 1000)::DOUBLE / 500.0 - 1.0) AS axr,
      (((i * 19 + 7) % 1000)::DOUBLE / 500.0 - 1.0) AS ayr,
      (((i * 23 + 11) % 1000)::DOUBLE / 500.0 - 1.0) AS azr,
      sqrt(
        (((i * 17) % 1000)::DOUBLE / 500.0 - 1.0) * (((i * 17) % 1000)::DOUBLE / 500.0 - 1.0) +
        (((i * 19 + 7) % 1000)::DOUBLE / 500.0 - 1.0) * (((i * 19 + 7) % 1000)::DOUBLE / 500.0 - 1.0) +
        (((i * 23 + 11) % 1000)::DOUBLE / 500.0 - 1.0) * (((i * 23 + 11) % 1000)::DOUBLE / 500.0 - 1.0)
      ) AS axis_norm
    FROM range(0, ${N_TRANSFORMS}) AS r(i)
  ) base
  WHERE axis_norm > 1e-12
) t;

CREATE OR REPLACE TABLE points AS
SELECT
  id,
  tid,
  x,
  y,
  z,
  struct_pack(x:=x, y:=y, z:=z) AS p3,
  ST_Point3D(x, y, z) AS g3
FROM (
  SELECT
    i AS id,
    CAST(floor(random() * ${N_TRANSFORMS}) AS INTEGER) AS tid,
    (random() * 2000.0 - 1000.0) AS x,
    (random() * 2000.0 - 1000.0) AS y,
    (random() * 2000.0 - 1000.0) AS z
  FROM range(0, ${N_POINTS}) AS r(i)
) p;

ANALYZE points;
ANALYZE transforms;
SQL

SE3_CONST_QUERY="SELECT sum(p2.x), sum(p2.y), sum(p2.z) FROM (SELECT se3_apply(se3_from_axis_angle(struct_pack(x:=-1.6054443867109798, y:=-1.5899085798031086, z:=4.0950871821057326), struct_pack(x:=0.2672612419124244, y:=0.5345224838248488, z:=0.8017837257372732), 0.73), p3) AS p2 FROM points) q;"
SPATIAL_CONST_QUERY="SELECT sum(ST_X(g2)), sum(ST_Y(g2)), sum(ST_Z(g2)) FROM (SELECT ST_Affine(g3, 0.7633762307488083, -0.49828156372644467, 0.41106229890136037, 0.5710888773421959, 0.8179817159606217, -0.0690174364211465, -0.30185132847773344, 0.2874393772684004, 0.9089908579803109, 1.25, -2.5, 3.75) AS g2 FROM points) q;"
SE3_JOIN_QUERY="SELECT sum(p2.x), sum(p2.y), sum(p2.z) FROM (SELECT se3_apply(t.w, p.p3) AS p2 FROM points p JOIN transforms t ON p.tid = t.tid) q;"
SPATIAL_JOIN_QUERY="SELECT sum(ST_X(g2)), sum(ST_Y(g2)), sum(ST_Z(g2)) FROM (SELECT ST_Affine(p.g3, t.a, t.b, t.c, t.d, t.e, t.f, t.g, t.h, t.i, t.tx, t.ty, t.tz) AS g2 FROM points p JOIN transforms t ON p.tid = t.tid) q;"

extract_time_seconds() {
  rg -o 'Total Time: [0-9.]+s' | head -n1 | rg -o '[0-9.]+' || true
}

run_explain_analyze() {
  local sql="$1"
  "${DUCKDB_BIN}" -unsigned "${DB_PATH}" -c "LOAD spatial; PRAGMA threads=${THREADS}; EXPLAIN ANALYZE ${sql}"
}

run_case() {
  local label="$1"
  local sql="$2"

  echo "== ${label} =="
  echo "Warm-up..."
  run_explain_analyze "${sql}" >/dev/null

  local total="0"
  local run
  for run in $(seq 1 "${RUNS}"); do
    local out
    out="$(run_explain_analyze "${sql}")"
    local t
    t="$(printf '%s\n' "${out}" | extract_time_seconds)"
    if [[ -z "${t}" ]]; then
      echo "Failed to parse time for ${label} run ${run}" >&2
      exit 1
    fi
    printf 'Run %d: %ss\n' "${run}" "${t}"
    total="$(awk -v a="${total}" -v b="${t}" 'BEGIN { printf "%.6f", a + b }')"
  done

  local avg
  avg="$(awk -v s="${total}" -v n="${RUNS}" 'BEGIN { printf "%.6f", s / n }')"
  printf 'Average: %ss\n\n' "${avg}"
  printf '%s,%s\n' "${label}" "${avg}" >> /tmp/se3_vs_spatial_results.csv
}

rm -f /tmp/se3_vs_spatial_results.csv

run_case "se3_constant_transform" "${SE3_CONST_QUERY}"
run_case "spatial_constant_transform" "${SPATIAL_CONST_QUERY}"
run_case "se3_joined_transform" "${SE3_JOIN_QUERY}"
run_case "spatial_joined_transform" "${SPATIAL_JOIN_QUERY}"

echo "Summary (avg seconds):"
column -t -s, /tmp/se3_vs_spatial_results.csv
