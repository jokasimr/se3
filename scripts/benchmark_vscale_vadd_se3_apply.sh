#!/usr/bin/env bash
set -euo pipefail

DUCKDB_BIN="${DUCKDB_BIN:-./build/release/duckdb}"
DB_PATH="${DB_PATH:-/tmp/se3_vscale_vadd_se3_apply_benchmark.duckdb}"
THREADS="${THREADS:-8}"
N_ROWS="${N_ROWS:-5000000}"
RUNS="${RUNS:-5}"
RESULTS_CSV="${RESULTS_CSV:-/tmp/se3_vscale_vadd_se3_apply_results.csv}"
CHECK_CORRECTNESS="${CHECK_CORRECTNESS:-1}"
ABS_TOL="${ABS_TOL:-1e-9}"
REL_TOL="${REL_TOL:-1e-12}"
CONST_Q_W="${CONST_Q_W:-1.0}"
CONST_Q_X="${CONST_Q_X:-0.0}"
CONST_Q_Y="${CONST_Q_Y:-0.0}"
CONST_Q_Z="${CONST_Q_Z:-0.0}"

if ! command -v rg >/dev/null 2>&1; then
  echo "This script requires 'rg' (ripgrep) on PATH." >&2
  exit 1
fi

echo "Using DuckDB binary: ${DUCKDB_BIN}"
echo "Database path: ${DB_PATH}"
echo "Threads: ${THREADS}"
echo "Rows: ${N_ROWS}"
echo "Measured runs per query: ${RUNS}"
echo "Correctness check: ${CHECK_CORRECTNESS}"
echo "Correctness tolerances: abs=${ABS_TOL}, rel=${REL_TOL}"
echo "Constant quaternion: (${CONST_Q_W}, ${CONST_Q_X}, ${CONST_Q_Y}, ${CONST_Q_Z})"
echo

echo "Preparing benchmark data..."
"${DUCKDB_BIN}" -unsigned "${DB_PATH}" <<SQL
PRAGMA threads=${THREADS};

CREATE OR REPLACE TABLE bench_vec AS
SELECT
  i AS id,
  struct_pack(
    x := ((i % 1000)::DOUBLE * 0.001) + 0.1,
    y := ((i % 997)::DOUBLE * 0.001) + 0.2,
    z := ((i % 991)::DOUBLE * 0.001) + 0.3
  ) AS a,
  struct_pack(
    x := (((i * 7) % 1000)::DOUBLE * 0.001) + 0.4,
    y := (((i * 11) % 997)::DOUBLE * 0.001) + 0.5,
    z := (((i * 13) % 991)::DOUBLE * 0.001) + 0.6
  ) AS b,
  (((i % 17)::DOUBLE) + 1.0) / 17.0 AS s
FROM range(0, ${N_ROWS}) AS r(i);

ANALYZE bench_vec;

create or replace macro m_vadd(a, b) as
  struct_pack(
    x := a.x + b.x,
    y := a.y + b.y,
    z := a.z + b.z
  );

create or replace macro m_vscale(a, s) as
  struct_pack(
    x := a.x * s,
    y := a.y * s,
    z := a.z * s
  );
SQL

run_explain_analyze() {
  local query_sql="$1"
  "${DUCKDB_BIN}" -unsigned "${DB_PATH}" <<SQL
PRAGMA threads=${THREADS};
EXPLAIN ANALYZE ${query_sql}
SQL
}

run_explain_analyze_pair() {
  local first_query_sql="$1"
  local second_query_sql="$2"
  "${DUCKDB_BIN}" -unsigned "${DB_PATH}" <<SQL
PRAGMA threads=${THREADS};
EXPLAIN ANALYZE ${first_query_sql}
EXPLAIN ANALYZE ${second_query_sql}
SQL
}

run_case_pair_interleaved() {
  local label="$1"
  local ext_query_sql="$2"
  local mac_query_sql="$3"

  echo "== ${label} (paired/interleaved) =="
  echo "Warm-up (both impls)..."
  run_explain_analyze "${ext_query_sql}" >/dev/null
  run_explain_analyze "${mac_query_sql}" >/dev/null

  local ext_total="0"
  local mac_total="0"
  local run
  for run in $(seq 1 "${RUNS}"); do
    local out
    local ext_t
    local mac_t

    if (( run % 2 == 1 )); then
      out="$(run_explain_analyze_pair "${ext_query_sql}" "${mac_query_sql}")"
      mapfile -t ts < <(printf '%s\n' "${out}" | rg -o 'Total Time: [0-9.]+s' | rg -o '[0-9.]+')
      if [[ "${#ts[@]}" -lt 2 ]]; then
        echo "Failed to parse paired times for ${label} run ${run}" >&2
        exit 1
      fi
      ext_t="${ts[0]}"
      mac_t="${ts[1]}"
    else
      out="$(run_explain_analyze_pair "${mac_query_sql}" "${ext_query_sql}")"
      mapfile -t ts < <(printf '%s\n' "${out}" | rg -o 'Total Time: [0-9.]+s' | rg -o '[0-9.]+')
      if [[ "${#ts[@]}" -lt 2 ]]; then
        echo "Failed to parse paired times for ${label} run ${run}" >&2
        exit 1
      fi
      mac_t="${ts[0]}"
      ext_t="${ts[1]}"
    fi

    printf 'Run %d: extension=%ss macro=%ss\n' "${run}" "${ext_t}" "${mac_t}"
    ext_total="$(awk -v a="${ext_total}" -v b="${ext_t}" 'BEGIN { printf "%.9f", a + b }')"
    mac_total="$(awk -v a="${mac_total}" -v b="${mac_t}" 'BEGIN { printf "%.9f", a + b }')"
  done

  local ext_avg
  local mac_avg
  ext_avg="$(awk -v s="${ext_total}" -v n="${RUNS}" 'BEGIN { printf "%.9f", s / n }')"
  mac_avg="$(awk -v s="${mac_total}" -v n="${RUNS}" 'BEGIN { printf "%.9f", s / n }')"
  printf 'Average: extension=%ss macro=%ss\n\n' "${ext_avg}" "${mac_avg}"
  printf '%s,%s,%s\n' "${label}" "extension" "${ext_avg}" >> "${RESULTS_CSV}"
  printf '%s,%s,%s\n' "${label}" "macro" "${mac_avg}" >> "${RESULTS_CSV}"
}

run_case_single() {
  local label="$1"
  local query_sql="$2"

  echo "== ${label} =="
  echo "Warm-up..."
  run_explain_analyze "${query_sql}" >/dev/null

  local total="0"
  local run
  for run in $(seq 1 "${RUNS}"); do
    local out
    local t
    out="$(run_explain_analyze "${query_sql}")"
    t="$(printf '%s\n' "${out}" | rg -o 'Total Time: [0-9.]+s' | head -n1 | rg -o '[0-9.]+' || true)"
    if [[ -z "${t}" ]]; then
      echo "Failed to parse time for ${label} run ${run}" >&2
      exit 1
    fi
    printf 'Run %d: %ss\n' "${run}" "${t}"
    total="$(awk -v a="${total}" -v b="${t}" 'BEGIN { printf "%.9f", a + b }')"
  done

  local avg
  avg="$(awk -v s="${total}" -v n="${RUNS}" 'BEGIN { printf "%.9f", s / n }')"
  printf 'Average: %ss\n\n' "${avg}"
  printf '%s,%s,%s\n' "${label}" "baseline" "${avg}" >> "${RESULTS_CSV}"
}

rm -f "${RESULTS_CSV}"

Q_EXT='SELECT sum(o.x) + sum(o.y) + sum(o.z) FROM (SELECT se3_apply(vvec('"${CONST_Q_W}"', '"${CONST_Q_X}"', '"${CONST_Q_Y}"', '"${CONST_Q_Z}"'), vadd(vscale(a, s), b)) AS o FROM bench_vec) q;'
Q_MAC='SELECT sum(o.x) + sum(o.y) + sum(o.z) FROM (SELECT se3_apply(vvec('"${CONST_Q_W}"', '"${CONST_Q_X}"', '"${CONST_Q_Y}"', '"${CONST_Q_Z}"'), m_vadd(m_vscale(a, s), b)) AS o FROM bench_vec) q;'
Q_BASE='SELECT sum(o.x) + sum(o.y) + sum(o.z) FROM (SELECT se3_apply(vvec('"${CONST_Q_W}"', '"${CONST_Q_X}"', '"${CONST_Q_Y}"', '"${CONST_Q_Z}"'), a) AS o FROM bench_vec) q;'

if [[ "${CHECK_CORRECTNESS}" != "0" ]]; then
  echo "Running correctness checks..."
  row="$("${DUCKDB_BIN}" -unsigned -csv -noheader "${DB_PATH}" -c "PRAGMA threads=${THREADS}; WITH vals AS (SELECT ((${Q_EXT%?})) AS ext_v, ((${Q_MAC%?})) AS mac_v) SELECT ext_v, mac_v, abs(ext_v - mac_v) AS diff_v, ${ABS_TOL} + ${REL_TOL} * greatest(abs(ext_v), abs(mac_v)) AS tol_v FROM vals;")"
  row="$(printf '%s\n' "${row}" | tail -n1 | tr -d '[:space:]')"
  if [[ -z "${row}" ]]; then
    echo "Correctness check failed to produce values" >&2
    exit 1
  fi
  IFS=',' read -r ext_v mac_v diff_v tol_v <<< "${row}"
  if [[ -z "${ext_v}" || -z "${mac_v}" || -z "${diff_v}" || -z "${tol_v}" ]]; then
    echo "Correctness check parsing failed: '${row}'" >&2
    exit 1
  fi
  if ! awk -v d="${diff_v}" -v t="${tol_v}" 'BEGIN { exit !(d <= t) }'; then
    echo "Correctness check failed: ext=${ext_v} macro=${mac_v} |diff|=${diff_v} tol=${tol_v}" >&2
    exit 1
  fi
  echo "Correctness checks passed."
  echo
fi

run_case_pair_interleaved "pipeline_vscale_vadd_se3_apply" "${Q_EXT}" "${Q_MAC}"
run_case_single "baseline_se3_apply_aggregate_only" "${Q_BASE}"

echo "Summary (avg seconds):"
column -t -s, "${RESULTS_CSV}"
echo

echo "Derived metrics:"
awk -F, '
{
  key=$1; impl=$2; avg=$3+0.0;
  t[key,impl]=avg;
}
END {
  k="pipeline_vscale_vadd_se3_apply";
  b="baseline_se3_apply_aggregate_only";
  ext=t[k,"extension"];
  mac=t[k,"macro"];
  base=t[b,"baseline"];
  printf "%-36s %.9f\n", "pipeline extension avg (s)", ext;
  printf "%-36s %.9f\n", "pipeline macro avg (s)", mac;
  printf "%-36s %.9f\n", "baseline avg (s)", base;
  if (ext > 0) {
    printf "%-36s %.3fx\n", "macro/extension speedup", mac / ext;
  }
  ext_extra=ext-base;
  mac_extra=mac-base;
  printf "%-36s %.9f\n", "extension incremental (s)", ext_extra;
  printf "%-36s %.9f\n", "macro incremental (s)", mac_extra;
  if (ext_extra > 0) {
    printf "%-36s %.3fx\n", "macro/extension incremental", mac_extra / ext_extra;
  }
}' "${RESULTS_CSV}"
