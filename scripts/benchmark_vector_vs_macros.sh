#!/usr/bin/env bash
set -euo pipefail

DUCKDB_BIN="${DUCKDB_BIN:-./build/release/duckdb}"
DB_PATH="${DB_PATH:-/tmp/se3_vector_vs_macro_benchmark.duckdb}"
THREADS="${THREADS:-8}"
N_ROWS="${N_ROWS:-5000000}"
RUNS="${RUNS:-3}"
RESULTS_CSV="${RESULTS_CSV:-/tmp/se3_vector_vs_macro_results.csv}"
CHECK_CORRECTNESS="${CHECK_CORRECTNESS:-1}"
ABS_TOL="${ABS_TOL:-1e-9}"
REL_TOL="${REL_TOL:-1e-12}"

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

create or replace macro m_vsub(a, b) as
  struct_pack(
    x := a.x - b.x,
    y := a.y - b.y,
    z := a.z - b.z
  );

create or replace macro m_vscale(a, s) as
  struct_pack(
    x := a.x * s,
    y := a.y * s,
    z := a.z * s
  );

create or replace macro m_vdot(a, b) as
  a.x * b.x + a.y * b.y + a.z * b.z;

create or replace macro m_vnorm2(a) as
  m_vdot(a, a);

create or replace macro m_vnorm(a) as
  sqrt(m_vnorm2(a));

create or replace macro m_vnormalize(a) as
  m_vscale(a, 1.0 / m_vnorm(a));
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

    # Alternate order per run to reduce thermal/cache/order bias.
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

    ext_total="$(awk -v a="${ext_total}" -v b="${ext_t}" 'BEGIN { printf "%.6f", a + b }')"
    mac_total="$(awk -v a="${mac_total}" -v b="${mac_t}" 'BEGIN { printf "%.6f", a + b }')"
  done

  local ext_avg
  local mac_avg
  ext_avg="$(awk -v s="${ext_total}" -v n="${RUNS}" 'BEGIN { printf "%.6f", s / n }')"
  mac_avg="$(awk -v s="${mac_total}" -v n="${RUNS}" 'BEGIN { printf "%.6f", s / n }')"

  printf 'Average: extension=%ss macro=%ss\n\n' "${ext_avg}" "${mac_avg}"
  printf '%s,%s,%s\n' "${label}" "extension" "${ext_avg}" >> "${RESULTS_CSV}"
  printf '%s,%s,%s\n' "${label}" "macro" "${mac_avg}" >> "${RESULTS_CSV}"
}

rm -f "${RESULTS_CSV}"

# Compute each vector expression once per row in a subquery to avoid
# benchmarking repeated expression duplication.
EXT_Q_VADD='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT vadd(a, b) AS r FROM bench_vec) q;'
MAC_Q_VADD='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT m_vadd(a, b) AS r FROM bench_vec) q;'

EXT_Q_VSUB='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT vsub(a, b) AS r FROM bench_vec) q;'
MAC_Q_VSUB='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT m_vsub(a, b) AS r FROM bench_vec) q;'

EXT_Q_VSCALE='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT vscale(a, s) AS r FROM bench_vec) q;'
MAC_Q_VSCALE='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT m_vscale(a, s) AS r FROM bench_vec) q;'

EXT_Q_VDOT='SELECT sum(vdot(a, b)) FROM bench_vec;'
MAC_Q_VDOT='SELECT sum(m_vdot(a, b)) FROM bench_vec;'

EXT_Q_VNORM2='SELECT sum(vnorm2(a)) FROM bench_vec;'
MAC_Q_VNORM2='SELECT sum(m_vnorm2(a)) FROM bench_vec;'

EXT_Q_VNORM='SELECT sum(vnorm(a)) FROM bench_vec;'
MAC_Q_VNORM='SELECT sum(m_vnorm(a)) FROM bench_vec;'

EXT_Q_VNORMALIZE='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT vnormalize(a) AS r FROM bench_vec) q;'
MAC_Q_VNORMALIZE='SELECT sum(r.x) + sum(r.y) + sum(r.z) FROM (SELECT m_vnormalize(a) AS r FROM bench_vec) q;'

if [[ "${CHECK_CORRECTNESS}" != "0" ]]; then
  echo "Running correctness checks..."
  check_case() {
    local label="$1"
    local ext_q="$2"
    local mac_q="$3"
    local row
    row="$("${DUCKDB_BIN}" -unsigned -csv -noheader "${DB_PATH}" -c "PRAGMA threads=${THREADS}; WITH vals AS (SELECT ((${ext_q%?})) AS ext_v, ((${mac_q%?})) AS mac_v) SELECT ext_v, mac_v, abs(ext_v - mac_v) AS diff_v, ${ABS_TOL} + ${REL_TOL} * greatest(abs(ext_v), abs(mac_v)) AS tol_v FROM vals;")"
    row="$(printf '%s\n' "${row}" | tail -n1 | tr -d '[:space:]')"
    if [[ -z "${row}" ]]; then
      echo "Correctness check failed to produce values for ${label}" >&2
      exit 1
    fi
    local ext_v
    local mac_v
    local diff_v
    local tol_v
    IFS=',' read -r ext_v mac_v diff_v tol_v <<< "${row}"
    if [[ -z "${ext_v}" || -z "${mac_v}" || -z "${diff_v}" || -z "${tol_v}" ]]; then
      echo "Correctness check parsing failed for ${label}: '${row}'" >&2
      exit 1
    fi
    if ! awk -v d="${diff_v}" -v t="${tol_v}" '
      BEGIN {
        exit !(d <= t);
      }
    '; then
      echo "Correctness check failed for ${label}: ext=${ext_v} macro=${mac_v} |diff|=${diff_v} tol=${tol_v}" >&2
      exit 1
    fi
  }

  check_case "vadd" "${EXT_Q_VADD}" "${MAC_Q_VADD}"
  check_case "vsub" "${EXT_Q_VSUB}" "${MAC_Q_VSUB}"
  check_case "vscale" "${EXT_Q_VSCALE}" "${MAC_Q_VSCALE}"
  check_case "vdot" "${EXT_Q_VDOT}" "${MAC_Q_VDOT}"
  check_case "vnorm2" "${EXT_Q_VNORM2}" "${MAC_Q_VNORM2}"
  check_case "vnorm" "${EXT_Q_VNORM}" "${MAC_Q_VNORM}"
  check_case "vnormalize" "${EXT_Q_VNORMALIZE}" "${MAC_Q_VNORMALIZE}"

  echo "Correctness checks passed."
  echo
fi

for label in vadd vsub vscale vdot vnorm2 vnorm vnormalize; do
  case "${label}" in
    vadd) ext_q="${EXT_Q_VADD}"; mac_q="${MAC_Q_VADD}" ;;
    vsub) ext_q="${EXT_Q_VSUB}"; mac_q="${MAC_Q_VSUB}" ;;
    vscale) ext_q="${EXT_Q_VSCALE}"; mac_q="${MAC_Q_VSCALE}" ;;
    vdot) ext_q="${EXT_Q_VDOT}"; mac_q="${MAC_Q_VDOT}" ;;
    vnorm2) ext_q="${EXT_Q_VNORM2}"; mac_q="${MAC_Q_VNORM2}" ;;
    vnorm) ext_q="${EXT_Q_VNORM}"; mac_q="${MAC_Q_VNORM}" ;;
    vnormalize) ext_q="${EXT_Q_VNORMALIZE}"; mac_q="${MAC_Q_VNORMALIZE}" ;;
    *) echo "Unknown label: ${label}" >&2; exit 1 ;;
  esac

  run_case_pair_interleaved "${label}" "${ext_q}" "${mac_q}"
done

echo "Summary (avg seconds):"
column -t -s, "${RESULTS_CSV}"
echo
echo "Speedup (macro_avg / extension_avg):"
awk -F, '
{
  key=$1
  impl=$2
  avg=$3+0.0
  if (impl=="extension") ext[key]=avg
  if (impl=="macro") mac[key]=avg
}
END {
  printf "%-12s %-12s\n", "case", "speedup"
  for (k in ext) {
    if (mac[k] > 0) {
      printf "%-12s %.3fx\n", k, mac[k] / ext[k]
    }
  }
}' "${RESULTS_CSV}" | sort
