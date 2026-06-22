#!/usr/bin/env bash
# cpu_affinity.env 가 있으면 로드한 뒤 5_end_to_end_measurement.py 실행
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "${SCRIPT_DIR}/cpu_affinity.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/cpu_affinity.env"
  set +a
fi
exec python3 "${SCRIPT_DIR}/../test/5_end_to_end_measurement.py" "$@"
