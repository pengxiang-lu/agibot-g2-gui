#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
COLLECT_PYTHON="${LEROBOT_PYTHON:-$HOME/anaconda3/envs/lerobot/bin/python}"
if [[ ! -x "$COLLECT_PYTHON" ]]; then
    echo "找不到 LeRobot Python：$COLLECT_PYTHON。请设置 LEROBOT_PYTHON。" >&2
    exit 1
fi
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
exec "$COLLECT_PYTHON" "$SCRIPT_DIR/src/collect.py" "$@"
