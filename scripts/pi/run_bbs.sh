#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV_DIR="${REPO_DIR}/.venv"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "Virtual environment not found at ${VENV_DIR}."
  echo "Create it with: python -m venv .venv && source .venv/bin/activate && pip install -e ."
  exit 1
fi

source "${VENV_DIR}/bin/activate"
exec python "${REPO_DIR}/scripts/ble_send_ttr.py" "$@"