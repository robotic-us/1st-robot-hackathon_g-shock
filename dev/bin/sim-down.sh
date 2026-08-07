#!/usr/bin/env bash
# sim-up.sh 로 띄운 세션을 내린다. 인자가 없으면 모든 sim 세션.
set -euo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

shopt -s nullglob
if (( $# )); then
  PIDFILES=("$PHORCE_DEV_LOGS/sim_$1.pid")
else
  PIDFILES=("$PHORCE_DEV_LOGS"/sim_*.pid)
fi

if (( ${#PIDFILES[@]} == 0 )); then
  printf '[INFO] 내릴 sim 세션이 없습니다.\n'
  exit 0
fi

for pidfile in "${PIDFILES[@]}"; do
  [[ -f "$pidfile" ]] || continue
  name="$(basename -- "$pidfile" .pid)"
  pid="$(cat "$pidfile")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 20); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL "$pid" 2>/dev/null || true
    printf '[INFO] %s (pid %s) 종료\n' "$name" "$pid"
  else
    printf '[INFO] %s 는 이미 죽어 있었습니다\n' "$name"
  fi
  rm -f "$pidfile"
done
