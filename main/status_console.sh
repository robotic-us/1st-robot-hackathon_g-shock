#!/usr/bin/env bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
printf 'G-SHOCK 오류/상태 로그\n'
printf '화면을 지우거나 반복 갱신하지 않습니다. 새 상태·오류만 아래에 추가됩니다.\n\n'

mkdir -p "$SCRIPT_DIR/logs"
touch "$SCRIPT_DIR/logs/phorce_monitor.log" "$SCRIPT_DIR/logs/motion_action_server.log"

grep -hEi 'state|status|ready|error|fail|fault|abort|reject|cancel|warn' \
  "$SCRIPT_DIR"/logs/*.log 2>/dev/null | tail -30 || true

tail -n 0 -F "$SCRIPT_DIR/logs/phorce_monitor.log" \
              "$SCRIPT_DIR/logs/motion_action_server.log" 2>/dev/null \
  | stdbuf -oL grep -Ei 'state|status|ready|error|fail|fault|abort|reject|cancel|warn'
