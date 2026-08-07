#!/usr/bin/env bash
# sim 백엔드로 motion_action_server 를 띄운다. 실물은 절대 움직이지 않는다.
# 사용법: dev/bin/sim-up.sh [세션이름]
set -euo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

SESSION="${1:-$PHORCE_SIM_SESSION}"
if ! [[ "$SESSION" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
  printf '[FAIL] 세션 이름은 영문자/밑줄로 시작하는 ROS 이름이어야 합니다: %s\n' "$SESSION" >&2
  exit 2
fi
NS="${PHORCE_SIM_NAMESPACE_ROOT}/${SESSION}"
LOG="$PHORCE_DEV_LOGS/sim_${SESSION}.log"
PIDFILE="$PHORCE_DEV_LOGS/sim_${SESSION}.pid"

if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  printf '[SKIP] sim:%s 가 이미 pid %s 로 떠 있습니다.\n' "$SESSION" "$(cat "$PIDFILE")"
  exit 0
fi

printf '[INFO] sim:%s 기동 — namespace=%s domain=%s\n' "$SESSION" "$NS" "$PHORCE_SIM_DOMAIN_ID"
printf '[INFO] motion_dir=%s\n' "$PHORCE_CATALOG"
ROS_DOMAIN_ID="$PHORCE_SIM_DOMAIN_ID" \
"$PHORCE_MOTION_SERVER_BIN" --ros-args \
  -r __ns:="$NS" \
  -p backend:=sim \
  -p motion_dir:="$PHORCE_CATALOG" \
  >"$LOG" 2>&1 &
echo $! >"$PIDFILE"

for _ in $(seq 40); do
  if phorce list --target "sim:$SESSION" >/dev/null 2>&1; then
    printf '[PASS] sim:%s 응답 확인 (로그: %s)\n' "$SESSION" "$LOG"
    phorce list --target "sim:$SESSION"
    exit 0
  fi
  if ! kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    printf '[FAIL] 서버가 조기 종료했습니다. 로그 마지막 20줄:\n' >&2
    tail -20 "$LOG" >&2
    rm -f "$PIDFILE"
    exit 1
  fi
  sleep 0.25
done
printf '[FAIL] sim:%s 가 10초 안에 응답하지 않았습니다. 로그: %s\n' "$SESSION" "$LOG" >&2
tail -20 "$LOG" >&2
exit 1
