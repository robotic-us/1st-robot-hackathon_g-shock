#!/usr/bin/env bash
set -u
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TERMINATOR_PID=""
# The validated real-robot setup uses domain 21. Export once so the GUI and
# every Terminator child (monitor/action/status consoles) share one graph.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

cleanup() {
  if [[ -n "$TERMINATOR_PID" ]] && kill -0 "$TERMINATOR_PID" 2>/dev/null; then
    kill -TERM "$TERMINATOR_PID" 2>/dev/null || true
    wait "$TERMINATOR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ -z "${DISPLAY:-}" ]]; then
  printf '[FAIL] GUI 세션이 아닙니다. 데스크톱 터미널에서 실행하세요.\n' >&2
  exit 1
fi
for command_name in terminator python3 ros2; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf '[FAIL] 필요한 명령을 찾을 수 없습니다: %s\n' "$command_name" >&2
    exit 1
  fi
done

PCM_PORT="${PHORCE_PCM_PORT:-/dev/ttyACM0}"
if [[ -e "$PCM_PORT" && ( ! -r "$PCM_PORT" || ! -w "$PCM_PORT" ) ]]; then
  printf '[WARN] PCM USB 포트 권한이 없습니다: %s\n' "$PCM_PORT" >&2
  printf '[WARN] 현재 사용자를 dialout 그룹에 추가하고 다시 로그인해야 USB 1번 버튼 기능을 쓸 수 있습니다.\n' >&2
fi

printf '[INFO] 오른쪽: terminator three 레이아웃 시작\n'
printf '[INFO] ROS_DOMAIN_ID=%s (GUI와 모든 통신 프로세스 공통)\n' "$ROS_DOMAIN_ID"
terminator --no-dbus --config "$SCRIPT_DIR/terminator-three.conf" --layout three &
TERMINATOR_PID=$!
sleep 1
if ! kill -0 "$TERMINATOR_PID" 2>/dev/null; then
  printf '[FAIL] terminator를 시작하지 못했습니다.\n' >&2
  exit 1
fi

printf '[INFO] 왼쪽: 충격 감지 GUI 시작\n'
python3 "$SCRIPT_DIR/shock_guard_gui.py"
GUI_STATUS=$?
printf '[INFO] GUI 종료. 통신 스택과 terminator를 정리합니다.\n'
exit "$GUI_STATUS"
