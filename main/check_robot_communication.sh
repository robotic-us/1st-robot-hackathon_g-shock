#!/usr/bin/env bash
# phorce 통신 경로만 확인한다. 모션 goal은 절대 전송하지 않는다.

set -u

NIC="${PHORCE_NIC:-eno1}"
AXES="${PHORCE_AXES:-2}"
STARTUP_TIMEOUT="${PHORCE_STARTUP_TIMEOUT:-20}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${SCRIPT_DIR}/logs"
MONITOR_LOG="${LOG_DIR}/phorce_monitor.log"
ACTION_LOG="${LOG_DIR}/motion_action_server.log"
MONITOR_PID=""
ACTION_PID=""
CLEANED_UP=0
PROGRESS_ACTIVE=0
TOTAL_STEPS=6

info() { printf '[INFO] %s\n' "$*"; }
pass() { printf '[PASS] %s\n' "$*"; }
fail() {
  finish_progress_line
  printf '[FAIL] %s\n' "$*" >&2
}

progress() {
  local current="$1" label="$2" width=24 filled empty bar empty_bar
  filled=$((current * width / TOTAL_STEPS))
  empty=$((width - filled))
  printf -v bar '%*s' "$filled" ''
  bar="${bar// /#}"
  printf -v empty_bar '%*s' "$empty" ''
  empty_bar="${empty_bar// /-}"
  PROGRESS_ACTIVE=1
  if [[ -t 1 ]]; then
    printf '\r[진행] [%s%s] %d/%d %s\033[K' "$bar" "$empty_bar" "$current" "$TOTAL_STEPS" "$label"
  else
    printf '[진행] [%s%s] %d/%d %s\n' "$bar" "$empty_bar" "$current" "$TOTAL_STEPS" "$label"
  fi
}

finish_progress_line() {
  if (( PROGRESS_ACTIVE )) && [[ -t 1 ]]; then
    printf '\n'
  fi
  PROGRESS_ACTIVE=0
  return 0
}

show_log_hint() {
  fail "상세 로그: $*"
}

stop_process() {
  local pid="$1"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  if (( CLEANED_UP )); then
    return
  fi
  CLEANED_UP=1
  stop_process "$ACTION_PID"
  stop_process "$MONITOR_PID"
}

on_interrupt() {
  finish_progress_line
  info "사용자 요청으로 통신 확인 프로세스를 종료합니다."
  exit 0
}

trap cleanup EXIT
trap on_interrupt INT TERM

wait_for_graph_name() {
  local kind="$1"
  local expected="$2"
  local owner_pid="$3"
  local elapsed=0

  while (( elapsed < STARTUP_TIMEOUT )); do
    if ! kill -0 "$owner_pid" 2>/dev/null; then
      return 2
    fi
    if ros2 "$kind" list 2>/dev/null | grep -Fxq "$expected"; then
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done
  return 1
}

wait_for_self_tests() {
  local owner_pid="$1"
  local elapsed=0
  local pass_count=0

  while (( elapsed < STARTUP_TIMEOUT )); do
    if ! kill -0 "$owner_pid" 2>/dev/null; then
      return 2
    fi
    pass_count="$(grep -c 'PASS' "$MONITOR_LOG" 2>/dev/null || true)"
    if (( pass_count >= 13 )); then
      printf '%s' "$pass_count"
      return 0
    fi
    sleep 1
    ((elapsed += 1))
  done
  printf '%s' "$pass_count"
  return 1
}

if [[ ! -r "/sys/class/net/${NIC}/operstate" ]]; then
  fail "네트워크 장치 ${NIC}를 찾을 수 없습니다."
  exit 1
fi

progress 1 "${NIC} 물리 링크 확인"
LINK_STATE="$(cat "/sys/class/net/${NIC}/operstate")"
if [[ "$LINK_STATE" != "up" ]]; then
  finish_progress_line
  fail "${NIC}가 up이 아닙니다. 로봇 전원과 케이블을 확인하세요."
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  fail "ros2 명령을 찾을 수 없습니다. ROS 2 환경을 source 하세요."
  exit 1
fi

progress 2 "ROS 2 실행 파일 확인"
for spec in \
  "agx_phorce_bridge phorce_monitor" \
  "agx_motion_slot motion_action_server"; do
  read -r package executable <<<"$spec"
  if ! ros2 pkg executables "$package" 2>/dev/null | grep -Eq "^${package}[[:space:]]+${executable}$"; then
    fail "${package}/${executable} 실행 파일을 찾을 수 없습니다."
    exit 1
  fi
done

mkdir -p "$LOG_DIR"
: >"$MONITOR_LOG"
: >"$ACTION_LOG"

progress 3 "피드백 토픽 연결"
ros2 run agx_phorce_bridge phorce_monitor --ros-args \
  -p "nic:=${NIC}" -p mode:=command -p "axes:=${AXES}" -p mbx_enabled:=true \
  >"$MONITOR_LOG" 2>&1 &
MONITOR_PID=$!

if wait_for_graph_name topic /phorce/feedback "$MONITOR_PID"; then
  :
else
  result=$?
  fail "phorce_monitor가 피드백 토픽을 열지 못했습니다."
  if (( result == 2 )); then
    fail "phorce_monitor 프로세스가 조기에 종료되었습니다."
  fi
  show_log_hint "$MONITOR_LOG"
  exit 1
fi

progress 4 "자가검사 13개 확인"
if PASS_COUNT="$(wait_for_self_tests "$MONITOR_PID")"; then
  :
else
  result=$?
  fail "자가검사 PASS가 13개에 도달하지 못했습니다 (현재 ${PASS_COUNT:-0}개)."
  if (( result == 2 )); then
    fail "phorce_monitor 프로세스가 조기에 종료되었습니다."
  fi
  show_log_hint "$MONITOR_LOG"
  exit 1
fi

progress 5 "실제 피드백 수신 확인"
if timeout 5s ros2 topic echo /phorce/feedback --once >/dev/null 2>&1; then
  :
else
  fail "토픽은 있지만 5초 안에 피드백을 받지 못했습니다."
  show_log_hint "$MONITOR_LOG"
  exit 1
fi

progress 6 "액션 서버 연결"
ros2 run agx_motion_slot motion_action_server --ros-args -p backend:=ecat \
  >"$ACTION_LOG" 2>&1 &
ACTION_PID=$!

ACTION_NAME="/motion_action_server/play_motion_sequence"
if wait_for_graph_name action "$ACTION_NAME" "$ACTION_PID"; then
  :
else
  result=$?
  fail "motion_action_server가 액션 창구를 열지 못했습니다."
  if (( result == 2 )); then
    fail "motion_action_server 프로세스가 조기에 종료되었습니다."
  fi
  show_log_hint "$ACTION_LOG"
  exit 1
fi

finish_progress_line
pass "통신 정상: 링크, 피드백, 자가검사(${PASS_COUNT}개), 액션 서버"
info "모션 goal 미전송 · 노드 유지 중 (종료: Ctrl+C)"
info "로그: ${MONITOR_LOG}, ${ACTION_LOG}"

while kill -0 "$MONITOR_PID" 2>/dev/null && kill -0 "$ACTION_PID" 2>/dev/null; do
  sleep 1
done

fail "실행 중인 통신 프로세스 하나가 종료되었습니다. 로그를 확인하세요."
exit 1
