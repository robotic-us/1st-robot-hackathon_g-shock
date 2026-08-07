# 공통 환경 — 실행하지 말고 `source dev/env.sh` 로 불러 쓴다.
#
# robot 과 sim 은 같은 DDS domain 을 쓰고 namespace 로만 가른다.
# phorce CLI 의 target 해석 규칙(_impl.py resolve_target)에 맞춰 놓은 값이다.
#   robot      → namespace $PHORCE_ROBOT_NAMESPACE (기본 root)
#   sim:<세션> → namespace $PHORCE_SIM_NAMESPACE_ROOT/<세션>

PHORCE_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PHORCE_WS
export PHORCE_DEV="$PHORCE_WS/dev"
export PHORCE_CATALOG="$PHORCE_DEV/catalog"
export PHORCE_DEV_LOGS="$PHORCE_DEV/logs"
export PHORCE_BASELINE="$PHORCE_WS/main"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # ROS setup 스크립트는 unbound 변수를 참조하므로 set -u 를 잠깐 끈다.
  _phorce_had_u=0
  [[ $- == *u* ]] && { _phorce_had_u=1; set +u; }
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  (( _phorce_had_u )) && set -u
  unset _phorce_had_u
fi

# ── domain ──────────────────────────────────────────────────────────────────
# 반드시 21. domain 0 을 쓰면 안 된다.
#
# 이 장비는 공유 WiFi(10.249.184.0/24)에 붙어 있고 ROS_LOCALHOST_ONLY=0 이라
# ROS 그래프가 네트워크 전체로 퍼진다. 2026-08-07 에 스택이 domain 0(기본값)으로
# 떠 있었더니 같은 해커톤 네트워크의 다른 팀 로봇과 그래프가 합쳐져서,
# `phorce list` 가 호출할 때마다 남의 로봇 카탈로그를 돌려줬다(슬롯 8개 → 26개 → 1개).
# 조회만 그런 게 아니라 `phorce play` 가 남의 로봇을 움직일 수 있는 상태였다.
# pcm 은 cancel 을 거부하므로 잘못 나간 모션은 끝까지 완주한다.
#
# main/run_integrated_system.sh 는 21 을 강제하지만
# main/check_robot_communication.sh 에는 그 줄이 없어서, 후자로 띄우면 0 으로 샌다.
# 실기 스택은 dev/bin/robot-stack.sh 로 띄우면 21 이 보장된다.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export PHORCE_ROBOT_DOMAIN_ID="${PHORCE_ROBOT_DOMAIN_ID:-$ROS_DOMAIN_ID}"
export PHORCE_SIM_DOMAIN_ID="${PHORCE_SIM_DOMAIN_ID:-$ROS_DOMAIN_ID}"

# ── namespace ───────────────────────────────────────────────────────────────
export PHORCE_SIM_NAMESPACE_ROOT="${PHORCE_SIM_NAMESPACE_ROOT:-/sim}"
export PHORCE_SIM_SESSION="${PHORCE_SIM_SESSION:-dev}"

# ── 노드 실행 파일 ──────────────────────────────────────────────────────────
# `ros2 run` 은 python 래퍼가 실제 노드를 자식으로 띄우므로, 래퍼를 kill 해도
# 자식이 살아남아 옛 설정의 서버가 계속 응답한다(실제로 테스트가 이걸로 틀렸다).
# 백그라운드로 띄우고 정확히 죽이려면 실행 파일을 직접 쓴다.
export PHORCE_MOTION_SERVER_BIN="${PHORCE_MOTION_SERVER_BIN:-/opt/ros/humble/lib/agx_motion_slot/motion_action_server}"

mkdir -p "$PHORCE_DEV_LOGS"
