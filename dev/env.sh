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
# 주의: main/run_integrated_system.sh 는 ROS_DOMAIN_ID=21 을 강제하지만,
# 2026-08-07 실기 검증에 실제로 쓰인 노드들은 ROS_DOMAIN_ID 미설정(=0)으로
# 떠 있었다. 여기서는 0 으로 고정한다. 21 로 옮기려면 아래 세 값을 함께 바꾸고
# dev/bin/doctor.sh 로 노드가 같은 domain 에 있는지 확인할 것.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
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
