#!/usr/bin/env bash
# 실기 스택(phorce_monitor + motion_action_server)을 domain 21 로 확실히 띄운다.
#
# 왜 이 래퍼가 필요한가:
#   main/check_robot_communication.sh 에는 ROS_DOMAIN_ID 설정이 없다.
#   그냥 실행하면 domain 0(모두의 기본값)으로 나가고, 공유 WiFi 상의 다른 팀
#   로봇과 ROS 그래프가 합쳐진다. 2026-08-07 에 실제로 그렇게 돼서 phorce list 가
#   남의 로봇 슬롯(26개)을 돌려줬다.
#
# 이 스크립트는 모션 goal 을 한 건도 보내지 않는다(원본 스크립트의 성질 그대로).
# 로봇은 움직이지 않고 통신만 확인한 뒤 노드를 유지한다. 종료는 Ctrl+C.
set -euo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

export ROS_DOMAIN_ID="$PHORCE_ROBOT_DOMAIN_ID"

if [[ "$ROS_DOMAIN_ID" == "0" ]]; then
  printf '[FAIL] domain 0 으로는 띄우지 않습니다 — 공유 네트워크에서 남의 로봇과 섞입니다.\n' >&2
  printf '       dev/env.sh 의 ROS_DOMAIN_ID 를 21 등으로 두십시오.\n' >&2
  exit 2
fi

running="$(pgrep -fc '/agx_motion_slot/motion_action_server' 2>/dev/null || echo 0)"
if [[ "$running" -gt 0 ]]; then
  printf '[WARN] 이미 motion_action_server 가 %s개 떠 있습니다.\n' "$running" >&2
  printf '       옛 스택이 domain 0 에 남아 있으면 먼저 그 터미널에서 Ctrl+C 로 내리세요.\n' >&2
  pgrep -af '/agx_motion_slot/motion_action_server' >&2 || true
  printf '계속하려면 yes 를 입력하세요: ' >&2
  read -r answer
  [[ "$answer" == "yes" ]] || { printf '취소했습니다.\n' >&2; exit 1; }
fi

printf '[INFO] 실기 스택 기동 — ROS_DOMAIN_ID=%s (모션 goal 미전송)\n' "$ROS_DOMAIN_ID"
printf '[INFO] 조회는 같은 domain 으로: phorce list --domain-id %s\n' "$ROS_DOMAIN_ID"
exec "$PHORCE_BASELINE/check_robot_communication.sh"
