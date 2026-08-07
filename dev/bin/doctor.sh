#!/usr/bin/env bash
# 워크스페이스/스택 상태 점검. 아무것도 실행하거나 움직이지 않는 읽기 전용 진단.
set -uo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

warn() { printf '  [WARN] %s\n' "$1"; }
ok()   { printf '  [OK]   %s\n' "$1"; }

printf '== 워크스페이스 ==\n'
ok "root      $PHORCE_WS"
ok "catalog   $PHORCE_CATALOG ($(ls "$PHORCE_CATALOG"/motion_*.csv 2>/dev/null | wc -l)개 모션)"
ok "baseline  $PHORCE_BASELINE (읽기 참조용 — 여기서 고치지 말 것)"

printf '\n== domain ==\n'
ok "이 셸: ROS_DOMAIN_ID=$ROS_DOMAIN_ID (robot=$PHORCE_ROBOT_DOMAIN_ID sim=$PHORCE_SIM_DOMAIN_ID)"

# 이 장비는 공유 WiFi 에 붙어 있고 ROS_LOCALHOST_ONLY=0 이므로 ROS 그래프가
# 네트워크 전체로 퍼진다. domain 0 은 모두의 기본값이라 반드시 피해야 한다.
if [[ "${ROS_LOCALHOST_ONLY:-0}" == "0" ]]; then
  shared_if="$(ip -4 -br addr 2>/dev/null | awk '$1!="lo" && $3!="" {print $1" "$3}' | grep -v docker | head -1)"
  [[ -n "$shared_if" ]] && ok "그래프가 네트워크로 퍼짐: $shared_if (ROS_LOCALHOST_ONLY=0)"
fi

# 실기 스택이 실제로 어느 domain 에 있는지 훑는다.
printf '  domain 탐색:\n'
for d in 0 "$PHORCE_ROBOT_DOMAIN_ID"; do
  graph="$(ROS_DOMAIN_ID="$d" timeout 8s ros2 node list 2>/dev/null | grep -c '^/motion_action_server$')"
  printf '    domain %-3s → /motion_action_server %s개\n' "$d" "${graph:-0}"
  if [[ "$d" == "0" && "${graph:-0}" -gt 0 ]]; then
    warn "domain 0 에 노드가 보입니다. 0 은 모든 팀의 기본값이라 남의 로봇과 그래프가 섞입니다."
  fi
done

# 그래프의 서버 수 > 로컬 프로세스 수 이면 남의 장비가 섞인 것이다.
local_n="$(pgrep -fc '/agx_motion_slot/motion_action_server' 2>/dev/null || echo 0)"
graph_n="$(timeout 8s ros2 node list 2>/dev/null | grep -c '^/motion_action_server$')"
ok "motion_action_server — 로컬 프로세스 ${local_n}개 / domain $ROS_DOMAIN_ID 그래프 ${graph_n:-0}개"
if [[ "${graph_n:-0}" -gt "${local_n:-0}" ]]; then
  warn "그래프에 로컬보다 많은 서버가 보입니다 — 다른 팀 장비가 같은 domain 에 섞여 있습니다."
  warn "이 상태의 phorce list 는 남의 로봇 카탈로그를 돌려줄 수 있고,"
  warn "phorce play 는 남의 로봇을 움직일 수 있습니다. domain 을 격리하기 전엔 play 금지."
fi

printf '\n== 실기(robot) 상태 ==\n'
if robot_json="$(timeout 10s phorce list --target robot --json 2>/dev/null)"; then
  python3 - "$robot_json" <<'PY'
import json, sys
d = json.loads(sys.argv[1])
slots = d.get("motions") or []
ids = [s.get("ms_id") for s in slots]
unnamed = [s.get("ms_id") for s in slots if "이름 없음" in (s.get("name") or "")]
print(f"  [OK]   적재 슬롯 {len(ids)}개: {ids}")
if unnamed:
    print(f"  [WARN] 이름 없는 슬롯 {unnamed} — 서버가 motion_dir 없이 떠서 이름을 못 붙입니다.")
    print( "  [WARN] 이 상태에서는 shock_guard_gui 의 GRAB/BOX/TAPE/원점복귀 버튼이 전부 비활성입니다.")
    print( "  [INFO] 우회: GUI 의 '모션 ID 직접 실행' 또는 phorce play <id>")
for issue in d.get("issues") or []:
    print(f"  [WARN] {issue}")
PY
else
  warn "robot target 응답 없음 — 스택이 안 떠 있거나 domain 이 다릅니다."
fi

printf '\n== sim 세션 ==\n'
shopt -s nullglob
found=0
for pidfile in "$PHORCE_DEV_LOGS"/sim_*.pid; do
  name="$(basename -- "$pidfile" .pid)"; name="${name#sim_}"
  if kill -0 "$(cat "$pidfile")" 2>/dev/null; then
    ok "sim:$name 실행 중 (pid $(cat "$pidfile"))"; found=1
  else
    warn "sim:$name pidfile 은 있는데 프로세스가 없습니다 — sim-down.sh 로 정리하세요"; found=1
  fi
done
(( found )) || printf '  (없음 — dev/bin/sim-up.sh 로 띄우세요)\n'

printf '\n== 안전 ==\n'
warn "pcm 은 EtherCAT 정지를 지원하지 않습니다. 액션 cancel 은 항상 거부되며"
warn "shock_guard_gui 의 DOB 충격 감지 자동정지는 실물에서 동작하지 않습니다."
warn "실물 유일한 정지 수단은 물리 E-Stop 이고, 눌리면 래치되어 재부팅·재호밍이 필요합니다."
warn "새 기능은 backend:=sim(dev/bin/sim-up.sh) 에서 먼저 검증하세요."
