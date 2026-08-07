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
ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (robot=$PHORCE_ROBOT_DOMAIN_ID sim=$PHORCE_SIM_DOMAIN_ID)"
legacy="$(grep -oE 'ROS_DOMAIN_ID:-[0-9]+' "$PHORCE_BASELINE/run_integrated_system.sh" 2>/dev/null | head -1 | cut -d- -f3)"
if [[ -n "$legacy" && "$legacy" != "$ROS_DOMAIN_ID" ]]; then
  warn "main/run_integrated_system.sh 는 domain $legacy 을 강제합니다. 지금 이 셸($ROS_DOMAIN_ID)과 다르므로"
  warn "두 스택을 섞어 띄우면 서로를 못 봅니다."
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
