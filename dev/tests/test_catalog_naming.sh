#!/usr/bin/env bash
# 실험 1 — motion_dir 을 물리면 슬롯 이름이 붙는가?
#
# 배경: 실기 서버는 지금 motion_dir 없이 떠 있어서 8개 슬롯(1,2,3,4,5,48,49,50)을
# 전부 보여주지만 이름이 전부 "(이름 없음 — 로봇에 적재된 슬롯 N)" 이다.
# shock_guard_gui 는 이름이 정확히 GRAB/BOX/TAPE/HOME 인 슬롯만 버튼에 연결하므로
# 이 상태에서는 버튼 4개가 전부 비활성이 된다.
#
# 이 테스트가 증명하는 것 : motion_dir 을 주면 CSV 의 MS Name 이 목록에 붙는다.
# 이 테스트가 증명 못 하는 것: backend:=ecat + motion_dir 조합에서 로컬 CSV 가
#   없는 슬롯(1/4/48)이 목록에 남는지. sim 백엔드에는 조회할 실기가 없어서
#   카탈로그가 motion_dir 만으로 구성되기 때문이다. 그건 실기 서버를
#   motion_dir 을 주고 재기동해서 phorce list 로 확인해야 한다(재생은 하지 않으므로
#   로봇은 움직이지 않는다).
set -uo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

SESSION="catalogtest"
NS="${PHORCE_SIM_NAMESPACE_ROOT}/${SESSION}"
PIDFILE="$PHORCE_DEV_LOGS/sim_${SESSION}.pid"
LOG="$PHORCE_DEV_LOGS/sim_${SESSION}.log"
FAIL=0

cleanup() {
  [[ -f "$PIDFILE" ]] || return 0
  kill -TERM "$(cat "$PIDFILE")" 2>/dev/null || true
  sleep 0.5
  kill -KILL "$(cat "$PIDFILE")" 2>/dev/null || true
  rm -f "$PIDFILE"
}
trap cleanup EXIT

run_case() {
  local label="$1" motion_dir="$2"
  cleanup
  local args=(-r __ns:="$NS" -p backend:=sim)
  [[ -n "$motion_dir" ]] && args+=(-p motion_dir:="$motion_dir")
  ROS_DOMAIN_ID="$PHORCE_SIM_DOMAIN_ID" \
    "$PHORCE_MOTION_SERVER_BIN" --ros-args "${args[@]}" >"$LOG" 2>&1 &
  echo $! >"$PIDFILE"
  local out=""
  for _ in $(seq 40); do
    out="$(timeout 5s phorce list --target "sim:$SESSION" --json 2>/dev/null)" && [[ -n "$out" ]] && break
    sleep 0.25
  done
  printf '\n--- %s ---\n' "$label"
  if [[ -z "$out" ]]; then
    printf '  [FAIL] sim 이 응답하지 않았습니다\n'; tail -10 "$LOG"; FAIL=1; return
  fi
  python3 - "$out" <<'PY'
import json, sys
d = json.loads(sys.argv[1])
slots = d.get("motions") or []
print(f"  슬롯 {len(slots)}개")
for s in slots:
    print(f"    {s.get('ms_id'):>3}  {s.get('name')}")
for issue in d.get("issues") or []:
    print(f"  issue: {issue}")
PY
}

run_case "motion_dir 없음" ""
run_case "motion_dir = dev/catalog" "$PHORCE_CATALOG"

cat <<'EOF'

기대 결과:
  · "motion_dir 없음"      → 슬롯 0개 (sim 에는 조회할 실기가 없다)
  · "motion_dir = catalog" → 슬롯 5개, 이름이 grab/box/BUTTON1_READY/grab_test/IK

이게 나오면 motion_dir 이 이름 문제의 해법인 것은 확인된 것이다.
남은 확인: 실기(ecat)에서 motion_dir 을 주었을 때 1/4/48 이 목록에 남는지.
EOF
exit "$FAIL"
