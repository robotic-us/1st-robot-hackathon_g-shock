#!/usr/bin/env bash
# 종단 검증 — 비전 좌표로 만든 grab 모션이 실제로 슬롯에 실려 재생되는가.
#
#   비전 좌표 → plan_grab.py → motion_NN.csv → sim motion_dir → phorce play
#
# sim 백엔드라 실물은 움직이지 않는다.
set -uo pipefail
# shellcheck disable=SC1091
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)/env.sh"

PLANNER="$PHORCE_DEV/features/grab_planner/plan_grab.py"
SESSION="grabe2e"
NS="${PHORCE_SIM_NAMESPACE_ROOT}/${SESSION}"
WORK="$(mktemp -d)"
PIDFILE="$PHORCE_DEV_LOGS/sim_${SESSION}.pid"
LOG="$PHORCE_DEV_LOGS/sim_${SESSION}.log"
SLOT=11
FAIL=0

cleanup() {
  [[ -f "$PIDFILE" ]] && { kill -TERM "$(cat "$PIDFILE")" 2>/dev/null; sleep 0.5
                           kill -KILL "$(cat "$PIDFILE")" 2>/dev/null; rm -f "$PIDFILE"; }
  rm -rf "$WORK"
}
trap cleanup EXIT

step() { printf '\n--- %s ---\n' "$1"; }
ok()   { printf '  [OK  ] %s\n' "$1"; }
bad()  { printf '  [FAIL] %s\n' "$1"; FAIL=1; }

step "1. 물체 위치 (X=15, Y=45) 로 grab 모션 생성"
if "$PLANNER" plan --x 15 --y 45 --slot "$SLOT" --name GRAB_E2E --out "$WORK" >"$WORK/plan.txt" 2>&1; then
  ok "생성 성공"
  grep -E "^  wp" "$WORK/plan.txt" | sed 's/^/    /'
else
  bad "생성 실패"; cat "$WORK/plan.txt"; exit 1
fi

step "2. 산출물 검증"
csv="$WORK/motion_$(printf '%02d' $SLOT).csv"
memo="$WORK/motion_$(printf '%02d' $SLOT).memo.json"
[[ -f "$csv" && -f "$memo" ]] && ok "CSV/memo 존재" || bad "산출물 없음"
actual="$(sha256sum "$csv" | cut -d' ' -f1)"
expect="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['motion_sha256'])" "$memo")"
[[ "$actual" == "$expect" ]] && ok "memo 해시 일치" || bad "해시 불일치"

step "3. sim 에 그 카탈로그로 서버 기동"
ROS_DOMAIN_ID="$PHORCE_SIM_DOMAIN_ID" \
  "$PHORCE_MOTION_SERVER_BIN" --ros-args -r __ns:="$NS" \
  -p backend:=sim -p motion_dir:="$WORK" >"$LOG" 2>&1 &
echo $! >"$PIDFILE"
listing=""
for _ in $(seq 40); do
  listing="$(timeout 5s phorce list --target "sim:$SESSION" --json 2>/dev/null)" \
    && [[ -n "$listing" ]] && break
  sleep 0.25
done
if [[ -z "$listing" ]]; then bad "sim 무응답"; tail -10 "$LOG"; exit 1; fi
ok "sim 기동"

step "4. 카탈로그에 슬롯 $SLOT 이 GRAB_E2E 로 보이는가"
python3 - "$listing" "$SLOT" <<'PY'
import json, sys
d = json.loads(sys.argv[1]); want = int(sys.argv[2])
hit = [m for m in d.get("motions", []) if m.get("ms_id") == want]
if hit and hit[0].get("name") == "GRAB_E2E":
    print(f"  [OK  ] 슬롯 {want} = {hit[0]['name']}")
else:
    print(f"  [FAIL] 슬롯 {want} 를 못 찾음: {[(m.get('ms_id'), m.get('name')) for m in d.get('motions', [])]}")
    sys.exit(1)
PY
[[ $? -eq 0 ]] || FAIL=1

step "5. 재생"
if out="$(timeout 30s phorce play "$SLOT" --target "sim:$SESSION" 2>&1)"; then
  ok "재생 성공"; printf '    %s\n' "$(echo "$out" | grep '결과:')"
else
  bad "재생 실패"; printf '    %s\n' "$out"
fi

step "6. 도달 불가 지점은 파일을 만들지 않아야 한다"
if "$PLANNER" plan --x 250 --y 0 --slot 12 --out "$WORK" >/dev/null 2>&1; then
  bad "도달 불가인데 성공으로 끝남"
else
  [[ -f "$WORK/motion_12.csv" ]] && bad "실패했는데 파일이 생성됨" || ok "실패 시 파일 미생성"
fi

printf '\n%s\n' "======================================"
[[ "$FAIL" -eq 0 ]] && printf '종단 검증 통과\n' || printf '종단 검증 실패\n'
exit "$FAIL"
