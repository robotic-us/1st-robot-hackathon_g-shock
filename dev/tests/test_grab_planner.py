#!/usr/bin/env python3
"""grab_planner 자체 검증 — 외부 의존성 없이 python3 만으로 돈다.

  python3 dev/tests/test_grab_planner.py
"""

from __future__ import annotations

import csv
import io
import math
import sys
import tempfile
from pathlib import Path

FEATURE = Path(__file__).resolve().parents[1] / "features" / "grab_planner"
CATALOG = Path(__file__).resolve().parents[1] / "catalog"
sys.path.insert(0, str(FEATURE))

import frame as frame_mod                      # noqa: E402
from csvgen import build_csv_rows, read_motion, write_motion   # noqa: E402
from grab import GrabParams, Unreachable, plan_grab, reachable  # noqa: E402
from kinematics import Kinematics               # noqa: E402

PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(f"  [{'OK  ' if cond else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


def test_kinematics():
    print("\n== 기구학 ==")
    k = Kinematics()
    # README 에 적힌 준비자세 EE 위치
    for arm, expect in zip(k.arms, [(-55.0, 0.0, 210.6), (55.0, 0.0, 210.6)]):
        ee = k.ready_world_ee(arm)
        err = max(abs(a - b) for a, b in zip(ee, expect))
        check(f"{arm.name} 준비자세 EE", err < 0.05, f"오차 {err:.4f}mm")

    # 월드 → 모터각 → 월드 왕복
    worst = 0.0
    for arm, sx in zip(k.arms, (-1, 1)):
        for x in (-40, -20, 0, 20, 40):
            for y in (-40, 0, 60, 120):
                for z in (90, 120, 160):
                    target = (x + sx * 30, y, z)
                    try:
                        motor = k.world_ik_to_motor(target, arm)
                    except ValueError:
                        continue
                    back = k.local_to_world(k.forward(k.motor_to_geometric(motor, arm)), arm)
                    worst = max(worst, max(abs(a - b) for a, b in zip(target, back)))
    check("IK→FK 왕복 오차 < 1e-6 mm", worst < 1e-6, f"최대 {worst:.2e}mm")


def test_csv_format():
    print("\n== CSV 포맷 (기존 슬롯 재생성 후 바이트 비교) ==")
    k = Kinematics()
    # motion_05 는 구버전 motion_builder 가 만든 레거시라 소수 자리수가 다르다(128.7 vs 128.700).
    # 현재 파이프라인이 만든 파일들만 대조한다.
    for name in ("motion_02.csv", "motion_03.csv", "motion_49.csv", "motion_50.csv"):
        path = CATALOG / name
        if not path.exists():
            check(f"{name} 존재", False, "카탈로그에 없음")
            continue
        slot, ms_name, wps = read_motion(path, k)
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerows(build_csv_rows(slot, ms_name, wps, k))
        check(f"{name} 바이트 일치", buf.getvalue().encode() == path.read_bytes())


def test_grab_plan():
    print("\n== grab 플랜 ==")
    k = Kinematics()
    p = GrabParams()
    wps = plan_grab((0.0, 0.0), p, k)
    check("웨이포인트 8개", len(wps) == 8, f"{len(wps)}개")
    check("시작이 준비자세", wps[0].label == "ready")
    check("끝이 준비자세", wps[-1].label == "ready")
    check("최대 20개 이하", len(wps) <= 20)

    # 집는 단계에서 두 EE 간격이 close_half_gap 의 2배여야 한다
    grip = next(w for w in wps if w.label == "close_grip")
    a, b = list(grip.ee_world.values())
    check("집은 간격 = 2×close_half_gap",
          abs(math.dist(a, b) - 2 * p.close_half_gap) < 1e-6,
          f"{math.dist(a, b):.1f}mm")

    # 벌린 단계가 집은 단계보다 넓어야 한다
    openw = next(w for w in wps if w.label == "approach_open")
    oa, ob = list(openw.ee_world.values())
    check("벌린 간격 > 집은 간격", math.dist(oa, ob) > math.dist(a, b))

    # 물체 중심이 두 EE 의 중점이어야 한다
    for target in [(0.0, 0.0), (30.0, 50.0), (-40.0, 80.0)]:
        w = next(x for x in plan_grab(target, p, k) if x.label == "close_grip")
        ea, eb = list(w.ee_world.values())
        mid = ((ea[0] + eb[0]) / 2, (ea[1] + eb[1]) / 2)
        check(f"중심 일치 {target}", math.dist(mid, target) < 1e-6)

    # 모든 각도가 ±360 안
    ok = all(-360 <= d <= 360 for w in wps for d in w.angles.values())
    check("모든 모터각 ±360° 이내", ok)

    # 도달 불가 판정
    check("먼 지점은 Unreachable", not reachable((300.0, 0.0), p, k))
    try:
        plan_grab((300.0, 0.0), p, k)
        check("Unreachable 예외 발생", False)
    except Unreachable as exc:
        check("Unreachable 예외 발생", True, str(exc)[:60])


def test_write_read_roundtrip():
    print("\n== 생성 파일 왕복 ==")
    k = Kinematics()
    wps = plan_grab((10.0, 30.0), GrabParams(), k)
    with tempfile.TemporaryDirectory() as tmp:
        csv_path, memo_path, digest = write_motion(tmp, 11, "GRAB_TEST", wps, k)
        import hashlib, json
        check("memo 해시 = CSV 해시",
              json.loads(memo_path.read_text())["motion_sha256"]
              == hashlib.sha256(csv_path.read_bytes()).hexdigest())
        slot, name, back = read_motion(csv_path, k)
        check("슬롯/이름 왕복", slot == 11 and name == "GRAB_TEST")
        check("웨이포인트 수 왕복", len(back) == len(wps))
        worst = max(abs(back[i].angles[md] - wps[i].angles[md])
                    for i in range(len(wps)) for md in k.active_md_ids)
        check("각도 왕복 오차 < 0.001°", worst < 1e-3, f"최대 {worst:.6f}°")
        check("슬롯 범위 밖 거부", _raises(lambda: write_motion(tmp, 51, "X", wps, k)))


def test_frame():
    print("\n== 비전 좌표 변환 ==")
    truth = frame_mod.BoardFrame(theta_deg=17.0, tx_mm=-12.0, ty_mm=45.0, calibrated=True)
    pairs = [((u, v), truth.to_world(u, v)) for u, v in [(0, 0), (100, 0), (0, 80), (60, 60)]]
    solved, residual = frame_mod.solve_from_pairs(pairs)
    check("θ 복원", abs(solved.theta_deg - truth.theta_deg) < 1e-6, f"{solved.theta_deg:.6f}°")
    check("tx/ty 복원",
          abs(solved.tx_mm - truth.tx_mm) < 1e-6 and abs(solved.ty_mm - truth.ty_mm) < 1e-6)
    check("잔차 < 1e-6mm", residual < 1e-6, f"{residual:.2e}mm")
    check("대응점 1쌍은 거부", _raises(lambda: frame_mod.solve_from_pairs(pairs[:1])))
    check("기본 캘리브레이션은 미보정 표시", frame_mod.BoardFrame().calibrated is False)


def _raises(fn):
    try:
        fn()
        return False
    except Exception:
        return True


def main():
    test_kinematics()
    test_csv_format()
    test_grab_plan()
    test_write_read_roundtrip()
    test_frame()
    print(f"\n{'='*54}\n통과 {len(PASS)} · 실패 {len(FAIL)}")
    if FAIL:
        print("실패 항목: " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
