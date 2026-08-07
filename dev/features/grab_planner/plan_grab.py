#!/usr/bin/env python3
"""비전 물체 위치 → 양팔 grab 모션 생성/조회 CLI.

  ./plan_grab.py reach                      집을 수 있는 영역을 그린다
  ./plan_grab.py plan --x 0 --y 40 --slot 10 --name GRAB_AUTO
  ./plan_grab.py plan --board 12.5,-8 --slot 10        (비전 좌표 입력)
  ./plan_grab.py grid --slots 10-45 --spacing 20       슬롯 그리드 + 조회표 생성
  ./plan_grab.py nearest --board 12.5,-8               비전 좌표 → 재생할 슬롯

★ 이 로봇은 미리 SD 카드에 적재된 슬롯만 재생할 수 있다. 여기서 만든 CSV 를
  실기에서 돌리려면 SD 에 설치하고 PCM 을 재시작해야 한다(README 참조).
  실시간 데모는 `grid` 로 미리 여러 지점을 깔아두고 `nearest` 로 고르는 방식이다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import frame as frame_mod
from csvgen import write_motion
from grab import GrabParams, Unreachable, grip_clearance, plan_grab, reach_map, reachable
from kinematics import Kinematics

OUT_ROOT = Path(__file__).resolve().parent / "out"
GRID_PATH = Path(__file__).resolve().parent / "slot_grid.json"


def _params(args) -> GrabParams:
    base = GrabParams()
    return GrabParams(
        grasp_z=args.grasp_z if args.grasp_z is not None else base.grasp_z,
        open_half_gap=args.open if args.open is not None else base.open_half_gap,
        close_half_gap=args.close if args.close is not None else base.close_half_gap,
        place_xy=tuple(args.place) if args.place else base.place_xy,
        segment_ms=args.segment_ms,
    )


def _target(args, fr: frame_mod.BoardFrame):
    if args.board is not None:
        u, v = args.board
        x, y = fr.to_world(u, v)
        if not fr.calibrated:
            print("  [WARN] calibration.json 이 없거나 calibrated=false 입니다 —", file=sys.stderr)
            print("         비전→로봇 변환이 자리표시자(항등변환)라 좌표를 믿을 수 없습니다.", file=sys.stderr)
        return x, y
    return args.x, args.y


def cmd_reach(args):
    kin = Kinematics()
    params = _params(args)
    print(f"집는 높이 Z={params.grasp_z}mm, 집은 반간격 {params.close_half_gap}mm, "
          f"벌린 반간격 {params.open_half_gap}mm")
    print("(# = 8개 웨이포인트 전부 도달 가능, . = 불가)\n")
    xs = list(range(-120, 121, 10))
    for y in range(200, -81, -10):
        row = "".join("#" if reachable((x, y), params, kin, args.grip_angle) else "." for x in xs)
        print(f"  Y={y:+4d} |{row}|")
    print("        " + "".join("^" if x == 0 else " " for x in xs) + "   ^ = X 0")
    print(f"        X {xs[0]} … {xs[-1]} mm (10mm 간격)")
    pts = reach_map(params, kin, grip_angle_deg=args.grip_angle)
    if pts:
        print(f"\n도달 가능 격자점 {len(pts)}개 — "
              f"X {min(p[0] for p in pts)}…{max(p[0] for p in pts)}mm, "
              f"Y {min(p[1] for p in pts)}…{max(p[1] for p in pts)}mm")


def cmd_plan(args):
    kin = Kinematics()
    fr = frame_mod.load()
    params = _params(args)
    x, y = _target(args, fr)
    try:
        waypoints = plan_grab((x, y), params, kin, args.grip_angle)
    except Unreachable as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1
    print(f"물체 위치 (X={x:.1f}, Y={y:.1f}) mm · 집는 축 {args.grip_angle:.1f}° · "
          f"웨이포인트 {len(waypoints)}개")
    if args.object_width is not None:
        squeeze, note = grip_clearance(args.object_width, params)
        print(f"  {'[OK]  ' if squeeze > 0 else '[WARN]'} {note}")
    for i, wp in enumerate(waypoints, 1):
        ee = "  ".join(
            f"{n.split()[0]}{n.split()[1]}=({p[0]:6.1f},{p[1]:6.1f},{p[2]:6.1f})"
            for n, p in wp.ee_world.items()
        )
        print(f"  wp{i} {wp.label:14s} {wp.duration_ms:5d}ms  {ee}")
    if args.dry_run:
        print("\n(--dry-run — 파일을 쓰지 않았습니다)")
        return 0
    out_dir = Path(args.out) if args.out else OUT_ROOT / args.name
    csv_path, memo_path, digest = write_motion(out_dir, args.slot, args.name, waypoints, kin)
    print(f"\n생성: {csv_path}\n      {memo_path}\n      sha256 {digest}")
    print(f"\nSD 설치: {Path(__file__).resolve().parents[3]}/main/motion_builder/install_to_sd.sh "
          f"{out_dir} <SD마운트경로>")
    return 0


def cmd_grid(args):
    kin = Kinematics()
    params = _params(args)
    lo, hi = (int(v) for v in args.slots.split("-"))
    slots = list(range(lo, hi + 1))
    # 슬롯 수에 맞는 균일 격자 간격을 고른다. 촘촘한 격자를 솎아내면 2D 균일성이
    # 깨지므로, 간격 자체를 키워서 개수를 맞춘다.
    spacing = args.spacing
    pts = reach_map(params, kin, x_range=tuple(args.x_range), y_range=tuple(args.y_range),
                    step=spacing, grip_angle_deg=args.grip_angle)
    while len(pts) > len(slots) and spacing < 200:
        spacing += 1
        pts = reach_map(params, kin, x_range=tuple(args.x_range), y_range=tuple(args.y_range),
                        step=spacing, grip_angle_deg=args.grip_angle)
    if not pts:
        print("[FAIL] 지정한 범위에 도달 가능한 지점이 없습니다", file=sys.stderr)
        return 1
    if spacing != args.spacing:
        print(f"[INFO] 슬롯 {len(slots)}개에 맞춰 격자 간격을 "
              f"{args.spacing}mm → {spacing}mm 로 넓혔습니다.")
    entries = []
    out_root = Path(args.out) if args.out else OUT_ROOT / "grid"
    for slot, (x, y) in zip(slots, pts):
        waypoints = plan_grab((x, y), params, kin, args.grip_angle)
        name = f"{args.name}_{slot:02d}"
        csv_path, _memo, digest = write_motion(out_root / name, slot, name, waypoints, kin)
        entries.append({"slot": slot, "x": x, "y": y, "name": name,
                        "csv": str(csv_path), "sha256": digest})
    GRID_PATH.write_text(json.dumps(
        {"spacing_mm": spacing, "grasp_z_mm": params.grasp_z, "grip_angle_deg": args.grip_angle,
         "close_half_gap_mm": params.close_half_gap, "entries": entries}, indent=2) + "\n")
    # 덮으려는 영역 안의 임의 점에서 가장 가까운 격자점까지의 실제 최대 거리.
    worst = 0.0
    gx = args.x_range[0]
    while gx <= args.x_range[1]:
        gy = args.y_range[0]
        while gy <= args.y_range[1]:
            if reachable((gx, gy), params, kin, args.grip_angle):
                worst = max(worst, min(math.dist((gx, gy), (e["x"], e["y"])) for e in entries))
            gy += 5
        gx += 5
    print(f"슬롯 {len(entries)}개 생성 → {out_root}")
    print(f"조회표: {GRID_PATH}")
    print(f"실측 최대 오차 {worst:.1f}mm — 물체 크기의 절반보다 작아야 집을 수 있습니다.")
    print("\n각 슬롯 폴더를 install_to_sd.sh 로 SD 에 설치한 뒤 PCM 을 재시작하세요.")
    return 0


def cmd_nearest(args):
    if not GRID_PATH.exists():
        print(f"[FAIL] {GRID_PATH} 가 없습니다 — 먼저 `grid` 를 실행하세요", file=sys.stderr)
        return 1
    grid = json.loads(GRID_PATH.read_text())
    fr = frame_mod.load()
    x, y = _target(args, fr)
    best = min(grid["entries"], key=lambda e: math.dist((e["x"], e["y"]), (x, y)))
    err = math.dist((best["x"], best["y"]), (x, y))
    print(f"물체 (X={x:.1f}, Y={y:.1f}) → 슬롯 {best['slot']} "
          f"({best['name']}, 격자점 {best['x']},{best['y']}) 오차 {err:.1f}mm")
    if err > grid["spacing_mm"]:
        print(f"  [WARN] 오차가 격자 간격({grid['spacing_mm']}mm)보다 큽니다 — "
              f"물체가 격자 밖일 수 있습니다.")
    print(f"\n재생: phorce play {best['slot']} --domain-id 21")
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp, with_target=True):
        sp.add_argument("--grasp-z", type=float, default=None, help="집는 높이 mm (기본 120)")
        sp.add_argument("--open", type=float, default=None, help="벌린 반간격 mm (기본 60)")
        sp.add_argument("--close", type=float, default=None, help="집은 반간격 mm (기본 30)")
        sp.add_argument("--place", type=float, nargs=2, metavar=("X", "Y"), default=None,
                        help="놓을 자리 mm (기본 0 110)")
        sp.add_argument("--segment-ms", type=int, default=1000, help="구간 길이 ms")
        sp.add_argument("--grip-angle", type=float, default=0.0,
                        help="집는 축 방향 도 (0=X축). 비전 자세각의 수직 방향을 넣는다")
        sp.add_argument("--object-width", type=float, default=None,
                        help="집는 축 방향 물체 폭 mm — 실제로 물리는지 검사")
        if with_target:
            sp.add_argument("--x", type=float, default=0.0, help="물체 월드 X mm")
            sp.add_argument("--y", type=float, default=0.0, help="물체 월드 Y mm")
            sp.add_argument("--board", type=lambda s: [float(v) for v in s.split(",")],
                            default=None, metavar="U,V", help="비전 보드 좌표 mm")

    sp = sub.add_parser("reach", help="집을 수 있는 영역 표시")
    common(sp, with_target=False)
    sp.set_defaults(func=cmd_reach)

    sp = sub.add_parser("plan", help="한 지점의 grab 모션 생성")
    common(sp)
    sp.add_argument("--slot", type=int, required=True, help="슬롯 ID 1~50")
    sp.add_argument("--name", default="GRAB_AUTO", help="MS Name")
    sp.add_argument("--out", default=None, help="출력 폴더")
    sp.add_argument("--dry-run", action="store_true", help="파일을 쓰지 않고 미리보기만")
    sp.set_defaults(func=cmd_plan)

    sp = sub.add_parser("grid", help="슬롯 그리드 일괄 생성")
    common(sp, with_target=False)
    sp.add_argument("--slots", default="10-45", help="쓸 슬롯 범위, 예: 10-45")
    sp.add_argument("--spacing", type=int, default=20, help="격자 간격 mm")
    sp.add_argument("--x-range", type=int, nargs=2, default=[-60, 60])
    sp.add_argument("--y-range", type=int, nargs=2, default=[-40, 140])
    sp.add_argument("--name", default="GRABGRID")
    sp.add_argument("--out", default=None)
    sp.set_defaults(func=cmd_grid)

    sp = sub.add_parser("nearest", help="비전 좌표 → 재생할 슬롯")
    common(sp)
    sp.set_defaults(func=cmd_nearest)

    args = p.parse_args()
    return args.func(args) or 0


if __name__ == "__main__":
    sys.exit(main())
