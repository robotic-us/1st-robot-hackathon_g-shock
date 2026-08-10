#!/usr/bin/env python3
"""빈손 토크 기준치 측정.

로봇이 "물건 없이" 들어올린 자세를 취한 상태에서 실행하면
리프트 관절 토크를 몇 초간 샘플링해 평균을 config.yaml 에 저장한다.

이후에는 물건을 실제로 잡고 다시 실행해 보면 (저장은 안 함)
빈손 대비 차이가 출력되므로 delta_threshold 를 정하는 데 쓸 수 있다.
"""

import argparse
import pathlib
import statistics
import sys
import time

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from packbot.backends import build_backends

CONFIG = ROOT / "config" / "config.yaml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=3.0, help="샘플링 시간")
    ap.add_argument("--save", action="store_true",
                    help="측정 평균을 baseline_torque 로 저장")
    args = ap.parse_args()

    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)

    joint = cfg["grasp"]["lift_joint"]
    hz = cfg["grasp"]["sample_hz"]
    _, torque = build_backends(cfg)

    print(f"관절 {joint} 토크를 {args.seconds}초간 샘플링...")
    vals = []
    end = time.time() + args.seconds
    while time.time() < end:
        r = torque.read()
        if joint in r:
            vals.append(r[joint])
        time.sleep(1.0 / hz)

    if not vals:
        print("토크를 읽지 못했습니다. 백엔드/연결을 확인하세요.")
        return

    mean = statistics.mean(vals)
    std = statistics.pstdev(vals)
    print(f"샘플 {len(vals)}개  평균 {mean:.4f}  표준편차 {std:.4f}  "
          f"범위 [{min(vals):.4f}, {max(vals):.4f}]")

    base = cfg["grasp"].get("baseline_torque")
    if base is not None:
        print(f"현재 baseline({base:.4f}) 대비 차이: {mean - base:+.4f}")

    if args.save:
        cfg["grasp"]["baseline_torque"] = round(mean, 4)
        with open(CONFIG, "w") as f:
            yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
        print(f"baseline_torque={mean:.4f} 저장됨 -> {CONFIG}")
    else:
        print("(저장하려면 --save 옵션)")

    torque.close()


if __name__ == "__main__":
    main()
