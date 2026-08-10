#!/usr/bin/env python3
"""packbot 메인 루프.

사용:
  python3 main.py                     # config.yaml 그대로 실행
  python3 main.py --no-window         # 디버그 창 없이 (헤드리스/ssh)
  python3 main.py --backend dummy     # 로봇 없이 비전만 시험
  python3 main.py --detector hsv      # 세그멘테이션 대신 HSV 임계값
"""

import argparse
import datetime
import pathlib
import sys
import time

import cv2
import yaml

ROOT = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from packbot.backends import build_backends
from packbot.camera import Camera
from packbot.detector_seg import build_detector
from packbot.fsm import PickCycle, State
from packbot.grasp import GraspChecker
from packbot.workspace import Workspace


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config" / "config.yaml"))
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--backend", choices=["dummy", "serial"])
    ap.add_argument("--detector", choices=["seg", "hsv"])
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.backend:
        cfg["motion"]["backend"] = args.backend
    if args.detector:
        cfg["detector"]["method"] = args.detector
    show = cfg["runtime"]["show_window"] and not args.no_window

    ws = Workspace(cfg["workspace"])
    detector = build_detector(cfg["detector"])
    motion, torque = build_backends(cfg)
    grasp = GraspChecker(cfg["grasp"], torque)
    cycle = PickCycle(ws, detector, motion, grasp, cfg)

    cap_dir = ROOT / "captures"
    loop_delay = float(cfg["runtime"]["loop_delay"])
    max_cycles = int(cfg["runtime"]["max_cycles"])

    print("packbot 시작 (Ctrl+C 로 종료)")
    try:
        with Camera(cfg["camera"]) as cam:
            while True:
                frame = cam.read()
                prev_state = cycle.state
                state = cycle.step(frame)

                # 파지 실패 순간의 프레임 저장 (원인 분석용)
                if (
                    cfg["runtime"]["save_failures"]
                    and prev_state == State.VERIFY
                    and state in (State.DETECT, State.IDLE)
                ):
                    ts = datetime.datetime.now().strftime("%m%d_%H%M%S")
                    cv2.imwrite(str(cap_dir / f"fail_{ts}.jpg"), frame)

                if show and cycle.last_warped is not None:
                    dbg = ws.draw_grid(cycle.last_warped, highlight=cycle.last_zone)
                    dbg = detector.draw(dbg, cycle.last_detections, cycle.last_target)
                    cv2.putText(
                        dbg, f"{state.value}", (8, dbg.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA,
                    )
                    cv2.imshow("packbot", dbg)
                    cv2.imshow("workspace", ws.draw_outline(frame))
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                if max_cycles and cycle.stats["picked"] >= max_cycles:
                    print(f"목표 {max_cycles}개 완료")
                    break

                time.sleep(loop_delay)
    except KeyboardInterrupt:
        print("\n종료 요청")
    finally:
        motion.stop()
        motion.close()
        torque.close()
        cv2.destroyAllWindows()
        print(f"통계: {cycle.stats}")


if __name__ == "__main__":
    main()
