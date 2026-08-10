#!/usr/bin/env python3
"""검출 튜닝 뷰어.

작업공간을 편 화면에 검출 결과를 실시간 표시한다.
  - seg 방식: FastSAM 마스크 + 흰색 비율 필터 결과 확인
  - hsv 방식: 트랙바로 s_max / v_min 실시간 조정 가능

  d : seg <-> hsv 전환      s : (hsv일 때) 임계값을 config에 저장      q : 종료
"""

import pathlib
import sys

import cv2
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from packbot.camera import Camera
from packbot.detector import WhiteObjectDetector
from packbot.detector_seg import build_detector
from packbot.workspace import Workspace

CONFIG = ROOT / "config" / "config.yaml"


def main():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)

    ws = Workspace(cfg["workspace"])
    method = cfg["detector"].get("method", "seg")
    det = build_detector(cfg["detector"])

    cv2.namedWindow("detect")
    cv2.namedWindow("mask")
    cv2.createTrackbar("s_max", "mask", cfg["detector"]["s_max"], 255, lambda v: None)
    cv2.createTrackbar("v_min", "mask", cfg["detector"]["v_min"], 255, lambda v: None)

    print(__doc__)
    with Camera(cfg["camera"]) as cam:
        while True:
            frame = cam.read()
            warped = ws.warp(frame)

            if method == "hsv":
                det.s_max = cv2.getTrackbarPos("s_max", "mask")
                det.v_min = cv2.getTrackbarPos("v_min", "mask")

            dets = det.detect_all(warped)
            primary = dets[0] if dets else None
            zone = ws.zone_of(primary.center) if primary else None

            disp = ws.draw_grid(warped, highlight=zone)
            disp = det.draw(disp, dets, primary)
            cv2.putText(disp, f"[{method}] objects: {len(dets)}"
                        + (f"  zone: {zone}" if zone is not None else ""),
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow("detect", disp)
            cv2.imshow("mask", det.mask(warped))

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("d"):
                method = "hsv" if method == "seg" else "seg"
                cfg["detector"]["method"] = method
                det = build_detector(cfg["detector"])
            elif k == ord("s") and method == "hsv":
                cfg["detector"]["s_max"] = int(det.s_max)
                cfg["detector"]["v_min"] = int(det.v_min)
                with open(CONFIG, "w") as f:
                    yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
                print(f"임계값 저장됨: s_max={det.s_max} v_min={det.v_min}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
