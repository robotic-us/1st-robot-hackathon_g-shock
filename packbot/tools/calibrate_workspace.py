#!/usr/bin/env python3
"""작업공간 꼭짓점 캘리브레이션.

라이브 화면에서 작업공간(나무 바닥)의 네 꼭짓점을
  좌상 -> 우상 -> 우하 -> 좌하
순서로 클릭하면 config.yaml 에 저장된다.

  r : 다시 찍기      s : 저장      q : 저장 없이 종료
"""

import pathlib
import sys

import cv2
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from packbot.camera import Camera

CONFIG = ROOT / "config" / "config.yaml"
LABELS = ["좌상", "우상", "우하", "좌하"]

points = []


def on_mouse(event, x, y, flags, _):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append([x, y])
        print(f"  {LABELS[len(points)-1]}: ({x}, {y})")


def main():
    with open(CONFIG) as f:
        cfg = yaml.safe_load(f)

    print(__doc__)
    cv2.namedWindow("calibrate")
    cv2.setMouseCallback("calibrate", on_mouse)

    with Camera(cfg["camera"]) as cam:
        while True:
            frame = cam.read()
            disp = frame.copy()

            for i, (x, y) in enumerate(points):
                cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
                cv2.putText(disp, LABELS[i], (x + 10, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if len(points) >= 2:
                cv2.polylines(
                    disp,
                    [__import__("numpy").array(points).reshape(-1, 1, 2)],
                    len(points) == 4, (0, 255, 0), 2,
                )
            nxt = LABELS[len(points)] if len(points) < 4 else "완료 - s로 저장"
            cv2.putText(disp, f"클릭: {nxt}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("calibrate", disp)

            k = cv2.waitKey(30) & 0xFF
            if k == ord("r"):
                points.clear()
                print("다시 찍기")
            elif k == ord("s") and len(points) == 4:
                cfg["workspace"]["corners"] = [list(map(int, p)) for p in points]
                with open(CONFIG, "w") as f:
                    yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
                print(f"저장됨 -> {CONFIG}")
                break
            elif k == ord("q"):
                print("저장하지 않고 종료")
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
