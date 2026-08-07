#!/usr/bin/env python3
"""웹캠 실시간 검출. 물체의 평면 mm 좌표와 자세를 화면에 겹쳐 보여준다.

키:  q 종료   s 현재 프레임 저장(data/frames)   m 마스크 토글
"""
import argparse
import json
import os
import time
from collections import deque

import cv2
import numpy as np

from detector import detect, draw, load_cfg


def open_cam(index, width, height, warmup=2.5):
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"카메라 {index} 를 열 수 없습니다")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 지연 최소화

    # 오픈 직후에는 오토 화이트밸런스/노출이 아직 수렴하지 않아 색이 크게 다르다.
    # (실측: 보드 Hue 가 수렴 전 22 -> 수렴 후 43) 반드시 버리고 시작해야 한다.
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < warmup:
        cap.read()
    return cap


def draw_hud(vis, objs, proc, fps_hist, hist):
    """성능을 눈으로 판단할 수 있게 지연시간/fps/지터를 좌상단에 겹쳐 쓴다."""
    lines = [
        (f"proc {np.mean(proc):5.2f} ms   p95 {np.percentile(proc, 95):5.2f} ms",
         (0, 255, 255)),
        (f"fps  {np.mean(fps_hist):5.1f}   obj {len(objs)}", (0, 255, 255)),
    ]
    if objs and "center_mm" in objs[0]:
        x, y = objs[0]["center_mm"]
        L, W = objs[0]["size_mm"]
        lines.append((f"pos  ({x:+7.2f}, {y:+7.2f}) mm  th {objs[0]['theta_deg']:5.1f}",
                      (0, 255, 0)))
        lines.append((f"size {L:5.1f} x {W:5.1f} mm", (0, 255, 0)))
    if len(hist) >= 10:
        a = np.array(hist)
        # 지터 = 최근 위치들의 표준편차. 물체가 정지해 있을 때의 재현성 지표.
        lines.append((f"jitter {a[:, 0].std():.3f} / {a[:, 1].std():.3f} mm "
                      f"(n={len(a)})", (255, 200, 0)))

    # 글자가 배경에 묻히지 않게 반투명 판 위에 올린다
    box = vis[0:18 + 20 * len(lines), 0:330]
    cv2.addWeighted(box, 0.35, np.zeros_like(box), 0.65, 0, box)
    for i, (text, color) in enumerate(lines):
        cv2.putText(vis, text, (8, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--board", default="config/pegboard.json")
    ap.add_argument("--warmup", type=float, default=2.5,
                    help="AWB/AE 수렴 대기 (초)")
    ap.add_argument("--headless", action="store_true", help="창 없이 수치만 출력")
    ap.add_argument("--seconds", type=float, default=0, help="0이면 무한")
    args = ap.parse_args()

    cfg = load_cfg()
    board = json.load(open(args.board)) if os.path.exists(args.board) else None
    if board is None:
        print("경고: 보드 캘리브레이션이 없어 픽셀 좌표만 출력합니다")

    cap = open_cam(args.device, args.width, args.height, args.warmup)
    print(f"해상도 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}  "
          f"{cap.get(cv2.CAP_PROP_FPS):.0f}fps")

    proc, show_mask, t0, n = deque(maxlen=60), False, time.perf_counter(), 0
    hist = deque(maxlen=60)          # 최근 위치 -> 지터(재현성) 표시용
    fps_hist = deque(maxlen=60)
    t_prev = t0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("프레임 획득 실패")
                break

            t = time.perf_counter()
            objs, mask = detect(frame, cfg, board)
            proc.append((time.perf_counter() - t) * 1000)
            n += 1

            if args.headless:
                if n % 30 == 0:
                    ms = np.mean(proc)
                    line = " | ".join(
                        f"({o['center_mm'][0]:.0f},{o['center_mm'][1]:.0f})mm "
                        f"{o['theta_deg']:.0f}d" if "center_mm" in o
                        else f"({o['center_px'][0]:.0f},{o['center_px'][1]:.0f})px"
                        for o in objs[:4]
                    )
                    print(f"[{n:5d}] {len(objs)}개  처리 {ms:5.2f}ms  "
                          f"{n / (time.perf_counter() - t0):5.1f}fps  {line}")
            else:
                now = time.perf_counter()
                fps_hist.append(1.0 / max(now - t_prev, 1e-6))
                t_prev = now
                if objs and "center_mm" in objs[0]:
                    hist.append(objs[0]["center_mm"])

                vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask \
                    else draw(frame, objs)
                draw_hud(vis, objs, proc, fps_hist, hist)
                cv2.imshow("pegboard vision", vis)
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("m"):
                    show_mask = not show_mask
                if k == ord("s"):
                    p = f"data/frames/snap_{n:05d}.jpg"
                    cv2.imwrite(p, frame)
                    print("저장:", p)

            if args.seconds and time.perf_counter() - t0 > args.seconds:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if proc:
            print(f"\n처리시간 평균 {np.mean(proc):.2f} ms, "
                  f"p95 {np.percentile(proc, 95):.2f} ms, "
                  f"전체 {n / (time.perf_counter() - t0):.1f} fps")


if __name__ == "__main__":
    main()
