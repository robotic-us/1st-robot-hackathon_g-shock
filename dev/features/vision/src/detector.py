#!/usr/bin/env python3
"""펙보드 위 물체(흑/백 스텝모터)를 분할해 평면 mm 좌표와 자세를 뽑는다.

배경인 나무색을 HSV 로 정의하고 뒤집는 방식이라 물체 쪽 색을 가정하지 않는다.
타공 구멍은 배경보다 작으므로 morphological opening 으로 지워진다.
"""
import json

import cv2
import numpy as np

from pegboard import px_to_mm

DEFAULTS = {
    "hue_tol": 10,          # 보드 색상(Hue)에서 이만큼 벗어나면 물체
    "sat_ratio": 0.45,      # 보드 채도의 이 배율 미만이면 물체 (흰색/검정)
    "val_ratio": 0.35,      # 보드 명도의 이 배율 미만이면 물체 (검정) 또는 구멍
    "open_ksize": 15,       # 타공 구멍을 지울 만큼 커야 함 (구멍 지름보다 큼)
    "close_ksize": 25,      # 물체 내부 구멍 메우기
    "min_area_px": 4000,    # 이보다 작은 블롭은 버림
    "border_margin": 3,     # 화면 테두리에 닿는 블롭은 버림 (고정 브래킷 제거용).
                            # 대상 물체는 보드 안쪽에 있고 브래킷은 전부 화면 밖으로
                            # 잘려 나가므로, 이 한 줄로 깔끔하게 갈린다. 0 이면 비활성.
}


def estimate_board(hsv):
    """프레임마다 보드 자체의 HSV 를 추정한다.

    웹캠 오토 화이트밸런스가 수렴하는 값에 따라 보드 Hue 가 16~46 까지 움직이는
    것을 실측했다. 고정 임계 대신 매 프레임 보드 색을 기준으로 삼아야 안 깨진다.
    보드가 화면의 최대 면적을 차지한다는 전제를 쓴다.
    """
    # 통계량만 필요하므로 4배 서브샘플링해도 결과가 같고 훨씬 빠르다.
    sub = hsv[::4, ::4]
    h, s, v = sub[:, :, 0], sub[:, :, 1], sub[:, :, 2]
    chromatic = (s > 25).astype(np.uint8)              # 무채색 픽셀은 Hue 가 무의미
    hist = cv2.calcHist([h], [0], chromatic, [180], [0, 180])
    hist = cv2.GaussianBlur(hist, (1, 9), 0).ravel()
    h0 = int(np.argmax(hist))
    d = hue_dist(h, h0)
    near = (chromatic > 0) & (d <= 8)
    if near.sum() < 500:                               # 보드가 안 보이면 포기
        return h0, 70.0, 120.0
    return h0, float(np.median(s[near])), float(np.median(v[near]))


def hue_dist(h, h0):
    """0/180 을 잇는 순환 거리."""
    d = np.abs(h.astype(np.int16) - int(h0))
    return np.minimum(d, 180 - d)


def load_cfg(path="config/detect.json"):
    cfg = dict(DEFAULTS)
    try:
        cfg.update(json.load(open(path)))
    except FileNotFoundError:
        pass
    return cfg


def segment(bgr, cfg):
    """보드 배경을 추정해 뒤집는 방식으로 물체 마스크를 만든다."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h0, s0, v0 = estimate_board(hsv)
    wood = ((hue_dist(h, h0) <= cfg["hue_tol"])
            & (s > cfg["sat_ratio"] * s0)
            & (v > cfg["val_ratio"] * v0))
    m = (~wood).astype(np.uint8) * 255
    k_open = np.ones((cfg["open_ksize"],) * 2, np.uint8)
    k_close = np.ones((cfg["close_ksize"],) * 2, np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k_open)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_close)
    return m


def detect(bgr, cfg, board=None):
    """물체 목록을 반환. board 가 주어지면 mm 좌표/자세까지 채운다."""
    mask = segment(bgr, cfg)
    # 라벨 영상을 물체마다 훑지 않도록 외곽선만 뽑는다 (minAreaRect 결과는 동일).
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ih, iw = mask.shape[:2]
    margin = cfg.get("border_margin", 0)

    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < cfg["min_area_px"]:
            continue
        if margin:
            x, y, w, h = cv2.boundingRect(c)
            if (x <= margin or y <= margin
                    or x + w >= iw - margin or y + h >= ih - margin):
                continue
        m = cv2.moments(c)
        corners = cv2.boxPoints(cv2.minAreaRect(c))

        obj = {
            "area_px": int(area),
            "center_px": (m["m10"] / m["m00"], m["m01"] / m["m00"]),
            "corners_px": corners,
        }

        if board is not None:
            # 코너를 보드 평면으로 옮긴 뒤 거기서 크기/각도를 계산해야
            # 원근·왜곡이 반영된 실제 치수가 나온다.
            cmm = px_to_mm(corners, board)
            e1, e2 = cmm[1] - cmm[0], cmm[2] - cmm[1]
            l1, l2 = np.linalg.norm(e1), np.linalg.norm(e2)
            long_edge = e1 if l1 >= l2 else e2
            obj["center_mm"] = tuple(px_to_mm([obj["center_px"]], board)[0])
            obj["size_mm"] = (float(max(l1, l2)), float(min(l1, l2)))
            obj["theta_deg"] = float(
                np.degrees(np.arctan2(long_edge[1], long_edge[0])) % 180.0
            )
        out.append(obj)

    out.sort(key=lambda o: -o["area_px"])
    return out, mask


def draw(bgr, objs):
    vis = bgr.copy()
    for o in objs:
        cv2.drawContours(vis, [o["corners_px"].astype(int)], 0, (0, 255, 0), 2)
        cx, cy = map(int, o["center_px"])
        cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)
        if "center_mm" in o:
            x, y = o["center_mm"]
            # 물체 위에 겹쳐 쓰면 흰 물체에 묻히므로 박스 바깥 위쪽에 둔다
            top = o["corners_px"][:, 1].min()
            left = o["corners_px"][:, 0].min()
            cv2.putText(vis, f"({x:+.1f},{y:+.1f})mm {o['theta_deg']:.0f}deg",
                        (int(left), max(14, int(top) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return vis
