"""나무색 바닥 위의 흰색 인형 검출.

색으로만 분리한다. 바닥이 나무색(채도 높음)이고 대상이 흰색(채도 낮음, 명도 높음)이라
HSV의 S/V 두 채널만으로 충분히 갈린다. 딥러닝 모델이 필요 없고 Orin에서 수 ms에 끝난다.

조명이 바뀌면 임계값이 흔들리므로 카메라 자동 노출은 꺼 두는 것을 전제로 한다.
"""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Detection:
    """검출된 물체 하나."""

    cx: float           # 펴진 이미지에서의 중심 x
    cy: float           # 펴진 이미지에서의 중심 y
    area: float         # 픽셀 면적
    angle: float        # 장축 방향 (도). 인형이 길쭉하므로 파지 각도 참고용
    length: float       # 장축 길이 (px)
    width: float        # 단축 길이 (px)
    contour: np.ndarray

    @property
    def center(self):
        return (self.cx, self.cy)

    @property
    def elongation(self):
        """장축/단축 비. 1에 가까우면 동그랗고 클수록 길쭉하다."""
        return self.length / self.width if self.width > 0 else 1.0


class WhiteObjectDetector:
    def __init__(self, cfg):
        self.s_max = int(cfg["s_max"])
        self.v_min = int(cfg["v_min"])
        self.min_area = float(cfg["min_area"])
        self.max_area = float(cfg["max_area"])
        self.pick_order = cfg.get("pick_order", "largest")

        ok = int(cfg["open_kernel"])
        ck = int(cfg["close_kernel"])
        self.k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ok, ok))
        self.k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ck, ck))

    def mask(self, warped_bgr):
        """흰색 영역 이진 마스크."""
        hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        # H는 무시한다. 흰색은 색상이 의미 없고, 채도가 낮으면 H가 불안정하다.
        lower = np.array([0, 0, self.v_min], dtype=np.uint8)
        upper = np.array([179, self.s_max, 255], dtype=np.uint8)
        m = cv2.inRange(hsv, lower, upper)

        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k_open)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.k_close)
        return m

    def detect_all(self, warped_bgr):
        """면적 조건을 통과한 모든 물체를 우선순위 순으로 반환."""
        m = self.mask(warped_bgr)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = []
        for c in contours:
            area = cv2.contourArea(c)
            if not (self.min_area <= area <= self.max_area):
                continue

            mo = cv2.moments(c)
            if mo["m00"] == 0:
                continue
            cx = mo["m10"] / mo["m00"]
            cy = mo["m01"] / mo["m00"]

            # 최소외접사각형으로 장축 방향과 길이를 얻는다
            (_, _), (w, h), ang = cv2.minAreaRect(c)
            if w < h:
                w, h = h, w
                ang += 90.0

            found.append(
                Detection(
                    cx=cx, cy=cy, area=area,
                    angle=ang % 180.0,
                    length=max(w, h), width=max(min(w, h), 1e-6),
                    contour=c,
                )
            )

        if self.pick_order == "center_first":
            mid = warped_bgr.shape[0] / 2.0
            found.sort(key=lambda d: (d.cx - mid) ** 2 + (d.cy - mid) ** 2)
        else:
            found.sort(key=lambda d: d.area, reverse=True)
        return found

    def detect(self, warped_bgr):
        """가장 먼저 집을 물체 하나. 없으면 None."""
        found = self.detect_all(warped_bgr)
        return found[0] if found else None

    @staticmethod
    def draw(warped_bgr, detections, primary=None):
        """검출 결과를 이미지에 그린다."""
        img = warped_bgr.copy()
        for d in detections:
            is_primary = primary is not None and d is primary
            color = (0, 0, 255) if is_primary else (255, 160, 0)
            cv2.drawContours(img, [d.contour], -1, color, 2)
            cv2.circle(img, (int(d.cx), int(d.cy)), 5, color, -1)

            if is_primary:
                # 장축 방향 표시 - 그리퍼 각도를 맞출 때 참고
                rad = np.deg2rad(d.angle)
                dx = np.cos(rad) * d.length / 2
                dy = np.sin(rad) * d.length / 2
                cv2.line(
                    img,
                    (int(d.cx - dx), int(d.cy - dy)),
                    (int(d.cx + dx), int(d.cy + dy)),
                    color, 2,
                )
        return img
