"""작업공간 정의와 16구역 격자 판정.

카메라가 작업대를 비스듬히 내려다보더라도, 네 꼭짓점을 지정해 두면
호모그래피로 "위에서 똑바로 내려다본" 정사각 이미지로 펼 수 있다.
격자 판정은 이 펴진 이미지 위에서 하기 때문에 원근 왜곡의 영향을 받지 않는다.
"""

import cv2
import numpy as np


class Workspace:
    def __init__(self, cfg):
        corners = cfg.get("corners")
        if not corners:
            raise ValueError(
                "작업공간 꼭짓점이 설정되지 않았습니다.\n"
                "  python3 tools/calibrate_workspace.py 를 먼저 실행하세요."
            )
        if len(corners) != 4:
            raise ValueError(f"꼭짓점은 4개여야 합니다 (현재 {len(corners)}개)")

        self.size = int(cfg["warp_size"])
        self.rows = int(cfg["grid_rows"])
        self.cols = int(cfg["grid_cols"])

        # 좌상 -> 우상 -> 우하 -> 좌하 순서로 정사각형에 대응시킨다
        self.src = np.array(corners, dtype=np.float32)
        s = float(self.size)
        self.dst = np.array([[0, 0], [s, 0], [s, s], [0, s]], dtype=np.float32)

        self.H = cv2.getPerspectiveTransform(self.src, self.dst)
        self.H_inv = np.linalg.inv(self.H)

    @property
    def zone_count(self):
        return self.rows * self.cols

    def warp(self, frame):
        """원본 프레임 -> 위에서 내려다본 정사각 작업공간 이미지."""
        return cv2.warpPerspective(frame, self.H, (self.size, self.size))

    def zone_of(self, pt):
        """펴진 이미지 좌표 (x, y) -> 구역 번호. 작업공간 밖이면 None.

        번호는 좌상단 0에서 시작해 오른쪽으로 증가하고, 행이 끝나면 다음 행으로 넘어간다.
        """
        x, y = pt
        if not (0 <= x < self.size and 0 <= y < self.size):
            return None
        col = min(int(x * self.cols / self.size), self.cols - 1)
        row = min(int(y * self.rows / self.size), self.rows - 1)
        return row * self.cols + col

    def zone_rect(self, zone):
        """구역 번호 -> 펴진 이미지에서의 (x1, y1, x2, y2)."""
        row, col = divmod(zone, self.cols)
        cw = self.size / self.cols
        ch = self.size / self.rows
        return (
            int(col * cw),
            int(row * ch),
            int((col + 1) * cw),
            int((row + 1) * ch),
        )

    def to_original(self, pt):
        """펴진 이미지 좌표 -> 원본 프레임 좌표 (디버그 표시용)."""
        v = np.array([[[float(pt[0]), float(pt[1])]]], dtype=np.float32)
        out = cv2.perspectiveTransform(v, self.H_inv)
        return tuple(out[0][0])

    def draw_grid(self, warped, highlight=None):
        """펴진 이미지 위에 격자와 구역 번호를 그린다."""
        img = warped.copy()
        cw = self.size / self.cols
        ch = self.size / self.rows

        if highlight is not None:
            x1, y1, x2, y2 = self.zone_rect(highlight)
            overlay = img.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 200, 255), -1)
            cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)

        for i in range(1, self.cols):
            x = int(i * cw)
            cv2.line(img, (x, 0), (x, self.size), (0, 255, 0), 1)
        for i in range(1, self.rows):
            y = int(i * ch)
            cv2.line(img, (0, y), (self.size, y), (0, 255, 0), 1)

        for z in range(self.zone_count):
            x1, y1, _, _ = self.zone_rect(z)
            cv2.putText(
                img, str(z), (x1 + 5, y1 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA,
            )
        return img

    def draw_outline(self, frame):
        """원본 프레임 위에 작업공간 테두리를 그린다."""
        img = frame.copy()
        pts = self.src.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        for i, (x, y) in enumerate(self.src.astype(int)):
            cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(
                img, str(i), (x + 8, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA,
            )
        return img
