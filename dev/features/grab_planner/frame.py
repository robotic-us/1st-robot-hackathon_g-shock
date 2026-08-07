"""비전 좌표 → 로봇 월드 좌표 변환.

비전(`/home/phorce/vision_ws`)은 펙보드 평면 위의 mm 좌표를 준다. 그 원점은
아직 "임의의 구멍"이라 로봇 월드 원점(두 팔 J1 의 중점)과 정렬돼 있지 않다.
둘을 잇는 2D 강체변환(회전 θ + 평행이동)을 여기서 정의한다.

     [X_world]   [cos θ  -sin θ] [u_vision]   [tx]
     [Y_world] = [sin θ   cos θ] [v_vision] + [ty]

★ 아래 CALIBRATION 값은 **아직 측정되지 않은 자리표시자**다. 이 값이 맞기 전에는
  비전 좌표로 만든 모션을 실기에서 재생하면 안 된다. 측정 방법은
  dev/features/grab_planner/VISION_INTERFACE.md 참조.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path

CALIBRATION_PATH = Path(__file__).resolve().parent / "calibration.json"


@dataclass(frozen=True)
class BoardFrame:
    """펙보드(비전) 좌표계 → 로봇 월드 좌표계."""

    theta_deg: float = 0.0     # 펙보드 u축이 로봇 X축에 대해 돌아간 각도
    tx_mm: float = 0.0         # 펙보드 원점의 로봇 월드 X
    ty_mm: float = 0.0         # 펙보드 원점의 로봇 월드 Y
    object_top_z_mm: float = 120.0  # 물체를 집는 높이(월드 Z). 보드 높이+물체 절반
    calibrated: bool = False   # ★ 실측 전에는 False. True 여야 실기 사용 허용.

    def to_world(self, u_mm, v_mm):
        t = math.radians(self.theta_deg)
        x = math.cos(t) * u_mm - math.sin(t) * v_mm + self.tx_mm
        y = math.sin(t) * u_mm + math.cos(t) * v_mm + self.ty_mm
        return (x, y)

    def to_board(self, x_mm, y_mm):
        t = math.radians(self.theta_deg)
        dx, dy = x_mm - self.tx_mm, y_mm - self.ty_mm
        return (math.cos(t) * dx + math.sin(t) * dy,
                -math.sin(t) * dx + math.cos(t) * dy)


def load(path=CALIBRATION_PATH) -> BoardFrame:
    path = Path(path)
    if not path.exists():
        return BoardFrame()
    data = json.loads(path.read_text())
    known = {f: data[f] for f in BoardFrame.__dataclass_fields__ if f in data}
    return BoardFrame(**known)


def save(frame: BoardFrame, path=CALIBRATION_PATH):
    Path(path).write_text(json.dumps(asdict(frame), indent=2) + "\n")


def solve_from_pairs(pairs):
    """대응점 2쌍 이상으로 θ·tx·ty 를 최소제곱 추정한다.

    pairs = [((u, v), (X, Y)), ...] — 같은 물리점을 비전과 로봇 양쪽에서 읽은 값.
    로봇 쪽 값은 팔 끝을 그 점에 대고 '현재각 → EE' 로 읽으면 된다.
    """
    if len(pairs) < 2:
        raise ValueError("대응점이 2쌍 이상 필요합니다")
    n = len(pairs)
    mu_u = sum(p[0][0] for p in pairs) / n
    mu_v = sum(p[0][1] for p in pairs) / n
    mu_x = sum(p[1][0] for p in pairs) / n
    mu_y = sum(p[1][1] for p in pairs) / n
    num = sum((u - mu_u) * (Y - mu_y) - (v - mu_v) * (X - mu_x) for (u, v), (X, Y) in pairs)
    den = sum((u - mu_u) * (X - mu_x) + (v - mu_v) * (Y - mu_y) for (u, v), (X, Y) in pairs)
    theta = math.atan2(num, den)
    tx = mu_x - (math.cos(theta) * mu_u - math.sin(theta) * mu_v)
    ty = mu_y - (math.sin(theta) * mu_u + math.cos(theta) * mu_v)
    frame = BoardFrame(math.degrees(theta), tx, ty, calibrated=True)
    residuals = [math.dist(frame.to_world(u, v), (X, Y)) for (u, v), (X, Y) in pairs]
    return frame, max(residuals)
