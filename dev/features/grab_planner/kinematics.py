"""양팔 3자유도 기구학 — motion_builder.py 의 계산을 GUI 없이 쓸 수 있게 뽑은 것.

좌표계 (motion_builder README 와 동일):
  · 월드 원점 = 두 팔 J1 의 중점, Z = 지면 기준 절대 높이
  · ARM A J1 지면점 = (-간격/2, 0, 0), ARM B = (+간격/2, 0, 0), 두 팔은 서로를 향한다
  · 각 팔은 yaw(J1) · shoulder(J2) · elbow(J3) 3자유도. 손목도 그리퍼도 없다.

부호/영점은 profile.json 의 축별 sign·offset_deg 를 그대로 따른다.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

PROFILE_PATH = Path(__file__).resolve().parents[3] / "main" / "motion_builder" / "profile.json"


@dataclass(frozen=True)
class Arm:
    """한쪽 팔의 배치와 모터 매핑."""

    name: str
    index: int          # 0 = ARM A, 1 = ARM B
    md_ids: tuple       # (J1, J2, J3) 에 해당하는 MD 번호
    sign: tuple
    offset_deg: tuple


class Kinematics:
    def __init__(self, profile_path: Path = PROFILE_PATH):
        profile = json.loads(Path(profile_path).read_text())
        self.profile = profile
        self.active_md_ids = tuple(profile["active_md_ids"])
        kin = profile["kinematics"]
        self.kin = kin
        self.l1 = kin["link_1_mm"]
        self.l2 = kin["link_2_to_tip_mm"]
        self.base_height = kin["base_height_mm"]
        self.separation = kin["base_separation_mm"]
        self.arms = tuple(
            Arm(name, i, tuple(cfg["md_ids"]), tuple(cfg["sign"]), tuple(cfg["offset_deg"]))
            for i, (name, cfg) in enumerate(kin["groups"].items())
        )
        self.ready_motor_deg = kin["button1_ready_motor_deg"]

    # ── 좌표 변환 ──────────────────────────────────────────────────────────
    def world_to_local(self, point, arm: Arm, separation=None):
        sep = self.separation if separation is None else separation
        x, y, z = point
        return (x + sep / 2, y, z) if arm.index == 0 else (sep / 2 - x, -y, z)

    def local_to_world(self, point, arm: Arm, separation=None):
        sep = self.separation if separation is None else separation
        x, y, z = point
        return (-sep / 2 + x, y, z) if arm.index == 0 else (sep / 2 - x, -y, z)

    # ── 기구학 ────────────────────────────────────────────────────────────
    def forward(self, geometric_deg):
        """로컬 관절각(도) → 로컬 EE 좌표(mm)."""
        yaw, shoulder, elbow = map(math.radians, geometric_deg)
        r1 = self.l1 * math.cos(shoulder)
        z1 = self.base_height + self.l1 * math.sin(shoulder)
        r2 = r1 + self.l2 * math.cos(shoulder + elbow)
        z2 = z1 + self.l2 * math.sin(shoulder + elbow)
        return (r2 * math.cos(yaw), r2 * math.sin(yaw), z2)

    def inverse(self, x, y, z, elbow_sign=-1):
        """로컬 EE 좌표(mm) → 로컬 관절각(도). 도달 불가면 ValueError."""
        radius = math.hypot(x, y)
        height = z - self.base_height
        reach = math.hypot(radius, height)
        cosine = (radius * radius + height * height - self.l1 ** 2 - self.l2 ** 2) / (2 * self.l1 * self.l2)
        if not -1.0 - 1e-9 <= cosine <= 1.0 + 1e-9:
            lo, hi = abs(self.l1 - self.l2), self.l1 + self.l2
            raise ValueError(f"도달 불가: 어깨로부터 {reach:.1f} mm (범위 {lo:.1f}~{hi:.1f} mm)")
        cosine = max(-1.0, min(1.0, cosine))
        elbow = elbow_sign * math.acos(cosine)
        shoulder = math.atan2(height, radius) - math.atan2(
            self.l2 * math.sin(elbow), self.l1 + self.l2 * math.cos(elbow)
        )
        return tuple(math.degrees(v) for v in (math.atan2(y, x), shoulder, elbow))

    # ── 관절각 ↔ 모터각 ───────────────────────────────────────────────────
    def geometric_to_motor(self, geometric_deg, arm: Arm):
        return tuple(a * s + o for a, s, o in zip(geometric_deg, arm.sign, arm.offset_deg))

    def motor_to_geometric(self, motor_by_md, arm: Arm):
        return tuple(
            (motor_by_md[md] - o) / s for md, s, o in zip(arm.md_ids, arm.sign, arm.offset_deg)
        )

    # ── 편의 ──────────────────────────────────────────────────────────────
    def world_ik_to_motor(self, world_point, arm: Arm, elbow_sign=-1, separation=None):
        """월드 좌표 → 그 팔의 MD별 모터각 dict. 도달 불가면 ValueError."""
        local = self.world_to_local(world_point, arm, separation)
        geometric = self.inverse(*local, elbow_sign=elbow_sign)
        motor = self.geometric_to_motor(geometric, arm)
        return dict(zip(arm.md_ids, motor))

    def ready_angles(self):
        """버튼1 준비자세의 MD별 모터각."""
        angles = {}
        for arm in self.arms:
            angles.update(zip(arm.md_ids, self.ready_motor_deg[arm.name]))
        return angles

    def ready_world_ee(self, arm: Arm, separation=None):
        geometric = self.motor_to_geometric(self.ready_angles(), arm)
        return self.local_to_world(self.forward(geometric), arm, separation)
