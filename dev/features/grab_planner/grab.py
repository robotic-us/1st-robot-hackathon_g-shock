"""비전이 준 물체 위치로 양팔 핀치(grab) 모션을 만든다.

이 로봇의 "grab" 은 그리퍼가 없다. 양팔이 물체를 **양옆에서 좁혀 집는 대칭 핀치**다.
기존 슬롯 2(grab) 모션을 디코드해서 나온 구조를 그대로 파라미터화했다.

  wp1 ready                      준비자세
  wp2 (X∓open,  Yp, Zg)          물체 양옆에 벌린 채 접근
  wp3 (X∓close, Yp, Zg)          좁혀서 집는다
  wp4 (X∓close, Yp, Zg+lift)     집은 채로 든다
  wp5 (X∓close, Yd, Zg)          놓을 자리로 이송
  wp6 (X∓close, Yd, Zg-down)     내려놓는다
  wp7 (X∓open,  Yd, Zg)          벌려서 놓고 후퇴
  wp8 ready                      복귀

★ 팔이 3자유도(yaw/shoulder/elbow)뿐이라 손목이 없다. 물체의 **자세(회전)는
  맞출 수 없고** 위치만 맞춘다. 비전이 각도를 줘도 이 플래너는 쓰지 않는다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from kinematics import Kinematics


@dataclass(frozen=True)
class GrabParams:
    """핀치 모션의 형상 파라미터. 단위는 전부 mm, 시간은 ms."""

    grasp_z: float = 120.0        # 집는 높이 (기존 grab 모션과 동일)
    lift_mm: float = 20.0         # 집은 뒤 들어올리는 높이
    place_down_mm: float = 30.0   # 놓을 때 내리는 깊이
    open_half_gap: float = 60.0   # 벌린 상태 반간격 (기존 grab: 60)
    close_half_gap: float = 30.0  # 집은 상태 반간격 (기존 grab: 30)
    place_xy: tuple = (0.0, 110.0)  # 놓을 자리 (기존 grab 과 동일)
    segment_ms: int = 1000
    elbow_sign: int = -1          # 버튼1 준비자세와 연속되는 해 (README 기준)


@dataclass
class Waypoint:
    label: str
    duration_ms: int
    angles: dict           # {md_id: motor_deg}
    s0: int = 0            # 가감속 파라미터 — 기존 모션들은 전부 0
    sd: int = 0
    ee_world: dict = field(default_factory=dict)   # {arm_name: (x,y,z)} — 검증·표시용


class Unreachable(ValueError):
    """어느 웨이포인트에서 어느 팔이 못 닿는지 담는다."""


def _pinch_pair(x, y, z, half_gap, grip_angle_deg=0.0):
    """(ARM A 목표, ARM B 목표) — 물체 중심에서 grip 축 방향으로 ±half_gap.

    grip_angle_deg 는 집는 방향이 월드 X축에 대해 돌아간 각도다. 0 이면 기존
    grab 모션처럼 X축을 따라 집는다. 팔에 손목이 없어 EE 자체는 못 돌리지만,
    두 팔의 목표점을 회전시키면 **집는 축**은 바꿀 수 있다.
    """
    t = math.radians(grip_angle_deg)
    dx, dy = half_gap * math.cos(t), half_gap * math.sin(t)
    return ((x - dx, y - dy, z), (x + dx, y + dy, z))


def plan_grab(pick_xy, params: GrabParams = GrabParams(), kin: Kinematics = None,
              grip_angle_deg: float = 0.0):
    """물체 위치 (X, Y) mm 를 받아 웨이포인트 목록을 만든다.

    grip_angle_deg 는 집는 축의 방향(도)이다. 비전이 물체 자세를 주면 그 **긴 축에
    수직**으로 집도록 넣으면 된다. 집어 든 뒤 이송·놓기 구간은 물체를 쥔 상태이므로
    같은 각도를 유지한다.

    도달 불가능한 지점이 하나라도 있으면 Unreachable 을 던진다 — 일부만 실행되면
    팔이 어정쩡한 자세로 멈추므로 부분 성공을 허용하지 않는다.
    """
    kin = kin or Kinematics()
    px, py = pick_xy
    dx, dy = params.place_xy
    zg = params.grasp_z
    op, cl = params.open_half_gap, params.close_half_gap
    ga = grip_angle_deg

    steps = [
        ("ready",          None),
        ("approach_open",  _pinch_pair(px, py, zg, op, ga)),
        ("close_grip",     _pinch_pair(px, py, zg, cl, ga)),
        ("lift",           _pinch_pair(px, py, zg + params.lift_mm, cl, ga)),
        ("transport",      _pinch_pair(dx, dy, zg, cl, ga)),
        ("place_down",     _pinch_pair(dx, dy, zg - params.place_down_mm, cl, ga)),
        ("release_open",   _pinch_pair(dx, dy, zg, op, ga)),
        ("ready",          None),
    ]

    ready = kin.ready_angles()
    waypoints = []
    for label, pair in steps:
        if pair is None:
            wp = Waypoint(label, params.segment_ms, dict(ready))
            wp.ee_world = {arm.name: kin.ready_world_ee(arm) for arm in kin.arms}
            waypoints.append(wp)
            continue
        angles, ee = {}, {}
        for arm, target in zip(kin.arms, pair):
            try:
                angles.update(kin.world_ik_to_motor(target, arm, elbow_sign=params.elbow_sign))
            except ValueError as exc:
                raise Unreachable(
                    f"'{label}' 단계에서 {arm.name} 가 {target} 에 못 닿습니다 — {exc}"
                ) from exc
            ee[arm.name] = target
        wp = Waypoint(label, params.segment_ms, angles)
        wp.ee_world = ee
        waypoints.append(wp)

    _check_limits(waypoints)
    return waypoints


def _check_limits(waypoints):
    """motion_builder 가 강제하는 것과 같은 범위 검사."""
    if len(waypoints) > 20:
        raise Unreachable(f"웨이포인트가 {len(waypoints)}개 — 최대 20개")
    for i, wp in enumerate(waypoints, 1):
        if not 1 <= wp.duration_ms <= 65535:
            raise Unreachable(f"wp{i} 구간 {wp.duration_ms}ms — 1~65535 범위를 벗어남")
        for md, deg in wp.angles.items():
            if not -360.0 <= deg <= 360.0:
                raise Unreachable(f"wp{i} MD{md} 각도 {deg:.1f}° — ±360° 범위를 벗어남")


def reachable(pick_xy, params: GrabParams = GrabParams(), kin: Kinematics = None,
              grip_angle_deg: float = 0.0):
    """그 위치를 집을 수 있는지만 빠르게 판정한다."""
    try:
        plan_grab(pick_xy, params, kin, grip_angle_deg)
        return True
    except Unreachable:
        return False


def reach_map(params: GrabParams = GrabParams(), kin: Kinematics = None,
              x_range=(-120, 120), y_range=(-80, 200), step=10,
              grip_angle_deg: float = 0.0):
    """집을 수 있는 (X, Y) 격자점 목록. 비전 좌표 정렬·슬롯 배치 계획용."""
    kin = kin or Kinematics()
    points = []
    x = x_range[0]
    while x <= x_range[1]:
        y = y_range[0]
        while y <= y_range[1]:
            if reachable((x, y), params, kin, grip_angle_deg):
                points.append((x, y))
            y += step
        x += step
    return points


def grip_clearance(object_width_mm, params: GrabParams = GrabParams()):
    """집을 때 물체를 실제로 무는지 판정한다.

    반환 (물림량_mm, 진단문). 물림량이 0 이하이면 팔 사이로 빠져나간다.
    양수라도 너무 크면 팔이 물체를 밀어버린다.
    """
    gap = 2 * params.close_half_gap
    squeeze = object_width_mm - gap
    if squeeze <= 0:
        return squeeze, (f"물체 폭 {object_width_mm:.1f}mm < 집은 간격 {gap:.1f}mm — "
                         f"팔 사이로 빠집니다. close_half_gap 을 "
                         f"{object_width_mm/2 - 2:.1f}mm 이하로 줄이세요.")
    return squeeze, f"물림량 {squeeze:.1f}mm (물체 {object_width_mm:.1f}mm, 간격 {gap:.1f}mm)"
