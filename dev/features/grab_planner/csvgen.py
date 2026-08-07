"""웨이포인트 → PCM 슬롯 파일(motion_NN.csv + motion_NN.memo.json).

포맷은 main/motion_builder/motion_builder.py 의 write_generated_files() 와
바이트 단위로 같아야 한다 (tests/test_csv_format.py 가 기존 슬롯 파일을 재생성해서
바이트 비교로 검증한다).
"""

from __future__ import annotations

import csv
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

from kinematics import Kinematics

FILE_VERSION = "3.0.0"
MAX_WAYPOINTS = 20
MD_COUNT = 12


def build_csv_rows(slot_id: int, name: str, waypoints, kin: Kinematics = None):
    kin = kin or Kinematics()
    profile = kin.profile
    active = set(kin.active_md_ids)
    rows = [
        ["robot_id", str(profile["robot_id"])] + [""] * 21,
        ["file_version", FILE_VERSION] + [""] * 21,
        ["MS ID", "MS Name", "MD ID", "P vector"] + [""] * 19,
        ["", "", ""] + [str(i) for i in range(MAX_WAYPOINTS)],
    ]
    for md in range(MD_COUNT):
        prefix = [str(slot_id), name, f"MD{md}"] if md == 0 else ["", "", f"MD{md}"]
        vectors = []
        for wp in waypoints:
            if md in active:
                vectors.append(f"{wp.angles[md]:.3f},{wp.duration_ms},{wp.s0},{wp.sd}")
            else:
                vectors.append("-")
        vectors += ["-"] * (MAX_WAYPOINTS - len(vectors))
        rows.append(prefix + vectors)
    return rows


def write_motion(out_dir, slot_id: int, name: str, waypoints, kin: Kinematics = None,
                 updated: str = None):
    """CSV/memo 를 쓰고 (csv_path, memo_path, sha256) 을 돌려준다."""
    if not 1 <= slot_id <= 50:
        raise ValueError(f"슬롯 ID 는 1~50 이어야 합니다: {slot_id}")
    if not waypoints:
        raise ValueError("웨이포인트가 비어 있습니다")
    if len(waypoints) > MAX_WAYPOINTS:
        raise ValueError(f"웨이포인트 {len(waypoints)}개 — 최대 {MAX_WAYPOINTS}개")

    kin = kin or Kinematics()
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    base = f"motion_{slot_id:02d}"
    csv_path = out_dir / f"{base}.csv"
    memo_path = out_dir / f"{base}.memo.json"

    with csv_path.open("w", newline="") as handle:
        csv.writer(handle, lineterminator="\n").writerows(
            build_csv_rows(slot_id, name, waypoints, kin)
        )
    digest = hashlib.sha256(csv_path.read_bytes()).hexdigest()

    profile = kin.profile
    memo = {
        "zero_snapshot": {
            "format_version": 2,
            "robot_uid": profile["robot_uid"],
            "known_mask": profile["known_mask"],
            "zero_offset_f32_le_hex": profile["zero_offset_f32_le_hex"],
        },
        "teaching_start_angles_rad": [None] * MD_COUNT,
        "motion_sha256": digest,
        "schema": 2,
        "slot_id": slot_id,
        "updated": updated or datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
    }
    memo_path.write_text(json.dumps(memo, indent=2) + "\n")
    return csv_path, memo_path, digest


def read_motion(csv_path, kin: Kinematics = None):
    """기존 슬롯 CSV 를 웨이포인트로 되읽는다 (검증·비교용)."""
    from grab import Waypoint

    kin = kin or Kinematics()
    rows = list(csv.reader(Path(csv_path).open()))
    name = rows[4][1]
    slot_id = int(rows[4][0])
    vectors = {}
    for row in rows[4:]:
        md_cell = row[2]
        if md_cell.startswith("MD"):
            vectors[int(md_cell[2:])] = row[3:3 + MAX_WAYPOINTS]
    first = kin.active_md_ids[0]
    count = sum(1 for cell in vectors[first] if cell != "-")
    waypoints = []
    for i in range(count):
        angles, duration, s0, sd = {}, None, 0, 0
        for md in kin.active_md_ids:
            deg, dur, a, b = vectors[md][i].split(",")
            angles[md] = float(deg)
            duration, s0, sd = int(dur), int(a), int(b)
        waypoints.append(Waypoint(f"wp{i+1}", duration, angles, s0, sd))
    return slot_id, name, waypoints
