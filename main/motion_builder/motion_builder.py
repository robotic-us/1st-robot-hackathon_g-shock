#!/usr/bin/env python3
"""GUI editor for PHORCE v3 waypoint/P-Vector motion files."""

import csv
import hashlib
import json
import math
import os
import re
import shutil
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk


ROOT = Path(__file__).resolve().parent
PROFILE = json.loads((ROOT / "profile.json").read_text())
ACTIVE = PROFILE["active_md_ids"]
OUTPUT = ROOT / "library"
PROJECTS = ROOT / "projects"
NAME_RE = re.compile(r"[A-Za-z0-9_-]+")
KIN = PROFILE["kinematics"]
OUTPUT.mkdir(exist_ok=True)
PROJECTS.mkdir(exist_ok=True)


def inverse_kinematics(x, y, z, elbow_sign=1):
    """Return geometric yaw/shoulder/elbow angles in degrees."""
    l1, l2 = KIN["link_1_mm"], KIN["link_2_to_tip_mm"]
    radius, height = math.hypot(x, y), z - KIN["base_height_mm"]
    cosine = (radius * radius + height * height - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    if cosine < -1.0 - 1e-9 or cosine > 1.0 + 1e-9:
        raise ValueError(f"도달 불가: 어깨로부터 거리 {math.hypot(radius, height):.1f} mm (범위 {abs(l1-l2):.1f}~{l1+l2:.1f} mm)")
    cosine = max(-1.0, min(1.0, cosine))
    elbow = elbow_sign * math.acos(cosine)
    shoulder = math.atan2(height, radius) - math.atan2(l2 * math.sin(elbow), l1 + l2 * math.cos(elbow))
    return tuple(math.degrees(v) for v in (math.atan2(y, x), shoulder, elbow))


def forward_kinematics(angles_deg):
    yaw, shoulder, elbow = map(math.radians, angles_deg)
    l1, l2, base = KIN["link_1_mm"], KIN["link_2_to_tip_mm"], KIN["base_height_mm"]
    r1, z1 = l1 * math.cos(shoulder), base + l1 * math.sin(shoulder)
    r2 = r1 + l2 * math.cos(shoulder + elbow)
    z2 = z1 + l2 * math.sin(shoulder + elbow)
    return ((0.0, 0.0, base),
            (r1 * math.cos(yaw), r1 * math.sin(yaw), z1),
            (r2 * math.cos(yaw), r2 * math.sin(yaw), z2))


class Builder:
    def __init__(self, root):
        self.root = root
        self.rows = []
        self.view_azimuth = -45.0
        self.view_elevation = 25.0
        self.view_zoom = 1.0
        self.drag_start = None
        root.title("PHORCE Waypoint Motion Builder")
        root.geometry("1360x820")
        outer = ttk.Frame(root, padding=14)
        outer.pack(fill="both", expand=True)

        meta = ttk.LabelFrame(outer, text="모션", padding=10)
        meta.pack(fill="x")
        self.slot = tk.StringVar(value="5")
        self.name = tk.StringVar(value="CUSTOM_MOTION")
        ttk.Label(meta, text="슬롯 ID (1~50)").grid(row=0, column=0, sticky="w")
        ttk.Entry(meta, textvariable=self.slot, width=8).grid(row=1, column=0, padx=(0, 12))
        ttk.Label(meta, text="MS Name").grid(row=0, column=1, sticky="w")
        ttk.Entry(meta, textvariable=self.name, width=28).grid(row=1, column=1, padx=(0, 12))
        ttk.Button(meta, text="프로젝트 불러오기", command=self.load_project).grid(row=1, column=2, padx=4)
        ttk.Button(meta, text="프로젝트 저장", command=self.save_project).grid(row=1, column=3, padx=4)
        ttk.Button(meta, text="CSV + memo 생성", command=self.generate).grid(row=1, column=4, padx=12)
        self.sd_root = tk.StringVar(value="/media/phorce/9016-4EF8")
        ttk.Label(meta, text="SD 마운트 경로").grid(row=0, column=5, sticky="w")
        ttk.Entry(meta, textvariable=self.sd_root, width=30).grid(row=1, column=5, padx=(8, 3))
        ttk.Button(meta, text="찾기", command=self.choose_sd).grid(row=1, column=6, padx=3)
        ttk.Button(meta, text="현재 모션 SD에 쓰기", command=self.install_current_to_sd).grid(row=1, column=7, padx=(3, 0))

        edit = ttk.LabelFrame(outer, text="웨이포인트 추가/수정 — 각도는 출력축 기준 degree", padding=10)
        edit.pack(fill="x", pady=10)
        self.inputs = {}
        specs = [("duration", "구간(ms)", "1000"), ("s0", "s0", "0"), ("sd", "sd", "0")]
        ready_by_md = {}
        for group_name, cfg in KIN["groups"].items():
            ready_by_md.update(zip(cfg["md_ids"], KIN["button1_ready_motor_deg"][group_name]))
        specs += [(f"md{md}", f"MD{md} (°)", str(ready_by_md[md])) for md in ACTIVE]
        for col, (key, label, default) in enumerate(specs):
            var = tk.StringVar(value=default)
            self.inputs[key] = var
            ttk.Label(edit, text=label).grid(row=0, column=col, sticky="w", padx=3)
            ttk.Entry(edit, textvariable=var, width=10).grid(row=1, column=col, padx=3)
        ttk.Button(edit, text="추가", command=self.add).grid(row=2, column=0, pady=8, sticky="ew")
        ttk.Button(edit, text="선택 수정", command=self.update).grid(row=2, column=1, pady=8, sticky="ew")
        ttk.Button(edit, text="삭제", command=self.delete).grid(row=2, column=2, pady=8, sticky="ew")
        ttk.Button(edit, text="▲", command=lambda: self.move(-1)).grid(row=2, column=3, pady=8)
        ttk.Button(edit, text="▼", command=lambda: self.move(1)).grid(row=2, column=4, pady=8)
        ttk.Button(edit, text="버튼 1 준비자세 입력", command=self.set_ready_inputs).grid(row=2, column=5, columnspan=2, padx=4, pady=8, sticky="ew")
        ttk.Button(edit, text="준비자세를 시작·끝에 추가", command=self.wrap_with_ready).grid(row=2, column=7, columnspan=2, padx=4, pady=8, sticky="ew")

        cart = ttk.LabelFrame(outer, text="엔드이펙터 월드 절대좌표 → 조인트 각도 (mm)", padding=8)
        cart.pack(fill="x", pady=(0, 10))
        initial_separation = KIN.get("base_separation_mm", 350.0)
        self.base_separation = tk.StringVar(value=str(initial_separation))
        self.xyz, self.elbow_modes, self.ik_status = {}, {}, {}
        for col, title in enumerate(("팔", "절대 X", "절대 Y", "절대 Z", "IK 해")):
            ttk.Label(cart, text=title).grid(row=0, column=col, sticky="w", padx=3)
        ttk.Label(cart, text="두 J1 간격").grid(row=0, column=8, sticky="w", padx=3)
        base_entry = ttk.Entry(cart, textvariable=self.base_separation, width=9)
        base_entry.grid(row=1, column=8, padx=4, sticky="w"); base_entry.bind("<Return>", self.base_separation_changed)
        for group_index, (group_name, cfg) in enumerate(KIN["groups"].items()):
            row = group_index + 1
            short_name = "1번팔 · J1/J2/J3=phact0/1/2" if group_index == 0 else "2번팔 · J1/J2/J3=phact8/6/7"
            ttk.Label(cart, text=short_name).grid(row=row, column=0, padx=(3, 8), pady=3, sticky="w")
            ready_motor = KIN["button1_ready_motor_deg"][group_name]
            ready_q = tuple((v-o)/s for v, o, s in zip(ready_motor, cfg["offset_deg"], cfg["sign"]))
            ready_world = self.local_to_world_point(forward_kinematics(ready_q)[-1], group_index, initial_separation)
            self.xyz[group_name] = {}
            for col, (key, value) in enumerate(zip(("x", "y", "z"), ready_world), 1):
                self.xyz[group_name][key] = tk.StringVar(value=f"{value:.1f}")
                ttk.Entry(cart, textvariable=self.xyz[group_name][key], width=10).grid(row=row, column=col, padx=3)
            self.elbow_modes[group_name] = tk.StringVar(value="팔꿈치 -")
            ttk.Combobox(cart, textvariable=self.elbow_modes[group_name], values=("팔꿈치 +", "팔꿈치 -"),
                         state="readonly", width=10).grid(row=row, column=4, padx=5)
            ttk.Button(cart, text="IK → 각도", command=lambda name=group_name: self.solve_ik(name)).grid(row=row, column=5, padx=4)
            ttk.Button(cart, text="현재각 → EE", command=lambda name=group_name: self.update_xyz_from_inputs(name)).grid(row=row, column=6, padx=4)
            self.ik_status[group_name] = tk.StringVar(value="준비자세")
            ttk.Label(cart, textvariable=self.ik_status[group_name]).grid(row=row, column=7, padx=8, sticky="w")
        ttk.Button(cart, text="경로 다시 그리기", command=self.draw_plan).grid(row=2, column=8, padx=4, sticky="w")

        columns = ["n", "duration", "s0", "sd"] + [f"md{md}" for md in ACTIVE]
        body = ttk.Panedwindow(outer, orient="horizontal")
        body.pack(fill="both", expand=True)
        table_frame = ttk.Frame(body)
        visual = ttk.LabelFrame(body, text="양팔 3D 플래닝 미리보기 — 조인트 공간 보간 예상 경로", padding=6)
        body.add(table_frame, weight=3); body.add(visual, weight=2)
        self.tree = ttk.Treeview(table_frame, columns=columns, show="headings", height=18)
        titles = {"n": "#", "duration": "구간(ms)", "s0": "s0", "sd": "sd"}
        titles.update({f"md{md}": f"MD{md} (°)" for md in ACTIVE})
        for key in columns:
            self.tree.heading(key, text=titles[key])
            self.tree.column(key, width=70 if key == "n" else 105, anchor="center")
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self.select)
        self.view3d = tk.Canvas(visual, width=500, height=540, bg="white", highlightthickness=1, highlightbackground="#aaa")
        self.view3d.pack(fill="both", expand=True)
        self.view3d.bind("<Configure>", lambda _e: self.draw_plan())
        self.view3d.bind("<ButtonPress-1>", self.start_view_drag)
        self.view3d.bind("<B1-Motion>", self.drag_view)
        self.view3d.bind("<ButtonRelease-1>", lambda _e: setattr(self, "drag_start", None))
        self.view3d.bind("<MouseWheel>", self.zoom_view)
        self.view3d.bind("<Button-4>", lambda event: self.zoom_view(event, 1))
        self.view3d.bind("<Button-5>", lambda event: self.zoom_view(event, -1))
        self.view3d.bind("<Double-Button-1>", self.reset_view)
        ttk.Label(outer, text="최대 20개 웨이포인트 · yd -360~360° · s0/sd -128~128 · 생성 파일은 motion_builder/library에 저장").pack(anchor="w", pady=(8, 0))
        self.draw_plan()

    def separation_value(self):
        try: value = float(self.base_separation.get())
        except (ValueError, AttributeError) as exc: raise ValueError("두 J1 간격을 숫자로 입력하세요") from exc
        if value < 0: raise ValueError("두 J1 간격은 0 이상이어야 합니다")
        return value

    @staticmethod
    def local_to_world_point(point, group_index, separation):
        x, y, z = point
        return (-separation / 2 + x, y, z) if group_index == 0 else (separation / 2 - x, -y, z)

    @staticmethod
    def world_to_local_point(point, group_index, separation):
        x, y, z = point
        return (x + separation / 2, y, z) if group_index == 0 else (separation / 2 - x, -y, z)

    @staticmethod
    def group_index(group_name):
        return list(KIN["groups"]).index(group_name)

    def base_separation_changed(self, _event=None):
        self.update_xyz_from_inputs()
        self.draw_plan()

    def update_xyz_from_inputs(self, group_name=None):
        names = [group_name] if group_name else list(KIN["groups"])
        for name in names:
            try:
                angles = {md: float(self.inputs[f"md{md}"].get()) for md in ACTIVE}
                geometric = self.motor_to_geometric(angles, name)
                local_ee = forward_kinematics(geometric)[-1]
                world_ee = self.local_to_world_point(local_ee, self.group_index(name), self.separation_value())
                for key, value in zip(("x", "y", "z"), world_ee): self.xyz[name][key].set(f"{value:.3f}")
                self.ik_status[name].set("EE " + ", ".join(f"{key.upper()}={value:.1f}" for key, value in zip(("x", "y", "z"), world_ee)))
            except (ValueError, KeyError, ZeroDivisionError) as exc:
                self.ik_status[name].set(str(exc))

    def ready_angles(self):
        angles = {}
        for group_name, cfg in KIN["groups"].items():
            angles.update(zip(cfg["md_ids"], KIN["button1_ready_motor_deg"][group_name]))
        return angles

    def set_ready_inputs(self):
        for md, value in self.ready_angles().items():
            self.inputs[f"md{md}"].set(str(value))
        self.update_xyz_from_inputs()
        for status in self.ik_status.values(): status.set("버튼 1 준비자세 · J2~J3 수직 / J3~EE 수평")
        self.draw_plan()

    def wrap_with_ready(self):
        if len(self.rows) > 18:
            return messagebox.showerror("제한", "시작·끝을 추가하려면 기존 웨이포인트가 18개 이하여야 합니다")
        ready = {"duration": int(self.inputs["duration"].get() or 1000), "s0": 0, "sd": 0,
                 "angles": self.ready_angles()}
        self.rows = [dict(ready, angles=dict(ready["angles"]))] + self.rows + [dict(ready, angles=dict(ready["angles"]))]
        self.refresh()

    def motor_to_geometric(self, angles, group_name=None):
        if group_name is None: raise ValueError("팔 그룹이 필요합니다")
        cfg = KIN["groups"][group_name]
        return tuple((angles[md] - offset) / sign for md, sign, offset in zip(cfg["md_ids"], cfg["sign"], cfg["offset_deg"]))

    def geometric_to_motor(self, angles, group_name):
        cfg = KIN["groups"][group_name]
        return tuple(angle * sign + offset for angle, sign, offset in zip(angles, cfg["sign"], cfg["offset_deg"]))

    def solve_ik(self, group_name):
        try:
            world_xyz = tuple(float(self.xyz[group_name][key].get()) for key in ("x", "y", "z"))
            local_xyz = self.world_to_local_point(world_xyz, self.group_index(group_name), self.separation_value())
            geometric = inverse_kinematics(*local_xyz, elbow_sign=1 if self.elbow_modes[group_name].get().endswith("+") else -1)
            motor = self.geometric_to_motor(geometric, group_name)
            for md, value in zip(KIN["groups"][group_name]["md_ids"], motor):
                self.inputs[f"md{md}"].set(f"{value:.3f}")
            self.ik_status[group_name].set("IK " + ", ".join(f"J{i+1}={v:+.1f}°" for i, v in enumerate(geometric)))
            self.draw_plan(preview={group_name: geometric})
        except (ValueError, ZeroDivisionError) as exc:
            self.ik_status[group_name].set(str(exc)); messagebox.showerror("IK 계산 실패", str(exc))

    @staticmethod
    def world_pose(local_pose, group_index, separation):
        """Place identical arms opposite each other in a common world frame."""
        return tuple(Builder.local_to_world_point(point, group_index, separation) for point in local_pose)

    def project_3d(self, point, scale, origin):
        x, y, z = point
        azimuth = math.radians(self.view_azimuth)
        elevation = math.radians(self.view_elevation)
        # Rotate the world around Z, then project at the selected elevation.
        horizontal = math.cos(azimuth) * x - math.sin(azimuth) * y
        depth = math.sin(azimuth) * x + math.cos(azimuth) * y
        vertical = math.cos(elevation) * z - math.sin(elevation) * depth
        effective_scale = scale * self.view_zoom
        return (origin[0] + horizontal * effective_scale,
                origin[1] - vertical * effective_scale)

    def start_view_drag(self, event):
        self.drag_start = (event.x, event.y, self.view_azimuth, self.view_elevation)

    def drag_view(self, event):
        if self.drag_start is None: return
        x0, y0, azimuth0, elevation0 = self.drag_start
        self.view_azimuth = (azimuth0 + (event.x - x0) * 0.45) % 360.0
        self.view_elevation = max(-80.0, min(80.0, elevation0 - (event.y - y0) * 0.35))
        self.draw_plan()

    def zoom_view(self, event, linux_direction=None):
        direction = linux_direction if linux_direction is not None else (1 if event.delta > 0 else -1)
        self.view_zoom = max(0.35, min(3.5, self.view_zoom * (1.12 if direction > 0 else 1 / 1.12)))
        self.draw_plan()
        return "break"

    def reset_view(self, _event=None):
        self.view_azimuth, self.view_elevation, self.view_zoom = -45.0, 25.0, 1.0
        self.drag_start = None
        self.draw_plan()
        return "break"

    def draw_plan(self, preview=None):
        if not hasattr(self, "view3d"): return
        canvas = self.view3d
        canvas.delete("all")
        try: separation = float(self.base_separation.get())
        except ValueError: separation = KIN.get("base_separation_mm", 300.0)
        separation = max(0.0, separation)
        width, height = max(canvas.winfo_width(), 480), max(canvas.winfo_height(), 420)
        span = max(520.0, separation + 2 * KIN["link_1_mm"] + 2 * KIN["link_2_to_tip_mm"])
        scale = min((width - 35) / span, (height - 80) / 390.0)
        origin = (width / 2, height - 55)
        colors = ("#1769aa", "#c43d3d")
        labels = ("ARM A · J1/J2/J3 = phact0/1/2", "ARM B · J1/J2/J3 = phact8/6/7")
        joint_labels = (("phact0 J1 / phact1 J2", "phact2 J3", "EE"),
                        ("phact8 J1 / phact6 J2", "phact7 J3", "EE"))

        # Ground plane and world axes.
        corners = [(-span/2, -170, 0), (span/2, -170, 0), (span/2, 170, 0), (-span/2, 170, 0)]
        ground = [self.project_3d(p, scale, origin) for p in corners]
        canvas.create_polygon(*sum(ground, ()), fill="#f5f5f2", outline="#c8c8c0")
        for endpoint, color, name in (((80, 0, 0), "#d33", "X"), ((0, 80, 0), "#299447", "Y"), ((0, 0, 80), "#286bc0", "Z")):
            p0, p1 = self.project_3d((0, 0, 0), scale, origin), self.project_3d(endpoint, scale, origin)
            canvas.create_line(*p0, *p1, fill=color, width=2, arrow="last"); canvas.create_text(*p1, text=name, fill=color)
        canvas.create_text(10, 10, anchor="nw", text=(f"월드 3D · J1 간격 {separation:.1f} mm · "
                                                       f"방위 {self.view_azimuth:.0f}° / 고도 {self.view_elevation:.0f}° / 확대 {self.view_zoom:.2f}×"))
        canvas.create_text(10, 30, anchor="nw", text=labels[0], fill=colors[0], font=("TkDefaultFont", 10, "bold"))
        canvas.create_text(10, 50, anchor="nw", text=labels[1], fill=colors[1], font=("TkDefaultFont", 10, "bold"))

        selected = self.tree.selection() if hasattr(self, "tree") else ()
        pose_row = self.rows[int(selected[0])] if selected else (self.rows[-1] if self.rows else None)
        if pose_row is None:
            pose_angles = self.ready_angles()
            pose_caption = "버튼 1 준비자세"
        else:
            pose_angles = pose_row["angles"]
            pose_caption = f"웨이포인트 {int(selected[0])+1}" if selected else f"웨이포인트 {len(self.rows)}"

        for group_index, (group_name, cfg) in enumerate(KIN["groups"].items()):
            color = colors[group_index]
            # Draw sampled EE path for this arm.
            geometries = [self.motor_to_geometric(row["angles"], group_name) for row in self.rows]
            path = []
            for start, end in zip(geometries, geometries[1:]):
                for step in range(21):
                    t = step / 20
                    q = tuple(a + (b-a)*t for a, b in zip(start, end))
                    path.append(self.world_pose(forward_kinematics(q), group_index, separation)[-1])
            if len(path) > 1:
                projected = [self.project_3d(p, scale, origin) for p in path]
                canvas.create_line(*sum(projected, ()), fill=color, width=2, dash=(4, 3))

            geometric = (preview[group_name] if isinstance(preview, dict) and group_name in preview
                         else self.motor_to_geometric(pose_angles, group_name))
            world = self.world_pose(forward_kinematics(geometric), group_index, separation)
            points = [self.project_3d(p, scale, origin) for p in world]
            # Ground-to-J1 vertical pedestal, then both manipulator links.
            base_ground = self.project_3d((world[0][0], world[0][1], 0), scale, origin)
            canvas.create_line(*base_ground, *points[0], fill="#555", width=5)
            canvas.create_line(*sum(points, ()), fill=color, width=7, capstyle="round", joinstyle="round")
            for index, point in enumerate(points):
                canvas.create_oval(point[0]-5, point[1]-5, point[0]+5, point[1]+5, fill="white", outline=color, width=2)
                canvas.create_text(point[0]+7, point[1]-8, anchor="sw", text=joint_labels[group_index][index], fill=color)
            canvas.create_text(points[-1][0], points[-1][1]+15, text=f"({world[-1][0]:.1f}, {world[-1][1]:.1f}, {world[-1][2]:.1f})", fill=color)

        canvas.create_text(width/2, height-12, text=f"{pose_caption}  |  좌드래그: 회전 · 휠: 확대/축소 · 더블클릭: 시점 초기화",
                           font=("TkDefaultFont", 10, "bold"))

    def parsed_input(self):
        try:
            duration = int(self.inputs["duration"].get())
            s0, sd = int(self.inputs["s0"].get()), int(self.inputs["sd"].get())
            angles = {md: float(self.inputs[f"md{md}"].get()) for md in ACTIVE}
        except ValueError as exc:
            raise ValueError("숫자 입력을 확인하세요") from exc
        if not 1 <= duration <= 65535:
            raise ValueError("구간은 1~65535ms여야 합니다")
        if not -128 <= s0 <= 128 or not -128 <= sd <= 128:
            raise ValueError("s0/sd는 -128~128이어야 합니다")
        if any(not -360 <= value <= 360 for value in angles.values()):
            raise ValueError("각도는 -360~360°여야 합니다")
        return {"duration": duration, "s0": s0, "sd": sd, "angles": angles}

    def add(self):
        if len(self.rows) >= 20:
            return messagebox.showerror("제한", "웨이포인트는 최대 20개입니다")
        try: self.rows.append(self.parsed_input())
        except ValueError as exc: return messagebox.showerror("입력 오류", str(exc))
        self.refresh()

    def update(self):
        selected = self.tree.selection()
        if not selected: return
        try: self.rows[int(selected[0])] = self.parsed_input()
        except ValueError as exc: return messagebox.showerror("입력 오류", str(exc))
        self.refresh()

    def delete(self):
        selected = self.tree.selection()
        if selected:
            del self.rows[int(selected[0])]
            self.refresh()

    def move(self, delta):
        selected = self.tree.selection()
        if not selected: return
        old = int(selected[0]); new = old + delta
        if 0 <= new < len(self.rows):
            self.rows[old], self.rows[new] = self.rows[new], self.rows[old]
            self.refresh(); self.tree.selection_set(str(new))

    def select(self, _event=None):
        selected = self.tree.selection()
        if not selected: return
        row = self.rows[int(selected[0])]
        for key in ("duration", "s0", "sd"): self.inputs[key].set(str(row[key]))
        for md in ACTIVE: self.inputs[f"md{md}"].set(str(row["angles"][md]))
        self.update_xyz_from_inputs()
        self.draw_plan()

    def refresh(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        for index, row in enumerate(self.rows):
            values = [index + 1, row["duration"], row["s0"], row["sd"]]
            values += [f'{row["angles"][md]:.3f}' for md in ACTIVE]
            self.tree.insert("", "end", iid=str(index), values=values)
        self.draw_plan()

    def metadata(self):
        try: slot = int(self.slot.get())
        except ValueError: raise ValueError("슬롯 ID는 정수여야 합니다")
        name = self.name.get().strip()
        if not 1 <= slot <= 50: raise ValueError("슬롯 ID는 1~50이어야 합니다")
        if NAME_RE.fullmatch(name) is None: raise ValueError("이름은 영문, 숫자, _, -만 사용하세요")
        if not self.rows: raise ValueError("웨이포인트를 하나 이상 추가하세요")
        return slot, name

    def project(self):
        slot, name = self.metadata()
        return {"schema": 1, "slot_id": slot, "name": name, "waypoints": self.rows}

    def choose_sd(self):
        path = filedialog.askdirectory(initialdir=self.sd_root.get() or "/media", title="Motions 폴더가 있는 SD카드 선택")
        if path: self.sd_root.set(path)

    def install_current_to_sd(self):
        try:
            slot, name = self.metadata()
            source_dir = OUTPUT / name
            basename = f"motion_{slot:02d}"
            csv_path, memo_path = source_dir / f"{basename}.csv", source_dir / f"{basename}.memo.json"
            sd_root = Path(self.sd_root.get()).expanduser().resolve()
            motions = sd_root / "Motions"
            if not motions.is_dir(): raise ValueError("선택한 경로에 Motions 폴더가 없습니다")
            if motions.is_symlink(): raise ValueError("안전을 위해 심볼릭 링크인 Motions 폴더에는 쓰지 않습니다")
            # "현재 모션" means the rows currently visible in the GUI. Always
            # rebuild derived CSV/memo here so a stale artifact cannot be copied.
            csv_path, memo_path, digest = self.write_generated_files(slot, name)
            memo = json.loads(memo_path.read_text())
            if memo.get("motion_sha256") != digest: raise ValueError("CSV와 memo의 SHA-256이 일치하지 않습니다")
            targets = (motions / csv_path.name, motions / memo_path.name)
            if any(target.is_symlink() for target in targets):
                raise ValueError("대상 슬롯 파일이 심볼릭 링크이므로 중단합니다")
            existing = [target for target in targets if target.exists()]
            if existing:
                phrase = f"OVERWRITE SLOT {slot:02d}"
                answer = simpledialog.askstring(
                    "기존 SD 슬롯 덮어쓰기",
                    f"SD카드에 {slot:02d}번 슬롯이 있습니다.\n기존 파일은 로컬에 백업됩니다.\n\n계속하려면 정확히 입력하세요:\n{phrase}",
                    parent=self.root)
                if answer != phrase:
                    messagebox.showinfo("취소", "확인 문구가 일치하지 않아 SD카드를 변경하지 않았습니다")
                    return
            elif not messagebox.askyesno("SD카드 설치 확인", f"새 슬롯 {slot:02d}를 다음 SD카드에 설치할까요?\n{sd_root}"):
                return

            backup_dir = None
            if existing:
                backup_dir = source_dir / ("sd_backup_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
                backup_dir.mkdir(parents=True, exist_ok=False)
                for target in existing: shutil.copy2(target, backup_dir / target.name)
            shutil.copy2(csv_path, targets[0])
            shutil.copy2(memo_path, targets[1])
            os.sync()
            if hashlib.sha256(targets[0].read_bytes()).hexdigest() != digest:
                raise OSError("SD CSV 복사 검증에 실패했습니다")
            if targets[1].read_bytes() != memo_path.read_bytes():
                raise OSError("SD memo 복사 검증에 실패했습니다")
            backup_text = f"\n기존 파일 백업: {backup_dir}" if backup_dir else ""
            messagebox.showinfo("SD 설치 완료", f"슬롯 {slot:02d} 설치와 검증이 완료되었습니다.{backup_text}\n안전하게 꺼낸 뒤 PCM을 재시작하세요")
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            messagebox.showerror("SD 설치 실패", f"{exc}\n\nSD카드가 읽기 전용이면 먼저 rw로 다시 마운트하세요")

    def save_project(self):
        try: data = self.project()
        except ValueError as exc: return messagebox.showerror("검증 실패", str(exc))
        path = filedialog.asksaveasfilename(initialdir=PROJECTS, initialfile=f'{data["name"]}.json', defaultextension=".json")
        if path: Path(path).write_text(json.dumps(data, indent=2) + "\n")

    def load_project(self):
        path = filedialog.askopenfilename(initialdir=PROJECTS, filetypes=[("JSON", "*.json")])
        if not path: return
        try:
            data = json.loads(Path(path).read_text())
            self.slot.set(str(data["slot_id"])); self.name.set(data["name"])
            self.rows = [{"duration": int(w["duration"]), "s0": int(w["s0"]), "sd": int(w["sd"]),
                          "angles": {int(k): float(v) for k, v in w["angles"].items()}}
                         for w in data["waypoints"]]
            for row in self.rows:
                for md in ACTIVE:
                    if md not in row["angles"]: raise ValueError(f"MD{md} 누락")
            if len(self.rows) > 20: raise ValueError("웨이포인트 20개 초과")
            self.refresh()
        except Exception as exc: messagebox.showerror("불러오기 실패", str(exc))

    def generate(self):
        try: slot, name = self.metadata()
        except ValueError as exc: return messagebox.showerror("검증 실패", str(exc))
        output = OUTPUT / name
        output.mkdir(parents=True, exist_ok=True)
        csv_path = output / f"motion_{slot:02d}.csv"
        memo_path = output / f"motion_{slot:02d}.memo.json"
        if (csv_path.exists() or memo_path.exists()) and not messagebox.askyesno("로컬 덮어쓰기", f"{output}의 기존 생성물을 덮어쓸까요?"):
            return
        csv_path, memo_path, digest = self.write_generated_files(slot, name)
        messagebox.showinfo("생성 완료", f"{csv_path}\n{memo_path}\nSHA-256: {digest}")

    def write_generated_files(self, slot, name):
        """Materialize the current GUI rows and return paths plus CSV digest."""
        output = OUTPUT / name
        output.mkdir(parents=True, exist_ok=True)
        csv_path = output / f"motion_{slot:02d}.csv"
        memo_path = output / f"motion_{slot:02d}.memo.json"
        rows = [
            ["robot_id", str(PROFILE["robot_id"])] + [""] * 21,
            ["file_version", "3.0.0"] + [""] * 21,
            ["MS ID", "MS Name", "MD ID", "P vector"] + [""] * 19,
            ["", "", ""] + [str(i) for i in range(20)],
        ]
        for md in range(12):
            prefix = [str(slot), name, f"MD{md}"] if md == 0 else ["", "", f"MD{md}"]
            vectors = []
            for wp in self.rows:
                if md in ACTIVE:
                    vectors.append(f'{wp["angles"][md]:.3f},{wp["duration"]},{wp["s0"]},{wp["sd"]}')
                else: vectors.append("-")
            vectors += ["-"] * (20 - len(vectors))
            rows.append(prefix + vectors)
        with csv_path.open("w", newline="") as handle: csv.writer(handle, lineterminator="\n").writerows(rows)
        digest = hashlib.sha256(csv_path.read_bytes()).hexdigest()
        memo = {
            "zero_snapshot": {"format_version": 2, "robot_uid": PROFILE["robot_uid"],
                              "known_mask": PROFILE["known_mask"],
                              "zero_offset_f32_le_hex": PROFILE["zero_offset_f32_le_hex"]},
            "teaching_start_angles_rad": [None] * 12,
            "motion_sha256": digest, "schema": 2, "slot_id": slot,
            "updated": datetime.now().astimezone().isoformat(timespec="seconds")}
        memo_path.write_text(json.dumps(memo, indent=2) + "\n")
        return csv_path, memo_path, digest


def main():
    root = tk.Tk(); Builder(root); root.mainloop()


if __name__ == "__main__": main()
