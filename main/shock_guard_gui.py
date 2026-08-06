#!/usr/bin/env python3
"""Motion-slot GUI with DOB based impact cancellation.

This is an operational stop helper, not an emergency-stop implementation.
"""

import queue
import math
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from agx_msgs.action import PlayMotionSequence
from agx_msgs.msg import MotionSlotState, PhorceFeedback
from agx_msgs.srv import ListMotionSlots


ACTION = "/motion_action_server/play_motion_sequence"
FEEDBACK = "/phorce/feedback"
CATALOG_SERVICE = "/motion_action_server/list_motion_slots"
MOTION_ALIASES = {
    "grab": ("GRAB",),
    "box": ("BOX", "BOX_TEST"),
    "tape": ("TAPE",),
    "home": ("HOME", "RETURN_HOME", "GO_HOME"),
}
# /phorce/feedback.position_rad reference poses from operator captures.
# HOME_ZERO_RAD is the unprepared home-zero pose; BUTTON1_READY_RAD is the pose
# after the physical button-1 sequence. Their separation is only up to 0.006
# rad in this feedback coordinate, so pose alone is not a reliable classifier.
HOME_ZERO_RAD = (
    -1.773, 1.034, 2.840,
    0.0, 0.0, 0.0,
    2.562, 1.746, 1.550,
    0.0, 0.0, 0.0,
)
BUTTON1_READY_RAD = (
    -1.776, 1.034, 2.837,
    0.0, 0.0, 0.0,
    2.568, 1.749, 1.549,
    0.0, 0.0, 0.0,
)
# The table uses the unprepared physical home pose as its displayed zero.
HOME_OFFSETS_RAD = HOME_ZERO_RAD
ACTIVE_AXES = (0, 1, 2, 6, 7, 8)
HOME_READY_TOLERANCE_RAD = 0.12
GAIN_ACTIVE_EPSILON = 0.01
HOME_READY_SAMPLES = 200  # 1 kHz feedback에서 약 0.2초 연속 확인
BUTTON1_MOVE_THRESHOLD_RAD = 0.03
BUTTON1_MOVE_VELOCITY_RAD_S = 0.05
BUTTON1_SETTLE_VELOCITY_RAD_S = 0.03
BACK_HOME_TOLERANCE_RAD = 0.05
BACK_HOME_SAMPLES = 200


def circular_difference_rad(position_rad, reference_rad):
    delta = position_rad - reference_rad
    return math.atan2(math.sin(delta), math.cos(delta))


def home_relative_rad(position_rad, axis_index):
    return circular_difference_rad(position_rad, HOME_OFFSETS_RAD[axis_index])


class GuardNode(Node):
    def __init__(self):
        super().__init__("shock_guard_gui")
        self.client = ActionClient(self, PlayMotionSequence, ACTION)
        self.catalog_client = self.create_client(ListMotionSlots, CATALOG_SERVICE)
        self.create_subscription(PhorceFeedback, FEEDBACK, self._on_feedback,
                                 qos_profile_sensor_data)
        self.create_subscription(MotionSlotState,
                                 "/motion_action_server/motion_slot_state",
                                 self._on_motion_state, 10)
        self.create_timer(0.05, self._process_commands)
        self.create_timer(1.0, self._refresh_catalog)
        self.commands = queue.SimpleQueue()
        self.lock = threading.Lock()
        self.axes = []
        self.last_frame = 0.0
        self.goal_handle = None
        self.motion_state = "대기"
        self.event = "피드백 대기 중"
        self.threshold = 2.0
        self.required_hits = 3
        self.hits = 0
        self.armed = False
        self.cancel_pending = False
        self.catalog_request = None
        self.motion_catalog = {}
        self.catalog_status = "PCM 모션 목록 대기 중"
        self.motion_ready_status = "확인 중"
        self.motion_ready_seen = False
        self.peak_axis = -1
        self.peak_dob = 0.0
        self.home_ready_samples = 0
        self.home_feedback_status = "피드백 대기 중"
        self.operation_mode = "NOT OP"
        self.op_seen = False
        self.button1_motion_seen = False
        self.op_away_seen = False
        self.back_home_samples = 0

    def request_play(self, motion_id, threshold, required_hits):
        self.commands.put(("play", motion_id, threshold, required_hits))

    def request_stop(self):
        self.commands.put(("stop",))

    def snapshot(self):
        with self.lock:
            return (list(self.axes), self.last_frame, self.motion_state,
                    self.event, self.armed, self.hits, self.threshold,
                    dict(self.motion_catalog), self.catalog_status,
                    self.motion_ready_status, self.peak_axis, self.peak_dob,
                    self.home_feedback_status, self.operation_mode)

    def _on_motion_state(self, message):
        if message.recovery_required:
            status = "복구 필요 — 2번 파킹 후 1번 버튼"
            self.motion_ready_seen = False
        elif message.contract_active and message.physical_idle:
            status = "모션 수신 준비 완료"
            self.motion_ready_seen = True
        elif self.motion_ready_seen:
            status = "현재 모션 실행 중"
        elif message.contract_active:
            status = "준비 안 됨 — 필요하면 1번 버튼을 0.6초 누르세요"
        else:
            status = "PCM 상태 계약 비활성"
        with self.lock:
            self.motion_ready_status = status

    @staticmethod
    def _normalized_name(name):
        return "_".join(name.strip().upper().replace("-", "_").split())

    def _refresh_catalog(self):
        if self.catalog_request is not None or not self.catalog_client.service_is_ready():
            return
        self.catalog_request = self.catalog_client.call_async(ListMotionSlots.Request())
        self.catalog_request.add_done_callback(self._on_catalog)

    def _on_catalog(self, future):
        self.catalog_request = None
        try:
            response = future.result()
        except Exception as exc:
            with self.lock:
                self.catalog_status = f"카탈로그 조회 실패: {exc}"
            return
        mapped = {}
        if response.library_loaded:
            normalized = [(slot.id, slot.name, self._normalized_name(slot.name))
                          for slot in response.slots]
            for key, aliases in MOTION_ALIASES.items():
                matches = [(slot_id, name) for slot_id, name, norm in normalized
                           if norm in aliases]
                if len(matches) == 1:
                    mapped[key] = matches[0]
        with self.lock:
            self.motion_catalog = mapped
            found = ", ".join(f"{key}={value[0]}" for key, value in mapped.items())
            self.catalog_status = f"카탈로그 매핑: {found}" if found else "grab/box/tape/HOME 이름과 일치하는 슬롯 없음"

    def _set_status(self, state=None, event=None):
        with self.lock:
            if state is not None:
                self.motion_state = state
            if event is not None:
                self.event = event

    def _process_commands(self):
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            if command[0] == "play":
                self._play(*command[1:])
            elif command[0] == "stop":
                self._cancel("사용자 정지 요청")

    def _play(self, motion_id, threshold, required_hits):
        if self.goal_handle is not None:
            self._set_status(event="이미 활성 모션이 있습니다")
            return
        if not self.client.server_is_ready():
            self._set_status(state="오류", event="액션 서버가 준비되지 않았습니다")
            return
        self.threshold = threshold
        self.required_hits = required_hits
        self.hits = 0
        self.cancel_pending = False
        goal = PlayMotionSequence.Goal()
        goal.motion_ids = [motion_id]
        goal.stop_on_error = True
        self._set_status(state="요청 중", event=f"모션 {motion_id} 전송")
        future = self.client.send_goal_async(goal, feedback_callback=self._on_action_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._set_status(state="오류", event=f"Goal 전송 실패: {exc}")
            return
        if not handle.accepted:
            self._set_status(state="거절", event="액션 서버가 Goal을 거절했습니다")
            return
        self.goal_handle = handle
        self.armed = True
        self._set_status(state="실행 중", event="충격 감시 활성화")
        result = handle.get_result_async()
        result.add_done_callback(self._on_result)

    def _on_action_feedback(self, message):
        motion_id = message.feedback.current_motion_id
        if motion_id:
            self._set_status(state=f"모션 {motion_id} 실행 중")

    def _on_result(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            labels = {0: "완료", 1: "거절", 2: "중단", 3: "취소"}
            state = labels.get(result.status, f"결과 {result.status}")
            detail = result.detail or state
        except Exception as exc:
            state, detail = "오류", f"결과 수신 실패: {exc}"
        self.goal_handle = None
        self.armed = False
        self.cancel_pending = False
        self.hits = 0
        self._set_status(state=state, event=detail)

    def _cancel(self, reason):
        self.armed = False
        if self.goal_handle is None:
            self._set_status(event="취소할 활성 모션이 없습니다")
            return
        if self.cancel_pending:
            return
        self.cancel_pending = True
        self._set_status(state="정지 요청 중", event=reason)
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future):
        try:
            accepted = bool(future.result().goals_canceling)
            event = "취소 요청 수락 — 실제 정지 결과 대기" if accepted else "취소 요청이 거부되었습니다"
        except Exception as exc:
            event = f"취소 요청 실패: {exc}"
        self._set_status(event=event)

    def _on_feedback(self, message):
        now = time.monotonic()
        values = []
        peak = 0.0
        peak_axis = -1
        for index, axis in enumerate(message.axis):
            if axis.valid:
                value = abs(axis.dob_a)
                home_position = home_relative_rad(axis.position_rad, index)
                values.append((index, home_position, axis.dob_a,
                               axis.current_a, True))
                if value > peak:
                    peak, peak_axis = value, index
            else:
                values.append((index, 0.0, 0.0, 0.0, False))
        with self.lock:
            self.axes = values
            self.last_frame = now
            self.peak_axis = peak_axis
            self.peak_dob = peak
        valid_home_axes = [message.axis[i] for i in ACTIVE_AXES
                           if message.axis[i].valid]
        if len(valid_home_axes) != len(ACTIVE_AXES):
            self.home_ready_samples = 0
            home_status = f"판정 불가 — 유효 축 {len(valid_home_axes)}/{len(ACTIVE_AXES)}"
        else:
            gain_axes = sum(
                1 for axis in valid_home_axes
                if abs(axis.kp_echo) > GAIN_ACTIVE_EPSILON or
                abs(axis.kd_echo) > GAIN_ACTIVE_EPSILON)
            max_home_error = max(
                abs(home_relative_rad(message.axis[i].position_rad, i))
                for i in ACTIVE_AXES)
            max_zero_error = max(
                abs(circular_difference_rad(
                    message.axis[i].position_rad, HOME_ZERO_RAD[i]))
                for i in ACTIVE_AXES)
            max_velocity = max(abs(message.axis[i].velocity_rad_s)
                               for i in ACTIVE_AXES)
            gain_ready = gain_axes == len(ACTIVE_AXES)

            # Button 1 has no public contact signal. Require observing its
            # physical sequence after GUI startup: servo gain plus either
            # leaving the home pose or measurable joint velocity.
            if self.operation_mode in ("NOT OP", "BACK TO HOME"):
                if gain_ready and (
                        max_zero_error >= BUTTON1_MOVE_THRESHOLD_RAD or
                        max_velocity >= BUTTON1_MOVE_VELOCITY_RAD_S):
                    self.button1_motion_seen = True
                if self.button1_motion_seen and gain_ready:
                    if max_velocity <= BUTTON1_SETTLE_VELOCITY_RAD_S:
                        self.home_ready_samples = min(
                            self.home_ready_samples + 1, HOME_READY_SAMPLES)
                    else:
                        self.home_ready_samples = 0
                    if self.home_ready_samples >= HOME_READY_SAMPLES:
                        home_status = ("1번 버튼 완료 추정 — 움직임 관측 후 안정, "
                                       f"gain {gain_axes}/{len(ACTIVE_AXES)}")
                    else:
                        home_status = ("1번 버튼 동작 관측 — 안정 확인 "
                                       f"{self.home_ready_samples}/{HOME_READY_SAMPLES}")
                else:
                    self.home_ready_samples = 0
                    home_status = ("1번 버튼 필요 — 홈 기준 대기, "
                                   f"gain {gain_axes}/{len(ACTIVE_AXES)}")
            else:
                home_status = ("OP 유지 — 모션 수행 가능, "
                               f"gain {gain_axes}/{len(ACTIVE_AXES)}")

            ready_confirmed = self.button1_motion_seen and gain_ready and (
                self.home_ready_samples >= HOME_READY_SAMPLES)
            # Explicit operating sequence:
            # NOT OP --button 1--> OP --motions--> OP --button 2--> BACK TO HOME
            # Motion position must not demote OP; only gain release at the
            # measured home-zero pose is treated as the button-2 transition.
            if self.operation_mode == "NOT OP":
                operation_mode = "OP" if ready_confirmed else "NOT OP"
            elif self.operation_mode == "OP":
                if max_zero_error > BACK_HOME_TOLERANCE_RAD:
                    self.op_away_seen = True
                    self.back_home_samples = 0
                back_home_candidate = (
                    self.goal_handle is None and
                    max_zero_error <= BACK_HOME_TOLERANCE_RAD and
                    max_velocity <= BUTTON1_SETTLE_VELOCITY_RAD_S and
                    (self.op_away_seen or gain_axes != len(ACTIVE_AXES)))
                if back_home_candidate:
                    self.back_home_samples = min(
                        self.back_home_samples + 1, BACK_HOME_SAMPLES)
                else:
                    self.back_home_samples = 0
                if self.back_home_samples >= BACK_HOME_SAMPLES:
                    operation_mode = "BACK TO HOME"
                    self.button1_motion_seen = False
                    self.home_ready_samples = 0
                    self.op_away_seen = False
                    home_status = ("2번 버튼 완료 추정 — 홈 영점 복귀 안정, "
                                   f"gain {gain_axes}/{len(ACTIVE_AXES)}")
                else:
                    operation_mode = "OP"
                    if back_home_candidate:
                        home_status = ("홈 복귀 안정 확인 "
                                       f"{self.back_home_samples}/{BACK_HOME_SAMPLES}")
            else:  # BACK TO HOME
                operation_mode = "OP" if ready_confirmed else "BACK TO HOME"
                if operation_mode == "BACK TO HOME" and not self.button1_motion_seen:
                    home_status = ("홈 영점 복귀 상태 — 다음 운전 전 "
                                   "1번 버튼이 필요합니다")
        with self.lock:
            self.home_feedback_status = home_status
            if len(valid_home_axes) == len(ACTIVE_AXES):
                self.operation_mode = operation_mode
        if not self.armed or self.cancel_pending:
            self.hits = 0
            return
        if peak >= self.threshold:
            self.hits += 1
            if self.hits >= self.required_hits:
                self._cancel(f"충격 감지: 축 {peak_axis}, |DOB|={peak:.2f} A")
        else:
            self.hits = 0


class GuardApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        root.title("G-SHOCK Motion Guard")
        # Window decorations extend beyond the Tk client area. Keep a small gap
        # at the center so the GUI border never covers the right-hand terminal.
        width = max(650, root.winfo_screenwidth() // 2 - 24)
        height = root.winfo_screenheight()
        root.geometry(f"{width}x{height}+0+0")
        root.minsize(650, 650)

        style = ttk.Style()
        style.configure("Title.TLabel", font=("Sans", 20, "bold"))
        style.configure("State.TLabel", font=("Sans", 15, "bold"))

        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        header = ttk.Frame(frame)
        header.pack(fill="x")
        header_text = ttk.Frame(header)
        header_text.pack(side="left", fill="x", expand=True)
        ttk.Label(header_text, text="외부 충격 모션 가드", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header_text, text="DOB 외란 추정값을 감시해 활성 모션에 취소 요청을 보냅니다.").pack(anchor="w", pady=(2, 14))
        self.mode_badge = tk.Label(header, text="NOT OP", bg="#8b1a1a", fg="white",
                                   font=("Sans", 13, "bold"), width=15,
                                   relief="solid", borderwidth=2, padx=10, pady=8)
        self.mode_badge.pack(side="right", anchor="ne", padx=(12, 0))

        controls = ttk.LabelFrame(frame, text="실행 설정", padding=12)
        controls.pack(fill="x")
        self.threshold = tk.StringVar(value="2.0")
        self.hits = tk.StringVar(value="3")
        self.manual_motion_id = tk.StringVar(value="1")
        for col, (label, variable) in enumerate((("DOB 임계값 (A)", self.threshold),
                                                  ("연속 프레임", self.hits))):
            ttk.Label(controls, text=label).grid(row=0, column=col, sticky="w", padx=5)
            ttk.Entry(controls, textvariable=variable, width=14).grid(row=1, column=col, padx=5, pady=5)
        ttk.Button(controls, text="소프트웨어 정지 요청", command=node.request_stop).grid(row=1, column=2, sticky="ew", padx=5, pady=5)

        motion_buttons = ttk.Frame(controls)
        motion_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(8, 2))
        self.motion_buttons = {}
        for col, (key, label) in enumerate((("grab", "GRAB"), ("box", "BOX"),
                                            ("tape", "TAPE"), ("home", "원점 복귀"))):
            button = ttk.Button(motion_buttons, text=label,
                                command=lambda selected=key: self.play_motion(selected),
                                state="disabled")
            button.grid(row=0, column=col, sticky="ew", padx=3)
            motion_buttons.columnconfigure(col, weight=1)
            self.motion_buttons[key] = button
        self.catalog_label = ttk.Label(controls, text="PCM 모션 목록 대기 중")
        self.catalog_label.grid(row=3, column=0, columnspan=3, sticky="w", padx=5, pady=(4, 0))
        manual = ttk.Frame(controls)
        manual.grid(row=4, column=0, columnspan=3, sticky="ew", padx=5, pady=(8, 0))
        ttk.Label(manual, text="모션 ID 직접 실행").pack(side="left")
        ttk.Entry(manual, textvariable=self.manual_motion_id, width=7).pack(side="left", padx=6)
        ttk.Button(manual, text="ID 실행", command=self.play_manual_id).pack(side="left")

        status = ttk.LabelFrame(frame, text="상태", padding=12)
        status.pack(fill="x", pady=12)
        self.state_label = ttk.Label(status, text="대기", style="State.TLabel")
        self.state_label.pack(anchor="w")
        self.event_label = ttk.Label(status, text="피드백 대기 중")
        self.event_label.pack(anchor="w", pady=4)
        self.fresh_label = ttk.Label(status, text="")
        self.fresh_label.pack(anchor="w")
        self.button_input_label = ttk.Label(
            status, text="운영 절차 추정: 피드백 대기 중")
        self.button_input_label.pack(anchor="w", pady=(4, 0))
        self.ready_label = ttk.Label(status, text="PCM 모션 준비: 확인 중")
        self.ready_label.pack(anchor="w", pady=(2, 0))
        self.motor_label = ttk.Label(status, text="감지 모터 ID: -")
        self.motor_label.pack(anchor="w", pady=(2, 0))

        axes_box = ttk.LabelFrame(frame, text="축별 실시간 값", padding=10)
        axes_box.pack(fill="both", expand=True)
        ttk.Label(axes_box, text="각도 표시: 시작 홈 영점 오프셋 적용됨 (홈 기준 최단 각도차)").pack(anchor="w", pady=(0, 5))
        self.tree = ttk.Treeview(axes_box, columns=("valid", "position", "dob", "current", "bar"),
                                 show="headings", height=12)
        for key, title, size in (("valid", "유효", 55), ("position", "홈 기준 각도 (rad)", 125),
                                 ("dob", "DOB (A)", 90), ("current", "전류 (A)", 90),
                                 ("bar", "임계값 대비", 210)):
            self.tree.heading(key, text=title)
            self.tree.column(key, width=size, anchor="center")
        self.tree.pack(fill="both", expand=True)
        for i in range(12):
            self.tree.insert("", "end", iid=str(i), values=("-", "-", "-", "-", ""))

        shutdown_row = ttk.Frame(frame)
        shutdown_row.pack(fill="x", pady=(8, 0))
        tk.Button(shutdown_row, text="전체 종료",
                  command=self.close, bg="#343434", fg="white",
                  activebackground="#555555", activeforeground="white",
                  font=("Sans", 9, "bold"), padx=8, pady=3).pack(side="right")

        warning = tk.Label(frame, text="주의: 액션 cancel은 E-Stop이 아닙니다. 위험 시 반드시 물리 E-Stop을 사용하세요.",
                           bg="#7a1111", fg="white", font=("Sans", 11, "bold"), padx=10, pady=9)
        warning.pack(fill="x", pady=(12, 0))
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.refresh)

    def play_motion(self, motion_key):
        try:
            threshold = float(self.threshold.get())
            hits = int(self.hits.get())
            if threshold <= 0 or hits < 1:
                raise ValueError
        except ValueError:
            messagebox.showerror("입력 오류", "양수 임계값과 연속 프레임 1 이상을 입력하세요.")
            return
        catalog = self.node.snapshot()[7]
        if motion_key not in catalog:
            messagebox.showerror("모션 없음", f"PCM 카탈로그에서 {motion_key} 모션을 찾지 못했습니다.")
            return
        motion_id, _name = catalog[motion_key]
        self.node.request_play(motion_id, threshold, hits)

    def play_manual_id(self):
        try:
            motion_id = int(self.manual_motion_id.get())
            threshold = float(self.threshold.get())
            hits = int(self.hits.get())
            if not 1 <= motion_id <= 50 or threshold <= 0 or hits < 1:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "입력 오류",
                "모션 ID 1~50, 양수 임계값, 연속 프레임 1 이상을 입력하세요.")
            return
        self.node.request_play(motion_id, threshold, hits)

    def refresh(self):
        (axes, last_frame, state, event, armed, hits, threshold,
         catalog, catalog_status, ready_status, peak_axis, peak_dob,
         home_feedback_status, operation_mode) = self.node.snapshot()
        self.state_label.configure(text=f"{state} · 충격 감시 {'ON' if armed else 'OFF'}")
        self.event_label.configure(text=event)
        age = time.monotonic() - last_frame if last_frame else 999.0
        self.fresh_label.configure(text=f"피드백: {'정상' if age < 0.2 else '끊김'} · 임계 연속 {hits}회")
        self.ready_label.configure(text=f"PCM 모션 준비: {ready_status}")
        self.button_input_label.configure(text=f"운영 절차 추정: {home_feedback_status}")
        badge_colors = {
            "NOT OP": ("#8b1a1a", "white"),
            "OP": ("#19713a", "white"),
            "BACK TO HOME": ("#d47a00", "white"),
        }
        bg, fg = badge_colors.get(operation_mode, ("#555555", "white"))
        self.mode_badge.configure(text=operation_mode, bg=bg, fg=fg)
        motor_text = "-" if peak_axis < 0 else f"{peak_axis}  (|DOB| {peak_dob:.3f} A)"
        self.motor_label.configure(text=f"감지 모터 ID(축 index): {motor_text}")
        self.catalog_label.configure(text=catalog_status)
        for key, button in self.motion_buttons.items():
            if key in catalog:
                motion_id, name = catalog[key]
                label = "원점 복귀" if key == "home" else key.upper()
                button.configure(text=f"{label}  [{motion_id}: {name}]", state="normal")
            else:
                button.configure(state="disabled")
        for index, position, dob, current, valid in axes:
            ratio = min(abs(dob) / threshold, 1.0) if threshold else 0
            bar = "█" * int(ratio * 20)
            self.tree.item(str(index), values=("OK" if valid else "무효",
                                                f"{position:+.3f}" if valid else "-",
                                                f"{dob:+.3f}" if valid else "-",
                                                f"{current:+.3f}" if valid else "-", bar))
        self.root.after(100, self.refresh)

    def close(self):
        self.node.request_stop()
        self.root.after(150, self.root.destroy)


def main():
    rclpy.init()
    node = GuardNode()
    root = tk.Tk()
    app = GuardApp(root, node)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
