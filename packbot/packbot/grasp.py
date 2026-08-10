"""토크 기반 파지 성공 판정.

물체를 들어올린 직후 리프트 관절의 토크를 짧게 샘플링해서,
빈손 기준치(baseline)보다 delta_threshold 이상 높으면 "잡았다"고 판정한다.
"""

import time


class GraspChecker:
    def __init__(self, cfg, torque_backend, log=print):
        self.joint = int(cfg["lift_joint"])
        self.baseline = cfg.get("baseline_torque")
        self.delta = float(cfg["delta_threshold"])
        self.settle = float(cfg["settle_seconds"])
        self.sample_sec = float(cfg["sample_seconds"])
        self.hz = float(cfg["sample_hz"])
        self.torque = torque_backend
        self.log = log

        if self.baseline is None:
            raise ValueError(
                "grasp.baseline_torque 가 설정되지 않았습니다.\n"
                "  python3 tools/calibrate_torque.py 로 빈손 토크를 측정하세요."
            )

    def sample_torque(self):
        """settle 후 sample_seconds 동안 토크 평균."""
        time.sleep(self.settle)
        vals = []
        interval = 1.0 / self.hz
        end = time.time() + self.sample_sec
        while time.time() < end:
            t0 = time.time()
            reading = self.torque.read()
            if self.joint in reading:
                vals.append(reading[self.joint])
            dt = interval - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
        if not vals:
            raise RuntimeError(f"관절 {self.joint} 토크를 한 번도 읽지 못함")
        return sum(vals) / len(vals)

    def holding(self):
        """물체를 잡고 있으면 True."""
        avg = self.sample_torque()
        diff = avg - float(self.baseline)
        ok = diff >= self.delta
        self.log(
            f"  토크 판정: 평균 {avg:.3f} (기준 {self.baseline:.3f} "
            f"+{diff:+.3f}, 문턱 {self.delta:.3f}) -> {'잡음' if ok else '빈손'}"
        )
        return ok
