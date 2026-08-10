"""픽앤플레이스 상태머신.

  DETECT  - 프레임 캡처 -> 물체 검출 -> 구역 판정
  PICK    - 해당 구역 티칭 모션 재생
  VERIFY  - 토크로 파지 확인. 실패 시 복귀 모션 후 DETECT로 (요구사항)
  PLACE   - 박스 투입 모션 재생
  IDLE    - 작업공간에 물체 없음

파지 실패가 max_retries 를 넘으면 해당 사이클을 포기하고 IDLE로 간다.
(같은 물체에 무한 반복하는 것을 막는다. 물체가 집기 불가능한 자세일 수 있음)
"""

import enum
import time


class State(enum.Enum):
    DETECT = "detect"
    PICK = "pick"
    VERIFY = "verify"
    PLACE = "place"
    IDLE = "idle"


class PickCycle:
    def __init__(self, workspace, detector, motion, grasp_checker, cfg, log=print):
        self.ws = workspace
        self.det = detector
        self.motion = motion
        self.grasp = grasp_checker
        self.cfg = cfg
        self.log = log

        self.zone_motions = {int(k): v for k, v in cfg["motion"]["zone_motions"].items()}
        self.place_motion = cfg["motion"]["place_motion"]
        self.recover_motion = cfg["motion"]["recover_motion"]
        self.max_retries = int(cfg["grasp"]["max_retries"])
        self.retry_delay = float(cfg["grasp"]["retry_delay"])

        self.state = State.DETECT
        self.retries = 0
        self.stats = {"picked": 0, "retried": 0, "given_up": 0}

        # 디버그 표시용 - 마지막 검출 결과
        self.last_warped = None
        self.last_detections = []
        self.last_target = None
        self.last_zone = None

    def step(self, frame):
        """프레임 한 장을 받아 상태를 한 단계 진행한다.

        DETECT/IDLE 은 매 프레임 호출되고, PICK/VERIFY/PLACE 는
        모션 재생 동안 블로킹된다 (그 사이 프레임은 버려도 된다).
        """
        if self.state in (State.DETECT, State.IDLE):
            self._detect(frame)
        elif self.state == State.PICK:
            self._pick()
        elif self.state == State.VERIFY:
            self._verify()
        elif self.state == State.PLACE:
            self._place()
        return self.state

    # ------------------------------------------------------------------
    def _detect(self, frame):
        warped = self.ws.warp(frame)
        dets = self.det.detect_all(warped)

        self.last_warped = warped
        self.last_detections = dets
        self.last_target = dets[0] if dets else None
        self.last_zone = None

        if not dets:
            self.state = State.IDLE
            return

        target = dets[0]
        zone = self.ws.zone_of(target.center)
        if zone is None:
            # 중심이 경계 밖 - 다음 프레임에서 다시
            self.state = State.DETECT
            return

        self.last_zone = zone
        self.log(
            f"물체 검출: 구역 {zone}, 중심 ({target.cx:.0f},{target.cy:.0f}), "
            f"면적 {target.area:.0f}, 장축각 {target.angle:.0f}도"
        )
        self.state = State.PICK

    def _pick(self):
        zone = self.last_zone
        name = self.zone_motions.get(zone)
        if name is None:
            self.log(f"경고: 구역 {zone} 에 티칭 모션이 없음 - 사이클 건너뜀")
            self.state = State.IDLE
            return
        self.log(f"픽 모션 재생: {name} (구역 {zone})")
        self.motion.play(name)
        self.state = State.VERIFY

    def _verify(self):
        if self.grasp.holding():
            self.retries = 0
            self.state = State.PLACE
            return

        # --- 파지 실패: 복귀 후 검출부터 다시 (요구사항) ---
        self.retries += 1
        self.stats["retried"] += 1
        self.log(f"파지 실패 ({self.retries}/{self.max_retries}) - 복귀 후 재검출")
        self.motion.play(self.recover_motion)

        if self.retries >= self.max_retries:
            self.log("재시도 한도 초과 - 이 물체는 포기하고 대기")
            self.stats["given_up"] += 1
            self.retries = 0
            self.state = State.IDLE
            return

        time.sleep(self.retry_delay)   # 물체가 튕겨 굴러갔으면 멈출 때까지
        self.state = State.DETECT

    def _place(self):
        self.log(f"박스 투입: {self.place_motion}")
        self.motion.play(self.place_motion)
        self.stats["picked"] += 1
        self.log(
            f"완료 (누적 성공 {self.stats['picked']}, "
            f"재시도 {self.stats['retried']}, 포기 {self.stats['given_up']})"
        )
        self.state = State.DETECT
