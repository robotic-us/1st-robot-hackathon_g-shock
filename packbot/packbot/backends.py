"""로봇 제어기 어댑터.

이미 동작하는 티칭/재생 모듈과 토크 읽기를 여기서만 감싼다.
비전과 상태머신은 이 인터페이스만 알고 있으므로, 실제 제어기가 무엇이든
아래 두 클래스만 구현하면 나머지 코드는 그대로 돌아간다.

  MotionBackend - 티칭된 모션 이름을 받아 재생하고, 끝날 때까지 기다린다
  TorqueBackend - 관절별 토크 값을 읽는다

>>> 실제 제어기 연동은 SerialMotionBackend / SerialTorqueBackend 의
>>> TODO 표시 부분에 프로토콜을 채워 넣으면 된다.
"""

import time
from abc import ABC, abstractmethod


# --------------------------------------------------------------------------
# 인터페이스
# --------------------------------------------------------------------------

class MotionBackend(ABC):
    @abstractmethod
    def play(self, motion_name, timeout=30.0):
        """티칭된 모션을 재생하고 완료될 때까지 블로킹. 성공하면 True."""

    @abstractmethod
    def stop(self):
        """진행 중인 모션을 즉시 중단."""

    def close(self):
        pass


class TorqueBackend(ABC):
    @abstractmethod
    def read(self):
        """관절 인덱스 -> 토크 값 딕셔너리를 반환."""

    def close(self):
        pass


# --------------------------------------------------------------------------
# 더미 구현 - 로봇 없이 비전/상태머신만 시험할 때 사용
# --------------------------------------------------------------------------

class DummyMotionBackend(MotionBackend):
    def __init__(self, duration=0.8, log=print):
        self.duration = duration
        self.log = log
        self.history = []

    def play(self, motion_name, timeout=30.0):
        self.log(f"  [dummy] 모션 재생: {motion_name}")
        self.history.append(motion_name)
        time.sleep(self.duration)
        return True

    def stop(self):
        self.log("  [dummy] 모션 중단")


class DummyTorqueBackend(TorqueBackend):
    """파지 성공/실패를 번갈아 흉내 내서 재시도 경로를 시험한다."""

    def __init__(self, joint=1, baseline=1.0, loaded=1.4, fail_every=3):
        self.joint = joint
        self.baseline = baseline
        self.loaded = loaded
        self.fail_every = fail_every
        self.n = 0

    def read(self):
        self.n += 1
        failed = self.fail_every > 0 and self.n % self.fail_every == 0
        return {self.joint: self.baseline if failed else self.loaded}


# --------------------------------------------------------------------------
# 시리얼 구현 - 실제 제어기 연동용 뼈대
# --------------------------------------------------------------------------

class SerialMotionBackend(MotionBackend):
    """시리얼로 티칭 모션 재생을 명령하는 어댑터.

    TODO: 실제 제어기 프로토콜에 맞춰 아래 세 곳을 채운다.
      1. _encode_play  - 모션 재생 명령 바이트 만들기
      2. _is_done      - 완료 응답 판별
      3. _encode_stop  - 정지 명령
    """

    def __init__(self, port, baudrate=115200, timeout=2.0):
        try:
            import serial
        except ImportError as e:
            raise RuntimeError(
                "pyserial이 설치되어 있지 않습니다: pip3 install pyserial"
            ) from e
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2.0)  # MCU 리셋 대기 (아두이노 계열은 DTR로 리부팅된다)
        self.ser.reset_input_buffer()

    def _encode_play(self, motion_name):
        # TODO: 실제 명령 형식으로 교체
        return f"PLAY {motion_name}\n".encode()

    def _encode_stop(self):
        # TODO: 실제 명령 형식으로 교체
        return b"STOP\n"

    def _is_done(self, line):
        # TODO: 실제 완료 응답으로 교체
        return line.strip().upper() in (b"DONE", b"OK")

    def play(self, motion_name, timeout=30.0):
        self.ser.reset_input_buffer()
        self.ser.write(self._encode_play(motion_name))
        self.ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.ser.readline()
            if not line:
                continue
            if self._is_done(line):
                return True
        raise TimeoutError(f"모션 '{motion_name}' 완료 응답이 {timeout}초 안에 오지 않음")

    def stop(self):
        self.ser.write(self._encode_stop())
        self.ser.flush()

    def close(self):
        self.ser.close()


class SerialTorqueBackend(TorqueBackend):
    """시리얼로 관절 토크를 읽는 어댑터.

    TODO: _parse 를 실제 응답 형식에 맞춘다.
    모션 백엔드와 같은 포트를 쓴다면 SerialMotionBackend 의 serial 객체를
    share_with 로 넘겨서 포트를 공유한다 (같은 포트를 두 번 열 수 없다).
    """

    def __init__(self, port=None, baudrate=115200, timeout=1.0, share_with=None):
        if share_with is not None:
            self.ser = share_with.ser
        else:
            try:
                import serial
            except ImportError as e:
                raise RuntimeError(
                    "pyserial이 설치되어 있지 않습니다: pip3 install pyserial"
                ) from e
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self._owns = share_with is None

    def _parse(self, line):
        # TODO: 실제 응답 형식으로 교체
        # 예시로 "TORQUE 0:1.02 1:1.44 2:0.31" 형태를 가정한다
        text = line.decode(errors="ignore").strip()
        if not text.upper().startswith("TORQUE"):
            return None
        out = {}
        for tok in text.split()[1:]:
            if ":" not in tok:
                continue
            j, v = tok.split(":", 1)
            try:
                out[int(j)] = float(v)
            except ValueError:
                continue
        return out or None

    def read(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"GET_TORQUE\n")  # TODO: 실제 요청 명령으로 교체
        self.ser.flush()

        deadline = time.time() + 1.0
        while time.time() < deadline:
            line = self.ser.readline()
            if not line:
                continue
            parsed = self._parse(line)
            if parsed is not None:
                return parsed
        raise TimeoutError("토크 응답 없음")

    def close(self):
        if self._owns:
            self.ser.close()


# --------------------------------------------------------------------------

def build_backends(cfg, log=print):
    """설정에 따라 모션/토크 백엔드 쌍을 만든다."""
    kind = cfg["motion"].get("backend", "dummy")

    if kind == "dummy":
        log("백엔드: dummy (로봇 없이 비전만 시험)")
        return DummyMotionBackend(log=log), DummyTorqueBackend(
            joint=cfg["grasp"]["lift_joint"]
        )

    if kind == "serial":
        m = cfg["motion"]
        log(f"백엔드: serial ({m['port']} @ {m['baudrate']})")
        motion = SerialMotionBackend(m["port"], m["baudrate"], m["timeout"])
        torque = SerialTorqueBackend(share_with=motion)
        return motion, torque

    raise ValueError(f"알 수 없는 backend: {kind}")
