#!/usr/bin/env python3
"""PCM USB-CDC servo control used to replace the physical button-1 action.

The protocol was recovered from phorce-studio.exe.  The public entry point is
``PcmUsbServoClient.request_servo_on()``; callers must still provide their own
human confirmation because a successful command can move the robot to its
configured boot posture.
"""

from __future__ import annotations

import fcntl
import glob
import os
import secrets
import select
import struct
import subprocess
import termios
import threading
import time
from dataclasses import dataclass
from typing import Optional


PCM_USB_VID = "0483"
PCM_USB_PID = "5741"
PCM_DOP_NODE_ID = 0x51
PCM_BAUDRATE = 921600

MSG_SDO_REQ = 0x01
MSG_SDO_RSP = 0x02
SDO_READ_CS = 0x40
SDO_WRITE_CS = 0x21

OD_SERVO = 0x5F08
SUB_SERVO_SET = 0x01
SUB_SERVO_STATE = 0x02
SUB_SERVO_RESULT = 0x03
SUB_SERVO_BUILD_ID = 0x06

OD_SESSION = 0x5F0C
SUB_SESSION_HELLO = 0x01
SUB_SESSION_COMMAND = 0x02
SUB_SESSION_STATUS = 0x03

SESSION_ABI_REVISION = 1
SESSION_USB_PROFILE_REVISION = 1
SESSION_MOTION_PROFILE_REVISION = 1
SESSION_COMMAND_GUARD = 827147088
SESSION_CAP_STORAGE = 1 << 0
SESSION_MODE_STUDIO_LIVE = 4
SESSION_PHASE_READY = 5
SESSION_PHASE_REJECTED = 6
SESSION_PHASE_FAULT = 0xFF
SESSION_OPCODE_LIVE_BEGIN = 1
SESSION_FLAG_HOST_RELEASE_CONFIRMED = 1 << 0
SESSION_STATUS_SAFE_PARKING = 1 << 3

SERVO_OFF = 0
SERVO_ON = 1
SERVO_TRANSITION = 2
SERVO_UNAVAILABLE = 3
SERVO_UNCONFIGURED = 4

SERVO_STATE_NAMES = {
    SERVO_OFF: "서보 OFF (파킹)",
    SERVO_ON: "서보 ON — OP 가능",
    SERVO_TRANSITION: "서보 전이 중 — 부팅 자세로 이동",
    SERVO_UNAVAILABLE: "서보 ON 불가 — 부팅/호밍/에러 확인 필요",
    SERVO_UNCONFIGURED: "축 미설정 — PCM Studio에서 활성 축 저장 필요",
}
SERVO_RESULT_NAMES = {
    1: "PCM이 서보 명령을 수리했습니다",
    -1: "현재 PCM 상태에서는 서보를 켤 수 없습니다",
    -2: "USB 호스트가 SD 카드를 사용 중이라 PCM이 거부했습니다",
}


class PcmUsbError(RuntimeError):
    """USB transport or PCM protocol failure."""


@dataclass(frozen=True)
class SdoResponse:
    index: int
    subindex: int
    data: bytes = b""
    is_abort: bool = False
    abort_code: int = 0
    is_write_ack: bool = False


@dataclass(frozen=True)
class PcmUsbSnapshot:
    connected: bool
    port: str
    phase: str
    message: str
    servo_state: Optional[int]
    build_id: Optional[int]


@dataclass(frozen=True)
class SessionHello:
    abi_revision: int
    usb_profile_revision: int
    motion_profile_revision: int
    capability_bits: int
    session_epoch: int


@dataclass(frozen=True)
class SessionStatus:
    abi_revision: int
    mode: int
    phase: int
    accepted_opcode: int
    reason: int
    flags: int
    session_epoch: int
    generation_echo: int
    media_generation: int
    nonce_low: int
    nonce_high: int


def crc16_ccitt(data: bytes) -> int:
    """AGR DOP CRC-16/CCITT: poly 0x1021, init 0xffff, xorout 0."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = (((crc << 1) ^ 0x1021) if crc & 0x8000 else
                   (crc << 1)) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray((0,))
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(byte)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    cursor = 0
    while cursor < len(data):
        code = data[cursor]
        if code == 0:
            raise ValueError("COBS frame contains a zero code")
        cursor += 1
        end = cursor + code - 1
        if end > len(data):
            raise ValueError("truncated COBS frame")
        output.extend(data[cursor:end])
        cursor = end
        if code != 0xFF and cursor < len(data):
            output.append(0)
    return bytes(output)


def build_dop_frame(msg_type: int, payload: bytes, sequence: int,
                    node_id: int = PCM_DOP_NODE_ID) -> bytes:
    raw = struct.pack("<BBH", msg_type, node_id & 0x7F,
                      sequence & 0xFFFF) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + b"\x00"


def build_sdo_write(index: int, subindex: int, data: bytes,
                    sequence: int) -> bytes:
    if not 1 <= len(data) <= 60:
        raise ValueError("SDO write data must be 1..60 bytes")
    payload = struct.pack("<BHBH", SDO_WRITE_CS, index, subindex,
                          len(data)) + data
    return build_dop_frame(MSG_SDO_REQ, payload, sequence)


def build_sdo_read(index: int, subindex: int, sequence: int) -> bytes:
    return build_dop_frame(
        MSG_SDO_REQ, struct.pack("<BHB", SDO_READ_CS, index, subindex),
        sequence)


def parse_dop_frame(encoded: bytes) -> tuple[int, int, int, bytes]:
    raw = cobs_decode(encoded)
    if len(raw) < 6:
        raise ValueError("DOP frame is too short")
    payload_with_header, expected_crc = raw[:-2], struct.unpack("<H", raw[-2:])[0]
    if crc16_ccitt(payload_with_header) != expected_crc:
        raise ValueError("DOP CRC mismatch")
    msg_type, node_id, sequence = struct.unpack("<BBH", payload_with_header[:4])
    return msg_type, node_id, sequence, payload_with_header[4:]


def parse_sdo_response(payload: bytes) -> Optional[SdoResponse]:
    if len(payload) < 4:
        return None
    cs, index, subindex = payload[0], struct.unpack("<H", payload[1:3])[0], payload[3]
    remainder = payload[4:]
    if cs & 0xE0 == 0x80:
        abort_code = struct.unpack("<I", remainder[:4])[0] if len(remainder) >= 4 else 0
        return SdoResponse(index, subindex, is_abort=True,
                           abort_code=abort_code)
    if cs & 0xE0 == 0x60:
        return SdoResponse(index, subindex, is_write_ack=True)
    if cs & 0x03 == 0x03:  # legacy expedited upload
        unused = (cs >> 2) & 0x03
        return SdoResponse(index, subindex, remainder[:4 - unused])
    if len(remainder) >= 2:
        length = struct.unpack("<H", remainder[:2])[0]
        if length <= len(remainder) - 2:
            return SdoResponse(index, subindex, remainder[2:2 + length])
    return SdoResponse(index, subindex)


def parse_session_hello(data: bytes) -> SessionHello:
    if len(data) != struct.calcsize("<HBBII"):
        raise PcmUsbError(f"PCM HELLO 길이가 올바르지 않습니다: {len(data)}")
    hello = SessionHello(*struct.unpack("<HBBII", data))
    if hello.abi_revision != SESSION_ABI_REVISION:
        raise PcmUsbError(
            f"지원하지 않는 PCM 세션 ABI {hello.abi_revision}")
    if hello.usb_profile_revision != SESSION_USB_PROFILE_REVISION:
        raise PcmUsbError("지원하지 않는 PCM USB profile revision입니다")
    if hello.motion_profile_revision != SESSION_MOTION_PROFILE_REVISION:
        raise PcmUsbError("지원하지 않는 PCM motion profile revision입니다")
    return hello


def parse_session_status(data: bytes) -> SessionStatus:
    if len(data) != struct.calcsize("<HBBBBHIIIII"):
        raise PcmUsbError(f"PCM 세션 상태 길이가 올바르지 않습니다: {len(data)}")
    status = SessionStatus(*struct.unpack("<HBBBBHIIIII", data))
    if status.abi_revision != SESSION_ABI_REVISION:
        raise PcmUsbError(
            f"지원하지 않는 PCM 세션 상태 ABI {status.abi_revision}")
    return status


def build_live_begin_command(session_epoch: int, generation: int,
                             nonce: int) -> bytes:
    if not session_epoch or not generation or not nonce:
        raise ValueError("session epoch, generation, nonce must be non-zero")
    return struct.pack(
        "<HBBIIIII",
        SESSION_ABI_REVISION,
        SESSION_OPCODE_LIVE_BEGIN,
        SESSION_FLAG_HOST_RELEASE_CONFIRMED,
        session_epoch & 0xFFFFFFFF,
        generation & 0xFFFFFFFF,
        nonce & 0xFFFFFFFF,
        (nonce >> 32) & 0xFFFFFFFF,
        SESSION_COMMAND_GUARD)


def _usb_identity_matches(tty_path: str) -> bool:
    current = os.path.realpath(
        os.path.join("/sys/class/tty", os.path.basename(tty_path), "device"))
    while current.startswith("/sys/") and current != "/sys":
        vendor_path = os.path.join(current, "idVendor")
        product_path = os.path.join(current, "idProduct")
        try:
            with open(vendor_path, encoding="ascii") as vendor_file:
                vendor = vendor_file.read().strip().lower()
            with open(product_path, encoding="ascii") as product_file:
                product = product_file.read().strip().lower()
            return vendor == PCM_USB_VID and product == PCM_USB_PID
        except OSError:
            current = os.path.dirname(current)
    return False


def find_pcm_port() -> Optional[str]:
    override = os.environ.get("PHORCE_PCM_PORT", "").strip()
    if override:
        return override
    candidates = sorted(glob.glob("/dev/ttyACM*"))
    matching = [path for path in candidates if _usb_identity_matches(path)]
    if matching:
        return matching[0]
    return candidates[0] if len(candidates) == 1 else None


class PcmUsbServoClient:
    """Reconnectable, single-outstanding PCM USB SDO client."""

    def __init__(self, port: Optional[str] = None):
        self._configured_port = port
        self._arm_requested = threading.Event()
        self._unarm_requested = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot = PcmUsbSnapshot(
            False, port or "", "SEARCHING",
            "PCM USB CDC 포트를 찾는 중", None, None)
        self._fd: Optional[int] = None
        self._sequence = 0
        self._session_generation = 0
        self._studio_live = False
        self._release_port_until_command = False
        self._port_release_not_before = 0.0
        self._rx = bytearray()
        self._thread = threading.Thread(
            target=self._run, name="PcmUsbServo", daemon=True)
        self._thread.start()

    def request_servo_on(self) -> None:
        self._arm_requested.set()

    def request_servo_off(self) -> None:
        """Request the safety-direction servo OFF command.

        This is not the physical button-2 homing sequence.  It removes torque
        immediately and takes priority over a pending/in-progress ARM request.
        """
        self._arm_requested.clear()
        self._unarm_requested.set()

    def snapshot(self) -> PcmUsbSnapshot:
        with self._lock:
            return self._snapshot

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._close_port()

    def _publish(self, *, connected: Optional[bool] = None,
                 port: Optional[str] = None, phase: Optional[str] = None,
                 message: Optional[str] = None,
                 servo_state: object = ..., build_id: object = ...) -> None:
        with self._lock:
            old = self._snapshot
            self._snapshot = PcmUsbSnapshot(
                old.connected if connected is None else connected,
                old.port if port is None else port,
                old.phase if phase is None else phase,
                old.message if message is None else message,
                old.servo_state if servo_state is ... else servo_state,
                old.build_id if build_id is ... else build_id)

    def _run(self) -> None:
        while not self._stop.is_set():
            port = self._configured_port or find_pcm_port()
            if not port:
                self._publish(
                    connected=False, port="", phase="SEARCHING",
                    message=("PCM USB CDC(/dev/ttyACM*)를 찾지 못했습니다. "
                             "USB 연결과 PHORCE_PCM_PORT를 확인하세요"),
                    servo_state=None)
                self._stop.wait(1.0)
                continue
            try:
                self._open_port(port)
                self._connected_loop(port)
            except (OSError, PcmUsbError) as exc:
                self._publish(
                    connected=False, port=port, phase="ERROR",
                    message=f"PCM USB 통신 오류: {exc}", servo_state=None)
            finally:
                self._close_port()
            # ARM needs Studio LIVE only long enough to request and verify
            # SERVO_ON. Keeping CDC/DTR open afterwards leaves PCM owned by
            # Studio, so the EtherCAT motion window reports physical_idle=0
            # and rejects every slot. Stay detached until the next explicit
            # ARM/OFF command so the PCM watchdog can return motion ownership.
            while (self._release_port_until_command and
                   not self._stop.is_set() and
                   (time.monotonic() < self._port_release_not_before or
                    (not self._arm_requested.is_set() and
                     not self._unarm_requested.is_set()))):
                self._stop.wait(0.1)
            if self._release_port_until_command:
                self._release_port_until_command = False
                self._port_release_not_before = 0.0
            else:
                self._stop.wait(1.0)

    def _open_port(self, port: str) -> None:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0
            attrs[1] = 0
            attrs[2] = (termios.CLOCAL | termios.CREAD | termios.CS8 |
                        termios.HUPCL)
            attrs[3] = 0
            baud = getattr(termios, "B921600")
            attrs[4] = baud
            attrs[5] = baud
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 1
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIOFLUSH)
            # PCM gates its CDC stream until the host asserts DTR.
            fcntl.ioctl(
                fd, getattr(termios, "TIOCMBIS", 0x5416),
                struct.pack("I", getattr(termios, "TIOCM_DTR", 0x002)))
            time.sleep(0.05)
        except Exception:
            os.close(fd)
            raise
        self._fd = fd
        self._rx.clear()

    def _close_port(self) -> None:
        fd, self._fd = self._fd, None
        self._studio_live = False
        if fd is not None:
            try:
                # Do not rely on close(2) alone: older runs configured the tty
                # without HUPCL, which can leave DTR asserted and make PCM
                # believe Studio still owns the CDC session.
                fcntl.ioctl(
                    fd, getattr(termios, "TIOCMBIC", 0x5417),
                    struct.pack("I", getattr(termios, "TIOCM_DTR", 0x002)))
            except OSError:
                pass
            try:
                os.close(fd)
            except OSError:
                pass

    def _connected_loop(self, port: str) -> None:
        self._publish(
            connected=True, port=port, phase="CONNECTED",
            message=f"PCM USB 연결됨: {port}")
        next_poll = 0.0
        build_queried = False
        while not self._stop.is_set():
            if self._unarm_requested.is_set():
                self._unarm_requested.clear()
                try:
                    self._perform_servo_off(port)
                except PcmUsbError as exc:
                    self._publish(
                        connected=True, port=port, phase="ERROR",
                        message=f"UNARM 실패: {exc}")
                    self._release_port_until_command = True
                if self._release_port_until_command:
                    return
                next_poll = time.monotonic() + 0.5
            elif self._arm_requested.is_set():
                self._arm_requested.clear()
                try:
                    self._perform_servo_on(port)
                except PcmUsbError as exc:
                    self._publish(
                        connected=True, port=port, phase="ERROR",
                        message=f"ARM 실패: {exc}")
                    # Never keep a descriptor that timed out during ARM. PCM
                    # may have re-enumerated CDC without invalidating the old
                    # fd, so the next request must start from a clean open.
                    self._release_port_until_command = True
                if self._release_port_until_command:
                    return
                next_poll = time.monotonic() + 2.0
            now = time.monotonic()
            if now >= next_poll:
                # phorce Studio establishes 0x5F0C LIVE before ordinary SDO
                # polling.  Sending servo/build reads first can leave a timed
                # out request in the PCM single-outstanding mailbox and starve
                # the subsequent HELLO/status exchange.  Before the operator
                # requests ARM, keep the CDC link quiet; UNARM still takes its
                # safety-priority direct path above.
                if self._studio_live:
                    try:
                        state = self._read_servo_state()
                        self._publish_servo_state(port, state)
                        if not build_queried:
                            self._read_build_id()
                            build_queried = True
                    except PcmUsbError as exc:
                        self._publish(
                            connected=True, port=port, phase="CONNECTED",
                            message=f"PCM 서보 상태 확인 지연: {exc}")
                next_poll = now + 1.0
            self._drain_input(0.05)

    def _perform_servo_on(self, port: str) -> None:
        self._ensure_studio_live(port)
        self._publish(
            connected=True, port=port, phase="ARMING",
            message="ARM 명령 전송 — PCM 응답 대기")
        response = self._exchange(
            OD_SERVO, SUB_SERVO_SET, bytes((SERVO_ON,)), timeout=1.0)
        if response.is_abort:
            if response.abort_code == 0x06020000:
                raise PcmUsbError(
                    "이 PCM 펌웨어에는 USB ARM 명령(0x5F08)이 없습니다")
            raise PcmUsbError(
                f"서보 ON SDO 거부 0x{response.abort_code:08X}")
        if not response.is_write_ack:
            raise PcmUsbError("서보 ON write ACK 형식이 올바르지 않습니다")

        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not self._stop.is_set():
            if self._unarm_requested.is_set():
                self._unarm_requested.clear()
                self._perform_servo_off(port)
                return
            try:
                state = self._read_servo_state()
            except PcmUsbError as exc:
                # PCM can briefly stop servicing CDC SDO reads while it
                # installs the position controller and moves to boot pose.
                # Studio treats this as a polling gap, not an ARM failure.
                if "응답 timeout" not in str(exc):
                    raise
                self._publish(
                    connected=True, port=port, phase="ARMING",
                    message="ARM 전이 중 — USB 상태 응답 재시도")
                self._stop.wait(0.25)
                continue
            if state == SERVO_ON:
                self._release_port_until_command = True
                self._publish(
                    connected=True, port=port, phase="ON", servo_state=state,
                    message=("ARM 완료 — PCM 서보 ON 확인 · "
                             "모션 소유권 반환을 위해 USB 세션 해제"))
                return
            if state in (SERVO_UNAVAILABLE, SERVO_UNCONFIGURED):
                result = self._read_servo_result()
                detail = SERVO_RESULT_NAMES.get(
                    result, SERVO_STATE_NAMES.get(state, f"상태 {state}"))
                raise PcmUsbError(detail)
            if state == SERVO_OFF:
                result = self._read_servo_result()
                if result < 0:
                    raise PcmUsbError(SERVO_RESULT_NAMES.get(
                        result, f"PCM 서보 명령 거부 {result}"))
            self._publish(
                connected=True, port=port, phase="ARMING", servo_state=state,
                message=SERVO_STATE_NAMES.get(state, f"PCM 서보 상태 {state}"))
            self._stop.wait(0.25)
        raise PcmUsbError("서보 ON이 12초 안에 확인되지 않았습니다")

    def _ensure_studio_live(self, port: str) -> None:
        """Enter the same PCM LIVE session that phorce Studio establishes.

        The PCM exposes its SD card to the host while attaching.  Servo ON is
        acknowledged but not executed until the host safely releases that
        volume and sends a correlated LIVE_BEGIN command.
        """
        if self._studio_live:
            # Physical button 2 leaves the USB descriptor present but exits
            # Studio LIVE and may expose the SD volume again. A cached True
            # cannot be trusted across operator button presses, so validate
            # the PCM session before every ARM request.
            try:
                status = self._read_session_status()
            except PcmUsbError:
                self._studio_live = False
            else:
                if (status.mode == SESSION_MODE_STUDIO_LIVE and
                        status.phase in (0, SESSION_PHASE_READY)):
                    return
                self._studio_live = False
        self._publish(
            connected=True, port=port, phase="ARMING",
            message="PCM LIVE 세션 준비 — SD 카드 안전 해제 중")
        # On Linux, unmounting the PCM mass-storage interface while its sibling
        # CDC descriptor stays open can stall that descriptor for tens of
        # seconds.  The reliable order observed on hardware is: identify the
        # exact composite-device volume, close CDC, unmount, then reopen CDC.
        mounted_devices = self._pcm_mounted_block_devices(port)
        if mounted_devices:
            self._close_port()
            self._unmount_block_devices(mounted_devices)
            self._stop.wait(0.5)
            self._open_port(port)
        self._stop.wait(0.5)
        hello_response = self._exchange_retry_reopen(
            port,
            OD_SESSION, SUB_SESSION_HELLO, timeout=1.5, attempts=5)
        if hello_response.is_abort:
            raise PcmUsbError(
                f"PCM HELLO 거부 0x{hello_response.abort_code:08X}")
        hello = parse_session_hello(hello_response.data)
        if not hello.capability_bits & SESSION_CAP_STORAGE:
            raise PcmUsbError("PCM이 Studio LIVE 세션을 지원하지 않습니다")

        status = self._read_session_status()
        if (status.mode == SESSION_MODE_STUDIO_LIVE and
                status.phase in (0, SESSION_PHASE_READY)):
            self._studio_live = True
            return
        if not status.flags & SESSION_STATUS_SAFE_PARKING:
            raise PcmUsbError(
                "PCM이 안전 파킹 상태가 아니라 LIVE 세션을 시작할 수 없습니다")

        self._session_generation = (self._session_generation + 1) & 0xFFFFFFFF
        if self._session_generation == 0:
            self._session_generation = 1
        nonce = secrets.randbits(64) or 1
        payload = build_live_begin_command(
            hello.session_epoch, self._session_generation, nonce)
        response = self._exchange_retry(
            OD_SESSION, SUB_SESSION_COMMAND, payload,
            timeout=1.5, attempts=3)
        if response.is_abort:
            raise PcmUsbError(
                f"PCM LIVE_BEGIN 거부 0x{response.abort_code:08X}")
        if not response.is_write_ack:
            raise PcmUsbError("PCM LIVE_BEGIN ACK 형식이 올바르지 않습니다")

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not self._stop.is_set():
            try:
                status = self._read_session_status()
            except PcmUsbError as exc:
                if "응답 timeout" not in str(exc):
                    raise
                self._stop.wait(0.2)
                continue
            if (status.mode == SESSION_MODE_STUDIO_LIVE and
                    status.phase in (0, SESSION_PHASE_READY)):
                self._studio_live = True
                self._publish(
                    connected=True, port=port, phase="ARMING",
                    message="PCM LIVE 세션 준비 완료 — ARM 시작")
                return
            if status.phase in (SESSION_PHASE_REJECTED, SESSION_PHASE_FAULT):
                raise PcmUsbError(
                    f"PCM LIVE 세션 거부 (reason {status.reason})")
            self._stop.wait(0.2)
        raise PcmUsbError("PCM LIVE 세션이 10초 안에 준비되지 않았습니다")

    def _read_session_status(self) -> SessionStatus:
        response = self._exchange_retry(
            OD_SESSION, SUB_SESSION_STATUS, timeout=1.5, attempts=3)
        if response.is_abort:
            raise PcmUsbError(
                f"PCM 세션 상태 거부 0x{response.abort_code:08X}")
        return parse_session_status(response.data)

    @staticmethod
    def _usb_device_root(device_path: str) -> Optional[str]:
        current = os.path.realpath(device_path)
        while current.startswith("/sys/") and current != "/sys":
            try:
                with open(os.path.join(current, "idVendor"),
                          encoding="ascii") as vendor_file:
                    vendor = vendor_file.read().strip().lower()
                with open(os.path.join(current, "idProduct"),
                          encoding="ascii") as product_file:
                    product = product_file.read().strip().lower()
                if vendor == PCM_USB_VID and product == PCM_USB_PID:
                    return current
            except OSError:
                pass
            current = os.path.dirname(current)
        return None

    @classmethod
    def _pcm_mounted_block_devices(cls, port: str) -> list[str]:
        usb_root = cls._usb_device_root(os.path.join(
            "/sys/class/tty", os.path.basename(port), "device"))
        if not usb_root:
            raise PcmUsbError("PCM USB 장치 경로를 확인하지 못했습니다")

        pcm_blocks = set()
        for block_path in glob.glob("/sys/class/block/*"):
            resolved = os.path.realpath(block_path)
            # A composite USB device exposes storage and CDC as sibling
            # interface paths such as ``1-4.3:1.0`` and ``1-4.3:1.1``.
            if (resolved == usb_root or
                    resolved.startswith(usb_root + os.sep) or
                    resolved.startswith(usb_root + ":")):
                pcm_blocks.add(os.path.realpath(os.path.join(
                    "/dev", os.path.basename(block_path))))

        mounted = []
        try:
            with open("/proc/self/mountinfo", encoding="utf-8") as mounts:
                for line in mounts:
                    fields = line.split()
                    try:
                        separator = fields.index("-")
                    except ValueError:
                        continue
                    if len(fields) <= separator + 2:
                        continue
                    source = os.path.realpath(fields[separator + 2])
                    if source in pcm_blocks:
                        mounted.append(source)
        except OSError as exc:
            raise PcmUsbError(f"마운트 상태를 읽지 못했습니다: {exc}") from exc
        return sorted(set(mounted))

    @classmethod
    def _unmount_pcm_volumes(cls, port: str) -> None:
        cls._unmount_block_devices(cls._pcm_mounted_block_devices(port))

    @staticmethod
    def _unmount_block_devices(devices: list[str]) -> None:
        for device in devices:
            try:
                completed = subprocess.run(
                    ["udisksctl", "unmount", "-b", device],
                    capture_output=True, text=True, timeout=20, check=False)
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise PcmUsbError(
                    f"PCM SD 카드 안전 해제 실행 실패: {exc}") from exc
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout).strip()
                raise PcmUsbError(
                    f"PCM SD 카드를 안전 해제하지 못했습니다: {detail}")

    def _perform_servo_off(self, port: str) -> None:
        # OFF is a safety-direction command. Prime the CDC/SDO channel with
        # HELLO, but do not acquire Studio LIVE: after a HOME motion the robot
        # is still servo-on and PCM correctly refuses LIVE_BEGIN because its
        # physical SAFE_PARKING flag is not set. Taking Studio ownership here
        # would also close the EtherCAT motion window again.
        self._prepare_servo_off_channel(port)
        self._publish(
            connected=True, port=port, phase="UNARMING",
            message="UNARM 전송 — PCM 서보 OFF 확인 중")
        response = self._exchange_retry(
            OD_SERVO, SUB_SERVO_SET, bytes((SERVO_OFF,)),
            timeout=1.0, attempts=3)
        if response.is_abort:
            if response.abort_code == 0x06020000:
                raise PcmUsbError(
                    "이 PCM 펌웨어에는 USB ARM/UNARM 명령(0x5F08)이 없습니다")
            raise PcmUsbError(
                f"서보 OFF SDO 거부 0x{response.abort_code:08X}")
        if not response.is_write_ack:
            raise PcmUsbError("서보 OFF write ACK 형식이 올바르지 않습니다")

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self._stop.is_set():
            try:
                state = self._read_servo_state()
            except PcmUsbError as exc:
                if "응답 timeout" not in str(exc):
                    raise
                self._stop.wait(0.1)
                continue
            if state == SERVO_OFF:
                # OFF returns PCM to storage/parking mode. The existing CDC
                # descriptor can remain visible while its fd stops answering,
                # so force the next ARM to begin with a fresh open/session.
                self._release_port_until_command = True
                # Give PCM time to finish switching back to storage/parking
                # before a queued next ARM is allowed to reopen CDC.
                self._port_release_not_before = time.monotonic() + 3.0
                self._publish(
                    connected=True, port=port, phase="CONNECTED",
                    servo_state=state,
                    message=("UNARM 완료 — PCM 서보 OFF 확인 · "
                             "다음 ARM 전까지 USB 포트 해제"))
                return
            self._publish(
                connected=True, port=port, phase="UNARMING",
                servo_state=state,
                message="UNARM 처리 중 — PCM 서보 OFF 대기")
            self._stop.wait(0.1)
        raise PcmUsbError("서보 OFF가 5초 안에 확인되지 않았습니다")

    def _prepare_servo_off_channel(self, port: str) -> None:
        self._publish(
            connected=True, port=port, phase="UNARMING",
            message="PCM OFF 채널 준비 — USB HELLO 확인 중")
        mounted_devices = self._pcm_mounted_block_devices(port)
        if mounted_devices:
            self._close_port()
            self._unmount_block_devices(mounted_devices)
            self._stop.wait(0.5)
            self._open_port(port)
        else:
            # No storage transition or CDC reopen occurred. Avoid adding the
            # former fixed 0.5 s delay to every DOB/emergency OFF request.
            self._stop.wait(0.05)
        hello_response = self._exchange_retry_reopen(
            port, OD_SESSION, SUB_SESSION_HELLO,
            timeout=1.5, attempts=5)
        if hello_response.is_abort:
            raise PcmUsbError(
                f"PCM HELLO 거부 0x{hello_response.abort_code:08X}")
        hello = parse_session_hello(hello_response.data)
        if not hello.capability_bits & SESSION_CAP_STORAGE:
            raise PcmUsbError("PCM이 USB 서보 제어 채널을 지원하지 않습니다")

    def _publish_servo_state(self, port: str, state: int) -> None:
        old = self.snapshot()
        phase = "ON" if state == SERVO_ON else "CONNECTED"
        message = SERVO_STATE_NAMES.get(state, f"알 수 없는 PCM 서보 상태 {state}")
        if old.phase == "ERROR" and state != SERVO_ON:
            phase = "CONNECTED"
        self._publish(connected=True, port=port, phase=phase,
                      message=message, servo_state=state)

    def _read_servo_state(self) -> int:
        response = self._exchange(OD_SERVO, SUB_SERVO_STATE, timeout=0.8)
        if response.is_abort:
            if response.abort_code == 0x06020000:
                raise PcmUsbError("PCM 펌웨어가 0x5F08 서보 제어를 지원하지 않습니다")
            raise PcmUsbError(f"상태 Read 거부 0x{response.abort_code:08X}")
        if not response.data:
            raise PcmUsbError("서보 상태 응답이 비어 있습니다")
        return response.data[0]

    def _read_servo_result(self) -> int:
        response = self._exchange(OD_SERVO, SUB_SERVO_RESULT, timeout=0.8)
        if response.is_abort or not response.data:
            return 0
        return struct.unpack("<b", response.data[:1])[0]

    def _read_build_id(self) -> None:
        response = self._exchange(OD_SERVO, SUB_SERVO_BUILD_ID, timeout=0.8)
        if not response.is_abort and response.data:
            build_id = int.from_bytes(response.data[:4], "little")
            self._publish(build_id=build_id)

    def _exchange(self, index: int, subindex: int,
                  data: Optional[bytes] = None, timeout: float = 0.8) -> SdoResponse:
        sequence = self._sequence
        self._sequence = (self._sequence + 1) & 0xFFFF
        frame = (build_sdo_read(index, subindex, sequence) if data is None else
                 build_sdo_write(index, subindex, data, sequence))
        self._write_all(frame)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop.is_set():
            for msg_type, _node_id, _sequence, payload in self._drain_input(0.05):
                if msg_type != MSG_SDO_RSP:
                    continue
                response = parse_sdo_response(payload)
                if (response is not None and response.index == index and
                        response.subindex == subindex):
                    return response
        raise PcmUsbError(f"SDO 0x{index:04X}:{subindex:02X} 응답 timeout")

    def _exchange_retry(self, index: int, subindex: int,
                        data: Optional[bytes] = None, timeout: float = 0.8,
                        attempts: int = 3) -> SdoResponse:
        """Retry bounded PCM polling gaps without hiding protocol errors."""
        if attempts < 1:
            raise ValueError("attempts must be at least one")
        last_error: Optional[PcmUsbError] = None
        for attempt in range(attempts):
            try:
                return self._exchange(index, subindex, data, timeout)
            except PcmUsbError as exc:
                if "응답 timeout" not in str(exc):
                    raise
                last_error = exc
                if attempt + 1 < attempts:
                    self._stop.wait(0.5)
        assert last_error is not None
        raise last_error

    def _exchange_retry_reopen(self, port: str, index: int, subindex: int,
                               data: Optional[bytes] = None,
                               timeout: float = 0.8,
                               attempts: int = 3) -> SdoResponse:
        """Retry a session exchange with a fresh CDC fd each time.

        PCM mode transitions can leave an open tty fd selectable/writable but
        permanently silent. Retrying frames on that fd cannot recover it.
        """
        if attempts < 1:
            raise ValueError("attempts must be at least one")
        last_error: Optional[BaseException] = None
        for attempt in range(attempts):
            if self._fd is None:
                try:
                    self._open_port(port)
                except OSError as open_exc:
                    last_error = open_exc
                    if attempt + 1 < attempts:
                        self._stop.wait(0.5)
                    continue
            try:
                return self._exchange(index, subindex, data, timeout)
            except (OSError, PcmUsbError) as exc:
                if isinstance(exc, PcmUsbError) and "응답 timeout" not in str(exc):
                    raise
                last_error = exc
                if attempt + 1 < attempts:
                    self._close_port()
                    self._stop.wait(0.5)
                    try:
                        self._open_port(port)
                    except OSError as open_exc:
                        last_error = open_exc
                        self._stop.wait(0.5)
        raise PcmUsbError(
            f"CDC 재연결 후에도 SDO 0x{index:04X}:{subindex:02X} 응답 timeout: "
            f"{last_error}")

    def _write_all(self, data: bytes) -> None:
        if self._fd is None:
            raise PcmUsbError("USB 포트가 닫혀 있습니다")
        view = memoryview(data)
        while view:
            try:
                count = os.write(self._fd, view)
                if count == 0:
                    raise PcmUsbError("USB write가 0바이트를 반환했습니다")
                view = view[count:]
            except BlockingIOError:
                select.select([], [self._fd], [], 0.1)

    def _drain_input(self, wait_s: float) -> list[tuple[int, int, int, bytes]]:
        if self._fd is None:
            raise PcmUsbError("USB 포트가 닫혀 있습니다")
        readable, _, _ = select.select([self._fd], [], [], wait_s)
        if readable:
            try:
                chunk = os.read(self._fd, 65536)
            except BlockingIOError:
                chunk = b""
            if chunk:
                self._rx.extend(chunk)
            else:
                raise PcmUsbError("PCM USB 연결이 종료되었습니다")
        frames = []
        while True:
            try:
                end = self._rx.index(0)
            except ValueError:
                break
            encoded = bytes(self._rx[:end])
            del self._rx[:end + 1]
            if not encoded:
                continue
            try:
                frames.append(parse_dop_frame(encoded))
            except ValueError:
                continue
        if len(self._rx) > 4096:
            self._rx.clear()
        return frames
