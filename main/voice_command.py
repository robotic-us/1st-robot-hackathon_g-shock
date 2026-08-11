#!/usr/bin/env python3
"""Push-to-talk Korean command recognition for the operator GUI."""

from __future__ import annotations

import json
import os
import select
import subprocess
import threading
import time
from array import array
from dataclasses import dataclass
from typing import Optional


DEFAULT_MODEL_DIR = os.path.expanduser(
    "~/.local/share/phorce-voice/vosk-model-small-ko-0.22")
# The emergency monitor is paused while push-to-talk owns the microphone, so
# use the C270's direct ALSA input instead of depending on the desktop's
# current PulseAudio default source.
DEFAULT_ALSA_DEVICE = "plughw:CARD=WEBCAM,DEV=0"
SAMPLE_RATE = 16000
RELEASE_TAIL_S = 0.4

COMMAND_ALIASES = {
    "준비": (
        "준비", "준비해", "준비 해", "준비해 줘", "준비 해 줘",
        "작업 준비", "시작", "시작해 줘", "시작 해 줘"),
    "물건 넣어줘": (
        "물건 넣어줘", "물건 넣어 줘", "물건을 넣어줘", "물건을 넣어 줘",
        "물건 넣어", "제품 넣어 줘", "물건 잡아 줘", "투입해 줘",
        "투입 해 줘"),
    "포장해줘": (
        "포장해줘", "포장 해줘", "포장해 줘", "포장 해 줘",
        "포장 시작", "포장 시작해 줘", "박스 포장", "박스에 포장해 줘"),
    "종료": (
        "종료", "종료해", "종료 해", "종료해 줘", "종료 해 줘",
        "작업 종료", "끝", "끝내 줘", "마무리", "마무리해 줘"),
}

GRAMMAR_PHRASES = [
    "준비", "준비 해 줘", "작업 준비", "시작", "시작 해 줘",
    "물건 넣어 줘", "물건을 넣어 줘", "제품 넣어 줘", "물건 잡아 줘",
    "투입 해 줘", "포장 해 줘", "포장 시작", "박스 포장",
    "박스에 포장 해 줘", "종료", "종료 해 줘", "작업 종료", "끝",
    "끝내 줘", "마무리", "마무리 해 줘",
]

@dataclass(frozen=True)
class VoiceCommandResult:
    command: Optional[str]
    transcript: str
    detail: str
    peak_rms: int = 0
    captured_seconds: float = 0.0


def _compact(text: str) -> str:
    return "".join(text.lower().split())


def classify_command(transcript: str) -> Optional[str]:
    compact = _compact(transcript)
    if not compact:
        return None
    # Korean small-model output can split or slightly vary verb endings.
    # The command nouns are distinctive enough for this four-command UI.
    if "물건" in compact and "넣" in compact:
        return "물건 넣어줘"
    if "제품" in compact and "넣" in compact:
        return "물건 넣어줘"
    if "물건" in compact and "잡" in compact:
        return "물건 넣어줘"
    if "투입" in compact:
        return "물건 넣어줘"
    if "포장" in compact:
        return "포장해줘"
    if "준비" in compact or "시작" in compact:
        return "준비"
    if "종료" in compact or "끝" in compact or "마무리" in compact:
        return "종료"
    for command, aliases in COMMAND_ALIASES.items():
        if any(_compact(alias) == compact for alias in aliases):
            return command
    return None


class VoiceCommandRecognizer:
    """Lazy Vosk recognizer using a bounded arecord capture."""

    def __init__(self, model_dir: Optional[str] = None,
                 alsa_device: Optional[str] = None, duration_s: int = 2):
        self.model_dir = (model_dir or os.environ.get("PHORCE_VOICE_MODEL") or
                          DEFAULT_MODEL_DIR)
        self.alsa_device = (alsa_device or
                            os.environ.get("PHORCE_VOICE_DEVICE") or
                            DEFAULT_ALSA_DEVICE)
        self.duration_s = duration_s
        self._model = None
        self._process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def _load_model(self):
        if self._model is not None:
            return self._model
        if not os.path.isdir(self.model_dir):
            raise RuntimeError(
                f"한국어 음성 모델이 없습니다: {self.model_dir}")
        try:
            from vosk import Model, SetLogLevel
        except ImportError as exc:
            raise RuntimeError(
                "Vosk가 설치되지 않았습니다: python3 -m pip install --user vosk"
            ) from exc
        SetLogLevel(-1)
        self._model = Model(self.model_dir)
        return self._model

    def warm_up(self) -> None:
        self._load_model()

    def recognize_once(self) -> VoiceCommandResult:
        stop_event = threading.Event()
        timer = threading.Timer(self.duration_s, stop_event.set)
        timer.start()
        try:
            return self.recognize_until(stop_event)
        finally:
            timer.cancel()

    def recognize_until(self, stop_event: threading.Event) -> VoiceCommandResult:
        model = self._load_model()
        try:
            from vosk import KaldiRecognizer
        except ImportError as exc:
            raise RuntimeError("Vosk 모듈을 불러오지 못했습니다") from exc

        # Use only tokenizations present in the Korean model vocabulary.
        # Agglutinated forms such as "넣어줘" are absent, while the separated
        # form "넣어 줘" is supported.
        grammar = json.dumps(GRAMMAR_PHRASES, ensure_ascii=False)
        recognizer = KaldiRecognizer(model, SAMPLE_RATE, grammar)
        args = [
            "arecord", "-q", "-D", self.alsa_device,
            "-t", "raw", "-f", "S16_LE", "-r", str(SAMPLE_RATE),
            "-c", "1",
        ]
        try:
            process = subprocess.Popen(
                args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except OSError as exc:
            raise RuntimeError(f"마이크 녹음을 시작하지 못했습니다: {exc}") from exc
        with self._lock:
            self._process = process
        captured = bytearray()
        peak_rms = 0
        started = time.monotonic()
        release_deadline = None
        terminated_after_capture = False
        try:
            while True:
                now = time.monotonic()
                if stop_event.is_set() and release_deadline is None:
                    # The final spoken syllable can still be buffered in ALSA
                    # when the key is released. Drain a short tail before
                    # terminating arecord so Korean verb endings are retained.
                    release_deadline = now + RELEASE_TAIL_S
                if release_deadline is not None and now >= release_deadline:
                    break
                readable, _, _ = select.select(
                    [process.stdout], [], [], 0.1)
                if readable:
                    chunk = os.read(process.stdout.fileno(), 4096)
                    if not chunk:
                        break
                    captured.extend(chunk)
                    peak_rms = max(peak_rms, pcm_rms(chunk))
                    recognizer.AcceptWaveform(chunk)
                if process.poll() is not None:
                    break
        finally:
            if process.poll() is None:
                # Some ALSA USB devices (including the Logitech C270) return
                # status 1, rather than -SIGTERM, after a successful capture
                # is stopped by terminate().  Remember that this was our
                # requested end of recording so it is not reported as a mic
                # failure when PCM data was actually received.
                terminated_after_capture = bool(captured)
                # The release tail above has already retained the final
                # syllable. Do not wait again for PulseAudio to drain its
                # internal buffer, which noticeably delays the GUI result.
                process.kill()
            try:
                remaining, error = process.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                remaining, error = process.communicate()
            if remaining:
                captured.extend(remaining)
                peak_rms = max(peak_rms, pcm_rms(remaining))
                recognizer.AcceptWaveform(remaining)
            with self._lock:
                self._process = None
        # SIGTERM is the normal end of push-to-talk recording.
        if (process.returncode not in (0, -9, -15) and
                not (terminated_after_capture and process.returncode == 1 and
                     not error.strip())):
            detail = error.decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"마이크 녹음 실패: {detail}")
        if time.monotonic() - started < 0.2 or len(captured) < 3200:
            return VoiceCommandResult(
                None, "", "녹음 시간이 너무 짧습니다", peak_rms,
                len(captured) / 2 / SAMPLE_RATE)
        payload = json.loads(recognizer.FinalResult())
        transcript = str(payload.get("text", "")).strip()
        command = classify_command(transcript)
        if command:
            return VoiceCommandResult(
                command, transcript, "명령 인식 완료", peak_rms,
                len(captured) / 2 / SAMPLE_RATE)
        if transcript:
            return VoiceCommandResult(
                None, transcript, "등록되지 않은 명령", peak_rms,
                len(captured) / 2 / SAMPLE_RATE)
        return VoiceCommandResult(
            None, "", "음성을 인식하지 못했습니다", peak_rms,
            len(captured) / 2 / SAMPLE_RATE)

    def close(self) -> None:
        with self._lock:
            process = self._process
        if process is not None and process.poll() is None:
            process.terminate()


def pcm_rms(pcm: bytes) -> int:
    """Return the RMS level of little-endian signed 16-bit mono PCM."""
    usable = len(pcm) - (len(pcm) % 2)
    if usable <= 0:
        return 0
    samples = array("h")
    samples.frombytes(pcm[:usable])
    if os.sys.byteorder != "little":
        samples.byteswap()
    return int((sum(sample * sample for sample in samples) /
                len(samples)) ** 0.5)
