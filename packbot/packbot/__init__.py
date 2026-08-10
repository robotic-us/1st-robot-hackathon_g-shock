"""packbot 패키지 초기화.

Jetson용 torch 2.11 휠은 libcudss.so.0 을 요구하는데, 이 라이브러리는
pip(nvidia-cudss-cu12)로 설치되어 시스템 로더 경로에 없다.
LD_LIBRARY_PATH 설정 없이도 동작하도록 torch 임포트 전에 미리 로드해 둔다.
"""

import ctypes
import glob
import os


def _preload_cudss():
    home = os.path.expanduser("~")
    for pat in (
        f"{home}/.local/lib/python3.10/site-packages/nvidia/cu12/lib/libcudss.so.0",
        f"{home}/.local/lib/python3.10/site-packages/nvidia/*/lib/libcudss.so*",
    ):
        for path in sorted(glob.glob(pat)):
            try:
                ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
                return
            except OSError:
                continue


_preload_cudss()
