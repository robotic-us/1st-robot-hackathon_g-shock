"""C270 웹캠 캡처 래퍼."""

import cv2


class Camera:
    """V4L2 백엔드로 웹캠을 열고 프레임을 읽는다.

    노출/화이트밸런스를 고정할 수 있게 해 두었다. 흰색 판정이 임계값 기반이라
    자동 노출이 켜져 있으면 조명이 조금만 변해도 검출이 흔들린다.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.cap = None

    def open(self):
        cap = cv2.VideoCapture(self.cfg["index"], cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"카메라 {self.cfg['index']} 번을 열 수 없습니다")

        fourcc = self.cfg.get("fourcc", "MJPG")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["width"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["height"])
        cap.set(cv2.CAP_PROP_FPS, self.cfg["fps"])

        # 0.25 = 수동, 0.75 = 자동 (V4L2 관례)
        if not self.cfg.get("auto_exposure", True):
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            cap.set(cv2.CAP_PROP_EXPOSURE, self.cfg.get("exposure", 150))
        if not self.cfg.get("auto_white_balance", True):
            cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            cap.set(cv2.CAP_PROP_WB_TEMPERATURE, self.cfg.get("white_balance", 4500))

        # 설정 반영 및 자동 게인 안정화를 위해 초기 몇 장은 버린다
        for _ in range(5):
            cap.read()

        self.cap = cap
        return self

    @property
    def actual_size(self):
        return int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
            self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )

    def read(self):
        """최신 프레임 한 장. 실패하면 RuntimeError."""
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("프레임 캡처 실패 - USB 연결을 확인하세요")
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.release()
        return False
