"""세그멘테이션 모델 기반 물체 검출 (FastSAM).

FastSAM은 클래스 무관(class-agnostic) 세그멘테이션이라 커스텀 인형도
학습 없이 분할된다. 분할된 마스크 중에서
  1. 면적 조건 (min_area ~ max_area)
  2. 마스크 내부 픽셀의 흰색 비율 (채도 낮고 명도 높은 픽셀 비율)
을 통과한 것만 "집을 물체"로 인정한다.

인터페이스는 detector.WhiteObjectDetector 와 동일하다 (mask / detect_all /
detect / draw). 상태머신·도구 코드는 어느 쪽을 쓰는지 몰라도 된다.
"""

import cv2
import numpy as np

from .detector import Detection, WhiteObjectDetector


class SegObjectDetector:
    def __init__(self, cfg):
        seg = cfg.get("seg", {})
        self.model_path = seg.get("model", "FastSAM-s.pt")
        self.imgsz = int(seg.get("imgsz", 480))
        self.conf = float(seg.get("conf", 0.4))
        self.iou = float(seg.get("iou", 0.9))
        self.device = seg.get("device", 0)          # 0 = GPU, "cpu" = CPU
        self.white_ratio = float(seg.get("white_ratio", 0.6))

        # 흰색 판정 임계값은 HSV 검출기와 공유 (마스크 "내부" 색 판정에만 사용)
        self.s_max = int(cfg["s_max"])
        self.v_min = int(cfg["v_min"])
        self.min_area = float(cfg["min_area"])
        self.max_area = float(cfg["max_area"])
        self.pick_order = cfg.get("pick_order", "largest")

        from ultralytics import FastSAM  # 지연 임포트 - dummy 실행 시 불필요

        self.model = FastSAM(self.model_path)
        # 첫 추론은 CUDA 초기화 때문에 수 초 걸린다. 미리 워밍업.
        warm = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        self.model(warm, device=self.device, imgsz=self.imgsz,
                   conf=self.conf, iou=self.iou, verbose=False)

    # ------------------------------------------------------------------
    def _white_mask(self, warped_bgr):
        """픽셀 단위 흰색 여부 (마스크 내부 색 판정용)."""
        hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, self.v_min], dtype=np.uint8)
        upper = np.array([179, self.s_max, 255], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def _segment(self, warped_bgr):
        """FastSAM 추론 -> 프레임 크기의 이진 마스크 리스트."""
        res = self.model(
            warped_bgr, device=self.device, imgsz=self.imgsz,
            conf=self.conf, iou=self.iou, retina_masks=True, verbose=False,
        )[0]
        if res.masks is None:
            return []
        h, w = warped_bgr.shape[:2]
        out = []
        for m in res.masks.data:                     # (N, mh, mw) 텐서
            arr = m.cpu().numpy().astype(np.uint8)
            if arr.shape != (h, w):
                arr = cv2.resize(arr, (w, h), interpolation=cv2.INTER_NEAREST)
            out.append(arr * 255)
        return out

    # ------------------------------------------------------------------
    def mask(self, warped_bgr):
        """조건을 통과한 물체들의 합성 마스크 (디버그 표시용)."""
        combined = np.zeros(warped_bgr.shape[:2], dtype=np.uint8)
        for d in self.detect_all(warped_bgr):
            cv2.drawContours(combined, [d.contour], -1, 255, -1)
        return combined

    def detect_all(self, warped_bgr):
        white = self._white_mask(warped_bgr)
        found = []

        for seg_mask in self._segment(warped_bgr):
            area = int(np.count_nonzero(seg_mask))
            if not (self.min_area <= area <= self.max_area):
                continue

            # 마스크 내부에서 흰색 픽셀 비율 - 나무 바닥/그림자 마스크 걸러냄
            inside = np.count_nonzero(cv2.bitwise_and(white, seg_mask))
            if inside / area < self.white_ratio:
                continue

            contours, _ = cv2.findContours(
                seg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)

            mo = cv2.moments(c)
            if mo["m00"] == 0:
                continue
            cx = mo["m10"] / mo["m00"]
            cy = mo["m01"] / mo["m00"]

            (_, _), (bw, bh), ang = cv2.minAreaRect(c)
            if bw < bh:
                bw, bh = bh, bw
                ang += 90.0

            found.append(
                Detection(
                    cx=cx, cy=cy, area=float(area),
                    angle=ang % 180.0,
                    length=max(bw, bh), width=max(min(bw, bh), 1e-6),
                    contour=c,
                )
            )

        # 같은 물체가 중복 분할되는 경우 중심이 가까운 것끼리 하나만 남긴다
        found = self._dedup(found)

        if self.pick_order == "center_first":
            mid = warped_bgr.shape[0] / 2.0
            found.sort(key=lambda d: (d.cx - mid) ** 2 + (d.cy - mid) ** 2)
        else:
            found.sort(key=lambda d: d.area, reverse=True)
        return found

    @staticmethod
    def _dedup(dets, dist=20.0):
        """중심 거리가 dist px 이내인 검출은 면적 큰 쪽만 남긴다."""
        dets = sorted(dets, key=lambda d: d.area, reverse=True)
        kept = []
        for d in dets:
            if all(
                (d.cx - k.cx) ** 2 + (d.cy - k.cy) ** 2 > dist ** 2 for k in kept
            ):
                kept.append(d)
        return kept

    def detect(self, warped_bgr):
        found = self.detect_all(warped_bgr)
        return found[0] if found else None

    # detector.WhiteObjectDetector 와 동일한 시각화 재사용
    draw = staticmethod(WhiteObjectDetector.draw)


def build_detector(cfg, log=print):
    """설정의 detector.method 에 따라 검출기를 만든다 (hsv | seg)."""
    method = cfg.get("method", "hsv")
    if method == "seg":
        log(f"검출기: FastSAM 세그멘테이션 ({cfg.get('seg', {}).get('model', 'FastSAM-s.pt')})")
        return SegObjectDetector(cfg)
    from .detector import WhiteObjectDetector

    log("검출기: HSV 색 임계값")
    return WhiteObjectDetector(cfg)
