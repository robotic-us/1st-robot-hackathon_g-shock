#!/usr/bin/env python3
"""펙보드 타공 격자로 homography 를 추정해 픽셀 <-> 평면 mm 변환을 만든다.

구멍이 등간격 격자라는 사실만 쓰기 때문에 별도 체스보드 타겟이 필요 없고,
렌즈 왜곡이 심하지 않은 범위에서 평면 좌표를 바로 얻을 수 있다.
"""
import argparse
import json

import cv2
import numpy as np


def detect_holes(bgr, min_area=15, max_area=800):
    """어두운 원형 블롭(타공 구멍)의 중심을 찾는다."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    # 국소 조명 편차에 강하도록 adaptive. 구멍이 어두우므로 INV.
    th = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 31, 8
    )
    th = cv2.morphologyEx(th, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    n, _, stats, cent = cv2.connectedComponentsWithStats(th)
    pts = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        if not (min_area <= area <= max_area) or w == 0 or h == 0:
            continue
        if not (0.6 <= w / h <= 1.67):      # 대략 원형
            continue
        if area / (w * h) < 0.55:            # 채움율 (원이면 ~0.785)
            continue
        pts.append(cent[i])
    return np.array(pts, np.float32).reshape(-1, 2)


def lattice_basis(pts):
    """이웃 벡터의 각도 분포에서 정사각 격자의 기저벡터 u, v 를 추정한다."""
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    np.fill_diagonal(d, np.inf)
    pitch = float(np.median(d.min(axis=1)))          # 최근접 거리 = 격자 간격

    ii, jj = np.where((d > 0.75 * pitch) & (d < 1.25 * pitch))
    vecs = pts[jj] - pts[ii]
    # 정사각 격자는 90도 주기 -> mod 90 으로 접어서 주방향 하나만 구한다
    ang = np.degrees(np.arctan2(vecs[:, 1], vecs[:, 0])) % 90.0
    # 원형 평균 (0/90 경계 wrap 방지)
    a4 = np.radians(ang * 4.0)
    theta = float(np.degrees(np.arctan2(np.sin(a4).mean(), np.cos(a4).mean())) / 4.0)

    t = np.radians(theta)
    u = pitch * np.array([np.cos(t), np.sin(t)])
    v = pitch * np.array([-np.sin(t), np.cos(t)])    # 정사각 격자 -> 90도 회전
    return u, v, pitch, theta


def fit_homography(pts, pitch_mm, iters=4):
    """격자 인덱스를 붙이고 board(mm) -> pixel homography 를 반복 정련한다."""
    u, v, pitch_px, theta = lattice_basis(pts)
    origin = pts[np.argmin(np.linalg.norm(pts - pts.mean(0), axis=1))]

    B = np.column_stack([u, v])                      # 격자 -> 픽셀 (선형 근사)
    idx = np.rint(np.linalg.solve(B, (pts - origin).T)).T

    H = inliers = None
    for _ in range(iters):
        board = idx * pitch_mm                       # 격자 인덱스 -> mm
        H, mask = cv2.findHomography(board, pts, cv2.RANSAC, 2.0)
        if H is None:
            raise RuntimeError("homography 추정 실패 - 구멍 검출을 확인하세요")
        inliers = mask.ravel().astype(bool)
        # homography 로 픽셀을 mm 로 되돌려 인덱스를 재할당 (원근 보정)
        back = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), np.linalg.inv(H))
        idx = np.rint(back.reshape(-1, 2) / pitch_mm)

    board = idx * pitch_mm
    proj = cv2.perspectiveTransform(board.reshape(-1, 1, 2), H).reshape(-1, 2)
    err = np.linalg.norm(proj - pts, axis=1)
    return H, idx, inliers, err, pitch_px, theta


POLY_DEG = 3
# 트리밍 임계는 픽셀 기준으로 둔다. mm 로 두면 --pitch-mm 을 바꿀 때마다
# 실제 걸러지는 기준이 같이 흔들린다.
TRIM_PX = 1.0


def poly_features(px, cx, cy, scale, deg=POLY_DEG):
    """픽셀 좌표를 정규화해 2D 다항 기저를 만든다 (수치 안정성 목적)."""
    x = (px[:, 0] - cx) / scale
    y = (px[:, 1] - cy) / scale
    return np.column_stack(
        [x ** i * y ** j for i in range(deg + 1) for j in range(deg + 1 - i)]
    )


def fit_poly(pts, board, cx, cy, scale, deg=POLY_DEG, trim=1.0):
    """픽셀 -> 보드 mm 다항 매핑. 렌즈 방사왜곡까지 함께 흡수한다.

    카메라와 보드가 고정이라는 전제 하에서, 내부파라미터 캘리브레이션 없이
    homography 보다 훨씬 작은 잔차를 얻는다. 가짜 검출점은 반복 트리밍으로 제거.
    """
    A = poly_features(pts, cx, cy, scale, deg)
    keep = np.ones(len(pts), bool)
    for _ in range(8):
        coef, *_ = np.linalg.lstsq(A[keep], board[keep], rcond=None)
        resid = np.linalg.norm(A @ coef - board, axis=1)
        new = resid < trim
        if new.sum() < A.shape[1] * 3:      # 과도한 트리밍 방지
            break
        if (new == keep).all():
            break
        keep = new
    return coef, keep, resid


def px_to_mm(px, cfg):
    """검출된 픽셀 좌표를 보드 평면 mm 좌표로 변환한다."""
    px = np.asarray(px, np.float64).reshape(-1, 2)
    A = poly_features(px, cfg["cx"], cfg["cy"], cfg["scale"], cfg["poly_deg"])
    return A @ np.asarray(cfg["poly_coef"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--pitch-mm", type=float, default=25.4,
                    help="타공 격자 실제 간격 (mm). 기본 25.4 = 1인치")
    ap.add_argument("--out", default="config/pegboard.json")
    ap.add_argument("--vis", default="out/pegboard_vis.png")
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"이미지를 열 수 없습니다: {args.image}")

    pts = detect_holes(img)
    print(f"구멍 검출        : {len(pts)}개")
    if len(pts) < 12:
        raise SystemExit("구멍이 너무 적습니다 - 임계값을 조정하세요")

    H, idx, inliers, err, pitch_px, theta = fit_homography(pts, args.pitch_mm)
    print(f"격자 간격        : {pitch_px:.2f} px  = {args.pitch_mm} mm "
          f"({args.pitch_mm / pitch_px:.4f} mm/px)")
    print(f"격자 회전        : {theta:+.2f} deg")
    print(f"inlier           : {inliers.sum()}/{len(pts)}")
    print(f"재투영 오차      : 중앙값 {np.median(err[inliers]):.3f} px, "
          f"최대 {err[inliers].max():.3f} px")

    # homography 는 격자 인덱스 확정용. 최종 매핑은 왜곡까지 흡수하는 다항식.
    h, w = img.shape[:2]
    cx, cy, scale = w / 2.0, h / 2.0, w / 2.0
    board = idx * args.pitch_mm
    trim_mm = TRIM_PX * args.pitch_mm / pitch_px
    coef, keep, resid = fit_poly(pts, board, cx, cy, scale, trim=trim_mm)
    rk = resid[keep]
    print(f"다항 매핑        : {POLY_DEG}차, 사용 {keep.sum()}/{len(pts)}점 "
          f"(트리밍 {TRIM_PX:.1f}px = {trim_mm:.3f}mm)")
    print(f"  잔차           : 중앙값 {np.median(rk):.3f} mm, p95 {np.percentile(rk, 95):.3f} mm, "
          f"최대 {rk.max():.3f} mm  (= {rk.max() / (args.pitch_mm / pitch_px):.2f} px)")

    json.dump(
        {
            "image_size": [w, h],
            "cx": cx, "cy": cy, "scale": scale,
            "poly_deg": POLY_DEG,
            "poly_coef": coef.tolist(),
            "H_board_to_px": H.tolist(),
            "pitch_mm": args.pitch_mm,
            "pitch_px": pitch_px,
            "mm_per_px": args.pitch_mm / pitch_px,
            "theta_deg": theta,
            "n_holes": int(len(pts)),
            "n_used": int(keep.sum()),
            "resid_mm_median": float(np.median(rk)),
            "resid_mm_max": float(rk.max()),
        },
        open(args.out, "w"),
        indent=2,
    )
    print(f"저장             : {args.out}")

    vis = img.copy()
    for p, ok in zip(pts, keep):
        cv2.circle(vis, tuple(p.astype(int)), 3, (0, 200, 0) if ok else (0, 0, 255), -1)
    cv2.imwrite(args.vis, vis)
    print(f"시각화           : {args.vis}  (초록=매핑에 사용, 빨강=제외)")


if __name__ == "__main__":
    main()
