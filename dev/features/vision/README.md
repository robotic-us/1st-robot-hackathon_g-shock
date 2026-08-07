# pegboard vision

젯슨 AGX Orin + USB 웹캠으로 펙보드 위 물체의 **평면 mm 좌표와 자세**를 실시간 추출한다.

이 좌표를 로봇 grab 모션으로 넘기는 쪽은 [`../grab_planner/`](../grab_planner/) 이며,
주고받는 값과 로봇 쪽 제약은 [`../grab_planner/VISION_INTERFACE.md`](../grab_planner/VISION_INTERFACE.md) 에 정리돼 있다.

## 환경 원칙

**시스템 패키지와 `~/.local` 은 절대 건드리지 않는다.** apt / pip 의 upgrade·update 금지.
필요한 것은 전부 `venv/` 안에서만 해결한다.

- `venv/` 는 `virtualenv`(워크스페이스 내부 `.bootstrap/` 에만 설치)로 생성했다.
  시스템에 `python3.10-venv` 가 없어 `python3 -m venv` 는 쓸 수 없다.
- `venv/lib/python3.10/site-packages/_user_local.pth` 가 `~/.local` 을 경로에 잇는다.
  덕분에 기존 `cv2 4.11.0` / `numpy 1.26.4` 를 **재설치 없이** 그대로 쓴다.
- `--system-site-packages` 로 시스템 `tensorrt 10.3.0` 도 보인다.

`venv/` 와 `.bootstrap/` 은 저장소에 넣지 않는다. 클론한 뒤 한 번 만들면 된다.

```bash
cd dev/features/vision
./setup_venv.sh          # 위 절차를 그대로 수행한다
./venv/bin/python -c "import cv2, numpy, tensorrt; print(cv2.__version__)"
```

## 사용법

모든 경로가 이 디렉터리 기준 상대경로다. **반드시 여기서 실행한다.**

```bash
cd dev/features/vision

# 1) 보드 캘리브레이션 (물체가 놓인 상태에서도 가능 - 보이는 구멍만 쓴다)
./venv/bin/python src/pegboard.py data/frames/warm.jpg --pitch-mm 10

# 2) 실시간 검출
PYTHONPATH=src ./venv/bin/python src/live.py                 # 창 표시 (q 종료, m 마스크, s 저장)
PYTHONPATH=src ./venv/bin/python src/live.py --headless      # 수치만 출력
```

창을 띄우려면 `DISPLAY=:1` 이 필요하다 (이 장비의 X 세션).
타공 간격은 **10 mm** (화면상 20.23 px). 카메라를 움직였다면 1) 을 다시 돌려야 한다.

## 대상 물체

보드 가운데 놓인 흰색 물체 하나만 추적한다 (실측 **75.5 x 53.7 mm**).
가장자리의 고정 브래킷은 전부 화면 밖으로 잘려 나가므로,
`border_margin` 으로 **테두리에 닿는 블롭을 버리는 것만으로** 깔끔하게 걸러진다.

## 구성

| 파일 | 역할 |
|---|---|
| `src/pegboard.py` | 타공 격자로 픽셀↔mm 매핑 생성 → `config/pegboard.json` |
| `src/detector.py` | 물체 분할 + mm 좌표/자세 추출 |
| `src/live.py` | 웹캠 실시간 루프 |

## 측정된 성능 (640x480, CPU 단일 코어)

| 항목 | 값 |
|---|---|
| 처리 시간 | **5.9 ms/frame** (p95 7.0 ms) |
| 매핑 잔차 | **중앙값 0.120 mm**, p95 0.345 mm, 최대 0.490 mm |
| 위치 반복성 | **0.16 mm** (화이트밸런스가 전혀 다른 두 프레임 간) |
| 카메라 상한 | 15~20 fps (50 ms/read) — 파이프라인이 아니라 카메라 한계 |

## 설계 근거 (실측으로 확정한 것들)

**픽셀→mm 는 homography 가 아니라 3차 다항식.** 렌즈 방사왜곡 때문에 순수
homography 는 화면 바깥쪽에서 잔차가 1.0 → 2.3 px 로 커지고 인라이어가 86% → 42% 로
떨어졌다. 카메라와 보드가 고정이므로, 내부파라미터 캘리브레이션 없이 다항식이
왜곡까지 통째로 흡수한다. 격자 간격은 자기상관(FFT)으로 독립 검증했다 (20 px).
트리밍 임계는 mm 가 아니라 **픽셀** 기준이다 (`TRIM_PX`). mm 로 두면
`--pitch-mm` 을 바꿀 때 걸러지는 기준이 같이 흔들린다.

**색 임계는 고정값이 아니라 매 프레임 추정.** 웹캠 오토 화이트밸런스가 수렴하는
값에 따라 보드 Hue 가 16 / 22 / 43 까지 움직인다. 고정 임계는 이때 마스크가 화면
전체로 터진다. 보드가 최대 면적이라는 전제로 Hue 모드를 잡고 상대 임계를 쓴다.
화이트밸런스가 전혀 다른 두 프레임에서 같은 물체가 0.4 mm 이내로 일치함을 확인했다.

**카메라 오픈 후 워밍업 필수.** AWB/AE 수렴 전 프레임은 색이 완전히 다르다.
`open_cam(warmup=2.5)` 이 그 구간을 버린다. 10 프레임 정도로는 부족하다.

## 한계

- **정사각 블롭에서 자세(θ)가 불안정하다.** `minAreaRect` 는 가로세로가 비슷하면
  각도가 90도 단위로 튄다. 자세 정밀도가 필요하면 OBB 모델이나 에지 기반 정합으로 가야 한다.
- 물체끼리 닿으면 하나의 블롭으로 합쳐진다.
- 물체 종류를 구분하지 못한다 (위치만).

이 셋 중 하나라도 필요해지는 시점이 YOLO11n-obb 로 넘어갈 시점이다.
학습 데이터는 이 파이프라인의 출력을 그대로 라벨로 쓰면 손라벨이 거의 필요 없다.

## 알려진 블로커

**좌표 원점이 아직 임의의 구멍이다.** 그래서 출력 좌표가 음수로 나온다.
로봇으로 좌표를 넘기려면 비전 좌표계와 로봇 월드 좌표계를 잇는 2D 강체변환을
한 번 실측해야 한다. 절차는 `../grab_planner/VISION_INTERFACE.md` 2절에 있다.
**이것 없이는 좌표를 외부로 내보낼 수 없다.**

시스템 `torch 2.11.0` 은 `libcudss.so.0` 이 없어 import 자체가 실패한다.
모델 학습·export 로 넘어가려면 먼저 해결해야 한다. **시스템을 고치지 말고**
JetPack 6.2 용 휠을 `venv` 안에만 설치할 것.
