# 인수인계 — pegboard vision

작성 2026-08-07. 다음 세션은 이 문서만 읽고 바로 이어서 작업할 수 있다.
설계 근거 요약은 `README.md`, 여기는 **재조사가 필요 없도록 실측값과 실패 기록**까지 담는다.

---

## 0. 지금 상태 한 줄

**동작한다.** 펙보드 위 대상 물체 하나의 평면 mm 좌표·자세를 5.9 ms/frame 으로 뽑고 있고,
매핑 잔차 중앙값 0.120 mm, 위치 재현성 0.16 mm 다. 고전 CV 만 쓰며 딥러닝 모델은 아직 없다.

```bash
cd ~/g-shock-dev/dev/features/vision
./setup_venv.sh                                               # venv 가 없을 때 한 번만
DISPLAY=:1 PYTHONPATH=src ./venv/bin/python src/live.py       # 창 띄우기 (q 종료, m 마스크, s 저장)
PYTHONPATH=src ./venv/bin/python src/live.py --headless       # 수치만
```

세션 종료 시점에 `src/live.py` 창이 `DISPLAY=:1` 에 떠 있는 상태로 두었다.
남아 있으면 `pkill -f src/live.py` 로 정리한다.

---

## 1. 절대 규칙 — 환경

> **`apt upgrade` / `apt update` / `pip install --upgrade` / 시스템·`~/.local` 패키지 재설치 금지.**
> 버전 문제는 전부 `venv/` 안에서만 해결한다. 사용자가 명시적으로 지시한 사항이다.

JetPack 은 CUDA/TensorRT/cuDNN 이 커널·드라이버와 묶여 있어 하나만 올려도 스택이 깨진다.

### venv 가 어떻게 만들어졌나 (다시 만들어야 할 때)

이 장비에는 `python3.10-venv` 가 없어서 `python3 -m venv` 가 **실패한다** (`ensurepip` 없음).
apt 를 건드리지 않고 우회한 방법:

`setup_venv.sh` 가 이 절차를 그대로 수행한다. 내용은 다음과 같다.

```bash
WS=~/g-shock-dev/dev/features/vision
python3 -m pip install --target=$WS/.bootstrap virtualenv          # 작업공간 안에만
PYTHONPATH=$WS/.bootstrap python3 -m virtualenv --system-site-packages -p python3.10 $WS/venv
python3 -m site --user-site \
  > $WS/venv/lib/python3.10/site-packages/_user_local.pth          # ~/.local 경로만 연결
```

- `--system-site-packages` → 시스템 `tensorrt 10.3.0` 이 보인다
- `_user_local.pth` → `~/.local` 의 `cv2 4.11.0`, `numpy 1.26.4` 를 **재설치 없이** 재사용
- venv 는 기본적으로 `~/.local` 을 무시하므로 이 `.pth` 가 없으면 `cv2` 를 못 찾는다

---

## 2. 하드웨어·환경 사실 (재조사 불필요)

| 항목 | 값 |
|---|---|
| 보드 | Jetson AGX Orin Developer Kit, RAM 61 GB, 12 코어 |
| JetPack | 6.2.1 / L4T R36.4.3, CUDA 12.6, TensorRT 10.3.0 |
| Python | 3.10.12 |
| 디스플레이 | X11 `DISPLAY=:1` (세션 tty2, 사용자 phorce) |
| 카메라 | `/dev/video0` 만 사용 가능. `/dev/video1` 은 열리지 않음 (UVC 메타데이터 노드) |
| 해상도 | 640x480. 카메라가 30fps 라고 보고하지만 **실측 15~20 fps** |
| cv2 | 4.11.0, pip 휠, **GStreamer 미지원 빌드** → V4L2 백엔드로만 캡처 |

**카메라 fps 상한은 파이프라인 탓이 아니다.** `cap.read()` 자체가 평균 50 ms 걸린다.
MJPG / YUYV 둘 다 동일하므로 코덱 문제가 아니고, 저조도에서 오토노출이 노출시간을
늘리며 프레임레이트를 깎는 것이다. 조명을 밝히면 올라간다. 처리는 6 ms 라 43 ms 여유가 있다.

### 카메라 오토 화이트밸런스 — 가장 크게 물렸던 지점

오픈 직후 AWB/AE 가 수렴하기 전 프레임은 **색이 완전히 다르다.** 실측한 보드 Hue:

| 프레임 | 워밍업 | 보드 H | 보드 S |
|---|---|---|---|
| `data/frames/cam0.jpg` | 5 프레임 | 17 | 71 |
| `data/frames/now.jpg` | 10 프레임 | 33 | 28 |
| `data/frames/warm.jpg` | 2.5 초 | 46 | 101 |

**프레임 개수로 워밍업하면 안 된다. 시간으로 해야 한다** (`open_cam(warmup=2.5)`).
일단 수렴한 뒤에는 12 초간 S 표준편차 0.42 로 매우 안정적이며, AWB/AE 잠금은 없어도 된다
(잠금 자체는 `CAP_PROP_AUTO_WB=0`, `CAP_PROP_AUTO_EXPOSURE=1` 로 동작하는 것을 확인했다).

---

## 3. 대상과 좌표계

- **추적 대상은 보드 가운데 놓인 흰색 물체 하나** (실측 75.5 x 53.7 mm).
- 가장자리의 고정 브래킷은 **무시 대상**. 전부 화면 밖으로 잘려 나가므로
  `border_margin` 으로 테두리에 닿는 블롭을 버리는 것만으로 걸러진다.
  → 물체가 화면 가장자리까지 움직일 일이 생기면 이 규칙은 못 쓴다. 다른 기준 필요.
- **타공 간격 10 mm** (화면상 20.23 px, 0.4943 mm/px, 격자 회전 +0.30 deg).
- **좌표 원점이 아직 임의의 구멍이다.** 그래서 현재 물체 위치가 `(-15.9, -27.9) mm` 로 음수다.

---

## 4. 측정된 성능

| 항목 | 값 |
|---|---|
| 처리 시간 | 5.9 ms/frame (p95 7.0 ms, CPU 단일 코어) |
| 매핑 잔차 | 중앙값 0.120 mm / p95 0.345 mm / 최대 0.490 mm |
| 캘리브레이션 | 구멍 565개 검출, 462개 사용 |
| 위치 재현성 | 0.16 mm (화이트밸런스가 전혀 다른 두 프레임 간 동일 물체) |

---

## 5. 설계 판단과 근거 (실측으로 확정)

### 픽셀→mm 는 homography 가 아니라 3차 다항식

순수 homography 는 화면 바깥쪽에서 무너진다. 반경 구간별 실측:

| 중심에서 거리 | 잔차 중앙값 | inlier 비율 |
|---|---|---|
| 0–80 px | 1.02 px | 86 % |
| 240–320 px | 1.33 px | 49 % |
| 320–500 px | 2.33 px | 42 % |

렌즈 방사왜곡이다. 카메라와 보드가 **고정**이므로 내부파라미터 캘리브레이션 없이
3차 2D 다항식이 왜곡까지 통째로 흡수한다. 잔차가 1/4 로 줄었다.
homography 는 격자 인덱스를 확정하는 용도로만 남아 있다.

격자 간격은 구멍 검출과 무관하게 **자기상관(FFT)으로 독립 검증**했다 (x 20 px, y 19 px).
눈대중으로는 29 px 로 보였으나 실제는 20 px 였다. 반드시 재서 확인할 것.

### 트리밍 임계는 mm 가 아니라 픽셀 기준

`TRIM_PX = 1.0`. mm 절대값으로 두면 `--pitch-mm` 을 바꿀 때 실제 걸러지는 기준이
같이 흔들린다 (25.4 → 10 으로 바꿨을 때 0.94 px → 2.4 px 로 헐거워지는 것을 확인).

### 색 임계는 고정값이 아니라 매 프레임 추정

보드가 화면 최대 면적이라는 전제로 Hue 히스토그램 모드를 잡고, 채도·명도는 그 값의
비율로 임계를 건다 (`hue_tol`, `sat_ratio`, `val_ratio`). 통계량만 필요하므로
**4배 서브샘플링**해도 결과가 같고 훨씬 빠르다 (10 ms → 6 ms).

---

## 6. 실패한 시도 — 다시 하지 말 것

| 시도 | 결과 | 이유 |
|---|---|---|
| 고정 HSV 임계 (`H∈[3,35], S>70, V>60`) | **마스크가 화면 전체(100%)로 터짐** | AWB 수렴값에 따라 보드 Hue 가 16~46 으로 이동 |
| 채도만으로 분할 (`S<70`) | 검은 모터 본체를 놓침 | 어두운 픽셀은 `S=(max-min)/max` 가 노이즈로 튐 |
| 텍스처(국소 표준편차)로 분할 | 분리도가 물체마다 **방향이 뒤집힘** | 매끈한 물체는 낮고, 무늬 있는 물체는 배경보다 높음 |
| Lab 크로마 거리로 분할 | 프레임에 따라 분리도 0.16~1.01 로 들쭉날쭉 | 화이트밸런스 의존 |
| `np.where(lab==i)` 로 컴포넌트별 픽셀 수집 | 24.6 ms | 물체마다 전체 영상을 훑음 → `findContours` 로 교체해 5 ms |
| MAD 기반 축소 트리밍 | 455점 중 186점만 남음 | 반복마다 임계가 계속 줄어 과도하게 잘림 → 고정 임계로 교체 |
| `python3 -m venv` | 실패 | `python3.10-venv` 미설치 (apt 금지이므로 virtualenv 우회) |

---

## 7. 파일 맵

`git` 에 들어가는 것은 소스·설정·기준 프레임뿐이다.
`venv/` `.bootstrap/` `out/` 은 재생성 가능하므로 `dev/.gitignore` 에서 제외한다.

```
dev/features/vision/
├── README.md              설계 근거 요약
├── HANDOFF.md             이 문서
├── setup_venv.sh          venv 생성 (시스템 무변경)
├── venv/                  가상환경 (12 MB, git 제외)
├── .bootstrap/            virtualenv 부트스트랩용 (9.7 MB, git 제외)
├── config/
│   ├── pegboard.json      픽셀→mm 다항 계수, pitch, 잔차 통계
│   └── detect.json        (없어도 됨 — 없으면 detector.py DEFAULTS 사용)
├── data/frames/
│   ├── cam0.jpg           워밍업 5프레임. 모터 2개가 놓여 있던 초기 장면
│   ├── now.jpg            워밍업 10프레임. AWB 미수렴 (실패 재현용으로 보존)
│   └── warm.jpg           워밍업 2.5초. **캘리브레이션 기준 프레임**
├── out/                   시각화 산출물 (git 제외)
└── src/
    ├── pegboard.py        구멍 검출 → 격자 → 다항 매핑 생성. px_to_mm() 제공
    ├── detector.py        보드색 추정 → 분할 → 컨투어 → mm 좌표/자세
    └── live.py            웹캠 루프 + HUD (proc/fps/pos/size/jitter)
```

**캘리브레이션 재생성** (카메라를 움직였으면 반드시):

```bash
./venv/bin/python src/pegboard.py data/frames/warm.jpg --pitch-mm 10
```

---

## 8. 다음 단계

### 사용자 입력이 필요해 막혀 있는 것

- **좌표 원점·축 정렬.** 현재 원점은 임의의 구멍이라 좌표가 음수로 나온다.
  로봇에 좌표를 넘기려면 보드의 어느 모서리 또는 로봇 베이스를 기준으로 삼을지
  사용자가 정해줘야 한다. **이것 없이는 좌표를 외부로 내보낼 수 없다.**

### 바로 진행 가능한 것

- **`torch` 복구.** 시스템 `torch 2.11.0` 은 `libcudss.so.0` 이 없어 **import 자체가 실패**한다.
  `ultralytics 8.4.115` 가 깔려 있어도 학습·export 모두 불가. JetPack 6.2 용 휠을
  **`venv` 안에만** 설치할 것. TensorRT 10.3 자체는 정상이라 엔진 파일만 있으면 추론은 torch 없이 된다.
- **해상도 1280x720 상향.** mm 정밀도 약 2배. 처리 ~25 ms 예상이라 카메라 20 fps 안에 들어간다.
- **자동 라벨링 + YOLO11n-obb 학습.** 현 파이프라인 출력을 그대로 YOLO 라벨로 떨구면
  손라벨이 거의 필요 없다. torch 복구가 선행돼야 한다.

### 언제 딥러닝으로 넘어가야 하나

고전 CV 로 충분한 상태이며, 아래 중 **하나라도 발생하면** YOLO11n-obb 로 전환한다
(AGX Orin + TensorRT FP16 기준 3~5 ms 예상이라 성능 여유는 충분하다):

1. 물체 종류를 구분해야 할 때 (현재는 위치만, 클래스 개념 없음)
2. 물체끼리 닿아 하나의 블롭으로 합쳐질 때
3. 자세 정밀도가 더 필요할 때 — `minAreaRect` 는 가로세로가 비슷하면 각도가 90도 단위로 튄다.
   현재 대상은 75.5 x 53.7 mm 로 종횡비가 충분해 실제로는 문제가 나타나지 않고 있다 (θ 4도 안정).
