# packbot — 3축 로봇팔 박스 포장 자동화 (비전·판단 계층)

Jetson AGX Orin에서 웹캠(C270)으로 작업공간 위 흰색 인형을 검출하고,
16등분 구역을 판정해 해당 구역의 티칭 모션을 재생시키는 시스템.
토크 피드백으로 파지 성공을 확인하고, 실패 시 재검출부터 다시 수행한다.

## 구조

```
[카메라] → 작업공간 워핑(호모그래피) → 물체 검출 → 16구역 판정
                                                  ↓
                                      구역별 티칭 모션 재생 (PICK)
                                                  ↓
                                      토크 샘플링 → 파지 판정 (VERIFY)
                                        ↓실패                ↓성공
                              home 복귀 → 재검출          박스 투입 (PLACE)
                              (max_retries 초과 시 포기)
```

- `packbot/camera.py` — C270 캡처 (MJPG 720p, 노출·WB 고정)
- `packbot/workspace.py` — 4꼭짓점 호모그래피 워핑 + 4x4 격자 판정
- `packbot/detector_seg.py` — **FastSAM 세그멘테이션** 검출 (기본, GPU)
- `packbot/detector.py` — HSV 색 임계값 검출 (폴백, CPU)
- `packbot/grasp.py` — 토크 기반 파지 성공 판정
- `packbot/fsm.py` — DETECT→PICK→VERIFY→PLACE 상태머신 (실패 재시도 포함)
- `packbot/backends.py` — 티칭 재생/토크 읽기 어댑터 (dummy | serial)
- `main.py` — 메인 루프

## 처음 셋업 순서

```bash
# 1. 작업공간 네 꼭짓점 클릭 (좌상→우상→우하→좌하)
python3 tools/calibrate_workspace.py

# 2. 검출 확인/튜닝 (d: seg↔hsv 전환, s: hsv 임계값 저장)
python3 tools/tune_detector.py

# 3. 빈손 토크 기준치 측정 (로봇을 들어올린 자세로)
python3 tools/calibrate_torque.py --save
#    물건 잡고 한 번 더 돌려서 (저장 없이) 차이 확인 → delta_threshold 결정

# 4. 로봇 없이 비전만 시험
python3 main.py --backend dummy

# 5. 실전
python3 main.py
```

## 실제 제어기 연동

`packbot/backends.py` 의 `SerialMotionBackend` / `SerialTorqueBackend` 에서
`TODO` 표시된 프로토콜 인코딩/파싱 부분만 실제 티칭 모듈 명령 형식으로
교체하면 된다. 상태머신·비전 코드는 수정 불필요.

`config/config.yaml` 의 `motion.backend: serial` 로 바꾸고
`zone_motions` 에 구역별(0~15) 티칭 모션 이름을 채운다.

## 구역 번호

```
 0  1  2  3
 4  5  6  7
 8  9 10 11
12 13 14 15
```
(카메라에서 워핑된 화면 기준 좌상단이 0)

## 검출 방식

- **seg (기본)**: FastSAM-s (클래스 무관 세그멘테이션) → 마스크별 흰색 비율
  필터. 커스텀 인형이라 학습 없이 동작. 조명 변화·그림자에 강함.
- **hsv (폴백)**: 채도 낮고 명도 높은 픽셀 = 흰색. torch 없이 동작.
  `--detector hsv` 로 전환 가능.
