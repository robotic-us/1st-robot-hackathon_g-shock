# 인수인계 — g-shock 모션 플래닝 작업 기록

작성 2026-08-07. 다음 세션이 이 문서만 읽고 이어서 작업할 수 있게 쓴 기록입니다.

---

## 0. 한 줄 요약

비전이 준 물체 위치로 양팔 grab 모션을 만드는 파이프라인을 `~/g-shock-dev` 에
구축·검증 완료. **다음 할 일은 비전↔로봇 좌표 변환 실측**이고, 그게 되기 전에는
실기 재생 불가.

## 1. 워크스페이스

| | 경로 | 성격 |
|---|---|---|
| 원본 | `~/Downloads` (git dir `.g-shock.git`) | **건드리지 않음** |
| 작업 | `~/g-shock-dev` | 독립 클론, 브랜치 `dev/new-features` |

`git clone --no-hardlinks` 로 뜬 완전히 독립된 저장소입니다. 원본에는 아무것도
쓰지 않았습니다(확인: `main/` 변경 0건, HEAD `911830e` 그대로).
원격은 `origin`(GitHub) + `baseline`(로컬 원본).

```
~/g-shock-dev/
├── CLAUDE.md          작업 지침 (패키지 금지 / 안전 / domain)
├── HANDOFF.md         이 문서
├── main/              원본 기준 코드 — 여기서 고치지 말 것
└── dev/
    ├── README.md      워크스페이스 사용법
    ├── env.sh         공통 환경 (domain 21) — source 해서 씀
    ├── bin/           robot-stack.sh · sim-up.sh · sim-down.sh · doctor.sh
    ├── catalog/       슬롯 2/3/5/49/50 CSV (motion_dir 로 넘길 평면 배치)
    ├── features/grab_planner/   ★ 이번에 만든 것
    └── tests/         테스트 3종
```

커밋 3개: `34b5073`(워크스페이스) → `f97fe58`(domain 21) → `1d0ae5a`(grab_planner).

---

## 2. 알아낸 사실 — 전부 실측/로그 근거 있음

### 2.1 로봇에 실린 모션은 8개

`phorce list` 가 이름 없이 `(이름 없음 — 로봇에 적재된 슬롯 N)` 만 보여주는 이유는
`motion_action_server` 가 `motion_dir` 없이 떠서입니다. 슬롯 자체는 있습니다.

권위 있는 값은 `phorce_monitor` 가 기동 시 EtherCAT 메일박스로 PCM 에서 직접 읽습니다
(네트워크를 안 타므로 신뢰 가능):

```
모션 창구 확인 — playable 1..50, ... 적재 슬롯 마스크 0x000700000000003E
  → 슬롯 [1, 2, 3, 4, 5, 48, 49, 50]
```

로컬 CSV 와의 대응: 2=grab, 3=box, 5=BUTTON1_READY, 49=grab_test, 50=IK.
1·4·48 은 대응하는 로컬 CSV 가 없습니다(기존 적재분).

`-p motion_dir:=dev/catalog` 를 주면 이름이 붙는 것을 sim 에서 확인했습니다.
**남은 확인:** 실기(ecat)에서 `motion_dir` 을 주면 로컬 CSV 가 없는 1/4/48 이
목록에서 사라지는지. sim 에는 조회할 실기가 없어 sim 으로는 답이 안 나옵니다.

### 2.2 ⚠️ cancel 이 항상 거부됩니다 — 안전 관련

`logs/motion_action_server.log` 에 남은 5번의 취소 시도가 전부 거부됐습니다.

> cancel 을 거부했습니다 — pcm 은 EtherCAT 정지를 지원하지 않습니다.
> 진행 중인 모션은 끝까지 갑니다.

즉 `shock_guard_gui.py` 의 존재 이유인 **DOB 충격 감지 자동정지가 실물에서 동작하지
않습니다.** GUI 가 "취소 요청 수락 대기" 를 표시해도 로봇은 완주합니다.
유일한 정지 수단은 물리 E-Stop 이고, 눌리면 래치되어 재부팅·재호밍이 필요합니다.

### 2.3 domain 0 을 쓰면 남의 팀 로봇과 섞입니다

`phorce list` 슬롯 수가 8 → 26 → 1 로 튀는 현상의 원인입니다. 로봇이 바뀐 게
아니라 다른 팀 서버가 번갈아 응답한 것입니다.

- 로컬 `motion_action_server` 프로세스 1개인데 그래프에는 3개
- `motion_action_server` 가 `10.249.184.90`(공유 WiFi)에 바인딩, `ROS_LOCALHOST_ONLY=0`
- `eno1`(로봇 EtherCAT)은 IP 가 없어 원격 경로가 아님
- domain 21 은 비어 있음

원인은 `main/check_robot_communication.sh` 에 `ROS_DOMAIN_ID` 설정이 없다는 것
(`main/run_integrated_system.sh` 에는 21 이 있음). `dev/bin/robot-stack.sh` 가 메웁니다.

**이 상태에서 `phorce play` 는 남의 로봇을 움직일 수 있습니다.** domain 격리 전엔 금지.

### 2.4 grab 은 그리퍼가 아니라 양팔 핀치

슬롯 2 `grab` 모션을 디코드해 나온 구조 (월드 EE 좌표):

```
wp1 ready          A=(-55,  0,210.6)  B=(+55,  0,210.6)
wp2 벌려서 접근     A=(-60,  0,120  )  B=(+60,  0,120  )
wp3 좁혀서 집기     A=(-30,  0,120  )  B=(+30,  0,120  )   ← 간격 60mm
wp4 들어올림        A=(-30,  0,140  )  B=(+30,  0,140  )
wp5 이송           A=(-30,110,120  )  B=(+30,110,120  )
wp6 내려놓기        A=(-30,110, 90  )  B=(+30,110, 90  )
wp7 벌려서 후퇴     A=(-60,110,120  )  B=(+60,110,120  )
wp8 ready
```

손목이 없어 EE 자체는 못 돌립니다. 대신 두 팔의 목표점을 회전시키면 **집는 축
방향**은 바꿀 수 있습니다(`--grip-angle` 로 구현).

집는 간격이 60mm 이므로 **집는 축 방향 물체 폭이 60mm 를 넘어야 물립니다.**
비전 물체 75.5 × 53.7mm 기준 — 긴 축을 가로지르면 물림량 15.5mm(잡힘),
짧은 축을 가로지르면 안 닿아서 빠집니다.

### 2.5 ⚠️ 실시간 비전→모션 루프가 불가능합니다

이 로봇은 **미리 SD 에 적재된 슬롯만 재생**합니다.

- `PlayMotionSequence` 액션 = 슬롯 ID 만. 궤적 데이터 못 보냄.
- 내부 서비스 `SubmitMotionRequest` 도 `uint8 motion_id` 뿐. 메시지 주석에
  *"이 서비스는 참가자 API 가 아니다"* 명시.
- RT 관절 스트리밍 `PhorceCommand` 는 죽어 있음. 메시지 주석:
  *"지금은 pcm ctrl frame 자체가 나가지 않는다 — `_FlushOutputs` 호출부가 삭제된
  상태가 **해커톤 의도 구성**이다(P-Vector 전용)."*

모션을 바꾸려면 CSV 생성 → SD 설치 → **SD 를 PCM 에 물리적으로 옮기고 PCM 재시작**.
따라서 실시간 데모는 미리 격자로 슬롯을 깔고 가장 가까운 것을 고르는 방식뿐입니다.

### 2.6 격자 방식의 실측 한계

빈 슬롯 42개(6~47)로 덮을 수 있는 영역:

| 작업 영역 | 자동 선택 간격 | 최대 위치 오차 |
|---|---|---|
| **60 × 60 mm** | 11 mm | **7.1 mm** ← 현실적 한계 |
| 80 × 80 mm | 14 mm | 14.1 mm |
| 100 × 100 mm | 17 mm | 21.2 mm |
| 140 × 140 mm | 23 mm | 21.1 mm |

무는 여유가 15.5mm 이므로 60×60mm 까지가 실용적입니다.
양팔 핀치 도달 영역 자체는 `X ∈ [-70,+70], Y ∈ [-60,+160] mm` (Z=120, 반간격 30).

---

## 3. 만든 것

### `dev/features/grab_planner/`

| 파일 | 역할 |
|---|---|
| `kinematics.py` | FK/IK/좌표변환. motion_builder 계산을 GUI 없이 뽑은 것 |
| `grab.py` | 8단계 대칭 핀치 플랜. `grip_angle` 로 집는 축 회전, `grip_clearance` 로 물림 검사 |
| `csvgen.py` | P-Vector CSV + memo 생성/역파싱 |
| `frame.py` | 비전 보드 좌표 → 로봇 월드 2D 강체변환, 대응점 최소제곱 해법 |
| `plan_grab.py` | CLI — `reach` / `plan` / `grid` / `nearest` |
| `VISION_INTERFACE.md` | 비전 쪽 계약과 확인 필요 항목 ★ |

### `dev/bin/`

| 파일 | 역할 |
|---|---|
| `robot-stack.sh` | 실기 스택을 domain 21 로 기동. domain 0 이면 거부. 모션 goal 미전송 |
| `sim-up.sh` / `sim-down.sh` | `backend:=sim` 안전 스택. 실물 안 움직임 |
| `doctor.sh` | domain 충돌·카탈로그·안전 진단 한 장 |

---

## 4. 검증한 것 / 못 한 것

**검증 완료**

- 기구학: 준비자세 EE 가 README 값 `(±55, 0, 210.6)` 과 일치, IK→FK 왕복 오차 `1e-13mm`
- CSV 포맷: 기존 슬롯 4개(02/03/49/50) 재생성 후 **바이트 일치**
  (`motion_05` 만 소수자리 다름 — 구버전 motion_builder 산출물이라 정상)
- 플랜 불변식: 물체 중심 = 두 EE 중점, 집은 간격 = 2×close_half_gap, 각도 ±360° 이내
- 좌표 변환: θ·tx·ty 복원 잔차 `1e-14mm`
- 종단: 비전좌표 → CSV → sim 카탈로그 → `phorce play` → `SUCCEEDED`
- 테스트 3종 전부 통과 (`test_grab_planner.py` 29개 포함)

**검증 못 함 — sim 으로는 불가능**

- **실제 관절 가동범위.** 플래너는 `±360°` 만 검사합니다. 기구 한계는 모릅니다.
- **자기충돌.** 두 팔이 서로/구조물과 부딪히는지 검사하지 않습니다.
- 실제 물체를 무는 힘·미끄러짐.

실기 투입 전 사람이 눈으로 확인해야 하고, cancel 이 거부되므로 발사하면 끝까지 갑니다.

---

## 5. 다음 세션에서 할 일 (우선순위)

1. **비전↔로봇 좌표 변환 실측** ← 이거 없이는 아무것도 못 함
   펙보드 위 점 3개를 비전 `(u,v)` 와 로봇 `(X,Y)` 양쪽에서 읽어
   `frame.solve_from_pairs()` 로 풀고 `frame.save()`. 잔차 2mm 이하 목표.
   로봇 쪽 값은 `main/motion_builder/motion_builder.py` 의 `현재각 → EE` 버튼.
2. **집는 높이 `grasp_z` 실측** — 기본 120mm 는 기존 grab 모션 값. 펙보드+물체 높이로 재측정.
3. **무는 간격 `close_half_gap` 조정** — 기본 30(간격 60mm)을 실제 물체 폭에 맞춤.
4. **놓을 자리 `place_xy` 결정** — 기본 (0, 110) 도 기존 모션 값.
5. **방식 A/B 선택** — 실시간 격자(60×60mm 제한) vs 매번 재생성(SD 왕복 수 분).
6. **실기에서 `motion_dir` 실험** — 슬롯 1/4/48 이 목록에서 사라지는지 (2.1 참조).
   재생하지 않고 `phorce list` 만 하므로 로봇은 안 움직임.
7. 관절 가동범위·자기충돌 검사를 플래너에 추가.

## 6. 사용자에게 물어봐야 할 것

- 물체를 항상 같은 방향으로 놓을 수 있는지 (격자 방식은 집는 축이 고정)
- 작업 영역을 60×60mm 로 좁힐 수 있는지, 아니면 정밀 방식으로 갈지
- 놓을 목표 위치가 어디인지

---

## 7. 주의사항

- **시스템 패키지 금지.** `apt update/upgrade`, `pip --upgrade`, `~/.local` 재설치 전부
  금지. 필요하면 작업공간 안에 venv (`CLAUDE.md` 에 방법 있음).
  이 장비는 `python3.10-venv` 가 없어 `virtualenv` 를 작업공간에 받아 써야 함.
- **`~/Downloads` 와 `main/` 은 건드리지 않는다.**
- **domain 21.** 실기 스택은 `dev/bin/robot-stack.sh` 로.
- 실기 스택은 2026-08-07 18:49 기준 **내려가 있음**. 재시작 필요.
- 비전 작업공간은 `/home/phorce/vision_ws` (별도 세션에서 진행 중).
  그쪽 기록은 `/home/phorce/vision_ws/HANDOFF.md` — 좌표계·실측 성능·실패 기록이
  거기 있습니다. 좌표 변환(§5 1번)을 할 때 양쪽 문서를 같이 봐야 합니다.

## 8. 자주 쓰는 명령

```bash
cd ~/g-shock-dev && source dev/env.sh
./dev/bin/doctor.sh                     # 상태 한 장
./dev/bin/robot-stack.sh                # 실기 스택 (domain 21)
./dev/bin/sim-up.sh myfeature           # 안전 테스트 스택
phorce list --domain-id 21

python3 dev/tests/test_grab_planner.py  # 29개
./dev/tests/test_grab_sim_e2e.sh        # 종단 검증

cd dev/features/grab_planner
./plan_grab.py reach
./plan_grab.py plan --board 12.5,-8 --grip-angle 121 --object-width 75.5 --slot 11
./plan_grab.py grid --slots 6-47 --x-range -30 30 --y-range 10 70
./plan_grab.py nearest --board 12.5,-8
```
