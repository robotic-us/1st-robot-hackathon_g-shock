# dev — 새 기능 개발/테스트 워크스페이스

`~/Downloads/main` 의 동작하는 코드를 건드리지 않고 새 기능을 만들고 검증하는 공간입니다.

## 기존과의 관계

| | 경로 | 성격 |
|---|---|---|
| 원본 | `~/Downloads` (git dir `.g-shock.git`) | 손대지 않음 |
| 여기 | `~/g-shock-dev` | 독립 클론, 브랜치 `dev/new-features` |

`git clone --no-hardlinks` 로 뜬 **완전히 독립된 저장소**입니다. 원본 저장소에는 아무것도
쓰지 않습니다. 원격은 두 개입니다.

- `origin` → GitHub `robotic-us/1st-robot-hackathon_g-shock`
- `baseline` → 로컬 원본 `~/Downloads/.g-shock.git` (원본이 갱신되면 `git fetch baseline`)

`main/` 은 원본에서 그대로 따라온 기준 코드입니다. **여기서 고치지 말고**, 새 기능은
`dev/features/` 에 만들고 검증된 뒤에 옮기세요.

## 시작하기

```bash
cd ~/g-shock-dev
source dev/env.sh          # 환경 (domain 21, namespace, 경로)
./dev/bin/doctor.sh        # 실기·sim·카탈로그·domain 충돌·안전 상태 한 장
```

## domain 은 반드시 21 — domain 0 금지

이 장비는 공유 WiFi(`10.249.184.0/24`)에 붙어 있고 `ROS_LOCALHOST_ONLY=0` 이라
**ROS 그래프가 네트워크 전체로 퍼집니다.** domain 0 은 모두의 기본값이라, 0 으로 띄우면
같은 해커톤 네트워크의 **다른 팀 로봇과 그래프가 합쳐집니다.**

실기 스택은 이 래퍼로 띄우세요. domain 21 을 보장하고, 0 이면 아예 거부합니다.

```bash
./dev/bin/robot-stack.sh              # = check_robot_communication.sh + domain 21
phorce list --domain-id 21            # 조회
```

셸에 `source dev/env.sh` 를 했다면 `ROS_DOMAIN_ID=21` 이 이미 잡혀 있어서
`--domain-id 21` 없이 `phorce list` 만 해도 같습니다. 두 방법 다 됩니다.

## sim 에서 안전하게 테스트

`backend:=sim` 은 mechanism-neutral fake PCM 이라 **로봇이 전혀 움직이지 않습니다**
(모션 1건당 800ms 로 완주 처리). 새 기능은 여기서 먼저 돌리세요.

```bash
./dev/bin/sim-up.sh              # 세션 'dev' 기동 (기본값)
./dev/bin/sim-up.sh myfeature    # 다른 세션도 동시에 가능

phorce list   --target sim:dev
phorce play 2 --target sim:dev
phorce status --target sim:dev

./dev/bin/sim-down.sh            # 전부 내리기
```

sim 은 `/sim/<세션>` namespace 로 뜨므로 실기(root namespace)와 **같은 domain 을 써도
충돌하지 않습니다**. 실기 스택을 띄운 채로 sim 을 병행해도 됩니다.

## grab_planner — 비전 좌표로 양팔 grab 모션 만들기

`dev/features/grab_planner/` 에 있습니다. 비전이 준 물체 위치를 받아 양팔 핀치
모션을 만들어 PCM 슬롯 파일(`motion_NN.csv` + memo)로 떨굽니다.

```bash
cd dev/features/grab_planner
./plan_grab.py reach                                   # 집을 수 있는 영역
./plan_grab.py plan --x 15 --y 45 --slot 11 --dry-run  # 한 지점 미리보기
./plan_grab.py grid --slots 6-47 --x-range -30 30 --y-range 10 70
./plan_grab.py nearest --board 12.5,-8                 # 비전 좌표 → 재생할 슬롯
```

인터페이스와 제약, 그리고 **사용자가 실측해야 하는 항목**은
[VISION_INTERFACE.md](features/grab_planner/VISION_INTERFACE.md) 에 정리돼 있습니다.
핵심만 옮기면:

- 이 로봇은 **그리퍼가 없고**, grab 은 양팔이 물체를 양옆에서 무는 대칭 핀치입니다.
  집는 순간 간격이 60mm 이므로 **집는 축 방향 물체 폭이 60mm 보다 커야** 물립니다.
- 손목이 없어 EE 는 못 돌리지만, 두 팔의 목표점을 회전시켜 **집는 축 방향**은
  바꿀 수 있습니다 (`--grip-angle`).
- **실시간 루프는 불가능합니다.** 이 로봇은 미리 SD 에 적재된 슬롯만 재생합니다.
  액션도 내부 서비스도 슬롯 ID 만 받고, RT 관절 스트리밍은 해커톤 구성에서
  꺼져 있습니다(`PhorceCommand.msg` 주석: *"pcm ctrl frame 자체가 나가지 않는다"*).
  → 미리 격자로 슬롯을 깔고 가장 가까운 것을 고르는 방식(방식 A)이 현실적입니다.
- 빈 슬롯 42개로 덮을 수 있는 영역은 **약 60×60mm (최대 오차 7.1mm)** 입니다.

`out/` 과 `slot_grid.json` 은 생성물이라 git 에 넣지 않습니다. 위 `grid` 명령으로
언제든 다시 만들 수 있습니다.

## 테스트

```bash
./dev/tests/test_catalog_naming.sh      # motion_dir 이 슬롯 이름을 붙이는가
python3 dev/tests/test_grab_planner.py  # 기구학·CSV 포맷·플랜·좌표변환 (29개)
./dev/tests/test_grab_sim_e2e.sh        # 비전좌표 → CSV → sim 재생 종단 검증
```

## 이 워크스페이스가 이미 밝혀낸 것

**1. 슬롯 이름이 안 보이는 원인과 해법.** 실기 서버는
`ros2 run agx_motion_slot motion_action_server --ros-args -p backend:=ecat` 로만 떠서
`motion_dir` 이 비어 있습니다. 그래서 `phorce list` 가 8개 슬롯(1,2,3,4,5,48,49,50)을
전부 보여주면서도 이름은 `(이름 없음 — 로봇에 적재된 슬롯 N)` 입니다.
`shock_guard_gui.py` 는 이름이 **정확히** `GRAB`/`BOX`/`TAPE`/`HOME` 인 슬롯만 버튼에
연결하므로 이 상태에서는 버튼 4개가 전부 비활성입니다.

`-p motion_dir:=dev/catalog` 를 주면 이름이 붙는 것을 sim 에서 확인했습니다
(`grab`/`box`/`BUTTON1_READY`/`grab_test`/`IK`).

아직 확인 안 된 것: **실기(ecat)에서 `motion_dir` 을 주면 로컬 CSV 가 없는 슬롯
1/4/48 이 목록에서 사라지는지.** sim 에는 조회할 실기가 없어서 이 조합은 sim 으로
답이 안 나옵니다. 실기 서버를 `motion_dir` 을 주고 재기동한 뒤 `phorce list` 로
확인해야 합니다(목록 조회만 하므로 로봇은 움직이지 않습니다).

**2. 이름을 붙여도 버튼 4개가 다 살아나지는 않습니다.** CSV 의 MS Name 이
`grab`/`box` 는 대문자 정규화로 매칭되지만 `grab_test`/`IK`/`BUTTON1_READY` 는
별칭 목록에 없고, `TAPE` 와 `HOME` 계열 슬롯은 아예 만들어진 적이 없습니다.
별칭 표를 고치거나 해당 모션을 만들어야 합니다.

**3. `phorce list` 가 남의 팀 로봇을 보여주던 문제 (2026-08-07).** 슬롯 개수가 호출할
때마다 8개 → 26개 → 1개로 튀는 현상이 있었습니다. 로봇이 바뀐 게 아니라 스택이
domain 0 으로 떠 있어서 **다른 팀 서버가 번갈아 응답**한 것입니다.

- 로컬 `motion_action_server` 프로세스는 1개인데 그래프에는 3개가 보였습니다.
- `motion_action_server` 가 `10.249.184.90`(공유 WiFi)에 바인딩돼 있었습니다.
- `eno1`(로봇 EtherCAT)은 IP 가 없어 원격 경로가 아닙니다.

우리 로봇의 진짜 슬롯은 `phorce_monitor` 가 기동 시 PCM 에서 직접 읽습니다.
네트워크를 안 타므로 이 값이 신뢰할 수 있는 기준입니다.

```
모션 창구 확인 — playable 1..50, ... 적재 슬롯 마스크 0x000700000000003E
  → 슬롯 [1, 2, 3, 4, 5, 48, 49, 50] = 8개
```

원인은 `main/check_robot_communication.sh` 에 `ROS_DOMAIN_ID` 설정이 없다는 것입니다
(`main/run_integrated_system.sh` 에는 21 이 있습니다). `dev/bin/robot-stack.sh` 가 이걸
메웁니다. `dev/bin/doctor.sh` 는 그래프의 서버 수가 로컬 프로세스 수보다 많으면
경고하므로 재발을 바로 잡아냅니다.

## ⚠️ 실물 안전

`logs/motion_action_server.log` 기준으로 **pcm 은 EtherCAT 정지를 지원하지 않습니다.**
액션 cancel 은 예외 없이 거부되며, 서버가 직접 이렇게 경고합니다.

> cancel 을 거부했습니다 — pcm 은 EtherCAT 정지를 지원하지 않습니다.
> 진행 중인 모션은 끝까지 갑니다.

즉 `shock_guard_gui.py` 의 존재 이유인 **DOB 충격 감지 자동 정지가 실물에서 동작하지
않습니다.** GUI 가 "취소 요청 수락 대기" 를 표시해도 로봇은 모션을 완주합니다.
실물의 유일한 정지 수단은 물리 E-Stop 이고, 눌리면 래치되어 재부팅·재호밍이 필요합니다.

새 기능은 반드시 sim 에서 먼저 검증하고, 실기 투입은 별도로 판단하세요.

## 구조

```
dev/
├── env.sh       source 해서 쓰는 공통 환경 (domain 21)
├── bin/         robot-stack.sh · sim-up.sh · sim-down.sh · doctor.sh
├── catalog/     motion_dir 로 넘길 평면 카탈로그 (motion_NN.csv + memo)
├── features/    새 기능 코드 — 여기에 만드세요
├── tests/       테스트 스크립트
└── logs/        런타임 로그·pidfile (git 무시)
```
