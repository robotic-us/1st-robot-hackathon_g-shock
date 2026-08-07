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
source dev/env.sh          # 환경 (domain, namespace, 경로)
./dev/bin/doctor.sh        # 실기·sim·카탈로그·안전 상태 한 장
```

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

## 테스트

```bash
./dev/tests/test_catalog_naming.sh
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
├── env.sh       source 해서 쓰는 공통 환경
├── bin/         sim-up.sh · sim-down.sh · doctor.sh
├── catalog/     motion_dir 로 넘길 평면 카탈로그 (motion_NN.csv + memo)
├── features/    새 기능 코드 — 여기에 만드세요
├── tests/       테스트 스크립트
└── logs/        런타임 로그·pidfile (git 무시)
```
