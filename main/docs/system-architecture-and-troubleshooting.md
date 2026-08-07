# G-SHOCK 통합 제어 시스템 구성과 트러블슈팅

## 1. 목표와 현재 상태

이 시스템은 실물 PHORCE 로봇에서 다음 기능을 하나의 운영 GUI로 통합한다.

- EtherCAT 기반 실시간 상태 수집과 DOB 충격 감시
- PCM SD카드에 적재된 모션 슬롯 조회 및 ROS 2 Action 실행
- PCM USB-CDC를 통한 ARM(서보 ON)과 즉시 OFF
- HOME 모션 성공을 확인한 뒤에만 토크를 해제하는 `HOME → UNARM`

ROS/EtherCAT 통신과 모션 카탈로그 로딩은 확인되었다. USB ARM도 PCM 재시작 직후에는
동작했으나, 물리 2번 버튼 및 PCM 재시작 이후 USB gadget이 재열거되지 않는 경우가
관찰되었다. 따라서 현재 버전은 소프트웨어 세션 복구를 보강했지만, PCM 펌웨어/USB
gadget 수준의 재열거 문제는 운영 절차와 추가 검증이 필요하다.

## 2. 시스템 구조

```text
통합 실행기 run_integrated_system.sh
├─ shock_guard_gui.py
│  ├─ ROS 2 Action Client ──> /motion_action_server/play_motion_sequence
│  ├─ ROS 2 Subscriber ────> /phorce/feedback
│  ├─ ROS 2 Subscriber ────> /motion_action_server/motion_slot_state
│  └─ PcmUsbServoClient ────> /dev/ttyACM* (USB-CDC SDO)
└─ check_robot_communication.sh
   ├─ phorce_monitor ───────> EtherCAT, OP_IDLE, 1 kHz feedback
   └─ motion_action_server ─> PCM motion slot gateway
```

제어 경로는 두 개로 분리된다.

1. **모션 경로:** GUI → ROS 2 Action → `motion_action_server` → EtherCAT → PCM
2. **ARM/OFF 경로:** GUI → `PcmUsbServoClient` → USB-CDC → PCM Object Dictionary

`즉시 OFF`가 되는데 모션 실행과 `HOME → UNARM`이 함께 실패할 수 있는 이유가 바로
이 경로 분리다. `HOME → UNARM`은 먼저 EtherCAT 모션 슬롯 1을 성공시킨 다음 USB로
서보 OFF를 요청한다.

## 3. 실행 구성

통합 실행은 다음 명령 하나로 시작한다.

```bash
cd /home/phorce/Downloads/main
./run_integrated_system.sh
```

기본 설정은 다음과 같다.

- `ROS_DOMAIN_ID=21`
- EtherCAT NIC: `eno1`
- 모니터 모드: `op_idle`
- 축: `auto` 감지
- PCM USB VID/PID: `0483:5741`
- PCM SD 모션 경로: `/media/phorce/9016-4EF8/Motions`

환경이 다르면 다음 변수로 덮어쓸 수 있다.

```bash
PHORCE_PCM_PORT=/dev/ttyACM0 \
PHORCE_MOTION_DIR=/media/phorce/9016-4EF8/Motions \
./run_integrated_system.sh
```

통신 스크립트는 PREEMPT_RT, CPU 격리, NIC 링크/IRQ/offload, ROS 실행 파일,
피드백 수신 및 Action 서버 노출을 확인한다. 안전 검사는 모션 Goal을 보내지 않는다.

## 4. SD 모션 카탈로그

PCM SD카드의 `Motions` 디렉터리에는 슬롯 1, 2, 3, 4, 5, 48, 49, 50이 확인됐다.
주요 슬롯은 다음과 같다.

| 슬롯 | MS Name | GUI 용도 |
|---:|---|---|
| 1 | `home` | HOME 및 HOME → UNARM |
| 2 | `grab` | GRAB |
| 3 | `box` | BOX |
| 5 | `BUTTON1_READY` | 준비 자세 |
| 49 | `grab_test` | 시험 모션 |
| 50 | `IK` | IK 시험 |

CSV의 SHA-256과 `.memo.json`의 `motion_sha256`은 전 슬롯에서 일치했다. 즉, 당시
모션 실행 실패는 SD 파일 손상이 아니었다.

## 5. ARM 및 UNARM 설계

ARM은 phorce Studio에서 관찰한 순서를 따른다.

1. PCM USB 복합장치에서 SD 마운트 식별
2. CDC 포트를 닫고 SD 볼륨을 안전 해제
3. CDC 포트를 다시 열고 DTR 설정
4. `0x5F0C` HELLO/STATUS 확인
5. `LIVE_BEGIN` 전송 및 Studio LIVE 상태 확인
6. `0x5F08:01=1`로 서보 ON 요청
7. `0x5F08:02`를 폴링하여 실제 `SERVO_ON` 확인

Write ACK만으로 ARM 성공을 판정하지 않는다. 실제 상태가 12초 안에 ON으로 확인되지
않으면 실패로 처리한다.

`HOME → UNARM`은 슬롯 1 Action 결과가 성공(`status=0`)일 때만
`0x5F08:01=0`을 전송한다. HOME이 거절·중단·취소되면 로봇이 임의 자세에서 무너지는
것을 막기 위해 토크를 유지한다. 별도의 `즉시 OFF`는 홈 복귀 없이 토크를 제거하는
안전 방향의 수동 수단이다.

## 6. 발생 문제와 해결 과정

### 6.1 ARM과 즉시 OFF는 되지만 모션 및 UNARM 실패

**증상**

- USB ARM과 즉시 OFF는 동작
- 모션 버튼이 비활성 또는 모션 실행 불가
- `HOME → UNARM`도 동작하지 않음

**원인**

ROS 노드가 종료된 경우가 있었고, 이후 노드가 정상일 때도 Action 서버에
`motion_dir`가 전달되지 않았다. 서버는 EtherCAT 슬롯 비트맵만 읽어 슬롯 존재는
알았지만 이름을 모두 `이름 없음`으로 반환했다. GUI는 이름을 근거로 HOME/GRAB/BOX를
안전하게 매핑하므로 버튼을 활성화할 수 없었다.

**해결**

`check_robot_communication.sh`가 Action 서버를 다음과 같이 기동하도록 수정했다.

```bash
ros2 run agx_motion_slot motion_action_server --ros-args \
  -p backend:=ecat \
  -p motion_dir:=/media/phorce/9016-4EF8/Motions
```

경로가 없으면 조기에 실패하고 `PHORCE_MOTION_DIR` 지정 방법을 안내한다.

### 6.2 첫 ARM 후 물리 2번 버튼을 누르면 다음 ARM 타임아웃

**증상**

- PCM 재시작 직후 첫 ARM은 성공
- 물리 2번 버튼으로 파킹한 뒤 다음 ARM은 12초 타임아웃

**원인**

클라이언트가 첫 ARM 후 `_studio_live=True`를 캐시했다. 물리 2번 버튼은 USB
descriptor를 유지한 채 PCM을 Studio LIVE에서 스토리지/파킹 모드로 되돌릴 수 있다.
클라이언트는 이 전이를 모르고 다음 ARM에서 LIVE 재협상을 생략했다.

**해결**

매 ARM 요청 전 `0x5F0C` 세션 STATUS를 다시 읽는다. PCM이 LIVE/READY가 아니거나
상태 읽기가 타임아웃이면 캐시를 폐기하고 SD 해제, CDC 재연결, LIVE_BEGIN을 처음부터
다시 수행한다. 이 동작을 단위 테스트로 고정했다.

### 6.3 PCM 재시작 후 GUI에서 USB 포트를 찾지 못함

**관찰 결과**

- `/dev/ttyACM0` 없음
- `lsusb`에 `0483:5741` 없음
- `/dev/sda` 및 sysfs block device도 없음
- 이전 SD 마운트 기록만 잠시 남고 실제 디렉터리는 접근 불가

**판정**

이는 GUI 포트 선택 문제가 아니라 PCM USB 복합장치 전체가 호스트 USB 버스에
재열거되지 않은 상태다. CDC 인터페이스만 차단된 것이 아니다.

**현재 대응 절차**

1. GUI와 통신 프로세스를 종료한다.
2. PCM USB 데이터 케이블을 분리하고 3~5초 후 다시 연결한다.
3. `lsusb`에서 `0483:5741`, `/dev/ttyACM*`, SD 마운트를 확인한다.
4. 그래도 없으면 PCM 전원과 USB를 모두 분리한 뒤 완전 재부팅한다.

이 문제는 호스트 애플리케이션이 존재하지 않는 포트를 다시 여는 방식으로 해결할 수
없다. PCM 펌웨어의 USB gadget 재기동 또는 하드웨어 연결 순서에 대한 추가 검증이
필요하다.

## 7. 검증 명령

```bash
env ROS_DOMAIN_ID=21 ROS_LOG_DIR=/tmp/phorce-ros-log phorce doctor
python3 -m unittest -v test_pcm_usb_servo.py
bash -n check_robot_communication.sh run_integrated_system.sh
```

USB 장애를 구분할 때는 다음 세 계층을 함께 본다.

```bash
lsusb
ls -l /dev/ttyACM* /dev/sd* 2>/dev/null
findmnt -rn -o SOURCE,TARGET,FSTYPE
```

## 8. 발표 핵심 메시지

- 실물 로봇 제어에서는 “명령을 보냈다”와 “상태 전이가 완료됐다”를 분리해야 한다.
- ROS/EtherCAT과 USB-CDC를 독립 경로로 진단해야 증상을 정확히 설명할 수 있다.
- SD 파일 존재뿐 아니라 Action 서버에 카탈로그 경로가 연결됐는지 확인해야 한다.
- 물리 버튼은 소프트웨어가 캐시한 세션 상태를 무효화할 수 있다.
- USB 장치가 `lsusb`에서 사라진 장애는 GUI 재시도만으로 복구할 수 없다.
- HOME 성공 후에만 UNARM하는 fail-closed 설계가 임의 자세 토크 해제를 방지한다.
