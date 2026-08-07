# phorce 통신 확인 시퀀스

## 통합 GUI 실행

화면 왼쪽에는 모션 실행·충격 감시 GUI, 오른쪽에는 `three`라는 이름의 Terminator
3분할 레이아웃(통신 스택 / 실시간 DOB·전류 / 상태·오류)을 엽니다.

```bash
cd ~/Downloads/main
chmod +x run_integrated_system.sh shock_guard_gui.py torque_console.py status_console.sh
./run_integrated_system.sh
```

통합 실행기는 `ROS_DOMAIN_ID`가 설정되지 않았으면 실기 검증값인 `21`을 사용하며,
GUI와 Terminator의 모든 자식 프로세스에 같은 값을 전달합니다. 다른 domain이 필요한
경우에만 `ROS_DOMAIN_ID=숫자 ./run_integrated_system.sh`로 덮어쓰세요.

GUI의 충격 판단값은 모터 전류에 비례하는 `current_a`보다 외부에서 가해진 힘을
추정한 `dob_a`의 절댓값을 사용합니다. 기본값은 `2.0 A` 이상이 3개 연속 유효
프레임에서 검출될 때 화면 상단의 DOB 상태를 크게 빨간색으로 표시합니다. DOB는
관측·경고 전용이며 Action cancel, 모션 변경, 자동 OFF 명령을 보내지 않습니다.
실제 기체의 무부하 DOB 노이즈를 먼저 관찰한 뒤 임계값을 조정하세요.
GUI를 닫으면 이 실행기가 연 Terminator와 통신 스택도 함께 종료됩니다.
GUI의 작은 **전체 종료** 버튼은 확인창 없이 즉시 활성 모션에 취소를 요청한 뒤 모든
창과 프로세스를 닫습니다.
축 표에는 현재 조인트 각도(`position_rad`)도 rad 단위로 표시됩니다.
GUI 상태 영역에는 PCM의 **모션 수신 준비 상태**와 현재 `|dob_a|`가 가장 큰 **감지
모터 ID(0 기반 축 index)**가 표시됩니다. 물리 1번 버튼 접점은 ROS에 공개되지 않으므로
눌림 자체가 아니라 아래의 실제 서보·홈 상태로 절차 완료를 추정합니다. `physical_idle`은
버튼 신호가 아니라 PCM의 모션 수신 가능 상태로만 별도 표시합니다.

## 실험 기능: 오프라인 한국어 음성 명령

GUI에서 키보드 **스페이스를 누르고 있는 동안** Logitech C270 내장 마이크로 녹음하고,
스페이스를 떼면 Vosk 한국어 소형 모델로 오프라인 인식합니다. 화면의 음성 버튼도
마우스로 누르고 있는 동안 동일하게 동작합니다. 현재 등록 명령은 `준비`,
`물건 넣어줘`, `포장해줘`, `종료`이며 `시작`, `제품 넣어 줘`, `포장 시작`, `마무리`
같은 표현도 각 명령으로 정규화합니다. GUI에는 실제 인식 문장과 정규화된 명령을 함께
표시합니다. 인식 원문과 후처리 명령은 별도 줄로 표시됩니다. 인식만으로는 동작하지
않고 별도의 **인식 결과 실행** 버튼이나 키보드 **Enter**를 눌러야 합니다.
실행 매핑은 `준비→ARM`, `물건 넣어줘→GRAB(4)`, `포장해줘→BOX(3)`,
`종료→HOME→UNARM`입니다. ARM과 종료는 기존 안전 확인창을 한 번 더 거칩니다.

최초 한 번 다음 설치를 실행합니다.

```bash
./setup_voice_recognition.sh
```

기본 마이크는 `plughw:CARD=WEBCAM,DEV=0`, 기본 모델은
`~/.local/share/phorce-voice/vosk-model-small-ko-0.22`입니다. 다른 장치는
`PHORCE_VOICE_DEVICE`, 다른 모델은 `PHORCE_VOICE_MODEL`로 지정할 수 있습니다.

GUI의 **ARM**은 PCM USB-CDC 명령 `0x5F08:01 = 1`을 사용합니다. 이 명령은 PCM
펌웨어에서 물리 1번 버튼 롱프레스와 같은 서보 ON 경로를 실행합니다.
실행 전 확인창이 열리며, 확인 후 로봇이 설정된 부팅 자세로 약 3~5초간 움직일 수
있습니다. ARM 요청 시 GUI는 phorce Studio와 같은 절차로 PCM SD 볼륨을 안전 해제하고
`0x5F0C`의 `LIVE_BEGIN` 세션을 확립한 뒤 서보 명령을 전송합니다. GUI 종료 후 PCM의
세션 watchdog이 SD를 PC에 다시 노출하며 데스크톱 자동 마운트가 복구합니다.
GUI는 Write ACK만으로 성공 처리하지 않고 `0x5F08:02`를 폴링해 상태 `1`
(서보 ON)을 직접 확인한 뒤에만 OP로 표시합니다. 확인 직후에는 CDC/DTR을 닫아
Studio LIVE 소유권을 반환합니다. 이 해제가 완료돼야 EtherCAT 모션 창구의
`physical_idle`이 true가 되어 슬롯 모션을 받을 수 있습니다. 다음 ARM 또는 OFF
요청이 들어올 때만 CDC를 다시 엽니다. 새 연결이나 재전원 뒤 USB 상태가
아직 확인되지 않은 동안에는 기존 gain/자세가 활성으로 보여도 안전 방향인 NOT OP를
유지합니다. **HOME → UNARM**은 PCM 모션 슬롯
`1`을 실행하고 Action 결과가 성공(`status=0`)한 경우에만 `0x5F08:01=0`을 보내
서보를 끕니다. ARM 후에는 모션 소유권 반환을 위해 LIVE를 해제하므로, UNARM은
CDC를 새로 열고 HELLO로 SDO 채널만 준비한 뒤 OFF를 전송합니다. 서보 ON 상태에서는
PCM의 `SAFE_PARKING` 비트가 없어 LIVE_BEGIN이 거부될 수 있으므로 UNARM은 Studio
LIVE 소유권을 요구하지 않습니다. OFF 확인 뒤에도 CDC 포트를 닫아 PCM의 스토리지/파킹 전환 중 stale
fd를 다음 ARM이 재사용하지 않도록 합니다. 이때 DTR을 명시적으로 내리고 HUPCL을
적용해 PCM이 호스트 세션 종료를 확실히 관측하게 합니다. 모션 1이 거절·중단·취소되거나 결과 확인에 실패하면 임의 자세에서
토크가 빠지지 않도록 서보를 유지합니다. 이 조합은 물리 2번 버튼의 펌웨어 내부
복구 상태 해제까지 대체하지는 않습니다. 홈 복귀 없이 즉시 토크를 제거해야 할 때는
별도 **즉시 OFF** 버튼을 사용합니다. 포트는 Angel Robotics USB 장치
`0483:5741`에 연결된 `/dev/ttyACM*`를 자동 선택합니다. 자동 선택이 어려운 경우
`PHORCE_PCM_PORT=/dev/ttyACM0 ./run_integrated_system.sh`처럼 지정할 수 있습니다.
포트 권한 오류가 나오면 실행 사용자가 `dialout` 그룹에 속하는지 확인하세요.
현재 Jetson의 `phorce` 계정은 필요하면 한 번만
`sudo usermod -aG dialout phorce`를 실행하고 로그아웃/로그인해야 합니다.

현재 기체에서는 `status_flags`가 버튼 전후 `72(0x48)`로 유지되므로 버튼 판정에 쓰지
않습니다. 홈 영점 기준은 축 0~2 `[-1.773, 1.034, 2.840] rad`, 축 6~8
`[2.562, 1.746, 1.550] rad`이고, 1번 준비 자세 기준은 아래 실측값입니다. 홈 영점
근처에서 gain이 비활성이면 **1번 버튼 필요 추정**으로 표시합니다. 유효 6축의
`kp_echo`/`kd_echo`가 모두 활성이고 준비 자세 오차가 `0.12 rad` 이내인 상태가 200개
피드백 프레임(약 0.2초) 연속 확인되면 **1번 버튼 완료 추정**으로 표시합니다. 두 자세는
토픽상 최대 `0.006 rad` 차이라 각도만으로 구분하지 않고 서보 gain을 함께 사용합니다.
GUI 우측 상단에는 상태 배지를 표시합니다. `NOT OP`는 준비 전(빨강), `OP`는 6축 gain과
준비 자세 확인(초록), `BACK TO HOME`은 OP 진입 이력 후 gain이 해제되어 2번 버튼의 홈
영점으로 복귀한 상태(주황)입니다. 상태 머신은 `NOT OP → 1번 → OP → 모션 수행 중 OP
유지 → 2번 → BACK TO HOME` 순서이며, 버튼 접점이 아니라 피드백 전이 기반 추정입니다.

각도 표의 0점은 시작 시 정렬하는 홈 영점 자세입니다. 축 0~2는
`[-1.773, 1.034, 2.840] rad`, 축 6~8은 `[2.562, 1.746, 1.550] rad`입니다. 시작
상태에서는 이 값이 0 근처여도 항상 `NOT OP`이며, GUI 시작 이후 gain 활성과 `0.03 rad`
이상 홈 이탈 또는 `0.05 rad/s` 이상 움직임을 실제 관측해야 1번 동작을 시작한 것으로
인정합니다. 이후 속도 `0.03 rad/s` 이하가 약 0.2초 유지돼야 `OP`로 전환합니다.

GUI가 허용하는 모션은 고정 슬롯 `1(HOME)`, `3(BOX)`, `4(GRAB)`뿐입니다. SD에 저장된
`MS Name`이 달라도 ID를 기준으로 버튼에 연결하며, 해당 슬롯이 없으면 버튼이
비활성화됩니다. **원점 복귀**는 PCM 원점을 다시 기록하는 기능이 아니라 슬롯 1의 홈
자세 모션을 재생합니다. **모션 ID 직접 실행**도 1, 3, 4만 허용하며 그 밖의 슬롯은
오조작 방지를 위해 GUI에서 거부합니다.

> **안전:** 유효 축의 `|DOB|` 최댓값이 `3.0 A`를 초과하면 GUI가 활성 모션 취소와
> USB 서보 즉시 OFF를 요청합니다. 값이 `2.5 A` 아래로 내려오기 전에는 같은 초과
> 상태에서 명령을 반복 전송하지 않습니다. 이 기능은 물리 E-Stop을 대체하지 않으므로
> 위험 상황에서는 반드시 로봇의 물리 E-Stop을 사용하세요.

로봇을 움직이지 않고 다음 항목만 순서대로 확인합니다.

1. `cat /sys/class/net/eno1/operstate` 결과가 `up`인지 확인
2. `phorce_monitor` 실행 파일 확인 및 기동
3. `phorce_monitor` 로그에서 자가검사 `PASS` 13개 확인
4. `/phorce/feedback` 토픽과 실제 메시지 1건 수신 확인
5. `motion_action_server` 기동
6. `/motion_action_server/play_motion_sequence` 액션 서버 노출 확인

통합 GUI는 기본적으로 PCM SD카드의
`/media/phorce/9016-4EF8/Motions`를 모션 카탈로그로 사용합니다. 다른 위치에
마운트한 경우 실행 전에 `PHORCE_MOTION_DIR` 환경변수로 경로를 지정합니다.

스크립트는 액션 goal이나 모션 ID를 전송하지 않습니다. 성공 후 두 노드를 계속 유지하며,
`Ctrl+C`를 누르면 두 프로세스를 순서대로 종료합니다.

## 실행 전

- 로봇(pcm/phact) 전원을 먼저 켭니다.
- `eno1` 케이블 연결을 확인합니다.
- 로봇을 껐다 켠 경우 이 스크립트도 종료 후 다시 실행합니다.
- 기본값은 모션 슬롯 전용 안전 구성인 `mode:=op_idle`, `axes:=auto`입니다. 연결된
  PhACT 축 마스크를 250개 연속 유효 프레임에서 확인한 뒤 stream-off 상태로 운용합니다.
  PCM 펌웨어와 모션 슬롯이 해당 축 구성을 지원하는지는 별도 계약입니다.

## 실행

```bash
cd ~/Downloads/main
source /opt/ros/humble/setup.bash
chmod +x check_robot_communication.sh
./check_robot_communication.sh
```

골든 이미지에서 로그인 시 ROS 환경이 자동 설정된다면 `source` 줄은 생략할 수 있습니다.

정상이면 마지막에 다음 문구가 출력됩니다.

```text
[PASS] 통신 확인 완료: 링크, 피드백, 액션 창구가 모두 정상입니다.
[INFO] 모션 goal은 한 건도 전송하지 않았습니다.
```

## 로그

실행 시 아래 파일이 새로 기록됩니다.

- `logs/phorce_monitor.log`: 13개 자가검사와 EtherCAT 통신 로그
- `logs/motion_action_server.log`: 모션 요청 창구 로그

실패하면 각 로그의 마지막 40줄을 자동으로 출력합니다. 전체 자가검사 결과는 다음처럼
확인할 수 있습니다.

```bash
less ~/Downloads/main/logs/phorce_monitor.log
```

환경이 느린 경우 시작 제한 시간을 늘릴 수 있습니다.

```bash
PHORCE_STARTUP_TIMEOUT=40 ./check_robot_communication.sh
```

고정 하드웨어 프로파일을 진단해야 할 때만 명시적으로 덮어씁니다. 예를 들어 node
`0x03` 단축 프로파일은 다음과 같습니다.

```bash
PHORCE_MODE=op_idle PHORCE_AXES=2 ./check_robot_communication.sh
```

`command` 모드는 RT 관절 스트리밍용 운영자 영역입니다. 모션 슬롯 재생만 사용하는
참가자 환경에서는 기본값을 유지하세요.
