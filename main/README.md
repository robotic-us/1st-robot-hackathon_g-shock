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
프레임에서 검출될 때 현재 Action goal에 cancel을 요청합니다. 실제 기체의 무부하
DOB 노이즈를 먼저 관찰한 뒤 임계값을 조정하세요.
GUI를 닫으면 이 실행기가 연 Terminator와 통신 스택도 함께 종료됩니다.
GUI의 작은 **전체 종료** 버튼은 확인창 없이 즉시 활성 모션에 취소를 요청한 뒤 모든
창과 프로세스를 닫습니다.
축 표에는 현재 조인트 각도(`position_rad`)도 rad 단위로 표시됩니다.
GUI 상태 영역에는 PCM의 **모션 수신 준비 상태**와 현재 `|dob_a|`가 가장 큰 **감지
모터 ID(0 기반 축 index)**가 표시됩니다. 물리 1번 버튼 접점은 ROS에 공개되지 않으므로
눌림 자체가 아니라 아래의 실제 서보·홈 상태로 절차 완료를 추정합니다. `physical_idle`은
버튼 신호가 아니라 PCM의 모션 수신 가능 상태로만 별도 표시합니다.

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

모션은 PCM 카탈로그를 자동 조회해 이름이 정확히 `GRAB`, `BOX`/`BOX_TEST`, `TAPE`인 슬롯만
각 버튼에 연결합니다. ID를 추측해서 전송하지 않으며, 해당 슬롯이 없으면 버튼이
비활성화됩니다. 범용 원점 복귀 API는 없으므로 `HOME`, `RETURN_HOME`, `GO_HOME` 중
정확히 일치하는 슬롯이 하나 있을 때만 **원점 복귀** 버튼이 활성화됩니다. 이 버튼은
PCM 원점을 다시 기록하는 기능이 아니라 PCM에 적재된 홈 자세 모션을 재생합니다.
카탈로그 이름이 매핑되지 않아도 **모션 ID 직접 실행** 입력칸에서 `1~50`을 지정할 수
있습니다. 기본값은 슬롯 `1`이며 **ID 실행** 버튼은 동일한 충격 감시를 활성화해 재생합니다.

> **안전:** 이 정지는 ROS Action cancel 요청이며 물리 E-Stop이 아닙니다. cancel은
> 실물에서 거절되거나 즉시 정지하지 않을 수 있습니다. 위험 상황에서는 반드시 로봇의
> 물리 E-Stop을 사용하세요.

로봇을 움직이지 않고 다음 항목만 순서대로 확인합니다.

1. `cat /sys/class/net/eno1/operstate` 결과가 `up`인지 확인
2. `phorce_monitor` 실행 파일 확인 및 기동
3. `phorce_monitor` 로그에서 자가검사 `PASS` 13개 확인
4. `/phorce/feedback` 토픽과 실제 메시지 1건 수신 확인
5. `motion_action_server` 기동
6. `/motion_action_server/play_motion_sequence` 액션 서버 노출 확인

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
