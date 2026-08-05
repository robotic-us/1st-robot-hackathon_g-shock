# phorce 통신 확인 시퀀스

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
