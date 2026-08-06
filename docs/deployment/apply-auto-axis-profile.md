# 새 Jetson 환경에 자동 축 프로파일 적용하기

## 목적

기존 구성은 Jetson 런타임과 PCM이 각각 별도의 하드웨어 축 구성을 가지고
있었다. 예를 들어 Jetson에서 `axes:=2`를 지정하면 고정 축 마스크
`0x0002`를 기대한다. PCM이 다른 `axis_oper_mask`를 보고하면 실제 축에
고장이 없어도 Jetson 안전 감시기가 피드백을 신뢰하지 않고 HARD 판정을
낼 수 있다.

현재 구성은 Jetson이 PCM에서 관측한 안정적인 운용 축 마스크를 자동으로
감시 기준으로 채택한다.

```text
PCM이 실제 axis_oper_mask를 보고
        ↓
Jetson이 250개 연속 유효 프레임에서 동일한 nonzero 마스크인지 확인
        ↓
확인된 마스크를 expected axis mask로 채택
        ↓
해당 축의 oper/stale/fault 상태를 계속 감시
```

이 변경은 PCM의 하드웨어 구성을 수정하지 않는다. Jetson의 피드백 감시
기준을 PCM이 보고하는 실제 구성에 맞추는 변경이다.

## 문서만 복사해서는 적용되지 않음

새 환경에 진단 문서만 복사해도 런타임 동작은 바뀌지 않는다. 다음 두 가지를
모두 적용해야 한다.

1. `axes:=auto`를 구현한 최신 `phorce_monitor` 런타임 또는 SDK 패키지
2. `mode:=op_idle`, `axes:=auto`를 사용하는 실행 설정

구버전 `phorce_monitor`가 숫자 축 마스크만 지원한다면 업데이트된 패키지를
설치하거나 해당 구현을 소스 코드에 반영해야 한다.

## 실행 설정

모션 슬롯 재생용 환경에서는 다음과 같은 기존 고정 구성을 사용하지 않는다.

```bash
-p mode:=command -p axes:=2
```

대신 아래와 같이 두 터미널을 실행한다. 로봇 전원과 `eno1` 케이블을 먼저
확인하고, 두 프로세스는 사용하는 동안 계속 실행해 둔다.

### 터미널 1: EtherCAT 피드백 및 메일박스 안전 게이트

```bash
export ROS_DOMAIN_ID=21

ros2 run agx_phorce_bridge phorce_monitor --ros-args \
  -p nic:=eno1 \
  -p mode:=op_idle \
  -p axes:=auto \
  -p mbx_enabled:=true
```

### 터미널 2: 모션 슬롯 액션 서버

```bash
export ROS_DOMAIN_ID=21

ros2 run agx_motion_slot motion_action_server --ros-args \
  -p backend:=ecat
```

모든 ROS 2 터미널과 관련 서비스에서 같은 `ROS_DOMAIN_ID=21`을 사용해야 한다.
로봇 전원을 껐다 켰다면 터미널 1도 종료한 후 다시 실행한다.

## 적용 확인

별도 터미널에서 다음을 실행한다.

```bash
export ROS_DOMAIN_ID=21

phorce doctor
ros2 param get /phorce_monitor axes
ros2 param get /phorce_monitor mode
ros2 topic echo /phorce/status --once
```

다음 조건을 확인한다.

```text
phorce doctor 판정: READY
axes: auto
mode: op_idle
axis_oper_mask: 실제 연결된 축 구성에 해당하는 nonzero 마스크
axis_stale_mask: 0
axis_fault_mask: 0
mbx_veto_active: false
ethercat_operational: true
verdict_pass: 계속 증가
```

6축 실험에서는 `axis_oper_mask=455` (`0x01C7`, 축 0·1·2·6·7·8)가
관측됐고, 모션 슬롯 4 실행 시 6축 모두 정상 동작했다.

```bash
phorce list
phorce play 4 --target robot
```

모션을 실행하기 전에 목록에서 올바른 ID를 확인하고, 작업 공간을 비우며,
물리 E-Stop을 준비한다.

## 자동 축 인식이 해결하지 않는 것

`axes:=auto`는 하드웨어 인식과 안전 감시 기준만 맞춘다. 다음 항목을 자동으로
수정하지는 않는다.

- PCM 펌웨어의 축 구성
- PCM에 적재된 모션 슬롯 데이터
- 슬롯과 실제 하드웨어 사이의 축 호환성
- 잘못 선택한 모션 ID

따라서 PCM 펌웨어와 적재된 슬롯이 새 하드웨어 구성을 지원해야 한다.
자동 인식은 호환되지 않는 모션을 변환하거나 호환 가능하게 만들지 않는다.

## 배포 체크리스트

- [ ] 로봇 전원과 `eno1` 링크가 정상이다.
- [ ] 설치된 `phorce_monitor`가 `axes:=auto`를 지원한다.
- [ ] 모든 프로세스에 `ROS_DOMAIN_ID=21`이 적용됐다.
- [ ] 모니터가 `mode:=op_idle`, `axes:=auto`로 실행된다.
- [ ] 액션 서버가 `backend:=ecat`으로 실행된다.
- [ ] `phorce doctor`가 `READY`를 보고한다.
- [ ] 실제 `axis_oper_mask`가 장착된 축 구성과 일치한다.
- [ ] stale/fault/veto 값이 모두 정상이다.
- [ ] 올바른 모션 슬롯 ID를 확인했다.
- [ ] 물리 E-Stop과 작업 공간 안전을 확인했다.

## 저장소 반영 상태 주의

자동 축 프로파일의 원인, 실험 및 결과는 진단 문서에 기록돼 있다.

```text
docs/diagnostics/2026-08-06-motion-slot-hard-veto.md
```

이 문서를 작성한 시점에는 진단 문서가 GitHub `main`에 반영돼 있지만,
`main/check_robot_communication.sh`, `main/README.md`, 참가자 가이드의 관련
실행 설정 변경은 로컬 작업트리에 미커밋 변경으로 남아 있었다. 새 환경에
저장소만 내려받아 적용하려면 해당 변경을 별도로 검토·커밋·푸시한 뒤 배포해야
한다.
