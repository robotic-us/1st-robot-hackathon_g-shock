# g-shock-dev 작업 지침

## 시스템 패키지 — 절대 금지

이 젯슨(AGX Orin, JetPack 6.2.1)에서 **`sudo apt update`, `apt upgrade`,
`pip install --upgrade`, 시스템/`~/.local` 패키지 재설치를 하지 않는다.**
JetPack 은 CUDA/TensorRT/cuDNN 이 커널·드라이버와 묶여 있어 패키지 하나만 올려도
전체 스택이 깨진다.

버전 문제가 생기면 **작업공간 안에 가상환경을 만들어 그 안에서만** 해결한다.

```bash
# python3.10-venv 가 없으므로 virtualenv 를 작업공간에 받아서 쓴다
pip install --target=$PWD/.bootstrap virtualenv
PYTHONPATH=$PWD/.bootstrap python3 -m virtualenv --system-site-packages .venv
source .venv/bin/activate
```

기존 `~/.local` 패키지(cv2, numpy)는 재설치하지 말고 venv 의 site-packages 에
`_user_local.pth` 로 경로만 이어 붙여 재사용한다.

새 의존성이 필요하면 설치하기 전에 먼저 사용자에게 알린다.

## 실물 로봇 안전

- **pcm 은 EtherCAT 정지를 지원하지 않는다.** 액션 cancel 은 예외 없이 거부되고,
  발사한 모션은 끝까지 간다. `shock_guard_gui` 의 DOB 충격 감지 자동정지는
  실물에서 동작하지 않는다.
- 유일한 정지 수단은 물리 E-Stop 이며, 눌리면 래치되어 재부팅·재호밍이 필요하다.
- 새 기능은 반드시 `backend:=sim`(`dev/bin/sim-up.sh`)에서 먼저 검증한다.

## ROS domain

- **domain 21 을 쓴다. domain 0 금지.** 이 장비는 공유 WiFi 에 붙어 있고
  `ROS_LOCALHOST_ONLY=0` 이라 domain 0 으로 띄우면 다른 팀 로봇과 그래프가 합쳐진다.
- 실기 스택은 `dev/bin/robot-stack.sh` 로 띄운다.
- 상태 점검은 `dev/bin/doctor.sh`.

## 건드리지 않는 것

- `~/Downloads` (원본 저장소) — 읽기만 한다.
- 이 워크스페이스의 `main/` — 원본에서 따라온 기준 코드. 새 기능은 `dev/features/` 에.
