#!/usr/bin/env bash
# venv 생성. 시스템 패키지와 ~/.local 은 건드리지 않는다.
#
# 이 젯슨에는 python3.10-venv 가 없어 `python3 -m venv` 가 ensurepip 없음으로 실패한다.
# apt 를 쓰지 않고 virtualenv 를 작업공간 안에만 받아서 우회한다.
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"

if [ -x venv/bin/python ]; then
    echo "venv 가 이미 있습니다: $WS/venv"
    exec venv/bin/python -c "import cv2, numpy; print('cv2', cv2.__version__, '/ numpy', numpy.__version__)"
fi

python3 -m pip install --target="$WS/.bootstrap" virtualenv
PYTHONPATH="$WS/.bootstrap" python3 -m virtualenv \
    --system-site-packages -p python3.10 "$WS/venv"

# venv 는 기본적으로 ~/.local 을 무시한다. 이 .pth 가 없으면 cv2 를 못 찾는다.
# 재설치하지 않고 경로만 이어 붙여 기존 cv2 4.11.0 / numpy 1.26.4 를 그대로 쓴다.
python3 -m site --user-site > "$WS/venv/lib/python3.10/site-packages/_user_local.pth"

venv/bin/python -c "import cv2, numpy; print('cv2', cv2.__version__, '/ numpy', numpy.__version__)"
echo "완료: $WS/venv"
