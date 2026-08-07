#!/usr/bin/env bash
set -euo pipefail

MODEL_ROOT="${PHORCE_VOICE_MODEL_ROOT:-${HOME}/.local/share/phorce-voice}"
MODEL_NAME="vosk-model-small-ko-0.22"
MODEL_DIR="${MODEL_ROOT}/${MODEL_NAME}"
MODEL_URL="https://alphacephei.com/vosk/models/${MODEL_NAME}.zip"
ARCHIVE="/tmp/${MODEL_NAME}.zip"

python3 -m pip install --user vosk
if [[ ! -d "$MODEL_DIR" ]]; then
  mkdir -p "$MODEL_ROOT"
  wget -O "$ARCHIVE" "$MODEL_URL"
  unzip -q "$ARCHIVE" -d "$MODEL_ROOT"
fi

printf '[PASS] Vosk 한국어 모델 준비: %s\n' "$MODEL_DIR"
printf '[INFO] 기본 마이크: plughw:CARD=WEBCAM,DEV=0 (C270 직접 입력)\n'
printf '[INFO] 필요하면 PHORCE_VOICE_DEVICE와 PHORCE_VOICE_MODEL로 덮어쓰세요.\n'
