#!/usr/bin/env bash
# Explicitly confirmed SD installer. Never overwrites without typed consent.
set -euo pipefail

if (( $# != 2 )); then
  printf '사용법: %s 생성폴더 SD마운트경로\n' "$0" >&2
  exit 2
fi
SOURCE_DIR="$(realpath -- "$1")"
SD_ROOT="$(realpath -- "$2")"
[[ -d "$SOURCE_DIR" && -d "$SD_ROOT/Motions" ]] || { printf '[FAIL] 경로를 확인하세요.\n' >&2; exit 1; }
[[ ! -L "$SD_ROOT/Motions" ]] || { printf '[FAIL] SD의 Motions가 심볼릭 링크입니다. 중단합니다.\n' >&2; exit 1; }
mapfile -t CSV_FILES < <(find "$SOURCE_DIR" -maxdepth 1 -type f -name 'motion_[0-9][0-9].csv' -print)
(( ${#CSV_FILES[@]} == 1 )) || { printf '[FAIL] 생성폴더에는 CSV가 정확히 1개여야 합니다.\n' >&2; exit 1; }
CSV_FILE="${CSV_FILES[0]}"
BASENAME="$(basename -- "$CSV_FILE" .csv)"
[[ "$BASENAME" =~ ^motion_([0-9]{2})$ ]] || exit 1
SLOT="${BASH_REMATCH[1]}"
MEMO_FILE="$SOURCE_DIR/${BASENAME}.memo.json"
[[ -f "$MEMO_FILE" ]] || { printf '[FAIL] memo 파일이 없습니다.\n' >&2; exit 1; }
for target in "$SD_ROOT/Motions/${BASENAME}.csv" "$SD_ROOT/Motions/${BASENAME}.memo.json"; do
  [[ ! -L "$target" ]] || { printf '[FAIL] 대상 파일이 심볼릭 링크입니다: %s\n' "$target" >&2; exit 1; }
done
EXPECTED_SHA="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["motion_sha256"])' "$MEMO_FILE")"
ACTUAL_SHA="$(sha256sum "$CSV_FILE" | cut -d' ' -f1)"
[[ "$EXPECTED_SHA" == "$ACTUAL_SHA" ]] || { printf '[FAIL] CSV와 memo 해시가 다릅니다.\n' >&2; exit 1; }

printf '대상 슬롯: %s\n' "$SLOT"
printf '원본: %s\n' "$SOURCE_DIR"
printf 'SD 대상: %s/Motions/%s.csv\n' "$SD_ROOT" "$BASENAME"
if [[ -e "$SD_ROOT/Motions/${BASENAME}.csv" || -e "$SD_ROOT/Motions/${BASENAME}.memo.json" ]]; then
  printf '\n[경고] 기존 슬롯 %s 파일이 있습니다. 백업 후 덮어씁니다.\n' "$SLOT"
fi
printf '계속하려면 정확히 OVERWRITE SLOT %s 를 입력하세요: ' "$SLOT"
read -r CONFIRMATION
[[ "$CONFIRMATION" == "OVERWRITE SLOT $SLOT" ]] || { printf '취소했습니다. SD카드는 변경하지 않았습니다.\n'; exit 1; }

BACKUP_DIR="$SOURCE_DIR/sd_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
for target in "$SD_ROOT/Motions/${BASENAME}.csv" "$SD_ROOT/Motions/${BASENAME}.memo.json"; do
  if [[ -e "$target" ]]; then cp --preserve=timestamps -- "$target" "$BACKUP_DIR/"; fi
done
cp -- "$CSV_FILE" "$SD_ROOT/Motions/${BASENAME}.csv"
cp -- "$MEMO_FILE" "$SD_ROOT/Motions/${BASENAME}.memo.json"
sync
cmp -- "$CSV_FILE" "$SD_ROOT/Motions/${BASENAME}.csv"
cmp -- "$MEMO_FILE" "$SD_ROOT/Motions/${BASENAME}.memo.json"
printf '[PASS] 슬롯 %s 설치 및 검증 완료. 기존 파일 백업: %s\n' "$SLOT" "$BACKUP_DIR"
