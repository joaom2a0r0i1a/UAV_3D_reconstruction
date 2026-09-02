#!/bin/bash
# Jetson to PC offload, deleting only after checksums match. Flags --check and --dry-run.
set -u

# ---- config ----
PC_HOST="${PC_HOST:-lt-l4@192.168.50.2}"                      # PC over the Alfa link (see README)
PC_ROOT="${PC_ROOT:-/home/lt-l4/real_experiments}"        # results live outside the repo
PC_BAGS_ROOT="${PC_BAGS_ROOT:-$PC_ROOT/session_bags}"
JETSON_ROOT="${JETSON_ROOT:-$HOME/real_experiments}"
BAGS_ROOT="${BAGS_ROOT:-$HOME/bag_files}"
MIN_FREE_GB="${MIN_FREE_GB:-15}"
MANIFEST="$JETSON_ROOT/offload_manifest.log"
# ----------------

free_gb() { df -BG --output=avail "$HOME" | tail -1 | tr -dc '0-9'; }

if [ "${1:-}" = "--check" ]; then
  G=$(free_gb)
  echo "[offload] free space on \$HOME: ${G} GB (threshold ${MIN_FREE_GB} GB)"
  if [ "$G" -lt "$MIN_FREE_GB" ]; then
    echo "[offload] *** LOW DISK — offload before flying! ***"; exit 1
  fi
  echo "[offload] OK to fly."
  exit 0
fi

DRY=""
[ "${1:-}" = "--dry-run" ] && DRY="--dry-run" && echo "[offload] DRY RUN — nothing moves"

command -v rsync >/dev/null || { echo "rsync missing"; exit 1; }
ssh -o ConnectTimeout=5 "$PC_HOST" true || { echo "[offload] PC $PC_HOST unreachable"; exit 1; }

offload_dir() {  # $1 = source dir, $2 = destination dir (on PC)
  local SRC="$1" DST="$2"
  ssh -n "$PC_HOST" "mkdir -p '$DST'"
  echo "[offload] copy  $SRC -> $PC_HOST:$DST"
  rsync -a --partial --info=progress2 $DRY "$SRC/" "$PC_HOST:$DST/" || return 1
  [ -n "$DRY" ] && return 0
  echo "[offload] verify (full checksum) $SRC"
  local DIFF
  DIFF=$(rsync -aic --dry-run "$SRC/" "$PC_HOST:$DST/" | grep -v '^\.d' | head -5)
  if [ -n "$DIFF" ]; then
    echo "[offload] *** VERIFY FAILED for $SRC — NOT deleting ***"; echo "$DIFF"; return 1
  fi
  local BYTES; BYTES=$(du -sb "$SRC" | cut -f1)
  local LINE="$(date -Is) verified $SRC -> $PC_HOST:$DST bytes=$BYTES"
  echo "$LINE" >> "$MANIFEST"
  ssh -n "$PC_HOST" "echo '$LINE' >> '$PC_ROOT/offload_manifest.log'"
  rm -rf "$SRC"
  echo "[offload] deleted Jetson copy of $SRC"
  return 0
}

FAIL=0

# 1. Finished runs carrying .run_complete. A run that lost power has no sentinel and stays put;
#    touch .crashed in it to force this script to take it.
while IFS= read -r RUN <&3; do
  RUN_DIR="$(dirname "$RUN")"
  REL="${RUN_DIR#"$JETSON_ROOT/"}"
  offload_dir "$RUN_DIR" "$PC_ROOT/$REL" || FAIL=1
done 3< <(find "$JETSON_ROOT" -maxdepth 5 \( -name ".run_complete" -o -name ".crashed" \) 2>/dev/null)

# 2. Per-label tmp_bags pools (eval bags) — only when the label dir has no unfinished run left.
while IFS= read -r TB <&3; do
  LABEL_DIR="$(dirname "$TB")"
  if ls -d "$LABEL_DIR"/2*_* >/dev/null 2>&1; then
    echo "[offload] $TB kept (label still has run dirs without sentinel or pending offload)"
    continue
  fi
  REL="${TB#"$JETSON_ROOT/"}"
  offload_dir "$TB" "$PC_ROOT/$REL" || FAIL=1
done 3< <(find "$JETSON_ROOT" -maxdepth 4 -type d -name tmp_bags 2>/dev/null)

# 3. Session bag dirs from record.sh heavy profiles (~/bag_files/<date>).
if [ -d "$BAGS_ROOT" ]; then
  while IFS= read -r BD <&3; do
    if ls "$BD"/*.bag.active >/dev/null 2>&1; then
      echo "[offload] $BD has an ACTIVE bag (recorder running?) — skipped"; continue
    fi
    REL="bag_files/$(basename "$BD")"
    offload_dir "$BD" "$PC_BAGS_ROOT/$REL" || FAIL=1
  done 3< <(find "$BAGS_ROOT" -mindepth 1 -maxdepth 1 -type d 2>/dev/null)
fi

G=$(free_gb)
echo "[offload] done (failures=$FAIL). Free space now: ${G} GB."
exit "$FAIL"
