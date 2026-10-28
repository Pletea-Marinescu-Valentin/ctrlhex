#!/bin/bash
# Usage: run_sih.sh <tag> "<flight script> [args]" [param files...]
# Starts PX4 SIH (hexarotor X), applies the parameter files in order (lines "NAME VALUE"), flies the script
# and writes <tag>.ulg, <tag>.txt and, for scripts that mark phases, <tag>.marks.json to $OUT.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
OUT=$(mkdir -p "${OUT:-$HERE/../results/sih}" && cd "${OUT:-$HERE/../results/sih}" && pwd)
TAG=$1
read -r SCRIPT ARGS <<< "$2"
shift 2
FILES=()
for f in "$@"; do FILES+=("$(readlink -f "$f")"); done

cd "$PX4_DIR/build/px4_sitl_sih/rootfs"
B=../bin
pkill -x px4 2>/dev/null || true
sleep 1
rm -rf fs/parameters*.bson fs/dataman fs/log

PX4_SIM_MODEL=sihsim_hex $B/px4 -d ../etc > /tmp/ctrlhex_px4_setup.log 2>&1 &
sleep 12
for n in COM_RC_IN_MODE:1 NAV_DLL_ACT:0 SYS_FAILURE_EN:1 CA_FAILURE_MODE:2 COM_ACT_FAIL_ACT:0 SDLOG_PROFILE:19; do
  $B/px4-param set "${n%%:*}" "${n##*:}" > /dev/null
done
for f in "${FILES[@]}"; do
  while read -r name value; do
    { [ -z "$name" ] || [ "${name:0:1}" = "#" ]; } && continue
    $B/px4-param set "$name" "$value" > /dev/null
  done < "$f"
done
$B/px4-param save
sleep 1
$B/px4-shutdown || true
sleep 3
pkill -x px4 2>/dev/null || true
sleep 1

PX4_SIM_MODEL=sihsim_hex $B/px4 -d ../etc > "/tmp/ctrlhex_px4_$TAG.log" 2>&1 &
sleep 18
CTRLHEX_MARKS="$OUT/$TAG.marks.json" python3 "$HERE/flights/$SCRIPT" $ARGS > "$OUT/$TAG.txt" 2>&1
sleep 2
$B/px4-shutdown || true
sleep 2
pkill -x px4 2>/dev/null || true
cp "$(ls -t fs/log/*/*.ulg | head -1)" "$OUT/$TAG.ulg"
echo "done $TAG"
