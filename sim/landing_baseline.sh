#!/bin/bash
# Landing with a stopped motor on unpatched PX4 (same commit), as the baseline of the touchdown rule.
# Copies $PX4_DIR (patched) to $BASE_DIR without build/, reverses firmware/ctrlhex.patch there, builds px4_sitl_sih
# and flies landing.py on the three plant models. MPC_HAGL_MAX exists only in the patch, so it is left out.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
BASE_DIR=${BASE_DIR:-$HOME/PX4-baseline}
export OUT=${OUT:-$HERE/../results/sih}
if [ ! -d "$BASE_DIR" ]; then
  rsync -a --exclude /build/ "$PX4_DIR/" "$BASE_DIR/"
  (cd "$BASE_DIR" && git apply -R "$HERE/../firmware/ctrlhex.patch")
fi
(cd "$BASE_DIR" && make px4_sitl_sih)
P=$(mktemp -d)
grep -v "^MPC_HAGL_MAX" "$HERE/params/safety.txt" > "$P/safety_upstream.txt"
cd "$HERE"
for m in nominal fast slow; do
  PX4_DIR=$BASE_DIR bash run_sih.sh baseline_landing_$m "landing.py keep" params/model_$m.txt params/gains_selected.txt "$P/safety_upstream.txt"
done
cd analysis && python3 landing.py "$OUT"/baseline_landing_*.ulg > "$OUT/summary_landing_baseline.txt"
echo "summary in $OUT/summary_landing_baseline.txt"
