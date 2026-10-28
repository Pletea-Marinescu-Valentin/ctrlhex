#!/bin/bash
# All SIH experiments, then the summaries in $OUT/summary_*.txt
set -e
cd "$(dirname "$0")"
export OUT=${OUT:-$(pwd)/../results/sih}
P=params
run() { bash run_sih.sh "$@"; }

# Failure handling of the PX4 allocator during manoeuvres, mode 1 (opposite rotor off) and mode 2 (reversible)
run maneuvers_mode1 maneuvers.py $P/failure_mode1.txt
run maneuvers_mode2 maneuvers.py

# Rate-controller gains: three gain sets on three plant models
for m in nominal fast slow; do
  for g in board px4 selected; do
    run gains_${g}_$m gains.py $P/model_$m.txt $P/gains_$g.txt
  done
done

# Motor restore: PX4 one-step restore against the gradual restore, with a 0.15 s motor start dead time
BASE="$P/model_nominal.txt $P/gains_selected.txt $P/safety.txt $P/motor_start_delay.txt"
run restore_alt_one_step  "restore.py 2 alt"   $BASE $P/restore_one_step.txt
run restore_alt_gradual   "restore.py 2 alt"   $BASE
run restore_stab_one_step "restore.py 2 stab"  $BASE $P/stabilized_ceiling.txt $P/restore_one_step.txt
run restore_stab_gradual  "restore.py 2 stab"  $BASE $P/stabilized_ceiling.txt
run restore_again         "restore.py 2 again" $BASE

# Landing with a stopped motor in mode 2, three plant models
for m in nominal fast slow; do
  run landing_$m "landing.py keep" $P/model_$m.txt $P/gains_selected.txt $P/safety.txt
done

# Safety layers without GNSS and without a range sensor: Stabilized ceiling, then the forced geofence alone
NR="$P/model_nominal.txt $P/gains_selected.txt $P/safety.txt $P/no_range_sensor.txt"
run ceiling_no_range  "ceiling.py ceiling --armed-wait 4"  $NR
run geofence_no_range "ceiling.py geofence --armed-wait 4" $NR $P/no_ceiling.txt

cd analysis
for t in mode1 mode2; do
  echo "=== maneuvers_$t"; python3 maneuvers.py "$OUT/maneuvers_$t.ulg" "$OUT/maneuvers_$t.marks.json"
done > "$OUT/summary_maneuvers.txt"
python3 gains.py "$OUT" gains_{board,px4,selected}_{nominal,fast,slow} > "$OUT/summary_gains.txt"
python3 restore.py "$OUT"/restore_*.ulg > "$OUT/summary_restore.txt"
python3 landing.py "$OUT"/landing_*.ulg > "$OUT/summary_landing.txt"
(cd "$OUT" && grep -H "max height\|Geofence\|Landing" ceiling_no_range.txt geofence_no_range.txt) > "$OUT/summary_safety.txt"
echo "summaries in $OUT"
