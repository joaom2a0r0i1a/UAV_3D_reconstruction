#!/bin/bash
#
# study4_run.sh — Study 4: AEP benchmark B/C/D timing.
#
# Drives N short benchmark passes through the host supervisor (container-drop
# resilient) with the benchmark spec injected, isolating + collecting the
# ~/.ros/benchmark_{timing,gains}.csv after each pass, then aggregates and runs
# process_benchmark.py to make the B/C/D plots.
#
#   AEP_BENCHMARK=true (via spec) makes AEPlanner.cpp benchmarkGains() time all
#   gain methods on the same tree per planning iteration and write:
#     ~/.ros/benchmark_timing.csv  (per batch)
#     ~/.ros/benchmark_gains.csv   (per node)
#
# Usage: ./study4_run.sh [SIM_TIME_SEC] [PASSES]
#   ./study4_run.sh 900 3

set -u
CTR=noetic_ws
SIM_TIME="${1:-900}"
PASSES="${2:-3}"

ONE_HOST=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone
DATA_HOST=/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/data
WS_CTR=/home/ros1/ros1_motion_ws
ONE_CTR=$WS_CTR/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone
DATA_CTR=$WS_CTR/src/UAV_3D_reconstruction/motion_planning/data
EVAL_CTR=$WS_CTR/src/UAV_3D_reconstruction/motion_planning/src/evaluate

LABEL=BENCH_gpu_marg
SPEC="BENCH_gpu_marg:false:true:gpu:false:true"   # rrt=false marg=true gpu split=false benchmark=true
BENCHOUT_HOST="$DATA_HOST/BENCH"
BENCHOUT_CTR="$DATA_CTR/BENCH"

mkdir -p "$BENCHOUT_HOST"
echo "############################################################"
echo "# STUDY 4 — AEP benchmark  ($PASSES passes x ${SIM_TIME}s)"
echo "############################################################"

for n in $(seq 1 "$PASSES"); do
  echo ""
  echo "=================== benchmark pass $n/$PASSES ==================="
  # Isolate this pass: wipe any prior benchmark CSVs in the node cwd.
  docker exec "$CTR" bash -lc 'rm -f ~/.ros/benchmark_timing.csv ~/.ros/benchmark_gains.csv' 2>/dev/null

  # One cumulative good run for this label (target = n so exactly one more runs).
  "$ONE_HOST/supervise_runs.sh" "$LABEL" "$n" "$SIM_TIME" "$SPEC"
  rc=$?
  [ $rc -ne 0 ] && echo "!!! pass $n supervisor rc=$rc — continuing to collect whatever exists."

  # Collect this pass's CSVs.
  RUNDIR_HOST="$BENCHOUT_HOST/run_$n"
  mkdir -p "$RUNDIR_HOST"
  docker exec "$CTR" bash -lc "mkdir -p $BENCHOUT_CTR/run_$n && cp -f ~/.ros/benchmark_timing.csv ~/.ros/benchmark_gains.csv $BENCHOUT_CTR/run_$n/ 2>/dev/null; ls -la $BENCHOUT_CTR/run_$n/"
  echo "--- pass $n CSV line counts ---"
  for f in benchmark_timing.csv benchmark_gains.csv; do
    if [ -f "$RUNDIR_HOST/$f" ]; then echo "  $f: $(wc -l < "$RUNDIR_HOST/$f") lines"; else echo "  $f: MISSING"; fi
  done
done

# --------------------------------------------------- aggregate + plot --------
echo ""
echo "=================== aggregate + plot ==================="
for f in benchmark_timing.csv benchmark_gains.csv; do
  out="$BENCHOUT_HOST/$f"; : > "$out"; hdr_done=0
  for n in $(seq 1 "$PASSES"); do
    src="$BENCHOUT_HOST/run_$n/$f"
    [ -f "$src" ] || continue
    if [ "$hdr_done" -eq 0 ]; then cat "$src" >> "$out"; hdr_done=1
    else tail -n +2 "$src" >> "$out"; fi
  done
  echo "  aggregated $f: $(wc -l < "$out" 2>/dev/null || echo 0) lines"
done

docker exec -w "$BENCHOUT_CTR" "$CTR" bash -lc \
  "source /opt/ros/noetic/setup.bash && MPLBACKEND=Agg python3 $EVAL_CTR/process_benchmark.py benchmark_timing.csv benchmark_gains.csv"
echo ""
echo "PNGs:"; ls -la "$BENCHOUT_HOST"/*.png 2>/dev/null
echo "############ STUDY 4 DONE ############"
