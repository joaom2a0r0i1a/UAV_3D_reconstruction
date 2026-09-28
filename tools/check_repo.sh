#!/bin/bash
# Dump every launch parse, parameters and remaps, run as check_repo.sh <output_dir> in noetic_ws

OUT="${1:?usage: check_repo.sh <output_dir>}"
mkdir -p "$OUT/params" "$OUT/parse" "$OUT/remaps"

source /opt/ros/noetic/setup.bash
source /home/ros1/ros1_motion_ws/devel/setup.bash

# Environment the launches expect
export UAV_NAME=uav1 UAV_TYPE=f450 RUN_TYPE=simulation WORLD_NAME=simulation UAV_ID=1

REPO=/home/ros1/ros1_motion_ws/src/UAV_3D_reconstruction
cd "$REPO"

ok=0
bad=0
: >"$OUT/summary.txt"

while read -r lf; do
  name=$(echo "${lf#./}" | tr '/' '_' | sed 's/\.launch$//')
  if roslaunch --files "$lf" >"$OUT/parse/$name.txt" 2>"$OUT/parse/$name.err"; then
    parse=ok
  else
    parse=FAIL
  fi
  if roslaunch --dump-params "$lf" >"$OUT/params/$name.yaml" 2>"$OUT/params/$name.err"; then
    dump=ok
  else
    dump=FAIL
  fi
  [ -s "$OUT/params/$name.err" ] || rm -f "$OUT/params/$name.err"
  [ -s "$OUT/parse/$name.err" ] || rm -f "$OUT/parse/$name.err"
  python3 - "$lf" >"$OUT/remaps/$name.txt" 2>/dev/null <<'PYREMAP'
import sys, roslaunch.config, roslaunch.xmlloader
loader = roslaunch.xmlloader.XmlLoader()
cfg = roslaunch.config.ROSLaunchConfig()
try:
    loader.load(sys.argv[1], cfg, argv=[], verbose=False)
except Exception as e:
    print('load failed', e); raise SystemExit(0)
for n in cfg.nodes:
    print(f'{n.package}/{n.type} as {n.namespace}{n.name}')
    for a, b in sorted(tuple(r) for r in n.remap_args):
        print(f'  {a} -> {b}')
PYREMAP
  printf "%-64s parse=%-4s params=%s\n" "${lf#./}" "$parse" "$dump" >>"$OUT/summary.txt"
  if [ "$parse" = ok ] && [ "$dump" = ok ]; then ok=$((ok + 1)); else bad=$((bad + 1)); fi
done < <(find . -name "*.launch" -not -path "*/.*/*" | sort)

echo >>"$OUT/summary.txt"
echo "launches ok $ok, launches with a failure $bad" >>"$OUT/summary.txt"
cat "$OUT/summary.txt"
