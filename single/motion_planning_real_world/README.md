# Real-World Motion Planning

Real-world (mavros/ArduPilot) versions of the exploration planners. `AEPReal` and `NBVReal`
are ports of the current sim planners (`motion_planning/{AEP,NBV}`) with the same layout and
feature set — GPU marginal gain, batched expansion, recovery/backtrack, edge collision — and
these real-world deltas:

- **mavros instead of MRS**: pose in from `geometry_msgs/PoseStamped`
  (`/mavros/local_position/pose`), commands out as `mavros_msgs/PositionTarget`
  (`/mavros/setpoint_raw/local`). No mrs_lib/mrs_msgs in the AEP/NBV code paths
  (Kino variants are older and still mrs-based).
- **Start offset, automatic**: configs define the bounded box and the gain box RELATIVE TO THE
  TAKEOFF POSE. On `~start` the planner snapshots the current pose, shifts both boxes by it
  (`GainEvaluator::setWorldOffset`), and publishes the offset LATCHED on `offset_out` (the
  cached frontier server consumes it). `~offset` re-captures manually; it is idempotent.
- **No benchmark suites, execution horizon fixed to 1** (NBV flies one step per replan; AEP
  flies its chosen branch as a waypoint chain with distance+yaw advance).
- **No auto-takeoff**: take off yourself (GUIDED), then call `~start`.

## How to fly

1. Pre-flight: `scripts/evaluate/offload_runs.sh --check` (refuses below 15 GB free).
2. `tmux/one_drone_real/mavros_tmux_{aep,nbv}.sh` — brings up mavros (`apm.launch`), realsense,
   TF connect, voxblox, pointcloud processing, the planner, (AEP) the cached frontier server
   via `cache_nodes cache_real.launch`, the experiment recorder
   (`evaluate_map_real.launch` — waits for mission start, does NOT start anything), the
   `record.sh eval` bag, and the `start_gate` pane.
3. Take off manually, fly/look around (voxblox maps from the get-go), switch to GUIDED.
4. The `start_gate` pane detects GUIDED and asks **"Start Planner? [Y/n]"** — `Y` starts the
   mission (offset captured, recorder clock starts on the latched offset). Afterwards the pane
   offers `s`=stop, `o`=re-capture offset, `q`=quit. Fallback: the old
   `rosservice call /uavX/planner_node/start` history line still exists.
5. Recording profiles (`record.sh <profile>`): `eval` (default, run-paired bag), `eval-viz`,
   `eval-camera` (paper video + live view), `mapping-replay` (raw depth for offline map
   rebuild), `full-debug`. Fly `eval` always; add at most one heavy profile per flight.
6. After the session: `scripts/evaluate/offload_runs.sh` on the Jetson — copies finished runs
   (+ bags) to the PC results root (`data/real_world/`), verifies with a full checksum pass,
   and only then deletes the Jetson copies (manifest kept on both machines).
7. On the PC: `scripts/evaluate/eval_real.sh <experiment_dir>` — per-run volume evaluation
   (box auto-shifted by each run's offset.txt), multi-series graphs, milestones table,
   path/velocity, `RESULTS.txt`.

Environment switch = edit the `bounded_box` in `config/{AEP,NBV}plannerReal.yaml` **and** the
`gain_evaluation` box in `config/GainConfigReal.yaml` (both takeoff-relative).

## Testing in the MRS simulator (no mavros)

Raw mavros setpoints do NOT work inside the MRS sim: either there is no FCU behind mavros at
all (MRS multirotor sim), or `mrs_uav_px4_api` owns the FCU and its own setpoint stream wins.
Use the bridge instead (`scripts/mrs_sim_bridge.py`): it converts MRS `uav_state` → the
planner's PoseStamped input, and the planner's PositionTarget → `control_manager/reference`.

```bash
# 1. normal MRS sim up (motion_planning tmux session), drone flying
roslaunch motion_planning_real_world RealPlannerSimTest.launch planner:=aep   # or nbv
roslaunch cache_nodes cache.launch     # AEP only (sim variant of cached)
rosservice call /uav1/planner_node/start
```

The test launch overrides the frames to the MRS convention (`uavX/world_origin`, `uavX/fcu`,
prefixed camera frame) since the real planner's tf2 lookups don't auto-prefix. Boxes stay the
real (takeoff-relative) ones — widen them in the yamls for bigger sim sweeps.

## Field link (Alfa AWUS036ACM, PC <-> Jetson)

The Alfa AC1200 (mt76 driver, Linux-native) is the field link for offload + live monitoring.
Recommended topology: **Jetson as 5 GHz AP** (no external infra needed):
- atlas: `hostapd` on the Alfa (5 GHz, WPA2), static `192.168.50.1`, `dnsmasq` for DHCP —
  one-time setup ON THE JETSON (not yet done; see task list).
- PC joins the AP, static/leased `192.168.50.2` — matches `PC_HOST` in offload_runs.sh.
- Live rviz on the PC during flights:
  `export ROS_MASTER_URI=http://192.168.50.1:11311 ROS_IP=192.168.50.2` (and on atlas
  `ROS_IP=192.168.50.1`) → markers/mesh/compressed camera stream in rviz in real time.
- Verify throughput once with `iperf3 -s` (atlas) / `iperf3 -c 192.168.50.1` (PC); expect
  ~20-40 MB/s realistic — a multi-GB session offloads in minutes.

## Package layout

- `src/{AEPReal,NBVReal}` — the ported planners; `src/planner_helpers_real.cpp` — mrs-free
  helpers (sim `planner_helpers` minus the benchmark section).
- `src/{KinoAEPReal,KinoNBVReal}` — older mrs-based kinodynamic variants (not yet ported).
- `config/` — planner yamls + `GainConfigReal.yaml` (real gain box; real launches load this
  instead of the sim GainConfig).
- `launch/` — per-planner launches, `sim_test/RealPlannerSimTest.launch`, voxblox/pointcloud
  processing, `tf_realsense_connect_mavros.launch`.
- `scripts/` — `mrs_sim_bridge.py`, `start_gate.py`, `rviz_bbx.py`;
  `scripts/evaluate/` — `eval_data_node_real.py` (stage-1 recorder),
  `eval_real.sh` (stage-2 orchestrator, PC), `offload_runs.sh` (Jetson→PC).
