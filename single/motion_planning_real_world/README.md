# Real-World Motion Planning

Real-world (mavros/ArduPilot) versions of the exploration planners. `AEP_rw` and `RH_NBVP_rw`
are ports of the current sim planners (`motion_planning/{AEP,RH_NBVP}`) with the same layout and
feature set — GPU marginal gain, batched expansion, recovery/backtrack, edge collision — and
these real-world deltas:

- **mavros instead of MRS**: pose in from `geometry_msgs/PoseStamped`
  (`/mavros/local_position/pose`), commands out as `mavros_msgs/PositionTarget`
  (`/mavros/setpoint_raw/local`). No mrs_lib/mrs_msgs in the AEP/RH_NBVP code paths
  (`KAEP_rw` and `KRH_NBVP_rw` are older and still use mrs_lib).
- **Start offset, automatic**: configs define the bounded box and the gain box RELATIVE TO THE
  TAKEOFF POSE. On `~start` the planner snapshots the current pose, shifts both boxes by it
  (`GainEvaluator::setWorldOffset`), and publishes the offset LATCHED on `offset_out` (the
  cached frontier server consumes it). `~offset` re-captures manually; it is idempotent.
- **No benchmark suites, execution horizon fixed to 1** (RH_NBVP flies one step per replan; AEP
  flies its chosen branch as a waypoint chain with distance+yaw advance).
- **No auto-takeoff**: take off yourself (GUIDED), then call `~start`.

## How to fly

1. Pre-flight: `scripts/evaluate/offload_runs.sh --check` (refuses below 15 GB free).
2. `tmux/one_drone_rw/{aep,kaep,rh_nbvp,krh_nbvp}.sh` — brings up mavros (`apm.launch`), realsense,
   TF connect, voxblox, pointcloud processing, the planner, (AEP) the cached frontier server
   via `cache_nodes cache_rw.launch`, the experiment recorder
   (`evaluate_map_rw.launch` — waits for mission start, does NOT start anything), the
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
7. On the PC: `scripts/evaluate/eval_rw.sh <experiment_dir>` — per-run volume evaluation
   (box auto-shifted by each run's offset.txt), multi-series graphs, milestones table,
   path/velocity, `RESULTS.txt`.

Environment switch = edit the `bounded_box` in `config/{AEP,RH_NBVP}_rw.yaml` **and** the
`gain_evaluation` box in `config/GainConfig_rw.yaml` (both takeoff-relative), then pick the site
yaml the evaluation measures (`config/<Site>.yaml`, its `reconstruction_box`) with
`EVAL_CONFIG=<Site>.yaml eval_rw.sh ...`. `scripts/evaluate/check_boxes.py` checks that the three
boxes agree before a flight.

## Field link (Alfa AWUS036ACM, PC <-> Jetson)

The Alfa AC1200 (mt76 driver, Linux-native) is the field link for offload + live monitoring.
Recommended topology: **Jetson as 5 GHz AP** (no external infra needed):
- Jetson: `hostapd` on the Alfa (5 GHz, WPA2), static `<jetson-ip>`, `dnsmasq` for DHCP —
  one-time setup on the Jetson.
- PC joins the AP, static/leased `<pc-ip>` — matches `PC_HOST` in offload_runs.sh.
- Live rviz on the PC during flights:
  `export ROS_MASTER_URI=http://<jetson-ip>:11311 ROS_IP=<pc-ip>` (and on the Jetson
  `ROS_IP=<jetson-ip>`) → markers/mesh/compressed camera stream in rviz in real time.
- Verify throughput once with `iperf3 -s` (Jetson) / `iperf3 -c <jetson-ip>` (PC); expect
  ~20-40 MB/s realistic — a multi-GB session offloads in minutes.

## Package layout

- `src/{AEP,RH_NBVP}/` — the ported planners; `src/planner_helpers_rw.cpp` — mrs-free helpers
  (sim `planner_helpers` minus the benchmark section); `src/mavros_tf_broadcaster.cpp` — mavros
  pose to TF, glitched poses dropped.
- `src/{KAEP,KRH_NBVP}/` — older mrs-based kinodynamic variants (not yet ported).
- `config/` — planner yamls, `GainConfig_rw.yaml` (real gain box; the real launches load this
  instead of the sim GainConfig) and one yaml per field site (`Basketball`, `Lamp`,
  `LongClearing`, `LongGrove`, `PatioLamp`, `ShortGrove`) holding its `reconstruction_box`.
- `launch/` — `planner_rw.launch` (`planner:=aep|rhnbvp|kaep|krhnbvp`), `start_gate.launch`,
  `processed_voxblox.launch`, `depth_to_pointcloud_rw.launch`,
  `tf_realsense_connect_mavros.launch`, `evaluate/{evaluate_map_rw,evaluate_plot_rw}.launch`.
- `scripts/` — `start_gate.py`, `pose_watchdog.py`, `rviz_bbx.py`;
  `scripts/evaluate/` — `eval_data_node_rw.py` (stage-1 recorder),
  `eval_rw.sh` (stage-2 orchestrator, PC), `offload_runs.sh` (Jetson→PC), `check_boxes.py`
  (pre-flight box check).
