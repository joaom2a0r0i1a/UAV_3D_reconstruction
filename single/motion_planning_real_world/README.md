# Real-World Motion Planning

This package contains the real-world versions of the exploration planners, for drones running
ArduPilot through mavros. `AEP_rw` and `RH_NBVP_rw` follow the simulation planners in
`motion_planning` and keep their features: marginal gain on the GPU, batched tree expansion,
recovery by backtracking and collision checks along each edge. The kinodynamic planners `KAEP_rw`
and `KRH_NBVP_rw` use mrs_lib.

The planners read the drone pose from `/mavros/local_position/pose` and send their commands to
`/mavros/setpoint_raw/local`. The planning box and the gain box are defined relative to the
takeoff pose. When the mission starts, the planner takes the current pose as the offset, shifts
both boxes by it and publishes it on `offset_out` for the cached node server. `~offset` captures
it again. RH-NBVP flies one step per replan, and AEP flies its chosen branch as a chain of
waypoints.

## Flying

1. Before a session, check the free disk space on the Jetson with
   `scripts/evaluate/offload_runs.sh --check`.
2. Start the session of the chosen planner, `tmux/one_drone_rw/{aep,kaep,rh_nbvp,krh_nbvp}.sh`. It
   brings up mavros, the RealSense camera, the transforms, voxblox, the pointcloud processing, the
   planner, the cached node server for AEP, the experiment recorder and the flight bag, together
   with a start gate window.
3. Take off manually, fly around so voxblox starts mapping, and switch to GUIDED.
4. The start gate asks "Start Planner? [Y/n]". `Y` captures the offset and starts the mission.
   After that, `s` stops the planner, `o` captures the offset again and `q` quits.
5. After the session, `scripts/evaluate/offload_runs.sh` copies the finished runs and their bags
   from the Jetson to the PC, checks the copies and deletes the runs from the Jetson.
6. On the PC, `scripts/evaluate/eval_rw.sh <experiment_dir>` evaluates every run by volume, with
   the box shifted by the offset of each run, and writes the multi-series graphs, the milestone
   table, the path length and average velocity, and a `RESULTS.txt` summary.

The recording profiles and the folder layout are described in `tmux/one_drone_rw/README.md`.

## Changing site

Each site needs three boxes, all relative to the takeoff pose: the `bounded_box` in
`config/AEP_rw.yaml`, `config/RH_NBVP_rw.yaml`, `config/KAEP_rw.yaml` and `config/KRH_NBVP_rw.yaml`, the `gain_evaluation` box in
`config/GainConfig_rw.yaml`, and the `reconstruction_box` of the site in `config/<Site>.yaml`. The
evaluation reads the site with `EVAL_CONFIG=<Site>.yaml eval_rw.sh ...`.
`scripts/evaluate/check_boxes.py` checks that the three agree before a flight.

## Package layout

- `src/AEP` and `src/RH_NBVP` contain the planners and `src/planner_helpers_rw.cpp` the helpers
  they share. `src/mavros_tf_broadcaster.cpp` publishes the mavros pose as a transform and skips
  glitched poses.
- `src/KAEP` and `src/KRH_NBVP` contain the kinodynamic planners.
- `config/` holds the planner configurations, `GainConfig_rw.yaml` with the gain box, and one
  file per site (`Basketball`, `Lamp`, `LongClearing`, `LongGrove`, `PatioLamp`, `ShortGrove`)
  with its reconstruction box.
- `launch/` holds `planner_rw.launch` (`planner:=aep|rhnbvp|kaep|krhnbvp`), the camera, pointcloud
  and voxblox launches, the start gate and the evaluation launches.
- `scripts/` holds the start gate, a pose watchdog and an RViz box display, and
  `scripts/evaluate/` the recorder node, the offload and evaluation scripts and the box check.
