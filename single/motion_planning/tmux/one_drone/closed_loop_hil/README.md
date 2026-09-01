# Closed-loop HIL (Hardware-in-the-Loop) — X2 gain-evaluation timing

Gazebo + MRS control run on the **desktop computer**; voxblox + RH-NBVP planner + GPU
gain run on the **onboard Jetson Orin**, over a shared ROS network. We benchmark the
gain-evaluation time (CPU vs GPU, marginal vs absolute) **live, in-situ, under closed-loop
load**. Distinct from the path-A replay experiment (serialize on desktop → replay isolated
on the Jetson).

Files are split by the machine you execute them on:

```
closed_loop_hil/
  config.sh.template    copy -> config.sh (gitignored) with your ORIN + DESKTOP_IP
  desktop_computer/     run on the ground desktop PC
    campaign.sh         ENTRY POINT: sweeps voxel × N_max; per config = fresh sim ->
                        wait until the cloud streams on BOTH machines -> ssh the Jetson
                        worker -> pull its log -> teardown. Env knobs: VOXELS, NS, WALLCAP.
    sim.sh              launches Gazebo + MRS (sim+control+sensors) inside the noetic_ws
                        container; reads ../../config/*.yaml + ../../current_config.env.
  onboard_jetson/       run on the Jetson Orin (also mirrored under jm_ws on the Jetson)
    run_config.sh V N   one config: seds NBVPlanner.yaml (N_max, N_termination=max(2N,300),
                        timing_after_s=600, x2_max=25, recovery_timeout=900), launches the
                        stack, starts the mission, waits for 10 whole-tree captures, tears down.
    stack.sh            launches voxblox + RH-NBVP planner (x2 suite) + cache; planner
                        console tee'd to the log. Sources ~/hil_env.sh.
    env.sh.template     template for the Jetson's ~/hil_env.sh (ROS master/IP).
```

## Run
```bash
# ONE-TIME setup (both hold only YOUR values; both are gitignored / not published):
#   desktop:  cp closed_loop_hil/config.sh.template closed_loop_hil/config.sh   # set ORIN + DESKTOP_IP
#   Jetson:   cp env.sh.template ~/hil_env.sh                                   # set the desktop master IP
# desktop PC:
cd ~/ros1_motion_ws/src/UAV_3D_reconstruction/single/motion_planning/tmux/one_drone/closed_loop_hil/desktop_computer
bash campaign.sh                                  # full sweep {0.2,0.1} × {50,100,500,1000,5000,10000}
VOXELS="0.2" NS="5000" bash campaign.sh           # subset
VOXELS="0.1" NS="10000" WALLCAP=3600 bash campaign.sh   # heavy config: 60-min cap (see below)
```
Pulled logs land in `desktop_computer/raw_logs/` (staging). Move the keepers into
`data/timing/X2_jetson_hilB/voxel_size_0_{2,1}/logs/` and analyse with
`scripts/evaluation/figures/x2_online.py` (see that folder's README for exact commands).

## Gotchas (baked into run_config.sh)
- **`N_termination > N_max`** or RH-NBVP self-terminates after one cycle.
- **`recovery_timeout: 900`, keep `recovery_enabled: true`** — the big-N build+benchmark cycle
  overruns the default 12 s backtrack budget; do NOT set `recovery_enabled=false` (its guard
  also stops the tree-build collide-retry, so a boxed drone spins at 100 % CPU forever).
- **0.1/N10000**: one cycle is ~2–4 min (CPU G_all), so 10 captures may not fit one run —
  raise `WALLCAP` and/or run again, keeping each run's log under a distinct name, then combine.

Full write-up + results: `data/HIL_PATH_B_REPRODUCE.md` and `data/timing/X2_jetson_hilB/`.
