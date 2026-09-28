# Recording Profiles

The flight sessions record a bag with `record.sh <profile>`, where the profile is `eval`,
`eval-viz`, `eval-camera`, `mapping-replay` or `full-debug`. The Rosbag window of the sessions
uses `eval`, the default.

## Choosing a profile

| Situation | Profile | Scored run? |
|---|---|---|
| Scored experiment, the normal case | `eval` | yes |
| Scored experiment, replayed in RViz later | `eval-viz` | yes |
| Scored experiment, with the camera images | `eval-camera` | yes |
| Rebuilding the map offline with other depth_to_pointcloud or voxblox settings | `mapping-replay` | no |
| Diagnosing problems | `full-debug` | no |

The first three profiles write to `$EXP_DIR/tmp_bags/tmp_bag_<date>.bag` under the node name
`eval_bag_recorder`, and the recorder node stops them at the end of a run. These runs are fully
scored. The last two write to `~/bag_files/<date>/` and are not used by the evaluation, so their
runs have no path length or average velocity.

## Folder layout

```
~/real_experiments/
  rhnbvp_marginal/                 one folder per planner and gain variant
    20260903_101500/               one run: voxblox_data.csv, voxblox_maps/, offset.txt, ...
    tmp_bags/tmp_bag_<date>.bag    the scored bag, matched to its run by timestamp
  aep_absolute/
  tmux_logs/                       the logs of every tmux session
    rhnbvp_marginal/
      1_<date>/tmux/*.log          one log per window, one folder per session
      2_<date>/tmux/*.log
      latest -> 2_<date>
    aep_absolute/
    latest -> rhnbvp_marginal/2_<date>
~/bag_files/<date>/                mapping-replay and full-debug only
```

`eval_rw.sh ~/real_experiments` uses the variant folders as labels. The variant name comes from
`PLANNER` and `GAIN` in `env.sh`. `offload_runs.sh` copies the bags of the large profiles to a
separate `session_bags` folder on the PC, apart from the results.

## eval

The topics needed for a scored run, about 1 MB per minute. Used for experiments.

| Topic | Use |
|---|---|
| `/mavros/local_position/pose` | path length and average velocity (`path_vel_mapped.py`) |
| `/mavros/local_position/velocity_local` | the speed condition for reaching a waypoint |
| `/mavros/state` | armed state and flight mode, for the timeline of the flight |
| `/mavros/setpoint_raw/local` | the commands sent by the planner |
| `/tf`, `/tf_static` | body and camera frames |
| `/$UAV_NAME/offset_out` | the takeoff offset, used to shift the evaluation box |
| `/$UAV_NAME/simulation_ready` | start and end of the mission |

## eval-viz

`eval` plus the voxblox mesh, the occupied nodes and the esdf, tsdf and surface pointclouds, to
replay the flight in RViz. Up to a few GB for a ten minute flight.

## eval-camera

`eval-viz` plus the compressed colour image and its `camera_info`, about 30 MB per minute. Used to
see what the drone saw.

## mapping-replay

The topics needed to run `depth_to_pointcloud` and voxblox again offline. The aligned depth is
stored raw, about 4.5 GB for a ten minute flight.

The colour image is stored compressed and `depth_to_pointcloud` needs it raw. Convert it while
replaying:

```bash
rosrun image_transport republish compressed \
  in:=/camera/color/image_raw raw out:=/camera/color/image_raw
```

Then play the bag and start `depth_to_pointcloud_rw.launch` and `processed_voxblox.launch`.

## full-debug

Every topic except parameter updates and compressed image copies, tens of GB per flight. Used for
diagnosing problems.

## Checking the recording

The Rosbag window prints a line every 15 seconds, set by `RECORD_HEARTBEAT`:

```
[record] RECORDING    120s     46M  tmp_bag_2026-09-03-10-15-04.bag.active
```

If these lines stop, the recording has stopped. On exit the window prints one of:

```
[record] STOPPED after 612s, bag closed: .../tmp_bag_....bag (46M)
[record] STOPPED after 612s but a .active file remains, the bag was NOT closed
```

The second means the bag was not closed and the run has no path length. Recover the bag with:

```bash
rosbag reindex tmp_bag_<date>.bag.active
mv tmp_bag_<date>.bag.active tmp_bag_<date>.bag
```

The bag closes properly with `rosnode kill /eval_bag_recorder`, with Ctrl+C in its pane, or with
`./kill.sh`.

## Disk space

Check the free space before a session with several flights:

```bash
df -h ~ ; du -sh ~/real_experiments ~/bag_files
```

`scripts/evaluate/offload_runs.sh` moves the runs to the PC and frees the space.
