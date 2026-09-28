# Recording Profiles

The flight sessions record a bag with `record.sh <profile>`, where the profile is `eval`,
`eval-viz`, `eval-camera`, `mapping-replay` or `full-debug`. The Rosbag window of the sessions
uses `eval`, which is also the default.

## Choosing a profile

| Situation | Profile | Scored run? |
|---|---|---|
| Scored experiment, the normal case | `eval` | yes |
| Scored experiment, replayed in RViz later | `eval-viz` | yes |
| Scored experiment, with the camera images | `eval-camera` | yes |
| Rebuilding the map offline with other depth_to_pointcloud or voxblox settings | `mapping-replay` | no |
| Diagnosing a problem that is not yet understood | `full-debug` | no |

The first three profiles write to `$EXP_DIR/tmp_bags/tmp_bag_<date>.bag` under the node name
`eval_bag_recorder`, which the recorder node stops at the end of a run, so these runs are fully
scored. The last two write to `~/bag_files/<date>/` and are not used by the evaluation, so a run
recorded with them has no path length or average velocity.

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
`PLANNER` and `GAIN` in `env.sh`. The bags of the large profiles stay in `~/bag_files`, and
`offload_runs.sh` copies them to a separate `session_bags` folder on the PC so they are kept apart
from the results.

## eval

The topics a scored run needs, about 1 MB per minute. This is the profile for experiments.

| Topic | Use |
|---|---|
| `/mavros/local_position/pose` | path length and average velocity (`path_vel_mapped.py`) |
| `/mavros/local_position/velocity_local` | the speed condition for reaching a waypoint |
| `/mavros/state` | armed state and flight mode, for the timeline of the flight |
| `/mavros/setpoint_raw/local` | the commands sent by the planner |
| `/tf`, `/tf_static` | body and camera frames |
| `/$UAV_NAME/offset_out` | the takeoff offset, by which the evaluation box is shifted |
| `/$UAV_NAME/simulation_ready` | start and end of the mission |

## eval-viz

`eval` plus the voxblox mesh, the occupied nodes and the esdf, tsdf and surface pointclouds, so
the flight can be replayed in RViz. The pointclouds take most of the space, from a few hundred MB
to a few GB for a ten minute flight depending on how much of the box is mapped.

## eval-camera

`eval-viz` plus the compressed colour image and its `camera_info`, about 30 MB per minute at
640x360 and 15 Hz. It shows what the drone saw, which also helps when a mapping problem may come
from exposure or motion blur.

## mapping-replay

The topics needed to run `depth_to_pointcloud` and voxblox again offline. The aligned depth is
stored raw, as the pipeline uses it, which is about 420 MB per minute at 640x360 and 15 Hz, or
4.5 GB for a ten minute flight.

The colour image is stored compressed, while `depth_to_pointcloud` subscribes to the raw image.
Convert it while replaying:

```bash
rosrun image_transport republish compressed \
  in:=/camera/color/image_raw raw out:=/camera/color/image_raw
```

Then play the bag and start `depth_to_pointcloud_rw.launch` and `processed_voxblox.launch`.

## full-debug

Every topic except parameter updates and the theora and compressedDepth copies of the images,
tens of GB per flight. It is meant for diagnosing problems during a setup day, not for scored
runs.

## Checking the recording

The Rosbag window prints a line every 15 seconds, set by `RECORD_HEARTBEAT`:

```
[record] RECORDING    120s     46M  tmp_bag_2026-09-03-10-15-04.bag.active
```

When these lines stop, the recording has stopped. On exit the window prints one of:

```
[record] STOPPED after 612s, bag closed: .../tmp_bag_....bag (46M)
[record] STOPPED after 612s but a .active file remains, the bag was NOT closed
```

The second means the bag was not closed, which happens when the recorder is killed. The
evaluation only reads closed bags, so the run would have no path length. The bag can be recovered
with:

```bash
rosbag reindex tmp_bag_<date>.bag.active
mv tmp_bag_<date>.bag.active tmp_bag_<date>.bag
```

The bag is closed properly when the recorder is stopped with `rosnode kill /eval_bag_recorder`,
as the recorder node does at the end of a run, with Ctrl+C in its pane, or with `./kill.sh`, which
sends Ctrl+C to every pane before closing the session.

## Disk space

Check the free space before a session with several flights, especially with the larger profiles:

```bash
df -h ~ ; du -sh ~/real_experiments ~/bag_files
```

`scripts/evaluate/offload_runs.sh` moves the runs to the PC and frees the space.
