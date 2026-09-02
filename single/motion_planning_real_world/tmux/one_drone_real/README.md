# Recording profiles

`record.sh <profile>` — `eval` | `eval-viz` | `eval-camera` | `mapping-replay` | `full-debug`

Default is `eval`. The Rosbag window of the flight sessions runs `record.sh eval`.

## Which one do I use?

| Situation | Profile | Scored run? |
|---|---|---|
| Scored experiment, the normal case | `eval` | yes |
| Scored experiment, want RViz playback later | `eval-viz` | yes |
| Scored experiment, want to see what the camera saw | `eval-camera` | yes |
| Rebuilding the map offline with other cam_to_ptcld or voxblox settings | `mapping-replay` | no |
| Something is broken and you cannot yet name it | `full-debug` | no |

The first three write to `$EXP_DIR/tmp_bags/tmp_bag_<date>.bag` under the node name
`eval_bag_recorder`, which is what `eval_data_node_real.py` looks for and kills at the end of a
run, so any of them yields a fully scored run. The last two write to `~/bag_files/<date>/`
instead and do **not** feed the eval pipeline: a run recorded with those has no path length and
no average velocity.

## Where everything lands

```
~/real_experiments/
  rhnbvp_marginal/                 one directory per planner and gain variant
    20260903_101500/               one run: voxblox_data.csv, voxblox_maps/, offset.txt, ...
    tmp_bags/tmp_bag_<date>.bag    the scored bag, matched to runs by timestamp
  aep_absolute/
  tmux_logs/                       every tmux session, grouped away from the run data
    rhnbvp_marginal/
      1_<date>/tmux/*.log          one log per window, one folder per session
      2_<date>/tmux/*.log
      latest -> 2_<date>
    aep_absolute/
    latest -> rhnbvp_marginal/2_<date>
~/bag_files/<date>/                mapping-replay and full-debug only
```

`eval_real.sh ~/real_experiments` picks up the variant folders as its labels. The variant name
comes from `PLANNER` and `GAIN` in `env.sh`. The heavy profiles stay under `~/bag_files` on
purpose: `offload_runs.sh` ships those to their own `session_bags` destination on the PC instead
of mixing tens of GB into the results tree.

## eval

The eight topics a scored run cannot do without. About 1 MB per minute, so it is never what
fills the disk. Use it unless you have a reason not to.

| Topic | Why |
|---|---|
| `/mavros/local_position/pose` | path length and average velocity (`path_vel_mapped.py`) |
| `/mavros/local_position/velocity_local` | the speed term of the waypoint arrival gate |
| `/mavros/state` | armed and mode, for reconstructing the timeline |
| `/mavros/setpoint_raw/local` | what the planner commanded, the record of the control loop |
| `/tf`, `/tf_static` | body and camera frames |
| `/$UAV_NAME/offset_out` | the takeoff offset, the eval box is shifted by it |
| `/$UAV_NAME/simulation_ready` | mission start and stop edges |

## eval-viz

`eval` plus the map and planner visualisation, so the flight can be replayed in RViz: the
voxblox mesh, occupied nodes, and the esdf/tsdf/surface pointclouds. The pointclouds are the
expensive part — hundreds of MB to a few GB for a ten minute flight, depending on how much of
the box gets mapped.

## eval-camera

`eval-viz` plus the colour image (compressed) and its `camera_info`. Compressed colour at
640x360 and 15 Hz is roughly 30 MB per minute. Use it to show what the drone saw, or to
diagnose a mapping failure that might really be exposure or motion blur.

## mapping-replay

Everything needed to re-run `cam_to_ptcld` and voxblox offline, and nothing else. Aligned depth
is stored raw because that is what the pipeline consumes: roughly 420 MB per minute at 640x360
and 15 Hz, so budget about 4.5 GB per ten minutes.

Colour is stored **compressed** to keep the size sane, but `cam_to_ptcld` subscribes to the raw
topic. Bridge it before replaying:

```bash
rosrun image_transport republish compressed \
  in:=/camera/color/image_raw raw out:=/camera/color/image_raw
```

Then play the bag and bring up `cam_to_ptcld_real.launch` and `processed_voxblox.launch` as
usual. The N4 harness under `motion_planning/data/n4_ptcld_verify` does exactly this.

## full-debug

Records everything except parameter chatter and the theora/compressedDepth duplicates. Tens of
GB per flight. A diagnostic tool for a bring-up day, not for a scored run.

## Is it still recording?

The Rosbag window prints a line every 15 seconds (`RECORD_HEARTBEAT` to change the interval):

```
[record] RECORDING    120s     46M  tmp_bag_2026-09-03-10-15-04.bag.active
```

If those lines stop, recording stopped. On exit the window prints one of:

```
[record] STOPPED after 612s, bag closed: .../tmp_bag_....bag (46M)
[record] STOPPED after 612s but a .active file remains, the bag was NOT closed
```

The second is the bad one. A `.bag.active` file is invisible to `path_vel_mapped.py`, which
globs `tmp_bags/*.bag`, so the run would score with no path length. It happens when the
recorder is SIGKILLed. Recover it:

```bash
rosbag reindex tmp_bag_<date>.bag.active
mv tmp_bag_<date>.bag.active tmp_bag_<date>.bag
```

Three ways of stopping close the bag properly, all verified: `rosnode kill /eval_bag_recorder`
(what the recorder node does at the end of a run), Ctrl+C in the pane, and `./kill.sh`, which
sends Ctrl+C to every pane and waits before killing the session. Prefer those over killing the
tmux server.

## Disk

Check before a session with several flights, especially for anything past `eval`:

```bash
df -h ~ ; du -sh ~/real_experiments ~/bag_files
```

Offload and free space with `scripts/evaluate/offload_runs.sh`.
