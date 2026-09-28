# Gain Benchmark Figures

Two scripts turn the planner's gain benchmark into tables and figures.

| script | input | output |
|---|---|---|
| `timing_analyze.py` | `timing_n<N>.log`, one per tree size | `timing_table.txt`, cost against N, compute against transfer, full algorithm |
| `accuracy_analyze.py` | `accuracy_n<N>.csv`, one per tree size | `gain_agreement.*`, `gain_overestimate.*`, gain and overestimate against depth |

N is 50, 100, 500, 1000, 5000 or 10000, any subset works.

## Producing the inputs

The benchmark runs inside the normal simulation (`tmux/one_drone/start.sh`) with RH-NBVP, the
session default. One run per tree size.

1. Tree size and benchmark start, in a yaml passed as overrides. `N_termination` must be larger
   than `N_max`.

   ```yaml
   # n500.yaml
   rrt:
     N_max: 500
     N_termination: 1000
   benchmark:
     timing_after_s: 600.0 # [s] sim time before the benchmark starts
     max_replans: 10       # benchmarked replans
   ```

2. Export in the shell that runs `start.sh`. The `AEP_` variables apply to both planners.

   ```bash
   mkdir -p ~/figures_in
   export PLANNER_OVERRIDES=$PWD/n500.yaml
   export AEP_BENCHMARK=true
   export AEP_BENCH_SUITE="accuracy timing"
   export RH_NBVP_ACCURACY_CSV=~/figures_in/accuracy_n500.csv
   ./start.sh
   ```

   The accuracy CSV is written directly, header included.

3. After the run, keep the timing lines of the planner log.

   ```bash
   grep -hE "\[timing_(marg|abs|cpu|full)\]" ~/.ros/log/latest/rosout.log* > ~/figures_in/timing_n500.log
   ```

## Running the scripts

```bash
python3 timing_analyze.py ~/figures_in [out_dir]
python3 accuracy_analyze.py ~/figures_in [out_dir]
```

Output goes to the input folder unless `out_dir` is given. `TIMING_TAG` changes the log prefix
(default `timing_n`), `DEPTH_N` and `DEPTH_MAX` pick the trees and depths of the accuracy panels.
