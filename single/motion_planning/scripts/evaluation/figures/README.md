# Gain Benchmark Figures

Two scripts turn the planner's gain benchmark into tables and figures.

| script | input | output |
|---|---|---|
| `timing_analyze.py` | `timing_n<N>.log`, one per tree size | `timing_table.txt`, cost against N, compute against transfer, full algorithm |
| `accuracy_analyze.py` | `accuracy_n<N>.csv`, one per tree size | `gain_agreement.*`, `gain_overestimate.*`, gain and overestimate against depth |

N is 50, 100, 500, 1000, 5000 or 10000, any subset works.

## Producing the inputs

The benchmark runs during a normal RH-NBVP flight (`tmux/one_drone`), one flight per tree size.

1. Set the tree size and when the benchmark starts in `config/RH_NBVP.yaml`:

   ```yaml
   rrt:
     N_max: 500
     N_termination: 1000   # larger than N_max

   benchmark:
     timing_after_s: 600.0 # [s] benchmark start time
     max_replans: 10       # benchmarked replans cap
   ```

2. Turn the benchmark on in the planner line of `tmux/one_drone/session.yml`, and choose where the
   accuracy results are written:

   ```yaml
   - waitForControl; export RH_NBVP_ACCURACY_CSV=~/figures_in/accuracy_n500.csv; roslaunch motion_planning planner.launch planner:=$PLANNER_KIND benchmark:=true benchmark_suite:="accuracy timing"
   ```

3. Create `~/figures_in` and fly with `./start.sh`. Right after the flight, keep the planner log
   as the timing input:

   ```bash
   cp ~/.ros/log/latest/rosout.log ~/figures_in/timing_n500.log
   ```

## Running the scripts

```bash
python3 timing_analyze.py ~/figures_in
python3 accuracy_analyze.py ~/figures_in
```

The tables and figures are written next to the inputs. A second folder can be given as the output
instead.
