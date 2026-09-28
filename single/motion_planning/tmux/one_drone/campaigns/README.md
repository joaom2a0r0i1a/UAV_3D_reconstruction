# Campaign files for `run_campaign.sh`

One parameterized driver (`../run_campaign.sh`) + `../lib_campaign.sh` run every experiment
campaign. Each campaign is a small sourced-bash file here. Settings reach the stack as
environment variables, no config file is edited.

## Run

```bash
cd ..
./run_campaign.sh --dry-run   campaigns/aep_smoke.conf   # print settings and commands, no launch
./run_campaign.sh             campaigns/aep_smoke.conf   # run + eval
./run_campaign.sh -N 5        campaigns/aep_smoke.conf   # override target good runs
./run_campaign.sh -T 900      campaigns/aep_smoke.conf   # override the per-run time limit
./run_campaign.sh --eval-only campaigns/aep_smoke.conf   # re-evaluate existing runs
./run_campaign.sh --no-eval   campaigns/aep_smoke.conf   # runs only
```

`--dry-run` prints the resolved world, planner and per-condition settings plus the
supervise/eval commands it *would* run. It never touches the container or tmux and writes
nothing in the repo.

## Campaign fields

| field | values | meaning |
|-------|--------|---------|
| `WORLD` | `school` \| `police` \| `warehouse` \| `multistory` \| `big_maze` | world, spawn and regions from `uav_gazebo_environments/config/<world>.yaml`, plus the per-world planner tables in `lib_campaign.sh` |
| `PLANNER` | `aep` \| `nbvp` \| `kaep` \| `krhnbvp` | planner launched by `planner.launch` |
| `N` | int | target **good** runs per condition (supervise tops up to this) |
| `T` | seconds | wall-clock kill per run, empty = world time limit + 50 s |
| `EARLY_STOP` | `true` \| `false` | stop ~`GRACE` s after the planner self-terminates, pad the coverage curve |
| `GRACE` | seconds | early-stop grace (default 60.0) |
| `KEEP_FOLDER` | `multi_series_<name>` | where the stage-2 render is moved |
| `SUMMARY` | filename | decision summary written under `variants_logs/` |
| `RESTORE` | `true` \| `false` | reset the exported settings to school / AEP marginal at the end (default true) |
| `MAP_KEEP` | int | after evaluation keep every `MAP_KEEP`-th voxblox map and the last (default 5) |
| `CONDITIONS` | array | one line per condition (see below) |

`VOXEL_SIZE` (default 0.2, or from the environment) can also be set in the conf.

## Condition format

**AEP:** `label|gain|_|rrt_star|objective|N_max|N_termination|N_min_nodes`
- `gain` = `abs` \| `marg` \| `control` (`control` = absolute gain)
- `rrt_star` = `false` \| `true` (optional, default false)
- `objective` = `expdecay` \| `rate_L` (optional, default expdecay)
- node set (optional, empty keeps the world default)

**RH_NBVP:** `label|gain|nmax|nterm|step|fixed|optyaw|objective|horizon`
- `gain` = `abs` \| `marg`
- `nterm` MUST be `> nmax` (receding-horizon ceiling, asserted)
- `fixed` = `fixed_step` (true = every edge == step_size)
- `optyaw` = `optimize_yaw` (optional, default true)
- `objective` = `expdecay` \| `rate_L` (optional, default expdecay)
- `horizon` = execution horizon in steps (optional, default 1)

## Presets

| conf | what it runs |
|------|--------------|
| `aep_smoke.conf`, `nbvp_smoke.conf` | one short run, pipeline check |
| `aep_school_GL.conf`, `nbvp_school_GL.conf` | school, rate_L objective, absolute vs marginal |
| `nbvp_school_yawopt.conf` | school RH-NBVP with yaw optimization, absolute vs marginal |
| `nbvp_school_nmax_sweep.conf` | school RH-NBVP node-count sweep |
| `nbvp_school_step_fixed.conf` | school RH-NBVP fixed step |
| `school_nbvp_fixedstep_{50_300,500_1000,1000_1500}.conf` | school RH-NBVP fixed-step node sets |
| `warehouse_aep.conf`, `warehouse_nbvp.conf` | warehouse, absolute vs marginal |
| `multistory_n10.conf`, `multistory_nbvp.conf` | multistory, absolute vs marginal |
| `big_maze_aep.conf`, `big_maze_nbvp.conf` | big maze, absolute vs marginal |
| `big_maze_nbvp_GL.conf` | big maze RH-NBVP, rate_L objective |
| `big_maze_nbvp_h3_10v10.conf`, `big_maze_nbvp_h5.conf` | big maze RH-NBVP execution horizon 3 and 5 |

Shared infra: `run_experiments.sh`, `supervise_runs.sh`, `environment.sh`, `session.yml`,
`start.sh`, `kill.sh`.
