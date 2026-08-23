# Campaign files for `run_campaign.sh`

One parameterized driver (`../run_campaign.sh`) + `../lib_campaign.sh` replace the
~25 one-off experiment drivers. Each campaign is a small sourced-bash file here.

## Run

```bash
cd ..
./run_campaign.sh --dry-run campaigns/school_n10.conf   # validate config injection, no launch
./run_campaign.sh          campaigns/school_n10.conf    # configure + run + eval
./run_campaign.sh -N 5     campaigns/school_n10.conf     # override target good runs
./run_campaign.sh --eval-only campaigns/school_n10.conf # re-evaluate existing runs
./run_campaign.sh --no-eval    campaigns/school_n10.conf # runs only
```

`--dry-run` copies the 4 config files to a temp dir, applies the edits there, and
prints the resulting diffs + the supervise/eval commands it *would* run — it never
touches the real configs, the container, or tmux.

## Campaign fields

| field | values | meaning |
|-------|--------|---------|
| `WORLD` | `school` \| `police` | session.yml world+spawn, AEP bounded_box, GainConfig region |
| `PLANNER` | `aep` \| `nbvp` | session.yml planner launch line |
| `N` | int | target **good** runs per condition (supervise tops up to this) |
| `T` | seconds | wall-clock kill per run (school 1850, police 950) |
| `EARLY_STOP` | `true`\|`false` | stop ~`GRACE`s after planner self-terminates, pad coverage curve |
| `GRACE` | seconds | early-stop grace (default 60.0) |
| `KEEP_FOLDER` | `multi_series_<name>` | where the stage-2 render is moved |
| `SUMMARY` | filename | decision summary written under `variants_logs/` |
| `RESTORE` | `true`\|`false` | restore repo to school / AEP-marginal-R1A at end (default true) |
| `CONDITIONS` | array | one line per condition (see below) |

## Condition format

**AEP:** `label|gain|variant[|rrt_star]`
- `gain` = `abs` \| `marg` \| `control`
- `variant` = `R1A|R2A|R1B|R2B` — R1/R2 = `marginal_edge_follow_yaw` false/true, A/B = `marginal_score_pathsum` true/false
- `rrt_star` = `false`\|`true` (optional, default false)

**NBVP:** `label|gain|nmax|nterm|step|fixed[|optyaw]`
- `gain` = `abs` \| `marg`
- `nterm` MUST be `> nmax` (receding-horizon ceiling; asserted)
- `fixed` = `fixed_step` (true = every edge == step_size)
- `optyaw` = `optimize_yaw` (optional, default true)

## Presets → which old drivers they replace

| conf | replaces |
|------|----------|
| `school_n10.conf` | school_to10_fs, school_campaign, school_to5/to9/3h, school_control_absR1A* (use `-N`) |
| `police_n10.conf` | police_to10_fs, police_absmarg_fresh_fs, police_campaign, police_all_to10_campaign |
| `police_variants.conf` | police_variants_campaign |
| `aep_school_rrt_sweep.conf` | aep_campaign |
| `nbvp_school_yawopt.conf` | nbvp_school_yawopt_2way |
| `nbvp_school_nmax_sweep.conf` | nbvp_school_nmax_sweep, nbvp_day1_complete/resume/extra, nbvp_day2_n500 |
| `nbvp_school_step_fixed.conf` | nbvp_overnight_20260730, nbvp_day2b_n250_fixedstep, nbvp_step05_n50_1200 |

Shared infra kept as-is: `run_experiments.sh`, `supervise_runs.sh`, `session.yml`,
`start.sh`, `kill.sh`, `multi_start.sh`. Eval-only helpers `aep_eval.sh`/`pol_eval.sh`
are covered by `--eval-only`.
