#!/usr/bin/env python3
# thin_maps.py — keep every Nth voxblox map per run, delete the rest (disk hygiene).
# MUST run AFTER the volume eval has filled voxblox_data.csv: once the Volume column is
# populated the .vxblx maps are redundant (the coverage curve lives in the CSV; the
# multi-series render reads CSVs, and eval_voxblox_node tolerates missing maps).
# Keeps every Nth map AND always the final map (so the end-state coverage is preserved even
# when the last index is not a multiple of N). SAFETY: a run whose CSV is not yet
# volume-evaluated (<=4 columns) is skipped, never thinned.
# Usage: thin_maps.py <dir> [keep_every=5]
#   keep_every=5 with a 60 s map interval -> maps 5 min apart; pass another N for a different spacing.
#   <dir> = a single run dir, OR a label dir containing timestamped run dirs.
import os, sys, glob


def thin_run(run, keep):
    mapdir = os.path.join(run, "voxblox_maps")
    csv = os.path.join(run, "voxblox_data.csv")
    if not os.path.isdir(mapdir):
        return None
    try:
        header = open(csv).readline()
    except OSError:
        return None
    if header.count(",") < 5:            # not volume-evaluated yet -> maps still needed
        return ("skip", run, 0, 0)
    maps = sorted(glob.glob(os.path.join(mapdir, "*.vxblx")))
    if not maps:
        return ("thinned", run, 0, 0)
    last = max(int(os.path.splitext(os.path.basename(f))[0]) for f in maps)
    removed = 0
    for f in maps:
        idx = int(os.path.splitext(os.path.basename(f))[0])
        if idx % keep != 0 and idx != last:   # keep every Nth AND always the final map
            os.remove(f)
            removed += 1
    return ("thinned", run, len(maps) - removed, removed)


def main():
    if len(sys.argv) < 2:
        print("usage: thin_maps.py <run_or_label_dir> [keep_every=5]")
        sys.exit(2)
    root = sys.argv[1]
    keep = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    runs = ([root] if os.path.isdir(os.path.join(root, "voxblox_maps"))
            else [d for d in sorted(glob.glob(os.path.join(root, "2*")))
                  if os.path.isdir(os.path.join(d, "voxblox_maps"))])
    for r in runs:
        res = thin_run(r, keep)
        if not res:
            continue
        if res[0] == "thinned":
            print(f"  thinned {os.path.basename(r)}: kept {res[2]}, removed {res[3]}")
        else:
            print(f"  SKIP {os.path.basename(r)}: CSV not volume-evaluated yet")


if __name__ == "__main__":
    main()
