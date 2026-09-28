#!/usr/bin/env python3
# Keeps every Nth voxblox map and the last one, after the volume eval
# Usage: thin_maps.py <dir> [keep_every=5]
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
    # Skip runs not yet evaluated
    if header.count(",") < 5:
        return ("skip", run, 0, 0)
    maps = sorted(glob.glob(os.path.join(mapdir, "*.vxblx")))
    if not maps:
        return ("thinned", run, 0, 0)
    last = max(int(os.path.splitext(os.path.basename(f))[0]) for f in maps)
    removed = 0
    for f in maps:
        idx = int(os.path.splitext(os.path.basename(f))[0])
        if idx % keep != 0 and idx != last:
            os.remove(f)
            removed += 1
    return ("thinned", run, len(maps) - removed, removed)


def main():
    if len(sys.argv) < 2:
        print("usage: thin_maps.py <run_or_label_dir> [keep_every=5]")
        sys.exit(2)
    root = sys.argv[1]
    keep = int(sys.argv[2]) if len(sys.argv) > 2 else 5
    runs = ([root] if os.path.isdir(os.path.join(root, "voxblox_maps")) else [
        d for d in sorted(glob.glob(os.path.join(root, "2*")))
        if os.path.isdir(os.path.join(d, "voxblox_maps"))
    ])
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
