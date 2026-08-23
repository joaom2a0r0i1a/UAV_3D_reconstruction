#!/usr/bin/env python3
# termination_time.py — AEP-only planner termination time, log-based: per run,
# ("Planner terminated" timestamp - "Succesfully started the simulation" timestamp)
# from data_log.txt. RH-NBVP never self-terminates (hits the time limit) -> NA/excluded.
# Usage: MP=<motion_planning_dir> termination_time.py <label1> [label2 ...]
import os, glob, re, sys
from datetime import datetime

DATA = os.path.join(os.environ.get('MP', ''), 'data')
TS = re.compile(r"^\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})\]")


def term_minutes(log):
    start = end = None
    with open(log, errors='replace') as fh:
        for line in fh:
            m = TS.match(line)
            if not m:
                continue
            t = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
            if start is None and "started the simulation" in line:
                start = t
            elif "Planner terminated" in line:
                end = t
                break
    if start is None or end is None:
        return None
    return (end - start).total_seconds() / 60.0


def stats(vals):
    vals = [v for v in vals if v is not None]
    n = len(vals)
    if n == 0:
        return (0, float('nan'), float('nan'))
    m = sum(vals) / n
    return (n, m, (sum((x - m) ** 2 for x in vals) / n) ** 0.5)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: MP=<dir> termination_time.py <label1> [label2 ...]")
    print("PLANNER TERMINATION TIME (log-based, min; AEP-only)")
    summary = []
    for L in sys.argv[1:]:
        runs = sorted(glob.glob(os.path.join(DATA, L, '2*')))
        vals = []
        print("\n### %s (%d runs)" % (L, len(runs)))
        for d in runs:
            log = os.path.join(d, 'data_log.txt')
            tm = term_minutes(log) if os.path.isfile(log) else None
            print("   %-20s term=%s" % (os.path.basename(d), "%.2f min" % tm if tm else "NA (no self-termination)"))
            vals.append(tm)
        n, m, s = stats(vals)
        print("   -> N=%d  mean=%.2f  std=%.2f  min" % (n, m, s))
        summary.append((L, n, m, s))
    print("\nSUMMARY")
    for L, n, m, s in summary:
        print("  %-24s N=%d  mean=%.2f  std=%.2f min" % (L, n, m, s))


if __name__ == '__main__':
    main()
