#!/usr/bin/env python3
# termination_time.py — average planner termination time per planner, from tmp_bags.
#
# "Termination" = the planner exhausts frontiers and self-stops (STATE_STOPPED).
# Uniform bag-derived proxy across ALL planners: the sim-time the drone LAST moved
# faster than a small threshold (after that it parks/hovers). Computed on the 5 Hz
# downsampled uav_state so estimator jitter while hovering doesn't count as motion.
# Also reports last /uav1/reference_out time (planner's last issued command) as a
# secondary proxy. Times are relative to the first uav_state stamp (sim time, same
# base as the %-known milestones).
import os, glob, math
import rosbag

DATA   = os.path.join(os.environ.get('MP', ''), 'data')
LABELS = ['POL_abs_RRT', 'POL_ctrl_absR1A_fs', 'POL_marg_R1A']
UAV    = '/uav1/estimation_manager/uav_state'
REF    = '/uav1/reference_out'
DT     = 1.0        # downsample step (s) for speed
SPEED  = 0.10       # m/s; below this = parked
MIN_DUR = 120.0


def analyze(path):
    t0 = None; tend = None
    dx = dy = dz = None; t_kept = None
    t_last_move = None
    t_last_ref = None
    with rosbag.Bag(path) as bag:
        for topic, msg, t in bag.read_messages(topics=[UAV, REF]):
            if topic == REF:
                # use uav-state-relative base; record raw bag time, convert later
                t_last_ref = t.to_sec()
                continue
            ts = msg.header.stamp.to_sec()
            if t0 is None:
                t0 = ts
            tend = ts
            if t_kept is None:
                dx, dy, dz, t_kept = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, ts
            elif ts - t_kept >= DT:
                x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
                sp = math.sqrt((x-dx)**2 + (y-dy)**2 + (z-dz)**2) / (ts - t_kept)
                if sp > SPEED:
                    t_last_move = ts
                dx, dy, dz, t_kept = x, y, z, ts
    dur = (tend - t0) if (t0 is not None and tend is not None) else 0.0
    term_move = (t_last_move - t0) if (t_last_move is not None and t0 is not None) else None
    return dict(dur=dur, term_move=term_move)


def stats(vals):
    vals = [v for v in vals if v is not None]
    n = len(vals)
    if n == 0:
        return (0, float('nan'), float('nan'))
    m = sum(vals) / n
    v = sum((x-m)**2 for x in vals) / n
    return (n, m, math.sqrt(v))


def main():
    print("=" * 76)
    print("PLANNER TERMINATION TIME  (last-motion sim-time, min; parked after this)")
    print("=" * 76)
    summary = []
    for L in LABELS:
        bags = sorted(glob.glob(os.path.join(DATA, L, 'tmp_bags', '*.bag')))
        vals = []
        print("\n### %s  (%d bags)" % (L, len(bags)))
        for b in bags:
            try:
                r = analyze(b)
            except Exception as e:
                print("   %-42s ERROR %s" % (os.path.basename(b), e)); continue
            if r['dur'] < MIN_DUR:
                print("   %-42s dur=%.0fs <PARTIAL, excluded>" % (os.path.basename(b), r['dur'])); continue
            tm = r['term_move']
            print("   %-42s term=%s min" % (os.path.basename(b),
                  ("%.2f" % (tm/60.0)) if tm is not None else "n/a"))
            vals.append(tm/60.0 if tm is not None else None)
        n, m, s = stats(vals)
        print("   -> N=%d  termination_time mean=%.2f  std=%.2f  min" % (n, m, s))
        summary.append((L, n, m, s))

    print("\n" + "=" * 76)
    print("SUMMARY — average planner termination time (last-motion, min)")
    print("=" * 76)
    print("  %-20s %4s  %10s  %8s" % ("planner", "N", "mean(min)", "std"))
    for L, n, m, s in summary:
        print("  %-20s %4d  %10.2f  %8.2f" % (L, n, m, s))


if __name__ == '__main__':
    main()
