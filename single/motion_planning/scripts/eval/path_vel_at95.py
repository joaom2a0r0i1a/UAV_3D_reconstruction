#!/usr/bin/env python3
# path_vel_at95.py — per-run PATH LENGTH and AVG VELOCITY truncated at the moment the run first reaches
# a coverage threshold (default 95% of the box volume, i.e. the "95% known" milestone).
# Answers: how far did the drone fly to reach 95%? If two horizons reach 95% at the same TIME but one flew a
# SHORTER path, that one explored more efficiently and the equal time is explained by extra planning compute.
#
# Per run: t95 = first RosTime where Volume >= FRAC*BOX (linear-interp between the two bracketing rows of
#   voxblox_data.csv). Map run->bag by nearest start-timestamp (same rule as path_vel_mapped.py). Accumulate
#   uav_state ds from the bag start until (t - t0) >= t95; vel@95 = ds / t95. Aggregate mean+/-std per label.
# Read-only. Usage: MP=<motion_planning> [BOX=4000] [FRAC=0.95] path_vel_at95.py <label> [<label> ...]
import os, glob, math, re, datetime, sys
import rosbag

DATA = os.path.join(os.environ['MP'], 'data')
BOX  = float(os.environ.get('BOX', '4000'))
FRAC = float(os.environ.get('FRAC', '0.95'))
TOPIC = '/uav1/estimation_manager/uav_state'
DT = 0.2
MATCH_S = 180.0


def t95_of(csv):
    thr = FRAC * BOX
    with open(csv) as f:
        rows = [r.strip().split(',') for r in f if r.strip()]
    data = [(float(r[1]), float(r[9])) for r in rows[2:]
            if len(r) >= 10 and r[1].replace('.', '', 1).isdigit()]
    for i, (t, v) in enumerate(data):
        if v >= thr:
            if i == 0:
                return t
            t0, v0 = data[i - 1]
            return t if v == v0 else t0 + (thr - v0) * (t - t0) / (v - v0)
    return None


def ds_until(path, t95):
    ds = 0.0; dx = dy = dz = tlk = t0 = None
    with rosbag.Bag(path) as bag:
        for _, msg, _ in bag.read_messages(topics=[TOPIC]):
            t = msg.header.stamp.to_sec()
            x, y, z = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
            if t0 is None:
                t0 = t
            if (t - t0) > t95:
                break
            if tlk is None:
                dx, dy, dz, tlk = x, y, z, t
            elif t - tlk >= DT:
                ds += math.sqrt((x - dx) ** 2 + (y - dy) ** 2 + (z - dz) ** 2)
                dx, dy, dz, tlk = x, y, z, t
    return ds


def rdep(n): return datetime.datetime.strptime(n, '%Y%m%d_%H%M%S').timestamp()
def bdep(fn):
    m = re.search(r'(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})', fn)
    return datetime.datetime(*map(int, m.groups())).timestamp() if m else None
def stats(v):
    n = len(v)
    if n == 0:
        return (0, float('nan'), float('nan'))
    m = sum(v) / n
    return (n, m, math.sqrt(sum((x - m) ** 2 for x in v) / n))


for L in sys.argv[1:]:
    ld = os.path.join(DATA, L)
    runs = sorted(d for d in glob.glob(ld + '/2026*') if os.path.isdir(d))
    bags = sorted(glob.glob(ld + '/tmp_bags/*.bag'))
    used = set(); paths = []; vels = []; t95s = []
    print('=' * 74); print(f'### {L}   ({FRAC * 100:.0f}% of {BOX:.0f} m^3 box)')
    for r in runs:
        t95 = t95_of(os.path.join(r, 'voxblox_data.csv'))
        if t95 is None:
            print(f'  {os.path.basename(r)}: never reached {FRAC * 100:.0f}%'); continue
        rt = rdep(os.path.basename(r)); best = None; bd = 1e18
        for b in bags:
            be = bdep(os.path.basename(b))
            if be is None or b in used:
                continue
            d = abs(be - rt)
            if d < bd:
                bd = d; best = b
        if best is None or bd > MATCH_S:
            print(f'  {os.path.basename(r)}: t95={t95/60:.2f}min  NO bag -> excluded'); continue
        used.add(best)
        ds = ds_until(best, t95); v = ds / t95 if t95 > 0 else 0.0
        paths.append(ds); vels.append(v); t95s.append(t95 / 60.0)
        print(f'  {os.path.basename(r)}: t95={t95/60:6.2f}min  path@95={ds:7.1f}m  vel@95={v:.3f}m/s')
    n, pm, ps = stats(paths); _, vm, vs = stats(vels); _, tm, ts = stats(t95s)
    print(f'  -> N={n}  t95={tm:.2f}+/-{ts:.2f}min  path@95={pm:.1f}+/-{ps:.1f}m  vel@95={vm:.3f}+/-{vs:.3f}m/s')
