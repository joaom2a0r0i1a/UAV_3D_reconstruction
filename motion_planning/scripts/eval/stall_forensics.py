#!/usr/bin/env python3
# Per-window (default 30s) rosbag autopsy of one run: dist/v/spread/centroid from uav_state,
# to tell a planning stall from a vehicle stop from an in-place loop. Read-only, no master.
# Usage: stall_forensics.py <tmp_bag.bag> [window_s]
import os, sys, math
import rosbag

BAG = sys.argv[1]
WIN = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
STATE = '/uav1/estimation_manager/uav_state'
REF   = '/uav1/reference_out'

pts = []
refs = []
with rosbag.Bag(BAG) as bag:
    for topic, msg, _ in bag.read_messages(topics=[STATE, REF]):
        if topic == STATE:
            pts.append((msg.header.stamp.to_sec(),
                        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        else:
            refs.append(msg)

if not pts:
    print("no %s messages in bag" % STATE); sys.exit(1)
pts.sort(key=lambda p: p[0])
t0 = pts[0][0]
tN = pts[-1][0]
nwin = int(math.ceil((tN - t0) / WIN))
print("bag: %s" % os.path.basename(BAG))
print("duration %.1fs  |  %d state msgs  |  %d reference msgs  |  window=%.0fs" %
      (tN - t0, len(pts), len(refs), WIN))
print("%-8s %-8s %8s %7s %8s %10s" % ("t_start", "t_end", "dist_m", "v_m/s", "spread_m", "centroid"))

total = 0.0
for w in range(nwin):
    a = t0 + w * WIN
    b = a + WIN
    seg = [p for p in pts if a <= p[0] < b]
    if len(seg) < 2:
        print("%-8.0f %-8.0f %8s %7s %8s %10s" % (a - t0, b - t0, "-", "-", "-", "-"))
        continue
    d = 0.0
    xs = [seg[0][1]]; ys = [seg[0][2]]; zs = [seg[0][3]]
    for i in range(1, len(seg)):
        d += math.sqrt((seg[i][1]-seg[i-1][1])**2 +
                       (seg[i][2]-seg[i-1][2])**2 +
                       (seg[i][3]-seg[i-1][3])**2)
        xs.append(seg[i][1]); ys.append(seg[i][2]); zs.append(seg[i][3])
    total += d
    dur = seg[-1][0] - seg[0][0]
    v = d / dur if dur > 0 else 0.0
    spread = math.sqrt((max(xs)-min(xs))**2 + (max(ys)-min(ys))**2 + (max(zs)-min(zs))**2)
    cx = sum(xs)/len(xs); cy = sum(ys)/len(ys)
    print("%-8.0f %-8.0f %8.1f %7.3f %8.1f  (%5.1f,%5.1f)" %
          (a - t0, b - t0, d, v, spread, cx, cy))

print("-" * 56)
print("total path length: %.1f m over %.1fs  (avg %.3f m/s)" % (total, tN - t0, total/(tN-t0)))
