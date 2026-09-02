#!/usr/bin/env python3
# path_vel_mapped.py — path length AND average velocity per label, mapping bags to
# GOOD run dirs by nearest start-timestamp (same logic as path_length_mapped.py).
# avg velocity per run = ds / mission_duration (total path / bag duration). Read-only.
# Usage: MP=<motion_planning> OUT=<json> python3 path_vel_mapped.py <LABEL> [...]
import os, glob, math, sys, re, datetime, json
import rosbag

DATA  = os.path.join(os.environ['MP'], 'data')
OUT   = os.environ.get('OUT', '')

# PV_TOPIC override for real-world bags: PoseStamped shares .header.stamp + .pose.position with the sim UavState, so nothing else changes.
TOPIC = os.environ.get('PV_TOPIC', '/uav1/estimation_manager/uav_state')
DT      = 0.2
MIN_DUR = 120.0
MATCH_S = 180.0
# mavros glitch guard; without it the 2025 bags report a path of 3.3e12 m. No op in sim.
MAX_POS   = float(os.environ.get('PV_MAX_POSITION', 1000.0))
MAX_SPEED = float(os.environ.get('PV_MAX_SPEED', 20.0))
# glitched messages also carry wild header stamps, which would become the run duration.
MAX_DT    = float(os.environ.get('PV_MAX_DT', 5.0))

def sane(x, y, z):
    return (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
            and abs(x) <= MAX_POS and abs(y) <= MAX_POS and abs(z) <= MAX_POS)

def ds_length(path):
    ds=0.0; dx=dy=dz=None; tlk=None; t0=t1=None; n=0; skipped=0
    with rosbag.Bag(path) as bag:
        for _, msg, _ in bag.read_messages(topics=[TOPIC]):
            t=msg.header.stamp.to_sec()
            x,y,z=msg.pose.position.x,msg.pose.position.y,msg.pose.position.z
            if not sane(x,y,z):
                skipped+=1; continue
            if t1 is not None and not (0.0 <= t - t1 <= MAX_DT):
                skipped+=1; continue
            if t0 is None: t0=t
            t1=t; n+=1
            if tlk is None: dx,dy,dz,tlk=x,y,z,t
            elif t-tlk>=DT:
                step=math.sqrt((x-dx)**2+(y-dy)**2+(z-dz)**2)
                if step/(t-tlk) > MAX_SPEED:
                    skipped+=1; dx,dy,dz,tlk=x,y,z,t; continue
                ds+=step; dx,dy,dz,tlk=x,y,z,t
    return dict(n=n, dur=(t1-t0 if t0 is not None else 0.0), ds=ds, skipped=skipped)

def rundir_epoch(name): return datetime.datetime.strptime(name,'%Y%m%d_%H%M%S').timestamp()
def bag_epoch(fn):
    m=re.search(r'(\d{4})-(\d{2})-(\d{2})-(\d{2})-(\d{2})-(\d{2})',fn)
    return datetime.datetime(*map(int,m.groups())).timestamp() if m else None
def stats(v):
    n=len(v)
    if n==0: return (0,float('nan'),float('nan'))
    m=sum(v)/n; var=sum((x-m)**2 for x in v)/n
    return (n,m,math.sqrt(var))

result={}
for L in sys.argv[1:]:
    ld=os.path.join(DATA,L)
    runs=[os.path.basename(d) for d in sorted(glob.glob(os.path.join(ld,'2*'))) if os.path.isdir(d)]
    good=[r for r in runs if os.path.exists(os.path.join(ld,r,'voxblox_data.csv'))
          and sum(1 for _ in open(os.path.join(ld,r,'voxblox_data.csv')))>1]
    bags=sorted(glob.glob(os.path.join(ld,'tmp_bags','*.bag')))
    print("="*74); print(f"### {L}   good_runs={len(good)}  bags_in_pool={len(bags)}")
    used=set(); paths=[]; vels=[]
    for r in good:
        rt=rundir_epoch(r); best=None; bestd=1e18
        for b in bags:
            be=bag_epoch(os.path.basename(b))
            if be is None or b in used: continue
            d=abs(be-rt)
            if d<bestd: bestd=d; best=b
        if best is None or bestd>MATCH_S:
            print(f"   run {r}: NO bag within {MATCH_S:.0f}s (nearest {bestd:.0f}s) -> excluded"); continue
        used.add(best)
        try: res=ds_length(best)
        except Exception as e:
            print(f"   run {r}: bag ERROR {e}"); continue
        if res['dur']<MIN_DUR:
            print(f"   run {r}: dur {res['dur']:.0f}s <PARTIAL excluded>"); continue
        v=res['ds']/res['dur'] if res['dur']>0 else 0.0
        note = f"  (skipped {res['skipped']} glitched poses)" if res.get('skipped') else ""
        print(f"   run {r} <- {os.path.basename(best)}  dur={res['dur']:6.1f}s  ds={res['ds']:7.1f} m  v={v:.3f} m/s{note}")
        paths.append(res['ds']); vels.append(v)
    n,pm,ps=stats(paths); _,vm,vs=stats(vels)
    print(f"   -> N={n}  path mean={pm:.1f}±{ps:.1f} m   avg_vel mean={vm:.3f}±{vs:.3f} m/s")
    result[L]=dict(N=n, path_mean=pm, path_std=ps, vel_mean=vm, vel_std=vs)

if OUT:
    with open(OUT,'w') as f: json.dump(result,f,indent=2)
    print(f"\nwrote {OUT}")
