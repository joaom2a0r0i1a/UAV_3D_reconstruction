#!/usr/bin/env python3
# X1a + X1b from the live per-node CSVs x1_n<N>.csv
#   cols: replan,depth,abs_cpu,abs_gpu,p1_cpu,p1_gpu,all_cpu,all_gpu   (all_cpu = G_true)
# X1a: GPU == CPU per definition (n, R^2, slope, RMSE, bias, mean|Δ|).  All node-rows.
# X1b: mean predicted gain & over-count vs NODE DEPTH.  Baseline = CPU G_all (= G_true).
#      Uses the deep trees (default N=10000) so the over-count contrast is visible; set
#      X1B_N and (optionally) X1B_REPLAN to pick the tree(s). Axes unchanged (depth vs gain/ratio).
import csv, os, math, statistics as st
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = os.environ.get("X1_LOG", "/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone/variants_logs")
OUT = "/home/lt-l4/ros1_motion_ws/src/UAV_3D_reconstruction/motion_planning/data"
NS  = [50, 100, 500, 1000, 5000, 10000]
X1B_N   = os.environ.get("X1B_N", "all")     # "all" = pool every tree size (clean slope separation); or a single N
X1B_DMAX = int(os.environ.get("X1B_DMAX", "25"))   # max depth shown

# journal-style labels shared with the X2 figures
L_ALL_CPU = r"$G_\mathrm{all}$ (CPU, $=G_\mathrm{true}$)"
L_ALL_GPU = r"$G_\mathrm{all}$ (GPU)"
L_1P_CPU  = r"$G_\mathrm{single\ parent}$ (CPU)"
L_1P_GPU  = r"$G_\mathrm{single\ parent}$ (GPU)"
L_ABS     = r"$G_\mathrm{absolute}$"

def fnum(x):
    try: return float(x)
    except: return None

rows = []
for n in NS:
    f = os.path.join(LOG, f"x1_n{n}.csv")
    if not os.path.exists(f): continue
    for r in csv.DictReader(open(f)):
        d = {k: fnum(v) for k, v in r.items()}
        if None in (d.get("all_cpu"), d.get("all_gpu")): continue
        d["N"] = n; rows.append(d)
print(f"loaded {len(rows)} node-rows")
if not rows: print("no data"); raise SystemExit

def r2(xs, ys):
    mx, my = st.mean(xs), st.mean(ys)
    sxy = sum((x-mx)*(y-my) for x, y in zip(xs, ys))
    sxx = sum((x-mx)**2 for x in xs); syy = sum((y-my)**2 for y in ys)
    return (sxy*sxy)/(sxx*syy) if sxx > 0 and syy > 0 else float('nan')

# ---------------- X1a: GPU vs CPU equality, with n, RMSE, bias ----------------
lines = ["X1a: GPU vs CPU gain equality (all node-rows)",
         f"{'gain':>10} {'n':>8} {'R^2':>8} {'slope':>7} {'RMSE':>8} {'bias':>9} {'mean|Δ|':>9}", "-"*66]
st_all = {}
for key, cc, gc, lab in [("abs","abs_cpu","abs_gpu","G_absolute"),
                         ("p1","p1_cpu","p1_gpu","G_1parent"),
                         ("all","all_cpu","all_gpu","G_all")]:
    xs = [r[cc] for r in rows if r.get(cc) is not None]
    ys = [r[gc] for r in rows if r.get(cc) is not None]
    n = len(xs); diff = [b-a for a, b in zip(xs, ys)]
    rmse = math.sqrt(sum(d*d for d in diff)/n); bias = sum(diff)/n; mad = sum(abs(d) for d in diff)/n
    slope = sum(x*y for x,y in zip(xs,ys))/sum(x*x for x in xs)
    st_all[key] = dict(n=n, R2=r2(xs,ys), rmse=rmse, bias=bias, slope=slope)
    lines.append(f"{lab:>10} {n:>8} {st_all[key]['R2']:>8.4f} {slope:>7.3f} {rmse:>8.4f} {bias:>+9.4f} {mad:>9.4f}")
x1a = "\n".join(lines); print("\n"+x1a); open(os.path.join(OUT,"x1a_correctness.txt"),"w").write(x1a+"\n")

fig, ax = plt.subplots(figsize=(5.8,5.8))
xc = [r["all_cpu"] for r in rows]; yg = [r["all_gpu"] for r in rows]
ax.scatter(xc, yg, s=6, alpha=0.22, color="#2471a3", edgecolors="none")
lim = max(max(xc), max(yg)) * 1.05
ax.plot([0,lim],[0,lim], "k--", lw=1, label="$y = x$")
ax.set_xlim(0,lim); ax.set_ylim(0,lim)
ax.set_xlabel(r"$G_\mathrm{all}$ CPU ($=G_\mathrm{true}$)  [m$^3$]")
ax.set_ylabel(r"$G_\mathrm{all}$ GPU  [m$^3$]")
ax.set_title("X1a: GPU vs CPU marginal gain")
s = st_all["all"]
box = (f"$n$ = {s['n']:,}\n$R^2$ = {s['R2']:.4f}\nRMSE = {s['rmse']:.3f}")
ax.text(0.04, 0.96, box, transform=ax.transAxes, va="top", ha="left", fontsize=10,
        bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.9))
ax.legend(loc="lower right"); ax.grid(True, ls=":", alpha=0.4); fig.tight_layout()
fig.savefig(os.path.join(OUT,"x1a_correctness.png"), dpi=140); plt.close(fig)

# ---------------- X1b: predicted gain / over-count vs depth ----------------
# Pool over all tree sizes (default): small-N shallow + large-N deep trees average to a clean
# per-depth curve where G_all (CPU=GPU) stays low and G_1parent/G_absolute peel away upward.
sel = rows if X1B_N == "all" else [r for r in rows if r["N"] == int(X1B_N)]
tag = "all N" if X1B_N == "all" else f"N={X1B_N}"
by_d = {}
for r in sel:
    if None in (r.get("all_cpu"), r.get("all_gpu"), r.get("p1_cpu"), r.get("p1_gpu"), r.get("abs_cpu")): continue
    by_d.setdefault(int(r["depth"]), []).append(r)
depths = [d for d in sorted(by_d) if 1 <= d <= X1B_DMAX and st.mean([x["all_cpu"] for x in by_d[d]]) > 0]

def dmean(g, k): return st.mean([r[k] for r in g])
# G_absolute shown as one line (CPU==GPU per X1a); G_all and G_single-parent keep CPU + GPU.
S = {k: [] for k in ("all_cpu","all_gpu","p1_cpu","p1_gpu","abs")}
R = {k: [] for k in ("all_gpu","p1_cpu","p1_gpu","abs")}
for d in depths:
    g = by_d[d]; base = dmean(g, "all_cpu")
    S["all_cpu"].append(base); S["all_gpu"].append(dmean(g,"all_gpu"))
    S["p1_cpu"].append(dmean(g,"p1_cpu")); S["p1_gpu"].append(dmean(g,"p1_gpu"))
    S["abs"].append(st.mean([0.5*(r["abs_cpu"]+r["abs_gpu"]) for r in g]))
    for k in R: R[k].append(S[k][-1]/base)

lines2 = [f"X1b: mean predicted gain (m^3) per depth [{tag}].  Baseline = CPU G_all (= G_true).",
          f"{'depth':>5} {'n':>6} | {'G_all_cpu':>9} {'G_all_gpu':>9} {'G_sp_cpu':>9} {'G_sp_gpu':>9} {'G_abs':>8}"
          f" | {'gpuAll':>7} {'sp_cpu':>7} {'sp_gpu':>7} {'abs':>7}", "-"*100]
for i, d in enumerate(depths):
    lines2.append(f"{d:>5} {len(by_d[d]):>6} | {S['all_cpu'][i]:>9.3f} {S['all_gpu'][i]:>9.3f} {S['p1_cpu'][i]:>9.3f} "
                  f"{S['p1_gpu'][i]:>9.3f} {S['abs'][i]:>8.3f} | {R['all_gpu'][i]:>7.2f} {R['p1_cpu'][i]:>7.2f} "
                  f"{R['p1_gpu'][i]:>7.2f} {R['abs'][i]:>7.2f}")
x1b = "\n".join(lines2); print("\n"+x1b); open(os.path.join(OUT,"x1b_overcount.txt"),"w").write(x1b+"\n")

styles = [("all_cpu","#000000","o",L_ALL_CPU),
          ("all_gpu","#2471a3","s",L_ALL_GPU),
          ("p1_cpu", "#e67e22","^",L_1P_CPU),
          ("p1_gpu", "#27ae60","v",L_1P_GPU),
          ("abs",    "#c0392b","D",L_ABS)]
fig, ax = plt.subplots(1, 2, figsize=(12,4.8))
for k, c, m, lab in styles:
    ax[0].plot(depths, S[k], m+"-", color=c, lw=1.8, ms=4, label=lab)
ax[0].set_xlabel("Node depth"); ax[0].set_ylabel(r"Mean predicted gain (m$^3$)")
ax[0].set_title("X1b: predicted gain vs depth"); ax[0].legend(fontsize=9); ax[0].grid(True, ls=":", alpha=0.4)

ax[1].axhline(1.0, color="#000000", ls="--", lw=1.2)
for k, c, m, lab in styles[1:]:
    ax[1].plot(depths, R[k], m+"-", color=c, lw=1.8, ms=4, label=lab)
ax[1].set_xlabel("Node depth"); ax[1].set_ylabel(r"Over-count factor ($G / G_\mathrm{true}$)")
ax[1].set_title("X1b: over-count vs true gain"); ax[1].legend(fontsize=9); ax[1].grid(True, ls=":", alpha=0.4)
fig.tight_layout(); fig.savefig(os.path.join(OUT,"x1b_overcount.png"), dpi=140); plt.close(fig)

print(f"\nwrote data/x1a_correctness.{{png,txt}} + data/x1b_overcount.{{png,txt}}  [X1b: {tag}]")
