#!/usr/bin/env python3
# Accuracy figures from the per-node CSVs
# DEPTH_N and DEPTH_REPLAN pick the trees of panel 2
# Usage accuracy_analyze.py [csv_dir] [out_dir], ACCURACY_LOG and ACCURACY_OUT otherwise
import csv, glob, os, sys, math, statistics as st
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# yapf: disable
MP  = os.environ.get("MP") or os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LOG = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ACCURACY_LOG", ".")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.environ.get("ACCURACY_OUT", LOG)
os.makedirs(OUT, exist_ok=True)
NS  = [50, 100, 500, 1000, 5000, 10000]
# all pools every tree size
DEPTH_N   = os.environ.get("DEPTH_N", "all")
# Max depth shown
DEPTH_MAX = int(os.environ.get("DEPTH_MAX", "25"))

# Shared plot labels
L_ALL_CPU = r"$g_\mathrm{all}$  (CPU, Reference)"
L_ALL_GPU = r"$g_\mathrm{all}$  (GPU, Ours)"
L_1P_CPU  = r"$g_\mathrm{sp}$  (CPU)"
L_1P_GPU  = r"$g_\mathrm{sp}$  (GPU)"
L_ABS     = r"$g_\mathrm{abs}$"
# yapf: enable


def fnum(x):
    try:
        return float(x)
    except:
        return None


rows = []
for n in NS:
    # Tree size glob
    hits = sorted(glob.glob(os.path.join(LOG, f"*_n{n}.csv")))
    f = hits[0] if hits else os.path.join(LOG, f"accuracy_n{n}.csv")
    if not os.path.exists(f): continue
    for r in csv.DictReader(open(f)):
        d = {k: fnum(v) for k, v in r.items()}
        if None in (d.get("all_cpu"), d.get("all_gpu")): continue
        d["N"] = n
        rows.append(d)
print(f"loaded {len(rows)} node-rows")
if not rows:
    print(f"no data in {LOG}, expected accuracy_n<N>.csv files with a header row")
    raise SystemExit(1)


def r2(xs, ys):
    mx, my = st.mean(xs), st.mean(ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    sxx = sum((x - mx)**2 for x in xs)
    syy = sum((y - my)**2 for y in ys)
    return (sxy * sxy) / (sxx * syy) if sxx > 0 and syy > 0 else float('nan')


# Panel 1, GPU against CPU
lines = [
    "Agreement with the reference implementation (all node rows)",
    f"{'gain':>10} {'n':>8} {'R^2':>8} {'slope':>7} {'RMSE':>8} {'bias':>9} {'mean|Δ|':>9}",
    "-" * 66
]
st_all = {}
for key, cc, gc, lab in [
    ("abs", "abs_cpu", "abs_gpu", "g_abs"),
    ("p1", "p1_cpu", "p1_gpu", "g_sp"),
    ("all", "all_cpu", "all_gpu", "g_all"),
]:
    xs = [r[cc] for r in rows if r.get(cc) is not None]
    ys = [r[gc] for r in rows if r.get(cc) is not None]
    n = len(xs)
    diff = [b - a for a, b in zip(xs, ys)]
    rmse = math.sqrt(sum(d * d for d in diff) / n)
    bias = sum(diff) / n
    mad = sum(abs(d) for d in diff) / n
    slope = sum(x * y for x, y in zip(xs, ys)) / sum(x * x for x in xs)
    st_all[key] = dict(n=n, R2=r2(xs, ys), rmse=rmse, bias=bias, slope=slope)
    lines.append(
        f"{lab:>10} {n:>8} {st_all[key]['R2']:>8.4f} {slope:>7.3f} {rmse:>8.4f} {bias:>+9.4f} {mad:>9.4f}"
    )
panel1 = "\n".join(lines)
print("\n" + panel1)
open(os.path.join(OUT, "gain_agreement.txt"), "w").write(panel1 + "\n")

fig, ax = plt.subplots(figsize=(5.8, 5.8))
xc = [r["all_cpu"] for r in rows]
yg = [r["all_gpu"] for r in rows]
ax.scatter(xc, yg, s=6, alpha=0.22, color="#2471a3", edgecolors="none")
lim = max(max(xc), max(yg)) * 1.05
ax.plot([0, lim], [0, lim], "k--", lw=1, label="$y = x$")
ax.set_xlim(0, lim)
ax.set_ylim(0, lim)
ax.set_xlabel(r"Reference path-dependent gain $g_\mathrm{all}$  [m$^3$]")
ax.set_ylabel(r"Path-dependent gain, this work  [m$^3$]")
ax.set_title("Agreement with the reference implementation")
s = st_all["all"]
box = (f"$n$ = {s['n']:,}\n$R^2$ = {s['R2']:.4f}\nRMSE = {s['rmse']:.3f}")
ax.text(0.04,
        0.96,
        box,
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.9))
ax.legend(loc="lower right")
ax.grid(True, ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "gain_agreement.png"), dpi=200)
plt.close(fig)

# Panel 2, gain and over-count by depth
sel = rows if DEPTH_N == "all" else [r for r in rows if r["N"] == int(DEPTH_N)]
tag = "all N" if DEPTH_N == "all" else f"N={DEPTH_N}"
by_d = {}
for r in sel:
    if None in (r.get("all_cpu"), r.get("all_gpu"), r.get("p1_cpu"), r.get("p1_gpu"),
                r.get("abs_cpu")):
        continue
    by_d.setdefault(int(r["depth"]), []).append(r)
depths = [
    d for d in sorted(by_d) if 1 <= d <= DEPTH_MAX and st.mean([x["all_cpu"] for x in by_d[d]]) > 0
]


def dmean(g, k):
    return st.mean([r[k] for r in g])


# Absolute as one line, CPU and GPU for the rest
S = {k: [] for k in ("all_cpu", "all_gpu", "p1_cpu", "p1_gpu", "abs")}
R = {k: [] for k in ("all_gpu", "p1_cpu", "p1_gpu", "abs")}
for d in depths:
    g = by_d[d]
    base = dmean(g, "all_cpu")
    S["all_cpu"].append(base)
    S["all_gpu"].append(dmean(g, "all_gpu"))
    S["p1_cpu"].append(dmean(g, "p1_cpu"))
    S["p1_gpu"].append(dmean(g, "p1_gpu"))
    S["abs"].append(st.mean([0.5 * (r["abs_cpu"] + r["abs_gpu"]) for r in g]))
    for k in R:
        R[k].append(S[k][-1] / base)

lines2 = [
    f"Mean information gain (m^3) per branch depth [{tag}].  Reference = g_all (CPU).",
    f"{'depth':>5} {'n':>6} | {'g_all_cpu':>9} {'g_all_gpu':>9} {'g_sp_cpu':>9} {'g_sp_gpu':>9} {'g_abs':>8}"
    f" | {'gpuAll':>7} {'sp_cpu':>7} {'sp_gpu':>7} {'abs':>7}", "-" * 100
]
for i, d in enumerate(depths):
    lines2.append(
        f"{d:>5} {len(by_d[d]):>6} | {S['all_cpu'][i]:>9.3f} {S['all_gpu'][i]:>9.3f} {S['p1_cpu'][i]:>9.3f} "
        f"{S['p1_gpu'][i]:>9.3f} {S['abs'][i]:>8.3f} | {R['all_gpu'][i]:>7.2f} {R['p1_cpu'][i]:>7.2f} "
        f"{R['p1_gpu'][i]:>7.2f} {R['abs'][i]:>7.2f}")
panel2 = "\n".join(lines2)
print("\n" + panel2)
open(os.path.join(OUT, "gain_overestimate.txt"), "w").write(panel2 + "\n")

# yapf: disable
styles = [("all_cpu","#000000","o",L_ALL_CPU),
          ("all_gpu","#2471a3","s",L_ALL_GPU),
          ("p1_cpu", "#e67e22","^",L_1P_CPU),
          ("p1_gpu", "#27ae60","v",L_1P_GPU),
          ("abs",    "#c0392b","D",L_ABS)]
# yapf: enable
fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
for k, c, m, lab in styles:
    ax[0].plot(depths, S[k], m + "-", color=c, lw=1.8, ms=4, label=lab)
ax[0].set_xlabel("Branch depth")
ax[0].set_ylabel(r"Mean information gain  [m$^3$]")
ax[0].set_title("Information gain versus branch depth")
ax[0].legend(fontsize=9)
ax[0].grid(True, ls=":", alpha=0.4)

ax[1].axhline(1.0, color="#000000", ls="--", lw=1.2)
for k, c, m, lab in styles[1:]:
    ax[1].plot(depths, R[k], m + "-", color=c, lw=1.8, ms=4, label=lab)
ax[1].set_xlabel("Branch depth")
ax[1].set_ylabel("Overestimate relative to reference")
ax[1].set_title("Overestimate relative to reference")
ax[1].legend(fontsize=9)
ax[1].grid(True, ls=":", alpha=0.4)
fig.tight_layout()
fig.savefig(os.path.join(OUT, "gain_overestimate.png"), dpi=200)
plt.close(fig)


# Each panel also saved alone
def _panel(kind):
    figp, axp = plt.subplots(figsize=(6.0, 4.8))
    if kind == "gain":
        for k, c, m, lab in styles:
            axp.plot(depths, S[k], m + "-", color=c, lw=1.8, ms=4, label=lab)
        axp.set_ylabel(r"Mean information gain  [m$^3$]")
        axp.set_title("Information gain versus branch depth")
        name = "gain_vs_depth.png"
    else:
        axp.axhline(1.0, color="#000000", ls="--", lw=1.2)
        for k, c, m, lab in styles[1:]:
            axp.plot(depths, R[k], m + "-", color=c, lw=1.8, ms=4, label=lab)
        axp.set_ylabel("Overestimate relative to reference")
        axp.set_title("Overestimate relative to reference")
        name = "overestimate_vs_depth.png"
    axp.set_xlabel("Branch depth")
    axp.legend(fontsize=9)
    axp.grid(True, ls=":", alpha=0.4)
    figp.tight_layout()
    figp.savefig(os.path.join(OUT, name), dpi=200)
    plt.close(figp)


_panel("gain")
_panel("ratio")

print(f"\nwrote in {OUT}:  gain_agreement.{{png,txt}}  gain_overestimate.{{png,txt}}"
      f"  gain_vs_depth.png  overestimate_vs_depth.png   [{tag}]")
