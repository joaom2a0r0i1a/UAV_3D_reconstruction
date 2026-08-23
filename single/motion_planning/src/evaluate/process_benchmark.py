#!/usr/bin/env python3
"""
Post-process the AEP gain benchmark CSVs written by AEPlanner.cpp (benchmarkGains)
when evaluation/benchmark:=true.  numpy + matplotlib only (no pandas/seaborn).

Figures are formatted for IEEE robotics papers (ICRA/IROS/RAL/TRO):
serif/Times typesetting, no in-figure titles (they go in the LaTeX caption),
grayscale-safe linestyle+marker per series, and axis scales chosen so the
scientific claim is legible rather than squashed:
  * timing/speedup use a log-scaled batch-size axis (geometric sweep),
  * timing additionally log-scales time so GPU scaling is not flattened by CPU,
  * the GPU-vs-CPU agreement is a log-density hexbin (data clusters near 0),
  * the gain plot is a box plot (shows the distribution, not a bare mean).

benchmark_timing.csv  (one row per planning batch):
  Phase,Nodes,v2_ms,v4_ms,fused_host_ms,split_host_ms,fused_dev_ms,split_dev_ms,
        abs_gpu_ms,abs_cpu_ms,cpu_hash_ms
benchmark_gains.csv   (one row per node):
  Phase,NodeIdx,Ancestors,g_fused,g_split,g_abs_gpu,g_abs_cpu,hash_cpu1p,v2_gpu1p,v4_multi

Usage:  python3 process_benchmark.py [benchmark_timing.csv] [benchmark_gains.csv]
"""

import csv, os, sys
import numpy as np
import matplotlib
matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from matplotlib.colors import LogNorm

# --------------------------------------------------------------------------- #
#  IEEE publication style (rcParams)                                          #
# --------------------------------------------------------------------------- #
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Computer Modern Roman", "DejaVu Serif"],
    "mathtext.fontset": "stix",          # serif-like math for R^2, etc.
    "axes.labelsize": 12,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "font.size": 11,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "figure.dpi": 200,
    "axes.linewidth": 0.9,
    "lines.linewidth": 1.6,
    "lines.markersize": 5,
})

TIMING_FILE = sys.argv[1] if len(sys.argv) > 1 else "benchmark_timing.csv"
GAINS_FILE  = sys.argv[2] if len(sys.argv) > 2 else "benchmark_gains.csv"

# Okabe-Ito colour-blind-safe palette.
C_BLUE   = "#0072B2"
C_GREEN  = "#009E73"
C_PURPLE = "#CC79A7"
C_ORANGE = "#E69F00"
C_VERM   = "#D55E00"
C_GREY   = "#666666"

# method -> (reader label, csv col, colour, linestyle, marker). Each series has a
# UNIQUE marker so the figure survives grayscale / B&W printing.
TIMING = [
    ("Batched GPU (fused)",       "fused_dev_ms", C_BLUE,   "-",  "o"),
    ("Batched GPU (split)",       "split_dev_ms", C_BLUE,   "--", "s"),
    ("GPU per-node, single-parent",    "v2_ms",        C_GREEN,  "-.", "^"),
    ("GPU per-node, multi-ancestor",  "v4_ms",        C_PURPLE, ":",  "v"),
    ("Absolute gain, GPU",        "abs_gpu_ms",   C_GREY,   "-",  "d"),
    ("Absolute gain, CPU",        "abs_cpu_ms",   C_ORANGE, "--", "D"),
    ("CPU per-node, single-parent",    "cpu_hash_ms",  C_VERM,   ":",  "x"),
]
# speedup over the CPU absolute-gain baseline (abs_cpu). >1 = faster than it.
SPEEDUP = [
    ("Batched GPU (fused)",       "fused_dev_ms", C_BLUE,   "-",  "o"),
    ("Batched GPU (split)",       "split_dev_ms", C_BLUE,   "--", "s"),
    ("GPU per-node, single-parent",    "v2_ms",        C_GREEN,  "-.", "^"),
    ("GPU per-node, multi-ancestor",  "v4_ms",        C_PURPLE, ":",  "v"),
    ("CPU per-node, single-parent",    "cpu_hash_ms",  C_VERM,   ":",  "x"),
]
# marginal-gain quality: distribution of each method's gain as a % of the
# absolute-CPU gain. fused/split/v4 share the IDENTICAL multi-ancestor value.
GAIN_METHODS = [
    ("Multi-ancestor\n(batched GPU)",   "g_split",    C_BLUE),
    ("Single-parent\n(CPU baseline)",   "hash_cpu1p", C_VERM),
    ("Single-parent\n(GPU)",            "v2_gpu1p",   C_GREEN),
]

BIN_EDGES = np.array([1, 2, 4, 8, 16, 24, 32, 48, 64, 96, 128, 192, 256, 320], dtype=float)


def load_csv(path):
    if not os.path.isfile(path):
        print(f"  [skip] {path} not found."); return None
    rows = list(csv.DictReader(open(path, newline="")))
    if not rows:
        print(f"  [skip] {path} is empty."); return None
    cols = {}
    for k in rows[0]:
        v = []
        for r in rows:
            try: v.append(float(r[k]))
            except (ValueError, TypeError): v.append(np.nan)
        cols[k] = np.array(v, dtype=float)
    print(f"  loaded {path}: {len(rows)} rows")
    return cols


def _binned_mean(N, t, edges=BIN_EDGES):
    """Bin the (N, t) points -> (bin_center, mean_t) with >=1 sample."""
    xs, ys = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = np.isfinite(N) & np.isfinite(t) & (t > 0) & (N >= lo) & (N < hi)
        if m.sum() < 1:
            continue
        xs.append(float(np.sqrt(lo * hi)))
        ys.append(float(np.mean(t[m])))
    return np.array(xs), np.array(ys)


FIGSIZE = (6.0, 4.0)     # larger fixed size: full axis labels + good definition


def _log_x(ax):
    """Log batch-size axis: explicit limits + ticks so the 300-node point is
    not clipped and the labels stay plain (not scientific notation)."""
    ax.set_xscale("log", base=10)
    ax.set_xlim(1, 320)
    ax.set_xticks([1, 10, 100, 300])
    ax.set_xticklabels(["1", "10", "100", "300"])
    ax.xaxis.set_minor_formatter(mticker.NullFormatter())


# --------------------------------------------------------------- timing ------
def timing_vs_batchsize(c):
    print("\n" + "=" * 64 + "\n[C] Runtime vs batch size (log-log, binned mean)\n" + "=" * 64)
    N = c.get("Nodes")
    if N is None:
        print("  [skip] no Nodes column."); return
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for lbl, col, cl, ls, mk in TIMING:
        if col not in c:
            continue
        x, y = _binned_mean(N, c[col])
        if len(x) < 2:
            continue
        ax.plot(x, y, ls=ls, marker=mk, color=cl, lw=1.6, ms=5, label=lbl)
        pernode = y[-1] / x[-1]
        print(f"  {lbl:26s}: {y[-1]:7.1f} ms @ ~{x[-1]:.0f} nodes  ({pernode*1000:6.1f} us/node)")
    _log_x(ax)
    ax.set_yscale("log")
    ax.set_xlabel("Batch size [candidate nodes]")
    ax.set_ylabel("Gain-evaluation time per batch [ms]")
    ax.grid(True, which="both", ls="--", alpha=0.3)
    ax.legend(loc="upper left", frameon=True, facecolor="white", framealpha=0.85,
              edgecolor="0.6", fontsize=9, handlelength=2.0, labelspacing=0.3,
              borderpad=0.4, ncol=1)
    fig.tight_layout()
    fig.savefig("timing_vs_batchsize.png")
    plt.close(fig)
    print("  saved timing_vs_batchsize.png")


def speedup_vs_batchsize(c):
    print("\n" + "=" * 64 + "\n[C2] Speedup over the CPU absolute-gain baseline\n" + "=" * 64)
    N = c.get("Nodes"); base = c.get("abs_cpu_ms")
    if N is None or base is None:
        print("  [skip] need Nodes + abs_cpu_ms."); return
    xb, yb = _binned_mean(N, base)
    if len(xb) < 2:
        print("  [skip] not enough batches."); return
    key = {float(np.round(x, 3)): b for x, b in zip(xb, yb)}
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for lbl, col, cl, ls, mk in SPEEDUP:
        if col not in c:
            continue
        x, y = _binned_mean(N, c[col])
        xi, sp = [], []
        for xx, yy in zip(x, y):
            b = key.get(float(np.round(xx, 3)))
            if b:
                xi.append(xx); sp.append(b / yy)
        if len(xi) < 2:
            continue
        xi, sp = np.array(xi), np.array(sp)
        ax.plot(xi, sp, ls=ls, marker=mk, color=cl, lw=1.6, ms=5, label=lbl)
        print(f"  {lbl:26s}: {sp.min():.2f}x -> {sp.max():.2f}x across batch sizes")
    ax.axhline(1, color="black", ls="--", lw=1.2,
               label="CPU absolute-gain baseline (1$\\times$)")
    _log_x(ax)
    ax.set_xlabel("Batch size [candidate nodes]")
    ax.set_ylabel("Speedup over CPU absolute-gain [$\\times$]")
    ax.grid(True, which="both", ls="--", alpha=0.3)
    ax.legend(loc="upper left", frameon=True, facecolor="white", framealpha=0.85,
              edgecolor="0.6", fontsize=9, handlelength=2.0, labelspacing=0.3,
              borderpad=0.4)
    fig.tight_layout()
    fig.savefig("speedup_vs_batchsize.png")
    plt.close(fig)
    print("  saved speedup_vs_batchsize.png")


# ---------------------------------------------------------------- gain -------
def gain_retained_bars(c):
    print("\n" + "=" * 64 + "\n[D] Gain kept as % of absolute-CPU gain\n" + "=" * 64)
    absc = c.get("g_abs_cpu")
    if absc is None:
        print("  [skip] no g_abs_cpu column."); return
    labels, heights, colours = [], [], []
    for lbl, col, cl in GAIN_METHODS:
        if col not in c:
            continue
        m = np.isfinite(absc) & np.isfinite(c[col]) & (absc > 0.5)
        if m.sum() == 0:
            continue
        ratio = np.clip(100.0 * c[col][m] / absc[m], 0, 150)
        labels.append(lbl); heights.append(float(np.mean(ratio))); colours.append(cl)
        print(f"  {lbl.replace(chr(10),' '):32s}: {np.mean(ratio):5.1f}%  (n={m.sum()})")
    if not labels:
        print("  [skip] no gain columns."); return
    hatches = ["///", "...", "xxx"]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for xi, h, cl, ht in zip(x, heights, colours, hatches):
        ax.bar(xi, h, width=0.5, color=cl, edgecolor="black", linewidth=0.9,
               hatch=ht, alpha=0.9)
        ax.text(xi, h + 1.5, f"{h:.0f}%", ha="center", va="bottom", fontsize=11)
    # dashed reference line at 100% == the absolute-gain value (explained by the
    # y-axis label, so no separate legend is needed).
    ax.axhline(100, color="black", ls="--", lw=1.4, zorder=5)
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=10)
    ax.set_ylim(0, 118)
    ax.set_ylabel("Gain retained [% of absolute-gain]")
    ax.grid(True, axis="y", ls="--", alpha=0.3)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig("gain_retained_bars.png")
    plt.close(fig)
    print("  saved gain_retained_bars.png")


def gain_gpu_vs_cpu(c):
    print("\n" + "=" * 64 + "\n[B] Agreement: GPU single-parent gain vs CPU baseline (hexbin)\n" + "=" * 64)
    if "v2_gpu1p" not in c or "hash_cpu1p" not in c:
        print("  [skip] missing v2_gpu1p / hash_cpu1p."); return
    gpu, cpu = c["v2_gpu1p"], c["hash_cpu1p"]
    m = np.isfinite(gpu) & np.isfinite(cpu); gpu, cpu = gpu[m], cpu[m]
    err = gpu - cpu
    ss_res = np.sum(err ** 2); ss_tot = np.sum((cpu - cpu.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    rmse = np.sqrt(np.mean(err ** 2)); bias = np.mean(err)
    print(f"  N={len(gpu)}  R2={r2:.4f}  RMSE={rmse:.3f}  bias={bias:+.3f}")

    fig, ax = plt.subplots(figsize=FIGSIZE)
    ax.set_facecolor("white")
    lim = [0, max(cpu.max(), gpu.max()) * 1.02]
    cmap = plt.get_cmap("viridis").copy()
    cmap.set_under("white")             # count == 0 stays white, not colormap floor
    hb = ax.hexbin(cpu, gpu, gridsize=100, bins="log", cmap=cmap,
                   mincnt=1, linewidths=0.0,
                   extent=(lim[0], lim[1], lim[0], lim[1]))
    cb = fig.colorbar(hb, ax=ax, pad=0.02)
    cb.set_label("point density [log$_{10}$ count]", fontsize=10)
    cb.ax.tick_params(labelsize=9)
    ax.plot(lim, lim, color="black", ls="--", lw=1.4, zorder=10,
            label="ideal ($y = x$)")
    ax.set_xlim(lim); ax.set_ylim(lim); ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("CPU single-parent gain (baseline)")
    ax.set_ylabel("GPU single-parent gain")
    ax.text(0.04, 0.96,
            f"$N$ = {len(gpu):,}\n$R^2$ = {r2:.3f}\nRMSE = {rmse:.2f}\nbias = {bias:+.2f}",
            transform=ax.transAxes, va="top", ha="left", fontsize=9,
            bbox=dict(boxstyle="round,pad=0.4", fc="white", ec="black",
                      lw=0.5, alpha=0.9), zorder=11)
    ax.legend(loc="lower right", frameon=True, facecolor="white", framealpha=0.85,
              edgecolor="0.6", fontsize=9, borderpad=0.4)
    ax.grid(True, ls="--", alpha=0.3)
    fig.tight_layout()
    fig.savefig("gain_gpu_vs_cpu.png")
    plt.close(fig)
    print("  saved gain_gpu_vs_cpu.png")


def main():
    print("Reading benchmark CSVs...")
    tdf = load_csv(TIMING_FILE)
    gdf = load_csv(GAINS_FILE)
    if tdf:
        timing_vs_batchsize(tdf)
        speedup_vs_batchsize(tdf)
    if gdf:
        gain_retained_bars(gdf)
        gain_gpu_vs_cpu(gdf)


if __name__ == "__main__":
    main()
