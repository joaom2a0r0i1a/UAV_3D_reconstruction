#!/usr/bin/env python3
# Gain evaluation cost against tree size, from the timing logs
# Tags timing_marg, timing_abs, timing_cpu and timing_full
# Usage timing_analyze.py [log_dir] [out_dir], TIMING_LOG and TIMING_OUT otherwise
import os, re, sys, statistics as st
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# yapf: disable
MP  = os.environ.get("MP") or os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LOG = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("TIMING_LOG", ".")
OUT = sys.argv[2] if len(sys.argv) > 2 else os.environ.get("TIMING_OUT", LOG)
NS  = [50, 100, 500, 1000, 5000, 10000]
BUDGET, BUDGET2 = 500.0, 1000.0
# TIMING_TAG and TIMING_SUFFIX select the yaw optimization run
TAG = os.environ.get("TIMING_TAG", "timing_n")
SUF = os.environ.get("TIMING_SUFFIX", "")
# First 10 captures per N
MAX_CAPS = 10

def msd(xs): return (st.mean(xs), (st.stdev(xs) if len(xs) > 1 else 0.0), len(xs)) if xs else (None, None, 0)

PATS = {
  "mg":  re.compile(r"\[timing_marg\] nodes=(\d+) total_ms=([0-9.]+) gain_computation_ms=([0-9.]+) cpu_to_gpu_transfer_ms=([0-9.]+)"),
  "ab":  re.compile(r"\[timing_abs\] nodes=(\d+) total_ms=([0-9.]+) gain_computation_ms=([0-9.]+) cpu_to_gpu_transfer_ms=([0-9.]+)"),
  "cpu": re.compile(r"\[timing_cpu\] nodes=(\d+) cpu_absolute_ms=([0-9.]+) cpu_gain_all_ms=([0-9.]+)"),
  "full":re.compile(r"\[timing_full\] nodes=(\d+) tree_construction_ms=([0-9.]+) gain_evaluation_ms=([0-9.]+) scoring_ms=([0-9.]+) full_algorithm_ms=([0-9.]+) gain_computation_ms=([0-9.]+)"),
}
KEYS = ["mg_total","mg_comp","mg_xfer","ab_total","ab_comp","ab_xfer",
        "cpu_abs","cpu_all","f_tree","f_eval","f_score","f_full","f_comp"]
# yapf: enable


def parse(n):
    f = f"{LOG}/{TAG}{n}.log"
    thr = 0.7 * n
    acc = {k: [] for k in KEYS}
    if os.path.exists(f):
        for L in open(f):
            m = PATS["mg"].search(L)
            if m and int(m.group(1)) >= thr:
                acc["mg_total"].append(float(m.group(2)))
                acc["mg_comp"].append(float(m.group(3)))
                acc["mg_xfer"].append(float(m.group(4)))
            m = PATS["ab"].search(L)
            if m and int(m.group(1)) >= thr:
                acc["ab_total"].append(float(m.group(2)))
                acc["ab_comp"].append(float(m.group(3)))
                acc["ab_xfer"].append(float(m.group(4)))
            m = PATS["cpu"].search(L)
            if m and int(m.group(1)) >= thr:
                acc["cpu_abs"].append(float(m.group(2)))
                acc["cpu_all"].append(float(m.group(3)))
            m = PATS["full"].search(L)
            if m and int(m.group(1)) >= thr:
                acc["f_tree"].append(float(m.group(2)))
                acc["f_eval"].append(float(m.group(3)))
                acc["f_score"].append(float(m.group(4)))
                acc["f_full"].append(float(m.group(5)))
                acc["f_comp"].append(float(m.group(6)))
    return {k: msd(v[:MAX_CAPS]) for k, v in acc.items()}


# Stop when no log matches
if not any(os.path.exists(f"{LOG}/{TAG}{n}.log") for n in NS):
    print(f"no timing logs in {LOG}, expected " + ", ".join(f"{TAG}{n}.log" for n in NS))
    raise SystemExit(1)
os.makedirs(OUT, exist_ok=True)
rows = [(n, parse(n)) for n in NS]


def g(P, k):
    return P[k][0]


# Table, mean and sample std per N
METRICS = [
    ("cpu_all", "CPU G_all (cached)"),
    ("cpu_abs", "CPU absolute"),
    ("mg_total", "GPU G_all total"),
    ("mg_comp", "GPU G_all gain-compute"),
    ("mg_xfer", "GPU G_all transfer"),
    ("ab_total", "GPU abs total"),
    ("ab_comp", "GPU abs gain-compute"),
    ("ab_xfer", "GPU abs transfer"),
    ("f_tree", "tree construction"),
    ("f_eval", "gain evaluation"),
    ("f_score", "scoring"),
    ("f_full", "full algorithm"),
]
ncap = {n: P["mg_total"][2] for n, P in rows}
w = 17
lines = [
    "timing suite online timings: mean +/- sample-std (ms), over the whole-tree captures (planning iterations) per N",
    "captures/N: " + "   ".join(f"N={n}: {ncap[n]}" for n in NS), ""
]
head = f"{'metric':>22} | " + " ".join(f"{('N='+str(n)):>{w}}" for n in NS)
lines += [head, "-" * len(head)]
for key, lab in METRICS:
    cells = []
    for n, P in rows:
        m, s, k = P[key]
        cells.append(f"{(f'{m:.2f}±{s:.2f}' if m is not None else '-'):>{w}}")
    lines.append(f"{lab:>22} | " + " ".join(cells))
# Speedup row
sp = [
    f"{(g(P,'cpu_all')/g(P,'mg_total')):.0f}x" if (g(P, 'cpu_all') and g(P, 'mg_total')) else "-"
    for _, P in rows
]
lines += ["-" * len(head), f"{'CPU-G_all / GPU-G_all':>22} | " + " ".join(f"{v:>{w}}" for v in sp)]
tbl = "\n".join(lines)
print(tbl)
open(f"{OUT}/timing_table{SUF}.txt", "w").write(tbl + "\n")


def draw(fname, title, specs, logy=True, budget=False):
    fig, ax = plt.subplots(figsize=(8.6, 5.9))
    for key, color, mk, lab in specs:
        xs, ys, es = [], [], []
        for n, P in rows:
            m, s, _ = P[key]
            if m is not None:
                xs.append(n)
                ys.append(m)
                es.append(s or 0)
        if xs: ax.errorbar(xs, ys, yerr=es, fmt=mk + "-", color=color, lw=2, capsize=3, label=lab)
    if budget:
        ax.axhline(1000.0, color="0.35", ls="--", lw=1.2)
        ax.text(NS[-1],
                1000 * 1.12,
                "1 s real-time budget",
                ha="right",
                va="bottom",
                color="0.35",
                fontsize=8.5)
    ax.set_xscale("log")
    if logy: ax.set_yscale("log")
    ax.set_xlabel("Tree size $N$ (nodes)")
    ax.set_ylabel("Time per replan (ms)")
    ax.set_title(title)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(f"{OUT}/{fname}", dpi=130)
    plt.close(fig)


# Fig A, time against tree size (log y)
draw(f"timing_cost_vs_N{SUF}.png",
     "Gain evaluation time versus tree size", [
         ("cpu_all", "#c0392b", "o", r"$G_\mathrm{all}$ (CPU)"),
         ("cpu_abs", "#d4ac0d", "P", r"$G_\mathrm{absolute}$ (CPU)"),
         ("mg_total", "#2471a3", "s", r"$G_\mathrm{all}$ (GPU)"),
         ("mg_comp", "#27ae60", "^", r"$G_\mathrm{all}$ gain computation (GPU)"),
         ("mg_xfer", "#16a085", "D", r"$G_\mathrm{all}$ CPU-GPU transfer (GPU)"),
         ("ab_total", "#8e44ad", "*", r"$G_\mathrm{absolute}$ (GPU)"),
     ],
     budget=True)

# Fig A2, linear y axis
draw(f"timing_cost_vs_N_lineary{SUF}.png",
     "Gain evaluation time versus tree size (linear scale)", [
         ("cpu_all", "#c0392b", "o", r"$G_\mathrm{all}$ (CPU)"),
         ("cpu_abs", "#d4ac0d", "P", r"$G_\mathrm{absolute}$ (CPU)"),
         ("mg_total", "#2471a3", "s", r"$G_\mathrm{all}$ (GPU)"),
         ("ab_total", "#8e44ad", "*", r"$G_\mathrm{absolute}$ (GPU)"),
     ],
     logy=False,
     budget=True)


# Stacked bars, compute below transfer
def fmt_ms(v):
    if v < 1: return f"{v:.2f}"
    if v < 10: return f"{v:.1f}"
    return f"{v:.0f}"


def decomp(fname, title, comp_key, tot_key):
    xs, comp, xfer, tot = [], [], [], []
    for n, P in rows:
        c, t = g(P, comp_key), g(P, tot_key)
        if None not in (c, t) and t > 0:
            xs.append(n)
            comp.append(c)
            xfer.append(t - c)
            tot.append(t)
    if not xs: return
    fig, ax = plt.subplots(figsize=(9.2, 6.2))
    # yapf: disable
    ax.plot(xs, tot,  "-o", color="#154360", lw=2.3, ms=6, label="Total")
    ax.plot(xs, comp, "-^", color="#27ae60", lw=2.0, ms=6, label="Gain computation")
    ax.plot(xs, xfer, "-v", color="#e67e22", lw=2.0, ms=6, label="CPU-GPU transfer")
    # yapf: enable
    # Percentage split labels
    for x, c, xf, t in zip(xs, comp, xfer, tot):
        # yapf: disable
        ax.annotate(f"{100*c/t:.0f}%",  (x, t), textcoords="offset points", xytext=(0, 15), ha="center", fontsize=8, color="#1e8449", fontweight="bold")
        ax.annotate(f"{100*xf/t:.0f}%", (x, t), textcoords="offset points", xytext=(0, 5),  ha="center", fontsize=8, color="#b9770e", fontweight="bold")
        # yapf: enable
    ax.set_xscale("log")
    # Headroom for labels
    ax.set_ylim(top=max(tot) * 1.15)
    ax.set_xlabel("Tree size $N$ (nodes)")
    ax.set_ylabel("Time per replan (ms)")
    ax.set_title(title)
    ax.legend(fontsize=9, loc="upper left")
    ax.grid(True, which="both", alpha=0.3)
    ax.set_xticks(xs)
    ax.set_xticklabels([str(n) for n in xs])
    fig.tight_layout()
    fig.savefig(f"{OUT}/{fname}", dpi=130)
    plt.close(fig)


decomp(f"timing_marg_compute_vs_transfer{SUF}.png",
       r"All-ancestors ($G_\mathrm{all}$) GPU gain time: computation versus transfer", "mg_comp",
       "mg_total")
decomp(f"timing_abs_compute_vs_transfer{SUF}.png",
       r"$G_\mathrm{absolute}$ (GPU) gain time: computation versus transfer", "ab_comp", "ab_total")

# Fig C, planning cycle breakdown
draw(f"timing_full_algorithm{SUF}.png", "Planning-cycle time breakdown versus tree size", [
    ("f_tree", "#c0392b", "o", "Tree construction"),
    ("f_eval", "#2471a3", "s", "Gain evaluation (GPU)"),
    ("f_score", "#8e44ad", "D", "Scoring"),
    ("f_full", "#000000", "*", "Full planning cycle"),
])
print(
    f"\nwrote {OUT}/timing_table.txt + timing_cost_vs_N.png + timing_marg_compute_vs_transfer.png + timing_full_algorithm.png"
)
