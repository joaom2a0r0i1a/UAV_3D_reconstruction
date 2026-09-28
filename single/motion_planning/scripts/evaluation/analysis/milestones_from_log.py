#!/usr/bin/env python3
# Milestone table from the eval_plotting_node output
# Usage: milestones_from_log.py <captured_node_stdout> [<out_table.txt>]
import re
import sys
from collections import OrderedDict

MILESTONES = [25, 50, 75, 95]
LINE_RE = re.compile(r"^(?P<series>.+?): Timing corresponding to Known voxels = "
                     r"(?P<known>\d+)% is time = (?P<t>[-\d.]+) \+/- (?P<s>[-\d.]+) minutes\.")
FINAL_RE = re.compile(r"^(?P<series>.+?): Final coverage = (?P<c>[-\d.]+) \+/- (?P<s>[-\d.]+)%\.")


def parse(path):
    # Last value per series and milestone
    ansi = re.compile(r"\x1b\[[0-9;]*m")
    data = OrderedDict()
    with open(path, errors="replace") as fh:
        for raw in fh:
            line = ansi.sub("", raw).rstrip("\n")
            f = FINAL_RE.match(line)
            if f:
                data.setdefault(f.group("series").strip(),
                                {})["final"] = (float(f.group("c")), float(f.group("s")))
                continue
            m = LINE_RE.match(line)
            if not m:
                continue
            series = m.group("series").strip()
            known = int(m.group("known"))
            data.setdefault(series, {})[known] = (float(m.group("t")), float(m.group("s")))
    return data


def fmt_cell(cell):
    if cell is None:
        return "    NA    "
    t, s = cell
    return f"{t:5.2f} ± {s:4.2f}"


def render(data):
    if not data:
        return "  (no milestone lines found in node output)\n"
    series = list(data.keys())
    lines = []
    header = "  % known │ " + " │ ".join(f"{s:^13}" for s in series)
    lines.append(header)
    lines.append("  ───────┼" + "┼".join("─" * 15 for _ in series))
    for k in MILESTONES:
        row = f"   {k:>3}%  │ " + " │ ".join(f"{fmt_cell(data[s].get(k)):^13}" for s in series)
        lines.append(row)
    # Final coverage in percent
    lines.append("  ───────┼" + "┼".join("─" * 15 for _ in series))
    lines.append("   final │ " + " │ ".join(f"{fmt_cell(data[s].get('final')):^13}"
                                            for s in series))
    # Pairwise delta for two series
    if len(series) == 2:
        a, b = series
        lines.append("")
        lines.append(f"  Δ (min, + => '{b}' faster):")
        for k in MILESTONES:
            ca, cb = data[a].get(k), data[b].get(k)
            if ca is None or cb is None:
                lines.append(f"   {k:>3}%  : NA")
            else:
                d = ca[0] - cb[0]
                pct = 100.0 * d / ca[0] if ca[0] else 0.0
                lines.append(f"   {k:>3}%  : {d:+.2f} ({pct:+.1f}%)")
    return "\n".join(lines) + "\n"


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: milestones_from_log.py <captured_node_stdout> [<out_table.txt>]")
    data = parse(sys.argv[1])
    table = "Exploration milestones (from eval_plotting_node's own computation)\n" + render(data)
    sys.stdout.write("\n" + table)
    if len(sys.argv) >= 3:
        with open(sys.argv[2], "w") as fh:
            fh.write(table)
        sys.stdout.write(f"  (saved -> {sys.argv[2]})\n")


if __name__ == "__main__":
    main()
