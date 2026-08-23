#!/usr/bin/env python3
# Table the 25/50/75/95% milestones from a multi_series eval by parsing eval_plotting_node's
# own stdout ("...Timing corresponding to Known voxels = P% is time = T +/- S minutes.") — no
# math of its own, so the table can never disagree with MultiSeriesOverview.png.
# Usage: milestones_from_log.py <captured_node_stdout> [<out_table.txt>]
import re
import sys
from collections import OrderedDict

MILESTONES = [25, 50, 75, 95]  # % known
LINE_RE = re.compile(
    r"^(?P<series>.+?): Timing corresponding to Known voxels = "
    r"(?P<known>\d+)% is time = (?P<t>[-\d.]+) \+/- (?P<s>[-\d.]+) minutes\."
)


def parse(path):
    # strip ANSI, keep the last value seen per (series, known%) in case a log
    # contains more than one eval run appended together.
    ansi = re.compile(r"\x1b\[[0-9;]*m")
    data = OrderedDict()  # series -> {known%: (t, s)}
    with open(path, errors="replace") as fh:
        for raw in fh:
            m = LINE_RE.match(ansi.sub("", raw).rstrip("\n"))
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
    # pairwise delta only when exactly two series (positive => second reaches it sooner)
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
