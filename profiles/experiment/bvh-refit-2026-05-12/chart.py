#!/usr/bin/env python3
"""Generate refit_chart_line.png + refit_chart_bar.png from refit_bench.csv.

Reads `refit_bench.csv` (CSV columns: method, particle_count, frame_index,
refit_time_ms) from the same directory as this script. Writes two PNGs
alongside it. Designed to be re-runnable after each `--bench-bvh-refit`
invocation; aggregates the 10 measured frames per (method, particle_count)
into mean + stddev.

stdlib + matplotlib only — no pandas dependency. Matches the project's
scripts/analyze_profile.py minimalism.
"""

import csv
import os
import statistics
import sys

import matplotlib

matplotlib.use("Agg")  # headless-safe
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "refit_bench.csv")
LINE_PNG = os.path.join(HERE, "refit_chart_line.png")
BAR_PNG = os.path.join(HERE, "refit_chart_bar.png")

METHOD_ORDER = ["FullCPU", "HybridD1", "HybridD2", "FullGPU"]
METHOD_COLORS = {
    "FullCPU":  "#1f77b4",
    "HybridD1": "#ff7f0e",
    "HybridD2": "#2ca02c",
    "FullGPU":  "#d62728",
}


def load(csv_path):
    """Return dict[(method, particle_count)] -> list[float ms]."""
    bucket = {}
    if not os.path.exists(csv_path):
        sys.exit(f"[chart.py] CSV not found: {csv_path}\n"
                 f"           Run `./build/src/ysim --bench-bvh-refit` first.")
    with open(csv_path, newline="") as fh:
        rd = csv.DictReader(fh)
        for row in rd:
            try:
                t = float(row["refit_time_ms"])
            except (KeyError, ValueError):
                continue
            if t < 0:
                continue  # bench wrote -1 for "section not found"
            key = (row["method"], int(row["particle_count"]))
            bucket.setdefault(key, []).append(t)
    return bucket


def stats(samples):
    if not samples:
        return float("nan"), 0.0
    mean = statistics.fmean(samples)
    stdev = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    return mean, stdev


def draw_line(bucket, out_path):
    sizes = sorted({pc for (_, pc) in bucket.keys()})
    fig, ax = plt.subplots(figsize=(8, 5))
    for method in METHOD_ORDER:
        xs, ys, yerr = [], [], []
        for pc in sizes:
            samples = bucket.get((method, pc))
            if not samples:
                continue
            m, s = stats(samples)
            xs.append(pc)
            ys.append(m)
            yerr.append(s)
        if xs:
            ax.errorbar(xs, ys, yerr=yerr, label=method,
                        marker="o", capsize=3,
                        color=METHOD_COLORS.get(method))
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Particle count (vertices, log scale)")
    ax.set_ylabel("Refit time per frame (ms, log scale)\n"
                  "= sum of 60 substep refits via broad_refit scope")
    ax.set_title("BVH refit time vs cloth particle count (D-031)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(title="Method")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def draw_bar(bucket, out_path):
    sizes = sorted({pc for (_, pc) in bucket.keys()})
    n_methods = len(METHOD_ORDER)
    width = 0.8 / n_methods
    fig, ax = plt.subplots(figsize=(9, 5))
    for idx, method in enumerate(METHOD_ORDER):
        means, stdevs = [], []
        for pc in sizes:
            samples = bucket.get((method, pc), [])
            m, s = stats(samples)
            means.append(m)
            stdevs.append(s)
        xs = [i + (idx - (n_methods - 1) / 2.0) * width for i in range(len(sizes))]
        ax.bar(xs, means, width=width, yerr=stdevs, capsize=2,
               label=method, color=METHOD_COLORS.get(method))
    ax.set_xticks(range(len(sizes)))
    ax.set_xticklabels([f"{pc:,}" for pc in sizes])
    ax.set_xlabel("Particle count (vertices)")
    ax.set_ylabel("Refit time per frame (ms)")
    ax.set_title("BVH refit time per method across cloth resolutions (D-031)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(title="Method")
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def main():
    bucket = load(CSV_PATH)
    if not bucket:
        sys.exit(f"[chart.py] CSV at {CSV_PATH} has no usable rows; "
                 f"did `--bench-bvh-refit` finish?")
    draw_line(bucket, LINE_PNG)
    draw_bar(bucket, BAR_PNG)


if __name__ == "__main__":
    main()
