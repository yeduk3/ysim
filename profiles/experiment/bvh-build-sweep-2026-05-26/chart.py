#!/usr/bin/env python3
"""BVH 빌드 모드 3-way × particle-count 4 단계 스윕 차트.

build_bench.csv (--bench-bvh-build 출력) 를 읽어 per-(case, particle_count)
평균/표준편차를 계산하고 line / bar 차트 두 장을 출력한다. stdlib +
matplotlib only.
"""

import csv
import os
import statistics
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "build_bench.csv")
LINE_PNG = os.path.join(HERE, "build_chart_line.png")
BAR_PNG = os.path.join(HERE, "build_chart_bar.png")

CASE_ORDER = ["RefitON_Karras", "RebuildKarras", "RebuildApetrei"]
CASE_LABEL = {
    "RefitON_Karras": "1. Refit ON + Karras (rebuild/10f)",
    "RebuildKarras":  "2. Rebuild/substep + Karras",
    "RebuildApetrei": "3. Rebuild/substep + Apetrei",
}
CASE_COLOR = {
    "RefitON_Karras": "#1f77b4",
    "RebuildKarras":  "#ff7f0e",
    "RebuildApetrei": "#2ca02c",
}


def load(path):
    if not os.path.exists(path):
        sys.exit(f"[chart.py] CSV not found: {path}\n"
                 f"           Run `./build/src/ysim --bench-bvh-build` first.")
    bucket = {}
    with open(path, newline="") as fh:
        rd = csv.DictReader(fh)
        for r in rd:
            if int(r["frame_index"]) == 0:
                # warmup-adjacent first measured frame; drop for stability.
                continue
            try:
                bb = float(r["bvh_build_ms"])
                br = float(r["broad_refit_ms"])
            except (KeyError, ValueError):
                continue
            if bb < 0 and br < 0:
                continue
            t = max(bb, 0.0) + max(br, 0.0)
            key = (r["case"], int(r["particle_count"]))
            bucket.setdefault(key, []).append(t)
    return bucket


def stats(samples):
    if not samples:
        return float("nan"), 0.0
    m = statistics.fmean(samples)
    s = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    return m, s


def draw_line(bucket, out_path):
    sizes = sorted({pc for (_, pc) in bucket.keys()})
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for case in CASE_ORDER:
        xs, ys, yerr = [], [], []
        for pc in sizes:
            samples = bucket.get((case, pc))
            if not samples:
                continue
            m, s = stats(samples)
            xs.append(pc)
            ys.append(m)
            yerr.append(s)
        if xs:
            ax.errorbar(xs, ys, yerr=yerr, label=CASE_LABEL[case],
                        marker="o", capsize=3, color=CASE_COLOR[case])
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Particle count (cloth vertices, log)")
    ax.set_ylabel("BVH build + refit per frame (ms, log)\n"
                  "= bvh_build + broad_refit (60 substeps aggregated)")
    ax.set_title("BVH build modes vs cloth resolution (3-way, 30f x 60sub)")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def draw_bar(bucket, out_path):
    sizes = sorted({pc for (_, pc) in bucket.keys()})
    n_cases = len(CASE_ORDER)
    width = 0.8 / n_cases
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for idx, case in enumerate(CASE_ORDER):
        means, stdevs = [], []
        for pc in sizes:
            m, s = stats(bucket.get((case, pc), []))
            means.append(m)
            stdevs.append(s)
        xs = [i + (idx - (n_cases - 1) / 2.0) * width for i in range(len(sizes))]
        ax.bar(xs, means, width=width, yerr=stdevs, capsize=2,
               label=CASE_LABEL[case], color=CASE_COLOR[case])
    ax.set_xticks(range(len(sizes)))
    ax.set_xticklabels([f"{pc:,}" for pc in sizes])
    ax.set_xlabel("Particle count (cloth vertices)")
    ax.set_ylabel("BVH build + refit per frame (ms)")
    ax.set_title("BVH build modes per particle count (mean ± stddev)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def main():
    bucket = load(CSV_PATH)
    if not bucket:
        sys.exit(f"[chart.py] {CSV_PATH} has no usable rows.")
    draw_line(bucket, LINE_PNG)
    draw_bar(bucket, BAR_PNG)


if __name__ == "__main__":
    main()
