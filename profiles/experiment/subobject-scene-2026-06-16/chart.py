#!/usr/bin/env python3
"""Sub-object (multi-root) LBVH — cloth-on-sphere 씬 차트 (Phase 1 + 2a + 2b).

입력 CSV (모두 draped cloth-on-sphere 씬, single-root 물리로 settle 후 측정):
  scene_refit_bench.csv      : `--bench-bvh-subobject-scene`
       variant, particle1d, num_prims, split_s, num_groups, sub_nodes,
       settle_frames, refitswept_us_per_call, refitswept_ms_per_frame,
       combine_us_amortized
  query_cost_bench.csv       : `--bench-bvh-subobject-query` (현재 = Phase 2b)
       variant, particle1d, num_prims, split_s, num_groups, settle_frames,
       detect_us_per_call, broad_collisions

출력:
  combine_vs_s.png            : Phase 1 — refit combine µs vs s (single vs multi).
  query_cost_2a_vs_2b.png     : Phase 2a(O(k) 선형) vs 2b(mini-TLAS, k-무관) detect.

Phase 2a 의 detect 수치는 그 코드가 2b 로 대체되어 CSV 가 보존되지 않으므로,
당시 벤치 출력(P=100)을 아래 PHASE2A_P100 리터럴로 박아 before/after 를 그린다.

stdlib + matplotlib only.
"""

import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


# ---------------------------------------------------------------- Phase 1 ---
def chart_combine_vs_s():
    path = os.path.join(HERE, "scene_refit_bench.csv")
    sizes = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            p = int(row["particle1d"])
            d = sizes.setdefault(p, {"single": None, "multi": {}, "prims": 0})
            d["prims"] = int(row["num_prims"])
            us = float(row["combine_us_amortized"])
            if row["variant"] == "single":
                d["single"] = us
            else:
                d["multi"][int(row["split_s"])] = us

    plt.figure(figsize=(8, 5))
    for i, p in enumerate(sorted(sizes)):
        c = COLORS[i % len(COLORS)]
        d = sizes[p]
        ss = sorted(d["multi"])
        ys = [d["multi"][s] for s in ss]
        plt.plot(ss, ys, "-o", color=c, label=f"P={p} (~{d['prims']//1000}k prims)")
        if d["single"] is not None:
            plt.axhline(d["single"], color=c, ls=":", lw=1, alpha=0.6)
    plt.xlabel("split s   (dotted = single-root baseline)")
    plt.ylabel("refit combine µs / dispatch (sync-floor removed)")
    plt.title("Phase 1 — sub-object refit combine vs single (draped cloth-on-sphere)")
    plt.yscale("log")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    out = os.path.join(HERE, "combine_vs_s.png")
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    print("wrote", out)


# ----------------------------------------------------------- Phase 2a/2b ---
# Phase 2a (O(k) per-point root scan) P=100 detect µs/call, archived from the
# `--bench-bvh-subobject-query` run BEFORE the mini-TLAS landed. Keyed by
# num_groups k. single-root (k=1) = 1576µs.
PHASE2A_P100 = {
    1: 1576.22, 4: 1445.65, 16: 1825.28, 64: 2000.79, 225: 2383.48,
    625: 3084.75, 2500: 4541.95, 9801: 7700.0,  # s=7..16 plateau ~7.7ms
}


def chart_query_2a_vs_2b():
    path = os.path.join(HERE, "query_cost_bench.csv")  # = Phase 2b
    single = None
    p2b = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            if int(row["particle1d"]) != 100:
                continue
            us = float(row["detect_us_per_call"])
            if row["variant"] == "single":
                single = us
                p2b[1] = us
            else:
                p2b[int(row["num_groups"])] = us  # last wins (plateau k stable)

    a_k = sorted(PHASE2A_P100)
    a_y = [PHASE2A_P100[k] for k in a_k]
    b_k = sorted(p2b)
    b_y = [p2b[k] for k in b_k]

    plt.figure(figsize=(8, 5))
    plt.plot(a_k, a_y, "-o", color="#d62728", label="Phase 2a  (O(k) root scan)")
    plt.plot(b_k, b_y, "-o", color="#2ca02c", label="Phase 2b  (mini-TLAS, log k)")
    if single is not None:
        plt.axhline(single, color="#555", ls=":", lw=1,
                    label=f"single-root ({single:.0f}µs)")
    plt.xscale("log")
    plt.xlabel("num_groups  k  (split s ↑ → k ↑, saturates at full split)")
    plt.ylabel("broad detectCollisions µs / call")
    plt.title("Phase 2 query cost vs k  (P=100, ~20k prims, cloth-on-sphere)")
    plt.grid(True, which="both", alpha=0.3)
    plt.legend()
    out = os.path.join(HERE, "query_cost_2a_vs_2b.png")
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    print("wrote", out)


if __name__ == "__main__":
    chart_combine_vs_s()
    chart_query_2a_vs_2b()
