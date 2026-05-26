#!/usr/bin/env python3
"""Segmented vs Atomic broad-phase query 비교 차트 생성.

source.csv (= segquery_bench.csv 의 사본) 은 1 회 ysim 세션에서 8 case ×
30 프레임 = 240 행을 캡처한 것:
  - 케이스 1..4: Atomic    + n ∈ {20, 50, 100, 200}
  - 케이스 5..8: Segmented + n ∈ {20, 50, 100, 200}

각 케이스는 frame_index 가 0..29 로 다시 시작한다.

차트 2 개를 같은 디렉토리에 png 로 출력:
  - detect_time.png : broad_detect (broad-phase 충돌 쿼리 시간, ms)
  - collisions.png  : narrow_collisions per frame (충돌 sanity check)

stdlib + matplotlib only.
"""

import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "source.csv")

MODES = ["Atomic", "Segmented"]
N_VALUES = [20, 50, 100, 200]
FRAMES_PER_CASE = 30

# 각 (mode, n) 조합에 색을 부여: n 끼리 hue 그룹, mode 가 line style.
N_COLORS = {
    20:  "#1f77b4",
    50:  "#ff7f0e",
    100: "#2ca02c",
    200: "#d62728",
}
MODE_STYLES = {
    "Atomic":    {"linestyle": "-",  "marker": "o"},
    "Segmented": {"linestyle": "--", "marker": "s"},
}


def load_rows(path):
    if not os.path.exists(path):
        sys.exit(f"[chart.py] CSV not found: {path}")
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def bucket(rows):
    """Return dict[(mode, n)] -> list of rows ordered by frame_index."""
    out = {}
    for r in rows:
        n = int(r["particle_count"])
        # n was stored as N*N (vertices); recover N1D.
        n1d = int(round(n ** 0.5))
        key = (r["query_mode"], n1d)
        out.setdefault(key, []).append(r)
    for v in out.values():
        v.sort(key=lambda r: int(r["frame_index"]))
    return out


def draw(out_path, title, ylabel, series_getter, buckets):
    fig, ax = plt.subplots(figsize=(11, 6))
    xs = list(range(FRAMES_PER_CASE))
    for n1d in N_VALUES:
        color = N_COLORS[n1d]
        for mode in MODES:
            key = (mode, n1d)
            if key not in buckets:
                continue
            ys = [series_getter(r) for r in buckets[key]]
            style = MODE_STYLES[mode]
            ax.plot(xs, ys,
                    linestyle=style["linestyle"],
                    marker=style["marker"],
                    markersize=3.5,
                    linewidth=1.4,
                    color=color,
                    label=f"{mode} n={n1d}")
    ax.set_xlabel("Frame index within case (0..29)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=2, framealpha=0.9, fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def main():
    rows = load_rows(CSV_PATH)
    buckets = bucket(rows)

    draw(os.path.join(HERE, "detect_time.png"),
         "Broad-phase detect time per frame (Atomic vs Segmented × 4 resolutions)",
         "broad_detect (ms)",
         lambda r: float(r["broad_detect_ms"]),
         buckets)

    draw(os.path.join(HERE, "collisions.png"),
         "Narrow-phase collisions detected per frame (sanity check)",
         "narrow_collisions",
         lambda r: float(r["narrow_collisions"]),
         buckets)


if __name__ == "__main__":
    main()
