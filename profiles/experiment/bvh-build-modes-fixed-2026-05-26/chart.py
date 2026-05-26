#!/usr/bin/env python3
"""BVH 빌드 모드 3-way 비교 차트 생성.

source.csv 는 1회 ysim 세션에서 90 프레임을 캡처한 것:
  - 프레임 0-29   : 케이스 1 (refit enabled, 10프레임마다 rebuild)
  - 프레임 30-59  : 케이스 2 (refit disabled + Karras 2012 build)
  - 프레임 60-89  : 케이스 3 (refit disabled + Apetrei 2014 build)

각 케이스는 frame_sequence 가 0 부터 다시 시작하므로 행 인덱스로 분리한다.

차트 두 개를 같은 디렉토리에 png로 출력:
  - build_refit.png  : bvh_build + broad_refit 합 (프레임당 BVH 갱신 총비용)
  - query.png        : broad_detect (broad-phase 충돌 쿼리)

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

CASES = [
    {"label": "1. Refit ON (rebuild every 10f)", "color": "#1f77b4"},
    {"label": "2. Refit OFF + Karras 2012",      "color": "#ff7f0e"},
    {"label": "3. Refit OFF + Apetrei 2014",     "color": "#2ca02c"},
]
FRAMES_PER_CASE = 30


def load_rows(path):
    if not os.path.exists(path):
        sys.exit(f"[chart.py] CSV not found: {path}")
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def split_by_case(rows):
    if len(rows) < FRAMES_PER_CASE * len(CASES):
        sys.exit(f"[chart.py] expected >= {FRAMES_PER_CASE * len(CASES)} rows, got {len(rows)}")
    chunks = []
    for i in range(len(CASES)):
        start = i * FRAMES_PER_CASE
        end = start + FRAMES_PER_CASE
        chunks.append(rows[start:end])
    return chunks


def col(rows, name):
    return [float(r[name]) for r in rows]


def draw(out_path, title, ylabel, series_per_case):
    """series_per_case: list[list[float]] aligned to CASES order."""
    fig, ax = plt.subplots(figsize=(10, 5))
    xs = list(range(FRAMES_PER_CASE))
    for case, ys in zip(CASES, series_per_case):
        ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.4,
                label=case["label"], color=case["color"])
    ax.set_xlabel("Frame index within case (0..29)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out_path}")


def main():
    rows = load_rows(CSV_PATH)
    chunks = split_by_case(rows)

    # Chart 1: per-frame BVH 갱신 비용 (build + refit 합).
    # 케이스 2,3에서는 SCENE::refit() 내부 fallback 으로 build 가 실행되지만
    # 프로파일러 스코프 이름이 "broad_refit" 이라 같은 컬럼에 누적된다.
    build_refit = []
    for ch in chunks:
        bb = col(ch, "bvh_build")
        br = col(ch, "broad_refit")
        build_refit.append([a + b for a, b in zip(bb, br)])
    draw(os.path.join(HERE, "build_refit.png"),
         "BVH build + refit time per frame (3-way A/B/C)",
         "Time (ms)  = bvh_build + broad_refit",
         build_refit)

    # Chart 2: per-frame broad-phase 쿼리 시간.
    detect = [col(ch, "broad_detect") for ch in chunks]
    draw(os.path.join(HERE, "query.png"),
         "BVH broad-phase query time per frame (3-way A/B/C)",
         "Time (ms)  = broad_detect",
         detect)


if __name__ == "__main__":
    main()
