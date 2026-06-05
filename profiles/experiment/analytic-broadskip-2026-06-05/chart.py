#!/usr/bin/env python3
"""Analytic broad-skip (c-2) vs BVH 비교 차트.

이전 c-1 실험(`analytic-collision-2026-06-04`)과 **동일한 방법**(같은 bench
`build/src/ysim --bench-analytic`, 18 case × 30 frame = 540 행, 같은 차트 3종)
으로 측정하되, 대상 코드가 c-2 (broad-skip + marker-driven narrow):
  - analytic ∈ {Off, On}
  - cloth_n  (a) ∈ {20, 50, 100}   FastGridCloth 분할수
  - sphere_n (b) ∈ {20, 50, 100}   UV-sphere 분할수

씬: 1x1 XZ-plane FastGridCloth(a) 가 직경 1 구(b, Rigid static) 위로 떨어진다.

c-2 변경점: broad 단계에서 (cloth, sphere) object-AABB 가 겹치면 구의 triangle
BVH 로 descend 하지 않고 (objShape==Sphere) 마커만 남긴다. 따라서 analytic ON
일 때 broad_detect/broad_collisions 가 0 으로 떨어지고, narrow 는 마커가 가리키는
한 개의 ellipsoid 만 정점마다 테스트한다 (sphere 분할수 b 와 무관). tri+analytic
은 같은 command buffer 로 묶여 commitAndWait 1 회.

출력 (c-1 과 동일한 3종 + before/after 1종):
  - headline_bars.png   : 9 (a,b) 별 broad Off/On + narrow Off/On (충돌 프레임 평균)
  - time_breakdown.png  : 3x3 그리드, Off/On stacked (broad + narrow + 그 외)
  - narrow_timeseries.png: 3x3 그리드, 프레임별 narrow_phase Off vs On
  - c1_vs_c2.png        : c-1 On 대비 c-2 On physics_total (before/after)
  - summary.md          : 표 형태 요약 (README 에서 인용)

numpy + pandas + matplotlib.
"""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "analytic_bench.csv")               # c-2
BASELINE_PATH = os.path.join(HERE, "analytic_bench_c1_baseline.csv")  # c-1

A_VALUES = [20, 50, 100]   # cloth_n
B_VALUES = [20, 50, 100]   # sphere_n
MODES = ["Off", "On"]

MODE_COLOR = {"Off": "#1f77b4", "On": "#d62728"}
# Stacked-bar component colors.
C_BROAD  = "#4c72b0"   # broad_detect (BVH)
C_NARROW = "#dd8452"   # narrow_phase
C_OTHER  = "#cccccc"   # physics_total - broad - narrow (integrate/sync/etc.)


def load(path):
    if not os.path.exists(path):
        raise SystemExit(f"[chart.py] CSV not found: {path}")
    df = pd.read_csv(path)
    # -1.0 sentinels (missing section that frame) → NaN so they don't skew means.
    for col in ["broad_detect_ms", "narrow_phase_ms", "physics_total_ms"]:
        df.loc[df[col] < 0, col] = np.nan
    return df


def case(df, mode, a, b):
    return df[(df.analytic == mode) & (df.cloth_n == a) & (df.sphere_n == b)]


def collision_frames(sub):
    """Rows where contacts actually fired (narrow_collisions > 0)."""
    hit = sub[sub.narrow_collisions > 0]
    return hit if len(hit) else sub


# ---------------------------------------------------------------- summary table
def build_summary(df):
    rows = []
    for a in A_VALUES:
        for b in B_VALUES:
            rec = {"cloth_n": a, "sphere_n": b}
            for mode in MODES:
                cf = collision_frames(case(df, mode, a, b))
                rec[f"broad_{mode}"]   = cf.broad_detect_ms.mean()
                rec[f"narrow_{mode}"]  = cf.narrow_phase_ms.mean()
                rec[f"phys_{mode}"]    = cf.physics_total_ms.mean()
                rec[f"ncol_{mode}"]    = cf.narrow_collisions.mean()
                rec[f"bcol_{mode}"]    = cf.broad_collisions.mean()
            rows.append(rec)
    return pd.DataFrame(rows)


def write_summary_md(s, base):
    path = os.path.join(HERE, "summary.md")
    lines = []
    lines.append("# Analytic broad-skip (c-2) — 충돌 프레임(narrow>0) 평균\n")
    lines.append("단위 ms. broad = broad_detect(BVH), narrow = narrow_phase, "
                 "phys = physics_total. ncol = narrow_collisions.\n")
    lines.append("| a | b | broad Off | broad On | narrow Off | narrow On | "
                 "phys Off | phys On | bcol On | ncol Off | ncol On |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for _, r in s.iterrows():
        lines.append(
            f"| {int(r.cloth_n)} | {int(r.sphere_n)} | "
            f"{r.broad_Off:.1f} | {r.broad_On:.1f} | "
            f"{r.narrow_Off:.1f} | {r.narrow_On:.1f} | "
            f"{r.phys_Off:.1f} | {r.phys_On:.1f} | "
            f"{r.bcol_On:.0f} | {r.ncol_Off:.0f} | {r.ncol_On:.0f} |")
    # Aggregate headline numbers.
    broad_off_share = (s.broad_Off / s.phys_Off * 100).mean()
    lines.append("")
    lines.append(f"- broad_detect(BVH) On 평균 **{s.broad_On.mean():.2f} ms**, "
                 f"broad_collisions On 평균 **{s.bcol_On.mean():.0f}** "
                 "(구 descent 제거 ⇒ ≈0).")
    lines.append(f"- broad_detect Off 는 physics_total 의 평균 "
                 f"**{broad_off_share:.0f}%** — c-2 On 에서 그만큼이 사라진다.")
    # before/after vs c-1
    bs = build_summary(base)
    ratio = []
    for a in A_VALUES:
        for b in B_VALUES:
            c1 = bs[(bs.cloth_n == a) & (bs.sphere_n == b)].phys_On.iloc[0]
            c2 = s[(s.cloth_n == a) & (s.sphere_n == b)].phys_On.iloc[0]
            ratio.append(c2 / c1)
    lines.append(f"- physics_total On: c-1 → c-2 평균 **{np.mean(ratio):.2f}×** "
                 f"(최선 {np.min(ratio):.2f}×).")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"[chart.py] wrote {path}")
    print("\n".join(lines))


# ----------------------------------------------------------------- headline bar
def draw_headline(s):
    """9 (a,b) 케이스: broad Off | broad On | narrow Off | narrow On 그룹 막대.

    c-1 headline 은 broad 가 mode-invariant 라 가정해 broad_Off 한 개만 그렸지만,
    c-2 는 broad On 이 0 으로 떨어지는 것이 핵심이라 broad On 막대를 추가한다.
    """
    labels = [f"a{int(r.cloth_n)}\nb{int(r.sphere_n)}" for _, r in s.iterrows()]
    x = np.arange(len(s))
    w = 0.2
    fig, ax = plt.subplots(figsize=(13, 6))
    ax.bar(x - 1.5*w, s.broad_Off,  w, label="broad (BVH) Off",      color=C_BROAD)
    ax.bar(x - 0.5*w, s.broad_On,   w, label="broad (BVH) On → ~0",  color="#aec7e8")
    ax.bar(x + 0.5*w, s.narrow_Off, w, label="narrow Off",           color="#55a868")
    ax.bar(x + 1.5*w, s.narrow_On,  w, label="narrow On (analytic)", color="#c44e52")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_xlabel("case  (a = cloth subdiv, b = sphere subdiv)")
    ax.set_ylabel("per-frame time (ms, collision-frame avg)")
    ax.set_title("Analytic broad-skip (c-2): analytic ON eliminates broad (BVH) "
                 "cost — sphere skipped at object level")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="upper left", framealpha=0.9)
    fig.tight_layout()
    out = os.path.join(HERE, "headline_bars.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out}")


# ------------------------------------------------------------ stacked breakdown
def draw_breakdown(s):
    fig, axes = plt.subplots(len(A_VALUES), len(B_VALUES),
                             figsize=(13, 10), sharex=True)
    ymax = 0
    for _, r in s.iterrows():
        ymax = max(ymax, r.phys_Off, r.phys_On)
    for i, a in enumerate(A_VALUES):
        for j, b in enumerate(B_VALUES):
            ax = axes[i][j]
            r = s[(s.cloth_n == a) & (s.sphere_n == b)].iloc[0]
            for k, mode in enumerate(MODES):
                broad  = r[f"broad_{mode}"]
                narrow = r[f"narrow_{mode}"]
                other  = max(r[f"phys_{mode}"] - broad - narrow, 0)
                ax.bar(k, broad,  color=C_BROAD,  label="broad_detect (BVH)" if (i==0 and j==0 and k==0) else None)
                ax.bar(k, narrow, bottom=broad, color=C_NARROW, label="narrow_phase" if (i==0 and j==0 and k==0) else None)
                ax.bar(k, other,  bottom=broad + narrow, color=C_OTHER, label="other (integrate/sync)" if (i==0 and j==0 and k==0) else None)
            ax.set_xticks([0, 1])
            ax.set_xticklabels(MODES)
            ax.set_ylim(0, ymax * 1.08)
            ax.set_title(f"a={a}, b={b}", fontsize=10)
            ax.grid(True, axis="y", alpha=0.25)
            if j == 0:
                ax.set_ylabel("ms / frame")
    fig.suptitle("physics_total breakdown — Off vs On (collision-frame avg)\n"
                 "rows: cloth subdiv a   cols: sphere subdiv b   "
                 "(On: blue broad segment → 0)", fontsize=12)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, framealpha=0.9)
    fig.tight_layout(rect=[0, 0.04, 1, 0.96])
    out = os.path.join(HERE, "time_breakdown.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out}")


# ----------------------------------------------------------- narrow time series
def draw_timeseries(df):
    fig, axes = plt.subplots(len(A_VALUES), len(B_VALUES),
                             figsize=(13, 10), sharex=True)
    for i, a in enumerate(A_VALUES):
        for j, b in enumerate(B_VALUES):
            ax = axes[i][j]
            for mode in MODES:
                sub = case(df, mode, a, b).sort_values("frame_index")
                ax.plot(sub.frame_index, sub.narrow_phase_ms,
                        marker="o", markersize=2.5, linewidth=1.3,
                        color=MODE_COLOR[mode], label=mode)
            ax.set_title(f"a={a}, b={b}", fontsize=10)
            ax.grid(True, alpha=0.25)
            if j == 0:
                ax.set_ylabel("narrow_phase (ms)")
            if i == len(A_VALUES) - 1:
                ax.set_xlabel("frame")
    axes[0][0].legend(loc="upper left", fontsize=9, framealpha=0.9)
    fig.suptitle("narrow_phase per frame — analytic Off vs On (c-2)\n"
                 "rows: cloth subdiv a   cols: sphere subdiv b", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(HERE, "narrow_timeseries.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out}")


# ----------------------------------------------------------- before/after vs c-1
def draw_before_after(s, base):
    bs = build_summary(base)
    labels, c1, c2 = [], [], []
    for a in A_VALUES:
        for b in B_VALUES:
            labels.append(f"a{a}\nb{b}")
            c1.append(bs[(bs.cloth_n == a) & (bs.sphere_n == b)].phys_On.iloc[0])
            c2.append(s[(s.cloth_n == a) & (s.sphere_n == b)].phys_On.iloc[0])
    x = np.arange(len(labels))
    w = 0.38
    fig, ax = plt.subplots(figsize=(13, 6))
    ax.bar(x - w/2, c1, w, label="c-1 On (loop all shapes, 2× commitAndWait)", color="#bbbbbb")
    ax.bar(x + w/2, c2, w, label="c-2 On (broad-skip + marker, 1× commitAndWait)", color="#d62728")
    for xi, (a, bb) in enumerate(zip(c1, c2)):
        ax.text(xi + w/2, bb, f"{bb/a:.2f}x", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_xlabel("case  (a = cloth subdiv, b = sphere subdiv)")
    ax.set_ylabel("physics_total (ms / frame, collision-frame avg)")
    ax.set_title("physics_total analytic-ON: c-1 → c-2 (lower is better)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(loc="upper left", framealpha=0.9)
    fig.tight_layout()
    out = os.path.join(HERE, "c1_vs_c2.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out}")


def main():
    df = load(CSV_PATH)
    base = load(BASELINE_PATH)
    s = build_summary(df)
    draw_headline(s)
    draw_breakdown(s)
    draw_timeseries(df)
    draw_before_after(s, base)
    write_summary_md(s, base)


if __name__ == "__main__":
    main()
