#!/usr/bin/env python3
"""Analytic-primitive on/off vs BVH 비교 차트.

analytic_bench.csv (= `build/src/ysim --bench-analytic` 출력) 는 한 세션에서
18 case × 30 frame = 540 행을 캡처한다:
  - analytic ∈ {Off, On}
  - cloth_n  (a) ∈ {20, 50, 100}   FastGridCloth 분할수
  - sphere_n (b) ∈ {20, 50, 100}   UV-sphere 분할수
각 case 는 frame_index 0..29.

씬: 1x1 XZ-plane FastGridCloth(a) 가 직경 1 구(b, Rigid static) 위로 떨어진다.
analytic ON 이면 구 충돌이 narrow_pt_tri 에서 skip 되고 per-vertex 타원체
테스트(narrow_pt_analytic)로 처리된다. 단 구는 BVH 에 그대로 남으므로
broad_detect(BVH) 비용은 토글과 무관하다(트레젝토리 차이만 존재).

출력:
  - headline_bars.png   : 9 (a,b) 별 broad_detect(BVH) vs narrow Off/On (충돌 프레임 평균)
  - time_breakdown.png  : 3x3 그리드, Off/On stacked (broad + narrow + 그 외)
  - narrow_timeseries.png: 3x3 그리드, 프레임별 narrow_phase Off vs On
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
CSV_PATH = os.path.join(HERE, "analytic_bench.csv")

A_VALUES = [20, 50, 100]   # cloth_n
B_VALUES = [20, 50, 100]   # sphere_n
MODES = ["Off", "On"]

MODE_COLOR = {"Off": "#1f77b4", "On": "#d62728"}
# Stacked-bar component colors.
C_BROAD  = "#4c72b0"   # broad_detect (BVH)
C_NARROW = "#dd8452"   # narrow_phase
C_OTHER  = "#cccccc"   # physics_total - broad - narrow (integrate/sync/etc.)


def load():
    if not os.path.exists(CSV_PATH):
        raise SystemExit(f"[chart.py] CSV not found: {CSV_PATH}")
    df = pd.read_csv(CSV_PATH)
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
            rows.append(rec)
    return pd.DataFrame(rows)


def write_summary_md(s):
    path = os.path.join(HERE, "summary.md")
    lines = []
    lines.append("# Analytic on/off — 충돌 프레임(narrow>0) 평균\n")
    lines.append("단위 ms. broad = broad_detect(BVH), narrow = narrow_phase, "
                 "phys = physics_total. ratio = narrow_On / narrow_Off.\n")
    lines.append("| a (cloth) | b (sphere) | broad Off | broad On | "
                 "narrow Off | narrow On | narrow ratio | phys Off | phys On |")
    lines.append("|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for _, r in s.iterrows():
        ratio = r.narrow_On / r.narrow_Off if r.narrow_Off else float("nan")
        lines.append(
            f"| {int(r.cloth_n)} | {int(r.sphere_n)} | "
            f"{r.broad_Off:.2f} | {r.broad_On:.2f} | "
            f"{r.narrow_Off:.2f} | {r.narrow_On:.2f} | {ratio:.2f}× | "
            f"{r.phys_Off:.2f} | {r.phys_On:.2f} |")
    # Aggregate headline numbers.
    broad_share = (s.broad_Off / s.phys_Off * 100).mean()
    narrow_off_share = (s.narrow_Off / s.phys_Off * 100).mean()
    lines.append("")
    lines.append(f"- broad_detect(BVH) 가 physics_total 의 평균 "
                 f"**{broad_share:.0f}%** (narrow_phase Off 는 평균 "
                 f"{narrow_off_share:.0f}%).")
    lines.append(f"- narrow On/Off 비율 평균 "
                 f"**{(s.narrow_On / s.narrow_Off).mean():.2f}×**.")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"[chart.py] wrote {path}")
    print("\n".join(lines))


# ----------------------------------------------------------------- headline bar
def draw_headline(s):
    """9 (a,b) 케이스: broad_detect(BVH) | narrow Off | narrow On 그룹 막대."""
    labels = [f"a{int(r.cloth_n)}\nb{int(r.sphere_n)}" for _, r in s.iterrows()]
    x = np.arange(len(s))
    w = 0.27
    fig, ax = plt.subplots(figsize=(13, 6))
    ax.bar(x - w, s.broad_Off,  w, label="broad_detect (BVH, Off)", color=C_BROAD)
    ax.bar(x,     s.narrow_Off, w, label="narrow_phase Off",        color="#55a868")
    ax.bar(x + w, s.narrow_On,  w, label="narrow_phase On (analytic)", color="#c44e52")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_xlabel("case  (a = cloth subdiv, b = sphere subdiv)")
    ax.set_ylabel("per-frame time (ms, collision-frame avg)")
    ax.set_title("Analytic toggle vs BVH: broad_detect dominates; "
                 "analytic narrow is NOT a win")
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
                 "rows: cloth subdiv a   cols: sphere subdiv b", fontsize=12)
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
    fig.suptitle("narrow_phase per frame — analytic Off vs On\n"
                 "rows: cloth subdiv a   cols: sphere subdiv b", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = os.path.join(HERE, "narrow_timeseries.png")
    fig.savefig(out, dpi=140)
    plt.close(fig)
    print(f"[chart.py] wrote {out}")


def main():
    df = load()
    s = build_summary(df)
    draw_headline(s)
    draw_breakdown(s)
    draw_timeseries(df)
    write_summary_md(s)


if __name__ == "__main__":
    main()
