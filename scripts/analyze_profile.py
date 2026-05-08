#!/usr/bin/env python3
"""Analyze a FrameProfiler CSV exported from ysim.

Usage:
    python3 scripts/analyze_profile.py <profile.csv>

Output: a `<profile>_analysis/` directory next to the CSV containing:
    - physics_breakdown.png   physics_total breakdown + per-stage correlations
    - collisions.png           per-frame broad/narrow counts + narrow/broad %
    - summary.md               text summary of breakdowns and correlations
"""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# physics_total wraps these stages (src/main.cpp lines ~4519-4637).
PHYSICS_STAGES = [
    "bvh_build",
    "broad_refit",
    "broad_enlarge_trajectory",
    "broad_detect",
    "narrow_phase",
    "system_update",
    "metal_commit",
]


def load(csv_path):
    df = pd.read_csv(csv_path)
    # Frame 0 carries one-time bvh allocation cost — exclude from analyses.
    warm = df.iloc[1:].copy() if len(df) > 1 else df.copy()
    return df, warm


def physics_breakdown_chart(warm, out_path, title):
    present = [s for s in PHYSICS_STAGES if s in warm.columns]
    mean_total = warm["physics_total"].mean()
    mean_stages = warm[present].mean().sort_values(ascending=False)
    other = max(mean_total - mean_stages.sum(), 0.0)

    fig = plt.figure(figsize=(16, 11))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.1])

    ax = fig.add_subplot(gs[0, 0])
    labels = list(mean_stages.index) + ["(other)"]
    values = list(mean_stages.values) + [other]
    pcts = [v / mean_total * 100 for v in values]
    y_pos = np.arange(len(labels))
    colors = plt.cm.tab10(np.linspace(0, 1, len(labels)))
    ax.barh(y_pos, values, color=colors)
    ax.set_yticks(y_pos)
    ax.set_yticklabels(labels)
    ax.invert_yaxis()
    for i, (v, p) in enumerate(zip(values, pcts)):
        ax.text(v, i, f"  {v:.2f} ms  ({p:.1f}%)", va="center", fontsize=9)
    ax.set_xlabel("Mean ms / frame")
    ax.set_title(f"physics_total breakdown — total {mean_total:.1f} ms / frame")
    ax.set_xlim(0, max(values) * 1.4)
    ax.grid(axis="x", alpha=0.3)

    ax = fig.add_subplot(gs[0, 1])
    ordered = list(mean_stages.index)
    color_map = dict(zip(ordered, colors))
    stack = warm[ordered].values.T
    ax.stackplot(warm["frame_sequence"].values, stack, labels=ordered,
                 colors=[color_map[s] for s in ordered], alpha=0.85)
    ax.plot(warm["frame_sequence"].values, warm["physics_total"].values,
            "k--", linewidth=1.2, label="physics_total", alpha=0.8)
    ax.set_xlabel("frame_sequence")
    ax.set_ylabel("ms")
    ax.set_title("physics_total — per-frame stacked breakdown")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)

    def correlation_panel(ax, x_col, x_label, panel_title):
        xs = warm[x_col].values
        rs = []
        for s in present:
            ys = warm[s].values
            if ys.std() > 0 and xs.std() > 0:
                r = float(np.corrcoef(xs, ys)[0, 1])
            else:
                r = float("nan")
            rs.append((s, r))
        rs.sort(key=lambda kv: -abs(kv[1]) if not np.isnan(kv[1]) else 0)

        for s, r in rs:
            ys = warm[s].values
            ax.scatter(xs, ys, s=22, alpha=0.65, color=color_map.get(s, "gray"),
                       label=f"{s}  r={r:+.2f}")
            if not np.isnan(r) and xs.std() > 0:
                slope, intercept = np.polyfit(xs, ys, 1)
                xfit = np.array([xs.min(), xs.max()])
                ax.plot(xfit, slope * xfit + intercept,
                        color=color_map.get(s, "gray"),
                        linewidth=1.0, alpha=0.55)
        ax.set_xlabel(x_label)
        ax.set_ylabel("ms / frame")
        ax.set_title(panel_title)
        ax.legend(loc="upper left", fontsize=8, framealpha=0.85)
        ax.grid(alpha=0.3)
        return rs

    ax = fig.add_subplot(gs[1, 0])
    broad_corr = correlation_panel(
        ax, "broad_collisions",
        "broad_collisions per frame",
        "Per-stage time vs broad_collisions (sorted by |r|)")

    ax = fig.add_subplot(gs[1, 1])
    narrow_corr = correlation_panel(
        ax, "narrow_collisions",
        "narrow_collisions per frame",
        "Per-stage time vs narrow_collisions (sorted by |r|)")

    fig.suptitle(title, fontsize=13)
    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)

    return mean_total, mean_stages, other, broad_corr, narrow_corr


def collisions_chart(warm, out_path, title):
    frames = warm["frame_sequence"].values
    broad = warm["broad_collisions"].values.astype(float)
    narrow = warm["narrow_collisions"].values.astype(float)
    with np.errstate(invalid="ignore", divide="ignore"):
        ratio_pct = np.where(broad > 0, narrow / broad * 100.0, 0.0)

    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                             gridspec_kw={"height_ratios": [1.4, 1.0]})

    ax = axes[0]
    ax2 = ax.twinx()
    l1, = ax.plot(frames, broad, color="tab:orange", linewidth=1.4,
                  label="broad_collisions")
    l2, = ax2.plot(frames, narrow, color="tab:green", linewidth=1.4,
                   label="narrow_collisions")
    ax.set_ylabel("broad_collisions per frame", color="tab:orange")
    ax2.set_ylabel("narrow_collisions per frame", color="tab:green")
    ax.tick_params(axis="y", labelcolor="tab:orange")
    ax2.tick_params(axis="y", labelcolor="tab:green")
    ax.set_title("Per-frame broad vs narrow collision counts")
    ax.grid(alpha=0.3)
    ax.legend(handles=[l1, l2], loc="upper left", fontsize=9)

    ax = axes[1]
    ax.plot(frames, ratio_pct, color="tab:purple", linewidth=1.4)
    ax.fill_between(frames, 0, ratio_pct, color="tab:purple", alpha=0.15)
    ax.axhline(ratio_pct.mean(), color="tab:purple", linestyle="--",
               linewidth=0.9, alpha=0.7,
               label=f"mean {ratio_pct.mean():.3f}%")
    ax.set_xlabel("frame_sequence")
    ax.set_ylabel("narrow / broad  (%)")
    ax.set_title("Narrow as % of broad  (narrow_collisions ÷ broad_collisions × 100)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(alpha=0.3)

    for label, idx in [("first", 0),
                       ("max", int(np.argmax(ratio_pct))),
                       ("last", len(frames) - 1)]:
        ax.annotate(f"{label}: {ratio_pct[idx]:.3f}%",
                    xy=(frames[idx], ratio_pct[idx]),
                    xytext=(0, 8), textcoords="offset points",
                    ha="center", fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white",
                              ec="tab:purple", alpha=0.8))

    fig.suptitle(title, fontsize=12)
    plt.tight_layout()
    plt.savefig(out_path, dpi=130)
    plt.close(fig)

    return broad, narrow, ratio_pct


def write_summary(path, csv_name, n, mean_total, mean_stages, other,
                  broad_corr, narrow_corr, broad, narrow, ratio_pct):
    lines = []
    lines.append(f"# Profile analysis — {csv_name}")
    lines.append("")
    lines.append(f"Frames analyzed: {n}  (frame 0 excluded as warm-up)")
    lines.append(f"Mean physics_total: {mean_total:.2f} ms / frame")
    lines.append("")
    lines.append("## physics_total breakdown")
    for name, val in mean_stages.items():
        lines.append(f"  {name:30s}  {val:8.3f} ms  ({val/mean_total*100:5.1f}%)")
    lines.append(f"  {'(other / overhead)':30s}  {other:8.3f} ms  "
                 f"({other/mean_total*100:5.1f}%)")
    lines.append("")
    lines.append("## Collision counts")
    lines.append(f"  broad_collisions   mean {broad.mean():>12,.0f}   max {broad.max():>12,.0f}")
    lines.append(f"  narrow_collisions  mean {narrow.mean():>12,.0f}   max {narrow.max():>12,.0f}")
    lines.append(f"  narrow / broad     mean {ratio_pct.mean():.4f}%   max {ratio_pct.max():.4f}%")
    lines.append("")
    lines.append("## Pearson r — stage vs broad_collisions")
    for s, r in broad_corr:
        lines.append(f"  {s:30s}  r = {r:+.3f}")
    lines.append("")
    lines.append("## Pearson r — stage vs narrow_collisions")
    for s, r in narrow_corr:
        lines.append(f"  {s:30s}  r = {r:+.3f}")
    lines.append("")
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        sys.exit(2)

    csv_path = os.path.abspath(sys.argv[1])
    if not os.path.isfile(csv_path):
        print(f"error: {csv_path} not found", file=sys.stderr)
        sys.exit(1)

    base = os.path.splitext(os.path.basename(csv_path))[0]
    out_dir = os.path.join(os.path.dirname(csv_path), f"{base}_analysis")
    os.makedirs(out_dir, exist_ok=True)

    df, warm = load(csv_path)
    n = len(warm)
    title = f"{os.path.basename(csv_path)}  ({n} warm frames)"

    breakdown_path = os.path.join(out_dir, "physics_breakdown.png")
    collisions_path = os.path.join(out_dir, "collisions.png")
    summary_path = os.path.join(out_dir, "summary.md")

    mean_total, mean_stages, other, broad_corr, narrow_corr = \
        physics_breakdown_chart(warm, breakdown_path, title)
    broad, narrow, ratio_pct = collisions_chart(warm, collisions_path, title)
    write_summary(summary_path, os.path.basename(csv_path), n,
                  mean_total, mean_stages, other,
                  broad_corr, narrow_corr, broad, narrow, ratio_pct)

    print(f"Wrote:\n  {breakdown_path}\n  {collisions_path}\n  {summary_path}")


if __name__ == "__main__":
    main()
