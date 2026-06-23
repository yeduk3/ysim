#!/usr/bin/env python3
"""SAP top-phase vs mini-TLAS vs regular BVH — real default scene (Human static).

Per condition (8 repeats, 30-frame profiler CSVs):
  regular   : single-root BVH            (no env)
  mini_s<s> : subobject + mini-TLAS top  (YSIM_SUBOBJECT=s)
  sap_s<s>  : subobject + CPU SAP top     (YSIM_SUBOBJECT=s YSIM_SAP=1)

The SAP top phase (CPU sweep + GPU per-pair descent) replaces the mini-TLAS
GPU descent, so the cost lands in broad_detect. Sim is non-deterministic
(GPU atomic append order → chaotic contact divergence), hence median across
repeats + IQR band. Charts mini vs sap at each s, with regular as reference.
"""
import csv, glob, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SPLITS = [1, 2, 3]
SECTIONS = ["broad_refit", "broad_detect", "narrow_phase", "bvh_build", "frame_ms"]
STEADY_FROM = 15   # cloth in sustained contact with Human from ~frame 15

# case key -> (label, color)
CASES = {"regular": ("Regular BVH", "#444444")}
PALETTE = {1: ("#1f77b4", "#9ecae1"), 2: ("#ff7f0e", "#fdd0a2"), 3: ("#2ca02c", "#a1d99b")}
for s in SPLITS:
    CASES[f"mini_s{s}"] = (f"mini-TLAS s={s}", PALETTE[s][0])
    CASES[f"sap_s{s}"]  = (f"SAP s={s}",       PALETTE[s][1])

def case_of(f):
    m = re.match(r"(regular|mini_s\d+|sap_s\d+)_r\d+\.csv$", os.path.basename(f))
    return m.group(1) if m else None

data = {k: {s: [] for s in SECTIONS} for k in CASES}
nframes = None
for f in sorted(glob.glob(os.path.join(HERE, "*.csv"))):
    ck = case_of(f)
    if ck not in CASES:
        continue
    rows = list(csv.DictReader(open(f)))
    nframes = len(rows)
    for s in SECTIONS:
        data[ck][s].append(np.array([float(r[s]) for r in rows]))

def stack(ck, s):
    arrs = data[ck][s]
    if not arrs:
        return None, None, None
    M = np.vstack(arrs)
    return np.median(M, axis=0), np.percentile(M, 25, axis=0), np.percentile(M, 75, axis=0)

def ss_median(ck, s):  # per-repeat steady mean, then median across repeats
    return float(np.median([a[STEADY_FROM:].mean() for a in data[ck][s]]))

present = [k for k in CASES if data[k]["broad_refit"]]

# ---- summary table ----------------------------------------------------------
nrep = {k: len(data[k]['broad_refit']) for k in present}
print(f"\n{'='*96}\nSAP top-phase vs mini-TLAS vs Regular — real default scene, Human static")
print(f"median across repeats; steady-state = frames {STEADY_FROM}-{(nframes-1) if nframes else '?'}")
print(f"repeats/case: " + ", ".join(f"{k}={nrep[k]}" for k in present) + f"\n{'='*96}")
hdr = f"{'case':16s} {'refit(ss)':>10s} {'detect(ss)':>11s} {'narrow(ss)':>11s} {'build(ss)':>10s} {'frame(ss)':>10s}"
print(hdr); print("-"*len(hdr))
summary = {}
for ck in present:
    summary[ck] = {s: ss_median(ck, s) for s in SECTIONS}
    print(f"{CASES[ck][0]:16s} {summary[ck]['broad_refit']:10.3f} {summary[ck]['broad_detect']:11.3f} "
          f"{summary[ck]['narrow_phase']:11.3f} {summary[ck]['bvh_build']:10.3f} {summary[ck]['frame_ms']:10.2f}")

# ---- head-to-head: SAP vs mini at each s (the actual question) ---------------
print(f"\n{'SAP vs mini-TLAS (steady-state %, neg = SAP faster)':50s}")
for s in SPLITS:
    mk, sk = f"mini_s{s}", f"sap_s{s}"
    if mk not in summary or sk not in summary:
        continue
    def pct(sec): return 100.0*(summary[sk][sec]-summary[mk][sec])/summary[mk][sec]
    print(f"  s={s}:  detect {pct('broad_detect'):+6.1f}%   refit {pct('broad_refit'):+6.1f}%   "
          f"narrow {pct('narrow_phase'):+6.1f}%   frame {pct('frame_ms'):+6.1f}%")

# ---- charts -----------------------------------------------------------------
x = np.arange(nframes)
fig = plt.figure(figsize=(15, 9))
fig.suptitle("SAP top-phase vs mini-TLAS vs Regular BVH — default scene, Human static (30-frame, median, IQR)",
             fontsize=13, fontweight="bold")

def line_panel(ax, sec, title, ylab, keys):
    for ck in keys:
        if ck not in present: continue
        label, color = CASES[ck]
        m, q25, q75 = stack(ck, sec)
        dashed = ck.startswith("sap_")
        ax.plot(x, m, label=label, color=color, lw=1.8, ls="--" if dashed else "-")
        ax.fill_between(x, q25, q75, color=color, alpha=0.10)
    ax.axvspan(STEADY_FROM, nframes-1, color="gray", alpha=0.06)
    ax.set_title(title, fontsize=11); ax.set_xlabel("frame"); ax.set_ylabel(ylab)
    ax.grid(alpha=0.25); ax.legend(fontsize=7)

allkeys = present
ax1 = fig.add_subplot(2,3,1); line_panel(ax1, "broad_detect", "Broad-phase query (broad_detect) — SAP changes THIS", "ms", allkeys)
ax2 = fig.add_subplot(2,3,2); line_panel(ax2, "frame_ms",     "Total frame time", "ms", allkeys)
ax3 = fig.add_subplot(2,3,3); line_panel(ax3, "narrow_phase", "Narrow-phase query", "ms", allkeys)

# steady-state grouped bars: broad_detect, mini vs sap per s (+ regular)
ax4 = fig.add_subplot(2,3,4)
groups = ["regular"] + [f"_s{s}" for s in SPLITS]
xb = np.arange(len(groups)); w = 0.38
mini_v = [summary["regular"]["broad_detect"] if "regular" in summary else 0] + \
         [summary.get(f"mini_s{s}", {}).get("broad_detect", 0) for s in SPLITS]
sap_v  = [0] + [summary.get(f"sap_s{s}", {}).get("broad_detect", 0) for s in SPLITS]
ax4.bar(xb - w/2, mini_v, w, label="mini-TLAS / regular", color="#666")
ax4.bar(xb + w/2, sap_v,  w, label="SAP", color="#5aa0d6")
ax4.set_xticks(xb); ax4.set_xticklabels(["regular"] + [f"s={s}" for s in SPLITS])
ax4.set_title(f"broad_detect steady-state (frames {STEADY_FROM}-{nframes-1})", fontsize=11)
ax4.set_ylabel("ms"); ax4.grid(alpha=0.25, axis="y"); ax4.legend(fontsize=8)

# SAP vs mini delta % per s
ax5 = fig.add_subplot(2,3,5)
ss = [s for s in SPLITS if f"sap_s{s}" in summary and f"mini_s{s}" in summary]
for sec, col in [("broad_detect","#1f77b4"),("frame_ms","#d62728"),("narrow_phase","#2ca02c")]:
    ys = [100.0*(summary[f"sap_s{s}"][sec]-summary[f"mini_s{s}"][sec])/summary[f"mini_s{s}"][sec] for s in ss]
    ax5.plot([f"s={s}" for s in ss], ys, "o-", label=sec, color=col)
ax5.axhline(0, color="k", lw=0.8)
ax5.set_title("SAP vs mini-TLAS (steady-state %, neg = SAP faster)", fontsize=11)
ax5.set_ylabel("% change"); ax5.grid(alpha=0.25); ax5.legend(fontsize=8)

ax6 = fig.add_subplot(2,3,6); line_panel(ax6, "broad_refit", "BVH refit (broad_refit)", "ms", allkeys)

fig.tight_layout(rect=[0,0,1,0.96])
out = os.path.join(HERE, "sap_vs_mini.png")
fig.savefig(out, dpi=130)
print(f"\nchart -> {out}")
