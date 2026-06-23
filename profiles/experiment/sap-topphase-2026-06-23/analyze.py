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
GPUCOL  = {1: "#08306b", 2: "#7f2704", 3: "#00441b"}
for s in SPLITS:
    CASES[f"mini_s{s}"] = (f"mini-TLAS s={s}", PALETTE[s][0])
    CASES[f"sap_s{s}"]  = (f"SAP s={s}",       PALETTE[s][1])
    CASES[f"gpu_s{s}"]  = (f"GPU-top s={s}",   GPUCOL[s])

def case_of(f):
    m = re.match(r"(regular|mini_s\d+|sap_s\d+|gpu_s\d+)_r\d+\.csv$", os.path.basename(f))
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

# ---- head-to-head: SAP & GPU-top vs mini at each s --------------------------
# refit is mode-INVARIANT (the top phase cannot affect BVH refit) so its delta
# is a pure noise/workload-divergence sentinel. The top-phase-specific effect is
# the RESIDUAL: detect% - refit%. If |residual| <= |refit%|, it's not signal.
print(f"\n{'vs mini-TLAS (steady-state %, neg = faster). refit = noise sentinel':66s}")
for s in SPLITS:
    mk = f"mini_s{s}"
    if mk not in summary: continue
    for variant in (f"sap_s{s}", f"gpu_s{s}"):
        if variant not in summary: continue
        def pct(sec): return 100.0*(summary[variant][sec]-summary[mk][sec])/summary[mk][sec]
        resid = pct('broad_detect') - pct('broad_refit')
        print(f"  {CASES[variant][0]:12s}: detect {pct('broad_detect'):+6.1f}%   "
              f"refit(noise) {pct('broad_refit'):+6.1f}%   frame {pct('frame_ms'):+6.1f}%   "
              f"=> detect residual {resid:+6.1f}%")

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
ax1 = fig.add_subplot(2,3,1); line_panel(ax1, "broad_detect", "Broad-phase query (broad_detect) — top phase changes THIS", "ms", allkeys)
ax2 = fig.add_subplot(2,3,2); line_panel(ax2, "frame_ms",     "Total frame time", "ms", allkeys)
ax3 = fig.add_subplot(2,3,3); line_panel(ax3, "narrow_phase", "Narrow-phase query", "ms", allkeys)

# steady-state grouped bars: broad_detect, mini vs sap vs gpu per s (+ regular)
ax4 = fig.add_subplot(2,3,4)
groups = ["regular"] + [f"_s{s}" for s in SPLITS]
xb = np.arange(len(groups)); w = 0.27
def col_detect(key): return summary.get(key, {}).get("broad_detect", 0)
mini_v = [col_detect("regular")] + [col_detect(f"mini_s{s}") for s in SPLITS]
sap_v  = [0] + [col_detect(f"sap_s{s}") for s in SPLITS]
gpu_v  = [0] + [col_detect(f"gpu_s{s}") for s in SPLITS]
ax4.bar(xb - w, mini_v, w, label="mini-TLAS / regular", color="#666")
ax4.bar(xb,     sap_v,  w, label="CPU SAP", color="#5aa0d6")
ax4.bar(xb + w, gpu_v,  w, label="GPU top", color="#08306b")
ax4.set_xticks(xb); ax4.set_xticklabels(["regular"] + [f"s={s}" for s in SPLITS])
ax4.set_title(f"broad_detect steady-state (frames {STEADY_FROM}-{nframes-1})", fontsize=11)
ax4.set_ylabel("ms"); ax4.grid(alpha=0.25, axis="y"); ax4.legend(fontsize=8)

# detect Δ% vs mini per s: SAP and GPU
ax5 = fig.add_subplot(2,3,5)
ss = [s for s in SPLITS if f"mini_s{s}" in summary]
for variant, col, mark in [("sap","#5aa0d6","o-"), ("gpu","#08306b","s-")]:
    pts = [(s, 100.0*(summary[f"{variant}_s{s}"]["broad_detect"]-summary[f"mini_s{s}"]["broad_detect"])
              /summary[f"mini_s{s}"]["broad_detect"]) for s in ss if f"{variant}_s{s}" in summary]
    if pts:
        ax5.plot([f"s={s}" for s,_ in pts], [y for _,y in pts], mark,
                 label=f"{variant.upper()} detect", color=col)
ax5.axhline(0, color="k", lw=0.8)
ax5.set_title("detect Δ% vs mini-TLAS (neg = faster)", fontsize=11)
ax5.set_ylabel("% change"); ax5.grid(alpha=0.25); ax5.legend(fontsize=8)

ax6 = fig.add_subplot(2,3,6); line_panel(ax6, "broad_refit", "BVH refit (broad_refit)", "ms", allkeys)

fig.tight_layout(rect=[0,0,1,0.96])
out = os.path.join(HERE, "sap_vs_mini.png")
fig.savefig(out, dpi=130)
print(f"\nchart -> {out}")
