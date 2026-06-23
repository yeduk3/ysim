#!/usr/bin/env python3
"""De-cluttered refit view: one panel per s, only mini/SAP/GPU refit (+ regular
reference). refit is mode-INVARIANT (the top phase can't affect BVH refit), so
these three should overlap; the IQR band is the run-to-run noise floor. Replaces
the unreadable 19-line refit panel in sap_vs_mini.png."""
import csv, glob, os, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
SPLITS = [1, 2, 3, 4, 5, 6]
KOF = {1: 4, 2: 16, 3: 49, 4: 169, 5: 625, 6: 2401}
STEADY_FROM = 15
MODES = [("mini", "mini-TLAS", "#1f77b4"),
         ("sap",  "CPU-SAP",   "#ff7f0e"),
         ("gpu",  "GPU-brute", "#2ca02c")]

def case_of(f):
    m = re.match(r"(regular|mini_s\d+|sap_s\d+|gpu_s\d+)_r\d+\.csv$", os.path.basename(f))
    return m.group(1) if m else None

data, nframes = {}, None
for f in sorted(glob.glob(os.path.join(HERE, "*.csv"))):
    ck = case_of(f)
    if not ck:
        continue
    rows = list(csv.DictReader(open(f)))
    nframes = len(rows)
    data.setdefault(ck, []).append(np.array([float(r["broad_refit"]) for r in rows]))

def band(ck):
    if ck not in data:
        return None
    M = np.vstack(data[ck])
    return np.median(M, 0), np.percentile(M, 25, 0), np.percentile(M, 75, 0)

x = np.arange(nframes)
fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True, sharey=True)
fig.suptitle("broad_refit per frame — mode-invariant sentinel (3 modes overlap; band = noise floor)",
             fontsize=13, fontweight="bold")

reg = band("regular")
for ax, s in zip(axes.flat, SPLITS):
    if reg is not None:
        ax.plot(x, reg[0], color="#999", lw=1.0, ls=":", label="regular (ref)")
    for key, label, col in MODES:
        b = band(f"{key}_s{s}")
        if b is None:
            continue
        ax.plot(x, b[0], color=col, lw=1.8, label=label)
        ax.fill_between(x, b[1], b[2], color=col, alpha=0.15)
    ax.axvspan(STEADY_FROM, nframes-1, color="gray", alpha=0.06)
    ax.set_title(f"s={s}  (k={KOF[s]})", fontsize=11)
    ax.grid(alpha=0.25); ax.legend(fontsize=8)
    ax.set_xlabel("frame"); ax.set_ylabel("refit ms")

fig.tight_layout(rect=[0, 0, 1, 0.96])
out = os.path.join(HERE, "refit_by_split.png")
fig.savefig(out, dpi=130)
print(f"chart -> {out}")
