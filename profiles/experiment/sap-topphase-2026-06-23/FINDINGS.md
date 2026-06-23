# Sub-object top-phase: mini-TLAS vs CPU-SAP vs GPU-brute (2026-06-23)

Branch `feat/subobject-topphase-broadphase`. Three ways to do the sub-object
broad-phase TOP level (point → which group subtree(s) to descend):

- **mini-TLAS** (mode 0, baseline): k group roots stitched into a binary top
  tree, descended on the GPU from a super-root.
- **CPU-SAP** (mode 1): CPU sweep-and-prune over the k group root boxes emits
  candidate (point, groupRoot) pairs; GPU descends one subtree per pair.
- **GPU-brute** (mode 2): each point thread brute-tests the k group roots on the
  GPU and descends overlapping subtrees inline. No CPU sort, no pair buffer.

## Setup
Real default scene (cloth tess 50 ≈ 50×50 grid, Human static, 50×50 floor).
30-frame profiler, **8 repeats**, **all conditions INTERLEAVED in one batch**
(round-robin) so thermal/load drift averages across modes. Steady state =
frames 15–29. Conditions: regular / {mini,sap,gpu}_s{1,2,3}. The top phase is
timed under `broad_detect`. Driver `run.sh`, analyzer `analyze.py` →
`sap_vs_mini.png`.

## The noise floor (read this first)
`broad_refit` is **mode-invariant** — the top phase cannot change BVH refit. Yet
across modes that must have identical refit it swings **−11% … +16%**. That is
the noise floor: the sim is chaotic (GPU atomic append order → contact order →
trajectory divergence → per-run workload differs). `broad_detect` and
`narrow_phase` track `refit` almost 1:1, so most cross-condition variation is
**common-mode workload noise, not the top-phase algorithm.**

The top-phase-specific effect is the **residual = detect% − refit%** (subtracting
the workload/thermal sentinel):

| s | k  | mode    | detect Δ% | refit(noise) Δ% | **residual** |
|---|----|---------|----------:|----------------:|-------------:|
| 1 | 4  | CPU-SAP |   +0.6    |   +2.0          | **−1.4**     |
| 1 | 4  | GPU-top |   +1.6    |   −0.3          | **+1.9**     |
| 2 | 16 | CPU-SAP |  −16.6    |  −10.5          | **−6.1**     |
| 2 | 16 | GPU-top |  −19.9    |  −11.3          | **−8.6**     |
| 3 | 49 | CPU-SAP |  +15.7    |  +16.3          | **−0.5**     |
| 3 | 49 | GPU-top |   −6.0    |   +2.0          | **−8.0**     |

## Reading (honest)
- **Small k (s=1): all three modes are at parity** (residual ±2%, inside noise).
- **Larger k (s≥2): a WEAK but CONSISTENT hint that GPU-brute shaves ~8% off
  `broad_detect`** beyond workload (residual −8.6% @ s2, −8.0% @ s3). CPU-SAP is
  inconsistent (−6.1% then −0.5%) — its fixed O(N log N) point sort eats its own
  cull win on this small mesh.
- The effect (~8%) is the same magnitude as the noise floor (refit ±10–16%), so
  this is **suggestive, not conclusive**. On this ~2.5k-tri cloth the top-phase
  cost is a small fraction of `broad_detect` (which is dominated by the actual
  pair workload + atomic appends), so no mode robustly wins.

⚠ **Correction to the earlier 2-way run** (mini vs sap only, separate batches):
its "clean monotone −9.3%" was a batch/thermal confound + noise, not signal. The
interleaved 3-way with the refit sentinel is the trustworthy version.

## Correctness
GPU-brute & CPU-SAP both: in-app `[SubObjectBVH] VALIDATE OK`, collision counts
within run-to-run noise of mini-TLAS, 0 crashes over 80 runs. CPU-SAP slice math
checked vs brute force (`test/test_sap_topphase.cpp`, 200 trials).

## Verdict / next
GPU-brute is the most promising (kills both the CPU sort AND the tree depth;
consistently slightly negative residual at k≥16) but the win is at the edge of
noise here. To get a definitive number: a **large actively-deforming mesh** (so
the top phase is a bigger fraction of broad_detect) and/or a **less chaotic /
position-replayed scene** (so refit-noise shrinks and the residual is clean).
Toggle: key M cycles mode 0→1→2; `YSIM_SUBOBJECT=s YSIM_SAP={1,2}`.
