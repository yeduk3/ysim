# Sub-object top-phase: mini-TLAS vs CPU-SAP vs GPU-brute (2026-06-23)

Branch `feat/subobject-topphase-broadphase`. Three ways to do the sub-object
broad-phase TOP level (point → which group subtree(s) to descend):

- **mini-TLAS** (mode 0, baseline): k group roots stitched into a binary top
  tree, descended on the GPU from a super-root (depth ~log k).
- **CPU-SAP** (mode 1): CPU sweep-and-prune over the k group root boxes emits
  candidate (point, groupRoot) pairs; GPU descends one subtree per pair.
- **GPU-brute** (mode 2): each point thread brute-tests the k group roots on the
  GPU (O(k)) and descends overlapping subtrees inline. No CPU sort, no pairs.

## Setup
Real default scene (cloth tess 50 ≈ 50×50 grid → ~4.8k tris, Human static).
30-frame profiler, **8 repeats**, **all conditions INTERLEAVED round-robin** in
one batch (thermal/load averages across modes). Steady state = frames 15–29.
Sweep **s = 1..6 ⇒ k = 4, 16, 49, 169, 625, 2401** (a 600× span in group count;
at s=6 each group is ~2 tris). Conditions: regular / {mini,sap,gpu}_s{1..6}.
Top phase timed under `broad_detect`. Driver `run.sh`, analyzer `analyze.py` →
`sap_vs_mini.png`. 152 runs, 0 crashes.

## The noise floor (read this first)
`broad_refit` is **mode-invariant** — the top phase cannot change BVH refit. Yet
across modes that must have identical refit it swings **−9% … +27%**. That is the
noise floor: the sim is chaotic (GPU atomic append order → contact order →
trajectory divergence → per-run workload differs). `broad_detect`/`narrow_phase`
track `refit` ~1:1, so most cross-condition variation is **workload noise, not
the top-phase algorithm.** Top-phase-specific effect = residual `detect%−refit%`.

## Result — residual (detect% − refit-noise%) vs mini-TLAS, per s
| s | k    | CPU-SAP | GPU-brute |
|---|------|--------:|----------:|
| 1 | 4    | +4.5    | +1.7      |
| 2 | 16   | −2.3    | −0.3      |
| 3 | 49   | +16.7*  | +3.4      |
| 4 | 169  | +6.8    | +5.6      |
| 5 | 625  | −0.7    | −2.8      |
| 6 | 2401 | +4.0    | +0.7      |

\* SAP s=3 is an outlier run (chaotic-heavy trajectory: detect 65ms, refit 39ms,
both far above the others) — noise, not a top-phase cost.

## Reading (honest, conclusive)
- **No mode separates from mini-TLAS beyond the noise floor, at ANY k from 4 to
  2401.** Residuals scatter −3…+17 with no sign and no trend in s; the refit
  sentinel itself swings ±9–27%. The signal is noise.
- The s=1..3-only run's apparent "GPU ~−8% at k≥16" **did NOT reproduce** here —
  it was a small-sample noise artifact.
- **The top-phase level is NOT the bottleneck on this mesh.** `broad_detect`
  (~40–50 ms steady, flat across all modes/s) is dominated by the collision
  WORKLOAD (overlapping pairs → narrow phase + atomic appends), not by how the
  point finds its candidate groups. Swapping mini-TLAS ↔ SAP ↔ GPU-brute, and
  varying k 600×, leaves it unchanged.

This matches the prior sub-object result: parity at all k; "sub-object pays only
with a LARGE actively-deforming mesh."

## Correctness
All three modes: in-app `[SubObjectBVH] VALIDATE OK` (incl. k=2401), collision
counts within run-to-run noise of mini-TLAS, 0 crashes / 152 runs. CPU-SAP slice
math checked vs brute force (`test/test_sap_topphase.cpp`, 200 trials).

## Verdict / next
Negative result, cleanly: on a ~5k-tri cloth the sub-object top-phase choice is
irrelevant — workload-bound, mode-invariant within noise across k=4..2401. The
GPU-brute mode is correct and the cheapest to reason about (no CPU sort, no pair
buffer, no top tree), so it is the natural default IF the sub-object path is
used. To find any real top-phase difference: a **large actively-deforming mesh**
(top phase a bigger fraction of broad_detect) and/or a **position-replayed /
non-chaotic scene** (shrinks the refit noise so residuals are clean).
Toggle: key M cycles mode 0→1→2; `YSIM_SUBOBJECT=s YSIM_SAP={1,2}`.
