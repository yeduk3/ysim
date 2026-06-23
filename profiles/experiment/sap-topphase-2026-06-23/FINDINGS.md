# SAP top-phase vs mini-TLAS — findings (2026-06-23)

Branch `feat/subobject-topphase-broadphase`. Replace the sub-object **mini-TLAS**
(binary tree stitched over the k group roots, descended on the GPU from a
super-root) with a **CPU sweep-and-prune top phase**: read the k group root
boxes, emit candidate `(point, groupRoot)` pairs, GPU descends one group subtree
per pair (`queryPointsPairs`). Point query boxes have constant width 2·margin ⇒
"overlaps group g on X" is a contiguous slice of points sorted by x ⇒ two binary
searches per group.

## Setup
Real default scene (cloth tess 50 ≈ 50×50 grid, Human declared static, 50×50 floor).
30-frame profiler, **8 repeats** per condition, median across repeats, steady
state = frames 15–29 (sustained cloth↔Human contact). Driver `run.sh`, analyzer
`analyze.py` → `sap_vs_mini.png`. Conditions: regular (single-root) /
mini_s{1,2,3} / sap_s{1,2,3}. The CPU sweep + per-pair dispatch are timed under
`broad_detect`.

## Result — steady-state median (ms)

| case          | refit | detect | narrow | frame |
|---------------|------:|-------:|-------:|------:|
| Regular BVH   | 28.35 | 44.08  | 28.14  | 121.4 |
| mini-TLAS s=1 | 25.32 | 37.36  | 24.08  | 116.7 |
| **SAP s=1**   | 27.89 | 41.46  | 27.28  | 127.2 |
| mini-TLAS s=2 | 25.75 | 40.31  | 25.08  | 120.9 |
| **SAP s=2**   | 26.04 | 39.33  | 25.39  | 121.7 |
| mini-TLAS s=3 | 29.54 | 45.69  | 28.70  | 141.1 |
| **SAP s=3**   | 28.89 | 41.45  | 28.88  | 133.4 |

### SAP vs mini-TLAS (Δ%, neg = SAP faster)
| s | k  | detect | frame |
|---|----|-------:|------:|
| 1 | 4  | **+11.0%** | +9.0% |
| 2 | 16 | −2.5%  | +0.7% |
| 3 | 49 | **−9.3%** | −5.5% |

## Reading
- **Monotone in k** (+11% → −2.5% → −9.3%): the trend, not the absolute noisy
  frame times, is the robust signal — and it matches theory.
- **SAP trades a fixed CPU sort for saved GPU descent depth.** Small k (k=4): the
  mini-TLAS it replaces is ~2 deep and nearly free, so SAP's O(N log N) point sort
  every substep is pure overhead → +11% detect. Large k (k=49): the mini-TLAS is
  ~6 deep and the per-point GPU descent over it dominates; SAP's contiguous-slice
  cull finds the few overlapping groups directly and each per-pair descent is one
  shallow group subtree → −9.3% detect, −5.5% frame. **Crossover ≈ s=2 (k=16).**

## Correctness
SAP output verified equal to brute force (`test/test_sap_topphase.cpp`, 200
trials); in-app `[SubObjectBVH] VALIDATE OK`; mini-vs-sap collision counts within
run-to-run noise (sim is non-deterministic — mini-vs-mini diverges identically,
GPU atomic append order).

## Caveats / next
- Sim is chaotic ⇒ wide IQR bands; only the median trend across 8 repeats is
  trustworthy. Per-frame curves diverge run-to-run by design.
- The CPU sort is single-thread, O(N log N) × 60 substeps — the whole cost SAP
  carries at small k. **Move the top phase to GPU** (the deferred option) to drop
  that fixed cost; would likely push the crossover below s=2 and deepen the s=3 win.
- This scene's mesh (~2.5k tris) is small; the subobject win itself is marginal
  here. SAP's advantage should widen on a LARGE actively-deforming mesh where
  group count and descent depth both grow.
