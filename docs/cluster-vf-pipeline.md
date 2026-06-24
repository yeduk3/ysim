# Cluster-VF Pipeline — Two-Mesh Collision Experiment

Static Human (~24k verts) ↔ deforming cloth, bidirectional VF collision. Goal: make
sub-object (clustered) BVH collision FASTER than a general single-root BVH, reversing
the earlier "sub-object always loses" verdict (the per-point top phase blew up with k).

## Method

The Human is split into `k = min(4^s, F)` connected balanced clusters (face dual
graph + single-level k-way flood fill + Lloyd; `include/mesh_cluster.hpp`). Each
cluster becomes a sub-object BVH subtree, with a single-owned vertex list. The live
GPU pipeline (`BroadPhase::detectCollisionsCluster`, flag `clusterVFPipeline`,
default OFF) replaces the per-(q,t) full broad phase with, per substep, fully async
(1 sync/substep, GPU-counter-bounded over-dispatch — no count readbacks):

1. `cluster_aabb` — per-cluster AABB = tight box over current face verts ∪ swept
   group root (the static cluster's cached root box is unreliable; the dynamic
   cloth's swept leaves exceed current verts — the union covers both).
2. `clusterpair_query` — grid over cluster AABBs (Human cells built once on CPU),
   margin-inflated, min-cell dedup → candidate (cloth, Human) cluster pairs.
3. `clusterpair_expand` — pairs → bidirectional point-query pairs (cloth vert →
   Human cluster subtree, Human vert → cloth cluster subtree).
4. `queryPointsPairsBounded` — per-pair subtree descent → broad VF set.

Validated: cluster-restricted broad SET == full `detectCollisionsTwoMesh` SET
(missing=0, extra=0) across s=1..8 and drape depths (up to ~100k collisions exact);
cloth drape (maxY ~1.186) matches.

## Results

Two-mesh, P=50, static Human + cloth. `single` = general single-root BVH;
`subobj` = sub-object + OLD full detect; `cluster` = sub-object clustered + the new
VF pipeline.

### Full-frame (PerFrame async, median ms — the real cost)

| s | k(Human) | single | subobj (old) | cluster (new) |
|---|---|---|---|---|
| 0 | 1 | **141** | — | — |
| 2 | 16 | — | 142 | 184 |
| 3 | 64 | — | 142 | 151 |
| 4 | 256 | — | 153 | 142 |
| 5 | 1024 | — | 184 | **134** ← global min |
| 6 | 4096 | — | 309 | 141 |
| 7 | 16384 | — | 309 (saturated) | 146 |
| 8 | 48918 | — | 309 (saturated) | 180 |

### Per-section (InFrame, median ms) — where the difference lives

| variant | refit | **detect** | narrow | frame |
|---|---|---|---|---|
| single (s0) | 14 | 30 | 25 | 70 |
| subobj s5 | 14 | 72 | 26 | 113 |
| **cluster s5** | 14 | **20** | 24 | **58** |
| subobj s6 | ~22 | ~325 | ~50 | ~393 |
| cluster s6 | 13 | 25 | 25 | 64 |

`refit` (~13–14ms, cloth multi-root combine, sync-bound) and `narrow` (~24–26ms,
s-invariant) are FLAT. **`detect` is the only variable section.** The old path's
detect blows up with k (per-point top phase scans every cluster: 30→72→325). The
cluster pipeline's detect descends only relevant subtrees and stays small,
bottoming at **20ms at s5** — below single-root's 30ms.

## Verdict

The cluster-VF pipeline **inverts the sub-object scaling**: the old path got worse
with k; the clustered path gets better, crossing single-root at **s≈4** and reaching
the global minimum at **s5 (k=1024): 134ms (PerFrame) / 58ms-frame with 20ms detect
(InFrame)** — ~9% under single-root and ~1.4× under old sub-object at the same s, up
to 2.2× at s6. Fine clustering is, for the first time, the fastest option.

The win is a SCALING reversal, not a large absolute gain under async — under InFrame
(sync-dominated) the detect-only speedup is larger (3.6× at s5). The honest
full-frame number is PerFrame.

## Caveats / follow-ups

- **U-shaped**: past s5 over-fragmentation hurts (s8 = 1 face/cluster, degenerate,
  loses to single-root). Sweet spot ≈ s5.
- **Low s (s1–3)**: over-dispatch + extra-kernel overhead loses to the already-cheap
  full path; gate the pipeline to high s, or tighten the dir-buffer over-dispatch.
- **s1 tunneling** is a coarse-cloth-BVH scene artifact (both paths), not the pipeline.
- Robustness guards: grid cell floored to ≤~1M cells; pair buffer capped at 4M
  (kc·ks would be ~117M at s8).
- The Human grid is fixed to static bounds; fixing the static multi-root combine
  would let the pipeline read group roots directly instead of recomputing AABBs.

## Reproduce

```
./build/ysim --test-cluster [s]                       # clustering self-test
./build/ysim --test-clustervf [P] [s] [settle]        # bidirectional VF == full (CPU drive)
./build/ysim --bench-cluster-live [P] [s] [frames]    # GPU pipeline == full + frame time
./build/ysim --bench-cluster-compare [P] [maxS] [frames] [reps]   # single/subobj/cluster sweep
```

Charts/CSVs land in `profiles/experiment/cluster/` (gitignored): `cluster_compare.*`
(PerFrame frame_ms) via `chart_compare.py`, `cluster_sections.*` (InFrame per-section)
via `chart_sections.py`.
