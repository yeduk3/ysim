# Current Work — D-042 R-3 — Scene::pack memcpys preview to packed

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review.

## How far

- `include/MeshRenderState.hpp` — added `clearPreviewBindings()` public method (folds R-2 Estimator turn-37 WARNING).
- `src/main.cpp` — Scene::pack gained a size-guarded `std::memcpy` block after `mesh.initialize()` that overrides `state.x` / `state.n` / `adjacency.facets` from `req.preview.{x,n,facets}.data()`; block sits BEFORE the existing `xPrev` mirror memcpy so xPrev mirrors the post-preview x.
- `src/main.cpp` — `Simulator::translateObject` now dual-writes the translate delta into the matching `req.preview.x` (BDD-003 + BDD-018 regression fix; R-4 will make preview the primary mutation target).
- `src/main.cpp` — `Simulator::loadScene` calls `renderState.clearPreviewBindings()` after `pendingRotations.clear()`.
- `src/main.cpp` — new Block 38 (D-042 R-3) inside the Metal-gated section after Block 37. Sentinel injection (`reqs[0].preview.x[1] = 99.0f`) verifies the memcpy survives initializer regen.
- `docs/DECISIONS.md` — D-042 R-3 entry appended.

## What's tested

- `./src/ysim --self-test` — **69/69 PASS deterministic across 5 macOS runs**. PLAN predicted 68 → 69; actual matches.
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged.
- Bug-probe (a): commenting out the entire R-3 memcpy block → Block 38 FAILs with `packedHasSentinel=0 packedY=-0.1` (initializer's natural cube y at vertex 0). Restored.
- Bug-probe (b) skipped — covered by (a)'s full-block probe; individual memcpy isolation didn't add coverage.
- Bug-probe (c) `clearPreviewBindings` from loadScene not directly load-bearing in self-test (no GL context); verified by code review.
- No `BUG-PROBE` markers remain.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. Next planning cycle picks up R-4 (route translate/rotate/setMaterial through preview as primary, with state.x derived; finish what R-3's translate dual-write started).
