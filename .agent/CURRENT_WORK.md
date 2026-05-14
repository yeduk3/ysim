# Current Work — D-042 R-2 — MeshGL ↔ PreviewState binding

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review.

## How far

- `include/MeshRenderState.hpp` — added `PreviewBinding` struct + `previewBindings` map + 3 methods (`registerPreviewBinding<PR>`, `previewBinding(int) const`, `removeById(int)`); modified `getOrCreate` to prefer preview binding over packed sub-view; legacy fallback retained for parallel-symbol invariant.
- `src/main.cpp` — added `Simulator::registerPreviewBindingForLastRequest()` helper; wired into 7 addX wrappers (addClothFile/addFloatMesh/addClothGridFast/addCloth/addSphere/addCube/addGround) + `loadScene` addGeneralMesh site; `Simulator::removeMesh` calls `renderState.removeById(meshId)` before the request erase; `Simulator::initialize`'s `renderState.clear()` call retired (line 5403) with an explanatory comment block update.
- `src/main.cpp` — new Block 37 (D-042 R-2) after Block 36 inside the Metal-gated section. Verifies pre-init `renderState.previewBinding(req.id)` exists + every pointer/count matches `req.preview.*Ptr()/numPoints()/numFacets()`.
- `docs/DECISIONS.md` — D-042 R-2 entry appended.

## What's tested

- `./src/ysim --self-test` — **68/68 PASS deterministic across 5 macOS runs** (baseline 67 from R-1 + 1 new Block 37). Plan predicted 66 → 67; actual measured baseline was already 67 (one-off drift documented in DECISIONS).
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged from R-1 baseline.
- Bug-probe (a): removing `registerPreviewBindingForLastRequest()` from `addCube` → Block 37 FAILs (`xMatch=0 ...`); restored.
- Bug-probe (b): forcing `b.xPtr = nullptr` in `registerPreviewBinding` → Block 37 FAILs with `xMatch=0` only; restored.
- Bug-probe (c) per PLAN: skipped per Generator note — the legacy packed-sub-view fallback in `getOrCreate` is dead code under R-2 (every addX registers), so harness can't drive the fallback path without a deliberate setup that bypasses addX. Documented gap.
- No `BUG-PROBE` markers remain in `src/` or `include/`.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. Next planning cycle picks up R-3 (rewrite `Scene::pack` to memcpy preview → packed).
