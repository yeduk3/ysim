# Current Work — D-042 R-4 — rotateObject writes preview.x

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review.

## How far

- `src/main.cpp` — `Simulator::rotateObject` gained a preview write-back loop (mirrors R-3's translateObject pattern). Loop runs AFTER state.x/xPrev rotation, BEFORE `mesh->rotationQuat = newAbs`.
- `src/main.cpp` — `pendingRotations[meshId] = newAbs;` line REMOVED from rotateObject (Generator-discovered necessity — without removal, R-4's preview rotation + the existing applyPendingMaterials re-apply path produce a double rotation; bug-probe (a) confirmed this is now the sole mechanism). `loadScene`'s separate `pendingRotations` stash for deferred-at-load rotations is UNCHANGED.
- `src/main.cpp` — translateObject comment refreshed from "transitional bridge" to "canonical dual-write" framing.
- `src/main.cpp` — new Block 39 (D-042 R-4) inside the Metal-gated section after Block 38. Calls `sim.initialize()` so `findById(0)` succeeds, then `rotateObject(0, q90)`, asserts preview.x[0..2] reflects 90° Y rotation of (0.1, -0.1, -0.1) → (-0.1, -0.1, -0.1).
- `docs/DECISIONS.md` — D-042 R-4 entry appended.

## What's tested

- `./src/ysim --self-test` — **70/70 PASS deterministic across 5 macOS runs**. PLAN predicted 69 → 70; actual matches.
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged.
- Bug-probe (a): disabling the preview rotate loop → 4 simultaneous FAILs (Block 39 `moved=0` + FR-004 UI + FR-004 pack-rebuild + BDD-018 rotate-inspector). Confirms the preview write is now the SOLE rotation-persistence mechanism. Restored.
- Initial build also surfaced a Generator-fix: vertex 0 of primitive::cube is at (+h, -h, -h) (+X face emitted first), not (-h, -h, -h) as the PLAN sketch assumed. Block 39's expected post-rotation value corrected to (-0.1, -0.1, -0.1) after 90° Y rotation of (0.1, -0.1, -0.1) → (z, y, -x).
- No `BUG-PROBE` markers remain.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. Next planning cycle picks up R-5 (packed → preview resync at end of `Simulator::update` so simulated motion lands in preview before the next render/edit).
