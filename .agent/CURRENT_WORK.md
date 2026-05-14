# Current Work — D-042 R-5 — packed→preview resync at end of update

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review.

## How far

- `src/main.cpp` — `Simulator::update` gained a one-way `state.x → preview.x` memcpy loop at the very end (after `frame++`). Nested per-mesh + id-match-against-request, size-guarded.
- `src/main.cpp` — new Block 40 inside the Metal-gated section after Block 39. 4-particle cloth at y=0.25, 30-frame pump, asserts `post_y < 0.20` AND `post_y < pre_y - 0.01` AND `|state_y - post_y| < 1e-5`.
- `docs/DECISIONS.md` — D-042 R-5 entry appended.

## What's tested

- `./src/ysim --self-test` — **71/71 PASS deterministic across 5 macOS runs**. PLAN predicted 70 → 71.
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged.
- Bug-probe (a): disabling the resync loop → Block 40 FAILs (pre_y=0.25, post_y=0.25, state_y=-0.98, all 3 sub-clauses fail). Confirms the resync is load-bearing. Restored.
- No `BUG-PROBE` markers remain.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. Next planning cycle picks up R-6 (self-test migration: replace `Scene::meshes[i].state.x` reads with `Scene::requestsGeneralMeshes[i].preview.x` aliasing where appropriate).
