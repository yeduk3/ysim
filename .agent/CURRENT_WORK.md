# Current Work — D-042 R-7 — preview.n recompute + cleanup (FINAL D-042 slice)

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review. **This is the final slice in the D-042 refactor sequence.**

## How far

- `src/main.cpp` — R-5 resync gained `req.preview.recomputeNormals()` after the position memcpy.
- `include/MeshRenderState.hpp` — legacy `getOrCreate` packed-sub-view fallback comment refreshed to reflect R-2+'s universal preview binding (dead-in-production, retained for safety).
- `src/main.cpp` — new Block 42 inside the Metal-gated section after Block 41. Corrupt-and-restore mechanic: sets `preview.n[0] = 99.0f` (sentinel; recomputed normals are unit-length so |component| ≤ 1), runs sim.update, asserts the sentinel was overwritten.
- `docs/DECISIONS.md` — D-042 R-7 entry appended + D-042 sequence closure note. Records the `D-042-ROUND-TRIP-INVARIANT` standing constraint inline (Generator's write-set excludes `docs/roles/PLANNER.md`; next Planner pass should fold the constraint into PLANNER.md's "Standing constraints" subsection).

## What's tested

- `./src/ysim --self-test` — **73/73 PASS deterministic across 5 macOS runs**. PLAN predicted 72 → 73.
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged.
- Bug-probe (a): disabling R-7 recomputeNormals() → Block 42 FAILs with `restored=99.0` (sentinel never overwritten). Restored.
- No `BUG-PROBE` markers remain.

## Generator-discovered

- Block 42's first draft used `addCube` (Float) + `sim.update` and the test hung past 60s. Root cause not deeply investigated — Float-cube + Metal substep loop hang. Switched to `addCloth` (proven safe in Block 40/41). recomputeNormals test is mesh-shape-agnostic so the substitution doesn't reduce coverage. Documented as a harness gotcha for a future investigation slice.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. After R-7 merges, **the D-042 refactor sequence is COMPLETE**. The user's goal "씬 수정과 시뮬레이션 관리에서 버그를 최대한 제어함" closes. The next Planner cycle should:
- Fold `D-042-ROUND-TRIP-INVARIANT` into `docs/roles/PLANNER.md`'s "Standing constraints" subsection.
- Investigate the Float-cube + Metal substep-loop hang as its own slice (low priority — workaround documented).
- Pick the next milestone from PROJECT_STATE.md's standing-feature candidates (Alembic export, source-file split, B-2.1 Bullet refinements).
