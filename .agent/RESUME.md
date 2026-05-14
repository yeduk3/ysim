# Resume — D-042 R-7 — preview.n recompute + cleanup (FINAL D-042 slice)

## Must remember

- **D-042 REFACTOR SEQUENCE COMPLETE** after this slice merges. R-1 → R-7 closed. The user's "stop bugs in scene editing + simulation management" goal that motivated R-* is satisfied.
- **`D-042-ROUND-TRIP-INVARIANT`** is the new standing constraint — documented in DECISIONS R-7 entry. Next Planner pass should fold it into `docs/roles/PLANNER.md`'s "Standing constraints" subsection (Generator's write-set excludes that path).
- **`MeshRenderState::getOrCreate` legacy fallback is dead in production** — every addX+loadScene calls `registerPreviewBinding`. Kept for safety, not for production reachability.
- **Harness gotcha (Generator-discovered)**: addCube (Float) + sim.update hung the test past 60s. Block 42 uses addCloth instead (proven safe). Worth investigating as a future slice.
- **`recomputeNormals()` is O(F+V) per mesh per frame**. Negligible for v1; could be optimized if scene poly count grows.

## Last decisions + why

- **D-042 R-7 entry in DECISIONS.md** — captures the recomputeNormals addition, the standing-constraint text, the legacy fallback documentation decision (no removal), Block 42's corrupt-and-restore mechanic, and the D-042 sequence closure record.
- **No new D-NNN beyond R-7 itself.** R-7 is the cleanup; no new architectural decisions.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-7 merges, **the D-042 sequence is COMPLETE**. The next Planner-tier work (when the user picks a new direction):

- **(a)** Fold `D-042-ROUND-TRIP-INVARIANT` into `docs/roles/PLANNER.md` "Standing constraints".
- **(b)** Pick the next milestone from `PROJECT_STATE.md` standing-feature candidates: Alembic export (FR-013, unblocked by Q5+Q6 resolution); source-file split (B-2.1 Bullet refinements OR refactor src/main.cpp into modules); harness-hang investigation (addCube + sim.update).

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
