# Resume — D-042 R-4 — rotateObject writes preview.x

## Must remember

- **R-4 RETIRED `pendingRotations[meshId] = newAbs;` from rotateObject.** Bug-probe (a) confirmed that disabling the new preview rotate causes 4 test FAILs — preview write is now the SOLE mechanism preserving rotation through Scene::pack. The applyPendingMaterials re-apply path would otherwise double-rotate (preview→packed by memcpy, then re-applied by rotateObject again).
- **`loadScene`'s `pendingRotations[id] = q` stash is UNCHANGED.** That path runs once at the first post-load initialize, BEFORE preview has been rotated by user actions. Different lifecycle, no conflict.
- **primitive::cube emits +X face FIRST** → vertex 0 lives at (+h, -h, -h), not (-h, -h, -h). PLAN sketch had wrong sign; Block 39 corrected mid-build.
- **Block 39 must call `sim.initialize()` before `rotateObject`** so `findById` sees the mesh in `Scene::meshes`. Without initialize, rotateObject early-returns and preview is never touched.
- **rotateObject's preview loop uses `delta` + `pivot` already computed for state.x.** PARALLEL-IMPL-LOCKSTEP via shared rotateVector — future rotation-pivot changes need both loops updated.

## Last decisions + why

- **D-042 R-4 entry in DECISIONS.md** — captures the preview write-back placement, the `pendingRotations` retirement decision, the bug-probe (a) result confirming load-bearingness, and the Block 39 sentinel-style verification.
- **No new D-NNN beyond R-4 itself.** No BDD/FRD touched.
- **Scope expansion**: `pendingRotations` stash removal — Generator-discovered during the build (BDD-018/FR-004 regression on first attempt). Necessary because applyPendingMaterials's re-apply would double-rotate after R-3+R-4.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-4 merges, the next slice is **R-5**: add packed → preview resync at end of `Simulator::update` so the post-sim-step packed state (with cloth/rigid motion accumulated) lands in preview before the next render/edit. R-5 closes the divergence loop in the other direction (R-3 was preview→packed at pack; R-5 is packed→preview at update end).

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
