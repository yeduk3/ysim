# Resume — FR-005 material-edit data-layer slice (D-027 lands; BDD-005 data-layer closed, render-side parked)

## Must remember

- **Branch:** `feat/material-inspector-data-layer` (off `main` at `de83164`).
- **D-027 — `Simulator::setMaterial(meshId, Material)` is the canonical edit-time material mutator.** Writes `mesh->material = mat` AND `pendingMaterials[meshId] = mat`. Mirrors D-014 (translateObject) and D-021 (rotateObject) shape; re-pack survival is free via D-025's auto-applyPendingMaterials.
- **Material edit flow has dual writes by design.** ImGui widgets mutate the pointer-targeted fields in place for live preview (renderer reads `mesh.material.baseColor` every frame). On every change the callback fires `setMaterial` which redundantly re-writes `mesh->material` AND newly writes `pendingMaterials[id]`. The double `mesh->material` write is idempotent — the load-bearing write is `pendingMaterials` for re-pack survival. Don't try to "optimize" by skipping one of the writes.
- **Future material consumers (PBR preview shader, v2 LLM control surface, presets):** read `mesh.material` for live state; write via `Simulator::setMaterial` for any edit that should persist. Do not mutate `mesh.material` directly — the live-state write works for the current frame but is lost on the next re-pack unless `pendingMaterials` is also populated. The inspector's `ColorEdit3`/`SliderFloat` in-place writes are a deliberate exception: live preview AND the callback fires on every change, so the side-table catches up immediately.
- **Callback signature uses 5 primitive args** (vec3 + 3 floats + vec3), not the `Material` struct directly, to avoid coupling `MeshInspectorWindow.hpp` to main.cpp's `Material` type. Same pattern as `on_rotate` (4 floats for w/x/y/z).
- **No `IsItemDeactivatedAfterEdit` gate on material widgets.** Per-frame firing during drag is fine because material edits are CPU memcpy + map insert, unlike D-014's translate which triggers `broadPhase.refit()`. If you ever change `setMaterial` to do BVH work, add the commit-on-release gate.
- **BDD-005 matrix row is `warning`, not `pass`.** The PBR-preview-shader clause from TESTS.md#BDD-005 is the parked render-side gap. Promoting to `pass` requires the PBR-preview-shader slice.
- **Block 20 Phase 4 re-calls `setMaterial`** after the save+load round-trip (Phase 3 already populates pendingMaterials via loadScene, but Phase 4's intent is to exercise the edit-time path explicitly — the re-call is part of the design, not redundancy).

## Last decisions + why

- **D-027 — Shape B (mutator + pendingMaterials write-back) over Shape A (direct pointer mutation only).** Shape A has no public test-callable API surface — Block 20 would have to mutate `mesh.material` directly (which the user-facing flow never does) or run ImGui from the harness (which the harness can't do). Mutator gives a clean BDD-005 test address that mirrors BDD-003 (translate) and BDD-004 (rotate). Re-pack survival is free via D-025.
- **No validation/clamping in `setMaterial`.** Slider widget range (0..1) is the inspector-side clamp. JSON load path intentionally does not clamp — save files may carry hand-edited values for debugging or v2 LLM control. Aligns with D-005's "data layer is the authority" stance.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **40/40** self-test PASS lines (Block 20 added 4; previous count was 36). Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md` (FR-005 data-layer is **off this list now**):

- **PBR preview shader slice (FR-005 renderer-side)** — completes BDD-005 by making the preview render respond to all 5 material parameters (currently consumes only baseColor at `src/main.cpp:4902`). Promotes BDD-005 row `warning → pass`. The data-layer side D-027 lands first; the shader work is the bulk.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes. Manual-test-only mechanization.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 + D-024 + D-025 + D-026 + D-027 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.
- **Role-doc maintenance pass** — `docs/roles/GENERATOR.md` stale-harness-gotcha entry about `objTrees.clear()` (CM-008 graduated; the gotcha is now wrong) still needs cleanup.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
