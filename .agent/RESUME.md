# Resume — Import Mesh UI (BDD-002) Slice

## Must remember

- **Branch:** `feat/import-mesh-ui` (off `main`, after `feat/cloth-drape` was fast-forwarded into `main`).
- **`Simulator::importMesh` is the gatekeeper for `.obj` imports** (`src/main.cpp` ~line 4290). It probes file existence with `std::ifstream(fullPath).good()` *before* calling `addFloatMesh`. **Do not bypass this guard** — `MeshFileInitializer`'s constructor calls `ObjData::loadObject` which is *graceful* on open-fail (returns with `nVertices=0` instead of crashing), so a missing-file `addFloatMesh` would silently queue a zero-vertex mesh and violate BDD-002's "no partial-add" clause.
- **Block 8 (BDD-015) explicitly calls `buildSyntheticScene(sim)` before its run.** Block 7 (BDD-002) leaves an imported `Human.obj` in the scene; if Block 8 inherits that, its save→load can't resolve `src/assets/Human.obj` relative to `/tmp/` (the persistence layer joins import paths with the scene file directory, per D-008). The reset keeps Block 8 independent of Block 7's mutation.
- **`importMesh`'s path is `prefix + "/" + fileName`** when prefix is non-empty; bare `fileName` when prefix is empty. The ImGui modal splits the user's path at the last `/` to populate prefix and fileName separately — same convention `loadScene` uses.
- **CM-005 tunneling FAIL is expected baseline noise.** `verify.sh` continues to exit non-zero because of it. The Estimator should expect a continuing BLOCK driven by CM-005, *not* by anything in this slice. The slice's BDD-002 deliverable is independent and clean.
- **Spec-vs-label discipline still applies.** Block 7's PASS strings are authored from `docs/TESTS.md#BDD-002`'s "Then" clauses verbatim, not from the matrix-row label.

## Last decisions + why

No new `DECISIONS.md` entries this slice. The path-existence guard in `importMesh` is implementation detail under D-008 (import path resolution); the `MeshRenderState` reuse for imported meshes is unchanged from D-011.

## Next step you were about to take

Slice complete. The next concrete step is the **Estimator's** turn. Expected verdict: BLOCK on the CM-005 tunneling FAIL, but with explicit acknowledgement that the BDD-002 deliverable lands clean — same shape as the cloth-drape slice's previous review.

Standing planner-tracked candidates after BDD-002:

- **Cloth-CCD slice** — closes BDD-007's tunneling clause (CM-005). Replace snapshot point-vs-triangle narrow-phase check with swept-segment-vs-triangle CCD. Has concrete localization in CM-005.
- **BDD-102 determinism mechanization** — extend the harness with two-runs-bit-identical assertions against a saved-scene baseline.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
