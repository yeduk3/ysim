# Current Work — Import Mesh UI (BDD-002) Slice (feat/import-mesh-ui)

- File in flight: none — slice complete; ready for Estimator. BDD-002 closes cleanly with 3 new PASS lines on top of the existing harness.
- How far: all 9 PLAN.md todos done.
  - `Simulator::importMesh(prefix, fileName, scale, mass, error*)` added with a **path-existence guard** before `addFloatMesh`. Without the guard, `MeshFileInitializer`'s constructor would silently load an empty `ObjData` (loadObject is graceful on open-fail) and queue a zero-vertex mesh, violating BDD-002's "no partial-add" clause.
  - ImGui `File > Import Mesh…` modal beside Save/Load — text-input for path + scale slider + Float-only behavior caption. Calls `importMesh` then `simulator.initialize()` + `applyPendingMaterials()`. Status flows into the existing `sceneIOStatus` panel.
  - Block 7 in `runSelfTest` mechanizes BDD-002's two "Then" clauses verbatim from `docs/TESTS.md#BDD-002` — happy path (numMeshes++ + Float behavior + path round-trip via `toSnapshot`) and error path (missing file → numMeshes unchanged + error names "not found").
  - Block 8 (was Block 7, BDD-015) now resets the scene via `buildSyntheticScene(sim)` first — Block 7 leaves an imported `Human.obj` whose path doesn't resolve when the temp scene file is in `/tmp/`. Independent block, independent setup.
- What's tested:
  - 14 of 15 self-test assertions PASS. The one FAIL is the pre-existing BDD-007 tunneling clause (CM-005 — parked under cloth-CCD slice). All other clauses including the new BDD-002 trio pass cleanly.
  - Doctest binaries unchanged.
  - `docs/TEST_MATRIX.md` row `BDD-002` promoted from `pending` to `pass` with addresses pointing at the new Block 7 strings.
- What's next: Estimator review. Expect `verify.sh` to continue exiting non-zero because of the BDD-007 tunneling FAIL (CM-005), but the slice's deliverable — BDD-002 acceptance — is complete and verified.
