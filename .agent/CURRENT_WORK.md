# Current Work — Persistence Slice (feat/persistence)

- File in flight: none — Estimator BLOCK feedback addressed; ready for re-review.
- How far: 3 BLOCK items + 1 WARNING item from `.agent/ESTIMATION.md` resolved.
  - **BLOCK 1** (primitive schema drift): `docs/design/scene_format.md` updated to declare `"grid"` as v1-shipped and `"sphere"`/`"cube"` as reserved-but-not-shipped, mirroring D-003.
  - **BLOCK 2** (rotation dropped): `Quat` struct + `GeneralMesh::rotationQuat` field, `toSnapshot` reads it, `loadScene` stashes it in a `pendingRotations` side-table, `applyPendingMaterials` paints it on realized meshes. D-007 supersedes D-004.
  - **BLOCK 3** (import path resolution): `scene_format::sceneDir` and `resolveImportPath` helpers in the header; `Simulator::loadScene` joins relative imports against the scene file's directory. D-008 added.
  - **WARNING** (silent material clamping): `clampInPlace` returns `bool`; `materialFromJson` records into a `LoadWarnings` channel surfaced via `SceneSnapshot::warnings`. D-009 added.
- What's tested: 9 doctest cases / 142 assertions (was 6/124). New cases: `import paths resolve relative to scene file directory`, `sceneDir extracts directory…`, `loader emits a warning and clamps out-of-range material values`. `BDD-014/015/016` rows in `docs/TEST_MATRIX.md` still pass.
- What's next: Estimator re-review. The remaining open WARNING (no app-level Simulator end-to-end test) is genuinely blocked on having a Metal harness — surfaced in `RESUME.md` for the next planning loop.
