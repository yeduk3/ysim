# Current Work — Rotate pack-roundtrip Slice (`feat/rotate-pack-roundtrip`)

- File in flight: none — slice complete; ready for Estimator. **35/35 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all 13 PLAN todos done.
  - **D-025 — Shape B over Shape A.** Picked the side-table re-apply path (rotateObject writes to `pendingRotations[id]`; `Simulator::initialize` auto-calls `applyPendingMaterials`; `applyPendingMaterials` rotation branch now calls `rotateObject(id, q)` to actually rotate fresh state.x). Rejected Shape A (initializer carries rotation, mirroring D-015 for translate) because Shape B is 5 lines vs Shape A's 4-class refactor.
  - **`Simulator::rotateObject`** writes `pendingRotations[meshId] = newAbs` at the end (after state.x mutation + broadPhase.refit).
  - **`Simulator::initialize`** auto-calls `applyPendingMaterials()` at the end. Existing explicit calls (after loadScene in main.cpp / runSelfTest) stay correct — they become no-ops because the maps are already cleared.
  - **`Simulator::applyPendingMaterials`** snapshots pendingRotations before the loop (rotateObject re-populates during its call, so iterating the map directly would mutate the container under the iterator), then calls `rotateObject(id, savedQuat)` for each entry. Material branch unchanged.
  - **Block 18** mechanizes the round-trip: 90deg-Z rotate cube at origin → addCube (forces re-pack) → assert state.x[0] still reflects the rotation. Pass label: `FR-004 / rotateObject survives Scene::pack rebuild`.
  - **Bug-probe verified** — skipping the `pendingRotations[meshId] = newAbs` write makes Block 18 FAIL with `post-repack state.x[0] drifted from rotated value: expected (0.25, 0.25, -0.25) got (0.25, -0.25, -0.25)` (the un-rotated witness). Restored.
  - **Estimator turn-18 WARNING fold-in** — `docs/TEST_MATRIX.md` BDD-017 row's test address gained a Block 17 cross-reference for the triangle-precision sister mechanization.
- What's tested:
  - **35/35 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - **No matrix-row promotion** — Block 18 covers FR-004 cross-cutting (rotate persistence across re-pack), not a specific BDD row.
- Non-goals respected: no initializer refactor, no `applyPendingMaterials` rename, no performance optimization, no CM-008 production-side fix, no other matrix rows.
- The "rotate pack-roundtrip" item is now **dropped from RESUME's carry-forward list** (5-slice deferral chain ended).
- What's next: Estimator review. Expect verdict at NOTE level — D-025 closes the long-deferred gap with bug-probe-verified Block 18 + matrix cross-reference fold-in.
