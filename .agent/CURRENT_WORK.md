# Current Work — FR-004 UI Rotate-Object Slice (`feat/rotate-object`)

- File in flight: none — slice complete; ready for Estimator. **31/31 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all 14 PLAN todos done.
  - **D-022** — two new Quat helpers next to D-019's family: `quatConjugate` (sign-flip imag for unit-norm inverse) and `rotateVector(q, v)` (Hamilton sandwich `q * v_pure * conjugate(q)` reusing D-019's `operator*`). ~10 lines, inline free functions.
  - **D-021** — `Simulator::rotateObject(meshId, newQuat)` mirrors D-014's translateObject API style. Computes `delta = newAbs * conjugate(currentQuat)`, rotates each particle around `mesh.transformPosition` pivot, mirrors to `state.xPrev` per D-013 invariant, writes `mesh.rotationQuat = newAbs` (normalized). `state.v` unchanged.
  - **Inspector wiring** — `MeshInspectorTarget` extended with `float* rotation_wxyz` + `std::function<void(int, float, float, float, float)> on_rotate`. Raw float pointer aliases `&mesh.rotationQuat.w` (Quat is `{w, x, y, z}` POD per D-019). `buildSelectedMeshTarget` populates both fields. `drawMeshInspectorWindow` renders a Rotation collapsing header with `InputFloat4("Quat (w,x,y,z)")` gated on `IsItemDeactivatedAfterEdit`. Raw quaternion input is intentionally minimal — Euler/axis-angle UX is a future slice.
  - **Block 15** mechanizes FR-004 UI-side via Float-tagged cube at origin, 90° Z rotation, witness vertex (0.25, -0.25, -0.25) → (0.25, 0.25, -0.25) per `(x,y,z) → (-y, x, z)` rule. Two pass labels: `FR-004 UI / rotate sets state.x rotated around pivot` + `FR-004 UI / next sim step preserves rotated state.x`.
  - **Bug-probe verified** — temporarily skipped the state.x rotation write inside `rotateObject`'s particle loop. Block 15 FAILed cleanly with `expected 90deg-Z (0.25, 0.25, -0.25) got (0.25, -0.25, -0.25)`. Restored.
- What's tested:
  - **31/31 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged (159 + 1120 assertions, both green).
  - **No matrix-row promotion** — FR-004 UI side has no dedicated BDD row; BDD-004 (already passed) covers the math/persistence side.
- Pack-roundtrip is **explicitly deferred** per the plan's Non-goals. Calling `Simulator::initialize()` after `rotateObject` (e.g., from `addCube` followed by `simulator.initialize()`) silently drops the rotate effect on state.x because re-pack rebuilds from the initializer's geometry. Same shape as the BDD-003 pre-D-015 bug. Closing it needs a Planner design call about whether `Simulator::initialize()` should auto-call `applyPendingMaterials()`. Surfaced in RESUME.
- Non-goals respected: no Euler/axis-angle UI, no inspector test mechanization, no renderer-side rotation uniform, no `state.v` rotation, no other matrix rows.
- What's next: Estimator review. Expect verdict at NOTE level — Block 15 verbatim from FR-004 + Block 9 shape, two pass labels both bug-probe-verified, deferred pack-roundtrip explicitly documented for the next slice.
