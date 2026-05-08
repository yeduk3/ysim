# Plan — Translate-Object Slice (`feat/translate-object`, closes BDD-003)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Course note: previous slice's verdict

The cloth-CCD slice's last `ESTIMATION.md` (turn 6, 2026-05-07) is **WARNING**, not BLOCK
(BLOCK list is empty; one WARNING + one NOTE). The user-facing message that
called this "block-level" is mis-labeled — confirmed by reading the file. Per
PLANNER.md procedure step 2 ("folding small WARNINGs into the next slice is
fine when both items are small"), the lone WARNING (`nparams.thickness = 0`
hardcode in `BruteForce::narrow`) is **folded into this slice as todo 11** —
it is one literal change.

The NOTE (BDD-007 ground-plane vs sphere substitution) is documented in the
prior plan's Non-goals and stays parked until the rigid-body slice unblocks
(Q4); no action this slice.

## Goal

Close `BDD-003` — let the user translate any selected object by editing its
position in the inspector, with the new position taking effect on the next
simulation step and the next rendered frame. After this slice, `BDD-003`'s
matrix row promotes from `pending` to `pass` and `runSelfTest` gains a Block 9
that mechanizes BDD-003's three "Then" clauses verbatim against
`docs/TESTS.md#BDD-003`.

## Scope

- **`BDD-003` — Translate a selected object** (acceptance per
  `docs/TESTS.md#BDD-003`):

  > **Given** an object positioned at the origin
  > **When**  the user sets its position to `(1, 2, 3)`
  > **Then**  the object's center is `(1, 2, 3)`; the next simulation step
  > uses the new position; rendering reflects the new position on the next
  > frame.

  All three "Then" clauses are in scope: center reads back, the simulator's
  next-step state begins from the new center, and the renderer reads the new
  position. (See todo 8 — assertions are authored from this verbatim, not
  from the matrix-row label.)

- **`FR-003`** — inspector field edit (the gizmo-drag option from BDD-003's
  Notes is **out of scope**; see Non-goals).

- **New `GeneralMesh::transformPosition` field** — parallel to D-007's
  `rotationQuat`. Initialized at pack-time from the initializer's center /
  offset (whichever is the source of truth for the corresponding shape) so
  *existing* meshes' centers are preserved through the rename.

- **New `Simulator::translateObject(int meshId, tinym::vec3 newPos)`** — the
  one place that mutates `transformPosition`. Computes
  `delta = newPos - mesh.transformPosition`, applies it once across
  `state.x` (and `state.xPrev`, see todo 5), updates `transformPosition`,
  marks the renderer dirty for that mesh id, marks scene dirty.

- **Inspector wiring** — `MeshInspectorTarget` gains
  `tinym::vec3* transform_position`; `drawMeshInspectorWindow` gets an
  `ImGui::InputFloat3("Position", …)` row; the `IsItemDeactivatedAfterEdit`
  pattern (matches existing color-edit flow in
  `src/mesh_inspector_gui.cpp`) calls `Simulator::translateObject(id, …)`
  on commit, not per-character.

- **Persistence round-trip update** — `Simulator::toSnapshot` reads
  `mesh.transformPosition` for the realized-meshes path
  (`src/main.cpp:4780/4787/4794/4802`) instead of the initializer's
  center/offset, so a translate→save→reload→translate-again cycle round-trips
  through the new field. The pending-requests path (lines ~4844) keeps reading
  the initializer because no `GeneralMesh` exists yet — that's correct.

- **`runSelfTest` Block 9** — exercises BDD-003 end-to-end (translate a
  freshly-created mesh; assert center; one `sim.update()`; assert state.x
  reflects the new center; check that `MeshRenderState`'s next-frame
  positions reflect the move).

- **Folded WARNING from cloth-CCD turn 6** — replace
  `nparams.thickness = 0; // temp.` (`src/main.cpp:4049`) with a real value
  so the swept-CCD slow-touch fallback uses `radius + thickness`, not just
  `radius`. See todo 11 for the resolution.

## Non-goals (this slice)

- **Transform gizmo** (BDD-003 Notes line: "cover both inspector field edit
  and gizmo drag if the gizmo is in v1 scope"). The gizmo is **not** in v1
  scope; the inspector field path is the v1 contract. Defer.
- **Rotation editing** — `BDD-004` is its own slice; `rotationQuat` already
  exists from D-007 but is not consumer-wired yet. Don't add a rotation edit
  field to the inspector here; that's BDD-004's slice.
- **Scaling** — not in `BDD.md` for v1.
- **Parent/child transform hierarchies** — out of v1.
- **Renderer model-matrix refactor** — v1 renders from `state.x` directly
  (positions are world-space). The translate path mutates `state.x`
  (and `state.xPrev`) in place; do **not** introduce a per-mesh model matrix
  in this slice — that is a renderer rework that would dwarf BDD-003.
- **Resolving any of `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**
- **Behavior-change-on-translate semantics** — translating doesn't reset
  velocities, restart the simulator, or invalidate constraints. The
  "next simulation step uses the new position" clause is satisfied by
  `state.x` already reflecting the move; `state.v` carries forward
  unchanged. (Note in DECISIONS as the load-bearing interpretation.)

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/translate-object` (off `main` at
   `a85aba8`, the doc-maintenance pass). No new branch needed; this is an
   `add:` slice. Confirm `git status` is clean before editing.

2. **Re-read the binding "Then" clauses.** Open `docs/TESTS.md#BDD-003`
   (lines 39–45) and `docs/specs/FRD.md#FR-003` (lines 43–50). The Block 9
   assertions in todo 8 are authored from these clauses verbatim, not from
   the matrix-row label "Translate a selected object" — that's the
   spec-vs-label trap codified in `docs/roles/GENERATOR.md` step 3.

3. **Add `tinym::vec3 transformPosition` to `GeneralMesh`** (`src/main.cpp`
   ~line 1557, next to `rotationQuat`). Default-initialize to `vec3(0)`.
   Update the move constructor (~line 1568) to copy
   `transformPosition(other.transformPosition)`. **D-007 is the precedent
   pattern** — follow it exactly so a future maintainer reading
   `rotationQuat` finds `transformPosition` next to it without surprise.

4. **Seed `transformPosition` at pack-time.** In `Scene::pack()` (around
   the existing realized-mesh population, search for `meshes.emplace_back`
   or `numMeshes++`), set `m.transformPosition` from whichever initializer
   subtype was used:

   - `MeshGridInitializer<BE,PR>* g` → `g->params.center`
   - `MeshSphereInitializer<BE,PR>* sp` → `sp->params.center`
   - `MeshCubeInitializer<BE,PR>* cb` → `cb->params.center`
   - `MeshFileInitializer<BE,PR>* f` → `f->params.offset`
   - any other (ground, etc.) → `vec3(0)`

   This mirrors the dynamic_cast cascade already in `toSnapshot`
   (`src/main.cpp:4772-4803`). A small helper
   `tinym::vec3 initializerCenter(GeneralMeshInitializer<BE,PR>*)` is
   acceptable but optional — inline cascade is fine for v1.

5. **Implement `Simulator::translateObject(int meshId, tinym::vec3 newPos)`**.
   Locate `Scene<BE,PR>::findById` (already exists; used in
   `buildSelectedMeshTarget`). New method on `Simulator` (alongside
   `addSphere` / `importMesh`):

   ```cpp
   void translateObject(int meshId, tinym::vec3 newPos) {
       auto* mesh = scene.findById(meshId);
       if (!mesh) return;
       tinym::vec3 delta = newPos - mesh->transformPosition;
       const Index n = mesh->state.x.size / 3;
       for (Index i = 0; i < n; ++i) {
           mesh->state.x.ptr[i*3+0] += delta.x;
           mesh->state.x.ptr[i*3+1] += delta.y;
           mesh->state.x.ptr[i*3+2] += delta.z;
           mesh->state.xPrev.ptr[i*3+0] += delta.x;
           mesh->state.xPrev.ptr[i*3+1] += delta.y;
           mesh->state.xPrev.ptr[i*3+2] += delta.z;
       }
       mesh->transformPosition = newPos;
       renderState.markDirty(meshId);  // or whatever the existing MeshRenderState invalidation is
       scene.dirty = true;
   }
   ```

   **`xPrev` must move with `x`** — D-013's swept-CCD reads `xPrev[id]` as
   the start-of-substep position. If we move `x` without moving `xPrev`,
   the next narrow-phase will see a swept segment from the *old* center
   to the *new* center and fire spurious contacts (or worse, treat the
   teleport as tunneling). This is the load-bearing subtlety; record it
   in DECISIONS.

   `state.v` is **unchanged** — translating doesn't reset velocity. (If
   the user reports "moving the object adds a kick", revisit, but per
   BDD-003 wording the next step "uses the new position", not "resets
   physics".)

6. **Extend `MeshInspectorTarget`.** Add to
   `include/MeshInspectorWindow.hpp`:

   ```cpp
   struct MeshInspectorTarget {
       int mesh_id = -1;
       const char* behavior_label = nullptr;
       const char* shape_label = nullptr;
       tinym::vec3* base_color = nullptr;
       tinym::vec3* transform_position = nullptr;     // NEW
       std::function<void(int, tinym::vec3)> on_translate;  // NEW
   };
   ```

   The callback indirection avoids `<MeshInspectorWindow.hpp>` needing to
   know about `Simulator`. Populate both in `buildSelectedMeshTarget`
   (`src/main.cpp:5974-5983`):

   ```cpp
   target.transform_position = &selectedMesh->transformPosition;
   target.on_translate = [&simulator](int id, tinym::vec3 v) {
       simulator.translateObject(id, v);
   };
   ```

7. **Render the inspector control.** In `src/mesh_inspector_gui.cpp`'s
   `drawMeshInspectorWindow`, after the existing color row, add:

   ```cpp
   if (target.transform_position && target.on_translate) {
       float p[3] = { target.transform_position->x,
                      target.transform_position->y,
                      target.transform_position->z };
       ImGui::InputFloat3("Position", p);
       if (ImGui::IsItemDeactivatedAfterEdit()) {
           target.on_translate(target.mesh_id,
                               tinym::vec3(p[0], p[1], p[2]));
       }
   }
   ```

   `IsItemDeactivatedAfterEdit` matches the pattern used elsewhere for
   color (one commit per finished edit, not per keystroke). If the
   existing color-edit code uses a different debounce idiom (e.g. live
   `ColorEdit3`), match that instead — the goal is consistency with the
   established inspector pattern, whichever one it is.

8. **Block 9 in `runSelfTest`.** Append after Block 8 (currently the last
   block) in the same file. Reset to a known scene
   (`resetScene(); sim.addCloth(...)` at origin or
   `sim.addSphere(vec3(0), …)` — whichever is cheaper to test against).
   Then mechanize the three "Then" clauses **verbatim** from
   `docs/TESTS.md#BDD-003`:

   - **Clause (a) — "the object's center is `(1, 2, 3)`":**
     `sim.translateObject(meshId, vec3(1,2,3));`
     assert `findById(meshId)->transformPosition` equals `(1,2,3)` within
     1e-6, AND that the per-axis mean of `state.x` equals `(1,2,3)` within
     a tolerance proportional to mesh extent (mean of `state.x` = original
     mean + delta, and original mean of a centered cloth is the center).
     **Pass label:** `BDD-003 / object's center is (1, 2, 3)`.

   - **Clause (b) — "the next simulation step uses the new position":**
     Capture `state.x[0]` (some witness vertex), call `sim.update()` once,
     then assert that the post-update position is consistent with starting
     from the translated position (e.g. for a Float-tagged mesh,
     post-step ≈ pre-step ± O(dt·gravity); for cloth, post-step.x and
     post-step.z should be within a small tolerance of the translated x/z
     since gravity is along -y). The simplest witness:
     `assert(post_update_state.x[0] ≈ translated_x[0])` to within
     `dt * |gravity| * 5` (slack for gravity on y, near-zero on x/z).
     **Pass label:** `BDD-003 / next simulation step uses the new position`.

   - **Clause (c) — "rendering reflects the new position on the next
     frame":** The renderer reads from `state.x` (or from
     `MeshRenderState`'s shadow if it caches). The headless harness has
     no actual rendering, so the testable proxy is: after
     `sim.translateObject(...)`, the data the renderer **would** read
     reflects the move. Concretely, assert that
     `MeshRenderState::getOrCreate(meshId)`'s position-buffer-source
     pointer aligns with `state.x.ptr` (the renderer reads through this
     handle), AND that `state.x.ptr` already has the translated values.
     If `MeshRenderState` doesn't expose such a query yet, the proxy
     check is just "post-translate `state.x` reflects the delta"
     (todo 7's path — same as clause (a)'s state.x check) — call out in
     DECISIONS that headless coverage of clause (c) is a state-x
     correctness check, not a frame-render check, and graduates when a
     pixel-render harness exists.
     **Pass label:** `BDD-003 / rendering reflects the new position on the next frame`.

   Three new PASS lines; total goes from 16 to 19.

9. **Update `Simulator::toSnapshot` to read `mesh.transformPosition`.**
   `src/main.cpp:4772-4803`'s realized-mesh path: replace each
   `o.transform.position = {init->params.center.x, ...}` with
   `o.transform.position = {m.transformPosition.x,
                            m.transformPosition.y,
                            m.transformPosition.z}`.
   The `encodeOne` lambda already takes `m` indirectly via the
   `meshes` loop (`src/main.cpp:4832-4837`); pass `m.transformPosition`
   in or capture `m` differently — the cleanest path is adding a
   `tinym::vec3 transformPosition` parameter to `encodeOne` next to
   the existing `m.rotationQuat`-mirroring `defaultRot`.
   The pending-requests path (`src/main.cpp:4839-4847`) is unchanged
   — still reads from initializer params, since no `GeneralMesh` exists
   for those yet.

10. **Update `loadScene` to seed `transformPosition` after construction.**
    `src/main.cpp` ~line 4909 reads
    `o.transform.position` into `tinym::vec3 pos` and currently feeds it
    to the initializer's `center`/`offset`. Keep that; **also** carry
    `pos` forward and set `mesh.transformPosition = pos` after
    `Scene::pack()` runs (or via a `pendingTransformPositions` side-table
    mirroring `pendingRotations`'s pattern — that's the D-007 idiom).
    Without this, a save→load cycle would reset `transformPosition` to 0
    while `state.x` retains the position, and the next translate-edit
    would compute a wrong delta.

11. **Folded WARNING fix — `nparams.thickness = 0` in `BruteForce::narrow`.**
    `src/main.cpp:4049`. The cloth-CCD slice's plan called for
    `radius + thickness` as the slow-touch fallback band; the value is
    currently 0 because the kernel runs across all cloth meshes globally
    while `thickness` is per-cloth. Fix direction: source it from
    `Simulator::margin` (`src/main.cpp:4290` — already plays the contact-
    band role for the simulator) so the fallback band becomes
    `radius + margin`. That preserves the slice plan's semantics without
    plumbing per-mesh thickness through the kernel signature this turn.
    One literal edit:
    `nparams.thickness = 0; // temp.` →
    `nparams.thickness = static_cast<float>(simulator.margin);`
    (resolve the access path — likely `narrow` doesn't currently see
    `Simulator`; if so, add a `PR margin` parameter to `narrow(...)`
    and pass `simulator.margin` from the call site, or read from a
    Scene-level field).
    Drop the `// temp.` comment.

12. **Verify locally.** `./scripts/verify-light.sh` after each phase
    (after todo 5 once `translateObject` compiles, after todo 8 once
    Block 9 compiles, etc.). Don't batch up failures.

13. **Promote `BDD-003` matrix row.** `docs/TEST_MATRIX.md:17`:
    test address column → `src/main.cpp::runSelfTest::BDD-003 (Block 9)`,
    status → `pass`. (The Estimator confirms the status — Generator records
    it per role doc.)

14. **DECISIONS / CURRENT_WORK / RESUME.** New numbered entry in
    `docs/DECISIONS.md`:
    > **D-NNN — Translate-on-render mutates `state.x` (and `state.xPrev`)
    > rather than introducing a model matrix.** v1 renders directly from
    > `state.x`; a per-mesh model matrix would force a renderer rework
    > that's much larger than BDD-003. The translate path bakes the new
    > position into `state.x` once on commit; physics carries forward
    > unchanged from there. `state.xPrev` is moved by the same delta so
    > D-013's swept CCD doesn't see the teleport as a contact event.
    > `state.v` is **not** reset.

    Update `.agent/CURRENT_WORK.md` four-line max as you go (Generator role
    step 6); write `.agent/RESUME.md` near end of turn (role step 7).

15. **Stop and hand off to the Estimator.** Don't touch the gizmo, don't
    touch rotation editing, don't refactor the renderer.

## Course corrections

- **Spec-vs-label discipline (codified in `docs/roles/GENERATOR.md` step 3
  during the maintenance pass).** Block 9's assertions are authored from
  `docs/TESTS.md#BDD-003`'s "Then" clauses verbatim. The matrix row's
  compressed label "Translate a selected object" is **not** the spec; using
  it as the source of assertions is what BLOCKed the harness slice in
  turn 2. See PROJECT_STATE's "2026-05-07 harness slice fix turn" entry
  for the prior incident.
- **`xPrev` parity with `x` is load-bearing.** D-013 (cloth-CCD slice)
  reads `xPrev` as the start-of-substep position for swept-segment-vs-
  triangle CCD. Any code path that mutates `state.x` outside the
  integrator must also mutate `state.xPrev` by the same delta, otherwise
  the next narrow phase will treat the user-driven move as tunneling and
  emit spurious contacts. See `.agent/RESUME.md` (cloth-CCD slice) for
  the full discussion.
- **D-007 is the precedent for `transformPosition`.** D-007 added
  `rotationQuat` as a real `GeneralMesh` field after persistence's
  initial JSON-only design failed round-trip. The same shape applies
  here: in-memory mirror is the missing piece for runtime editing.
  Don't re-debate "should this live on the initializer or on the mesh?"
  — D-007 already settled it for the mesh.
- **Render decoupling (D-011).** `MeshRenderState` is keyed by mesh id,
  so `markDirty(meshId)` (or whatever the equivalent invalidator is)
  is the right hook in `translateObject`. Don't reach into per-mesh GL
  handles.
- **Folded WARNING is small.** The `nparams.thickness = 0` fix is a
  one-line edit per the maintenance-pass-codified small-WARNING rule
  (PLANNER.md procedure step 2). If the Generator finds during
  implementation that it requires more plumbing (e.g. adding a
  `margin` parameter to `BruteForce::narrow`), keep the change tightly
  scoped — do **not** refactor the kernel signature beyond what this
  one-line semantic change requires.

## What to read before writing code

- `docs/TESTS.md#BDD-003` (lines 39–45) — binding "Then" clauses, verbatim.
- `docs/specs/FRD.md#FR-003` (lines 43–50) — functional contract.
- `docs/specs/BDD.md#BDD-003` (lines 39–45) — user-story framing.
- `docs/specs/BDD.md#BDD-103` (line 209) — backend-boundary invariant; this
  slice mutates `state.x` but does not change the kernel ABI, so should
  not violate BDD-103.
- `docs/DECISIONS.md` — D-007 (rotation field, the precedent), D-011
  (`MeshRenderState` keyed by id), D-013 (`xPrev` for swept CCD).
- `docs/mistakes/COMMON_MISTAKES.md` — any active entries before touching
  the area named there. (CM-005 is graduated to OLD_MISTAKES.)
- `include/MeshInspectorWindow.hpp` and `src/mesh_inspector_gui.cpp` —
  inspector struct + draw function; the existing `base_color` flow is the
  pattern to mirror for `transform_position`.
- `src/main.cpp::GeneralMesh` (~line 1546), `Simulator` struct (~line 4255),
  `Simulator::toSnapshot` (~line 4757), `loadScene` (~line 4857),
  `BruteForce::narrow` (~line 4034), `runSelfTest` end-of-file (last block
  is around line 5750+).
- `.agent/RESUME.md` (cloth-CCD slice) — `xPrev` parity discussion;
  load-bearing for todo 5.
- `.agent/ESTIMATION.md` — turn 6 verdict (WARNING) being folded as todo 11.
