# PLAN — BDD-018 inspector live-edit propagation (`feat/bdd-018-inspector-live-edit`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-13

## Course note: previous slice's verdict

Estimator turn 27 returned **WARNING** on the fbo-render-harness slice
(D-032 + D-033). One WARNING item (shader-load skip-safety in
`Program::loadShader`) + one NOTE item (`HiddenGLContext` destructor not
calling `glfwTerminate()` on early-init failure). Per the user's slice
brief, **neither is folded into this slice** — both are in harness-glue
areas unrelated to inspector live-edit propagation; the load-shader
WARNING is queued for a future small `fix/loadshader-skip-safety` slice
and the destructor NOTE travels with it.

## Goal

Close `BDD-018` (currently `pending` in `docs/TEST_MATRIX.md` row 32) by
mechanizing the "Inspector edits propagate live" invariant in the
harness. Production wiring at `src/main.cpp:8349-8387` already builds a
`mesh_inspector::MeshInspectorTarget` per frame and routes the three
implemented edit paths — color/material (D-027 `Simulator::setMaterial`),
translate (D-014 `Simulator::translateObject`), rotate
(D-021 `Simulator::rotateObject`) — into the simulator. Before this slice
the BDD has zero harness coverage; the user's manual GUI iteration is
the only signal. Block 26 in `runSelfTest` adds an assertion that (a)
the callback shape production uses, when fed a synthetic edit, fires
exactly one Simulator setter call with the expected value; (b) the
change is observable to the next-frame render (in-place mutation through
the pointer fields the renderer dereferences every frame); (c) the next
`sim.update()` does not require a `simulator.initialize()` between
callback fire and pump.

Pure mechanization slice; no new D-NNN, no new CM-NNN, no new BDD/FR.

## Scope

**Mechanization shape: option (b) from the slice brief.** The harness
builds a `mesh_inspector::MeshInspectorTarget` *exactly the way
production does* at `src/main.cpp:8349-8387` — same callback lambda
shape, same `Simulator` setter calls inside. The harness then invokes
the callbacks directly with synthetic edit values (instead of routing
through ImGui widget events) and asserts the propagation invariants.
This shape mechanizes the seam between inspector UI and Simulator
without introducing an ImGui-side test harness; ImGui itself stays
upstream of the seam and is covered by the user's manual visual gate.

Why not option (a) — synthesize ImGui widget events programmatically:
brittle (requires `ImGui::SetKeyboardFocusHere` + manual `InputText`
event injection or an `imgui-test` dependency) and exercises the widget
layer that's already user-validated. The load-bearing piece is the
callback → Simulator → in-place-pointer-mutation propagation, not ImGui
itself.

Why not option (c) — extract a new `InspectorBindings` struct parallel
to `MeshInspectorTarget`: the existing struct already serves the harness
unchanged; a parallel struct would be redundant.

**NEW symbols this slice adds** (none modify existing signatures):

- `src/main.cpp::runSelfTest` Block 26 (new lines after Block 25's
  closing brace, before the failure-counter summary).
- Three lambda definitions inside Block 26 mirroring the production
  lambdas at `src/main.cpp:8357-8384`, plus a small set of
  capture-by-reference counters (`int materialCalls`, `int
  translateCalls`, `int rotateCalls`) so the assertions can be
  *stricter* than the BDD's literal wording (per PLANNER.md procedure
  step 7 — "exactly one setter call with the expected value" catches
  double-fire bugs that "value propagated somewhere" would mask).

**PRESERVED symbols** (this slice MUST NOT modify any of these):

- `include/MeshInspectorWindow.hpp` — `MeshInspectorTarget` struct, all
  9 fields + 3 callback signatures. No widening.
- `src/mesh_inspector_gui.cpp` — `drawMeshInspectorWindow` function
  body, all 5 widget rows.
- `src/main.cpp:8349-8387` — production `buildSelectedMeshTarget`
  lambda. The harness replicates the shape; it does NOT call this
  lambda or extract a shared helper.
- `Simulator::setMaterial` (D-027), `Simulator::translateObject`
  (D-014), `Simulator::rotateObject` (D-021) — all signatures and
  bodies unchanged.
- `runSelfTest` Blocks 1 through 25 — Block 26 is appended at the end
  of the block sequence.

**Spec substitution** (documented per PLANNER.md spec-substitution
discipline):

- `BDD-018`'s "Then ... if the behavior changed, the next simulation
  step dispatches through the new behavior" clause is **parked**.
  Behavior-tag inspector edits do not yet exist in
  `MeshInspectorTarget` because in-place behavior switching is
  `BDD-006` / `FR-006` territory, which is `Q2`-blocked (cloth UX
  surface — one option vs two). When `BDD-006` ships, the behavior-tag
  clause returns to scope. Documented in the matrix row + PROJECT_STATE.
- `BDD-018`'s "color or behavior tag" wording is read as "color
  (FR-005/D-005 OpenPBR 5-tuple) is one named inspector edit
  category." The harness adds translate (D-014) and rotate (D-021) as
  additional stricter clauses — both are inspector edits implemented
  today and both must propagate live for the spirit of FR-018 to hold.
  This is a stricter-than-spec scope expansion per PLANNER.md step 7
  (additional regression-catching surface).
- `BDD-018`'s "visible in the very next rendered frame" wording is
  mechanized as "the value the renderer reads each frame is updated in
  place by the time the callback returns." The renderer reads
  `mesh.material.*` and `state.x` every frame; in-place pointer
  mutation through `MeshInspectorTarget`'s exposed pointers is the
  production discipline. **No FBO render** — that's D-032's territory
  and only needed when the renderer's *output pixels* are the
  load-bearing claim, not the *input data*.
- `BDD-018`'s "must not require pause/resume of the simulation"
  notes-line is mechanized as: do not call `simulator.initialize()`
  between the callback fire and the subsequent `sim.update()`. The
  harness pumps a frame, fires the callback, pumps another frame
  without re-initialize, and asserts the second frame ran without
  crash and continued from where it left off.

## Non-goals

- **NO new `MeshInspectorTarget` fields.** The parallel-symbol rule
  (per `~/.claude/projects/-Users-gyu-codes-ysim/memory/feedback_make_means_add_new.md`)
  applies in its softer form here: the user used "Mechanize" (not a
  creation verb), but a pure harness slice should not need to widen
  the inspector struct anyway. If the Generator finds an unavoidable
  widening, STOP and hand back.
- **NO ImGui event synthesis** — option (a) rejected.
- **NO new parallel struct** — option (c) rejected.
- **NO behavior-tag inspector edit** — spec-substituted out per
  BDD-006/Q2 blocked.
- **NO folding of turn-27 WARNING on `Program::loadShader`.** User's
  slice brief explicitly says: "that WARNING is in unrelated harness
  glue and should NOT be folded into this slice; queue it as a future
  small fix slice."
- **NO folding of turn-27 NOTE on `HiddenGLContext` destructor** —
  same reasoning. Travels with the loadshader fix slice.
- **NO new `D-NNN`** — pure mechanization, no architectural decision.
- **NO new `CM-NNN`** — production wiring is already correct.
- **NO source-file split** (still user-deferred per the
  bvh-refit-bench slice brief).
- **NO updates to** `docs/specs/PRD.md` / `FRD.md` / `BDD.md` /
  `docs/TESTS.md` — the BDD wording stays as-is; the substitution is
  documented in PLAN.md (this file) + matrix row note + pass labels,
  not in the spec.

## Todo

1. **Branch hygiene.** Already on `feat/bdd-018-inspector-live-edit`
   (off `main` at `01092b9`). Commit prefix: `add:` (closes a
   `pending` BDD).
2. **Matrix row placeholder.** Update `docs/TEST_MATRIX.md` row
   `BDD-018` test address column from blank to a placeholder string
   (e.g., `pending — Block 26 in progress`) so the Estimator sees the
   Generator has the row in flight. Status column stays `pending`
   until step 6 below.
3. **Author Block 26 in `src/main.cpp::runSelfTest`** at the end of
   the block sequence, before the failure-counter summary lines (look
   for the `BDD-005 / FBO PBR render` pass label as anchor — Block 26
   goes right after Block 25's closing brace). Block 26 structure:
    1. Reset scene. Add a single `Float` cube via `addCube` (Float so
       `setMaterial` / `translateObject` / `rotateObject` all apply
       cleanly — material is universal, translate/rotate are pose
       edits and Float behavior pins state.x against gravity for clean
       per-vertex assertions). Note `int meshId = sim.scene.numMeshes
       - 1`. `sim.initialize()`.
    2. Pump 1 frame via `sim.update()` to establish a baseline
       `state.x` snapshot. Record `tinym::vec3 baselinePos =
       sim.scene.meshes[0].state.x[someVertexIndex]` (pick a
       non-origin vertex like vertex 0 of a unit cube at
       `(-0.5,-0.5,-0.5)`).
    3. Build a `mesh_inspector::MeshInspectorTarget target` matching
       the production shape at `src/main.cpp:8349-8387` *exactly* —
       same field assignments, same lambda bodies. Capture three
       counters by reference: `int materialCalls = 0; int
       translateCalls = 0; int rotateCalls = 0;`. Each lambda
       increments its counter before calling the Simulator setter.
    4. **Material clause:** Invoke
       `target.on_material_edit(meshId, tinym::vec3(1,0,0), 0.5f,
       0.25f, 0.7f, tinym::vec3(0,0,0))` — synthetic edit setting
       `baseColor=red`, `metallic=0.5`, `roughness=0.25`,
       `specularWeight=0.7`, `emission=zero`. Assert (stricter than
       spec literal):
        - `materialCalls == 1` (exactly one callback fire, no
          double-call).
        - `sim.scene.meshes[0].material.baseColor.r == 1.0f` AND
          `.g == 0.0f` AND `.b == 0.0f` (live update through the
          production path).
        - `sim.scene.meshes[0].material.metallic == 0.5f` AND
          `.roughness == 0.25f` AND `.specularWeight == 0.7f` (all 5
          D-005 fields propagated).
        - `*target.base_color == sim.scene.meshes[0].material.baseColor`
          (pointer aliasing — the renderer's read-source is the same
          memory the harness writes through).
        - `pump sim.update()` once WITHOUT `sim.initialize()` between.
          Assert `sim.scene.meshes[0].material.baseColor.r == 1.0f`
          still holds (BDD-018 "must not require pause/resume" —
          sim-step survival without re-init).
        - Pass label: `BDD-018 / material inspector edit propagates
          live (color + metallic/roughness/specular/emission + no-restart
          sim step)`.
    5. **Translate clause:** Invoke `target.on_translate(meshId,
       tinym::vec3(1.0f, 2.0f, 3.0f))`. Assert:
        - `translateCalls == 1`.
        - `sim.scene.meshes[0].transformPosition.x == 1.0f` AND
          `.y == 2.0f` AND `.z == 3.0f` (live update).
        - `sim.scene.meshes[0].state.x[someVertexIndex] ≈
          baselinePos + tinym::vec3(1,2,3)` within `1e-5` (state.x
          shifted by the delta — D-014's three-site-cascade invariant
          under live edit). Float pins state.x so the post-baseline
          pump in step 3.4 should not have drifted; the Generator
          should re-snapshot baselinePos right before this clause if
          needed.
        - `pump sim.update()` once WITHOUT `sim.initialize()`. Assert
          state.x[someVertexIndex] still reflects the translated
          position (no snap-back).
        - Pass label: `BDD-018 / translate inspector edit propagates
          live (transformPosition + state.x + no-restart sim step)`.
    6. **Rotate clause:** Build a 90°-Z quat `q` via D-019's
       `quatNormalize` family (`q = {cos(45°), 0, 0, sin(45°)}` for
       90° around z; unit-norm). Invoke `target.on_rotate(meshId, q.w,
       q.x, q.y, q.z)`. Assert:
        - `rotateCalls == 1`.
        - `sim.scene.meshes[0].rotationQuat.w ≈ q.w` (and x/y/z
          within `1e-5`).
        - For a witness vertex with known pre-rotate offset from
          `transformPosition`, post-rotate offset matches the
          90°-Z-rotated offset within `1e-5`. The cube was translated
          to `transformPosition = (1,2,3)`; vertex 0 was at
          `(-0.5,-0.5,-0.5)` in object space, so its world pre-rotate
          position is `(1,2,3) + (-0.5,-0.5,-0.5) = (0.5,1.5,2.5)`.
          90°-Z rotation maps `(dx,dy,dz)` to `(-dy,dx,dz)`, so the
          expected post-rotate offset is `(0.5,-0.5,-0.5)` and the
          expected world position is `(1.5,1.5,2.5)`. Hard-coded
          witness math avoids the "use D-022 to verify D-022"
          tautology — the harness computes the expected by hand, not
          by calling `rotateVector`.
        - `pump sim.update()` once WITHOUT `sim.initialize()`. Assert
          rotation persists.
        - Pass label: `BDD-018 / rotate inspector edit propagates live
          (rotationQuat + state.x + no-restart sim step)`.
    7. Three pass-label increments → Block 26 contributes 3 new PASS
       lines. Self-test count goes 45 → 48.
4. **Bug-probe each clause** (per GENERATOR.md bug-probe discipline).
   For each clause:
    1. **Material probe:** comment out the `simulator.setMaterial(id,
       m);` call inside `target.on_material_edit`'s lambda body so the
       callback fires but the write doesn't happen. Block 26's
       material clause should FAIL with `expected baseColor.r == 1.0,
       got <baseline>`. Restore.
    2. **Translate probe:** comment out the
       `simulator.translateObject(id, v);` call inside
       `target.on_translate`'s lambda. Block 26's translate clause
       should FAIL. Restore.
    3. **Rotate probe:** comment out the `simulator.rotateObject(id,
       q);` call inside `target.on_rotate`'s lambda. Block 26's rotate
       clause should FAIL. Restore.
    4. **Strictness probe (optional, only if cheap):** invoke
       `target.on_material_edit` *twice* with the same values; assert
       `materialCalls == 1` would FAIL with `expected 1, got 2`. This
       proves the "exactly one setter call" assertion's strictness
       shape. Skip if it adds non-trivial code; the load-bearing
       probes are 4a/b/c above.
5. **Build + verify deterministic.** `cmake --build build` then
   `./src/ysim --self-test` from `build/` 5 times in a row; expect
   `48/48 PASS` every time. If any run differs, STOP and hand back.
6. **Promote matrix row.** Update `docs/TEST_MATRIX.md` row `BDD-018`
   from `pending` → `pass`. Test address column gets the three Block
   26 pass labels. Add a parenthetical note: "Behavior-tag inspector
   edit parked under BDD-006 / Q2 — returns to scope when in-place
   behavior switching ships."
7. **`verify-light.sh` cross-check.** Run `./scripts/verify-light.sh`
   from project root. Expect doctest `159/159 SUCCESS` + `1120/1120
   SUCCESS` unchanged (no test/ files edited).
8. **Update `.agent/CURRENT_WORK.md`** with: file in flight (none —
   slice complete), how far (Block 26 authored + 48/48 deterministic +
   3 bug-probes verified), what's tested, what's next (Estimator
   review).
9. **Update `.agent/RESUME.md`** with: must-remember (the
   parallel-callback-shape constraint, the spec-substitution for
   behavior-tag, the no-`simulator.initialize()` invariant, the
   stricter-than-spec counter assertions), last decisions + why (no
   new D-NNN; mechanization shape (b) chosen over (a)/(c)), next step
   (Estimator review).
10. **No DECISIONS.md update** — pure mechanization slice, no new
    architectural decision.

## Course corrections

- **`feedback_make_means_add_new` rule.** The user's slice brief used
  "Mechanize" (mechanization verb), not a creation verb — so the
  parallel-symbol rule applies in its softer form: don't widen
  existing inspector signatures, don't add a new struct field, don't
  pre-compute a shared helper that production and harness both call
  (that's a refactor the next slice can do if needed). Block 26 is a
  new harness-side block that *uses* the existing struct + invokes
  its callbacks; that's a clean parallel add at the test layer.
- **`D-014` three-site cascade applies** to the translate clause's
  state.x assertion. Block 26 translates FIRST then rotates SECOND,
  so the translate clause sees an unrotated cube — straightforward
  delta.
- **`D-021` rotation-around-`transformPosition`-pivot applies** to
  the rotate clause. The harness computes the expected post-rotate
  world position by hand (not by calling D-022's `rotateVector`) to
  avoid the "use the implementation to verify the implementation"
  tautology.
- **`D-027` `setMaterial` writes both `mesh->material` AND
  `pendingMaterials[meshId]`.** The harness asserts on
  `mesh->material.*` directly (what the renderer reads). The
  pendingMaterials write-back path is already verified by Block 20
  phase 4; Block 26 doesn't re-verify it.
- **`D-025` auto-`applyPendingMaterials` from `sim.initialize`** is
  INTENTIONALLY not exercised — Block 26 specifically avoids
  `sim.initialize()` between callback fire and pump (that's the "no
  pause/resume" invariant).
- **`D-005` material 5-tuple** — the harness writes all 5 fields and
  asserts on all 5; partial coverage would let a regression in
  metallic/specularWeight/emission slip past.
- **Float behavior vs cloth.** Block 26 uses `addCube` (Float) for
  clean per-vertex assertions. Cloth would work too but mass-spring
  forces complicate the "state.x preserved across one sim step"
  assertion. Float pins state.x against gravity (force is zero), so
  one pump leaves state.x unchanged — the cleanest baseline.
- **The harness ImGui-context absence is fine.** Block 26 never
  calls `drawMeshInspectorWindow` and never instantiates ImGui. It
  only builds `MeshInspectorTarget` and invokes its callbacks
  directly; no ImGui dependency reaches the test path.

Expected matrix delta: `BDD-018` `pending` → `pass`.
Expected self-test count: 45 → 48 (Block 26 adds 3 PASS labels).
Expected verify.sh: exits 0 on macOS dev host. On Estimator's Linux
container the Metal SKIP path returns 0 before Block 26 executes;
doctest binaries pass unchanged.
Expected Estimator verdict: NOTE or WARNING. Possible items: (i) the
harness duplicates production's `buildSelectedMeshTarget` shape rather
than refactoring it into a shared helper — could be a NOTE about
future deduplication when source-file split lands; (ii) the witness
math in the rotate clause is hand-written (not via `rotateVector`),
which catches D-022 regressions but also means any error in the
hand-math is on the harness side, not the production side; (iii) the
behavior-tag clause is spec-substituted out, so a future BDD-006 slice
will need to extend Block 26 (or add a parallel block) to mechanize
the dispatch claim.
