# Plan — FR-004 UI-side rotate-object (`feat/rotate-object`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-10

## Course note: previous slice's verdict

Estimator turn 16 returned **NOTE** (clean) on the BDD-017 coverage
fix-turn — both turn-15 WARNINGs closed; the slice also picked up
D-020 (BVH leaf-return) as an unplanned scope expansion which the
Estimator flagged as an informational NOTE. Nothing to fold.

Role docs (`docs/roles/PLANNER.md`, `GENERATOR.md`, `ESTIMATOR.md`)
were updated last commit (`4217619`) with discipline learned from
the last 7 slices — re-read them before this slice if you haven't.

## Goal

Close FR-004's UI side — the natural pair to D-014's translateObject.
After this slice:

- `Simulator::rotateObject(int meshId, ::Quat newQuat)` is callable.
  It rotates `state.x` and `state.xPrev` around `mesh.transformPosition`
  pivot and writes `mesh.rotationQuat = newQuat` (normalized). Mirrors
  D-014 / D-015's translate pattern.
- The inspector exposes a Rotation collapsing header with an
  InputFloat4 `(w, x, y, z)` row using the existing
  `IsItemDeactivatedAfterEdit` commit gate.
- Block 15 in `runSelfTest` mechanizes "rotate then sim.update
  preserves the rotation in state.x" — analogous to Block 9's
  BDD-003 clauses (b) and (c). Float-tagged cube as the test
  target so the assertion is a strict-equality check (Float
  doesn't move).
- Two new D-NNN entries:
  - **D-021** — `Simulator::rotateObject` API + the
    rotate-around-pivot semantic (pivot = `transformPosition`).
  - **D-022** — `quatConjugate` and `rotateVector(Quat, vec3)`
    helpers join D-019's canonical Quat math family. Hamilton
    convention `a * b = apply b first, then a` carries forward.
- 30/30 self-test PASS deterministic (was 29/29; +1 BDD-003-style
  rotation line — see Scope item 4 for the exact pass-label
  decision).

This is **NOT** a matrix-row promotion slice. FR-004 doesn't have a
dedicated BDD row for the UI side (BDD-004 covers the
math/persistence layer, already passed). The slice ships an
internal feature with regression coverage in Block 15.

## Scope

### 1. New `Quat` math helpers — `quatConjugate`, `rotateVector` (D-022)

Add immediately after D-019's existing helpers in
`src/main.cpp` (~line 1554 area, near the existing `operator*` /
`quatNorm` / `quatNormalize`):

```cpp
inline Quat quatConjugate(const Quat& q) {
    return Quat{q.w, -q.x, -q.y, -q.z};
}

// Rotate a vector v by the quaternion q (assumes q is unit norm —
// callers should pass a normalized quat). Implementation uses the
// Hamilton-product form: v_rot = q * v_pure * conjugate(q),
// where v_pure = Quat{0, v.x, v.y, v.z}. Equivalent to the
// cross-product form but reuses the existing operator*.
inline tinym::vec3 rotateVector(const Quat& q, const tinym::vec3& v) {
    Quat vp{0.0f, v.x, v.y, v.z};
    Quat r = q * vp * quatConjugate(q);
    return tinym::vec3(r.x, r.y, r.z);
}
```

`operator*(Quat, Quat)` from D-019 left-associates, so
`q * vp * conjugate(q)` = `(q * vp) * conjugate(q)` — the canonical
Hamilton form. The conjugate is only valid for unit quaternions,
which is the intended invariant for rotation usage.

### 2. `Simulator::rotateObject` (D-021)

Add right after `Simulator::translateObject` in `src/main.cpp`
(~line 4432 area). API is **absolute** (matches translateObject's
`newPos` style — the inspector passes the new absolute rotation,
the simulator computes the delta internally):

```cpp
// FR-004 / D-021: rotate the named mesh by setting its absolute
// orientation to newQuat. Particles in state.x and state.xPrev are
// rotated around the mesh's transformPosition pivot by the delta
// from the current rotationQuat. xPrev moves with x by the same
// delta (D-013 invariant) so the next narrow phase doesn't see the
// rotate as a tunneling event. state.v is unchanged — rotating
// does not reset velocity. mesh.rotationQuat is updated to newQuat
// (normalized).
//
// Pack-roundtrip — like translateObject before D-015 — is NOT
// covered by this slice. Re-pack rebuilds state.x from the
// initializer's geometry, then applyPendingMaterials would re-
// apply pendingRotations[mesh.id] but the harness's flow does
// not currently populate pendingRotations on rotateObject. A
// future slice closes the gap (mirrors D-015's translate write-
// back invariant). See Non-goals below.
void rotateObject(int meshId, ::Quat newQuat) {
    auto* mesh = Scene<BE, PR>::findById(meshId);
    if (!mesh) return;
    if (!mesh->state.x.ptr) return;

    Quat newAbs = quatNormalize(newQuat);
    Quat delta  = quatNormalize(newAbs * quatConjugate(mesh->rotationQuat));
    tinym::vec3 pivot = mesh->transformPosition;

    const Index n = mesh->state.x.size / 3;
    for (Index i = 0; i < n; ++i) {
        tinym::vec3 p_curr(mesh->state.x.ptr[i*3+0],
                           mesh->state.x.ptr[i*3+1],
                           mesh->state.x.ptr[i*3+2]);
        tinym::vec3 p_rot = pivot + rotateVector(delta, p_curr - pivot);
        mesh->state.x.ptr[i*3+0] = p_rot.x;
        mesh->state.x.ptr[i*3+1] = p_rot.y;
        mesh->state.x.ptr[i*3+2] = p_rot.z;
        if (mesh->state.xPrev.ptr) {
            tinym::vec3 prev(mesh->state.xPrev.ptr[i*3+0],
                             mesh->state.xPrev.ptr[i*3+1],
                             mesh->state.xPrev.ptr[i*3+2]);
            tinym::vec3 prev_rot = pivot + rotateVector(delta, prev - pivot);
            mesh->state.xPrev.ptr[i*3+0] = prev_rot.x;
            mesh->state.xPrev.ptr[i*3+1] = prev_rot.y;
            mesh->state.xPrev.ptr[i*3+2] = prev_rot.z;
        }
    }
    mesh->rotationQuat = newAbs;
}
```

### 3. Inspector wiring

`include/MeshInspectorWindow.hpp` — extend `MeshInspectorTarget`:

```cpp
// FR-004 inspector rotate path. The 4-float pointer reads the
// mesh's rotationQuat as (w, x, y, z) (matches the on-disk schema
// order); the callback applies a finished edit through
// Simulator::rotateObject (which mutates state.x and state.xPrev
// in tandem and composes the delta onto mesh.rotationQuat).
// Both must be set together to enable the InputFloat4 row.
float* rotation_wxyz = nullptr;  // 4 contiguous floats: w, x, y, z
std::function<void(int, float, float, float, float)> on_rotate;
```

`float*` (raw pointer) avoids coupling the inspector header to
main.cpp's `Quat` type. The Quat struct is `{w, x, y, z}` POD per
D-019, so `&mesh->rotationQuat.w` aliases a contiguous `float[4]`.

`buildSelectedMeshTarget` in `src/main.cpp` adds:

```cpp
target.rotation_wxyz = &selectedMesh->rotationQuat.w;
target.on_rotate = [&simulator](int id, float w, float x, float y, float z) {
    simulator.rotateObject(id, ::Quat{w, x, y, z});
};
```

`src/mesh_inspector_gui.cpp` — add Rotation collapsing header
after the existing Transform header:

```cpp
if (target.rotation_wxyz && target.on_rotate) {
    if (ImGui::CollapsingHeader("Rotation", ImGuiTreeNodeFlags_DefaultOpen)) {
        float q[4] = { target.rotation_wxyz[0], target.rotation_wxyz[1],
                       target.rotation_wxyz[2], target.rotation_wxyz[3] };
        ImGui::InputFloat4("Quat (w,x,y,z)", q);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            target.on_rotate(target.mesh_id, q[0], q[1], q[2], q[3]);
            state.status_message = "Rotation updated.";
        }
    }
}
```

Quat input as raw `(w, x, y, z)` is intentionally minimal —
ergonomic (Euler/axis-angle) UI is FR-004 Notes territory ("UI may
expose Euler/axis-angle as input affordances") but a future slice.
The current InputFloat4 is enough to verify the API end-to-end.

### 4. Block 15 mechanization

Append after Block 14 in `src/main.cpp::runSelfTest`. Float-tagged
cube at origin, rotate by 90° around Z (axis-angle quat), assert
state.x reflects the rotation, run one `sim.update()`, assert
state.x is unchanged (Float doesn't move).

```cpp
// ---- Block 15: FR-004 UI — rotate-object via Simulator::rotateObject. -
// FR-004 wording (FRD.md): "the user shall rotate the center of any
// selected object, with rotation stored canonically as a quaternion;
// the same quaternion is used for rendering, simulation, and
// persistence." Block 12 (BDD-004) already covers the
// math/persistence side. This block covers the UI side: rotateObject
// mutates state.x by rotating around transformPosition pivot, and
// the next sim step / render reads the rotated positions.
//
// Substitution for "rendering reflects": same as Block 9's clause
// (c) — headless harness has no pixel render, so the testable proxy
// is "render-source state.x already carries the rotated values".
{
    constexpr float kPi = 3.14159265358979323846f;
    auto quatAxisAngleLocal = [](tinym::vec3 axis, float angle) {
        float half = angle * 0.5f;
        float s = std::sin(half);
        return ::Quat{std::cos(half), axis.x * s, axis.y * s, axis.z * s};
    };

    sim.collisionPipeline.broadPhase.objTrees.clear();
    resetScene();
    sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();
    const int rotateId = 0;

    auto* mesh0 = Scene<Backend, Precision>::findById(rotateId);
    if (!mesh0) {
        fail("FR-004 UI / rotate sets state.x rotated around pivot",
             "rotate target id=" + std::to_string(rotateId) + " not found");
    } else {
        // Witness vertex: state.x[0] (some corner of the cube).
        double x0 = mesh0->state.x.ptr[0];
        double y0 = mesh0->state.x.ptr[1];
        double z0 = mesh0->state.x.ptr[2];

        // Apply 90° rotation around Z. For a corner at (x, y, z),
        // 90°-Z rotation maps (x, y, z) -> (-y, x, z).
        ::Quat newAbs = quatAxisAngleLocal(tinym::vec3(0, 0, 1), kPi / 2.0f);
        sim.rotateObject(rotateId, newAbs);

        auto* meshAfter = Scene<Backend, Precision>::findById(rotateId);
        double rx = meshAfter->state.x.ptr[0];
        double ry = meshAfter->state.x.ptr[1];
        double rz = meshAfter->state.x.ptr[2];

        const double posTol = 1e-5;
        bool rotateOk =
            std::abs(rx - (-y0)) < posTol &&
            std::abs(ry - ( x0)) < posTol &&
            std::abs(rz - ( z0)) < posTol;

        if (!rotateOk) {
            fail("FR-004 UI / rotate sets state.x rotated around pivot",
                 "expected 90deg-Z rotation: (" + std::to_string(-y0) + ", " +
                 std::to_string(x0) + ", " + std::to_string(z0) + ") got (" +
                 std::to_string(rx) + ", " + std::to_string(ry) + ", " +
                 std::to_string(rz) + ")");
        } else {
            pass("FR-004 UI / rotate sets state.x rotated around pivot");

            // Clause analogous to Block 9 (b) + (c): next sim step uses
            // the rotated state, and render-source state.x reflects it.
            // Float-tagged cube doesn't move; strict equality.
            pumpFrames(sim, 1);
            auto* meshStep = Scene<Backend, Precision>::findById(rotateId);
            double sx = meshStep->state.x.ptr[0];
            double sy = meshStep->state.x.ptr[1];
            double sz = meshStep->state.x.ptr[2];
            if (std::abs(sx - rx) > posTol ||
                std::abs(sy - ry) > posTol ||
                std::abs(sz - rz) > posTol) {
                fail("FR-004 UI / next sim step preserves rotated state.x",
                     "Float-tagged witness moved between rotate and post-step");
            } else {
                pass("FR-004 UI / next sim step preserves rotated state.x");
            }
        }
    }
}
```

Two new pass labels:
- `FR-004 UI / rotate sets state.x rotated around pivot`
- `FR-004 UI / next sim step preserves rotated state.x`

Total: **31/31 self-test PASS** after this slice (was 29/29; +2
new pass labels).

### 5. Bookkeeping

- **No `docs/TEST_MATRIX.md` change.** No matrix row promotion —
  FR-004 UI side has no dedicated BDD row. BDD-004 (already passed)
  covers the math/persistence side.
- **`docs/DECISIONS.md`:** D-021 (rotateObject API + pivot
  semantic) and D-022 (`quatConjugate` + `rotateVector` helpers).
- **`.agent/CURRENT_WORK.md` / `RESUME.md`:** update for the
  slice. RESUME notes the deferred pack-roundtrip explicitly so
  the next planner can pick it up.

## Non-goals (this slice)

- **Pack-roundtrip after `rotateObject`.** Re-pack (any
  `Simulator::initialize()` after rotate, e.g., from
  `addCube`/`importMesh`) rebuilds `state.x` from the
  initializer's geometry → rotation effect on state.x is lost. The
  initializer doesn't carry rotation; D-007's `pendingRotations`
  side-table is only populated by `loadScene`. A clean fix mirrors
  D-015's translate write-back: `rotateObject` writes
  `pendingRotations[mesh.id] = newAbs`, and either
  `applyPendingMaterials()` is called automatically after every
  `Simulator::initialize()` (contract change) or the user must
  call it explicitly. This is its own slice; the design call about
  whether to auto-call applyPendingMaterials is what makes it
  Planner-scope, not a Generator hand-fix. **Defer.** Block 15 in
  this slice does NOT exercise re-pack — only "rotate then 1 sim
  step", which is enough to verify the rotateObject API works.

- **Inspector ergonomics (Euler / axis-angle inputs).** FR-004
  Notes line says "UI may expose Euler/axis-angle as input
  affordances." The current slice ships raw quaternion (w, x, y,
  z) input. Euler/axis-angle UX is its own slice; tied to BDD-004
  Notes' warning about silent Euler↔quaternion conversion.

- **Inspector UI test mechanization.** Block 15 calls
  `sim.rotateObject(...)` directly. The inspector's
  `IsItemDeactivatedAfterEdit` gate is exercised only via manual
  GUI use — same gap as BDD-018's "live-edit propagation" which
  remains unmechanized. Not in scope here.

- **Renderer-side rotation application via uniform.** Like D-014
  for translate, the rotate path bakes the rotation into `state.x`
  rather than passing a model matrix to the shader. Per-mesh
  model matrix is the renderer-rework slice; out of scope.

- **`state.v` rotation.** Velocities are NOT rotated by
  `rotateObject`. For Float-tagged meshes (the test target) v is
  always zero; for cloth that's mid-flight, rotating v with the
  delta would be physically meaningful but adds another path.
  Defer. Block 15 only tests Float so this doesn't matter for
  v1's mechanization.

- **Other matrix rows, other features, spec edits.**

- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/rotate-object` (off `main`
   at `4217619`). Commit prefix: `add:` (new feature).

2. **Re-read** `docs/specs/FRD.md#FR-004` (lines 52–60),
   `docs/DECISIONS.md::D-014` (translateObject precedent),
   `D-015` (translate pack-roundtrip — note that this slice
   does NOT close the rotate analog), `D-019` (Quat math and
   Hamilton convention).

3. **Add Quat helpers (D-022).** Insert `quatConjugate` and
   `rotateVector` directly after `quatNormalize` in `src/main.cpp`
   (~line 1554 area). Both inline; both rely on D-019's existing
   `operator*(Quat, Quat)`. Add `<cmath>` if not transitively
   included (it is — D-019's `std::sqrt` already pulls it in).

4. **Add `Simulator::rotateObject`.** Insert immediately after
   `Simulator::translateObject` (~line 4432 in `src/main.cpp`).
   Mirror translateObject's structure: `findById`, early-return
   on null mesh / null state.x.ptr, compute `delta = newAbs *
   conjugate(currentQuat)` normalized, loop over particles
   rotating around `transformPosition` pivot, mirror to
   `state.xPrev` for D-013 parity, write `mesh->rotationQuat = newAbs`.
   Do NOT touch `state.v`. Do NOT touch the initializer (that's
   the deferred D-015 analog).

5. **Extend `MeshInspectorTarget`.** Add `float* rotation_wxyz`
   and `std::function<void(int, float, float, float, float)> on_rotate`
   to `include/MeshInspectorWindow.hpp`. Comment cites FR-004 +
   the `Quat` POD layout that makes `&mesh.rotationQuat.w` an
   alias for `float[4]`.

6. **Wire `buildSelectedMeshTarget`.** In `src/main.cpp` (~line
   5974 area), populate `target.rotation_wxyz =
   &selectedMesh->rotationQuat.w;` and `target.on_rotate =
   [&simulator](...) { simulator.rotateObject(id,
   ::Quat{w, x, y, z}); };`.

7. **Render the inspector control.** In
   `src/mesh_inspector_gui.cpp::drawMeshInspectorWindow` (after
   the existing Transform header), add the Rotation collapsing
   header with `InputFloat4("Quat (w,x,y,z)", q)` gated on
   `IsItemDeactivatedAfterEdit`. Match the existing translate
   path's idiom.

8. **Author Block 15.** Mirror Block 9's structure (BDD-003
   clauses b/c) — Float-tagged cube at origin, rotate 90° around
   Z, assert state.x[0] reflects the rotation, pump 1 frame,
   assert state.x[0] unchanged.

9. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

10. **Run `--self-test` 5+ times.** Expect **31/31 PASS**
    consistently. Quaternion math is fully deterministic on a
    single machine; intermittent FAIL would indicate something
    weird (most likely D-018 jiggle entering through the cube's
    initial state.x — but cubes don't use jiggle, so no issue).

11. **Optional bug-probe.** For confidence: temporarily change
    the rotation axis from Z to X (or skip the
    `mesh->rotationQuat = newAbs` write); confirm Block 15 FAILs
    with the diagnostic showing the mismatch. Restore.

12. **Add `D-021` and `D-022` to `docs/DECISIONS.md`.** Both
    follow the standard format: file/function, decision,
    alternatives-considered, rationale, date.

13. **Update CURRENT_WORK / RESUME.** Note explicitly that the
    pack-roundtrip closure is deferred and surfaced in RESUME so
    the next planner picks it up (the rotate analog of D-015).

14. **Stop and hand off to the Estimator.** No matrix-row change,
    no spec edits, no other features.

## Course corrections

- **Spec-vs-label discipline (codified in `docs/roles/PLANNER.md`
  step 7).** Block 15's pass labels are authored from FR-004's
  "Output" line + Block 9's clause-(b)/(c) shape. Stricter than
  spec? The slice's primary value IS the strictness — Block 12
  already passed BDD-004's persistence tolerance test, and Block
  15 closes the orthogonal "state.x reflects the new rotation
  for sim/render" gap. Without this, a future regression that
  breaks `rotateObject`'s rotate-around-pivot logic would be
  invisible until BDD-101's spine slice exercised it.

- **Architectural invariants applying here (per PLANNER.md step
  8):** `xPrev` parity with `x` (D-013 — must move with state.x);
  D-019's Hamilton convention (a * b = apply b first, then a);
  pivot is `mesh.transformPosition` (D-014's transform field).
  `state.v` is unchanged (D-014 mirror — translating/rotating
  doesn't reset velocity).

- **Pack-roundtrip is deferred but documented.** Per D-015's
  precedent, the right fix is a write-back invariant. The design
  call about whether `applyPendingMaterials()` should be
  auto-called from `Simulator::initialize()` is what makes the
  fix Planner-scope, not Generator hand-fix. Surface the gap in
  CURRENT_WORK and RESUME explicitly.

- **`Quat` POD layout `{w, x, y, z}`.** D-019 keeps the struct as
  a 4-float aggregate matching the on-disk schema. Using
  `&mesh.rotationQuat.w` as an alias for `float[4]` is safe ONLY
  because D-019's docstring locks that order. If a future slice
  reorders Quat (it shouldn't), this slice's float-pointer
  inspector wiring breaks; D-019's docstring is the canonical
  source of truth.

- **No Euler↔Quat conversion.** BDD-004 Notes warn about silent
  conversions in the persistence layer. Block 15 stays in
  quaternion-space throughout (axis-angle helper produces a Quat
  directly via `cos(half), sin(half)*axis`). The inspector's
  raw-quaternion input similarly avoids the trap.

## What to read before writing code

- `docs/specs/FRD.md#FR-004` (lines 52–60) — functional contract.
- `docs/specs/BDD.md#BDD-004` and `docs/TESTS.md#BDD-004` —
  already-passing math/persistence side; this slice is the
  orthogonal UI/state.x side.
- `docs/DECISIONS.md::D-014` — translateObject pattern this slice
  mirrors.
- `docs/DECISIONS.md::D-015` — translate pack-roundtrip; the
  rotate analog is deferred and the deferral is documented.
- `docs/DECISIONS.md::D-019` — canonical Quat math, Hamilton
  convention, struct layout invariant.
- `src/main.cpp::struct Quat` (~line 1548) — field order
  `{w, x, y, z}`, default identity.
- `src/main.cpp::Simulator::translateObject` (~line 4432) — the
  precedent function this slice mirrors.
- `src/main.cpp::runSelfTest` Block 9 (~line 5807) — template for
  Block 15's clause (b)/(c) structure.
- `include/MeshInspectorWindow.hpp` and
  `src/mesh_inspector_gui.cpp` — existing translate inspector
  path.
