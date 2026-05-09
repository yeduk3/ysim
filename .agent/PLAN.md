# Plan — BDD-004 Quaternion Composition + Round-Trip (`feat/bdd-004-rotation`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict

Estimator turn 12 returned **WARNING** (no BLOCK). The single
WARNING — BDD-102 mechanizes against `state.x` snapshots rather
than Alembic bytes — is **not foldable** into this slice. Closing
it would mean shipping the entire Alembic export pipeline (FR-013),
which is blocked on Q5 (Alembic schema) + Q6 (export FPS / time-step
decoupling rule). It stays as a documented structural acknowledgement
until those open questions resolve.

## Goal

Promote `BDD-004` from `pending` to `pass` in
`docs/TEST_MATRIX.md` by adding **Block 12 in `runSelfTest`** that
mechanizes BDD-004 verbatim against `docs/TESTS.md#BDD-004`. The
test exercises three properties of the existing
`GeneralMesh::rotationQuat` field (D-007):
1. **Quaternion composition is associative through save/load.**
   Two rotations composed across a save → load boundary equal the
   same two rotations composed in-memory.
2. **Unit-norm invariant holds after each composition.** Repeated
   multiplication followed by normalization keeps `|q|` within
   tight tolerance of 1.
3. **Round-trip preserves the orientation, not just the bytes.**
   The post-load quaternion's rotation action on a witness vector
   matches the pre-save action within tolerance.

When this slice ships:
- `BDD-004` matrix row flips `pending → pass`.
- 25/25 self-test PASS (was 24/24; one new BDD-004 line).
- New `Quat` math helpers (composition, normalization, norm) live
  alongside the existing struct so the harness — and any future
  rotation consumer (FR-004 inspector, FR-008 rigid body, etc.) —
  has a single canonical implementation. Today the struct is bare
  (just `w, x, y, z` floats); BDD-004 forces the math to land.

## Scope

- **`docs/TESTS.md#BDD-004` (lines 47–53)** is the binding "Then"
  clause:
  > **Given** an object with rotation R₀
  > **When**  the user composes a sequence of rotations and the
  > scene is saved, reloaded, and rotated further
  > **Then**  the resulting orientation matches the mathematically
  > composed quaternion within floating-point tolerance, and the
  > stored quaternion remains unit-norm after each composition.
  >
  > *Notes: this is a round-trip + composition test, not just a
  > "set rotation" test. The point is to catch normalization drift
  > and any silent Euler↔quaternion conversion in the persistence
  > layer.*

- **New `Quat` math helpers in `src/main.cpp`** (~line 1548 area).
  The existing `struct Quat` is bare. Add:
  - `Quat operator*(const Quat& a, const Quat& b)` — Hamilton
    product (rotation composition): `a * b` applies `b` first, then
    `a`. Standard formula.
  - `float Quat::norm() const` — `sqrt(w² + x² + y² + z²)`.
  - `Quat Quat::normalized() const` — divide by norm; if norm <
    1e-12, return identity (avoids NaN propagation).

  Place the helpers immediately after the struct definition. Keep
  the struct itself unchanged so D-007's existing field semantics
  (default-constructed = identity) stay intact.

- **Block 12 in `runSelfTest`.** Append after Block 11 (last block
  before the `if (failures == 0)` summary). Concrete shape — the
  Generator may tweak idiomatically:

  ```cpp
  // ---- Block 12: BDD-004 — Rotate with quaternion canonical storage. ----
  // TESTS.md#BDD-004 wording (verbatim, *not* the matrix-row label):
  //   Given an object with rotation R0
  //   When  the user composes a sequence of rotations and the scene is
  //         saved, reloaded, and rotated further
  //   Then  the resulting orientation matches the mathematically composed
  //         quaternion within floating-point tolerance, and the stored
  //         quaternion remains unit-norm after each composition.
  //   Notes: round-trip + composition test, not just "set rotation".
  //          Catches normalization drift and silent Euler↔quaternion
  //          conversions in the persistence layer.
  {
      resetScene();
      sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                  /*size=*/0.2f, /*mass=*/0.1f);
      sim.initialize();
      const int rotateId = 0;

      // R0: rotation about Y by 30°.
      Quat r0 = quatAxisAngle(tinym::vec3(0, 1, 0), kPi / 6.0f);
      // dq1: rotation about X by 45°. dq2: about Z by 60°.
      Quat dq1 = quatAxisAngle(tinym::vec3(1, 0, 0), kPi / 4.0f);
      Quat dq2 = quatAxisAngle(tinym::vec3(0, 0, 1), kPi / 3.0f);

      // Apply R0 to the mesh.
      auto* mesh0 = Scene<Backend, Precision>::findById(rotateId);
      mesh0->rotationQuat = r0;

      // Compose dq1 in-memory: R1 = (dq1 * r0).normalized()
      Quat r1_in_memory = (dq1 * mesh0->rotationQuat).normalized();
      mesh0->rotationQuat = r1_in_memory;

      // Save → reload.
      const std::string path = "/tmp/ysim_bdd004.ysim.json";
      std::string saveErr;
      if (!sim.saveScene(path, &saveErr)) {
          fail("BDD-004 / quaternion composition round-trip", "saveScene failed: " + saveErr);
      } else {
          auto lr = sim.loadScene(path);
          if (!lr.ok) {
              fail("BDD-004 / quaternion composition round-trip",
                   "loadScene failed: " + lr.error.message);
          } else {
              sim.initialize();
              sim.applyPendingMaterials();  // also applies pendingRotations.

              // Compose dq2 onto the reloaded rotation.
              auto* meshAfterLoad = Scene<Backend, Precision>::findById(rotateId);
              Quat r1_post_load = meshAfterLoad->rotationQuat;
              Quat r2_round_trip = (dq2 * r1_post_load).normalized();
              meshAfterLoad->rotationQuat = r2_round_trip;

              // Reference: same composition done entirely in-memory.
              Quat r2_in_memory = (dq2 * r1_in_memory).normalized();

              // Clause: orientation matches within FP tolerance.
              float dw = std::abs(r2_round_trip.w - r2_in_memory.w);
              float dx = std::abs(r2_round_trip.x - r2_in_memory.x);
              float dy = std::abs(r2_round_trip.y - r2_in_memory.y);
              float dz = std::abs(r2_round_trip.z - r2_in_memory.z);
              const float quatTol = 1e-5f;
              bool orientationOk = dw < quatTol && dx < quatTol &&
                                   dy < quatTol && dz < quatTol;

              // Clause: unit-norm after composition.
              float norm0 = r0.norm();
              float norm1 = r1_in_memory.norm();
              float norm2 = r2_round_trip.norm();
              const float normTol = 1e-5f;
              bool unitNormOk =
                  std::abs(norm0 - 1.0f) < normTol &&
                  std::abs(norm1 - 1.0f) < normTol &&
                  std::abs(norm2 - 1.0f) < normTol;

              if (!orientationOk) {
                  fail("BDD-004 / quaternion composition round-trip",
                       "orientation drift (round-trip vs in-memory): " +
                       std::to_string(dw) + ", " + std::to_string(dx) + ", " +
                       std::to_string(dy) + ", " + std::to_string(dz));
              } else if (!unitNormOk) {
                  fail("BDD-004 / quaternion composition round-trip",
                       "unit-norm drift; norms = " +
                       std::to_string(norm0) + ", " +
                       std::to_string(norm1) + ", " +
                       std::to_string(norm2));
              } else {
                  pass("BDD-004 / quaternion composition round-trip");
              }
          }
          std::remove(path.c_str());
      }
  }
  ```

  The helper `quatAxisAngle(axis, angle)` and the constant `kPi`
  are local to the block (or defined inside `runSelfTest`'s
  preamble). Generator's call.

- **`docs/TEST_MATRIX.md` row `BDD-004`** — promote `pending →
  pass`. Test address: `src/main.cpp::runSelfTest::BDD-004 (Block
  12)`.

- **No new `docs/specs/*` edits.** TESTS.md / FRD.md are not
  touched.

## Non-goals (this slice)

- **`Simulator::rotateObject(meshId, deltaQuat)` API + inspector
  wiring** — the BDD wording is purely about quaternion math +
  persistence, not UI. A "user composes rotations" semantic is
  satisfied by the harness directly mutating `mesh.rotationQuat`.
  Adding a runtime mutator + inspector control is its own slice
  (would mirror D-014's translateObject pattern). Defer to a
  future "rotation editing UI" slice if desired.

- **Renderer-side rotation application.** The mesh draw path still
  reads `state.x` directly without applying `rotationQuat`. This
  slice does not change that. Closing the visual side requires
  either (a) baking the rotation into `state.x` once on commit
  (D-014 pattern, but rotation has subtleties — pivot point,
  cloth-in-flight semantics) or (b) a per-mesh model matrix in the
  shader (renderer rework, much larger). Both are out of scope.

- **Euler↔quaternion conversion path** in the persistence layer.
  The current scene format stores quaternions directly as
  `{w, x, y, z}` arrays (per `toSnapshot`/`loadScene`). No Euler
  conversion exists. The BDD's Notes line "catches silent
  Euler↔quaternion conversion" is a guard against future drift —
  Block 12's tolerance check would fire if such a conversion is
  ever introduced.

- **`Simulator::rotateObject` parallel to `translateObject`**
  (D-014). Mirrors translate's pattern but adds renderer-side
  questions (does rotation update state.x? if so, around what
  pivot? if not, where does it apply?). Defer.

- **Cross-build determinism for Quat math.** v1's
  same-binary-same-machine scope per PRD §6 is sufficient.

- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/bdd-004-rotation` (off
   `main` at `992b658`). No new branch. Commit prefix: `add:`.

2. **Re-read the binding "Then" clause** in
   `docs/TESTS.md#BDD-004` (lines 47–53) and the Notes line. Block
   12 assertions are authored from this verbatim.

3. **Add Quat math helpers in `src/main.cpp`** (right after the
   `struct Quat` definition at line 1548):

   ```cpp
   inline Quat operator*(const Quat& a, const Quat& b) {
       Quat r;
       r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
       r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
       r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
       r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
       return r;
   }

   inline float quatNorm(const Quat& q) {
       return std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
   }

   inline Quat quatNormalize(const Quat& q) {
       float n = quatNorm(q);
       if (n < 1e-12f) return Quat{};
       Quat r{q.w/n, q.x/n, q.y/n, q.z/n};
       return r;
   }
   ```

   `<cmath>` is already pulled in transitively. If not, add
   `#include <cmath>` near the other std headers. Generator
   may prefer member functions on `Quat` instead of free functions
   — either is fine, just stay consistent.

4. **Author Block 12 in `runSelfTest`** per the shape in the Scope
   section. Concrete details:
   - `quatAxisAngle(axis, angle)` helper returns
     `Quat{cos(angle/2), sin(angle/2)*axis.x, ...}`. Local to the
     block or a sibling lambda inside `runSelfTest`.
   - `kPi = 3.14159265358979323846f` constant inline.
   - Use `addCube` (Float-tagged) — rotation composition is a
     CPU-side mutation that doesn't depend on simulation behavior.
   - The `pendingRotations` apply path runs inside
     `applyPendingMaterials` (mis-named for historical reasons —
     it also applies rotations). Call it after `loadScene + initialize`.
   - Tolerance: 1e-5 for both orientation and norm. Three
     compositions of unit quaternions accumulate at most ~3 ULPs
     of float drift; 1e-5 is comfortably above that.

5. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120; nothing in their surface changes.

6. **Run `--self-test` 5+ times.** Quaternion math is fully
   deterministic on a single machine — expect **25/25 PASS**
   consistently. If Block 12 FAILs intermittently, the cause is
   real nondeterminism (saveScene/loadScene side, since the
   composition itself is pure CPU math). **Stop and hand back to
   Planner** in that case.

7. **Optional bug-probe.** For confidence: temporarily change `dq1`
   to a non-unit quaternion (skip the `.normalized()`) and confirm
   Block 12 FAILs with the unit-norm clause; or perturb the saved
   scene's rotation field by hand and confirm the orientation
   clause FAILs. Same discipline as prior slices.

8. **Promote `BDD-004` matrix row.** `docs/TEST_MATRIX.md:18`:
   - Status: `pending → pass`.
   - Test address: `src/main.cpp::runSelfTest::BDD-004 (Block 12)
     — quaternion composition round-trip + unit-norm invariant
     across saveScene/loadScene boundary.`

9. **DECISIONS / CURRENT_WORK / RESUME.** New numbered entry in
   `docs/DECISIONS.md`:
   > **D-NNN — `Quat` gains a canonical math implementation
   > (Hamilton product + normalization).** Lives alongside the
   > existing struct in `src/main.cpp`. The implementation is
   > load-bearing for BDD-004 mechanization and is the canonical
   > source any future rotation consumer should use (FR-004
   > inspector, FR-008 rigid body, eventual renderer-side
   > rotation). Document this so the next consumer doesn't
   > duplicate the math.

   Update `CURRENT_WORK.md` and `RESUME.md` per role doc steps
   6/7.

10. **Stop and hand off to the Estimator.** No inspector wiring,
    no renderer change, no `Simulator::rotateObject` API, no
    other matrix rows.

## Course corrections

- **Spec-vs-label discipline.** Block 12's pass label and
  assertions come from `docs/TESTS.md#BDD-004`'s "Then" clause
  verbatim. The matrix-row label "Rotate with quaternion canonical
  storage" is too compressed — assertions written from labels
  have BLOCKed past slices.

- **`pendingRotations` is the load-side mirror.** When `loadScene`
  parses `o.transform.rotation`, it stashes the quaternion in
  `pendingRotations[meshId]`. `applyPendingMaterials()` (despite
  its name) applies both materials and rotations. Block 12's
  saveScene/loadScene/initialize/applyPendingMaterials sequence
  must call `applyPendingMaterials` to actually write the
  rotation back into `mesh.rotationQuat`. Forgetting this would
  make the assertion compare an identity quaternion against the
  expected composition and fail with a misleading message.

- **`Quat` is a CPU-only struct.** It does not cross the Metal
  kernel boundary. No buffer binding to verify, no `[[buffer(N)]]`
  to grep. CM-004's lesson doesn't apply here.

- **Hamilton product convention: `a * b` applies `b` first, then
  `a`.** This matches the standard "rotate by R₁, then by R₂"
  reads as `R₂ * R₁`. The BDD wording "composes a sequence of
  rotations" is direction-agnostic; the test just needs the
  in-memory and round-trip paths to use the **same** convention.
  Document the convention in the new D-NNN entry so future
  consumers don't get it backwards.

- **No D-018 cascade.** D-015's three-site cascade
  (translateObject + Scene::pack + toSnapshot) is about position
  write-back. D-018's seed-from-id invariant is about
  initializer randomness. Neither applies to this slice — the
  rotation field is mutated directly by the harness, persistence
  already round-trips it via `pendingRotations`, and there's no
  randomness involved.

## What to read before writing code

- `docs/TESTS.md#BDD-004` (lines 47–53) — binding "Then" clause +
  Notes line. Verbatim source for Block 12.
- `docs/specs/BDD.md#BDD-004` and `docs/specs/FRD.md#FR-004` —
  user-story framing and functional contract (the FRD entry sets
  context but BDD-004's spec is purely about the quaternion math
  / persistence side, not UI).
- `src/main.cpp::struct Quat` (~line 1548) — the bare struct that
  will gain math helpers.
- `src/main.cpp::Simulator::toSnapshot` (~line 4880) — encodes
  rotation as `o.transform.rotation = {w, x, y, z}` (4-element
  array). Confirm the format matches Block 12's expectation
  before writing the assertion.
- `src/main.cpp::Simulator::loadScene` (~line 5060–5095) —
  decodes `o.transform.rotation` into `pendingRotations[meshId]`
  and `applyPendingMaterials()` writes it into
  `mesh.rotationQuat`. Block 12 must call `applyPendingMaterials`
  after `loadScene + initialize`.
- `src/main.cpp::runSelfTest` Block 8 (BDD-015) and Block 9
  (BDD-003 round-trip) — existing block templates that exercise
  saveScene/loadScene; mirror their structure.
- `.agent/RESUME.md` (current) — D-018 invariants don't apply
  here; rotation is not seed-derived.
