# Tests

> Owner: **Planner** authors scenarios; **Generator** implements them; **Estimator** validates BDD-fidelity.
>
> One block per behavior id from `docs/specs/BDD.md`. The Generator turns each block into a test function and records the test address in `docs/TEST_MATRIX.md`.

## Format

```
## <BDD-ID> — <short scenario name>

**Given** <preconditions>
**When**  <action>
**Then**  <observable outcome>

Notes: <any edge case or constraint that the test must cover>
```

## Scenarios

### Scene authoring

#### BDD-001 — Create a sphere primitive

**Given** an empty scene
**When**  the user invokes "Create primitive" with shape `Sphere`, size `1.0`, default tessellation
**Then**  the scene contains one object whose geometry is a sphere mesh, with default material, `BehaviorType::Float`, identity rotation (quaternion `(1, 0, 0, 0)`), and position at the origin.

Notes: cube creation follows the same shape; cover both in the test. Defaulting new primitives to `Float` (rather than e.g. `Rigid`) is a design call recorded in `FRD.md#FR-001` — flag if changed.

#### BDD-002 — Import a `.obj` mesh

**Given** an empty scene and a valid `.obj` file on disk
**When**  the user imports it via "Import mesh"
**Then**  the scene contains a new object whose geometry matches the file, with default material and `Float` behavior; the scene's persisted state records the import path so a later save/reload reproduces the same source.

Notes: also assert that an invalid or unreadable file produces a clear error and does **not** mutate the scene (no partial-add).

#### BDD-003 — Translate a selected object

**Given** an object positioned at the origin
**When**  the user sets its position to `(1, 2, 3)`
**Then**  the object's center is `(1, 2, 3)`; the next simulation step uses the new position; rendering reflects the new position on the next frame.

Notes: cover both inspector field edit and gizmo drag if the gizmo is in v1 scope.

#### BDD-004 — Rotate with quaternion canonical storage

**Given** an object with rotation `R₀`
**When**  the user composes a sequence of rotations and the scene is saved, reloaded, and rotated further
**Then**  the resulting orientation matches the mathematically composed quaternion within floating-point tolerance, and the stored quaternion remains unit-norm after each composition.

Notes: this is a round-trip + composition test, not just a "set rotation" test. The point is to catch normalization drift and any silent Euler↔quaternion conversion in the persistence layer.

#### BDD-005 — Edit OpenPBR material parameter

**Given** an object with default material parameters
**When**  the user changes `roughness` from `0.5` to `0.1`
**Then**  the in-app preview render reflects the lower roughness, and the persisted material stores `roughness = 0.1` under the OpenPBR-conformant name and value range.

Notes: also test base color, metallic, specular weight, and emission color (the proposed v1 subset). Out-of-range values must clamp to the OpenPBR-defined range, not silently accept.

### Simulation behaviors

#### BDD-006 — Assign behavior type to an object

**Given** an object with `BehaviorType::Float`
**When**  the user changes its behavior to `Cloth` via the inspector
**Then**  the object's behavior tag is `Cloth`, its parameter struct is populated with cloth defaults, and the next simulation step dispatches it through the cloth pipeline.

Notes: cover all three v1 behaviors (`Float`, `Cloth`, `Rigid`). Selecting a reserved-but-not-shipped behavior (`Elastic`, `Fluid`, `Generator`) must not be possible from the v1 UI.

#### BDD-007 — Cloth drapes onto a rigid surface

**Given** a cloth grid suspended above a static rigid sphere, with gravity `(0, -9.81, 0)` and wind `(0, 0, 0)`
**When**  the simulation advances `N` steps (enough for the cloth to contact the sphere)
**Then**  the cloth's mean y-position decreases over time, contact constraints fire on the broad/narrow-phase pipeline, no cloth vertex tunnels through the sphere, and total energy stays bounded (no explosion).

Notes: tolerance for "stays bounded" must be defined — propose energy ≤ 10× initial potential energy as a coarse stability check. Tunneling check: no vertex's distance-to-sphere goes negative beyond cloth thickness.

#### BDD-008 — Rigid body falls and rests

**Given** a rigid box dropped from height `h` onto a static ground plane, with gravity `(0, -9.81, 0)`
**When**  the simulation advances until kinetic energy stays under threshold for a sustained window
**Then**  the box's final y-position is consistent with the box resting on the plane (within a small tolerance), and its angular velocity has decayed.

Notes: must pass under both Bullet and Jolt backends (code-level switch — `FR-008`). Tolerance is intentionally loose: this validates that the rigid pipeline produces a usable result, not that two backends agree numerically.

#### BDD-009 — Float behavior ignores gravity and wind

**Given** an object with `BehaviorType::Float` at position `p` and zero velocity, with non-zero gravity and wind
**When**  the simulation advances any number of steps
**Then**  the object's position is exactly `p` and its velocity is exactly zero, irrespective of force settings.

Notes: this is a strict equality test (no tolerance) — Float must be force-exempt by construction, not by integration cancelling out.

#### BDD-010 — Collision detected between simulated objects

**Given** two simulated meshes positioned so that their AABBs overlap on the next step
**When**  the broad-phase + narrow-phase pipeline runs once
**Then**  the resulting constraint set contains at least one contact pair `(A, B)` between the two objects.

Notes: also assert that the same scene with non-overlapping AABBs produces an empty constraint set (negative case). Self-collision is parked (`PRD §4`); the test scene must not trigger self-collision.

### Environment forces

#### BDD-011 — Change gravity at runtime

**Given** a scene with a single non-`Float` object and gravity `(0, -9.81, 0)`
**When**  the user changes gravity to `(9.81, 0, 0)` while the simulation is running
**Then**  the object's velocity begins accumulating in the `+x` direction within a few steps; no restart is required.

Notes: covers `FR-018` live-edit propagation as well.

#### BDD-012 — Apply wind force

**Given** a cloth object at rest with wind `(0, 0, 0)`
**When**  the user sets wind to `(5, 0, 0)`
**Then**  cloth particle velocities gain a positive `x` component over subsequent steps, scaled with the wind magnitude.

Notes: the test must continue to pass after the v2 wind reformulation (force → velocity field). Authors of that change should keep the user-observable behavior here qualitatively the same.

### Export

#### BDD-013 — Export simulation to Alembic

**Given** a simulated scene with one cloth object and one rigid object, and a chosen frame range `[f₀, f₁]` at frame rate `r`
**When**  the user invokes "Export Alembic" with a target path
**Then**  an `.abc` file is written at the path; it contains `f₁ - f₀ + 1` frames; per-frame vertex positions exist for the cloth; per-frame transforms exist for the rigid; mesh topology is preserved; opening the file in a downstream consumer (Unreal, or another Alembic reader) reproduces the visible motion.

Notes: the round-trip-through-Unreal step is part of `BDD-101`; this test only needs to verify a valid Alembic file structurally, then defer end-to-end to BDD-101. Schema specifics depend on PRD Q5.

### Scene persistence

#### BDD-014 — Save scene to disk

**Given** an authored scene with at least one primitive, one imported mesh, an edited material, a non-default behavior, and non-default forces
**When**  the user saves to a target path
**Then**  a file exists at the path; its contents are structured (named fields, enumerated behavior tags), include a format version, and include all of: object list, per-object transforms (with quaternion rotation), materials, behavior tags + parameters, global forces, and the import path for any externally-loaded meshes.

Notes: format choice depends on PRD Q3; the test asserts structural properties rather than a specific syntax. "Human-diffable" is asserted by spot-checking that two scenes differing in one field produce a small, localized diff.

#### BDD-015 — Load scene reproduces saved state

**Given** a scene file produced by `BDD-014`
**When**  the user loads it into a fresh application instance
**Then**  the scene state in memory matches the saved state field-by-field; running the simulation from this state produces the same first-step output as the original session would have.

Notes: "same first-step output" is a strict equality check on initial conditions, not on long-horizon trajectories — long-horizon equality is `BDD-102`'s job.

#### BDD-016 — Reject incompatible scene file version

**Given** a scene file whose version field is unrecognized by the running build
**When**  the user attempts to load it
**Then**  the load fails with an explicit error message that names the version mismatch; the application's current scene state is unchanged (no partial mutation).

Notes: also test the case where the version field is missing entirely — same outcome.

### Existing capabilities (no-regression)

#### BDD-017 — Ray-pick selects nearest hit object

**Given** a scene with two objects whose screen-space projections do not overlap
**When**  the user clicks on one object's screen position
**Then**  that object becomes the selected object; the inspector displays its parameters.

Notes: also test the overlapping case — the front-most object (smallest ray `t`) wins.

#### BDD-018 — Inspector edits propagate live

**Given** a running simulation with an object selected
**When**  the user changes the object's color or behavior tag in the inspector
**Then**  the change is visible in the very next rendered frame; if the behavior changed, the next simulation step dispatches through the new behavior.

Notes: must not require pause/resume of the simulation.

#### BDD-019 — Frame profiler shows and exports timings

**Given** a running simulation with at least one named timing section
**When**  the user opens the profiler window and then invokes "Export CSV"
**Then**  the GUI displays per-section timings updated each frame, and a CSV file is written under `profiles/` containing the recorded history.

Notes: history collection must pause when the simulation pauses (`CLAUDE.md` invariant).

### Cross-cutting

#### BDD-101 — End-to-end round-trip to Alembic into Unreal

**Given** a cloth-on-sphere scene authored from primitives in a fresh ysim instance
**When**  the user simulates, saves the scene, reloads it in a new instance, simulates again, and exports an Alembic over a chosen frame range
**Then**  the Alembic imports into Unreal and plays back showing the cloth draping onto the sphere; the reloaded scene produces the same export bytes (or visually identical playback) as the pre-save run would have.

Notes: this is the v1 ship-blocker spine. Any failure here — even with all individual FRs passing — is a `BLOCK` for v1.

#### BDD-102 — Single-machine determinism

**Given** a saved scene file and a fixed build of ysim on one machine
**When**  the user runs the full simulate-and-export flow twice in succession
**Then**  the two Alembic outputs are visually identical (per-frame vertex positions agree within a tight floating-point tolerance).

Notes: cross-machine and cross-build determinism are explicitly **not** in scope (`PRD §6`). The tolerance must be tight enough to catch real nondeterminism (e.g. unordered atomic accumulation) but loose enough to absorb ULP-level drift from compiler reordering.

#### BDD-103 — Backend-boundary invariant holds

**Given** a candidate code change that introduces a new behavior or swaps a simulation stage (e.g., broad-phase, narrow-phase, integrator)
**When**  the change is reviewed for boundary compliance
**Then**  the change set touches the simulation pipeline only; the renderer (`src/shader/`, OpenGL render loop), GUI (ImGui windows in `src/main.cpp`, `src/*_gui.cpp`), and scene-IO layer are not modified; the existing test suite passes; rendered output for unchanged scenes is visually unchanged.

Notes: this is an architectural invariant the **Estimator** enforces during review. It is testable through diff-shape inspection and a regression render of canonical scenes — Generator and Estimator agree on the canonical scene set.
