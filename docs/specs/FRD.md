# FRD — Functional Requirements

> Authored from `PRD.md` on 2026-05-06. One requirement per entry.
> The Planner translates these into todo items in `.agent/PLAN.md` and into matrix rows in `docs/TEST_MATRIX.md`.
> Where a requirement depends on an unresolved PRD open question, the dependency is called out in **Notes**.

## Format

```
## FR-NNN — <short title>

The system **shall** <observable behavior>.

- Trigger: <what causes this>
- Inputs: <what it depends on>
- Output: <what it produces / changes>
- Acceptance: <pointer to TESTS.md>
```

## Requirements

### Scene authoring

#### FR-001 — Create primitive object

The system **shall** allow the user to create a sphere or cube primitive in the active scene.

- Trigger: user invokes "Create primitive" via the GUI and selects a shape.
- Inputs: shape kind (`Sphere` | `Cube`), size, tessellation density, optional initial transform.
- Output: a new scene object with a generated mesh, default material, default `BehaviorType::Float`, and an entry in the scene's object list.
- Acceptance: see `docs/TESTS.md#BDD-001`.

#### FR-002 — Import external mesh

The system **shall** allow the user to import an external mesh into the scene.

- Trigger: user invokes "Import mesh" and selects a file.
- Inputs: file path. v1 supports `.obj` (consumed by `include/objreader.hpp`).
- Output: a scene object whose geometry is populated from the file, with default material and `BehaviorType::Float`. The scene file records the import path so reload reproduces the same source.
- Acceptance: see `docs/TESTS.md#BDD-002`.
- Notes: non-`.obj` formats are explicitly out of v1 (PRD §4).

#### FR-003 — Edit object position

The system **shall** allow the user to translate the center of any selected object.

- Trigger: user edits position fields in the inspector or drags a transform gizmo.
- Inputs: target object id, new translation vector.
- Output: object's center position is updated; downstream simulation reads the new position on the next step.
- Acceptance: see `docs/TESTS.md#BDD-003`.

#### FR-004 — Edit object rotation

The system **shall** allow the user to rotate the center of any selected object, with rotation stored canonically as a quaternion.

- Trigger: user edits rotation in the inspector.
- Inputs: target object id, new rotation (UI may expose Euler/axis-angle as input affordances).
- Output: object's quaternion is updated and renormalized; the same quaternion is used for rendering, simulation, and persistence.
- Acceptance: see `docs/TESTS.md#BDD-004`.
- Notes: canonical representation is quaternion (PRD §3.1).

#### FR-005 — Edit object material (OpenPBR subset)

The system **shall** allow the user to edit OpenPBR material parameters of any selected object.

- Trigger: user edits material fields in the inspector.
- Inputs: target object id, parameter name, new value.
- Output: the object's material is updated; preview rendering reflects the change immediately.
- Acceptance: see `docs/TESTS.md#BDD-005`.
- Notes: depends on PRD Q1 — proposed v1 subset is base color, metallic, roughness, specular weight, emission color. Names, value ranges, and units must match OpenPBR so future expansion is additive.

### Simulation behaviors

#### FR-006 — Assign behavior type

The system **shall** allow the user to assign a `BehaviorType` to any selected object.

- Trigger: user changes the behavior selector in the inspector.
- Inputs: target object id, new `BehaviorType` (v1 user-facing choices: `Float`, `Cloth`, `Rigid`).
- Output: object's behavior tag and parameter struct are updated to defaults for the new behavior; the simulation loop dispatches to the new behavior on the next step.
- Acceptance: see `docs/TESTS.md#BDD-006`.
- Notes: `Elastic`, `Fluid`, `Generator` remain as reserved enum values (PRD §4) but are not selectable in v1 UI. Depends on PRD Q2 (single vs. dual cloth surface) — current decision: present `TriangularCloth` and `FastGridCloth` as separate selectable options because the codebase already distinguishes them and merging them later is cheaper than splitting later.

#### FR-007 — Cloth simulation step

The system **shall** advance every object with a cloth behavior by one simulation step on the GPU.

- Trigger: simulation loop tick (running, not paused).
- Inputs: per-object cloth state (positions, velocities, masses, rest geometry), cloth parameters, environment forces, collision constraints from the broad/narrow phases.
- Output: updated positions and velocities for the object, written into the GPU-resident state buffers.
- Acceptance: see `docs/TESTS.md#BDD-007`.
- Notes: v1 uses explicit Euler integration (already implemented). Self-collision is parked (PRD §4).

#### FR-008 — Rigid simulation step

The system **shall** advance every object with a rigid behavior by one simulation step using the configured rigid backend.

- Trigger: simulation loop tick.
- Inputs: per-object rigid state, rigid parameters, environment forces, collision constraints.
- Output: updated transform (position + quaternion) for the object.
- Acceptance: see `docs/TESTS.md#BDD-008`.
- Notes: v1 integrates both Bullet and Jolt; selection is a code-level switch (PRD §3.2). Depends on PRD Q4 for the documentation/test default.

#### FR-009 — Float behavior is force-exempt

The system **shall** leave a `Float` object's velocity and position unchanged by environment forces during a simulation step.

- Trigger: simulation loop tick.
- Inputs: object with `BehaviorType::Float`, any active environment forces.
- Output: object's transform is unchanged unless it is moved by user input or directly authored animation.
- Acceptance: see `docs/TESTS.md#BDD-009`.

#### FR-010 — Collision detection (broad + narrow phase)

The system **shall** detect collisions between simulated objects each step using a two-stage pipeline: LBVH broad phase followed by point-triangle narrow phase.

- Trigger: simulation loop tick.
- Inputs: all simulated meshes' current positions, per-object and scene-level BVH structures.
- Output: a constraint set consumed by behavior-specific responses.
- Acceptance: see `docs/TESTS.md#BDD-010`.
- Notes: spatial-hashing variant exists but is parked (PRD §4). Self-collision is parked.

### Environment forces

#### FR-011 — Configure gravity

The system **shall** allow the user to set a global gravity vector that applies to all non-`Float` objects.

- Trigger: user edits the gravity vector in the environment panel.
- Inputs: 3-component vector (default `(0, -9.81, 0)`).
- Output: subsequent simulation steps use the new gravity.
- Acceptance: see `docs/TESTS.md#BDD-011`.

#### FR-012 — Configure wind

The system **shall** allow the user to set a global wind force vector that applies to wind-susceptible behaviors.

- Trigger: user edits the wind vector in the environment panel.
- Inputs: 3-component vector (default `(0, 0, 0)`).
- Output: subsequent simulation steps add wind as an external force to susceptible objects.
- Acceptance: see `docs/TESTS.md#BDD-012`.
- Notes: v1 models wind as a force. PRD §3.3 calls out a v2 reformulation as an air velocity field; the scene format must be able to migrate (PRD §5.2).

### Export

#### FR-013 — Export simulation to Alembic

The system **shall** export a simulation run to a single Alembic (`.abc`) file containing per-frame geometry and transforms over a user-specified frame range.

- Trigger: user invokes "Export Alembic" with a target path, frame range, and frame rate.
- Inputs: the active scene, the simulation's output stream over the requested frame range.
- Output: an `.abc` file containing per-frame vertex positions for simulated meshes, per-frame transforms for rigid bodies, and topology sufficient to reconstruct meshes.
- Acceptance: see `docs/TESTS.md#BDD-013`.
- Notes: depends on PRD Q5 (schema choices) and Q6 (export FPS rule). Material export through Alembic is **not** in v1 (PRD §4).

### Scene persistence

#### FR-014 — Save scene

The system **shall** save the current scene to a file on disk.

- Trigger: user invokes "Save scene" with a target path.
- Inputs: the active scene (object list, per-object transforms incl. quaternion rotation, materials, behavior tags + parameters, global forces).
- Output: a versioned scene file at the target path. Format is structured, human-diffable, and addressable by a future LLM control surface (PRD §5.2).
- Acceptance: see `docs/TESTS.md#BDD-014`.
- Notes: depends on PRD Q3 (concrete format). Imported meshes are referenced by path, not embedded, in v1.

#### FR-015 — Load scene

The system **shall** load a previously saved scene from disk and reproduce its initial conditions.

- Trigger: user invokes "Load scene" with a source path.
- Inputs: a scene file produced by FR-014.
- Output: the scene state matches what was saved; running the simulation from a fresh state produces the same initial trajectory as the original session would have.
- Acceptance: see `docs/TESTS.md#BDD-015`.

#### FR-016 — Scene file is versioned

The system **shall** write a format version into every scene file and reject incompatible versions with an explicit error.

- Trigger: save (writes version) or load (reads and checks version).
- Inputs: a scene file's version field.
- Output: on save, a version tag is included; on load, an unsupported version produces a clear error rather than silent corruption.
- Acceptance: see `docs/TESTS.md#BDD-016`.

### Existing capabilities (no-regression)

These mirror PRD success metric #3 — already implemented and must keep working through v1.

#### FR-017 — Ray-pick selection

The system **shall** select the nearest object hit by a ray cast from the camera through the cursor on click.

- Trigger: user clicks in the viewport.
- Inputs: cursor screen position, camera, scene geometry.
- Output: the hit object becomes the selected object; the inspector updates.
- Acceptance: see `docs/TESTS.md#BDD-017`.

#### FR-018 — Inspector edits propagate live

The system **shall** apply inspector edits (color, behavior, transform, material) to the selected object without restarting the simulation.

- Trigger: user edits a field in the inspector.
- Inputs: target object id, field, new value.
- Output: the change is visible in the next render; if simulation is running, the next sim step uses the new value.
- Acceptance: see `docs/TESTS.md#BDD-018`.

#### FR-019 — Frame profiler

The system **shall** measure named timing sections in the simulation loop and surface them in a GUI window, with the ability to export to CSV.

- Trigger: simulation loop runs (collection); user opens the profiler window (display); user invokes "Export CSV" (export).
- Inputs: section start/end markers placed in the simulation pipeline.
- Output: a per-section timing table in the GUI; CSV files written under `profiles/`.
- Acceptance: see `docs/TESTS.md#BDD-019`.
- Notes: history collection pauses when simulation is paused (per `CLAUDE.md`).
