# BDD — Behavior-Driven Design

> Authored from `PRD.md` and `FRD.md` on 2026-05-06. Each behavior is uniquely identified by `BDD-NNN` and referenced from `docs/TESTS.md`, `docs/TEST_MATRIX.md`, and commit messages.
>
> Each entry below is paired 1:1 with an `FR-NNN` in `FRD.md`. A handful of cross-cutting behaviors (round-trip, determinism, backend-boundary) follow as `BDD-101+`.

## Format

```
## BDD-NNN — <short title>

As a **<actor>**
I want **<function>**
So that **<effect>**

Acceptance scenarios → see `docs/TESTS.md#BDD-NNN`
```

## Behaviors

### Scene authoring

#### BDD-001 — Create primitive object

As a **scene author**
I want **to add a sphere or cube to the scene with one action**
So that **I can start building a simulation without leaving the application or scripting**

Pairs with: `FR-001`. Acceptance scenarios → see `docs/TESTS.md#BDD-001`.

#### BDD-002 — Import external mesh

As a **scene author**
I want **to import a `.obj` mesh and have it become a normal scene object**
So that **I can use my own geometry alongside primitives without per-mesh setup**

Pairs with: `FR-002`. Acceptance scenarios → see `docs/TESTS.md#BDD-002`.

#### BDD-003 — Translate object

As a **scene author**
I want **to set an object's position by entering or dragging a value**
So that **I can stage scenes precisely without scripting transforms**

Pairs with: `FR-003`. Acceptance scenarios → see `docs/TESTS.md#BDD-003`.

#### BDD-004 — Rotate object (quaternion-canonical)

As a **scene author**
I want **to rotate an object and have the rotation behave consistently across editing, simulation, and persistence**
So that **I can rely on stable composition and round-trip without unexpected drift**

Pairs with: `FR-004`. Acceptance scenarios → see `docs/TESTS.md#BDD-004`.

Notes: the canonical representation is a quaternion (PRD §3.1). The behavior under test is round-trip stability — saving a rotation and reloading it yields the same orientation, and composing many small rotations does not accumulate normalization error.

#### BDD-005 — Edit OpenPBR material

As a **scene author**
I want **to tune an object's material with PBR parameters I already understand**
So that **the preview matches what downstream renderers will see**

Pairs with: `FR-005`. Acceptance scenarios → see `docs/TESTS.md#BDD-005`.

### Simulation behaviors

#### BDD-006 — Assign behavior type

As a **scene author**
I want **to choose how an object simulates (Float, Cloth, Rigid)**
So that **I can mix kinematic, soft, and rigid elements in the same scene**

Pairs with: `FR-006`. Acceptance scenarios → see `docs/TESTS.md#BDD-006`.

#### BDD-007 — Cloth drapes under gravity and collides

As a **scene author**
I want **a cloth-tagged object to fall under gravity and rest on whatever it collides with**
So that **I can produce a draped-cloth shot without writing simulation code**

Pairs with: `FR-007`, also exercises `FR-010`, `FR-011`. Acceptance scenarios → see `docs/TESTS.md#BDD-007`.

#### BDD-008 — Rigid body falls and rests

As a **scene author**
I want **a rigid-tagged object to fall under gravity and come to rest on a static surface**
So that **I can stage rigid-body dynamics with the engine's defaults**

Pairs with: `FR-008`. Acceptance scenarios → see `docs/TESTS.md#BDD-008`.

#### BDD-009 — Float object ignores environment

As a **scene author**
I want **a Float-tagged object to stay where I put it regardless of gravity or wind**
So that **I can use it as an anchor or visual-only mesh in a simulating scene**

Pairs with: `FR-009`. Acceptance scenarios → see `docs/TESTS.md#BDD-009`.

#### BDD-010 — Collisions detected between simulated objects

As a **scene author**
I want **simulated objects to detect contact with each other**
So that **cloth lands on rigid, rigid lands on rigid, and nothing tunnels through trivial scenes**

Pairs with: `FR-010`. Acceptance scenarios → see `docs/TESTS.md#BDD-010`.

### Environment forces

#### BDD-011 — Set gravity

As a **scene author**
I want **to change gravity for the whole scene**
So that **I can test edge cases (zero-g, sideways, planet defaults) without restarting**

Pairs with: `FR-011`. Acceptance scenarios → see `docs/TESTS.md#BDD-011`.

#### BDD-012 — Set wind

As a **scene author**
I want **to apply a global wind to the scene**
So that **cloth and other susceptible behaviors react to a directional force**

Pairs with: `FR-012`. Acceptance scenarios → see `docs/TESTS.md#BDD-012`.

### Export

#### BDD-013 — Export Alembic bake

As a **scene author**
I want **to export a simulated frame range as Alembic**
So that **I can take the result into Unreal or another DCC and render the final frames there**

Pairs with: `FR-013`. Acceptance scenarios → see `docs/TESTS.md#BDD-013`.

### Scene persistence

#### BDD-014 — Save scene

As a **scene author**
I want **to save the current scene to a file**
So that **I can stop work and resume it later, or share the setup with collaborators**

Pairs with: `FR-014`. Acceptance scenarios → see `docs/TESTS.md#BDD-014`.

#### BDD-015 — Load scene

As a **scene author**
I want **to reopen a saved scene and find it identical to how I saved it**
So that **resuming work is reliable and I can re-run a bake from a known state**

Pairs with: `FR-015`. Acceptance scenarios → see `docs/TESTS.md#BDD-015`.

#### BDD-016 — Reject incompatible scene file version

As a **scene author**
I want **the application to refuse to load an unrecognized scene file with a clear message**
So that **I never lose data to a silent migration error**

Pairs with: `FR-016`. Acceptance scenarios → see `docs/TESTS.md#BDD-016`.

### Existing capabilities (no-regression)

#### BDD-017 — Ray-pick selection

As a **scene author**
I want **to click an object in the viewport to select it**
So that **I can edit its parameters without hunting through a list**

Pairs with: `FR-017`. Acceptance scenarios → see `docs/TESTS.md#BDD-017`.

#### BDD-018 — Inspector edits propagate live

As a **scene author**
I want **to change an object's color or behavior and see the result immediately, including while simulating**
So that **iteration is fast and I do not need to restart the simulation to try a tweak**

Pairs with: `FR-018`. Acceptance scenarios → see `docs/TESTS.md#BDD-018`.

#### BDD-019 — Frame profiling visible and exportable

As a **simulation engineer**
I want **to see per-section timings of the simulation loop in the GUI and export a CSV**
So that **I can find the slow part of a frame and track regressions over runs**

Pairs with: `FR-019`. Acceptance scenarios → see `docs/TESTS.md#BDD-019`.

### Cross-cutting behaviors

#### BDD-101 — End-to-end round-trip (PRD success metric #1)

As a **scene author**
I want **to author → simulate → save → reload → export → consume the result in Unreal as one continuous flow**
So that **ysim is a complete tool for the cloth-on-sphere class of shot rather than a step in someone else's pipeline**

Acceptance scenarios → see `docs/TESTS.md#BDD-101`.

Notes: this is the user-visible spine of v1. It exercises `FR-001`, `FR-002`, `FR-006`, `FR-007`, `FR-010`, `FR-011`, `FR-013`, `FR-014`, `FR-015`. Failure here is a v1 ship-blocker even if every individual FR passes.

#### BDD-102 — Determinism on a single machine (PRD success metric #5)

As a **scene author**
I want **two runs of the same saved scene on the same machine to produce visually identical bakes**
So that **I can iterate on materials or framing without the underlying motion shifting beneath me**

Acceptance scenarios → see `docs/TESTS.md#BDD-102`.

Notes: scoped to single-machine determinism. Cross-machine determinism is explicitly **not** promised in v1 (PRD §6).

#### BDD-103 — Backend-boundary holds (PRD success metric #4)

As a **simulation engineer**
I want **to swap a simulation stage (e.g., broad phase) without touching the renderer or scene-IO layer**
So that **the architectural promise of "replaceable parts" is real, not aspirational**

Acceptance scenarios → see `docs/TESTS.md#BDD-103`.

Notes: this is an architectural invariant rather than a user behavior. The Estimator should flag plans that violate it. Concretely: an attempt to introduce a new behavior or a new collision pipeline must not require edits in the rendering, GUI, or scene-persistence subsystems.
