# PRD — ysim

> Authored from the project brief on 2026-05-06. Owner: Planner reads; humans edit.
> When this document changes, follow up by reconciling `FRD.md`, `BDD.md`, `docs/TESTS.md`, and `docs/TEST_MATRIX.md`.

## 1. Problem

Authoring physically-based simulation scenes for offline rendering pipelines is split between two unsatisfying poles:

- **Houdini-class DCCs** are accurate and flexible but carry a steep complexity ceiling. The node-graph and parameter surface mean that a user who only wants to "drop a cloth on a sphere and bake the result for Unreal" pays the full DCC tax to do so.
- **Unreal/real-time engines** are approachable but optimize for frame budget over fidelity. Simulation results are good enough to play, but not good enough to ship as cinematics, and the export path back out to other tooling is awkward.

The user this product targets is a technically-fluent creator (TD, indie cinematic artist, graphics engineer) who needs *correct enough* physics for cloth and rigid body shots, wants a short path from "set up scene" to "export bake," and does not want to learn Houdini's authoring model to get there.

## 2. Outcome

When ysim ships its v1 milestone, the user can:

1. Open ysim, build a small scene (basic primitives + imported meshes), assign per-object simulation behaviors, configure global forces, run the sim, and export the result as Alembic — all without leaving the application and without scripting.
2. Save that scene to disk and reopen it later in the same state.
3. Take the Alembic into Unreal (or any Alembic-aware renderer) and drive a cinematic with it.

The measurable difference vs. the status quo is **time-to-first-bake** for a non-Houdini user: from "open application" to "Alembic file written for a cloth-on-sphere shot" should be achievable in a single sitting by someone who has read no manual beyond the in-app affordances.

A v2 (out of scope here, but designed-around) attaches an LLM control surface so the same operations can be driven by natural-language prompts. v1 must not foreclose that — see §4.

## 3. Scope (MVP v1)

### 3.1 Scene authoring

- **Primitive creation** — sphere and cube, parameterized by size and tessellation density.
- **External mesh import** — at minimum `.obj` (the project already ships an `objreader`). Future formats are not in v1 scope.
- **Per-object transform editing** — translate and rotate object center. Rotation is stored internally as a quaternion. The UI may surface Euler/axis-angle as an input affordance, but the canonical representation is quaternion to keep composition stable and to match how downstream simulation code consumes orientation.
- **Per-object material editing** — material parameters follow the **OpenPBR** surface model. v1 may expose a meaningful subset of OpenPBR (base color, metallic, roughness, specular, emission) rather than the entire spec; the subset must be a strict subset — names, value ranges, and units must match OpenPBR so a future expansion is additive only.

### 3.2 Simulation behaviors

Every scene object carries a `BehaviorType` tag. v1 requires three behaviors usable end-to-end:

- **Float** — object is unaffected by environment forces (gravity, wind). Used for kinematic anchors and visual-only meshes.
- **Cloth** — particle-system cloth, driven by spring/constraint forces, integrated on the GPU. The codebase currently exposes two cloth variants (`TriangularCloth` for arbitrary triangle meshes, `FastGridCloth` for regular grid topology). Both remain valid in v1; user-facing UI may present them as one "Cloth" choice with a topology hint, or as two — that decision is deferred to FRD.
- **Rigid** — environment-affected rigid body. v1 integrates **both Bullet and Jolt** as candidate backends. The choice between them is a **compile-time/code-level switch** in v1; a runtime selector is a v2 concern. The point of integrating both up front is to avoid binding the public scene-format and behavior API to a single physics library's idioms.

Each behavior has its own parameter struct. v1 ships sensible defaults; a richer parameter surface (per-behavior tuning panel) is planned but explicitly **not blocking** v1 acceptance — defaults must produce a usable result without tuning.

### 3.3 Environment forces

v1 supports two global forces:

- **Gravity** — vector, default `(0, -9.81, 0)`.
- **Wind** — vector force applied as an external force on susceptible behaviors.

Wind is modeled as a force in v1. There is a known follow-up to reformulate wind as an **air velocity field**, where the relevant quantity for each particle is its velocity *relative to the air*, not a force applied directly. This change must remain feasible without breaking the v1 scene format — see §5 (Constraints).

### 3.4 Export

Simulation results are exportable to **Alembic** (`.abc`). Alembic is the integration target because it is the lingua franca for baked simulation across DCCs and Unreal. v1 export must include, at minimum:

- per-frame vertex positions for simulated meshes,
- per-frame transforms for rigid bodies,
- topology (face indices) sufficient for the consumer to reconstruct the mesh.

Material export through Alembic is **not** required in v1 — Alembic is the geometry/cache format, not the material format. Material data is preserved in the scene save file (§3.5) but is not promised in the bake.

### 3.5 Scene persistence

Scenes can be **saved to** and **loaded from** disk. A scene file is the durable representation of: object list (primitive vs. imported, with file references for imports), per-object transforms (with quaternion rotation), per-object material parameters, per-object behavior tag and parameters, and global force settings. Reloading a saved scene must reproduce the same initial conditions; the simulation result itself is reproducible from those initial conditions plus the deterministic-enough integration choice the engine commits to.

Format choice (JSON, binary, custom) is deferred to FRD/architecture, but the format must be:
- human-diffable enough to support hand-editing in v1 (the LLM-control-surface v2 will read/write it),
- versioned, so format evolution does not silently corrupt old scenes.

## 4. Non-goals (v1)

These are explicitly out of v1 to keep the slice shippable. Each is listed with the reason so it does not silently scope back in.

- **LLM-driven scene control.** Deferred to v2. v1 must not block it: the scene file format and the behavior/force APIs should be expressible as structured data (so an LLM can later read/write scenes and call into a control API). No LLM code, prompts, or APIs ship in v1.
- **Self-collision for cloth.** Currently disabled in the engine due to instability. Re-enabling it is a follow-up; v1 ships without it and the UI does not pretend otherwise.
- **Spatial hashing as a broad-phase option.** Implementation exists but is too slow; v1 ships LBVH as the only broad-phase. Spatial hashing is parked, not deleted, in case a future revision makes it competitive.
- **Fluid, Elastic, Generator behaviors.** The `BehaviorType` enum already lists these; v1 does **not** ship them as user-facing options. They remain in the enum so the on-disk format reserves their identifiers, but the UI hides them and the engine returns an explicit "unsupported" if loaded from a scene that references them.
- **Runtime switching between Bullet and Jolt.** Code-level only in v1.
- **Air-as-velocity-field wind.** Force model only in v1; the velocity-field reformulation is the follow-up that must not be blocked by the v1 scene format.
- **Material export through Alembic.** Geometry/cache only.
- **Non-`.obj` mesh imports.** Add as needed post-v1.
- **Render quality polish beyond OpenPBR-compliant material editing.** ysim's renderer is for *scene authoring preview*, not for final-pixel rendering. Final pixels are the downstream consumer's job.
- **Cross-platform support.** v1 targets macOS only — the simulation backend is Metal-based, the build pipeline uses `xcrun metal`/`xcrun metallib`, and the architecture is dual-GPU (OpenGL render + Metal compute). The CPU backend exists in the type system (`MeshState<CPU,PR>`) for testability and future portability, but is not a shipping target in v1.

## 5. Constraints

Non-negotiable. The Planner enforces these on every plan; the Estimator flags violations.

### 5.1 Architectural constraints

- **Backend extensibility.** Core simulation types are template-parameterized on a backend tag (`CPU`, `METAL`, with `CUDA` reserved). New behaviors must be implemented in a way that does not assume Metal — even if the CPU implementation is a stub, the type structure must accept it. This is what makes a future port (CUDA, Vulkan compute, etc.) tractable.
- **Cache-friendly memory layout.** Per-object data that is accessed together in a kernel must be stored together. Memory-pool allocation (`ByteMemoryPool<BE>`, `MemoryBlock<METAL,T>`) is the v1 mechanism. New systems may not introduce ad-hoc allocations on the hot path.
- **Replaceable simulation parts.** Each stage of the simulation pipeline (broad phase, narrow phase, integration, constraint response) must be swappable without rewriting the others. The current LBVH-vs-spatial-hashing parking lot is the proof case for this.
- **Templates over inheritance.** Behavior dispatch is via templates and `std::variant`-style sum types, not virtual functions. The cost is compile time; the win is no vtable indirection on the hot path and no surprise heap layout. New behaviors follow the same pattern.

### 5.2 Forward-compatibility constraints

- **Scene format must survive the wind reformulation.** Wind is a force in v1; in v2 it becomes an air velocity field. v1 scene files must either store wind in a form that the v2 loader can interpret, or be versioned such that the v2 loader can migrate them without data loss.
- **Scene format must be LLM-addressable.** The format must be structured (named fields, enumerated behavior tags, explicit units) so that the v2 LLM control surface can read and write it without inferring conventions.
- **Behavior identifiers are reserved, not reused.** The `BehaviorType` enum values are part of the on-disk contract; reordering or renumbering them in code breaks saved scenes.

### 5.3 Platform constraints

- **macOS only** in v1 (Metal compute, OpenGL render, `xcrun` shader pipeline).
- **C++17, CMake 3.10+, Eigen 5.0+, GLFW 3.4, GLEW, OpenGL, Metal**, plus Bullet and Jolt as physics dependencies for the Rigid behavior.
- **ImGui (OpenGL backend)** for the GUI. The GUI uses the OpenGL backend rather than Metal because presentation is OpenGL-based.

## 6. Success metrics

ysim v1 is successful if all of the following hold for a representative shot (e.g., a cloth draped over a rigid sphere on a kinematic floor):

1. **Round-trip works.** Author the scene → save → reload → simulate → export Alembic → import in Unreal → playback matches the in-app simulation.
2. **Time-to-first-bake.** A user new to ysim, given only the in-app UI and `README.md`, can produce a first Alembic bake of the representative shot in under 60 minutes.
3. **No regression in the implemented core.** Existing capabilities continue to work: GPU-accelerated cloth simulation (Metal explicit-Euler integration), object-level and scene-level LBVH broad-phase collision, ray-pick object selection with color/behavior editing, and the `FrameProfiler` timing breakdown in the GUI.
4. **Backend boundary holds.** Adding a new behavior or swapping a simulation stage does not require touching the renderer or the scene-IO layer.
5. **Determinism, scoped.** Two runs of the same saved scene on the same machine produce visually identical bakes. Cross-machine determinism is not promised in v1.

## 7. Open questions

These are the questions the Planner will surface to the human when they become blocking. Resolve them before authoring the FRD entries that depend on them.

- **Q1 — OpenPBR subset.** Which OpenPBR parameters does v1 expose? Proposed minimum: base color, metallic, roughness, specular weight, emission color. Decision needed before the material UI is specified.
- **Q2 — Cloth UX surface.** Are `TriangularCloth` and `FastGridCloth` presented to the user as one option (engine picks based on topology) or two (user picks)? Affects the FRD wording for the Cloth assignment flow.
- **Q3 — Scene file format.** JSON (human-diffable, easy LLM target, slow for large bakes) vs. a custom format vs. a hybrid (JSON for scene, sidecar binary for heavy data). The forward-compat constraints in §5.2 push toward JSON or JSON-superset.
- **Q4 — Rigid backend default.** When v1 ships with both Bullet and Jolt code-selectable, which is the default the documentation and acceptance tests assume? Affects the success-metric round-trip.
- **Q5 — Alembic schema specifics.** Which Alembic schemas are written — `Xform` + `PolyMesh` for cloth, `Xform` for rigid? Does v1 also write velocities (some consumers use them for motion blur)?
- **Q6 — Export FPS / time-step decoupling.** The bake frame rate (e.g., 24/30/60 fps) is independent of the simulation substep. v1 needs a clear rule for how the user picks the export rate and how the engine resamples to it.
- **Q7 — Save-file forward migration policy.** When the v2 wind reformulation lands, do v1 saves auto-migrate on load, or does the user run an explicit migration? Affects how versioning is implemented in v1.

---

**Next steps for the Planner:** translate §3 into FRD entries (one per capability), author BDD scenarios for the success-metric round-trip and for backend-boundary invariants, and only then draft a concrete `.agent/PLAN.md` slice. Resolve Q1–Q4 with the human before writing FRD entries that depend on them.
