# Architecture

> Owner: **Planner**. Generators read this to know boundaries; Estimators read this to detect violations.
> Updated: 2026-05-06

## 1. System purpose

ysim is a macOS simulation engine for cloth and rigid-body shots, sitting between Houdini (too complex) and Unreal (real-time-biased). v1 is an end-to-end author → simulate → save → reload → export-to-Alembic flow. The architecture is shaped by four non-negotiable constraints from the PRD: backend extensibility (CPU/Metal today, more later), cache-friendly memory layout, replaceable simulation parts, and templates over inheritance on the hot path.

The architecture is dual-GPU: **OpenGL for rendering**, **Metal compute for simulation**. ImGui sits on the OpenGL backend because presentation is OpenGL-based.

## 2. Boxes

Each box is a conceptual subsystem. File-level structure is mostly flat (the bulk of v1 lives in `src/main.cpp`); the boxes below describe *responsibilities*, not files. The Estimator should flag any change that blurs a box's responsibilities into another's.

### 2.1 Application shell

The GLFW window, the main loop, and the ImGui context. Owns the input dispatch (mouse/keyboard → selection, gizmo, inspector edits) and the frame pacing. Does **not** own simulation state or rendering primitives — it tells those subsystems when to step and when to draw.

### 2.2 Scene model

The in-memory representation of an authored scene: object list (`GeneralMesh<BE, PR>`), per-object transforms (position + quaternion), materials, behavior tags (`BehaviorType`) and behavior parameter structs (`BehaviorParams<PR> = std::variant<...>`), global forces (`ExternalForces`). This is the shared truth read by the renderer, written by the simulation, edited by the GUI, and serialized by scene I/O.

The scene model is **template-parameterized on backend** (`Scene<METAL, PR>` today; `Scene<CPU, PR>` reserved). New behaviors extend the variant; new fields extend `MeshState` / `MeshAdjacency`. No virtual dispatch.

### 2.3 Renderer

OpenGL pipeline that reads the scene model and draws each frame. Owns shader programs (`src/shader/*.vert/.frag/.geom`), framebuffers, the camera, and ray-pick selection. Does **not** know about Metal, behavior dispatch, or collision detection — it consumes positions/normals/transforms and renders them.

### 2.4 GUI / inspector

ImGui-driven panels that mutate the scene model: object list, inspector for the selected object (transform, material, behavior tag + params), environment forces (gravity, wind), profiler window, scene I/O actions. Lives on the OpenGL ImGui backend. Edits propagate live (`BDD-018`); no pause/resume required.

### 2.5 Simulation pipeline

The hot path. One simulation step is a fixed sequence of stages, each replaceable independently of the others (`PRD §5.1`):

- **Behavior dispatch** — for each object, dispatch to its behavior's per-step kernel based on `BehaviorType`. v1 has user-facing `Float`, `Cloth` (`TriangularCloth` + `FastGridCloth`), and (planned) `Rigid`.
- **Force computation** — gravity + wind applied to non-`Float` objects, plus behavior-internal forces (e.g. spring forces for cloth).
- **Broad phase** — LBVH built per object and per scene; produces candidate collision pairs. Spatial-hashing variant exists (`src/metal/spatialhashing.metal`) but is parked for performance.
- **Narrow phase** — point-triangle intersection (`src/metal/bruteforce.metal`) over candidate pairs.
- **Integration** — explicit Euler for cloth today; rigid integration delegated to Bullet or Jolt (code-level switch, PRD Q4).
- **Constraint response** — collision constraints applied to corrected positions/velocities.

All stages run as Metal compute kernels (`src/metal/*.metal`) dispatched from C++ via a singleton `MetalGlobalContext` and `MetalKernelContext`. Memory is allocated through `ByteMemoryPool<METAL>` / `MemoryBlock<METAL, T>` — no ad-hoc allocations on the hot path (`PRD §5.1`).

### 2.6 Scene I/O (planned — persistence slice)

Save/load of the scene model to a versioned, human-diffable format (provisionally JSON, PRD Q3 → resolved by Planner; see `.agent/PROJECT_STATE.md`). Reads the same scene model the GUI mutates; reuses the same construction paths the GUI uses for primitive creation and mesh import (so authoring and loading produce identical in-memory state). Does **not** touch the simulation pipeline or the renderer.

### 2.7 Export (planned — export slice)

Alembic baker that consumes per-frame simulation output over a chosen frame range and writes a `.abc` file. Reads the simulation pipeline's per-step state but is **outside** the hot path — bake is a user-triggered, finite operation, not a per-frame cost.

### 2.8 Profiler

`FrameProfiler` + `FrameProfilerHistory` sample named timing sections in the simulation pipeline and surface them in a GUI window. CSV export to `profiles/`. New sections are registered dynamically by name. History collection pauses when simulation is paused.

## 3. Arrows

Who initiates, what crosses the boundary, sync vs. async.

```
                       ┌────────────────────┐
                       │ Application shell  │  (initiates everything)
                       └─────────┬──────────┘
                                 │
                ┌────────────────┼────────────────┐
                ▼                ▼                ▼
         ┌──────────┐     ┌──────────┐     ┌──────────────┐
         │   GUI    │     │ Renderer │     │  Simulation  │
         └────┬─────┘     └────▲─────┘     │   pipeline   │
              │ mutate         │ read       └──────▲───────┘
              ▼                │                   │ read+write
            ┌────────────────────────────┐         │
            │       Scene model          │◄────────┘
            └────┬───────────────▲───────┘
                 │ read           │ write
                 ▼                │
            ┌──────────┐    ┌──────────┐
            │ Scene IO │    │ Profiler │  (samples sim pipeline)
            └──────────┘    └──────────┘
                                 ▲
                                 │
                            ┌────┴─────┐
                            │  Export  │  (reads sim output frames)
                            └──────────┘
```

- **Shell → everything**: synchronous per-frame call. Shell decides when to step the sim and when to draw.
- **GUI → Scene model**: synchronous, mutates fields directly. Lives on the same thread as the renderer.
- **Renderer → Scene model**: synchronous read each frame. Never writes back.
- **Simulation pipeline → Scene model**: read at start of step, write at end of step. Mutation happens through Metal-resident buffers; the C++-side `Scene` views the same memory through `MemoryBlock<METAL, T>`.
- **Scene I/O ↔ Scene model**: synchronous, only on user save/load actions. Never invoked from the hot path.
- **Profiler → Simulation pipeline**: passive — section markers are inserted *into* the simulation kernels; profiler aggregates results. Profiler does not steer the simulation.
- **Export → Simulation pipeline**: pulls per-frame output during a finite bake operation. Not on the hot path.

There is **no** asynchronous boundary in v1. Single-threaded except for what Metal does internally on the GPU.

## 4. Boundaries (Estimator-enforced invariants)

Violations of any of these are a `BLOCK`-class concern.

### 4.1 Backend boundary (`BDD-103`)

The renderer, GUI, scene I/O, and export must **not** depend on Metal-specific types or kernels. They consume the scene model through its template-parameterized interface only. Adding a new behavior or swapping a simulation stage must not require touching `src/shader/*`, the OpenGL render loop, ImGui windows, or the scene-IO code.

Concretely: a diff that introduces a new collision pipeline and also touches `src/main.cpp`'s render or GUI sections is suspect — the Estimator should ask whether the GUI/render edits are *consequence* (a new inspector field for new behavior parameters) or *coupling* (the renderer learning about the new pipeline).

### 4.2 No virtual dispatch on the hot path (`PRD §5.1`)

Behavior dispatch goes through templates and `std::variant`; no `virtual` functions in `MeshState`, `Scene`, `BehaviorParams`, or any per-step code. New behaviors extend the variant. The cost is compile time; the win is no vtable indirection and predictable layout.

### 4.3 No ad-hoc allocations on the hot path

All per-frame buffers go through `ByteMemoryPool<BE>` / `MemoryBlock<BE, T>`. New per-frame data structures must allocate from the pool, not via `new`/`malloc`/`std::vector::push_back` on each step.

### 4.4 BehaviorType identifiers are reserved

The `BehaviorType` enum values (`TriangularCloth`, `FastGridCloth`, `Elastic`, `Rigid`, `Float`, `Fluid`, `Generator`) are part of the on-disk scene format. Reordering or renumbering them silently corrupts saved scenes. Adding a new behavior appends to the enum; never inserts into the middle.

### 4.5 macOS-only platform surface (v1)

The simulation backend depends on Metal (`xcrun metal` / `metallib`). The CPU backend exists in the type system for testability and future portability but is **not** a v1 shipping target. Code that gates Metal-specific logic must do so through the backend tag (`<METAL, PR>`), not through `#ifdef __APPLE__`.

### 4.6 Single-machine determinism, only

`BDD-102` promises that two runs of the same saved scene on the same machine produce visually identical bakes. Cross-machine and cross-build determinism are **not** promised. Code reviews that introduce new sources of nondeterminism (e.g., unordered atomic accumulation in a kernel) within a single machine should be flagged; cross-machine drift should not be escalated to BLOCK in v1.

## 5. Open structural questions

The Planner is tracking these. Each lists what would force a decision.

- **Q-A — Where does the Rigid behavior's Bullet/Jolt adapter live?** It cannot live in the simulation pipeline alongside the Metal kernels (Bullet and Jolt are CPU libraries). Options: (i) a sibling box "rigid bridge" that runs on CPU between Metal sim steps and feeds positions back to GPU buffers; (ii) ship rigid as a `<CPU, PR>` `Scene` view, decoupling rigid's backend from cloth's. Decision forced by the rigid-body slice (`FR-008` / `BDD-008`).
- **Q-B — How do persisted scenes reference imported meshes when the asset moves?** v1 references by path; if the path becomes invalid, `loadScene` fails. Embedding mesh data in the scene file is the alternative. Decision forced by user reports of broken loads, or by the LLM-control v2 needing to manipulate scenes without their assets present.
- **Q-C — How is the Alembic export decoupled from the live simulation step rate?** The bake frame rate (e.g., 24/30/60 fps) is independent of the simulation substep. Open: does the export pull from a recorded buffer of all sub-steps and resample to the requested rate, or does the simulation run at the export rate during a bake? Decision forced by the export slice (PRD Q6 → blocks `FR-013`).
- **Q-D — Who owns the test backend (CPU)?** The CPU backend is reserved in the type system but unused. If `BDD-102` (single-machine determinism) and `BDD-103` (backend-boundary) need verification, a CPU implementation of at least one stage may be needed as a reference. Decision forced by the determinism slice or by Estimator pressure on backend-boundary claims.
