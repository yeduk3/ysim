# ysim Refactor Blueprint

Companion to `architecture.html` (interactive diagram). This is the prose + decisions record.

Goal: take one 20k-line `src/main.cpp` → swappable, render-agnostic, headless-capable sim for **research** (swap algorithms freely, run off-screen, sweep params).

---

## 1. Current state (what we refactor FROM)

`src/main.cpp` ≈ 20,713 lines. Everything fused. Key landmarks:

| Concept | Where | Note |
|---|---|---|
| Backend tags | `main.cpp:54` (`CPU`/`CUDA`/`METAL`) | seam works — keep |
| Memory pools / allocators | `:79`+ `ByteMemoryPool`, `:374` `DynamicByteMemoryPool`, `:509` `GlobalAutoAllocator` | keep |
| Metal context | `:126` `MetalGlobalContext`, `:200` `MetalKernelContext` (singletons) | keep, inject where practical |
| Mesh / state | `:789` `MeshState`, `:811` `MeshAdjacency`, many `*Initializer` | live state tangled with description |
| Behaviors | `:2010` `TriangularClothBehavior`, `:2080` `FastGridClothBehavior` | per-material force models |
| **Scene** | `:2772` (~700 lines) | mixes data + GPU upload + behavior dispatch + live state |
| Spatial hashing | `:3540` `SpatialHashing`, `:4500` `MultiLevelSpatialHashing` (hgrid) | broad-phase algos |
| BVH | `:4966`+ (LINEAR/SCENE modes) | broad-phase algo |
| Narrow phase | `:7372` `BruteForce<CPU/METAL>` | point-triangle |
| CD pipeline shell | `:7717` `CollisionPipeline` | tiny; real work fused into Simulator |
| **Simulator** | `:7727` (~3300 lines) | sim + GL buffers + ImGui + GUI windows + bench, all fused |
| System | `:11022` `ExplicitSystem<CPU>`, `:11172` `<METAL>` | integrator, no shared seam |
| Bench configs | `:11463`–`:12565` (6 structs) | belong in `/bench`, not the sim |
| SimulatorBuilder | `:17652` | partial, uncommitted → **dropped** (see decisions) |
| GUI | `profiler_gui.cpp` (235), `mesh_inspector_gui.cpp` (659) | separate files, but driven from Simulator |

Selection of algorithms today = env vars (`YSIM_SUBOBJECT`, `YSIM_HUMAN_STATIC`) + branching. RunConfig JSON exists (`sim_config.hpp`, uncommitted) for data-driven setup.

---

## 2. Target architecture (what we refactor TO)

Data flow left→right (mirrors the diagram columns):

```
setup*() / loadFromConfig()           ← build & configure (free functions, NO builder)
        │
   Runner ── owns N ── Simulator      ← the hub
                          │ owns: Scene, SimState, CDPipeline, System,
                          │       ConstraintSet, Profiler, LUT
   step() = dcd → accumulate → integrate → ccd → recover   (per substep)
```

- **Scene** (static): topology, collision groups, `isStatic`, primitive layout, materials. Immutable after build. Passive fat struct — algos read fields directly.
- **SimState** (live): GPU positions/velocities. Passed per-substep. *Keystone of render split.*
- **CDPipeline** (abstract base, the **collision swap unit**): `dcd(const Scene&, SimState&, ConstraintSet&)` / `ccd(...)`.
  - A concrete pipeline is the researcher's choice:
    - **(A) composed** — `BroadPhase` part → optional `CandidateSet` carrier → `NarrowPhase` part.
    - **(B) monolithic / fused** — no broad→narrow handoff, no candidate buffer written to global memory.
  - `BroadPhase` / `NarrowPhase` / `CandidateSet` are **optional internal parts**, concrete reusable helpers — NOT forced virtual interfaces. Existing BVH/SH/hgrid/BruteForce become parts and/or whole pipelines.
- **ConstraintSet**: contacts buffer. Simulator-owned scratch, reused across substeps. `ccd()` writes, `recoveryPenetration()` reads. Explicit param-passing.
- **System** (abstract base): `accumulate()` / `integration()` / `recoveryPenetration(state, contacts)`. Explicit / Implicit / XPBD swap.
- **LUT** (`name → handle`): unified bus. Two views — render-out (float buffers) + params. `bind<T>("wind", &w, Live)`. Merges what would've been FrameSnapshot + DataRegistry.
- **IRenderer**: `init(SceneDescription)` once + `draw()` reads LUT render-out by name each frame. YGL / GL / Offscreen(null=headless). Sim names zero GL types.
- **Profiler** (+History): runtime level (Off/Coarse/Fine) set on Sim; `export() → CSV/JSON`.

---

## 3. The 5 goals (user's)

1. **Algorithm swap easy** (research) — CDPipeline is the swap unit; System swappable; selection at code-level or paused-swap.
2. **All sim config in Simulator** — no Builder; `setup*(Simulator&)` (code) or `loadFromConfig` (JSON). One generic Simulator.
3. **Sim / render separated** — Simulator exports render-friendly data via LUT; any external renderer (incl. YGL) consumes it; off-screen + Runner batch possible.
4. **Profiler everywhere, leveled, exportable** — level preset on Simulator; export results.
5. **Profiling + GUI windows separated, talk via user-defined data dict** — LUT of typed pointers; windows read/write; shared pointer flows back to sim (rebuild / restart-frame-0 on structural change).

---

## 4. Invariants (enforce in code)

- **(a) Cache the KEY, never the pointer.** Renderer/GUI hold the LUT name and re-lookup each frame. Rebuild re-points the entry → no dangling pointer.
- **(b) Structural swap = paused-only.** `assert(paused)` before destroy+recreate of CDPipeline/System. This is what makes (a) race-free (no concurrent read of a freed GPU buffer).
- **(c) No Simulator subclass.** One generic `Simulator`. Scene specificity lives in `setup*()` free functions, never in subtypes — else goal 2 erodes into copy-paste hell.
- **(d) Live state ∉ Scene.** `Scene` = static description; live GPU pos/vel = `SimState`. If live state leaks into Scene, the clean render-export (goal 3) breaks.

---

## 5. Design decisions (why, settled in discussion)

- **CDPipeline interface is fine despite high call frequency.** Virtual dispatch is at *phase* granularity: ~50 substeps × 2 phases × 60fps ≈ 6k calls/s × ~1–2ns = ~12µs/s. Each call dispatches **ms** of GPU work (`commitAndWait` sync floor is the real cost — see BVH refit diagnosis). Per-primitive work stays in Metal kernels, never through a C++ vtable. **Rule: virtual boundary coarse (per phase), never on the inner loop.**

- **Swap unit = whole CDPipeline, not broad/narrow separately.** Forcing a common broad→narrow candidate type would flatten away per-algo wins (BVTT-front reuse, query coherence, fused inline tests). Keep parts as optional internal composition; allow monolithic. Fused (skip CandidateSet) can be *faster*, not just simpler — avoids writing the pair list to global memory.

- **No Builder.** Research codebase writes a new scene per experiment → fluent builder is over-engineering. Plain constructor + member assignment in a `setup*()` free function = clearer, more flexible, full C++ expressiveness, fast iteration. Builder only pays for data-driven/combinatorial config — that path is `loadFromConfig` over the existing RunConfig JSON. **Two separate paths; don't force JSON for experiments (serialization friction kills iteration), don't hardcode repro scenes (can't sweep).** Build `loadFromConfig` only when a sweep actually needs it (YAGNI).

- **Param-passing, not observer.** The step() loop is a fixed, known sequence — consumers are known at compile time. Explicit `ccd(.., contacts); recoveryPenetration(.., contacts);` is clearer than hidden observer control flow. Observer is for unknown/dynamic consumers; the UI side is already covered by LUT polling. → no observer pattern anywhere. ConstraintSet owned by Simulator as reused scratch (no per-substep alloc churn).

- **FrameSnapshot + DataRegistry unified into one LUT.** Same mechanism (`name → handle`), two uses (render-out + params). Avoids dangling via invariant (a). Same-machine renderers share the GPU buffer handle (don't copy → don't re-trigger the sync floor); POD copy only for offscreen/cross-process.

- **Runner ≠ free parallelism (honest).** N Simulators share one device/queue; each `commitAndWait` serializes them → it's a headless serial loop + N× memory. Real speedup needs a batched command buffer. Don't sell it as parallel.

- **Profiler is a runtime branch, not compiled-out.** Research wants a runtime level toggle. Cost = one bool check per section — negligible vs GPU work. (Earlier "compiles out / zero cost" claim was wrong.)

- **Keep what works.** Backend tags, memory pools/allocators, MetalContext — the backend seam already functions. Don't refactor what isn't in the way.

---

## 6. Attack order (phasing)

Slice by slice, **compile every step**. Simulator (~3300 fused lines) is the risk node.

1. **SimState + LUT seam** — separate live state from Scene; expose render-out handles. *Render-split keystone; unlocks goals 1 & 3.*
2. **Carve GL/ImGui out of Simulator** → IRenderer + GUI windows reading the LUT.
3. **Extract SceneDescription** — static render info to renderer.init() once.
4. **Hoist CDPipeline** behind the abstract base; move existing BVH/SH/hgrid/BruteForce into parts/pipelines.
5. **Hoist System** behind its base.
6. **Move bench configs** (`:11463`–`:12565`) → `/bench`.
7. **Runner** + (if/when needed) `loadFromConfig` for sweeps.

Profiler leveling + ConstraintSet scratch fold in alongside steps 4–5.

---

## 7. Skipped / open

- No source refactoring done yet — this + the diagram are the plan only.
- `loadFromConfig` / Runner: YAGNI until a sweep needs them; RunConfig JSON already exists.
- Determinism/repro seed for reproducible runs: tie into RunConfig when Runner lands.
- Batched-command-buffer Runner (true parallel): out of scope until headless serial proves insufficient.
