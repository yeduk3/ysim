# Conventions

> Semi-auto generated. Re-derive when the project's stack changes. Keep it short — anything the linter enforces does not belong here.
> Updated: 2026-05-06

## Language / stack

- **Language:** C++17.
- **Build system:** CMake 3.10+. Build with `cmake -B build && cmake --build build`. Run with `./build/src/ysim`.
- **Platform:** macOS only (v1). The simulation backend is Metal; the build pipeline shells out to `xcrun metal` and `xcrun metallib`.
- **GPU:** Metal compute for simulation, OpenGL for rendering. ImGui uses the OpenGL backend.
- **Dependencies:** Eigen 5.0+, GLFW 3.4, GLEW, OpenGL, Metal, Bullet, Jolt (when rigid lands), nlohmann/json (when persistence lands — see `docs/DECISIONS.md`).
- **Test framework:** **doctest** — single-header, drops in like the existing `stb_image.h`. Wired into CMake by the persistence slice. *Provisional Planner decision; see `docs/DECISIONS.md` once recorded.*
- **Linter / formatter:** none enforced in v1. A `.clang-format` is a planned follow-up but not a v1 requirement.

## What lives where

- `src/` — production code. The bulk of v1 lives in `src/main.cpp` (~5200 lines: simulation setup, main loop, GUI integration). GUI helpers (`mesh_inspector_gui.cpp`, `profiler_gui.cpp`) and Metal C++ glue (`metal_impl.cpp`) live alongside it. Throwaway code goes nowhere — delete instead. (`temp.cpp` and `test_temp.cpp` are pre-existing scratch files; do not add more of them.)
- `src/metal/` — Metal compute kernels (`.metal`) and their shared types (`common_types.metalh`). One file per pipeline stage (`physics.metal`, `bvh.metal`, `bruteforce.metal`, `radixsort.metal`, `spatialhashing.metal`).
- `src/shader/` — OpenGL shaders (`.vert`, `.frag`, `.geom`).
- `include/` — public headers and small third-party single-header libraries (`stb_image.h`).
- `third_party/` — vendored external libraries (currently `imgui`).
- `lib/` — built/static libraries (output, not source).
- `assets/` — runtime assets used by example scenes.
- `profiles/` — output directory for `FrameProfiler` CSVs. Path is wired via `YSIM_PROJECT_ROOT` CMake define.
- `test/` — *planned* — created by the persistence slice. Mirrors `src/` structure where a test maps to a single source file; otherwise organize by behavior id.
- `scripts/` — *planned* — repo automation (verify, checkpoint). Not application code.
- `docs/` — specs (`docs/specs/`), tests (`docs/TESTS.md`, `docs/TEST_MATRIX.md`), this file, `ARCHITECTURE.md`, `DECISIONS.md`, `mistakes/`, `roles/`, per-slice design notes (`docs/design/`).
- `.agent/` — Planner/Generator/Estimator working state (`PLAN.md`, `PROJECT_STATE.md`, etc.). Not application code.

## Naming

The codebase has a *de facto* split — the convention is to **document and follow it**, not to unify. Existing files do both styles for a reason (header-only utilities are lowercase; primary class headers are PascalCase).

- **Headers (`.hpp` / `.h`):**
  - **PascalCase** when the header defines a primary class or system: `YGLWindow.hpp`, `MeshInspectorWindow.hpp`, `FrameProfiler.hpp`, `ProfilerWindow.hpp`, `MemoryPool.hpp`.
  - **lowercase** for free-function / utility headers: `camera.hpp`, `error.hpp`, `framebuffer.hpp`, `objreader.hpp`, `program.hpp`, `tinym.hpp`.
  - Single-header third-party libraries keep their upstream name: `stb_image.h`, `metal-cpp/`.
- **Source files (`.cpp`):** **snake_case** — `main.cpp`, `mesh_inspector_gui.cpp`, `metal_impl.cpp`, `profiler_gui.cpp`.
- **Metal kernels (`.metal`) and shaders (`.vert`/`.frag`/`.geom`):** **lowercase**, one file per stage.
- **Types (classes, structs, enums):** **PascalCase** — `BehaviorType`, `MeshState`, `Scene`, `GeneralMesh`, `FrameProfiler`.
- **Functions and variables:** **camelCase** — `addGeneralMesh`, `behaviorTypeName`, `loadScene`. No Hungarian prefix.
- **Enum values:** **PascalCase** — `BehaviorType::TriangularCloth`. Do not renumber existing values (architectural invariant — see `ARCHITECTURE.md §4.4`).
- **Template parameters:** the project uses `BE` (backend tag) and `PR` (precision) consistently. Match this when extending.

## Tests

- Tests are authored from `docs/TESTS.md`. Every test that maps to a behavior id should reference the id in its name or a comment so `TEST_MATRIX.md` can be reconciled by grep — e.g. `TEST_CASE("BDD-014: save populated scene to disk")`.
- Prefer integration over heavily-mocked unit tests at module boundaries. Mocks are reserved for external network/IO; ysim has none in v1, so this rarely applies.
- The CPU backend (`Scene<CPU, PR>`) exists in the type system but is unused. When tests require backend-agnostic behavior (`BDD-102`, `BDD-103`), the test should run against whichever backend is available without `#ifdef`-style branching.
- A failing test is never deleted to make CI green. It is fixed, marked `WARNING` in the matrix with a reason, or reverted with the code that broke it.
- `test/` does not exist yet — the persistence slice creates it (`PLAN.md` step 7).

## Comments

- Default to no comments. Add one only when *why* is non-obvious — a hidden constraint, a workaround for a specific bug, surprising behavior. (This matches `CLAUDE.md`.)
- Do not write comments that restate the code, reference the current task, or name callers.
- One short line max. Multi-paragraph docstrings belong nowhere in `src/`.

## Decisions and mistakes

- A non-obvious tradeoff goes in `docs/DECISIONS.md` (numbered, with file/function and rationale). The first entry is reserved for the JSON-as-scene-format choice once the persistence slice lands.
- A recurring failure mode goes in `docs/mistakes/COMMON_MISTAKES.md` per the criteria in `docs/roles/GENERATOR.md`.

## Commits

- One slice per commit. The slice corresponds to one or more todo items in `.agent/PLAN.md`.
- Commit message body cites behavior ids touched (e.g. `Implements BDD-014, BDD-015`) so the matrix is greppable from git history.
- The recent commit-message style in this repo is short prefix-tagged one-liners: `add: harness`, `wip: save`, `fix: ...`. New commits should match — keep the prefix and append the behavior id citation in the body when there is one.
