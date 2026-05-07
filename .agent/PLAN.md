# Plan — Headless Self-Test Harness Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Goal

Add a `--self-test` mode to the `ysim` binary that mechanically verifies the simulator paths the unit-test net has been unable to reach. After this slice ships, `./scripts/verify.sh` runs an end-to-end save/load/sim-step check against a real Metal device — closing the parked sim-step clauses on `BDD-009`/`011`/`012`/`015` and turning each of CM-002/003/004 into a regression that fails fast in CI.

**Q-D resolved: headless Metal harness, not CPU backend reference.** The CPU backend stays as type-system reservation only. Reasons in the slice's first DECISIONS entry.

## Scope

- Modify `int main()` → `int main(int argc, char** argv)`. When `--self-test` (or `--self-test=<path>` for a temp save path) is passed, dispatch to a new `runSelfTest()` and return its exit code without entering the GUI loop.
- New helper `runSelfTest()` in `src/main.cpp`. Runs:
  - **CM-002 regression** — call `simulator.initialize()` twice on a synthetic scene with no `addGeneralMesh` between the calls. Must not crash (this is exactly what segfaulted before commit `2000aea`).
  - **CM-003 regression** — call `Simulator::initialize()`, then `simulator.addCube(...)`, then `simulator.initialize()` again. Must not crash. Must not corrupt the scene-level BVH.
  - **CM-004 / BDD-009 / BDD-011 regression** — set `Scene::environment.gravity = (0, -9.81, 0)`, run `simulator.update()` for a small number of frames, assert that **at least one** non-Float mesh particle's Y coordinate decreased. Then set gravity to `(0, +9.81, 0)`, reset the scene, run again, assert Y *increased*. Then set gravity to `(0, 0, 0)`, run, assert a `Float`-tagged ground mesh didn't move (BDD-009 strict equality).
  - **BDD-015 round-trip** — populate a scene, `saveScene` to a temp path, clear the simulator, `loadScene` the same path, assert `numMeshes` and `Scene::environment.gravity`/`wind` round-trip bit-stable. Run one `simulator.update()` and assert it does not crash.
  - **CM-002 + load** — after the load, call `simulator.initialize()` again to confirm the load → init cycle is also stable.
- The helper writes pass/fail diagnostics to stderr; returns 0 on all-pass, 1 on any failure.
- `scripts/verify.sh` runs `cd build && ./src/ysim --self-test` after the doctest binaries (cwd needs to be `build/` so `default.metallib` is found via the existing `NS::String::string("default.metallib", ...)` lookup).
- `scripts/verify-light.sh` does **not** run the self-test (the light gate stays GL-free; the GUI binary is built but not executed there).
- `docs/TEST_MATRIX.md` rows promoted from `warning` to `pass` for `BDD-009`, `BDD-011`, `BDD-012`, `BDD-015` once the relevant clauses are mechanically asserted by the self-test. The `warning` row legend stays — there will be future BDDs whose data-layer half passes ahead of their sim-step half.

Behaviors covered: `BDD-009`, `BDD-011`, `BDD-012`, `BDD-015` sim-step clauses (those parked since the persistence + env-forces slices). Tests already exist in `docs/TESTS.md` for those ids; the self-test mechanizes the previously-manual clauses.

## Non-goals (this slice)

- **Extracting main.cpp's types into a header-set or static library.** The `--self-test` lives inside the same TU as `int main()` precisely so we do not pay that refactor's cost in this slice. Future improvement; not now.
- **A doctest-style fine-grained assertion harness for Metal-backed tests.** Pass/fail via exit code + one diagnostic line per failure is sufficient. doctest expects an executable per test file with one `main`; we already have two such binaries (`ysim_tests`, `ysim_primitive_tests`). The self-test is a third surface that exercises the live shipping binary, intentionally.
- **Headless rendering / pixel checks.** The self-test does not call `Simulator::draw` and does not exercise `MeshRenderState`. Visual regression remains the user's manual gate (it always was).
- **Closing BDD-007 (cloth-drape onto rigid surface).** That requires verifying collision *response* end-to-end, which is its own slice. The self-test only checks force-application correctness, not contact resolution.
- **Determinism (BDD-102) verification.** Two-runs-bit-identical is strictly more than what this slice promises; tempting to pile on but expands scope. Add when there's a concrete need.
- **Touching the Metal kernels, the persistence layer, the GUI menu code, or the renderer.** Self-test reads `Scene::environment` and per-mesh state.x positions directly; no kernel or shader edits.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6, Q7.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Make `main()` accept argv.** Change `int main()` → `int main(int argc, char** argv)`. Parse the first arg only — if it is `--self-test`, call `runSelfTest()` and return its result *before* any `YGLWindow` or `ImGui::CreateContext()` call. Anything else (including no args) keeps the current GUI flow.
2. **Implement `runSelfTest()`** in `src/main.cpp` (above `int main()`, function-static helpers OK):
   - Bring up Metal: `MTL::CreateSystemDefaultDevice()` via the existing `MetalGlobalContext` (no special init needed — it lazy-inits on first `getDevice()`).
   - Construct a `Simulator<METAL, Precision>` against an `ExplicitSystem<METAL, Precision>`.
   - Build a synthetic scene **without imports** (avoid asset-path fragility): one `addCube(center=(0,0,0), tessellation=2, size=1)`, one `addCloth(particleNum1D=4, size1D=0.5, center=(0, 0.25, 0))`, one `addGround(...)` for collision context (still `Float`).
   - Run the four assertion blocks listed in Scope. Each block prints `[self-test PASS] <name>` or `[self-test FAIL] <name>: <reason>` to stderr and bumps a fail counter.
   - Return `failures > 0 ? 1 : 0`.
3. **Update `scripts/verify.sh`.** Append a `(cd build && ./src/ysim --self-test)` line after `./build/test/ysim_primitive_tests`. The `cd` is required so `default.metallib` is found by the existing relative-path lookup; do **not** change the lookup itself in this slice (that's a separate concern).
4. **Update `docs/TEST_MATRIX.md`.** Promote the four parked `warning` rows (BDD-009, BDD-011, BDD-012, BDD-015) to `pass` and update each row's "Test address" to point at `src/main.cpp::runSelfTest` plus the existing JSON-layer addresses. Keep the `warning` legend entry — future BDDs will use it.
5. **Add a DECISIONS entry** documenting the Q-D resolution. Number it sequentially. Cite ARCHITECTURE §5 Q-D explicitly. Record the alternative (CPU backend reference) and why it was rejected for v1.
6. **Stop and hand off to the Estimator.** Do not extend the self-test to BDD-007 (cloth drape) or BDD-102 (determinism) in this slice; both are larger and can land separately.

## Course corrections

- **Pattern for the past four slices** — `Generator declares done → manual user test catches Metal-side bug → patch`. Three of those bugs (CM-002, CM-003, CM-004) and the persistence-slice WARNING all live in the path this slice mechanizes. After this slice ships, the *next* slice's failure mode should change to "Generator declares done → `verify.sh` catches a regression locally → patch before review". If it doesn't, the harness needs more coverage — surface that to the next planning loop.
- **Decoupling enabled the slice** — the immediately-prior slice (`feat/render-state-decoupling`, D-011) removed `mesh.initialize()`'s GL coupling. Without that, `runSelfTest()` would still need a window. This slice is the payoff that justifies the prior slice's "no new BDDs, no visible feature" status.
- **Visual regression remains user-only.** The self-test exercises sim correctness, not rendering. If a future bug shifts to the rendering side, we'll need a separate visual-regression harness — explicitly not this slice's job.

## What to read before writing code

- `docs/specs/TESTS.md#BDD-009`, `#BDD-011`, `#BDD-012`, `#BDD-015` — exact wording of the clauses being mechanized.
- `docs/specs/BDD.md#BDD-103` — the boundary invariant. Self-test is part of the test/render box, not the simulation pipeline; reading `Scene::environment` and per-mesh state is fine.
- `docs/ARCHITECTURE.md §5 Q-D` — the open question this slice resolves.
- `src/main.cpp` `int main()` (~line 4960 onward) — where to insert the argv parse and the early-return self-test branch. Note the `yglwindow = new YGLWindow(...)` line is the first thing today; that must NOT run in self-test mode.
- `src/main.cpp` `Simulator::initialize` (~line 4365), `Simulator::addCube`/`addSphere`/`addCloth`/`addGround` (~line 4290 onward), `Simulator::saveScene/loadScene` (~line 4710 onward), and `Scene::environment` (~line 1612).
- `MetalKernelContext::getLibrary` — reads `default.metallib` from cwd; this is why `verify.sh` needs `cd build` before invoking the self-test. **Do not change** the lookup logic in this slice.
