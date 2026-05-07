# Plan — Self-Test Harness BLOCK Fix

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07 (fix turn after `BLOCK` verdict)

## Goal

Make `runSelfTest()` actually exercise the BDD wording it claims to verify, and make `./scripts/verify.sh` produce a valid verdict on hosts where Metal is unavailable. Concretely:

- `BDD-009` becomes a strict-equality check on the **full state.x and state.v arrays** of a Float-tagged mesh under **non-zero gravity AND non-zero wind**, not a displacement norm with wind=0.
- `BDD-011` runs gravity = `(0, -9.81, 0)` for a few frames, then changes gravity to `(9.81, 0, 0)` **without re-initialising the simulator**, and asserts cloth mean x-velocity grows from ~zero to clearly positive — matching the spec's "while the simulation is running... no restart is required" clause.
- `BDD-012` sets `wind = (5, 0, 0)` on a cloth at rest with zero gravity, runs a few frames, and asserts cloth mean x-velocity is clearly positive — the wind-drives-cloth clause the matrix currently overstates.
- When `MetalGlobalContext::getDevice()` (or `getLibrary()`) returns null, the harness prints `[self-test SKIP] metal-device: …` and exits **0**, not 1. The gate distinguishes "Metal absent → skipped" from "Metal present → assertion failed".

When this fix ships, `./scripts/verify.sh` is green on macOS Apple Silicon (full self-test runs and asserts) **and** green on a Metal-less Estimator host (build + doctests pass; self-test skips with a clear stderr note).

## Scope

- Rewrite three of the four assertion blocks inside `src/main.cpp::runSelfTest`. The `CM-002` and `CM-003` blocks stay as-is — they are correct.
- Add the Metal-unavailable skip path at the top of `runSelfTest` (already detects null device but currently *fails*; flip to skip-and-return-0).
- Update `docs/TEST_MATRIX.md` test addresses for `BDD-009`, `BDD-011`, `BDD-012` to point at the rewritten assertion blocks. Status stays `pass` only after the rewritten blocks actually pass on the user's macOS host.
- Refresh `.agent/CURRENT_WORK.md` and `.agent/RESUME.md`.

## Non-goals (this fix)

- **Resolving the Estimator's Metal access.** The skip path is the right answer for v1: the Estimator's verdict on the *build + JSON-layer tests* is independent of its ability to spin up a GPU. If a future v1.x slice wants Metal-on-Estimator, that's a separate infra slice.
- **Adding determinism (`BDD-102`) or cloth-drape (`BDD-007`) coverage.** The harness shape supports both later; not now.
- **The optional `--self-test=<path>` form** mentioned in the prior plan. Bare `--self-test` is sufficient; the path-arg form is YAGNI.
- **Re-extracting types out of `main.cpp`.** Still a non-goal; the doctest binary route remains future work.
- **Touching kernels, persistence, GUI menu code, or the renderer.** This is purely a harness rewrite.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6, Q7.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Read the spec wording verbatim.** Open `docs/TESTS.md` blocks `#BDD-009`, `#BDD-011`, `#BDD-012`. Each "Then" clause is the literal acceptance the harness must mechanise. The Estimator pattern-checks against this wording.
2. **Flip the Metal-unavailable path to SKIP.** In `runSelfTest`, when `MetalGlobalContext::getDevice()` returns null OR `getLibrary()` returns null, print `[self-test SKIP] <reason>: …` to stderr and `return 0;`. **Do not** print `[self-test FAIL]` and **do not** increment the failure counter for that case. Add a one-line comment naming the Estimator-host context so a future reader does not "fix" the skip back to a fail.
3. **Rewrite Block 3 (`BDD-009`).**
   - Snapshot the **full** `state.x` and `state.v` arrays of the ground mesh (`Float`-tagged, id 2) into `std::vector<float>` after `simulator.initialize()` — call this `xRest` and `vRest`.
   - Set `Scene::environment.gravity = (0, -9.81, 0)` and `Scene::environment.wind = (0.5, 0.25, -0.75)` — both non-zero, the spec demands both.
   - Run a small number of frames (e.g. 6).
   - Re-read `state.x` and `state.v` for the same ground mesh; assert each element matches `xRest[i]` and `vRest[i]` **bitwise** (`==`, no tolerance — the spec says "strict equality, no tolerance").
   - On any mismatch, fail with the offending index and the two values.
4. **Rewrite Block 4 (`BDD-011`).**
   - Reset the scene (`simulator.initialize()`) so cloth velocity starts at zero.
   - Set `Scene::environment.gravity = (0, -9.81, 0)`. Run 4 frames. Read cloth mean x-velocity → should be ~zero (gravity has no x-component yet). Capture as `vxBefore`.
   - **Without re-initializing**, change `Scene::environment.gravity = (9.81, 0, 0)`. Run 4 more frames. Read cloth mean x-velocity → should be clearly positive (the spec says "begins accumulating in the +x direction within a few steps"). Capture as `vxAfter`.
   - Assert `vxAfter - vxBefore > <tolerance>` — pick a tolerance that's well above floating-point noise but well below what 4 frames of 9.81 acceleration would produce on a 0.1-mass particle.
   - The "no restart" clause is enforced by *not* calling `simulator.initialize()` between the two pumps — make sure the test code reads cleanly on this point so the Estimator can verify by inspection.
5. **Rewrite Block 5 (`BDD-012`).**
   - Reset the scene. Set `gravity = (0, 0, 0)` and `wind = (5, 0, 0)`.
   - Run 4 frames.
   - Read cloth mean x-velocity. Assert it is positive and above the same noise tolerance.
   - Optionally (cheap, gives the Estimator confidence): also capture mean x-velocity at `wind = (0, 0, 0)` first and compare. Skip if it adds bookkeeping noise.
6. **Update `docs/TEST_MATRIX.md`.** Replace each row's `Test address` for `BDD-009`/`011`/`012` with the new block names (e.g. `…BDD-009 / Float exact x and v under non-zero gravity and wind`). Keep status `pass`. The new row text must match the `[self-test PASS]` strings the harness prints, so a future Estimator can grep both directions.
7. **Run `./scripts/verify.sh` locally.** Confirm:
   - Build clean.
   - `ysim_tests` + `ysim_primitive_tests` pass.
   - `./src/ysim --self-test` from cwd=build/ either prints the new PASS lines and exits 0 (Generator's macOS host) or prints SKIP and exits 0 (Metal-less host).
   - Exit code is 0 either way.
8. **Refresh `CURRENT_WORK.md` and `RESUME.md`.** Note explicitly that BDD-009/011/012 acceptance now matches the spec wording, and that Metal-unavailable is a SKIP rather than a FAIL.

## Course corrections

- **The original self-test slice (turn 1) shipped pattern-matched assertions** that the Estimator caught. Cause: `runSelfTest` was written from the matrix-row labels (`gravity down moves cloth -Y`, `Float exempt under gravity`) instead of the BDD's "Then" clauses verbatim. The fix re-grounds in `docs/TESTS.md` wording. Future harness extensions should open `TESTS.md` first; the matrix label is too compressed to drive an assertion from.
- **The Metal-unavailable case is not a Generator bug; it is a planner-level scoping miss** — the prior plan's verify.sh wiring assumed Metal everywhere. We accept now that the Estimator host may not have Metal, and the harness skips rather than fails.
- **No new DECISIONS entry.** D-012 (Q-D resolution: headless Metal harness) is unchanged. The skip path is implementation detail under D-012, not a separate decision.

## What to read before writing code

- `docs/TESTS.md#BDD-009`, `#BDD-011`, `#BDD-012` — the spec's "Then" clauses are what the assertions must check, character-for-character.
- `docs/specs/BDD.md#BDD-009`/`#BDD-011`/`#BDD-012` — confirms the user-facing intent. Note `BDD-009`'s explicit "no tolerance" notes line.
- `src/main.cpp::runSelfTest` (the existing block bodies) — keep CM-002 / CM-003 blocks unchanged; rewrite blocks 3, 4, 5 in place.
- `src/main.cpp::Simulator::applyEnvironmentForces` — confirms gravity is applied as `gravity * mass` per particle (non-Float) and wind as force-per-particle (cloth only). The acceleration the harness expects in BDD-011/012 follows from this — sanity-check tolerances against `subh = h / subSteps` (`h = 1/60`, `subSteps = 4` in the harness).
- `.agent/ESTIMATION.md` (the BLOCK report) — the three citation triples (`src/main.cpp:line`, `docs/TEST_MATRIX.md:line`, `docs/specs/BDD.md:line`, `docs/TESTS.md:line`) tell the Generator exactly which lines the Estimator will re-read on the next pass.
