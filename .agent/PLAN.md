# Plan — Cloth Drape (BDD-007) Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Goal

Close `BDD-007` — cloth drapes onto a static rigid surface — by mechanizing the spec's "Then" clauses inside the now-existing `runSelfTest` harness, and confirming visually in the GUI that the cloth actually rests on the ground (or Human.obj torso) instead of falling forever or tunneling through.

When this slice ships:
- A Block 6 inside `runSelfTest` asserts: (a) cloth mean-Y decreases over time, (b) no cloth vertex tunnels through the static surface beyond cloth thickness, (c) total cloth kinetic energy stays bounded under a coarse "≤ 10× initial potential energy" check (per `TESTS.md#BDD-007` notes).
- `./scripts/verify.sh` runs the assertion automatically on macOS Apple Silicon and SKIPs on Metal-less hosts.
- The pre-loaded GUI scene (cloth + Human.obj + ground) actually drapes — cloth lands on Human and ground geometry, no obvious tunneling, no exploding particles.

If the existing collision pipeline already satisfies these, the slice is purely additive (a new assertion block + matrix row promotion). If it doesn't, the harness localizes the failure to one of: broad phase / narrow phase / cloth-integrator collision response. Bug fixes land in the same slice.

## Scope

- `BDD-007` — cloth drapes onto a rigid surface (FR-007 / FR-010 / FR-011 all already wired by prior slices; this slice is the end-to-end acceptance).
- New Block 6 inside `src/main.cpp::runSelfTest` named `BDD-007 / cloth drapes onto static surface` (or split into multiple PASS lines if helpful).
- Optionally fix any collision-response bug surfaced by the assertion. Likely candidate areas (do not pre-emptively edit any of these — change only what the failing assertion forces):
  - The `integrate_cloth_grid` / `integrate_cloth` kernels' collision-response loop (`vertColFacets` consumption, normal-projection, `pos += (thickness - distance) * n`).
  - `Simulator::update`'s broad-phase rebuild cadence (`if (frame % 10 == 0)` — could miss contact frames).
  - Stale BVH after re-pack (D-007 / CM-003 territory; the harness already exercises this).
- `docs/TEST_MATRIX.md` row `BDD-007` promoted from `pending` to `pass`.
- Manual visual confirmation in `./build/src/ysim` is the user's gate (not a Generator hand-off requirement, but called out in the slice's "ready" state).

## Non-goals (this slice)

- **Rigid sphere literally as in the spec.** `TESTS.md#BDD-007` says "above a static rigid sphere"; the v1 simulator does not yet have a Rigid backend (Q4 blocked). The harness substitutes the existing Float-tagged ground plane (and visually, the existing Human.obj). The spec's *intent* — "cloth drapes onto a static surface" — is satisfied by either; the rigid-sphere variant returns when the rigid slice (Q4 resolved) ships.
- **Self-collision.** Parked at `PRD §4`. Cloth-on-cloth contact stays out of scope.
- **Multi-cloth scenes.** One cloth is enough for the BDD.
- **Tuning spring constants / thickness for visual prettiness.** Use the existing GUI scene's defaults; only adjust kernel parameters if the harness assertion forces it.
- **Alembic export.** BDD-013 stays blocked on Q5/Q6.
- **Material UI / Behavior UI / Rigid / Determinism mechanization.** Each is its own slice.
- **Touching `MeshRenderState`, persistence, or scene_format.** Pure simulation-side work.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6, Q7.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Read the spec wording verbatim.** `docs/TESTS.md#BDD-007` is the authoritative source. The "Then" clauses are: mean-Y decreases over time, contact constraints fire, no vertex tunneling beyond cloth thickness, total energy stays bounded (≤ 10× initial PE — see the Notes line). Author the assertion code from these strings.
2. **Run `./build/src/ysim` and observe current behavior.** Press Space to unpause; watch the cloth fall. Note which of: (a) cloth lands on the ground (the current `addGround(...)` plane at y=-1) and rests, (b) cloth tunnels straight through, (c) cloth explodes / NaNs. The observation drives whether step 4 is "add assertion only" or "also fix a kernel bug".
3. **Add Block 6 in `runSelfTest`** (after Block 5). Use the existing synthetic scene (cloth at `(0, 0.25, 0)` above ground at `(0, -1, 0)`):
   - Reset (`simulator.initialize()`), set `gravity = (0, -9.81, 0)`, `wind = (0, 0, 0)`.
   - Capture cloth particles' initial Y (max, mean, min); compute initial potential energy `PE_0 = sum(m_i * |g| * (y_i - y_min))` so the energy bound is well-defined.
   - Run enough frames for the cloth to reach the ground — at `h = 1/60` with `subSteps = 4` and a 0.25 → -1.0 fall distance under 9.81, ~30 frames is comfortable; 60 is generous.
   - Assert (a) `meanY_after < meanY_before - tol`, (b) `min(state.x[i].y) >= groundY - thickness` for every cloth vertex, (c) `KE_after = sum(0.5 * m_i * |v_i|^2) <= 10 * PE_0`.
   - On any failure, the diagnostic must name which clause broke and the offending value.
4. **If step 3's assertion fails on a real cloth-on-ground tunnel/explode**, fix in the smallest possible scope. The most likely fix locations are listed in Scope above. Do not refactor surrounding code; the slice is BDD-007, not "cleanup the cloth pipeline".
5. **Update `docs/TEST_MATRIX.md`.** Promote `BDD-007` row from `pending` to `pass`. The `Test address` cell must point at `src/main.cpp::runSelfTest` (Block 6 name) so a future Estimator can grep both directions like the BDD-009/011/012 rows do.
6. **Run `./scripts/verify.sh` locally.** Build + doctest binaries clean + 9 self-test PASS lines (the new Block 6 added on top of the existing 8).
7. **Refresh `CURRENT_WORK.md` and `RESUME.md`.** Note the BDD-007 mechanization; flag any kernel-level fix in `RESUME.md`'s "Must remember" if step 4 fired.
8. **Stop and hand off to the Estimator.** Do not pile on BDD-002 import UI or BDD-102 determinism even if the cloth-drape passes cleanly and the slice feels small. Each is its own future slice.

## Course corrections

- **The harness's value is now testable in production.** This is the first slice that *uses* the harness for real new coverage rather than just regression-protecting prior slices. If Block 6 passes immediately, the harness validates the past three slices' cumulative work. If it fails, the harness localizes the bug — exactly what was missing during CM-002/003/004.
- **Spec-vs-label discipline carries forward.** The prior slice's BLOCK was caused by writing assertions from compressed matrix-row labels. Block 6 must be authored from `docs/TESTS.md#BDD-007`'s "Then" clauses character-for-character. The Estimator will check.
- **No PLAN-time "no Metal kernel changes" non-goal.** Past two slices burned on this assumption (CM-004, then no-op). If step 4 needs a kernel patch, it gets one — accompanied by a `CM-NNN` entry naming the trap so a future Generator does not re-introduce it. The non-goal stays "do not refactor surrounding code", not "do not touch kernels at all".

## What to read before writing code

- `docs/TESTS.md#BDD-007` — the binding "Then" clauses and the Notes line on energy-bound tolerance ("energy ≤ 10× initial potential energy as a coarse stability check").
- `docs/specs/BDD.md#BDD-007` and `#BDD-103` — user intent + the boundary invariant. Kernel changes inside `src/metal/physics.metal` for collision-response are within the simulation pipeline; not BDD-103 violations.
- `src/metal/physics.metal::integrate_cloth_grid` and `integrate_cloth` — the existing collision-response loop. Already reads `vertColFacets` + applies normal-projection + position correction. Inspect it to predict whether step 2 will pass clean or fail.
- `src/main.cpp::Simulator::update` — broad/narrow phase invocation cadence. The `if (frame % 10 == 0)` BVH rebuild is suspect for cloth-drape: a cloth that lands between rebuilds could tunnel.
- `src/main.cpp::runSelfTest` — the existing Block 5 (BDD-012) is the structural template for Block 6: reset → set env → pump frames → read state.x / state.v → assert.
- `docs/CONVENTIONS.md` "Tests" section — every test ID is referenced in the test name or comment; the new Block 6's PASS string follows the established `BDD-007 / <one-line summary>` pattern.
- `.agent/RESUME.md` — the spec-vs-label trap and the SKIP-not-FAIL convention; both still load-bearing this slice.
