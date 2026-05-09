# Current Work — BDD-004 Quaternion Slice (`feat/bdd-004-rotation`)

- File in flight: none — slice complete; ready for Estimator. **25/25 self-test PASS** deterministic across 5 consecutive runs. `verify-light.sh` clean.
- How far: all 10 PLAN todos done.
  - **D-019** — three new free functions live next to `struct Quat` (~line 1554): `operator*(Quat, Quat)` (Hamilton product, `a * b = apply b first then a`), `quatNorm`, `quatNormalize` (with 1e-12 degenerate-input guard returning identity). Struct unchanged — aggregate-init shape and on-disk schema stay intact.
  - **Block 12** mechanizes `docs/TESTS.md#BDD-004` verbatim:
    - Compose R₁ = (dq1 * R₀).normalized() in memory.
    - saveScene → loadScene → applyPendingMaterials (writes pendingRotations into rotationQuat).
    - Compose R₂_round_trip = (dq2 * R₁_post_load).normalized() and compare to R₂_in_memory = (dq2 * R₁).normalized().
    - Assert orientation drift < 1e-5 component-wise + unit-norm < 1e-5 at each step.
    - Local helper `quatAxisAngle(axis, angle)` lives inside the block; `kPi = 3.14159265358979323846f` constant inline.
  - **Bug-probe verified** — temporarily added `meshAfterLoad->rotationQuat.w += 0.01f` after the reload; Block 12 FAILed with `orientation drift (round-trip vs in-memory): 0.002179, 0.001776, 0.003470, 0.000245`. Restored; flips back to PASS.
- What's tested:
  - **25/25 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged (159 + 1120 assertions).
  - `docs/TEST_MATRIX.md` row `BDD-004` promoted `pending → pass`.
- Non-goals respected: no `Simulator::rotateObject` API, no inspector wiring, no renderer-side rotation application, no Euler conversion. The slice is purely data-layer math + persistence-round-trip mechanization, matching BDD-004's "Then" wording exactly.
- What's next: Estimator review. Expect verdict at NOTE level — Block 12 is verbatim-from-spec, bug-probe-verified, and the canonical Quat math (D-019) is the load-bearing piece for any future rotation consumer.
