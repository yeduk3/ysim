# Current Work — Refit-After-Edit Fix-turn (`fix/refit-after-edit`)

- File in flight: none — slice complete; ready for Estimator. **33/33 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all 11 PLAN todos done.
  - **D-023 production fix** — one-line `collisionPipeline.broadPhase.refit();` call added at the end of both `Simulator::translateObject` and `Simulator::rotateObject` after the state.x mutation. `BroadPhase::refit()` covers per-mesh tree refit AND SCENE-level rebuild in a single call.
  - **Block 16** mechanizes the invariant for both functions with stricter-than-spec assertions:
    - Clause (a) translate-refit: cube at origin, sim.update populates BVH, translate to (5,0,0), assert ray at NEW position hits AND ray at OLD position misses.
    - Clause (b) rotate-refit: cube at origin, witness ray at x=0.30 misses pre-rotate (outside ±0.25 AABB); apply 45°-Z (chosen specifically because 90° leaves the cube AABB symmetric); same witness ray hits post-rotate (inside ±0.354 rotated AABB).
  - **Bug-probe verified** — commented out the `refit()` call in `translateObject`; clause (a) FAILed with `ray at NEW position (5, 0, 0) returned 0 hits — BVH still on old pose`. Clause (b) (rotate) still PASSed because rotate's refit was untouched. Restored.
- What's tested:
  - **33/33 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - **No matrix-row promotion** — Block 16 covers FR-003 + FR-004 cross-cutting refit invariant, not a specific BDD row.
- Non-goals respected: no rotate pack-roundtrip closure, no CM-008 production-side fix, no refit on other state.x mutations beyond translateObject/rotateObject, no performance optimization.
- What's next: Estimator review. Expect verdict at NOTE level — Block 16 is verbatim from D-023's invariant, both clauses bug-probe-verified, the production fix is one line per function and the comment cites D-023.
