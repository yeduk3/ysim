# Current Work — Headless Self-Test Harness Slice (feat/sim-self-test, BLOCK-fix turn)

- File in flight: none — slice complete; ready for Estimator re-review.
- How far: all 5 fix-plan todos done. The three blocks the Estimator BLOCKed (BDD-009/011/012) are rewritten against the `docs/TESTS.md` "Then" clauses verbatim:
  - **BDD-009** — full bitwise compare of `state.x` and `state.v` arrays for the `Float`-tagged ground mesh under non-zero gravity `(1.5, -9.81, -2.0)` AND non-zero wind `(0.5, 0.25, -0.75)` after 6 frames. Element-by-element `==`, no tolerance.
  - **BDD-011** — gravity `(0, -9.81, 0)` for 4 frames, then **without** `simulator.initialize()` flip to `(9.81, 0, 0)` for 4 more; cloth mean vx must grow above tolerance. The "no restart" clause is enforced by absence of any `initialize()` call between pumps.
  - **BDD-012** — fresh init with gravity `(0,0,0)`, wind `(0,0,0)`; capture vx; flip wind to `(5, 0, 0)` for 4 frames; cloth mean vx must be positive and grew above tolerance.
- Plus the WARNING fix: `MetalGlobalContext::getDevice()` / `getLibrary()` returning null now prints `[self-test SKIP] …` and returns 0, not 1. The Estimator's Linux container (no Metal) gets a green build + doctest verdict; the user's macOS host still runs the full 8 assertions.
- What's tested: `./scripts/verify.sh` clean. Doctests unchanged (11 + 9). Self-test prints **8 PASS lines** and exits 0 on macOS Apple Silicon. `docs/TEST_MATRIX.md` test-address strings rewritten to match the new PASS strings so a future Estimator can grep both directions.
- What's next: Estimator re-review. The BLOCK from turn-1 should resolve cleanly; the slice can ship after that.
