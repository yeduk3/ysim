# Current Work — BDD-017 Coverage Fix-turn (`fix/bdd-017-coverage`)

- File in flight: none — slice complete; ready for Estimator. **29/29 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all 10 PLAN todos done, **plus a real production bug discovered and fixed mid-build (D-020 / CM-009)**.
  - **Block 14 clause (b) gain — both-cubes-present check.** Walks `clickRayCollisions[0..numHits)` setting `sawFront`/`sawBack` flags; assert both true. Caught the production bug below directly.
  - **Block 14 clause (a) and (b) — `sim.selectedObj` mutation + assertion.** Both clauses now write `pickClosest()` into `sim.selectedObj` (mirroring production's mouse-callback at `src/main.cpp:6718`), and assertions read from `sim.selectedObj`. Closes Estimator turn-15 WARNING #2.
  - **D-020 / CM-009 — BVH leaf-recursion bug.** While running the new both-cubes assertion, `numHits=4096` (buffer cap) appeared with `sawFront=1, sawBack=0`. Diagnostic prints showed `BVH::queryClickRay`'s leaf branch wrote the hit but then fell through to `if(node.childB > 0) queryClickRay(ray, tree[node.childB])`, treating the leaf's `childB` (a primitive id) as a node index. Recursion exploded within cubeFront's tree, filled the 4096-entry buffer with spurious cubeFront-objid hits, and cubeBack's outer-loop entry found the buffer already full → 0 cubeBack hits. **Fix:** add `return;` after the leaf-write at `src/main.cpp:3826`. One-line production fix.
  - **Production was masking the bug** because (a) smallest-tmin walk found the right object id even among spurious cubeFront hits, and (b) most clicks have ≤1 object on screen so the buffer-overflow performance cost was invisible. The harness's both-cubes assertion was the right kind of stricter test to catch it.
  - **Bug-probe verified** during build: without the production fix, the both-cubes assertion FAILed at `sawFront=1, sawBack=0`; with the fix it PASSes.
- What's tested:
  - **29/29 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - `docs/TEST_MATRIX.md` row `BDD-017` test address unchanged (pass labels stayed verbatim).
- Non-goals respected: no other matrix rows, no spec edits, CM-008 production-side fix still deferred.
- What's next: Estimator review. Expect verdict at NOTE level — both turn-15 WARNINGs closed; D-020 is the surprise production fix the harness assertion forced.
