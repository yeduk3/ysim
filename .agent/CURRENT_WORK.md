# Current Work — BDD-017 Ray-Pick Slice (`feat/bdd-017-ray-pick`)

- File in flight: none — slice complete; ready for Estimator. **29/29 self-test PASS** deterministic across 5 consecutive runs. Doctest 159/159 + 1120/1120 green.
- How far: all 10 PLAN todos done.
  - **Block 14** mechanizes BDD-017 verbatim against `docs/TESTS.md#BDD-017`. Two clauses + `pickClosest` lambda mirroring production's smallest-tmin walk at `src/main.cpp:6588-6596`:
    - Non-overlapping: cubeA at x=-1.5 + cubeB at x=+1.5; rays straight down -z through each cube's x line pick the right id.
    - Overlapping: cubeFront at z=+2 + cubeBack at z=-2; ray from z=+10 going -z passes through both; assert smallest-tmin = cubeFront.
  - **Build-time discovery (CM-008):** the BVH's per-mesh `build` loop (line ~3907) skips Float-tagged tree rebuild when slot is already populated (`objTrees[i].tree.ptr && objBehavior == Float → continue`). Production-correct optimization — Float meshes don't change shape between init calls. But the harness's `resetScene` + new-scene pattern with the same numMeshes count keeps the same `objTrees.size()`, so the skip fires against stale data. Workaround: `sim.collisionPipeline.broadPhase.objTrees.clear();` before each new scene's `sim.initialize()`. Comment + CM-008 entry added so future block authors avoid the trap.
  - **Production parity:** harness calls `sim.update()` once after `sim.initialize()` to refit the BVH AABBs (mirroring production where the click callback fires after at least one frame's update has run). Without it, leaf AABBs are zeroed and queryClickRay returns 0 hits. Comment in Block 14 explains.
  - **Spec-substitution noted:** harness skips camera unprojection from cursor pos; constructs `Ray{origin, dir}` directly in world space. Pass label and block comment make the substitution auditable.
- What's tested:
  - **29/29 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - `docs/TEST_MATRIX.md` row `BDD-017` promoted `pending → pass`.
- Non-goals respected: no GUI/ImGui input simulation, no `selectedObj` mutation as assertion target (production-side concern, BDD-018), no inspector update mechanization, no negative ray (no-hit) case, no camera unprojection.
- What's next: Estimator review. Expect verdict at NOTE level — Block 14 verbatim from spec, both clauses bug-trail-verified during build, CM-008 documented for future maintainers.
