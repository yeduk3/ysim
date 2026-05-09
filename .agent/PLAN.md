# Plan — BDD-102 Fix-Turn: Deterministic Jiggle + Strict Block 11 (`feat/bdd-102-determinism`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict (BLOCK)

Estimator turn 11 returned **BLOCK** on the same branch. Two
findings:

- **BLOCK item.** Block 11 turns the discovered divergence into a
  `SKIP` rather than a failing assertion. The slice's goal was
  `BDD-102 pending → pass`; SKIP suppresses the signal and leaves
  the matrix row unclosed. The slice **cannot ship as-is**.
- **WARNING (acceptable).** Block 11 compares simulator state
  buffers instead of Alembic bytes. Documented substitution while
  FR-013 is blocked; not a blocker for this fix-turn.

Per PLANNER.md procedure step 2 ("BLOCK rewrites the plan") and
GENERATOR.md procedure step 1 ("Bug-fix turns in response to an
Estimator BLOCK stay on the same slice branch"), this is a fix-turn
on `feat/bdd-102-determinism`. Commit prefix: `fix:`.

The Generator's earlier code-review feedback (4 items, surfaced by
the user-issued `/codex:rescue --wait estimator 리뷰`) are folded
into this plan as concrete edits — see Scope items 2.a/b/c/d. They
were never written to ESTIMATION.md but are valid observations.

## Goal

Close the BLOCK by fixing the underlying nondeterminism (CM-007) so
Block 11 actually PASSes — not via SKIP-suppression, not via
test-side workaround. After this slice:

- `MeshGridInitializer` uses a **per-mesh seeded `std::mt19937`**
  for jiggle instead of global `rand()`. Two consecutive
  `buildSyntheticScene` calls produce bit-identical `state.x` at
  init.
- Block 11's mismatch path is `fail()`, not `skip()` — SKIP is
  reserved for unsupported environments (Metal-less host) per the
  Estimator's earlier review point #1.
- Block 11 compares **per-frame** position snapshots, not just the
  terminal state — divergence-then-reconvergence can't mask drift
  (Estimator point #2).
- Block 11 fails loudly on null `state.x.ptr` rather than silently
  skipping the mesh (Estimator point #3).
- Block 11 compares **positions only** (drop `state.v` from the
  byte buffer) per BDD-102's wording "vertex positions agree"
  (Estimator point #4). Strict bit-equality on positions stays —
  same-binary-same-machine deterministic kernels should produce
  bit-identical positions; loosening to epsilon would mask future
  ordering bugs that the test exists to catch.
- `docs/TEST_MATRIX.md` row `BDD-102` flips `pending → pass`.
- `CM-007` graduates to `OLD_MISTAKES.md` — the structural cause
  (`rand()`-based jiggle) is replaced.
- `verify.sh` exits 0 with **24/24 PASS** on macOS Apple Silicon.
  Linux container takes the Metal SKIP path unchanged.

## Scope

### 1. CM-007 fix — per-mesh seeded RNG in `MeshGridInitializer`

- **`src/main.cpp::MeshGridInitializerParams`** (~line 1021): add
  `uint32_t seed = 0;` field. Extend the constructor to accept the
  seed (defaulted in callers; keep existing call sites
  source-compatible).
- **`src/main.cpp::MeshGridInitializer::initialize`** (~line 1063):
  replace `rand()/PR(RAND_MAX)/10000.f` with a `std::mt19937`
  seeded from `params.seed`. Use
  `std::uniform_real_distribution<PR>(0.0, 1.0/10000.0)` to keep
  the jiggle scale identical. The RNG is local to the
  `initialize()` call so two calls with the same seed produce
  identical sequences.
- **`src/main.cpp::Simulator::addCloth`** (~line 4381): pass a
  deterministic seed. Two reasonable choices:
  - **(i)** Hardcoded constant (e.g., `0xC0FFEE`). All cloths in a
    scene share the same jiggle; trivially deterministic.
  - **(ii)** Derive from the request id via
    `Scene<BE,PR>::numMeshes` (the value `addGeneralMesh` will
    increment). Different cloths in one scene get different
    jiggles; still deterministic across runs of the same scene.
  - **Recommendation:** option (ii) — read `Scene<BE,PR>::numMeshes`
    before the `addGeneralMesh` call and use it as the seed. One
    line. Each cloth in the scene gets its own deterministic
    jiggle; reloading the same scene reproduces the exact same
    positions.
- **`Simulator::addGround`** uses the grid initializer with
  `jiggle = false` so no seed change is needed (the jiggle branch
  short-circuits when `params.jiggle == false`). But still pass the
  seed parameter for type-completeness; default to 0.
- **`loadScene`** (~line 5011): when reconstructing
  `MeshGridInitializerParams` from a saved scene, derive the seed
  from `o.id` (the saved mesh id) so a saved + reloaded scene
  produces the same jiggle as the original.
  - **Open question** — does the saved scene format include the
    seed? Today `o.source.primitive.jiggle` is a bool. The
    seed isn't serialized. For BDD-102's single-process scope this
    is fine because the seed is derived from `mesh.id` deterministically.
    If `mesh.id` is stable across save/load (it is — D-007/D-014
    treat ids as stable), the seed is stable too. **No scene
    format change needed.** Document this in D-018 below.

### 2. Block 11 cleanup per Estimator's review feedback

- **(a) SKIP → FAIL on mismatch.** Remove the `skip(...)` call;
  use `fail(...)` for any size or byte mismatch. After CM-007 is
  fixed, the assertion PASSes; if a future regression breaks
  determinism, the test FAILs and verify.sh exits non-zero — that
  is the correct gate behavior. Reserve `skip` for the
  Metal-device check at the top of `runSelfTest`.

- **(b) Per-frame snapshot comparison.** Pump 1 frame at a time;
  snapshot positions after each frame; compare run-A's per-frame
  vector against run-B's element-wise. First differing frame
  becomes the diagnostic. Implementation: `std::vector<std::vector
  <unsigned char>> framesA, framesB;` — push the snapshot at the
  end of each frame's pump. ~10 lines of additional code.

- **(c) Fail loudly on null buffers.** `snapshotState` currently
  `continue`s past a mesh with null `state.x.ptr`. Replace with an
  early-fail: if any mesh has null buffers, that's a real
  initialization failure and Block 11 should report it (not pretend
  the mesh doesn't exist). Diagnostic includes mesh id and missing
  field.

- **(d) Positions-only.** Drop `state.v` from the byte buffer. BDD-
  102's "Then" clause is explicitly about vertex positions
  ("per-frame vertex positions agree"). Velocity drift could be a
  symptom of nondeterminism but isn't part of the BDD's contract.
  Strict bit-equality on positions stays — within one process on
  the same binary, positions should be bit-identical, and loosening
  to epsilon would mask future ordering bugs.

### 3. Bookkeeping

- **`docs/TEST_MATRIX.md` row `BDD-102`** — promote `pending → pass`.
  Test address: `src/main.cpp::runSelfTest::BDD-102 (Block 11)`.
- **`docs/mistakes/COMMON_MISTAKES.md::CM-007`** — replace the
  active entry with a graduation breadcrumb (mirror the CM-005 /
  CM-006 patterns). Add a section to `docs/mistakes/OLD_MISTAKES.md`
  under a new high-level cause: "Global RNG state leaks across
  scene reconstructions" (or similar).
- **`docs/DECISIONS.md`** — new D-018 entry for the per-mesh
  seeded RNG decision (file/function, decision,
  alternatives-considered, rationale). Note that `mesh.id`-derived
  seeds give save/load reproducibility for free (no scene-format
  change needed).
- **`.agent/CURRENT_WORK.md` / `RESUME.md`** — update for the
  fix-turn.

## Non-goals (this slice)

- **Persist post-jiggle `state.x` in saved scenes** (CM-007 option
  b). Heavier; touches scene format. Not needed for BDD-102 single-
  process determinism — `mesh.id`-derived seed gives load-time
  reproducibility for free. Defer to a future BDD-101 spine slice
  if seeding-from-id ever proves insufficient.
- **Harness `srand(0)` pre-seed** (CM-007 option c). The harness
  doesn't need a workaround once production is fixed.
- **Scene-format version bump** for the seed field. Not needed —
  mesh.id is already serialized via `o.id`.
- **Alembic-byte compare** instead of state-buffer compare. The
  Estimator's WARNING called this out as an acceptable substitution
  while FR-013 is blocked. Not a blocker for this fix-turn.
- **Epsilon-tolerant comparison.** Strict bit-equality on positions
  is the most informative tolerance for same-process two-runs;
  epsilon would mask real ordering bugs. The Estimator's review
  point #4 was specifically about velocity inclusion, not strict-
  vs-epsilon for positions.
- **Removing or refactoring the `state.v` buffer.** Only the
  comparison drops it; `state.v` itself stays in `MeshState` and
  in the production path.
- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Stay on `feat/bdd-102-determinism`. Commit
   prefix: `fix:` (per GENERATOR.md "Bug-fix turns in response to
   an Estimator BLOCK stay on the same slice branch"). The
   Generator's prior uncommitted changes (Block 11 + CM-007 +
   bookkeeping) carry forward; this fix-turn modifies them in
   place.

2. **Add `seed` field to `MeshGridInitializerParams`.** Add
   `uint32_t seed = 0;` member; extend the constructor to accept it
   as a defaulted parameter so existing call sites stay
   source-compatible without edits. (Verify: there are 3 call
   sites — `addCloth`, `addGround`, `loadScene`. The defaulted
   parameter keeps `addGround`'s call source-compatible since it
   uses `jiggle = false` and doesn't care about the seed.)

3. **Replace `rand()` in `MeshGridInitializer::initialize`.**
   - Add `#include <random>` at the top of `src/main.cpp` if not
     already present.
   - Construct `std::mt19937 rng(params.seed);` once outside the
     particle loop.
   - Construct `std::uniform_real_distribution<PR> jiggleDist(PR(0),
     PR(1.0/10000.0));` once.
   - Replace `pz = params.jiggle ? rand()/PR(RAND_MAX)/10000.f :
     0.f;` with `pz = params.jiggle ? jiggleDist(rng) : PR(0);`.
   - The math is equivalent (uniform in `[0, 1/10000)` instead of
     `[0, 1/10000]`); the visual difference is at the ULP level.

4. **Wire seed in `addCloth`.** Read `Scene<BE,PR>::numMeshes`
   *before* the `addGeneralMesh` call; pass that value as the
   seed:
   ```cpp
   uint32_t seed = static_cast<uint32_t>(Scene<BE, PR>::numMeshes);
   scene.addGeneralMesh(
       new MeshGridInitializer<BE, PR>({
           PlaneDirection::XZPlane,
           center,
           particleNum1D,
           size1D,
           mass,
           true, // jiggle
           seed
       }), ...);
   ```
   `numMeshes` increments inside `addGeneralMesh`, so the value
   read pre-call is the id this mesh will receive.

5. **Wire seed in `loadScene`.** When reconstructing
   `MeshGridInitializerParams` from a saved scene, pass `static_cast
   <uint32_t>(o.id)` as the seed. This makes save/load
   reproduce the original jiggle since `o.id` is preserved by the
   scene format.

6. **Block 11 cleanup (Estimator points 1–4):**

   - **Drop `state.v`** from the byte buffer. `snapshotState` only
     copies `state.x`. The buffer name can stay
     `unsigned char` byte-vector.
   - **Fail-loud on null buffers.** Replace
     `if (!m.state.x.ptr || !m.state.v.ptr) continue;` with
     `if (!m.state.x.ptr) { fail(...); return; }` (or equivalent —
     break out of Block 11 with a fail). Drop the `state.v` check
     since we're not snapshotting v anymore.
   - **Per-frame compare.** Replace the single-snapshot pattern
     with a per-frame loop: `for (int f = 0; f < detFrames; ++f) {
     sim.update(); snapshotState(perFrameA[f]); }` — same for
     run B. After both runs, compare `perFrameA[i]` vs
     `perFrameB[i]` for each `i`; the first differing frame is
     reported with the byte offset.
   - **SKIP → FAIL.** Replace the `skip(...)` call on the mismatch
     path with `fail(...)`. Keep the existing `pass(...)` on the
     match path. After CM-007 is fixed (todos 2-5), Block 11 will
     PASS; this `fail()` is the diagnostic for future regressions.
   - **Pass-label wording stays unchanged**: `BDD-102 / two runs
     produce bit-identical state.x and state.v` becomes
     `BDD-102 / two runs produce bit-identical per-frame state.x`.
     The matrix-row test address must be updated to match (single
     line in `docs/TEST_MATRIX.md`). Confirm the new wording is
     a valid spec-substitution against `docs/TESTS.md#BDD-102`'s
     "Then" clause: still satisfies "per-frame vertex positions
     agree".

7. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

8. **Run `--self-test` 5+ times.** Expect **24/24 PASS**
   consistently. If Block 11 still FAILs, the cause is something
   beyond `rand()` — possibly atomic ordering in `narrow_pt_tri`,
   BVH instability, or a different RNG source elsewhere. **Stop
   and hand back to the Planner.**

9. **Promote `BDD-102` matrix row.** `docs/TEST_MATRIX.md:35`:
   - Status: `pending → pass`.
   - Test address: `src/main.cpp::runSelfTest::BDD-102 (Block 11)
     — per-frame bit-identical positions across two runs of
     buildSyntheticScene; substitution noted (state.x stands in for
     Alembic outputs while FR-013 blocked).`

10. **Graduate CM-007 to `docs/mistakes/OLD_MISTAKES.md`.** Add a
    new high-level-cause section ("Global RNG state leaks across
    scene reconstructions" or similar). Replace the active entry
    in `COMMON_MISTAKES.md` with a one-line graduation breadcrumb
    mirroring the CM-005 / CM-006 patterns.

11. **Add D-018 to `docs/DECISIONS.md`.** Records the per-mesh
    seeded RNG decision: file/function, decision (`mesh.id` is the
    seed), alternatives considered (hardcoded constant, persisting
    post-jiggle state.x, harness pre-seed), rationale (smallest
    behavior change, save/load reproducibility for free since
    `o.id` is serialized).

12. **Update CURRENT_WORK / RESUME.** Four-line max as work
    proceeds; write RESUME near end of turn.

13. **Stop and hand off to the Estimator.** Don't touch other
    matrix rows, don't refactor the BVH or narrow phase, don't
    bump scene format version.

## Course corrections

- **`mesh.id` is stable across save/load.** D-007 (`rotationQuat`
  side-table keyed by mesh id) and D-014 (`transformPosition`
  field on `GeneralMesh`) both rely on this. The persistence layer
  serializes `o.id` in the scene format. So deriving the jiggle
  seed from `mesh.id` gives save/load reproducibility for free —
  no scene-format change. Document this explicitly in D-018 so
  future maintainers don't add a redundant `seed` field to the
  scene format.

- **The `numMeshes` read in `addCloth`** is the
  about-to-be-assigned id (because `addGeneralMesh` does
  `numMeshes++`). This is the same id that Scene::pack later
  writes into `meshes[i].id`. Confirm by reading `addGeneralMesh`'s
  body once before writing the addCloth change; if it differs, use
  the actual mechanism Scene uses for id assignment.

- **`std::uniform_real_distribution` is implementation-defined** —
  different libstdc++ versions may produce different sequences.
  For BDD-102's single-process scope (same binary, same run), this
  is fine because both runs use the same library version. For
  cross-build determinism (explicitly out of scope per PRD §6),
  this is not a guarantee.

- **The Estimator's WARNING about state-buffer vs Alembic-byte
  compare** is acknowledged in the spec-substitution comment in
  Block 11 (was already there pre-block) and stays. Not a blocker.
  When FR-013 ships, BDD-102 mechanization can extend to compare
  Alembic bytes too.

- **If a fifth initializer subtype ships** (e.g., FR-008 Rigid),
  D-015's three-site cascade invariant applies (translateObject,
  Scene::pack, toSnapshot) AND that subtype needs to consider its
  own randomness sources. D-018 should mention this so the next
  initializer subtype starts deterministic.

## What to read before writing code

- `docs/TESTS.md#BDD-102` (lines 195–201) — binding "Then" clause.
- `docs/mistakes/COMMON_MISTAKES.md::CM-007` (current active entry)
  — exact diagnosis of the bug being fixed; the Generator should
  read this to confirm the fix matches the cause.
- `src/main.cpp::MeshGridInitializerParams` (~line 1021) — params
  struct to extend with `seed`.
- `src/main.cpp::MeshGridInitializer::initialize` (~line 1063) —
  replace `rand()` here.
- `src/main.cpp::Simulator::addCloth` (~line 4381) — wire
  `Scene::numMeshes`-derived seed.
- `src/main.cpp::loadScene` (~line 5011) — wire `o.id`-derived
  seed.
- `src/main.cpp::runSelfTest` Block 11 (~line 6107) — Block 11 in
  its current SKIP state; fix-turn rewrites it per Estimator
  points 1–4.
- `src/main.cpp::Scene::addGeneralMesh` — to confirm `numMeshes`
  increments inside this call (so the pre-call read is the
  about-to-be-assigned id).
- `docs/mistakes/OLD_MISTAKES.md` — graduation format.
- `.agent/ESTIMATION.md` — current BLOCK verdict + WARNING.
