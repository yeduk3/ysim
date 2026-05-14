# PLAN — D-042 R-6: self-test BC alias demonstration — `feat/d-042-r-6-selftest-migration`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 41 (D-042 R-5 — packed→preview resync at update end) returned **NOTE** with 0 BLOCK + 0 WARNING + 1 NOTE. NOTE: `O(meshes × requests)` nested loop in the resync is fine for v1 scene sizes; could swap for keyed lookup if scenes grow. Acceptable; deferred to a future perf slice.

R-5 merged to `main` via commits `9748620 add:` + `f23b710 chore:`. Self-test 70 → 71 PASS deterministic on macOS.

## Goal

**Add a self-test block (Block 41) that demonstrates the R-3+R-5 round-trip invariant**: after every `Simulator::update`, `state.x` and `preview.x` are byte-equal for every mesh. This codifies the "BC alias" property — future self-test blocks (and downstream consumer code) can equivalently read EITHER `Scene::meshes[i].state.x` OR `Scene::requestsGeneralMeshes[i].preview.x` with identical semantics. The invariant is:
- R-5 memcpys state.x → preview.x at the END of every `Simulator::update`.
- R-3 memcpys preview.x → state.x in `Scene::pack` when a re-init runs.
- R-4 translateObject/rotateObject dual-writes keep them in sync at edit time.
Together these guarantee preview ≡ state.x at every observable boundary.

Block 41 mechanizes this for a cloth scene over 5 frames, asserting `memcmp(state.x.ptr, preview.x.data(), nverts * 3 * sizeof(PR)) == 0` at each iteration. Bug-probe: corrupt the resync's first vertex write to break byte-equality → Block 41 FAILs.

Pure additive slice — no existing production code changes. Self-test count 71 → 72.

## Scope

**Design call (1) — Pure additive, test-only slice.** The R-3+R-5 invariant is already established. Block 41 captures it as a long-lived self-test guard. If a future slice breaks the round-trip (e.g., changes resync placement, drops the memcpy under some condition, introduces a write to preview that doesn't propagate), Block 41 catches it.

**Design call (2) — Cloth at y=0.25 with no ground.** Simplest scenario that exercises the resync each frame. Cloth falls under gravity; state.x changes every step; resync should propagate to preview.x. 4-particle cloth (2×2) keeps memcmp size small + deterministic across runs. No ground means no narrow-phase collision interaction perturbs cloth equally on state.x and preview.x — both should stay byte-equal regardless of dynamics.

**Design call (3) — Block 41 shape**:
```cpp
// ---- Block 41: D-042 R-6 — preview ≡ state.x round-trip invariant. ----
// R-3 memcpys preview→packed at Scene::pack; R-4 keeps preview in sync
// with state.x during edits (dual-write); R-5 memcpys state.x→preview at
// update end. After R-3+R-4+R-5 ship, the per-mesh preview.x and the
// packed state.x are byte-equal at every observable boundary (post-update,
// post-edit, post-init). Block 41 codifies that invariant so any future
// slice that breaks it FAILs noisily.
//
// Bug-probe: corrupting the resync write to write `state.x.ptr[0] + 0.01f`
// into preview.x[0] breaks byte-equality on the first vertex; memcmp fails
// from frame 1 onward.
{
    resetScene();
    sim.addCloth(/*particleNum1D=*/2, /*size1D=*/0.5f,
                 tinym::vec3(0.0f, 0.25f, 0.0f),
                 /*kstretch=*/1e3f, /*kshear=*/1e3f, /*kbend=*/1e3f,
                 /*thickness=*/0.01f, /*mass=*/0.1f);
    sim.initialize();

    auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
    bool oneReq = (reqs.size() == 1);
    bool meshExists = !Scene<Backend, Precision>::meshes.empty();

    sim.pause = false;

    int frameMismatch = -1;
    int eqAllFrames = 0;
    if (oneReq && meshExists) {
        auto& m = Scene<Backend, Precision>::meshes[0];
        for (int f = 0; f < 5; ++f) {
            sim.update();
            const size_t bytes = (size_t)(m.state.x.size) * sizeof(Precision);
            if (reqs[0].preview.x.size() * sizeof(Precision) != bytes) {
                frameMismatch = f;
                break;
            }
            if (std::memcmp(m.state.x.ptr, reqs[0].preview.x.data(), bytes) != 0) {
                frameMismatch = f;
                break;
            }
            eqAllFrames++;
        }
    }

    if (oneReq && meshExists && frameMismatch < 0 && eqAllFrames == 5) {
        pass("D-042 R-6 / state.x ≡ preview.x byte-equal after every Simulator::update (5-frame cloth round-trip)");
    } else {
        fail("D-042 R-6 / state.x ≡ preview.x byte-equal after every Simulator::update (5-frame cloth round-trip)",
             std::string("oneReq=") + std::to_string((int)oneReq)
             + " meshExists=" + std::to_string((int)meshExists)
             + " frameMismatch=" + std::to_string(frameMismatch)
             + " eqAllFrames=" + std::to_string(eqAllFrames));
    }
}
```

Pass label: `D-042 R-6 / state.x ≡ preview.x byte-equal after every Simulator::update (5-frame cloth round-trip)`.

**Design call (4) — Block 41 placement: INSIDE the Metal-gated section, AFTER Block 40.** Uses `sim.initialize` + `sim.update` (Metal). Linux container SKIPs. Same pattern.

**Design call (5) — Why memcmp is the right tool.** The invariant is BYTE-equality (R-5's `std::memcpy` from state.x to preview.x is bit-identical). `memcmp` is the strictest possible check; anything that diverges by even one bit (rounding error, NaN substitution, partial copy) flags. Per ESTIMATOR.md's "stricter-than-spec assertions are valuable signal" — a stricter test catches more regressions.

**NEW symbols this slice adds**:
- Block 41 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-6 entry.

**MODIFIED symbols in place**:
- `.agent/PROJECT_STATE.md` — next-milestone updates.

**PRESERVED symbols** (parallel-symbol invariant):
- All of `src/main.cpp`'s production code paths — UNCHANGED. R-6 is test-only.
- `MeshRenderState` API — UNCHANGED.
- `PreviewState<PR>` — UNCHANGED.
- All other Blocks 1-40 — UNCHANGED.

## Non-goals

- **NO new public accessor on Scene.** A `Scene::previewForMeshId(int)` helper was considered but adds API surface without test value (Block 41 reads `requestsGeneralMeshes[0]` directly).
- **NO migration of existing self-test blocks** to read via preview. Existing blocks that read `state.x` continue to work — the invariant Block 41 guards means either read returns the same value. Adding migration would be churn without observable effect.
- **NO recomputation of preview.n in the resync.** Same as R-5's design decision; R-7 cleanup may revisit.
- **NO production code changes.**
- **NO new BDD/FRD/CM.**

## Spec substitution

None. R-6 is infrastructure / test-coverage work.

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED.
- **BDD-102-vs-ALEMBIC-BYTES** — UNCHANGED.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r6-selftest-migration` on branch `feat/d-042-r-6-selftest-migration` (off main HEAD `f23b710`). Submodules already initialized. Commit prefix `add:`.

2. **New Block 41** in `runSelfTest` — inserted INSIDE the Metal-gated section, AFTER Block 40 (D-042 R-5). Body per Design call (3).

3. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **72/72 PASS** each time.

4. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

5. **Bug-probes** (each FAIL after revert; restore after):
   - **(a) Modify R-5's resync to write `state.x.ptr[0] + 0.01f` into preview.x[0]**: Block 41 FAILs (memcmp non-zero, `frameMismatch=0`, `eqAllFrames=0`). Restore.
   - **(b) Skip the entire R-5 resync loop**: Block 41 FAILs because preview stays frozen while state.x falls. Block 40 also FAILs. Skip — already covered by R-5's bug-probe (a).

6. **Append D-042 R-6 to `docs/DECISIONS.md`**.

7. **Update `.agent/PROJECT_STATE.md`**: next-milestone updates R-6 → R-7.

8. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "Add a self-test block" — additive verb, parallel-symbol invariant trivially satisfied.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice. Edges: zero-frame run (loop body skipped, eqAllFrames=0, FAIL); zero-vertex mesh (memcmp size=0 returns 0 — equal, but `eqAllFrames` still increments → PASS; degenerate case not specifically tested but not broken either).

## Expected metrics

- Self-test count: **71 → 72**.
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest 159/159 + 1120/1120 SUCCESS unchanged.
- Linux container: Block 41 SKIPs (Metal-gated).
- Expected matrix delta: none.
- Expected DECISIONS.md delta: D-042 R-6 entry added.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTEs:
  - (i) Pure additive test slice — no production diff to review.
  - (ii) memcmp is bit-strict; future floating-point reordering could trigger spurious fails (acceptable for v1 since R-5 is std::memcpy).
  - **WARNING** if: Block 41 FAILs (resync isn't actually byte-equal — would indicate a hidden bug in R-5 or R-3).
  - **BLOCK** if Block 41 FAILs on macOS.
