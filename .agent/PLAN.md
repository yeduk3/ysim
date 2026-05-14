# PLAN — D-042 R-7: final cleanup — `feat/d-042-r-7-cleanup`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 42 (D-042 R-6 — Block 41 round-trip invariant) returned **NOTE** with zero BLOCK / WARNING / NOTE items — clean verdict. R-6 merged via commits `fefc43a add:` + `96ba6b1 chore:`. Self-test 71 → 72 PASS deterministic on macOS.

## Goal

**Final cleanup slice in the D-042 refactor sequence.** Closes the user's "stop bugs in scene editing + simulation management" goal that motivated the entire R-* sequence (R-1 PreviewState infra → R-2 MeshGL binding → R-3 pack memcpy → R-4 edit dual-write → R-5 update resync → R-6 invariant test → R-7 cleanup).

Three small items:
1. **Recompute `preview.n` in the R-5 resync loop** so normals stay current with simulated geometry (cloth deformation, rigid rotation, translate). Currently preview.n holds addX-time normals — visually stale post-sim.
2. **Add a new long-lived standing constraint** to `docs/roles/PLANNER.md`: the R-3/R-4/R-5 round-trip invariant is load-bearing — any future slice touching `Scene::pack` / `Simulator::update` / `translate|rotate|setMaterial` must preserve byte-equality at observable boundaries.
3. **Document** the legacy `MeshRenderState::getOrCreate` packed-sub-view fallback as "effectively dead in production but kept for safety" — comment update only; no removal.

Block 42 verifies preview.n is no longer stale post-update by snapshotting normal pre/post a 30-frame cloth fall.

Self-test count 72 → 73.

## Scope

**Design call (1) — Add `req.preview.recomputeNormals();` to the R-5 resync.** R-5 currently writes only `state.x → preview.x` (vertex positions). Normals (`preview.n`) stay frozen at addX/populatePreview time. Adding `recomputeNormals()` after the position memcpy:
```cpp
std::memcpy(req.preview.x.data(),
            mesh.state.x.ptr,
            stateVerts * 3 * sizeof(PR));
req.preview.recomputeNormals();  // R-7 addition
break;
```

`recomputeNormals()` is O(numFacets + numPoints), a few hundred ops per cloth/cube. Negligible CPU overhead.

**Design call (2) — Block 42 shape (preview.n no longer stale).**
```cpp
{
    resetScene();
    sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5f,
                 tinym::vec3(0.0f, 0.25f, 0.0f),
                 /*kstretch=*/1e3f, /*kshear=*/1e3f, /*kbend=*/1e3f,
                 /*thickness=*/0.01f, /*mass=*/0.1f);
    sim.initialize();

    auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
    bool oneReq = (reqs.size() == 1);
    bool hasN   = oneReq && reqs[0].preview.n.size() >= 3;

    Precision pre_nx = hasN ? reqs[0].preview.n[0] : Precision(0);
    Precision pre_ny = hasN ? reqs[0].preview.n[1] : Precision(0);
    Precision pre_nz = hasN ? reqs[0].preview.n[2] : Precision(0);

    sim.pause = false;
    for (int i = 0; i < 30; ++i) sim.update();

    Precision post_nx = hasN ? reqs[0].preview.n[0] : Precision(0);
    Precision post_ny = hasN ? reqs[0].preview.n[1] : Precision(0);
    Precision post_nz = hasN ? reqs[0].preview.n[2] : Precision(0);

    // Cloth deformation under gravity should change the per-vertex normal
    // (the cloth bends as it falls — not perfectly planar after 30 frames).
    // Threshold: at least one component differs by > 1e-3.
    const Precision tol = Precision(1e-3f);
    bool nxChanged = std::fabs(post_nx - pre_nx) > tol;
    bool nyChanged = std::fabs(post_ny - pre_ny) > tol;
    bool nzChanged = std::fabs(post_nz - pre_nz) > tol;
    bool normalMoved = nxChanged || nyChanged || nzChanged;

    if (oneReq && hasN && normalMoved) {
        pass("D-042 R-7 / preview.n recomputed in R-5 resync (cloth deformation reflects in preview normal)");
    } else {
        fail("D-042 R-7 / preview.n recomputed in R-5 resync (cloth deformation reflects in preview normal)",
             std::string("oneReq=") + std::to_string((int)oneReq)
             + " hasN=" + std::to_string((int)hasN)
             + " pre_n=(" + std::to_string(pre_nx) + "," + std::to_string(pre_ny) + "," + std::to_string(pre_nz) + ")"
             + " post_n=(" + std::to_string(post_nx) + "," + std::to_string(post_ny) + "," + std::to_string(post_nz) + ")"
             + " normalMoved=" + std::to_string((int)normalMoved));
    }
}
```

Pass label: `D-042 R-7 / preview.n recomputed in R-5 resync (cloth deformation reflects in preview normal)`.

**Design call (3) — Block 42 placement: INSIDE the Metal-gated section, AFTER Block 41.** Uses `sim.initialize` + `sim.update`. Linux container SKIPs.

**Design call (4) — New standing constraint in PLANNER.md.** Append to the `## Standing constraints` section:

> **D-042-ROUND-TRIP-INVARIANT** (R-3..R-6, 2026-05-14). PreviewState and packed state.x are byte-equal at every observable boundary: R-3 memcpys preview → packed at `Scene::pack` time; R-4 dual-writes both on `translateObject` / `rotateObject` edits; R-5 memcpys packed → preview at the end of `Simulator::update`; R-6's Block 41 + R-7's Block 42 pin the invariant via memcmp + normal-change tests. Any future slice touching `Scene::pack`, `Simulator::update`, `Simulator::translateObject`, `Simulator::rotateObject`, `Simulator::setMaterial`, or `MeshRenderState::getOrCreate` must preserve this round-trip. Breaking it triggers Block 41 + Block 42 FAIL plus cascading BDD-003/006/018 regressions (proven by R-6 turn-42 bug-probe). Retires only if PreviewState is structurally replaced or merged into packed.

**Design call (5) — `MeshRenderState::getOrCreate` legacy fallback documentation.** The function still has a "legacy fallback" branch for the case where `previewBindings.find(mesh.id) == previewBindings.end()`. With R-2's universal `registerPreviewBinding` on every `addX` + `loadScene`, the fallback is effectively dead in production. Comment update only — no code removal:
```cpp
// Legacy fallback: construct from packed sub-views (pre-R-2 path).
// R-2 onward: every Simulator::addX wrapper + loadScene calls
// registerPreviewBinding before MeshGL can be materialized, so this
// branch is dead in production. Retained for safety + parallel-symbol
// invariant — hand-built MeshGL via getOrCreate from a test or future
// non-addX path falls back here. R-7 cleanup left intact intentionally.
```

**NEW symbols this slice adds**:
- Block 42 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-7 entry.
- `docs/roles/PLANNER.md` — new D-042-ROUND-TRIP-INVARIANT standing constraint.

**MODIFIED symbols in place**:
- `src/main.cpp` — R-5 resync loop gains `req.preview.recomputeNormals();` after the position memcpy.
- `include/MeshRenderState.hpp` — comment update on the legacy `getOrCreate` fallback branch (no code change).
- `.agent/PROJECT_STATE.md` — D-042 R-7 marks the refactor sequence COMPLETE.

**PRESERVED symbols** (parallel-symbol invariant):
- Legacy `getOrCreate` fallback branch — UNCHANGED (just doc).
- All R-1..R-6 production code — UNCHANGED.
- `PreviewState::recomputeNormals()` — UNCHANGED (R-1 author).
- All Blocks 1-41 — UNCHANGED.

## Non-goals

- **NO removal of legacy `getOrCreate` fallback.** Doc-only.
- **NO removal of `MeshRenderState::clear()` public API.** Doc-only candidate; not this slice.
- **NO removal of `mesh.initialize()`-in-pack adjacency duplication.** Could be split into "adjacency-only" path; deferred.
- **NO new BDD/FRD/CM.**
- **NO C-* FlatBuffers work.**

## Spec substitution

None.

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED.
- **BDD-102-vs-ALEMBIC-BYTES** — UNCHANGED.
- **D-042-ROUND-TRIP-INVARIANT** (NEW) — R-3..R-6 invariant pinned by Blocks 41 + 42; any future slice touching pack/update/translate/rotate/setMaterial/getOrCreate must preserve byte-equality at observable boundaries.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r7-cleanup` on branch `feat/d-042-r-7-cleanup` (off main HEAD `96ba6b1`). Submodules already initialized. Commit prefix `add:`.

2. **`src/main.cpp` — R-5 resync gains `req.preview.recomputeNormals()`** right after the position memcpy. Comment refresh to note this is R-7's addition.

3. **`include/MeshRenderState.hpp` — legacy fallback comment refresh** per Design call (5).

4. **New Block 42** in `runSelfTest` — inserted INSIDE the Metal-gated section, AFTER Block 41 (D-042 R-6). Cloth-deformation-normal-change verification per Design call (2).

5. **`docs/roles/PLANNER.md` — append D-042-ROUND-TRIP-INVARIANT** to the `## Standing constraints` section.

6. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **73/73 PASS** each time.

7. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

8. **Bug-probes** (each FAIL after revert; restore after):
   - **(a) Comment out `req.preview.recomputeNormals()`**: Block 42 FAILs (`normalMoved=0` because preview.n stays at addX-time normals despite cloth deformation). Restore.
   - **(b) Skipping the position memcpy**: would break Block 40 + Block 41 first. Don't need a separate probe for R-7.

9. **Append D-042 R-7 to `docs/DECISIONS.md`** with a slice-summary note that the D-042 refactor sequence is COMPLETE.

10. **Update `.agent/PROJECT_STATE.md`**: mark D-042 refactor sequence COMPLETE; recent scope changes append; standing feature candidates next priorities (Alembic export / B-2.1 Bullet refinements / source-file split).

11. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "Recompute preview.n in the R-5 resync loop" — additive verb in existing function body. The recompute is a NEW operation appended after the existing memcpy. PARALLEL-IMPL-LOCKSTEP not affected.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice in the strict sense, but `recomputeNormals` is math. Edges:
  - All-coplanar mesh → all face normals identical → recomputed n matches addX n trivially. (Block 42's cloth bends, so this edge is bypassed.)
  - Degenerate triangle in preview.facets → `recomputeNormals` already handles zero-area (skip contribution per R-1's implementation).

## Expected metrics

- Self-test count: **72 → 73**.
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest 159/159 + 1120/1120 SUCCESS unchanged.
- Linux container: Block 42 SKIPs.
- Expected matrix delta: none.
- Expected DECISIONS.md delta: D-042 R-7 entry + closure note for D-042 sequence.
- Expected PLANNER.md delta: D-042-ROUND-TRIP-INVARIANT standing constraint added.
- Expected PROJECT_STATE.md delta: "D-042 refactor sequence COMPLETE"; future-slice candidates promoted.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTEs:
  - (i) recomputeNormals() O(F+V) per frame per mesh — negligible for v1 scenes; perf flag for very high-poly scenes.
  - (ii) Legacy fallback now documented-but-untested; future R-7+ might prune.
  - **WARNING** if: Block 42 FAILs OR existing Blocks 1-41 regress.
  - **BLOCK** if Block 42 FAILs on macOS.
