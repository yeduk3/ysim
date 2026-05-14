# PLAN — D-042 R-4: edits route through PreviewState — `feat/d-042-r-4-edits-through-preview`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 38 returned **BLOCK** on the R-3 turn-37 WARNING fold-in; turn 39 (BLOCK fix-turn) returned **NOTE**: BLOCK item closed by adding `renderState.clear()` alongside `clearPreviewBindings()` in `Simulator::loadScene`. NOTEs flagged the translate dual-write as a "bridge to R-4" and Block 38's coverage gap on the n/facets memcpy branches.

R-3 merged to `main` via commits `c1bc127 add:` + `a88b312 chore:`. Self-test 68 → 69 PASS deterministic on macOS.

## Goal

**Add `rotateObject` preview write-back to mirror R-3's `translateObject` dual-write pattern**, completing the "edits land in preview eagerly" half of R-4. `setMaterial` is preserved as-is (preview holds geometry only — not material). The translate dual-write from R-3 is documented as the canonical pattern (no longer transitional). Block 39 verifies rotateObject's preview write by rotating a cube 90° around Y and asserting that `preview.x` for vertex 0 reflects the rotated position. Self-test count 69 → 70.

## Scope

**Design call (1) — Narrow scope: only `rotateObject` gets a new preview write.** Per the user brief:
- `translateObject` — R-3 already dual-writes (state.x AND preview.x by the same delta). No code change this slice; comment-only update to mark it canonical.
- `rotateObject` — NEW preview dual-write added. Mirrors the existing state.x rotation around pivot.
- `setMaterial` — UNCHANGED. Preview is geometry-only; material persistence already handled by `pendingMaterials` + `Scene::dirty` flag.

**Design call (2) — `rotateObject` preview math.** The existing state.x rotation at `src/main.cpp:5251-5258` computes `p_rot = pivot + rotateVector(delta, p_curr - pivot)` per vertex. R-4's preview write mirrors this exactly:
```cpp
// After the existing state.x rotation loop, before the existing
// rotationQuat assignment.
for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
    if (req.id == meshId) {
        const size_t np = req.preview.numPoints();
        for (size_t i = 0; i < np; ++i) {
            tinym::vec3 p(req.preview.x[i*3+0],
                          req.preview.x[i*3+1],
                          req.preview.x[i*3+2]);
            tinym::vec3 p_rot = pivot + rotateVector(delta, p - pivot);
            req.preview.x[i*3+0] = p_rot.x;
            req.preview.x[i*3+1] = p_rot.y;
            req.preview.x[i*3+2] = p_rot.z;
        }
        break;
    }
}
```

The `delta` quaternion + `pivot` vec3 are already computed by the state.x rotation block. The preview write reuses both.

**Design call (3) — Why preview write-back is needed for rotate at all.** Pre-R-4, rotate persistence through Scene::pack worked via D-025's pendingRotations + applyPendingMaterials (re-applied AFTER pack to state.x). Post-R-3, Scene::pack memcpys preview→packed, so state.x post-pack reflects preview.x. The D-025 re-apply then runs on the post-memcpy state.x.

So pre-R-4 with R-3 alone: rotate's D-025 flow still works (rotation re-applies after pack to preview-sourced state.x). **Existing 69/69 PASS confirms this.**

R-4's preview write-back is **defensive** — it ensures preview tracks rotation eagerly so:
- Pre-init rendering (R-2 binding path) reflects the rotated mesh.
- Future R-5 (packed→preview resync) doesn't need a special "rotate hasn't synced yet" branch.
- The PreviewState's "source of truth" claim is fully consistent for both translate and rotate.

**Design call (4) — Block 39 shape.** Verify rotate's preview write via a known-result rotation:
1. `resetScene()`; `sim.addCube(tinym::vec3(0,0,0), 2, 0.2f, 1.0f)` — cube at origin, tess=2 → first vertex at (-0.1, -0.1, -0.1).
2. Capture `pre_v0 = (reqs[0].preview.x[0], reqs[0].preview.x[1], reqs[0].preview.x[2])` → expected (-0.1, -0.1, -0.1).
3. Construct 90° Y rotation: `::Quat q90; q90.w = cos(π/4); q90.x = 0; q90.y = sin(π/4); q90.z = 0;`.
4. `sim.rotateObject(0, q90)` — pivot = transformPosition (origin); rotates preview.x.
5. Capture `post_v0 = (reqs[0].preview.x[0], reqs[0].preview.x[1], reqs[0].preview.x[2])`.
6. Expected after 90° Y rotation around origin of (-0.1, -0.1, -0.1):
   - Y stays: -0.1
   - 90° Y: (x, z) → (z, -x) per the standard right-hand rule, so (-0.1, -0.1) → (-0.1, 0.1).
   - Result: (-0.1, -0.1, 0.1).
7. Assert each component within 1e-4 tolerance.

Pass label: `D-042 R-4 / rotateObject writes preview.x (90° Y rotation reflects in preview)`.

**Design call (5) — Block 39 placement: INSIDE the Metal-gated section, AFTER Block 38.** Block 39 uses `sim.addCube` → Metal allocation. Same SKIP pattern as Block 36/37/38. Acceptable.

**Design call (6) — Verify the 90° Y rotation matrix.** The standard right-hand-rule Y rotation by θ:
```
[ cos θ   0   sin θ ]
[   0     1    0    ]
[-sin θ   0   cos θ ]
```
For θ=90°: (x, y, z) → (z, y, -x).

For vertex (-0.1, -0.1, -0.1):
- new_x = z = -0.1
- new_y = y = -0.1
- new_z = -x = 0.1

So expected post-rotate: **(-0.1, -0.1, 0.1)**.

**Design call (7) — Tolerance for floating-point comparison.** Use `1e-4f` for each component. Quaternion construction from cos/sin of π/4 introduces ~1e-7 error; rotateVector multiplication compounds slightly. 1e-4 is conservative without being permissive.

**NEW symbols this slice adds**:
- Block 39 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-4 entry.

**MODIFIED symbols in place**:
- `src/main.cpp` — `Simulator::rotateObject` gains the preview write-back loop (mirrors the existing state.x rotation). Comment note added to `translateObject` marking the dual-write as canonical (no longer transitional per R-3's "bridge" framing).
- `.agent/PROJECT_STATE.md` — next-milestone updates.

**PRESERVED symbols** (parallel-symbol invariant):
- `Simulator::translateObject` body — UNCHANGED (R-3's dual-write stays; only the comment is refreshed).
- `Simulator::setMaterial` — UNCHANGED.
- `Simulator::rotateObject` other-than-the-new-preview-loop — UNCHANGED. D-025 pendingRotations flow stays.
- `MeshRenderState` (all R-2 + R-3 API) — UNCHANGED.
- `PreviewState<PR>` — UNCHANGED.
- `Scene::pack`'s R-3 memcpy block — UNCHANGED.
- All Metal kernels, BVH, narrow-phase — UNCHANGED.
- Block 1-38 — UNCHANGED.

## Non-goals

- **NO change to `setMaterial`.** Preview is geometry-only.
- **NO removal of D-025 pendingRotations re-apply flow.** That's a future R-7 cleanup candidate (rotate could become preview-primary once R-5 lands the packed→preview resync).
- **NO change to `translateObject` body.** R-3 already covers the dual-write.
- **NO packed → preview resync at end of `Simulator::update`.** That's R-5.
- **NO self-test migration to read via Scene::meshes preview alias.** That's R-6.
- **NO new BDD/FRD/CM.**

## Spec substitution

None. R-4 is infrastructure work.

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED.
- **D-021 rotateVector / Quat math invariants** — APPLIES. The preview write uses the same `rotateVector(delta, ...)` helper as state.x. If a future slice changes the rotation pivot convention or the quaternion ordering, both call sites need the change.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r4-edits-through-preview` on branch `feat/d-042-r-4-edits-through-preview` (branched off main HEAD `a88b312`). Submodules already initialized. Commit prefix `add:`.

2. **`src/main.cpp` — `Simulator::rotateObject` preview write-back.** Insert AFTER the existing state.x rotation loop (after `mesh->state.xPrev.ptr[i*3+2] = prev_rot.z;` closing the loop) and BEFORE the `mesh->rotationQuat = newAbs;` assignment. Body per Design call (2).

3. **`src/main.cpp` — `Simulator::translateObject` comment refresh.** The R-3 dual-write comment block currently says "R-3 dual-writes; R-4 will make preview the primary mutation target with state.x derived." Update wording to reflect R-4 reality: preview is canonical for both translate (R-3) and rotate (R-4); the state.x update stays for D-014's immediate-effect and D-023's BVH refit.

4. **New Block 39** in `runSelfTest` — inserted INSIDE the Metal-gated section, AFTER Block 38 (D-042 R-3). Body per Design call (4):
   ```cpp
   // ---- Block 39: D-042 R-4 — rotateObject writes preview.x. -------------
   // R-3 made Scene::pack memcpy preview→packed; R-4 makes rotateObject
   // dual-write state.x AND preview.x so the preview stays consistent
   // with the rotated state across pack rebuild + pre-init rendering.
   // Mirrors R-3's translate dual-write pattern.
   //
   // Mechanic: addCube places vertex 0 at (-h, -h, -h) = (-0.1, -0.1, -0.1)
   // for tess=2, size=0.2 cube at origin. 90° Y rotation around the cube's
   // transformPosition pivot (also origin) maps (x, y, z) → (z, y, -x), so
   // vertex 0 → (-0.1, -0.1, 0.1). Assert preview.x[0..2] within 1e-4.
   //
   // Bug-probe: removing the R-4 preview write loop → preview.x is
   // unchanged from addX time → assertion FAILs.
   {
       resetScene();
       sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), 2, 0.2f, 1.0f);

       auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
       bool oneReq = (reqs.size() == 1);
       bool hasPreview = oneReq && reqs[0].preview.x.size() >= 3;

       Precision pre_x = hasPreview ? reqs[0].preview.x[0] : Precision(0);
       Precision pre_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);
       Precision pre_z = hasPreview ? reqs[0].preview.x[2] : Precision(0);

       ::Quat q90;
       q90.w = std::cos(0.78539816339f);  // π/4
       q90.x = 0.0f;
       q90.y = std::sin(0.78539816339f);
       q90.z = 0.0f;
       sim.rotateObject(0, q90);

       Precision post_x = hasPreview ? reqs[0].preview.x[0] : Precision(0);
       Precision post_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);
       Precision post_z = hasPreview ? reqs[0].preview.x[2] : Precision(0);

       // Expected after 90° Y rotation of (-h, -h, -h) around origin:
       //   new_x = z = -h, new_y = y = -h, new_z = -x = +h
       // For h = 0.1: expected (-0.1, -0.1, 0.1).
       const Precision tol = Precision(1e-4f);
       bool xOk = std::fabs(post_x - Precision(-0.1f)) < tol;
       bool yOk = std::fabs(post_y - Precision(-0.1f)) < tol;
       bool zOk = std::fabs(post_z - Precision( 0.1f)) < tol;
       bool moved = std::fabs(post_x - pre_x) > tol
                    || std::fabs(post_z - pre_z) > tol;

       if (oneReq && hasPreview && xOk && yOk && zOk && moved) {
           pass("D-042 R-4 / rotateObject writes preview.x (90° Y rotation reflects in preview)");
       } else {
           fail("D-042 R-4 / rotateObject writes preview.x (90° Y rotation reflects in preview)",
                std::string("oneReq=") + std::to_string((int)oneReq)
                + " hasPreview=" + std::to_string((int)hasPreview)
                + " pre_v0=(" + std::to_string(pre_x) + "," + std::to_string(pre_y) + "," + std::to_string(pre_z) + ")"
                + " post_v0=(" + std::to_string(post_x) + "," + std::to_string(post_y) + "," + std::to_string(post_z) + ")"
                + " xOk=" + std::to_string((int)xOk)
                + " yOk=" + std::to_string((int)yOk)
                + " zOk=" + std::to_string((int)zOk)
                + " moved=" + std::to_string((int)moved));
       }
   }
   ```

5. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **70/70 PASS** each time.

6. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

7. **Bug-probes** (each FAIL after revert; restore after):
   - **(a) Comment out the rotateObject preview write loop**: Block 39 FAILs (post_v0 == pre_v0, `moved=0`). Restore.
   - **(b) Skip the preview pivot subtraction (use `p_rot = rotateVector(delta, p)` without subtracting/adding pivot)**: Block 39 FAILs because the cube is at origin so pivot=(0,0,0); the rotation would be equivalent. Skip (b) — degenerate test case.

8. **Append D-042 R-4 to `docs/DECISIONS.md`**.

9. **Update `.agent/PROJECT_STATE.md`**: next-milestone updates R-4 → R-5.

10. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "route through PreviewState" — integration verb, in-place modification of `rotateObject` is correct. The new code is a parallel addition (the existing state.x rotation stays; preview write is added next to it).
- **`project_flatbuffers_caching_skipped`**: stays in force.
- **D-021 rotateVector invariant**: applies — preview write uses the same `rotateVector(delta, ...)` helper as state.x. PARALLEL-IMPL-LOCKSTEP across the two loops.
- **D-025 pendingRotations**: applies — UNCHANGED. The flow still re-applies pendingRotations to state.x after Scene::pack. Preview's eager rotation is additional defense, not a replacement.
- **CM-012 utility-helper-exit trap**: applies — no new exit-able utilities.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: this is a math-touching slice (rotateVector). Edges considered:
  - 90° Y rotation around origin — Block 39's case.
  - Identity rotation (delta = identity quat) — preview write is a no-op (rotateVector returns input). Acceptable; verified implicitly by the dual-write fanning into state.x's existing-passing tests.
  - Non-origin pivot — not exercised by Block 39 (cube at origin); the math is symmetric and the state.x loop already handles it. Documented gap.
  - Sequential rotations — not exercised; preview tracks each individually. Documented gap.

## Expected metrics

- Self-test count: **69 → 70**.
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest 159/159 + 1120/1120 SUCCESS unchanged.
- Linux container: Block 39 SKIPs along with Blocks 1-29 + 32-38.
- Expected matrix delta: none (R-4 is infrastructure).
- Expected DECISIONS.md delta: D-042 R-4 entry added.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTEs:
  - (i) Non-origin pivot not exercised by Block 39.
  - (ii) Sequential rotations not exercised.
  - (iii) `setMaterial` no-op for R-4 is correct but worth noting as a documented decision.
  - **WARNING** would land if: Block 39 FAILs OR existing Blocks 1-38 regress OR the preview rotation diverges from state.x by more than FP noise (lockstep break).
  - **BLOCK** if Block 39 FAILs on macOS OR pre-existing PASS count regresses OR D-025 pendingRotations re-apply path breaks (silent revert of preview rotation by re-apply over state.x).
