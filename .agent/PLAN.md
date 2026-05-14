# PLAN — D-042 R-5: packed→preview resync at end of update — `feat/d-042-r-5-update-resync`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 40 (D-042 R-4) returned **WARNING** with 0 BLOCK + 1 WARNING + 2 NOTEs. WARNING flagged scope drift on the per-call `pendingRotations` retirement (Generator-discovered necessity to avoid double-rotation; documented in DECISIONS). NOTEs flagged stale comments in `rotateObject` + `applyPendingMaterials` (fixed in same slice). All folded.

R-4 merged to `main` via commits `0884e2a add:` + `450385e chore:`. Self-test 69 → 70 PASS deterministic on macOS.

## Goal

**Add a packed → preview resync loop at the END of `Simulator::update`** so the post-substep packed state (cloth integrator output, rigid backend Δpos delta-loop output, narrow-phase constraint resolution) lands in `req.preview.x` before any subsequent edit or render. R-3 was preview→packed at pack time; R-4 was edit-side dual-writes; R-5 closes the loop from packed→preview at update end. After R-5, PreviewState is a true "current sim state" view — `translateObject` / `rotateObject` post-update reads a preview that already reflects simulated motion, and future renderers iterating preview see the correct positions.

Block 40 verifies the resync via cloth-under-gravity: addCloth at y=0.25 → sim.initialize → unpause → 30 sim.update() → assert `req.preview.x[1] < 0.20` (cloth fell ≥ 0.05 in y over 30 frames). Without R-5's resync, preview.x stays frozen at addX-time positions while state.x falls. Self-test count 70 → 71.

## Scope

**Design call (1) — Resync placement at the VERY END of `Simulator::update`.** The current `Simulator::update` body (line ~5606-5817) does (in order):
1. Pre-pause dirty check (R-3 + earlier: re-initialize if dirty).
2. `applyEnvironmentForces()` — populates `externalForces` for cloth meshes.
3. `rigid_.step(h, 1)` + Δpos delta-loop (D-039): updates state.x + xPrev + transformPosition for Rigid meshes.
4. Substep loop: `ExplicitSystem<METAL>::update(...)` runs the cloth integrator on GPU; broad/narrow phase + constraint resolution.
5. `system.acctime += system.h; frame++;`.

R-5 inserts the resync RIGHT AFTER `frame++` (line ~5813) and BEFORE the closing `}` of `update()`. By placement at the very end, all integrator + delta-loop + constraint updates have already landed in `state.x`. The resync is a one-way copy `state.x → preview.x`.

**Design call (2) — Resync body and size guard.** Mirrors R-3's pack memcpy shape:
```cpp
// D-042 R-5 (2026-05-14): resync packed state.x into per-request
// preview.x at the END of every update. After this point preview
// reflects the current simulated positions; subsequent edits
// (translateObject / rotateObject) and future renderers reading
// preview see post-sim state. Mirrors R-3's preview→packed memcpy
// in reverse. Size-guarded — silently skipped on mismatch (defensive
// for future no-preview initializers; for v1 every mesh has preview).
for (auto& mesh : Scene<BE, PR>::meshes) {
    if (!mesh.state.x.ptr) continue;
    const size_t stateVerts = (size_t)(mesh.state.x.size / 3);
    for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
        if (req.id != mesh.id) continue;
        if (req.preview.numPoints() != stateVerts) break;
        if (req.preview.x.size() < stateVerts * 3) break;
        std::memcpy(req.preview.x.data(),
                    mesh.state.x.ptr,
                    stateVerts * 3 * sizeof(PR));
        break;
    }
}
```

**Design call (3) — Why update normals here?** R-1's PreviewState carries `n` (per-vertex normals) computed at addX/populatePreview time. The resync above copies `state.x` only — `preview.n` becomes stale. Two options:
- **(a) Recompute preview.n at end of resync** by calling `req.preview.recomputeNormals()`. Cost: O(numFacets + numPoints) per mesh per frame.
- **(b) Leave preview.n stale.** Current renderer uses MeshGL's own normals (computed at materialization OR refreshed via `MeshGL::updateBuffer` → `computeNormal`). Preview.n is currently unread by any post-R-2 consumer.

Pick **(b)** — leave preview.n unchanged this slice. R-7 cleanup can decide whether to wire preview.n into a future renderer path. Block 40 only checks preview.x, so leaving preview.n stale doesn't affect this slice's gate.

**Design call (4) — Block 40 shape (cloth-under-gravity).**
1. `resetScene()`; `sim.addCloth(2, 0.5f, tinym::vec3(0, 0.25f, 0), 1e3, 1e3, 1e3, 0.01f, 0.1f);` — 4-particle cloth (2×2), small + light so gravity drops it fast.
2. `sim.initialize();` — populates meshes + pack memcpys preview.
3. Capture `pre_y = reqs[0].preview.x[1]` (vertex 0's y component after init — should be ~0.25 modulo any addX jitter).
4. `sim.pause = false;`.
5. Pump 30 frames: `for (int i = 0; i < 30; ++i) sim.update();`.
6. Capture `post_y = reqs[0].preview.x[1]`.
7. Assert `post_y < 0.20f` (fell ≥ 0.05m — semi-implicit Euler at h=1/60 with g=-9.81 gives Δy ≈ -0.122m over 30 frames, well past the 0.05 threshold).
8. Also assert `post_y < pre_y - 0.01f` (preview moved at all — guards against pre_y already being ≪ 0.20).
9. Sanity: `state_y = Scene<>::meshes[0].state.x[1]` — assert `|state_y - post_y| < 1e-5f` (resync is byte-equal). Confirms the resync IS the source of the drop, not a stale preview value happening to match.

Pass label: `D-042 R-5 / Simulator::update resyncs packed state.x into preview.x (cloth falls in preview after 30 frames)`.

**Design call (5) — Block 40 placement: INSIDE the Metal-gated section, AFTER Block 39.** Block 40 needs `sim.initialize` + `sim.update` (Metal compute). Linux container SKIPs along with prior Metal-gated blocks. Same pattern.

**Design call (6) — `state.x[1]` semantics for the cloth test.** `addCloth(particleNum1D=2, size1D=0.5, center=(0, 0.25, 0))` creates a 2×2 grid in the XZ plane. Vertex 0 is at (`-0.25, 0.25, -0.25`) or similar — y-component is 0.25 modulo any per-particle jiggle (per D-018). The jiggle perturbation is small (~1mm); pre_y ≈ 0.25 ± 0.001. Cloth falls under gravity; post_y ≈ 0.25 - g*h²*30² ≈ -0.12.

Threshold `post_y < 0.20` gives 0.05m headroom over the perfect-physics expectation of 0.13m drop. Generous to accommodate the cloth's spring damping + collision-with-nothing (no ground in Block 40's scene). The assertion is qualitative: "preview tracks falling motion".

**NEW symbols this slice adds**:
- Block 40 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-5 entry.

**MODIFIED symbols in place**:
- `src/main.cpp` — `Simulator::update` gains the resync loop at the very end (after `frame++`).
- `.agent/PROJECT_STATE.md` — next-milestone updates.

**PRESERVED symbols** (parallel-symbol invariant):
- `Simulator::update` other-than-the-resync — UNCHANGED. All substep + dirty-check + integrator + Δpos delta-loop logic intact.
- `Simulator::translateObject` / `rotateObject` / `setMaterial` — UNCHANGED.
- `MeshRenderState` API — UNCHANGED.
- `PreviewState<PR>` — UNCHANGED.
- `Scene::pack`'s R-3 memcpy — UNCHANGED.
- All Metal kernels, BVH, narrow-phase — UNCHANGED.
- Block 1-39 — UNCHANGED.

## Non-goals

- **NO update of `preview.n` in the resync.** Deferred to R-7 cleanup.
- **NO update of `preview.facets` in the resync.** Topology never changes mid-sim.
- **NO removal of any other code paths.** R-7 cleanup candidate: `meshes[i].initialize()` in `Scene::pack` could become adjacency-only (skip data write) since R-3 memcpy overrides it.
- **NO self-test migration to read via Scene::meshes preview alias.** That's R-6.
- **NO new BDD/FRD/CM.**
- **NO C-* FlatBuffers work.**

## Spec substitution

None. R-5 is infrastructure work.

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED.
- **D-013 swept-CCD xPrev invariant** — Resync writes state.x → preview.x. preview.xPrev is NOT in PreviewState (we don't have it). `state.xPrev` is mutated by R-4 translate/rotate dual-write paths only; integrator updates state.x but writes back to xPrev internally during the substep. The resync ignoring xPrev is correct — preview tracks the post-step position, not the swept-CCD snapshot.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r5-update-resync` on branch `feat/d-042-r-5-update-resync` (branched off main HEAD `450385e`). Submodules already initialized. Commit prefix `add:`.

2. **`src/main.cpp` — Simulator::update resync loop.** Insert AFTER `frame++;` (line ~5813) and BEFORE the closing `}` of `update()`. Body per Design call (2).

3. **New Block 40** in `runSelfTest` — inserted INSIDE the Metal-gated section, AFTER Block 39 (D-042 R-4). Cloth-under-gravity verification per Design call (4).

4. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **71/71 PASS** each time.

5. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

6. **Bug-probes** (each FAIL after revert; restore after):
   - **(a) Comment out the entire R-5 resync loop**: Block 40 FAILs with `post_y ≈ pre_y ≈ 0.25` (preview frozen at addX positions). Sanity-y mismatch `state_y << post_y`. Restore.
   - **(b) Inside the resync, write only the first 3 floats (vertex 0)**: Block 40 PASSes if vertex 0's y drops (test only checks vertex 0). Skip (b) — degenerate.
   - **(c) Replace state.x source with preview.x in the memcpy (no-op self-copy)**: Block 40 FAILs (preview never updates). Equivalent signal to (a); skip.

7. **Append D-042 R-5 to `docs/DECISIONS.md`**.

8. **Update `.agent/PROJECT_STATE.md`**: next-milestone updates R-5 → R-6.

9. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "add resync at end of update" — additive verb, parallel-symbol invariant. The resync is a NEW write; nothing pre-existing changes.
- **D-013 swept-CCD invariant**: doesn't apply to preview (preview doesn't carry xPrev).
- **D-026 lifetimeId**: doesn't apply (resync iterates meshes by id, which is monotone post-D-041).
- **CM-012**: applies — no new exit-able utilities.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice. Edges: empty preview (size guard skips), size mismatch (size guard skips), zero-vertex mesh (`!ptr` continue), mesh removed mid-sim (covered by the per-mesh loop iterating Scene::meshes which is the post-removeMesh set).

## Expected metrics

- Self-test count: **70 → 71**.
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest 159/159 + 1120/1120 SUCCESS unchanged.
- Linux container: Block 40 SKIPs (Metal-gated).
- Expected matrix delta: none (infrastructure).
- Expected DECISIONS.md delta: D-042 R-5 entry added.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTEs:
  - (i) preview.n stays stale — deferred to R-7.
  - (ii) Per-frame O(N) memcpy adds CPU work; for v1 scenes (≤ a few thousand vertices) negligible.
  - (iii) Nested loop is O(meshes × requests) — could use a hash map for large scenes. Acceptable for v1.
  - **WARNING** if: Block 40 FAILs OR existing Blocks 1-39 regress OR the resync runs on a freed-pool buffer (state.x.ptr dangling after dirty-rebuild).
  - **BLOCK** if Block 40 FAILs on macOS OR pre-existing PASS count regresses.
