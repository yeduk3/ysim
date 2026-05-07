# Plan — Cloth-CCD Slice (closes CM-005, BDD-007)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Goal

Close `BDD-007`'s tunneling clause by replacing the snapshot point-vs-triangle narrow-phase check with a swept-segment-vs-triangle CCD, so cloth-on-static-ground contact is detected for every substep the cloth's *trajectory* crosses the ground — not only the substep where the cloth happens to be within `radius+thickness=0.022` of the surface at sample time.

When this slice ships:
- `[self-test FAIL] BDD-007 / no cloth vertex tunnels through ground` flips to PASS — CM-005's last unresolved clause.
- `./scripts/verify.sh` exits 0 cleanly on macOS Apple Silicon for the first time since the cloth-drape slice landed (Estimator's BLOCK trigger is gone).
- `docs/TEST_MATRIX.md` row `BDD-007` promotes from `warning` to `pass`.
- The slice also folds in two small follow-up todos from the BDD-002 estimator's WARNING (item 4 below).

## Scope

- `BDD-007` — cloth drapes onto static rigid surface (4/4 clauses pass after this slice; currently 3/4).
- **Narrow-phase swept-segment-vs-triangle check** in `src/metal/bruteforce.metal::narrow_pt_tri`. The cloth particle's *previous* position (`x_prev`) and *current* position (`x_cur`) define a swept segment; if it crosses the ground triangle plane between them and the crossing point lies inside the triangle, register a contact at the crossing. Today's snapshot test (`l = |dot(n, p)|`, accept if `l < radius + thickness`) only catches contacts where the current position is close to the surface — fast-moving thin cloth slips through.
- **`x_prev` plumbing.** The narrow kernel currently only reads `scenePackedPositions` (the *current* state.x). To do CCD it needs the *prior* state.x as well. Cleanest path: a new `packedMeshData.xPrev` buffer, populated by C++ each substep right before `system.update(scene)`. Bind to a new buffer slot in `narrow_pt_tri` (likely buffer(10) or similar — pick a free slot).
- **C++ wiring.** Allocate `xPrev` in `Scene::pack()` alongside `x`; per-mesh `mesh.state.xPrev` slice. In `Simulator::update`'s substep loop, copy `state.x` into `state.xPrev` *before* the integrator runs (so `xPrev` reflects the position at the *start* of the substep). Bind `xPrev` to the narrow kernel's new buffer slot via `BruteForce::narrow`'s `setBuffer` call.
- **Integrator response on CCD contacts.** The existing integrator does `if (vn < 0) vel -= vn*n` and `if (distance < thickness) pos += (thickness - distance) * n`. For a CCD-detected crossing, `distance` should be the post-step distance to the triangle plane (signed); the same response logic works as long as `n` points outward from the triangle into the swept space.
- Estimator follow-up #1 (from `.agent/ESTIMATION.md` turn 4): tighten Block 7's BDD-002 happy-path assertion to also check the imported mesh's AABB is well-defined (`max > min` along each axis) and `numFacets > 0`. One-line change inside the existing block.
- Estimator follow-up #2: change the import modal's default path from `assets/Human.obj` to `src/assets/Human.obj` so the launch-from-`build/` context (used by `verify.sh`) works as a one-click import smoke without retyping. Single literal change.

## Non-goals (this slice)

- **Cloth-on-cloth (self) collision.** Parked at `PRD §4`. Self-CCD would be a separate, larger slice.
- **Continuous-time response for inter-substep cloth-on-rigid.** No rigid pipeline yet (Q4 blocked); the slice only addresses Float-tagged static surfaces.
- **A second narrow-phase kernel.** Modify `narrow_pt_tri` in place rather than introducing a parallel CCD kernel — the existing snapshot path is replaced, not paralleled.
- **Velocity-aware AABB inflation.** Already done (D-005-area; `enlargeTrajectory(system.subh)` in Simulator::update). Keep that as-is.
- **Refactoring the narrow-collision struct.** The existing `NarrowCollision::collisionNormalAndDistance` (vec4) carries (n, l) which is sufficient for both snapshot and swept results — the swept kernel writes the same shape.
- **Resolving the BDD-002 NOTE** about `importMesh`'s coalesced "file not found" error message. Tasteful, deferred.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6, Q7.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Read CM-005 and the relevant kernels.** `docs/mistakes/COMMON_MISTAKES.md::CM-005` lays out the snapshot-vs-swept gap and the localization. `src/metal/bruteforce.metal::narrow_pt_tri` (~line 30) is the kernel to rewrite. `src/metal/physics.metal::integrate_cloth` (~line 280) and `integrate_cloth_grid` (~line 156) consume `vertColFacets` — their loops should keep working with the new contact-distance semantics.
2. **Add `xPrev` to PackedMeshData and per-mesh state.** In `src/main.cpp::Scene::pack()`, allocate `packedMeshData.xPrev = VectorBase<BE, PR>(numStatesData)`, slice into each `meshes[i].state.xPrev`. The `MeshState` struct (~line 819) already has room semantically; either add `Vector xPrev;` next to `x, v, f, m, n` or carry it on the Simulator side as a parallel mirror. The Generator's call which to pick — favor adding to `MeshState` so the field travels with the mesh.
3. **Populate `xPrev` per substep before the integrator.** In `Simulator::update`'s substep loop, after the broad/narrow phase and **before** `system.update(scene)`, copy `state.x` → `state.xPrev` for every cloth mesh. Use the existing CPU-readable Metal pointers (Apple Silicon unified memory). One memcpy per cloth per substep — cheap.
4. **Bind `xPrev` to the narrow kernel.** Look at `BruteForce<METAL, PR>::narrow(...)` (~line 4012, where `bruteForcePSO` is dispatched). It already binds `scenePackedPositions` at buffer 3. Add a parallel `setBuffer(packedMesh.xPrev, /*new slot*/)` call. Pick a free slot — current `narrow_pt_tri` uses 0–9; slot 10 is unused. Update the kernel signature to accept `device const packed_float3* xPrevPacked [[buffer(10)]]`.
5. **Rewrite `narrow_pt_tri` to do swept CCD.** New logic:
   - Read both `x_prev = xPrev[point + offset]` and `x_cur = scenePackedPositions[point + offset]`.
   - Compute signed distances `d_prev = dot(n, x_prev - t0)` and `d_cur = dot(n, x_cur - t0)` (n is the triangle's outward normal — keep the existing `cross(v0, v1)` then normalize).
   - **CCD trigger:** if `sign(d_prev) != sign(d_cur)` (segment crosses the plane) **OR** `|d_cur| < radius + thickness` (snapshot fallback for slow particles already touching), proceed. Otherwise return.
   - Compute the crossing point parametric `t = d_prev / (d_prev - d_cur)` if signs differ, else `t = 1`. Crossing position `p_cross = lerp(x_prev, x_cur, t)`.
   - Run the existing `pointInTriangleBary` against the crossing (or against `x_cur`'s in-plane projection — pick whichever stays consistent with the integrator's response).
   - Write `n` (oriented outward, flipped if the *current* position is on the negative side) and `l = d_cur` (signed) — the integrator already handles `if (distance < thickness) pos += (thickness - distance) * n` which correctly pushes both barely-above and below-surface particles outward.
6. **Drop the spurious `if (l < 0) { n = -n; l = -l; }` flip.** That existing block makes `l` always positive, which is wrong for tunneled particles (`distance < 0` should remain negative so the integrator's `(thickness - distance)` push grows correctly). Replace with a "set `n` to outward-relative-to-current-position" rule that does **not** flip `l`'s sign.
7. **Verify locally.** Run `./scripts/verify.sh` from the repo root. Expect:
   - Build clean.
   - Doctest binaries pass.
   - 15 self-test PASS lines (was 14); the BDD-007 tunneling clause flips to PASS.
   - `verify.sh` exits 0.
8. **Promote `BDD-007` matrix row from `warning` to `pass`.** The new test address points at the same Block 6 lines (no harness change for BDD-007) — the assertion was already correctly written; the kernel is what changes.
9. **BDD-002 follow-ups (housekeeping; from estimator turn 4 WARNING).**
   - In `src/main.cpp` modal init (~line 5912 area): change `static char importPathBuf[512] = "assets/Human.obj";` → `"src/assets/Human.obj";` so the default path resolves in the `verify.sh` launch context.
   - In Block 7's happy-path assertion (`src/main.cpp::runSelfTest` ~line 5558): after the existing `state.x.size > 0 && behaviorType == Float` check, also assert `mesh->adjacency.facets.size > 0` and that the per-axis AABB max > min over the imported vertex positions. One small loop. Add a third PASS line `BDD-002 / imported mesh has well-defined geometry` — addresses estimator follow-up #1 by widening coverage.
10. **Update DECISIONS, CURRENT_WORK, RESUME.** Append a new D-NNN entry: "Narrow phase upgraded from snapshot to swept-segment CCD (closes CM-005)". Note the kernel signature change (new `xPrev` buffer slot) so future kernel-side reviewers know to grep for it. Update CM-005 to "fixed in this slice; entry can graduate to `OLD_MISTAKES.md` after one slice with no recurrence" per the file's own promotion rule.
11. **Stop and hand off to the Estimator.** Do not pile on cloth-on-cloth, rigid, or visual-rendering changes.

## Course corrections

- **Standing CM-005 BLOCK ends here.** Two slices have shipped with it as the only `verify.sh` failure. After this slice, Estimator's BLOCK signal returns to its proper meaning ("real failure") instead of "the standing tunneling FAIL".
- **`enlargeTrajectory(system.subh)` from the cloth-drape slice is a *prerequisite* for CCD**, not a substitute. Without trajectory-inflated AABBs, broad phase doesn't even feed pairs to narrow. Both fixes are load-bearing for BDD-007; do not remove either.
- **The narrow kernel was the locus of the last two structural bugs** (CM-004's hardcoded gravity ignored the bound buffer; CM-005's snapshot test missed swept contacts). Future slices touching `src/metal/bruteforce.metal` or `physics.metal` should grep both the C++ side `setBuffer` index *and* the kernel body for that buffer to confirm consumption — that's the lesson from CM-004 carrying forward.
- **BDD-002 follow-ups bundled into this slice as housekeeping**, per Planner role: "WARNING adds a follow-up todo." Both items are tiny (≈3 lines each); a separate slice would be process overhead for less than 30 minutes of work.

## What to read before writing code

- `docs/mistakes/COMMON_MISTAKES.md::CM-005` — full localization + fix direction.
- `docs/TESTS.md#BDD-007` — the four "Then" clauses; tunneling clause is `no cloth vertex tunnels through the sphere`.
- `src/metal/bruteforce.metal::narrow_pt_tri` (~line 30) — the kernel to rewrite.
- `src/metal/physics.metal::integrate_cloth` (~line 280) and `integrate_cloth_grid` (~line 156) — consumers of `vertColFacets`; the response loop should keep working with signed `l`.
- `src/main.cpp::Scene::pack` (~line 1804), `MeshState` (~line 819), `Simulator::update`'s substep loop (~line 4470) — for the `xPrev` plumbing.
- `src/main.cpp::BruteForce::narrow` (~line 4012, `bruteForcePSO` dispatch) — to add the `xPrev` buffer binding.
- `.agent/ESTIMATION.md` (turn 4) — the two follow-up WARNINGs being folded into todo 9.
- `.agent/RESUME.md` — `enlargeTrajectory` line is load-bearing; `cumulativeNarrowCollisions` infrastructure carries forward; `importMesh` path-existence guard carries forward.
