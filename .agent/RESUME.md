# Resume — Triangle-precision Click-pick Slice (D-024 lands; user-reported regression closed)

## Must remember

- **Branch:** `fix/click-triangle-precision` (off `main` at `ab0cae4`).
- **D-024 is the canonical click-pick semantic.** `BVH::queryClickRay`'s leaf branch does ray-vs-triangle (Möller–Trumbore) using the BVH's existing `positions.ptr` (a slice into mesh.state.x) and `primitives.ptr` (a slice into mesh.adjacency.facets) — both populated at `BVH::build` time and pointing into the mesh's live storage. The interior-node AABB filter stays for cheap traversal pruning, but the **value written to `clickRayCollisions[]` is the triangle's actual `t`**, so the consumer's existing smallest-tmin walk (production callback + Block 14 `pickClosest`) is automatically triangle-precise.
- **No consumer-side helper, no two-stage walk.** The BVH produces triangle hits directly; the old proposal of a separate `pickClosestTriangleHit` was redundant. User's intuition ("그냥 ray를 BVH에 쿼리 시키면 원래 잘 intersection 정보가 모일텐데") was correct.
- **The BVH stores mesh-data slices via `positions = pos; primitives = prim;`** at line 3401–3402 in `BVH::build(int oid, ...)`. These are slice views (ptr/offset/size), not deep copies — mutations to `mesh.state.x` (via `rotateObject`/`translateObject`) are seen by the BVH automatically. D-023's refit re-reads through these to update leaf AABBs; D-024 reads through them at query time for triangle-vs-ray.
- **Möller–Trumbore epsilon = 1e-6** for the determinant test. Smaller would reject grazing hits; larger would let parallel rays divide by near-zero. v1's mesh sizes don't graze.
- **CM-010 records the trap pattern.** Any future BVH variant for "user-facing intersection" (shadow rays, hover hit-tests, pickup, raycast queries) must do primitive-vs-ray at the leaf, not just AABB-tmin. AABB filter is for performance; the leaf write is the truth.
- **Block 17 discriminates AABB-vs-triangle** with a size-30 ground tilted 60°-X — Plane AABB tmin = 2.5 (would beat cube's 9.75 under old AABB-only logic), but triangle tmin = 10.577 (loses to cube under new triangle logic). The user's exact reported quat (1, 2, 0, 0)-normalized actually has Plane geometrically in front of cube, so it doesn't discriminate; Block 17 uses a more targeted scene.
- **No specific commit "broke" click-pick** — the leaf-AABB-tmin write was latent since the file's history. D-021 + D-023 just made tilted poses reachable.

## Last decisions + why

- **D-024 — Shape A (BVH-internal) over Shape B (consumer-side).** Rejected the two-stage walk because the BVH's job is to produce hits, and "hit" should mean triangle hit by definition (not "AABB candidate"). The user's question made this obvious. Cost: BVH leaf grows from 1 instruction (write AABB tmin) to ~20 (vertex lookup + Möller–Trumbore) per leaf — but only leaves that pass the AABB filter get the test, so the per-query work scales with hit count, not tree size.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **34/34** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Rotate pack-roundtrip slice (FR-004 follow-up).** Now in its 5th carry-over. Promote next — needs Planner design call about whether `Simulator::initialize()` should auto-call `applyPendingMaterials()`.
- **CM-008 production-side fix** — `BroadPhase::build`'s Float-mesh skip robust against scene-swap-at-same-count cases. Theoretical concern in v1.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 + D-024 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
