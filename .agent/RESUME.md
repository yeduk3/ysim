# Resume — CM-008 production-side fix Slice (D-026 lands; harness workarounds removed)

## Must remember

- **Branch:** `fix/cm-008-broadphase-skip` (off `main` at `7d45c6b`).
- **D-026 is the canonical lifetime-identity invariant.** Any per-mesh BVH cache that wants to skip a rebuild must check `objTrees[i].builtForLifetimeId == scene.meshes[i].lifetimeId` in addition to whatever tag-style match (behavior, shape, etc.) the cache already uses. Tag-only gates permit "different mesh, same tag" to satisfy the gate falsely — that was CM-008's exact failure mode.
- **`mesh.id` and `mesh.lifetimeId` are deliberately distinct.**
  - `mesh.id` (numMeshes-derived, **resets** on `resetScene`) — D-018 RNG seed for cloth jiggle. Don't change.
  - `mesh.lifetimeId` (never-reset monotone counter `Scene::lifetimeMeshCount`) — BVH cache identity. Don't persist (in-memory only).
- **The `GeneralMesh` move constructor explicitly carries `lifetimeId(other.lifetimeId)`.** Without that line, every move-realized mesh would default to `-1` and the gate would force rebuilds. Watch this if you ever hand-write the move constructor again.
- **`Scene::pack` realization site** (~line 1832) sets `meshes[i].lifetimeId = req.lifetimeId;` adjacent to the existing `meshes[i].id = req.id;` line. New initializer subtypes that go through `addGeneralMesh` get this for free; new realization paths must mirror both lines.
- **Harness gotcha removed.** `sim.collisionPipeline.broadPhase.objTrees.clear()` is no longer needed before any block. The 7 sites that had it have been removed; future Generator slices should NOT add new ones — write the block normally and trust the production fix. The role doc `docs/roles/GENERATOR.md`'s stable-harness-gotcha entry about `objTrees.clear()` is now stale and should be cleaned in the next role-doc maintenance pass (out of scope for the Generator's write-set this turn).
- **Block 19 bug-probe** caught 5 FAILs simultaneously (Block 19 + the four blocks whose workarounds were removed). The cluster is the load-bearing proof that the fix is doing real work, not just satisfying its own narrowly-scoped assertion.

## Last decisions + why

- **D-026 — Shape A (lifetimeId field) over Shape B (explicit invalidate API).** Foolproof beats explicit for an internal optimization. Any future scene-reset path (v2 LLM control surface, undo/redo, scripted scene rebuilds) that forgets a Shape B handshake silently re-introduces CM-008. Shape A is structural — the gate-correctness guarantee doesn't depend on caller discipline. Cost: one new field on three structs (`Scene`, `GeneralMesh`, `TRI_LBVH`) + one assignment site + one extra conjunct on the skip. ~8 lines net.
- **CM-008 graduated** to OLD_MISTAKES under "skip optimizations silently inherit prior-iteration identity when slot indices align." Pattern statement covers any future cross-iteration cache (per-mesh constraint lists, render-state slots, GPU resource caches), not just this BVH instance.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **36/36** self-test PASS lines (Block 19 added; previous count was 35). Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md` (CM-008 is **off this list now**):

- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes. Manual-test-only mechanization.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 + D-024 + D-025 + D-026 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.
- **Role-doc maintenance pass** — `docs/roles/GENERATOR.md` stale-harness-gotcha entry about `objTrees.clear()` and the "CM-008 production-side fix" carry-forward note across role docs need a refresh.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
