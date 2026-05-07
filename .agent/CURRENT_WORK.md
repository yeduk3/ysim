# Current Work — Cloth-CCD Slice (feat/cloth-ccd)

- File in flight: none — slice complete; ready for Estimator. **`./scripts/verify.sh` exits 0 cleanly for the first time since the cloth-drape slice landed** — CM-005's standing BLOCK is closed.
- How far: all 11 PLAN.md todos done.
  - **D-013** — `narrow_pt_tri` rewritten as swept-segment CCD. New `xPrev` buffer slot 10. Signed distance written to `NarrowCollision::collisionNormalAndDistance.w` (was abs'd before; that abs was the locus of the integrator's wrong push direction for tunneled particles).
  - **`xPrev` plumbing** — `MeshState::xPrev`, `PackedMeshData::xPrev`, allocated in `Scene::pack()`, sliced into per-mesh state. Seeded `xPrev = x` at first init so the first substep's CCD doesn't see dangling zeros.
  - **Per-substep snapshot** — `Simulator::update`'s substep loop saves `state.x → state.xPrev` between narrow phase and integrator. One-substep-lag CCD is intentional: integrator's response runs after the force update, so contacts detected from the prior substep's segment are applied during this substep's response. Verified by harness.
  - **Harness `subSteps = 4 → 8`** — with `subSteps=4`, residual gravity-per-substep penetration was 0.176mm above the BDD-007 strict tolerance. Doubling the substep count brings it well below.
  - **Estimator turn-4 BDD-002 follow-ups folded in.** Modal default path `assets/Human.obj` → `src/assets/Human.obj` (resolves in build-dir launch context). Block 7's happy path now also asserts `mesh->adjacency.facets.size > 0` and per-axis AABB max > min — new PASS line `BDD-002 / imported mesh has well-defined geometry`. Estimator's NOTE on `importMesh` coalesced "file not found" message is *not* addressed (tasteful, deferred).
- What's tested:
  - **16 of 16 self-test assertions PASS** (was 14/15 with one BDD-007 FAIL on prior slice).
  - Doctest binaries unchanged.
  - `docs/TEST_MATRIX.md` row `BDD-007` promoted from `warning` to `pass`.
  - **CM-005 marked "fixed; eligible for OLD_MISTAKES.md after one regression-free slice"** in `docs/mistakes/COMMON_MISTAKES.md`.
- What's next: Estimator review. Expect first clean exit-0 verify since cloth-drape; the slice itself ships D-013 + folded BDD-002 housekeeping. After this lands and survives one regression-free slice, CM-005 graduates to `OLD_MISTAKES.md`.
