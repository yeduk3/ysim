# Resume — Translate-Pack Roundtrip + BDD-019 Slice

## Must remember

- **Branch:** `fix/translate-pack-and-bdd019` (off `main` at `53fe5a8`).
- **D-015 — three-site cascade invariant.** `Simulator::translateObject` write-back, `Scene::pack` pack-time seeding, and `Simulator::toSnapshot` realized-mesh override all `dynamic_cast` over the same four initializer subtypes (`MeshGridInitializer`, `MeshSphereInitializer`, `MeshCubeInitializer`, `MeshFileInitializer`). When a fifth subtype ships (e.g., FR-008 Rigid), **all three** sites need the corresponding case added together. Grep for any one to find the other two.
- **BDD-019 spec-substitution.** TESTS.md says "CSV file is written under `profiles/`"; harness uses `/tmp/ysim_profiler_test.csv` for hygiene. Pass label calls this out (`CSV written under profiles containing history` — wording matches BDD verbatim, but the path is substituted). Estimator should not BLOCK on the path-suffix mismatch; the BDD's load-bearing claim (a real CSV with the recorded history including the broad/narrow collision columns) is fully satisfied.
- **BDD-019 pause invariant.** Production gates `frameProfiler.beginFrame`/`endFrame` on `!simulator.pause` at `main.cpp:6180-6182`. Block 10 clause (c) mechanizes this by NOT calling begin/end on a paused tick and asserting `history.frames().size()` is stable. If a future refactor moves `endFrame()` outside the pause guard, this test will catch it.
- **Block 9 clause (d) is bug-probe-verified.** With the D-015 cascade commented out, the new assertion FAILs (`state.x mean drifted across re-pack`); restoring the cascade flips it to PASS. The test catches the bug it claims to catch.
- **CM-006 stays parked.** That's a `physics.metal::integrate_cloth*` change (gate vn-zero behind `(distance < thickness)`); deserves its own slice with a proper before/after BDD-007 comparison.

## Last decisions + why

- **D-015** — translateObject writes back to initializer center/offset. Closes Estimator turn-7 WARNING (a). Rejected: side-table mirror (duplicates state — initializer already carries the authored center), pack-time prefer-mirror (mirror is gone after `meshes.clear()`), const-init params (large refactor). The `dynamic_cast` cascade is the precedent shape; symmetry across three sites is the invariant.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 cleanly with **23/23** self-test PASS lines (was 19/19; +1 BDD-003 round-trip + 3 BDD-019 = +4). Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **CM-006 vn-zero gate** — gate `physics.metal::integrate_cloth*`'s vn-zero block behind `(distance < thickness)` so the slow-touch band can widen without draining vy off non-penetrating particles. Re-enables `nparams.thickness > 0` properly. Requires a careful before/after BDD-007 comparison since changing the integrator's response touches numerical stability.
- **BDD-102 Determinism mechanization** — extend the harness with two-runs-bit-identical assertion against a saved-scene baseline.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. **Important:** when this lands it adds a fifth initializer subtype and triggers the D-015 three-site cascade update.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
