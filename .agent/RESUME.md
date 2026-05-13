# Resume — B-1 RigidPhysicsBackend contract + Null impl (D-037)

## Must remember

- **Branch / worktree**: `feat/b-1-rigid-physics-backend-contract` in worktree `.claude/worktrees/b-1-rigid-backend`, branched off local `main` HEAD `186578d`. Commit prefix `add:` (new feature). Worktree's `third_party/imgui` submodule was initialized fresh with `git submodule update --init --recursive` so cmake builds the imgui sources from third_party rather than the system path. The `/slice` close-out merges `feat/b-1-rigid-physics-backend-contract` to `main` via `--ff-only` with one `add:` commit + one `chore: estimator turn 33` commit.
- **Three new headers under `include/`**: `Quat.hpp` (cut/paste from src/main.cpp:1554; struct body byte-identical), `RigidPhysicsTypes.hpp` (POD types in `namespace ysim::physics`), `NullRigidPhysicsBackend.hpp` (12-method contract no-op class). All inline in headers (the null backend's bodies are short; future B-2 Bullet impl will likely have a .cpp).
- **Block 30 placed OUTSIDE the Metal-gated section** in `runSelfTest` (~line 9081, immediately after Block 29). Pure C++ contract surface runs on macOS + Linux containers alike. 3 clauses: lifecycle / body-state-query / step-is-noop. Pass labels use `D-037 / ...` prefix.
- **Baseline 56, not 57**. The prior BDD-006 fix-turn merged to `main` reported "57/57" in PROJECT_STATE.md / PLAN.md / CURRENT_WORK.md, but the actual measured PASS count at commit `186578d` is **56**. The discrepancy is an off-by-one counting bug in the prior slice's reported headline (the underlying clauses still PASS). B-1 adds 3 → measured total is **59/59 PASS deterministic** across 5 runs. D-037 entry documents this in its "Self-test baseline note" paragraph.
- **PRESERVED**: Simulator template signature UNCHANGED (no 4th `RigidBackend` parameter — B-3's surface change); `ExplicitSystem<METAL, PR>::update`'s Rigid branch UNCHANGED (still no-op fall-through at src/main.cpp:5891); `GeneralMesh<BE, PR>` UNCHANGED (no `rigidBodyHandle` field yet); `applyEnvironmentForces` UNCHANGED (still accumulates gravity into Rigid-tagged meshes — harmless because the integrator doesn't read it). All four are B-3 surface changes.
- **CM-012 discipline**: NullRigidPhysicsBackend methods contain NO `exit()` / `abort()` / release `assert()`. Out-of-range BodyHandle returns sentinel zeros / identity quat; caller (Simulator at B-3) decides fatality at its layer.
- **New standing constraint RIGID-BACKEND-PORTABILITY**: any contract change must update every backend in the same commit. Recorded in `docs/roles/PLANNER.md` Standing constraints subsection (last bullet, after BDD-102-vs-ALEMBIC-BYTES). Future Estimator turns reference by label.
- **BDD-006-RIGID-DISPATCH-PARKED** standing constraint UNCHANGED. B-1 doesn't retire it; B-3 does. BDD-008 row in TEST_MATRIX stays `pending` (Null backend is kinematic by design; row promotes to `pass` when B-3 lands).
- **All 4 bug-probes verified load-bearing.** (a) `addBody` returns `kInvalidBodyHandle` → Clause 2 `handleOk=0` (cascade). (b) `getPosition` returns zero unconditionally → Clause 2 `posOk=0` + Clause 3 `posInvariantOk=0`. (c) `step()` mutates position by `gravity*h*h` → Clause 3 `posInvariantOk=0` only (Clauses 1+2 still PASS). (d) `getRotation` returns `::Quat{0,0,0,0}` → Clause 2 `rotOk=0` + Clause 3 `rotInvariantOk=0`. All restored. No BUG-PROBE markers remain in src/ or include/.
- **C-1..C-4 FlatBuffers mesh-cache slices deferred indefinitely** per user directive 2026-05-14 (memory: `~/.claude/projects/-Users-gyu-codes-ysim/memory/project_flatbuffers_caching_skipped.md`). Active order: B-1 (this) → B-2 (Bullet) → B-3 (wire Rigid behavior). Alembic export slice (when picked up) goes direct to Alembic library, not via FlatBuffers intermediate.

## Last decisions + why

- **D-037 (DECISIONS.md)** — RigidPhysicsBackend template-based contract + Null impl. Foundation slice; Simulator surface deferred to B-3. Per-body initial snapshot for round-trip semantics over stateless null (latter loses the design doc's example assertion). Three headers under `include/` (split for compile-time isolation; Quat extracted as minimum scope-expansion for `RigidInitial::rotation` by-value storage). CM-012 discipline preserved (no `exit()` in helpers).
- **Quat extraction** — only the aggregate body moves to `include/Quat.hpp`; helpers (`operator*`, `quatNormalize`, `quatFromAxisAngle`, etc. at src/main.cpp:~1562-1700+) stay in main.cpp. Migrating helpers is future cleanup (likely with source-file split slice).
- **Block 30 outside Metal gate** — pure C++ contract surface has no Metal dependency; placing it outside means Estimator's Linux Metal-less container runs Block 30 too (60→60 there, not SKIP-emits-0 for the new clauses). Block 29 stays inside the Metal gate per its existing structure.
- **Stricter-than-design assertions** in Clause 3 — design doc only asserts position invariant; we additionally assert rotation + linear + angular velocity invariants. PLANNER §7: "stricter-than-spec assertions are valuable signal." Gives bug-probe (d) a target.

## Next step you were about to take

Slice complete. Next concrete step: **Estimator's turn 33** (Codex). `./scripts/verify.sh` should exit 0 with **59/59** self-test PASS on macOS AND Linux containers (Block 30 outside Metal gate). Expected verdict: NOTE-clean. Possible NOTE items:

- (i) Block 30 placement (outside Metal gate) — taste-level call; either is defensible.
- (ii) Quat helpers stay in main.cpp — future cleanup with source-file split.
- (iii) `RigidShape::mesh_vertex_data` / `mesh_index_data` as raw `const float*` / `const uint32_t*` — caller-owns-buffer dangling-pointer subtlety. D-037 mentions it; null backend never reads the pointers, but B-2's Bullet backend will snapshot.
- (iv) Block 30 doesn't exercise ConvexMesh / StaticMesh shape types. Null backend ignores shape; B-2 will cover.
- (v) Out-of-range handle behavior covered indirectly (Clause 2's `handleOk=true` is the proxy); no explicit out-of-range clause.
- (vi) Self-test baseline counting note in D-037 (prior "57" was off by one) — informational.

After this slice lands + Estimator approves + `/slice` close-out merges:

- **B-2 Bullet RigidPhysicsBackend impl** — vendor Bullet 3.25 under `include/bullet3/`, replace `NullRigidPhysicsBackend` in Block 30 with `BulletRigidPhysicsBackend`, add 2 dynamics assertions (sphere falls under gravity for one step within 1e-4; sphere + static plane rest at y ≈ radius). 59 → 61 self-test count.
- **B-3 Wire Rigid behavior tag into Bullet backend** — adds `GeneralMesh::rigidBodyHandle` field; widens `Simulator` template with 4th `RigidBackend` parameter; new `Simulator::addRigidBody(...)` mutator; `ExplicitSystem::update`'s Rigid branch reads `rigid_.getPosition / getRotation` and updates `state.x` via D-021's rotateVector path. **Retires BDD-006-RIGID-DISPATCH-PARKED + BDD-018-BEHAVIOR-TAG-PARKED.** BDD-008 row promotes `pending → pass`.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
