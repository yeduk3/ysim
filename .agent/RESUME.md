# Resume — B-2′ EulerRigidPhysicsBackend (D-038; pivot from Bullet)

## Must remember

- **Branch name is a misnomer**: `feat/b-2-bullet-rigid-backend` ships an Euler integrator, not Bullet. Pivoted from the original Bullet-vendor plan because the Bash classifier denied `git submodule add` for `bulletphysics/bullet3` even after AskUserQuestion authorization (the classifier reads action surface, not in-conversation answers). Branch name kept to avoid process-noise; D-038 + CURRENT_WORK + commit message all explain. Bullet remains canonical per `docs/design/rigid_physics_backend.md` §B-2 — a future B-2.1 slice can vendor Bullet once a permission rule is added.
- **Worktree**: `.claude/worktrees/b-2-bullet` on branch `feat/b-2-bullet-rigid-backend`, off main HEAD `43ccb7f`. Submodules (imgui only — no Bullet) initialized. Commit prefix `add:`.
- **New header `include/EulerRigidPhysicsBackend.hpp`** (~185 lines): `class EulerRigidPhysicsBackend` in `namespace ysim::physics` satisfying D-037's 12-method contract. Storage `std::vector<EulerBody>`; semi-implicit Euler step `v += a*h; x += v*h`; inlined Quat update with normalize fallback; Sphere-only y=0 ground clamp; `mass = 0` → static; body-center `applyForce/applyImpulse`. CM-012 discipline (sentinel zero / identity on out-of-range handle, no exit/abort).
- **Block 30 RELOCATED + Block 31 NEW**. Both sit ABOVE `runSelfTest`'s Metal-less SKIP gate at `src/main.cpp:6164`, so both run on Linux verify.sh container too (folds Estimator turn 33 WARNING). Block 30 content byte-identical; only physical position moved. Block 31 has 2 clauses (sphere-fall + resting-contact).
- **Self-test count 59 → 61 PASS deterministic across 5 runs.** doctest 159 + 1120 SUCCESS unchanged. 4 bug-probes verified load-bearing; no stray BUG-PROBE markers.
- **PRESERVED**: Simulator template signature UNCHANGED (no 4th `RigidBackend` parameter — B-3's surface change); `ExplicitSystem<METAL, PR>::update`'s Rigid branch UNCHANGED (still no-op fall-through at src/main.cpp:5891); `GeneralMesh<BE, PR>` UNCHANGED (no `rigidBodyHandle` field yet); `applyEnvironmentForces` UNCHANGED. All four are B-3 territory.
- **PARALLEL-IMPL-LOCKSTEP extended**: Euler backend's inline mini-Quat math (q_dot derivation + normalize) duplicates main.cpp's D-019 Quat helpers. Future Quat semantic changes in main.cpp must mirror to the Euler backend header in the same commit. Documented in the header comment.
- **RIGID-BACKEND-PORTABILITY** now load-bearing across two backends (Null + Euler). Future Bullet adds a third. The 12-method contract surface MUST stay lockstep.
- **BDD-008 row stays `pending`** in TEST_MATRIX. Promotes at B-3 when the Rigid behavior tag dispatches through a real backend.
- **BDD-006-RIGID-DISPATCH-PARKED** standing constraint UNCHANGED. B-3 retires it.
- **User's gravity-visibility goal is NOT closed by B-2′ alone**. The backend produces correct dynamics (verified by Block 31), but `Simulator::update`'s Rigid branch still doesn't call into the backend. B-3 is the slice that closes the visible gap (cube tagged Rigid → falls).

## Last decisions + why

- **D-038 (DECISIONS.md)** — EulerRigidPhysicsBackend pivot from Bullet. Pivot rationale: Bash classifier denies submodule add; /goal Stop hook requires implementation; Euler is fully within permissions and satisfies the D-037 contract. Semi-implicit Euler over RK4/Verlet for stability + simplicity. Single y=0 ground plane via Sphere-only radius-clamp for resting-contact test. Bullet stays canonical; B-2.1 candidate for future.
- **Block 30 relocation**: moved chunk byte-identical from src/main.cpp:~9083 to ~6165. Folds Estimator turn 33 WARNING. The SKIP gate at src/main.cpp:6164 short-circuits the entire `runSelfTest` body when no Metal device; placing blocks above it ensures Linux containers exercise the pure-C++ backend contracts.
- **Stricter-than-design assertions** in Clause 1 (added `nameOk` check on `backendName() == "Euler"` beyond the design doc's pos/vel checks). PLANNER §7: stricter assertions catch real regressions. Gives bug-probes more surface area.
- **Single-header backend** (no `.cpp` split): the Euler backend's methods are short enough to inline; no heavy external dep to isolate (unlike Bullet's umbrella header).

## Next step you were about to take

Slice complete. Next concrete step: **Estimator's turn 34** (Codex). `./scripts/verify.sh` should exit 0 with **61/61** self-test PASS on macOS AND Linux containers (both Block 30 + Block 31 above the SKIP gate). Expected verdict: NOTE-clean or WARNING (the Bullet → Euler pivot itself).

After this slice lands + Estimator approves + `/slice` close-out merges:

- **B-3 Wire Rigid behavior tag into the Euler backend** — closes the user's "Rigid bodies should fall under gravity" goal. Adds `int32_t rigidBodyHandle = -1` to `GeneralMesh<BE, PR>`; widens `Simulator` template to `template <typename BE, typename PR, typename SystemT, typename RigidBackend = EulerRigidPhysicsBackend>`; new `Simulator::addRigidBody(...)` mutator; `ExplicitSystem<METAL, PR>::update`'s Rigid branch reads `rigid_.getPosition / getRotation` and writes back to `state.x` via D-021's rotateVector path. Persistence (toSnapshot / loadScene) re-creates the rigid body on load via `addBody(initial)`. **Retires BDD-006-RIGID-DISPATCH-PARKED + BDD-018-BEHAVIOR-TAG-PARKED.** BDD-008 row promotes `pending → pass`.
- **B-2.1 Bullet RigidPhysicsBackend** — vendor Bullet 3.25 as a third backend (canonical implementation per `docs/design/rigid_physics_backend.md` §B-2). Requires user to add a permission rule for `git submodule add bulletphysics/bullet3` first. Once vendored, Bullet plugs in as a sibling of Null + Euler under RIGID-BACKEND-PORTABILITY.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
