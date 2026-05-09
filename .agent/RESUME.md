# Resume — BDD-102 Fix-Turn (CM-007 closed, BDD-102 promoted to pass)

## Must remember

- **Branch:** `feat/bdd-102-determinism` (off `main` at `288b4a1`); fix-turn for Estimator turn-11 BLOCK.
- **D-018 invariant — `mesh.id` is the canonical RNG seed source.** Any future per-mesh randomness in any initializer subtype must (a) add a `seed` field to its params struct, (b) be wired from the same call sites that set `mesh.id` — for new meshes that's `Scene<BE, PR>::numMeshes` pre-call; for loaded scenes that's `o.id`. Avoid global `rand()`, `std::random_device`, `time()`, hash-of-pointer, or anything else that leaks environmental state.
- **No scene-format change for the seed.** `o.id` is already serialized; `MeshGridInitializerParams::seed` is reconstructed from it on load. If a future slice considers adding a `seed` field to the JSON scene format, that's redundant — don't.
- **Strict bit-equality on positions stays.** Block 11 uses `memcmp` per-frame. Loosening to epsilon would mask future ordering bugs (atomic accumulation in narrow_pt_tri, BVH instability, etc.) that BDD-102 is designed to catch. The Estimator's earlier review point #4 was about velocity inclusion — that's addressed by dropping `state.v` from the compare; positions stay strict.
- **`std::uniform_real_distribution` is libstdc++-implementation-defined** — fine for v1's same-binary-same-machine BDD-102 scope, but cross-build determinism is explicitly out of scope per PRD §6.
- **`Scene::numMeshes` is read PRE-call in `addCloth`.** That's the value `addGeneralMesh` will increment (`requestsGeneralMeshes.emplace_back(numMeshes++, ...)`), so it equals the about-to-be-assigned mesh id. Don't move the read after the addGeneralMesh call.
- **Block 11 covers per-frame compare**, not just terminal. Divergence-then-reconvergence cannot mask drift. First-divergent-frame is the diagnostic.

## Last decisions + why

- **D-018** — per-mesh seeded `std::mt19937` from `mesh.id`. Closes CM-007 (graduated). Rejected: hardcoded constant (visually awkward when two cloths share the pattern), persisting post-jiggle state.x (scope creep — would need scene-format bump), harness-only `srand(0)` (doesn't fix production), `std::random_device`/`time()` (the opposite of the goal). The `Scene::numMeshes`-pre-call read for `addCloth` and `o.id` for `loadScene` give save/load reproducibility for free.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 cleanly with **24/24** self-test PASS lines. Expected verdict: NOTE level. The BLOCK from turn 11 is strictly closed (SKIP-suppression replaced by real PASS); the Estimator's earlier 4 code-review points are all folded.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands it adds a fifth initializer subtype; D-015's three-site cascade applies AND D-018's seed-from-mesh-id invariant applies (any rigid-body randomness needs a `seed` field wired from `mesh.id`).
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6. When this lands, BDD-102 mechanization can extend to compare Alembic bytes too (the substitution noted in Block 11's pass label).

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
