# Current Work — BDD-102 Fix-Turn (`feat/cloth-thickness-band` → wait, `feat/bdd-102-determinism`)

- File in flight: none — fix-turn complete, BLOCK closed. **24/24 self-test PASS, deterministic across 5 consecutive runs.** Doctest 159/159 + 1120/1120 green.
- How far: all 13 PLAN todos done.
  - **D-018** — `MeshGridInitializerParams` gained `uint32_t seed = 0` field; `MeshGridInitializer::initialize` uses a local `std::mt19937(params.seed)` with `std::uniform_real_distribution<PR>(0, 1.0/10000.0)` instead of `rand()`. RNG is fully scoped to `initialize()` — no global state leak.
  - **Seed wiring:** `addCloth` reads `Scene<BE, PR>::numMeshes` pre-call (the about-to-be-assigned id) and passes it as the seed. `loadScene` passes `o.id` from the saved scene. Same scene + same mesh ids → bit-identical jiggle. Save/load reproduces jiggle for free; no scene-format change needed.
  - **Block 11 cleanup per Estimator's 4 review points:** (1) SKIP→FAIL — mismatch is now a hard failure; (2) per-frame compare — 30-frame snapshot vectors with first-divergent-frame diagnostic; (3) fail-loud on null buffers — `snapshotPositions` returns false and emits a fail with mesh id; (4) positions-only — dropped state.v from the byte buffer. Strict bit-equality on positions stays per BDD-102 wording.
  - **Pass label changed** to `BDD-102 / two runs produce bit-identical per-frame state.x` (was `…state.x and state.v`); matrix row updated to match.
  - **Bug-probe verified** — temporarily added `state.x[0] += 0.001f` between the two runs; Block 11 FAILed at frame 0 byte 0 with the per-frame diagnostic format (`888 bytes` confirms positions-only). Restored.
  - **CM-007 graduated** to OLD_MISTAKES.md under a new high-level cause "Global RNG state leaks across scene reconstructions". Active list keeps a graduation breadcrumb. Direction-for-similar-problems written for future initializer subtypes that introduce randomness.
- Tests:
  - **24/24 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs of `--self-test`.
  - Doctest binaries unchanged.
  - `BDD-102` matrix row promoted `pending → pass` with test address pointing at Block 11 + D-018.
- Non-goals respected: no scene-format version bump, no harness `srand(0)` workaround, no Alembic-byte compare (substitution stays documented), no epsilon-tolerant comparison, no other matrix rows touched.
- What's next: Estimator review. Expect verdict at NOTE level — BLOCK was strictly the SKIP-suppression; fix-turn closes both the underlying nondeterminism (CM-007) and the four code-review points the Estimator surfaced earlier.
