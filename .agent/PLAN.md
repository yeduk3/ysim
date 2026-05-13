# PLAN — BDD-006 BLOCK fix-turn (Estimator turn 32) — `feat/bdd-006-behavior-assignment-ui`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 32 returned **BLOCK + WARNING + 0 NOTE** on the in-flight BDD-006 slice.

- **BLOCK** — `src/main.cpp:5402-5423` Inspector save path serializes `mesh.behaviorType` via `behaviorTypeName(btype)` which emits `"Rigid"` for Rigid-tagged meshes; `include/scene_format.hpp:283-396` `behaviorFromJson` rejects `"Rigid"` as reserved-not-shipped. Result: user switches to Rigid → saves scene → JSON written → reload fails with `behavior 'Rigid' not available in this build (objects[N])`. Round-trip broken; the user's behavior choice is silently unloadable.
- **WARNING** — `Simulator::changeBehavior` (`src/main.cpp:4920-4967`) mutates `mesh.behaviorType` immediately, but the broad-phase caches `objTrees[i].objBehavior` (`src/main.cpp:3297, 3637`) and `shBroadPhase.meshBehaviors[i]` (`src/main.cpp:2189, 2336-2340`) are stale until the next full rebuild. Result: Float↔Cloth switches keep the old behavior tag in the broad-phase collision-filter for one frame; the `if (tree.objBehavior == Float) continue;` guards at `src/main.cpp:4309/4328` over-skip (new Cloth seen as Float) or under-skip (former Cloth still treated as Cloth) until the next `BroadPhase::build()` / `rebuildMeshKinds()`.

This fix-turn stays on the SAME branch (`feat/bdd-006-behavior-assignment-ui`) per BLOCK-fix-turn cadence (GENERATOR.md / PLANNER.md step 10). Commit prefix `fix:`. No new D-NNN — both items get an addendum to D-036, paralleling D-035's turn-30 addendum shape. Working on isolation worktree `fixturn-bdd-006` (branched off `037d83f` with the slice's uncommitted state replayed via patch).

## Goal

Close turn-32 BLOCK (Rigid save→load round-trip) and turn-32 WARNING (cached broad-phase behavior arrays go stale on `changeBehavior`) in the same fix-turn. Keep the 55/55 self-test baseline intact and add 2 new pass clauses (57/57 expected).

## Scope

**Design call (1) — BLOCK closure: option (b) "load-side accept Rigid" over option (a) "save-side downgrade" or option (c) "save-side skip."**

User brief enumerated three candidates:

- **(a) Save-side downgrade Rigid → Float.** Cheapest (one line in `encodeOne`'s `behaviorTypeName` call site), but silently demotes the user's Rigid choice on save. The user explicitly clicked "Rigid" in the inspector; persisting Float is silent data-loss UX. Forward-compatibility cost: B-3 has to revert this when Rigid dispatch lands.
- **(b) Load-side accept Rigid.** Narrows `isReservedBehavior` from `{Rigid, Elastic, Fluid, Generator}` to `{Elastic, Fluid, Generator}`; widens `isKnownBehavior` to include `"Rigid"`; adds a Rigid branch in `loadScene` (tag-set only, no params to read — matches D-036's "Rigid tag-set only" invariant). Symmetric with `Simulator::changeBehavior` (which already accepts Rigid). Round-trip preserved. Forward-compatible: B-3 widens dispatch, not persistence — this change stays correct.
- **(c) Save-side skip with warning.** Lose the mesh entirely on save. Strictly worse than (a). Rejected.

Picking **(b)**: it's the only option that preserves the round-trip a user expects from a save/load cycle. The "reserved-not-shipped" intent of the original list was "behaviors with no runtime support" — after D-036, Rigid HAS runtime support (tag-set only); the list-membership is stale.

**Design call (2) — WARNING closure: in-place cache-sync inside `Simulator::changeBehavior`.**

Two cached behavior arrays read by the broad-phase:

- **`collisionPipeline.broadPhase.objTrees[idx].objBehavior`** (`src/main.cpp:3297`) — populated during `objTrees[i].build()` at line 3637 from `mesh->behaviorType`. The skip-rebuild gate at line 4234-4236 (`objTrees[i].objBehavior == Float && lifetimeId matches`) bypasses re-population unless the lifetimeId changes. In-place `changeBehavior` does NOT touch lifetimeId (per D-026 invariant), so the gate stays "skip" and `objBehavior` stays stale.
- **`shBroadPhase.meshBehaviors[idx]`** (`src/main.cpp:2189`) — populated by `SpatialHashing::rebuildMeshKinds()` which short-circuits if `meshBehaviors.size == numMeshes`. After `changeBehavior`, size matches numMeshes → short-circuit → the GPU mirror stays stale.

Fix: inside `changeBehavior`, after mutating `mesh->behaviorType`, write the new behavior directly into both caches if they're allocated. Guards handle the "caches not yet allocated" edge (before `initialize()`); in that case the next `rebuildMeshKinds()` / `build()` pass picks up the correct value from `meshes[i].behaviorType`.

**NEW symbols this fix-turn adds**: none. Both items are surgical fixes inside existing functions / existing reserved-list strings.

**MODIFIED symbols in place** (this is a `fix:` fix-turn; modification language is permitted):

- `include/scene_format.hpp::isReservedBehavior` — narrow membership (`Rigid` out).
- `include/scene_format.hpp::isKnownBehavior` — widen membership (`Rigid` in).
- `src/main.cpp` `loadScene` body (~5483-5505) — add a `Rigid` branch that sets `btype = BehaviorType::Rigid; bparams = FloatBehaviorParams<PR>{}` (no params read; Rigid's params are empty per D-036; FloatBehaviorParams placeholder satisfies the variant).
- `src/main.cpp` `Simulator::changeBehavior` (~4920-4969) — append cache-sync writes after each accept case's tag/param mutation. Lambda-ize to avoid 4× duplication.
- `test/scene_io_test.cpp` BDD-016 `"reject reserved-but-not-shipped behavior type"` case (~237-255) — replace the `"Rigid"` example with `"Elastic"` (still-reserved). Test intent unchanged: validate that reserved-not-shipped behaviors are rejected on load.
- `docs/DECISIONS.md` D-036 — append a turn-32 fix-turn addendum paragraph documenting (i) persistence symmetry for Rigid, (ii) cache-sync invariant in changeBehavior.
- `docs/TEST_MATRIX.md` BDD-006 row — append the 2 new pass labels.
- `docs/roles/PLANNER.md` Standing constraints — narrow BDD-006-RIGID-DISPATCH-PARKED entry: persistence is now mechanized; only the integrator dispatch stays parked.

**PRESERVED symbols** (this fix-turn MUST NOT modify):

- `Simulator::changeBehavior`'s switch shape — same 4 accept cases + 3 reject cases + default. Only the per-case body grows by a single lambda invocation.
- `behaviorFromJson`, `objectFromJson`, `fromJson`, `loadScene` outer structure — only the reserved-list strings + the new Rigid branch.
- D-036's main entry text — addendum is appended; original paragraphs unchanged.
- `Simulator::initialize`'s post-pack flow — unchanged; cache-sync is changeBehavior's responsibility, not initialize's.
- D-026's `builtForLifetimeId` invariant — unchanged; in-place behavior switch still does NOT touch lifetimeId.
- Block 29's 4 existing clauses — unchanged; the 2 new clauses are clauses 5 and 6.
- All other Blocks (1–28) — unchanged.
- All other `scene_format.hpp` helpers, all other doctest cases — unchanged.
- `MeshInspectorTarget` shape — unchanged.
- `mesh_inspector_gui.cpp` Behavior combo — unchanged.

## Non-goals

- **NO B-3 work.** Rigid dispatch stays parked under BDD-006-RIGID-DISPATCH-PARKED. After this fix-turn the standing-constraint scope shrinks (persistence done) but the integrator-dispatch gap stays open until B-3.
- **NO new D-NNN.** D-036 addendum captures the fix-turn — both items enforce D-036's stated invariants (persistence symmetry; runtime cache consistency) that the original D-036 entry under-specified.
- **NO change to changeBehavior's switch shape or rejection set.** Reserved-not-shipped enum values (Elastic / Fluid / Generator) still get rejected at the setter level.
- **NO loosening of the reserved-not-shipped widget hiding.** The inspector combo still shows only 4 entries (Float / TriangularCloth / FastGridCloth / Rigid).
- **NO change to params shape for Rigid on disk.** Rigid serializes with `params: {}` (the existing `o.behavior.params = nlohmann::json::object()` initial value at line 5403 stays; the `std::visit` else-branch for non-Cloth / non-FastGridCloth variants leaves params empty by design). Load reads no params for Rigid — `FloatBehaviorParams<PR>{}` placeholder.
- **NO Block-26 (BDD-018) extension.** The cache-sync invariant is changeBehavior-specific and doesn't change BDD-018's live-edit assertion shape.
- **NO new BDD/FR.**
- **NO CM-NNN.** This is enforcement of an existing decision (D-036), not a new mistake-pattern.

## Spec substitution

No new substitutions. The standing **BDD-006-RIGID-DISPATCH-PARKED** entry in PLANNER.md narrows slightly:

> **Was**: "BDD-006 mechanized for Float / TriangularCloth / FastGridCloth; the Rigid dropdown entry exists and `Simulator::changeBehavior(id, Rigid)` sets the tag, but `applyEnvironmentForces` doesn't zero external forces for Rigid (gravity accumulates) and the integrator's `if (behaviorType == Float) continue;` line at `src/main.cpp:~5084` means Rigid-tagged meshes run cloth-style integration. Rigid bodies move like soft bodies until slice B-3 wires `IRigidPhysicsBackend`'s Bullet impl into the simulator's dispatch. The 'next sim step dispatches through rigid pipeline' clause from BDD-006's 'Then' stays parked. Retires when slice B-3 lands."
>
> **Becomes**: "BDD-006 mechanized for Float / TriangularCloth / FastGridCloth / Rigid runtime tag-set AND Rigid persistence (save/load round-trip preserves Rigid tag — fix-turn turn-32, 2026-05-14). Only the integrator dispatch stays parked: `applyEnvironmentForces` accumulates gravity into Rigid-tagged meshes and the integrator runs cloth-style integration on them until slice B-3 wires `IRigidPhysicsBackend`'s Bullet impl. Retires when slice B-3 lands."

## Todo

1. **Branch hygiene.** Working on isolation worktree `fixturn-bdd-006` (branched off `037d83f` with slice WIP patched in). Commit prefix `fix:` per BLOCK-fix-turn cadence; the fix-turn's content + the original slice's content land as a single `fix:` commit (matches the D-035 turn-30 precedent — commit `1f21335 fix: inspector rotation ergonomics + axis-angle antipodal canonicalization (D-035)` rolled the original slice + the BLOCK fix-turn together). The `/slice` close-out then merges to main via `--ff-only` with the `chore: estimator turn N` commit alongside.
2. **`include/scene_format.hpp` reserved-list edits**:
    - Narrow `isReservedBehavior`: drop `"Rigid"` — final list is `t == "Elastic" || t == "Fluid" || t == "Generator"`.
    - Widen `isKnownBehavior`: add `"Rigid"` — final list is `t == "Float" || t == "TriangularCloth" || t == "FastGridCloth" || t == "Rigid"`.
3. **`src/main.cpp` loadScene Rigid branch** (~5483-5505): add `else if (o.behavior.type == "Rigid")` between the FastGridCloth branch and the closing fall-through. Body:
    ```cpp
    } else if (o.behavior.type == "Rigid") {
        btype = BehaviorType::Rigid;
        // BDD-006-RIGID-DISPATCH-PARKED: Rigid is tag-set only until
        // slice B-3 wires IRigidPhysicsBackend. No params to read;
        // FloatBehaviorParams placeholder satisfies the variant.
        bparams = FloatBehaviorParams<PR>{};
    }
    ```
4. **`src/main.cpp` Simulator::changeBehavior cache-sync** (~4920-4969): after each accept-case's `mesh->behaviorType = …; mesh->behaviorParams = …;` assignments, call a small lambda that writes the new behavior into both caches:
    ```cpp
    auto syncBroadPhaseCaches = [&](BehaviorType bt) {
        if (Scene<BE,PR>::meshes.empty()) return;
        Index idx = (Index)(mesh - &Scene<BE,PR>::meshes[0]);
        if (idx < 0 || idx >= (Index)Scene<BE,PR>::meshes.size()) return;
        if (idx < (Index)collisionPipeline.broadPhase.objTrees.size()) {
            collisionPipeline.broadPhase.objTrees[idx].objBehavior = bt;
        }
        if (shBroadPhase.meshBehaviors.ptr
            && idx < (Index)shBroadPhase.meshBehaviors.size) {
            shBroadPhase.meshBehaviors[idx] = (uint32_t)bt;
        }
    };
    ```
    Call `syncBroadPhaseCaches(newType);` immediately before each accept case's `return true;` (Float, TriangularCloth, FastGridCloth, Rigid). Rejection cases (Elastic / Fluid / Generator / not-found) skip the sync since `mesh.behaviorType` is unchanged.
5. **`test/scene_io_test.cpp` BDD-016 example swap**: change the `"behavior": {"type":"Rigid", ...}` literal to `"type":"Elastic"`; change the error-message find check from `r.error.message.find("Rigid")` to `r.error.message.find("Elastic")`. Test intent (reserved-not-shipped rejection on load) preserved.
6. **New self-test clause 5 — Rigid save→load round-trip** (added to Block 29 at the end, after clause 4):
    - resetScene. `addCube` (Float). `initialize`. `changeBehavior(0, Rigid)`. Assert `mesh.behaviorType == Rigid`.
    - Round-trip via the in-memory `SceneSnapshot` path (no temp file required): build the snapshot with `Scene<BE,PR>::toSnapshot()` (or whichever existing API the Generator finds — must be the same path the save button uses). Serialize → `scene_format::toJson(snap)` → `scene_format::fromJson(json)` → assert `r.ok && r.value.objects[0].behavior.type == "Rigid"`.
    - Then exercise the runtime path: feed the round-tripped JSON back through `loadScene` (from a temp file in `$TMPDIR`, or via direct `scene_format::fromJson` followed by manual decode of the resulting `Object` if temp-file machinery isn't readily available in the harness) and assert the post-load `Scene::meshes[0].behaviorType == BehaviorType::Rigid`.
    - Pass label: `BDD-006 / Rigid round-trip through saveScene/loadScene preserves the Rigid tag (D-036 addendum, turn-32 fix-turn)`.
7. **New self-test clause 6 — changeBehavior cache-sync**:
    - resetScene. `addCube` (Float). `initialize`. Pump 1 frame so caches are populated (`objTrees[0].objBehavior` and `meshBehaviors[0]` set from the Float-tagged mesh).
    - Snapshot pre-state: `objTrees[0].objBehavior == BehaviorType::Float` and `meshBehaviors[0] == (uint32_t)BehaviorType::Float`. (Skip the meshBehaviors check if `meshBehaviors.ptr == nullptr` after 1 frame — the spatial-hashing path may not have allocated; Generator confirms which sub-path runs in the harness's default-config sim and gates the assertion on `.ptr` presence.)
    - `changeBehavior(0, TriangularCloth)`. **Immediately** (BEFORE any further `sim.step()` / `BroadPhase::build()` / `rebuildMeshKinds()` call): assert `objTrees[0].objBehavior == BehaviorType::TriangularCloth` and (when allocated) `meshBehaviors[0] == (uint32_t)BehaviorType::TriangularCloth`.
    - Pass label: `BDD-006 / changeBehavior immediately syncs broad-phase cached behavior arrays (D-036 addendum, turn-32 fix-turn)`.
8. **Bug-probes** (each must FAIL after the listed revert; restore after):
    - **(a) scene_format reserved-list revert**: re-add `"Rigid"` to `isReservedBehavior` → clause 5 FAILs at `scene_format::fromJson` with `behavior 'Rigid' not available in this build`. Restore.
    - **(b) loadScene Rigid-branch revert**: comment out the new `else if (o.behavior.type == "Rigid")` branch → clause 5's runtime-path sub-assertion FAILs (loaded `btype == Float`, expected `Rigid`). Restore.
    - **(c) Cache-sync revert**: comment out the `syncBroadPhaseCaches` call inside `changeBehavior`'s TriangularCloth case → clause 6 FAILs (`objBehavior=Float, expected=TriangularCloth`). Restore.
    - **(d) Sanity for the test-update**: temporarily restore the BDD-016 case's literal to `"Rigid"` without reverting steps 2/3 → the doctest case FAILs because `r.ok` is now true (Rigid is no longer reserved). Confirms the test-update is load-bearing. Restore.
9. **Build + verify deterministic.** `cmake --build build` then `./src/ysim --self-test` from `build/` 5 times in a row; expect `57/57 PASS` every time (55 prior + 2 fix-turn clauses).
10. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged. BDD-016 stays green with the Elastic substitution.
11. **Append D-036 turn-32 fix-turn addendum** to `docs/DECISIONS.md`. Shape: paragraph appended AFTER D-036's existing body, prefixed with `**Turn-32 fix-turn addendum (2026-05-14)**:` documenting (i) persistence-side symmetry (Rigid moved out of reserved-not-shipped at scene_format layer + new loadScene Rigid branch; round-trip preserved), (ii) cache-sync invariant (changeBehavior writes `objTrees[idx].objBehavior` + `shBroadPhase.meshBehaviors[idx]` directly; the "caches not yet allocated" edge is gated), (iii) bug-probe shapes for both (revert reserved-narrowing → load fails; revert cache-sync → broad-phase filter lags by one frame). Reference: Estimator turn 32 (`.agent/ESTIMATION.md`).
12. **Update `docs/TEST_MATRIX.md` BDD-006 row** test-address column: append `+ clause 5 (Rigid round-trip) + clause 6 (cache-sync)`. Status stays `pass`.
13. **Update `docs/roles/PLANNER.md` Standing constraints — BDD-006-RIGID-DISPATCH-PARKED entry**: replace the existing paragraph with the **Becomes** text from the Spec substitution section above (persistence done; only dispatch parked).
14. **Update `.agent/CURRENT_WORK.md`**: file in flight (none — fix-turn complete); how far (scene_format reserved-list narrowed + loadScene Rigid branch + cache-sync in changeBehavior + 2 new self-test clauses + D-036 addendum); what's tested (57/57 PASS deterministic + 4 bug-probes verified); what's next (Estimator turn 33 review).
15. **Update `.agent/RESUME.md`**: must-remember (Rigid persistence symmetric across save/load; cache-sync invariant in changeBehavior; D-036 addendum shape); last decisions + why (option B over A/C); next step (Estimator turn 33).
16. **Manual GUI test (user-driven, post-slice)**: launch `./build/src/ysim`. (1) Create cube. (2) Inspector → Behavior → Rigid. (3) Save scene to disk. (4) Load scene back. (5) Confirm the cube's behavior tag is still Rigid in the inspector after reload. (6) Bonus: switch through Float→Cloth on a freshly-loaded scene; the collision filter should react immediately (no one-frame lag) — visually this surfaces as the cube starting to fall in the same frame the Cloth tag is selected, not the next frame.

## Course corrections

- **`feedback_make_means_add_new` rule.** This is a `fix:` fix-turn, not a `make/create` slice. Modification language is permitted because the user's brief says "두 항목을 같은 슬라이스의 fix-turn으로 닫는다." The reserved-list edit + loadScene Rigid branch + cache-sync writes are surgical fixes inside existing surfaces. NO new parallel symbol required.
- **D-026 lifetimeId invariant**: still holds. In-place `changeBehavior` does NOT bump lifetimeId; the cache-sync writes `objBehavior` directly without touching `builtForLifetimeId`. Subsequent `BroadPhase::build()` calls still hit the skip-rebuild gate as intended.
- **D-036 invariant restated**: Rigid is tag-set only. Persistence symmetry is now mechanized at the scene_format layer; the integrator-dispatch gap (`applyEnvironmentForces` accumulates gravity into Rigid; integrator runs cloth-style step) stays parked under BDD-006-RIGID-DISPATCH-PARKED until B-3.
- **PARALLEL-IMPL-LOCKSTEP constraint**: does NOT apply this turn. No conversion math (D-035 family) is touched; the changes are persistence + cache writes, neither of which has a duplicated implementation in `MeshInspectorWindow.hpp`.
- **CM-012 utility-helper-exit trap**: `Simulator::changeBehavior` already returns `bool` (per D-036 + PLANNER §10); the cache-sync additions don't introduce any `exit`/`abort` paths. Caller (`MeshInspectorTarget::on_behavior_change`) still decides on widget reaction.
- **PROBE-COVERAGE-EDGES per PLANNER step 9**: this fix-turn's edges:
    - **Empty-caches edge** (caches not yet allocated before `initialize()`): step 4's guards cover it; clause 6 places `initialize` BEFORE the cache-sync assertion so the caches exist. Generator notes the gating-on-.ptr pattern when `meshBehaviors` isn't allocated by default config.
    - **Reserved-other-than-Rigid edge**: the reserved-list narrowing only drops Rigid; Elastic/Fluid/Generator stay rejected. BDD-016's updated case with Elastic exercises this; no separate harness clause needed.
    - **Rigid-to-Rigid no-op edge**: `changeBehavior(id, Rigid)` when already Rigid. The switch case body runs unconditionally; cache-sync writes are idempotent (writing the same value). No special handling needed. Skip.

## Expected metrics

- Self-test count: **55 → 57** (Block 29 gains 2 pass clauses).
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest **159/159 + 1120/1120 SUCCESS** unchanged.
- Estimator's Linux Metal-less container: SKIP path returns 0 before Block 29 reaches; both doctest binaries pass with BDD-016's Elastic substitution.
- Expected matrix delta: `BDD-006` stays `pass` (test-address column gains 2 entries).
- Expected DECISIONS.md delta: D-036 addendum paragraph appended; no new D-NNN.
- Expected PLANNER.md delta: Standing-constraints entry BDD-006-RIGID-DISPATCH-PARKED text narrowed.
- Estimator verdict next turn: **NOTE** if implementation is clean; **WARNING** if cache-sync gates don't cover the empty-caches edge or D-036 addendum prose is unclear; **BLOCK** only if either clause's bug-probe is silently masked or the round-trip ergonomic doesn't survive Generator's chosen API path.
