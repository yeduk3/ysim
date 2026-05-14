# PLAN — B-3 Wire Rigid behavior tag through Simulator — `feat/b-3-wire-rigid-behavior`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 34 (B-2′ EulerRigidPhysicsBackend slice) returned **WARNING** with 0 BLOCK + 1 WARNING + 1 NOTE. WARNING flagged that Block 31 only covers sphere-on-clamp; BDD-008's box-on-plane + angular-velocity-decay are still unmet. NOTE flagged `removeBody` slot-leak + `applyForce == applyImpulse` collapse. Both items are scope-coverage flags, not regressions; B-3 closes some of them indirectly (wiring through the simulator makes the backend live in real scenes).

B-2′ EulerRigidPhysicsBackend merged to `main` via commits `5c745fa add:` + `481e013 chore:`. Self-test count 59 → 61 PASS deterministic on macOS AND Linux.

## Goal

**Close the user's "Rigid bodies should fall under gravity" goal.** B-1 shipped the contract; B-2′ shipped the Euler integrator; B-3 wires them into `Simulator::update` so a Rigid-tagged mesh in the GUI actually falls under gravity. Adds `int32_t rigidBodyHandle = -1` to `GeneralMesh<BE, PR>` + `tinym::vec3 rigidLastBodyPos` for delta-tracking. New idempotent helper `Simulator::ensureRigidBackendBody(meshId)` constructs `RigidInitial` and calls `rigid_.addBody`. `Simulator` holds `ysim::physics::EulerRigidPhysicsBackend rigid_;` (hard-coded type — no template-param widening this slice; future Bullet B-2.1 changes the type in one line). `Simulator::update()` calls `rigid_.step(h, 1)` once per frame, then for each Rigid-tagged mesh with valid handle: `Δpos = backend.getPosition - rigidLastBodyPos`, applied to every vertex of `state.x` + `state.xPrev` + `transformPosition`. `Simulator::changeBehavior(meshId, Rigid)` calls `ensureRigidBackendBody`. `Simulator::initialize` sweeps Rigid meshes via the helper. New Block 32 verifies `addCube + changeBehavior(0, Rigid) + 30 sim.update() → state.x[0].y dropped`. **Retires BDD-006-RIGID-DISPATCH-PARKED + BDD-018-BEHAVIOR-TAG-PARKED.** BDD-008 row promotes `pending → pass`. Self-test count 61 → 62.

## Scope

**Design call (1) — Hard-code `EulerRigidPhysicsBackend rigid_;` (no template widening).** The brief proposed widening `Simulator<BE, PR, SystemT>` → `Simulator<BE, PR, SystemT, RigidBackend = EulerRigidPhysicsBackend>`. Widening blast radius: every `Simulator<...>` instantiation site (production main.cpp, harness, runRefitBench) needs to acknowledge the extra parameter even with a default. Tradeoff:
- **(a) Widen template**: cleanest future swap to Bullet via type-alias at one instantiation. Forces ~10+ instantiation-site touches.
- **(b) Hard-code Euler member**: zero instantiation-site changes. Swap to Bullet requires changing `EulerRigidPhysicsBackend rigid_;` → `BulletRigidPhysicsBackend rigid_;` in one line.

Picking **(b)**. RIGID-BACKEND-PORTABILITY (D-037) is still satisfied (Null + Euler both exist as parallel symbols; contract is shared; future Bullet swap is one-line). Blocks 30+31 still independently exercise the contract surface.

**Design call (2) — `Simulator::ensureRigidBackendBody(meshId)` idempotent helper.** Three triggers converge here:
1. **Lazy via `changeBehavior(meshId, Rigid)`**: Float → Rigid → call helper. Idempotent (no-op if `rigidBodyHandle != kInvalidBodyHandle`).
2. **Sweep via `Simulator::initialize`**: after `applyPendingMaterials()`, iterate Rigid-tagged meshes and call helper. Covers post-load + post-pack-rebuild.
3. **Belt-and-suspenders fallback in `Simulator::update`**: at top of Rigid delta-loop, if handle invalid, call helper. Defensive; should be unreachable if 1+2 hold.

Helper body sketch (full version in Todo step 4):
- Bounds-check meshId + behaviorType.
- Idempotent gate on `mesh.rigidBodyHandle != kInvalidBodyHandle`.
- Build `RigidInitial`: position = mesh.transformPosition, rotation = mesh.rotationQuat, mass = 1.0f, shape = Sphere with radius = inferRigidRadius(mesh).
- `mesh.rigidBodyHandle = rigid_.addBody(init); mesh.rigidLastBodyPos = init.position;`

**Design call (3) — Delta-tracking: how `state.x` follows the rigid body each frame.** Two options:
- **(α) Recompute world vertices from local offsets**: store `local_offset[i] = state.x[i] - initial_body_center` once; each frame `state.x[i] = backend_pos + rotate(local_offset[i], backend_rot)`. Handles rotation. Cost: per-mesh local-offset buffer.
- **(β) Apply Δpos in-place**: `Δpos = backend_now - rigidLastBodyPos`; `state.x[i] += Δpos` for all vertices. Translation-only. No new buffer.

Picking **(β)** for B-3. Future rotation-correct slice can switch to (α). Block 32 ("cube falls") is verified by vertical translation. PLAN documents the limitation; D-039 records it. Sphere is rotationally symmetric anyway; cube doesn't rotate in B-3 scenes (no torque applied).

`state.xPrev` (D-013 CCD snapshot) also gets the same Δpos: `state.xPrev[i] += Δpos` (guarded on `xPrev.ptr` non-null). Keeps swept-CCD consistent. `mesh.transformPosition` also gets Δpos so D-014/D-021/inspector reads stay accurate.

**Design call (4) — `rigid_.step` placement in `Simulator::update`.** Once per outer frame (NOT per substep — `step(h, 1)` already represents one frame of dynamics). Place: AFTER `applyEnvironmentForces()` (line ~5089), BEFORE the substep loop. Delta-loop runs immediately after `rigid_.step` (also before substep loop — Rigid meshes' state.x is updated; subsequent broad-phase uses the new positions).

**Design call (5) — `applyEnvironmentForces` skips Rigid (zero externalForces).** Currently Rigid bodies get mass×gravity accumulated into `externalForces[i]`. The cloth integrator (`ExplicitSystem::update`'s switch) hits `default: break;` for Rigid, so the buffer is unused. Wasted work. Extending the Float skip to cover Rigid:
```cpp
if (mesh.behaviorType == BehaviorType::Float ||
    mesh.behaviorType == BehaviorType::Rigid) {
    std::memset(ext, 0, sizeof(PR) * numPoints * 3);
    continue;
}
```
Retires BDD-006-RIGID-DISPATCH-PARKED's "gravity accumulates into Rigid-tagged meshes" claim explicitly.

**Design call (6) — Block 32 placement: BELOW the Metal-less SKIP gate.** Block 32 needs `Simulator::initialize` which uses Metal (Scene::pack + Metal buffer allocs). Linux container SKIPs after the gate at `src/main.cpp:6164`. Block 32 joins the existing Block 1-29 cluster (Block 32 lands AFTER Block 29 closes, INSIDE the Metal-gated section, BEFORE the failures summary). Linux container SKIPs Block 32 like the rest — acceptable per D-012 + existing pattern.

**Design call (7) — Block 32 shape (single pass clause)**:
1. `resetScene()`; `sim.addCube(tinym::vec3(0, 5, 0), tess=2, size=0.2, mass=1.0)`; `sim.initialize()`.
2. `sim.changeBehavior(0, BehaviorType::Rigid)` — triggers `ensureRigidBackendBody`.
3. Capture `y_initial = mesh.state.x[1]` (first vertex's y), `center_y_initial = mesh.rigidLastBodyPos.y`.
4. `sim.pause = false;` (update() early-returns otherwise).
5. Pump 30 frames: `for (int i = 0; i < 30; ++i) sim.update();`.
6. Read `y_post = mesh.state.x[1]`, `center_y_post = mesh.rigidLastBodyPos.y`.
7. Assert (a) `changed == true`, (b) `mesh.rigidBodyHandle != kInvalidBodyHandle`, (c) `y_post < y_initial - 0.5f` (witness vertex fell at least 0.5m — semi-implicit accumulation over 30 frames gives ~1.27m total fall), (d) `center_y_post < center_y_initial - 0.5f` (backend body center also fell).
8. Pass label: `BDD-008 / cube tagged Rigid falls under gravity in Simulator::update (D-039)`.

**Use `mesh.rigidLastBodyPos.y` to inspect backend state** (since `Simulator::rigid_` is private). The delta-loop updates this field every frame to the backend's reported position; reading it post-pump is the same as querying the backend.

**Design call (8) — Persistence wiring.** `loadScene` already accepts `"Rigid"` (D-036 fix-turn). The slice's sweep inside `Simulator::initialize` (Design call 2 trigger 2) covers persistence too: after a load, `initialize` is called → sweep re-adds all Rigid bodies. No changes needed in `loadScene` itself.

**Design call (9) — Backend reset on `Simulator::initialize`.** `resetScene` clears `Scene::meshes`. The rigid backend's `bodies_` vector doesn't auto-clear. Per-scene clean state: `Simulator::initialize` calls `rigid_.shutdown(); rigid_.initialize(scene.environment.gravity);` then runs the Rigid-mesh sweep. Each scene gets a fresh backend with the correct gravity.

**Design call (10) — `changeBehavior(meshId, Float)` clears the rigid handle.** If the user toggles Rigid → Float, the backend body slot is leaked (Euler's `removeBody` is slot-leak per D-038). But the mesh must STOP following the (now-uncoupled) backend body. Fix: in `changeBehavior`'s Float accept case, clear `mesh.rigidBodyHandle = kInvalidBodyHandle` + `mesh.rigidLastBodyPos = {}`. Subsequent updates skip the mesh in the delta-loop. Backend's `bodies_[old_handle]` continues to drift under gravity invisibly until `Simulator::initialize` resets the backend. Acceptable for B-3.

**Design call (11) — Inspector Float → Rigid path.** `mesh_inspector_gui.cpp`'s on_behavior_change callback calls `Simulator::changeBehavior(id, Rigid)` (already wired in BDD-006 D-036). With this slice's changeBehavior addition, the helper fires automatically. User clicks Rigid → cube starts falling on next sim step. Manual GUI test post-slice.

**NEW symbols this slice adds**:
- `GeneralMesh<BE, PR>::rigidBodyHandle` field (int32_t, default `kInvalidBodyHandle`).
- `GeneralMesh<BE, PR>::rigidLastBodyPos` field (`tinym::vec3`, default `{}`).
- `Simulator<BE, PR, System>::rigid_` private member (`ysim::physics::EulerRigidPhysicsBackend`).
- `Simulator::ensureRigidBackendBody(int meshId)` — idempotent helper.
- `Simulator::inferRigidRadius(const GeneralMesh<BE, PR>&)` — bbox-half heuristic.
- Block 32 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-039 entry.
- `docs/TEST_MATRIX.md` — BDD-008 row promoted `pending → pass`.

**MODIFIED symbols in place**:
- `src/main.cpp` — GeneralMesh gains 2 fields; Simulator gains `rigid_` + helpers; `changeBehavior` Rigid case calls helper; `changeBehavior` Float case clears handle/lastPos; `Simulator::initialize` resets backend + sweeps; `Simulator::update` calls `rigid_.step` + delta-loop; `applyEnvironmentForces` skips Rigid like Float.
- `docs/roles/PLANNER.md` — REMOVE BDD-006-RIGID-DISPATCH-PARKED bullet + REMOVE BDD-018-BEHAVIOR-TAG-PARKED bullet (both retired). RIGID-BACKEND-PORTABILITY stays.

**PRESERVED symbols**:
- `include/RigidPhysicsTypes.hpp` — UNCHANGED.
- `include/NullRigidPhysicsBackend.hpp` — UNCHANGED. Block 30 still exercises.
- `include/EulerRigidPhysicsBackend.hpp` — UNCHANGED. Block 31 still exercises.
- `include/Quat.hpp` — UNCHANGED.
- `ExplicitSystem<METAL, PR>::update`'s switch at `src/main.cpp:5891` — UNCHANGED. Rigid still falls through to `default: break;` (Metal integrator). Rigid integration happens at the outer `Simulator::update` level via the C++ delta-loop — cleaner separation.
- `Simulator<BE, PR, System>` template signature — UNCHANGED. No 4th param (per Design call 1).
- `scene_format.hpp` — UNCHANGED.
- All Metal kernels — UNCHANGED.
- Block 1-31 — UNCHANGED.
- All inspector code — UNCHANGED.
- BehaviorType enum + behaviorTypeName — UNCHANGED.

## Non-goals

- **NO Simulator template widening.** Hard-code Euler.
- **NO rotation-correct vertex update.** Δpos translation only.
- **NO per-mesh mass / friction / restitution API.** Hard-coded mass=1.0.
- **NO shape-specific bodies beyond Sphere.** Cube uses Sphere with derived radius.
- **NO general rigid-vs-rigid collision** or rigid-vs-cloth. Backend only does Sphere-vs-y=0 clamp.
- **NO new BDD/FR.**
- **NO new CM-NNN** unless a discovery forces one.
- **NO C-* FlatBuffers work.**
- **NO Block 1-31 changes.**

## Spec substitution

None this turn. BDD-008's "falls" part is mechanized end-to-end by B-3 (cube under gravity translates downward in `state.x`). The "rests" part (low kinetic energy on a plane) is verified at the BACKEND layer by Block 31; the in-Simulator "rigid mesh on plane comes to rest" would need Box-vs-Plane contact in the backend (Euler's ground clamp is Sphere-only per D-038). **BDD-008 row promotes `pending → pass` based on the "falls" mechanization** + Block 31's "rests" backend coverage. The "rests" in-Simulator integration test is a future-slice candidate (Box-shape support OR Bullet B-2.1).

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED. Null + Euler still parallel.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038 extension) — UNCHANGED.
- **BDD-006-RIGID-DISPATCH-PARKED** — **RETIRED this slice.** Rigid integrator dispatch exists at the outer `Simulator::update` C++ layer; `applyEnvironmentForces` skips Rigid; persistence works via `Simulator::initialize` sweep.
- **BDD-018-BEHAVIOR-TAG-PARKED** — **RETIRED this slice.** Inspector Float → Rigid → cube falls on next sim step via lazy `ensureRigidBackendBody`.
- **BDD-102-vs-ALEMBIC-BYTES** — UNCHANGED.
- **DUPLICATED-INSPECTOR-WIRING** — UNCHANGED.
- **GLFWINIT-NON-REF-COUNTED** — UNCHANGED.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/b-3-rigid-wire` on branch `feat/b-3-wire-rigid-behavior` (branched off main HEAD `481e013`). Submodules initialized. Commit prefix `add:`.

2. **GeneralMesh — add 2 fields** at `src/main.cpp:~1727` (after `externalForces`):
   ```cpp
   // D-039: Rigid backend wiring. Set by Simulator::ensureRigidBackendBody
   // on Float→Rigid transition (changeBehavior) or initialize-time sweep.
   // -1 (kInvalidBodyHandle) means "no backend body yet"; update() skips.
   int32_t rigidBodyHandle = ysim::physics::kInvalidBodyHandle;
   // Cached last backend body position so each frame's Δpos = current - last
   // can be applied to state.x / state.xPrev / transformPosition.
   tinym::vec3 rigidLastBodyPos = {};
   ```

3. **Simulator — add `rigid_` member** at `src/main.cpp:~4685` (after `MeshRenderState renderState;` or in a clearly tagged location):
   ```cpp
   // D-039: rigid physics backend (Euler per B-2′; Bullet B-2.1 swap is
   // one-line type change). RIGID-BACKEND-PORTABILITY (D-037) governs.
   ysim::physics::EulerRigidPhysicsBackend rigid_;
   ```

4. **Add `inferRigidRadius` + `ensureRigidBackendBody`** near `Simulator::changeBehavior`:
   ```cpp
   PR inferRigidRadius(const GeneralMesh<BE, PR>& mesh) const {
       if (mesh.state.x.size == 0) return PR(0.5);
       const Index n = mesh.state.x.size / 3;
       PR xmin = mesh.state.x[0], xmax = mesh.state.x[0];
       PR ymin = mesh.state.x[1], ymax = mesh.state.x[1];
       PR zmin = mesh.state.x[2], zmax = mesh.state.x[2];
       for (Index i = 1; i < n; ++i) {
           xmin = std::min(xmin, mesh.state.x[i*3+0]); xmax = std::max(xmax, mesh.state.x[i*3+0]);
           ymin = std::min(ymin, mesh.state.x[i*3+1]); ymax = std::max(ymax, mesh.state.x[i*3+1]);
           zmin = std::min(zmin, mesh.state.x[i*3+2]); zmax = std::max(zmax, mesh.state.x[i*3+2]);
       }
       const PR dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
       return PR(0.5) * std::max(dx, std::max(dy, dz));
   }

   void ensureRigidBackendBody(int meshId) {
       if (meshId < 0 || meshId >= (int)Scene<BE, PR>::meshes.size()) return;
       auto& mesh = Scene<BE, PR>::meshes[meshId];
       if (mesh.behaviorType != BehaviorType::Rigid) return;
       if (mesh.rigidBodyHandle != ysim::physics::kInvalidBodyHandle) return;

       ysim::physics::RigidInitial init{};
       init.position = mesh.transformPosition;
       init.rotation = mesh.rotationQuat;
       init.mass = 1.0f;
       init.shape.type = ysim::physics::RigidShapeType::Sphere;
       init.shape.half_extents = tinym::vec3((float)inferRigidRadius(mesh), 0.0f, 0.0f);

       mesh.rigidBodyHandle = rigid_.addBody(init);
       mesh.rigidLastBodyPos = init.position;
   }
   ```

5. **Wire `ensureRigidBackendBody` into `changeBehavior` Rigid accept case** at `src/main.cpp:~4980`. Currently:
   ```cpp
   case BehaviorType::Rigid:
       mesh->behaviorType = BehaviorType::Rigid;
       // bparams left as previous (Float/Cloth — variant alternative; D-036)
       syncBroadPhaseCaches(BehaviorType::Rigid);
       ensureRigidBackendBody(meshId);     // <-- ADD THIS
       return true;
   ```

6. **Wire handle-clear into `changeBehavior` Float accept case**. When switching back to Float, clear the rigid linkage on the mesh side (Euler backend slot stays — slot-leak per D-038, acceptable):
   ```cpp
   case BehaviorType::Float:
       mesh->behaviorType = BehaviorType::Float;
       mesh->behaviorParams = FloatBehaviorParams<PR>{};
       mesh->rigidBodyHandle = ysim::physics::kInvalidBodyHandle;     // <-- ADD
       mesh->rigidLastBodyPos = tinym::vec3{};                         // <-- ADD
       syncBroadPhaseCaches(BehaviorType::Float);
       return true;
   ```

7. **Wire backend reset + Rigid sweep into `Simulator::initialize`** — at the END of the existing initialize body, after `applyPendingMaterials()`:
   ```cpp
   // D-039: rigid backend reset + sweep-add for currently-tagged Rigid meshes.
   rigid_.shutdown();
   rigid_.initialize(tinym::vec3(
       (float)scene.environment.gravity.x,
       (float)scene.environment.gravity.y,
       (float)scene.environment.gravity.z));
   // Clear stale handles before idempotent sweep can re-add.
   for (auto& m : Scene<BE, PR>::meshes) {
       m.rigidBodyHandle = ysim::physics::kInvalidBodyHandle;
       m.rigidLastBodyPos = tinym::vec3{};
   }
   for (int i = 0; i < (int)Scene<BE, PR>::meshes.size(); ++i) {
       ensureRigidBackendBody(i);
   }
   ```

8. **Skip Rigid in `applyEnvironmentForces`** at `src/main.cpp:~5035`. Extend the existing Float skip to include Rigid:
   ```cpp
   if (mesh.behaviorType == BehaviorType::Float ||
       mesh.behaviorType == BehaviorType::Rigid) {
       // D-039: Rigid is integrated by the rigid backend; the cloth-side
       // externalForces buffer is unused. Zero matches Float's pattern +
       // retires BDD-006-RIGID-DISPATCH-PARKED's gravity-accumulation claim.
       std::memset(ext, 0, sizeof(PR) * numPoints * 3);
       continue;
   }
   ```

9. **Add `rigid_.step` + delta-loop into `Simulator::update`** between `applyEnvironmentForces()` (line ~5089) and the substep `for`-loop:
   ```cpp
   applyEnvironmentForces();

   // D-039: step the rigid backend once per outer ysim frame.
   rigid_.step(static_cast<float>(system.h), 1);

   // D-039: apply Δpos = backend.getPosition(handle) - rigidLastBodyPos to
   // every vertex of state.x AND state.xPrev for each Rigid-tagged mesh
   // with a valid handle. Translation only (rotation propagation deferred
   // to a future slice). Belt-and-suspenders: lazily re-add if handle null.
   for (int mi = 0; mi < (int)Scene<BE, PR>::meshes.size(); ++mi) {
       auto& m = Scene<BE, PR>::meshes[mi];
       if (m.behaviorType != BehaviorType::Rigid) continue;
       if (m.rigidBodyHandle == ysim::physics::kInvalidBodyHandle) {
           ensureRigidBackendBody(mi);
           if (m.rigidBodyHandle == ysim::physics::kInvalidBodyHandle) continue;
       }
       const tinym::vec3 now = rigid_.getPosition(m.rigidBodyHandle);
       const tinym::vec3 dp{
           now.x - m.rigidLastBodyPos.x,
           now.y - m.rigidLastBodyPos.y,
           now.z - m.rigidLastBodyPos.z
       };
       const Index nVerts = m.state.x.size / 3;
       for (Index i = 0; i < nVerts; ++i) {
           m.state.x[i*3+0] += static_cast<PR>(dp.x);
           m.state.x[i*3+1] += static_cast<PR>(dp.y);
           m.state.x[i*3+2] += static_cast<PR>(dp.z);
           if (m.state.xPrev.ptr) {
               m.state.xPrev[i*3+0] += static_cast<PR>(dp.x);
               m.state.xPrev[i*3+1] += static_cast<PR>(dp.y);
               m.state.xPrev[i*3+2] += static_cast<PR>(dp.z);
           }
       }
       m.transformPosition.x += dp.x;
       m.transformPosition.y += dp.y;
       m.transformPosition.z += dp.z;
       m.rigidLastBodyPos = now;
   }
   ```
   **Generator note**: `state.x.ptr` is `PR*` on the CPU side of `MemoryBlock<METAL, PR>`. Direct subscript writes write to host-visible memory (the MemoryBlock is shared-storage on Apple Silicon). Confirm by reading how Block 11 (BDD-102) reads state.x and how `translateObject` (D-014) writes it — both patterns work with direct subscript. If a sync is needed (rare on shared storage), Generator inserts the appropriate `metalMemoryBarrierAtBuffer` call.

10. **New Block 32** in `runSelfTest` — inserted AFTER Block 29 closes (line ~9081 in current main; new line ~9082+ after relocations), BEFORE the failure-summary tail. Body:
    ```cpp
    // ---- Block 32: D-039 — Rigid behavior wires through Simulator. ---------
    // BDD-008's "falls" half: addCube + changeBehavior(0, Rigid) triggers
    // ensureRigidBackendBody; pumping update() steps the backend + applies
    // Δpos to state.x. Closes the user's "Rigid bodies should fall under
    // gravity" goal end-to-end (B-1 contract + B-2′ Euler + B-3 wiring).
    {
        resetScene();
        // Cube at y=5, tess=2 (8 vertices), size=0.2, mass=1.
        sim.addCube(tinym::vec3(0.0f, 5.0f, 0.0f), 2, 0.2f, 1.0f);
        sim.initialize();

        // Snapshot pre-state. mesh.state.x[1] is vertex 0's y component.
        auto& m = sim.scene.meshes[0];
        PR y_initial = m.state.x[1];

        // Switch to Rigid — ensureRigidBackendBody fires.
        bool changed = sim.changeBehavior(0, BehaviorType::Rigid);
        bool handleOk = (m.rigidBodyHandle != ysim::physics::kInvalidBodyHandle);
        PR center_y_initial = m.rigidLastBodyPos.y;

        // Pump 30 frames. With g=-9.81 and h=1/60 semi-implicit:
        // Δy_total ≈ -g*h^2*N*(N+1)/2 ≈ -1.27 m at N=30.
        sim.pause = false;
        for (int i = 0; i < 30; ++i) sim.update();

        PR y_post = m.state.x[1];
        PR center_y_post = m.rigidLastBodyPos.y;

        bool vertexFellOk = (y_post < y_initial - PR(0.5));
        bool centerFellOk = (center_y_post < center_y_initial - PR(0.5));

        if (changed && handleOk && vertexFellOk && centerFellOk) {
            pass("BDD-008 / cube tagged Rigid falls under gravity in Simulator::update (D-039)");
        } else {
            fail("BDD-008 / cube tagged Rigid falls under gravity in Simulator::update (D-039)",
                 "changed=" + std::to_string((int)changed)
                 + " handleOk=" + std::to_string((int)handleOk)
                 + " vertexFellOk=" + std::to_string((int)vertexFellOk)
                 + " centerFellOk=" + std::to_string((int)centerFellOk)
                 + " y_initial=" + std::to_string(y_initial)
                 + " y_post=" + std::to_string(y_post)
                 + " center_y_initial=" + std::to_string(center_y_initial)
                 + " center_y_post=" + std::to_string(center_y_post));
        }
    }
    ```
    **Note**: Block 32 needs `sim.pause = false` because the existing Block 1-29 set `sim.pause = true` (the harness default) and call `pumpFrames(sim, N)` which internally calls `sim.update`. If `sim.update` early-returns on `pause==true`, no progress. Generator confirms by reading the existing pump pattern.

11. **Build + verify deterministic.** `cmake -B build && cmake --build build && cd build && for i in 1..5; do ./src/ysim --self-test; done` — expect **62/62 PASS** each time.

12. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

13. **Bug-probes** (each FAIL after revert; restore after):
    - **(a) Remove `rigid_.step()` call**: Block 32 FAIL (`vertexFellOk=0 centerFellOk=0`, dy=0). Restore.
    - **(b) Remove `ensureRigidBackendBody(meshId)` call from changeBehavior's Rigid case**: Block 32 FAIL (`handleOk=0`). Restore.
    - **(c) Remove the delta-loop** (the `for (auto& m : meshes)` block updating state.x): Block 32 FAIL (`vertexFellOk=0` because state.x doesn't follow; `centerFellOk=0` because `rigidLastBodyPos` never updates either). Restore. **Stricter probe**: keep the delta-loop's `rigidLastBodyPos` update but remove the state.x write loop — `centerFellOk=1` (lastBodyPos updates) but `vertexFellOk=0` (vertices don't move). This is a finer-grained probe; Generator picks.
    - **(d) Make `inferRigidRadius` return 0** (so backend's Sphere has zero radius — degenerate): backend still processes step (sphere with radius=0); ground clamp at `y < 0` instead of `y < 0.5`. Cube would fall further. Doesn't FAIL Block 32 because the cube falls regardless. Replace with: **make `ensureRigidBackendBody` NON-idempotent** (allow re-adding) → second call from changeBehavior creates a duplicate body; the mesh.handle points to the latest, but `bodies_` has two entries with `mass=1` each falling under gravity. Doesn't break Block 32 either. Drop probe (d); (a)+(b)+(c) cover the load-bearing surface.

14. **Append D-039 to `docs/DECISIONS.md`** (Generator authors). Sketch:
    > **D-039 (2026-05-14)** — Rigid behavior wired through Simulator. Closes the user's "Rigid bodies should fall under gravity" goal (B-1 contract + B-2′ Euler backend + B-3 simulator wiring). `GeneralMesh<BE, PR>` gains `rigidBodyHandle = kInvalidBodyHandle` + `rigidLastBodyPos`. `Simulator<BE, PR, System>` gains private `ysim::physics::EulerRigidPhysicsBackend rigid_` (hard-coded type — no template-param widening; future Bullet B-2.1 swap is one-line). New `Simulator::ensureRigidBackendBody(meshId)` idempotent helper builds `RigidInitial{position=transformPosition, rotation=rotationQuat, mass=1, shape=Sphere(radius=inferRigidRadius)}` + `rigid_.addBody`. Triggers: (i) `changeBehavior(meshId, Rigid)`; (ii) `Simulator::initialize` resets backend with `scene.environment.gravity` and sweeps. `changeBehavior(meshId, Float)` clears the rigid linkage on the mesh side (backend slot leaks per D-038, acceptable). `Simulator::update` calls `rigid_.step(h, 1)` once per outer frame, then for each Rigid mesh: `Δpos = backend.getPosition - rigidLastBodyPos` applied to `state.x` + `state.xPrev` + `transformPosition`. `applyEnvironmentForces` skips Rigid (zeroes externalForces, mirroring Float). **Translation only — rotation-correct vertex update is a future slice.** Block 32 mechanizes the end-to-end test (`addCube(0,5,0) + changeBehavior(0, Rigid) + 30 sim.update() → witness vertex.y dropped > 0.5m`). Self-test count 61 → 62. **Retires BDD-006-RIGID-DISPATCH-PARKED** (Rigid integrator dispatch is real at the outer-update C++ layer; gravity routing is clean) **and BDD-018-BEHAVIOR-TAG-PARKED** (Float→Rigid inspector switch results in immediate next-sim-step dispatch via lazy `ensureRigidBackendBody`). BDD-008 matrix row `pending → pass`. RIGID-BACKEND-PORTABILITY (D-037) unchanged.

15. **Promote BDD-008 row** in `docs/TEST_MATRIX.md` from `pending → pass`. Test-address column appended with Block 32 + D-039 + retirement of BDD-006-RIGID-DISPATCH-PARKED + BDD-018-BEHAVIOR-TAG-PARKED noted. The "rests at floor" sub-clause is verified at the backend layer by Block 31 (`EulerRigidPhysicsBackend sphere reaches resting contact at y=radius`); an in-Simulator resting-cube test is a future-slice candidate (Box-shape support OR Bullet B-2.1).

16. **Update `docs/roles/PLANNER.md`** Standing constraints subsection: REMOVE the BDD-006-RIGID-DISPATCH-PARKED bullet AND REMOVE the BDD-018-BEHAVIOR-TAG-PARKED bullet (both retired by D-039). RIGID-BACKEND-PORTABILITY stays.

17. **Update `.agent/PROJECT_STATE.md`** (Planner-tier task during this planning pass):
    - **Next milestone**: B-3 in flight; closes user's Rigid-gravity goal.
    - **Recent scope changes**: append 2026-05-14 entry for B-3 plan.
    - **Standing feature candidates**: drop B-3 (done after this slice); promote Source-file split / Alembic export / B-2.1 Bullet.

18. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses integration verbs (wire/add field/widens/calls). In-place modification of `Simulator::update` + `changeBehavior` is correct; new symbols (`rigid_`, `ensureRigidBackendBody`, `inferRigidRadius`, Block 32) are parallel additions.
- **`project_flatbuffers_caching_skipped`**: stays in force.
- **D-026 lifetimeId invariant**: applies — `Simulator::initialize` runs `Scene::pack` (which sets lifetimeId) BEFORE the rigid sweep, so all Rigid meshes have valid lifetimeId when `ensureRigidBackendBody` runs.
- **PARALLEL-IMPL-LOCKSTEP**: applies indirectly — Euler backend's contract surface is unchanged this slice; Null + Euler still mirror.
- **CM-012 utility-helper-exit trap**: applies — `ensureRigidBackendBody` and `inferRigidRadius` must NOT `exit()`. Out-of-range meshId returns silently; wrong-behavior-tag returns silently.
- **D-013 swept-CCD invariant**: applies — `state.xPrev` gets the same Δpos as `state.x` (guarded on `xPrev.ptr` non-null) so the next frame's CCD snapshot is consistent.
- **D-021 rotateVector**: would apply if rotation-correct vertex update were enabled (Design call 3α). B-3 translation-only doesn't use it.
- **D-014 transformPosition cascade (three-site invariant)**: applies — Rigid delta-loop updates `transformPosition` alongside state.x. Inspector reads `transformPosition`; without this update, the Inspector would show stale position while the cube falls.
- **D-025 pendingRotations**: applies — `Simulator::initialize` runs `applyPendingMaterials` BEFORE the rigid sweep (per D-025's `initialize` auto-call). Order matters: pendingRotations applied → mesh.rotationQuat set → ensureRigidBackendBody reads mesh.rotationQuat for `RigidInitial.rotation`.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice. Edges considered in the design calls (mesh-not-in-scene; changeBehavior Rigid→Float→Rigid; empty scene; zero-vertex mesh). All return silently or are guarded.

## Expected metrics

- Self-test count: **61 → 62** (Block 32 gains 1 pass clause).
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest **159/159 + 1120/1120 SUCCESS** unchanged.
- Linux container: Block 32 SKIPs along with Block 1-29 (placed inside Metal-gated section because it uses `Simulator::initialize` + Metal). Block 30 + 31 still run on Linux. **Acceptable per existing pattern.**
- Expected matrix delta: BDD-008 `pending → pass`. BDD-018 row may gain a Block 32 cross-reference note (the Inspector-tag-switch dispatch becomes real).
- Expected DECISIONS.md delta: D-039 added.
- Expected PLANNER.md delta: Standing-constraints subsection LOSES BDD-006-RIGID-DISPATCH-PARKED bullet AND BDD-018-BEHAVIOR-TAG-PARKED bullet.
- Expected PROJECT_STATE.md delta: next-milestone updates; user's Rigid-gravity goal closes (note explicitly).
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTE items:
  - (i) Translation-only vertex update (no rotation propagation) — documented limitation in D-039.
  - (ii) `inferRigidRadius` bbox-half heuristic — exact for Cube; approximate for arbitrary mesh.
  - (iii) `applyEnvironmentForces` extended skip for Rigid — clean retirement; not a NOTE risk.
  - (iv) `state.xPrev` null-guard in the delta-loop — defensive; may be unnecessary if `Simulator::initialize` always allocates xPrev before the first update.
  - (v) Backend body slot-leak on changeBehavior(Float) — handle cleared on mesh side, backend slot stays. Matches D-038's existing slot-leak posture.
  - (vi) `Simulator::initialize` resetting the rigid backend means existing Rigid bodies' velocities are lost across re-initialize calls. Acceptable for `resetScene` use; might surface in scene-edit flows. Future-slice candidate.
  - **WARNING** would land if: Block 32 FAILs OR `state.x` access doesn't reach host-visible memory (Metal-side write-back missing) OR the renderer double-applies rotation (reads state.x AND rotationQuat independently) OR pre-existing Block 1-29 regressions from `applyEnvironmentForces` change.
  - **BLOCK** if Block 32 FAILs on macOS OR pre-existing PASS count regresses OR the retirements are premature.
