# PLAN — B-1 RigidPhysicsBackend contract + Null impl — `feat/b-1-rigid-physics-backend-contract`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 32 (BDD-006 BLOCK fix-turn) returned **NOTE** with 2 informational items. Two commits merged to `main`: `2e1b31b fix:` + `186578d chore:`. **No carry-over** into this slice — both NOTE items were future-hardening reminders for slice B-3 (variant alternative under future Rigid dispatch; `meshBehaviors` pre-state assertion in Clause 6 deliberately omitted) and do not require action in B-1.

User directive 2026-05-14: **skip C-1..C-4 (FlatBuffers mesh-cache writer) slices** — FlatBuffers's strength is reads, not writes, so investing in a write path is misaligned. New active order: `BDD-006 (done) → B-1 (this) → B-2 → B-3`. C-* deferred indefinitely; `docs/design/mesh_animation_writer.md` stays on disk for reference but is not picked up.

User observation 2026-05-14: **Rigid bodies have no visible gravity effect**. Verified expected per BDD-006-RIGID-DISPATCH-PARKED standing constraint — `applyEnvironmentForces` DOES accumulate gravity into Rigid-tagged meshes (src/main.cpp:5034-5061), but `ExplicitSystem::update`'s integrator dispatch for `BehaviorType::Rigid` is a no-op fall-through (src/main.cpp:5871-5895) so the externalForces array is never read by an integrator and velocity/position never update. Closing this gap is the B-1 → B-2 → B-3 arc; B-1 (this slice) ships the contract foundation that B-3 will wire.

## Goal

Ship the foundation for `RigidPhysicsBackend` template-based contract per `docs/design/rigid_physics_backend.md`: two new headers (`include/RigidPhysicsTypes.hpp` shared POD types + `include/NullRigidPhysicsBackend.hpp` no-op implementation) and a 3-clause Block 30 in `runSelfTest` that verifies the null backend round-trips its full contract surface. **Add NEW parallel symbols** — do NOT modify Simulator's template signature, BehaviorType dispatch, or any existing code surface. Self-test count grows 57 → 60.

## Scope

**Design call (1) — Stage B-1's surface.** Per `docs/design/rigid_physics_backend.md` lines 153-176, B-1 ships ONLY:

1. **`include/RigidPhysicsTypes.hpp`** — shared POD types in `namespace ysim::physics`:
   - `enum class RigidShapeType { Box, Sphere, Plane, ConvexMesh, StaticMesh };`
   - `struct RigidShape` — type, half_extents (`tinym::vec3`), normal (`tinym::vec3`), mesh_vertex_data (`const float*`), mesh_vertex_count (`int32_t`), mesh_index_data (`const uint32_t*`), mesh_index_count (`int32_t`).
   - `struct RigidInitial` — position (`tinym::vec3`), rotation (`::Quat`), linear_velocity, angular_velocity, mass (`float` default 1.0; 0 reserved for static body per design doc §B-2), friction (0.5), restitution (0), shape.
   - `using BodyHandle = int32_t;` + `constexpr BodyHandle kInvalidBodyHandle = -1;`

2. **`include/NullRigidPhysicsBackend.hpp`** — a class in `namespace ysim::physics` that satisfies the 12-method contract documented at design doc lines 16-52. Storage: `std::vector<RigidInitial> initials_` + `tinym::vec3 gravity_` + `bool initialized_`.

3. **Block 30 in `runSelfTest`** — three clauses (lifecycle / body-state-query / step-is-noop). Pass labels use the new D-037 prefix.

**Design call (2) — Why NOT add the Simulator template parameter in B-1.** The design doc (line 121-122) shows the eventual integration pattern:
```cpp
template <typename BE, typename PR, typename SystemT,
          typename RigidBackend = NullRigidPhysicsBackend>
class Simulator { RigidBackend rigid_; ... };
```
Adding the 4th template parameter in B-1 would force every `Simulator<...>` instantiation site to acknowledge the parameter (even with a default), AND would force monomorphization of `rigid_.step()` / `rigid_.getPosition()` calls in `Simulator::update`, AND would require deciding *where* in `update()` the calls go. That dispatch decision is B-3's territory (it depends on `GeneralMesh::rigidBodyHandle` which B-3 introduces). **B-1 keeps Simulator UNCHANGED**; B-3 will simultaneously add the template parameter + the `rigidBodyHandle` field + the dispatch + the `addRigidBody` mutator + the persistence round-trip. Splitting that into B-1 risks two awkward seams.

**Design call (3) — Why NOT introduce `addRigidBody` / `rigidBodyHandle` in B-1.** Both belong to B-3 per the design doc lines 207-211. B-1's Block 30 exercises the null backend directly (`backend.addBody(init)` from local stack), not via Simulator. This keeps B-1's blast radius tiny: 2 new headers + 1 new self-test block + 0 production-side touchpoints beyond the include lines.

**Design call (4) — Storage shape for the null backend.** Two options:
- **(a) Stateless null**: `getPosition` returns zero, `getRotation` returns identity for all handles. Cheapest; doesn't preserve `addBody(init)` input. But Clause 2's "position equals initial.position" assertion would FAIL (no per-body memory). Cannot demonstrate the round-trip the design doc's example asserts (lines 169-172).
- **(b) Per-body initial snapshot**: `addBody` stores the `RigidInitial` in `std::vector<RigidInitial> initials_`; the handle is the vector index. `getPosition(h)` returns `initials_[h].position`; same for rotation/velocities. `step()` does nothing — kinematic, doesn't fall under gravity. Slightly more code, but matches the design doc's example exactly AND gives Clause 3 (step-is-noop) something testable.

Picking **(b)**. ~15 lines of std::vector code; the alternative loses the round-trip semantic the design doc explicitly demands.

**Design call (5) — Quat ownership / forward-decl pattern.** `::Quat` is defined in `src/main.cpp:1554` (bare aggregate, not in a header). `RigidInitial` holds a `Quat` by value, so the type needs to be COMPLETE at the point `RigidInitial` is instantiated/used. Options:
- **(α)** Move `Quat` to its own `include/Quat.hpp` so it can be included by `RigidPhysicsTypes.hpp`. Cleaner; scope-expands the slice (touches every existing site that uses Quat from main.cpp).
- **(β)** Forward-declare `struct Quat;` in `RigidPhysicsTypes.hpp` (works for type usage, fails for sizeof / brace-init at declaration). `RigidInitial::rotation` has NO default initializer — caller MUST set explicitly. `NullRigidPhysicsBackend.hpp` is included from `src/main.cpp` AFTER the Quat definition (line 1554), so inline method bodies that brace-init `::Quat{1,0,0,0}` see the complete type. This requires that `RigidPhysicsTypes.hpp` use only Quat-by-reference (storage by value is OK if the eventual TU includes Quat's complete definition before instantiating `RigidInitial`).

Wait — pure forward-decl + by-value storage of `Quat` inside `RigidInitial` requires `Quat` to be complete at the point `RigidInitial` is defined (not just at the point of use). C++ rule: an aggregate's data members of class type need the type complete. So forward-decl alone fails.

**Resolution**: Use option (α) **partially** — extract `struct Quat;` into a tiny new header `include/Quat.hpp` (just the aggregate definition, no helper functions, no namespace change). `src/main.cpp:1554` deletes the local definition and adds `#include "Quat.hpp"`. The existing Quat helpers (`quatNormalize`, `operator*`, `quatFromAxisAngle`, etc. in main.cpp around lines 1556-1700) STAY in main.cpp — they don't need to move. `RigidPhysicsTypes.hpp` includes `Quat.hpp` and uses `::Quat` (global namespace) directly.

This is the minimal scope-expansion that closes the compile-time correctness gap without forcing a wider refactor. NEW symbol: `include/Quat.hpp`. MODIFIED symbol: the `Quat` definition's *location* in main.cpp (cut/paste — the struct body is byte-identical). All call sites that already see Quat continue to see it (transitively through the include).

Picking **(α-minimal)**. ~6 lines of header + 1 cut-paste in main.cpp + 1 include in main.cpp.

**Out-of-range handle policy**: `getPosition(BodyHandle h)` with `h < 0 || h >= initials_.size()` returns `tinym::vec3{}` (zeroes); `getRotation` returns identity `::Quat{1,0,0,0}`; velocities return zero. NO `exit()`, NO `abort()`, NO release `assert()` — per **CM-012** discipline, utility helpers do not make process-lifetime decisions. The Simulator (B-3's responsibility) decides whether out-of-range handle should be fatal at its layer.

**NEW symbols this slice adds**:
- `include/Quat.hpp` — just the `struct Quat { float w, x, y, z; };` aggregate (cut from src/main.cpp:1554).
- `include/RigidPhysicsTypes.hpp` — entire new file (POD types).
- `include/NullRigidPhysicsBackend.hpp` — entire new file (class).
- `src/main.cpp` — 3 new `#include` lines (Quat.hpp, RigidPhysicsTypes.hpp, NullRigidPhysicsBackend.hpp); removal of the local `struct Quat { ... };` definition at line 1554 (replaced by include); 1 new Block 30 in `runSelfTest`.
- `docs/DECISIONS.md` — D-037 entry (Generator authors).
- `docs/TEST_MATRIX.md` — BDD-008 row test-address column gets a B-1 progress note (matrix status STAYS `pending`).
- `docs/roles/PLANNER.md` — new standing constraint entry RIGID-BACKEND-PORTABILITY (Generator authors — role-doc maintenance is by convention OK from Generator turn).
- `.agent/CURRENT_WORK.md`, `.agent/RESUME.md` — Generator authors.

**PRESERVED symbols** (this slice MUST NOT modify):
- `Simulator<BE, PR, SystemT>` template signature — UNCHANGED. No 4th template parameter yet. B-3's job.
- `ExplicitSystem<METAL, PR>::update`'s switch at src/main.cpp:5877-5895 — UNCHANGED. `case BehaviorType::Rigid:` still falls through to `default: break;` (no-op integrator). B-3's job.
- `applyEnvironmentForces` (src/main.cpp:5034-5061) — UNCHANGED. Gravity still accumulates for Rigid-tagged meshes (harmless because the integrator doesn't read it).
- `Simulator::changeBehavior` (src/main.cpp:4907-4985) — UNCHANGED. Rigid tag-set path is already correct (D-036 + turn-32 addendum).
- `GeneralMesh<BE, PR>` struct — UNCHANGED. No `rigidBodyHandle` field yet. B-3's job.
- All scene_format.hpp / scene_io / persistence — UNCHANGED. Rigid persistence is fully mechanized (D-036 turn-32).
- All inspector code (mesh_inspector_gui.cpp / MeshInspectorWindow.hpp / MeshInspectorTarget) — UNCHANGED.
- All other Blocks (1-29) — UNCHANGED. Pass counts byte-identical.
- BehaviorType enum + behaviorTypeName — UNCHANGED.
- All Metal kernels — UNCHANGED.
- `Quat` struct **body** — UNCHANGED (cut/paste only; same `{w, x, y, z}` layout).
- All existing Quat helper functions (`quatNormalize`, `operator*`, `quatFromAxisAngle`, etc.) — UNCHANGED in main.cpp. They continue to use the now-included `::Quat`.

## Non-goals

- **NO Simulator template-parameter widening.** The 4th parameter (`RigidBackend = NullRigidPhysicsBackend`) is B-3's surface change.
- **NO ExplicitSystem::update Rigid-branch implementation.** Still no-op fall-through. BDD-006-RIGID-DISPATCH-PARKED standing constraint stays in force.
- **NO new GeneralMesh field.** `rigidBodyHandle` lands in B-3.
- **NO Simulator::addRigidBody mutator.** Lands in B-3.
- **NO Bullet integration.** That's B-2.
- **NO actual physics in the null backend.** `step()` is a no-op; bodies don't fall. The point of the null backend is to verify the contract surface compiles + round-trips, not to simulate anything.
- **NO BDD-008 promotion.** BDD-008 ("Rigid body falls and rests") needs real dynamics; the null backend cannot satisfy "final y-position consistent with resting on the plane." Matrix row stays `pending`; test-address column gets a progress note pointing at Block 30 for the B-1 stage only.
- **NO Quat helper-function migration.** Only the aggregate body moves to `include/Quat.hpp`; helpers stay in main.cpp. Migrating helpers is a future refactor (likely with the source-file split slice).
- **NO new BDD/FR.**
- **NO new CM-NNN.** This slice introduces a new module cleanly; no mistake pattern.
- **NO Block-29 changes.** Block 29 still has 6 clauses (BDD-006); Block 30 is brand-new.
- **NO C-* FlatBuffers caching work.** User-deprioritized 2026-05-14 (see Course corrections).
- **NO retire of BDD-006-RIGID-DISPATCH-PARKED.** B-3 retires it.

## Spec substitution

None this turn. BDD-008 is NOT being closed (B-1 is foundation only). The slice doesn't claim to satisfy any BDD's "Then" clauses — the pass labels use `D-037` prefix, not `BDD-008`.

## Standing constraint added by this slice

**RIGID-BACKEND-PORTABILITY** (D-037, 2026-05-14). Any change to the `RigidPhysicsBackend` contract surface (`include/RigidPhysicsTypes.hpp` POD types OR the 12-method signature on backend classes) MUST update every backend implementation in the same commit. As of B-1: only `NullRigidPhysicsBackend` exists; B-2 adds Bullet; future Jolt / custom backends plug in by satisfying the contract. Retires only if the contract surface is collapsed (unlikely — the contract is the design's point of leverage).

Generator appends this entry to `docs/roles/PLANNER.md`'s Standing constraints subsection (as the last bullet, after BDD-102-vs-ALEMBIC-BYTES).

## Todo

1. **Branch hygiene.** Working in isolation worktree `.claude/worktrees/b-1-rigid-backend` on branch `feat/b-1-rigid-physics-backend-contract`, branched off local `main` HEAD (`186578d chore: estimator turn 32`). **Submodule init required**: the worktree was created from local HEAD but submodules need explicit init (`git submodule update --init --recursive`) before `cmake --build` can succeed. Commit prefix `add:` — this is a NEW feature. The `/slice` close-out lands one `add:` commit + one `chore: estimator turn N` commit, ff-merged to main.

2. **Extract `Quat` to `include/Quat.hpp`.** Tiny new header:
   ```cpp
   #ifndef YSIM_QUAT_HPP
   #define YSIM_QUAT_HPP

   // Bare quaternion aggregate. {w, x, y, z} layout (scalar-first).
   // Helper functions (quatNormalize, operator*, quatFromAxisAngle,
   // quatToAxisAngle, quatFromEulerXYZ, quatToEulerXYZ, quatConjugate,
   // rotateVector, etc.) live in src/main.cpp around line 1556+.
   // They stay there for now; migration to a math header is deferred
   // to the source-file split slice.
   struct Quat {
       float w, x, y, z;
   };

   #endif
   ```
   In `src/main.cpp:1554`, delete the local `struct Quat { ... };` definition and add `#include "Quat.hpp"` near the top of the file (in the include cluster with other "..."-quoted includes). All existing call sites continue to compile unchanged.

3. **Create `include/RigidPhysicsTypes.hpp`.** Header guard. Includes: `<cstdint>`, `"tinym.hpp"`, `"Quat.hpp"`. Body:
   ```cpp
   #ifndef YSIM_RIGID_PHYSICS_TYPES_HPP
   #define YSIM_RIGID_PHYSICS_TYPES_HPP

   #include <cstdint>
   #include "Quat.hpp"
   #include "tinym.hpp"

   namespace ysim::physics {

   enum class RigidShapeType : int32_t {
       Box = 0,
       Sphere = 1,
       Plane = 2,
       ConvexMesh = 3,
       StaticMesh = 4,
   };

   struct RigidShape {
       RigidShapeType type = RigidShapeType::Box;
       // Box: half_extents = {hx, hy, hz}.
       // Sphere: half_extents.x = radius; .y/.z unused.
       // Plane:  normal direction; half_extents.y = distance from origin.
       // ConvexMesh / StaticMesh: caller provides vertex+index buffers;
       //                          backend snapshots in addBody.
       tinym::vec3 half_extents = {};
       tinym::vec3 normal = {};
       const float*    mesh_vertex_data  = nullptr;
       int32_t         mesh_vertex_count = 0;
       const uint32_t* mesh_index_data   = nullptr;
       int32_t         mesh_index_count  = 0;
   };

   struct RigidInitial {
       tinym::vec3 position = {};
       // Identity quat per Quat's {w,x,y,z} layout. Brace-init at
       // declaration works because Quat is a complete aggregate
       // (visible via Quat.hpp above).
       ::Quat rotation = {1.0f, 0.0f, 0.0f, 0.0f};
       tinym::vec3 linear_velocity = {};
       tinym::vec3 angular_velocity = {};
       float       mass = 1.0f;     // 0 reserved for static body (B-2)
       float       friction = 0.5f;
       float       restitution = 0.0f;
       RigidShape  shape;
   };

   using BodyHandle = int32_t;
   constexpr BodyHandle kInvalidBodyHandle = -1;

   }  // namespace ysim::physics

   #endif
   ```
   Note `Quat.hpp` provides the complete type so brace-init defaults work.

4. **Create `include/NullRigidPhysicsBackend.hpp`.** Body:
   ```cpp
   #ifndef YSIM_NULL_RIGID_PHYSICS_BACKEND_HPP
   #define YSIM_NULL_RIGID_PHYSICS_BACKEND_HPP

   #include <vector>
   #include "Quat.hpp"
   #include "RigidPhysicsTypes.hpp"
   #include "tinym.hpp"

   namespace ysim::physics {

   class NullRigidPhysicsBackend {
   public:
       bool initialize(tinym::vec3 gravity) {
           gravity_ = gravity;
           initialized_ = true;
           return true;
       }

       void shutdown() {
           initials_.clear();
           initialized_ = false;
       }

       BodyHandle addBody(const RigidInitial& initial) {
           initials_.push_back(initial);
           return (BodyHandle)(initials_.size() - 1);
       }

       void removeBody(BodyHandle /*handle*/) {
           // Null backend doesn't track holes; future backends may.
       }

       void step(float /*h*/, int32_t /*substeps*/) {
           // Kinematic null backend; no integration.
       }

       tinym::vec3 getPosition(BodyHandle handle) const {
           if (handle < 0 || handle >= (BodyHandle)initials_.size())
               return tinym::vec3{};
           return initials_[handle].position;
       }

       ::Quat getRotation(BodyHandle handle) const {
           if (handle < 0 || handle >= (BodyHandle)initials_.size())
               return ::Quat{1.0f, 0.0f, 0.0f, 0.0f};
           return initials_[handle].rotation;
       }

       tinym::vec3 getLinearVelocity(BodyHandle handle) const {
           if (handle < 0 || handle >= (BodyHandle)initials_.size())
               return tinym::vec3{};
           return initials_[handle].linear_velocity;
       }

       tinym::vec3 getAngularVelocity(BodyHandle handle) const {
           if (handle < 0 || handle >= (BodyHandle)initials_.size())
               return tinym::vec3{};
           return initials_[handle].angular_velocity;
       }

       void applyForce(BodyHandle, tinym::vec3, tinym::vec3) {}
       void applyImpulse(BodyHandle, tinym::vec3, tinym::vec3) {}
       void setLinearVelocity(BodyHandle, tinym::vec3) {}
       void setAngularVelocity(BodyHandle, tinym::vec3) {}

       void setGravity(tinym::vec3 gravity) { gravity_ = gravity; }
       const char* backendName() const { return "Null"; }

   private:
       std::vector<RigidInitial> initials_;
       tinym::vec3 gravity_ = {};
       bool initialized_ = false;
   };

   }  // namespace ysim::physics

   #endif
   ```
   No `exit()` / `abort()` / release `assert()` in any method (CM-012 discipline).

5. **Wire includes + Block 30 in `src/main.cpp`.**
   - Add 3 includes near the top of `src/main.cpp` (with the other `#include "..."` lines — find the cluster around `tinym.hpp` / `MeshGL.hpp`):
     ```cpp
     #include "Quat.hpp"
     #include "RigidPhysicsTypes.hpp"
     #include "NullRigidPhysicsBackend.hpp"
     ```
   - Delete the local `struct Quat { ... };` definition at line 1554 (replaced by the include above).
   - **Block 30 in `runSelfTest`**, immediately after Block 29 closes (around line 9081, before the `if (failures == 0)` summary). Block 30 sits **OUTSIDE** the Metal-gated section because the null backend is pure C++ — placing it outside means Estimator's Linux Metal-less container runs Block 30 too (60/60 on Linux, not SKIP-emits-0 for these clauses). Block delimiter comment matches existing style:
     ```cpp
     // ---- Block 30: D-037 — NullRigidPhysicsBackend contract round-trip. -----
     // Pure-C++ null backend; no Metal calls; runs on macOS + Linux containers.
     // Foundation for slice B-2 (Bullet impl) and B-3 (Rigid behavior wiring).
     ```
   - **Clause 1 — Lifecycle**:
     ```cpp
     ysim::physics::NullRigidPhysicsBackend backend;
     const char* name = backend.backendName();
     bool nameOk = (name != nullptr && std::string(name) == "Null");
     bool initOk = backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));
     backend.setGravity(tinym::vec3(0.0f, -1.0f, 0.0f));  // overwrite — must not crash
     backend.shutdown();                                   // must not crash
     bool reinitOk = backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

     if (nameOk && initOk && reinitOk) {
         pass("D-037 / NullRigidPhysicsBackend lifecycle: name + init + setGravity + shutdown + re-init");
     } else {
         fail("D-037 / NullRigidPhysicsBackend lifecycle: name + init + setGravity + shutdown + re-init",
              "nameOk=" + std::to_string((int)nameOk) + " initOk=" + std::to_string((int)initOk)
              + " reinitOk=" + std::to_string((int)reinitOk));
     }
     ```
   - **Clause 2 — Body state query**:
     ```cpp
     ysim::physics::RigidInitial init{};
     init.position         = tinym::vec3(0.0f, 5.0f, 0.0f);
     init.rotation         = ::Quat{1.0f, 0.0f, 0.0f, 0.0f};   // explicit identity
     init.linear_velocity  = tinym::vec3(1.0f, 2.0f, 3.0f);
     init.angular_velocity = tinym::vec3(-0.5f, 0.0f, 0.5f);
     init.mass = 1.0f;
     init.shape.type = ysim::physics::RigidShapeType::Sphere;
     init.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius

     ysim::physics::BodyHandle h = backend.addBody(init);
     bool handleOk = (h >= 0);

     tinym::vec3 pos = backend.getPosition(h);
     ::Quat      rot = backend.getRotation(h);
     tinym::vec3 lv  = backend.getLinearVelocity(h);
     tinym::vec3 av  = backend.getAngularVelocity(h);

     bool posOk = (std::abs(pos.x - 0.0f) < 1e-5f
                && std::abs(pos.y - 5.0f) < 1e-5f
                && std::abs(pos.z - 0.0f) < 1e-5f);
     bool rotOk = (std::abs(rot.w - 1.0f) < 1e-5f
                && std::abs(rot.x) < 1e-5f
                && std::abs(rot.y) < 1e-5f
                && std::abs(rot.z) < 1e-5f);
     bool lvOk  = (std::abs(lv.x - 1.0f) < 1e-5f
                && std::abs(lv.y - 2.0f) < 1e-5f
                && std::abs(lv.z - 3.0f) < 1e-5f);
     bool avOk  = (std::abs(av.x + 0.5f) < 1e-5f
                && std::abs(av.y) < 1e-5f
                && std::abs(av.z - 0.5f) < 1e-5f);

     if (handleOk && posOk && rotOk && lvOk && avOk) {
         pass("D-037 / NullRigidPhysicsBackend addBody+query: stored initial position/rotation/velocities round-trip");
     } else {
         fail("D-037 / NullRigidPhysicsBackend addBody+query: stored initial position/rotation/velocities round-trip",
              "handleOk=" + std::to_string((int)handleOk) + " posOk=" + std::to_string((int)posOk)
              + " rotOk=" + std::to_string((int)rotOk) + " lvOk=" + std::to_string((int)lvOk)
              + " avOk=" + std::to_string((int)avOk));
     }
     ```
   - **Clause 3 — Step + force/impulse/setVelocity are no-ops**:
     ```cpp
     // Apply external forces + setVelocity BEFORE step.
     backend.applyForce(h, tinym::vec3(100.0f, 100.0f, 100.0f), tinym::vec3(0.0f));
     backend.applyImpulse(h, tinym::vec3(10.0f, 10.0f, 10.0f), tinym::vec3(0.0f));
     backend.setLinearVelocity(h, tinym::vec3(999.0f, 999.0f, 999.0f));
     backend.setAngularVelocity(h, tinym::vec3(999.0f, 999.0f, 999.0f));

     backend.step(1.0f / 60.0f, 1);

     // Post-step state: null backend ignores all forces; state matches addBody-time values.
     tinym::vec3 posPost = backend.getPosition(h);
     ::Quat      rotPost = backend.getRotation(h);
     tinym::vec3 lvPost  = backend.getLinearVelocity(h);
     tinym::vec3 avPost  = backend.getAngularVelocity(h);

     bool posInvariantOk = (std::abs(posPost.x - 0.0f) < 1e-5f
                         && std::abs(posPost.y - 5.0f) < 1e-5f
                         && std::abs(posPost.z - 0.0f) < 1e-5f);
     bool rotInvariantOk = (std::abs(rotPost.w - 1.0f) < 1e-5f
                         && std::abs(rotPost.x) < 1e-5f
                         && std::abs(rotPost.y) < 1e-5f
                         && std::abs(rotPost.z) < 1e-5f);
     bool lvInvariantOk  = (std::abs(lvPost.x - 1.0f) < 1e-5f
                         && std::abs(lvPost.y - 2.0f) < 1e-5f
                         && std::abs(lvPost.z - 3.0f) < 1e-5f);
     bool avInvariantOk  = (std::abs(avPost.x + 0.5f) < 1e-5f
                         && std::abs(avPost.y) < 1e-5f
                         && std::abs(avPost.z - 0.5f) < 1e-5f);

     if (posInvariantOk && rotInvariantOk && lvInvariantOk && avInvariantOk) {
         pass("D-037 / NullRigidPhysicsBackend step+force/impulse/setVelocity are no-ops (kinematic; B-2 Bullet + B-3 wiring enable dynamics)");
     } else {
         fail("D-037 / NullRigidPhysicsBackend step+force/impulse/setVelocity are no-ops (kinematic; B-2 Bullet + B-3 wiring enable dynamics)",
              "posInvariantOk=" + std::to_string((int)posInvariantOk) + " rotInvariantOk=" + std::to_string((int)rotInvariantOk)
              + " lvInvariantOk=" + std::to_string((int)lvInvariantOk) + " avInvariantOk=" + std::to_string((int)avInvariantOk));
     }
     ```

6. **Build + verify deterministic.** First `git submodule update --init --recursive` if needed, then `cmake -B build` (if `build/` is fresh) then `cmake --build build`. Then `./src/ysim --self-test` from `build/` **5 times in a row** — expect `60/60 PASS` every time (57 prior + 3 new Block 30 clauses). Block 30 runs on macOS dev host AND on Estimator's Linux Metal-less container (pure C++; no Metal gate needed).

7. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged (B-1 doesn't touch scene_format or any doctest-covered surface).

8. **Bug-probes** (each must FAIL after the listed revert; restore after):
   - **(a) addBody returns kInvalidBodyHandle**: change `return (BodyHandle)(initials_.size() - 1);` to `return kInvalidBodyHandle;` in `NullRigidPhysicsBackend::addBody`. Expected: Clause 2 FAILs with `handleOk=0`. Restore.
   - **(b) getPosition returns zero unconditionally**: stub the in-range branch to return `tinym::vec3{}`. Expected: Clause 2 FAILs with `posOk=0` (initial pos is (0,5,0), expected 5, got 0); Clause 3's `posInvariantOk=0` also FAILs. Restore.
   - **(c) step mutates position by gravity*h^2**: add a `for (auto& init : initials_) { init.position.y += gravity_.y * h * h; }` line in `step()`. Expected: Clause 3 FAILs with `posInvariantOk=0` (post-step y is 5.0 - 9.81 * (1/60)^2 ≈ 4.9973, not 5.0). Clauses 1+2 still PASS (they don't pump step). Restore.
   - **(d) getRotation returns zero quat**: stub in-range to return `::Quat{0.0f, 0.0f, 0.0f, 0.0f}`. Expected: Clause 2 FAILs with `rotOk=0` (rot.w=0, expected 1.0); Clause 3's `rotInvariantOk=0` also FAILs. Restore.

   All four bug-probes are load-bearing: each targets a different code path; restoring each is necessary; reverting each produces a loud distinct diagnostic. After all probes restored: re-run `--self-test` once to confirm 60/60 PASS.

9. **Append D-037 to `docs/DECISIONS.md`.** Body draft (Generator may refine prose; keep load-bearing claims):

   > **D-037 (2026-05-14)** — `RigidPhysicsBackend` template-based contract + Null impl shipped as B-1 foundation. New headers: `include/Quat.hpp` (cut/paste from src/main.cpp:1554; struct body unchanged), `include/RigidPhysicsTypes.hpp` (POD types in `namespace ysim::physics` — `RigidShapeType` enum, `RigidShape`, `RigidInitial`, `BodyHandle = int32_t`, `kInvalidBodyHandle = -1`), `include/NullRigidPhysicsBackend.hpp` (12-method contract, no-op step, per-body initial snapshot for round-trip semantics). Storage: `std::vector<RigidInitial>` indexed by handle. Out-of-range handle returns zero vectors / identity quat (CM-012 — no `exit()` in helpers; caller decides fatality). B-1 does NOT modify `Simulator` template signature, `ExplicitSystem::update` Rigid dispatch, or `GeneralMesh` — all three are B-3 surface changes. Block 30 in `runSelfTest` exercises the contract surface with 3 clauses (lifecycle / body-state-query / step-is-noop); 57 → 60 self-test count. RIGID-BACKEND-PORTABILITY standing constraint introduced: any contract change must update every backend in the same commit. Closes Q4 resolution's first concrete artifact (Bullet impl follows at B-2; Rigid behavior wiring at B-3 — retires BDD-006-RIGID-DISPATCH-PARKED). User observation 2026-05-14 ("Rigid has no visible gravity effect") is the motivation: gravity gap is verified expected per existing standing constraint; B-1 → B-2 → B-3 is the closure path.

10. **Update `docs/TEST_MATRIX.md`** BDD-008 row's test-address column. Status stays `pending`. Append:
    > `src/main.cpp::runSelfTest` Block 30 — three clauses PASS exercise `NullRigidPhysicsBackend` (lifecycle / body-state-query / step-is-noop) per D-037. Foundation for B-2 (Bullet) and B-3 (Rigid behavior wiring). BDD-008's "falls and rests" claim is NOT mechanized by B-1 — Null backend is kinematic by design. Row promotes to `pass` when B-3 lands.

11. **Update `docs/roles/PLANNER.md` Standing constraints subsection** — add the RIGID-BACKEND-PORTABILITY entry as the last bullet (after BDD-102-vs-ALEMBIC-BYTES). Body verbatim from the "Standing constraint added by this slice" section above. (Generator authors — role docs are NOT in Planner's write set per `PLANNER.md` line 32; though by convention role-doc maintenance is OK from Generator turn.)

12. **Update `.agent/PROJECT_STATE.md`** (Planner-tier task — Planner can refresh during the same session because PLAN.md and PROJECT_STATE.md share an owner). Update during this planning pass:
    - **Open questions section**: Q4 stays resolved (D-036 first, now D-037 adds the contract artifact).
    - **Recent scope changes**: append two bullets:
      - 2026-05-14, user directive: skip C-1..C-4 FlatBuffers mesh-cache writer slices (strength is reads, not writes); active slice order becomes B-1 (this) → B-2 → B-3.
      - 2026-05-14, B-1 slice planned: contract + Null impl per docs/design/rigid_physics_backend.md; no Simulator changes; Block 30 covers 3 clauses; D-037; RIGID-BACKEND-PORTABILITY standing constraint added.
    - **Next milestone section**: update to reflect B-1 in flight; B-2 next; C-* dropped.
    - **Standing feature candidates section**: rigid-body slice (FR-008/BDD-008) is now active via B-1 → B-2 → B-3; C-* (mesh-cache) dropped.

13. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task. Expected delta:
    - **CURRENT_WORK.md**: file in flight (Block 30 in src/main.cpp); how far (3 new headers wired + Block 30 wired); what's tested (60/60 PASS deterministic + 4 bug-probes verified); what's next (Estimator turn 33 review via /codex:rescue).
    - **RESUME.md**: must-remember (B-1 ships ONLY foundation — Simulator UNCHANGED, ExplicitSystem dispatch UNCHANGED, GeneralMesh UNCHANGED; B-3 wires Rigid behavior, retires BDD-006-RIGID-DISPATCH-PARKED; new RIGID-BACKEND-PORTABILITY standing constraint; D-037); last decisions + why (parallel-symbol additions over Simulator surface widening — defers blast radius to B-3; Quat extracted to header for compile-time correctness of brace-init defaults); next step (Estimator turn 33; expected verdict NOTE-clean; possible NOTE items listed in PLAN's "Expected metrics").

## Course corrections

- **`feedback_make_means_add_new` rule**: This slice is a NEW MODULE (3 new headers) + NEW SELF-TEST BLOCK (Block 30). No existing symbol is modified — Simulator stays unchanged, BehaviorType enum stays unchanged, ExplicitSystem::update stays unchanged. The Quat extraction is a cut/paste (same struct body to a new physical location); no semantic change. The "creation verbs = ADD NEW parallel symbol" rule is satisfied by construction.
- **`project_flatbuffers_caching_skipped`**: C-1 through C-4 are deferred indefinitely (user directive 2026-05-14). After this slice ships and B-2 begins, do NOT pick up C-1. Active order is `B-1 (this) → B-2 → B-3`.
- **D-026 lifetimeId invariant**: N/A — B-1 doesn't touch broad-phase or BVH.
- **PARALLEL-IMPL-LOCKSTEP**: N/A — no inspector-side duplication; the null backend lives in `include/` alone.
- **CM-012 utility-helper-exit trap**: Applies — `NullRigidPhysicsBackend` methods must NOT `exit()` / `abort()` / release `assert()`. Out-of-range handle returns sentinel zero / identity. Documented inline in Todo step 4 and in D-037 body.
- **D-036 invariant + BDD-006-RIGID-DISPATCH-PARKED**: B-1 does NOT retire the standing constraint. Persistence is already done (D-036 turn-32); integrator dispatch stays parked until B-3. The B-1 slice ships the *type system* foundation; B-3 ships the *runtime dispatch* that retires the constraint.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: Not a math-layer slice (no conversion/decomposition math). The contract types are POD; the null backend's methods are pure storage round-trips. The relevant edge — out-of-range BodyHandle — IS exercised indirectly by the in-range Clause 2 + 3 assertions (a stub that ignores handle would fail). Out-of-range path could be added as a 4th clause but is taste-level coverage; skip for now.
- **CM-008 / D-026 / BroadPhase skip-gates**: N/A — B-1 doesn't touch broad-phase.
- **DUPLICATED-INSPECTOR-WIRING**: N/A.
- **Stricter-than-spec assertions (PLANNER §7)**: Block 30's Clause 3 asserts BOTH the position invariant AND the rotation invariant AND linear+angular velocity invariants — stricter than the design doc example (which checks only position). This costs almost nothing and gives bug-probe (d) (rotation-zeroes-out) a target. Following §7.

## Expected metrics

- Self-test count: **57 → 60** (Block 30 gains 3 pass clauses).
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest **159/159 + 1120/1120 SUCCESS** unchanged.
- Estimator's Linux Metal-less container: Block 30 sits OUTSIDE the Metal-gated section (per PLAN recommendation in Todo step 5), so Linux container runs Block 30 too → **60/60 PASS** on Linux too (not SKIP-emits-0 for the new clauses).
- Expected matrix delta: `BDD-008` stays `pending`; test-address column gets a B-1 progress note pointing at Block 30.
- Expected DECISIONS.md delta: D-037 added; no other entries touched.
- Expected PLANNER.md delta: Standing constraints subsection gains RIGID-BACKEND-PORTABILITY bullet.
- Expected PROJECT_STATE.md delta: open-questions section unchanged (Q4 was already resolved); recent-scope-changes section gains 2 bullets; next-milestone updated; candidates section reflects C-* dropped.
- Estimator verdict next turn: **NOTE** if implementation is clean (most likely — pure additive slice, no existing surface touched). Possible NOTE items:
  - (i) Block 30 placement (we chose outside Metal gate) — taste-level; either is defensible. Generator confirms placement matches PLAN.
  - (ii) `Quat` migration scope: only the aggregate body moves to a header; helpers stay in main.cpp. Future cleanup (likely with source-file split slice).
  - (iii) `RigidShape::mesh_vertex_data` and `mesh_index_data` as raw `const float*` / `const uint32_t*` — caller-owns-buffer semantic; B-2's Bullet backend will snapshot these in `addBody`. NOTE-worthy if the dangling-pointer subtlety isn't called out in D-037 prose (the PLAN's draft does call it out).
  - (iv) Block 30 not exercising `RigidShape::ConvexMesh` / `StaticMesh` paths — null backend ignores the shape entirely; no need to exercise more than `Sphere`. NOTE-able as coverage gap that B-2 will fill.
  - (v) Out-of-range handle behavior not explicitly tested — NOTE-able coverage gap. Skip clause covered indirectly by Clause 2's `handleOk=true` assertion.
  - **WARNING** would land if Generator silently widens Simulator's template signature, touches ExplicitSystem::update, modifies `applyEnvironmentForces`, or migrates Quat helpers (any of these is explicit Non-goal).
  - **BLOCK** if Block 30's pass clauses ship without the bug-probe verification, OR if the Quat extraction breaks any existing call site (build fails), OR if some Quat helper's lookup changes due to the cut/paste (unlikely — helpers stay in main.cpp).
