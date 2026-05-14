# PLAN — B-2′ Euler RigidPhysicsBackend impl (PIVOT from Bullet) — `feat/b-2-bullet-rigid-backend`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14 (REWRITTEN — pivot from Bullet vendor)

## Pivot rationale

The original B-2 plan called for vendoring Bullet 3.25 via git submodule. The user authorized the vendor in AskUserQuestion (option a). The Bash classifier denied the `git submodule add` regardless because it reads the action surface, not the in-conversation answer. With the /goal Stop hook enforcing "rigid 오브젝트가 중력 영향을 받는지 재검토" and the user explicit about continuing implementation, the practical path is option (c) — a minimal semi-implicit Euler integrator backend.

The branch name (`feat/b-2-bullet-rigid-backend`) is now a misnomer; renaming would be process-noise. The CURRENT_WORK file documents the pivot. The commit message + D-038 explain. **If the user later adds a `git submodule add bulletphysics/...` permission rule, the canonical Bullet B-2 can be done as B-2.1 — the contract is the same; Bullet plugs in as a third backend alongside Null + Euler.**

## Course note: previous turn's verdict

Estimator turn 33 returned **WARNING** with 0 BLOCK + 1 WARNING + 3 NOTE. The WARNING is **folded into this slice**: Block 30 sits below `runSelfTest`'s Metal-less SKIP gate at `src/main.cpp:6164`, so on Linux containers the harness returns 0 before exercising the null contract. The 3 NOTE items are informational.

B-1 RigidPhysicsBackend contract + Null impl slice (D-037) merged to `main` via commits `8a44275 add:` + `43ccb7f chore:`. 56 → 59 self-test PASS deterministic.

## Goal

Ship `EulerRigidPhysicsBackend` — a minimal semi-implicit Euler integrator implementing the D-037 contract. Supports world gravity per body; supports a single y=0 ground plane via `y_clamp(radius)` for the resting-contact assertion. Add Block 31 in `runSelfTest` (placed ABOVE the Metal-less SKIP gate) with 2 dynamics clauses (sphere-fall + sphere-on-plane resting contact). **Fold Estimator turn 33 WARNING** by relocating Block 30 above the same SKIP gate so both Null AND Euler contracts run on Linux verify.sh. **No Simulator dispatch wiring** — B-3's surface. Self-test count 59 → 61.

## Scope

**Design call (1) — Semi-implicit Euler over RK4 or Verlet.** Semi-implicit Euler (`v_new = v + a*h; x_new = x + v_new*h`) is:
- Stable for gravity-driven motion (no oscillation amplification at typical h=1/60).
- Reversible-enough for the user's "rigid bodies fall and rest" use case.
- Trivial to verify against analytic expectations: Δy ≈ -g*h² for one step from rest.
- Matches what a future Bullet integration would produce within ~10× tolerance.

RK4 is overkill; explicit Euler is unstable; Verlet needs xPrev tracking. Semi-implicit is the sweet spot for a ~150-line backend.

**Design call (2) — Single y=0 ground plane (radius-clamped).** Block 31 Clause 2 ("sphere on plane reaches resting") needs collision detection. Full polygon-vs-polygon contact resolution is out of scope. Implement: at end of `step()`, for each body with a Sphere shape, if `position.y < radius`, clamp to `position.y = radius` AND zero `linear_velocity.y` (perfect inelastic ground at y=0). This satisfies the resting-contact assertion deterministically without any contact-pair machinery.

Limitations explicitly accepted: only Sphere shapes get plane collision; Box and Plane don't bounce off each other; arbitrary plane orientation is unsupported (always y=0). Adequate for B-2′'s test surface; B-3 + future "real Bullet" slice fill broader collision.

**Design call (3) — Backend storage shape.** Per-body state stored in a simple struct:
```cpp
struct EulerBody {
    tinym::vec3 position;
    tinym::vec3 linear_velocity;
    tinym::vec3 angular_velocity;
    ::Quat      rotation;
    float       mass;        // 0 → static (kinematic)
    RigidShape  shape;       // for plane-collision lookup
};
std::vector<EulerBody> bodies_;
tinym::vec3 gravity_;
```
No external lib types; pure POD-ish. Body destruction is trivial (no Bullet-like ownership-order subtlety).

**Design call (4) — Quat update during step (rotation evolution).** Angular velocity `ω` integrates rotation via the quaternion derivative: `q_dot = 0.5 * Quat(0, ω.x, ω.y, ω.z) * q`. Update: `q_new = quatNormalize(q + q_dot * h)`. For B-2′ this is a stretch — Block 31 doesn't exercise angular motion (sphere is rotationally symmetric and starts with `angular_velocity = 0`). Implement the update anyway so the backend is rotation-correct for B-3's use cases; the test will skip non-trivial angular cases.

Use the existing `Quat` and free functions (`operator*`, `quatNormalize`) from `src/main.cpp:1556+` if reachable from the header. Issue: the helpers are defined in main.cpp (per the B-1 cut/paste); their declarations need to be visible in `EulerRigidPhysicsBackend.hpp`. Either:
- (a) Forward-declare the helpers in the backend header. Brittle.
- (b) Inline the small operations in the backend header (write a 4-line quat-multiply + 3-line normalize inline). No external dep.
- (c) Extract helpers to `include/QuatMath.hpp`. Scope expansion — moves helpers out of main.cpp.

Picking **(b)**. ~7 lines of inline math; avoids touching main.cpp's helper-cluster. Document the duplication in PLANNER.md's PARALLEL-IMPL-LOCKSTEP standing constraint (same shape as D-035's main.cpp ↔ MeshInspectorWindow.hpp pattern).

**Design call (5) — Header-only.** The Euler backend has no heavy external headers (just `tinym.hpp`, `Quat.hpp`, `RigidPhysicsTypes.hpp`, `<vector>`). All bodies fit in a single header. No `.cpp` needed — keeps the slice's footprint to one new file under `include/`.

**Design call (6) — Block 30 relocation.** Move Block 30 from `src/main.cpp:~9083` to `src/main.cpp:~6161` (after the harness lambdas at lines 6112-6152, before `auto* device = MetalGlobalContext::getDevice()` at line 6163). Content byte-identical; only position moves. Block 31 (new) sits immediately after relocated Block 30.

**NEW symbols this slice adds**:
- `include/EulerRigidPhysicsBackend.hpp` — entire new file (~150 lines, header-only).
- Block 31 in `src/main.cpp::runSelfTest` (2 pass clauses).
- `src/main.cpp` — 1 new `#include "EulerRigidPhysicsBackend.hpp"` near the existing rigid-physics includes.
- `docs/DECISIONS.md` — D-038 entry.
- `docs/TEST_MATRIX.md` — BDD-008 row test-address column appended with Block 31 reference; status stays `pending`.

**MODIFIED symbols in place**:
- `src/main.cpp` — Block 30 relocated from ~9083 to ~6161 (chunk moves, content unchanged); Block 31 inserted immediately after Block 30 in the new position; 1 new #include added.
- `docs/roles/PLANNER.md` — RIGID-BACKEND-PORTABILITY entry updates from "as of B-1" to "as of B-2′ — both Null + Euler backends live; Bullet is a future slice candidate."

**PRESERVED symbols** (this slice MUST NOT modify):
- `include/RigidPhysicsTypes.hpp` — UNCHANGED. Contract POD types frozen.
- `include/NullRigidPhysicsBackend.hpp` — UNCHANGED. Both backends co-exist as parallel symbols.
- `include/Quat.hpp` — UNCHANGED.
- `Simulator<BE, PR, SystemT>` template signature — UNCHANGED. B-3's surface.
- `ExplicitSystem<METAL, PR>::update`'s Rigid branch at `src/main.cpp:5891` — UNCHANGED. Still no-op fall-through.
- `applyEnvironmentForces` — UNCHANGED.
- `Simulator::changeBehavior` — UNCHANGED.
- `GeneralMesh<BE, PR>` — UNCHANGED.
- All scene_format / persistence — UNCHANGED.
- All inspector code — UNCHANGED.
- Block 30's 3 clauses — content byte-identical; only physical location changes.
- Blocks 1-29 — UNCHANGED.

## Non-goals

- **NO Bullet vendor.** Pivoted from. Future slice can add Bullet as a third backend once the user grants permission for the submodule add.
- **NO Simulator template-parameter widening.** B-3's surface.
- **NO ExplicitSystem::update Rigid-branch implementation.** Still no-op fall-through. BDD-006-RIGID-DISPATCH-PARKED stays in force.
- **NO `Simulator::addRigidBody` mutator.** B-3.
- **NO `GeneralMesh::rigidBodyHandle` field.** B-3.
- **NO full collision detection.** Single y=0 ground plane via radius-clamp; no contact pairs, no rotation-on-impact, no general convex-vs-convex.
- **NO angular collision response.** Sphere is rotationally symmetric for our test; no angular dynamics tested.
- **NO friction or restitution beyond perfect inelastic at the ground clamp.** Adequate for "sphere comes to rest at y ≈ radius."
- **NO BDD-008 promotion.** "Falls and rests" needs Simulator-side wiring (B-3) AND a tagged Rigid mesh creation path. Matrix row stays `pending`.
- **NO retire of BDD-006-RIGID-DISPATCH-PARKED.** B-3.
- **NO C-* FlatBuffers slices.** Skipped per 2026-05-14 directive.
- **NO new BDD/FR.**
- **NO new CM-NNN.**
- **NO Block-29 changes.**

## Spec substitution

None this turn. BDD-008 stays `pending`. Pass labels use `D-038 / EulerRigidPhysicsBackend ...` prefix.

The pivot from Bullet → Euler IS a kind of "spec substitution" at the design-doc layer: `docs/design/rigid_physics_backend.md` §B-2 specifies Bullet 3.25 as the implementation for stage B-2. The Euler integrator is a substitute that satisfies the same D-037 contract surface but with a smaller physics surface (no contact pairs, no convex-convex). Documented as a deviation in D-038; the design doc stays unchanged (Bullet remains the eventual goal).

## Standing constraint reinforced

**RIGID-BACKEND-PORTABILITY** (D-037, 2026-05-14) — now load-bearing across two backends: Null + Euler. Future Bullet add would make three. Lockstep verified by the Generator (grep both backend headers for the method signatures).

**PARALLEL-IMPL-LOCKSTEP** (D-035, 2026-05-13) — extended: D-035's main.cpp ↔ MeshInspectorWindow.hpp Quat math duplication now joined by the Euler backend's inline mini-Quat helpers (q_dot computation + normalize). If a future slice changes Quat semantics in main.cpp's `operator*` / `quatNormalize`, the Euler backend's inline copies must be checked too. Documented in the backend header.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/b-2-bullet` on branch `feat/b-2-bullet-rigid-backend` (branched off main HEAD `43ccb7f`). Branch name is now a misnomer (ships Euler not Bullet); the pivot is documented in CURRENT_WORK / RESUME / D-038 / commit message. Commit prefix `add:`.

2. **Create `include/EulerRigidPhysicsBackend.hpp`** — header-only ~150 lines. Class in `namespace ysim::physics`:
   ```cpp
   #ifndef YSIM_EULER_RIGID_PHYSICS_BACKEND_HPP
   #define YSIM_EULER_RIGID_PHYSICS_BACKEND_HPP

   #include <vector>
   #include "Quat.hpp"
   #include "RigidPhysicsTypes.hpp"
   #include "tinym.hpp"

   namespace ysim::physics {

   // Minimal semi-implicit Euler integrator backend for the D-037
   // RigidPhysicsBackend contract. Supports world gravity per body;
   // supports a single y=0 ground plane via radius-clamp on Sphere
   // shapes only. NO general contact resolution; NO angular collision;
   // NO friction. Adequate for B-2′'s scope (gravity + resting contact);
   // future Bullet slice (B-2.1) replaces with full dynamics.
   //
   // D-038 records the pivot from the Bullet-vendor B-2 plan.
   class EulerRigidPhysicsBackend {
   public:
       bool initialize(tinym::vec3 gravity) {
           gravity_ = gravity;
           initialized_ = true;
           return true;
       }

       void shutdown() {
           bodies_.clear();
           initialized_ = false;
       }

       BodyHandle addBody(const RigidInitial& initial) {
           EulerBody b;
           b.position         = initial.position;
           b.rotation         = initial.rotation;
           b.linear_velocity  = initial.linear_velocity;
           b.angular_velocity = initial.angular_velocity;
           b.mass             = initial.mass;
           b.friction         = initial.friction;
           b.restitution      = initial.restitution;
           b.shape            = initial.shape;
           bodies_.push_back(b);
           return static_cast<BodyHandle>(bodies_.size() - 1);
       }

       void removeBody(BodyHandle /*handle*/) {
           // Slot-leak by design: handles are stable indices. Future
           // slice may compact / free-list. CM-012 discipline.
       }

       void step(float h, int32_t /*substeps*/) {
           if (!initialized_) return;
           for (auto& b : bodies_) {
               if (b.mass <= 0.0f) continue;  // static body — no integration

               // Semi-implicit Euler: v += a*h; x += v*h.
               b.linear_velocity.x += gravity_.x * h;
               b.linear_velocity.y += gravity_.y * h;
               b.linear_velocity.z += gravity_.z * h;

               b.position.x += b.linear_velocity.x * h;
               b.position.y += b.linear_velocity.y * h;
               b.position.z += b.linear_velocity.z * h;

               // Angular: q_dot = 0.5 * Quat(0, ω) * q ; q += q_dot * h ; normalize.
               // Inline mini-Quat multiply because Quat helpers live in main.cpp
               // and we don't want a header dep on main.cpp internals
               // (PARALLEL-IMPL-LOCKSTEP — if main.cpp's quatNormalize/operator*
               // semantics change, this inline copy must follow).
               const ::Quat w_quat{0.0f, b.angular_velocity.x, b.angular_velocity.y, b.angular_velocity.z};
               const ::Quat& q = b.rotation;
               ::Quat q_dot;
               q_dot.w = 0.5f * (w_quat.w * q.w - w_quat.x * q.x - w_quat.y * q.y - w_quat.z * q.z);
               q_dot.x = 0.5f * (w_quat.w * q.x + w_quat.x * q.w + w_quat.y * q.z - w_quat.z * q.y);
               q_dot.y = 0.5f * (w_quat.w * q.y - w_quat.x * q.z + w_quat.y * q.w + w_quat.z * q.x);
               q_dot.z = 0.5f * (w_quat.w * q.z + w_quat.x * q.y - w_quat.y * q.x + w_quat.z * q.w);
               ::Quat q_new;
               q_new.w = q.w + q_dot.w * h;
               q_new.x = q.x + q_dot.x * h;
               q_new.y = q.y + q_dot.y * h;
               q_new.z = q.z + q_dot.z * h;
               // Normalize (avoid divide-by-zero — fall back to identity).
               const float n2 = q_new.w*q_new.w + q_new.x*q_new.x + q_new.y*q_new.y + q_new.z*q_new.z;
               if (n2 > 1e-12f) {
                   const float inv = 1.0f / std::sqrt(n2);
                   q_new.w *= inv; q_new.x *= inv; q_new.y *= inv; q_new.z *= inv;
               } else {
                   q_new = ::Quat{};  // identity
               }
               b.rotation = q_new;

               // Ground-plane collision: Sphere only, plane at y=0 with normal up.
               // If position.y < radius, clamp + zero downward velocity (perfect
               // inelastic). Other shape types pass through (no collision).
               if (b.shape.type == RigidShapeType::Sphere) {
                   const float radius = b.shape.half_extents.x;
                   if (b.position.y < radius) {
                       b.position.y = radius;
                       if (b.linear_velocity.y < 0.0f) b.linear_velocity.y = 0.0f;
                   }
               }
           }
       }

       tinym::vec3 getPosition(BodyHandle handle) const {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size()))
               return tinym::vec3{};
           return bodies_[handle].position;
       }

       ::Quat getRotation(BodyHandle handle) const {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size()))
               return ::Quat{};
           return bodies_[handle].rotation;
       }

       tinym::vec3 getLinearVelocity(BodyHandle handle) const {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size()))
               return tinym::vec3{};
           return bodies_[handle].linear_velocity;
       }

       tinym::vec3 getAngularVelocity(BodyHandle handle) const {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size()))
               return tinym::vec3{};
           return bodies_[handle].angular_velocity;
       }

       void applyForce(BodyHandle handle, tinym::vec3 force_world, tinym::vec3 /*at_world_point*/) {
           // Simple body-center impulse approximation: F = m*a, so Δv = (F/m).
           // Caller-supplied "at_world_point" ignored (no angular response).
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
           auto& b = bodies_[handle];
           if (b.mass <= 0.0f) return;
           const float inv_m = 1.0f / b.mass;
           b.linear_velocity.x += force_world.x * inv_m;
           b.linear_velocity.y += force_world.y * inv_m;
           b.linear_velocity.z += force_world.z * inv_m;
       }

       void applyImpulse(BodyHandle handle, tinym::vec3 impulse_world, tinym::vec3 at_world_point) {
           // Impulse and force have identical body-center treatment here.
           applyForce(handle, impulse_world, at_world_point);
       }

       void setLinearVelocity(BodyHandle handle, tinym::vec3 v) {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
           bodies_[handle].linear_velocity = v;
       }

       void setAngularVelocity(BodyHandle handle, tinym::vec3 w) {
           if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
           bodies_[handle].angular_velocity = w;
       }

       void setGravity(tinym::vec3 gravity) { gravity_ = gravity; }
       const char* backendName() const { return "Euler"; }

   private:
       struct EulerBody {
           tinym::vec3 position         = {};
           ::Quat      rotation;
           tinym::vec3 linear_velocity  = {};
           tinym::vec3 angular_velocity = {};
           float       mass             = 1.0f;
           float       friction         = 0.5f;
           float       restitution      = 0.0f;
           RigidShape  shape;
       };

       std::vector<EulerBody> bodies_;
       tinym::vec3            gravity_ = {};
       bool                   initialized_ = false;
   };

   }  // namespace ysim::physics

   #endif
   ```
   `#include <cmath>` needed for `std::sqrt`. No external deps.

3. **Wire `#include "EulerRigidPhysicsBackend.hpp"`** near the existing rigid-physics includes in `src/main.cpp` (after `NullRigidPhysicsBackend.hpp`).

4. **Relocate Block 30** in `src/main.cpp` — currently at ~line 9083 (AFTER the Metal-less SKIP gate). Move the entire chunk to ~line 6161 (BEFORE the `auto* device = MetalGlobalContext::getDevice();` line, AFTER the `pass / fail / skip / pumpFrames / resetScene / buildSyntheticScene` lambdas). Content byte-identical.

5. **Add Block 31** immediately after the relocated Block 30:
   - **Clause 1 — Sphere falls under gravity for one step (semi-implicit Euler: Δy ≈ -2.725 mm)**:
     ```cpp
     ysim::physics::EulerRigidPhysicsBackend backend;
     backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

     ysim::physics::RigidInitial init{};
     init.position         = tinym::vec3(0.0f, 5.0f, 0.0f);
     init.rotation         = ::Quat{1.0f, 0.0f, 0.0f, 0.0f};
     init.mass             = 1.0f;
     init.shape.type       = ysim::physics::RigidShapeType::Sphere;
     init.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius
     ysim::physics::BodyHandle h = backend.addBody(init);
     bool handleOk = (h >= 0);

     // Pre-step: position should match initial.
     tinym::vec3 pre = backend.getPosition(h);
     bool preOk = (std::abs(pre.y - 5.0f) < 1e-5f);

     backend.step(1.0f / 60.0f, 1);

     // Semi-implicit: v_new = -9.81/60; x_new = 5 + v_new/60 = 5 - 9.81/3600.
     tinym::vec3 post = backend.getPosition(h);
     float dy = post.y - 5.0f;
     float expected_dy = -9.81f / 3600.0f;  // -0.002725 m
     bool dyOk = (std::abs(dy - expected_dy) < 1e-5f);

     tinym::vec3 lv = backend.getLinearVelocity(h);
     float expected_vy = -9.81f / 60.0f;     // -0.1635 m/s
     bool vyOk = (std::abs(lv.y - expected_vy) < 1e-5f);

     if (handleOk && preOk && dyOk && vyOk) {
         pass("D-038 / EulerRigidPhysicsBackend sphere falls under gravity for one step (semi-implicit Δy=-2.725mm)");
     } else {
         fail("D-038 / EulerRigidPhysicsBackend sphere falls under gravity for one step (semi-implicit Δy=-2.725mm)",
              "handleOk=" + std::to_string((int)handleOk)
              + " preOk=" + std::to_string((int)preOk)
              + " dyOk=" + std::to_string((int)dyOk)
              + " vyOk=" + std::to_string((int)vyOk)
              + " dy=" + std::to_string(dy)
              + " lv.y=" + std::to_string(lv.y));
     }
     ```
   - **Clause 2 — Sphere reaches resting contact at y = radius via radius-clamp**:
     ```cpp
     ysim::physics::EulerRigidPhysicsBackend backend2;
     backend2.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

     // No plane body needed — Euler backend has built-in y=0 ground clamp
     // for Sphere shapes. Drop sphere from y=2.
     ysim::physics::RigidInitial sphere{};
     sphere.position = tinym::vec3(0.0f, 2.0f, 0.0f);
     sphere.mass = 1.0f;
     sphere.shape.type = ysim::physics::RigidShapeType::Sphere;
     sphere.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius
     ysim::physics::BodyHandle sphereH = backend2.addBody(sphere);

     // Pump 120 steps. With radius-clamp, sphere reaches y=0.5 within
     // ~60 steps (2.725mm/step × 60 = 16cm fall) and stays there.
     for (int i = 0; i < 120; ++i) backend2.step(1.0f / 60.0f, 1);

     tinym::vec3 pos = backend2.getPosition(sphereH);
     tinym::vec3 lv2 = backend2.getLinearVelocity(sphereH);
     bool restPosOk = (std::abs(pos.y - 0.5f) < 1e-5f);
     bool restVelOk = (lv2.y >= 0.0f);  // y-velocity clamped non-negative at ground

     if (restPosOk && restVelOk) {
         pass("D-038 / EulerRigidPhysicsBackend sphere reaches resting contact at y=radius (y=0 ground clamp)");
     } else {
         fail("D-038 / EulerRigidPhysicsBackend sphere reaches resting contact at y=radius (y=0 ground clamp)",
              "restPosOk=" + std::to_string((int)restPosOk)
              + " restVelOk=" + std::to_string((int)restVelOk)
              + " pos.y=" + std::to_string(pos.y)
              + " lv.y=" + std::to_string(lv2.y));
     }
     ```

6. **Build + verify deterministic.** `cmake --build build`. Then `./src/ysim --self-test` from `build/` **5 times in a row** — expect **61/61 PASS** every time (59 prior + 2 Block 31). Block 31 is pure C++ with deterministic float arithmetic; deterministic across runs.

7. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

8. **Bug-probes** (each must FAIL after the listed revert; restore after):
   - **(a) EulerRigidPhysicsBackend::step does nothing**: comment out the `for (auto& b : bodies_) { ... }` loop body → Clauses 1+2 FAIL (`dy=0`, `restPosOk=0` because position stays at 2.0). Restore.
   - **(b) Gravity term zeroed**: replace `gravity_.y * h` with `0.0f` in the velocity update → Clauses 1+2 FAIL (`dy=0`, sphere never falls). Restore.
   - **(c) Mass = 0 (static body)**: in Clause 1 test setup, change `init.mass = 1.0f` → `init.mass = 0.0f` → Clause 1 FAILs (`dy=0`, body skipped by `if (mass <= 0) continue;`). Restore.
   - **(d) Ground-plane clamp commented out**: comment out the `if (position.y < radius) { ... }` block → Clause 2 FAILs (`pos.y` continues falling past 0.5; after 120 steps it's well below 0). Restore.

   All 4 bug-probes load-bearing on distinct code paths. After all restored: `--self-test` returns 61/61 PASS.

9. **Append D-038 to `docs/DECISIONS.md`** (Generator authors). Body sketch:
    > **D-038 (2026-05-14)** — `EulerRigidPhysicsBackend` shipped as second backend implementing D-037's contract. **Pivoted from the original Bullet-vendor B-2 plan**: user authorized Bullet via AskUserQuestion but the Bash classifier denies `git submodule add` regardless (the classifier reads the action surface, not the in-conversation answer). With the /goal Stop hook enforcing implementation + gravity verification, pivoted to a minimal Euler integrator (Option C from the slice's CURRENT_WORK menu). Single header `include/EulerRigidPhysicsBackend.hpp` (~180 lines). Semi-implicit Euler: `v_new = v + a*h; x_new = x + v_new*h`. Quat update via inline `q_dot = 0.5 * Quat(0, ω) * q` (PARALLEL-IMPL-LOCKSTEP — duplicated against main.cpp's Quat helpers). Single y=0 ground plane via Sphere-only `y_clamp(radius)` for the resting-contact assertion. Mass=0 → static (no integration). NO friction, NO restitution, NO general convex-vs-convex contact, NO angular collision. Bullet remains the canonical eventual implementation per `docs/design/rigid_physics_backend.md` §B-2; future slice can add it as a third backend (B-2.1) once permission is granted. Block 31 (2 clauses: sphere falls semi-implicit Δy=-2.725mm in one step; sphere reaches resting contact at y=radius after 120 steps). Block 30 relocated from src/main.cpp:~9083 to ~6161 (BEFORE Metal-less SKIP gate at 6164 — folds Estimator turn 33 WARNING; both Null and Euler contracts now run on Linux verify.sh). Self-test count 59 → 61. RIGID-BACKEND-PORTABILITY (D-037) now load-bearing across Null + Euler. NO Simulator dispatch wiring — B-3's surface; BDD-006-RIGID-DISPATCH-PARKED stays in force.

10. **Update `docs/TEST_MATRIX.md`** BDD-008 row. Status stays `pending`. Append:
    > `src/main.cpp::runSelfTest` Block 31 — two clauses PASS exercise `EulerRigidPhysicsBackend` (sphere falls under gravity for one step semi-implicit Δy=-2.725mm; sphere reaches resting contact at y=radius after 120 steps via y=0 ground clamp) per D-038. **Backend exists with deterministic gravity; Rigid behavior tag dispatch still parked under BDD-006-RIGID-DISPATCH-PARKED until B-3 wires Simulator integration.** Row promotes to `pass` when B-3 lands. Bullet backend (canonical B-2 per design doc) deferred to a future slice (B-2.1) once vendor permission is granted.

11. **Update `docs/roles/PLANNER.md`** — RIGID-BACKEND-PORTABILITY entry's "As of B-1" → "As of B-2′" wording:
    > **RIGID-BACKEND-PORTABILITY** (D-037, 2026-05-14; reinforced by D-038, 2026-05-14). Any change to the contract surface (`include/RigidPhysicsTypes.hpp` POD types OR the 12-method signature on backend classes) MUST update every backend implementation in the same commit. **As of B-2′: `NullRigidPhysicsBackend` (`include/NullRigidPhysicsBackend.hpp`) AND `EulerRigidPhysicsBackend` (`include/EulerRigidPhysicsBackend.hpp`) are both live; the constraint is load-bearing across two backends.** Bullet backend (`docs/design/rigid_physics_backend.md` §B-2) deferred to a future slice (B-2.1). Future Jolt / custom backends plug in by satisfying the contract.

12. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task. Expected delta documented in CURRENT_WORK's "Pivot summary" already; finalize with metrics + bug-probe table.

## Course corrections

- **`feedback_make_means_add_new` rule**: NEW parallel backend (`EulerRigidPhysicsBackend`) co-exists with NullRigidPhysicsBackend; contract surface unchanged. Block 30 stays intact; Block 31 brand-new. Rule satisfied.
- **`project_flatbuffers_caching_skipped`**: stays in force.
- **D-026 lifetimeId invariant**: N/A.
- **PARALLEL-IMPL-LOCKSTEP**: extended this slice. Euler backend's inline Quat math (q_dot derivation + normalize) duplicates main.cpp's Quat helpers. If main.cpp's Quat semantics change, the Euler backend's inline copies must follow. Documented in the header comment.
- **CM-012 utility-helper-exit trap**: Applies — `EulerRigidPhysicsBackend` methods must NOT `exit()` / `abort()`. Out-of-range BodyHandle returns sentinel zero / identity. Documented inline.
- **D-037 invariant + RIGID-BACKEND-PORTABILITY**: B-2′ is the first parallel backend; Generator greps both `EulerRigidPhysicsBackend.hpp` + `NullRigidPhysicsBackend.hpp` to confirm 12-method signatures match.
- **D-036 invariant + BDD-006-RIGID-DISPATCH-PARKED**: B-2′ does NOT retire. B-3 wires.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: Euler integrator domain edges:
  - mass=0 (static body — bug-probe (c) covers; the `if (b.mass <= 0.0f) continue;` line is load-bearing).
  - position at exactly radius (no clamping needed; sphere already at rest). Implicitly handled by `if (position.y < radius)` — strict less-than means at-radius is no-op. Good.
  - very small h (numerical stability). Not exercised; Bullet would worry about this, Euler not so much.
  - Quat with zero ω (no rotation). The q_dot multiply with all-zero ω produces q_dot=0; q_new=q before normalize; n2=1; q_new unchanged. Good.
  - Quat normalization with near-zero norm. Handled by `if (n2 > 1e-12f)` fallback to identity. Bug-probe could verify but not load-bearing for Block 31.

## Expected metrics

- Self-test count: **59 → 61** (Block 31 gains 2 pass clauses; Block 30 keeps 3 clauses but relocates).
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest **159/159 + 1120/1120 SUCCESS** unchanged.
- Estimator's Linux Metal-less container: **61/61 PASS** (both Block 30 + Block 31 above the SKIP gate; both pure C++). **This is the actual fold of Estimator turn 33's WARNING.**
- Expected matrix delta: `BDD-008` stays `pending`; test-address column appended.
- Expected DECISIONS.md delta: D-038 added (pivot + Euler backend).
- Expected PLANNER.md delta: RIGID-BACKEND-PORTABILITY entry text reads "as of B-2′ — both Null + Euler backends live; Bullet deferred to B-2.1."
- Expected PROJECT_STATE.md delta: covered in earlier planning pass; Generator may need a small note about the Bullet → Euler pivot in the next-milestone section.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTE items:
  - (i) The Bullet → Euler pivot itself — design-doc deviation, well-documented but worth flagging.
  - (ii) Quat math duplicated between main.cpp helpers and the Euler backend header (PARALLEL-IMPL-LOCKSTEP item).
  - (iii) Ground-plane is Sphere-only (Box doesn't bounce). Documented limitation.
  - (iv) `applyImpulse == applyForce` (both treat impulse as body-center impulse) — physically wrong for body-with-inertia but acceptable for the test surface.
  - (v) `removeBody`'s slot-leak — taste-level.
  - **WARNING** would land if Block 31 FAILs OR Block 30 relocation breaks Block 1-29 OR RIGID-BACKEND-PORTABILITY signatures drift.
  - **BLOCK** if Block 31's pass clauses ship without bug-probe verification OR pre-existing PASS count regresses.
