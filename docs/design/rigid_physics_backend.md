# Design — `RigidPhysicsBackend` template-based contract

> Status: Design (pre-implementation). Author: human-Planner conversation 2026-05-13. First implementing slice: B-1 (interface + null backend). Bullet backend: B-2. Wired into `Rigid` behavior tag: B-3.
>
> Resolves Q4 (rigid backend default: Bullet first via this contract). Closes BDD-008 in stages.

## Why this is a contract, not a virtual interface

Same rationale as `MeshAnimationWriter`: ysim is C++17, template-parameterizes hot paths, and the user wants compile-time efficiency on the per-step rigid-body update. Virtual dispatch from `Simulator::step` into the rigid backend would add overhead per step per body; template instantiation keeps it monomorphic.

The contract is a duck-typed C++17 set of methods. Concepts are not used; missing methods produce template-instantiation errors at the call site.

## The contract

A type `B` is a valid `RigidPhysicsBackend` if it provides:

```cpp
struct RigidPhysicsBackendContract {  // Documentation-only struct; not compiled.
    // Lifecycle.
    bool initialize(tinym::vec3 gravity);
    void shutdown();

    // Body management. Handles are opaque; backend defines storage.
    ysim::physics::BodyHandle addBody(const ysim::physics::RigidInitial& initial);
    void removeBody(ysim::physics::BodyHandle handle);

    // Step the world. Caller passes the same h that cloth uses so the
    // pipelines stay synchronized at frame boundaries. `substeps` is a
    // hint; backend may honor or use its own.
    void step(float h, int32_t substeps);

    // Post-step state query. Backend reports the position/rotation
    // observed at the end of the most recent step() call.
    tinym::vec3 getPosition(ysim::physics::BodyHandle) const;
    ::Quat getRotation(ysim::physics::BodyHandle) const;
    tinym::vec3 getLinearVelocity(ysim::physics::BodyHandle) const;
    tinym::vec3 getAngularVelocity(ysim::physics::BodyHandle) const;

    // External forces / impulses; effect on next step().
    void applyForce(ysim::physics::BodyHandle, tinym::vec3 force_world,
                    tinym::vec3 at_world_point);
    void applyImpulse(ysim::physics::BodyHandle, tinym::vec3 impulse_world,
                      tinym::vec3 at_world_point);
    void setLinearVelocity(ysim::physics::BodyHandle, tinym::vec3);
    void setAngularVelocity(ysim::physics::BodyHandle, tinym::vec3);

    // Runtime gravity change (FR-011).
    void setGravity(tinym::vec3 gravity);

    const char* backendName() const;
};
```

## Quat handling at the boundary

Per the user directive on 2026-05-13: **each framework uses its own native quaternion type internally; conversions happen at the backend's boundary.** The contract takes/returns ysim's `::Quat` (bare struct at `src/main.cpp:1554`, `{w, x, y, z}` layout). Inside the Bullet backend, the internal storage is `btQuaternion`; conversion is a 4-float copy at the API boundary:

```cpp
// Inside BulletRigidPhysicsBackend
::Quat getRotation(BodyHandle h) const {
    const btQuaternion& bt_q = bodies_[h]->getOrientation();
    return ::Quat{bt_q.w(), bt_q.x(), bt_q.y(), bt_q.z()};
}

BodyHandle addBody(const RigidInitial& init) {
    btQuaternion bt_q(init.rotation.x, init.rotation.y, init.rotation.z, init.rotation.w);
    // ... btTransform construction with bt_q
}
```

Note Bullet's `btQuaternion` constructor takes `(x, y, z, w)` order — opposite from ysim's `(w, x, y, z)` — so the conversion at the boundary swaps. This is the only quat-marshalling discipline; the contract surface stays uniform.

Future Jolt backend would do the same conversion against Jolt's `Quat` type (`{x, y, z, w}` storage with `(x, y, z, w)` constructor — same as Bullet). Future custom backend uses ysim's `::Quat` directly with zero conversion.

## Shared POD types

These live in `include/RigidPhysicsTypes.hpp` and are included by every backend AND by the Simulator:

```cpp
namespace ysim::physics {

enum class RigidShapeType { Box, Sphere, Plane, ConvexMesh, StaticMesh };

struct RigidShape {
    RigidShapeType type;
    // Box: half_extents = {hx, hy, hz}.
    // Sphere: half_extents.x = radius; .y/.z unused.
    // Plane:  normal direction; .y of half_extents = distance from origin.
    // ConvexMesh / StaticMesh: caller provides vertex+index buffers;
    //                          backend snapshots in addBody.
    tinym::vec3 half_extents;
    tinym::vec3 normal;
    const float* mesh_vertex_data = nullptr;
    int32_t mesh_vertex_count = 0;
    const uint32_t* mesh_index_data = nullptr;
    int32_t mesh_index_count = 0;
};

struct RigidInitial {
    tinym::vec3 position;
    ::Quat rotation = {1.0f, 0.0f, 0.0f, 0.0f};
    tinym::vec3 linear_velocity = {};
    tinym::vec3 angular_velocity = {};
    float mass = 1.0f;          // 0 → static body
    float friction = 0.5f;
    float restitution = 0.0f;
    RigidShape shape;
};

using BodyHandle = int32_t;     // backend-defined opaque; -1 = invalid

}  // namespace ysim::physics
```

## Simulator integration pattern

The `Simulator` template gains a fourth parameter for the rigid backend, defaulting to a null backend so existing code paths that don't use Rigid bodies don't have to specify:

```cpp
template <typename BE, typename PR, typename SystemT,
          typename RigidBackend = NullRigidPhysicsBackend>
class Simulator {
    RigidBackend rigid_;
    // ...
public:
    void update() {
        // Cloth path — existing.
        // ...
        // Rigid path — call into the rigid backend at the frame
        // boundary. The compiler monomorphizes; no virtual dispatch.
        rigid_.step(system_.h, system_.subSteps);
        // After step, refresh state.x of Rigid-tagged meshes from the
        // backend's queried position+rotation.
        for (auto& mesh : Scene<BE, PR>::meshes) {
            if (mesh.behaviorType == BehaviorType::Rigid) {
                auto pos = rigid_.getPosition(mesh.rigidBodyHandle);
                auto rot = rigid_.getRotation(mesh.rigidBodyHandle);
                applyTransformToStateX(mesh, pos, rot);
            }
        }
    }
};
```

`mesh.rigidBodyHandle` is a new field on `GeneralMesh` set when `addRigidBody` is called. Float / cloth meshes leave it as -1 (sentinel).

## Runtime backend selection

Same `std::variant` pattern as the mesh-cache writer. At Simulator construction time, a top-level switch picks the variant alternative; downstream calls go through `std::visit`. Inside the visit lambda, the rigid backend is monomorphic; the inner `step` and `getPosition` calls are inlinable.

For builds that want compile-time-only selection (smaller binary, single backend), the variant goes away and the chosen backend's type is hardcoded in `using ChosenRigidBackend = BulletRigidPhysicsBackend;`. CMake feature flag controls which path. Default: variant (allows CLI flag for backend choice — useful for benchmarking and harness coverage).

## Stage B-1 — Null backend

`NullRigidPhysicsBackend` is a no-op that:
- Returns identity rotation and zero velocity for any handle.
- Returns the position passed at `addBody` time (kinematic; doesn't fall under gravity).
- Has trivial implementations so the Simulator wiring can ship without Bullet vendored.

**Harness verification** (Stage B-1 self-test block):
```cpp
NullRigidPhysicsBackend backend;
backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));
RigidInitial init;
init.position = tinym::vec3(0.0f, 5.0f, 0.0f);
init.shape.type = RigidShapeType::Sphere;
init.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius = 0.5
BodyHandle h = backend.addBody(init);
backend.step(1.0f / 60.0f, 1);
auto pos = backend.getPosition(h);
// Null backend: position stays at (0, 5, 0); rotation = identity.
assert(std::abs(pos.x) < 1e-5 && std::abs(pos.y - 5.0f) < 1e-5);
pass("D-NNN / NullRigidPhysicsBackend round-trips initial state");
```

Self-test count after B-1: 52 → 53.

## Stage B-2 — Bullet backend

**Library**: Bullet 3.25, vendored under `include/bullet3/`. Only `BulletDynamics` + `BulletCollision` + `LinearMath` modules needed; ~50 MB source. CMake sub-project. zlib license.

**Shape translation**:

| `RigidShape::type` | Bullet shape | Notes |
|---|---|---|
| `Box` | `btBoxShape(btVector3(hx, hy, hz))` | Half-extents direct |
| `Sphere` | `btSphereShape(half_extents.x)` | Radius from .x; .y/.z unused |
| `Plane` | `btStaticPlaneShape(btVector3(normal), distance)` | Static only |
| `ConvexMesh` | `btConvexHullShape` populated from `mesh_vertex_data` | For dynamic convex bodies |
| `StaticMesh` | `btBvhTriangleMeshShape` over `btTriangleIndexVertexArray` | Static-only triangle soup (Bullet's docs warn against dynamic triangle meshes) |

**Mass / motion-state**:
- `mass = 0` → static body (`btRigidBody::CF_STATIC_OBJECT`).
- `mass > 0` → dynamic body; Bullet computes inertia tensor via `shape->calculateLocalInertia(mass, ...)`.
- Initial transform → `btDefaultMotionState(btTransform(bt_quat, bt_pos))`.

**Step semantics**: `dynamicsWorld->stepSimulation(h, substeps, fixedTimeStep)`. Bullet's `stepSimulation` internally clamps to `substeps` max sub-steps with `fixedTimeStep` as the internal timestep. Default plan: pass `h` as both `timeStep` AND `fixedTimeStep`, with `substeps = 1`. This makes Bullet step exactly once per ysim frame; the user's sim h (typically 1/60) is preserved.

**Harness verification** (Stage B-2 replaces the null backend in the existing self-test block AND adds dynamics-specific assertions):
- Same sphere at (0, 5, 0) with mass=1, falls under gravity for one step. Expected delta: `Δy ≈ -0.5 * 9.81 * (1/60)^2 ≈ -1.36 mm`. Assert within `1e-4` tolerance.
- Static plane at y=0 + dynamic sphere at (0, 0.6, 0) with mass=1, fall + contact. After ~30 steps the sphere should rest with `y ≈ 0.5` (radius). Assert `y` within `restitution * 0.5` band.

Self-test count after B-2: 53 → 55 (replace null assertion + add dynamics + add resting-contact).

## Stage B-3 — Wire into Rigid behavior tag

Existing code reserves the `Rigid` enum value in `BehaviorType` but the simulator's dispatch leaves it unhandled. B-3 adds:
- New field `int32_t rigidBodyHandle = -1` on `GeneralMesh<BE, PR>`.
- New `Simulator::addRigidBody(...)` (mirrors `addCloth` / `addCube`) that constructs a `RigidInitial` from a primitive shape + calls `rigid_.addBody(initial)` + stores the handle on the mesh.
- `Simulator::update`'s Rigid branch reads `rigid_.getPosition / getRotation` and updates `state.x` via D-021's rotateVector path.
- Persistence (toSnapshot / loadScene) snapshots `rigidBodyHandle` indirectly through the initial position + rotation (handle itself is runtime-only; re-acquired on load via `addBody`).

Self-test: existing Block 13 (BDD-010 collision detected) and Block 6 (BDD-007 cloth drape) start exercising rigid-tagged scenes. BDD-008 row promotes `pending → pass`.

## Cloth-rigid interaction (out of scope for B-1/B-2/B-3)

The current cloth pipeline runs Metal-side narrow-phase against static scene triangles (D-013 swept CCD). Integrating dynamic rigid bodies into the cloth collision pass requires either:
- Rigid backend exposes per-step triangle sampling (Bullet has `btCollisionWorld::contactTest` but it's not free).
- ysim's Metal narrow-phase queries an updated AABB tree of rigid bodies each substep.

Neither is in v1 scope. Cloth-on-rigid (BDD-007 sphere variant) was already spec-substituted to cloth-on-static-ground per the cloth-drape slice's plan. Cloth-on-dynamic-rigid is a v2 candidate.

## Standing constraints introduced by this design

- **RIGID-BACKEND-PORTABILITY** (added when B-1 ships): any change to the contract (new method, signature widening) MUST update every backend (Null at B-1; Bullet at B-2; future Jolt / custom) in the same commit. Documented in `docs/roles/PLANNER.md`'s Standing constraints subsection.

## Open sub-questions

1. **Bullet stepSimulation substeps**: ysim's `subSteps = 60` (60 sub-iterations per frame) is cloth's internal CCD substep count, not "do 60 mini-frames of duration h/60." Bullet's `substeps` parameter is the latter. Passing `substeps = 60, fixedTimeStep = h/60` would make Bullet run 60 mini-steps per call — likely overkill for rigid bodies and slows the sim. Default plan: pass `substeps = 1, fixedTimeStep = h`. Revisit if rigid bodies look unstable.
2. **Collision shape ownership**: Bullet's `btCollisionShape` instances must outlive their bodies. `BulletRigidPhysicsBackend` owns a `std::vector<std::unique_ptr<btCollisionShape>>` parallel to the bodies vector; cleared on `shutdown()`.
3. **Determinism**: Bullet is single-threaded-deterministic by default (BDD-102 single-machine determinism is preserved). Bullet's broadphase / narrowphase iterations are deterministic given the same inputs. Will verify in Stage B-2 with a two-run bit-equality probe on a falling sphere.
