#ifndef YSIM_BULLET_RIGID_PHYSICS_BACKEND_HPP
#define YSIM_BULLET_RIGID_PHYSICS_BACKEND_HPP

#include <memory>
#include <vector>
#include "Quat.hpp"
#include "RigidPhysicsTypes.hpp"
#include "tinym.hpp"

// D-040: Bullet 3.25 backend implementing the D-037 12-method contract.
// Vendored at third_party/bullet3 (CMake sub-project). Forward-declared
// Bullet types here so the heavy btBulletDynamicsCommon.h umbrella header
// only enters the BulletRigidPhysicsBackend.cpp TU — keeps the rest of
// the project's compile time stable. RIGID-BACKEND-PORTABILITY (D-037)
// governs the contract surface (Null + Euler + Bullet all parallel).
//
// Quat marshalling at the boundary: ysim's ::Quat{w,x,y,z} ↔ Bullet's
// btQuaternion(x,y,z,w) — opposite scalar position.
//
// Shape coverage: Box + Sphere + Plane (used by Block 32/33). ConvexMesh
// and StaticMesh return kInvalidBodyHandle (stub; can land in a future
// slice if a user-facing path exposes them).
//
// CM-012 discipline: no exit/abort. Out-of-range BodyHandle returns
// sentinel zero / identity (matches Null + Euler patterns).

class btDiscreteDynamicsWorld;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btCollisionShape;
class btDefaultMotionState;
class btRigidBody;

namespace ysim::physics {

class BulletRigidPhysicsBackend {
public:
    BulletRigidPhysicsBackend();
    ~BulletRigidPhysicsBackend();

    bool initialize(tinym::vec3 gravity);
    void shutdown();

    BodyHandle addBody(const RigidInitial& initial);
    void removeBody(BodyHandle handle);

    void step(float h, int32_t substeps);

    tinym::vec3 getPosition(BodyHandle handle) const;
    ::Quat      getRotation(BodyHandle handle) const;
    tinym::vec3 getLinearVelocity(BodyHandle handle) const;
    tinym::vec3 getAngularVelocity(BodyHandle handle) const;

    void applyForce(BodyHandle handle, tinym::vec3 force_world,
                    tinym::vec3 at_world_point);
    void applyImpulse(BodyHandle handle, tinym::vec3 impulse_world,
                      tinym::vec3 at_world_point);
    void setLinearVelocity(BodyHandle handle, tinym::vec3 v);
    void setAngularVelocity(BodyHandle handle, tinym::vec3 w);
    // Teleport the body's centre of mass to `p`, keeping its CURRENT
    // rotation. Used by the PBD cloth→rigid coupling writeback: the cloth
    // solver accumulates a positional correction for the body over a frame
    // and pushes it here (position-based, so it is a position write, not an
    // impulse). Also wakes the body — a deactivated (sleeping) island is
    // skipped by the next stepSimulation and the write would be ignored.
    // Out-of-range handles are no-ops (CM-012).
    void setPosition(BodyHandle handle, tinym::vec3 p);

    void setGravity(tinym::vec3 gravity);
    // Per-body gravity override. Used by the per-mesh Apply Gravity
    // toggle so a Rigid floor / ceiling can opt out of Bullet's global
    // gravity without affecting the rest of the scene. Out-of-range
    // handles are no-ops (CM-012).
    void setBodyGravity(BodyHandle handle, tinym::vec3 gravity);
    const char* backendName() const;

private:
    // Per-body owned state. Declaration order is destruction-order-correct:
    // ~body() runs before ~motionState() before ~shape(). Bullet's
    // documented requirement is "body destroyed first" so the body's
    // dtor can dereference shape safely.
    struct BulletBody {
        std::unique_ptr<btCollisionShape>     shape;
        std::unique_ptr<btDefaultMotionState> motionState;
        std::unique_ptr<btRigidBody>          body;
    };

    std::unique_ptr<btDefaultCollisionConfiguration>     collisionConfig_;
    std::unique_ptr<btCollisionDispatcher>               dispatcher_;
    std::unique_ptr<btBroadphaseInterface>               broadphase_;
    std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
    std::unique_ptr<btDiscreteDynamicsWorld>             world_;
    std::vector<BulletBody> bodies_;
    bool initialized_ = false;
};

}  // namespace ysim::physics

#endif
