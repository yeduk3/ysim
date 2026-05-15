#include "BulletRigidPhysicsBackend.hpp"

#include <btBulletDynamicsCommon.h>

namespace ysim::physics {

BulletRigidPhysicsBackend::BulletRigidPhysicsBackend() = default;

BulletRigidPhysicsBackend::~BulletRigidPhysicsBackend() {
    if (initialized_) shutdown();
}

bool BulletRigidPhysicsBackend::initialize(tinym::vec3 gravity) {
    if (initialized_) shutdown();
    collisionConfig_ = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher_      = std::make_unique<btCollisionDispatcher>(collisionConfig_.get());
    broadphase_      = std::make_unique<btDbvtBroadphase>();
    solver_          = std::make_unique<btSequentialImpulseConstraintSolver>();
    world_           = std::make_unique<btDiscreteDynamicsWorld>(
        dispatcher_.get(), broadphase_.get(), solver_.get(), collisionConfig_.get());
    world_->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
    initialized_ = true;
    return true;
}

void BulletRigidPhysicsBackend::shutdown() {
    if (world_) {
        for (auto& b : bodies_) {
            if (b.body) world_->removeRigidBody(b.body.get());
        }
    }
    bodies_.clear();
    // Reverse-order reset (world first, then solver, broadphase, dispatcher,
    // config). std::unique_ptr destructors handle deletion.
    world_.reset();
    solver_.reset();
    broadphase_.reset();
    dispatcher_.reset();
    collisionConfig_.reset();
    initialized_ = false;
}

BodyHandle BulletRigidPhysicsBackend::addBody(const RigidInitial& initial) {
    if (!initialized_) return kInvalidBodyHandle;

    std::unique_ptr<btCollisionShape> shape;
    switch (initial.shape.type) {
    case RigidShapeType::Box:
        shape = std::make_unique<btBoxShape>(btVector3(
            initial.shape.half_extents.x,
            initial.shape.half_extents.y,
            initial.shape.half_extents.z));
        break;
    case RigidShapeType::Sphere:
        shape = std::make_unique<btSphereShape>(initial.shape.half_extents.x);
        break;
    case RigidShapeType::Plane:
        shape = std::make_unique<btStaticPlaneShape>(
            btVector3(initial.shape.normal.x,
                      initial.shape.normal.y,
                      initial.shape.normal.z),
            initial.shape.half_extents.y);
        break;
    case RigidShapeType::ConvexMesh:
    case RigidShapeType::StaticMesh:
        // Stub — caller never reaches these in current ysim paths.
        return kInvalidBodyHandle;
    }

    btTransform startTransform;
    startTransform.setOrigin(btVector3(
        initial.position.x, initial.position.y, initial.position.z));
    startTransform.setRotation(btQuaternion(
        initial.rotation.x, initial.rotation.y,
        initial.rotation.z, initial.rotation.w));

    auto motionState = std::make_unique<btDefaultMotionState>(startTransform);

    btVector3 inertia(0, 0, 0);
    btScalar mass = initial.mass;
    if (mass > 0.0f) shape->calculateLocalInertia(mass, inertia);

    btRigidBody::btRigidBodyConstructionInfo info(
        mass, motionState.get(), shape.get(), inertia);
    info.m_friction    = initial.friction;
    info.m_restitution = initial.restitution;

    auto body = std::make_unique<btRigidBody>(info);
    if (mass > 0.0f) {
        body->setLinearVelocity(btVector3(
            initial.linear_velocity.x,
            initial.linear_velocity.y,
            initial.linear_velocity.z));
        body->setAngularVelocity(btVector3(
            initial.angular_velocity.x,
            initial.angular_velocity.y,
            initial.angular_velocity.z));
    }

    world_->addRigidBody(body.get());

    BulletBody slot;
    slot.shape       = std::move(shape);
    slot.motionState = std::move(motionState);
    slot.body        = std::move(body);
    bodies_.push_back(std::move(slot));
    return static_cast<BodyHandle>(bodies_.size() - 1);
}

void BulletRigidPhysicsBackend::removeBody(BodyHandle handle) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (world_ && slot.body) world_->removeRigidBody(slot.body.get());
    slot.body.reset();
    slot.motionState.reset();
    slot.shape.reset();
}

void BulletRigidPhysicsBackend::step(float h, int32_t /*substeps*/) {
    if (!initialized_ || !world_) return;
    // (h, 1, h) — exactly one substep per call with internal timestep h.
    world_->stepSimulation(h, 1, h);
}

static inline tinym::vec3 toVec3(const btVector3& v) {
    return tinym::vec3((float)v.x(), (float)v.y(), (float)v.z());
}

tinym::vec3 BulletRigidPhysicsBackend::getPosition(BodyHandle handle) const {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return tinym::vec3{};
    const auto& slot = bodies_[handle];
    if (!slot.body) return tinym::vec3{};
    return toVec3(slot.body->getWorldTransform().getOrigin());
}

::Quat BulletRigidPhysicsBackend::getRotation(BodyHandle handle) const {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return ::Quat{};
    const auto& slot = bodies_[handle];
    if (!slot.body) return ::Quat{};
    const btQuaternion bq = slot.body->getWorldTransform().getRotation();
    return ::Quat{(float)bq.w(), (float)bq.x(), (float)bq.y(), (float)bq.z()};
}

tinym::vec3 BulletRigidPhysicsBackend::getLinearVelocity(BodyHandle handle) const {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return tinym::vec3{};
    const auto& slot = bodies_[handle];
    if (!slot.body) return tinym::vec3{};
    return toVec3(slot.body->getLinearVelocity());
}

tinym::vec3 BulletRigidPhysicsBackend::getAngularVelocity(BodyHandle handle) const {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return tinym::vec3{};
    const auto& slot = bodies_[handle];
    if (!slot.body) return tinym::vec3{};
    return toVec3(slot.body->getAngularVelocity());
}

void BulletRigidPhysicsBackend::applyForce(BodyHandle handle,
                                           tinym::vec3 force_world,
                                           tinym::vec3 at_world_point) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (!slot.body) return;
    // Convert world-space application point to local-space via current transform.
    const btTransform& tr = slot.body->getWorldTransform();
    const btVector3 rel = btVector3(at_world_point.x, at_world_point.y,
                                    at_world_point.z) - tr.getOrigin();
    slot.body->applyForce(btVector3(force_world.x, force_world.y, force_world.z), rel);
}

void BulletRigidPhysicsBackend::applyImpulse(BodyHandle handle,
                                             tinym::vec3 impulse_world,
                                             tinym::vec3 at_world_point) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (!slot.body) return;
    const btTransform& tr = slot.body->getWorldTransform();
    const btVector3 rel = btVector3(at_world_point.x, at_world_point.y,
                                    at_world_point.z) - tr.getOrigin();
    slot.body->applyImpulse(btVector3(impulse_world.x, impulse_world.y,
                                      impulse_world.z), rel);
}

void BulletRigidPhysicsBackend::setLinearVelocity(BodyHandle handle, tinym::vec3 v) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (!slot.body) return;
    slot.body->setLinearVelocity(btVector3(v.x, v.y, v.z));
}

void BulletRigidPhysicsBackend::setAngularVelocity(BodyHandle handle, tinym::vec3 w) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (!slot.body) return;
    slot.body->setAngularVelocity(btVector3(w.x, w.y, w.z));
}

void BulletRigidPhysicsBackend::setGravity(tinym::vec3 gravity) {
    if (world_) world_->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}

void BulletRigidPhysicsBackend::setBodyGravity(BodyHandle handle, tinym::vec3 gravity) {
    if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
    auto& slot = bodies_[handle];
    if (!slot.body) return;
    slot.body->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}

const char* BulletRigidPhysicsBackend::backendName() const {
    return "Bullet";
}

}  // namespace ysim::physics
