#ifndef YSIM_NULL_RIGID_PHYSICS_BACKEND_HPP
#define YSIM_NULL_RIGID_PHYSICS_BACKEND_HPP

#include <vector>
#include "Quat.hpp"
#include "RigidPhysicsTypes.hpp"
#include "tinym.hpp"

// Null `RigidPhysicsBackend` — satisfies the 12-method contract per
// docs/design/rigid_physics_backend.md but performs no physics. Kinematic
// by design: `step()` is a no-op, all `apply*` / `set*Velocity` are no-ops,
// gravity is stored but ignored. `addBody` snapshots the `RigidInitial`
// by value so subsequent `getPosition / getRotation / getLinearVelocity /
// getAngularVelocity` round-trip the initial state.
//
// Out-of-range BodyHandle returns sentinel zero vectors / identity quat;
// no `exit()` / `abort()` / release `assert()` (CM-012: utility helpers
// do not make process-lifetime decisions; caller decides fatality).

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
        return static_cast<BodyHandle>(initials_.size() - 1);
    }

    void removeBody(BodyHandle /*handle*/) {
        // Null backend doesn't track holes; future backends may.
    }

    void step(float /*h*/, int32_t /*substeps*/) {
        // Kinematic null backend; no integration.
    }

    tinym::vec3 getPosition(BodyHandle handle) const {
        if (handle < 0 || handle >= static_cast<BodyHandle>(initials_.size()))
            return tinym::vec3{};
        return initials_[handle].position;
    }

    ::Quat getRotation(BodyHandle handle) const {
        if (handle < 0 || handle >= static_cast<BodyHandle>(initials_.size()))
            return ::Quat{};  // identity {w=1, x=0, y=0, z=0}
        return initials_[handle].rotation;
    }

    tinym::vec3 getLinearVelocity(BodyHandle handle) const {
        if (handle < 0 || handle >= static_cast<BodyHandle>(initials_.size()))
            return tinym::vec3{};
        return initials_[handle].linear_velocity;
    }

    tinym::vec3 getAngularVelocity(BodyHandle handle) const {
        if (handle < 0 || handle >= static_cast<BodyHandle>(initials_.size()))
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
