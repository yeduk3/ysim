#ifndef YSIM_EULER_RIGID_PHYSICS_BACKEND_HPP
#define YSIM_EULER_RIGID_PHYSICS_BACKEND_HPP

#include <cmath>
#include <vector>
#include "Quat.hpp"
#include "RigidPhysicsTypes.hpp"
#include "tinym.hpp"

// Minimal semi-implicit Euler integrator implementing the D-037
// `RigidPhysicsBackend` contract. Supports world gravity per body;
// supports a single y=0 ground plane via radius-clamp on Sphere shapes
// only. NO general contact resolution; NO angular collision response;
// NO friction. Adequate for B-2′'s scope (gravity + resting contact);
// the canonical Bullet impl per `docs/design/rigid_physics_backend.md`
// §B-2 is deferred to a future slice (B-2.1) once the vendor permission
// is granted.
//
// PARALLEL-IMPL-LOCKSTEP: the Quat math inlined in `step()` (q_dot
// derivation + normalize) duplicates `src/main.cpp`'s `operator*` and
// `quatNormalize` (D-019). If main.cpp's Quat semantics change, this
// inline copy MUST follow in the same commit.
//
// CM-012 discipline: out-of-range BodyHandle returns sentinel zero
// vectors / identity quat; no exit() / abort() in any method.

namespace ysim::physics {

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
        // Slot-leak by design: handles are stable indices into bodies_.
        // Future slice may compact or use a free-list; not needed for
        // B-2′'s test surface.
    }

    void step(float h, int32_t /*substeps*/) {
        if (!initialized_) return;
        for (auto& b : bodies_) {
            if (b.mass <= 0.0f) continue;  // static body — no integration

            // Semi-implicit Euler: v_new = v + a*h; x_new = x + v_new*h.
            b.linear_velocity.x += gravity_.x * h;
            b.linear_velocity.y += gravity_.y * h;
            b.linear_velocity.z += gravity_.z * h;

            b.position.x += b.linear_velocity.x * h;
            b.position.y += b.linear_velocity.y * h;
            b.position.z += b.linear_velocity.z * h;

            // Angular: q_dot = 0.5 * Quat(0, ω) * q; q += q_dot * h; normalize.
            // Inline mini-Quat multiply — see PARALLEL-IMPL-LOCKSTEP note above.
            const ::Quat& q = b.rotation;
            const float wx = b.angular_velocity.x;
            const float wy = b.angular_velocity.y;
            const float wz = b.angular_velocity.z;
            // (0, wx, wy, wz) * (q.w, q.x, q.y, q.z) — Hamilton product.
            ::Quat q_dot;
            q_dot.w = 0.5f * (-wx * q.x - wy * q.y - wz * q.z);
            q_dot.x = 0.5f * ( wx * q.w + wy * q.z - wz * q.y);
            q_dot.y = 0.5f * (-wx * q.z + wy * q.w + wz * q.x);
            q_dot.z = 0.5f * ( wx * q.y - wy * q.x + wz * q.w);

            ::Quat q_new;
            q_new.w = q.w + q_dot.w * h;
            q_new.x = q.x + q_dot.x * h;
            q_new.y = q.y + q_dot.y * h;
            q_new.z = q.z + q_dot.z * h;

            const float n2 = q_new.w*q_new.w + q_new.x*q_new.x
                           + q_new.y*q_new.y + q_new.z*q_new.z;
            if (n2 > 1e-12f) {
                const float inv = 1.0f / std::sqrt(n2);
                q_new.w *= inv;
                q_new.x *= inv;
                q_new.y *= inv;
                q_new.z *= inv;
            } else {
                q_new = ::Quat{};  // identity fallback
            }
            b.rotation = q_new;

            // Ground-plane collision: Sphere only, plane at y=0 with normal up.
            // If position.y < radius, clamp + zero downward velocity (perfect
            // inelastic). Other shape types pass through (no collision in B-2′).
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
            return ::Quat{};  // identity {w=1, x=0, y=0, z=0}
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

    void applyForce(BodyHandle handle, tinym::vec3 force_world,
                    tinym::vec3 /*at_world_point*/) {
        // Body-center force: Δv = F/m. Caller-supplied at_world_point ignored
        // (no angular response in B-2′).
        if (handle < 0 || handle >= static_cast<BodyHandle>(bodies_.size())) return;
        auto& b = bodies_[handle];
        if (b.mass <= 0.0f) return;
        const float inv_m = 1.0f / b.mass;
        b.linear_velocity.x += force_world.x * inv_m;
        b.linear_velocity.y += force_world.y * inv_m;
        b.linear_velocity.z += force_world.z * inv_m;
    }

    void applyImpulse(BodyHandle handle, tinym::vec3 impulse_world,
                      tinym::vec3 at_world_point) {
        // Impulse and force collapse to the same body-center treatment in B-2′.
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
        ::Quat      rotation;             // Quat default = identity {w=1}.
        tinym::vec3 linear_velocity  = {};
        tinym::vec3 angular_velocity = {};
        float       mass             = 1.0f;
        float       friction         = 0.5f;
        float       restitution      = 0.0f;
        RigidShape  shape;
    };

    std::vector<EulerBody> bodies_;
    tinym::vec3            gravity_     = {};
    bool                   initialized_ = false;
};

}  // namespace ysim::physics

#endif
