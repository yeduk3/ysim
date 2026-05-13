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
    //                          backend snapshots in addBody (B-2 Bullet).
    tinym::vec3    half_extents      = {};
    tinym::vec3    normal            = {};
    const float*   mesh_vertex_data  = nullptr;
    int32_t        mesh_vertex_count = 0;
    const uint32_t* mesh_index_data  = nullptr;
    int32_t        mesh_index_count  = 0;
};

struct RigidInitial {
    tinym::vec3 position         = {};
    ::Quat      rotation;                  // Quat's own default is identity {w=1}.
    tinym::vec3 linear_velocity  = {};
    tinym::vec3 angular_velocity = {};
    float       mass             = 1.0f;   // 0 reserved for static body (B-2 Bullet).
    float       friction         = 0.5f;
    float       restitution      = 0.0f;
    RigidShape  shape;
};

using BodyHandle = int32_t;
constexpr BodyHandle kInvalidBodyHandle = -1;

}  // namespace ysim::physics

#endif
