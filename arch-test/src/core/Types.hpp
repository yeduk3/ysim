#pragma once
#include "backend/Backend.hpp"
#include "backend/VectorBase.hpp"

#include "tinym.hpp"
#include "Quat.hpp"

#include <cstdint>

// Behavior/shape tags. APPEND only — values are GPU-visible (serialized by
// index into mesh buffers). See DECISIONS C4.
enum struct BehaviorType : Index {
    TriangularCloth, FastGridCloth, Elastic, Rigid, Float, Fluid, Generator, Kinematic,
};
enum struct ShapeType : Index { Mesh, Plane, Sphere, Cube, Cylinder };
enum struct PlaneDirection : Index { XYPlane, YZPlane, XZPlane };

struct alignas(8) IndexPair {
    union {
        struct { Index query, target; };
        struct { Index point, triangle; };
        struct { Index edge1, edge2; };
    };
    bool operator<(const IndexPair& o) const {
        if (query == o.query) return target < o.target;
        return query < o.query;
    }
};

// Host mirrors of common_types.metalh — byte-identical (setBytes/setBuffer
// ABI). static_asserts below lock the contract (DECISIONS C4).
struct alignas(32) BroadCollision {
    IndexPair indexPair, objPair, behaviorPair, shapePair;
};
struct NarrowCollision {
    IndexPair indexPair, objPair;
    tinym::vec4 collisionNormalAndDistance;
    IndexPair behaviorPair, shapePair;
};
struct alignas(16) AnalyticShape {
    tinym::vec4 centerRadius, halfExtHeight, rotQuat, prevCenterPad;
    uint32_t shapeType, objId, behaviorType, flags;
};

static_assert(sizeof(IndexPair) == 8);
static_assert(sizeof(BroadCollision) == 32);
static_assert(sizeof(NarrowCollision) == 48);
static_assert(sizeof(AnalyticShape) == 80);

struct FixedVertex { uint32_t vid; tinym::vec3 pos; };
struct ReferencePointConstraint { IndexPair objPair, vertexPair; };

struct Material {
    tinym::vec3 baseColor = tinym::vec3(1.0f);
    float metallic = 0.0f, roughness = 0.5f, specularWeight = 1.0f;
    tinym::vec3 emissionColor = tinym::vec3(0.0f);
};

struct SceneEnvironment {
    tinym::vec3 gravity = tinym::vec3(0.0f, -9.81f, 0.0f);
    tinym::vec3 wind    = tinym::vec3(0.0f, 0.0f, 0.0f);
    tinym::vec3 lightColor = tinym::vec3(1.0f);
    float lightIntensity = 1.6f;
    tinym::vec3 backgroundColor = tinym::vec3(0.886f, 0.906f, 0.922f);
};
