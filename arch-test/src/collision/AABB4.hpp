#pragma once
#include "tinym.hpp"
#include <algorithm>
#include <limits>
#include <cstdint>

// Karras12 BVH data ABI. Byte layouts MUST match bvh.metal: AABB4(7-12),
// BVHNode(14-19), MortonNode(21-24), and radixsort.metal SortPair(5-8).
// Ported from src/main.cpp 5000-5136 + bvh.metal types. The static_asserts
// are the layout-lock — drift = silent GPU corruption (BVH_VERSIONS.md §5.2).

// Ray / RayHit — ported from main.cpp ~2740 (picking only; kept so the
// AABB4::intersect(Ray) overload compiles, NOT on the sim hot path).
struct Ray { tinym::vec3 origin, dir; };
struct RayHit { float tmin = 0.0f, tmax = 0.0f; };

// 32B. union{ (v0,v1) | (min,i0,max,i1) }. i0 overloads as numPrimitives
// scratch — bvh.metal reads sceneBox._pad0 == numPrimitives (fillMortons_Tri
// line 49, bottomUpBoxes line 517). i1 overloads as query pid/objid.
struct alignas(32) AABB4 {
    union {
        struct { tinym::vec3f1i v0, v1; };
        struct { tinym::vec3 min; int i0; tinym::vec3 max; int i1; };
    };
    AABB4() : v0(0), v1(0) {}
    AABB4(tinym::vec3_view e0, tinym::vec3_view e1)
        : min(tinym::min(e0, e1)), i0(0), max(tinym::max(e0, e1)), i1(0) {}
    AABB4(tinym::vec3_view t0, tinym::vec3_view t1, tinym::vec3_view t2)
        : min(tinym::min(t0, t1, t2)), i0(0), max(tinym::max(t0, t1, t2)), i1(0) {}
    void combine(const tinym::vec3_view& v) { min = tinym::min(v, min); max = tinym::max(v, max); }
    void combine(const AABB4& a) { min = tinym::min(min, a.min); max = tinym::max(max, a.max); }
    bool intersect(const AABB4& a) const {
        if (max.x < a.min.x || min.x > a.max.x) return false;
        if (max.y < a.min.y || min.y > a.max.y) return false;
        if (max.z < a.min.z || min.z > a.max.z) return false;
        return true;
    }
    // Ray slab test — verbatim main.cpp 5031-5060. Picking-only.
    bool intersect(const Ray& ray, RayHit& hit) const {
        float tmin = 0.0f;
        float tmax = std::numeric_limits<float>::infinity();
        const float eps = 1e-6f;
        for (int axis = 0; axis < 3; ++axis) {
            float o = ray.origin[axis];
            float d = ray.dir[axis];
            if (std::abs(d) < eps) {
                if (o < min[axis] || o > max[axis]) return false;
            } else {
                float t1 = (min[axis] - o) / d;
                float t2 = (max[axis] - o) / d;
                if (t1 > t2) std::swap(t1, t2);
                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);
                if (tmin > tmax) return false;
            }
        }
        hit.tmin = tmin;
        hit.tmax = tmax;
        return tmax >= 0.0f;
    }
};
static_assert(sizeof(AABB4) == 32);

// 8B. radix-sort layout lock (matches bvh.metal MortonNode AND radixsort.metal
// SortPair {key,value} — code<->key, index<->value).
struct alignas(8) MortonNode { uint32_t code; uint32_t index; };
static_assert(sizeof(MortonNode) == 8);

// 32B leaf sentinel: childA==-1 => leaf, childB == primitive(facet) id.
// union{ AABB4 | (min,childA,max,childB) }. Matches bvh.metal BVHNode.
// leaf slot = N + sortedId - 1 (kernel convention, buildLeaf_Tri 278).
struct BVHNode {
    union {
        AABB4 aabb;
        struct alignas(32) { tinym::vec3 min; int childA; tinym::vec3 max; int childB; };
    };
    BVHNode() : aabb() {}
};
static_assert(sizeof(BVHNode) == 32);

// CPU Morton parity (bit-identical to bvh.metal 26-40 / main.cpp 5691-5704).
// Only used if parts.morton==CPU; the default GPU fillMortons_Tri is prod.
inline uint32_t expandBits(uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}
inline uint32_t mortonCode(const tinym::vec3& point) {
    tinym::vec3 p = tinym::min(tinym::max(point * 1024.f, tinym::vec3(0.f)), tinym::vec3(1023.f));
    return expandBits((uint32_t)p.x) * 4 + expandBits((uint32_t)p.y) * 2 + expandBits((uint32_t)p.z);
}
