#ifndef YSIM_PRIMITIVE_GEOMETRY_HPP
#define YSIM_PRIMITIVE_GEOMETRY_HPP

// CPU-pure geometry generators for v1 primitive shapes (BDD-001).
// Header-only and free of Metal / GLFW / tinym dependencies so the test
// harness can exercise it without a GPU device (D-002, D-003 amendment).
//
// The runtime-side initializers (MeshSphereInitializer, MeshCubeInitializer
// in src/main.cpp) wrap these generators and feed the result into
// MeshState<BE,PR>::x and MeshAdjacency<BE,PR>::facets.

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace primitive {

using Index = uint32_t;

struct Geometry {
    std::vector<float> positions;   // 3 * numPoints, [x0,y0,z0, x1,y1,z1, ...]
    std::vector<Index> facets;      // 3 * numFacets, [a,b,c, a,b,c, ...]
};

// ---- UV sphere -------------------------------------------------------------

// Tessellation parameter doubles as longitude segment count and latitude
// segment count. Constraints: tessellation >= 3 (clamped silently). The
// generated mesh is a closed manifold with 2 pole vertices and (tess-1)*tess
// inner-ring vertices; the existing MeshAdjacencyInitializer handles this
// shape directly because every triangle is non-degenerate.
inline int sphereVertexCount(int tessellation) {
    int t = tessellation < 3 ? 3 : tessellation;
    return (t - 1) * t + 2;
}

inline int sphereFacetCount(int tessellation) {
    int t = tessellation < 3 ? 3 : tessellation;
    return 2 * t * (t - 1);
}

// Closed manifold ⇒ E = V + F - 2 (Euler) = 3 * t * (t - 1).
inline int sphereEdgeCount(int tessellation) {
    int t = tessellation < 3 ? 3 : tessellation;
    return 3 * t * (t - 1);
}

inline Geometry sphere(float size, int tessellation,
                        std::array<float, 3> center = {0.f, 0.f, 0.f}) {
    int t = tessellation < 3 ? 3 : tessellation;
    const int lat = t;
    const int lon = t;
    const float r = size * 0.5f;
    const float pi = 3.14159265358979323846f;

    Geometry g;
    g.positions.reserve(3 * sphereVertexCount(t));
    g.facets.reserve(3 * sphereFacetCount(t));

    auto push = [&](float x, float y, float z) {
        g.positions.push_back(x + center[0]);
        g.positions.push_back(y + center[1]);
        g.positions.push_back(z + center[2]);
    };

    // North pole (index 0).
    push(0.f, r, 0.f);
    // Inner rings: lat-1 rings × lon vertices each.
    for (int i = 1; i < lat; ++i) {
        float theta = pi * (float)i / (float)lat;  // 0..pi
        float sy = std::cos(theta) * r;
        float sr = std::sin(theta) * r;
        for (int j = 0; j < lon; ++j) {
            float phi = 2.f * pi * (float)j / (float)lon;
            push(sr * std::cos(phi), sy, sr * std::sin(phi));
        }
    }
    // South pole (index = (lat-1)*lon + 1).
    push(0.f, -r, 0.f);

    const Index north = 0;
    const Index south = (Index)((lat - 1) * lon + 1);
    auto inner = [&](int i, int j) -> Index {
        return (Index)(1 + (i - 1) * lon + (j % lon));
    };

    auto pushTri = [&](Index a, Index b, Index c) {
        g.facets.push_back(a);
        g.facets.push_back(b);
        g.facets.push_back(c);
    };

    // Top cap (lon triangles).
    for (int j = 0; j < lon; ++j) {
        pushTri(north, inner(1, j), inner(1, j + 1));
    }
    // Middle bands ((lat-2) × lon × 2 triangles).
    for (int i = 1; i < lat - 1; ++i) {
        for (int j = 0; j < lon; ++j) {
            Index a = inner(i, j);
            Index b = inner(i, j + 1);
            Index c = inner(i + 1, j);
            Index d = inner(i + 1, j + 1);
            pushTri(a, c, d);
            pushTri(a, d, b);
        }
    }
    // Bottom cap (lon triangles).
    for (int j = 0; j < lon; ++j) {
        pushTri(south, inner(lat - 1, j + 1), inner(lat - 1, j));
    }

    return g;
}

// ---- Cube ------------------------------------------------------------------

// Tessellation = number of cells per face edge. tessellation >= 1.
// Faces are emitted independently (no vertex sharing across faces) — the
// resulting mesh is a disconnected manifold, which is fine for v1: the
// default Float behavior never runs cloth physics over the surface, and
// the BVH/collision pipeline treats triangles uniformly regardless of
// connectivity.
inline int cubeVertexCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 6 * (t + 1) * (t + 1);
}

inline int cubeFacetCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 12 * t * t;
}

// Per face (planar grid disk): V - E + F = 1 ⇒ E = V + F - 1
//                              = (t+1)^2 + 2*t^2 - 1 = 3*t^2 + 2*t.
// Six independent faces ⇒ E_total = 6 * (3*t^2 + 2*t).
inline int cubeEdgeCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 6 * (3 * t * t + 2 * t);
}

inline Geometry cube(float size, int tessellation,
                      std::array<float, 3> center = {0.f, 0.f, 0.f}) {
    int t = tessellation < 1 ? 1 : tessellation;
    const float h = size * 0.5f;

    Geometry g;
    g.positions.reserve(3 * cubeVertexCount(t));
    g.facets.reserve(3 * cubeFacetCount(t));

    // Each face: anchor + two basis axes spanning the face plane.
    // Order is fixed; tests rely on it for the per-face coordinate check.
    struct Face {
        std::array<float, 3> origin;  // corner of the face
        std::array<float, 3> u;       // first axis (full edge length)
        std::array<float, 3> v;       // second axis (full edge length)
    };
    const Face faces[6] = {
        // +X face: at x=+h, spans y and z.
        {{ h, -h, -h}, {0,  size, 0},      {0, 0,  size}},
        // -X face: at x=-h, spans y and -z (so winding stays outward).
        {{-h, -h,  h}, {0,  size, 0},      {0, 0, -size}},
        // +Y face: at y=+h, spans x and -z.
        {{-h,  h,  h}, { size, 0, 0},      {0, 0, -size}},
        // -Y face: at y=-h, spans x and +z.
        {{-h, -h, -h}, { size, 0, 0},      {0, 0,  size}},
        // +Z face: at z=+h, spans x and y.
        {{-h, -h,  h}, { size, 0, 0},      {0,  size, 0}},
        // -Z face: at z=-h, spans -x and y.
        {{ h, -h, -h}, {-size, 0, 0},      {0,  size, 0}},
    };

    auto pushTri = [&](Index a, Index b, Index c) {
        g.facets.push_back(a);
        g.facets.push_back(b);
        g.facets.push_back(c);
    };

    for (int f = 0; f < 6; ++f) {
        const auto& F = faces[f];
        Index base = (Index)(g.positions.size() / 3);
        // (t+1) × (t+1) vertices on this face.
        for (int i = 0; i <= t; ++i) {
            float vi = (float)i / (float)t;
            for (int j = 0; j <= t; ++j) {
                float uj = (float)j / (float)t;
                float x = F.origin[0] + uj * F.u[0] + vi * F.v[0] + center[0];
                float y = F.origin[1] + uj * F.u[1] + vi * F.v[1] + center[1];
                float z = F.origin[2] + uj * F.u[2] + vi * F.v[2] + center[2];
                g.positions.push_back(x);
                g.positions.push_back(y);
                g.positions.push_back(z);
            }
        }
        // 2 * t * t triangles on this face.
        for (int i = 0; i < t; ++i) {
            for (int j = 0; j < t; ++j) {
                Index p00 = base + (Index)(i * (t + 1) + j);
                Index p10 = p00 + 1;
                Index p01 = p00 + (t + 1);
                Index p11 = p01 + 1;
                pushTri(p00, p01, p11);
                pushTri(p00, p11, p10);
            }
        }
    }

    return g;
}

}  // namespace primitive

#endif  // YSIM_PRIMITIVE_GEOMETRY_HPP
