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
#include <unordered_map>
#include <vector>

namespace primitive {

using Index = uint32_t;

struct Geometry {
    std::vector<float> positions;   // 3 * numPoints, [x0,y0,z0, x1,y1,z1, ...]
    std::vector<Index> facets;      // 3 * numFacets, [a,b,c, a,b,c, ...]

    // 2026-05-15 (A2 — render/physics topology split): optional secondary
    // buffers carrying an UNWELDED rendering view of the same geometry.
    // Currently populated by `cube()` only — the welded primary buffers
    // hold a single closed manifold so cloth springs cross face seams,
    // while the unwelded render buffers hold per-face vertex copies so
    // each face's flat normal is preserved (no averaging across seams).
    //
    // Layout invariants when populated:
    //   - renderPositions.size() % 3 == 0
    //   - renderFacets.size()   % 3 == 0
    //   - renderNormals.size()  == renderPositions.size()  (flat per-face)
    //   - renderToPhysics.size() == renderPositions.size() / 3,
    //         renderToPhysics[i] = welded vertex index that the unwelded
    //         vertex `i` corresponds to. Used by the resync loop to push
    //         physics state.x back into renderPositions each frame.
    //
    // Empty for sphere/grid/file initializers — the welded primary mesh is
    // already a single manifold there, so renderer + physics share x/n/facets.
    std::vector<float> renderPositions;
    std::vector<Index> renderFacets;
    std::vector<float> renderNormals;
    std::vector<Index> renderToPhysics;
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
//
// 2026-05-15: cube() now welds coincident vertices across face seams so
// the result is a single closed manifold. Pre-weld the 6 faces emitted
// independently with no shared corners/edges; switching the mesh to
// TriangularCloth produced 6 disconnected cloth patches that flew apart
// under gravity because cloth springs (built from mesh.adjacency.edges)
// could not cross face seams. The welded topology fixes that and the
// vertex-count / edge-count formulae below are the post-weld counts.
//
// Welded counts (single closed polyhedron, Euler V - E + F = 2):
//   V = 6t² + 2     (interior face verts + edge-midpoint verts + corners)
//   F = 12t²        (unchanged — same triangle layout per face)
//   E = V + F - 2 = 18t²
inline int cubeVertexCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 6 * t * t + 2;
}

inline int cubeFacetCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 12 * t * t;
}

inline int cubeEdgeCount(int tessellation) {
    int t = tessellation < 1 ? 1 : tessellation;
    return 18 * t * t;
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
        // Outward face normal = normalize(u × v). Per face, all (t+1)² verts
        // emitted below get this same normal so the unwelded render buffer
        // shades flat per face (no averaging across seams in
        // PreviewState::recomputeNormals since each unwelded vertex appears
        // only in this face's triangles).
        float nx = F.u[1] * F.v[2] - F.u[2] * F.v[1];
        float ny = F.u[2] * F.v[0] - F.u[0] * F.v[2];
        float nz = F.u[0] * F.v[1] - F.u[1] * F.v[0];
        float nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nlen > 1e-20f) { nx /= nlen; ny /= nlen; nz /= nlen; }
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
                g.renderNormals.push_back(nx);
                g.renderNormals.push_back(ny);
                g.renderNormals.push_back(nz);
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

    // Snapshot the unwelded mesh into the render-side buffers BEFORE the
    // weld pass collapses positions. Render buffers carry per-face vertex
    // copies so flat normals (already populated above) shade crisp cube
    // edges; physics buffers (g.positions / g.facets) become a single
    // closed manifold below so cloth springs can cross face seams. The
    // renderToPhysics map (populated inside the weld pass) lets the per-
    // frame resync push physics state.x back into renderPositions.
    g.renderPositions = g.positions;
    g.renderFacets    = g.facets;

    // Weld coincident vertices across face seams so the cube is a single
    // closed manifold. Without this, switching the mesh to TriangularCloth
    // splits it into 6 free-falling patches because cloth springs cannot
    // cross face seams. Quantize to 1e-5 of `size` so corners and shared
    // edge vertices generated via different basis arithmetic still
    // collapse to one key. Vertices are walked in emission order so the
    // first occurrence keeps its slot — this preserves the invariant that
    // vertex 0 is the +X face's (+h,-h,-h) corner (Block 39 in self-test).
    {
        const float quantScale = 1.0f / (size > 0.f ? size * 1e-5f : 1e-5f);
        struct Key { int64_t x, y, z; };
        struct KeyHash {
            size_t operator()(const Key& k) const noexcept {
                size_t h1 = std::hash<int64_t>{}(k.x);
                size_t h2 = std::hash<int64_t>{}(k.y);
                size_t h3 = std::hash<int64_t>{}(k.z);
                return h1 ^ (h2 << 1) ^ (h3 << 2);
            }
        };
        struct KeyEq {
            bool operator()(const Key& a, const Key& b) const noexcept {
                return a.x == b.x && a.y == b.y && a.z == b.z;
            }
        };
        auto quantize = [&](float v) -> int64_t {
            return (int64_t)std::llround((double)v * (double)quantScale);
        };

        const size_t inVerts = g.positions.size() / 3;
        std::unordered_map<Key, Index, KeyHash, KeyEq> seen;
        seen.reserve(inVerts);
        std::vector<float> outPos;
        outPos.reserve(g.positions.size());
        std::vector<Index> remap(inVerts);

        for (size_t i = 0; i < inVerts; ++i) {
            const float x = g.positions[3 * i + 0];
            const float y = g.positions[3 * i + 1];
            const float z = g.positions[3 * i + 2];
            Key k{quantize(x), quantize(y), quantize(z)};
            auto it = seen.find(k);
            if (it != seen.end()) {
                remap[i] = it->second;
            } else {
                Index newIdx = (Index)(outPos.size() / 3);
                outPos.push_back(x);
                outPos.push_back(y);
                outPos.push_back(z);
                seen.emplace(k, newIdx);
                remap[i] = newIdx;
            }
        }
        for (auto& fi : g.facets) fi = remap[fi];
        g.positions = std::move(outPos);
        g.renderToPhysics = std::move(remap);
    }

    return g;
}

}  // namespace primitive

#endif  // YSIM_PRIMITIVE_GEOMETRY_HPP
