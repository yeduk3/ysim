#ifndef YSIM_PREVIEW_STATE_HPP
#define YSIM_PREVIEW_STATE_HPP

// D-042 R-1 (2026-05-14): per-mesh "preview" state — heap-owned vertex
// data populated by the initializer at addGeneralMesh time. Decoupled
// from Simulator::packedMeshData so MeshGL can bind to a stable pointer
// IMMEDIATELY at mesh add-time (before Scene::pack runs).
//
// Pre-R-1 lifecycle: initializer wrote into packedMeshData sub-views
// via Scene::pack. MeshGL had to wait for pack — pre-pack the mesh was
// invisible. R-1 introduces PreviewState as a parallel symbol; later
// slices (R-2+) point MeshGL at PreviewState and sync packed→preview
// every Simulator::update.
//
// Memory: std::vector — heap-owned, NOT pool-backed. Pool-reset (D-041)
// cannot invalidate. Lifetime is tied to the Scene's request entry
// (removed in Simulator::removeMesh).

#include <cstddef>
#include <cstdint>
#include <vector>

template <typename PR>
struct PreviewState {
    // 3 floats per vertex (x, y, z), tightly packed. PHYSICS topology —
    // for primitives whose render and physics meshes differ (currently
    // only `cube`, which welds positions across face seams) this is the
    // welded (single-manifold) buffer that Scene::pack memcpys into
    // state.x. For sphere/grid/file primitives it doubles as the render
    // buffer too (see render*Ptr accessors below).
    std::vector<PR> x;
    // 3 floats per vertex normal (nx, ny, nz). Computed from face normals
    // averaged over each vertex's incident facets. For welded cube this
    // gives smoothed corner normals — fine for physics-side state.n
    // (currently unused by collision) but NOT what we render with; see
    // renderN below.
    std::vector<PR> n;
    // 3 uint32 per facet (v0, v1, v2). Indices into x/n (physics).
    std::vector<uint32_t> facets;

    // 2026-05-15 (A2 split): optional RENDER topology — populated by
    // primitive::cube to carry the pre-weld per-face vertex copies + flat
    // per-face normals. When non-empty, MeshGL binds to renderX/renderN/
    // renderFacets instead of x/n/facets so cube edges shade crisp.
    // `renderToPhysics[i]` is the welded vertex index for unwelded vertex
    // `i` — Simulator::update's resync uses it to push physics state.x
    // back into renderX each frame.
    //
    // For sphere/grid/file the render-side vectors stay empty; MeshGL
    // falls back to x/n/facets via the accessors below.
    std::vector<PR>       renderX;
    std::vector<PR>       renderN;
    std::vector<uint32_t> renderFacets;
    std::vector<uint32_t> renderToPhysics;

    bool   hasRender() const { return !renderX.empty(); }

    // Convenience accessors so MeshGL ctor's expected (count, ptr) tuple
    // is easy to wire. Physics accessors are unchanged; the render*Ptr
    // family returns the render-side buffer when present, otherwise falls
    // through to the physics buffer (sphere/grid/file path).
    size_t numPoints() const { return x.size() / 3; }
    size_t numFacets() const { return facets.size() / 3; }
    PR*       xPtr()        { return x.data(); }
    PR*       nPtr()        { return n.data(); }
    uint32_t* facetsPtr()   { return facets.data(); }

    size_t numRenderPoints() const { return hasRender() ? renderX.size() / 3 : numPoints(); }
    size_t numRenderFacets() const { return hasRender() ? renderFacets.size() / 3 : numFacets(); }
    PR*       renderXPtr()        { return hasRender() ? renderX.data() : x.data(); }
    PR*       renderNPtr()        { return hasRender() ? renderN.data() : n.data(); }
    uint32_t* renderFacetsPtr()   { return hasRender() ? renderFacets.data() : facets.data(); }

    // Push the current physics positions (x) through the renderToPhysics
    // map back into renderX, then recompute renderN as per-vertex face-
    // normal averages (which collapses to flat per-face shading because
    // each render vertex appears in only one face's triangles). No-op
    // when the render topology is absent.
    void resyncRenderFromPhysics() {
        if (!hasRender()) return;
        const size_t nR = renderX.size() / 3;
        for (size_t i = 0; i < nR; ++i) {
            const uint32_t pi = renderToPhysics[i];
            renderX[i * 3 + 0] = x[pi * 3 + 0];
            renderX[i * 3 + 1] = x[pi * 3 + 1];
            renderX[i * 3 + 2] = x[pi * 3 + 2];
        }
        recomputeRenderNormals();
    }

    // Recompute vertex normals from current positions + facets via
    // per-face cross-product averaged across each vertex's incident
    // facets. Handles degenerate triangles (zero area → contributes
    // zero). O(numFacets + numPoints).
    void recomputeNormals() { recomputeNormalsInto(x, facets, n); }

    // 2026-05-15 (A2 split): same algorithm applied to the render-side
    // buffers. Because the render mesh is unwelded for cube, each render
    // vertex appears in triangles from only ONE face — the averaging
    // collapses to per-face flat normals automatically.
    void recomputeRenderNormals() {
        if (!hasRender()) return;
        recomputeNormalsInto(renderX, renderFacets, renderN);
    }

private:
    static void recomputeNormalsInto(const std::vector<PR>& pos,
                                     const std::vector<uint32_t>& fac,
                                     std::vector<PR>& out) {
        if (fac.empty() || pos.empty()) return;
        const size_t nVerts = pos.size() / 3;
        const size_t nFacets = fac.size() / 3;
        out.assign(nVerts * 3, PR(0));
        for (size_t f = 0; f < nFacets; ++f) {
            const uint32_t i0 = fac[f * 3 + 0];
            const uint32_t i1 = fac[f * 3 + 1];
            const uint32_t i2 = fac[f * 3 + 2];
            if (i0 >= nVerts || i1 >= nVerts || i2 >= nVerts) continue;
            const PR ax = pos[i1 * 3 + 0] - pos[i0 * 3 + 0];
            const PR ay = pos[i1 * 3 + 1] - pos[i0 * 3 + 1];
            const PR az = pos[i1 * 3 + 2] - pos[i0 * 3 + 2];
            const PR bx = pos[i2 * 3 + 0] - pos[i0 * 3 + 0];
            const PR by = pos[i2 * 3 + 1] - pos[i0 * 3 + 1];
            const PR bz = pos[i2 * 3 + 2] - pos[i0 * 3 + 2];
            const PR cx = ay * bz - az * by;
            const PR cy = az * bx - ax * bz;
            const PR cz = ax * by - ay * bx;
            out[i0 * 3 + 0] += cx; out[i0 * 3 + 1] += cy; out[i0 * 3 + 2] += cz;
            out[i1 * 3 + 0] += cx; out[i1 * 3 + 1] += cy; out[i1 * 3 + 2] += cz;
            out[i2 * 3 + 0] += cx; out[i2 * 3 + 1] += cy; out[i2 * 3 + 2] += cz;
        }
        for (size_t v = 0; v < nVerts; ++v) {
            const PR nx_ = out[v * 3 + 0];
            const PR ny_ = out[v * 3 + 1];
            const PR nz_ = out[v * 3 + 2];
            const PR len2 = nx_ * nx_ + ny_ * ny_ + nz_ * nz_;
            if (len2 > PR(1e-20)) {
                const PR inv = PR(1) / std::sqrt(len2);
                out[v * 3 + 0] = nx_ * inv;
                out[v * 3 + 1] = ny_ * inv;
                out[v * 3 + 2] = nz_ * inv;
            }
        }
    }
};

#endif  // YSIM_PREVIEW_STATE_HPP
