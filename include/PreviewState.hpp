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
    // 3 floats per vertex (x, y, z), tightly packed.
    std::vector<PR> x;
    // 3 floats per vertex normal (nx, ny, nz). Computed by the
    // initializer's populatePreview from face normals averaged
    // over each vertex's incident facets. Re-computed when state.x
    // changes meaningfully (e.g., on resync or future explicit refresh).
    std::vector<PR> n;
    // 3 uint32 per facet (v0, v1, v2). Triangle indices into x/n.
    std::vector<uint32_t> facets;

    // Convenience accessors so MeshGL ctor's expected (count, ptr) tuple
    // is easy to wire.
    size_t numPoints() const { return x.size() / 3; }
    size_t numFacets() const { return facets.size() / 3; }
    PR*       xPtr()        { return x.data(); }
    PR*       nPtr()        { return n.data(); }
    uint32_t* facetsPtr()   { return facets.data(); }

    // Recompute vertex normals from current positions + facets via
    // per-face cross-product averaged across each vertex's incident
    // facets. Handles degenerate triangles (zero area → contributes
    // zero). O(numFacets + numPoints).
    void recomputeNormals() {
        if (facets.empty() || x.empty()) return;
        const size_t nVerts = numPoints();
        const size_t nFacets = numFacets();
        n.assign(nVerts * 3, PR(0));
        for (size_t f = 0; f < nFacets; ++f) {
            const uint32_t i0 = facets[f * 3 + 0];
            const uint32_t i1 = facets[f * 3 + 1];
            const uint32_t i2 = facets[f * 3 + 2];
            if (i0 >= nVerts || i1 >= nVerts || i2 >= nVerts) continue;
            const PR ax = x[i1 * 3 + 0] - x[i0 * 3 + 0];
            const PR ay = x[i1 * 3 + 1] - x[i0 * 3 + 1];
            const PR az = x[i1 * 3 + 2] - x[i0 * 3 + 2];
            const PR bx = x[i2 * 3 + 0] - x[i0 * 3 + 0];
            const PR by = x[i2 * 3 + 1] - x[i0 * 3 + 1];
            const PR bz = x[i2 * 3 + 2] - x[i0 * 3 + 2];
            const PR cx = ay * bz - az * by;
            const PR cy = az * bx - ax * bz;
            const PR cz = ax * by - ay * bx;
            n[i0 * 3 + 0] += cx; n[i0 * 3 + 1] += cy; n[i0 * 3 + 2] += cz;
            n[i1 * 3 + 0] += cx; n[i1 * 3 + 1] += cy; n[i1 * 3 + 2] += cz;
            n[i2 * 3 + 0] += cx; n[i2 * 3 + 1] += cy; n[i2 * 3 + 2] += cz;
        }
        for (size_t v = 0; v < nVerts; ++v) {
            const PR nx_ = n[v * 3 + 0];
            const PR ny_ = n[v * 3 + 1];
            const PR nz_ = n[v * 3 + 2];
            const PR len2 = nx_ * nx_ + ny_ * ny_ + nz_ * nz_;
            if (len2 > PR(1e-20)) {
                const PR inv = PR(1) / std::sqrt(len2);
                n[v * 3 + 0] = nx_ * inv;
                n[v * 3 + 1] = ny_ * inv;
                n[v * 3 + 2] = nz_ * inv;
            }
        }
    }
};

#endif  // YSIM_PREVIEW_STATE_HPP
