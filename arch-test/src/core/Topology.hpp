#pragma once
#include "backend/Backend.hpp"
#include "backend/VectorBase.hpp"
#include "core/SimState.hpp"
#include "tinym.hpp"

#include <algorithm>
#include <vector>
#include <cassert>

// Per-mesh triangle topology + spring adjacency, ported from src/main.cpp
// MeshAdjacency (811-848) + MeshAdjacencyInitializer (856-1171) + grid
// triangulation (1261-1288). Only the 8 buffers the spring kernel binds
// (slots 10-17, main.cpp:2054-2063) are kept; restFacetAreas /
// vertexAdjFacets are dropped (no consumer this pass). GPU VectorBase
// buffers are host-visible (same pattern as SimState.x in Scene::realize),
// so we CPU-build straight into them. Indices are mesh-local [0, N*N);
// position reads add `base` for the rest-length measurement only (the
// kernel later resolves global offset via statesOffsets — do NOT bake
// `base` into stored indices). See PORT_MAP.md for rationale.

struct EdgeInfo {
    int v0 = -1, v1 = -1; // v0 < v1
    int f0 = -1, f1 = -1;
    int o0 = -1, o1 = -1; // opposite verts (bend springs)
};

template <typename BE, typename PR>
struct MeshTopology {
    using VecI = VectorBase<BE, Index>;
    using VecP = VectorBase<BE, PR>;

    VecI facets;                                       // slot 11: numFacets*3
    VecI edges;                                        // slot 10: numEdges*2
    VecI vertexAdjEdges, vertexAdjEdgesOffsets;        // slots 12/13: stretch CSR
    VecP restEdgeLengths;                              // slot 14
    VecI vertexOppVertices, vertexOppVerticesOffsets;  // slots 15/16: bend CSR
    VecP restOppLengths;                               // slot 17

    Index numFacets = 0;
    Index numEdges = 0;
    bool built = false;

    // Triangulate an NxN alternating-diagonal grid + build spring adjacency.
    // Positions are already seeded into s.x by Scene::realize; rest lengths
    // are measured from them. `base` is the mesh's vertex offset.
    void build(int N, Index base, SimState<BE, PR>& s) {
        const Index numPoints = Index(N) * Index(N);
        numFacets = 2u * Index(N - 1) * Index(N - 1);

        // --- triangulation (verbatim main.cpp:1261-1288) ---
        facets = VecI(numFacets * 3);
        Index fIdx = 0;
        auto addFacet = [&](Index a, Index b, Index c) {
            facets[fIdx++] = a; facets[fIdx++] = b; facets[fIdx++] = c;
        };
        for (Index row = 0; row < Index(N) - 1; ++row) {
            for (Index col = 0; col < Index(N) - 1; ++col) {
                Index p00 = (row * Index(N) + col);
                Index p10 = (row * Index(N) + col + 1);
                Index p01 = ((row + 1) * Index(N) + col);
                Index p11 = ((row + 1) * Index(N) + col + 1);
                if (((row + col) & 1) == 0) {  // even: diagonal p00-p11
                    addFacet(p00, p01, p11);
                    addFacet(p00, p11, p10);
                } else {                       // odd: diagonal p10-p01
                    addFacet(p00, p01, p10);
                    addFacet(p10, p01, p11);
                }
            }
        }

        // --- adjacency (verbatim algorithm main.cpp:953-1107) ---
        // Offsets are zero-filled so the [v+1]++ accumulation works.
        vertexAdjEdgesOffsets = VecI(numPoints + 1, 0);
        vertexOppVerticesOffsets = VecI(numPoints + 1, 0);

        auto edgeLength = [&](Index a, Index b) -> PR {
            auto va = tinym::vec3_view(s.x.ptr + (base + a) * 3);
            auto vb = tinym::vec3_view(s.x.ptr + (base + b) * 3);
            return PR((vb - va).norm());
        };

        // (b) fill temp edge infos (opposite vert per facet corner).
        std::vector<EdgeInfo> tmp;
        tmp.reserve(numFacets * 3);
        for (Index fid = 0; fid < numFacets; ++fid) {
            Index a = facets[fid * 3], b = facets[fid * 3 + 1], c = facets[fid * 3 + 2];
            auto push = [&](Index v0, Index v1, Index o0) {
                if (v0 > v1) std::swap(v0, v1);
                tmp.push_back(EdgeInfo{int(v0), int(v1), int(fid), -1, int(o0), -1});
            };
            push(a, b, c); push(b, c, a); push(c, a, b);
        }
        std::sort(tmp.begin(), tmp.end(), [](const EdgeInfo& x, const EdgeInfo& y) {
            return x.v0 < y.v0 || (x.v0 == y.v0 && x.v1 < y.v1);
        });

        // (c) reduce to unique edges + rest lengths + count CSR degrees.
        std::vector<EdgeInfo> uniq;
        std::vector<Index> e0, e1;
        std::vector<PR> restE;
        for (size_t i = 0; i < tmp.size(); ++i) {
            if (uniq.empty() || tmp[i].v0 != uniq.back().v0 || tmp[i].v1 != uniq.back().v1) {
                uniq.push_back(tmp[i]);
                e0.push_back(Index(tmp[i].v0));
                e1.push_back(Index(tmp[i].v1));
                restE.push_back(edgeLength(Index(tmp[i].v0), Index(tmp[i].v1)));
                vertexAdjEdgesOffsets[tmp[i].v0 + 1]++;
                vertexAdjEdgesOffsets[tmp[i].v1 + 1]++;
            } else {  // second facet sharing this edge -> bend spring
                uniq.back().o1 = tmp[i].o0;
                uniq.back().f1 = tmp[i].f0;
                vertexOppVerticesOffsets[uniq.back().o0 + 1]++;
                vertexOppVerticesOffsets[uniq.back().o1 + 1]++;
            }
        }
        numEdges = Index(uniq.size());

        // (d) prefix-sum both CSR offset arrays (main.cpp:1037-1040).
        for (Index v = 0; v < numPoints; ++v) {
            vertexOppVerticesOffsets[v + 1] += vertexOppVerticesOffsets[v];
            vertexAdjEdgesOffsets[v + 1] += vertexAdjEdgesOffsets[v];
        }

        // (e) allocate edge/rest/opp buffers now that sizes are known.
        edges = VecI(numEdges * 2);
        restEdgeLengths = VecP(numEdges);
        for (Index e = 0; e < numEdges; ++e) {
            edges[e * 2] = e0[e];
            edges[e * 2 + 1] = e1[e];
            restEdgeLengths[e] = restE[e];
        }
        vertexOppVertices = VecI(vertexOppVerticesOffsets[numPoints], 0);
        restOppLengths = VecP(vertexOppVerticesOffsets[numPoints]);
        vertexAdjEdges = VecI(vertexAdjEdgesOffsets[numPoints], 0);

        // (f) scatter opposite-vertices (bend) + adjacent-edges (stretch).
        std::vector<Index> oppCur(numPoints, 0), adjCur(numPoints, 0);
        for (Index ei = 0; ei < numEdges; ++ei) {
            if (uniq[ei].o1 != -1) {
                Index o0 = Index(uniq[ei].o0), o1 = Index(uniq[ei].o1);
                Index b0 = oppCur[o0]++ + vertexOppVerticesOffsets[o0];
                Index b1 = oppCur[o1]++ + vertexOppVerticesOffsets[o1];
                vertexOppVertices[b0] = o1;
                vertexOppVertices[b1] = o0;
                PR L = edgeLength(o0, o1);
                restOppLengths[b0] = L;
                restOppLengths[b1] = L;
            }
            Index v0 = Index(uniq[ei].v0), v1 = Index(uniq[ei].v1);
            vertexAdjEdges[adjCur[v0]++ + vertexAdjEdgesOffsets[v0]] = ei;
            vertexAdjEdges[adjCur[v1]++ + vertexAdjEdgesOffsets[v1]] = ei;
        }

        built = true;

        // --- verification (§5; assert-only, no release cost) ---
        assert(numFacets == 2u * Index(N - 1) * Index(N - 1));
        assert(facets.size == numFacets * 3);
        assert(numEdges == Index(N - 1) * (3u * Index(N) - 1u));
        assert(edges.size == numEdges * 2);
        assert(restEdgeLengths.size == numEdges);
        for (Index e = 0; e < numEdges; ++e) assert(restEdgeLengths[e] > PR(0));
        assert(vertexAdjEdgesOffsets[numPoints] == numEdges * 2);
        for (Index k = 0; k < restOppLengths.size; ++k) assert(restOppLengths[k] > PR(0));
        (void)numPoints;
    }
};
