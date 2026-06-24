// mesh_cluster.hpp — connectivity-based mesh clustering for sub-object BVH.
//
// Splits an arbitrary triangle mesh into k = (<=) 4^s connected, balanced
// clusters by viewing it as a face dual graph (node = triangle, edge = shared
// mesh edge) and running recursive 4-way balanced flood fill. Each triangle
// lands in exactly ONE cluster; each USED vertex is assigned to exactly ONE
// cluster (single ownership, majority of its incident faces — caller's rule,
// overriding the multi-ownership option some references suggest).
//
// This is a BUILD-TIME CPU precompute (runs once per static mesh): flood fill is
// inherently sequential BFS, so GPU buys nothing for a one-shot. The RUNTIME
// collision pipeline (grid cluster-pair broad phase, bidirectional VF dispatch)
// is the part that goes on the GPU; this just produces the cluster tables it
// consumes.
//
// Output `faceCluster[F]` plugs straight into the existing sub-object BVH group
// representation (groupOfPrim) — the GPU build/refit/query kernels don't care
// how prims were grouped, only that each prim has a group id.
//
// ponytail: world-space (Euclidean centroid) seeds + flood fill. Folded/coplanar
// meshes can put surface-far faces in one cluster; upgrade seed + cost to dual-
// graph geodesic distance if skinny clusters show up. Single component assumed
// well-connected; disconnected leftovers fall back to nearest-seed (may break
// strict connectivity on multi-component meshes — split by component first then).

#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <limits>

namespace meshcluster {

struct ClusterResult {
    int numClusters = 0;
    std::vector<int> faceCluster;            // [F]  face  -> cluster id (0..k-1)
    std::vector<int> vertCluster;            // [V]  vert  -> cluster id, -1 if unused
    std::vector<uint32_t> faceOffsets;       // [k+1] CSR: cluster g faces in
    std::vector<uint32_t> clusterFaces;      // [F]        clusterFaces[faceOffsets[g]..]
    std::vector<uint32_t> vertOffsets;       // [k+1] CSR: cluster g verts (single-owned)
    std::vector<uint32_t> clusterVerts;      // [<=V]
};

// Face dual-graph adjacency: faceAdj[f] = up to 3 edge-neighbor faces, -1 pad.
inline std::vector<std::array<int,3>>
buildFaceAdjacency(const uint32_t* facets, size_t numFaces) {
    std::vector<std::array<int,3>> adj(numFaces, {-1,-1,-1});
    std::unordered_map<uint64_t,int> edgeOwner;          // edge key -> first face seen
    edgeOwner.reserve(numFaces * 3);
    auto key = [](uint32_t a, uint32_t b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (uint64_t(a) << 32) | uint64_t(b);
    };
    auto link = [&](int fa, int fb) {
        for (int s = 0; s < 3; ++s) if (adj[fa][s] == fb) return;
        for (int s = 0; s < 3; ++s) if (adj[fa][s] == -1) { adj[fa][s] = fb; return; }
    };
    for (size_t f = 0; f < numFaces; ++f) {
        uint32_t v[3] = { facets[3*f], facets[3*f+1], facets[3*f+2] };
        for (int e = 0; e < 3; ++e) {
            uint64_t k = key(v[e], v[(e+1)%3]);
            auto it = edgeOwner.find(k);
            if (it == edgeOwner.end()) edgeOwner.emplace(k, (int)f);
            else { link((int)f, it->second); link(it->second, (int)f); }  // manifold: 2 faces/edge
        }
    }
    return adj;
}

namespace detail {

// Per-face centroid (mean of its 3 vertex positions).
inline std::vector<std::array<float,3>>
faceCentroids(const uint32_t* facets, size_t numFaces, const float* pos) {
    std::vector<std::array<float,3>> c(numFaces);
    for (size_t f = 0; f < numFaces; ++f) {
        const float* a = pos + 3*facets[3*f];
        const float* b = pos + 3*facets[3*f+1];
        const float* d = pos + 3*facets[3*f+2];
        for (int i = 0; i < 3; ++i) c[f][i] = (a[i] + b[i] + d[i]) / 3.0f;
    }
    return c;
}

inline float dist2(const std::array<float,3>& a, const std::array<float,3>& b) {
    float dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return dx*dx + dy*dy + dz*dz;
}

// Farthest-point sampling of k seed faces. seed0 = most central (nearest the
// global centroid); each next seed = the face with the largest distance to the
// nearest already-chosen seed (running minDist²). O(k·F).
inline std::vector<int>
kwayFarthestSeeds(size_t numFaces, const std::vector<std::array<float,3>>& cen, int k) {
    std::array<float,3> mid{0,0,0};
    for (size_t f = 0; f < numFaces; ++f) for (int i = 0; i < 3; ++i) mid[i] += cen[f][i];
    for (int i = 0; i < 3; ++i) mid[i] /= (float)numFaces;

    std::vector<int> seeds;
    int s0 = 0; float best = std::numeric_limits<float>::max();
    for (size_t f = 0; f < numFaces; ++f) { float d = dist2(cen[f], mid); if (d < best) { best = d; s0 = (int)f; } }
    seeds.push_back(s0);
    std::vector<float> minD(numFaces);
    for (size_t f = 0; f < numFaces; ++f) minD[f] = dist2(cen[f], cen[s0]);
    while ((int)seeds.size() < k) {
        int cand = -1; float bestMin = -1.0f;
        for (size_t f = 0; f < numFaces; ++f) if (minD[f] > bestMin) { bestMin = minD[f]; cand = (int)f; }
        if (cand < 0 || bestMin <= 0.0f) break;       // ran out of distinct faces
        seeds.push_back(cand);
        for (size_t f = 0; f < numFaces; ++f) minD[f] = std::min(minD[f], dist2(cen[f], cen[cand]));
    }
    return seeds;
}

// Multi-source priority-queue region growing over the WHOLE mesh (single level,
// k = #seeds). A face is only ever assigned when popped as a neighbor of an
// already-assigned face of that cluster ⇒ every cluster is CONNECTED by
// construction. Growth cost = GRAPH distance (BFS hops) from the seed — hops
// spread uniformly over the surface, so an extremity seed can't grab a far
// interior region through a thin corridor (the Euclidean failure mode). Once a
// cluster passes target = F/k a capacity penalty (≥ F, larger than any hop
// count) is added: a full cluster's frontier then always loses to any
// under-target cluster ⇒ balanced, no leftovers. Fills owner[f] ∈ [0,k).
inline void
growKWay(size_t numFaces, const std::vector<std::array<int,3>>& adj,
         const std::vector<int>& seeds, int target, std::vector<int>& owner) {
    int k = (int)seeds.size();
    const float PEN = (float)numFaces + 1.0f;
    std::fill(owner.begin(), owner.end(), -1);
    struct Item { float cost; int hops; int face; int cluster; };
    struct Cmp { bool operator()(const Item&a, const Item&b) const { return a.cost > b.cost; } };
    std::priority_queue<Item, std::vector<Item>, Cmp> pq;
    std::vector<int> count(k, 0);
    auto pushNbrs = [&](int f, int hops, int c) {
        float extra = count[c] > target ? (count[c]-target) * PEN : 0.0f;
        for (int nb : adj[f]) if (nb >= 0 && owner[nb] == -1)
            pq.push({ (float)(hops+1) + extra, hops+1, nb, c });   // penalty non-compounding
    };
    for (int c = 0; c < k; ++c) { owner[seeds[c]] = c; count[c] = 1; pushNbrs(seeds[c], 0, c); }
    while (!pq.empty()) {
        Item it = pq.top(); pq.pop();
        if (owner[it.face] != -1) continue;
        owner[it.face] = it.cluster; count[it.cluster]++;
        pushNbrs(it.face, it.hops, it.cluster);
    }
    // Any face in a component with no seed stays -1; attach to a fixpoint wave so
    // the result is fully assigned (a separate component just joins one cluster).
    bool changed = true;
    while (changed) {
        changed = false;
        for (size_t f = 0; f < numFaces; ++f) if (owner[f] == -1)
            for (int nb : adj[f]) if (nb >= 0 && owner[nb] != -1) { owner[f] = owner[nb]; changed = true; break; }
    }
    for (size_t f = 0; f < numFaces; ++f) if (owner[f] == -1) owner[f] = 0;  // isolated face
}

} // namespace detail

// Single-level k-way connected balanced clustering on the face dual graph →
// k = min(4^s, F) clusters. Recursive 4-way was abandoned: its per-level
// imbalance COMPOUNDS multiplicatively (a 3.5:1 top split on the Human became
// ~400:1 by s=5). One k-way growth has no compounding; a few Lloyd relaxations
// (regrow, move each seed to its region's medoid) equalize the regions further.
inline ClusterResult
clusterMeshDualGraph(const uint32_t* facets, size_t numFaces,
                     const float* pos, size_t numVerts, int s) {
    ClusterResult R;
    if (numFaces == 0) return R;
    auto adj = buildFaceAdjacency(facets, numFaces);
    auto cen = detail::faceCentroids(facets, numFaces, pos);

    int kReq = 1; for (int i = 0; i < s; ++i) kReq *= 4;          // 4^s
    int k = std::min<int>(kReq, (int)numFaces);
    std::vector<int> seeds = detail::kwayFarthestSeeds(numFaces, cen, k);
    k = (int)seeds.size();
    int target = std::max(1, (int)numFaces / k);

    std::vector<int> owner(numFaces, -1);
    detail::growKWay(numFaces, adj, seeds, target, owner);
    // Lloyd relaxation: move each seed to its region medoid (face nearest the
    // region centroid) and regrow. Converges fast; stop early when seeds settle.
    for (int iter = 0; iter < 6; ++iter) {
        std::vector<std::array<double,3>> sum(k, {0,0,0});
        std::vector<int> cnt(k, 0);
        for (size_t f = 0; f < numFaces; ++f) {
            int c = owner[f]; ++cnt[c];
            for (int i = 0; i < 3; ++i) sum[c][i] += cen[f][i];
        }
        std::vector<std::array<float,3>> ctr(k);
        for (int c = 0; c < k; ++c) for (int i = 0; i < 3; ++i)
            ctr[c][i] = cnt[c] ? (float)(sum[c][i] / cnt[c]) : cen[seeds[c]][i];
        std::vector<int> ns(k, -1); std::vector<float> nb(k, std::numeric_limits<float>::max());
        for (size_t f = 0; f < numFaces; ++f) {
            int c = owner[f]; float d = detail::dist2(cen[f], ctr[c]);
            if (d < nb[c]) { nb[c] = d; ns[c] = (int)f; }
        }
        bool same = true;
        for (int c = 0; c < k; ++c) if (ns[c] >= 0 && ns[c] != seeds[c]) { seeds[c] = ns[c]; same = false; }
        if (same) break;
        detail::growKWay(numFaces, adj, seeds, target, owner);
    }

    R.numClusters = k;
    R.faceCluster.assign(numFaces, -1);
    for (size_t f = 0; f < numFaces; ++f) R.faceCluster[f] = owner[f];
    R.faceOffsets.assign(k + 1, 0);
    for (size_t f = 0; f < numFaces; ++f) R.faceOffsets[owner[f] + 1]++;
    for (int g = 0; g < k; ++g) R.faceOffsets[g+1] += R.faceOffsets[g];
    R.clusterFaces.assign(numFaces, 0);
    { std::vector<uint32_t> w(R.faceOffsets.begin(), R.faceOffsets.end() - 1);
      for (size_t f = 0; f < numFaces; ++f) R.clusterFaces[w[owner[f]]++] = (uint32_t)f; }

    // Vertex single ownership = cluster holding the majority of its incident
    // faces (ties → lowest cluster id). One pass: tally per (vertex) the cluster
    // counts via a small local map, pick the argmax.
    R.vertCluster.assign(numVerts, -1);
    std::vector<std::unordered_map<int,int>> tally(numVerts);
    for (size_t f = 0; f < numFaces; ++f) {
        int g = R.faceCluster[f];
        for (int e = 0; e < 3; ++e) tally[facets[3*f+e]][g]++;
    }
    R.vertOffsets.assign(k + 1, 0);
    std::vector<uint32_t> counts(k, 0);
    for (size_t v = 0; v < numVerts; ++v) {
        if (tally[v].empty()) continue;
        int bg = -1, bc = -1;
        for (auto& [g, c] : tally[v]) if (c > bc || (c == bc && g < bg)) { bc = c; bg = g; }
        R.vertCluster[v] = bg; counts[bg]++;
    }
    for (int g = 0; g < k; ++g) R.vertOffsets[g+1] = R.vertOffsets[g] + counts[g];
    R.clusterVerts.assign(R.vertOffsets[k], 0);
    std::vector<uint32_t> w(R.vertOffsets.begin(), R.vertOffsets.end() - 1);
    for (size_t v = 0; v < numVerts; ++v) {
        int g = R.vertCluster[v];
        if (g >= 0) R.clusterVerts[w[g]++] = (uint32_t)v;
    }
    return R;
}

// ---- self-check: synthetic N×N grid mesh → asserts cluster invariants -------
// Runs the full pipeline on a known single-component mesh, so connectivity +
// balance + single-ownership must hold. Returns true iff all invariants pass.
inline bool selfTest(int N = 64, int s = 3, bool verbose = true) {
    const size_t V = (size_t)N * N;
    const size_t F = (size_t)2 * (N - 1) * (N - 1);
    std::vector<float> pos(3 * V);
    for (int i = 0; i < N; ++i) for (int j = 0; j < N; ++j) {
        size_t v = (size_t)i * N + j;
        pos[3*v] = (float)j; pos[3*v+1] = (float)i; pos[3*v+2] = 0.0f;
    }
    std::vector<uint32_t> fac(3 * F);
    size_t fi = 0;
    for (int i = 0; i < N - 1; ++i) for (int j = 0; j < N - 1; ++j) {
        uint32_t a = (uint32_t)(i*N + j),     b = (uint32_t)(i*N + j + 1);
        uint32_t c = (uint32_t)((i+1)*N + j), d = (uint32_t)((i+1)*N + j + 1);
        fac[3*fi]=a; fac[3*fi+1]=b; fac[3*fi+2]=d; ++fi;   // shared diagonal a-d
        fac[3*fi]=a; fac[3*fi+1]=d; fac[3*fi+2]=c; ++fi;
    }

    ClusterResult R = clusterMeshDualGraph(fac.data(), F, pos.data(), V, s);
    int k = R.numClusters;
    bool ok = true;
    auto check = [&](bool c, const char* msg) {
        if (!c) { ok = false; if (verbose) printf("  [FAIL] %s\n", msg); }
    };

    check(k > 1 && k <= (1 << (2*s)), "cluster count in (1, 4^s]");

    // (1) every face assigned to a valid cluster
    for (size_t f = 0; f < F; ++f)
        if (R.faceCluster[f] < 0 || R.faceCluster[f] >= k) { check(false, "face unassigned/oob"); break; }

    // (2) clusterFaces is a permutation of [0,F): each face exactly once
    {
        std::vector<char> seen(F, 0); size_t total = 0;
        for (uint32_t f : R.clusterFaces) { if (f < F && !seen[f]) { seen[f] = 1; total++; } }
        check(total == F && R.clusterFaces.size() == F, "clusterFaces covers all faces once");
        for (int g = 0; g < k; ++g)
            check(R.faceOffsets[g+1] >= R.faceOffsets[g], "faceOffsets monotone");
    }

    // (3) each cluster connected over the dual graph
    auto adj = buildFaceAdjacency(fac.data(), F);
    {
        std::vector<char> vis(F, 0);
        for (int g = 0; g < k && ok; ++g) {
            uint32_t lo = R.faceOffsets[g], hi = R.faceOffsets[g+1];
            if (hi <= lo) continue;
            std::queue<int> q; int start = (int)R.clusterFaces[lo];
            q.push(start); vis[start] = 1; uint32_t reached = 1;
            while (!q.empty()) {
                int f = q.front(); q.pop();
                for (int nb : adj[f])
                    if (nb >= 0 && R.faceCluster[nb] == g && !vis[nb]) { vis[nb]=1; q.push(nb); reached++; }
            }
            for (uint32_t i = lo; i < hi; ++i) vis[R.clusterFaces[i]] = 0;  // reset
            check(reached == (hi - lo), "cluster connected");
        }
    }

    // (4) balance: max/min face-count ratio. Only asserted when clusters are big
    // enough (avg ≥ 16 faces) to be meaningful — below that the ratio is pure
    // integer-granularity noise (a 7-face "cluster" can't balance to ±15%).
    uint32_t mn = (uint32_t)F, mx = 0;
    for (int g = 0; g < k; ++g) { uint32_t c = R.faceOffsets[g+1]-R.faceOffsets[g]; mn=std::min(mn,c); mx=std::max(mx,c); }
    double ratio = mn ? (double)mx / mn : 1e9;
    if ((double)F / k >= 16.0) check(ratio < 3.0, "face balance ratio < 3 (uniform mesh)");

    // (5) vertex single ownership: clusterVerts permutation of used verts, each once
    {
        size_t used = 0; for (size_t v = 0; v < V; ++v) if (R.vertCluster[v] >= 0) used++;
        std::vector<char> seen(V, 0); size_t total = 0;
        for (uint32_t v : R.clusterVerts) { check(v < V && !seen[v], "vert single-owned once"); if (v<V) { seen[v]=1; total++; } }
        check(total == used && R.clusterVerts.size() == used, "clusterVerts covers used verts once");
        check(R.vertOffsets[k] == (uint32_t)used, "vertOffsets total == used verts");
    }

    if (verbose)
        printf("[mesh_cluster selfTest] N=%d s=%d  V=%zu F=%zu -> k=%d  "
               "faces/cluster=[%u,%u] ratio=%.2f  %s\n",
               N, s, V, F, k, mn, mx, ratio, ok ? "PASS" : "FAIL");
    return ok;
}

} // namespace meshcluster
