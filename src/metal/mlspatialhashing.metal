#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"

// ============================================================================
// Multi-level (hierarchical hash-grid / hgrid) spatial-hashing broad phase.
//
// L grids with GEOMETRICALLY-spaced cell sizes  cellSize[k] = c0 * 2^k
// (k in [0, L-1]), cellSize[L-1] = 2*maxRadius so the coarsest grid always fits
// the biggest (non-excluded) primitive. Each face is assigned to exactly ONE
// HOME cell — the centroid cell of its "fitting" level Lf, the finest grid whose
// cell already fits the inflated face BV. NO phantom replication.
//
// The 32-bit sort key packs the level into the top 4 bits:
//     cellID = (level << 28) | (linearCellId & 0x0FFFFFFF)
// so the SAME 4-pass radix sort segregates all levels into contiguous runs;
// a cell run never straddles a level boundary.
//
// Cross-level pairs are found at QUERY time, not insert time: the per-face broad
// phase (ml_broadPhase) walks levels Lf..L-1 and, for each cell its BV overlaps,
// binary-searches the sorted entries and collides against the faces HOME there.
// This keeps every cell sparse (only faces home at that level live in it), so
// the pair work is bounded — unlike phantom replication, which packs the coarse
// levels with copies of every finer face and blows up O(N^3).
//
// Dedup: a pair (A,B) is emitted once. At A's own level both are home, so emit
// only when faceB > faceA; at coarser levels A is strictly finer than any home
// there and owns the pair outright (the coarser face never walks back down).
//
// Reused from spatialhashing.metal (same library): sh_reduceMaxRadius + the
// radix sort. New here: ml_buildBV, ml_assignCells, ml_broadPhase.
//
// Floor / oversized-primitive handling: faces whose owning mesh is flagged in
// faceExclude[] (host builds it from a per-mesh AABB-diagonal threshold) emit
// a sentinel entry and get radius 0 so they drop out of the max-radius
// reduction and the scene AABB — keeping cell size and occupancy bounded.
// ============================================================================

constant uint ML_LEVEL_SHIFT = 28u;       // cellID = (level << 28) | linearCellId
constant uint ML_CELL_MASK    = 0x0FFFFFFFu;
// MAX level budget. The active level count (params.numLevels <= this) is chosen
// on the host from the min/max face-size ratio so the finest level matches the
// smallest face (no pile-up) and the coarsest fits the largest — the defining
// property of a hierarchical hash grid. cellSize/gridRes arrays are sized to it.
#define ML_MAXL 4

// Per-level grid parameters. Layout MUST byte-match MultiLevelSpatialHashing::
// MLParamsHost (host mirror): plain 4-byte scalars/arrays only, no packed_*,
// so there is no implicit padding. gridRes is indexed [level*3 + axis].
struct MLParams {
    uint  numFaces;
    uint  numLevels;
    float epsilon;
    float cellSize[ML_MAXL];
    uint  gridRes[ML_MAXL * 3];
    float originMin[3];
};

// Home-only entry: one per face, sorted by cellID. value is the global face id.
struct SHEntry {
    uint cellID;   // (level<<28) | linearCellId, or 0xFFFFFFFF for sentinel/empty
    uint value;    // global face id of the home face
};

// ----- Step 1: per-triangle bounding sphere (with floor exclusion) -----
//
// Same as sh_buildBV, plus: excluded faces get radius 0 so they neither raise
// maxRadius nor (downstream) emit any cell entries.
kernel void ml_buildBV(
    device const packed_float3* scenePackedPositions        [[buffer(0)]],
    device const uint*          scenePackedPositionsOffsets [[buffer(1)]],
    device const packed_uint3*  scenePackedFacets           [[buffer(2)]],
    device const uint*          faceObj                     [[buffer(3)]],
    device const uchar*         faceExclude                 [[buffer(4)]],
    constant MLParams&          params                      [[buffer(5)]],
    device packed_float3*       centroid                    [[buffer(6)]],
    device float*               radius                      [[buffer(7)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numFaces) return;

    uint  obj   = faceObj[id];
    uint  vbase = scenePackedPositionsOffsets[obj];
    uint3 facet = scenePackedFacets[id];

    float3 v0 = float3(scenePackedPositions[facet.x + vbase]);
    float3 v1 = float3(scenePackedPositions[facet.y + vbase]);
    float3 v2 = float3(scenePackedPositions[facet.z + vbase]);

    float3 c = (v0 + v1 + v2) * (1.0f / 3.0f);
    centroid[id] = packed_float3(c);

    if (faceExclude[id] != 0u) {
        radius[id] = 0.0f;             // excluded from reduce + assign
        return;
    }
    float d0 = distance(c, v0);
    float d1 = distance(c, v1);
    float d2 = distance(c, v2);
    radius[id] = max(d0, max(d1, d2));
}

// ----- Step 4: home-level cell assignment (hierarchical hash grid) -----
//
// One entry per face: its HOME cell at the fitting level Lf (the finest level
// whose cell already fits the inflated BV diameter). No cross-level phantom
// replication — coarser-level overlaps are discovered at query time by the
// per-face broad phase walking UP the levels (ml_broadPhase). This keeps every
// level's cells sparse (a cell holds only faces that are HOME there), so the
// pair work is bounded instead of the O(N^3) packed-coarse-cell blowup the
// phantom-replication scheme produced.
kernel void ml_assignCells(
    device const packed_float3* centroid     [[buffer(0)]],
    device const float*         radius       [[buffer(1)]],
    device const uchar*         faceExclude  [[buffer(2)]],
    constant     MLParams&      params       [[buffer(3)]],
    device       SHEntry*       entries      [[buffer(4)]],   // one slot / face
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numFaces) return;

    float r = radius[id];
    // Excluded (floor) or degenerate face: sentinel, sorted to the tail.
    if (faceExclude[id] != 0u || r <= 0.0f) {
        entries[id].cellID = 0xFFFFFFFFu;
        entries[id].value  = 0xFFFFFFFFu;
        return;
    }

    uint   L    = params.numLevels;
    float3 c    = float3(centroid[id]);
    float3 oMin = float3(params.originMin[0], params.originMin[1], params.originMin[2]);
    float  rIns = r + 0.5f * params.epsilon;

    // Fitting (home) level: finest level whose cell >= inflated BV diameter.
    uint Lf = L - 1u;
    for (uint k = 0u; k < L; ++k) {
        if (params.cellSize[k] >= 2.0f * rIns) { Lf = k; break; }
    }

    float cs  = params.cellSize[Lf];
    int3  res = int3((int)params.gridRes[Lf*3+0],
                     (int)params.gridRes[Lf*3+1],
                     (int)params.gridRes[Lf*3+2]);
    int3  g   = clamp(int3(floor((c - oMin) / cs)), int3(0), res - int3(1));
    uint  lin = (uint)g.z * (uint)res.y * (uint)res.x
              + (uint)g.y * (uint)res.x
              + (uint)g.x;

    entries[id].cellID = (Lf << ML_LEVEL_SHIFT) | (lin & ML_CELL_MASK);
    entries[id].value  = id;          // home-only: value is the global face id
}

// ----- Step 6: per-face broad phase (hierarchical query walk) -----

struct MLBroadParams {
    uint  numFaces;
    uint  numValid;
    uint  maxNumCollisions;
    uint  enableSelfCollisions;
    uint  numLevels;
    float epsilon;
};

// [lo,hi) of entries sharing exactly `key` in the cellID-sorted, sentinel-tailed
// `entries` buffer (binary lower-bound + forward scan; cells are sparse so the
// scan is short).
inline uint2 ml_cellRange(device const SHEntry* entries, uint numValid, uint key) {
    uint lo = 0u, hi = numValid;
    while (lo < hi) {
        uint mid = lo + (hi - lo) / 2u;
        if (entries[mid].cellID < key) lo = mid + 1u;
        else                           hi = mid;
    }
    uint e = lo;
    while (e < numValid && entries[e].cellID == key) ++e;
    return uint2(lo, e);
}

// One thread per face. Face A (home at level Lf) walks levels Lf..L-1, and at
// each level visits the cells its inflated BV overlaps, colliding against the
// faces HOME in those cells. Dedup: at A's own level only emit when faceB>faceA;
// at coarser levels A is strictly finer than any home there, so it owns the
// pair outright (the coarser face never walks back down to A's level).
kernel void ml_broadPhase(
    device const SHEntry*       entries                  [[buffer(0)]],
    constant     MLParams&      gp                       [[buffer(1)]],  // grid params
    device const uint*          faceObj                  [[buffer(2)]],
    device const packed_uint3*  scenePackedFacets        [[buffer(3)]],
    device const uint*          scenePackedFacetsOffsets [[buffer(4)]],
    device const packed_float3* centroid                 [[buffer(5)]],
    device const float*         radiusBuf                [[buffer(6)]],
    device const uint*          meshBehaviors            [[buffer(7)]],
    device const uint*          meshShapes               [[buffer(8)]],
    constant     MLBroadParams& params                   [[buffer(9)]],
    device atomic_uint*         numBroadCollisions       [[buffer(10)]],
    device BroadCollision*      broadCollisions          [[buffer(11)]],
    uint fid [[thread_position_in_grid]]
) {
    if (fid >= params.numFaces) return;

    float rA = radiusBuf[fid];
    if (rA <= 0.0f) return;                 // excluded / degenerate face

    float  eps   = params.epsilon;
    float3 cA    = float3(centroid[fid]);
    float3 oMin  = float3(gp.originMin[0], gp.originMin[1], gp.originMin[2]);
    uint   L     = gp.numLevels;
    float  rInsA = rA + 0.5f * eps;

    uint Lf = L - 1u;
    for (uint k = 0u; k < L; ++k) {
        if (gp.cellSize[k] >= 2.0f * rInsA) { Lf = k; break; }
    }

    uint  objA     = faceObj[fid];
    uint  behaviorA = meshBehaviors[objA];
    uint  shapeA    = meshShapes[objA];
    uint3 triA      = scenePackedFacets[fid];
    uint  localFA   = fid - scenePackedFacetsOffsets[objA];
    float queryR    = rA + eps;             // BV inflation for cell overlap

    for (uint kc = Lf; kc < L; ++kc) {
        float cs  = gp.cellSize[kc];
        int3  res = int3((int)gp.gridRes[kc*3+0],
                         (int)gp.gridRes[kc*3+1],
                         (int)gp.gridRes[kc*3+2]);
        int3 gmin = clamp(int3(floor((cA - queryR - oMin) / cs)), int3(0), res - int3(1));
        int3 gmax = clamp(int3(floor((cA + queryR - oMin) / cs)), int3(0), res - int3(1));

        for (int gz = gmin.z; gz <= gmax.z; ++gz)
        for (int gy = gmin.y; gy <= gmax.y; ++gy)
        for (int gx = gmin.x; gx <= gmax.x; ++gx) {
            uint lin = (uint)gz * (uint)res.y * (uint)res.x
                     + (uint)gy * (uint)res.x
                     + (uint)gx;
            uint key = (kc << ML_LEVEL_SHIFT) | (lin & ML_CELL_MASK);
            uint2 rng = ml_cellRange(entries, params.numValid, key);

            for (uint i = rng.x; i < rng.y; ++i) {
                uint faceB = entries[i].value;
                // Same-level dedup; finer-owns-coarser across levels.
                if (kc == Lf && faceB <= fid) continue;

                uint objB = faceObj[faceB];
                if (params.enableSelfCollisions == 0u && objA == objB) continue;

                uint behaviorB = meshBehaviors[objB];
                if (behaviorA == 4u && behaviorB == 4u) continue;   // Float-Float

                float3 cB = float3(centroid[faceB]);
                if (distance(cA, cB) > rA + radiusBuf[faceB] + eps) continue;

                uint3 triB    = scenePackedFacets[faceB];
                uint  localFB = faceB - scenePackedFacetsOffsets[objB];
                uint  shapeB  = meshShapes[objB];

                {
                    uint vIds[3] = { triA.x, triA.y, triA.z };
                    for (uint v = 0u; v < 3u; ++v) {
                        uint outIdx = atomic_fetch_add_explicit(numBroadCollisions, 1u, memory_order_relaxed);
                        if (outIdx >= params.maxNumCollisions) return;
                        broadCollisions[outIdx].indexPair    = uint2(vIds[v], localFB);
                        broadCollisions[outIdx].objPair      = uint2(objA, objB);
                        broadCollisions[outIdx].behaviorPair = uint2(behaviorA, behaviorB);
                        broadCollisions[outIdx].shapePair    = uint2(shapeA, shapeB);
                    }
                }
                {
                    uint vIds[3] = { triB.x, triB.y, triB.z };
                    for (uint v = 0u; v < 3u; ++v) {
                        uint outIdx = atomic_fetch_add_explicit(numBroadCollisions, 1u, memory_order_relaxed);
                        if (outIdx >= params.maxNumCollisions) return;
                        broadCollisions[outIdx].indexPair    = uint2(vIds[v], localFA);
                        broadCollisions[outIdx].objPair      = uint2(objB, objA);
                        broadCollisions[outIdx].behaviorPair = uint2(behaviorB, behaviorA);
                        broadCollisions[outIdx].shapePair    = uint2(shapeB, shapeA);
                    }
                }
            }
        }
    }
}
