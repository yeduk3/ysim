#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"

// ----- Spatial Hashing types (Step 0 scaffold) -----
//
// SHParams: uniform-grid parameters. Filled on host once cellSize / gridRes
// are known.
//
// CellProp: per-active-cell metadata after Stage 6/7. Indexed in [0, numCells).
//   start    -> first index in sortedEntries that belongs to this cell
//   nH       -> number of home entries inside the cell
//   nP       -> number of phantom entries inside the cell
//   pairCnt  -> nH*(nH-1)/2 + nH*nP  (Step 7)
//   cellType -> homeT bits (0..7) of the cell coord; written during cellprop fill
//
// Kernels are added per implementation step; this file currently contains
// no kernel definitions on purpose.

struct SHParams {
    uint  numFaces;
    float cellSize;
    float epsilon;
    packed_uint3  gridRes;
    packed_float3 originMin;
};

// nH/nP are atomic so multiple entries belonging to the same cell can
// race-free increment them in sh_fillCellProp. Layout is bytewise identical
// to a plain uint (4 bytes each); the host mirror uses uint32_t and reads
// the values directly after a commitAndWait.
struct CellProp {
    uint        start;
    atomic_uint nH;
    atomic_uint nP;
    uint        pairCnt;
    uint        cellType;
};

// Hash-table entry. Layout matches host SpatialHashing::SHEntry exactly and
// is also compatible with the existing SortPair-shaped radix sorter
// (8 bytes, key/value).
//   cellID : grid hash (z*resY*resX + y*resX + x), or 0xFFFFFFFF for empty.
//   value  : (faceId << 4) | (phantom << 3) | homeT
//            faceId is 28 bits, phantom is 1 bit, homeT is 3 bits.
struct SHEntry {
    uint cellID;
    uint value;
};

// ----- Step 1: per-triangle bounding sphere -----
//
// Each thread = one global face id (across the whole packed scene).
// faceObj[id] tells us which mesh owns this face; we use that to translate
// the object-local vertex ids stored in scenePackedFacets into global indices
// in scenePackedPositions, mirroring the lookup pattern used by narrow_pt_tri.
//
// Output:
//   centroid[id] = (v0 + v1 + v2) / 3
//   radius[id]   = max distance from centroid to any of the three vertices

kernel void sh_buildBV(
    device const packed_float3* scenePackedPositions        [[buffer(0)]],
    device const uint*          scenePackedPositionsOffsets [[buffer(1)]],
    device const packed_uint3*  scenePackedFacets           [[buffer(2)]],
    device const uint*          faceObj                     [[buffer(3)]],
    constant SHParams&          params                      [[buffer(4)]],
    device packed_float3*       centroid                    [[buffer(5)]],
    device float*               radius                      [[buffer(6)]],
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

    //float d0 = distance(c, v0);
    //float d1 = distance(c, v1);
    //float d2 = distance(c, v2);
    //radius[id] = max(d0, max(d1, d2));
    radius[id] = distance(c, v0); // d0 == d1 == d2
}

// ----- Step 2: max-radius reduction -----
//
// One-kernel max reducer, dispatched iteratively from the host.
// Each call reduces `count` floats from `in` into `ceil(count/256)` floats
// in `out`. Threadgroup size is fixed at 256; out-of-range threads
// contribute 0 (radii are non-negative so 0 is the identity for max).
//
// Caller iterates: 1st pass radius->partial, then partial->partial2,
// alternating, until count == 1; the final pass targets `maxRadius`.

kernel void sh_reduceMaxRadius(
    device const float* in    [[buffer(0)]],
    device       float* out   [[buffer(1)]],
    constant     uint&  count [[buffer(2)]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    constexpr uint TG = 256;
    threadgroup float shared[TG];

    uint id = gid * TG + lid;
    shared[lid] = (id < count) ? in[id] : 0.0f;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint s = TG / 2; s > 0; s >>= 1) {
        if (lid < s) {
            shared[lid] = max(shared[lid], shared[lid + s]);
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (lid == 0) out[gid] = shared[0];
}

// ----- Step 4: cell assignment -----
//
// Per-face thread. Writes 1 home + up to 7 phantom entries into entries[].
// Empty slots get cellID = value = 0xFFFFFFFF so they sort to the back.
// Also accumulates an 8-bit cell-type bitmask into faceCB[id].
//
// Phantom-octant logic (Pabst):
//   1. Compute home cell coord g and offset of centroid within that cell.
//   2. Determine octant signs (sx,sy,sz) ∈ {+1,-1} from "which half".
//   3. Enumerate 8 cells (g + (ox,oy,oz)) where each o ∈ {0, sign[axis]}.
//   4. For each non-zero offset axis, accept iff distance from centroid to
//      that face of the home cell < radius. (Per-axis test = conservative
//      sphere-vs-AABB check; fine for broad-phase.)
//   5. (0,0,0) is always the home; emitted unconditionally.

kernel void sh_assignCells(
    device const packed_float3* centroid [[buffer(0)]],
    device const float*         radius   [[buffer(1)]],
    constant     SHParams&      params   [[buffer(2)]],
    device       SHEntry*       entries  [[buffer(3)]],
    device       uchar*         faceCB   [[buffer(4)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numFaces) return;

    float3 c    = float3(centroid[id]);
    float  r    = radius[id];
    float3 oMin = float3(params.originMin);
    float3 cs   = float3(params.cellSize);
    int3   res  = int3(params.gridRes);

    // Home cell coord, clamped to grid bounds.
    float3 cm = (c - oMin) / cs;
    int3   g  = int3(floor(cm));
    g = clamp(g, int3(0), res - int3(1));

    uint homeT = ((uint)g.x & 1u)
               | (((uint)g.y & 1u) << 1)
               | (((uint)g.z & 1u) << 2);

    // Position within home cell (in world units), and distance to the
    // far/near face along each axis given the chosen octant sign.
    float3 cellOriginWorld = oMin + float3(g) * cs;
    float3 local = c - cellOriginWorld;             // [0, cellSize) in each axis
    float  hcs   = params.cellSize * 0.5f;

    int3   sgn;
    float3 dist;
    sgn.x = (local.x > hcs) ? +1 : -1;
    sgn.y = (local.y > hcs) ? +1 : -1;
    sgn.z = (local.z > hcs) ? +1 : -1;
    dist.x = (sgn.x > 0) ? (params.cellSize - local.x) : local.x;
    dist.y = (sgn.y > 0) ? (params.cellSize - local.y) : local.y;
    dist.z = (sgn.z > 0) ? (params.cellSize - local.z) : local.z;

    uint cb   = 0u;
    uint slot = 0u;
    uint base = id * 8u;

    for (int oi = 0; oi <= 1; ++oi) {
        int ox = oi * sgn.x;
        if (ox != 0 && dist.x > r) continue;
        for (int oj = 0; oj <= 1; ++oj) {
            int oy = oj * sgn.y;
            if (oy != 0 && dist.y > r) continue;
            for (int ok = 0; ok <= 1; ++ok) {
                int oz = ok * sgn.z;
                if (oz != 0 && dist.z > r) continue;

                int3 gp = int3(g.x + ox, g.y + oy, g.z + oz);
                if (gp.x < 0 || gp.x >= res.x ||
                    gp.y < 0 || gp.y >= res.y ||
                    gp.z < 0 || gp.z >= res.z) continue;

                uint T = ((uint)gp.x & 1u)
                       | (((uint)gp.y & 1u) << 1)
                       | (((uint)gp.z & 1u) << 2);
                cb |= (1u << T);

                uint cellID = (uint)gp.z * params.gridRes.y * params.gridRes.x
                            + (uint)gp.y * params.gridRes.x
                            + (uint)gp.x;
                bool phantom = (oi != 0) || (oj != 0) || (ok != 0);
                uint value = (id << 4)
                           | (phantom ? (1u << 3) : 0u)
                           | homeT;

                entries[base + slot].cellID = cellID;
                entries[base + slot].value  = value;
                slot++;
            }
        }
    }

    // Sentinel-fill remaining slots so radix sort pushes them to the tail.
    for (uint k = slot; k < 8u; ++k) {
        entries[base + k].cellID = 0xFFFFFFFFu;
        entries[base + k].value  = 0xFFFFFFFFu;
    }

    faceCB[id] = (uchar)cb;
}

// ----- Step 6: cell ranges + nH/nP per cell -----
//
// Two kernels with a host-side exclusive scan in between.
//
//   sh_markStarts   : flag[i] = 1 iff entries[i] starts a new cell.
//                     (Host then turns flags into per-entry cellIdx via
//                      inclusive-prefix-sum-minus-1.)
//   sh_fillCellProp : per entry, atomically bump nH or nP on its cell;
//                     the start thread also writes start, cellID-derived
//                     cellType.

kernel void sh_markStarts(
    device const SHEntry* entries [[buffer(0)]],
    constant     uint&    numValid [[buffer(1)]],
    device       uint*    flag     [[buffer(2)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numValid) return;
    uint cur = entries[id].cellID;
    flag[id] = (id == 0u || cur != entries[id - 1].cellID) ? 1u : 0u;
}

kernel void sh_fillCellProp(
    device const SHEntry*  entries    [[buffer(0)]],
    device const uint*     flag       [[buffer(1)]],
    device const uint*     cellIdxOf  [[buffer(2)]],   // inclusive scan - 1
    constant     uint&     numValid   [[buffer(3)]],
    constant     SHParams& params     [[buffer(4)]],
    device       CellProp* cellProp   [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numValid) return;

    uint     cellIdx = cellIdxOf[id];
    SHEntry  e       = entries[id];
    bool     phantom = ((e.value >> 3) & 1u) != 0u;

    if (phantom) {
        atomic_fetch_add_explicit(&cellProp[cellIdx].nP, 1u, memory_order_relaxed);
    } else {
        atomic_fetch_add_explicit(&cellProp[cellIdx].nH, 1u, memory_order_relaxed);
    }

    if (flag[id] != 0u) {
        cellProp[cellIdx].start = id;
        uint cellID = e.cellID;
        uint gx = cellID % params.gridRes.x;
        uint gy = (cellID / params.gridRes.x) % params.gridRes.y;
        uint gz = cellID / (params.gridRes.x * params.gridRes.y);
        cellProp[cellIdx].cellType = (gx & 1u)
                                   | ((gy & 1u) << 1)
                                   | ((gz & 1u) << 2);
    }
}

// ----- Step 7: per-cell pair count -----
//
// Phantom-only cells (nH == 0) contribute 0 — fictitious-phantom promotion
// (Pabst Fig. 3) is deferred to a later optimisation pass; v1 accepts the
// rare miss in exchange for a simpler pipeline.

kernel void sh_computePairCount(
    device       CellProp* cellProp  [[buffer(0)]],
    constant     uint&     numCells  [[buffer(1)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numCells) return;
    uint nH = atomic_load_explicit(&cellProp[id].nH, memory_order_relaxed);
    uint nP = atomic_load_explicit(&cellProp[id].nP, memory_order_relaxed);
    uint pairs = (nH * (nH - 1u)) / 2u + nH * nP;
    cellProp[id].pairCnt = pairs;
}

// ----- Step 8: per-candidate-pair broad-phase emit -----
//
// One thread per candidate pair (P total). The thread:
//   1. binary-searches pairPrefix to find its (cellIdx, localPairId)
//   2. unranks localPairId into (a, b) = either two home entries or
//      (home, phantom)
//   3. walks the cell to find the a-th home and b-th home/phantom indices
//      (entries within a cell are not reordered after radix sort, so we
//      must scan; cell sizes are small so the loop is cheap)
//   4. applies inter-object gate, Float-Float skip, marking-CB dedup,
//      sphere-sphere BV cull
//   5. emits 6 BroadCollision rows (3 vertices of A vs triB, 3 of B vs triA)
//      via atomic counter into the shared output buffer

struct SHBroadParams {
    uint  numCandidatePairs;
    uint  numCells;
    uint  maxNumCollisions;
    uint  enableSelfCollisions; // 0 == off, 1 == on
    float epsilon;
};

inline uint sh_findCellIdx(
    uint gpid,
    uint numCells,
    device const uint* pairPrefix
) {
    // smallest cellIdx s.t. pairPrefix[cellIdx+1] > gpid
    uint lo = 0u;
    uint hi = numCells;
    while (lo < hi) {
        uint mid = lo + (hi - lo) / 2u;
        if (pairPrefix[mid + 1u] <= gpid) lo = mid + 1u;
        else                              hi = mid;
    }
    return lo;
}

// Walk the cell's nH+nP entries, return offset (relative to cp.start) of
// the k-th home (phantomBit==0) or phantom (phantomBit==1) entry.
inline uint sh_kthInCell(
    device const SHEntry* entries,
    uint cpStart,
    uint nTotal,
    uint k,
    uint phantomBit
) {
    uint count = 0u;
    for (uint i = 0u; i < nTotal; ++i) {
        uint v  = entries[cpStart + i].value;
        uint pb = (v >> 3) & 1u;
        if (pb == phantomBit) {
            if (count == k) return i;
            count++;
        }
    }
    return 0u; // should not be reached if pair count was correct
}

kernel void sh_broadPhase(
    device const SHEntry*       entries                  [[buffer(0)]],
    device const CellProp*      cellProp                 [[buffer(1)]],
    device const uint*          pairPrefix               [[buffer(2)]],
    device const uchar*         faceCB                   [[buffer(3)]],
    device const uint*          faceObj                  [[buffer(4)]],
    device const packed_uint3*  scenePackedFacets        [[buffer(5)]],
    device const uint*          scenePackedFacetsOffsets [[buffer(6)]],
    device const packed_float3* centroid                 [[buffer(7)]],
    device const float*         radiusBuf                [[buffer(8)]],
    device const uint*          meshBehaviors            [[buffer(9)]],
    device const uint*          meshShapes               [[buffer(10)]],
    constant     SHBroadParams& params                   [[buffer(11)]],
    device atomic_uint*         numBroadCollisions       [[buffer(12)]],
    device BroadCollision*      broadCollisions          [[buffer(13)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numCandidatePairs) return;

    // 1. Locate cell.
    uint cellIdx  = sh_findCellIdx(id, params.numCells, pairPrefix);
    uint cpStart  = cellProp[cellIdx].start;
    uint nH       = atomic_load_explicit(
                        (device atomic_uint*)&cellProp[cellIdx].nH,
                        memory_order_relaxed);
    uint nP       = atomic_load_explicit(
                        (device atomic_uint*)&cellProp[cellIdx].nP,
                        memory_order_relaxed);
    uint cellType = cellProp[cellIdx].cellType;
    uint k        = id - pairPrefix[cellIdx];

    // 2. Unrank pair into (offsetA, offsetB) within the cell.
    uint nHH = (nH * (nH - 1u)) / 2u;
    uint offA, offB;
    if (k < nHH) {
        // home-home: simple sequential unrank for typically-small nH.
        uint a = 0u;
        uint cumulative = 0u;
        // rows have size (nH-1), (nH-2), ..., 1
        while (cumulative + (nH - 1u - a) <= k) {
            cumulative += (nH - 1u - a);
            a++;
        }
        uint b = a + 1u + (k - cumulative);
        offA = sh_kthInCell(entries, cpStart, nH + nP, a, 0u);
        offB = sh_kthInCell(entries, cpStart, nH + nP, b, 0u);
    } else {
        uint k2 = k - nHH;
        uint a  = k2 / nP;  // home index
        uint b  = k2 % nP;  // phantom index
        offA = sh_kthInCell(entries, cpStart, nH + nP, a, 0u);
        offB = sh_kthInCell(entries, cpStart, nH + nP, b, 1u);
    }

    uint faceA = entries[cpStart + offA].value >> 4;
    uint faceB = entries[cpStart + offB].value >> 4;

    // 3. Self-collision gate (v1: self off).
    uint objA = faceObj[faceA];
    uint objB = faceObj[faceB];
    if (params.enableSelfCollisions == 0u && objA == objB) return;

    // 4. Float-Float skip (BehaviorType::Float == 4).
    uint behaviorA = meshBehaviors[objA];
    uint behaviorB = meshBehaviors[objB];
    if (behaviorA == 4u && behaviorB == 4u) return;

    // 5. Marking-scheme dedup: bail if any cell-type < currentT is
    //    common to both faces' inclusion masks.
    uint common    = (uint)faceCB[faceA] & (uint)faceCB[faceB];
    uint maskBelow = (1u << cellType) - 1u;
    if ((common & maskBelow) != 0u) return;

    // 6. Sphere-sphere BV cull.
    float3 cA = float3(centroid[faceA]);
    float3 cB = float3(centroid[faceB]);
    float  d  = distance(cA, cB);
    if (d > radiusBuf[faceA] + radiusBuf[faceB] + params.epsilon) return;

    // 7. Emit 6 BroadCollision rows. Local indices are facet-array offsets
    //    inside their owning mesh; statesOffsets/facetsOffsets translate
    //    to globals on the narrow side.
    uint3 triA       = scenePackedFacets[faceA];
    uint3 triB       = scenePackedFacets[faceB];
    uint  localFA    = faceA - scenePackedFacetsOffsets[objA];
    uint  localFB    = faceB - scenePackedFacetsOffsets[objB];
    uint  shapeA     = meshShapes[objA];
    uint  shapeB     = meshShapes[objB];

    // A's three vertices vs B's triangle.
    {
        uint vIds[3] = { triA.x, triA.y, triA.z };
        for (uint v = 0u; v < 3u; ++v) {
            uint outIdx = atomic_fetch_add_explicit(
                numBroadCollisions, 1u, memory_order_relaxed);
            if (outIdx >= params.maxNumCollisions) return;
            broadCollisions[outIdx].indexPair    = uint2(vIds[v], localFB);
            broadCollisions[outIdx].objPair      = uint2(objA, objB);
            broadCollisions[outIdx].behaviorPair = uint2(behaviorA, behaviorB);
            broadCollisions[outIdx].shapePair    = uint2(shapeA, shapeB);
        }
    }
    // B's three vertices vs A's triangle.
    {
        uint vIds[3] = { triB.x, triB.y, triB.z };
        for (uint v = 0u; v < 3u; ++v) {
            uint outIdx = atomic_fetch_add_explicit(
                numBroadCollisions, 1u, memory_order_relaxed);
            if (outIdx >= params.maxNumCollisions) return;
            broadCollisions[outIdx].indexPair    = uint2(vIds[v], localFA);
            broadCollisions[outIdx].objPair      = uint2(objB, objA);
            broadCollisions[outIdx].behaviorPair = uint2(behaviorB, behaviorA);
            broadCollisions[outIdx].shapePair    = uint2(shapeB, shapeA);
        }
    }
}
