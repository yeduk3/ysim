#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"

// ─── Data structures ────────────────────────────────────────────────

struct SHParams {
    uint  numTriangles;       // total triangles in scene
    float cellSize;           // 2 * largestRadius * scaleFactor
    float invCellSize;        // 1 / cellSize
    int   resX;               // grid resolution along X
    int   resY;               // grid resolution along Y
    int   resZ;               // grid resolution along Z
    uint  tableSize;          // hash table capacity (= numTriangles, or next-power-of-two)
    uint  maxNumCollisions;   // max BroadCollision output
};

struct SHTriEntry {
    uint hashValue;           // H(x,y,z)
    uint triIndex;            // original triangle index
    uint objId;               // object that owns this triangle
    uint behavior;            // BehaviorType
    uint shape;               // ShapeType
    uint controlBits;         // bits [0..2] = home cell type, bits [3..10] = 8 inclusion bits
    uint isHome;              // 1 if home cell, 0 if phantom
    uint _pad;
};

struct SHCellRange {
    uint start;
    uint end;
    uint numHome;
    uint numPhantom;
};

struct SHCollisionPassParams {
    uint  numCells;
    uint  passIndex;          // 0..7
    uint  maxNumCollisions;
    uint  numTriangles;
    float radius;
    float thickness;
};

// ─── Helpers ────────────────────────────────────────────────────────

inline int3 gridCoord(float3 centroid, float invCellSize) {
    return int3(floor(centroid * invCellSize));
}

inline uint gridHashFull(int3 gc, int resX, int resY, int resZ) {
    int x = ((gc.x % resX) + resX) % resX;
    int y = ((gc.y % resY) + resY) % resY;
    int z = ((gc.z % resZ) + resZ) % resZ;
    return uint(z) * uint(resY) * uint(resX) + uint(y) * uint(resX) + uint(x);
}

inline uint cellType(int3 gc) {
    // (x%2)*4 + (y%2)*2 + (z%2)
    return uint(((gc.x & 1) << 2) | ((gc.y & 1) << 1) | (gc.z & 1));
}

// ─── Kernel 1: Compute bounding spheres per triangle ────────────────

kernel void sh_computeBoundingSpheres(
    device float*               radii      [[buffer(0)]],  // output: radius per triangle
    device packed_float3*       centroids  [[buffer(1)]],  // output: centroid per triangle
    constant uint&              numTris    [[buffer(2)]],
    device const packed_float3* x          [[buffer(3)]],
    device const packed_uint3*  facets     [[buffer(4)]],
    device const uint*          triObjIds  [[buffer(5)]],
    device const uint*          statesOffsets [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numTris) return;

    uint objId = triObjIds[id];
    uint voff = statesOffsets[objId];

    uint3 f = facets[id];
    float3 v0 = float3(x[f.x + voff]);
    float3 v1 = float3(x[f.y + voff]);
    float3 v2 = float3(x[f.z + voff]);

    float3 c = (v0 + v1 + v2) / 3.0f;
    float r0 = length(v0 - c);
    float r1 = length(v1 - c);
    float r2 = length(v2 - c);
    float r = max(r0, max(r1, r2));

    centroids[id] = c;
    radii[id] = r;
}

// ─── Kernel 2: Parallel reduction to find max radius ────────────────

kernel void sh_reduceMaxRadius(
    device const float* radii     [[buffer(0)]],
    device float*       partials  [[buffer(1)]],  // partial max per threadgroup
    constant uint&      numTris   [[buffer(2)]],
    uint id  [[thread_position_in_grid]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup float shared[256];

    float val = (id < numTris) ? radii[id] : 0.0f;
    shared[lid] = val;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint s = 128; s > 0; s >>= 1) {
        if (lid < s) {
            shared[lid] = max(shared[lid], shared[lid + s]);
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (lid == 0) {
        partials[gid] = shared[0];
    }
}

// Final reduction (single threadgroup)
kernel void sh_reduceMaxRadiusFinal(
    device float*   maxRadius  [[buffer(0)]],   // output: single float
    device float*   partials   [[buffer(1)]],   // reuses buf1 from reduceMaxRadius
    constant uint&  numGroups  [[buffer(2)]],
    uint lid [[thread_index_in_threadgroup]]
) {
    threadgroup float shared[256];

    shared[lid] = (lid < numGroups) ? partials[lid] : 0.0f;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    for (uint s = 128; s > 0; s >>= 1) {
        if (lid < s) {
            shared[lid] = max(shared[lid], shared[lid + s]);
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
    }

    if (lid == 0) {
        maxRadius[0] = shared[0];
    }
}

// ─── Kernel 3: Assign grid cells and build entries ──────────────────
// Each triangle produces 1 home entry + up to 7 phantom entries = up to 8 entries total.
// We write into a flat array of size numTriangles * 8 (worst case).
// A separate count array tracks how many entries each triangle actually produces.

kernel void sh_assignCells(
    device const packed_float3* centroids [[buffer(0)]],
    device const float*     radii       [[buffer(1)]],
    constant SHParams&      params      [[buffer(2)]],
    device SHTriEntry*      entries     [[buffer(3)]],  // size = numTriangles * 8
    device uint*            entryCounts [[buffer(4)]],   // size = numTriangles, how many entries per tri
    // per-triangle object info
    device const uint*      triObjIds     [[buffer(5)]],
    device const uint*      triBehaviors  [[buffer(6)]],
    device const uint*      triShapes     [[buffer(7)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numTriangles) return;

    float3 c = centroids[id];
    float  r = radii[id];

    int3 homeGC = gridCoord(c, params.invCellSize);
    uint homeCT = cellType(homeGC);

    // Determine which of the 8 surrounding cells the triangle overlaps.
    // The home cell is always included. For each of the 8 cells at offsets (dx,dy,dz) in {0,-1},
    // check if the sphere overlaps that cell.
    uint inclusionBits = 0;
    uint count = 0;
    uint baseIdx = id * 8;

    uint objId = triObjIds[id];
    uint behavior = triBehaviors[id];
    uint shape = triShapes[id];

    for (int dz = 0; dz >= -1; dz--) {
        for (int dy = 0; dy >= -1; dy--) {
            for (int dx = 0; dx >= -1; dx--) {
                int3 gc = homeGC + int3(dx, dy, dz);
                uint ct = cellType(gc);

                // Check if sphere overlaps this cell
                float3 cellMin = float3(gc) * params.cellSize;
                float3 cellMax = cellMin + float3(params.cellSize);

                // Closest point on cell AABB to centroid
                float3 closest = clamp(c, cellMin, cellMax);
                float dist = length(c - closest);

                if (dist <= r || (dx == 0 && dy == 0 && dz == 0)) {
                    // This cell includes the triangle
                    inclusionBits |= (1u << ct);

                    uint hash = gridHashFull(gc, params.resX, params.resY, params.resZ);
                    bool isHome = (dx == 0 && dy == 0 && dz == 0);

                    SHTriEntry entry;
                    entry.hashValue = hash;
                    entry.triIndex = id;
                    entry.objId = objId;
                    entry.behavior = behavior;
                    entry.shape = shape;
                    entry.controlBits = 0; // filled later
                    entry.isHome = isHome ? 1u : 0u;
                    entry._pad = 0;

                    entries[baseIdx + count] = entry;
                    count++;
                }
            }
        }
    }

    // Now fill control bits: lower 3 bits = home cell type, upper 8 bits = inclusion
    uint ctrlBits = (homeCT & 0x7u) | (inclusionBits << 3);
    for (uint i = 0; i < count; i++) {
        entries[baseIdx + i].controlBits = ctrlBits;
    }

    entryCounts[id] = count;
}

// ─── Kernel 4: Compact entries using prefix sum offsets ──────────────

kernel void sh_compactEntries(
    device const SHTriEntry* entriesIn  [[buffer(0)]],  // size = numTri*8
    device const uint*       offsets    [[buffer(1)]],   // exclusive prefix sum of entryCounts
    device const uint*       counts     [[buffer(2)]],   // entryCounts
    device SHTriEntry*       entriesOut [[buffer(3)]],   // compacted
    constant uint&           numTris    [[buffer(4)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numTris) return;

    uint srcBase = id * 8;
    uint dstBase = offsets[id];
    uint cnt = counts[id];

    for (uint i = 0; i < cnt; i++) {
        entriesOut[dstBase + i] = entriesIn[srcBase + i];
    }
}

// ─── Kernel 5: Prefix sum (exclusive) for entry counts ──────────────
// Simple Blelloch-style scan for small arrays. For large arrays, use multi-block.

kernel void sh_prefixSumExclusive(
    device uint*    data      [[buffer(0)]],   // in: counts, out: exclusive prefix sum
    device uint*    totalOut  [[buffer(1)]],   // output: total sum
    constant uint&  n         [[buffer(2)]],
    uint lid [[thread_index_in_threadgroup]]
) {
    // This kernel works for n <= 256. For larger, dispatch multiple groups.
    threadgroup uint shared[512];

    // Load
    shared[lid]       = (lid < n)       ? data[lid]       : 0;
    shared[lid + 256] = (lid + 256 < n) ? data[lid + 256] : 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint count = min(n, 512u);

    // Up-sweep
    uint offset = 1;
    for (uint d = count >> 1; d > 0; d >>= 1) {
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (lid < d) {
            uint ai = offset * (2 * lid + 1) - 1;
            uint bi = offset * (2 * lid + 2) - 1;
            if (bi < count) shared[bi] += shared[ai];
        }
        offset <<= 1;
    }

    // Clear last
    if (lid == 0) {
        totalOut[0] = shared[count - 1];
        shared[count - 1] = 0;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Down-sweep
    for (uint d = 1; d < count; d <<= 1) {
        offset >>= 1;
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (lid < d) {
            uint ai = offset * (2 * lid + 1) - 1;
            uint bi = offset * (2 * lid + 2) - 1;
            if (bi < count) {
                uint t = shared[ai];
                shared[ai] = shared[bi];
                shared[bi] += t;
            }
        }
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Write back
    if (lid < n)       data[lid]       = shared[lid];
    if (lid + 256 < n) data[lid + 256] = shared[lid + 256];
}

// Large prefix sum: per-block scan + combine
kernel void sh_prefixSumBlocks(
    device uint*    data        [[buffer(0)]],
    device uint*    blockSums   [[buffer(1)]],  // one sum per block
    constant uint&  n           [[buffer(2)]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup uint shared[512];

    uint base = gid * 512;
    uint i0 = base + lid;
    uint i1 = base + lid + 256;

    shared[lid]       = (i0 < n) ? data[i0] : 0;
    shared[lid + 256] = (i1 < n) ? data[i1] : 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint blockN = min(512u, (i0 < n) ? min(512u, n - base) : 0);
    if (blockN == 0) blockN = min(512u, n - base);

    // Up-sweep
    uint offset = 1;
    for (uint d = 256; d > 0; d >>= 1) {
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (lid < d) {
            uint ai = offset * (2 * lid + 1) - 1;
            uint bi = offset * (2 * lid + 2) - 1;
            if (bi < 512) shared[bi] += shared[ai];
        }
        offset <<= 1;
    }

    if (lid == 0) {
        blockSums[gid] = shared[511];
        shared[511] = 0;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Down-sweep
    for (uint d = 1; d < 256; d <<= 1) {
        offset >>= 1;
        threadgroup_barrier(mem_flags::mem_threadgroup);
        if (lid < d) {
            uint ai = offset * (2 * lid + 1) - 1;
            uint bi = offset * (2 * lid + 2) - 1;
            if (bi < 512) {
                uint t = shared[ai];
                shared[ai] = shared[bi];
                shared[bi] += t;
            }
        }
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (i0 < n) data[i0] = shared[lid];
    if (i1 < n) data[i1] = shared[lid + 256];
}

kernel void sh_prefixSumAddBlockSums(
    device uint*        data      [[buffer(0)]],
    device const uint*  blockSums [[buffer(1)]],
    constant uint&      n         [[buffer(2)]],
    uint id  [[thread_position_in_grid]],
    uint gid [[threadgroup_position_in_grid]]
) {
    if (gid == 0) return;  // first block has no offset
    if (id >= n) return;
    data[id] += blockSums[gid];
}


// ─── Sort key for lightweight radix sort ────────────────────────────

struct SHSortKey {
    uint hashValue;
    uint index;       // index into the original compacted entries array
};

// ─── Radix sort for SHTriEntry by hashValue ─────────────────────────
// Reuse the same radix sort pattern as BVH (8-bit passes, 4 passes for 32-bit keys).

struct SHRadixParams {
    uint numElements;
    uint shift;
    uint numBlocks;
};

// Unified buffer layout for radix sort:
//   buf0 = src, buf1 = dst, buf2 = params, buf3 = blockHist, buf4 = blockOff, buf5 = bucketBase

kernel void sh_radixCount(
    device const SHTriEntry* src       [[buffer(0)]],
    constant SHRadixParams&  params    [[buffer(2)]],
    device uint*             blockHist [[buffer(3)]],
    uint tid [[thread_position_in_grid]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup uint localHist[256];

    localHist[lid] = 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint idx = gid * 256 + lid;
    if (idx < params.numElements) {
        uint bucket = (src[idx].hashValue >> params.shift) & 0xFF;
        atomic_fetch_add_explicit((threadgroup atomic_uint*)&localHist[bucket], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    blockHist[gid * 256 + lid] = localHist[lid];
}

kernel void sh_radixComputeOffsets(
    constant SHRadixParams& params      [[buffer(2)]],
    device const uint*      blockHist   [[buffer(3)]],
    device uint*            blockOff    [[buffer(4)]],
    device uint*            bucketBase  [[buffer(5)]],
    uint tid [[thread_position_in_grid]]
) {
    // Single-thread kernel: compute prefix sums
    if (tid > 0) return;

    uint numBlocks = params.numBlocks;

    // For each bucket b, scan across blocks
    for (uint b = 0; b < 256; b++) {
        uint sum = 0;
        for (uint blk = 0; blk < numBlocks; blk++) {
            blockOff[blk * 256 + b] = sum;
            sum += blockHist[blk * 256 + b];
        }
        bucketBase[b] = sum;
    }

    // Exclusive prefix sum of bucketBase
    uint total = 0;
    for (uint b = 0; b < 256; b++) {
        uint v = bucketBase[b];
        bucketBase[b] = total;
        total += v;
    }
}

kernel void sh_radixScatter(
    device const SHTriEntry* src        [[buffer(0)]],
    device SHTriEntry*       dst        [[buffer(1)]],
    constant SHRadixParams&  params     [[buffer(2)]],
    device const uint*       blockOff   [[buffer(4)]],
    device const uint*       bucketBase [[buffer(5)]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    // Reconstruct local histogram prefix for this block
    threadgroup uint localPrefix[256];
    threadgroup uint localCount[256];

    localCount[lid] = 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint idx = gid * 256 + lid;
    uint bucket = 0;
    if (idx < params.numElements) {
        bucket = (src[idx].hashValue >> params.shift) & 0xFF;
        atomic_fetch_add_explicit((threadgroup atomic_uint*)&localCount[bucket], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Build local exclusive prefix
    if (lid == 0) {
        uint sum = 0;
        for (uint b = 0; b < 256; b++) {
            localPrefix[b] = sum;
            sum += localCount[b];
        }
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (idx < params.numElements) {
        // Position within local block for this bucket
        uint localPos = atomic_fetch_add_explicit((threadgroup atomic_uint*)&localPrefix[bucket], 1u, memory_order_relaxed);
        uint globalPos = bucketBase[bucket] + blockOff[gid * 256 + bucket] + localPos;
        dst[globalPos] = src[idx];
    }
}

// ─── Lightweight radix sort (SHSortKey = 8 bytes) ───────────────────

kernel void sh_buildSortKeys(
    device const SHTriEntry* entries [[buffer(0)]],
    device SHSortKey*        keys    [[buffer(1)]],
    constant uint&           n       [[buffer(2)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= n) return;
    keys[id].hashValue = entries[id].hashValue;
    keys[id].index = id;
}

kernel void sh_gatherEntries(
    device const SHTriEntry* entriesIn  [[buffer(0)]],
    device const SHSortKey*  sortedKeys [[buffer(1)]],
    device SHTriEntry*       entriesOut [[buffer(2)]],
    constant uint&           n          [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= n) return;
    entriesOut[id] = entriesIn[sortedKeys[id].index];
}

// Unified buffer layout: buf0=src, buf1=dst, buf2=params, buf3=blockHist, buf4=blockOff, buf5=bucketBase

kernel void sh_radixCountKeys(
    device const SHSortKey*  src       [[buffer(0)]],
    constant SHRadixParams&  params    [[buffer(2)]],
    device uint*             blockHist [[buffer(3)]],
    uint tid [[thread_position_in_grid]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup uint localHist[256];
    localHist[lid] = 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint idx = gid * 256 + lid;
    if (idx < params.numElements) {
        uint bucket = (src[idx].hashValue >> params.shift) & 0xFF;
        atomic_fetch_add_explicit((threadgroup atomic_uint*)&localHist[bucket], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    blockHist[gid * 256 + lid] = localHist[lid];
}

kernel void sh_radixScatterKeys(
    device const SHSortKey*  src        [[buffer(0)]],
    device SHSortKey*        dst        [[buffer(1)]],
    constant SHRadixParams&  params     [[buffer(2)]],
    device const uint*       blockOff   [[buffer(4)]],
    device const uint*       bucketBase [[buffer(5)]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup uint localPrefix[256];
    threadgroup uint localCount[256];

    localCount[lid] = 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint idx = gid * 256 + lid;
    uint bucket = 0;
    if (idx < params.numElements) {
        bucket = (src[idx].hashValue >> params.shift) & 0xFF;
        atomic_fetch_add_explicit((threadgroup atomic_uint*)&localCount[bucket], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (lid == 0) {
        uint sum = 0;
        for (uint b = 0; b < 256; b++) {
            localPrefix[b] = sum;
            sum += localCount[b];
        }
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (idx < params.numElements) {
        uint localPos = atomic_fetch_add_explicit((threadgroup atomic_uint*)&localPrefix[bucket], 1u, memory_order_relaxed);
        uint globalPos = bucketBase[bucket] + blockOff[gid * 256 + bucket] + localPos;
        dst[globalPos] = src[idx];
    }
}

// ─── Kernel 6: Scan sorted entries for cell boundaries ──────────────

kernel void sh_findCellRanges(
    device const SHTriEntry* sortedEntries [[buffer(0)]],
    device SHCellRange*      cellRanges    [[buffer(1)]],  // size = tableSize
    constant uint&           numEntries    [[buffer(2)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numEntries) return;

    uint myHash = sortedEntries[id].hashValue;

    // Detect start of a new cell
    bool isStart = (id == 0) || (sortedEntries[id - 1].hashValue != myHash);
    // Detect end of a cell
    bool isEnd = (id == numEntries - 1) || (sortedEntries[id + 1].hashValue != myHash);

    if (isStart) {
        uint cellIdx = myHash;
        cellRanges[cellIdx].start = id;
    }
    if (isEnd) {
        uint cellIdx = myHash;
        cellRanges[cellIdx].end = id + 1;  // exclusive end
    }
}

// Count home/phantom per cell
kernel void sh_countHomePhantom(
    device const SHTriEntry* sortedEntries [[buffer(0)]],
    device SHCellRange*      cellRanges    [[buffer(1)]],
    device const uint*       activeCells   [[buffer(2)]],  // list of active cell indices
    constant uint&           numActiveCells [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numActiveCells) return;

    uint cellIdx = activeCells[id];
    uint start = cellRanges[cellIdx].start;
    uint end   = cellRanges[cellIdx].end;

    uint homeCount = 0;
    uint phantomCount = 0;
    for (uint i = start; i < end; i++) {
        if (sortedEntries[i].isHome) homeCount++;
        else phantomCount++;
    }

    cellRanges[cellIdx].numHome = homeCount;
    cellRanges[cellIdx].numPhantom = phantomCount;
}

// ─── Kernel 7: Collect active cells ─────────────────────────────────

kernel void sh_collectActiveCells(
    device const SHTriEntry* sortedEntries  [[buffer(0)]],  // reuses buf0 from findCellRanges
    device uint*             activeCells    [[buffer(1)]],
    constant uint&           numEntries     [[buffer(2)]],   // reuses buf2 from findCellRanges
    device atomic_uint*      numActiveCells [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numEntries) return;

    uint myHash = sortedEntries[id].hashValue;
    bool isStart = (id == 0) || (sortedEntries[id - 1].hashValue != myHash);

    if (isStart) {
        uint slot = atomic_fetch_add_explicit(numActiveCells, 1u, memory_order_relaxed);
        activeCells[slot] = myHash;
    }
}

// ─── Kernel 8: Handle phantom-phantom pairs ─────────────────────────
// For cells with only phantom entries, check pairs and promote one to home
// if their triangles' home cells don't share a common cell type.

kernel void sh_handlePhantomPhantom(
    device SHTriEntry*       sortedEntries  [[buffer(0)]],
    device const SHCellRange* cellRanges    [[buffer(1)]],
    device const uint*       activeCells    [[buffer(2)]],
    constant uint&           numActiveCells [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= numActiveCells) return;

    uint cellIdx = activeCells[id];
    uint start = cellRanges[cellIdx].start;
    uint end   = cellRanges[cellIdx].end;
    uint numHome = cellRanges[cellIdx].numHome;

    // Only process cells that have phantom-phantom pairs (numHome == 0)
    if (numHome > 0) return;
    if (end - start < 2) return;

    // For each pair of phantoms in this cell, check if they can see each other
    // through any home cell. If not, promote one to home.
    for (uint i = start; i < end; i++) {
        if (sortedEntries[i].isHome) continue;

        uint ctrlA = sortedEntries[i].controlBits;
        uint inclusionA = ctrlA >> 3;

        for (uint j = i + 1; j < end; j++) {
            if (sortedEntries[j].isHome) continue;

            uint ctrlB = sortedEntries[j].controlBits;
            uint inclusionB = ctrlB >> 3;

            // Check if they share any common home cell type
            // Triangle A's home cell type
            uint homeTypeA = ctrlA & 0x7;
            uint homeTypeB = ctrlB & 0x7;

            // If A is included in B's home cell type, or B is included in A's home cell type,
            // they will be checked in the home-phantom pass of that cell.
            bool coveredByA = (inclusionB & (1u << homeTypeA)) != 0;
            bool coveredByB = (inclusionA & (1u << homeTypeB)) != 0;

            if (!coveredByA && !coveredByB) {
                // Promote the first phantom to home for this cell
                sortedEntries[i].isHome = 1;
                break;
            }
        }
        if (sortedEntries[i].isHome) break;
    }
}


// ─── Kernel 9: 8-pass collision detection with inline narrow phase ──
// Directly outputs NarrowCollision instead of BroadCollision.
// For each triangle pair, checks all 6 point-triangle combinations
// and only emits those that pass the distance + barycentric test.

// BehaviorType::Float == 4 (must match CPU enum)
constant constexpr uint BEHAVIOR_FLOAT = 4;

inline bool sh_pointInTriangleBary(float3 p, float3 e0, float3 e1) {
    float d00 = dot(e0, e0);
    float d01 = dot(e0, e1);
    float d11 = dot(e1, e1);
    float d20 = dot(p,  e0);
    float d21 = dot(p,  e1);
    float denom = d00 * d11 - d01 * d01;
    if (fabs(denom) < 1e-12f) return false;
    float b = (d11 * d20 - d01 * d21) / denom;
    float c = (d00 * d21 - d01 * d20) / denom;
    float a = 1.0f - b - c;
    return (a >= 0.0f && b >= 0.0f && c >= 0.0f);
}

// Check one point against one triangle; emit NarrowCollision if close enough.
inline void narrowCheckPT(
    uint point, uint localTri,
    uint qObj, uint tObj,
    uint behQ, uint behT,
    uint shpQ, uint shpT,
    device const packed_float3* sceneX,
    device const uint*          statesOffsets,
    device const packed_uint3*  sceneFacets,
    device const uint*          facetsOffsets,
    device const uint*          vertexAdjFacets,
    device const uint*          vertexAdjFacetsOffsets,
    device NarrowCollision*     narrowCollisions,
    device atomic_uint*         numNarrowCollisions,
    uint maxNumCollisions,
    float radius, float thickness
) {
    uint globalTri = localTri + facetsOffsets[tObj];
    uint3 tri = sceneFacets[globalTri];

    // Self-collision adjacency filter
    if (qObj == tObj) {
        uint gPoint = point + statesOffsets[tObj];
        for (uint i = vertexAdjFacetsOffsets[gPoint]; i < vertexAdjFacetsOffsets[gPoint + 1]; i++) {
            if (vertexAdjFacets[i] == localTri) return;
        }
    }

    float3 qpos  = float3(sceneX[point + statesOffsets[qObj]]);
    float3 t0pos = float3(sceneX[tri.x + statesOffsets[tObj]]);
    float3 t1pos = float3(sceneX[tri.y + statesOffsets[tObj]]);
    float3 t2pos = float3(sceneX[tri.z + statesOffsets[tObj]]);

    float3 e0 = t1pos - t0pos;
    float3 e1 = t2pos - t0pos;

    float3 n = cross(e0, e1);
    float nlen2 = dot(n, n);
    if (nlen2 < 1e-12f) return;
    n *= rsqrt(nlen2);

    float3 p = qpos - t0pos;
    float l = dot(n, p);
    if (l < 0.0f) { n = -n; l = -l; }

    if (l > radius + thickness) return;

    float3 inplane = p - n * l;
    if (!sh_pointInTriangleBary(inplane, e0, e1)) return;

    uint outIdx = atomic_fetch_add_explicit(numNarrowCollisions, 1u, memory_order_relaxed);
    if (outIdx >= maxNumCollisions) return;

    narrowCollisions[outIdx].indexPair = uint2(point, localTri);
    narrowCollisions[outIdx].objPair = uint2(qObj, tObj);
    narrowCollisions[outIdx].collisionNormalAndDistance = float4(n, l);
    narrowCollisions[outIdx].behaviorPair = uint2(behQ, behT);
    narrowCollisions[outIdx].shapePair = uint2(shpQ, shpT);
}

// Helper: narrow-check all 6 point-triangle pairs for a triangle pair (A vs B)
inline void narrowCheckTriPair(
    uint globalTriA, uint globalTriB,
    uint objA, uint objB,
    uint behA, uint behB,
    uint shpA, uint shpB,
    device const packed_float3* sceneX,
    device const packed_uint3*  sceneFacets,
    device const uint*          statesOffsets,
    device const uint*          facetsOffsets,
    device const uint*          vertexAdjFacets,
    device const uint*          vertexAdjFacetsOffsets,
    device NarrowCollision*     narrowCollisions,
    device atomic_uint*         numNarrowCollisions,
    uint maxNumCollisions,
    float radius, float thickness
) {
    if (behA == BEHAVIOR_FLOAT && behB == BEHAVIOR_FLOAT) return;

    uint localTriA = globalTriA - facetsOffsets[objA];
    uint localTriB = globalTriB - facetsOffsets[objB];

    uint3 fA = sceneFacets[globalTriA];
    uint3 fB = sceneFacets[globalTriB];

    // 3 vertices of A against triangle B
    uint vidsA[3] = { fA.x, fA.y, fA.z };
    for (uint v = 0; v < 3; v++) {
        narrowCheckPT(vidsA[v], localTriB, objA, objB, behA, behB, shpA, shpB,
                       sceneX, statesOffsets, sceneFacets, facetsOffsets,
                       vertexAdjFacets, vertexAdjFacetsOffsets,
                       narrowCollisions, numNarrowCollisions,
                       maxNumCollisions, radius, thickness);
    }
    // 3 vertices of B against triangle A
    uint vidsB[3] = { fB.x, fB.y, fB.z };
    for (uint v = 0; v < 3; v++) {
        narrowCheckPT(vidsB[v], localTriA, objB, objA, behB, behA, shpB, shpA,
                       sceneX, statesOffsets, sceneFacets, facetsOffsets,
                       vertexAdjFacets, vertexAdjFacetsOffsets,
                       narrowCollisions, numNarrowCollisions,
                       maxNumCollisions, radius, thickness);
    }
}

kernel void sh_collisionPass(
    device const SHTriEntry*  sortedEntries  [[buffer(0)]],
    device const SHCellRange* cellRanges     [[buffer(1)]],
    device const uint*        activeCells    [[buffer(2)]],
    constant SHCollisionPassParams& passParams [[buffer(3)]],
    device NarrowCollision*   narrowCollisions   [[buffer(4)]],
    device atomic_uint*       numNarrowCollisions [[buffer(5)]],
    device const packed_float3* sceneX       [[buffer(6)]],
    device const packed_uint3*  sceneFacets  [[buffer(7)]],
    device const uint*          statesOffsets [[buffer(8)]],
    device const uint*          facetsOffsets [[buffer(9)]],
    device const uint*          vertexAdjFacets        [[buffer(10)]],
    device const uint*          vertexAdjFacetsOffsets  [[buffer(11)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= passParams.numCells) return;

    uint cellIdx = activeCells[id];

    uint start = cellRanges[cellIdx].start;
    uint end   = cellRanges[cellIdx].end;

    if (start >= end) return;

    uint thisCellType = 0xFF;
    for (uint i = start; i < end; i++) {
        if (sortedEntries[i].isHome) {
            thisCellType = sortedEntries[i].controlBits & 0x7u;
            break;
        }
    }

    if (thisCellType == 0xFF) return;
    if (thisCellType != passParams.passIndex) return;

    constexpr uint MAX_PER_CELL = 128;
    uint homeIndices[MAX_PER_CELL];
    uint phantomIndices[MAX_PER_CELL];
    uint nHome = 0, nPhantom = 0;

    for (uint i = start; i < end; i++) {
        if (sortedEntries[i].isHome && nHome < MAX_PER_CELL) {
            homeIndices[nHome++] = i;
        } else if (!sortedEntries[i].isHome && nPhantom < MAX_PER_CELL) {
            phantomIndices[nPhantom++] = i;
        }
    }

    // Home-home pairs
    for (uint hi = 0; hi < nHome; hi++) {
        for (uint hj = hi + 1; hj < nHome; hj++) {
            uint eiA = homeIndices[hi];
            uint eiB = homeIndices[hj];
            narrowCheckTriPair(
                sortedEntries[eiA].triIndex, sortedEntries[eiB].triIndex,
                sortedEntries[eiA].objId,    sortedEntries[eiB].objId,
                sortedEntries[eiA].behavior, sortedEntries[eiB].behavior,
                sortedEntries[eiA].shape,    sortedEntries[eiB].shape,
                sceneX, sceneFacets, statesOffsets, facetsOffsets,
                vertexAdjFacets, vertexAdjFacetsOffsets,
                narrowCollisions, numNarrowCollisions,
                passParams.maxNumCollisions, passParams.radius, passParams.thickness
            );
        }
    }

    // Home-phantom pairs
    for (uint hi = 0; hi < nHome; hi++) {
        for (uint pi = 0; pi < nPhantom; pi++) {
            uint eiH = homeIndices[hi];
            uint eiP = phantomIndices[pi];

            uint ctrlH = sortedEntries[eiH].controlBits;
            uint ctrlP = sortedEntries[eiP].controlBits;
            uint phantomHomeType = ctrlP & 0x7u;

            if (phantomHomeType < passParams.passIndex) {
                uint inclusionH = ctrlH >> 3;
                uint inclusionP = ctrlP >> 3;
                if ((inclusionH & (1u << phantomHomeType)) != 0 &&
                    (inclusionP & (1u << phantomHomeType)) != 0) {
                    continue;
                }
            }

            narrowCheckTriPair(
                sortedEntries[eiH].triIndex, sortedEntries[eiP].triIndex,
                sortedEntries[eiH].objId,    sortedEntries[eiP].objId,
                sortedEntries[eiH].behavior, sortedEntries[eiP].behavior,
                sortedEntries[eiH].shape,    sortedEntries[eiP].shape,
                sceneX, sceneFacets, statesOffsets, facetsOffsets,
                vertexAdjFacets, vertexAdjFacetsOffsets,
                narrowCollisions, numNarrowCollisions,
                passParams.maxNumCollisions, passParams.radius, passParams.thickness
            );
        }
    }
}


// ─── Kernel: Clear cell ranges ──────────────────────────────────────

kernel void sh_clearCellRanges(
    device SHCellRange* cellRanges [[buffer(0)]],
    constant uint&      tableSize  [[buffer(1)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= tableSize) return;
    cellRanges[id].start = 0;
    cellRanges[id].end   = 0;
    cellRanges[id].numHome = 0;
    cellRanges[id].numPhantom = 0;
}
