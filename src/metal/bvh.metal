#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"

struct AABB4 {
    packed_float3 min;
    int _pad0;
    packed_float3 max;
    int _pad1;
};

struct BVHNode {
    packed_float3 min;
    int childA;
    packed_float3 max;
    int childB;
};

struct MortonNode {
    uint code;
    uint index;
};

inline uint expandBits(uint v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

inline uint mortonCode(const float3 point) {
    float3 p = min(max(point*1024.f, float3(0.f)), float3(1023.f));
    uint x = expandBits((uint)p.x);
    uint y = expandBits((uint)p.y);
    uint z = expandBits((uint)p.z);
    return x*4 + y*2 + z;
}

kernel void fillMortons_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant AABB4& sceneBox [[buffer(2)]],
    device MortonNode* mortons [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;
    
    uint3 facet = facets[id];
    float3 x0 = x[facet.x];
    float3 x1 = x[facet.y];
    float3 x2 = x[facet.z];

    float3 center = (x0+x1+x2)/3.0f;
    float3 width = max(sceneBox.max - sceneBox.min, float3(1e-8f));
    center = (center-sceneBox.min)/width;

    mortons[id].code = mortonCode(center);
    mortons[id].index = id;
}

kernel void fillMortons_Edge(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint2* edges [[buffer(1)]],
    constant AABB4& sceneBox [[buffer(2)]],
    device MortonNode* mortons [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;
    
    uint2 edge = edges[id];
    float3 x0 = x[edge.x];
    float3 x1 = x[edge.y];

    float3 center = (x0+x1)/2.0f;
    float3 width = max(sceneBox.max - sceneBox.min, float3(1e-8f));
    center = (center-sceneBox.min)/width;

    mortons[id].code = mortonCode(center);
    mortons[id].index = id;
}


//#define RADIX_BITS 8u
//#define RADIX (1u << RADIX_BITS)
//#define RADIX_MASK (RADIX - 1u)
//#define RADIX_BLOCK_SIZE 256u
//
//struct RadixSortParams {
//    uint numElements;
//    uint shift;
//    uint numBlocks;
//};
//
//inline uint radixBucket(uint code, uint shift) {
//    return (code >> shift) & RADIX_MASK;
//}
//
//kernel void radixCountMortonBlocks(
//    device const MortonNode* src [[buffer(0)]],
//    constant RadixSortParams& params [[buffer(1)]],
//    device uint* blockHistograms [[buffer(2)]], // size = numBlocks * 256
//    uint tid [[thread_position_in_grid]],
//    uint lid [[thread_index_in_threadgroup]], // per threads in a block(tg)
//    uint gid [[threadgroup_position_in_grid]] // per blocks(tg) in a grid
//) {
//    if (gid >= params.numBlocks) return;
//
//    threadgroup atomic_uint localHist[RADIX];
//
//    // init local histogram
//    if (lid < RADIX) {
//        atomic_store_explicit(&localHist[lid], 0u, memory_order_relaxed);
//    }
//    threadgroup_barrier(mem_flags::mem_threadgroup);
//
//    uint blockStart = gid * RADIX_BLOCK_SIZE;
//    uint idx = blockStart + lid;
//
//    // one block = one threadgroup
//    if (idx < params.numElements) {
//        uint b = radixBucket(src[idx].code, params.shift);
//        atomic_fetch_add_explicit(&localHist[b], 1u, memory_order_relaxed);
//    }
//
//    threadgroup_barrier(mem_flags::mem_threadgroup);
//
//    // flush to global
//    if (lid < RADIX) {
//        blockHistograms[gid * RADIX + lid] =
//            atomic_load_explicit(&localHist[lid], memory_order_relaxed);
//    }
//}
//
//kernel void radixComputeOffsets(
//    constant RadixSortParams& params [[buffer(0)]],
//    device const uint* blockHistograms [[buffer(1)]], // numBlocks * 256
//    device uint* blockOffsets [[buffer(2)]],          // numBlocks * 256
//    device uint* bucketBase [[buffer(3)]],            // 256
//    uint tid [[thread_position_in_grid]]
//) {
//    if (tid != 0) return;
//
//    uint runningGlobal = 0;
//
//    for (uint b = 0; b < RADIX; ++b) {
//        bucketBase[b] = runningGlobal;
//
//        uint runningBucket = 0;
//        for (uint g = 0; g < params.numBlocks; ++g) {
//            uint idx = g * RADIX + b;
//            blockOffsets[idx] = runningBucket;
//            runningBucket += blockHistograms[idx];
//        }
//
//        runningGlobal += runningBucket;
//    }
//}
//
//kernel void radixScatterMortonBlocks(
//    device const MortonNode* src [[buffer(0)]],
//    device MortonNode* dst [[buffer(1)]],
//    constant RadixSortParams& params [[buffer(2)]],
//    device const uint* blockOffsets [[buffer(3)]], // numBlocks * 256
//    device const uint* bucketBase [[buffer(4)]],   // 256
//    uint lid [[thread_index_in_threadgroup]],
//    uint gid [[threadgroup_position_in_grid]]
//) {
//    if (gid >= params.numBlocks) return;
//
//    if (lid != 0) return; // correctness-first stable scatter
//
//    uint localCount[RADIX];
//    for (uint b = 0; b < RADIX; ++b) localCount[b] = 0;
//
//    uint blockStart = gid * RADIX_BLOCK_SIZE;
//    uint blockEnd   = min(blockStart + RADIX_BLOCK_SIZE, params.numElements);
//
//    for (uint i = blockStart; i < blockEnd; ++i) {
//        MortonNode m = src[i];
//        uint b = radixBucket(m.code, params.shift);
//
//        uint dstIndex = bucketBase[b]
//                      + blockOffsets[gid * RADIX + b]
//                      + localCount[b];
//
//        dst[dstIndex] = m;
//        localCount[b]++;
//    }
//}


inline uint findSplit(
    device const MortonNode* mortons,
    int i0, int i1
) {
    uint c0 = mortons[i0].code;
    uint c1 = mortons[i1].code;

    if(c0 == c1) return (i0+i1) >> 1;

    uint commonPrefix = clz(c0 ^ c1);

    uint split = i0;
    uint step = i1-i0;

    do {
        step = (step+1) >> 1;
        uint newSplit = split + step;

        if(newSplit < (uint)i1) {
            uint splitCode = mortons[newSplit].code;
            uint splitPrefix = clz(c0 ^ splitCode);
            if(splitPrefix > commonPrefix) split = newSplit;
        }
    } while (step > 1);

    return split;
}

inline int delta(
    device const MortonNode* mortons,
    int numLeafs,
    int i, int j
) {
    if(j < 0 || j >= numLeafs) return -1;

    uint ci = mortons[i].code;
    uint cj = mortons[j].code;

    if(ci != cj) return clz(ci ^ cj);
    return 32 + clz(mortons[i].index ^ mortons[j].index);
}

inline int2 determineRange(
    device const MortonNode* mortons,
    int numLeafs,
    int index
) {
    int d = (delta(mortons, numLeafs, index, index+1) - delta(mortons, numLeafs, index, index-1) >= 0 ? 1 : -1);
    int deltaMin = delta(mortons, numLeafs, index, index - d);

    int lmax = 2;
    while(delta(mortons, numLeafs, index, index + lmax*d) > deltaMin) lmax <<= 1;

    int l = 0;
    for(int t = lmax >> 1; t >= 1; t >>=1) 
        if(delta(mortons, numLeafs, index, index + (l+t)*d) > deltaMin) l += t;

    int j = index + l*d;

    if(d < 0) return int2(j, index);
    else      return int2(index, j);
}

kernel void buildLeaf_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;

    // leaf nodes
    if (idx >= numPrimitives) return;

    int fid = mortons[id].index;
    uint3 facet = facets[fid];
    float3 v0 = x[facet.x];
    float3 v1 = x[facet.y];
    float3 v2 = x[facet.z];
    int leafid = numPrimitives+idx-1;
    tree[leafid].min = min3(v0, v1, v2);
    tree[leafid].childA = -1;
    tree[leafid].max = max3(v0, v1, v2);
    tree[leafid].childB = fid;
}
// GPU port of BVH::enlargeTrajectory's single-root leaf pass. Expands each
// leaf AABB to cover the swept volume {x, x + vel*dt} of its triangle's 3
// verts. Mirrors buildLeaf_Tri's leaf-slot math (numPrimitives+idx-1). The
// existing box (set by the preceding refit) is unioned, so the result equals
// the CPU loop's `min(box, x+vel*dt)` (refit already put x in the box).
kernel void enlargeLeaf_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* vel [[buffer(1)]],
    device const packed_uint3* facets [[buffer(2)]],
    constant int& numPrimitives [[buffer(3)]],
    constant float& dt [[buffer(4)]],
    device const MortonNode* mortons [[buffer(5)]],
    device BVHNode* tree [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;
    if (idx >= numPrimitives) return;

    int fid = mortons[id].index;
    uint3 facet = facets[fid];
    int leafid = numPrimitives+idx-1;

    float3 lo = tree[leafid].min;
    float3 hi = tree[leafid].max;
    uint vid[3] = { facet.x, facet.y, facet.z };
    for (int k = 0; k < 3; ++k) {
        float3 p = x[vid[k]];
        float3 s = p + vel[vid[k]] * dt;
        lo = min(lo, min(p, s));
        hi = max(hi, max(p, s));
    }
    tree[leafid].min = lo;
    tree[leafid].max = hi;
}
// Fused refit+enlarge leaf pass (single-root). SETS the full leaf node
// (box + childA/childB) from the swept hull {x, x+vel*dt} in ONE kernel —
// equal to buildLeaf_Tri immediately followed by enlargeLeaf_Tri, but
// self-contained (no prior refit). Lets the fused path do one leaf pass +
// one bottom-up + one sync instead of refit's pass and enlarge's pass.
kernel void buildSweptLeaf_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* vel [[buffer(1)]],
    device const packed_uint3* facets [[buffer(2)]],
    constant int& numPrimitives [[buffer(3)]],
    constant float& dt [[buffer(4)]],
    device const MortonNode* mortons [[buffer(5)]],
    device BVHNode* tree [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;
    if (idx >= numPrimitives) return;

    int fid = mortons[id].index;
    uint3 facet = facets[fid];

    float3 p0 = x[facet.x]; float3 s0 = p0 + vel[facet.x] * dt;
    float3 lo = min(p0, s0), hi = max(p0, s0);
    float3 p1 = x[facet.y]; float3 s1 = p1 + vel[facet.y] * dt;
    lo = min(lo, min(p1, s1)); hi = max(hi, max(p1, s1));
    float3 p2 = x[facet.z]; float3 s2 = p2 + vel[facet.z] * dt;
    lo = min(lo, min(p2, s2)); hi = max(hi, max(p2, s2));

    int leafid = numPrimitives+idx-1;
    tree[leafid].min = lo;
    tree[leafid].childA = -1;
    tree[leafid].max = hi;
    tree[leafid].childB = fid;
}
kernel void buildTree_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;

    // leaf nodes
    if (idx >= numPrimitives) return;

    int fid = mortons[id].index;
    uint3 facet = facets[fid];
    float3 v0 = x[facet.x];
    float3 v1 = x[facet.y];
    float3 v2 = x[facet.z];
    int leafid = numPrimitives+idx-1;
    tree[leafid].min = min3(v0, v1, v2);
    tree[leafid].childA = -1;
    tree[leafid].max = max3(v0, v1, v2);
    tree[leafid].childB = fid;

    // intermediate nodes
    if(idx == numPrimitives-1) return;

    int2 range = determineRange(mortons, numPrimitives, idx);
    uint split = findSplit(mortons, range.x, range.y);

    int childA, childB;
    if((int)split == range.x) childA = split+numPrimitives-1;
    else                      childA = split;
    if((int)split+1 == range.y) childB = split+numPrimitives;
    else                        childB = split+1;

    tree[id].childA = childA;
    tree[id].childB = childB;
    treeParent[childA] = id;
    treeParent[childB] = id;
}


kernel void buildLeaf_Edge(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint2* edges [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;

    // leaf nodes
    if (idx >= numPrimitives) return;

    int eid = mortons[id].index;
    uint2 edge = edges[eid];
    float3 v0 = x[edge.x];
    float3 v1 = x[edge.y];
    int leafid = numPrimitives+idx-1;
    tree[leafid].min = min(v0, v1);
    tree[leafid].childA = -1;
    tree[leafid].max = max(v0, v1);
    tree[leafid].childB = eid;
}
kernel void buildTree_Edge(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint2* edges [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    int idx = (int)id;

    // leaf nodes
    if (idx >= numPrimitives) return;

    int eid = mortons[id].index;
    uint2 edge = edges[eid];
    float3 v0 = x[edge.x];
    float3 v1 = x[edge.y];
    int leafid = numPrimitives+idx-1;
    tree[leafid].min = min(v0, v1);
    tree[leafid].childA = -1;
    tree[leafid].max = max(v0, v1);
    tree[leafid].childB = eid;

    // intermediate nodes
    if(idx == numPrimitives-1) return;

    int2 range = determineRange(mortons, numPrimitives, idx);
    uint split = findSplit(mortons, range.x, range.y);

    int childA, childB;
    if((int)split == range.x) childA = split+numPrimitives-1;
    else                      childA = split;
    if((int)split+1 == range.y) childB = split+numPrimitives;
    else                        childB = split+1;

    tree[id].childA = childA;
    tree[id].childB = childB;
    treeParent[childA] = id;
    treeParent[childB] = id;
}

// D-029: zero treeVisitCounts before each bottomUpBoxes dispatch.
// Each thread writes one slot; dispatched over numNodes threads.
kernel void zeroVisitCounts(
    device uint* counts [[buffer(0)]],
    constant uint& numNodes [[buffer(1)]],
    uint id [[thread_position_in_grid]]
) {
    if (id < numNodes) counts[id] = 0u;
}

// D-029: lock-free single-dispatch bottom-up AABB combine.
//
// Each leaf thread walks up to the root via `treeParent`. At each
// parent it does atomic_fetch_add(readyCount[parent], 1):
//   - old == 0: first child to arrive — exit (the second arrival
//     will do the combine, which halves the surviving threads at
//     each level → log(N) work depth).
//   - old == 1: second child has arrived — both children's AABBs
//     are now writeable; this thread reads them, computes the
//     parent AABB, writes it, then continues up to grandparent.
//
// **Memory ordering (Metal 3.2+).** MSL's atomic operations are
// restricted to `memory_order_relaxed`, but `atomic_thread_fence`
// supports `memory_order_seq_cst` with explicit `mem_flags` and
// `thread_scope` (MSL Spec §6.15.1–§6.15.3). The seq_cst fence is
// a sequentially-consistent acquire-AND-release barrier within
// `thread_scope_device`, scoped to `mem_flags::mem_device`. Two
// fence sites bracket the atomic publication boundary:
//   (a) after the second-arrival's `atomic_fetch_add` returns 1,
//       before reading `tree[childA/B]` — acquires the sibling
//       (first-arrival) thread's child-AABB writes.
//   (b) after this thread writes `tree[parent].min/max`, before
//       the next loop iteration's `atomic_fetch_add` at the
//       grandparent — releases this thread's parent-AABB writes
//       to the sibling thread that will later acquire them.
// No fence is needed at kernel entry: leaf AABBs are written by
// the prior `buildLeaf*` dispatch in the same command queue, and
// Metal guarantees cross-dispatch visibility at the command
// boundary.
//
// Pre-condition: `treeVisitCounts[]` must be zero on entry — use
// `zeroVisitCounts` immediately before dispatching this kernel.
// Pre-condition: `treeParent[]` must already be populated by
// `buildTree_*` (build) OR carried from the prior build (refit;
// topology unchanged across refits).
//
// Replaces the multi-pass `bottomUpCombineStep` driver (removed
// in the same slice).
kernel void bottomUpBoxes(
    constant AABB4& sceneBox [[buffer(2)]],
    device BVHNode* tree [[buffer(4)]],
    device const int* treeParent [[buffer(5)]],
    device atomic_uint* treeVisitCounts [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;

    int child = int(id + numPrimitives - 1); // leaf node index

    // WEDGE GUARD: a legitimate leaf→root walk visits at most numNodes
    // (= 2N-1) parents. If buildTree produced a corrupted topology (a
    // parent cycle — observed with degenerate/exploded vertex data), an
    // unbounded walk spins this kernel forever; a wedged GPU kernel
    // survives kill -9 of the host and needs a reboot. Bail out instead:
    // the worst case is a stale/wrong parent AABB, which the broad phase
    // tolerates (over/under-report), unlike a frozen GPU.
    uint maxSteps = 2u * numPrimitives;
    int numNodes = int(2u * numPrimitives) - 1;   // valid index range [0, numNodes)

    for (uint step = 0; step < maxSteps; ++step) {
        int parent = treeParent[child];

        // INDEX GUARD: the step cap above catches a parent *cycle* (valid
        // indices looping), but a garbage `parent` (out-of-range) would make
        // the atomic below an OUT-OF-BOUNDS device write → GPU memory fault
        // that wedges the device (survives host kill). Bail on a bad index
        // instead — same "stale/wrong box tolerated" trade-off as the step cap.
        if (parent < 0 || parent >= numNodes) return;

        // Atomics are relaxed-only in MSL; fences carry the
        // memory ordering — see kernel doc-block above.
        uint old = atomic_fetch_add_explicit(
            &treeVisitCounts[parent],
            1u,
            memory_order_relaxed
        );

        if (old == 0u) {
            // First arrival — exit; second arrival will combine.
            return;
        }

        // (a) Acquire the first arrival's child-AABB writes
        // before reading children's bounds.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        int childA = tree[parent].childA;
        int childB = tree[parent].childB;
        // INDEX GUARD: corrupt internal node → OOB read fault. Bail.
        if (childA < 0 || childA >= numNodes ||
            childB < 0 || childB >= numNodes) return;

        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);

        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        // (b) Release this thread's parent-AABB write before the
        // next iteration's atomic at the grandparent — so the
        // sibling thread (which will acquire via fence (a) after
        // its own fetch_add) sees our write.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        child = parent;
        if (child == 0) return; // wrote root, done
    }
}

// ============================================================================
// Sub-object (multi-root) LBVH — square-cloth experiment (Phase 1).
//
// The primitive array is partitioned into k = 4^s groups by MATERIAL-space
// tile (fixed for the cloth's lifetime). Each group owns an INDEPENDENT
// Karras tree living in its own contiguous slot window of the shared `tree`
// buffer. There are k roots; the broad phase (Phase 2) treats each group
// root as a separate object via the existing scene TLAS.
//
// Layout for group g with M = groupSize[g] prims:
//   sorted positions : [primBase[g], primBase[g]+M)        (set by host
//                       stable group-partition after the global Morton sort)
//   node window      : [nodeBase[g], nodeBase[g]+(2M-1))
//   internal local l : 0..M-2   -> global slot nodeBase[g] + l
//   leaf     local l : 0..M-1   -> global slot nodeBase[g] + (M-1) + l
//   group root       : nodeBase[g]
// Total nodes across all groups = 2N - k (<= the 2N-1 single-root alloc).
//
// These mirror buildTree_Tri / buildLeaf_Tri / bottomUpBoxes with N->M_g,
// the mortons pointer shifted by primBase[g], and slots offset by
// nodeBase[g]. determineRange/findSplit operate on the group's LOCAL range,
// so the Karras delta rule never crosses a group boundary.
// ============================================================================

// Grouped hierarchy + leaf build. One thread per global sorted position.
kernel void buildTree_Tri_Grouped(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    device const uint* sortedPosToGroup [[buffer(6)]],
    device const uint* groupSize [[buffer(7)]],
    device const uint* groupPrimBase [[buffer(8)]],
    device const uint* groupNodeBase [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    int gid = (int)id;
    if (gid >= numPrimitives) return;

    uint g     = sortedPosToGroup[gid];
    int  M     = (int)groupSize[g];
    int  pbase = (int)groupPrimBase[g];
    int  nbase = (int)groupNodeBase[g];
    int  l     = gid - pbase;          // local sorted index within group

    // leaf node
    int fid = mortons[gid].index;
    uint3 facet = facets[fid];
    float3 v0 = x[facet.x];
    float3 v1 = x[facet.y];
    float3 v2 = x[facet.z];
    int leafid = nbase + (M - 1) + l;
    tree[leafid].min = min3(v0, v1, v2);
    tree[leafid].childA = -1;
    tree[leafid].max = max3(v0, v1, v2);
    tree[leafid].childB = fid;

    // last sorted position in the group is a leaf only (M-1 internals)
    if (l == M - 1) return;

    // LOCAL Karras range/split (mortons shifted to group start)
    int2 range = determineRange(mortons + pbase, M, l);
    uint split = findSplit(mortons + pbase, range.x, range.y);

    int childA, childB;
    if ((int)split == range.x)     childA = nbase + (M - 1) + (int)split;
    else                           childA = nbase + (int)split;
    if ((int)split + 1 == range.y) childB = nbase + (M - 1) + (int)split + 1;
    else                           childB = nbase + (int)split + 1;

    int self = nbase + l;
    tree[self].childA = childA;
    tree[self].childB = childB;
    treeParent[childA] = self;
    treeParent[childB] = self;
}

// Grouped leaf-only refit. Topology (treeParent + childA/B) carried from the
// prior grouped build; only leaf AABBs are recomputed from current positions.
kernel void buildLeaf_Tri_Grouped(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device const uint* sortedPosToGroup [[buffer(6)]],
    device const uint* groupSize [[buffer(7)]],
    device const uint* groupPrimBase [[buffer(8)]],
    device const uint* groupNodeBase [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    int gid = (int)id;
    if (gid >= numPrimitives) return;

    uint g     = sortedPosToGroup[gid];
    int  M     = (int)groupSize[g];
    int  pbase = (int)groupPrimBase[g];
    int  nbase = (int)groupNodeBase[g];
    int  l     = gid - pbase;

    int fid = mortons[gid].index;
    uint3 facet = facets[fid];
    float3 v0 = x[facet.x];
    float3 v1 = x[facet.y];
    float3 v2 = x[facet.z];
    int leafid = nbase + (M - 1) + l;
    tree[leafid].min = min3(v0, v1, v2);
    tree[leafid].childA = -1;
    tree[leafid].max = max3(v0, v1, v2);
    tree[leafid].childB = fid;
}
// Multi-root counterpart of enlargeLeaf_Tri. Same swept-union expansion, but
// the leaf lives in its group window (nbase + (M-1) + l) like
// buildLeaf_Tri_Grouped. Buffer slots shift by one vs the single-root kernel
// (vel at 1) so the grouped index buffers move to 7..10.
kernel void enlargeLeaf_Tri_Grouped(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* vel [[buffer(1)]],
    device const packed_uint3* facets [[buffer(2)]],
    constant int& numPrimitives [[buffer(3)]],
    constant float& dt [[buffer(4)]],
    device const MortonNode* mortons [[buffer(5)]],
    device BVHNode* tree [[buffer(6)]],
    device const uint* sortedPosToGroup [[buffer(7)]],
    device const uint* groupSize [[buffer(8)]],
    device const uint* groupPrimBase [[buffer(9)]],
    device const uint* groupNodeBase [[buffer(10)]],
    uint id [[thread_position_in_grid]]
) {
    int gid = (int)id;
    if (gid >= numPrimitives) return;

    uint g     = sortedPosToGroup[gid];
    int  M     = (int)groupSize[g];
    int  pbase = (int)groupPrimBase[g];
    int  nbase = (int)groupNodeBase[g];
    int  l     = gid - pbase;

    int fid = mortons[gid].index;
    uint3 facet = facets[fid];
    int leafid = nbase + (M - 1) + l;

    float3 lo = tree[leafid].min;
    float3 hi = tree[leafid].max;
    uint vid[3] = { facet.x, facet.y, facet.z };
    for (int k = 0; k < 3; ++k) {
        float3 p = x[vid[k]];
        float3 s = p + vel[vid[k]] * dt;
        lo = min(lo, min(p, s));
        hi = max(hi, max(p, s));
    }
    tree[leafid].min = lo;
    tree[leafid].max = hi;
}
// Fused refit+enlarge leaf pass (multi-root). SETS the full grouped leaf node
// from the swept hull, like buildLeaf_Tri_Grouped + enlarge in one kernel.
kernel void buildSweptLeaf_Tri_Grouped(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* vel [[buffer(1)]],
    device const packed_uint3* facets [[buffer(2)]],
    constant int& numPrimitives [[buffer(3)]],
    constant float& dt [[buffer(4)]],
    device const MortonNode* mortons [[buffer(5)]],
    device BVHNode* tree [[buffer(6)]],
    device const uint* sortedPosToGroup [[buffer(7)]],
    device const uint* groupSize [[buffer(8)]],
    device const uint* groupPrimBase [[buffer(9)]],
    device const uint* groupNodeBase [[buffer(10)]],
    uint id [[thread_position_in_grid]]
) {
    int gid = (int)id;
    if (gid >= numPrimitives) return;

    uint g     = sortedPosToGroup[gid];
    int  M     = (int)groupSize[g];
    int  pbase = (int)groupPrimBase[g];
    int  nbase = (int)groupNodeBase[g];
    int  l     = gid - pbase;

    int fid = mortons[gid].index;
    uint3 facet = facets[fid];

    float3 p0 = x[facet.x]; float3 s0 = p0 + vel[facet.x] * dt;
    float3 lo = min(p0, s0), hi = max(p0, s0);
    float3 p1 = x[facet.y]; float3 s1 = p1 + vel[facet.y] * dt;
    lo = min(lo, min(p1, s1)); hi = max(hi, max(p1, s1));
    float3 p2 = x[facet.z]; float3 s2 = p2 + vel[facet.z] * dt;
    lo = min(lo, min(p2, s2)); hi = max(hi, max(p2, s2));

    int leafid = nbase + (M - 1) + l;
    tree[leafid].min = lo;
    tree[leafid].childA = -1;
    tree[leafid].max = hi;
    tree[leafid].childB = fid;
}

// Multi-root bottom-up AABB combine. Identical lock-free walk to
// `bottomUpBoxes`, except each leaf stops at its GROUP root (nodeBase[g])
// instead of slot 0. Group node windows are disjoint, so treeVisitCounts
// atomics never collide across groups; one dispatch over all N leaves.
//
// Divergence note: walk depth is ~log(M_g) (not ~log N), and consecutive
// sorted leaves share a group, so per-warp walk depths are uniform — the
// motivation for this layout.
kernel void bottomUpBoxesMultiRoot(
    constant AABB4& sceneBox [[buffer(2)]],
    device BVHNode* tree [[buffer(4)]],
    device const int* treeParent [[buffer(5)]],
    device atomic_uint* treeVisitCounts [[buffer(6)]],
    device const uint* sortedPosToGroup [[buffer(7)]],
    device const uint* groupSize [[buffer(8)]],
    device const uint* groupPrimBase [[buffer(9)]],
    device const uint* groupNodeBase [[buffer(10)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;

    uint g     = sortedPosToGroup[id];
    int  M     = (int)groupSize[g];
    int  pbase = (int)groupPrimBase[g];
    int  root  = (int)groupNodeBase[g];
    int  l     = (int)id - pbase;

    int child = root + (M - 1) + l;     // this thread's leaf slot
    if (child == root) return;          // singleton group (M == 1): leaf IS root

    int numNodes = int(2u * numPrimitives) - 1;  // shared tree buffer bound

    uint maxSteps = 2u * (uint)M;       // group-local wedge guard
    for (uint step = 0; step < maxSteps; ++step) {
        int parent = treeParent[child];
        // INDEX GUARD: garbage parent → OOB atomic = GPU fault/wedge. Bail.
        if (parent < 0 || parent >= numNodes) return;

        uint old = atomic_fetch_add_explicit(
            &treeVisitCounts[parent], 1u, memory_order_relaxed);

        if (old == 0u) return;          // first arrival — second will combine

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst, thread_scope_device);

        int childA = tree[parent].childA;
        int childB = tree[parent].childB;
        if (childA < 0 || childA >= numNodes ||
            childB < 0 || childB >= numNodes) return;   // corrupt node → bail
        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);
        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst, thread_scope_device);

        child = parent;
        if (child == root) return;      // wrote group root, done
    }
}

// Apetrei (2014) "Fast and Simple Agglomerative LBVH Construction".
// Single-kernel replacement for buildTree_* + bottomUpBoxes. Each thread
// starts from a leaf and walks toward the root, deciding the parent at
// each step via Apetrei's δ rule while combining AABBs.
//
// Node layout — same buffer shape as the Karras path:
//   leaves     : slots [N-1, 2N-2]   (leafSlot = N - 1 + sortedId)
//   internals  : slots [0, N-2]      (Apetrei internal i ↔ slot i)
// Apetrei's internal node i splits between sorted keys i and i+1.
//
// Parent rule for a node covering [L, R] (L,R in leaf-index space):
//   if L == 0          → parent = R,    current is childA
//   else if R == N-1   → parent = L-1,  current is childB
//   else compare δ(R) vs δ(L-1) (smaller = closer ancestor):
//        δ(R)  < δ(L-1) → parent = R,    childA
//        else            → parent = L-1, childB
// Special case L==0 && R==N-1: current node is the root.
//
// δ(i) := highest differing bit between mortons[i].code and mortons[i+1].code.
// Implemented as raw XOR (monotonic in bit position). Ties broken by
// (index XOR) — same convention as the Karras `delta()` helper above.
//
// Synchronization mirrors bottomUpBoxes: relaxed atomic_fetch_add on
// `nodeVisitFlags[parent]` gates first/second arrival; seq_cst device
// fences acquire the sibling's writes and release this thread's writes
// before the next iteration's atomic.
//
// Pre-condition: nodeVisitFlags[] zeroed; mortons[] sorted.
// Post-condition: tree[] + treeParent[] populated; rootIndexOut holds
// the slot of the actual root (which may be any slot in [0, N-2]).
// Caller follows up with `agglomerativeSwapRoot` to relocate the root
// to slot 0 if needed.
kernel void agglomerativeBuild_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    device atomic_uint* nodeVisitFlags [[buffer(6)]],
    device int* nodeRangeLeft [[buffer(7)]],
    device int* nodeRangeRight [[buffer(8)]],
    device atomic_int* rootIndexOut [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    int N = numPrimitives;
    if ((int)id >= N) return;

    int fid = mortons[id].index;
    uint3 facet = facets[fid];
    float3 v0 = x[facet.x];
    float3 v1 = x[facet.y];
    float3 v2 = x[facet.z];
    int leafSlot = N + (int)id - 1;
    tree[leafSlot].min = min3(v0, v1, v2);
    tree[leafSlot].childA = -1;
    tree[leafSlot].max = max3(v0, v1, v2);
    tree[leafSlot].childB = fid;

    if (N <= 1) {
        // Single-leaf degenerate: leaf IS the root (slot 0 since N-1==0).
        atomic_store_explicit(rootIndexOut, leafSlot, memory_order_relaxed);
        return;
    }

    int currentNode = leafSlot;
    int L = (int)id;
    int R = (int)id;

    // WEDGE GUARD: a valid agglomerative walk widens [L, R] by at least
    // one leaf per iteration, so it takes at most N steps to reach the
    // root. Corrupted mortons (degenerate/exploded vertex data) can break
    // the δ rule's monotonicity and cycle the walk; bound it so the GPU
    // can never spin forever (worst case: an unfinished subtree AABB,
    // which the broad phase tolerates — a frozen GPU is not recoverable).
    for (int step = 0; step < 2 * N; ++step) {
        if (L == 0 && R == N - 1) {
            atomic_store_explicit(rootIndexOut, currentNode, memory_order_relaxed);
            return;
        }

        bool goRight;
        if (L == 0) {
            goRight = true;
        } else if (R == N - 1) {
            goRight = false;
        } else {
            uint dR_code = mortons[R].code ^ mortons[R + 1].code;
            uint dL_code = mortons[L - 1].code ^ mortons[L].code;
            if (dR_code != dL_code) {
                goRight = (dR_code < dL_code);
            } else {
                // Tie on codes — fall back to index XOR (same convention
                // as Karras `delta()` above, where tied codes are
                // disambiguated via mortons[*].index).
                uint dR_idx = mortons[R].index ^ mortons[R + 1].index;
                uint dL_idx = mortons[L - 1].index ^ mortons[L].index;
                goRight = (dR_idx < dL_idx);
            }
        }

        int parent;
        if (goRight) {
            parent = R;
            tree[parent].childA = currentNode;
            nodeRangeLeft[parent] = L;
        } else {
            parent = L - 1;
            tree[parent].childB = currentNode;
            nodeRangeRight[parent] = R;
        }
        treeParent[currentNode] = parent;

        // **Release fence** — bug-fix for repeated-rebuild hangs.
        // 이 라인 위에서 쓴 값들 (leaf AABB 첫 진입 + tree[parent].childA/B
        // + nodeRangeLeft/Right[parent] + treeParent[currentNode]) 는 다른
        // 스레드의 second-arrival 쪽에서 atomic_fetch_add 후 seq_cst fence (a)
        // 다음에 읽힌다. T1 쪽에 release semantics 가 없으면 (relaxed atomic
        // 하나뿐이면) 그 happens-before edge 가 형성되지 않아 T2 가 stale
        // 값을 읽을 수 있다 — 특히 매 프레임 재빌드 시 이전 프레임의
        // nodeRangeLeft/Right 가 그대로 보이면 L/R 가 엉뚱한 값이 되고
        // 루프가 L==0&&R==N-1 종료 조건에 도달 못해 GPU 무한 루프 →
        // 10~20 프레임 후 워치독 stall.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        uint old = atomic_fetch_add_explicit(
            &nodeVisitFlags[parent],
            1u,
            memory_order_relaxed
        );
        if (old == 0u) return;

        // (a) Acquire sibling's writes (child AABBs + the opposite-side
        //     range entry) before reading them.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        int childA = tree[parent].childA;
        int childB = tree[parent].childB;

        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);

        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        // (b) Release this thread's parent-AABB write before the next
        //     iteration's atomic at the grandparent.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        currentNode = parent;
        L = nodeRangeLeft[parent];
        R = nodeRangeRight[parent];
    }
}

kernel void agglomerativeBuild_Edge(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint2* edges [[buffer(1)]],
    constant int& numPrimitives [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    device atomic_uint* nodeVisitFlags [[buffer(6)]],
    device int* nodeRangeLeft [[buffer(7)]],
    device int* nodeRangeRight [[buffer(8)]],
    device atomic_int* rootIndexOut [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    int N = numPrimitives;
    if ((int)id >= N) return;

    int eid = mortons[id].index;
    uint2 edge = edges[eid];
    float3 v0 = x[edge.x];
    float3 v1 = x[edge.y];
    int leafSlot = N + (int)id - 1;
    tree[leafSlot].min = min(v0, v1);
    tree[leafSlot].childA = -1;
    tree[leafSlot].max = max(v0, v1);
    tree[leafSlot].childB = eid;

    if (N <= 1) {
        atomic_store_explicit(rootIndexOut, leafSlot, memory_order_relaxed);
        return;
    }

    int currentNode = leafSlot;
    int L = (int)id;
    int R = (int)id;

    // WEDGE GUARD: a valid agglomerative walk widens [L, R] by at least
    // one leaf per iteration, so it takes at most N steps to reach the
    // root. Corrupted mortons (degenerate/exploded vertex data) can break
    // the δ rule's monotonicity and cycle the walk; bound it so the GPU
    // can never spin forever (worst case: an unfinished subtree AABB,
    // which the broad phase tolerates — a frozen GPU is not recoverable).
    for (int step = 0; step < 2 * N; ++step) {
        if (L == 0 && R == N - 1) {
            atomic_store_explicit(rootIndexOut, currentNode, memory_order_relaxed);
            return;
        }

        bool goRight;
        if (L == 0) {
            goRight = true;
        } else if (R == N - 1) {
            goRight = false;
        } else {
            uint dR_code = mortons[R].code ^ mortons[R + 1].code;
            uint dL_code = mortons[L - 1].code ^ mortons[L].code;
            if (dR_code != dL_code) {
                goRight = (dR_code < dL_code);
            } else {
                uint dR_idx = mortons[R].index ^ mortons[R + 1].index;
                uint dL_idx = mortons[L - 1].index ^ mortons[L].index;
                goRight = (dR_idx < dL_idx);
            }
        }

        int parent;
        if (goRight) {
            parent = R;
            tree[parent].childA = currentNode;
            nodeRangeLeft[parent] = L;
        } else {
            parent = L - 1;
            tree[parent].childB = currentNode;
            nodeRangeRight[parent] = R;
        }
        treeParent[currentNode] = parent;

        // Release fence — 동일 버그 수정. _Tri 쪽 주석 참고.
        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        uint old = atomic_fetch_add_explicit(
            &nodeVisitFlags[parent],
            1u,
            memory_order_relaxed
        );
        if (old == 0u) return;

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        int childA = tree[parent].childA;
        int childB = tree[parent].childB;

        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);

        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        currentNode = parent;
        L = nodeRangeLeft[parent];
        R = nodeRangeRight[parent];
    }
}

// Post-pass for agglomerativeBuild_*: relocates the root from its natural
// Apetrei slot (rootIndexBuf[0]) to slot 0 so downstream traversal code
// (queryPoints, queryClickRay, SCENE-level reads) can keep its `tree[0]
// is root` assumption intact. Single-threaded — dispatched with 1 thread.
//
// Swap semantics (k := rootIndexBuf[0]):
//   tree[0] gets the root's content (with any 0-references retargeted to k).
//   tree[k] gets the original tree[0]'s content (the displaced Apetrei
//   internal node, "OldNode0").
// Then treeParent and parent-child pointers are patched so the tree is
// internally consistent — every node's children point to the correct
// post-swap slot, and every node's treeParent points to the correct
// post-swap slot.
kernel void agglomerativeSwapRoot(
    device BVHNode* tree [[buffer(0)]],
    device int* treeParent [[buffer(1)]],
    device const int* rootIndexBuf [[buffer(2)]],
    constant int& numPrimitives [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id != 0) return;
    if (numPrimitives <= 1) return;  // single leaf; already at slot 0

    int k = rootIndexBuf[0];
    if (k == 0) return;

    BVHNode R = tree[k];   // root content
    BVHNode O = tree[0];   // OldNode0 content (Apetrei's slot-0 internal)
    int p0 = treeParent[0];

    int OA = O.childA;
    int OB = O.childB;

    // If root's child pointed to slot 0 (OldNode0 was root's direct child),
    // retarget to slot k where OldNode0 lands after swap.
    int RA = (R.childA == 0) ? k : R.childA;
    int RB = (R.childB == 0) ? k : R.childB;

    tree[0].min = R.min;
    tree[0].max = R.max;
    tree[0].childA = RA;
    tree[0].childB = RB;
    tree[k] = O;

    // OldNode0's children: parent was 0, now k (OldNode0's new slot).
    // OldNode0 is internal (slot in [0, N-2]), so OA, OB are valid slots.
    treeParent[OA] = k;
    treeParent[OB] = k;

    // Root's children: parent was k, now 0 (root's new slot).
    // If RA == k or RB == k (the root-had-OldNode0-as-child case), this
    // also correctly sets treeParent[k] = 0 — OldNode0's parent is now
    // the root at slot 0.
    treeParent[RA] = 0;
    treeParent[RB] = 0;

    // If OldNode0's parent (p0) is NOT the root, we still need to:
    //   (i)  retarget tree[p0]'s childA/B pointer from 0 to k, and
    //   (ii) write treeParent[k] = p0 (OldNode0 retains its parent).
    // If p0 == k (OldNode0's parent IS the root), tree[0] (the root) was
    // already patched above (R.childA/B → k), and treeParent[k] = 0 was
    // set by the treeParent[RA]/[RB] = 0 line that covers the case where
    // RA == k or RB == k. Either way, we're done in the p0 == k branch.
    if (p0 != k) {
        if (tree[p0].childA == 0) tree[p0].childA = k;
        else                      tree[p0].childB = k;
        treeParent[k] = p0;
    }

    // Root has no parent; sentinel for clarity.
    treeParent[0] = -1;
}

// D-030: partial-depth variant of `bottomUpBoxes`. Each thread walks
// up via treeParent and combines AABBs identically to bottomUpBoxes,
// but stops after writing `maxDepth` levels from the leaf side. The
// CPU follow-up (`bottomUpCombineWithSkip` in main.cpp) finishes the
// remaining top-of-tree.
//
// Kept as a SEPARATE kernel (not a depth-parameterized extension of
// bottomUpBoxes) so D-029's pure-GPU walk-to-root path remains
// callable and benchmarkable unchanged. The two kernels share the
// same Metal 3.2 seq_cst fence shape; only the loop's depth check
// + per-thread depth counter differ.
//
// **Frontier invariant**: the depth check is BEFORE the atomic so
// threads exiting at the cutoff never increment `treeVisitCounts` at
// the post-cutoff level. Post-dispatch, every node at the GPU
// frontier has `treeVisitCounts == 2` and every node above the
// frontier has `treeVisitCounts == 0` — never 1. The CPU completion
// uses that as an unambiguous "GPU completed this subtree" marker.
kernel void bottomUpBoxesPartial(
    constant AABB4& sceneBox [[buffer(2)]],
    device BVHNode* tree [[buffer(4)]],
    device const int* treeParent [[buffer(5)]],
    device atomic_uint* treeVisitCounts [[buffer(6)]],
    constant uint& maxDepth [[buffer(7)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;

    int child = int(id + numPrimitives - 1); // leaf node index
    uint depth = 0u;
    int numNodes = int(2u * numPrimitives) - 1;   // valid index range [0, numNodes)

    while (true) {
        int parent = treeParent[child];

        // Hybrid cutoff — check BEFORE the atomic so threads exiting
        // here don't touch treeVisitCounts at this level (preserves
        // the frontier invariant above).
        if (depth >= maxDepth) return;
        // INDEX GUARD: garbage parent → OOB atomic = GPU fault/wedge. Bail.
        if (parent < 0 || parent >= numNodes) return;

        uint old = atomic_fetch_add_explicit(
            &treeVisitCounts[parent],
            1u,
            memory_order_relaxed
        );

        if (old == 0u) return;

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        int childA = tree[parent].childA;
        int childB = tree[parent].childB;
        if (childA < 0 || childA >= numNodes ||
            childB < 0 || childB >= numNodes) return;   // corrupt node → bail

        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);

        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        atomic_thread_fence(mem_flags::mem_device,
                            memory_order_seq_cst,
                            thread_scope_device);

        depth++;                 // just combined a level
        child = parent;
        if (child == 0) return;  // wrote root, done
    }
}







inline bool intersectAABB(float3 qmin, float3 qmax, BVHNode node) {
    if(qmax.x < node.min.x || qmin.x > node.max.x) return false;
    if(qmax.y < node.min.y || qmin.y > node.max.y) return false;
    if(qmax.z < node.min.z || qmin.z > node.max.z) return false;
    return true;
}

struct QueryPointsParams {
    float queryMargin;
    uint numPoints;
    uint qObjId;
    uint tObjId;
    uint maxNumCollisions;
    uint qBehavior;
    uint tBehavior;
    uint qShape;
    uint tShape;
    uint numNodes;   // node count of the queried tree → traversal index bound
};


struct QueryFlag {
    uint stackOverflow;
    uint collisionOverflow;
};



void queryAABB(
    const float3 qmin, 
    const float3 qmax, 
    device const packed_uint3* facets,
    device const BVHNode* tree,
    constant QueryPointsParams& qParams,
    device BroadCollision* broadCollisions,
    device atomic_uint* numBroadCollisions,
    device QueryFlag* qFlag,
    uint id
) {
    const int stackDepth = 64;

    int stack[stackDepth];
    int sp = 0;
    stack[sp++] = 0;

    // WEDGE GUARD: a healthy traversal visits at most the node count of
    // the tree (~2M for a 1M-facet mesh). A corrupted topology (child
    // pointing back at an ancestor) re-feeds the stack forever; cap the
    // visit count so the kernel always terminates — reported through the
    // existing stackOverflow flag (same "incomplete query" semantics).
    uint visited = 0u;

    while(sp > 0) {
        if (++visited > (1u << 22)) {
            qFlag[0].stackOverflow = 1u;
            return;
        }
        int nodeid = stack[--sp];
        // INDEX GUARD: a corrupt childA/childB pushed below could be an
        // out-of-range slot → tree[nodeid] OOB read = GPU fault/wedge. Skip
        // invalid indices (the visited cap already bounds healthy traversals).
        if (nodeid < 0 || nodeid >= (int)qParams.numNodes) continue;
        BVHNode node = tree[nodeid];

        if (!intersectAABB(qmin, qmax, node))
            continue;

        if (node.childA < 0) { // leaf
            uint fid = (uint)node.childB;
            uint3 facet = facets[fid];
            if(qParams.qObjId == qParams.tObjId && (id == facet.x || id == facet.y || id == facet.z)) continue;

            uint idx = atomic_fetch_add_explicit(numBroadCollisions, 1u, memory_order_relaxed);
            if(idx >= qParams.maxNumCollisions) {
                qFlag[0].collisionOverflow = 1u;
                continue;
            }
            broadCollisions[idx].indexPair = {id, fid};
            broadCollisions[idx].objPair = {qParams.qObjId, qParams.tObjId};
            broadCollisions[idx].behaviorPair = {qParams.qBehavior, qParams.tBehavior};
            broadCollisions[idx].shapePair = {qParams.qShape, qParams.tShape};
            continue;
        }
        if (sp + 2 > stackDepth) {
            qFlag[0].stackOverflow = 1u;
            continue;
        }

        stack[sp++] = node.childA;
        stack[sp++] = node.childB;
    }
}

kernel void queryPoints(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    device const BVHNode* tree [[buffer(2)]],
    constant QueryPointsParams& qParams [[buffer(3)]],
    device BroadCollision* broadCollisions [[buffer(4)]],
    device atomic_uint* numBroadCollisions [[buffer(5)]],
    device QueryFlag* qFlag [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    if(id >= qParams.numPoints) return;

    float3 pos = x[id];
    float3 margin = float3(qParams.queryMargin);
    float3 qmin = pos-margin;
    float3 qmax = pos+margin;

    queryAABB(qmin, qmax, facets, tree, qParams, broadCollisions, numBroadCollisions, qFlag, id);
}


// ============================================================
// Segmented (per-threadgroup) detect + reduce variant.
//
// Motivation (Tang et al. 2011, "Collision-Streams", §3.4 / Alg.2):
// the baseline `queryPoints` does a device-global `atomic_fetch_add`
// on `numBroadCollisions` for EVERY leaf hit — hot, contended, and
// dominates the kernel's wall time on dense scenes. The paper's
// "segmented locking" replaces this with a threadgroup-local atomic
// + per-TG private slice in device memory, then a separate compact
// pass that uses one device atomic for the WHOLE dispatch (to
// reserve `globalBase`) instead of one-per-hit.
//
// Pipeline per `queryPointsSegmented` call:
//   (1) queryPointsSegmented — detection. Each TG: threadgroup
//       atomic_uint claims slots into a per-TG slice of
//       `tgPrivateCollisions`; lid==0 publishes the final TG count.
//   (2) scanReserveSegmented — single-thread serial exclusive
//       scan of `tgPrivateCount[0..numTGs]`, then ONE device
//       atomic_fetch_add on the shared `numBroadCollisions` to
//       reserve `globalBase`; biases per-TG offsets by base.
//   (3) compactSegmented    — scatters each TG slice into the
//       global `broadCollisions` at `tgPrivateOffset[gid] + i`.
//
// Kept as separate kernels (not a flag-parameterized extension of
// `queryPoints`) so the baseline path remains callable / benchable
// unchanged, per slice goal of A/B comparison before deprecation.
// ============================================================

struct QuerySegParams {
    float queryMargin;
    uint numPoints;
    uint qObjId;
    uint tObjId;
    uint perTGCap;       // capacity (in BroadCollisions) of each TG's private slice
    uint qBehavior;
    uint tBehavior;
    uint qShape;
    uint tShape;
    uint numNodes;       // queried tree's node count → traversal index bound
};

void queryAABBSegmented(
    const float3 qmin,
    const float3 qmax,
    device const packed_uint3* facets,
    device const BVHNode* tree,
    constant QuerySegParams& qParams,
    device BroadCollision* tgPrivateCollisions,
    threadgroup atomic_uint* tgCount,
    device QueryFlag* qFlag,
    uint gid,
    uint id
) {
    const int stackDepth = 64;
    int stack[stackDepth];
    int sp = 0;
    stack[sp++] = 0;

    // WEDGE GUARD: same visit cap as queryAABB — see comment there.
    uint visited = 0u;

    while(sp > 0) {
        if (++visited > (1u << 22)) {
            qFlag[0].stackOverflow = 1u;
            return;
        }
        int nodeid = stack[--sp];
        // INDEX GUARD: skip out-of-range slots (corrupt child pointer) →
        // avoids an OOB tree read that would wedge the GPU. See queryAABB.
        if (nodeid < 0 || nodeid >= (int)qParams.numNodes) continue;
        BVHNode node = tree[nodeid];

        if (!intersectAABB(qmin, qmax, node)) continue;

        if (node.childA < 0) { // leaf
            uint fid = (uint)node.childB;
            uint3 facet = facets[fid];
            if(qParams.qObjId == qParams.tObjId && (id == facet.x || id == facet.y || id == facet.z)) continue;

            // Threadgroup-local atomic (paper's "segmented locking"):
            // only TG_SIZE threads contend here, vs all device threads
            // contending on the global counter in the baseline.
            uint local = atomic_fetch_add_explicit(tgCount, 1u, memory_order_relaxed);
            if (local >= qParams.perTGCap) {
                qFlag[0].collisionOverflow = 1u;
                continue;
            }
            uint slot = gid * qParams.perTGCap + local;
            tgPrivateCollisions[slot].indexPair    = {id, fid};
            tgPrivateCollisions[slot].objPair      = {qParams.qObjId, qParams.tObjId};
            tgPrivateCollisions[slot].behaviorPair = {qParams.qBehavior, qParams.tBehavior};
            tgPrivateCollisions[slot].shapePair    = {qParams.qShape, qParams.tShape};
            continue;
        }
        if (sp + 2 > stackDepth) {
            qFlag[0].stackOverflow = 1u;
            continue;
        }
        stack[sp++] = node.childA;
        stack[sp++] = node.childB;
    }
}

kernel void queryPointsSegmented(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    device const BVHNode* tree [[buffer(2)]],
    constant QuerySegParams& qParams [[buffer(3)]],
    device BroadCollision* tgPrivateCollisions [[buffer(4)]],
    device uint* tgPrivateCount [[buffer(5)]],
    device QueryFlag* qFlag [[buffer(6)]],
    uint id  [[thread_position_in_grid]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    threadgroup atomic_uint localCount;
    if (lid == 0) atomic_store_explicit(&localCount, 0u, memory_order_relaxed);
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (id < qParams.numPoints) {
        float3 pos    = x[id];
        float3 margin = float3(qParams.queryMargin);
        float3 qmin   = pos - margin;
        float3 qmax   = pos + margin;
        queryAABBSegmented(qmin, qmax, facets, tree, qParams,
                           tgPrivateCollisions, &localCount, qFlag, gid, id);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Publish this TG's hit count (clamped to capacity so compact
    // never reads past the private slice).
    if (lid == 0) {
        uint c = atomic_load_explicit(&localCount, memory_order_relaxed);
        tgPrivateCount[gid] = min(c, qParams.perTGCap);
    }
}

struct SegScanParams {
    uint numTGs;
    uint maxNumCollisions;
};

// Single-thread serial exclusive scan over tgPrivateCount[0..numTGs],
// followed by ONE device-level atomic to reserve `globalBase` in the
// shared `numBroadCollisions`. numTGs is typically O(numPoints/256)
// — small enough that serial scan dwarfs the kernel-launch cost of a
// parallel scan, and avoids needing a separate scan algorithm.
kernel void scanReserveSegmented(
    device const uint* tgPrivateCount [[buffer(0)]],
    device uint* tgPrivateOffset [[buffer(1)]],
    device atomic_uint* numBroadCollisions [[buffer(2)]],
    constant SegScanParams& sp [[buffer(3)]],
    device QueryFlag* qFlag [[buffer(4)]],
    uint tid [[thread_position_in_grid]]
) {
    if (tid != 0) return;

    uint running = 0;
    for (uint i = 0; i < sp.numTGs; ++i) {
        tgPrivateOffset[i] = running;
        running += tgPrivateCount[i];
    }
    // One device atomic for the entire dispatch (vs one-per-hit in
    // the baseline). Accumulates across successive queryPointsSegmented
    // calls so the global `broadCollisions` packs all (q,t) pairs.
    uint base = atomic_fetch_add_explicit(numBroadCollisions, running, memory_order_relaxed);
    if (base + running > sp.maxNumCollisions) qFlag[0].collisionOverflow = 1u;
    for (uint i = 0; i < sp.numTGs; ++i) {
        tgPrivateOffset[i] += base;
    }
}

struct SegCompactParams {
    uint numTGs;
    uint perTGCap;
    uint tgSize;
    uint maxNumCollisions;
};

kernel void compactSegmented(
    device const BroadCollision* tgPrivateCollisions [[buffer(0)]],
    device const uint* tgPrivateCount [[buffer(1)]],
    device const uint* tgPrivateOffset [[buffer(2)]],
    constant SegCompactParams& cp [[buffer(3)]],
    device BroadCollision* broadCollisions [[buffer(4)]],
    uint lid [[thread_index_in_threadgroup]],
    uint gid [[threadgroup_position_in_grid]]
) {
    if (gid >= cp.numTGs) return;
    uint count   = tgPrivateCount[gid];
    uint dstBase = tgPrivateOffset[gid];
    uint srcBase = gid * cp.perTGCap;

    for (uint i = lid; i < count; i += cp.tgSize) {
        uint dst = dstBase + i;
        if (dst >= cp.maxNumCollisions) return; // overflow already flagged in scan
        broadCollisions[dst] = tgPrivateCollisions[srcBase + i];
    }
}
