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

    while (true) {
        int parent = treeParent[child];

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

    while (true) {
        int parent = treeParent[child];

        // Hybrid cutoff — check BEFORE the atomic so threads exiting
        // here don't touch treeVisitCounts at this level (preserves
        // the frontier invariant above).
        if (depth >= maxDepth) return;

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

    while(sp > 0) {
        int nodeid = stack[--sp];
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

    while(sp > 0) {
        int nodeid = stack[--sp];
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
