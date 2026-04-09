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

//kernel void radixSort_Histogram(
//    mortons,
//    threadgroup histogram[group][256?],
//    uint pass;
//    uint3 tid [[thread_position_in_grid]]
//) {
//    // accumulate per-thread-group histogram
//    // threadgroup uint histogram[threads per threadgroup]
//    // bucket = (mortons[lid].code >> shift) & MASK
//    // histogram[thread_group][bucket]++
//    // 
//}
//
//kernel void radixSort_PrefixSum(
//
//) {
//    // 
//}

//kernel void sortMortons(
//    device MortonNode* mortons [[buffer()]],
//    device MortonNode* mortons [[buffer()]],
//    uint id 
//) {
//
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

kernel void buildTree_Tri(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint3* facets [[buffer(1)]],
    constant AABB4& sceneBox [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    int numPrimitives = sceneBox._pad0;
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


kernel void buildTree_Edge(
    device const packed_float3* x [[buffer(0)]],
    device const packed_uint2* edges [[buffer(1)]],
    constant AABB4& sceneBox [[buffer(2)]],
    device const MortonNode* mortons [[buffer(3)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    int numPrimitives = sceneBox._pad0;
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

kernel void bottomUpBoxes(
    constant AABB4& sceneBox [[buffer(2)]],
    device BVHNode* tree [[buffer(4)]],
    device int* treeParent [[buffer(5)]],
    device atomic_uint* treeVisitCounts [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    uint numPrimitives = (uint)sceneBox._pad0;
    if (id >= numPrimitives) return;

    int child = int(id + numPrimitives - 1); // leaf node index

    while (true) {
        int parent = treeParent[child];

        // old == 0 : first child arrives
        // old == 1 : second child arrives
        uint old = atomic_fetch_add_explicit(
            &treeVisitCounts[parent],
            1u,
            memory_order_relaxed // this is problematic. No guarantee for the saving childA, B
        );

        if (old == 0u) {
            // first child: do nothing more
            return;
        }

        // second child: both children are now ready
        int childA = tree[parent].childA;
        int childB = tree[parent].childB;

        float3 minA = float3(tree[childA].min);
        float3 maxA = float3(tree[childA].max);
        float3 minB = float3(tree[childB].min);
        float3 maxB = float3(tree[childB].max);

        tree[parent].min = packed_float3(min(minA, minB));
        tree[parent].max = packed_float3(max(maxA, maxB));

        // now parent is finished, continue upward
        child = parent;
        if (child == 0) return; // root reached
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

//kernel void queryPoints(
//    device const packed_float3* x [[buffer(0)]],
//    device const packed_uint3* facets [[buffer(1)]],
//    device const BVHNode* tree [[buffer(2)]],
//    constant QueryPointsParams& qParams [[buffer(3)]],
//    device BroadCollision* broadCollisions [[buffer(4)]],
//    device atomic_uint* numBroadCollisions [[buffer(5)]],
//    device QueryFlag* qFlag [[buffer(6)]],
//    uint id [[thread_position_in_grid]]
//) {
//    if(id >= qParams.numPoints) return;
//
//    float3 pos = x[id];
//    float3 margin = float3(qParams.queryMargin);
//    float3 qmin = pos-margin;
//    float3 qmax = pos+margin;
//
//    const int stackDepth = 64;
//
//    int stack[stackDepth];
//    int sp = 0;
//    stack[sp++] = 0;
//
//    while(sp > 0) {
//        int nodeid = stack[--sp];
//        BVHNode node = tree[nodeid];
//
//        if (!intersectAABB(qmin, qmax, node))
//            continue;
//
//        if (node.childA < 0) { // leaf
//            uint fid = (uint)node.childB;
//            uint3 facet = facets[fid];
//            if(id == facet.x || id == facet.y || id == facet.z) continue;
//
//            uint idx = atomic_fetch_add_explicit(numBroadCollisions, 1u, memory_order_relaxed);
//            if(idx >= qParams.maxNumCollisions) {
//                qFlag[0].collisionOverflow = 1u;
//                continue;
//            }
//            broadCollisions[idx].indexPair = {id, fid};
//            broadCollisions[idx].objPair = {qParams.qObjId, qParams.tObjId};
//            broadCollisions[idx].behaviorPair = {qParams.qBehavior, qParams.tBehavior};
//            broadCollisions[idx].shapePair = {qParams.qShape, qParams.tShape};
//            continue;
//        }
//        if (sp + 2 > stackDepth) {
//            qFlag[0].stackOverflow = 1u;
//            continue;
//        }
//
//        stack[sp++] = node.childA;
//        stack[sp++] = node.childB;
//    }
//}


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
