#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"



inline bool pointInTriangleBary(
    float3 p,   // point projected into triangle plane, expressed from t0
    float3 v0,  // t1 - t0
    float3 v1   // t2 - t0
) {
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(p,  v0);
    float d21 = dot(p,  v1);

    float denom = d00 * d11 - d01 * d01;
    if (fabs(denom) < 1e-12f) return false;

    float b = (d11 * d20 - d01 * d21) / denom;
    float c = (d00 * d21 - d01 * d20) / denom;
    float a = 1.0f - b - c;

    return (a >= 0.0f && b >= 0.0f && c >= 0.0f);
}

kernel void narrow_pt_tri(
    device const BroadCollision* broadCollisions [[buffer(0)]],
    device atomic_uint* numNarrowCollisions      [[buffer(1)]],
    device NarrowCollision* narrowCollisions     [[buffer(2)]],

    // query mesh state
    device const packed_float3* scenePackedPositions       [[buffer(3)]],
    device const uint* scenePackedPositionsOffsets       [[buffer(4)]],

    // target mesh state
    device const packed_uint3*  scenePackedFacets          [[buffer(5)]],
    device const uint*  scenePackedFacetsOffsets          [[buffer(6)]],

    device const uint* sceneVertexAdjFacets [[buffer(7)]],
    device const uint* sceneVertexAdjFacetsOffsets [[buffer(8)]],

    constant NarrowParams& params                [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numBroadCollisions) return;

    BroadCollision bc = broadCollisions[id];

    uint point    = bc.indexPair.x;
    uint triangle = bc.indexPair.y;

    uint qObjId = bc.objPair.x;
    uint tObjId = bc.objPair.y;

    uint3 tri = scenePackedFacets[triangle + scenePackedFacetsOffsets[tObjId]];
    uint f0 = tri.x;
    uint f1 = tri.y;
    uint f2 = tri.z;

    if (qObjId == tObjId) { // same object
        //if (point == f0 || point == f1 || point == f2) return;
        uint gPoint = point+scenePackedPositionsOffsets[tObjId];
        for(uint i = sceneVertexAdjFacetsOffsets[gPoint]; i < sceneVertexAdjFacetsOffsets[gPoint+1]; ++i) {
            uint f = sceneVertexAdjFacets[i];
            if (f == triangle) return;
        }
    }

    

    float3 qpos  = float3(scenePackedPositions[point + scenePackedPositionsOffsets[qObjId]]);
    float3 t0pos = float3(scenePackedPositions[f0 + scenePackedPositionsOffsets[tObjId]]);
    float3 t1pos = float3(scenePackedPositions[f1 + scenePackedPositionsOffsets[tObjId]]);
    float3 t2pos = float3(scenePackedPositions[f2 + scenePackedPositionsOffsets[tObjId]]);

    float3 v0 = t1pos - t0pos;
    float3 v1 = t2pos - t0pos;

    float3 n = cross(v0, v1);
    float nlen2 = dot(n, n);
    if (nlen2 < 1e-12f) return;
    n *= rsqrt(nlen2);

    float3 p = qpos - t0pos;
    float l = dot(n, p);

    // query point 기준으로 normal 방향 정렬
    if (l < 0.0f) {
        n = -n;
        l = -l;
    }

    if (l > params.radius + params.thickness) return;

    // 평면 위 투영 벡터
    float3 inplane = p - n * l;

    if (!pointInTriangleBary(inplane, v0, v1)) return;

    uint outIdx = atomic_fetch_add_explicit(
        numNarrowCollisions,
        1u,
        memory_order_relaxed
    );

    if (outIdx >= params.maxNumCollisions) {
        // 넘치면 그냥 버림
        return;
    }

    narrowCollisions[outIdx].indexPair = uint2(point, triangle);
    narrowCollisions[outIdx].objPair = bc.objPair;
    narrowCollisions[outIdx].collisionNormalAndDistance = float4(n, l);
    narrowCollisions[outIdx].behaviorPair = bc.behaviorPair;
    narrowCollisions[outIdx].shapePair = bc.shapePair;
}


kernel void fill_vf_offsets(
    device uint* numNarrowCollisions [[buffer(1)]],
    device NarrowCollision* narrowCollisions [[buffer(2)]],
    device const uint* scenePackedPositionsOffsets [[buffer(4)]],
    device atomic_uint* vertColFacetsOffsets [[buffer(8)]],
    uint id [[thread_position_in_grid]]
) {
    if(id >= numNarrowCollisions[0]) return;

    NarrowCollision nc = narrowCollisions[id];
    uint ppid = scenePackedPositionsOffsets[nc.objPair.x] + nc.indexPair.x;
    atomic_fetch_add_explicit(&vertColFacetsOffsets[ppid+1], 1u, memory_order_relaxed);
}
