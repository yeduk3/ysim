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

// Swept-segment-vs-triangle narrow phase (D-013). Replaces the prior
// snapshot point-vs-triangle distance check, which silently missed contacts
// where a fast-moving thin cloth crossed a static surface between samples
// (CM-005). The swept check looks at the segment from x_prev (start of the
// previous substep's integrate, snapshotted by Simulator::update before
// system.update runs) to x_cur (current position): if the segment crosses
// the triangle plane OR the current position is within radius+thickness of
// the plane, register a contact with signed distance. The integrator's
// `(thickness - distance) * n` push grows correctly for both barely-above
// and tunneled particles because `distance` is now signed (negative ⇒
// penetrating), unlike the prior abs'd `l`.
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

    // x_prev (start-of-prior-substep position; D-013).
    device const packed_float3* scenePackedXPrev [[buffer(10)]],

    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numBroadCollisions) return;

    BroadCollision bc = broadCollisions[id];

    // Slice (c-1) A6, runtime-gated: only when params.skipSphere is
    // set (Simulator::useAnalyticPrimitive == true) are sphere pairs
    // skipped here so narrow_pt_analytic feeds them instead (no
    // double-feed). When the toggle is OFF (default) skipSphere==0 and
    // spheres flow through the original triangle-soup path unchanged.
    // Cube/Cylinder always use triangle soup until c-3.
    if (params.skipSphere != 0u
     && (bc.shapePair.x == YSIM_SHAPE_SPHERE
      || bc.shapePair.y == YSIM_SHAPE_SPHERE)) return;

    uint point    = bc.indexPair.x;
    uint triangle = bc.indexPair.y;

    uint qObjId = bc.objPair.x;
    uint tObjId = bc.objPair.y;

    uint3 tri = scenePackedFacets[triangle + scenePackedFacetsOffsets[tObjId]];
    uint f0 = tri.x;
    uint f1 = tri.y;
    uint f2 = tri.z;

    if (qObjId == tObjId) { // same object
        uint gPoint = point+scenePackedPositionsOffsets[tObjId];
        for(uint i = sceneVertexAdjFacetsOffsets[gPoint]; i < sceneVertexAdjFacetsOffsets[gPoint+1]; ++i) {
            uint f = sceneVertexAdjFacets[i];
            if (f == triangle) return;
        }
    }

    uint qOff = scenePackedPositionsOffsets[qObjId];
    uint tOff = scenePackedPositionsOffsets[tObjId];

    float3 qcur  = float3(scenePackedPositions[point + qOff]);
    float3 qprev = float3(scenePackedXPrev[point + qOff]);
    float3 t0pos = float3(scenePackedPositions[f0 + tOff]);
    float3 t1pos = float3(scenePackedPositions[f1 + tOff]);
    float3 t2pos = float3(scenePackedPositions[f2 + tOff]);

    float3 v0 = t1pos - t0pos;
    float3 v1 = t2pos - t0pos;

    float3 n = cross(v0, v1);
    float nlen2 = dot(n, n);
    if (nlen2 < 1e-12f) return;
    n *= rsqrt(nlen2);

    // Signed distances from the triangle plane to query positions.
    float d_prev = dot(n, qprev - t0pos);
    float d_cur  = dot(n, qcur  - t0pos);

    // Orient n outward toward x_prev's side. Falls back to x_cur's side when
    // d_prev is exactly 0 (degenerate first-substep case where xPrev was
    // seeded equal to x).
    float dref = (d_prev != 0.0f) ? d_prev : d_cur;
    if (dref < 0.0f) {
        n = -n;
        d_prev = -d_prev;
        d_cur  = -d_cur;
    }

    // CCD trigger: segment crossed the plane OR snapshot proximity (slow
    // particle inside the radius+thickness band).
    bool crossed   = (d_cur < 0.0f);
    bool inMargin  = (d_cur < params.radius + params.thickness);
    if (!crossed && !inMargin) return;

    // Choose contact point: crossing of the segment if it actually crossed
    // the plane (and d_prev was on the outward side); otherwise the current
    // position projected to the plane.
    float3 contactPoint;
    if (crossed && d_prev > 0.0f) {
        float t = d_prev / (d_prev - d_cur);
        contactPoint = mix(qprev, qcur, t);
    } else {
        contactPoint = qcur;
    }

    float3 p_contact = contactPoint - t0pos;
    float l_contact = dot(n, p_contact);
    float3 inplane = p_contact - n * l_contact;

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
    // SIGNED current distance — negative ⇒ tunneled. The integrator's
    // `(thickness - distance) * n` push grows accordingly.
    narrowCollisions[outIdx].collisionNormalAndDistance = float4(n, d_cur);
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

// Slice (c-1) — analytic cloth-vertex vs primitive narrow phase.
// One thread per cloth vertex of ONE cloth mesh (oid). Loops the
// compact AnalyticShape array and, for spheres (c-1 scope), emits a
// NarrowCollision into the SAME shared narrowCollisions buffer the
// triangle path uses, so the existing CPU sort + unchanged cloth
// integrators consume it transparently. DCD only (Q2): tests the
// current position, no swept xPrev. Cube/Cylinder are skipped here
// (still triangle-soup) until c-3.
kernel void narrow_pt_analytic(
    device atomic_uint*          numNarrowCollisions [[buffer(0)]],
    device NarrowCollision*      narrowCollisions    [[buffer(1)]],
    device const packed_float3*  scenePackedPositions [[buffer(2)]],
    device const uint*           scenePackedPositionsOffsets [[buffer(3)]],
    device const AnalyticShape*  shapes              [[buffer(4)]],
    constant AnalyticNarrowParams& p                 [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= p.numVerts) return;

    uint gid = scenePackedPositionsOffsets[p.oid] + id;
    float3 pos = float3(scenePackedPositions[gid]);

    for (uint s = 0u; s < p.numShapes; ++s) {
        AnalyticShape sh = shapes[s];
        if ((sh.flags & 1u) == 0u) continue;        // not collidable
        if (sh.shapeType != YSIM_SHAPE_SPHERE) continue; // c-1: sphere only

        float3 c = sh.centerRadius.xyz;
        float  r = sh.centerRadius.w;

        float3 dir = pos - c;
        float  len = length(dir);

        // Signed surface distance; gate matches narrow_pt_tri
        // (radius + thickness band). Negative ⇒ penetrating.
        float d = len - r;
        if (d >= p.radius + p.thickness) continue;

        // Outward radial normal. Degenerate at the exact center —
        // pick +Y so the contact is still well-formed.
        float3 n = (len > 1e-6f) ? (dir / len) : float3(0.0f, 1.0f, 0.0f);

        uint outIdx = atomic_fetch_add_explicit(
            numNarrowCollisions, 1u, memory_order_relaxed);
        if (outIdx >= p.maxNumCollisions) return;

        narrowCollisions[outIdx].indexPair = uint2(id, 0u);
        narrowCollisions[outIdx].objPair   = uint2(p.oid, sh.objId);
        narrowCollisions[outIdx].collisionNormalAndDistance = float4(n, d);
        narrowCollisions[outIdx].behaviorPair = uint2(p.clothBehavior,
                                                      sh.behaviorType);
        narrowCollisions[outIdx].shapePair = uint2(0u, sh.shapeType);
    }
}
