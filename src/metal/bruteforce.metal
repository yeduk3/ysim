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

    // numBroadCollisions counter (update() sync refactor, P4). The async
    // narrow path (None/PerFrame) dispatches over maxNumCollisions and sets
    // params.numBroadCollisions = maxNumCollisions, then bounds the real work
    // with this GPU-side count — no CPU read of the broad count. InFrame
    // dispatches over the tight count as before, so this check is redundant
    // there (id is already < params.numBroadCollisions <= numBroadBuf[0]).
    device const uint* numBroadBuf [[buffer(11)]],

    uint id [[thread_position_in_grid]]
) {
    if (id >= params.numBroadCollisions) return;
    if (id >= numBroadBuf[0]) return;

    BroadCollision bc = broadCollisions[id];

    // P1 (decision 2): when the analytic path is live, every row whose
    // either lane carries a non-Mesh ColliderKind is skipped here so
    // narrow_pt_analytic feeds it instead (no double-feed). Under the
    // SH / multi-level-SH broad phases no analytic markers exist, so
    // the CPU passes skipAnalytic == 0 and analytic colliders flow
    // through the triangle soup unchanged (no contact holes).
    if (params.skipAnalytic != 0u
     && (bc.shapePair.x != YSIM_COLLIDER_MESH
      || bc.shapePair.y != YSIM_COLLIDER_MESH)) return;

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

// Rotate vector v by unit quaternion (scalar qw, vector qv). Forward
// (local→world) uses qv as-is; inverse (world→local) passes -qv (the
// conjugate of a unit quaternion). v + 2·qv×(qv×v + qw·v).
inline float3 quatRotate(float qw, float3 qv, float3 v) {
    return v + 2.0f * cross(qv, cross(qv, v) + qw * v);
}

// Dominant box axis for a local point q against half-extents h: the
// axis maximising (|q_i| - h_i). Outside the box that is the dominant
// separating axis (the face plane whose signed distance is the largest
// lower bound on the true distance — exact in the face Voronoi region,
// conservative near edges/corners). Inside the box it is the
// minimum-penetration axis, i.e. the cheapest way out. Returns the axis
// in `axis` and the outward side in `sgn`.
inline void boxDominantAxis(float3 q, float3 h, thread int& axis, thread float& sgn) {
    float3 gap = abs(q) - h;
    axis = 0;
    float best = gap.x;
    if (gap.y > best) { best = gap.y; axis = 1; }
    if (gap.z > best) { best = gap.z; axis = 2; }
    float qk = (axis == 0) ? q.x : ((axis == 1) ? q.y : q.z);
    sgn = (qk >= 0.0f) ? 1.0f : -1.0f;
}

// P1 — analytic cloth-vertex vs ONE collider narrow phase, with swept
// CCD by MOTION INVERSION (collider_pipeline_rework.md §3).
//
// One thread per cloth vertex of ONE cloth mesh (oid). The BVH broad
// phase already found this (cloth, collider) object pair overlaps and
// skipped descending the collider's triangle BVH, so this kernel tests
// exactly ONE AnalyticShape (p.shapeIndex) — no whole-array loop, no
// per-frame all-cloth×all-shape scan. Emits a NarrowCollision into the
// SAME shared narrowCollisions buffer the triangle path uses, so the
// existing sort + unchanged cloth integrators consume it transparently.
//
// CCD: instead of building a swept volume for the shape (a rotating box
// sweep is non-convex), the shape's motion is inverted into the vertex:
//   a = worldToLocal(shape @ t0, x_prev)
//   b = worldToLocal(shape @ t1, x_cur)
// and the STATIC canonical shape is tested against the segment a→b.
// Exact for translation, 1st-order for rotation — p.rotMargin
// (theta * r_max, CPU-computed) inflates the band to absorb the
// linearization error, so the test can only over-report.
//
// Emit convention (identical to narrow_pt_tri, which is what the
// integrator's `(thickness - d) * n` push and PBD's `nd.w + n·(p−x)`
// re-evaluation expect): pick a contact surface point s and outward
// normal n at the entry/TOI point (or the closest feature when the
// segment never crossed), then emit the CURRENT position's signed
// distance measured along that normal, d = dot(n, p_cur - s) — the
// tangent-plane linearization of the contact. Negative d ⇒ penetrating
// (including a full tunnel-through, which lands far on the inside and
// therefore gets a large restoring push toward the ENTRY side).
//
// Entry-side rule: when a is already inside the real surface, the
// feature is chosen from a, never from b. That is what stops a deeply
// penetrating vertex from being ejected through the FAR side.
kernel void narrow_pt_analytic(
    device atomic_uint*          numNarrowCollisions [[buffer(0)]],
    device NarrowCollision*      narrowCollisions    [[buffer(1)]],
    device const packed_float3*  scenePackedPositions [[buffer(2)]],
    device const uint*           scenePackedPositionsOffsets [[buffer(3)]],
    device const AnalyticShape*  shapes              [[buffer(4)]],
    constant AnalyticNarrowParams& p                 [[buffer(5)]],
    // x_prev (start-of-prior-substep position), the same buffer
    // narrow_pt_tri binds at slot 10 — the swept segment's tail.
    device const packed_float3*  scenePackedXPrev    [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= p.numVerts) return;

    AnalyticShape sh = shapes[p.shapeIndex];
    if ((sh.flags & 1u) == 0u) return;              // not collidable
    if (sh.kind == YSIM_COLLIDER_MESH) return;      // triangle soup, not ours

    uint gid = scenePackedPositionsOffsets[p.oid] + id;
    float3 pCur  = float3(scenePackedPositions[gid]);
    float3 pPrev = float3(scenePackedXPrev[gid]);

    // ── motion inversion ────────────────────────────────────────────
    // quatRotate(qw, -qv, v) = inverse rotation (conjugate of a unit
    // quaternion). Packed order is (w, x, y, z).
    float3 a = quatRotate(sh.prevRotQuat.x, -sh.prevRotQuat.yzw,
                          pPrev - sh.prevCenterPad.xyz);
    float3 b = quatRotate(sh.rotQuat.x,     -sh.rotQuat.yzw,
                          pCur  - sh.centerRadius.xyz);
    float3 seg = b - a;

    // Contact band. rotMargin is 0 for a non-rotating collider (all v1
    // scenes), so this is exactly the triangle path's radius+thickness.
    const float band = p.radius + p.thickness + p.rotMargin;

    float3 nLocal = float3(0.0f, 1.0f, 0.0f);
    float  d      = 1e30f;

    if (sh.kind == YSIM_COLLIDER_PLANE) {
        // Infinite half-space, local +Y up (§1). "Outside" is the +Y
        // half-space and nothing else, so the outward normal is the
        // CONSTANT local +Y — which trivially satisfies the entry-side
        // rule (there is no far side to flip to) and lets a vertex that
        // starts below the floor recover upward instead of being held
        // under it. A two-sided sheet is a Box collider, not a Plane.
        // s = (b.x, 0, b.z) ⇒ d = dot(+Y, b - s) = b.y.
        // No segment term is needed: a half-space cannot be tunneled by a
        // snapshot test — "below the plane" stays inside no matter how fast
        // the vertex got there — so `a` carries no extra information here.
        nLocal = float3(0.0f, 1.0f, 0.0f);
        d = b.y;
    }
    else if (sh.kind == YSIM_COLLIDER_SPHERE) {
        // Ellipsoid with semi-axes e (a uniform sphere reduces exactly).
        // Work in the normalised space u = q/e where the surface is the
        // UNIT sphere; the band maps to 1 + band/min(e) — exact for a
        // uniform sphere, conservative (over-inflated) for an ellipsoid.
        float3 e = max(sh.halfExtHeight.xyz, float3(1e-8f));
        float  R = 1.0f + band / min(e.x, min(e.y, e.z));
        float3 A = a / e;
        float3 B = b / e;
        float  lenA = length(A);

        float3 u;
        bool   got = false;
        if (lenA < 1.0f) {
            // ENTRY-SIDE RULE: already inside ⇒ push along a's radial
            // direction. Using b here is what would eject a deeply
            // penetrating vertex out the far side.
            u = (lenA > 1e-6f) ? A
              : ((length(B) > 1e-6f) ? B : float3(0.0f, 1.0f, 0.0f));
            got = true;
        } else {
            // Quadratic |A + t(B-A)| = R, first root in [0,1].
            float3 D  = B - A;
            float  qa = dot(D, D);
            float  qb = dot(A, D);
            float  qc = dot(A, A) - R * R;
            float  t  = -1.0f;
            if (qa > 1e-20f) {
                float disc = qb * qb - qa * qc;
                if (disc >= 0.0f) {
                    float t0 = (-qb - sqrt(disc)) / qa;
                    if (t0 >= 0.0f && t0 <= 1.0f) t = t0;
                }
            }
            if (t >= 0.0f) { u = A + t * D; got = true; }
            else if (length(B) > 1e-6f) { u = B; got = true; }
        }
        if (!got) return;

        float3 uh = normalize(u);
        float3 s  = e * uh;                 // point on the REAL ellipsoid
        // Outward normal = gradient of Sum (q_i/e_i)^2, i.e. q/e^2.
        nLocal = normalize(uh / e);
        d = dot(nLocal, b - s);
    }
    else if (sh.kind == YSIM_COLLIDER_BOX) {
        float3 h = max(sh.halfExtHeight.xyz, float3(1e-8f));
        float3 H = h + band;
        int   k  = 0;
        float sg = 1.0f;

        if (all(abs(a) < h)) {
            // ENTRY-SIDE RULE: started inside ⇒ minimum-penetration
            // axis of a, side taken from a.
            boxDominantAxis(a, h, k, sg);
        } else {
            // Slab test of the segment against the band-inflated box.
            float tEnter = 0.0f, tExit = 1.0f;
            int   ek = -1;
            float esg = 1.0f;
            bool  miss = false;
            for (int i = 0; i < 3 && !miss; ++i) {
                float di = (i == 0) ? seg.x : ((i == 1) ? seg.y : seg.z);
                float ai = (i == 0) ? a.x   : ((i == 1) ? a.y   : a.z);
                float Hi = (i == 0) ? H.x   : ((i == 1) ? H.y   : H.z);
                if (fabs(di) < 1e-12f) {
                    if (fabs(ai) > Hi) miss = true;
                } else {
                    float inv = 1.0f / di;
                    float t1 = (-Hi - ai) * inv;
                    float t2 = ( Hi - ai) * inv;
                    float tn = min(t1, t2), tf = max(t1, t2);
                    if (tn > tEnter) { tEnter = tn; ek = i; esg = (di > 0.0f) ? -1.0f : 1.0f; }
                    tExit = min(tExit, tf);
                }
            }
            if (!miss && ek >= 0 && tEnter <= tExit && tEnter <= 1.0f) {
                k = ek; sg = esg;           // entry face of the crossing
            } else {
                boxDominantAxis(b, h, k, sg);   // closest face of b
            }
        }
        nLocal = float3(k == 0 ? sg : 0.0f, k == 1 ? sg : 0.0f, k == 2 ? sg : 0.0f);
        // s = b with component k pinned to the face ⇒ d = sg*b[k] - h[k].
        float bk = (k == 0) ? b.x : ((k == 1) ? b.y : b.z);
        float hk = (k == 0) ? h.x : ((k == 1) ? h.y : h.z);
        d = sg * bk - hk;
    }
    else if (sh.kind == YSIM_COLLIDER_CYLINDER) {
        // Local +Y axis, radius r, half-height hh.
        float r  = max(sh.centerRadius.w,  1e-8f);
        float hh = max(sh.halfExtHeight.w, 1e-8f);
        float R  = r + band;
        float HH = hh + band;

        // feature 0 = lateral wall, 1 = cap.
        int   feat  = 0;
        float capSg = 1.0f;
        float2 nxz  = float2(1.0f, 0.0f);

        // Closest feature of a local point: whichever gap (r - |q_xz|,
        // hh - |q_y|) is smaller — minimum penetration inside, dominant
        // separating feature outside.
        bool aInside = (length(a.xz) < r) && (fabs(a.y) < hh);
        bool resolved = false;
        if (!aInside) {
            // Segment vs band-inflated capped cylinder: radial quadratic
            // intersected with the cap slab.
            float tEnter = 0.0f, tExit = 1.0f;
            int   ef = -1;
            float ecap = 1.0f;
            float2 eXZ = float2(1.0f, 0.0f);
            bool  miss = false;

            float2 A2 = a.xz, D2 = seg.xz;
            float qa = dot(D2, D2);
            float qb = dot(A2, D2);
            float qc = dot(A2, A2) - R * R;
            if (qa > 1e-20f) {
                float disc = qb * qb - qa * qc;
                if (disc < 0.0f) miss = true;
                else {
                    float sq = sqrt(disc);
                    float t1 = (-qb - sq) / qa;
                    float t2 = (-qb + sq) / qa;
                    if (t1 > tEnter) { tEnter = t1; ef = 0; }
                    tExit = min(tExit, t2);
                }
            } else if (qc > 0.0f) {
                miss = true;                 // axis-parallel and outside R
            }
            if (!miss) {
                if (fabs(seg.y) < 1e-12f) {
                    if (fabs(a.y) > HH) miss = true;
                } else {
                    float inv = 1.0f / seg.y;
                    float t1 = (-HH - a.y) * inv;
                    float t2 = ( HH - a.y) * inv;
                    float tn = min(t1, t2), tf = max(t1, t2);
                    if (tn > tEnter) { tEnter = tn; ef = 1; ecap = (seg.y > 0.0f) ? -1.0f : 1.0f; }
                    tExit = min(tExit, tf);
                }
            }
            if (!miss && ef >= 0 && tEnter <= tExit && tEnter <= 1.0f) {
                if (ef == 0) {
                    float2 hit = A2 + tEnter * D2;
                    float  l   = length(hit);
                    eXZ = (l > 1e-9f) ? (hit / l) : float2(1.0f, 0.0f);
                }
                feat = ef; capSg = ecap; nxz = eXZ; resolved = true;
            }
        }
        if (!resolved) {
            // Entry-side rule when a is inside; otherwise the closest
            // feature of the current position.
            float3 q = aInside ? a : b;
            float  lxz = length(q.xz);
            feat  = ((r - lxz) <= (hh - fabs(q.y))) ? 0 : 1;
            capSg = (q.y >= 0.0f) ? 1.0f : -1.0f;
            nxz   = (lxz > 1e-9f) ? (q.xz / lxz) : float2(1.0f, 0.0f);
        }

        if (feat == 0) {
            nLocal = float3(nxz.x, 0.0f, nxz.y);
            d = dot(nLocal, b) - r;          // s = r*n + b.y*(0,1,0)
        } else {
            nLocal = float3(0.0f, capSg, 0.0f);
            d = capSg * b.y - hh;            // s = (b.x, capSg*hh, b.z)
        }
    }
    else {
        return;                              // unknown kind
    }

    // Gate mirrors narrow_pt_tri: crossing lands at d < 0 and is
    // therefore covered; the band term catches slow proximity contacts.
    // rotMargin only ever widens it (conservative).
    if (!(d < band)) return;

    // Normal back to world with the CURRENT transform.
    float3 n = normalize(quatRotate(sh.rotQuat.x, sh.rotQuat.yzw, nLocal));

    uint outIdx = atomic_fetch_add_explicit(
        numNarrowCollisions, 1u, memory_order_relaxed);
    if (outIdx >= p.maxNumCollisions) return;

    narrowCollisions[outIdx].indexPair = uint2(id, 0u);
    // §4: objPair is the ARRAY-INDEX namespace on BOTH lanes, exactly
    // like the triangle path (bvh.hpp queryPoints passes indices).
    narrowCollisions[outIdx].objPair   = uint2(p.oid, sh.objIndex);
    narrowCollisions[outIdx].collisionNormalAndDistance = float4(n, d);
    narrowCollisions[outIdx].behaviorPair = uint2(p.clothBehavior,
                                                  sh.behaviorType);
    // Debug/forward-compat only (A5). Query lane is the deformable side.
    narrowCollisions[outIdx].shapePair = uint2(YSIM_COLLIDER_MESH, sh.kind);
}
