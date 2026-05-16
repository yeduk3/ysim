#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

#include "common_types.metalh"

// CPU에서 한 번에 넘겨줄 설정값들 (C++의 구조체와 동일해야 합니다)
struct SimParams {
    float subh;
    float G;
    float kair;
    float kd;
    uint vertexNum;
    float acctime;
};


// ========================================================
// [커널 1] 힘만 계산하는 파이프라인
// ========================================================
kernel void compute_forces(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    constant SimParams& params [[buffer(6)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return; 

    float3 vel = v[id];
    
    float3 force = float3(0.0, params.G * m[id], 0.0) + vel * params.kair;
    f[id] = force;
}

kernel void compute_spring_forces(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device atomic_float* f_flat [[buffer(2)]],      
    constant SimParams& params [[buffer(6)]],
    device const uint2* springIndices [[buffer(10)]],
    device const float2* springParams [[buffer(11)]],
    uint id [[thread_position_in_grid]]
) {
    // 1. 스프링 정보 읽기
    uint idA = springIndices[id].x;
    uint idB = springIndices[id].y;
    float restLength = springParams[id].x;
    float kspring = springParams[id].y;

    float3 pA = x[idA];
    float3 pB = x[idB];
    float3 vA = v[idA];
    float3 vB = v[idB];

    // 2. 물리 수식 계산
    float3 dx = pB - pA;
    float len = length(dx);
    if(len < 1e-9) return;

    float3 dv = vB - vA;
    float3 ndx = dx / len;
    float3 sf = (kspring * (len - restLength) + params.kd * dot(dv, ndx)) * ndx;

    // idA의 x, y, z 인덱스는 각각 idA*3+0, idA*3+1, idA*3+2 입니다.
    atomic_fetch_add_explicit(&f_flat[idA * 3 + 0], sf.x, memory_order_relaxed);
    atomic_fetch_add_explicit(&f_flat[idA * 3 + 1], sf.y, memory_order_relaxed);
    atomic_fetch_add_explicit(&f_flat[idA * 3 + 2], sf.z, memory_order_relaxed);

    atomic_fetch_sub_explicit(&f_flat[idB * 3 + 0], sf.x, memory_order_relaxed);
    atomic_fetch_sub_explicit(&f_flat[idB * 3 + 1], sf.y, memory_order_relaxed);
    atomic_fetch_sub_explicit(&f_flat[idB * 3 + 2], sf.z, memory_order_relaxed);
}


//struct NarrowCollision {
//    uint2 indexPair;     // 혹은 네 구조에 맞는 형태
//    uint2 objPair;
//    float4 collisionNormalAndDistance;
//};

// ========================================================
// [커널 2] 위치와 속도만 업데이트하는 파이프라인
// ========================================================

inline float3 calc_spring(float3 p0, float3 v0, float3 p1, float3 v1, float rest_len, float ks, float kd) {
    float3 dx = p1 - p0;
    float3 dv = v1 - v0;
    float len = length(dx);
    if (len < 1e-7) return float3(0.0); // 0 나누기 방지
    float3 ndx = dx / len;
    return (ks * (len - rest_len) + kd * dot(dv, ndx)) * ndx;
}



struct ClothGridParams {
    uint particleNum1D;
    float stretchRest, shearRest, bendRest;
    float kstretch, kshear, kbend;
    float thickness;
};

kernel void compute_cloth_grid_forces_fast(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    // External-forces buffer (gravity*mass + wind, filled per frame by
    // Simulator::applyEnvironmentForces from Scene::environment).
    device const packed_float3* externalForces [[buffer(7)]],
    constant SimParams& params [[buffer(8)]],
    constant ClothGridParams& clothParams [[buffer(9)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return;

    uint N = clothParams.particleNum1D;

    uint row = id / N;
    uint col = id % N;

    // packed_float3를 연산이 빠른 float3로 캐스팅하여 사용
    float3 pos = x[id];
    float3 vel = v[id];

    // 1. 환경 외력 (중력 + 바람) + 공기 저항. 중력은 더 이상 params.G로 하드코딩하지 않고,
    //    Scene::environment 가 GUI/JSON 경로로 채운 externalForces 값을 그대로 사용한다.
    float3 force = float3(externalForces[id]) + vel * params.kair;

    // 1D 인덱스를 구하는 람다 함수
    auto get_idx = [N](uint r, uint c) { return r * N + c; };

    // 2. 스프링 힘 (Gather) - 조건에 맞는 이웃이 있으면 힘을 누적합니다.
    // [Stretch] 상하좌우
    if (col > 0)    force += calc_spring(pos, vel, x[get_idx(row, col-1)], v[get_idx(row, col-1)], clothParams.stretchRest, clothParams.kstretch, params.kd);
    if (col < N-1)  force += calc_spring(pos, vel, x[get_idx(row, col+1)], v[get_idx(row, col+1)], clothParams.stretchRest, clothParams.kstretch, params.kd);
    if (row > 0)    force += calc_spring(pos, vel, x[get_idx(row-1, col)], v[get_idx(row-1, col)], clothParams.stretchRest, clothParams.kstretch, params.kd);
    if (row < N-1)  force += calc_spring(pos, vel, x[get_idx(row+1, col)], v[get_idx(row+1, col)], clothParams.stretchRest, clothParams.kstretch, params.kd);

    // [Shear] 대각선
    if (col > 0 && row > 0)     force += calc_spring(pos, vel, x[get_idx(row-1, col-1)], v[get_idx(row-1, col-1)], clothParams.shearRest, clothParams.kshear, params.kd);
    if (col < N-1 && row > 0)   force += calc_spring(pos, vel, x[get_idx(row-1, col+1)], v[get_idx(row-1, col+1)], clothParams.shearRest, clothParams.kshear, params.kd);
    if (col > 0 && row < N-1)   force += calc_spring(pos, vel, x[get_idx(row+1, col-1)], v[get_idx(row+1, col-1)], clothParams.shearRest, clothParams.kshear, params.kd);
    if (col < N-1 && row < N-1) force += calc_spring(pos, vel, x[get_idx(row+1, col+1)], v[get_idx(row+1, col+1)], clothParams.shearRest, clothParams.kshear, params.kd);

    // [Bend] 2칸 너머
    if (col > 1)    force += calc_spring(pos, vel, x[get_idx(row, col-2)], v[get_idx(row, col-2)], clothParams.bendRest, clothParams.kbend, params.kd);
    if (col < N-2)  force += calc_spring(pos, vel, x[get_idx(row, col+2)], v[get_idx(row, col+2)], clothParams.bendRest, clothParams.kbend, params.kd);
    if (row > 1)    force += calc_spring(pos, vel, x[get_idx(row-2, col)], v[get_idx(row-2, col)], clothParams.bendRest, clothParams.kbend, params.kd);
    if (row < N-2)  force += calc_spring(pos, vel, x[get_idx(row+2, col)], v[get_idx(row+2, col)], clothParams.bendRest, clothParams.kbend, params.kd);

    //force += float3(1, 0, 1) * min(row * abs(col-col/2) * abs(cos(params.acctime/2.f)), 50.f);

    // 내 메모리에만 기록!
    f[id] = force;
}

kernel void integrate_cloth_grid(
    device packed_float3* x [[buffer(0)]],
    device packed_float3* v [[buffer(1)]],
    device const packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    device const float* fixedParticle [[buffer(4)]],
    device const NarrowCollision* vertColFacets [[buffer(5)]],
    device const uint* vertColFacetsOffsets [[buffer(6)]],
    constant SimParams& params [[buffer(8)]],
    constant ClothGridParams& clothParams [[buffer(9)]],
    device const uint* statesOffsets [[buffer(10)]],
    constant uint& oid [[buffer(11)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return; 

    float mask = fixedParticle[id];
    
    // float3로 연산 후 packed_float3로 다시 저장
    float3 vel = v[id];
    float3 pos = x[id];
    float3 force = f[id];

    vel += (params.subh * force / m[id]) * mask;

    // apply collision constraints
    uint obase = statesOffsets[oid];
    uint begin = vertColFacetsOffsets[obase+id];
    uint end   = vertColFacetsOffsets[obase+id+1];

    for (uint i = begin; i < end; ++i) {
        float3 n = vertColFacets[i].collisionNormalAndDistance.xyz;

        float nlen2 = dot(n, n);
        if (nlen2 < 1e-12f) continue;

        // 안전하게 normalize
        n *= rsqrt(nlen2);

        float vn = dot(vel, n);

        float distance = vertColFacets[i].collisionNormalAndDistance.w;
        float thickness = clothParams.thickness;

        // D-016: vn-zero and position-push share the (distance < thickness)
        // gate. The narrow-phase fires contacts on the wider radius+thickness
        // band, but the integrator's response only fires when the particle is
        // actually within thickness of the surface. Without this gate, far-
        // from-surface particles in the slow-touch band drained vn (CM-006).
        if (distance < thickness) {
            if (vn < 0.0f) {
                vel -= vn * n;
            }
            pos += (thickness - distance) * n;
        }
    }

    pos += (params.subh * vel) * mask;

    v[id] = vel;
    x[id] = pos;
}

struct ClothParams {
    float kstretch, kshear, kbend;
    float thickness;
};

kernel void compute_tri_spring_forces(
    // state
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    // constraints
    device const float* fixedParticles [[buffer(4)]],
    // External-forces buffer (gravity*mass + wind, filled per frame by
    // Simulator::applyEnvironmentForces from Scene::environment).
    device const packed_float3* externalForces [[buffer(7)]],
    // simulation parameters
    constant SimParams& simParams [[buffer(8)]],
    constant ClothParams& clothParams [[buffer(9)]],
    // adjacency
    device const uint2* edges [[buffer(10)]],
    device const uint* facets [[buffer(11)]],
    // stretch springs
    device const uint* vertexAdjEdges [[buffer(12)]],
    device const uint* vertexAdjEdgesOffsets [[buffer(13)]],
    device const float* restEdgeLengths [[buffer(14)]],
    // bend springs
    device const uint* vertexOppVertices [[buffer(15)]],
    device const uint* vertexOppVerticesOffsets [[buffer(16)]],
    device const float* restOppLengths [[buffer(17)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= simParams.vertexNum) return; 

    float3 pos = x[id];
    float3 vel = v[id];

    // 환경 외력 (중력*질량 + 바람) + 공기 저항. simParams.G 하드코딩은 제거.
    float3 force = float3(externalForces[id]) + vel * simParams.kair;

    // stretch
    uint adjEdgesStart = vertexAdjEdgesOffsets[id];
    uint adjEdgesEnd   = vertexAdjEdgesOffsets[id+1];
    for(uint i = adjEdgesStart; i < adjEdgesEnd; ++i) {
        uint edgeid = vertexAdjEdges[i];
        uint e0 = edges[edgeid].x;
        uint e1 = edges[edgeid].y;
        uint other = 0;
        if(e0 == id) other = e1;
        else other = e0;

        force += calc_spring(pos, vel, x[other], v[other], restEdgeLengths[edgeid], clothParams.kstretch, simParams.kd);
    }

    // bend
    uint oppVerticesStart = vertexOppVerticesOffsets[id];
    uint oppVerticesEnd   = vertexOppVerticesOffsets[id+1];
    for(uint i = oppVerticesStart; i < oppVerticesEnd; ++i) {
        uint other = vertexOppVertices[i];

        force += calc_spring(pos, vel, x[other], v[other], restOppLengths[i], clothParams.kbend, simParams.kd);
    }

    f[id] = force;
}

kernel void integrate_cloth(
    device packed_float3* x [[buffer(0)]],
    device packed_float3* v [[buffer(1)]],
    device const packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    device const float* fixedParticle [[buffer(4)]],
    device const NarrowCollision* vertColFacets [[buffer(5)]],
    device const uint* vertColFacetsOffsets [[buffer(6)]],
    constant SimParams& params [[buffer(8)]],
    constant ClothParams& clothParams [[buffer(9)]],
    device const uint* statesOffsets [[buffer(18)]],
    constant uint& oid [[buffer(19)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return; 

    float mask = fixedParticle[id];
    
    // float3로 연산 후 packed_float3로 다시 저장
    float3 vel = v[id];
    float3 pos = x[id];
    float3 force = f[id];

    vel += (params.subh * force / m[id]) * mask;

    // apply collision constraints
    uint obase = statesOffsets[oid];
    uint begin = vertColFacetsOffsets[obase+id];
    uint end   = vertColFacetsOffsets[obase+id+1];

    for (uint i = begin; i < end; ++i) {
        float3 n = vertColFacets[i].collisionNormalAndDistance.xyz;

        float nlen2 = dot(n, n);
        if (nlen2 < 1e-12f) continue;

        // 안전하게 normalize
        n *= rsqrt(nlen2);

        float vn = dot(vel, n);

        float distance = vertColFacets[i].collisionNormalAndDistance.w;
        float thickness = clothParams.thickness;

        // D-016: vn-zero and position-push share the (distance < thickness)
        // gate. The narrow-phase fires contacts on the wider radius+thickness
        // band, but the integrator's response only fires when the particle is
        // actually within thickness of the surface. Without this gate, far-
        // from-surface particles in the slow-touch band drained vn (CM-006).
        if (distance < thickness) {
            if (vn < 0.0f) {
                vel -= vn * n;
            }
            pos += (thickness - distance) * n;
        }
    }

    pos += (params.subh * vel) * mask;

    v[id] = vel;
    x[id] = pos;
}

// ========================================================
// Reference-point coincidence constraint (point panel).
// Operates on the GLOBAL packed state buffers; `pairs[i]` holds
// {queryGlobalVid, targetGlobalVid} pre-resolved on the host from
// statesOffsets[objId] + localVid, so a single dispatch handles
// cross-mesh constraints. Serial encoder ordering supplies the
// "fence" the design calls for: copy_pos runs before the force
// kernels; copy_force runs after them and before integrate.
// ========================================================

// Step 1-1: snap each follower (query) onto its leader (target)
// BEFORE the cloth force kernels, so the follower's springs are
// evaluated as if it sat at the leader's location.
kernel void ref_constraint_copy_pos(
    device packed_float3* x [[buffer(0)]],
    device packed_float3* xprev [[buffer(1)]],
    device const uint2* pairs [[buffer(2)]],
    constant uint& count [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= count) return;
    uint q = pairs[id].x;
    uint t = pairs[id].y;
    x[q] = x[t];
    xprev[q] = xprev[t];
}

// Step 1-3: after force computation, give the follower the leader's
// force (and velocity, so the subsequent integrate produces an
// identical position update — without matched velocity the two
// points drift apart within the substep).
kernel void ref_constraint_copy_force(
    device packed_float3* f [[buffer(0)]],
    device packed_float3* v [[buffer(1)]],
    device const uint2* pairs [[buffer(2)]],
    constant uint& count [[buffer(3)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= count) return;
    uint q = pairs[id].x;
    uint t = pairs[id].y;
    f[q] = f[t];
    v[q] = v[t];
}

//kernel void integrate_all(
//    device packed_float3* x [[buffer(0)]],
//    device packed_float3* v [[buffer(1)]],
//    device const packed_float3* f [[buffer(2)]],
//    device const float* m [[buffer(3)]],
//    device const float* fixedParticle [[buffer(4)]],
//    device const NarrowCollision* vertColPrims [[buffer(5)]],
//    device const uint* vertColPrimsOffsets [[buffer(6)]],
//    constant SimParams& params [[buffer(8)]],
//    uint id [[thread_position_in_grid]]
//) {
//
//}
