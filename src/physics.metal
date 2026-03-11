#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

// CPU에서 한 번에 넘겨줄 설정값들 (C++의 구조체와 동일해야 합니다)
struct SimParams {
    float subh;
    float G;
    float kair;
    float kd;
    uint vertexNum;
};


// ========================================================
// [커널 1] 힘만 계산하는 파이프라인
// ========================================================
kernel void compute_forces(
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    constant SimParams& params [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return; 

    float3 vel = v[id];
    
    float3 force = float3(0.0, params.G * m[id], 0.0) + vel * params.kair;
    f[id] = force;
}

kernel void compute_spring_forces(
    device const uint2* springIndices [[buffer(6)]],
    device const float2* springParams [[buffer(7)]],
    device const packed_float3* x [[buffer(0)]],
    device const packed_float3* v [[buffer(1)]],
    device atomic_float* f_flat [[buffer(2)]],      
    constant SimParams& params [[buffer(5)]],
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



// ========================================================
// [커널 2] 위치와 속도만 업데이트하는 파이프라인
// ========================================================
kernel void integrate(
    device packed_float3* x [[buffer(0)]],
    device packed_float3* v [[buffer(1)]],
    device const packed_float3* f [[buffer(2)]],
    device const float* m [[buffer(3)]],
    device const float* fixedParticle [[buffer(4)]],
    constant SimParams& params [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    if (id >= params.vertexNum) return; 

    float mask = fixedParticle[id];
    
    // float3로 연산 후 packed_float3로 다시 저장
    float3 vel = v[id];
    float3 pos = x[id];
    float3 force = f[id];

    vel += (params.subh * force / m[id]) * mask;
    pos += (params.subh * vel) * mask;

    v[id] = vel;
    x[id] = pos;
}
