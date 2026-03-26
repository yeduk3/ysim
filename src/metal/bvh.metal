#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

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

kernel void fillMortons(
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
    float3 width = sceneBox.max-sceneBox.min;
    center = (center-sceneBox.min)/width;

    mortons[id].code = mortonCode(center);
    mortons[id].index = id;
}

//kernel void sortMortons(
//    device MortonNode* mortons [[buffer()]],
//    device MortonNode* mortons [[buffer()]],
//    uint id 
//) {
//
//}



//inline void queryAABB(
    
