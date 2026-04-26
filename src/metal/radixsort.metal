#include <metal_stdlib>
#include <metal_atomic>
using namespace metal;

struct SortPair {
    uint key;
    uint value;
};

struct RadixParams {
    uint numBlocks;
    uint numElements;
    uint blockSize;
    uint shift;
};

inline uint getBucket_8Bits(uint key, uint shift) {
    return (key >> shift) & 255u; // 255u is mask for 8bits
}

kernel void radixCountBlock_8Bits(
    device const SortPair* src [[buffer(0)]],
    device uint* blockHist [[buffer(1)]],
    constant RadixParams& params [[buffer(2)]],
    uint tid [[thread_position_in_grid]],
    uint lid [[thread_index_in_threadgroup]], // per threads in a block(tg)
    uint gid [[threadgroup_position_in_grid]] // per blocks(tg) in a grid
) {
    if(gid >= params.numBlocks) return;

    threadgroup atomic_uint localHist[256];
    if(lid < 256) atomic_store_explicit(&localHist[lid], 0u, memory_order_relaxed);
    threadgroup_barrier(mem_flags::mem_threadgroup);

    uint idx = gid * params.blockSize + lid;
    if(idx < params.numElements) {
        uint b = getBucket_8Bits(src[idx].key, params.shift);
        atomic_fetch_add_explicit(&localHist[b], 1u, memory_order_relaxed);
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if(lid < 256) blockHist[gid * 256 + lid] = atomic_load_explicit(&localHist[lid], memory_order_relaxed);
}

kernel void radixScanOffset_8Bits(
    device const uint* blockHist [[buffer(1)]],
    constant RadixParams& params [[buffer(2)]],
    device uint* hist [[buffer(3)]],
    device uint* blockOffset [[buffer(4)]],
    uint tid [[thread_position_in_grid]] // 256, radix per thread as a one threadgroup
) {
    if(tid >= 256) return;

    threadgroup uint offset[256];
    offset[tid] = 0;

    uint sum = 0;
    for(uint block = 0; block < params.numBlocks; ++block) {
        blockOffset[block*256 + tid] = sum; // 해당 block 이전까지 radix가 tid인 값의 개수
        sum += blockHist[block*256 + tid];
    }
    offset[tid] = sum; // radix가 tid인 전체 개수
    hist[tid] = 0;
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if(tid >= 1) return;
    for(uint b = 1; b < 256; ++b) {
        hist[b] = hist[b-1] + offset[b-1];
    }
}

kernel void radixScatter_8Bits(
    device const SortPair* src [[buffer(0)]],
    constant RadixParams& params [[buffer(2)]],
    device const uint* hist [[buffer(3)]],
    device const uint* blockOffset [[buffer(4)]],
    device SortPair* dst [[buffer(5)]],
    uint tid [[thread_position_in_grid]], // num blocks * 256
    uint lid [[thread_index_in_threadgroup]], // per threads in a block(tg)
    uint gid [[threadgroup_position_in_grid]] // per blocks(tg) in a grid
) {
    if (gid >= params.numBlocks) return;

    //결과 인덱스 = (현재 radix b보다 작은 값의 전체 개수 = hist[b])
    //            + (현재 radix b 값이 이전 threadgroup에서 발생한 개수 = blockOffset[gid*256 + b])
    //            + (현재 threadgroup에서 radix가 b인 값 중 내가 몇 번째인지?) --> 얘를 구해야됨.

    threadgroup uint localBucket[1024]; // max number of threads per a threadgroup

    uint idx = gid * params.blockSize + lid;
    if (idx < params.numElements) {
        localBucket[lid] = getBucket_8Bits(src[idx].key, params.shift);
    } else {
        localBucket[lid] = 0xFFFFFFFFu;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);

    if (idx < params.numElements) {
        uint bucket = localBucket[lid];
        uint localRank = 0;
        for (uint j = 0; j < lid; ++j) {
            if (localBucket[j] == bucket) {
                localRank++;
            }
        }
        uint globalPos =
            hist[bucket]
          + blockOffset[gid * 256u + bucket]
          + localRank;
        dst[globalPos] = src[idx];
    }
}

kernel void radixScatter_8Bits_PerBlock(
    device const SortPair* src [[buffer(0)]],
    constant RadixParams& params [[buffer(2)]],
    device const uint* hist [[buffer(3)]],
    device const uint* blockOffset [[buffer(4)]],
    device SortPair* dst [[buffer(5)]],
    uint tid [[thread_position_in_grid]], // num blocks
    uint lid [[thread_index_in_threadgroup]], // per threads in a block(tg)
    uint gid [[threadgroup_position_in_grid]] // per blocks(tg) in a grid
) {
    if(tid >= params.numBlocks) return;

    uint localCount[256];
    for(uint b = 0; b < 256; ++b) localCount[b] = 0u;

    uint blockStart = tid*params.blockSize;
    uint blockEnd = min(blockStart+params.blockSize, params.numElements);

    for(uint i = blockStart; i < blockEnd; ++i) {
        SortPair s = src[i];
        uint b = getBucket_8Bits(s.key, params.shift);
        uint dstIdx = hist[b] + blockOffset[tid*256u + b] + localCount[b];
        dst[dstIdx] = s;
        localCount[b]++;
    }
}
