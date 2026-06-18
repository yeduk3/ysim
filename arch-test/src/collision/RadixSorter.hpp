#pragma once
#include "backend/MetalContext.hpp"
#include "backend/VectorBase.hpp"

#include <utility>

// 4-pass 8-bit LSD radix sort over MortonNode by .code (aliases SortPair.key).
// Ported verbatim from src/main.cpp 3470-3535. Element-templated; Karras12BVH
// instantiates RadixSorter<METAL, MortonNode>. Does NOT commitAndWait — the
// caller batches the single commit (matches OLD radixSortGPU 5685-5687).
template <typename BE, typename Element>
struct RadixSorter {};

template <typename Element>
struct RadixSorter<METAL, Element> {
    uint32_t BITS_PER_PASS = 8;
    uint32_t NUM_BUCKETS   = 1u << 8;   // 256
    uint32_t BLOCK_SIZE    = 1024;      // range 256-1024
    VectorBase<METAL, Element> dst;
    VectorBase<METAL, Index>   blockHist;   // numBlocks * 256
    VectorBase<METAL, Index>   hist;        // 256
    VectorBase<METAL, Index>   blockOffset; // numBlocks * 256
    MTL::ComputePipelineState *radixCountBlockPSO, *radixScanOffsetPSO, *radixScatterPSO;

    // matches radixsort.metal RadixParams (10-15)
    struct RadixParams { uint32_t numBlocks, numElements, blockSize, shift; };

    uint32_t getNumBlocks(const VectorBase<METAL, Element>& src) const {
        return (uint32_t)((src.size + BLOCK_SIZE - 1) / BLOCK_SIZE);
    }

    RadixSorter() {
        radixCountBlockPSO = MetalKernelContext::getPSO("radixCountBlock_8Bits");
        radixScanOffsetPSO = MetalKernelContext::getPSO("radixScanOffset_8Bits");
        radixScatterPSO    = MetalKernelContext::getPSO("radixScatter_8Bits");
    }

    void memoryAllocation(const VectorBase<METAL, Element>& src) {
        uint32_t nb = getNumBlocks(src);
        if (!dst.ptr || dst.size < src.size)                       dst = VectorBase<METAL, Element>(src.size);
        if (!hist.ptr || hist.size < NUM_BUCKETS)                  hist = VectorBase<METAL, Index>(NUM_BUCKETS);
        if (!blockHist.ptr || blockHist.size < NUM_BUCKETS * nb)   blockHist = VectorBase<METAL, Index>(NUM_BUCKETS * nb);
        if (!blockOffset.ptr || blockOffset.size < NUM_BUCKETS * nb) blockOffset = VectorBase<METAL, Index>(NUM_BUCKETS * nb);
    }

    void sort(VectorBase<METAL, Element>& src) {
        memoryAllocation(src);
        RadixParams p{ getNumBlocks(src), (uint32_t)src.size, BLOCK_SIZE, 0 };
        for (uint32_t r = 0; r < 32; r += BITS_PER_PASS) {
            p.shift = r;
            MetalGlobalContext::setBuffer(src, 0);
            MetalGlobalContext::setBuffer(blockHist, 1);
            MetalGlobalContext::setBytes(p, 2);
            MetalGlobalContext::setBuffer(hist, 3);
            MetalGlobalContext::setBuffer(blockOffset, 4);
            MetalGlobalContext::setBuffer(dst, 5);
            MetalGlobalContext::dispatchThreads(radixCountBlockPSO, p.numBlocks * BLOCK_SIZE, BLOCK_SIZE);
            MetalGlobalContext::dispatchThreads(radixScanOffsetPSO, NUM_BUCKETS, NUM_BUCKETS);
            MetalGlobalContext::dispatchThreads(radixScatterPSO,    p.numBlocks * BLOCK_SIZE, BLOCK_SIZE);
            std::swap(src, dst);
        }
    }
};
