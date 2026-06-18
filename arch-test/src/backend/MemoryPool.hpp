#pragma once
#include "Backend.hpp"
#include "MetalContext.hpp"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <vector>

template <typename BE, typename PR>
struct MemoryBlock {};

template <typename PR>
struct MemoryBlock<CPU, PR> {
    PR* ptr;
    size_t size;
    MemoryBlock() : ptr(nullptr), size(0) {}
};

template <typename PR>
struct MemoryBlock<METAL, PR> {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    size_t size;
    MemoryBlock() : pool(nullptr), offset(0), ptr(nullptr), size(0) {}
};

template <typename BE>
struct ByteMemoryPool {};

template <>
struct ByteMemoryPool<CPU> {
    std::vector<char> pool;
    size_t marker = 0;
    size_t capacity = 0;
    ByteMemoryPool() {}
    ByteMemoryPool(size_t N) : pool(N), capacity(pool.size()) {}
    template <typename PR>
    MemoryBlock<CPU, PR> alloc(size_t count) {
        MemoryBlock<CPU, PR> ret;
        if (count == 0) return ret;
        size_t align = alignof(PR);
        size_t alignedMarker = marker + ((align - (marker % align)) % align);
        size_t bytes = sizeof(PR) * count;
        if (alignedMarker + bytes > capacity) {
            std::cout << "[Pool] Tried to allocate more than tha capacity" << std::endl;
            return ret;
        }
        ret.ptr = reinterpret_cast<PR*>(pool.data() + alignedMarker);
        ret.size = count;
        marker = alignedMarker + bytes;
        return ret;
    }
    template <typename PR>
    MemoryBlock<CPU, PR> zeros(size_t count) {
        auto ret = alloc<PR>(count);
        memset(ret.ptr, 0, count * sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<CPU, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        std::fill(ret.ptr, ret.ptr + count, fill);
        return ret;
    }
    char* bytePtr() { return pool.data(); }
};

template <>
struct ByteMemoryPool<METAL> {
    MTL::Device* device;
    MTL::Buffer* pool;
    size_t marker = 0;
    size_t capacity = 0;
    ByteMemoryPool() : device(nullptr), pool(nullptr), marker(0), capacity(0) {}
    ByteMemoryPool(size_t N) : device(MetalGlobalContext::getDevice()), capacity(N) {
        pool = device->newBuffer(N, MTL::ResourceStorageModeShared);
    }
    ByteMemoryPool(ByteMemoryPool&& other) noexcept
        : device(other.device), pool(other.pool), marker(other.marker), capacity(other.capacity) {
        other.pool = nullptr;
    }
    ~ByteMemoryPool() { if (pool) pool->release(); }
    ByteMemoryPool& operator=(ByteMemoryPool&& other) noexcept {
        if (this != &other) {
            if (pool) pool->release();
            device = other.device;
            pool = other.pool;
            marker = other.marker;
            capacity = other.capacity;
            other.pool = nullptr;
        }
        return *this;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> alloc(size_t count) {
        MemoryBlock<METAL, PR> ret;
        if (count == 0) return ret;
        size_t bytes = sizeof(PR) * count;
        size_t alignedBytes = (bytes + 255) & ~size_t(255); // Metal 256B binding align
        if (marker + alignedBytes > capacity) {
            std::cout << "[Pool] Tried to allocate more than tha capacity" << std::endl;
            return ret;
        }
        ret.pool = pool;
        ret.offset = marker;
        ret.ptr = reinterpret_cast<PR*>(reinterpret_cast<char*>(pool->contents()) + marker);
        ret.size = count;
        marker += alignedBytes;
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> zeros(size_t count) {
        auto ret = alloc<PR>(count);
        memset(ret.ptr, 0, count * sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        std::fill(ret.ptr, ret.ptr + count, fill);
        return ret;
    }
    char* bytePtr() { return reinterpret_cast<char*>(pool->contents()); }
};

template <typename BE>
struct FakeMemoryPool {};
template <>
struct FakeMemoryPool<CPU> {
    size_t marker = 0;
    template <typename PR>
    MemoryBlock<CPU, PR> alloc(size_t count) {
        size_t align = alignof(PR);
        marker += (align - marker % align) % align;
        marker += count * sizeof(PR);
        return MemoryBlock<CPU, PR>();
    }
    template <typename PR>
    MemoryBlock<CPU, PR> zeros(size_t count) { return alloc<PR>(count); }
    template <typename PR>
    MemoryBlock<CPU, PR> allocFill(size_t count, PR fill) { return alloc<PR>(count); }
};
template <>
struct FakeMemoryPool<METAL> {
    size_t marker = 0;
    template <typename PR>
    MemoryBlock<METAL, PR> alloc(size_t count) {
        MemoryBlock<METAL, PR> ret;
        marker += (count * sizeof(PR) + 255) & ~255;
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> zeros(size_t count) { return alloc<PR>(count); }
    template <typename PR>
    MemoryBlock<METAL, PR> allocFill(size_t count, PR fill) { return alloc<PR>(count); }
};

template <typename BE>
struct DynamicByteMemoryPool {
    std::vector<ByteMemoryPool<BE>> poolList;
    // cursor rewinds to 0 on resetMarkers() so re-init REPLAYS the same
    // alloc sequence into the same sub-pools at the same offsets — pointer
    // stability invariant (DECISIONS C1 / D-041).
    size_t cursor = 0;

    DynamicByteMemoryPool() {}
    DynamicByteMemoryPool(size_t N) { poolList.emplace_back(N); }

    template <typename PR>
    size_t requiredBytes(size_t count) {
        if constexpr (std::is_same_v<BE, CPU>) {
            return sizeof(PR) * count + alignof(PR);
        } else {
            size_t bytes = sizeof(PR) * count;
            return (bytes + 255) & ~size_t(255);
        }
    }

    template <typename PR>
    auto alloc(size_t count) {
        size_t need = requiredBytes<PR>(count);
        size_t minBoundSize = 1 << 20;
        if (poolList.empty()) {
            poolList.emplace_back(std::max<size_t>(need, minBoundSize));
            cursor = 0;
        }
        if (count == 0)
            return poolList[std::min(cursor, poolList.size() - 1)].template alloc<PR>(0);
        for (;;) {
            if (cursor >= poolList.size())
                poolList.emplace_back(std::max<size_t>(need, minBoundSize));
            auto ret = poolList[cursor].template alloc<PR>(count);
            if (ret.ptr) return ret;
            ++cursor;
        }
    }
    template <typename PR>
    auto zeros(size_t count) {
        auto ret = alloc<PR>(count);
        if (ret.ptr) std::memset(ret.ptr, 0, sizeof(PR) * count);
        return ret;
    }
    template <typename PR>
    auto allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        if (ret.ptr) std::fill(ret.ptr, ret.ptr + count, fill);
        return ret;
    }

    size_t totalUsedBytes() const {
        size_t size = 0;
        for (const auto& pool : poolList) size += pool.marker;
        return size;
    }
    size_t totalCapacity() {
        size_t capacity = 0;
        for (const auto& pool : poolList) capacity += pool.capacity;
        return capacity;
    }
    ByteMemoryPool<BE> pack() {
        size_t totalBytes = totalUsedBytes();
        ByteMemoryPool<BE> ret(totalBytes);
        char* dst = ret.bytePtr();
        size_t dstOffset = 0;
        for (auto& srcPool : poolList) {
            if (srcPool.marker == 0) continue;
            char* src = srcPool.bytePtr();
            std::memcpy(dst + dstOffset, src, srcPool.marker);
            dstOffset += srcPool.marker;
        }
        ret.marker = totalBytes;
        return ret;
    }
    void clear() { poolList.clear(); }
    void resetMarkers() {
        for (auto& p : poolList) p.marker = 0;
        cursor = 0;
    }
};

template <typename BE>
struct DynamicMemoryAllocator {
    DynamicByteMemoryPool<BE> pool;
    DynamicMemoryAllocator() {}
    DynamicMemoryAllocator(size_t N) : pool(N) {}
    template <typename PR>
    auto alloc(size_t count) { return pool.template alloc<PR>(count); }
    template <typename PR>
    auto zeros(size_t count) { return pool.template zeros<PR>(count); }
    template <typename PR>
    auto allocFill(size_t count, PR fill) { return pool.template allocFill<PR>(count, fill); }
    ByteMemoryPool<BE> pack() { return pool.pack(); }
};

template <typename BE>
struct GlobalAutoAllocator {
    inline static DynamicMemoryAllocator<BE> globalPool;
    inline static bool globalInitialized = false;

    // Swappable routing target (O2): each Simulator setActive(&ownPool) so its
    // allocations land in its own pool; defaults to globalPool for back-compat.
    inline static DynamicMemoryAllocator<BE>* active = &globalPool;

    static void globalInitialize(size_t N) {
        if (globalInitialized) return;
        globalPool = DynamicMemoryAllocator<BE>(N);
        globalInitialized = true;
        active = &globalPool;
    }

    static void setActive(DynamicMemoryAllocator<BE>* a) { active = a ? a : &globalPool; }
    static DynamicMemoryAllocator<BE>* getActive() { return active; }

    static void reset() { active->pool.resetMarkers(); }   // D-041 replay rewind of ACTIVE pool
    template <typename PR>
    static MemoryBlock<BE, PR> alloc(size_t count) { return active->template alloc<PR>(count); }
    template <typename PR>
    static MemoryBlock<BE, PR> zeros(size_t count) { return active->template zeros<PR>(count); }
    template <typename PR>
    static MemoryBlock<BE, PR> allocFill(size_t count, PR fill) { return active->template allocFill<PR>(count, fill); }
};
