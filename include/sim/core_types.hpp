#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

struct Backend {};
struct CPU : Backend {};
struct CUDA : Backend {};
struct METAL : Backend {};

#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <cstring>
#include <filesystem>
#include <random>
#include <unordered_map>
#include <vector>
#include <iostream>
/// Static Memory Pool ///

template <typename BE, typename PR>
struct MemoryBlock {};

template <typename PR>
struct MemoryBlock<CPU, PR> {
    PR* ptr;
    size_t size;
    MemoryBlock() : ptr(nullptr), size(0) {}
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
        memset(ret.ptr, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<CPU, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        std::fill(ret.ptr, ret.ptr+count, fill);
        return ret;
    }

    char* bytePtr() { return pool.data(); }
};

template <typename BE, typename PR>
struct VectorBase {};

#include <Metal/Metal.hpp>
struct MetalGlobalContext {
    static MTL::Device* getDevice() {
        // C++의 static 변수는 프로그램 실행 중 딱 한 번만 초기화됩니다!
        static MTL::Device* device = MTL::CreateSystemDefaultDevice();
        return device;
    }
    static MTL::CommandQueue* getCommandQueue() {
        static MTL::CommandQueue* commandQueue = getDevice()->newCommandQueue();
        return commandQueue;
    }
    inline static MTL::CommandBuffer* commandBuffer = nullptr;
    inline static MTL::ComputeCommandEncoder* computeCommandEncoder = nullptr;
    // Diagnostic: counts REAL CPU<->GPU syncs (a commitAndWait that actually
    // flushed an encoder, not the no-op early-return). The update() sync
    // refactor uses this to assert None/PerFrame do 0 in-loop syncs vs
    // InFrame's ~3/substep. Reset at a known point, read after.
    inline static uint64_t syncCount = 0;
    static MTL::ComputeCommandEncoder* getComputeCommandEncoder() {
        if (computeCommandEncoder) return computeCommandEncoder;
        commandBuffer = getCommandQueue()->commandBuffer();
        computeCommandEncoder = commandBuffer->computeCommandEncoder();
        return computeCommandEncoder;
    }
    template <typename BE, typename PR>
    static void setBuffer(const VectorBase<BE, PR>& vec, Index index) {
        getComputeCommandEncoder()->setBuffer(vec.pool, vec.offset, index);
    }
    template <typename PR>
    static void setBytes(const PR& data, Index index) {
        getComputeCommandEncoder()->setBytes(&data, sizeof(PR), index);
    }
    //static void dispatchThreads(MTL::ComputePipelineState* pso, Index numThreads) {
    //    MTL::Size gridSize = MTL::Size(numThreads, 1, 1);
    //    MTL::Size threadGroupSize = MTL::Size(std::min((Index)pso->maxTotalThreadsPerThreadgroup(), numThreads), 1, 1);
    //    getComputeCommandEncoder()->setComputePipelineState(pso);
    //    getComputeCommandEncoder()->dispatchThreads(gridSize, threadGroupSize);
    //}
    static void dispatchThreads(
        MTL::ComputePipelineState* pso,
        Index numThreads
    ) {
        Index tg = std::min<Index>(pso->maxTotalThreadsPerThreadgroup(), numThreads);
        dispatchThreads(pso, numThreads, tg);
    }

    static void dispatchThreads(
        MTL::ComputePipelineState* pso,
        Index numThreads,
        Index threadsPerThreadgroup
    ) {
        Index maxTG = (Index)pso->maxTotalThreadsPerThreadgroup();
        if (threadsPerThreadgroup == 0 || threadsPerThreadgroup > maxTG) {
            std::cerr << "[MetalGlobalContext] invalid threadsPerThreadgroup: "
                      << threadsPerThreadgroup
                      << ", max = " << maxTG << std::endl;
            return;
        }

        MTL::Size gridSize(numThreads, 1, 1);
        MTL::Size groupSize(threadsPerThreadgroup, 1, 1);

        getComputeCommandEncoder()->setComputePipelineState(pso);
        getComputeCommandEncoder()->dispatchThreads(gridSize, groupSize);
    }
    // Last command buffer flushed by commit() (no-wait) with no later encoder yet.
    // commitAndWait() waits on it so a frame that ended on a per-substep commit
    // still synchronizes at its boundary (else the CPU would read GPU-in-flight x).
    inline static MTL::CommandBuffer* lastCommitted = nullptr;
    static void commitAndWait() {
        if (computeCommandEncoder) {
            computeCommandEncoder->endEncoding();
            commandBuffer->commit();
            commandBuffer->waitUntilCompleted();
            commandBuffer = nullptr;
            computeCommandEncoder = nullptr;
            lastCommitted = nullptr;
            ++syncCount;
            return;
        }
        // No pending encoder. If the last work was flushed by commit() (no-wait),
        // wait on it now — otherwise nothing to do (D-030: a no-op path is needed
        // when prior dispatches already committed, e.g. a pure-CPU branch between
        // two GPU-dispatching paths).
        if (lastCommitted) {
            lastCommitted->waitUntilCompleted();
            lastCommitted = nullptr;
            ++syncCount;
        }
    }
    // Flush the current command buffer to the GPU WITHOUT blocking the CPU. The
    // next dispatch opens a fresh command buffer; same-queue buffers run in order
    // so correctness is preserved. Lets the GPU start a substep while the CPU
    // encodes the next (pipelining) instead of one giant per-frame buffer. NOT a
    // sync (no waitUntilCompleted, no syncCount++); the frame-boundary
    // commitAndWait waits on `lastCommitted`.
    static void commit() {
        if (!computeCommandEncoder) return;
        computeCommandEncoder->endEncoding();
        commandBuffer->commit();
        lastCommitted = commandBuffer;
        commandBuffer = nullptr;
        computeCommandEncoder = nullptr;
    }

};
struct MetalKernelContext {
    static MTL::Library* getLibrary() {
        static MTL::Library* library = nullptr;

        if (!library) {
            NS::Error* error = nullptr;

            // metallib is a build artifact next to the binary — resolve via
            // the executable dir, not the cwd (ysim runs from any directory).
            std::string libPath = ysim_paths::runtimeFile("default.metallib");
            auto path = NS::String::string(libPath.c_str(), NS::UTF8StringEncoding);

            library = MetalGlobalContext::getDevice()->newLibrary(path, &error);

            if (!library) {
                std::cout << "[Metal Error] Failed to load " << path->utf8String() << "!\n";
                if (error) {
                    std::cout << error->localizedDescription()->utf8String() << std::endl;
                }
                exit(1);
            }

            path->release();
        }

        return library;
    }
    static MTL::Function* getFunction(const char* name) {
        static std::unordered_map<std::string, MTL::Function*> cache;

        auto it = cache.find(name);
        if (it != cache.end()) return it->second;

        auto nsName = NS::String::string(name, NS::UTF8StringEncoding);
        auto func = getLibrary()->newFunction(nsName);

        if (!func) {
            std::cout << "[Metal Error] Failed to load function: " << name << "\n";
            exit(1);
        }

        cache[name] = func;
        nsName->release();

        return func;
    }
    static MTL::ComputePipelineState* getPSO(const char* name) {
        static std::unordered_map<std::string, MTL::ComputePipelineState*> cache;

        auto it = cache.find(name);
        if (it != cache.end()) return it->second;

        NS::Error* error = nullptr;
        auto func = getFunction(name);

        auto pso = MetalGlobalContext::getDevice()->newComputePipelineState(func, &error);

        if (!pso) {
            std::cout << "[Metal Error] PSO creation failed: " << name << "\n";
            exit(1);
        }

        cache[name] = pso;
        return pso;
    }
};





template <typename PR>
struct MemoryBlock<METAL, PR> {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    size_t size;
    MemoryBlock() : pool(nullptr), offset(0), ptr(nullptr), size(0) {}
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
    ~ByteMemoryPool() { if(pool) pool->release(); }
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
        size_t alignedBytes = (bytes + 255) & ~size_t(255);

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
        memset(ret.ptr, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        std::fill(ret.ptr, ret.ptr+count, fill);
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
        marker += (count*sizeof(PR) + 255) & ~255;
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> zeros(size_t count) { return alloc<PR>(count); }
    template <typename PR>
    MemoryBlock<METAL, PR> allocFill(size_t count, PR fill) { return alloc<PR>(count); }
};
// TODO: Dynamic Memory Pool
template <typename BE>
struct DynamicByteMemoryPool {
    std::vector<ByteMemoryPool<BE>> poolList;
    // Index of the sub-pool alloc() currently bump-allocates from. Advances
    // when a sub-pool fills; rewound to 0 by resetMarkers() so a re-init
    // REPLAYS the original allocation sequence into the SAME sub-pools at
    // the SAME offsets. Without this, alloc() always used poolList.back(),
    // so after a reset every allocation piled onto the last/new sub-pool —
    // a different memory layout than the first run. Any pool pointer not
    // re-pointed on re-init then aliased a live buffer → corruption that
    // surfaced at the first collision (largest, layout-sensitive buffers).
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

        // A zero-count request never yields a non-null ptr; return the
        // empty block directly (the walk below would loop forever on it).
        if (count == 0)
            return poolList[std::min(cursor, poolList.size() - 1)]
                       .template alloc<PR>(0);

        // Walk sub-pools from the cursor. On a fresh run this appends new
        // pools exactly as before; after resetMarkers() (cursor=0, all
        // markers=0) it replays the SAME sequence into the SAME pools, so
        // every buffer lands at the SAME address as the first run.
        for (;;) {
            if (cursor >= poolList.size())
                poolList.emplace_back(std::max<size_t>(need, minBoundSize));
            auto ret = poolList[cursor].template alloc<PR>(count);
            if (ret.ptr) return ret;
            // Current sub-pool can't fit this request — move to the next.
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
        for(const auto& pool : poolList) capacity += pool.capacity;
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

    // D-041 (2026-05-14): rewind all sub-pool markers to 0 so the next
    // bump allocations reuse the existing backing buffers. The pool list
    // is NOT freed — Bullet/Metal buffers stay live, just the marker
    // resets. Safe IFF all VectorBase<*> consumers refresh their pointers
    // via Scene::pack + BroadPhase::build etc. on the next initialize
    // (current invariant). Before this, every Simulator::initialize
    // bump-allocated fresh space and never reclaimed the old — adding
    // a cube and re-initializing N times grew the pool roughly N×.
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

    // TODO: Pack complete
    ByteMemoryPool<BE> pack() {
        return pool.pack();
    }
};

template <typename BE>
struct GlobalAutoAllocator {
    inline static DynamicMemoryAllocator<BE> globalPool;
    inline static bool globalInitialized = false;

    static void globalInitialize(size_t N) {
        if(globalInitialized) return;
        globalPool = DynamicMemoryAllocator<BE>(N);
        globalInitialized = true;
    }

    // D-041: rewind the global pool so the next set of Scene::pack +
    // BroadPhase::build allocations reuse the existing backing buffers
    // instead of leaking forward. Call at the TOP of Simulator::initialize.
    static void reset() { globalPool.pool.resetMarkers(); }

    // Hard reset: FREE every sub-pool (releasing the Metal buffers) and force
    // the next globalInitialize to rebuild from scratch. reset() only rewinds
    // markers, so when the allocation SEQUENCE changes between runs (e.g. a
    // bench sweeping single-root vs clustered, which allocate different buffer
    // sets) the pool fragments and appends fresh sub-pools every iteration —
    // unbounded growth that eventually exceeds the GPU working set and faults.
    // Bench harnesses call this between scene variants to cap the footprint at
    // a single variant's true need.
    static void hardReset() { globalPool.pool.clear(); globalInitialized = false; }

    template <typename PR>
    static MemoryBlock<BE, PR> alloc(size_t count) { return globalPool.template alloc<PR>(count); }
    template <typename PR>
    static MemoryBlock<BE, PR> zeros(size_t count) { return globalPool.template zeros<PR>(count); }
    template <typename PR>
    static MemoryBlock<BE, PR> allocFill(size_t count, PR fill) { return globalPool.template allocFill<PR>(count, fill); }
};


//template <typename T>
//struct ComputeMemoryPool : MemoryPool<T> {
//    ComputeMemoryPool(size_t N) : MemoryPool<T>(N) { }
//    void reset() { this->marker = 0; }
//};
#include "tinym.hpp"
#include "Quat.hpp"
#include "RigidPhysicsTypes.hpp"
#include "NullRigidPhysicsBackend.hpp"
#include "EulerRigidPhysicsBackend.hpp"
#include "BulletRigidPhysicsBackend.hpp"
#include "PreviewState.hpp"

using Precision = float;


//ComputeMemoryPool<Precision> computePool(100000000);


/// TODO: Math ///
#include <Eigen/Dense>


template <typename PR>
struct VectorBase<CPU, PR> {
    //Eigen::Map<Eigen::VectorX<PR>> data;
    PR* ptr;
    size_t size;
    VectorBase() : /*data(nullptr, 0)*/ptr(nullptr), size(0) {}
    VectorBase(size_t size) {
        auto block = GlobalAutoAllocator<CPU>::template alloc<PR>(size);
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(size_t size, PR fill) {
        MemoryBlock<CPU, PR> block;
        if(fill == 0) block = GlobalAutoAllocator<CPU>::template zeros<PR>(size);
        else block = GlobalAutoAllocator<CPU>::template allocFill<PR>(size, fill);
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(const MemoryBlock<CPU, PR>& block) : /*data(ptr, size)*/ptr(block.ptr), size(block.size) {}
    //void setZero() { data.setZero(); }
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
    PR& operator[](Index index) { return ptr[index]; }
};

template <typename PR>
struct VectorBase<METAL, PR> {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    size_t size;
    VectorBase() : pool(nullptr), offset(0), ptr(nullptr), size(0) {}
    VectorBase(size_t size) {
        auto block = GlobalAutoAllocator<METAL>::template alloc<PR>(size);
        this->pool = block.pool;
        this->offset = block.offset;
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(size_t size, PR fill) {
        MemoryBlock<METAL, PR> block;
        if(fill == 0) block = GlobalAutoAllocator<METAL>::template zeros<PR>(size);
        else block = GlobalAutoAllocator<METAL>::template allocFill<PR>(size, fill);
        this->pool = block.pool;
        this->offset = block.offset;
        this->ptr = block.ptr;
        this->size = block.size;
    }
    VectorBase(const MemoryBlock<METAL, PR>& block) : pool(block.pool), offset(block.offset), ptr(block.ptr), size(block.size) {}
    VectorBase(const VectorBase<METAL, PR>& v, size_t start, size_t size) {
        this->pool = v.pool;
        this->offset = v.offset + start*sizeof(PR);
        this->ptr = v.ptr + start;
        this->size = size;
    }
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
    PR& operator[](Index index) { return ptr[index]; }
};



template <typename BE, typename PR>
struct Matrix {};

template <typename PR>
struct Matrix<CPU, PR> {
    //Eigen::Map<Eigen::MatrixX<PR>> data;
    PR* ptr;
    size_t rows, cols;
    Matrix(PR* ptr, size_t rows, size_t cols) : /*data(ptr, rows, cols)*/ptr(ptr), rows(rows), cols(cols) {}
    auto map() { return Eigen::Map<Eigen::MatrixX<PR>>(ptr, rows, cols); }
};

#include <Eigen/Sparse>
template <typename BE, typename PR>
struct SparseMatrix {};

template <typename PR>
struct SparseMatrix<CPU, PR> {
    Eigen::SparseMatrix<PR> data;
    size_t rows, cols;
    SparseMatrix(std::vector<Eigen::Triplet<PR>>& triplets, size_t rows, size_t cols) : rows(rows), cols(cols) {
        data = Eigen::SparseMatrix<PR>(rows, cols);
        data.setFromTriplets(triplets.begin(), triplets.end());
    }
    auto& map() { return data; }
};








// TODO: ECS


// TODO: Solver?


// MeshGL<CPU> moved to include/MeshGL.hpp; renderer-side ownership in
// MeshRenderState (include/MeshRenderState.hpp). See D-011 / CM-002–004.

