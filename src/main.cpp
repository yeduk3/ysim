#include "Foundation/NSString.hpp"
#include "Metal/MTLBuffer.hpp"
#include "Metal/MTLComputeCommandEncoder.hpp"
#include "Metal/MTLComputePipeline.hpp"
#include "YGLWindow.hpp"
#include "camera.hpp"
#include "FrameProfiler.hpp"
#include "MeshInspectorWindow.hpp"
#include "ProfilerWindow.hpp"
#include "program.hpp"
#include "objreader.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <cstddef>
#include <cmath>
#include <iostream>
#include <iterator>
#include <ratio>
#include <string>
#include <type_traits>
#include <typeindex>
#include <set>

YGLWindow* yglwindow;

#include <cstdint>

using Index = uint32_t;

//#include "MemoryPool.hpp"

struct Backend {};
struct CPU : Backend {};
struct CUDA : Backend {};
struct METAL : Backend {};

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
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
        if (ret.ptr) memset(ret.ptr, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<CPU, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        if (ret.ptr) std::fill(ret.ptr, ret.ptr+count, fill);
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
    static void commitAndWait() {
        computeCommandEncoder->endEncoding();
        commandBuffer->commit();
        commandBuffer->waitUntilCompleted();

        commandBuffer = nullptr;
        computeCommandEncoder = nullptr;
    }

};
struct MetalKernelContext {
    static MTL::Library* getLibrary() {
        static MTL::Library* library = nullptr;

        if (!library) {
            NS::Error* error = nullptr;

            auto path = NS::String::string("default.metallib", NS::UTF8StringEncoding);

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
        auto label = NS::String::string(name, NS::UTF8StringEncoding);

        auto desc = MTL::ComputePipelineDescriptor::alloc()->init();
        desc->setComputeFunction(func);
        desc->setLabel(label);

        auto pso = MetalGlobalContext::getDevice()->newComputePipelineState(desc, 0, nullptr, &error);
        desc->release();
        label->release();

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
        if (ret.ptr) memset(ret.ptr, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    MemoryBlock<METAL, PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        if (ret.ptr) std::fill(ret.ptr, ret.ptr+count, fill);
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

        if (poolList.empty())
            poolList.emplace_back(std::max<size_t>(need, minBoundSize));

        auto ret = poolList.back().template alloc<PR>(count);
        if (!ret.ptr) {
            //std::cout << "[DynamicByteMemoryPool alloc] allocate new one" << std::endl;
            poolList.emplace_back(std::max<size_t>(need, minBoundSize));
            ret = poolList.back().template alloc<PR>(count);
        }
        return ret;
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
        if (totalBytes == 0) {
            return ByteMemoryPool<BE>();
        }
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
    void clear() {
        poolList.clear();
        poolList.shrink_to_fit();
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

    void clear() { pool.clear(); }

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

    static void reset() {
        if (!globalInitialized) return;
        globalPool.clear();
    }

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
        if (!v.ptr || size == 0) {
            this->pool = nullptr;
            this->offset = 0;
            this->ptr = nullptr;
            this->size = 0;
            return;
        }
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


template<typename BE>
struct MeshGL {};

template <>
struct MeshGL<CPU> {
    GLuint vao;
    GLuint vertexBuffer, normalBuffer, facetBuffer, texCoordBuffer;

    size_t vertexNum;
    size_t facetNum;
    float* vertexPtr;
    unsigned int* facetPtr;
    float* normalPtr;

    MeshGL()
        : vao(0),
          vertexBuffer(0),
          normalBuffer(0),
          facetBuffer(0),
          texCoordBuffer(0),
          vertexNum(0),
          facetNum(0),
          vertexPtr(nullptr),
          facetPtr(nullptr),
          normalPtr(nullptr) {}

    MeshGL(const MeshGL&) = delete;
    MeshGL& operator=(const MeshGL&) = delete;

    MeshGL(MeshGL&& other) noexcept
        : MeshGL() {
        *this = std::move(other);
    }

    MeshGL& operator=(MeshGL&& other) noexcept {
        if (this == &other) return *this;

        release();

        vao = other.vao;
        vertexBuffer = other.vertexBuffer;
        normalBuffer = other.normalBuffer;
        facetBuffer = other.facetBuffer;
        texCoordBuffer = other.texCoordBuffer;
        vertexNum = other.vertexNum;
        facetNum = other.facetNum;
        vertexPtr = other.vertexPtr;
        facetPtr = other.facetPtr;
        normalPtr = other.normalPtr;

        other.vao = 0;
        other.vertexBuffer = 0;
        other.normalBuffer = 0;
        other.facetBuffer = 0;
        other.texCoordBuffer = 0;
        other.vertexNum = 0;
        other.facetNum = 0;
        other.vertexPtr = nullptr;
        other.facetPtr = nullptr;
        other.normalPtr = nullptr;

        return *this;
    }

    ~MeshGL() { release(); }

    void release() {
        if (texCoordBuffer) glDeleteBuffers(1, &texCoordBuffer);
        if (normalBuffer) glDeleteBuffers(1, &normalBuffer);
        if (vertexBuffer) glDeleteBuffers(1, &vertexBuffer);
        if (facetBuffer) glDeleteBuffers(1, &facetBuffer);
        if (vao) glDeleteVertexArrays(1, &vao);

        vao = 0;
        vertexBuffer = 0;
        normalBuffer = 0;
        facetBuffer = 0;
        texCoordBuffer = 0;
        vertexNum = 0;
        facetNum = 0;
        vertexPtr = nullptr;
        facetPtr = nullptr;
        normalPtr = nullptr;
    }

    MeshGL(size_t vertexNum, float* vertexPtr, size_t facetNum, unsigned int* facetPtr, float* normalPtr, float* texCoordPtr=nullptr) 
        : vertexNum(vertexNum), vertexPtr(vertexPtr), facetNum(facetNum), facetPtr(facetPtr), normalPtr(normalPtr) {
        std::cout << "[MeshGL] Try to create..." << std::endl;
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     vertexNum * sizeof(float) * 3,
                     vertexPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

        glGenBuffers(1, &facetBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     facetNum * sizeof(unsigned int) * 3,
                     facetPtr,
                     GL_STATIC_DRAW);

        computeNormal();
        
        glGenBuffers(1, &normalBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     vertexNum * sizeof(float) * 3,
                     normalPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

        if(texCoordPtr) {
            glGenBuffers(1, &texCoordBuffer);
            glBindBuffer(GL_ARRAY_BUFFER, texCoordBuffer);
            glBufferData(GL_ARRAY_BUFFER,
                         vertexNum * sizeof(float) * 2,
                         texCoordPtr,
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
        }
        std::cout << "[MeshGL] Created!" << std::endl;
    }

    void computeNormal() {
        //std::cout << "[MeshGL] Try to compute normal..." << std::endl;
        Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic>> V(vertexPtr, 3, vertexNum);
        Eigen::Map<Eigen::Matrix<unsigned int, 3, Eigen::Dynamic>> F(facetPtr, 3, facetNum); 
        Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> N(normalPtr, 3, vertexNum);

        N.setZero();

        for (size_t i = 0; i < facetNum; ++i) {
            unsigned int i0 = F(0, i);
            unsigned int i1 = F(1, i);
            unsigned int i2 = F(2, i);

            auto v0 = V.col(i0);
            auto v1 = V.col(i1);
            auto v2 = V.col(i2);

            Eigen::Vector3f cross = (v1 - v0).cross(v2 - v0);

            N.col(i0) += cross;
            N.col(i1) += cross;
            N.col(i2) += cross;
        }

        for (size_t i = 0; i < vertexNum; ++i) {
            float len = N.col(i).norm();
            if (len > 1e-7f) {
                N.col(i) /= len;
            }
        }
        //std::cout << "[MeshGL] Normal Computed" << std::endl;
    }

    void updateBuffer(float* newVertexPtr) {
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, newVertexPtr);

        computeNormal();

        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, normalPtr);
    }

    void draw(Program& shader, const tinym::vec3& baseColor) {
        shader.setUniform("diffuseColor", baseColor);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);

        glDrawElements(GL_TRIANGLES, facetNum*3, GL_UNSIGNED_INT, 0);
    }
};

template <typename BE>
struct DebugLineGL {};
template <>
struct DebugLineGL<CPU> {
    GLuint vao = 0;
    GLuint vertexBuffer = 0;

    float* vertexPtr = nullptr;
    size_t vertexNum = 0;
    size_t capacityVertices = 0;

    DebugLineGL() = default;

    DebugLineGL(const DebugLineGL&) = delete;
    DebugLineGL& operator=(const DebugLineGL&) = delete;

    DebugLineGL(DebugLineGL&& other) noexcept
        : DebugLineGL() {
        *this = std::move(other);
    }

    DebugLineGL& operator=(DebugLineGL&& other) noexcept {
        if (this == &other) return *this;

        release();

        vao = other.vao;
        vertexBuffer = other.vertexBuffer;
        vertexPtr = other.vertexPtr;
        vertexNum = other.vertexNum;
        capacityVertices = other.capacityVertices;

        other.vao = 0;
        other.vertexBuffer = 0;
        other.vertexPtr = nullptr;
        other.vertexNum = 0;
        other.capacityVertices = 0;

        return *this;
    }

    DebugLineGL(size_t vertexNum, float* vertexPtr)
        : vertexPtr(vertexPtr), vertexNum(vertexNum), capacityVertices(vertexNum) {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     capacityVertices * sizeof(float) * 3,
                     vertexPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    }

    ~DebugLineGL() { release(); }

    void release() {
        if (vertexBuffer) glDeleteBuffers(1, &vertexBuffer);
        if (vao) glDeleteVertexArrays(1, &vao);

        vao = 0;
        vertexBuffer = 0;
        vertexPtr = nullptr;
        vertexNum = 0;
        capacityVertices = 0;
    }

    void clear() {
        vertexNum = 0;
    }

    void updateBuffer(float* newVertexPtr, size_t newVertexNum) {
        vertexPtr = newVertexPtr;
        vertexNum = newVertexNum;

        if (!vao || !vertexBuffer || vertexNum == 0) return;

        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);

        if (vertexNum > capacityVertices) {
            capacityVertices = vertexNum;
            glBufferData(GL_ARRAY_BUFFER,
                         capacityVertices * sizeof(float) * 3,
                         vertexPtr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBufferSubData(GL_ARRAY_BUFFER,
                            0,
                            vertexNum * sizeof(float) * 3,
                            vertexPtr);
        }
    }

    void draw() {
        if (!vao || vertexNum == 0) return;
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, vertexNum);
    }
};

template <typename BE>
struct DebugPointGL {};
template <>
struct DebugPointGL<CPU> {
    GLuint vao = 0;
    GLuint vertexBuffer = 0;

    float* vertexPtr = nullptr;
    size_t vertexNum = 0;

    DebugPointGL() = default;

    DebugPointGL(const DebugPointGL&) = delete;
    DebugPointGL& operator=(const DebugPointGL&) = delete;

    DebugPointGL(DebugPointGL&& other) noexcept
        : DebugPointGL() {
        *this = std::move(other);
    }

    DebugPointGL& operator=(DebugPointGL&& other) noexcept {
        if (this == &other) return *this;

        release();

        vao = other.vao;
        vertexBuffer = other.vertexBuffer;
        vertexPtr = other.vertexPtr;
        vertexNum = other.vertexNum;

        other.vao = 0;
        other.vertexBuffer = 0;
        other.vertexPtr = nullptr;
        other.vertexNum = 0;

        return *this;
    }

    DebugPointGL(size_t vertexNum, float* vertexPtr) : vertexNum(vertexNum), vertexPtr(vertexPtr) {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     vertexNum * sizeof(float) * 3,
                     vertexPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    }

    ~DebugPointGL() { release(); }

    void release() {
        if (vertexBuffer) glDeleteBuffers(1, &vertexBuffer);
        if (vao) glDeleteVertexArrays(1, &vao);

        vao = 0;
        vertexBuffer = 0;
        vertexPtr = nullptr;
        vertexNum = 0;
    }

    void clear() {
        vertexNum = 0;
    }

    void updateBuffer(float* newVertexPtr) {
        vertexPtr = newVertexPtr;
        if (!vao || !vertexBuffer || vertexNum == 0) return;
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, newVertexPtr);
    }

    void draw() {
        if (!vao || vertexNum == 0) return;
        glBindVertexArray(vao);
        
        glDrawArrays(GL_POINTS, 0, vertexNum);
    }
};


template <typename BE, typename PR>
struct Scene;

// General Mesh Deinfe!

template <typename BE, typename PR>
struct GeneralMesh;


enum struct BehaviorType : Index {
    TriangularCloth = 0,
    FastGridCloth,
    Elastic,
    Rigid,
    Float,
    Fluid,
    Generator,
};

enum struct InitializerType : Index {
    MeshGridSpring,
    MeshFile,

};



template <typename BE, typename PR>
struct ExternalForces {
    VectorBase<BE, PR> externalForces;
};

template <typename BE, typename PR>
struct MeshState {
    using Vector = VectorBase<BE, PR>;
    Vector x, v, f, m, n;
    template <typename InitializerParams>
    void memoryAllocation(InitializerParams& params) {
        Index numData = params.numPoints*3;
        if(!x.ptr) x = Vector(numData);
        if(!v.ptr) v = Vector(numData, 0);
        if(!f.ptr) f = Vector(numData, 0);
        if(!m.ptr) m = Vector(numData, params.mass);
        if(!n.ptr) n = Vector(numData);
    }
};

template <typename BE, typename PR>
struct MeshAdjacency {
    using Vector = VectorBase<BE, PR>;
    using Vectorui = VectorBase<BE, Index>;
    //! Length: numPrimitives * numVerticesPerPrimitive
    //! For constructing BVH.
    Vectorui facets, edges;
    //! Length: numPrimitives
    //! Each index holds the rest area/length of corresponding primitive.
    Vector restFacetAreas, restEdgeLengths, restOppLengths;
    Vectorui vertexAdjFacets, vertexAdjFacetsOffsets;
    Vectorui vertexAdjEdges, vertexAdjEdgesOffsets;
    //Vectorui edgeAdjEdges, edgeAdjEdgesOffsets;
    Vectorui vertexOppVertices, vertexOppVerticesOffsets; // for spring
    template <typename InitializerParams>
    void memoryAllocation(InitializerParams& params) {
        if(params.numFacets > 0) {
            if(!facets.ptr) 
                facets = Vectorui(params.numFacets*3);
            if(!restFacetAreas.ptr) 
                restFacetAreas = Vector(params.numFacets);
        }
        if(params.numEdges > 0) {
            if(!edges.ptr) 
                edges = Vectorui(params.numEdges*2);
            if(!restEdgeLengths.ptr) 
                restEdgeLengths = Vector(params.numEdges);
        }
        if(params.numPoints > 0) {
            if(!vertexAdjFacetsOffsets.ptr)
                vertexAdjFacetsOffsets = Vectorui(params.numPoints+1, 0);
            if(!vertexAdjEdgesOffsets.ptr)
                vertexAdjEdgesOffsets = Vectorui(params.numPoints+1, 0);
            if(!vertexOppVerticesOffsets.ptr)
                vertexOppVerticesOffsets = Vectorui(params.numPoints+1, 0);
        }
    }
};

struct EdgeInfo {
    int v0=-1, v1=-1; // v0 < v1
    int f0=-1, f1=-1; // f0: (v1, v0, o0), f1: (v0, v1, o1)
    int o0=-1, o1=-1; // o0 in f0, o1 in f1
};

template <typename PR>
struct InitializerParams {
    // Commons
    Index numPoints, numFacets, numEdges;
    PR mass;
    InitializerParams(Index numPoints, Index numFacets, Index numEdges, PR mass) : numPoints(numPoints), numFacets(numFacets), numEdges(numEdges), mass(mass) {}
};
template <typename BE, typename PR>
struct GeneralMeshInitializer {
    virtual ~GeneralMeshInitializer() = default;
    virtual void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) = 0;
    virtual InitializerParams<PR>* getParams() = 0;
};
//! Suppose that the positions and facets are given
template <typename BE, typename PR>
struct MeshAdjacencyInitializer {
    using Vector = VectorBase<BE, PR>;

    static void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) {
        std::cout << "[MeshSpringInitializer initialize] start" << std::endl;
        Index maxNumEdgeInfos = adjacency.facets.size;
        Index numPoints = state.x.size/3;
        DynamicMemoryAllocator<BE> tempPool;
        VectorBase<BE, EdgeInfo> tempEdgeInfos(tempPool.template alloc<EdgeInfo>(maxNumEdgeInfos));
        VectorBase<BE, EdgeInfo> edgeInfos(tempPool.template alloc<EdgeInfo>(maxNumEdgeInfos));
        std::cout << "[MeshSpringInitializer initialize] temp vector allocated" << std::endl;

        // vertex adj facets
        
        Index numFacets = adjacency.facets.size/3;
        
        for(Index fid = 0; fid < numFacets; fid++) {
            Index fbase = fid*3;
            Index a = adjacency.facets[fbase];
            Index b = adjacency.facets[fbase+1];
            Index c = adjacency.facets[fbase+2];

            adjacency.vertexAdjFacetsOffsets[a+1]++;
            adjacency.vertexAdjFacetsOffsets[b+1]++;
            adjacency.vertexAdjFacetsOffsets[c+1]++;
        }

        for(Index vid = 0; vid < numPoints; ++vid) 
            adjacency.vertexAdjFacetsOffsets[vid+1] += adjacency.vertexAdjFacetsOffsets[vid];

        std::cout << "[MeshSpringInitializer initialize] vertex adj facets offsets set" << std::endl;

        adjacency.vertexAdjFacets = VectorBase<BE, Index>(adjacency.vertexAdjFacetsOffsets[numPoints]);
        
        VectorBase<BE, Index> offsets(tempPool.template zeros<Index>(numPoints));
        for(Index fid = 0; fid < numFacets; ++fid) {
            Index fbase = fid*3;
            Index v0 = adjacency.facets[fbase];
            Index v1 = adjacency.facets[fbase+1];
            Index v2 = adjacency.facets[fbase+2];

            Index v0base = offsets[v0]+adjacency.vertexAdjFacetsOffsets[v0];
            Index v1base = offsets[v1]+adjacency.vertexAdjFacetsOffsets[v1];
            Index v2base = offsets[v2]+adjacency.vertexAdjFacetsOffsets[v2];

            adjacency.vertexAdjFacets[v0base] = fid;
            adjacency.vertexAdjFacets[v1base] = fid;
            adjacency.vertexAdjFacets[v2base] = fid;

            offsets[v0]++;
            offsets[v1]++;
            offsets[v2]++;
        }
        std::cout << "[MeshSpringInitializer initialize] vertex adjacent facets set" << std::endl;
        for(Index i = 0; i < 10; ++i) {
            std::cout << i << "-th adjacent facets: ";
            for(Index fi = adjacency.vertexAdjFacetsOffsets[i]; fi < adjacency.vertexAdjFacetsOffsets[i+1]; ++fi) {
                std::cout << adjacency.vertexAdjFacets[fi] << ", ";
            }
            std::cout << std::endl;
        }


        
        // Fill temp edge infos (opposite edges are inserted twice)
        Index eIdx = 0;
        auto fillEdgeInfos = [&](Index edgeIndex, Index v0, Index v1, Index o0, Index f0) {
            if(v0 > v1) {
                Index temp = v0;
                v0 = v1;
                v1 = temp;
            }

            tempEdgeInfos[edgeIndex].v0 = v0;
            tempEdgeInfos[edgeIndex].v1 = v1;
            tempEdgeInfos[edgeIndex].o0 = o0;
            tempEdgeInfos[edgeIndex].o1 = -1;
            tempEdgeInfos[edgeIndex].f0 = f0;
            tempEdgeInfos[edgeIndex].f1 = -1;

        };
        for(Index fid = 0; fid < numFacets; ++fid) {
            Index fbase = fid*3;
            Index v0 = adjacency.facets[fbase];
            Index v1 = adjacency.facets[fbase+1];
            Index v2 = adjacency.facets[fbase+2];

            fillEdgeInfos(eIdx++, v0, v1, v2, fid);
            fillEdgeInfos(eIdx++, v1, v2, v0, fid);
            fillEdgeInfos(eIdx++, v2, v0, v1, fid);
        }
        std::cout << "[MeshSpringInitializer initialize] temp edges are filled" << std::endl;

        // Sort the temp edge infos to reduce
        std::sort(tempEdgeInfos.ptr, tempEdgeInfos.ptr+eIdx, [](EdgeInfo& a, EdgeInfo& b) {
            return a.v0 < b.v0 || (a.v0 == b.v0 && a.v1 < b.v1);
        });
        std::cout << "[MeshSpringInitializer initialize] temp edges are sorted" << std::endl;
        std::cout << " ---- test output ---- " << std::endl;
        for(int i = 0; i < 10; i++) 
            std::cout << tempEdgeInfos[i].v0 << ", " << tempEdgeInfos[i].v1 << " in facet id " << tempEdgeInfos[i].f0 << std::endl;

        // Reduce temp edge infos into unique edge infos
        // And initialize restEdgeLengths
        auto edgeLength = [&](Index vid0, Index vid1) {
            auto v0 = tinym::vec3_view(state.x.ptr+vid0*3);
            auto v1 = tinym::vec3_view(state.x.ptr+vid1*3);
            auto l = v1-v0;
            return l.norm();
        };
        adjacency.edges[0] = tempEdgeInfos[0].v0;
        adjacency.edges[1] = tempEdgeInfos[0].v1;
        edgeInfos[0] = tempEdgeInfos[0];
        Index edgeid = 0;
        adjacency.restEdgeLengths[edgeid] = edgeLength(tempEdgeInfos[0].v0, tempEdgeInfos[0].v1);
        adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[0].v0+1]++;
        adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[0].v1+1]++;
        //Index
        for(Index ei = 1; ei < eIdx; ++ei) {
            Index edgeBase = edgeid*2;
            if(tempEdgeInfos[ei].v0 != adjacency.edges[edgeBase] || tempEdgeInfos[ei].v1 != adjacency.edges[edgeBase+1]) {
                edgeid++;
                edgeBase = edgeid*2;

                adjacency.edges[edgeBase  ] = tempEdgeInfos[ei].v0;
                adjacency.edges[edgeBase+1] = tempEdgeInfos[ei].v1;
                
                adjacency.restEdgeLengths[edgeid] = edgeLength(tempEdgeInfos[ei].v0, tempEdgeInfos[ei].v1);
                adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[ei].v0+1]++;
                adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[ei].v1+1]++;

                edgeInfos[edgeid] = tempEdgeInfos[ei];
            } else {
                edgeInfos[edgeid].o1 = tempEdgeInfos[ei].o0;
                edgeInfos[edgeid].f1 = tempEdgeInfos[ei].f0;
                adjacency.vertexOppVerticesOffsets[edgeInfos[edgeid].o0+1]++;
                adjacency.vertexOppVerticesOffsets[edgeInfos[edgeid].o1+1]++;
            }
        }
        Index edgeNum = edgeid+1;
        for(int i = 0; i < 10; i++) {
            std::cout << adjacency.edges[i*2] << ", " << adjacency.edges[i*2+1] << ": " << adjacency.restEdgeLengths[i] << std::endl;
            //Index vid0 = adjacency.edges[i*2];
            //Index vid1 = adjacency.edges[i*2+1];
            //std::cout << state.x[vid0*3] << ", " << state.x[vid0*3+1] << ", " << state.x[vid0*3+2] << std::endl;
            //std::cout << state.x[vid1*3] << ", " << state.x[vid1*3+1] << ", " << state.x[vid1*3+2] << std::endl;
        }

        for(Index i = 0; i < numPoints; ++i) {
            adjacency.vertexOppVerticesOffsets[i+1] += adjacency.vertexOppVerticesOffsets[i];
            adjacency.vertexAdjEdgesOffsets[i+1] += adjacency.vertexAdjEdgesOffsets[i];
        }
        std::cout << "vertexOppVerticesOffsets: " << std::endl;
        for(int i = 0; i < 10; i++) 
            std::cout << adjacency.vertexOppVerticesOffsets[i] << std::endl;
        std::cout << "..." << adjacency.vertexOppVerticesOffsets[numPoints] << std::endl;
        std::cout << "vertexAdjEdgesOffsets: " << std::endl;
        for(int i = 0; i < 10; i++) 
            std::cout << adjacency.vertexAdjEdgesOffsets[i] << std::endl;
        std::cout << "..." << adjacency.vertexAdjEdgesOffsets[numPoints] << std::endl;
        std::cout << "edgeInfos: " << std::endl;
        for(Index i = 0; i < 10; i++)
            std::cout << edgeInfos[i].v0 << " " << edgeInfos[i].v1 << " " << edgeInfos[i].o0 << " " << edgeInfos[i].o1 << " " << edgeInfos[i].f0 << " " << edgeInfos[i].f1 << std::endl;

        
        // set vertexOppVertices and vertexAdjEdges
        VectorBase<BE, Index> oppOffsets(tempPool.template zeros<Index>(numPoints));
        VectorBase<BE, Index> adjOffsets(tempPool.template zeros<Index>(numPoints));
        std::cout << "[MeshSpringInitializer initialize] oppOffsets allocated" << std::endl;
        adjacency.vertexOppVertices = VectorBase<BE, Index>(adjacency.vertexOppVerticesOffsets[numPoints], 0);
        adjacency.restOppLengths = VectorBase<BE, PR>(adjacency.vertexOppVerticesOffsets[numPoints]);
        adjacency.vertexAdjEdges = VectorBase<BE, Index>(adjacency.vertexAdjEdgesOffsets[numPoints], 0);
        std::cout << "[MeshSpringInitializer initialize] vertex opposite vertices allocated" << std::endl;
        for(Index ei = 0; ei < edgeNum; ++ei) {
            if(edgeInfos[ei].o1 != -1) {
                Index o0 = edgeInfos[ei].o0;
                Index o1 = edgeInfos[ei].o1;

                Index o0base = oppOffsets[o0]+adjacency.vertexOppVerticesOffsets[o0];
                Index o1base = oppOffsets[o1]+adjacency.vertexOppVerticesOffsets[o1];

                adjacency.vertexOppVertices[o0base] = o1;
                adjacency.vertexOppVertices[o1base] = o0;

                adjacency.restOppLengths[o0base] = edgeLength(o0, o1);
                adjacency.restOppLengths[o1base] = edgeLength(o0, o1);

                oppOffsets[o0]++;
                oppOffsets[o1]++;
            }

            Index v0 = edgeInfos[ei].v0;
            Index v1 = edgeInfos[ei].v1;

            Index v0base = adjOffsets[v0]+adjacency.vertexAdjEdgesOffsets[v0];
            Index v1base = adjOffsets[v1]+adjacency.vertexAdjEdgesOffsets[v1];

            adjacency.vertexAdjEdges[v0base] = ei;
            adjacency.vertexAdjEdges[v1base] = ei;

            adjOffsets[v0]++;
            adjOffsets[v1]++;
        }
        std::cout << "[MeshSpringInitializer initialize] Opposite vertices set" << std::endl;

        for(Index i = 0; i < 10; i++) {
            std::cout << i << "-th opposite: ";
            for(Index oi = adjacency.vertexOppVerticesOffsets[i]; oi < adjacency.vertexOppVerticesOffsets[i+1]; ++oi) {
                std::cout << adjacency.vertexOppVertices[oi] << ", ";
            }
            std::cout << std::endl;
        }
        for(Index i = 0; i < 10; i++) {
            std::cout << i << "-th adj edges: ";
            for(Index ei = adjacency.vertexAdjEdgesOffsets[i]; ei < adjacency.vertexAdjEdgesOffsets[i+1]; ++ei) {
                std::cout << adjacency.vertexAdjEdges[ei] << ", ";
            }
            std::cout << std::endl;
        }
    }
};

enum struct PlaneDirection : Index {
    XYPlane,
    YZPlane,
    XZPlane,
};

//! Special class for grid cloth
template <typename PR>
struct MeshGridInitializerParams : InitializerParams<PR> {

    // Specifics
    Index particleNum1D;
    PR size1D;
    bool jiggle;
    PlaneDirection dir;
    tinym::vec3 center;

    MeshGridInitializerParams(PlaneDirection dir, tinym::vec3 center, Index particleNum1D, PR size1D, PR mass, bool jiggle)
        : dir(dir), center(center),
        particleNum1D(particleNum1D), 
        InitializerParams<PR>(
                particleNum1D*particleNum1D, // numPoints
                2*(particleNum1D-1)*(particleNum1D-1), // numFacets
                2*(particleNum1D-1)*particleNum1D+2*(particleNum1D-1)*(particleNum1D-1), // numEdges
                mass),
        size1D(size1D), jiggle(jiggle) {}
};

template <typename BE, typename PR>
struct MeshGridInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshGridInitializerParams<PR>;
    ParamsType params;

    MeshGridInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        std::cout << "[MeshGridSpringInitializer initialize] start" << std::endl;
        
        state.memoryAllocation(params); // numPoints
        adjacency.memoryAllocation(params); // numPoints, numFacets, numEdges

        PR halfSize = params.size1D / 2.0;
        PR length = params.size1D/PR(params.particleNum1D-1);

        for (int row = 0; row < params.particleNum1D; ++row) {
            for (int col = 0; col < params.particleNum1D; ++col) {
                int base = (row*params.particleNum1D + col)*3; 
                
                PR px =  col*length - halfSize;
                PR py = -row*length + halfSize;
                PR pz = params.jiggle ? rand()/PR(RAND_MAX)/10000.f : 0.f;
                
                switch(params.dir) {
                    case PlaneDirection::XYPlane:
                        state.x[base  ] = px+params.center.x;
                        state.x[base+1] = py+params.center.y;
                        state.x[base+2] = pz+params.center.z;
                        break;
                    case PlaneDirection::YZPlane:
                        state.x[base  ] = pz+params.center.x;
                        state.x[base+1] = px+params.center.y;
                        state.x[base+2] = -py+params.center.z;
                        break;
                    case PlaneDirection::XZPlane:
                        state.x[base  ] = px+params.center.x;
                        state.x[base+1] = pz+params.center.y;
                        state.x[base+2] = -py+params.center.z;
                        break;
                    default: break;
                }
            }
        }
        std::cout << "[MeshGridSpringInitializer initialize] position set" << std::endl;

        if(adjacency.vertexAdjFacets.ptr) return;

        Index fIdx = 0;
        for (Index row = 0; row < params.particleNum1D - 1; ++row) {
            for (Index col = 0; col < params.particleNum1D - 1; ++col) {
                Index p00 = (row * params.particleNum1D + col);
                Index p10 = (row * params.particleNum1D + col + 1);
                Index p01 = ((row + 1) * params.particleNum1D + col);
                Index p11 = ((row + 1) * params.particleNum1D + col + 1);
                // p00   p10
                //
                // p01   p11

                auto addFacet = [&](Index a, Index b, Index c) {
                    adjacency.facets[fIdx++] = a;
                    adjacency.facets[fIdx++] = b;
                    adjacency.facets[fIdx++] = c;
                };

                if (((row + col) & 1) == 0) { // even
                    // diagonal: p00 - p11
                    addFacet(p00, p01, p11);
                    addFacet(p00, p11, p10);
                } else { // odd
                    // diagonal: p10 - p01
                    addFacet(p00, p01, p10);
                    addFacet(p10, p01, p11);
                }
            }
        }
        std::cout << "[MeshGridSpringInitializer initialize] facets set" << std::endl;


        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

template <typename PR>
struct MeshFileInitializerParams : InitializerParams<PR> {
    // Specifics
    std::string prefix, fileName;
    tinym::vec3 offset;
    PR scale;

    MeshFileInitializerParams(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR mass) 
        : prefix(prefix), fileName(fileName), offset(offset), scale(scale), InitializerParams<PR>(0,0,0,mass) {}
};

template <typename BE, typename PR>
struct MeshFileInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshFileInitializerParams<PR>;
    ParamsType params;
    ObjData data;

    MeshFileInitializer(ParamsType params) : params(params) {
        data.loadObject(params.prefix, params.fileName);

        this->params.numPoints = data.nVertices;
        this->params.numFacets = data.nElements3;

        std::set<std::pair<int,int>> edges;
        for (const auto& face : data.elements3) {
            int n = 3;
            for (int i = 0; i < n; ++i) {
                int a = face[i];
                int b = face[(i + 1) % n];

                if (a > b) std::swap(a, b);
                edges.insert({a, b});
            }
        }
        this->params.numEdges = edges.size();
    }

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params); // numPoints
        adjacency.memoryAllocation(params); // numPoints, numFacets, numEdges

        for(Index vid = 0; vid < params.numPoints; vid++) {
            Index vbase = vid*3;
            state.x[vbase  ] = data.vertices[vid].x*params.scale + params.offset.x;
            state.x[vbase+1] = data.vertices[vid].y*params.scale + params.offset.y;
            state.x[vbase+2] = data.vertices[vid].z*params.scale + params.offset.z;
        }

        if(adjacency.vertexAdjFacets.ptr) return;
        for(Index fid = 0; fid < params.numFacets; fid++) {
            Index fbase = fid*3;
            adjacency.facets[fbase  ] = data.elements3[fid].x;
            adjacency.facets[fbase+1] = data.elements3[fid].y;
            adjacency.facets[fbase+2] = data.elements3[fid].z;
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    InitializerParams<PR>* getParams() override { return &params; }
};


struct alignas(8) IndexPair {
    union {
        struct { Index query, target; }; 
        struct { Index point, triangle; }; 
        struct { Index edge1, edge2; };
    };

    bool operator<(const IndexPair& o) const {
        if(query == o.query) return target < o.target;
        return query < o.query;
    }
};

enum struct ShapeType : Index {
    Mesh, // each collisions are checked in a particle way
    Plane,
};

const char* behaviorTypeName(BehaviorType behaviorType) {
    switch (behaviorType) {
        case BehaviorType::TriangularCloth: return "TriangularCloth";
        case BehaviorType::FastGridCloth: return "FastGridCloth";
        case BehaviorType::Elastic: return "Elastic";
        case BehaviorType::Rigid: return "Rigid";
        case BehaviorType::Float: return "Float";
        case BehaviorType::Fluid: return "Fluid";
        case BehaviorType::Generator: return "Generator";
        default: return "Unknown";
    }
}

int behaviorInspectorIndex(BehaviorType behaviorType) {
    switch (behaviorType) {
        case BehaviorType::TriangularCloth: return 0;
        case BehaviorType::FastGridCloth: return 1;
        case BehaviorType::Float: return 2;
        default: return -1;
    }
}

BehaviorType behaviorFromInspectorIndex(int index) {
    switch (index) {
        case 0: return BehaviorType::TriangularCloth;
        case 1: return BehaviorType::FastGridCloth;
        case 2: return BehaviorType::Float;
        default: return BehaviorType::Float;
    }
}

const char* shapeTypeName(ShapeType shapeType) {
    switch (shapeType) {
        case ShapeType::Mesh: return "Mesh";
        case ShapeType::Plane: return "Plane";
        default: return "Unknown";
    }
}

struct alignas(32) BroadCollision {
    IndexPair indexPair;
    IndexPair objPair;
    IndexPair behaviorPair;
    IndexPair shapePair;
};

struct NarrowCollision {
    IndexPair indexPair;
    IndexPair objPair;
    tinym::vec4 collisionNormalAndDistance;
    tinym::vec4 varycentricCoord;
    IndexPair behaviorPair;
    IndexPair shapePair;
};

template <typename BE, typename PR>
struct Constraints {
    VectorBase<BE, PR> fixedParticles;

    //Index maxNumCollisions = 0;
    ////Index numBroadCollisions = 0;
    //VectorBase<BE, Index> numBroadCollisions;
    //VectorBase<BE, Index> numNarrowCollisions;
    //Index approxColPerVertex = 15;
    //VectorBase<BE, BroadCollision> broadCollisions;
    //VectorBase<BE, NarrowCollision> narrowCollisions;

    //VectorBase<BE, NarrowCollision> vertexColPrims;
    //VectorBase<BE, Index> vertexColPrimsOffsets;

    private:

    public:
    void memoryAllocation(Index numPoints) {
        if(fixedParticles.ptr) return;
        fixedParticles = VectorBase<BE, PR>(numPoints, 1);

        //maxNumCollisions = numPoints * approxColPerVertex;
        //numBroadCollisions = VectorBase<BE, Index>(1);
        //numNarrowCollisions = VectorBase<BE, Index>(1);

        //broadCollisions = VectorBase<BE, BroadCollision>(maxNumCollisions);
        //narrowCollisions = VectorBase<BE, NarrowCollision>(maxNumCollisions);

        //vertexColPrims = VectorBase<BE, NarrowCollision>(maxNumCollisions);
        //vertexColPrimsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
    }

    void fixParticle(Index id) { fixedParticles[id] = PR(0); }
    void releaseParticle(Index id) { fixedParticles[id] = PR(1); }

};

template <typename PR>
struct ClothBehaviorParams {
    PR stretch, shear, bend;
    PR thickness;
};

template <typename PR>
struct FastGridClothBehaviorParams {
    uint particleNum1D;
    PR stretchRest, shearRest, bendRest;
    PR kstretch, kshear, kbend;
    PR thickness;
};

template <typename PR>
struct FloatBehaviorParams {};

template <typename PR>
using BehaviorParams = std::variant<
    ClothBehaviorParams<PR>, 
    FastGridClothBehaviorParams<PR>,
    FloatBehaviorParams<PR>
>;

//! Force accumulator
template <typename BE, typename PR>
struct TriangularClothBehavior {};

template <typename PR>
struct TriangularClothBehavior<METAL, PR> {

    static MTL::ComputePipelineState* getPSO() {
        static MTL::ComputePipelineState* pso = nullptr;
        if (!pso) {
            pso = MetalKernelContext::getPSO("compute_tri_spring_forces");
        }
        return pso;
    }

    template <typename SimParams>
    static void setBuffer(
            GeneralMesh<METAL, PR>& mesh,
            SimParams& simParams) {
        Index offset = 0;
        // state 0-3
        MetalGlobalContext::setBuffer(mesh.state.x, offset++);
        MetalGlobalContext::setBuffer(mesh.state.v, offset++);
        MetalGlobalContext::setBuffer(mesh.state.f, offset++);
        MetalGlobalContext::setBuffer(mesh.state.m, offset++);
        // constraints 4-6
        MetalGlobalContext::setBuffer(mesh.constraints.fixedParticles, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL,PR>::packedCollisionData.vertColFacets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL,PR>::packedCollisionData.vertColFacetsOffsets, offset++);
        // external forces 7
        MetalGlobalContext::setBuffer(mesh.externalForces.externalForces, offset++);
        //simulation parameters 8-9
        MetalGlobalContext::setBytes(simParams, offset++);
        MetalGlobalContext::setBytes(std::get<ClothBehaviorParams<PR>>(mesh.behaviorParams), offset++);
        // adjacency 10-11
        MetalGlobalContext::setBuffer(mesh.adjacency.edges, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.facets, offset++);
        // stretch springs 12-14
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdges, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdgesOffsets, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.restEdgeLengths, offset++);
        // bend springs 15-17
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVertices, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVerticesOffsets, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.restOppLengths, offset++);
        // packed data
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedMeshData.statesOffsets, offset++);
        MetalGlobalContext::setBytes(mesh.id, offset++);
    }

    static void update(MeshState<METAL, PR>& state) {
        auto* pso = getPSO();
        size_t vertexNum = state.x.size/3;
        MetalGlobalContext::dispatchThreads(pso, vertexNum);
    }
};

template <typename BE, typename PR>
struct FastGridClothBehavior {};

template <typename PR>
struct FastGridClothBehavior<METAL, PR> {
    static MTL::ComputePipelineState* getPSO() {
        static MTL::ComputePipelineState* pso = nullptr;
        if (!pso) {
            pso = MetalKernelContext::getPSO("compute_cloth_grid_forces_fast");
        }
        return pso;
    }

    template <typename SimParams>
    static void setBuffer(
            GeneralMesh<METAL, PR>& mesh,
            SimParams& simParams) {
        Index offset = 0;
        // state 0-3
        MetalGlobalContext::setBuffer(mesh.state.x, offset++);
        MetalGlobalContext::setBuffer(mesh.state.v, offset++);
        MetalGlobalContext::setBuffer(mesh.state.f, offset++);
        MetalGlobalContext::setBuffer(mesh.state.m, offset++);
        // constraints 4-6
        MetalGlobalContext::setBuffer(mesh.constraints.fixedParticles, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedCollisionData.vertColFacets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets, offset++);
        // external forces 7
        MetalGlobalContext::setBuffer(mesh.externalForces.externalForces, offset++);
        //simulation parameters 8-9
        MetalGlobalContext::setBytes(simParams, offset++);
        MetalGlobalContext::setBytes(std::get<FastGridClothBehaviorParams<PR>>(mesh.behaviorParams), offset++);
        //// adjacency 10-11
        //MetalGlobalContext::setBuffer(mesh.adjacency.edges, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.facets, offset++);
        //// stretch springs 12-14
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdges, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdgesOffsets, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.restEdgeLengths, offset++);
        //// bend springs 15-17
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVertices, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVerticesOffsets, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.restOppLengths, offset++);

        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedMeshData.statesOffsets, offset++);
        MetalGlobalContext::setBytes(mesh.id, offset++);
    }

    static void update(MeshState<METAL, PR>& state) {
        auto* pso = getPSO();
        size_t vertexNum = state.x.size/3;
        MetalGlobalContext::dispatchThreads(pso, vertexNum);
    }
};

struct Material {
    tinym::vec3 baseColor = tinym::vec3(1.0f);
};




template <typename BE, typename PR>
struct GeneralMesh {
    int id;


    MeshState<BE, PR> state;
    MeshAdjacency<BE, PR> adjacency;
    GeneralMeshInitializer<BE, PR>* initializer;
    BehaviorType behaviorType;
    ShapeType shapeType = ShapeType::Mesh;
    BehaviorParams<PR> behaviorParams;
    Material material;
    Constraints<BE, PR> constraints;
    ExternalForces<BE, PR> externalForces;

    MeshGL<CPU> meshGL;




    GeneralMesh(GeneralMeshInitializer<BE, PR>* initializer, BehaviorType behaviorType, BehaviorParams<PR> behaviorParams) 
    : initializer(initializer), behaviorType(behaviorType), behaviorParams(behaviorParams) {}
    GeneralMesh(GeneralMesh&& other) noexcept
        : id(other.id),
          state(std::move(other.state)),
          adjacency(std::move(other.adjacency)),
          initializer(other.initializer),
          behaviorType(other.behaviorType),
          behaviorParams(other.behaviorParams),
          material(std::move(other.material)),
          constraints(std::move(other.constraints)),
          externalForces(std::move(other.externalForces)),
          meshGL(std::move(other.meshGL)) 
        {
            other.initializer = nullptr;
        }
    ~GeneralMesh() = default;

    void initialize() {
        std::cout << "  - [GeneralMesh initialize] id " << id << " try to initialize\n";
        std::cout << "  - [GeneralMesh initialize] initializer " << initializer << "\n";
        initializer->initialize(state, adjacency);
        std::cout << "  - [GeneralMesh initialize] id " << id << " initializer init\n";
        constraints.memoryAllocation(state.x.size/3);
        std::cout << "  - [GeneralMesh initialize] id " << id << " constraints init\n";
        meshGL = MeshGL<CPU>(state.x.size/3, state.x.ptr, adjacency.facets.size/3, adjacency.facets.ptr, state.n.ptr);
        std::cout << "  - [GeneralMesh initialize] id " << id << " meshGL init. finished.\n";
    }

    void draw(Program& shader, const tinym::vec3& baseColor) {
        meshGL.draw(shader, baseColor);
    }
};






struct Ray {
    tinym::vec3 origin;
    tinym::vec3 dir;
};


struct RayHit {
    Index obj;
    Index primId;
    float tmin, tmax;
};


template <typename BE, typename PR>
struct Scene {
    inline static int numMeshes = 0;

    inline static std::vector<GeneralMesh<BE, PR>> meshes;

    struct RequestGeneralMesh {
        int id;
        std::unique_ptr<GeneralMeshInitializer<BE, PR>> initializer;
        BehaviorType behaviorType;
        BehaviorParams<PR> behaviorParams;
        Material material;

        RequestGeneralMesh(int id, GeneralMeshInitializer<BE, PR> *initializer,
                           BehaviorType behaviorType,
                           BehaviorParams<PR> behaviorParams)
            : id(id),
              initializer(initializer),
              behaviorType(behaviorType),
              behaviorParams(std::move(behaviorParams)),
              material() {}
    };

    inline static std::vector<RequestGeneralMesh> requestsGeneralMeshes;
    inline static bool dirty = true;

    void addGeneralMesh(GeneralMeshInitializer<BE, PR>* initializer, BehaviorType behaviorType, BehaviorParams<PR> behaviorParams) {
        //meshes.emplace_back(initializer, behaviorType, behaviorParams);
        //meshes.back().id = numMeshes++;

        requestsGeneralMeshes.emplace_back(numMeshes++, initializer, behaviorType, behaviorParams);

        dirty = true;

        //std::cout << "id " << meshes.back().id << " object is created\n";
    }

    static bool supportsBehavior(const RequestGeneralMesh& request, BehaviorType behaviorType) {
        switch (behaviorType) {
            case BehaviorType::TriangularCloth:
            case BehaviorType::Float:
                return true;
            case BehaviorType::FastGridCloth:
                return dynamic_cast<const MeshGridInitializer<BE, PR>*>(request.initializer.get()) != nullptr;
            default:
                return false;
        }
    }

    static RequestGeneralMesh* findRequestById(int id) {
        for (auto& request : requestsGeneralMeshes) {
            if (request.id == id) return &request;
        }
        return nullptr;
    }

    static BehaviorParams<PR> makeBehaviorParamsForBehavior(BehaviorType behaviorType, const RequestGeneralMesh& request) {
        switch (behaviorType) {
            case BehaviorType::TriangularCloth: {
                if (const auto* cloth = std::get_if<ClothBehaviorParams<PR>>(&request.behaviorParams)) {
                    return *cloth;
                }
                if (const auto* fast_grid = std::get_if<FastGridClothBehaviorParams<PR>>(&request.behaviorParams)) {
                    return ClothBehaviorParams<PR>{fast_grid->kstretch, fast_grid->kshear, fast_grid->kbend, fast_grid->thickness};
                }
                return ClothBehaviorParams<PR>{1e4, 1e4, 2e4, 0.01};
            }
            case BehaviorType::FastGridCloth: {
                if (const auto* fast_grid = std::get_if<FastGridClothBehaviorParams<PR>>(&request.behaviorParams)) {
                    return *fast_grid;
                }

                FastGridClothBehaviorParams<PR> params{};
                params.particleNum1D = 2;
                params.stretchRest = PR(1);
                params.shearRest = PR(std::sqrt(2.0));
                params.bendRest = PR(2);

                if (request.initializer) {
                    if (const auto* grid_initializer = dynamic_cast<const MeshGridInitializer<BE, PR>*>(request.initializer.get())) {
                        params.particleNum1D = grid_initializer->params.particleNum1D;
                        PR rest = grid_initializer->params.size1D / PR(grid_initializer->params.particleNum1D);
                        params.stretchRest = rest;
                        params.shearRest = rest * PR(std::sqrt(2.0));
                        params.bendRest = rest * PR(2);
                    }
                }

                if (const auto* cloth = std::get_if<ClothBehaviorParams<PR>>(&request.behaviorParams)) {
                    params.kstretch = cloth->stretch;
                    params.kshear = cloth->shear;
                    params.kbend = cloth->bend;
                    params.thickness = cloth->thickness;
                } else {
                    params.kstretch = 1e4;
                    params.kshear = 1e4;
                    params.kbend = 2e4;
                    params.thickness = 0.01;
                }
                return params;
            }
            case BehaviorType::Float:
                return FloatBehaviorParams<PR>{};
            default:
                return FloatBehaviorParams<PR>{};
        }
    }

    static bool syncRequestMaterialFromMesh(int id) {
        return syncRequestStateFromMesh(id);
    }

    static bool syncRequestStateFromMesh(int id) {
        auto* mesh = findById(id);
        auto* request = findRequestById(id);
        if (!mesh || !request) return false;
        request->material = mesh->material;
        request->behaviorType = mesh->behaviorType;
        request->behaviorParams = mesh->behaviorParams;
        return true;
    }

    static bool setRequestBehavior(int id, BehaviorType behaviorType) {
        auto* request = findRequestById(id);
        if (!request || !supportsBehavior(*request, behaviorType)) return false;
        request->behaviorType = behaviorType;
        request->behaviorParams = makeBehaviorParamsForBehavior(behaviorType, *request);
        dirty = true;
        return true;
    }


    struct PackedMeshData {
        // MeshState
        VectorBase<BE, PR> x;
        VectorBase<BE, PR> v;
        VectorBase<BE, PR> f;
        VectorBase<BE, PR> m;
        VectorBase<BE, PR> n;
        // MeshAdjacency
        VectorBase<BE, Index> facets;
        VectorBase<BE, Index> edges;
        VectorBase<BE, Index> vertexAdjFacets, vertexAdjFacetsOffsets;
        VectorBase<BE, Index> vertexAdjEdges, vertexAdjEdgesOffsets;

        // offset data by id
        VectorBase<BE, Index> statesOffsets;
        VectorBase<BE, Index> facetsOffsets;
        VectorBase<BE, Index> edgesOffsets;

        // per-object thickness (0 for behaviors without thickness)
        VectorBase<BE, PR> thicknesses;

        void reset() {
            x = {};
            v = {};
            f = {};
            m = {};
            n = {};
            facets = {};
            edges = {};
            vertexAdjFacets = {};
            vertexAdjFacetsOffsets = {};
            vertexAdjEdges = {};
            vertexAdjEdgesOffsets = {};
            statesOffsets = {};
            facetsOffsets = {};
            edgesOffsets = {};
            thicknesses = {};
        }

    };
    inline static PackedMeshData packedMeshData;
    inline static void clearForces() {
        static MTL::ComputePipelineState* pso = MetalKernelContext::getPSO("clearForces");

        MetalGlobalContext::setBuffer(packedMeshData.f, 0);
        MetalGlobalContext::setBytes(packedMeshData.x.size, 1);
        MetalGlobalContext::dispatchThreads(pso, packedMeshData.x.size);
    }

    struct PackedCollisionData {
        VectorBase<BE, BroadCollision> broadCollisions;
        VectorBase<BE, Index> numBroadCollisions;
        VectorBase<BE, NarrowCollision> narrowCollisions;
        VectorBase<BE, Index> numNarrowCollisions;
        Index approxColsPerPoints = 15;
        Index maxNumCollisions;
        VectorBase<BE, NarrowCollision> vertColFacets;
        VectorBase<BE, Index> vertColFacetsOffsets;

        void reset() {
            broadCollisions = {};
            numBroadCollisions = {};
            narrowCollisions = {};
            numNarrowCollisions = {};
            maxNumCollisions = 0;
            vertColFacets = {};
            vertColFacetsOffsets = {};
        }

        void resetNarrow() {
            if (!narrowCollisions.ptr || !numNarrowCollisions.ptr) return;

            Index currentCount = numNarrowCollisions[0];
            if (currentCount > narrowCollisions.size) currentCount = narrowCollisions.size;
            if (narrowCollisions.ptr && currentCount > 0) {
                std::memset(narrowCollisions.ptr, 0, sizeof(NarrowCollision) * currentCount);
            }
            if (vertColFacets.ptr && currentCount > 0) {
                std::memset(vertColFacets.ptr, 0, sizeof(NarrowCollision) * currentCount);
            }
            if (vertColFacetsOffsets.ptr) {
                std::memset(vertColFacetsOffsets.ptr, 0, sizeof(Index) * vertColFacetsOffsets.size);
            }
            numNarrowCollisions[0] = 0;
        }
    };
    inline static PackedCollisionData packedCollisionData;

    struct RepulsionParams {
        float krepulsion;   // repulsion spring stiffness
    };
    inline static void applyRepulsionForces() {
        static MTL::ComputePipelineState* pso = MetalKernelContext::getPSO("applyRepulsionForces_noSort");
        if(packedCollisionData.numNarrowCollisions[0] == 0) return;

        RepulsionParams repParams = {1e6};
        MetalGlobalContext::setBuffer(packedMeshData.x, 0);
        MetalGlobalContext::setBuffer(packedMeshData.f, 2);
        MetalGlobalContext::setBuffer(packedMeshData.m, 3);
        MetalGlobalContext::setBuffer(packedCollisionData.narrowCollisions, 5);
        MetalGlobalContext::setBuffer(packedCollisionData.numNarrowCollisions, 6);
        MetalGlobalContext::setBytes(repParams, 8);
        MetalGlobalContext::setBuffer(packedMeshData.thicknesses, 9);
        MetalGlobalContext::setBuffer(packedMeshData.statesOffsets, 18);
        MetalGlobalContext::setBuffer(packedMeshData.facetsOffsets, 19);
        MetalGlobalContext::setBuffer(packedMeshData.facets, 20);
        
        MetalGlobalContext::dispatchThreads(pso, packedCollisionData.numNarrowCollisions[0]);
    }

    struct RayTracedData {
        VectorBase<BE, RayHit> clickRayCollisions;
        VectorBase<BE, Index> numClickRayCollisions;
        Index approxColsPerRay = 4096;

        void reset() {
            clickRayCollisions = {};
            numClickRayCollisions = {};
        }
    };
    inline static RayTracedData rayTracedData;

    static void resetRuntimeMemory() {
        meshes.clear();
        packedMeshData.reset();
        packedCollisionData.reset();
        rayTracedData.reset();
        GlobalAutoAllocator<BE>::reset();
    }

    static void pack() {
        if(!dirty) {
            initialize();
            return;
        }

        resetRuntimeMemory();
        meshes.reserve(numMeshes);

        // count sizes
        packedMeshData.statesOffsets = VectorBase<BE, Index>(numMeshes+1, 0);
        packedMeshData.facetsOffsets = VectorBase<BE, Index>(numMeshes+1, 0);
        packedMeshData.edgesOffsets  = VectorBase<BE, Index>(numMeshes+1, 0);
        packedMeshData.thicknesses  = VectorBase<BE, PR>(numMeshes, 0);
        std::vector<PR> masses(numMeshes);

        for(Index i = 0; i < requestsGeneralMeshes.size(); ++i) {
            RequestGeneralMesh& req = requestsGeneralMeshes[i];

            packedMeshData.statesOffsets[i+1] = packedMeshData.statesOffsets[i] + req.initializer->getParams()->numPoints;
            packedMeshData.facetsOffsets[i+1] = packedMeshData.facetsOffsets[i] + req.initializer->getParams()->numFacets;
            packedMeshData.edgesOffsets [i+1] = packedMeshData.edgesOffsets [i] + req.initializer->getParams()->numEdges;
            masses[i] = req.initializer->getParams()->mass;

            std::visit([&](auto&& params) {
                using T = std::decay_t<decltype(params)>;
                if constexpr (std::is_same_v<T, ClothBehaviorParams<PR>>) {
                    packedMeshData.thicknesses[i] = params.thickness;
                } else if constexpr (std::is_same_v<T, FastGridClothBehaviorParams<PR>>) {
                    packedMeshData.thicknesses[i] = params.thickness;
                }
                // FloatBehaviorParams and others: stays 0
            }, req.behaviorParams);
        }

        // allocate MeshState
        Index numPoints = packedMeshData.statesOffsets[numMeshes];
        Index numStatesData = numPoints*3;
        packedMeshData.x = VectorBase<BE, PR>(numStatesData);
        packedMeshData.v = VectorBase<BE, PR>(numStatesData, 0);
        packedMeshData.f = VectorBase<BE, PR>(numStatesData, 0);
        packedMeshData.m = VectorBase<BE, PR>(numStatesData);
        for(Index i = 0; i < requestsGeneralMeshes.size(); ++i) 
            std::fill(packedMeshData.m.ptr + packedMeshData.statesOffsets[i]*3,
                    packedMeshData.m.ptr + packedMeshData.statesOffsets[i+1]*3,
                    masses[i]);
        packedMeshData.n = VectorBase<BE, PR>(numStatesData);
        
        // allocate MeshAdjacency
        packedMeshData.facets = VectorBase<BE, Index>(packedMeshData.facetsOffsets[numMeshes]*3);
        packedMeshData.edges  = VectorBase<BE, Index>(packedMeshData.edgesOffsets [numMeshes]*2);
        packedMeshData.vertexAdjFacetsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
        packedMeshData.vertexAdjEdgesOffsets  = VectorBase<BE, Index>(numPoints+1, 0);

        // initialize meshes
        //meshes.resize(numMeshes);
        Index numVertexAdjFacets = 0, numVertexAdjEdges = 0;
        for(Index i = 0; i < numMeshes; ++i) {
            RequestGeneralMesh& req = requestsGeneralMeshes[i];
            meshes.emplace_back(req.initializer.get(), req.behaviorType, req.behaviorParams);
            meshes[i].id = req.id;
            meshes[i].material = req.material;
            Index prevNumPoints = packedMeshData.statesOffsets[i];
            Index curNumPoints = packedMeshData.statesOffsets[i+1]-prevNumPoints;
            meshes[i].state.x = VectorBase<BE, PR>(packedMeshData.x, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.v = VectorBase<BE, PR>(packedMeshData.v, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.f = VectorBase<BE, PR>(packedMeshData.f, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.m = VectorBase<BE, PR>(packedMeshData.m, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.n = VectorBase<BE, PR>(packedMeshData.n, prevNumPoints*3, curNumPoints*3);

            meshes[i].adjacency.facets = VectorBase<BE, Index>(packedMeshData.facets, packedMeshData.facetsOffsets[i]*3, (packedMeshData.facetsOffsets[i+1]-packedMeshData.facetsOffsets[i])*3);
            meshes[i].adjacency.edges  = VectorBase<BE, Index>(packedMeshData.edges, packedMeshData.edgesOffsets[i]*2, (packedMeshData.edgesOffsets[i+1]-packedMeshData.edgesOffsets[i])*2);

            meshes[i].initialize();

            for(int j = 0; j < curNumPoints; ++j) {
                packedMeshData.vertexAdjFacetsOffsets[prevNumPoints+j+1] = meshes[i].adjacency.vertexAdjFacetsOffsets[j+1]+numVertexAdjFacets;
                packedMeshData.vertexAdjEdgesOffsets [prevNumPoints+j+1] = meshes[i].adjacency.vertexAdjEdgesOffsets [j+1]+numVertexAdjEdges;
            }
            numVertexAdjFacets += meshes[i].adjacency.vertexAdjFacets.size;
            numVertexAdjEdges  += meshes[i].adjacency.vertexAdjEdges.size;
        }

        packedMeshData.vertexAdjFacets = VectorBase<BE, Index>(numVertexAdjFacets);
        packedMeshData.vertexAdjEdges  = VectorBase<BE, Index>(numVertexAdjEdges);
        numVertexAdjFacets = numVertexAdjEdges = 0;
        Index numFacets = 0, numEdges = 0;
        for(Index i = 0; i < numMeshes; ++i) {
            auto& vaf = meshes[i].adjacency.vertexAdjFacets;
            auto& vae = meshes[i].adjacency.vertexAdjEdges;
            std::copy(vaf.ptr, vaf.ptr+vaf.size, packedMeshData.vertexAdjFacets.ptr+numVertexAdjFacets);
            std::copy(vae.ptr, vae.ptr+vae.size, packedMeshData.vertexAdjEdges.ptr+numVertexAdjEdges);
            for(Index j = 0; j < vaf.size; ++j)
                if(packedMeshData.vertexAdjFacets[numVertexAdjFacets+j] != vaf[j]) exit(1);
            for(Index j = 0; j < vae.size; ++j)
                if(packedMeshData.vertexAdjEdges[numVertexAdjEdges+j]  != vae[j]) exit(1);
            vaf = VectorBase<BE, Index>(packedMeshData.vertexAdjFacets, numVertexAdjFacets, vaf.size);
            vae = VectorBase<BE, Index>(packedMeshData.vertexAdjEdges,  numVertexAdjEdges,  vae.size);
            numVertexAdjFacets += vaf.size;
            numVertexAdjEdges  += vae.size;
        }

        // allocate collisions
        packedCollisionData.maxNumCollisions = numPoints*packedCollisionData.approxColsPerPoints;
        packedCollisionData.broadCollisions = VectorBase<BE, BroadCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.numBroadCollisions = VectorBase<BE, Index>(1, 0);
        packedCollisionData.narrowCollisions = VectorBase<BE, NarrowCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.numNarrowCollisions = VectorBase<BE, Index>(1, 0);
        packedCollisionData.vertColFacets = VectorBase<BE, NarrowCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.vertColFacetsOffsets = VectorBase<BE, Index>(numPoints+1, 0);


        // allocate ray traced data
        rayTracedData.clickRayCollisions = VectorBase<BE, RayHit>(rayTracedData.approxColsPerRay);
        rayTracedData.numClickRayCollisions = VectorBase<BE, Index>(1, 0);


        // adjacency data

        dirty = false;
    }

    static void initialize() {
        for(auto& mesh : meshes) {
            std::cout << "  - try to initialize mesh " << mesh.id << "\n";
            mesh.initialize();
            mesh.state.v.map().setZero();
            std::cout << "  - mesh " << mesh.id << " is initialized\n";
        }
        if(meshes.size() > 0) std::cout << "[Simulator Init] general mesh objects are initialized" << std::endl;
    }

    /// postpack() is for data which is able to allocate after the mesh initialization.



    static GeneralMesh<BE, PR>* findById(int id) {
        for(auto& mesh : meshes) {
            if(mesh.id == id) return &mesh;
        }
        return nullptr;
    }
};



// TODO: BroadPhase, SpatialHashing
template <typename BE, typename PR>
struct SpatialHashing {};

template<typename PR>
struct SpatialHashing<METAL, PR> {

    // ─── GPU params structs (must match Metal side) ──────────────
    struct SHParams {
        uint32_t numTriangles;
        float    cellSize;
        float    invCellSize;
        int32_t  resX;
        int32_t  resY;
        int32_t  resZ;
        uint32_t tableSize;
        uint32_t maxNumCollisions;
        float    sceneMin[3];
        float    _pad;
    };

    struct SHTriEntry {
        uint32_t hashValue;
        uint32_t triIndex;
        uint32_t objId;
        uint32_t behavior;
        uint32_t shape;
        uint32_t controlBits;
        uint32_t isHome;
        uint32_t _pad;
    };

    struct SHSortKey {
        uint32_t hashValue;
        uint32_t index;
    };

    struct SHCellRange {
        uint32_t start;
        uint32_t end;
        uint32_t numHome;
        uint32_t numPhantom;
    };

    struct SHCollisionPassParams {
        uint32_t numCells;       // totalPairs for flat dispatch
        uint32_t passIndex;
        uint32_t maxNumCollisions;
        uint32_t numTriangles;   // repurposed as numWorkCells for flat dispatch
        float    radius;
        float    thickness;
    };

    struct SHCellWork {
        uint32_t numPairs;
        uint32_t cellType;
        uint32_t cellIdx;
    };

    struct SHRadixParams {
        uint32_t numElements;
        uint32_t shift;
        uint32_t numBlocks;
        uint32_t blockSize;
    };

    // ─── PSOs ───────────────────────────────────────────────────
    MTL::ComputePipelineState* computeBoundingSpheresPSO;
    MTL::ComputePipelineState* reduceMaxRadiusPSO;
    MTL::ComputePipelineState* reduceMaxRadiusFinalPSO;
    MTL::ComputePipelineState* assignCellsPSO;
    MTL::ComputePipelineState* compactEntriesPSO;
    MTL::ComputePipelineState* prefixSumBlocksPSO;
    MTL::ComputePipelineState* prefixSumAddBlockSumsPSO;
    MTL::ComputePipelineState* prefixSumExclusivePSO;
    MTL::ComputePipelineState* radixCountPSO;
    MTL::ComputePipelineState* radixComputeOffsetsPSO;
    MTL::ComputePipelineState* radixScatterPSO;
    MTL::ComputePipelineState* findCellRangesPSO;
    MTL::ComputePipelineState* countHomePhantomPSO;
    MTL::ComputePipelineState* collectActiveCellsPSO;
    MTL::ComputePipelineState* handlePhantomPhantomPSO;
    MTL::ComputePipelineState* collisionPassPSO;
    MTL::ComputePipelineState* countCollisionPairsPSO;
    MTL::ComputePipelineState* collisionPassFlatPSO;
    MTL::ComputePipelineState* clearCellRangesPSO;

    // ─── GPU buffers ────────────────────────────────────────────
    VectorBase<METAL, PR>       radii;           // per-triangle radius
    VectorBase<METAL, PR>       centroids;       // per-triangle centroid (3 floats packed as float3 = 16 bytes)
    VectorBase<METAL, PR>       maxRadiusPartials;
    VectorBase<METAL, PR>       maxRadiusResult; // single float

    VectorBase<METAL, SHTriEntry> entriesRaw;    // numTri * 8
    VectorBase<METAL, SHTriEntry> entriesCompact; // totalEntries after compact
    VectorBase<METAL, SHTriEntry> entriesTemp;    // for radix sort swap
    VectorBase<METAL, uint32_t>   entryCounts;   // per-triangle
    VectorBase<METAL, uint32_t>   entryOffsets;  // prefix sum of counts
    VectorBase<METAL, uint32_t>   totalEntries;  // single uint

    VectorBase<METAL, uint32_t>  triObjIds;      // per-triangle object id
    VectorBase<METAL, uint32_t>  triBehaviors;   // per-triangle behavior
    VectorBase<METAL, uint32_t>  triShapes;      // per-triangle shape

    VectorBase<METAL, uint32_t>  radixBlockHist;
    VectorBase<METAL, uint32_t>  radixBlockOff;
    VectorBase<METAL, uint32_t>  radixBucketBase;

    VectorBase<METAL, SHCellRange> cellRanges;
    VectorBase<METAL, uint32_t>    activeCells;
    VectorBase<METAL, uint32_t>    numActiveCells; // single uint (atomic)

    // Per-cell collision work
    VectorBase<METAL, SHCellWork>  cellWorkBuf;
    VectorBase<METAL, uint32_t>    pairOffsets;   // exclusive prefix sum of numPairs, size nActive+1

    // Lightweight sort key buffers
    VectorBase<METAL, SHSortKey> sortKeysA;
    VectorBase<METAL, SHSortKey> sortKeysB;
    VectorBase<METAL, SHTriEntry> entriesGathered; // reordered entries after key sort

    // Prefix-sum block sums
    VectorBase<METAL, uint32_t>  prefixBlockSums;

    static constexpr uint32_t RADIX_BLOCK_SIZE = 1024;
    float scaleFactor = 1.1f;
    Index numTotalTriangles = 0;
    bool useKeySorting = false; // toggle: true = lightweight key sort, false = full entry sort

    // ─── Profiling accumulators ─────────────────────────────────
    Index profileFrame = 0;
    static constexpr Index PROFILE_INTERVAL = 5;
    double acc_spheres = 0, acc_cpuGrid = 0, acc_clearAssign = 0;
    double acc_prefixSum = 0, acc_compact = 0, acc_radixSort = 0;
    double acc_cellBounds = 0, acc_phantom = 0, acc_8pass = 0;
    double acc_sortByVert = 0, acc_total = 0;

    SpatialHashing() {
        computeBoundingSpheresPSO = MetalKernelContext::getPSO("sh_computeBoundingSpheres");
        reduceMaxRadiusPSO        = MetalKernelContext::getPSO("sh_reduceMaxRadius");
        reduceMaxRadiusFinalPSO   = MetalKernelContext::getPSO("sh_reduceMaxRadiusFinal");
        assignCellsPSO            = MetalKernelContext::getPSO("sh_assignCells");
        compactEntriesPSO         = MetalKernelContext::getPSO("sh_compactEntries");
        prefixSumBlocksPSO        = MetalKernelContext::getPSO("sh_prefixSumBlocks");
        prefixSumAddBlockSumsPSO  = MetalKernelContext::getPSO("sh_prefixSumAddBlockSums");
        prefixSumExclusivePSO     = MetalKernelContext::getPSO("sh_prefixSumExclusive");
        radixCountPSO             = MetalKernelContext::getPSO("sh_radixCount");
        radixComputeOffsetsPSO    = MetalKernelContext::getPSO("sh_radixComputeOffsets");
        radixScatterPSO           = MetalKernelContext::getPSO("sh_radixScatter");
        findCellRangesPSO         = MetalKernelContext::getPSO("sh_findCellRanges");
        countHomePhantomPSO       = MetalKernelContext::getPSO("sh_countHomePhantom");
        collectActiveCellsPSO     = MetalKernelContext::getPSO("sh_collectActiveCells");
        handlePhantomPhantomPSO   = MetalKernelContext::getPSO("sh_handlePhantomPhantom");
        collisionPassPSO          = MetalKernelContext::getPSO("sh_collisionPass");
        countCollisionPairsPSO    = MetalKernelContext::getPSO("sh_countCollisionPairs");
        collisionPassFlatPSO      = MetalKernelContext::getPSO("sh_collisionPassFlat");
        clearCellRangesPSO        = MetalKernelContext::getPSO("sh_clearCellRanges");
    }

    void resetMemory() {
        radii = {};
        centroids = {};
        maxRadiusPartials = {};
        maxRadiusResult = {};
        entriesRaw = {};
        entriesCompact = {};
        entriesTemp = {};
        entryCounts = {};
        entryOffsets = {};
        totalEntries = {};
        triObjIds = {};
        triBehaviors = {};
        triShapes = {};
        radixBlockHist = {};
        radixBlockOff = {};
        radixBucketBase = {};
        cellRanges = {};
        activeCells = {};
        numActiveCells = {};
        cellWorkBuf = {};
        pairOffsets = {};
        prefixBlockSums = {};
        sortKeysA = {};
        sortKeysB = {};
        entriesGathered = {};
        numTotalTriangles = 0;
    }

    void build(Scene<METAL, PR>& scene) {
        if (scene.numMeshes == 0) {
            resetMemory();
            return;
        }

        // Count total triangles across all meshes and build per-triangle metadata
        typename Scene<METAL, PR>::PackedMeshData& pm = Scene<METAL, PR>::packedMeshData;
        Index totalTris = pm.facetsOffsets[scene.numMeshes];

        if (totalTris == 0) {
            resetMemory();
            return;
        }

        // Check if we need reallocation
        if (numTotalTriangles != totalTris) {
            resetMemory();
            numTotalTriangles = totalTris;
            allocateBuffers(scene);
        }

        // Fill per-triangle object info (CPU side, since mesh count is small)
        for (Index i = 0; i < scene.numMeshes; i++) {
            auto& mesh = scene.meshes[i];
            Index facetStart = pm.facetsOffsets[i];
            Index facetEnd   = pm.facetsOffsets[i + 1];
            for (Index t = facetStart; t < facetEnd; t++) {
                triObjIds[t]    = mesh.id;
                triBehaviors[t] = (uint32_t)mesh.behaviorType;
                triShapes[t]    = (uint32_t)mesh.shapeType;
            }
        }
    }

    void refit() {
        // Spatial hashing is rebuilt each frame, nothing to refit.
        // The actual grid construction happens in detectCollisions.
    }

    void detectCollisions(PR margin, bool enableSelfCollisions = true, PR radius = 0.012f, PR thickness = 0.0f) {
        if (numTotalTriangles == 0) return;

        typename Scene<METAL, PR>::PackedMeshData& pm = Scene<METAL, PR>::packedMeshData;
        typename Scene<METAL, PR>::PackedCollisionData& pc = Scene<METAL, PR>::packedCollisionData;

        pc.numNarrowCollisions[0] = 0;

        auto now = []() { return std::chrono::high_resolution_clock::now(); };
        auto ms = [](auto start, auto end) {
            return std::chrono::duration<double, std::milli>(end - start).count();
        };
        auto t0 = now();

        std::cout<< "[SH] start detection\n";
        // ── Step 1: Compute bounding spheres ──
        // buf layout: 0=radii, 1=centroids, 2=numTris, 3=pm.x, 4=pm.facets, 5=triObjIds, 6=statesOffsets
        uint32_t numTris = (uint32_t)numTotalTriangles;
        MetalGlobalContext::setBuffer(radii, 0);
        MetalGlobalContext::setBuffer(centroids, 1);
        MetalGlobalContext::setBytes(numTris, 2);
        MetalGlobalContext::setBuffer(pm.x, 3);
        MetalGlobalContext::setBuffer(pm.facets, 4);
        MetalGlobalContext::setBuffer(triObjIds, 5);
        MetalGlobalContext::setBuffer(pm.statesOffsets, 6);
        MetalGlobalContext::dispatchThreads(computeBoundingSpheresPSO, numTris);

        // ── Step 2: Find max radius via reduction ──
        // buf0=radii(keep), buf2=numTris(keep), only set buf1
        uint32_t numGroups = (numTris + 255) / 256;
        MetalGlobalContext::setBuffer(maxRadiusPartials, 1);
        MetalGlobalContext::dispatchThreads(reduceMaxRadiusPSO, numGroups * 256, 256);

        // buf1=maxRadiusPartials(keep), only set buf0, buf2
        MetalGlobalContext::setBuffer(maxRadiusResult, 0);
        MetalGlobalContext::setBytes(numGroups, 2);
        MetalGlobalContext::dispatchThreads(reduceMaxRadiusFinalPSO, 256, 256);

        MetalGlobalContext::commitAndWait();
        auto t1 = now();

        // ── Step 3: Compute cell size and grid resolution ──
        float maxR = maxRadiusResult[0];
        float cellSize = 2.0f * maxR * scaleFactor;
        if (cellSize < 1e-8f) cellSize = 1e-8f;
        float invCellSize = 1.0f / cellSize;

        // Compute scene AABB for resolution
        // Simple CPU scan over packed positions
        float xmin = 1e30f, ymin = 1e30f, zmin = 1e30f;
        float xmax = -1e30f, ymax = -1e30f, zmax = -1e30f;
        for (Index i = 0; i < pm.x.size / 3; i++) {
            float px = pm.x[i * 3];
            float py = pm.x[i * 3 + 1];
            float pz = pm.x[i * 3 + 2];
            if (px < xmin) xmin = px; if (px > xmax) xmax = px;
            if (py < ymin) ymin = py; if (py > ymax) ymax = py;
            if (pz < zmin) zmin = pz; if (pz > zmax) zmax = pz;
        }

        int32_t resX = std::max(1, (int32_t)std::ceil((xmax - xmin) / cellSize) + 2);
        int32_t resY = std::max(1, (int32_t)std::ceil((ymax - ymin) / cellSize) + 2);
        int32_t resZ = std::max(1, (int32_t)std::ceil((zmax - zmin) / cellSize) + 2);

        uint32_t tableSize = (uint32_t)(resX * resY * resZ);

        // Reallocate cellRanges if table size changed
        if (!cellRanges.ptr || cellRanges.size < tableSize) {
            cellRanges = VectorBase<METAL, SHCellRange>(tableSize);
        }

        SHParams shParams = {
            numTris, cellSize, invCellSize,
            resX, resY, resZ,
            tableSize,
            pc.maxNumCollisions,
            {xmin, ymin, zmin},
            0.0f
        };

        std::cout << "[SH] cellSize=" << cellSize << " maxR=" << maxR
                  << " grid=" << resX << "x" << resY << "x" << resZ
                  << " tableSize=" << tableSize << " numTris=" << numTris << "\n";

        auto t2 = now();

        // ── Clear cell ranges ──
        MetalGlobalContext::setBuffer(cellRanges, 0);
        MetalGlobalContext::setBytes(tableSize, 1);
        MetalGlobalContext::dispatchThreads(clearCellRangesPSO, tableSize);

        // ── Step 4: Assign cells (home + phantom entries) ──
        MetalGlobalContext::setBuffer(centroids, 0);
        MetalGlobalContext::setBuffer(radii, 1);
        MetalGlobalContext::setBytes(shParams, 2);
        MetalGlobalContext::setBuffer(entriesRaw, 3);
        MetalGlobalContext::setBuffer(entryCounts, 4);
        MetalGlobalContext::setBuffer(triObjIds, 5);
        MetalGlobalContext::setBuffer(triBehaviors, 6);
        MetalGlobalContext::setBuffer(triShapes, 7);
        MetalGlobalContext::dispatchThreads(assignCellsPSO, numTris);

        MetalGlobalContext::commitAndWait();
        auto t3 = now();

        // ── Step 5: Prefix sum on entryCounts to get offsets ──
        // Copy counts to offsets for in-place scan
        std::memcpy(entryOffsets.ptr, entryCounts.ptr, sizeof(uint32_t) * numTris);

        if (numTris <= 512) {
            MetalGlobalContext::setBuffer(entryOffsets, 0);
            MetalGlobalContext::setBuffer(totalEntries, 1);
            MetalGlobalContext::setBytes(numTris, 2);
            MetalGlobalContext::dispatchThreads(prefixSumExclusivePSO, 256, 256);
            MetalGlobalContext::commitAndWait();
        } else {
            // Multi-block prefix sum
            uint32_t numScanBlocks = (numTris + 511) / 512;
            if (!prefixBlockSums.ptr || prefixBlockSums.size < numScanBlocks) {
                prefixBlockSums = VectorBase<METAL, uint32_t>(numScanBlocks);
            }

            MetalGlobalContext::setBuffer(entryOffsets, 0);
            MetalGlobalContext::setBuffer(prefixBlockSums, 1);
            MetalGlobalContext::setBytes(numTris, 2);
            MetalGlobalContext::dispatchThreads(prefixSumBlocksPSO, numScanBlocks * 256, 256);
            MetalGlobalContext::commitAndWait();

            // Scan block sums on CPU (small array)
            uint32_t runningSum = 0;
            for (uint32_t b = 0; b < numScanBlocks; b++) {
                uint32_t v = prefixBlockSums[b];
                prefixBlockSums[b] = runningSum;
                runningSum += v;
            }
            totalEntries[0] = runningSum;

            // Add block sums back
            MetalGlobalContext::setBuffer(entryOffsets, 0);
            MetalGlobalContext::setBuffer(prefixBlockSums, 1);
            MetalGlobalContext::setBytes(numTris, 2);
            MetalGlobalContext::dispatchThreads(prefixSumAddBlockSumsPSO, numTris);
            MetalGlobalContext::commitAndWait();
        }

        auto t4 = now();

        uint32_t numEntries = totalEntries[0];
        if (numEntries == 0) return;

        // Ensure compact buffer is large enough
        if (!entriesCompact.ptr || entriesCompact.size < numEntries) {
            entriesCompact = VectorBase<METAL, SHTriEntry>(numEntries);
            entriesTemp    = VectorBase<METAL, SHTriEntry>(numEntries);
        }

        // ── Step 6: Compact entries ──
        MetalGlobalContext::setBuffer(entriesRaw, 0);
        MetalGlobalContext::setBuffer(entryOffsets, 1);
        MetalGlobalContext::setBuffer(entryCounts, 2);
        MetalGlobalContext::setBuffer(entriesCompact, 3);
        MetalGlobalContext::setBytes(numTris, 4);
        MetalGlobalContext::dispatchThreads(compactEntriesPSO, numTris);

        MetalGlobalContext::commitAndWait();
        auto t5_compact = now();

        // ── Step 7: Radix sort by hashValue ──
        radixSort(numEntries);

        MetalGlobalContext::commitAndWait();
        auto t5_sort = now();

        // ── Step 8: Find cell boundaries ──
        // buf layout: 0=entriesCompact, 1=cellRanges, 2=numEntries
        MetalGlobalContext::setBuffer(entriesCompact, 0);
        MetalGlobalContext::setBuffer(cellRanges, 1);
        MetalGlobalContext::setBytes(numEntries, 2);
        MetalGlobalContext::dispatchThreads(findCellRangesPSO, numEntries);

        // ── Collect active cells ──
        // buf0=entriesCompact(keep), buf2=numEntries(keep), only set buf1, buf3
        numActiveCells[0] = 0;
        MetalGlobalContext::setBuffer(activeCells, 1);
        MetalGlobalContext::setBuffer(numActiveCells, 3);
        MetalGlobalContext::dispatchThreads(collectActiveCellsPSO, numEntries);

        MetalGlobalContext::commitAndWait();
        auto t5 = now();

        uint32_t nActive = numActiveCells[0];
        if (nActive == 0) return;

        std::cout << "[SH] numEntries=" << numEntries << " activeCells=" << nActive << "\n";

        // ── Count home/phantom + phantom handling ──
        MetalGlobalContext::setBuffer(entriesCompact, 0);
        MetalGlobalContext::setBuffer(cellRanges, 1);
        MetalGlobalContext::setBuffer(activeCells, 2);
        MetalGlobalContext::setBytes(nActive, 3);
        MetalGlobalContext::dispatchThreads(countHomePhantomPSO, nActive);
        MetalGlobalContext::dispatchThreads(handlePhantomPhantomPSO, nActive);
        MetalGlobalContext::dispatchThreads(countHomePhantomPSO, nActive);

        MetalGlobalContext::commitAndWait();
        auto t6 = now();

        // ── Step 10: Count collision pairs per cell ──
        if (!cellWorkBuf.ptr || cellWorkBuf.size < nActive) {
            cellWorkBuf = VectorBase<METAL, SHCellWork>(nActive);
            pairOffsets = VectorBase<METAL, uint32_t>(nActive + 1);
        }

        // New encoder after commitAndWait — must rebind all buffers
        MetalGlobalContext::setBuffer(entriesCompact, 0);
        MetalGlobalContext::setBuffer(cellRanges, 1);
        MetalGlobalContext::setBuffer(activeCells, 2);
        MetalGlobalContext::setBytes(nActive, 3);
        MetalGlobalContext::setBuffer(cellWorkBuf, 4);
        MetalGlobalContext::dispatchThreads(countCollisionPairsPSO, nActive);

        MetalGlobalContext::commitAndWait();

        // CPU prefix sum on cellWork to build pairOffsets (per-pass)
        pairOffsets[0] = 0;
        for (uint32_t i = 0; i < nActive; i++) {
            pairOffsets[i + 1] = pairOffsets[i] + cellWorkBuf[i].numPairs;
        }
        uint32_t totalPairs = pairOffsets[nActive];

        // Debug: per-cellType pair counts
        uint32_t pairsPerType[8] = {};
        for (uint32_t i = 0; i < nActive; i++) {
            uint32_t ct = cellWorkBuf[i].cellType;
            if (ct < 8) pairsPerType[ct] += cellWorkBuf[i].numPairs;
        }
        std::cout << "[SH] totalPairs=" << totalPairs
                  << " perType=[";
        for (int t = 0; t < 8; t++) std::cout << pairsPerType[t] << (t<7?",":"");
        std::cout << "]\n";

        // ── Step 11: 8-pass flat collision detection (inline narrow phase) ──
        pc.resetNarrow();
        if (totalPairs > 0) {
            MetalGlobalContext::setBuffer(entriesCompact, 0);
            MetalGlobalContext::setBuffer(cellRanges, 1);
            MetalGlobalContext::setBuffer(cellWorkBuf, 2);
            MetalGlobalContext::setBuffer(pc.narrowCollisions, 4);
            MetalGlobalContext::setBuffer(pc.numNarrowCollisions, 5);
            MetalGlobalContext::setBuffer(pm.x, 6);
            MetalGlobalContext::setBuffer(pm.facets, 7);
            MetalGlobalContext::setBuffer(pm.statesOffsets, 8);
            MetalGlobalContext::setBuffer(pm.facetsOffsets, 9);
            MetalGlobalContext::setBuffer(pm.vertexAdjFacets, 10);
            MetalGlobalContext::setBuffer(pm.vertexAdjFacetsOffsets, 11);
            MetalGlobalContext::setBuffer(pairOffsets, 12);

            for (uint32_t pass = 0; pass < 8; pass++) {
                SHCollisionPassParams passParams = {
                    totalPairs,       // numCells = total pairs to dispatch
                    pass,
                    pc.maxNumCollisions,
                    nActive,          // numTriangles repurposed as numWorkCells for binary search
                    radius,
                    thickness
                };
                MetalGlobalContext::setBytes(passParams, 3);
                MetalGlobalContext::dispatchThreads(collisionPassFlatPSO, totalPairs);
            }
        }

        MetalGlobalContext::commitAndWait();
        auto t7 = now();

        // ── Sort narrow collisions by vertex ──
        sortNarrowByVertices();
        auto t8 = now();

        acc_spheres    += ms(t0,t1);
        acc_cpuGrid    += ms(t1,t2);
        acc_clearAssign+= ms(t2,t3);
        acc_prefixSum  += ms(t3,t4);
        acc_compact    += ms(t4,t5_compact);
        acc_radixSort  += ms(t5_compact,t5_sort);
        acc_cellBounds += ms(t5_sort,t5);
        acc_phantom    += ms(t5,t6);
        acc_8pass      += ms(t6,t7);
        acc_sortByVert += ms(t7,t8);
        acc_total      += ms(t0,t8);
        profileFrame++;

        if (profileFrame >= PROFILE_INTERVAL) {
            double n = (double)PROFILE_INTERVAL;
            std::cout << "[SH avg/" << PROFILE_INTERVAL << "f"
                      << (useKeySorting ? " KEY" : " FULL") << "]"
                      << "  spheres=" << acc_spheres/n
                      << "  cpuGrid=" << acc_cpuGrid/n
                      << "  clr+assign=" << acc_clearAssign/n
                      << "  prefixSum=" << acc_prefixSum/n
                      << "  compact=" << acc_compact/n
                      << "  radixSort=" << acc_radixSort/n
                      << "  cellBounds=" << acc_cellBounds/n
                      << "  phantom=" << acc_phantom/n
                      << "  8-pass=" << acc_8pass/n
                      << "  sortVert=" << acc_sortByVert/n
                      << "  TOTAL=" << acc_total/n << " ms\n";
            std::cout << "[SH] Narrow: " << pc.numNarrowCollisions[0] << "\n";
            acc_spheres = acc_cpuGrid = acc_clearAssign = acc_prefixSum = 0;
            acc_compact = acc_radixSort = acc_cellBounds = acc_phantom = 0;
            acc_8pass = acc_sortByVert = acc_total = 0;
            profileFrame = 0;
        }
        std::cout<< "[SH] finish detection\n";
    }

    void showBox() {
        // Spatial hashing has no persistent tree structure to visualize.
    }

    void showSceneBox() {
        // No scene-level bounding box.
    }

    void queryClickRay(const Ray& ray) {
        // Not implemented for spatial hashing.
    }

    void sortNarrowByVertices() {
        typename Scene<METAL, PR>::PackedMeshData& packedMesh = Scene<METAL, PR>::packedMeshData;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;

        if (packedCol.numNarrowCollisions[0] == 0) return;

        packedCol.vertColFacets.map().setZero();
        packedCol.vertColFacetsOffsets.map().setZero();

        for (Index i = 0; i < packedCol.numNarrowCollisions[0]; ++i) {
            NarrowCollision& nc = packedCol.narrowCollisions[i];
            Index pid = nc.indexPair.point;
            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + pid;
            packedCol.vertColFacetsOffsets[ppid + 1]++;
        }

        for (Index i = 1; i < packedCol.vertColFacetsOffsets.size; ++i)
            packedCol.vertColFacetsOffsets[i] += packedCol.vertColFacetsOffsets[i - 1];

        DynamicMemoryAllocator<METAL> tempPool;
        VectorBase<METAL, Index> offsets(tempPool.template zeros<Index>(packedCol.vertColFacetsOffsets.size - 1));

        for (Index i = 0; i < packedCol.numNarrowCollisions[0]; ++i) {
            NarrowCollision& nc = packedCol.narrowCollisions[i];
            Index pid = nc.indexPair.point;
            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + pid;
            Index colid = packedCol.vertColFacetsOffsets[ppid] + offsets[ppid];
            packedCol.vertColFacets[colid] = packedCol.narrowCollisions[i];
            offsets[ppid]++;
        }
    }

private:
    void allocateBuffers(Scene<METAL, PR>& scene) {
        Index n = numTotalTriangles;
        uint32_t numGroups = ((uint32_t)n + 255) / 256;

        radii            = VectorBase<METAL, PR>(n);
        centroids        = VectorBase<METAL, PR>(n * 3); // packed_float3 = 12 bytes = 3 floats
        maxRadiusPartials = VectorBase<METAL, PR>(numGroups);
        maxRadiusResult  = VectorBase<METAL, PR>(1);

        entriesRaw   = VectorBase<METAL, SHTriEntry>(n * 8);
        entryCounts  = VectorBase<METAL, uint32_t>(n);
        entryOffsets = VectorBase<METAL, uint32_t>(n);
        totalEntries = VectorBase<METAL, uint32_t>(1);

        triObjIds    = VectorBase<METAL, uint32_t>(n);
        triBehaviors = VectorBase<METAL, uint32_t>(n);
        triShapes    = VectorBase<METAL, uint32_t>(n);

        // Radix sort buffers (sized for max possible entries = n*8)
        uint32_t maxEntries = (uint32_t)(n * 8);
        uint32_t numBlocks = (maxEntries + RADIX_BLOCK_SIZE - 1) / RADIX_BLOCK_SIZE;
        radixBlockHist  = VectorBase<METAL, uint32_t>(numBlocks * 256);
        radixBlockOff   = VectorBase<METAL, uint32_t>(numBlocks * 256);
        radixBucketBase = VectorBase<METAL, uint32_t>(256);

        // Active cells buffer (worst case = all entries have unique hashes)
        activeCells    = VectorBase<METAL, uint32_t>(maxEntries);
        numActiveCells = VectorBase<METAL, uint32_t>(1);
    }

    void radixSort(uint32_t numEntries) {
        uint32_t numBlocks = (numEntries + RADIX_BLOCK_SIZE - 1) / RADIX_BLOCK_SIZE;

        VectorBase<METAL, SHTriEntry>* src = &entriesCompact;
        VectorBase<METAL, SHTriEntry>* dst = &entriesTemp;

        SHRadixParams params;
        params.numElements = numEntries;
        params.numBlocks = numBlocks;
        params.blockSize = RADIX_BLOCK_SIZE;

        // Unified layout: buf0=src, buf1=dst, buf2=params, buf3=blockHist, buf4=blockOff, buf5=bucketBase
        // buf3-5 are constant across all iterations, set once
        MetalGlobalContext::setBuffer(radixBlockHist, 3);
        MetalGlobalContext::setBuffer(radixBlockOff, 4);
        MetalGlobalContext::setBuffer(radixBucketBase, 5);

        for (uint32_t shift = 0; shift < 32; shift += 8) {
            params.shift = shift;

            // Count: needs buf0=src, buf2=params, buf3=blockHist(kept)
            MetalGlobalContext::setBuffer(*src, 0);
            MetalGlobalContext::setBytes(params, 2);
            MetalGlobalContext::dispatchThreads(radixCountPSO, numBlocks * RADIX_BLOCK_SIZE, RADIX_BLOCK_SIZE);

            // Offsets: buf2=params(kept), buf3=blockHist(kept), buf4=blockOff(kept), buf5=bucketBase(kept)
            MetalGlobalContext::dispatchThreads(radixComputeOffsetsPSO, 1);

            // Scatter: buf0=src(kept), buf2=params(kept), buf4=blockOff(kept), buf5=bucketBase(kept), set buf1=dst
            MetalGlobalContext::setBuffer(*dst, 1);
            MetalGlobalContext::dispatchThreads(radixScatterPSO, numBlocks * RADIX_BLOCK_SIZE, RADIX_BLOCK_SIZE);

            std::swap(src, dst);
        }

        // After 4 passes (even number), result is back in src.
        if (src != &entriesCompact) {
            std::swap(entriesCompact, entriesTemp);
        }
    }

};



// TODO: BroadPhase, BVH
template <typename BE, typename PR, Index MODE, Index PRIMITIVE>
struct BVH {};
enum BVHMODE {
    SCENE,
    LINEAR,
    SAH,
};
enum BVHPRIMITIVE {
    OBJECT = 0,
    POINT = 1,
    EDGE = 2,
    TRIANGLE = 3,
};

//struct AABB {
//    tinym::vec3 min, max;
//    AABB() : min(0), max(0) {}
//    AABB(tinym::vec3_view e1, tinym::vec3_view e2) : min(tinym::min(e1, e2)), max(tinym::max(e1, e2)) {}
//    AABB(tinym::vec3_view t1, tinym::vec3_view t2, tinym::vec3_view t3) : min(tinym::min(t1, t2, t3)), max(tinym::max(t1, t2, t3)) {}
//    void combine(const tinym::vec3_view& a) {
//        min = tinym::min(a, min);
//        max = tinym::max(a, max);
//    }
//    void combine(const AABB& aabb) {
//        min = tinym::min(min, aabb.min);
//        max = tinym::max(max, aabb.max);
//    }
//    bool intersect(const AABB& aabb) const {
//        if (max.x < aabb.min.x || min.x > aabb.max.x) return false;
//        if (max.y < aabb.min.y || min.y > aabb.max.y) return false;
//        if (max.z < aabb.min.z || min.z > aabb.max.z) return false;
//        return true;
//    }
//};


struct alignas(32) AABB4 {
    union {
        struct {
            tinym::vec3f1i v0, v1;
        };
        struct {
            tinym::vec3 min;
            int i0;
            tinym::vec3 max;
            int i1;
        };
    };
    AABB4() : v0(0), v1(0) {}
    AABB4(tinym::vec3_view e0, tinym::vec3_view e1) : min(tinym::min(e0, e1)), i0(0), max(tinym::max(e0, e1)), i1(0) {}
    AABB4(tinym::vec3_view t0, tinym::vec3_view t1, tinym::vec3_view t2) : min(tinym::min(t0, t1, t2)), i0(0), max(tinym::max(t0, t1, t2)), i1(0) {}
    void combine(const tinym::vec3_view& v) {
        min = tinym::min(v, min);
        max = tinym::max(v, max);
        return;
    }
    void combine(const AABB4& aabb) {
        min = tinym::min(min, aabb.min);
        max = tinym::max(max, aabb.max);
        return;
    }
    bool intersect(const AABB4& aabb) const {
        if (max.x < aabb.min.x || min.x > aabb.max.x) return false;
        if (max.y < aabb.min.y || min.y > aabb.max.y) return false;
        if (max.z < aabb.min.z || min.z > aabb.max.z) return false;
        return true;
    }
    bool intersect(const Ray& ray, RayHit& hit) const {
        float tmin = 0.0f;
        float tmax = std::numeric_limits<float>::infinity();
        const float eps = 1e-6f;

        for (int axis = 0; axis < 3; ++axis) {
            float o = ray.origin[axis];
            float d = ray.dir[axis];

            if (std::abs(d) < eps) {
                // 이 축에 대해 ray가 평행
                if (o < min[axis] || o > max[axis]) return false;
            } else {
                float t1 = (min[axis] - o) / d;
                float t2 = (max[axis] - o) / d;

                if (t1 > t2) std::swap(t1, t2);

                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);

                if (tmin > tmax) return false;
            }
        }

        hit.tmin = tmin;
        hit.tmax = tmax;

        return tmax >= 0.0f;
    }
};
static_assert(sizeof(AABB4) == 32);


// TODO: BroadPhase, LBVH
template <typename BE, typename PR, Index PRIMITIVE>
struct BVH<BE, PR, BVHMODE::LINEAR, PRIMITIVE> {
    struct alignas(8) MortonNode {
        uint code; // Morton code, 32bit, 10bit per coordinate
        uint index; // obj index ( primitive index
    };
    static_assert(sizeof(MortonNode) == 8);
    struct BVHNode {
        union {
            AABB4 aabb;
            struct alignas(32) {
                tinym::vec3 min;
                int childA; // -2 if leaf, -1 if no child, nonnegative if intermediate
                tinym::vec3 max;
                int childB; // -1 if no child, nonnegative if intermediate
            };
        };
        //int childA = -1, childB = -1;
        //int pid = -1;
        // constructor for leaf nodes.
        BVHNode(MortonNode& mortonNode, VectorBase<METAL, PR>& pos, VectorBase<METAL, Index>& prim) {
            Index base = mortonNode.index*PRIMITIVE;
            if constexpr (PRIMITIVE == 2) {
                Index vid0 = prim[base], vid1 = prim[base+1];
                aabb = AABB4(pos.ptr+vid0*3, pos.ptr+vid1*3);
            } else if constexpr (PRIMITIVE == 3) {
                Index vid0 = prim[base], vid1 = prim[base+1], vid2 = prim[base+2];
                aabb = AABB4(pos.ptr+vid0*3, pos.ptr+vid1*3, pos.ptr+vid2*3);
            }
            else aabb = AABB4();
            childA = -1;
            childB = mortonNode.index;
        }
        // constructor for intermediate nodes. Their AABBs are combined lazily.
        BVHNode(int childA, int childB) : childA(childA), childB(childB) {}
    };
    static_assert(sizeof(BVHNode) == 32);

    VectorBase<METAL, PR> positions; // Length 3*N, N is the number of points
    VectorBase<METAL, PR> velocities;
    VectorBase<METAL, Index> primitives; // Length PRIMITIVE*M, M is the number of primitives.
    VectorBase<METAL, MortonNode> mortons; // Length M, turn the each centers of the primitives into MortonNodes
    VectorBase<METAL, MortonNode> mortonsTemp; // Length M, turn the each centers of the primitives into MortonNodes
    VectorBase<METAL, BVHNode> tree; // Length 2*M-1, M leaf nodes, M-1 intermediate nodes.
    VectorBase<METAL, int> treeParent; // Length 2*M-1, M leaf nodes, M-1 intermediate nodes.
    //VectorBase<METAL, BroadCollision> collisions;
    //Index numCollisions = 0, maxNumCollisions;

    VectorBase<METAL, uint32_t> radixBlockHistograms; // numBlocks * 256
    VectorBase<METAL, uint32_t> radixBlockOffsets;    // numBlocks * 256
    VectorBase<METAL, uint32_t> radixBucketBase;      // 256

    VectorBase<METAL, uint32_t> bottomUpReadyA;       // numNodes
    VectorBase<METAL, uint32_t> bottomUpReadyB;       // numNodes
    VectorBase<METAL, uint32_t> bottomUpProgress;     // 1

    int objid; // who made this tree
    VectorBase<METAL, int> objIds;
    BehaviorType objBehavior;
    VectorBase<METAL, BehaviorType> objBehaviors;
    ShapeType objShape;
    VectorBase<METAL, ShapeType> objShapes;

    MTL::ComputePipelineState* fillMortonsPSO;
    MTL::ComputePipelineState* buildTreePSO;
    MTL::ComputePipelineState* buildLeafPSO;
    MTL::ComputePipelineState* bottomUpBoxesPSO;
    MTL::ComputePipelineState* queryPointsPSO;

    MTL::ComputePipelineState* radixCountBlocksPSO;
    MTL::ComputePipelineState* radixComputeOffsetsPSO;
    MTL::ComputePipelineState* radixScatterBlocksPSO;

    MTL::ComputePipelineState* initBottomUpReadyPSO;
    MTL::ComputePipelineState* clearBottomUpProgressPSO;
    MTL::ComputePipelineState* bottomUpCombineStepPSO;

    // Debuggings...
    DebugLineGL<CPU> debugBox;
    VectorBase<METAL, PR> debugBoxLines;

    struct QueryFlag {
        uint32_t stackOverflow;
        uint32_t collisionOverflow;
    };
    VectorBase<METAL, QueryFlag> qFlag;


    BVH() {
        // recieve all points, facets and edges;
        if constexpr (PRIMITIVE == BVHPRIMITIVE::TRIANGLE) {
            fillMortonsPSO = MetalKernelContext::getPSO("fillMortons_Tri");
            buildTreePSO = MetalKernelContext::getPSO("buildTree_Tri");
            buildLeafPSO = MetalKernelContext::getPSO("buildLeaf_Tri");
        } else if constexpr (PRIMITIVE == BVHPRIMITIVE::EDGE) {
            fillMortonsPSO = MetalKernelContext::getPSO("fillMortons_Edge");
            buildTreePSO = MetalKernelContext::getPSO("buildTree_Edge");
            buildLeafPSO = MetalKernelContext::getPSO("buildLeaf_Edge");
        }
        bottomUpBoxesPSO = MetalKernelContext::getPSO("bottomUpBoxes");
        queryPointsPSO = MetalKernelContext::getPSO("queryPoints");

        radixCountBlocksPSO     = MetalKernelContext::getPSO("radixCountMortonBlocks");
        radixComputeOffsetsPSO  = MetalKernelContext::getPSO("radixComputeOffsets");
        radixScatterBlocksPSO   = MetalKernelContext::getPSO("radixScatterMortonBlocks");

        initBottomUpReadyPSO    = MetalKernelContext::getPSO("initBottomUpReady");
        clearBottomUpProgressPSO= MetalKernelContext::getPSO("clearBottomUpProgress");
        bottomUpCombineStepPSO  = MetalKernelContext::getPSO("bottomUpCombineStep");
    }

    void resetMemory() {
        debugBox.release();
        debugBoxLines = {};

        positions = {};
        velocities = {};
        primitives = {};
        mortons = {};
        mortonsTemp = {};
        tree = {};
        treeParent = {};

        radixBlockHistograms = {};
        radixBlockOffsets = {};
        radixBucketBase = {};

        bottomUpReadyA = {};
        bottomUpReadyB = {};
        bottomUpProgress = {};

        objid = -1;
        objIds = {};
        objBehavior = BehaviorType::Float;
        objBehaviors = {};
        objShape = ShapeType::Mesh;
        objShapes = {};
        qFlag = {};
    }

    void memoryAllocation() {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) {
            return;
        }
        Index numNodes = 2 * numPrimitives - 1;
        Index numBlocks = (numPrimitives + 255) / 256;

        mortons = VectorBase<METAL, MortonNode>(numPrimitives);
        mortonsTemp = VectorBase<METAL, MortonNode>(numPrimitives);
        tree = VectorBase<METAL, BVHNode>(numNodes);
        treeParent = VectorBase<METAL, int>(numNodes);
        qFlag = VectorBase<METAL, QueryFlag>(1);

        radixBlockHistograms = VectorBase<METAL, uint32_t>(numBlocks * 256);
        radixBlockOffsets    = VectorBase<METAL, uint32_t>(numBlocks * 256);
        radixBucketBase      = VectorBase<METAL, uint32_t>(256);

        bottomUpReadyA       = VectorBase<METAL, uint32_t>(numNodes);
        bottomUpReadyB       = VectorBase<METAL, uint32_t>(numNodes);
        bottomUpProgress     = VectorBase<METAL, uint32_t>(1);
    }

    bool needsMemoryAllocation(Index numPrimitives) const {
        if (numPrimitives == 0) return false;
        Index numNodes = 2 * numPrimitives - 1;
        Index numBlocks = (numPrimitives + 255) / 256;
        return !tree.ptr ||
               mortons.size != numPrimitives ||
               mortonsTemp.size != numPrimitives ||
               tree.size != numNodes ||
               treeParent.size != numNodes ||
               radixBlockHistograms.size != numBlocks * 256 ||
               radixBlockOffsets.size != numBlocks * 256 ||
               radixBucketBase.size != 256 ||
               bottomUpReadyA.size != numNodes ||
               bottomUpReadyB.size != numNodes ||
               bottomUpProgress.size != 1 ||
               qFlag.size != 1;
    }

    void build(GeneralMesh<METAL, PR>& mesh) {
        build(mesh.id, mesh.state.x, mesh.adjacency.facets);
    }

    struct RadixSortParamsCPU {
        uint32_t numElements;
        uint32_t shift;
        uint32_t numBlocks;
    };
    struct BottomUpParamsCPU {
        uint32_t numPrimitives;
        uint32_t numNodes;
    };
    

    void bottomUpCombineGPU() {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;

        Index numNodes = 2 * numPrimitives - 1;
        BottomUpParamsCPU params = {
            (uint32_t)numPrimitives,
            (uint32_t)numNodes
        };

        // init ready buffers
        MetalGlobalContext::setBytes(params, 0);
        MetalGlobalContext::setBuffer(bottomUpReadyA, 1);
        MetalGlobalContext::setBuffer(bottomUpReadyB, 2);
        MetalGlobalContext::dispatchThreads(initBottomUpReadyPSO, numNodes);
        //MetalGlobalContext::commitAndWait();

        auto* readyCur = &bottomUpReadyA;
        auto* readyNext = &bottomUpReadyB;

        // worst case: skewed tree면 internal node 수만큼 반복 가능
        int depth = std::log2(numPrimitives)+1;
        for (Index iter = 0; iter < depth; ++iter) {
            // progress = 0
            MetalGlobalContext::setBuffer(bottomUpProgress, 0);
            MetalGlobalContext::dispatchThreads(clearBottomUpProgressPSO, 1);
            //MetalGlobalContext::commitAndWait();

            // one combine step
            MetalGlobalContext::setBytes(params, 0);
            MetalGlobalContext::setBuffer(tree, 1);
            MetalGlobalContext::setBuffer(*readyCur, 2);
            MetalGlobalContext::setBuffer(*readyNext, 3);
            MetalGlobalContext::setBuffer(bottomUpProgress, 4);
            MetalGlobalContext::dispatchThreads(bottomUpCombineStepPSO, numNodes);
            //MetalGlobalContext::commitAndWait();

            // root ready?
            //if ((*readyNext)[0]) {
            //    break;
            //}

            // no progress -> something is wrong
            //if (bottomUpProgress[0] == 0) {
            //    std::cout << "[bottomUpCombineGPU] no progress; tree may be invalid\n";
            //    break;
            //}

            std::swap(readyCur, readyNext);
        }
    }


    void buildLeafGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;

        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(numPrimitives, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);

        MetalGlobalContext::dispatchThreads(buildLeafPSO, numPrimitives);
    }
    void buildTreeGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;

        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(numPrimitives, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);

        MetalGlobalContext::dispatchThreads(buildTreePSO, numPrimitives);
    }

    void radixSortCPU() {
        Index numPrimitives = primitives.size/PRIMITIVE;
        auto radixSortByMortonCode = [](MortonNode* in, MortonNode* tmp, size_t n) {
            constexpr int BITS_PER_PASS = 8;
            constexpr int RADIX = 1 << BITS_PER_PASS; // 256
            constexpr int MASK = RADIX - 1;

            MortonNode* src = in;
            MortonNode* dst = tmp;

            for (int shift = 0; shift < 32; shift += BITS_PER_PASS) {
                size_t count[RADIX] = {};

                // 1. histogram
                for (size_t i = 0; i < n; ++i) {
                    unsigned bucket = (src[i].code >> shift) & MASK;
                    count[bucket]++;
                }

                // 2. exclusive prefix sum
                size_t offset[RADIX];
                size_t sum = 0;
                for (int b = 0; b < RADIX; ++b) {
                    offset[b] = sum;
                    sum += count[b];
                }

                // 3. stable scatter
                for (size_t i = 0; i < n; ++i) {
                    unsigned bucket = (src[i].code >> shift) & MASK;
                    dst[offset[bucket]++] = src[i];
                }

                std::swap(src, dst);
            }

            // pass 횟수가 짝수면 in에, 홀수면 tmp에 최종 결과가 있을 수 있음
            if (src != in) {
                std::copy(src, src + n, in);
            }
        };
        radixSortByMortonCode(mortons.ptr, mortonsTemp.ptr, numPrimitives);
    }
    void radixSortGPU() {
        Index numPrimitives = primitives.size/PRIMITIVE;

        constexpr uint32_t BLOCK_SIZE = 256;
        uint32_t numBlocks = (numPrimitives + BLOCK_SIZE - 1) / BLOCK_SIZE;

        VectorBase<METAL, MortonNode>* src = &mortons;
        VectorBase<METAL, MortonNode>* dst = &mortonsTemp;

        for (uint32_t shift = 0; shift < 32; shift += 8) {
            RadixSortParamsCPU params = {
                (uint32_t)numPrimitives,
                shift,
                numBlocks
            };

            // 1) block histograms
            MetalGlobalContext::setBuffer(*src, 0);
            MetalGlobalContext::setBytes(params, 1);
            MetalGlobalContext::setBuffer(radixBlockHistograms, 2);
            MetalGlobalContext::dispatchThreads(radixCountBlocksPSO, numBlocks * 256, 256);
            //MetalGlobalContext::commitAndWait();

            // 2) offsets and bucket bases
            MetalGlobalContext::setBytes(params, 0);
            MetalGlobalContext::setBuffer(radixBlockHistograms, 1);
            MetalGlobalContext::setBuffer(radixBlockOffsets, 2);
            MetalGlobalContext::setBuffer(radixBucketBase, 3);
            MetalGlobalContext::dispatchThreads(radixComputeOffsetsPSO, 1);
            //MetalGlobalContext::commitAndWait();

            // 3) stable scatter
            MetalGlobalContext::setBuffer(*src, 0);
            MetalGlobalContext::setBuffer(*dst, 1);
            MetalGlobalContext::setBytes(params, 2);
            MetalGlobalContext::setBuffer(radixBlockOffsets, 3);
            MetalGlobalContext::setBuffer(radixBucketBase, 4);
            MetalGlobalContext::dispatchThreads(radixScatterBlocksPSO, numBlocks * 256, 256);
            //MetalGlobalContext::commitAndWait();

            std::swap(src, dst);
        }
    }
    void fillMortonsCPU(AABB4& sceneBox) {
        Index numPrimitives = primitives.size/PRIMITIVE;
        tinym::vec3 width = sceneBox.max - sceneBox.min;
        auto expandBits = [](uint v) {
            v = (v * 0x00010001u) & 0xFF0000FFu;
            v = (v * 0x00000101u) & 0x0F00F00Fu;
            v = (v * 0x00000011u) & 0xC30C30C3u;
            v = (v * 0x00000005u) & 0x49249249u;
            return v;
        };
        auto mortonCode = [&](const tinym::vec3& point) {
            tinym::vec3 p = tinym::min(tinym::max(point*1024.f, tinym::vec3(0.f)), tinym::vec3(1023.f));
            unsigned int xx = expandBits((unsigned int)p.x);
            unsigned int yy = expandBits((unsigned int)p.y);
            unsigned int zz = expandBits((unsigned int)p.z);
            return xx * 4 + yy * 2 + zz;
        };
        for(Index pid = 0; pid < numPrimitives; ++pid) {
            Index base = pid*PRIMITIVE;
            tinym::vec3 center(0);
            for(Index k = 0; k < PRIMITIVE; ++k) {
                Index vid = primitives[base + k];
                center += tinym::vec3_view(positions.ptr + vid*3);
            }
            center = center/PRIMITIVE; // real center
            center = (center - sceneBox.min)/width; // normalized center [0, 1]
            mortons[pid].code = mortonCode(center);
            mortons[pid].index = pid;
        }
    }
    void fillMortonsGPU(AABB4& sceneBox) {
        Index numPrimitives = primitives.size/PRIMITIVE;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(mortons, 3);

        MetalGlobalContext::dispatchThreads(fillMortonsPSO, numPrimitives);
        //MetalGlobalContext::commitAndWait();
    }
    void build(int oid, VectorBase<METAL, PR>& pos, VectorBase<METAL, Index>& prim) {
        //std::cout << "[BVH Build] Memory allocated, BVH build start" << std::endl;
        objid = oid;

        auto* mesh = Scene<METAL, PR>::findById(objid);
        positions = pos;
        primitives = prim;
        if(mesh) {
            velocities = mesh->state.v;
            objBehavior = mesh->behaviorType;
        }
        objShape = ShapeType::Mesh;
        //std::cout << "[BVH Build] positions and primitives are assigned" << std::endl;
        Index numPrimitives = primitives.size/PRIMITIVE;
        if (numPrimitives == 0) {
            resetMemory();
            return;
        }
        if(needsMemoryAllocation(numPrimitives)) memoryAllocation();

        // [stage 1] compute biggest aabb
        // input: each points
        // output: biggest aabbs
        //std::cout << "  - [Stage 1] Compute biggest AABB" << std::endl;
        AABB4 sceneBox(positions.ptr, positions.ptr+3);
        for(Index i = 6; i < positions.size; i+=3) sceneBox.combine(positions.ptr+i);
        sceneBox.i0 = numPrimitives;
        //std::cout << "  - [Stage 1] Scene Box Range: " << sceneBox.min << " to " << sceneBox.max << std::endl;
        
        // [stage 2] transform each center into morton code
        // input: each elements' center. elements can be either triangles or edges
        // output: each elements' morton code
        //std::cout << "  - [Stage 2] Transform each center into morton code" << std::endl;
        //fillMortonsCPU(sceneBox);
        fillMortonsGPU(sceneBox);
        //MetalGlobalContext::commitAndWait();
        
        // [stage 3] radix sort
        // input: array of each elements' morton code
        // output: sorted array of input
        //std::cout << "  - [Stage 3] Radix sort" << std::endl;
        //radixSortCPU();
        // Very slow radix sort
        radixSortGPU();
        //MetalGlobalContext::commitAndWait();

        // [stage 4] build tree
        // input: sorted array of elements' morton code
        // output: linear bvh tree
        //std::cout << "  - [Stage 4] Build tree" << std::endl;
        buildTreeGPU();
        MetalGlobalContext::commitAndWait();

        // set intermediate node's aabb
        //std::cout << "  - AABB combining for intermediate" << std::endl;
        bottomUpCombine();
        //bottomUpCombineGPU();
        //MetalGlobalContext::commitAndWait();

        //DynamicMemoryAllocator<METAL> tempPool;
        //VectorBase<METAL, uint> treeVisitCounts(tempPool.template zeros<uint>(numPrimitives - 1));
        //MetalGlobalContext::setBuffer(treeVisitCounts, 6);

        //MetalGlobalContext::dispatchThreads(bottomUpBoxesPSO, numPrimitives);
        //MetalGlobalContext::commitAndWait();

        //int l = 0;
        //std::cout << "Tree test: " << std::endl;
        //for(Index i = 0; i < 5; i++) {
        //    std::cout << "node " << l << ": " <<  tree[l].aabb.min << ", " << tree[l].aabb.max << " and ";
        //    l = tree[l].childA;
        //    std::cout << " next is " << l << std::endl;
        //    if(l < 0) break;
        //}
    }

    void buildCPU(int oid, VectorBase<METAL, PR>& pos, VectorBase<METAL, Index>& prim) {
        //std::cout << "[BVH Build] Memory allocated, BVH build start" << std::endl;
        objid = oid;
        positions = pos;
        primitives = prim;
        //std::cout << "[BVH Build] positions and primitives are assigned" << std::endl;
        Index numberOfPrimitives = primitives.size/PRIMITIVE;
        if (numberOfPrimitives == 0) {
            resetMemory();
            return;
        }
        if(needsMemoryAllocation(numberOfPrimitives)) memoryAllocation();

        // [stage 1] compute biggest aabb
        // input: each points
        // output: biggest aabbs
        //std::cout << "  - [Stage 1] Compute biggest AABB" << std::endl;
        AABB4 sceneBox(positions.ptr, positions.ptr+3);
        for(Index i = 6; i < positions.size; i+=3) sceneBox.combine(positions.ptr+i);
        //std::cout << "  - [Stage 1] Scene Box Range: " << sceneBox.min << " to " << sceneBox.max << std::endl;
        
        // [stage 2] transform each center into morton code
        // input: each elements' center. elements can be either triangles or edges
        // output: each elements' morton code
        //std::cout << "  - [Stage 2] Transform each center into morton code" << std::endl;
        tinym::vec3 width = sceneBox.max - sceneBox.min;
        auto expandBits = [](uint v) {
            v = (v * 0x00010001u) & 0xFF0000FFu;
            v = (v * 0x00000101u) & 0x0F00F00Fu;
            v = (v * 0x00000011u) & 0xC30C30C3u;
            v = (v * 0x00000005u) & 0x49249249u;
            return v;
        };
        auto mortonCode = [&](const tinym::vec3& point) {
            tinym::vec3 p = tinym::min(tinym::max(point*1024.f, tinym::vec3(0.f)), tinym::vec3(1023.f));
            unsigned int xx = expandBits((unsigned int)p.x);
            unsigned int yy = expandBits((unsigned int)p.y);
            unsigned int zz = expandBits((unsigned int)p.z);
            return xx * 4 + yy * 2 + zz;
        };
        for(Index pid = 0; pid < numberOfPrimitives; ++pid) {
            Index base = pid*PRIMITIVE;
            tinym::vec3 center(0);
            for(Index k = 0; k < PRIMITIVE; ++k) {
                Index vid = primitives[base + k];
                center += tinym::vec3_view(positions.ptr + vid*3);
            }
            center = center/PRIMITIVE; // real center
            center = (center - sceneBox.min)/width; // normalized center [0, 1]
            mortons[pid].code = mortonCode(center);
            mortons[pid].index = pid;
        }
        
        // [stage 3] radix sort
        // input: array of each elements' morton code
        // output: sorted array of input
        //std::cout << "  - [Stage 3] Radix sort" << std::endl;
        auto radixSortByMortonCode = [](MortonNode* in, MortonNode* tmp, size_t n) {
            constexpr int BITS_PER_PASS = 8;
            constexpr int RADIX = 1 << BITS_PER_PASS; // 256
            constexpr int MASK = RADIX - 1;

            MortonNode* src = in;
            MortonNode* dst = tmp;

            for (int shift = 0; shift < 32; shift += BITS_PER_PASS) {
                size_t count[RADIX] = {};

                // 1. histogram
                for (size_t i = 0; i < n; ++i) {
                    unsigned bucket = (src[i].code >> shift) & MASK;
                    count[bucket]++;
                }

                // 2. exclusive prefix sum
                size_t offset[RADIX];
                size_t sum = 0;
                for (int b = 0; b < RADIX; ++b) {
                    offset[b] = sum;
                    sum += count[b];
                }

                // 3. stable scatter
                for (size_t i = 0; i < n; ++i) {
                    unsigned bucket = (src[i].code >> shift) & MASK;
                    dst[offset[bucket]++] = src[i];
                }

                std::swap(src, dst);
            }

            // pass 횟수가 짝수면 in에, 홀수면 tmp에 최종 결과가 있을 수 있음
            if (src != in) {
                std::copy(src, src + n, in);
            }
        };
        radixSortByMortonCode(mortons.ptr, mortonsTemp.ptr, numberOfPrimitives);

        // [stage 4] build tree
        // input: sorted array of elements' morton code
        // output: linear bvh tree
        //std::cout << "  - [Stage 4] Build tree" << std::endl;
        auto clzSafe = [](unsigned int x) {
            return x ? __builtin_clz(x) : 32;
        };
        auto findSplit = [&]( MortonNode* mortons,
                       int           first,
                       int           last) {
            // Identical Morton codes => split the range in the middle.

            unsigned int firstCode = mortons[first].code;
            unsigned int lastCode = mortons[last].code;

            if (firstCode == lastCode)
                return (first + last) >> 1;

            // Calculate the number of highest bits that are the same
            // for all objects, using the count-leading-zeros intrinsic.

            int commonPrefix = clzSafe(firstCode ^ lastCode);

            // Use binary search to find where the next bit differs.
            // Specifically, we are looking for the highest object that
            // shares more than commonPrefix bits with the first one.

            int split = first; // initial guess
            int step = last - first;

            do
            {
                step = (step + 1) >> 1; // exponential decrease
                int newSplit = split + step; // proposed new position

                if (newSplit < last)
                {
                    unsigned int splitCode = mortons[newSplit].code;
                    int splitPrefix = clzSafe(firstCode ^ splitCode);
                    if (splitPrefix > commonPrefix)
                        split = newSplit; // accept proposal
                }
            }
            while (step > 1);

            return split;
        };

        auto determineRange = [&](MortonNode* mortons, Index numberOfPrimitives, Index index) {
            auto delta = [&](int i, int j) {
                if (j < 0 || j >= (int)numberOfPrimitives) return -1;

                unsigned int codeA = mortons[i].code;
                unsigned int codeB = mortons[j].code;

                if (codeA != codeB)
                    return clzSafe(codeA ^ codeB);

                return 32 + clzSafe(mortons[i].index ^ mortons[j].index);
            };

            int d = (delta(index, index + 1) - delta(index, index - 1) >= 0) ? 1 : -1;
            int deltaMin = delta(index, index - d);

            int lmax = 2;
            while (delta(index, index + lmax * d) > deltaMin) {
                lmax <<= 1;
            }

            int l = 0;
            for (int t = lmax >> 1; t >= 1; t >>= 1) {
                if (delta(index, index + (l + t) * d) > deltaMin) {
                    l += t;
                }
            }

            int j = index + l * d;

            if (d < 0) return tinym::vec3((float)j, (float)index, 0.0f);
            else       return tinym::vec3((float)index, (float)j, 0.0f);
        };

        // construct leaf
        //std::cout << "  - Leaf constructing" << std::endl;
        for(Index i = 0; i < numberOfPrimitives; ++i) {
            tree[numberOfPrimitives+i-1] = BVHNode(mortons[i], positions, primitives);
            tree[numberOfPrimitives+i-1].childA = -1;
        }

        // construct intermediate
        //std::cout << "  - intermediate constructing" << std::endl;
        for(Index i = 0; i < numberOfPrimitives-1; ++i) {
            tinym::vec3 range = determineRange(mortons.ptr, numberOfPrimitives, i); // = ?
            Index first = range.x;
            Index last = range.y;

            Index split = findSplit(mortons.ptr, first, last);

            int childA, childB;
            if(split == first) childA = split + numberOfPrimitives-1; // leaf node index
            else childA = split; // intermediate node
            if(split+1 == last) childB = split+1 + numberOfPrimitives-1; // leaf node index
            else childB = split+1; // intermediate node

            tree[i] = BVHNode(childA, childB);
        }

        // set intermediate node's aabb
        auto combineAABB = [&](auto&& self, BVHNode& node) -> void {
            if(node.childA < 0) return;
            self(self, tree[node.childA]);
            self(self, tree[node.childB]);
            node.aabb.min = tree[node.childA].aabb.min;
            node.aabb.max = tree[node.childA].aabb.max;
            node.aabb.combine(tree[node.childB].aabb);
        };
        //std::cout << "  - AABB combining for intermediate" << std::endl;
        combineAABB(combineAABB, tree[0]);


        int l = 0;
        std::cout << "Tree test: " << std::endl;
        for(Index i = 0; i < 5; i++) {
            std::cout << "node " << l << ": " <<  tree[l].aabb.min << ", " << tree[l].aabb.max << " and ";
            l = tree[l].childA;
            std::cout << " next is " << l << std::endl;
            if(l < 0) break;
        }
    }

    void bottomUpCombine() {
        auto combineAABB = [&](auto&& self, BVHNode& node) -> void {
            if(node.childA < 0) return;
            self(self, tree[node.childA]);
            self(self, tree[node.childB]);
            node.aabb.min = tree[node.childA].aabb.min;
            node.aabb.max = tree[node.childA].aabb.max;
            node.aabb.combine(tree[node.childB].aabb);
        };
        combineAABB(combineAABB, tree[0]);
    }

    void refit() {
        Index numPrimitives = primitives.size/PRIMITIVE;
        for(Index i = 0; i < numPrimitives; ++i) {
            tree[numPrimitives+i-1] = BVHNode(mortons[i], positions, primitives);
        }
        bottomUpCombine();

        //buildLeafGPU();
        //bottomUpCombineGPU();
        //MetalGlobalContext::commitAndWait();
    }

    void enlargeTrajectory(PR dt) {
        Index numPrimitives = primitives.size/PRIMITIVE;
        for(Index i = 0; i < numPrimitives; ++i) {
            BVHNode& t = tree[numPrimitives+i-1];
            Index pid = t.childB;
            Index pbase = pid*PRIMITIVE;
            for(Index p = 0; p < PRIMITIVE; ++p) {
                Index posid = primitives[pbase+p];
                tinym::vec3_view pos (positions.ptr + posid*3);
                tinym::vec3_view v (velocities.ptr + posid*3);
                t.min = tinym::min(t.min, pos+v*dt);
                t.max = tinym::max(t.max, pos+v*dt);
            }
        }
        bottomUpCombine();
    }
    
    void queryAABB(const AABB4& queryBox, const BVHNode& node) {
        if(! node.aabb.intersect(queryBox)) return;

        auto* qmesh = Scene<METAL, PR>::findById(queryBox.i1);
        auto& c = qmesh->constraints;
        if(c.numBroadCollisions[0] >= c.maxNumCollisions) return;
        
        if(node.childA < 0) { // leaf
            Index fid = (Index)node.childB;
            Index fbase = fid * 3;
            Index f0 = primitives[fbase];
            Index f1 = primitives[fbase + 1];
            Index f2 = primitives[fbase + 2];
            Index pid = (Index)queryBox.i0;

            if (pid == f0 || pid == f1 || pid == f2) return;

            c.broadCollisions[c.numBroadCollisions[0]].indexPair = {(Index)queryBox.i0, (Index)node.childB}; // conversion is safe since it is leaf
            c.broadCollisions[c.numBroadCollisions[0]].objPair = {(Index)queryBox.i1, (Index)objid};
            c.numBroadCollisions[0]++;
            return;
        }

        if(node.childA > 0) queryAABB(queryBox, tree[node.childA]);
        if(node.childB > 0) queryAABB(queryBox, tree[node.childB]);
        
    }

    void queryAABB(const AABB4& queryBox) {
        // tree 탐색
        queryAABB(queryBox, tree[0]);
    }

    template <typename ObjType>
    void queryPointCPU(int pid, ObjType* qobj, PR queryMargin) {
        Constraints<METAL, PR>& c = qobj->constraints;
        if(c.numBroadCollisions[0] >= c.maxNumCollisions) {
            //std::cout << "Too many collisions are occurs. Some collisions will be ignored.\n";
            return;
        }
        if(pid < 0 || qobj->id < 0) {
            std::cout << "Invalid id pairs: pid " << pid << " and qobjid " << qobj->id << "\n";
            return;
        }

        tinym::vec3_view pointPtr(qobj->state.x.ptr + pid*3);
        //AABBQuery<METAL, PR> query;
        tinym::vec3 min(pointPtr[0]-queryMargin, pointPtr[1]-queryMargin, pointPtr[2]-queryMargin);
        tinym::vec3 max(pointPtr[0]+queryMargin, pointPtr[1]+queryMargin, pointPtr[2]+queryMargin);
        AABB4 query(min, max);
        query.i0 = pid;
        query.i1 = qobj->id;
        //query.aabb = AABB4(min, max);
        //query.aabb.minAndChildA.i = (Index)pid;
        //query.aabb.maxAndChildB.i = (Index)qobj->id;
        //query.qmesh = qobj;

        
        queryAABB(query);
        return;
    }

    void checkSelfCollisionsCPU(PR queryMargin) {
        auto* qmesh = Scene<METAL, PR>::findById(objid); // self query.
        Index numPoints = qmesh->state.x.size/3;
        qmesh->constraints.numBroadCollisions[0] = 0; // clear the previous collisions.

        for(Index vid = 0; vid < numPoints; ++vid) {
            //Index vbase = vid*3;

            queryPointCPU(vid, qmesh, queryMargin);
        }
    }

    struct QueryPointsParams {
        float queryMargin;
        uint32_t numPoints;
        uint32_t qObjId;
        uint32_t tObjId;
        uint32_t maxNumCollisions;
        uint32_t qBehavior;
        uint32_t tBehavior;
        uint32_t qShape;
        uint32_t tShape;
    };

    void queryBegin() {
        qFlag[0].stackOverflow = 0;
        qFlag[0].collisionOverflow = 0;
    }
    void queryPoints(Index qObjId, PR queryMargin) {
        auto* qmesh = Scene<METAL, PR>::findById(qObjId);
        auto& qpos = qmesh->state.x;
        Index qnumPoints = qpos.size/3;
        //auto& constraints = qmesh->constraints;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;
        QueryPointsParams qParams = {
            queryMargin, qnumPoints, qObjId, (Index)objid, /*constraints.maxNumCollisions*/ packedCol.maxNumCollisions,
            (Index)qmesh->behaviorType, (Index)objBehavior, (Index)qmesh->shapeType, (Index)objShape
        };
        //auto& broadCols = constraints.broadCollisions;
        //auto& numBroadCols = constraints.numBroadCollisions;

        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);

        MetalGlobalContext::dispatchThreads(queryPointsPSO, qnumPoints);
    }
    void checkSelfCollisions(PR queryMargin) {
        //auto* qmesh = Scene<METAL, PR>::findById(objid); // self query.
        //qmesh->constraints.numBroadCollisions[0] = 0; // clear the previous collisions.

        queryPoints(objid, queryMargin);
    }
    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        std::cout << "found\n";
        if(qFlag[0].stackOverflow) std::cout << "[QueryPoints] query stack overflowed\n";
        if(qFlag[0].collisionOverflow) std::cout << "[QueryPoints] collision buffer overflowed\n";
    }


    void queryClickRay(const Ray& ray, const BVHNode& node) {
        RayHit hit;
        if(! node.aabb.intersect(ray, hit)) return;
        
        if(node.childA < 0) { // found
            auto& rayTracedData = Scene<BE, PR>::rayTracedData;
            auto& rayTraced = rayTracedData.clickRayCollisions;
            auto& numTraced = rayTracedData.numClickRayCollisions;
            if(numTraced[0] >= rayTracedData.approxColsPerRay) return;
            rayTraced[numTraced[0]] = {
                (Index)objid, (Index)node.childB, hit.tmin, hit.tmax};
            numTraced[0]++;
        }

        if(node.childA > 0) queryClickRay(ray, tree[node.childA]);
        if(node.childB > 0) queryClickRay(ray, tree[node.childB]);
    }

    void queryClickRay(const Ray& ray) {
        //std::cout << "Ray entered the tree " << objid << std::endl;
        queryClickRay(ray, tree[0]);
    }




    void showBox() {
        Index numLines = tree.size*12;
        Index numVertices = numLines*2;
        if (numVertices == 0) {
            debugBox.clear();
            return;
        }

        if(!debugBoxLines.ptr || debugBoxLines.size < numVertices*3) {
            debugBoxLines = VectorBase<METAL, PR>(numVertices*3);
        }

        for(Index nodeid = 0; nodeid < tree.size; ++nodeid) {
            Index linebase = nodeid*12;
            Index vbase = linebase*2;
            Index elebase = vbase*3; // base of each element
            AABB4 aabb = tree[nodeid].aabb;

            auto addLine = [&](const tinym::vec3& a, const tinym::vec3& b) {
                debugBoxLines[elebase++] = a.x;
                debugBoxLines[elebase++] = a.y;
                debugBoxLines[elebase++] = a.z;
                debugBoxLines[elebase++] = b.x;
                debugBoxLines[elebase++] = b.y;
                debugBoxLines[elebase++] = b.z;
            };
            addLine(aabb.min, tinym::vec3(aabb.max.x, aabb.min.y, aabb.min.z));
            addLine(aabb.min, tinym::vec3(aabb.min.x, aabb.max.y, aabb.min.z));
            addLine(aabb.min, tinym::vec3(aabb.min.x, aabb.min.y, aabb.max.z));
            addLine(tinym::vec3(aabb.max.x, aabb.min.y, aabb.min.z), tinym::vec3(aabb.max.x, aabb.max.y, aabb.min.z));
            addLine(tinym::vec3(aabb.max.x, aabb.min.y, aabb.min.z), tinym::vec3(aabb.max.x, aabb.min.y, aabb.max.z));
            addLine(tinym::vec3(aabb.min.x, aabb.max.y, aabb.min.z), tinym::vec3(aabb.max.x, aabb.max.y, aabb.min.z));
            addLine(tinym::vec3(aabb.min.x, aabb.max.y, aabb.min.z), tinym::vec3(aabb.min.x, aabb.max.y, aabb.max.z));
            addLine(tinym::vec3(aabb.min.x, aabb.min.y, aabb.max.z), tinym::vec3(aabb.max.x, aabb.min.y, aabb.max.z));
            addLine(tinym::vec3(aabb.min.x, aabb.min.y, aabb.max.z), tinym::vec3(aabb.min.x, aabb.max.y, aabb.max.z));
            addLine(tinym::vec3(aabb.min.x, aabb.max.y, aabb.max.z), aabb.max);
            addLine(tinym::vec3(aabb.max.x, aabb.min.y, aabb.max.z), aabb.max);
            addLine(tinym::vec3(aabb.max.x, aabb.max.y, aabb.min.z), aabb.max);
        }

        if(!debugBox.vao) debugBox = DebugLineGL<CPU>(numVertices, debugBoxLines.ptr);
        else debugBox.updateBuffer(debugBoxLines.ptr, numVertices);

        debugBox.draw();
    }
};


template <typename BE, typename PR>
struct BVH<BE, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT> {
    using TRI_LBVH = BVH<BE, PR, BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE>;
    using EDGE_LBVH = BVH<BE, PR, BVHMODE::LINEAR, BVHPRIMITIVE::EDGE>;
    std::vector<TRI_LBVH> objTrees;

    // BVH for each object's BV
    VectorBase<METAL, PR> positions;
    VectorBase<METAL, Index> indices;
    EDGE_LBVH tree;

    void resetMemory() {
        for (auto& objTree : objTrees) {
            objTree.resetMemory();
        }
        objTrees.clear();
        positions = {};
        indices = {};
        tree.resetMemory();
    }

    //BVH(SceneObject<METAL, PR>& scene) 
    //    : objTrees(scene.numMeshes), positions(scene.numMeshes*3), indices(scene.numMeshes*2) {}

    void build(Scene<METAL, PR>& scene) {
        if (scene.numMeshes == 0) {
            resetMemory();
            return;
        }

        // allocations
        if(objTrees.size() != scene.numMeshes) {
            resetMemory();
            objTrees = std::vector<TRI_LBVH>(scene.numMeshes);
            positions = VectorBase<METAL, PR>(scene.numMeshes*6);
            indices = VectorBase<METAL, Index>(scene.numMeshes*2);
        }

        for(Index i = 0; i < scene.numMeshes; ++i) {
            if(objTrees[i].tree.ptr && objTrees[i].objBehavior == BehaviorType::Float) continue;
            objTrees[i].build(scene.meshes[i]);
            Index pbase = i*6;
            positions[pbase  ] = objTrees[i].tree[0].min.x;
            positions[pbase+1] = objTrees[i].tree[0].min.y;
            positions[pbase+2] = objTrees[i].tree[0].min.z;
            positions[pbase+3] = objTrees[i].tree[0].max.x;
            positions[pbase+4] = objTrees[i].tree[0].max.y;
            positions[pbase+5] = objTrees[i].tree[0].max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        tree.build(-1, positions, indices);
        //print(tree.tree[0]);
    }

    void print(typename EDGE_LBVH::BVHNode node, Index l = 0) {
        for(Index i = 0; i < l; ++i) std::cout << "  ";
        std::cout << "- ";
        if(node.childA < 0) std::cout << "(leaf) box id " << node.childB << "'s ";
        std::cout << "Min: (" << node.min.x << ", " << node.min.y << ", " << node.min.z << ") and ";
        std::cout << "Max: (" << node.max.x << ", " << node.max.y << ", " << node.max.z << ")" << std::endl;
        if(node.childA < 0) return;
        print(tree.tree[node.childA], l+1);
        print(tree.tree[node.childB], l+1);
    }

    void refit() {
        for(Index i = 0; i < objTrees.size(); ++i) {
            objTrees[i].refit();
            //std::cout << "[objTree root before scene build] id " << i
            //  << " min=" << objTrees[i].tree[0].min
            //  << " max=" << objTrees[i].tree[0].max << std::endl;
            Index pbase = i*6;
            positions[pbase  ] = objTrees[i].tree[0].min.x;
            positions[pbase+1] = objTrees[i].tree[0].min.y;
            positions[pbase+2] = objTrees[i].tree[0].min.z;
            positions[pbase+3] = objTrees[i].tree[0].max.x;
            positions[pbase+4] = objTrees[i].tree[0].max.y;
            positions[pbase+5] = objTrees[i].tree[0].max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        tree.build(-1, positions, indices);
        //    std::cout << "[tree root before scene build] scene tree " 
        //      << " min=" << tree.tree[0].min
        //      << " max=" << tree.tree[0].max << std::endl;
    }

    void enlargeTrajectory(PR dt) {
        for(Index i = 0; i < objTrees.size(); ++i) {
            objTrees[i].enlargeTrajectory(dt);
            Index pbase = i*6;
            positions[pbase  ] = objTrees[i].tree[0].min.x;
            positions[pbase+1] = objTrees[i].tree[0].min.y;
            positions[pbase+2] = objTrees[i].tree[0].min.z;
            positions[pbase+3] = objTrees[i].tree[0].max.x;
            positions[pbase+4] = objTrees[i].tree[0].max.y;
            positions[pbase+5] = objTrees[i].tree[0].max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        tree.build(-1, positions, indices);
    }

    void checkSelfCollisions(PR margin) { 
        for(auto& tree : objTrees) {
            if(tree.objBehavior == BehaviorType::Float) continue;
            tree.checkSelfCollisions(margin);
        }
    }

    void queryBegin() {
        for(auto& tree : objTrees) {
            tree.qFlag[0].stackOverflow = 0;
            tree.qFlag[0].collisionOverflow = 0;
            //Scene<METAL, PR>::findById(tree.objid)->constraints.numBroadCollisions[0] = 0;
        }
        Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] = 0;
    }
    void detectCollisions(PR margin, bool enableSelfCollisions=true, PR radius=0, PR thickness=0) {
        queryBegin();

        std::set<IndexPair> checked;
        for(Index q = 0; q < objTrees.size(); ++q) {
            auto& queryTree = objTrees[q];
            if(queryTree.objBehavior == BehaviorType::Float) continue;

            for(Index t = 0; t < objTrees.size(); ++t) {
                Index a = std::min(q, t);
                Index b = std::max(q, t);
                if(q == t) {
                    if(!enableSelfCollisions) continue;

                    //std::cout << "BVH Query Meshes with " << a << " and " << b << '\n';
                    queryTree.checkSelfCollisions(margin);
                    checked.insert({a, b});
                    continue;
                }
                if(checked.find({a, b}) != checked.end()) continue; // already checked
                // check root insertection
                if(objTrees[t].tree[0].aabb.intersect(queryTree.tree[0].aabb)) {
                    //std::cout << "BVH Query Meshes with " << a << " and " << b << '\n';
                    objTrees[t].queryPoints(queryTree.objid, margin);
                    checked.insert({a, b});
                }
            }
        }
        queryEnd();
    }
    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        for(auto& tree : objTrees) {
            if(tree.qFlag[0].stackOverflow) std::cout << "[Scene BVH detect collisions] " << tree.objid << "'s tree got query stack overflowed\n";
            if(tree.qFlag[0].collisionOverflow) {
                std::cout << "[Scene BVH detect collisions] " << tree.objid << "'s tree got buffer overflowed: "
                          << '(' << Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] << '/'
                          << Scene<METAL, PR>::packedCollisionData.maxNumCollisions << ")\n";
            }
        }
    }

    void showBox() { for(auto& tree : objTrees) tree.showBox(); }

    void showSceneBox() { tree.showBox(); }


    void queryClickRay(const Ray& ray) {
        for(auto& objTree : objTrees) {
            RayHit hit;
            if(! objTree.tree[0].aabb.intersect(ray, hit)) continue;
            objTree.queryClickRay(ray);
        }
    }
};

// TODO: later
template <typename BE, typename PR, BVHPRIMITIVE PRIMITIVE>
struct BVH<BE, PR, BVHMODE::SCENE, PRIMITIVE> {};




// TODO: BroadPhase, NarrowPhase, BruteForce
template <typename BE, typename PR>
struct BruteForce {};
template <typename PR>
struct BruteForce<CPU, PR> {
    using Vector = VectorBase<CPU, PR>;
    //Collision collision;
    //Vector& position;
    //BruteForce(Vector& pos) : position(pos) {}
    //void collide(const VectorBase<CPU, PR>& other) {
    //    auto p = position.map();
    //    auto o = other.map();
    //    auto P = Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>>(p);
    //    auto O = Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>>(o);
    //    for(size_t pi = 0; pi < P.cols; ++pi) for(size_t oi = pi+1; oi < O.cols; ++oi) {
    //        
    //    }
    //}

};


template <typename PR>
struct BruteForce<METAL, PR> {
    MTL::ComputePipelineState* bruteForcePSO;
    BruteForce() {
        bruteForcePSO = MetalKernelContext::getPSO("narrow_pt_tri");
    }
    struct NarrowParams {
        uint32_t numBroadCollisions;
        uint32_t maxNumCollisions;
        float radius;
    };
    bool narrow(PR radius) {
        typename Scene<METAL, PR>::PackedMeshData& packedMesh = Scene<METAL, PR>::packedMeshData;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;

        packedCol.resetNarrow();
        if(packedCol.numBroadCollisions[0] == 0) return false;

        NarrowParams nparams{};
        nparams.numBroadCollisions = std::min(
            packedCol.numBroadCollisions[0],
            packedCol.maxNumCollisions
        );
        nparams.maxNumCollisions = packedCol.maxNumCollisions;
        nparams.radius = radius;


        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 0);
        MetalGlobalContext::setBuffer(packedCol.numNarrowCollisions, 1);
        MetalGlobalContext::setBuffer(packedCol.narrowCollisions, 2);

        MetalGlobalContext::setBuffer(packedMesh.x, 3);
        MetalGlobalContext::setBuffer(packedMesh.statesOffsets, 4);
        MetalGlobalContext::setBuffer(packedMesh.facets, 5);
        MetalGlobalContext::setBuffer(packedMesh.facetsOffsets, 6);
        MetalGlobalContext::setBuffer(packedMesh.vertexAdjFacets, 7);
        MetalGlobalContext::setBuffer(packedMesh.vertexAdjFacetsOffsets, 8);
        MetalGlobalContext::setBytes(nparams, 9);
        MetalGlobalContext::setBuffer(packedMesh.thicknesses,10);

        MetalGlobalContext::dispatchThreads(bruteForcePSO, nparams.numBroadCollisions);
        return true;
    }
    void narrowAndSortByVertices(PR radius) {

        if(narrow(radius))
            MetalGlobalContext::commitAndWait();
        else return;

        typename Scene<METAL, PR>::PackedMeshData& packedMesh = Scene<METAL, PR>::packedMeshData;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;

        packedCol.vertColFacets.map().setZero();
        packedCol.vertColFacetsOffsets.map().setZero();

        for(Index i = 0; i < packedCol.numNarrowCollisions[0]; ++i) {
            NarrowCollision& nc = packedCol.narrowCollisions[i];
            Index pid = nc.indexPair.point;
            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + pid;
            packedCol.vertColFacetsOffsets[ppid+1]++;
        }

        for(Index i = 1; i < packedCol.vertColFacetsOffsets.size; ++i) 
            packedCol.vertColFacetsOffsets[i] += packedCol.vertColFacetsOffsets[i-1];

        DynamicMemoryAllocator<METAL> tempPool;
        VectorBase<METAL, Index> offsets(tempPool.template zeros<Index>(packedCol.vertColFacetsOffsets.size-1));
        
        for(Index i = 0; i < packedCol.numNarrowCollisions[0]; ++i) {
            NarrowCollision& nc = packedCol.narrowCollisions[i];
            Index pid = nc.indexPair.point;
            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + pid;
            Index colid = packedCol.vertColFacetsOffsets[ppid]+offsets[ppid];
            packedCol.vertColFacets[colid] = packedCol.narrowCollisions[i];
            offsets[ppid]++;
        }
    }

    void narrowCPU(
            VectorBase<METAL, BroadCollision>& ptCollisions, 
            Index numCollisions,
            MeshAdjacency<METAL, PR>& adjacency,
            Constraints<METAL, PR>& constraints,
            PR radius) {
        if(!constraints.narrowCollisions.ptr) constraints.narrowCollisions = VectorBase<METAL, NarrowCollision>(ptCollisions.size);
        else constraints.numNarrowCollisions[0] = 0;
        numCollisions = std::min(constraints.numBroadCollisions[0], constraints.maxNumCollisions);
        for(Index cid = 0; cid < numCollisions; ++cid) {
            // check if the pairs are really close than the radius.
            auto& point = ptCollisions[cid].indexPair.point;
            auto& triangle = ptCollisions[cid].indexPair.triangle;
            auto& queryObjId = ptCollisions[cid].objPair.query;
            auto& targetObjId = ptCollisions[cid].objPair.target;
            auto& behaviorPair = ptCollisions[cid].behaviorPair;
            auto& shapePair = ptCollisions[cid].shapePair;
            //collisions[cid].type;

            auto* qmesh = Scene<METAL, PR>::findById(queryObjId);
            auto* tmesh = Scene<METAL, PR>::findById(targetObjId);

            auto& qpositions = qmesh->state.x;
            auto& tpositions = tmesh->state.x;
            auto& tfacets = tmesh->adjacency.facets;

            // compute the min distance from point to triangle
            Index qposbase = point*3;
            Index fbase = triangle*3;

            Index f0 = tfacets[fbase  ];
            Index f1 = tfacets[fbase+1];
            Index f2 = tfacets[fbase+2];

            if(queryObjId == targetObjId) {
                if(point == f0 || point == f1 || point == f2) continue;
            }



            Index t0posbase = f0*3;
            Index t1posbase = f1*3;
            Index t2posbase = f2*3;
            tinym::vec3_view qpos(qpositions.ptr+qposbase);
            tinym::vec3_view t0pos(tpositions.ptr+t0posbase);
            tinym::vec3_view t1pos(tpositions.ptr+t1posbase);
            tinym::vec3_view t2pos(tpositions.ptr+t2posbase);

            tinym::vec3 v0 = t1pos-t0pos;
            tinym::vec3 v1 = t2pos-t0pos;
            tinym::vec3 n = tinym::cross(v0, v1).normalize();

            tinym::vec3 p = qpos-t0pos;

            PR l = n.dot(p);

            // query point 기준으로 normal 방향 정렬
            if (l < 0) {
                n = -n;
                l = -l;
            }
            //if(l < 0) continue; // TODO: already-penetrated case, not handled,
            // TODO: 1.0 is THICK parameter. fix later
            BehaviorType qBehaviorType = (BehaviorType)behaviorPair.query;
            PR thickness = 0.0;
            switch (qBehaviorType) {
                case BehaviorType::FastGridCloth:
                    thickness = std::get<FastGridClothBehaviorParams<PR>>(qmesh->behaviorParams).thickness;
                    break;
                case BehaviorType::TriangularCloth:
                    thickness = std::get<ClothBehaviorParams<PR>>(qmesh->behaviorParams).thickness;
                    break;
                default:
                    break;

            }
            if(l > radius + thickness) continue; // too far.

            // barycentric coordinate to check in-plane
            tinym::vec3 inplane = p - n*l;
            tinym::vec3 v2 = inplane;

            PR d00 = v0.dot(v0);
            PR d01 = v0.dot(v1);
            PR d11 = v1.dot(v1);
            PR d20 = v2.dot(v0);
            PR d21 = v2.dot(v1);

            PR b = (d11*d20 - d01*d21) / (d00*d11 - d01*d01);
            PR c = (d00*d21 - d01*d20) / (d00*d11 - d01*d01);
            PR a = 1-b-c;

            if(a >= 0 && b >= 0 && c >= 0) { // inside plane
                constraints.narrowCollisions[constraints.numNarrowCollisions[0]] = {{point, triangle}, {queryObjId, targetObjId}, {n, l}, behaviorPair, shapePair};
                constraints.numNarrowCollisions[0]++;
            }
        }
    }

    void narrowAndSortByVerticesCPU(
            VectorBase<METAL, BroadCollision>& ptCollisions, 
            Index numCollisions,
            MeshAdjacency<METAL, PR>& adjacency,
            Constraints<METAL, PR>& constraints,
            PR radius) {
        narrowCPU(ptCollisions, numCollisions, adjacency, constraints, radius);
        //narrow(ptCollisions, numCollisions, adjacency, constraints, radius);

        constraints.vertexColPrimsOffsets.map().setZero();
        constraints.vertexColPrims.map().setZero();

        if(constraints.numNarrowCollisions[0] == 0) return;

        for(Index i = 0; i < constraints.numNarrowCollisions[0]; ++i) {
            Index pid = constraints.narrowCollisions[i].indexPair.point;
            constraints.vertexColPrimsOffsets[pid+1]++;
        }

        for(Index i = 1; i < constraints.vertexColPrimsOffsets.size; ++i) 
            constraints.vertexColPrimsOffsets[i] += constraints.vertexColPrimsOffsets[i-1];

        DynamicMemoryAllocator<METAL> tempPool;
        VectorBase<METAL, Index> offsets(tempPool.template zeros<Index>(constraints.vertexColPrimsOffsets.size-1));
        
        for(Index i = 0; i < constraints.numNarrowCollisions[0]; ++i) {
            Index pid = constraints.narrowCollisions[i].indexPair.point;
            Index base = offsets[pid]+constraints.vertexColPrimsOffsets[pid];
            constraints.vertexColPrims[base] = constraints.narrowCollisions[i];
            offsets[pid]++;
        }
    }
};

// No-op narrow phase for pipelines where broadphase already produces NarrowCollision
template <typename BE, typename PR>
struct NoOpNarrowPhase {
    void narrowAndSortByVertices(PR radius) {
        // Spatial hashing already outputs NarrowCollision and sorts by vertex.
    }
};

template <typename BroadPhase, typename NarrowPhase>
struct CollisionPipeline {
    BroadPhase broadPhase;
    BroadPhase broadPhaseTest;
    NarrowPhase narrowPhase;
};

template <typename BE, typename PR, typename System>
struct Simulator {
    System& system;

    Scene<BE, PR> scene;

    //using BroadPhase = BVH<BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE, PR>;
    //using BroadPhase = BVH<BE, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT>;
    using BroadPhase = SpatialHashing<BE, PR>;
    //using NarrowPhase = BruteForce<BE, PR>;
    using NarrowPhase = NoOpNarrowPhase<BE, PR>;
    CollisionPipeline<BroadPhase, NarrowPhase> collisionPipeline;


    // sim viewer?
    bool pause = true;
    bool checkCollision = true;
    bool enableSelfCollisions = false;
    Index frame = 0;
    profiler::FrameProfiler* profiler = nullptr;


    PR margin = 0.015;
    PR radius = 0.012;


    // object select
    int selectedObj = -1;



    Simulator(System& system) : system(system) {}

    void addClothFile(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.001, PR mass=0.1) {
        scene.addGeneralMesh(
                new MeshFileInitializer<BE, PR>({prefix, fileName, offset, scale, mass}),
                BehaviorType::TriangularCloth,
                ClothBehaviorParams<PR>{kstretch, kshear, kbend, thickness}
                );
    };

    void addFloatMesh(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR mass=0.1) {
        scene.addGeneralMesh(
                new MeshFileInitializer<BE, PR>({prefix, fileName, offset, scale, mass}),
                BehaviorType::Float,
                FloatBehaviorParams<PR>{}
                );
    }
    void addClothGridFast(Index particleNum1D = 200, PR size1D = 100, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.001, PR mass=0.1) {
        //particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),{
        //size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2) 
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XYPlane,
                tinym::vec3(0),
                particleNum1D,
                size1D,
                mass,
                true
            }),
            BehaviorType::FastGridCloth, 
            FastGridClothBehaviorParams<PR>{
                particleNum1D, 
                size1D/particleNum1D,
                size1D/particleNum1D*std::sqrtf(2),
                size1D/particleNum1D*2,
                kstretch, 
                kshear, 
                kbend,
                thickness
            }
        );
    }

    void addCloth(Index particleNum1D, PR size1D, tinym::vec3 center, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.01, PR mass=0.1) {
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XZPlane, 
                center,
                particleNum1D, 
                size1D, 
                mass, 
                true // jiggle
            }),
            BehaviorType::TriangularCloth,
            ClothBehaviorParams<PR>{kstretch, kshear, kbend, thickness}
        );
    };
    void addGround(PlaneDirection dir, tinym::vec3 center, PR size1D, PR mass=0.1) {
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                dir, 
                center, 
                2, 
                size1D, 
                mass, 
                false // jiggle
            }),
            BehaviorType::Float,
            FloatBehaviorParams<PR>{}
        );
    };

    void memoryAllocation() {
        //for(auto& plane : sceneObjects.planes) plane.memoryAllocation(pool);
        for(auto& mesh : scene.meshes) mesh.memoryAllocation();
    }


    void initialize() {
        GlobalAutoAllocator<BE>::globalInitialize(1<<20);
        std::cout << "[Simulator Init] Memory pool allocated" << std::endl;

        rebuildScene();

        //Scene<BE, PR>::initialize();

        frame = 0;


        std::cout << "[Simulator Init] All scene objects are initialized" << std::endl;
    }

    void rebuildScene() {
        collisionPipeline.broadPhase.resetMemory();
        collisionPipeline.broadPhaseTest.resetMemory();
        debugSelfCollisions.release();
        debugObjCollisions.release();
        debugSelfCollisionNormals = {};
        debugObjCollisionNormals = {};
        Scene<BE, PR>::pack();
        collisionPipeline.broadPhase.build(scene);
    }


    void update() {
        if(pause) return;
        //std::cout << "[Simulator Update] Start update" << std::endl;
        
        
        if(frame % 10 == 0) {
            if (profiler) {
                auto scope = profiler->scoped("bvh_build");
                collisionPipeline.broadPhase.build(scene);
            } else {
                collisionPipeline.broadPhase.build(scene);
            }
        }

        
        for(int i = 0; i < system.subSteps; i++) {

            //if(i % 10 == 0) checkCollision = true;
            //else checkCollision = false;
            checkCollision = true;

            if(Scene<BE, PR>::numMeshes > 0 && checkCollision) {
                //MetalGlobalContext::commitAndWait();



                //collisionPipeline.broadPhase.enlargeTrajectory(system.h);
                if (profiler) {
                    auto scope = profiler->scoped("broad_refit");
                    collisionPipeline.broadPhase.refit();
                } else {
                    collisionPipeline.broadPhase.refit();
                }
                if (profiler) {
                    auto scope = profiler->scoped("broad_detect");
                    collisionPipeline.broadPhase.detectCollisions(margin, enableSelfCollisions, radius);
                } else {
                    collisionPipeline.broadPhase.detectCollisions(margin, enableSelfCollisions, radius);
                }

                if (profiler) {
                    auto scope = profiler->scoped("narrow_phase");
                    collisionPipeline.narrowPhase.narrowAndSortByVertices(radius);
                } else {
                    collisionPipeline.narrowPhase.narrowAndSortByVertices(radius);
                }

            }

            // clear forces
            if (profiler) {
                auto scope = profiler->scoped("clear_forces");
                scene.clearForces();
            } else {
                scene.clearForces();
            }

            if (profiler) {
                auto scope = profiler->scoped("applyRepulsionForces");
                scene.applyRepulsionForces();
            } else {
                scene.applyRepulsionForces();
            }

            if (profiler) {
                auto scope = profiler->scoped("system_update");
                system.update(scene);
            } else {
                system.update(scene);
            }
        }

        if (profiler) {
            auto scope = profiler->scoped("metal_commit");
            MetalGlobalContext::commitAndWait();
        } else {
            MetalGlobalContext::commitAndWait();
        }
        std::cout << "Broad: " << Scene<BE, PR>::packedCollisionData.numBroadCollisions[0] << ", "
            << "Narrow: " << Scene<BE, PR>::packedCollisionData.numNarrowCollisions[0] << '\n';
        
        system.acctime += system.h;
        frame++;


        if (profiler) {
            auto scope = profiler->scoped("mesh_upload");
            for(auto& mesh : scene.meshes)
                mesh.meshGL.updateBuffer(mesh.state.x.ptr);
        } else {
            for(auto& mesh : scene.meshes)
                mesh.meshGL.updateBuffer(mesh.state.x.ptr);
        }

        //collisionPipeline.broadPhase.build(sceneObjects.squareClothes[0].x, sceneObjects.squareClothes[0].facet);
        //std::cout << "[Simulator Update] Finished update" << std::endl;
    }
    
    void draw(Program& shader) {
        //system.draw();

        for(auto& mesh : scene.meshes) mesh.meshGL.draw(shader, mesh.material.baseColor);

        if(selectedObj >= 0) {
            // Reserved for a future selected-mesh overlay pass.
        }
    }

    void debugEachBoxes(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        collisionPipeline.broadPhase.showBox();
    }
    void debugSceneBox(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        collisionPipeline.broadPhase.showSceneBox();
    }

    VectorBase<BE, PR> debugSelfCollisionNormals, debugObjCollisionNormals;
    DebugLineGL<CPU> debugSelfCollisions;
    DebugLineGL<CPU> debugObjCollisions;
    void prepareDebugCollisions() {
        typename Scene<BE, PR>::PackedCollisionData& packedCol = Scene<BE, PR>::packedCollisionData;
        typename Scene<BE, PR>::PackedMeshData& packedMesh = Scene<BE, PR>::packedMeshData;
        if(packedCol.numNarrowCollisions[0] <= 0) {
            debugSelfCollisions.clear();
            debugObjCollisions.clear();
            return;
        }

        const Index neededCollisionLineValues = packedCol.maxNumCollisions * 6;
        if(!debugSelfCollisionNormals.ptr || debugSelfCollisionNormals.size < neededCollisionLineValues) {
            debugSelfCollisionNormals = VectorBase<BE, PR>(neededCollisionLineValues);
        }
        if(!debugObjCollisionNormals.ptr || debugObjCollisionNormals.size < neededCollisionLineValues) {
            debugObjCollisionNormals = VectorBase<BE, PR>(neededCollisionLineValues);
        }

        debugSelfCollisions.clear();
        debugObjCollisions.clear();

        Index selfBase = 0;
        Index objBase = 0;
        for(Index cid = 0; cid < packedCol.numNarrowCollisions[0]; ++cid) {
            NarrowCollision& nc = packedCol.narrowCollisions[cid];

            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + nc.indexPair.point;
            tinym::vec3_view v(packedMesh.x.ptr + ppid*3);
            tinym::vec3_view n(nc.collisionNormalAndDistance.v);
            tinym::vec3 t = v+n*.2f;

            if(nc.objPair.query == nc.objPair.target) {
                debugSelfCollisionNormals[selfBase  ] = v[0];
                debugSelfCollisionNormals[selfBase+1] = v[1];
                debugSelfCollisionNormals[selfBase+2] = v[2];
                debugSelfCollisionNormals[selfBase+3] = t[0];
                debugSelfCollisionNormals[selfBase+4] = t[1];
                debugSelfCollisionNormals[selfBase+5] = t[2];
                selfBase += 6;
            }
            else {
                debugObjCollisionNormals[objBase  ] = v[0];
                debugObjCollisionNormals[objBase+1] = v[1];
                debugObjCollisionNormals[objBase+2] = v[2];
                debugObjCollisionNormals[objBase+3] = t[0];
                debugObjCollisionNormals[objBase+4] = t[1];
                debugObjCollisionNormals[objBase+5] = t[2];
                objBase += 6;
            }
        }
        if(selfBase > 0) {
            if(!debugSelfCollisions.vao) debugSelfCollisions = DebugLineGL<CPU>(selfBase/3, debugSelfCollisionNormals.ptr);
            else debugSelfCollisions.updateBuffer(debugSelfCollisionNormals.ptr, selfBase/3);
        }
        else {
            debugSelfCollisions.clear();
        }
        if(objBase > 0) {
            if(!debugObjCollisions.vao) debugObjCollisions = DebugLineGL<CPU>(objBase/3, debugObjCollisionNormals.ptr);
            else debugObjCollisions.updateBuffer(debugObjCollisionNormals.ptr, objBase/3);
        }
        else {
            debugObjCollisions.clear();
        }
    }
    void showSelfCollisions() {
        if(debugSelfCollisions.vertexNum > 0)
            debugSelfCollisions.draw();
    }
    void showObjCollisions() {
        if(debugObjCollisions.vertexNum > 0)
            debugObjCollisions.draw();
    }

    void debugCollisions(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");

        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        prepareDebugCollisions();
        showSelfCollisions();
        showObjCollisions();
    }

    Program debugLineShader;
    DebugLineGL<CPU> debugLineGL;
    std::vector<tinym::vec3> debugLines;
    void clearDebugLines() { debugLines.clear(); }
    void addDebugLines(tinym::vec3& a, tinym::vec3& b) {
        debugLines.push_back(a);
        debugLines.push_back(b);
    }
    void showDebugLines(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        if (debugLines.empty()) {
            debugLineGL.clear();
            return;
        }

        if(! debugLineGL.vao) debugLineGL = DebugLineGL<CPU>(debugLines.size(), (float*)debugLines.data());
        else debugLineGL.updateBuffer((float*)debugLines.data(), debugLines.size());

        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        debugLineGL.draw();
    }
};


template <typename BE, typename PR>
struct ExplicitSystem {};
template <typename PR>
struct ExplicitSystem<CPU, PR> {
    using Vector = VectorBase<CPU, PR>;
    using Vectorui = VectorBase<CPU, unsigned int>;

    PR h = 1/PR(60);
    size_t subSteps = 50;
    PR subh = h/subSteps;
    PR G = -980; // in cm/s^2
    PR kair = -0.1;
    PR kd = 0.1;

    ExplicitSystem(PR h=1/PR(60), Index subSteps=50) : h(h), subSteps(subSteps), subh(h/subSteps) {}

    //void initialize(SceneObject<CPU, PR>& sceneObjects) {
    //    std::cout << "[System Creation] Try to create Explicit System..." << std::endl;
    //}

    void clearForce(Scene<CPU, PR>& sceneObjects) { 
        //for(SquareCloth<CPU, PR>& squareCloth : sceneObjects.squareClothes) squareCloth.f.map().setZero(); 
        //for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) deformableMesh.f.map().setZero(); 
    }
    
    void addForce(Scene<CPU, PR>& sceneObjects) {
        // View change: (3Nx1) to (3xN)
        //for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) {
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(deformableMesh.x.ptr, 3, deformableMesh.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(deformableMesh.v.ptr, 3, deformableMesh.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(deformableMesh.f.ptr, 3, deformableMesh.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(deformableMesh.m.ptr, 3, deformableMesh.vertexNum);
        //    
        //    // add gravity
        //    F.row(1) += G * M.row(1);

        //    // add air drag
        //    F += V * kair;

        //    // add spring
        //    // 스프링 포스 계산용 람다 함수 (인자로 ID를 받습니다)
        //    auto addSpringForce = [&](size_t idA, size_t idB, PR restLength, PR kspring) {
        //        // .col()을 쓰면 Eigen::Vector3f 처럼 다룰 수 있습니다!
        //        auto dx = X.col(idB) - X.col(idA); 
        //        
        //        PR len = dx.norm();
        //        if (len < 1E-9) return; // 0 나누기 방지

        //        auto dv = (V.col(idB) - V.col(idA)).cwiseAbs();
        //        auto ndx = dx / len;
        //        auto sf = (kspring * (len - restLength) + kd * dv.dot(ndx)) * ndx;

        //        // 작용-반작용 법칙: A에는 더하고 B에는 뺍니다 (제자리 갱신)
        //        F.col(idA) += sf;
        //        F.col(idB) -= sf;
        //    };
        //    Index springNum = deformableMesh.springIndex.size/2;
        //    for(Index i = 0; i < springNum; ++i) {
        //        addSpringForce(
        //                deformableMesh.springIndex.map()[i*2],
        //                deformableMesh.springIndex.map()[i*2+1], 
        //                deformableMesh.springCoef.map()[i*2], 
        //                deformableMesh.springCoef.map()[i*2+1]
        //        );
        //    }
        //} // deformableMeshes
        //for(SquareCloth<CPU, PR>& squareCloth : sceneObjects.squareClothes) {
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(squareCloth.x.ptr, 3, squareCloth.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(squareCloth.v.ptr, 3, squareCloth.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(squareCloth.f.ptr, 3, squareCloth.vertexNum);
        //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(squareCloth.m.ptr, 3, squareCloth.vertexNum);
        //    
        //    // add gravity
        //    F.row(1) += G * M.row(1);

        //    // add air drag
        //    F += V * kair;

        //    // add spring
        //    // 스프링 포스 계산용 람다 함수 (인자로 ID를 받습니다)
        //    auto addSpringForce = [&](size_t idA, size_t idB, PR restLength, PR kspring) {
        //        // .col()을 쓰면 Eigen::Vector3f 처럼 다룰 수 있습니다!
        //        auto dx = X.col(idB) - X.col(idA); 
        //        
        //        PR len = dx.norm();
        //        if (len < 1E-9) return; // 0 나누기 방지

        //        auto dv = V.col(idB) - V.col(idA);
        //        auto ndx = dx / len;
        //        auto sf = (kspring * (len - restLength) + kd * dv.dot(ndx)) * ndx;

        //        // 작용-반작용 법칙: A에는 더하고 B에는 뺍니다 (제자리 갱신)
        //        F.col(idA) += sf;
        //        F.col(idB) -= sf;
        //    };
        //    for(size_t pid = 0; pid < squareCloth.vertexNum; pid++) {
        //        auto col = pid % squareCloth.particleNum1D;
        //        auto row = pid / squareCloth.particleNum1D;
        //        
        //        // stretch
        //        if(col < squareCloth.particleNum1D-1) addSpringForce(pid, pid+1, squareCloth.stretchRestLength, squareCloth.kstretch); // right
        //        if(row < squareCloth.particleNum1D-1) addSpringForce(pid, pid+squareCloth.particleNum1D, squareCloth.stretchRestLength, squareCloth.kstretch); // bottom
        //        // shear
        //        if(col < squareCloth.particleNum1D-1 && row < squareCloth.particleNum1D-1) addSpringForce(pid, pid+squareCloth.particleNum1D+1, squareCloth.shearRestLength, squareCloth.kshear); // right-bottom
        //        if(col > 0 && row < squareCloth.particleNum1D-1) addSpringForce(pid, pid+squareCloth.particleNum1D-1, squareCloth.shearRestLength, squareCloth.kshear); // left-bottom
        //        // bend
        //        if(col < squareCloth.particleNum1D-2) addSpringForce(pid, pid+2, squareCloth.bendRestLength, squareCloth.kbend); // right-riht
        //        if(row < squareCloth.particleNum1D-2) addSpringForce(pid, pid+2*squareCloth.particleNum1D, squareCloth.bendRestLength, squareCloth.kbend); // bottom-bottom
        //    }
        //} // SquareCloth
    }
    
    void update(Scene<CPU, PR>& sceneObjects) {
        //for(size_t i = 0; i < subSteps; i++) {
        //    clearForce(sceneObjects);
        //    addForce(sceneObjects);
        //    for(SquareCloth<CPU, PR>& squareCloth : sceneObjects.squareClothes) {
        //        Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(squareCloth.x.ptr, 3, squareCloth.vertexNum);
        //        Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(squareCloth.v.ptr, 3, squareCloth.vertexNum);
        //        Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(squareCloth.f.ptr, 3, squareCloth.vertexNum);
        //        Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(squareCloth.m.ptr, 3, squareCloth.vertexNum);

        //        Eigen::Map<Eigen::Matrix<PR, 1, Eigen::Dynamic>> Mask(squareCloth.fixedParticle.ptr, 1, squareCloth.vertexNum);

        //        V.array() += (F.array() / M.array()).rowwise() * Mask.array() * subh;
        //        X.array() += V.array().rowwise() * Mask.array() * subh;
        //    }
        //    //for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) {
        //    //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(deformableMesh.x.ptr, 3, deformableMesh.vertexNum);
        //    //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(deformableMesh.v.ptr, 3, deformableMesh.vertexNum);
        //    //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(deformableMesh.f.ptr, 3, deformableMesh.vertexNum);
        //    //    Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(deformableMesh.m.ptr, 3, deformableMesh.vertexNum);

        //    //    Eigen::Map<Eigen::Matrix<PR, 1, Eigen::Dynamic>> Mask(deformableMesh.fixedParticle.ptr, 1, deformableMesh.vertexNum);

        //    //    V.array() += (F.array() / M.array()).rowwise() * Mask.array() * subh;
        //    //    X.array() += V.array().rowwise() * Mask.array() * subh;
        //    //}
        //}
        //for(SquareCloth<CPU, PR>& squareCloth : sceneObjects.squareClothes) 
        //    squareCloth.mesh.updateBuffer(squareCloth.x.ptr);
        //for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) 
        //    deformableMesh.mesh.updateBuffer(deformableMesh.x.ptr);
        for(auto& mesh : sceneObjects.meshes)
            mesh.meshGL.updateBuffer(mesh.state.x.ptr);
    }
};


template <typename PR>
struct ExplicitSystem<METAL, PR> {
    using Vector = VectorBase<METAL, PR>;
    using Vectorui = VectorBase<METAL, unsigned int>;

    // Metal vars
    MTL::ComputePipelineState* forcePSO;
    MTL::ComputePipelineState* springForcePSO;
    MTL::ComputePipelineState* integrateClothPSO;
    MTL::ComputePipelineState* clothGridFastForcePSO;
    MTL::ComputePipelineState* integrateClothGridPSO;

    // Sim vars
    PR h = 1/PR(60);
    size_t subSteps = 50;
    PR subh = h/subSteps;
    PR G = -9.8; // in m/s^2
    PR kair = -0.001;
    PR kd = .5;
    PR acctime = 0;
    // TODO: bending은 더 강하게 줘야할 듯. - 교수님


    ExplicitSystem(PR h=1/PR(60), Index subSteps=50) 
        : h(h), subSteps(subSteps), subh(h/subSteps) {
        std::cout << "[System Creation] Try to create Explicit System..." << std::endl;
        std::cout << "  - Connecting device..." << std::endl;
        //device = MTL::CreateSystemDefaultDevice();
        //pool = ByteMemoryPool<METAL>(device, 50*1024*1024*sizeof(PR));

        std::cout << "  - Creating a new command pipeline state object (PSO)..." << std::endl;
        // find own metal kernel library

        // find desired kernel
        forcePSO = MetalKernelContext::getPSO("compute_forces");
        springForcePSO = MetalKernelContext::getPSO("compute_spring_forces");
        integrateClothPSO = MetalKernelContext::getPSO("integrate_cloth");
        clothGridFastForcePSO = MetalKernelContext::getPSO("compute_cloth_grid_forces_fast");
        integrateClothGridPSO = MetalKernelContext::getPSO("integrate_cloth_grid");
    }
    
    struct SimParams {
        float subh, G, kair, kd;
        uint vertexNum; 
        float acctime;
    };

    struct ClothGridParams {
        uint particleNum1D;
        float stretchRest, shearRest, bendRest;
        float kstretch, kshear, kbend;
        float thickness;
    };
    struct ClothParams {
        float kstretch, kshear, kbend;
        float thickness;
    };

    // 2. update() 함수 수정
    void update(Scene<METAL, PR>& sceneObjects) {

        for(auto& mesh : sceneObjects.meshes) {

            SimParams params = { subh, G, kair, kd, (uint)mesh.state.x.size/3, acctime };

            //BehaviorParams<PR> clothParams = mesh.behaviorParams;

            //switch(mesh.behaviorType) {
            //    case BehaviorType::TriangularCloth:
            //        TriangularClothBehavior<METAL, PR>::setBuffer(mesh, params);
            //        break;
            //    case BehaviorType::FastGridCloth:
            //        FastGridClothBehavior<METAL, PR>::setBuffer(mesh, params);
            //        break;
            //    case BehaviorType::Float:
            //        break;
            //    case BehaviorType::Elastic:
            //    case BehaviorType::Rigid:
            //    case BehaviorType::Fluid:
            //    case BehaviorType::Generator:
            //    default: break;
            //}
            //for(size_t i = 0; i < subSteps; i++) {
            switch(mesh.behaviorType) {
                case BehaviorType::TriangularCloth:
                    TriangularClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    TriangularClothBehavior<METAL, PR>::update(mesh.state);
                    MetalGlobalContext::dispatchThreads(integrateClothPSO, mesh.state.x.size/3);
                    break;
                case BehaviorType::FastGridCloth:
                    FastGridClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    FastGridClothBehavior<METAL, PR>::update(mesh.state);
                    MetalGlobalContext::dispatchThreads(integrateClothGridPSO, mesh.state.x.size/3);
                    break;
                case BehaviorType::Float:
                    break;
                case BehaviorType::Elastic:
                case BehaviorType::Rigid:
                case BehaviorType::Fluid:
                case BehaviorType::Generator:
                default: break;
            }

            //}
        }
    }
};

// TODO: BVH


// TODO: Spatial Hash


// simulation mamage



int main() {
    
    std::cout << "Run simulator" << std::endl;

    //window = new YGLWindow(640, 480, "ysim");
    yglwindow = new YGLWindow(1600, 900, "ysim");


    // Add Ground
    

    // Add Cloth
#define METAL_SYSTEM

#ifdef METAL_SYSTEM
    using Backend = METAL;
#else
    using Backend = CPU;
#endif

    //ByteMemoryPool<METAL> pool(50*1024*1024*sizeof(Precision));
    Precision h = 1/Precision(240);
    Index subSteps = 20;
    ExplicitSystem<Backend, Precision> system(h, subSteps);
    Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>> simulator(system);
    std::cout << "[Main] simulator created" << std::endl;

    //Index particleNum1D = 20;
    //Precision size1D = 100;
    //Precision kstretch = 2e3;
    //Precision kshear = 2e3;
    //Precision kbend = 4e3;
    Index particleNum1D = 20;
    Precision size1D = 0.5;
    Precision kstretch = 1e4;
    Precision kshear = 1e4;
    Precision kbend = 2e4;
    Precision mass = 0.1;
    Precision thickness = 0.01;
    //simulator.addClothGridFast(particleNum1D, size1D, kstretch, kshear, kbend, thickness, mass);
    //simulator.addClothGridFast(100, 1, kstretch, kshear, kbend, thickness, mass);
    //for(int i = 0; i < 1; i++) 
    //    simulator.addCloth(particleNum1D, size1D, tinym::vec3(0, 0.15+(float)i*0.05f, 0), kstretch, kshear, kbend, thickness, mass);
    simulator.addCloth(100, 1, tinym::vec3(0, 0.25, 0), kstretch, kshear, kbend, thickness, mass);
    //simulator.addClothFile("src/assets", "teapot.obj", {0,0,0} 15, 1e4, 0, 2e4, thickness mass);
    //simulator.addClothFile("src/assets", "horse-gallop-01.obj", {0,0,0}, 80, 1e4, 0, 2e4, thickness mass);
    //simulator.addFloatMesh("src/assets", "horse-gallop-01.obj", {0, -1, 0}, 1.2);
    //simulator.addFloatMesh("src/assets", "camel-gallop-reference.obj", {0, -1, 0}, 1.2);
    simulator.addFloatMesh("src/assets", "Human.obj", {0, -1, 0}, 0.05);
    //simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0), 5);

    std::cout << "[Main] mesh added to scene" << std::endl;

    simulator.initialize();
    std::cout << "[Main] simulator is initialized" << std::endl;

    if(Scene<Backend, Precision>::numMeshes > 0) {
        std::cout << "Try to pin general meshes\n";
        for(auto& mesh: Scene<Backend, Precision>::meshes) {
            std::cout << mesh.id << std::endl;
        }
        auto* mesh = Scene<Backend, Precision>::findById(0);
        std::cout << mesh << std::endl;
        mesh->constraints.fixParticle(0);
        //mesh->constraints.fixParticle(particleNum1D-1);
    }

    std::cout << "[Main] particles are pinned" << std::endl;




    Program shader;
    shader.loadShader("shader.vert", "shader.geom", "shader.frag");


    bool debugEachBoxes = false;
    bool debugSceneBox = false;
    bool debugCollisions = true;
    profiler::FrameProfiler frameProfiler(360);
    profiler::ProfilerWindowState profilerWindowState;
    mesh_inspector::MeshInspectorWindowState meshInspectorWindowState;

    std::cout << "[Main] programs are loaded" << std::endl;


    camera.setPosition(tinym::vec3(0, 0, 5));

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui_ImplGlfw_InitForOpenGL(yglwindow->getGLFWWindow(), false);
#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 410");
#else
    ImGui_ImplOpenGL3_Init("#version 330");
#endif

    struct CallbacksDataPack {
        Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>>* simulator;
        bool* debugEachBoxes;
        bool* debugSceneBox;
        bool* debugCollisions;
        profiler::FrameProfiler* frameProfiler;
    };
    CallbacksDataPack pack = {&simulator, &debugEachBoxes, &debugSceneBox, &debugCollisions, &frameProfiler};

    //glfwSetWindowUserPointer(window->getGLFWWindow(), &system);
    glfwSetWindowUserPointer(yglwindow->getGLFWWindow(), &(pack));

    auto cursorCallback = [](GLFWwindow* window, double xpos, double ypos) {
        ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
        if (ImGui::GetIO().WantCaptureMouse) return;
        YGL::cursorPosCallback(window, xpos, ypos);
    };
    auto scrollCallbackWrapped = [](GLFWwindow* window, double xoffset, double yoffset) {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
        if (ImGui::GetIO().WantCaptureMouse) return;
        YGL::scrollCallback(window, xoffset, yoffset);
    };
    auto mouseButtonCallback = [](GLFWwindow* window, int button, int action, int mods) {
        ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
        if (ImGui::GetIO().WantCaptureMouse) return;
        if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
            auto* simulator = pack->simulator;

            // pixel position에서 model space로 변환
            // ray를 bvh에 태워서 체크 (별도의 ray 객체 생성 후 intersection test)
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            tinym::vec3 npp = camera.unProjectPerspective(window, x, y, -1);
            tinym::vec3 fpp = camera.unProjectPerspective(window, x, y,  1);
            //std::cout << npp << " to " << fpp << std::endl;

            Ray ray;
            ray.origin = npp;
            ray.dir = (fpp-npp).normalize();

            simulator->clearDebugLines();
            simulator->addDebugLines(ray.origin, fpp);

            simulator->scene.rayTracedData.numClickRayCollisions[0] = 0;
            simulator->collisionPipeline.broadPhase.queryClickRay(ray);

            
            Index numRayCols = simulator->scene.rayTracedData.numClickRayCollisions[0];
            if(numRayCols == 0) {
                simulator->selectedObj = -1;
                return;
            }
            auto& rayCols = simulator->scene.rayTracedData.clickRayCollisions;

            Index closestObj = rayCols[0].obj;
            float tmin = rayCols[0].tmin;
            for(int i = 1; i < numRayCols; ++i) if(rayCols[i].tmin < tmin) {
                closestObj = rayCols[i].obj;
                tmin = rayCols[i].tmin;
            }
            simulator->selectedObj = closestObj;
            std::cout << "ClosestObj: " << closestObj << std::endl;
        }
    };
    auto charCallback = [](GLFWwindow* window, unsigned int c) {
        ImGui_ImplGlfw_CharCallback(window, c);
    };
    auto keyCallback = [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
        if (ImGui::GetIO().WantCaptureKeyboard) return;

        auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
        auto* simulator = pack->simulator;
        auto* debugEachBoxes = pack->debugEachBoxes;
        auto* debugSceneBox = pack->debugSceneBox;
        auto* debugCollisions = pack->debugCollisions;

        if(key == GLFW_KEY_0 && action == GLFW_PRESS) {
            simulator->initialize();
        } else if(key == GLFW_KEY_9 && action == GLFW_PRESS) {
        } else if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
            if(simulator->scene.meshes.size() > 0)
                simulator->scene.meshes[0].constraints.fixedParticles[0] = !((bool)simulator->scene.meshes[0].constraints.fixedParticles[0]);
        } else if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
            if(simulator->scene.meshes.size() > 0)
                simulator->scene.meshes[0].constraints.fixedParticles[200-1] = !((bool)simulator->scene.meshes[0].constraints.fixedParticles[200-1]);
        } else if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
            simulator->pause = !(simulator->pause);
        } else if(key == GLFW_KEY_B && action == GLFW_PRESS) {
            *debugEachBoxes = !(*debugEachBoxes);
        } else if(key == GLFW_KEY_S && action == GLFW_PRESS) {
            *debugSceneBox = !(*debugSceneBox);
        } else if(key == GLFW_KEY_C && action == GLFW_PRESS) {
            *debugCollisions = !(*debugCollisions);
        }
    };
    glfwSetCursorPosCallback(yglwindow->getGLFWWindow(), cursorCallback);
    glfwSetScrollCallback(yglwindow->getGLFWWindow(), scrollCallbackWrapped);
    glfwSetMouseButtonCallback(yglwindow->getGLFWWindow(), mouseButtonCallback);
    glfwSetCharCallback(yglwindow->getGLFWWindow(), charCallback);
    glfwSetKeyCallback(yglwindow->getGLFWWindow(), keyCallback);

    std::cout << "[Main] callbacks are set" << std::endl;

    simulator.profiler = &frameProfiler;

    auto init = []() {
        glfwSwapInterval(1);
    };
    auto render = [&]() {
        double currentTime = glfwGetTime();
        bool collectProfileFrame = !simulator.pause;
        if (collectProfileFrame) {
            frameProfiler.beginFrame(simulator.frame, currentTime);
        }

        auto buildSelectedMeshTarget = [&]() {
            mesh_inspector::MeshInspectorTarget target;
            if (auto* selectedMesh = Scene<Backend, Precision>::findById(simulator.selectedObj)) {
                target.mesh_id = selectedMesh->id;
                target.behavior_index = behaviorInspectorIndex(selectedMesh->behaviorType);
                target.fast_grid_supported =
                    selectedMesh->initializer != nullptr &&
                    dynamic_cast<MeshGridInitializer<Backend, Precision>*>(selectedMesh->initializer) != nullptr;
                target.behavior_label = behaviorTypeName(selectedMesh->behaviorType);
                target.shape_label = shapeTypeName(selectedMesh->shapeType);
                target.base_color = &selectedMesh->material.baseColor;

                if (auto* cloth = std::get_if<ClothBehaviorParams<Precision>>(&selectedMesh->behaviorParams)) {
                    target.cloth_stretch = &cloth->stretch;
                    target.cloth_shear = &cloth->shear;
                    target.cloth_bend = &cloth->bend;
                    target.cloth_thickness = &cloth->thickness;
                } else if (auto* fast_grid = std::get_if<FastGridClothBehaviorParams<Precision>>(&selectedMesh->behaviorParams)) {
                    target.fast_stretch_rest = &fast_grid->stretchRest;
                    target.fast_shear_rest = &fast_grid->shearRest;
                    target.fast_bend_rest = &fast_grid->bendRest;
                    target.fast_kstretch = &fast_grid->kstretch;
                    target.fast_kshear = &fast_grid->kshear;
                    target.fast_kbend = &fast_grid->kbend;
                    target.fast_thickness = &fast_grid->thickness;
                }
            }
            return target;
        };

        auto applyPendingMeshInspectorChanges = [&](const mesh_inspector::MeshInspectorTarget& target) {
            if (target.mesh_id >= 0) {
                Scene<Backend, Precision>::syncRequestStateFromMesh(target.mesh_id);
            }

            if (!meshInspectorWindowState.behavior_change_requested) return;

            const int requestMeshId = meshInspectorWindowState.pending_behavior_mesh_id;
            const int requestBehaviorIndex = meshInspectorWindowState.pending_behavior_index;
            meshInspectorWindowState.behavior_change_requested = false;
            meshInspectorWindowState.pending_behavior_mesh_id = -1;
            meshInspectorWindowState.pending_behavior_index = -1;

            if (requestMeshId < 0 || requestBehaviorIndex < 0) return;

            const BehaviorType behaviorType = behaviorFromInspectorIndex(requestBehaviorIndex);
            if (!Scene<Backend, Precision>::setRequestBehavior(requestMeshId, behaviorType)) {
                meshInspectorWindowState.status_message = "Failed to change mesh behavior.";
                return;
            }

            simulator.rebuildScene();
            meshInspectorWindowState.status_message = std::string("Behavior changed to ") + behaviorTypeName(behaviorType) + ".";
        };

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (collectProfileFrame) {
            auto scope = frameProfiler.scoped("physics_total");
            simulator.update();
        } else {
            simulator.update();
        }

        if (collectProfileFrame) {
            auto scope = frameProfiler.scoped("render_total");

            shader.use();
            glViewport(0, 0, yglwindow->width(), yglwindow->height());
            glClearColor(0, 0, 0, 0);
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            tinym::mat4 M(1);
            tinym::mat4 V = camera.lookAt();
            shader.setUniform("M", M);
            shader.setUniform("V", V);
            tinym::mat4 P = camera.perspective(yglwindow->aspect(), 0.1f, 1000.f);
            shader.setUniform("P", P);
            auto w = yglwindow->width()/2;
            auto h = yglwindow->height()/2;
            tinym::mat4 viewport = tinym::mat4(
                    tinym::vec4(w,0.0f,0.0f,0.0f),
                    tinym::vec4(0.0f,h,0.0f,0.0f),
                    tinym::vec4(0.0f,0.0f,1.0f,0.0f),
                    tinym::vec4(w+0, h+0, 0.0f, 1.0f));
            shader.setUniform("ViewportMatrix", viewport);
            shader.setUniform("lightColor", tinym::vec3(160.0f));

            {
                auto drawScope = frameProfiler.scoped("scene_draw");
                simulator.draw(shader);
            }

            {
                auto debugScope = frameProfiler.scoped("debug_draw");
                if(debugEachBoxes) {
                    simulator.debugEachBoxes(V, P);
                }
                if(debugSceneBox) {
                    simulator.debugSceneBox(V, P);
                }

                if(debugCollisions) {
                    simulator.debugCollisions(V, P);
                }
            }
        } else {
            shader.use();
            glViewport(0, 0, yglwindow->width(), yglwindow->height());
            glClearColor(0, 0, 0, 0);
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            tinym::mat4 M(1);
            tinym::mat4 V = camera.lookAt();
            shader.setUniform("M", M);
            shader.setUniform("V", V);
            tinym::mat4 P = camera.perspective(yglwindow->aspect(), 0.1f, 1000.f);
            shader.setUniform("P", P);
            auto w = yglwindow->width()/2;
            auto h = yglwindow->height()/2;
            tinym::mat4 viewport = tinym::mat4(
                    tinym::vec4(w,0.0f,0.0f,0.0f),
                    tinym::vec4(0.0f,h,0.0f,0.0f),
                    tinym::vec4(0.0f,0.0f,1.0f,0.0f),
                    tinym::vec4(w+0, h+0, 0.0f, 1.0f));
            shader.setUniform("ViewportMatrix", viewport);
            shader.setUniform("lightColor", tinym::vec3(160.0f));

            simulator.draw(shader);

            if(debugEachBoxes) {
                simulator.debugEachBoxes(V, P);
            }
            if(debugSceneBox) {
                simulator.debugSceneBox(V, P);
            }

            if(debugCollisions) {
                simulator.debugCollisions(V, P);
            }
            simulator.showDebugLines(V, P);
        }

        auto selectedMeshTarget = buildSelectedMeshTarget();
        auto drawInspectorWindows = [&]() {
            profiler::drawProfilerWindow(
                profilerWindowState,
                frameProfiler,
                &simulator.pause,
                &debugEachBoxes,
                &debugSceneBox,
                &debugCollisions,
                &meshInspectorWindowState.open
            );
            mesh_inspector::drawMeshInspectorWindow(meshInspectorWindowState, selectedMeshTarget);
            applyPendingMeshInspectorChanges(selectedMeshTarget);
        };

        if (collectProfileFrame) {
            auto imguiScope = frameProfiler.scoped("imgui_draw");
            drawInspectorWindows();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        } else {
            drawInspectorWindows();
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }

        if (collectProfileFrame) {
            frameProfiler.endFrame();
        }

        if (const auto* latest = frameProfiler.history().latestFrame()) {
            char title[256];
            std::snprintf(
                title,
                sizeof(title),
                "ysim | FPS: %.1f | Frame: %.2f ms",
                latest->fps,
                latest->frame_ms
            );
            glfwSetWindowTitle(yglwindow->getGLFWWindow(), title);
        }
    };


    yglwindow->mainLoop(init, render);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
