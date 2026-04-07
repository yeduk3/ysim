#include "Foundation/NSString.hpp"
#include "Metal/MTLBuffer.hpp"
#include "Metal/MTLComputeCommandEncoder.hpp"
#include "Metal/MTLComputePipeline.hpp"
#include "YGLWindow.hpp"
#include "camera.hpp"
#include "program.hpp"
#include "objreader.hpp"

#include <cstddef>
#include <iostream>
#include <iterator>
#include <ratio>
#include <string>
#include <type_traits>
#include <typeindex>
#include <set>

YGLWindow* window;

#include <cstdint>

using Index = uint32_t;

//#include "MemoryPool.hpp"

struct Backend {};
struct CPU : Backend {};
struct CUDA : Backend {};
struct METAL : Backend {};

#include <algorithm>
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
    static void dispatchThreads(MTL::ComputePipelineState* pso, Index numThreads) {
        MTL::Size gridSize = MTL::Size(numThreads, 1, 1);
        MTL::Size threadGroupSize = MTL::Size(std::min((Index)pso->maxTotalThreadsPerThreadgroup(), numThreads), 1, 1);
        getComputeCommandEncoder()->setComputePipelineState(pso);
        getComputeCommandEncoder()->dispatchThreads(gridSize, threadGroupSize);
    }
    static void commitAndWait() {
        computeCommandEncoder->endEncoding();
        commandBuffer->commit();
        commandBuffer->waitUntilCompleted();

        commandBuffer = nullptr;
        computeCommandEncoder = nullptr;
    }

};
struct MetalPhysicsContext {
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

        auto pso = MetalGlobalContext::getDevice()->newComputePipelineState(func, &error);

        if (!pso) {
            std::cout << "[Metal Error] PSO creation failed: " << name << "\n";
            exit(1);
        }

        cache[name] = pso;
        return pso;
    }
};

struct MetalBVHContext {
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

    MeshGL() {}
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

    void draw() {
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

    void updateBuffer(float* newVertexPtr, size_t newVertexNum = 0) {
        if (newVertexNum > 0) vertexNum = newVertexNum;
        vertexPtr = newVertexPtr;

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
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, vertexNum);
    }
};

template <typename BE>
struct DebugPointGL {};
template <>
struct DebugPointGL<CPU> {
    GLuint vao;
    GLuint vertexBuffer;

    float* vertexPtr;
    size_t vertexNum;

    DebugPointGL() : vertexPtr(nullptr), vertexNum(0) {}
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

    void updateBuffer(float* newVertexPtr) {
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, newVertexPtr);
    }

    void draw() {
        glBindVertexArray(vao);
        
        glDrawArrays(GL_POINTS, 0, vertexNum);
    }
};


// General Mesh Deinfe!

template <typename BE, typename PR>
struct GeneralMesh;


enum struct BehaviorType : Index {
    TriangularCloth,
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
        if(x.ptr) return;
        Index numData = params.numPoints*3;
        x = Vector(numData);
        v = Vector(numData, 0);
        f = Vector(numData, 0);
        m = Vector(numData, params.mass);
        n = Vector(numData);
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
        if(facets.ptr) return;
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

template <typename BE, typename PR>
struct GeneralMeshInitializer {
    virtual ~GeneralMeshInitializer() = default;
    virtual void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) = 0;
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
struct MeshGridInitializerParams {
    // Commons
    Index numPoints, numFacets, numEdges;
    PR mass;

    // Specifics
    Index particleNum1D;
    PR size1D;
    bool jiggle;
    PlaneDirection dir;
    tinym::vec3 center;

    MeshGridInitializerParams(PlaneDirection dir, tinym::vec3 center, Index particleNum1D, PR size1D, PR mass, bool jiggle)
        : dir(dir), center(center),
        particleNum1D(particleNum1D), 
        numPoints(particleNum1D*particleNum1D),
        numFacets(2*(particleNum1D-1)*(particleNum1D-1)), 
        numEdges(2*(particleNum1D-1)*particleNum1D+numFacets),
        size1D(size1D), mass(mass), jiggle(jiggle) {}
};

template <typename BE, typename PR>
struct MeshGridInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshGridInitializerParams<PR>;
    ParamsType params;

    MeshGridInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) {
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
};

template <typename PR>
struct MeshFileInitializerParams {
    // Commons
    Index numPoints, numFacets, numEdges;
    PR mass;


    // Specifics
    std::string prefix, fileName;
    PR scale;

    MeshFileInitializerParams(std::string prefix, std::string fileName, PR scale, PR mass) 
        : prefix(prefix), fileName(fileName), scale(scale), mass(mass) {}
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
            state.x[vbase  ] = data.vertices[vid].x*params.scale;
            state.x[vbase+1] = data.vertices[vid].y*params.scale;
            state.x[vbase+2] = data.vertices[vid].z*params.scale;
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
    IndexPair behaviorPair;
    IndexPair shapePair;
};

template <typename BE, typename PR>
struct Constraints {
    VectorBase<BE, PR> fixedParticles;

    Index maxNumCollisions = 0;
    //Index numBroadCollisions = 0;
    VectorBase<BE, Index> numBroadCollisions;
    Index numNarrowCollisions = 0;
    Index approxColPerVertex = 15;
    VectorBase<BE, BroadCollision> broadCollisions;
    VectorBase<BE, NarrowCollision> narrowCollisions;

    VectorBase<BE, NarrowCollision> vertexColPrims;
    VectorBase<BE, Index> vertexColPrimsOffsets;

    private:

    public:
    void memoryAllocation(Index numPoints) {
        if(fixedParticles.ptr) return;
        fixedParticles = VectorBase<BE, PR>(numPoints, 1);

        maxNumCollisions = numPoints * approxColPerVertex;
        numBroadCollisions = VectorBase<BE, Index>(1);

        broadCollisions = VectorBase<BE, BroadCollision>(maxNumCollisions);
        narrowCollisions = VectorBase<BE, NarrowCollision>(maxNumCollisions);

        vertexColPrims = VectorBase<BE, NarrowCollision>(maxNumCollisions);
        vertexColPrimsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
    }

    void fixParticle(Index id) { fixedParticles[id] = PR(0); }
    void releaseParticle(Index id) { fixedParticles[id] = PR(1); }

};

template <typename PR>
struct ClothBehaviorParams {
    PR stretch, shear, bend;
};

template <typename PR>
struct FastGridClothBehaviorParams {
    uint particleNum1D;
    PR stretchRest, shearRest, bendRest;
    PR kstretch, kshear, kbend;
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
            pso = MetalPhysicsContext::getPSO("compute_tri_spring_forces");
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
        MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
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
            pso = MetalPhysicsContext::getPSO("compute_cloth_grid_forces_fast");
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
        MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
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
    }

    static void update(MeshState<METAL, PR>& state) {
        auto* pso = getPSO();
        size_t vertexNum = state.x.size/3;
        MetalGlobalContext::dispatchThreads(pso, vertexNum);
    }
};

struct Material {};




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


    // debug
    DebugLineGL<CPU> debugSelfCollisions;
    DebugLineGL<CPU> debugObjCollisions;
    VectorBase<BE, PR> debugSelfCollisionNormals;
    VectorBase<BE, PR> debugObjCollisionNormals;


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
    ~GeneralMesh() { delete initializer; }

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


    void prepareDebugCollisions() {
        if(constraints.numNarrowCollisions <= 0) return;

        if(!debugSelfCollisionNormals.ptr) {
            debugSelfCollisionNormals = VectorBase<BE, PR>(constraints.maxNumCollisions*6);
            debugObjCollisionNormals = VectorBase<BE, PR>(constraints.maxNumCollisions*6);
        }

        Index selfBase = 0;
        Index objBase = 0;
        for(Index cid = 0; cid < constraints.numNarrowCollisions; ++cid) {
            NarrowCollision& nc = constraints.narrowCollisions[cid];

            tinym::vec3_view v(state.x.ptr + nc.indexPair.point*3);
            tinym::vec3_view n(nc.collisionNormalAndDistance.v);
            tinym::vec3 t = v+n*20.f;

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
            if(!debugSelfCollisions.vertexPtr) debugSelfCollisions = DebugLineGL<CPU>(selfBase/3, debugSelfCollisionNormals.ptr);
            else debugSelfCollisions.updateBuffer(debugSelfCollisionNormals.ptr, selfBase/3);
        }
        if(objBase > 0) {
            if(!debugObjCollisions.vertexPtr) debugObjCollisions = DebugLineGL<CPU>(objBase/3, debugObjCollisionNormals.ptr);
            else debugObjCollisions.updateBuffer(debugObjCollisionNormals.ptr, objBase/3);
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
};




struct Sphere {};

// TODO: Plane Collier
struct Plane {
    tinym::vec3 p;
};

// TODO: Box Collier
struct Box {};

template <typename BE, typename PR>
struct SceneObject {
    inline static int numMeshes = 0;

    std::vector<Plane> planes;
    inline static std::vector<GeneralMesh<BE, PR>> meshes;

    void addGeneralMesh(GeneralMeshInitializer<BE, PR>* initializer, BehaviorType behaviorType, BehaviorParams<PR> behaviorParams) {
        meshes.emplace_back(initializer, behaviorType, behaviorParams);
        meshes.back().id = numMeshes++;
        std::cout << "id " << meshes.back().id << " object is created\n";
    }

    static GeneralMesh<BE, PR>* findById(int id) {
        for(auto& mesh : meshes) {
            if(mesh.id == id) return &mesh;
        }
        return nullptr;
    }
};



// TODO: BroadPhase, SpatialHashing
struct SpatialHashing {
    
};


// TODO: BroadPhase, BVH
template <Index MODE, Index PRIMITIVE, typename PR>
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
};
static_assert(sizeof(AABB4) == 32);


// TODO: BroadPhase, LBVH
template <Index PRIMITIVE, typename PR>
struct BVH<BVHMODE::LINEAR, PRIMITIVE, PR> {
    struct alignas(8) MortonNode {
        uint code; // Morton code, 32bit, 10bit per coordinate
        uint index; // facet index
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

    int objid; // who made this tree
    VectorBase<METAL, int> objIds;
    BehaviorType objBehavior;
    VectorBase<METAL, BehaviorType> objBehaviors;
    ShapeType objShape;
    VectorBase<METAL, ShapeType> objShapes;

    MTL::ComputePipelineState* fillMortonsPSO;
    MTL::ComputePipelineState* buildTreePSO;
    MTL::ComputePipelineState* bottomUpBoxesPSO;
    MTL::ComputePipelineState* queryPointsPSO;


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
            fillMortonsPSO = MetalBVHContext::getPSO("fillMortons_Tri");
            buildTreePSO = MetalBVHContext::getPSO("buildTree_Tri");
        } else if constexpr (PRIMITIVE == BVHPRIMITIVE::EDGE) {
            fillMortonsPSO = MetalBVHContext::getPSO("fillMortons_Edge");
            buildTreePSO = MetalBVHContext::getPSO("buildTree_Edge");
        }
        bottomUpBoxesPSO = MetalBVHContext::getPSO("bottomUpBoxes");
        queryPointsPSO = MetalBVHContext::getPSO("queryPoints");
    }

    void memoryAllocation() {
        Index numPrimitives = primitives.size/PRIMITIVE;
        mortons = VectorBase<METAL, MortonNode>(numPrimitives);
        mortonsTemp = VectorBase<METAL, MortonNode>(numPrimitives);
        tree = VectorBase<METAL, BVHNode>(2*numPrimitives-1);
        treeParent = VectorBase<METAL, int>(2*numPrimitives-1);
        qFlag = VectorBase<METAL, QueryFlag>(1);
    }

    void build(GeneralMesh<METAL, PR>& mesh) {
        build(mesh.id, mesh.state.x, mesh.adjacency.facets);
    }

    void build(int oid, VectorBase<METAL, PR>& pos, VectorBase<METAL, Index>& prim) {
        //std::cout << "[BVH Build] Memory allocated, BVH build start" << std::endl;
        objid = oid;

        auto* mesh = SceneObject<METAL, PR>::findById(objid);
        positions = pos;
        velocities = mesh->state.v;
        primitives = prim;
        objBehavior = mesh->behaviorType;
        objShape = ShapeType::Mesh;
        //std::cout << "[BVH Build] positions and primitives are assigned" << std::endl;
        Index numPrimitives = primitives.size/PRIMITIVE;
        if(!tree.ptr) memoryAllocation();

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
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(mortons, 3);

        MetalGlobalContext::dispatchThreads(fillMortonsPSO, numPrimitives);
        MetalGlobalContext::commitAndWait();
        
        
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
        radixSortByMortonCode(mortons.ptr, mortonsTemp.ptr, numPrimitives);

        //encoder->setThreadgroupMemoryLength(?, NS::UInteger index)
        

        // [stage 4] build tree
        // input: sorted array of elements' morton code
        // output: linear bvh tree
        //std::cout << "  - [Stage 4] Build tree" << std::endl;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);

        MetalGlobalContext::dispatchThreads(buildTreePSO, numPrimitives);
        MetalGlobalContext::commitAndWait();

        // set intermediate node's aabb
        //std::cout << "  - AABB combining for intermediate" << std::endl;
        auto combineAABB = [&](auto&& self, BVHNode& node) -> void {
            if(node.childA < 0) return;
            self(self, tree[node.childA]);
            self(self, tree[node.childB]);
            node.aabb.min = tree[node.childA].aabb.min;
            node.aabb.max = tree[node.childA].aabb.max;
            node.aabb.combine(tree[node.childB].aabb);
        };
        combineAABB(combineAABB, tree[0]);

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
        if(!tree.ptr) memoryAllocation();

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

        auto* qmesh = SceneObject<METAL, PR>::findById(queryBox.i1);
        auto& c = qmesh->constraints;
        if(c.numBroadCollisions[0] >= c.maxNumCollisions) return;
        
        if(node.childA == -1) { // leaf
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
        auto* qmesh = SceneObject<METAL, PR>::findById(objid); // self query.
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
        auto* qmesh = SceneObject<METAL, PR>::findById(qObjId);
        auto& pos = qmesh->state.x;
        Index numPoints = pos.size/3;
        auto& constraints = qmesh->constraints;
        QueryPointsParams qParams = {
            queryMargin, numPoints, qObjId, (Index)objid, constraints.maxNumCollisions,
            (Index)qmesh->behaviorType, (Index)objBehavior, (Index)qmesh->shapeType, (Index)objShape
        };
        auto& broadCols = constraints.broadCollisions;
        auto& numBroadCols = constraints.numBroadCollisions;

        MetalGlobalContext::setBuffer(pos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(broadCols, 4);
        MetalGlobalContext::setBuffer(numBroadCols, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);

        MetalGlobalContext::dispatchThreads(queryPointsPSO, numPoints);
    }
    void checkSelfCollisions(PR queryMargin) {
        auto* qmesh = SceneObject<METAL, PR>::findById(objid); // self query.
        qmesh->constraints.numBroadCollisions[0] = 0; // clear the previous collisions.

        queryPoints(objid, queryMargin);
    }
    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        std::cout << "found\n";
        if(qFlag[0].stackOverflow) std::cout << "[QueryPoints] query stack overflowed\n";
        if(qFlag[0].collisionOverflow) std::cout << "[QueryPoints] collision buffer overflowed\n";
    }




    void showBox() {
        Index numLines = tree.size*12;
        Index numVertices = numLines*2;
        if(!debugBoxLines.ptr) {
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

        if(!debugBox.vertexPtr) debugBox = DebugLineGL<CPU>(numVertices, debugBoxLines.ptr);
        else debugBox.updateBuffer(debugBoxLines.ptr);

        debugBox.draw();
    }
};


template <typename PR>
struct BVH<BVHMODE::SCENE, BVHPRIMITIVE::OBJECT, PR> {
    using TRI_LBVH = BVH<BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE, PR>;
    using EDGE_LBVH = BVH<BVHMODE::LINEAR, BVHPRIMITIVE::EDGE, PR>;
    std::vector<TRI_LBVH> objTrees;

    // BVH for each object's BV
    VectorBase<METAL, PR> positions;
    VectorBase<METAL, Index> indices;
    EDGE_LBVH tree;

    //BVH(SceneObject<METAL, PR>& scene) 
    //    : objTrees(scene.numMeshes), positions(scene.numMeshes*3), indices(scene.numMeshes*2) {}

    void build(SceneObject<METAL, PR>& scene) {
        // allocations
        if(objTrees.size() != scene.numMeshes) {
            objTrees = std::vector<TRI_LBVH>(scene.numMeshes);
            positions = VectorBase<METAL, PR>(scene.numMeshes*6);
            indices = VectorBase<METAL, Index>(scene.numMeshes*2);
        }

        for(Index i = 0; i < scene.numMeshes; ++i) {
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
        print(tree.tree[0]);
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
            SceneObject<METAL, PR>::findById(tree.objid)->constraints.numBroadCollisions[0] = 0;
        }
    }
    void detectCollisions(PR margin) {
        queryBegin();

        std::set<IndexPair> checked;
        for(Index q = 0; q < objTrees.size(); ++q) {
            auto& queryTree = objTrees[q];
            if(queryTree.objBehavior == BehaviorType::Float) continue;

            for(Index t = 0; t < objTrees.size(); ++t) {
                if(q == t) {
                    queryTree.checkSelfCollisions(margin);
                    checked.insert({q, t});
                    continue;
                }
                if(checked.find({q, t}) != checked.end()) continue; // already checked
                objTrees[t].queryPoints(queryTree.objid, margin);
                checked.insert({q, t});
            }
        }
        queryEnd();
    }
    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        for(auto& tree : objTrees) {
            if(tree.qFlag[0].stackOverflow) std::cout << "[Scene BVH detect collisions] " << tree.objid << "'s tree got query stack overflowed\n";
            if(tree.qFlag[0].collisionOverflow) std::cout << "[Scene BVH detect collisions] " << tree.objid << "'s tree got buffer overflowed\n";
        }
    }

    void showBox() { for(auto& tree : objTrees) tree.showBox(); }

    void showSceneBox() { tree.showBox(); }
};






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

    void narrow(
            VectorBase<METAL, BroadCollision>& ptCollisions, 
            Index numCollisions,
            MeshAdjacency<METAL, PR>& adjacency,
            Constraints<METAL, PR>& constraints,
            PR radius) {
        if(!constraints.narrowCollisions.ptr) constraints.narrowCollisions = VectorBase<METAL, NarrowCollision>(ptCollisions.size);
        else constraints.numNarrowCollisions = 0;
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

            auto* qmesh = SceneObject<METAL, PR>::findById(queryObjId);
            auto* tmesh = SceneObject<METAL, PR>::findById(targetObjId);

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
            if(l > radius + 0.5) continue; // too far.

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
                constraints.narrowCollisions[constraints.numNarrowCollisions] = {{point, triangle}, {queryObjId, targetObjId}, {n, l}, behaviorPair, shapePair};
                constraints.numNarrowCollisions++;
            }
        }
    }

    void narrowAndSortByVertices(
            VectorBase<METAL, BroadCollision>& ptCollisions, 
            Index numCollisions,
            MeshAdjacency<METAL, PR>& adjacency,
            Constraints<METAL, PR>& constraints,
            PR radius) {
        narrow(ptCollisions, numCollisions, adjacency, constraints, radius);

        constraints.vertexColPrimsOffsets.map().setZero();
        constraints.vertexColPrims.map().setZero();

        for(Index i = 0; i < constraints.numNarrowCollisions; ++i) {
            Index pid = constraints.narrowCollisions[i].indexPair.point;
            constraints.vertexColPrimsOffsets[pid+1]++;
        }

        for(Index i = 1; i < constraints.vertexColPrimsOffsets.size; ++i) 
            constraints.vertexColPrimsOffsets[i] += constraints.vertexColPrimsOffsets[i-1];

        DynamicMemoryAllocator<METAL> tempPool;
        VectorBase<METAL, Index> offsets(tempPool.template zeros<Index>(constraints.vertexColPrimsOffsets.size-1));
        
        for(Index i = 0; i < constraints.numNarrowCollisions; ++i) {
            Index pid = constraints.narrowCollisions[i].indexPair.point;
            Index base = offsets[pid]+constraints.vertexColPrimsOffsets[pid];
            constraints.vertexColPrims[base] = constraints.narrowCollisions[i];
            offsets[pid]++;
        }
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

    SceneObject<BE, PR> sceneObjects;

    //using BroadPhase = BVH<BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE, PR>;
    using BroadPhase = BVH<BVHMODE::SCENE, BVHPRIMITIVE::OBJECT, PR>;
    using NarrowPhase = BruteForce<METAL, PR>;
    CollisionPipeline<BroadPhase, NarrowPhase> collisionPipeline;


    // sim viewer?
    bool pause = true;
    bool checkCollision = true;
    Index frame = 0;


    PR margin = 1;
    PR radius = 0.5;



    Simulator(System& system) : system(system) {}

    void addClothFile(std::string prefix, std::string fileName, PR scale, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR mass=0.1) {
        sceneObjects.addGeneralMesh(
                new MeshFileInitializer<BE, PR>({prefix, fileName, scale, mass}),
                BehaviorType::TriangularCloth,
                ClothBehaviorParams<PR>{kstretch, kshear, kbend}
                );
    };

    void addClothGridFast(Index particleNum1D = 200, PR size1D = 100, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR mass=0.1) {
        //particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),{
        //size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2) 
        sceneObjects.addGeneralMesh(
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
                kbend
            }
        );
    }

    void addGeneralMesh(Index particleNum1D = 200, PR size1D = 100, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR mass=0.1) {
        sceneObjects.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XYPlane, 
                tinym::vec3(0),
                particleNum1D, 
                size1D, 
                mass, 
                true // jiggle
            }),
            BehaviorType::TriangularCloth,
            ClothBehaviorParams<PR>{kstretch, kshear, kbend}
        );
    };
    void addGround(PlaneDirection dir, tinym::vec3 center, PR size1D, PR mass=0.1) {
        sceneObjects.addGeneralMesh(
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
        for(auto& mesh : sceneObjects.meshes) mesh.memoryAllocation();
    }


    void initialize() {
        GlobalAutoAllocator<BE>::globalInitialize(1<<20);
        std::cout << "[Simulator Init] Memory pool allocated" << std::endl;

        for(auto& plane : sceneObjects.planes) {}
        for(auto& mesh : sceneObjects.meshes) {
            std::cout << "  - try to initialize mesh " << mesh.id << "\n";
            mesh.initialize();
            mesh.state.v.map().setZero();
            std::cout << "  - mesh " << mesh.id << " is initialized\n";
        }
        if(sceneObjects.meshes.size() > 0) std::cout << "[Simulator Init] general mesh objects are initialized" << std::endl;

        frame = 0;

        std::cout << "[Simulator Init] All scene objects are initialized" << std::endl;
            
    }


    void update() {
        if(pause) return;
        //std::cout << "[Simulator Update] Start update" << std::endl;
        
        
        if(frame % 10 == 0) collisionPipeline.broadPhase.build(sceneObjects);

        
        for(int i = 0; i < system.subSteps; i++) {

            //if(i % 10 == 0) checkCollision = true;
            //else checkCollision = false;
            checkCollision = true;

            if(SceneObject<BE, PR>::numMeshes > 0 && checkCollision) {
                //MetalGlobalContext::commitAndWait();

                auto* mesh = SceneObject<BE, PR>::findById(0);


                //collisionPipeline.broadPhase.enlargeTrajectory(system.h);
                collisionPipeline.broadPhase.refit();
                collisionPipeline.broadPhase.detectCollisions(margin);

                auto& c = mesh->constraints;
                if(c.numBroadCollisions[0] > 0) {
                    std::cout << "Collision detected: (frame,substep)=" << frame << "," << i << " (" << c.numBroadCollisions[0] << "/" << c.maxNumCollisions << ")\n";
                    //for(Index i = 0; i < 5; ++i) {
                    //    std::cout << "  - Collision " << i << ": point " 
                    //        << c.broadCollisions[i].indexPair.point
                    //        << " and triangle "
                    //        << c.broadCollisions[i].indexPair.triangle << std::endl;
                    //}
                collisionPipeline.narrowPhase.narrowAndSortByVertices(
                        c.broadCollisions,
                        c.numBroadCollisions[0],
                        mesh->adjacency,
                        c,
                        radius);
                }


                if(c.numNarrowCollisions > 0) {
                    std::cout << "Narrowed collision detected: (frame,substep)=" << frame << "," << i << " (" << c.numNarrowCollisions << "/" << c.maxNumCollisions << ")\n";
                    //Index min = 5 < c.numNarrowCollisions ? 5 : c.numNarrowCollisions;
                    //for(Index i = 0; i < min; ++i) {
                    //    std::cout << "  - Collision " << i << ": point " 
                    //        << c.narrowCollisions[i].indexPair.point
                    //        << " and triangle "
                    //        << c.narrowCollisions[i].indexPair.triangle 
                    //        << " and normal " 
                    //        << '(' << c.narrowCollisions[i].collisionNormalAndDistance.x 
                    //        << ", " << c.narrowCollisions[i].collisionNormalAndDistance.y 
                    //        << ", " << c.narrowCollisions[i].collisionNormalAndDistance.z << ')'
                    //        << " and distance " 
                    //        << c.narrowCollisions[i].collisionNormalAndDistance.w
                    //        << std::endl;
                    //}
                    //std::cout << "Narrowed self collision sorted: (" << c.numNarrowCollisions << "/" << c.maxNumCollisions << ")\n";
                    //Index count = 0;
                    //for(Index i = 0; i < c.vertexColPrimsOffsets.size-1; ++i) {
                    //    if(count >= min) break;
                    //    if(c.vertexColPrimsOffsets[i] == c.vertexColPrimsOffsets[i+1]) continue;
                    //    for(Index j = c.vertexColPrimsOffsets[i]; j < c.vertexColPrimsOffsets[i+1]; ++j) {
                    //        if(count >= min) break;
                    //        std::cout << "  - Collision " << j << ": point " 
                    //            << c.vertexColPrims[j].indexPair.point
                    //            << " and triangle "
                    //            << c.vertexColPrims[j].indexPair.triangle 
                    //            << " and normal " 
                    //            << '(' << c.vertexColPrims[j].collisionNormalAndDistance.x 
                    //            << ", " << c.vertexColPrims[j].collisionNormalAndDistance.y 
                    //            << ", " << c.vertexColPrims[j].collisionNormalAndDistance.z << ')'
                    //            << " and distance " 
                    //            << c.vertexColPrims[j].collisionNormalAndDistㅅance.w
                    //            << std::endl;
                    //        count++;
                    //    }
                    //}
                }
            }



            system.update(sceneObjects);
        }

        MetalGlobalContext::commitAndWait();
        
        system.acctime += system.h;
        frame++;

        for(auto& mesh : sceneObjects.meshes)
            mesh.meshGL.updateBuffer(mesh.state.x.ptr);

        //collisionPipeline.broadPhase.build(sceneObjects.squareClothes[0].x, sceneObjects.squareClothes[0].facet);
        //std::cout << "[Simulator Update] Finished update" << std::endl;
    }
    
    void draw() {
        //system.draw();

        for(auto& mesh : sceneObjects.meshes) mesh.meshGL.draw();
    }

    void debugEachBoxes() {
        collisionPipeline.broadPhase.showBox();
    }
    void debugSceneBox() {
        collisionPipeline.broadPhase.showSceneBox();
    }


    void debugCollisions() {
        for(GeneralMesh<BE, PR>& mesh : sceneObjects.meshes) {
            if(mesh.behaviorType == BehaviorType::Float) continue;
            mesh.prepareDebugCollisions();
            mesh.showObjCollisions();
        }
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

    void clearForce(SceneObject<CPU, PR>& sceneObjects) { 
        //for(SquareCloth<CPU, PR>& squareCloth : sceneObjects.squareClothes) squareCloth.f.map().setZero(); 
        //for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) deformableMesh.f.map().setZero(); 
    }
    
    void addForce(SceneObject<CPU, PR>& sceneObjects) {
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
    
    void update(SceneObject<CPU, PR>& sceneObjects) {
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
    MTL::ComputePipelineState* integratePSO;
    MTL::ComputePipelineState* clothGridFastForcePSO;

    // Sim vars
    PR h = 1/PR(60);
    size_t subSteps = 50;
    PR subh = h/subSteps;
    PR G = -980; // in cm/s^2
    PR kair = -0.1;
    PR kd = 0.1;
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
        forcePSO = MetalPhysicsContext::getPSO("compute_forces");
        springForcePSO = MetalPhysicsContext::getPSO("compute_spring_forces");
        integratePSO = MetalPhysicsContext::getPSO("integrate");
        clothGridFastForcePSO = MetalPhysicsContext::getPSO("compute_cloth_grid_forces_fast");
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
    };
    struct ClothParams {
        float kstretch, kshear, kbend;
    };

    // 2. update() 함수 수정
    void update(SceneObject<METAL, PR>& sceneObjects) {

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
                    MetalGlobalContext::dispatchThreads(integratePSO, mesh.state.x.size/3);
                    break;
                case BehaviorType::FastGridCloth:
                    FastGridClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    FastGridClothBehavior<METAL, PR>::update(mesh.state);
                    MetalGlobalContext::dispatchThreads(integratePSO, mesh.state.x.size/3);
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
constexpr int particleN = 20;
constexpr int groundN = 1;



#include <chrono> // 시간 관련 라이브러리


int main() {
    
    std::cout << "Render" << std::endl;

    //window = new YGLWindow(640, 480, "ysim");
    window = new YGLWindow(1600, 900, "ysim");


    // Add Ground
    

    // Add Cloth
#define METAL_SYSTEM

#ifdef METAL_SYSTEM
    using Backend = METAL;
#else
    using Backend = CPU;
#endif

    //ByteMemoryPool<METAL> pool(50*1024*1024*sizeof(Precision));
    Precision h = 1/Precision(60);
    Index subSteps = 60;
    ExplicitSystem<Backend, Precision> system(h, subSteps);
    Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>> simulator(system);
    std::cout << "[Main] simulator created" << std::endl;

    //Index particleNum1D = 20;
    //Precision size1D = 100;
    //Precision kstretch = 2e3;
    //Precision kshear = 2e3;
    //Precision kbend = 4e3;
    Index particleNum1D = 200;
    Precision size1D = 100;
    Precision kstretch = 2e5;
    Precision kshear = 1e5;
    Precision kbend = 4e5;
    Precision mass = 0.1;
    //simulator.addClothGridFast(particleNum1D, size1D, kstretch, kshear, kbend, mass);
    simulator.addClothGridFast(20, 50, 1e5, 1e5, 2e5, 0.1);
    //simulator.addGeneralMesh(particleNum1D, size1D, kstretch/2, kshear, kbend/2, mass);
    //simulator.addClothFile("src/assets", "teapot.obj", 15, 1e4, 0, 2e4, mass);
    //simulator.addClothFile("src/assets", "horse-gallop-01.obj", 80, 1e4, 0, 2e4, mass);
    simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -100, 0), 500);
    
    std::cout << "[Main] mesh added to scene" << std::endl;

    simulator.initialize();
    std::cout << "[Main] simulator is initialized" << std::endl;

    if(SceneObject<Backend, Precision>::numMeshes > 0) {
        std::cout << "Try to pin general meshes\n";
        for(auto& mesh: SceneObject<Backend, Precision>::meshes) {
            std::cout << mesh.id << std::endl;
        }
        auto* mesh = SceneObject<Backend, Precision>::findById(0);
        std::cout << mesh << std::endl;
        mesh->constraints.fixParticle(0);
        //mesh->constraints.fixParticle(particleNum1D-1);
    }

    std::cout << "[Main] particles are pinned" << std::endl;




    Program shader;
    shader.loadShader("shader.vert", "shader.geom", "shader.frag");

    Program debugLineShader;
    debugLineShader.loadShader("line.vert", "line.frag");

    bool debugEachBoxes = false;
    bool debugSceneBox = false;
    bool debugCollisions = true;

    std::cout << "[Main] programs are loaded" << std::endl;


    camera.setPosition(tinym::vec3(0, 0, 500));

    camera.glfwSetCallbacks(window->getGLFWWindow());

    struct CallbacksDataPack {
        Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>>* simulator;
        bool* debugEachBoxes;
        bool* debugSceneBox;
    };
    CallbacksDataPack pack = {&simulator, &debugEachBoxes, &debugSceneBox};

    //glfwSetWindowUserPointer(window->getGLFWWindow(), &system);
    glfwSetWindowUserPointer(window->getGLFWWindow(), &(pack));
    auto keyCallback = [](GLFWwindow* window, int key, int scancode, int action, int mods) {

        auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
        auto* simulator = pack->simulator;
        auto* debugEachBoxes = pack->debugEachBoxes;
        auto* debugSceneBox = pack->debugSceneBox;

        if(key == GLFW_KEY_0 && action == GLFW_PRESS) {
            simulator->initialize();
        } else if(key == GLFW_KEY_9 && action == GLFW_PRESS) {
        } else if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
            if(simulator->sceneObjects.meshes.size() > 0)
                simulator->sceneObjects.meshes[0].constraints.fixedParticles[0] = !((bool)simulator->sceneObjects.meshes[0].constraints.fixedParticles[0]);
        } else if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
            if(simulator->sceneObjects.meshes.size() > 0)
                simulator->sceneObjects.meshes[0].constraints.fixedParticles[200-1] = !((bool)simulator->sceneObjects.meshes[0].constraints.fixedParticles[200-1]);
        } else if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
            simulator->pause = !(simulator->pause);
        } else if(key == GLFW_KEY_B && action == GLFW_PRESS) {
            *debugEachBoxes = !(*debugEachBoxes);
        } else if(key == GLFW_KEY_S && action == GLFW_PRESS) {
            *debugSceneBox = !(*debugSceneBox);
        } else if(key == GLFW_KEY_C && action == GLFW_PRESS) {
            simulator->checkCollision = !(simulator->checkCollision);
        }

    };
    glfwSetKeyCallback(window->getGLFWWindow(), keyCallback);

    std::cout << "[Main] callbacks are set" << std::endl;

    int frameCounter = 0;
    double lastTime = glfwGetTime();

    auto init = []() {};
    auto render = [&]() {
        auto physicsStart = std::chrono::high_resolution_clock::now();
        //system.update();
        simulator.update();
        auto physicsEnd = std::chrono::high_resolution_clock::now();
        double physicsTime = std::chrono::duration<double, std::milli>(physicsEnd - physicsStart).count();
        
        auto renderingStart = std::chrono::high_resolution_clock::now();
        shader.use();
        glViewport(0, 0, window->width(), window->height());
        glClearColor(0, 0, 0, 0);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //float M[16] = {1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0,   0, -0.5, 0, 1};
        tinym::mat4 M(1);
        //M[3][1] = -0.5; // translation
        tinym::mat4 V = camera.lookAt();
        shader.setUniform("M", M);
        shader.setUniform("V", V);
        tinym::mat4 P = camera.perspective(window->aspect(), 0.1f, 1000.f);
        shader.setUniform("P", P);
        auto w = window->width()/2;
        auto h = window->height()/2;
        tinym::mat4 viewport = tinym::mat4(
                tinym::vec4(w,0.0f,0.0f,0.0f),
                tinym::vec4(0.0f,h,0.0f,0.0f),
                tinym::vec4(0.0f,0.0f,1.0f,0.0f),
                tinym::vec4(w+0, h+0, 0.0f, 1.0f));
        shader.setUniform("ViewportMatrix", viewport);
        glfwSwapInterval(1);

        //system.draw();
        simulator.draw();

        if(debugEachBoxes) {
            debugLineShader.use();
            debugLineShader.setUniform("V", V);
            debugLineShader.setUniform("P", P);
            glLineWidth(2.5f);
            simulator.debugEachBoxes();
        }
        if(debugSceneBox) {
            debugLineShader.use();
            debugLineShader.setUniform("V", V);
            debugLineShader.setUniform("P", P);
            glLineWidth(2.5f);
            simulator.debugSceneBox();
        }

        if(debugCollisions) {
            debugLineShader.use();
            debugLineShader.setUniform("V", V);
            debugLineShader.setUniform("P", P);
            glLineWidth(2.5f);
            simulator.debugCollisions();
        }

        auto renderingEnd = std::chrono::high_resolution_clock::now();
        double renderingTime = std::chrono::duration<double, std::milli>(renderingEnd - renderingStart).count();

        frameCounter++;
        double currentTime = glfwGetTime();
        if (currentTime - lastTime >= 1.0) { // 1초마다 업데이트
            char title[256];
            snprintf(title, sizeof(title), 
                     "ysim | FPS: %d | Physics: %.2f ms | Rendering: %.2f ms", 
                     frameCounter, physicsTime, renderingTime);
            glfwSetWindowTitle(window->getGLFWWindow(), title); // GLFWwindow 포인터 전달
            
            frameCounter = 0;
            lastTime += 1.0;
        }

    };


    window->mainLoop(init, render);

    return 0;
}


