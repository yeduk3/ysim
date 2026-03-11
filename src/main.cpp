#include "Foundation/NSString.hpp"
#include "Metal/MTLBuffer.hpp"
#include "YGLWindow.hpp"
#include "camera.hpp"
#include "program.hpp"


#include <cstddef>
#include <iostream>
#include <ratio>
#include <type_traits>

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
template <typename T>
struct MemoryPool {
    std::vector<T> pool;
    size_t marker = 0;
    MemoryPool() {}
    MemoryPool(size_t N) : pool(N) {}
    //T& operator[](size_t i) { return pool[i]; }
    //T* data() { return pool.data(); }
    T* alloc(size_t count) {
        if(marker >= pool.size()) {
            std::cout << "[Pool] Memory overflow" << std::endl;
            exit(1);
        }
        T* ret = pool.data() + marker;
        marker += count;
        return ret;
    }
    T* zeros(size_t count) {
        T* ret = alloc(count);
        memset(ret, 0, count*sizeof(T));
        return ret;
    }
    T* allocFill(size_t count, T fill) {
        T* ret = alloc(count);
        std::fill(ret, ret+count, fill);
        return ret;
    }
};

template <typename BE>
struct ByteMemoryPool {};
template <>
struct ByteMemoryPool<CPU> {
    std::vector<char> pool;
    size_t marker = 0;
    ByteMemoryPool() {}
    ByteMemoryPool(size_t N) : pool(N) {}
    template <typename PR>
    PR* alloc(size_t count) {
        if(marker >= pool.size()) {
            std::cout << "[Pool] Memory overflow" << std::endl;
            exit(1);
        }
        size_t align = alignof(PR);
        marker += (align - marker % align) % align; // byte align
        PR* ret = reinterpret_cast<PR*>(pool.data() + marker);
        marker += sizeof(PR)*count;
        return ret;
    }
    template <typename PR>
    PR* zeros(size_t count) {
        PR* ret = alloc<PR>(count);
        memset(ret, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    PR* allocFill(size_t count, PR fill) {
        PR* ret = alloc<PR>(count);
        std::fill(ret, ret+count, fill);
        return ret;
    }
};

#include <Metal/Metal.hpp>
struct MetalContext {
    static MTL::Device* getDevice() {
        // C++의 static 변수는 프로그램 실행 중 딱 한 번만 초기화됩니다!
        static MTL::Device* device = MTL::CreateSystemDefaultDevice();
        return device;
    }
};
template <typename PR>
struct MetalMemoryBlock {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    MetalMemoryBlock() : pool(nullptr), offset(0), ptr(nullptr) {}
};
template <>
struct ByteMemoryPool<METAL> {
    MTL::Device* device;
    MTL::Buffer* pool;
    size_t marker = 0;
    size_t capacity = 0;
    ByteMemoryPool() : device(nullptr), pool(nullptr), marker(0), capacity(0) {}
    ByteMemoryPool(size_t N) : device(MetalContext::getDevice()), capacity(N) {
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
    MetalMemoryBlock<PR> alloc(size_t count) {
        if(marker >= capacity) {
            std::cout << "[Pool] Memory overflow" << std::endl;
            exit(1);
        }
        size_t aligned = (count*sizeof(PR) + 255) & ~255;
        MetalMemoryBlock<PR> ret;
        ret.pool = pool;
        ret.offset = marker;
        ret.ptr = reinterpret_cast<PR*>(reinterpret_cast<char*>(pool->contents())+marker);
        marker += aligned;
        return ret;
    }
    template <typename PR>
    MetalMemoryBlock<PR> zeros(size_t count) {
        auto ret = alloc<PR>(count);
        memset(ret.ptr, 0, count*sizeof(PR));
        return ret;
    }
    template <typename PR>
    MetalMemoryBlock<PR> allocFill(size_t count, PR fill) {
        auto ret = alloc<PR>(count);
        std::fill(ret.ptr, ret.ptr+count, fill);
        return ret;
    }
};

template <typename BE>
struct FakeMemoryPool {};
template <>
struct FakeMemoryPool<CPU> {
    size_t marker = 0;
    template <typename PR>
    PR* alloc(size_t count) { 
        size_t align = alignof(PR);
        marker += (align - marker % align) % align;
        marker += count * sizeof(PR); 
        return nullptr;
    }
    template <typename PR>
    PR* zeros(size_t count) { return alloc<PR>(count); }
    template <typename PR>
    PR* allocFill(size_t count, PR fill) { return alloc<PR>(count); }
};
template <>
struct FakeMemoryPool<METAL> {
    size_t marker = 0;
    template <typename PR>
    MetalMemoryBlock<PR> alloc(size_t count) { 
        MetalMemoryBlock<PR> ret;
        marker += (count*sizeof(PR) + 255) & ~255;
        return ret;
    }
    template <typename PR>
    MetalMemoryBlock<PR> zeros(size_t count) { return alloc<PR>(count); }
    template <typename PR>
    MetalMemoryBlock<PR> allocFill(size_t count, PR fill) { return alloc<PR>(count); }
};
// TODO: Dynamic Memory Pool
//template <typename T>
//struct DynamicMemoryPool {
//    std::vector<T> pool;
//};

template <typename T>
struct ComputeMemoryPool : MemoryPool<T> {
    ComputeMemoryPool(size_t N) : MemoryPool<T>(N) { }
    void reset() { this->marker = 0; }
};
#include "tinym.hpp"

using Precision = float;


ComputeMemoryPool<Precision> computePool(100000000);


/// TODO: Math ///
#include <Eigen/Dense>

template <typename BE, typename PR>
struct VectorBase {};

template <typename PR>
struct VectorBase<CPU, PR> {
    //Eigen::Map<Eigen::VectorX<PR>> data;
    PR* ptr;
    size_t size;
    VectorBase() : /*data(nullptr, 0)*/ptr(nullptr), size(0) {}
    VectorBase(PR* ptr, size_t size) : /*data(ptr, size)*/ptr(ptr), size(size) {}
    //void setZero() { data.setZero(); }
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
};

template <typename PR>
struct VectorBase<METAL, PR> {
    MTL::Buffer* pool;
    size_t offset;
    PR* ptr;
    size_t size;
    VectorBase() : pool(nullptr), offset(0), ptr(nullptr), size(0) {}
    VectorBase(const MetalMemoryBlock<PR>& block, size_t size) : pool(block.pool), offset(block.offset), ptr(block.ptr), size(size) {}
    auto map() { return Eigen::Map<Eigen::VectorX<PR>>(ptr, size); }
};

//template <typename PR>
//struct Vector<METAL, PR> {
//    
//};

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


template <typename BE, typename PR>
struct Particle {};

template <typename PR>
struct Particle<CPU, PR> {
    using Vector = VectorBase<CPU, PR>;
    Vector x, v, f, m;
    template <typename PoolType>
    void memoryAllocation(PoolType& pool, size_t dataNum, PR mass) {
        x = Vector(pool.template alloc<PR>(dataNum), dataNum);
        v = Vector(pool.template zeros<PR>(dataNum), dataNum);
        f = Vector(pool.template zeros<PR>(dataNum), dataNum);
        m = Vector(pool.template allocFill<PR>(dataNum, mass), dataNum);
    }
};

template <typename PR>
struct Spring {
    Index a, b;
    PR restLength;
    PR kspring;
    Spring(Index a, Index b, PR restLength, PR kspring) : a(a), b(b), restLength(restLength), kspring(kspring) {}
};

template <typename PR>
struct MeshInitializer {
    virtual ~MeshInitializer() = default;
    virtual void initializeGeometry(PR* xptr, Index* facetptr) = 0;
    virtual void initializeSpring(Index* springIndex, PR* springCoef) = 0;
    virtual Index springCounter() = 0;
};

template <typename PR>
struct DeformableMeshGridInitializer : MeshInitializer<PR> {
    Index particleNum1D, vertexNum;
    PR size1D;
    PR kstretch, kshear, kbend;
    PR stretchRestLength, shearRestLength, bendRestLength;
    Index springNum;

    DeformableMeshGridInitializer(Index particleNum1D, PR size1D, PR kstretch, PR kshear, PR kbend)
        : particleNum1D(particleNum1D), vertexNum(particleNum1D*particleNum1D), size1D(size1D), kstretch(kstretch), kshear(kshear), kbend(kbend), 
          stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(bendRestLength = stretchRestLength*2),
          springNum((particleNum1D-1)*particleNum1D*2 + (particleNum1D-1)*(particleNum1D-1)*2 + (particleNum1D-2)*particleNum1D*2) {}

    void initializeGeometry(PR* xptr, Index* facetptr) {
        Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(xptr, 3, vertexNum);
        
        PR halfSize = size1D / 2.0;

        for (int row = 0; row < particleNum1D; ++row) {
            for (int col = 0; col < particleNum1D; ++col) {
                int pid = row * particleNum1D + col; 
                
                PR px = col * stretchRestLength - halfSize;
                PR py = -row * stretchRestLength + halfSize;
                PR pz = rand()/PR(RAND_MAX)/10000.f;
                
                X.col(pid) << px, py, pz;
            }
        }

        Index fIdx = 0;
        for (Index row = 0; row < particleNum1D - 1; ++row) {
            for (Index col = 0; col < particleNum1D - 1; ++col) {
                Index p00 = (row * particleNum1D + col);           
                Index p10 = (row * particleNum1D + col + 1);       
                Index p01 = ((row + 1) * particleNum1D + col);     
                Index p11 = ((row + 1) * particleNum1D + col + 1); 

                facetptr[fIdx++] = p00;
                facetptr[fIdx++] = p10;
                facetptr[fIdx++] = p11;

                facetptr[fIdx++] = p00;
                facetptr[fIdx++] = p11;
                facetptr[fIdx++] = p01;
            }
        }
    }

    void initializeSpring(Index* springIndexPtr, PR* springCoefPtr) {
        auto springIndex = Eigen::Map<Eigen::VectorX<Index>>(springIndexPtr, springNum*2);
        auto springCoef = Eigen::Map<Eigen::VectorX<PR>>(springCoefPtr, springNum*2);
        Index sid = 0;
        for(size_t pid = 0; pid < vertexNum; pid++) {
            auto col = pid % particleNum1D;
            auto row = pid / particleNum1D;

            // stretch
            if(col < particleNum1D-1) {
                //springs.emplace_back(pid, pid+1, stretchRestLength, kstretch); // right
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+1;
                springCoef(sid*2) = stretchRestLength; springCoef(sid*2+1) = kstretch;
                sid++;
            }
            if(row < particleNum1D-1) {
                //springs.emplace_back(pid, pid+particleNum1D, stretchRestLength, kstretch); // bottom
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+particleNum1D;
                springCoef(sid*2) = stretchRestLength; springCoef(sid*2+1) = kstretch;
                sid++;
            }
            // shear
            if(col < particleNum1D-1 && row < particleNum1D-1) {
                ///springs.emplace_back(pid, pid+particleNum1D+1, shearRestLength, kshear); // right-bottom
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+particleNum1D+1;
                springCoef(sid*2) = shearRestLength; springCoef(sid*2+1) = kshear;
                sid++;
            }
            if(col > 0 && row < particleNum1D-1) {
                //springs.emplace_back(pid, pid+particleNum1D-1, shearRestLength, kshear); // left-bottom
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+particleNum1D-1;
                springCoef(sid*2) = shearRestLength; springCoef(sid*2+1) = kshear;
                sid++;
            }
            // bend
            if(col < particleNum1D-2) {
                //springs.emplace_back(pid, pid+2, bendRestLength, kbend); // right-riht
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+2;
                springCoef(sid*2) = bendRestLength; springCoef(sid*2+1) = kbend;
                sid++;
            }
            if(row < particleNum1D-2) {
                //springs.emplace_back(pid, pid+2*particleNum1D, bendRestLength, kbend); // bottom-bottom
                springIndex(sid*2) = pid; springIndex(sid*2+1) = pid+2*particleNum1D;
                springCoef(sid*2) = bendRestLength; springCoef(sid*2+1) = kbend;
                sid++;
            }
        }
    }

    Index springCounter() { return springNum; }
};

template <typename PR>
struct MeshFileInitializer : MeshInitializer<PR> {
    void initilalize() {

    }
};

template <typename BE, typename PR>
struct DeformableMesh {
    using Vector = VectorBase<BE, PR>;
    using Vectorui = VectorBase<BE, Index>;

    MeshInitializer<PR>* initializer;

    Vector x, v, f, m;
    Vector n;
    Vectorui facet;
    Vector fixedParticle;

    Index vertexNum, facetNum;

    Vectorui springIndex;
    Vector springCoef;
    //std::vector<Spring<PR>> springs;
    PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;

    MeshGL<CPU> mesh;

    DeformableMesh(MeshInitializer<PR>* initializer, Index vertexNum, Index facetNum) 
        : initializer(initializer), vertexNum(vertexNum), facetNum(facetNum) {}
    ~DeformableMesh() { delete initializer; }
    template <typename PoolType>
    void memoryAllocation(PoolType& pool, PR mass=0.1) {
        if(x.ptr) return;
        Index vertexDataNum = vertexNum*3;
        Index facetDataNum = facetNum*3;
        Index springDataNum = initializer->springCounter()*2;
        x = Vector(pool.template alloc<PR>(vertexDataNum), vertexDataNum);
        v = Vector(pool.template zeros<PR>(vertexDataNum), vertexDataNum);
        f = Vector(pool.template zeros<PR>(vertexDataNum), vertexDataNum);
        m = Vector(pool.template allocFill<PR>(vertexDataNum, mass), vertexDataNum);
        n = Vector(pool.template alloc<PR>(vertexDataNum), vertexDataNum);
        facet = Vectorui(pool.template alloc<Index>(facetDataNum), facetDataNum);
        fixedParticle = Vector(pool.template allocFill<PR>(vertexNum, 1), vertexNum);
        springIndex = Vectorui(pool.template alloc<Index>(springDataNum), springDataNum);
        springCoef = Vector(pool.template alloc<PR>(springDataNum), springDataNum);
    }
    template <typename PoolType>
    void initialize(PoolType& pool) {
        memoryAllocation(pool);
        initializer->initializeGeometry(x.ptr, facet.ptr);
        initializer->initializeSpring(springIndex.ptr, springCoef.ptr);
        mesh = MeshGL<CPU>(vertexNum, x.ptr, facetNum, facet.ptr, n.ptr);

        
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
    std::vector<DeformableMesh<BE, PR>> deformableMeshes;
    std::vector<Plane> planes;
};


struct IndexPair {
    union {
        struct { size_t first, second; }; 
        struct { size_t point, triangle; }; 
        struct { size_t edge1, edge2; }; 
    };
};
struct Collision {
    std::vector<IndexPair> ptPair;
    std::vector<IndexPair> eePair;
};

// TODO: BroadPhase, SpatialHashing
struct SpatialHashing {
    
};
// TODO: BroadPhase, BVH
struct BVH {

};
// TODO: BroadPhase, LBVH
struct LBVH {
    LBVH() {
        // recieve all points, facets and edges;
    }
};

// TODO: BroadPhase, NarrowPhase, BruteForce
template <typename BE, typename PR>
struct BruteForce {};
template <typename PR>
struct BruteForce<CPU, PR> {
    using Vector = VectorBase<CPU, PR>;
    Collision collision;
    Vector& position;
    BruteForce(Vector& pos) : position(pos) {}
    void collide(const VectorBase<CPU, PR>& other) {
        auto p = position.map();
        auto o = other.map();
        auto P = Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>>(p);
        auto O = Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>>(o);
        for(size_t pi = 0; pi < P.cols; ++pi) for(size_t oi = pi+1; oi < O.cols; ++oi) {
            
        }
    }
};

template <typename BroadPhase, typename NarrowPhase>
struct CollisionPipeline {
    BroadPhase broadPhase;
    NarrowPhase narrowPhase;
};

template <typename BE, typename PR, typename System>
struct Simulator {
    System& system;

    ByteMemoryPool<BE> pool;
    SceneObject<BE, PR> sceneObjects;

    Simulator(System& system) : system(system) {}

    template <typename FileReader>
    void addCloth(FileReader& reader) {

    };

    void addClothGrid(size_t particleNum1D = 200, PR size1D = 100, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5) {
        //particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),{
        //size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2) 
        sceneObjects.deformableMeshes.emplace_back(
                new DeformableMeshGridInitializer<PR>(particleNum1D, size1D, kstretch, kshear, kbend),
                particleNum1D*particleNum1D, // vertexNum
                (particleNum1D-1)*(particleNum1D-1)*2 // facetNum
        );
    }


    template <typename PoolType>
    void memoryAllocation(PoolType& pool) {
        for(auto& deformableMesh : sceneObjects.deformableMeshes) deformableMesh.memoryAllocation(pool);
        //for(auto& plane : sceneObjects.planes) plane.memoryAllocation(pool);
    }


    void initialize() {
        auto fakePool = FakeMemoryPool<BE>();
        memoryAllocation(fakePool);
        std::cout << "[Simulator Init] "<< fakePool.marker << " Bytes are needed for Memory Pool" << std::endl;

        pool = ByteMemoryPool<BE>(fakePool.marker);
        std::cout << "[Simulator Init] Memory pool allocated" << std::endl;
        //system.initialize(pool);
        //memoryAllocation(pool);

        for(auto& deformableMesh : sceneObjects.deformableMeshes) 
            deformableMesh.initialize(pool);
        for(auto& plane : sceneObjects.planes) {}
            
    }

    void update() {
        system.update(sceneObjects);
    }
    
    void draw() {
        //system.draw();

        for(auto& deformableMesh : sceneObjects.deformableMeshes) deformableMesh.mesh.draw();
    }
};


template <typename BE, typename PR>
struct ExplicitSystem {};
#define USE_MEMORY_POOL
#ifndef USE_MEMORY_POOL
template <typename PR>
struct ExplicitSystem<CPU, PR> {
    // VectorBase를 더 이상 사용하지 않습니다!

    size_t particleNum1D, particleNum2D, particleDataNum;
    
    // 💡 개별적인 힙(Heap) 메모리를 가지는 Eigen 행렬들로 직접 정의합니다.
    Eigen::Matrix<PR, 3, Eigen::Dynamic> X, V, F, M, N;
    Eigen::Matrix<PR, 1, Eigen::Dynamic> fixedParticle;
    std::vector<unsigned int> facet; // 인덱스 배열도 std::vector로 독립 할당

    PR mass = 0.1;
    PR h = 1/PR(60);
    size_t subSteps = 40;
    PR subh = h/subSteps;
    PR G = -980; // in cm/s^2
    PR kair = -0.1;
    PR ks = 1e5, kd = 0.1;
    PR size1D;
    PR stretchRestLength, shearRestLength, bendRestLength;

    MeshGL<CPU> cloth;

    ExplicitSystem(size_t particleNum1D = 200, PR size1D = 100) 
        : particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),
        size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2) {}

    // Simulator와의 호환성을 위해 PoolType을 받지만, 실제로는 안 씁니다.
    template <typename PoolType>
    void initialize(PoolType& pool) {
        std::cout << "[System Creation] Try to create Explicit System (Eigen Native)..." << std::endl;
        
        memoryAllocation(pool); // Eigen 내부 할당
        initCloth();

        size_t fIdx = 0;
        for (size_t row = 0; row < particleNum1D - 1; ++row) {
            for (size_t col = 0; col < particleNum1D - 1; ++col) {
                unsigned int p00 = (row * particleNum1D + col);          
                unsigned int p10 = (row * particleNum1D + col + 1);       
                unsigned int p01 = ((row + 1) * particleNum1D + col);     
                unsigned int p11 = ((row + 1) * particleNum1D + col + 1); 

                facet[fIdx++] = p00;
                facet[fIdx++] = p10;
                facet[fIdx++] = p11;

                facet[fIdx++] = p00;
                facet[fIdx++] = p11;
                facet[fIdx++] = p01;
            }
        }

        std::cout << "[System Creation] Facet index computed" << std::endl;

        // 💡 Eigen::Matrix는 열 우선(Column-major)이므로 .data()를 호출하면 OpenGL VBO 구조와 완벽히 호환되는 1차원 포인터가 나옵니다.
        cloth = MeshGL<CPU>(particleNum2D, X.data(), facet.size()/3, facet.data(), N.data());
        
        std::cout << "[System Creation] Explicit System Created" << std::endl;
    }

    template <typename PoolType>
    void memoryAllocation(PoolType& pool) {
        // 넘어온 pool 파라미터는 무시하고, Eigen 라이브러리의 자체 동적 할당을 사용합니다.
        // 각각이 운영체제의 힙 영역 어딘가에 산발적으로 할당됩니다.
        X.resize(3, particleNum2D); X.setZero();
        V.resize(3, particleNum2D); V.setZero();
        F.resize(3, particleNum2D); F.setZero();
        M.resize(3, particleNum2D); M.setConstant(mass);
        N.resize(3, particleNum2D); N.setZero();

        size_t numQuads = (particleNum1D - 1) * (particleNum1D - 1);
        size_t numFacetIndices = numQuads * 6; 
        facet.resize(numFacetIndices);

        fixedParticle.resize(1, particleNum2D); 
        fixedParticle.setConstant(1.0f);
    }

    void initCloth() {
        // 이미 X가 Eigen 객체이므로 Map이 필요 없습니다.
        PR halfSize = size1D / 2.0;

        for (int row = 0; row < particleNum1D; ++row) {
            for (int col = 0; col < particleNum1D; ++col) {
                int pid = row * particleNum1D + col; 
                
                PR px = col * stretchRestLength - halfSize;
                PR py = -row * stretchRestLength + halfSize;
                PR pz = rand()/PR(RAND_MAX)/10000.f;
                
                X.col(pid) << px, py, pz;
            }
        }
    }

    void initClothHorz() {
        PR halfSize = size1D / 2.0;

        for (int row = 0; row < particleNum1D; ++row) {
            for (int col = 0; col < particleNum1D; ++col) {
                int pid = row * particleNum1D + col; 
                
                PR px = col * stretchRestLength - halfSize;
                PR pz = row * stretchRestLength - halfSize;
                PR py = rand()/PR(RAND_MAX)/10000.f;
                
                X.col(pid) << px, py, pz;
            }
        }
    }

    void clearForce() { 
        F.setZero(); // Map 없이 바로 호출
    }
    
    void addForce() {
        // Map 객체 생성 과정도 필요 없어졌습니다.
        F.row(1).array() += G * M.row(1).array();
        F += V * kair;

        auto addSpringForce = [&](size_t idA, size_t idB, PR restLength) {
            auto dx = X.col(idB) - X.col(idA); 
            
            PR len = dx.norm();
            if (len < 1E-9) return;

            auto dv = (V.col(idB) - V.col(idA)).cwiseAbs();
            auto ndx = dx / len;
            auto sf = (ks * (len - restLength) + kd * dv.dot(ndx)) * ndx;

            F.col(idA) += sf;
            F.col(idB) -= sf;
        };

        for(size_t pid = 0; pid < particleNum2D; pid++) {
            auto col = pid % particleNum1D;
            auto row = pid / particleNum1D;

            if(col < particleNum1D-1) addSpringForce(pid, pid+1, stretchRestLength); 
            if(row < particleNum1D-1) addSpringForce(pid, pid+particleNum1D, stretchRestLength); 
            if(col < particleNum1D-1 && row < particleNum1D-1) addSpringForce(pid, pid+particleNum1D+1, shearRestLength); 
            if(col > 0 && row < particleNum1D-1) addSpringForce(pid, pid+particleNum1D-1, shearRestLength); 
            if(col < particleNum1D-2) addSpringForce(pid, pid+2, bendRestLength); 
            if(row < particleNum1D-2) addSpringForce(pid, pid+2*particleNum1D, bendRestLength); 
        }
    }

    void update() {
        for(size_t i = 0; i < subSteps; i++) {
            clearForce();
            addForce();

            // Map 객체를 만들지 않으므로 코드가 더 직관적이 되었습니다.
            V.array() += (F.array() / M.array()).rowwise() * fixedParticle.array() * subh;
            X.array() += V.array().rowwise() * fixedParticle.array() * subh;
        }

        cloth.updateBuffer(X.data());
    }

    void draw() {
        cloth.draw();
    }
};
#else
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

    void initialize(SceneObject<CPU, PR>& sceneObjects) {
        std::cout << "[System Creation] Try to create Explicit System..." << std::endl;
    }

    void clearForce(SceneObject<CPU, PR>& sceneObjects) { 
        for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) deformableMesh.f.map().setZero(); 
    }
    
    void addForce(SceneObject<CPU, PR>& sceneObjects) {
        // View change: (3Nx1) to (3xN)
        for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) {
            Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(deformableMesh.x.ptr, 3, deformableMesh.vertexNum);
            Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(deformableMesh.v.ptr, 3, deformableMesh.vertexNum);
            Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(deformableMesh.f.ptr, 3, deformableMesh.vertexNum);
            Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(deformableMesh.m.ptr, 3, deformableMesh.vertexNum);
            
            // add gravity
            F.row(1) += G * M.row(1);

            // add air drag
            F += V * kair;

            // add spring
            // 스프링 포스 계산용 람다 함수 (인자로 ID를 받습니다)
            auto addSpringForce = [&](size_t idA, size_t idB, PR restLength, PR kspring) {
                // .col()을 쓰면 Eigen::Vector3f 처럼 다룰 수 있습니다!
                auto dx = X.col(idB) - X.col(idA); 
                
                PR len = dx.norm();
                if (len < 1E-9) return; // 0 나누기 방지

                auto dv = (V.col(idB) - V.col(idA)).cwiseAbs();
                auto ndx = dx / len;
                auto sf = (kspring * (len - restLength) + kd * dv.dot(ndx)) * ndx;

                // 작용-반작용 법칙: A에는 더하고 B에는 뺍니다 (제자리 갱신)
                F.col(idA) += sf;
                F.col(idB) -= sf;
            };
            Index springNum = deformableMesh.springIndex.size/2;
            for(Index i = 0; i < springNum; ++i) {
                addSpringForce(
                        deformableMesh.springIndex.map()[i*2],
                        deformableMesh.springIndex.map()[i*2+1], 
                        deformableMesh.springCoef.map()[i*2], 
                        deformableMesh.springCoef.map()[i*2+1]
                );
            }
        }
    }
    
    void update(SceneObject<CPU, PR>& sceneObjects) {
        for(size_t i = 0; i < subSteps; i++) {
            for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) {
                Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> X(deformableMesh.x.ptr, 3, deformableMesh.vertexNum);
                Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> V(deformableMesh.v.ptr, 3, deformableMesh.vertexNum);
                Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> F(deformableMesh.f.ptr, 3, deformableMesh.vertexNum);
                Eigen::Map<Eigen::Matrix<PR, 3, Eigen::Dynamic>> M(deformableMesh.m.ptr, 3, deformableMesh.vertexNum);

                Eigen::Map<Eigen::Matrix<PR, 1, Eigen::Dynamic>> Mask(deformableMesh.fixedParticle.ptr, 1, deformableMesh.vertexNum);

                clearForce(sceneObjects);
                addForce(sceneObjects);

                V.array() += (F.array() / M.array()).rowwise() * Mask.array() * subh;
                X.array() += V.array().rowwise() * Mask.array() * subh;

            }
        }
        for(DeformableMesh<CPU, PR>& deformableMesh : sceneObjects.deformableMeshes) 
            deformableMesh.mesh.updateBuffer(deformableMesh.x.ptr);
    }
};
#endif


template <typename PR>
struct ExplicitSystem<METAL, PR> {
    using Vector = VectorBase<METAL, PR>;
    using Vectorui = VectorBase<METAL, unsigned int>;

    // Metal vars
    MTL::Device* device;
    MTL::CommandQueue* commandQueue;
    MTL::ComputePipelineState* forcePSO;
    MTL::ComputePipelineState* springForcePSO;
    MTL::ComputePipelineState* integratePSO;

    // Sim vars
    PR h = 1/PR(60);
    size_t subSteps = 50;
    PR subh = h/subSteps;
    PR G = -980; // in cm/s^2
    PR kair = -0.1;
    PR kd = 0.1;
    // TODO: bending은 더 강하게 줘야할 듯. - 교수님


    ExplicitSystem(PR h=1/PR(60), Index subSteps=50) 
        : device(MetalContext::getDevice()), h(h), subSteps(subSteps), subh(h/subSteps) {
        std::cout << "[System Creation] Try to create Explicit System..." << std::endl;
        std::cout << "  - Connecting device..." << std::endl;
        //device = MTL::CreateSystemDefaultDevice();
        //pool = ByteMemoryPool<METAL>(device, 50*1024*1024*sizeof(PR));

        std::cout << "  - Creating a new command queue..." << std::endl;
        commandQueue = device->newCommandQueue();

        std::cout << "  - Creating a new command pipeline state object (PSO)..." << std::endl;
        NS::Error* error = nullptr;
        // find own metal kernel library
        NS::String* libPath = NS::String::string("physics.metallib", NS::UTF8StringEncoding);
        MTL::Library* library = device->newLibrary(libPath, &error);
        if (!library) {
            std::cout << "[Metal Error] Failed to load physics.metallib!\n";
            exit(1);
        }
        // find desired kernel
        NS::String* forceFuncName = NS::String::string("compute_forces", NS::UTF8StringEncoding);
        MTL::Function* forceFunc = library->newFunction(forceFuncName);
        if (!forceFunc) {
            std::cout << "[Metal Error] Failed to find compute_forces function!\n";
            exit(1);
        }
        // create a PSO (Heavy...)
        forcePSO = device->newComputePipelineState(forceFunc, &error);
        if (!forcePSO) {
            std::cout << "[Metal Error] Failed to create Pipeline State! forcePSO\n";
            exit(1);
        }

        NS::String* springForceFuncName = NS::String::string("compute_spring_forces", NS::UTF8StringEncoding);
        MTL::Function* springForceFunc = library->newFunction(springForceFuncName);
        if(!springForceFunc) {
            std::cout << "[Metal Error] Failed to create Pipeline State! forcePSO\n";
            exit(1);
        }
        springForcePSO = device->newComputePipelineState(springForceFunc, &error);
        if (!springForcePSO) {
            std::cout << "[Metal Error] Failed to create Pipeline State! forcePSO\n";
            exit(1);
        }

        NS::String* integrateFuncName = NS::String::string("integrate", NS::UTF8StringEncoding);
        MTL::Function* integrateFunc = library->newFunction(integrateFuncName);
        if (!integrateFunc) {
            std::cout << "[Metal Error] Failed to find compute_forces function!\n";
            exit(1);
        }
        // create a PSO (Heavy...)
        integratePSO = device->newComputePipelineState(integrateFunc, &error);
        if (!integratePSO) {
            std::cout << "[Metal Error] Failed to create Pipeline State! integratePSO\n";
            exit(1);
        }
        
        // release all temporal objects
        library->release();
        libPath->release();
        forceFuncName->release();
        forceFunc->release();
        integrateFuncName->release();
        integrateFunc->release();
    }
    
    // 1. C++에도 파라미터 구조체를 정의해 둡니다. (메모리 구조 일치)
// 💡 아주 심플해진 범용 파라미터
    struct SimParams {
        float subh, G, kair, kd;
        uint vertexNum; 
    };

    // 2. update() 함수 수정
    void update(SceneObject<METAL, PR>& sceneObjects) {
        MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
        MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();

        for(auto& deformableMesh : sceneObjects.deformableMeshes) {
            // 변수들을 패킹합니다.
            SimParams params = { subh, G, kair, kd, (uint)deformableMesh.vertexNum };

            // [버퍼는 루프 밖에서 딱 한 번만 바인딩합니다!]
            computeEncoder->setBuffer(deformableMesh.x.pool, deformableMesh.x.offset, 0);
            computeEncoder->setBuffer(deformableMesh.v.pool, deformableMesh.v.offset, 1);
            computeEncoder->setBuffer(deformableMesh.f.pool, deformableMesh.f.offset, 2);
            computeEncoder->setBuffer(deformableMesh.m.pool, deformableMesh.m.offset, 3);
            computeEncoder->setBuffer(deformableMesh.fixedParticle.pool, deformableMesh.fixedParticle.offset, 4);
            
            // 4KB 이하의 작은 데이터는 버퍼를 안 만들고 setBytes로 즉시 꽂을 수 있습니다.
            computeEncoder->setBytes(&params, sizeof(SimParams), 5);

            computeEncoder->setBuffer(deformableMesh.springIndex.pool, deformableMesh.springIndex.offset, 6);
            computeEncoder->setBuffer(deformableMesh.springCoef.pool, deformableMesh.springCoef.offset, 7);


            // 💡 서브스텝 만큼 GPU 파이프라인을 교차 실행합니다!
            for(size_t i = 0; i < subSteps; i++) {
                { // force overwrite with gravity + air drag
                    MTL::Size gridSize = MTL::Size(deformableMesh.vertexNum, 1, 1);
                    MTL::Size threadGroupSize = MTL::Size(std::min((size_t)forcePSO->maxTotalThreadsPerThreadgroup(), (size_t)deformableMesh.vertexNum), 1, 1);
                    computeEncoder->setComputePipelineState(forcePSO);
                    computeEncoder->dispatchThreads(gridSize, threadGroupSize);
                }
                { // spring force add
                    MTL::Size gridSize = MTL::Size(deformableMesh.springIndex.size/2, 1, 1);
                    MTL::Size threadGroupSize = MTL::Size(std::min((size_t)springForcePSO->maxTotalThreadsPerThreadgroup(), deformableMesh.springIndex.size/2), 1, 1);
                    computeEncoder->setComputePipelineState(springForcePSO);
                    computeEncoder->dispatchThreads(gridSize, threadGroupSize);
                }
                { // integration
                    MTL::Size gridSize = MTL::Size(deformableMesh.vertexNum, 1, 1);
                    MTL::Size threadGroupSize = MTL::Size(std::min((size_t)integratePSO->maxTotalThreadsPerThreadgroup(), (size_t)deformableMesh.vertexNum), 1, 1);
                    computeEncoder->setComputePipelineState(integratePSO);
                    computeEncoder->dispatchThreads(gridSize, threadGroupSize);
                }
            }

            computeEncoder->endEncoding();
            commandBuffer->commit();
            commandBuffer->waitUntilCompleted(); // 전체 30스텝이 다 끝날 때까지 CPU는 휴식
        //cloth.updateBuffer(x.ptr);
        }

        for(auto& deformableMesh : sceneObjects.deformableMeshes) 
            deformableMesh.mesh.updateBuffer(deformableMesh.x.ptr);

    }
};

// TODO: BVH


// TODO: Spatial Hash


// simulation mamage
constexpr int particleN = 20;
constexpr int groundN = 1;



#include <chrono> // 시간 관련 라이브러리
void addTest_CPU() {
    std::cout << "----- Start: Addition Test -----" << std::endl;
    constexpr size_t num = 100000;
    std::cout << "----- Info: Vec size = (" << num << ") -----" << std::endl;
    MemoryPool<float> pool(num*10);

    auto start = std::chrono::steady_clock::now();
    VectorBase<CPU, float> x(pool.zeros(num), num);
    VectorBase<CPU, float> y(pool.zeros(num), num);
    VectorBase<CPU, float> c(pool.alloc(num), num);
    c.map() = x.map()+y.map();
    //std::cout << "Vector c: " << c.data << std::endl;
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    float dummy = c.map()[0];
    std::cout << "Ex1 on my Vec: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    float* mem1 = pool.zeros(num);
    for(int i = 0; i < num; i++) *(mem1+i) = i;
    float* mem2 = pool.zeros(num);
    for(int i = 0; i < num; i++) *(mem2+i) = i+num;
    VectorBase<CPU, float> x1(mem1, num);
    VectorBase<CPU, float> x2(mem2, num);
    c.map() = x1.map()+x2.map();
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += c.map()[0];
    std::cout << "Ex2 on my Vec: " << diff.count() << " us" << std::endl;
    //std::cout << "Vector c1: " << c1.data << std::endl;

    start = std::chrono::steady_clock::now();
    Eigen::VectorXf xe1 = Eigen::VectorXf::Zero(num), xe2 = Eigen::VectorXf::Zero(num);
    Eigen::VectorXf ce1 = xe1+xe2;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    std::cout << "Ex1 on Eigen: " << diff.count() << " us" << std::endl;
    dummy += ce1[0];

    start = std::chrono::steady_clock::now();
    Eigen::VectorXf xe3(num), xe4(num);
    for(int i = 0; i < num; i++) xe3.coeffRef(i) = i;
    for(int i = 0; i < num; i++) xe4.coeffRef(i) = i+num;
    Eigen::VectorXf ce2 = xe3+xe4;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += ce2[0];
    std::cout << "Ex2 on Eigen: " << diff.count() << " us" << std::endl;

    std::cout<< "dummy: " << dummy << std::endl;
    std::cout << "----- End: Addition Test -----" << std::endl << std::endl;
}
void mulTest_CPU() {
    std::cout << "----- Start: MV Multiplication Test -----" << std::endl;
    constexpr size_t num = 300;
    std::cout << "----- Info: Mat size = (" << num << "x" << num << ") * Vec size = (" << num << ") -----" << std::endl;
    constexpr size_t num2 = num*num;
    MemoryPool<float> pool(num2*10);

    auto start = std::chrono::steady_clock::now();
    Matrix<CPU, float> m(pool.zeros(num2), num, num);
    VectorBase<CPU, float> a(pool.zeros(num), num);
    VectorBase<CPU, float> b(pool.alloc(num), num);
    b.map() = m.map()*a.map();
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    float dummy = b.map()[0];
    std::cout << "Ex1 on my Mat*Vec: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    float* mem1 = pool.zeros(num2);
    for(size_t i = 0; i < num2; i++) *(mem1+i) = i;
    Matrix<CPU, float> m2(mem1, num, num);
    float* mem2 = pool.zeros(num);
    for(size_t i = 0; i < num; i++) *(mem2+i) = i;
    VectorBase<CPU, float> a2(mem2, num);
    b.map() = m2.map()*a2.map();
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy = b.map()[0];
    std::cout << "Ex2 on my Mat*Vec: " << diff.count() << " us" << std::endl;


    // --- 여기서부터 채워진 Eigen 테스트 부분입니다 ---
    start = std::chrono::steady_clock::now();
    Eigen::MatrixXf me1 = Eigen::MatrixXf::Zero(num, num);
    Eigen::VectorXf ae1 = Eigen::VectorXf::Zero(num);
    Eigen::VectorXf be1 = me1 * ae1;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += be1[0];
    std::cout << "Ex1 on Eigen: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    Eigen::MatrixXf me2(num, num);
    // Eigen의 내부 1차원 배열 포인터(data())를 이용해 mem1, mem2와 완벽히 동일한 방식으로 초기화합니다.
    for(size_t i = 0; i < num2; i++) *(me2.data() + i) = i;
    Eigen::VectorXf ae2(num);
    for(size_t i = 0; i < num; i++) *(ae2.data() + i) = i;
    Eigen::VectorXf be2 = me2 * ae2;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += be2[0];
    std::cout << "Ex2 on Eigen: " << diff.count() << " us" << std::endl;

    // 컴파일러의 Dead Code Elimination 최적화를 막기 위한 더미 출력
    std::cout << "dummy: " << dummy << std::endl;
    std::cout << "----- End: MV Multiplication Test -----" << std::endl << std::endl;
}
#include <random>
void sparseMatrixMulTest_CPU() {
    std::cout << "----- Start: SMV Multiplication Test -----" << std::endl;
    constexpr size_t size = 10000;
    constexpr float nnzratio = 0.1;
    constexpr size_t nnz = size * size * nnzratio; 
    std::cout << "----- Info: Sparse Mat size = (" << size << "x" << size << ") * Vec size = (" << size << "), non-zeros ratio = " << nnzratio << " -----" << std::endl;
    // 1000 * 1000 의 10% = 100,000 개의 0이 아닌 요소(Non-zeros)

    // 1. 임의의 Triplet 데이터 생성 (타이머 바깥에서 준비)
    std::vector<Eigen::Triplet<float>> triplets;
    triplets.reserve(nnz);
    
    // 고정 시드를 사용하여 매번 동일한 패턴의 난수 생성
    std::mt19937 gen(42); 
    std::uniform_int_distribution<int> dist(0, size - 1);
    std::uniform_real_distribution<float> val_dist(-1.0f, 1.0f);

    for (size_t i = 0; i < nnz; ++i) {
        triplets.emplace_back(dist(gen), dist(gen), val_dist(gen));
    }

    MemoryPool<float> pool(size * 10);
    float dummy = 0.0f;

    // ==========================================
    // Ex1: 내 커스텀 SparseMatrix & Vector 연산
    // ==========================================
    // 데이터 세팅
    SparseMatrix<CPU, float> mySm(triplets, size, size);
    float* memX = pool.zeros(size);
    for(int i = 0; i < size; i++) memX[i] = 1.0f; // 벡터를 1.0으로 초기화
    VectorBase<CPU, float> myX(memX, size);
    VectorBase<CPU, float> myRes(pool.alloc(size), size);

    auto start = std::chrono::steady_clock::now();
    
    // 행렬-벡터 곱셈 실행
    myRes.map() = mySm.map() * myX.map();
    
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Sparse * Vec on my Struct: " << diff.count() << " us" << std::endl;
    dummy += myRes.map()[0];

    // ==========================================
    // Ex2: 순수 Eigen SparseMatrix & Vector 연산
    // ==========================================
    // 데이터 세팅
    Eigen::SparseMatrix<float> eigenSm(size, size);
    eigenSm.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::VectorXf eigenX = Eigen::VectorXf::Ones(size);

    start = std::chrono::steady_clock::now();
    
    // 행렬-벡터 곱셈 실행
    Eigen::VectorXf eigenRes = eigenSm * eigenX;
    
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Sparse * Vec on pure Eigen: " << diff.count() << " us" << std::endl;
    dummy += eigenRes[0];

    std::cout << "dummy: " << dummy << std::endl;
    std::cout << "----- End: SMV Multiplication Test -----" << std::endl << std::endl;
}

void test() {
    addTest_CPU();
    mulTest_CPU();
    sparseMatrixMulTest_CPU();
}


int main() {
    
    std::cout << "Render" << std::endl;

    window = new YGLWindow(640, 480, "ysim");

    // Test
    //test();


    // Add Ground
    

    // Add Cloth
//#define METAL_SYSTEM

#ifdef METAL_SYSTEM
    using Backend = METAL;
#else
    using Backend = CPU;
#endif

    //ByteMemoryPool<METAL> pool(50*1024*1024*sizeof(Precision));
    Precision h = 1/Precision(60);
    Index subSteps = 50;
    ExplicitSystem<Backend, Precision> system(h, subSteps);
    Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>> simulator(system);
    //system.initialize(pool);

    Index particleNum1D = 200;
    Precision size1D = 100;
    Precision kstretch = 1e5;
    Precision kshear = 1e5;
    Precision kbend = 2e5;
    simulator.addClothGrid(particleNum1D, size1D, kstretch, kshear, kbend);

    simulator.initialize();

#ifdef USE_MEMORY_POOL
    simulator.sceneObjects.deformableMeshes[0].fixedParticle.map()[0] = 0.f;
    simulator.sceneObjects.deformableMeshes[0].fixedParticle.map()[particleNum1D-1] = 0.f;
    //system.fixedParticle.map()[0] = 0.f;
    //system.fixedParticle.map()[system.particleNum1D-1] = 0.f;
    //simulator.system.fixedParticle.map()[0] = 0.f;
    //simulator.system.fixedParticle.map()[simulator.system.particleNum1D-1] = 0.f;
#else
    simulator.system.fixedParticle[0] = 0.f;
    simulator.system.fixedParticle[simulator.system.particleNum1D-1] = 0.f;
#endif




    Program shader;
    shader.loadShader("shader.vert", "shader.frag");

    camera.setPosition(tinym::vec3(0, 0, 200));

    camera.glfwSetCallbacks(window->getGLFWWindow());

    //glfwSetWindowUserPointer(window->getGLFWWindow(), &system);
    glfwSetWindowUserPointer(window->getGLFWWindow(), &(simulator));
    auto keyCallback = [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto* simulator = static_cast<Simulator<Backend, Precision, ExplicitSystem<Backend, Precision>>*>(glfwGetWindowUserPointer(window));

        if(key == GLFW_KEY_0 && action == GLFW_PRESS) {
            //sys->initCloth();
        } else if(key == GLFW_KEY_9 && action == GLFW_PRESS) {
            //sys->initClothHorz();
#ifdef USE_MEMORY_POOL
        } else if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
            //sys->fixedParticle.map()[0] = !((bool)sys->fixedParticle.map()[0]);
            simulator->sceneObjects.deformableMeshes[0].fixedParticle.map()[0] = !((bool)simulator->sceneObjects.deformableMeshes[0].fixedParticle.map()[0]);
        } else if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
            //sys->fixedParticle.map()[sys->particleNum1D-1] = !((bool)sys->fixedParticle.map()[sys->particleNum1D-1]);
            simulator->sceneObjects.deformableMeshes[0].fixedParticle.map()[200-1] = !((bool)simulator->sceneObjects.deformableMeshes[0].fixedParticle.map()[200-1]);
        }
#else
        } else if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
            sys->fixedParticle[0] = !((bool)sys->fixedParticle[0]);
        } else if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
            sys->fixedParticle[sys->particleNum1D-1] = !((bool)sys->fixedParticle[sys->particleNum1D-1]);
        }
#endif
    };
    glfwSetKeyCallback(window->getGLFWWindow(), keyCallback);

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
        glClearColor(0, 0, 0.3, 0);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //float M[16] = {1, 0, 0, 0,   0, 1, 0, 0,   0, 0, 1, 0,   0, -0.5, 0, 1};
        tinym::mat4 M(1);
        //M[3][1] = -0.5; // translation
        tinym::mat4 V = camera.lookAt();
        shader.setUniform("M", M);
        shader.setUniform("V", V);
        shader.setUniform("P", camera.perspective(window->aspect(), 0.1f, 1000.f));
        glfwSwapInterval(1);

        //system.draw();
        simulator.draw();
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


