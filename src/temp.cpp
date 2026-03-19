```{main.cpp}
...
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

    //ByteMemoryPool<METAL>& pool;

    size_t particleNum1D, particleNum2D, particleDataNum;
    //Vector x, v, f, m;
    //Vector n;
    PR mass = 0.1;
    PR h = 1/PR(60);
    size_t subSteps = 50;
    PR subh = h/subSteps;
    PR G = -980; // in cm/s^2
    PR kair = -0.1;
    PR ks = 1e5, kd = 0.1;
    // TODO: bending은 더 강하게 줘야할 듯. - 교수님
    PR size1D;
    PR stretchRestLength, shearRestLength, bendRestLength;
    //Vectorui facet;

    //MeshGL<CPU> cloth;
    //Vector fixedParticle;


    ExplicitSystem(size_t particleNum1D = 200, PR size1D = 100) 
        : device(MetalContext::getDevice()), particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),
        size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2) {
        std::cout << "[System Creation] Try to create Explicit System..." << std::endl;
        std::cout << "  - Connecting device..." << std::endl;
        //device = MTL::CreateSystemDefaultDevice();
        //pool = ByteMemoryPool<METAL>(device, 50*1024*1024*sizeof(PR));

        std::cout << "  - Creating a new command queue..." << std::endl;
        commandQueue = device->newCommandQueue();

        std::cout << "  - Creating a new command pipeline state object (PSO)..." << std::endl;
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
    struct SimParams {
        float subh, G, kair, ks, kd, stretchRest, shearRest, bendRest;
        uint particleNum1D;
    };

    // 2. update() 함수 수정
    void update(SceneObject<METAL, PR>& sceneObjects) {
        MTL::CommandBuffer* commandBuffer = commandQueue->commandBuffer();
        MTL::ComputeCommandEncoder* computeEncoder = commandBuffer->computeCommandEncoder();

        for(auto& deformableMesh : sceneObjects.deformableMeshes) {
            // 변수들을 패킹합니다.
            SimParams params = { subh, G, kair, ks, kd, stretchRestLength, shearRestLength, bendRestLength, (uint)particleNum1D };

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
                    MTL::Size gridSize = MTL::Size(particleNum2D, 1, 1);
                    MTL::Size threadGroupSize = MTL::Size(std::min((size_t)forcePSO->maxTotalThreadsPerThreadgroup(), particleNum2D), 1, 1);
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
                    MTL::Size gridSize = MTL::Size(particleNum2D, 1, 1);
                    MTL::Size threadGroupSize = MTL::Size(std::min((size_t)integratePSO->maxTotalThreadsPerThreadgroup(), particleNum2D), 1, 1);
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
...
```
```{physics.metal}
#include <metal_stdlib>
using namespace metal;

// CPU에서 한 번에 넘겨줄 설정값들 (C++의 구조체와 동일해야 합니다)
struct SimParams {
    float subh;
    float G;
    float kair;
    float ks;
    float kd;
    float stretchRest;
    float shearRest;
    float bendRest;
    uint particleNum1D;
};


// 스프링 힘을 계산하는 헬퍼 함수
inline float3 calc_spring(float3 p0, float3 v0, float3 p1, float3 v1, float rest_len, float ks, float kd) {
    float3 dx = p1 - p0;
    float3 dv = v1 - v0;
    float len = length(dx);
    if (len < 1e-7) return float3(0.0); // 0 나누기 방지
    float3 ndx = dx / len;
    return (ks * (len - rest_len) + kd * dot(dv, ndx)) * ndx;
}

kernel void compute_spring_forces(
    device const packed_float2* springIndex [[buffer(6)]],
    device const packed_float2* springCoef [[buffer(7)]],
    device packed_float3* f [[buffer(2)]],
    constant SimParams& params [[buffer(5)]],
    uint id [[thread_position_in_grid]]
) {
    // 1. 내 스레드(id)가 담당하는 스프링 정보 읽기
    uint idA = springIndices[id].x;
    uint idB = springIndices[id].y;
    float restLength = springParams[id].x;
    float kspring = springParams[id].y;

    // 2. 물리 수식 계산 (CPU 버전과 동일)
    float3 dx = X[idB] - X[idA];
    float len = length(dx);
    if(len < 1e-9) return;

    float3 dv = V[idB] - V[idA];
    float3 ndx = dx / len;
    float3 sf = (kspring * (len - restLength) + 0.1 * dot(dv, ndx)) * ndx; // kd=0.1 고정 가정

    // 3. 🚨 대망의 Atomic Add (동시 접근 방어)
    // 입자 A에는 더해주고, 입자 B에는 빼줍니다.
    atomic_fetch_add_explicit(&F_x[idA], sf.x, memory_order_relaxed);
    atomic_fetch_add_explicit(&F_y[idA], sf.y, memory_order_relaxed);
    atomic_fetch_add_explicit(&F_z[idA], sf.z, memory_order_relaxed);

    atomic_fetch_sub_explicit(&F_x[idB], sf.x, memory_order_relaxed);
    atomic_fetch_sub_explicit(&F_y[idB], sf.y, memory_order_relaxed);
    atomic_fetch_sub_explicit(&F_z[idB], sf.z, memory_order_relaxed);
}


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
    uint N = params.particleNum1D;
    if (id >= N * N) return;

    int row = id / N;
    int col = id % N;

    // packed_float3를 연산이 빠른 float3로 캐스팅하여 사용
    float3 pos = x[id];
    float3 vel = v[id];

    // 1. 중력 & 공기 저항
    float3 force = float3(0.0, params.G * m[id], 0.0) + vel * params.kair;

    //// 1D 인덱스를 구하는 람다 함수
    //auto get_idx = [N](int r, int c) { return r * N + c; };

    //// 2. 스프링 힘 (Gather) - 조건에 맞는 이웃이 있으면 힘을 누적합니다.
    //// [Stretch] 상하좌우
    //if (col > 0)   force += calc_spring(pos, vel, x[get_idx(row, col-1)], v[get_idx(row, col-1)], params.stretchRest, params.ks, params.kd);
    //if (col < N-1) force += calc_spring(pos, vel, x[get_idx(row, col+1)], v[get_idx(row, col+1)], params.stretchRest, params.ks, params.kd);
    //if (row > 0)   force += calc_spring(pos, vel, x[get_idx(row-1, col)], v[get_idx(row-1, col)], params.stretchRest, params.ks, params.kd);
    //if (row < N-1) force += calc_spring(pos, vel, x[get_idx(row+1, col)], v[get_idx(row+1, col)], params.stretchRest, params.ks, params.kd);

    //// [Shear] 대각선
    //if (col > 0 && row > 0)     force += calc_spring(pos, vel, x[get_idx(row-1, col-1)], v[get_idx(row-1, col-1)], params.shearRest, params.ks, params.kd);
    //if (col < N-1 && row > 0)   force += calc_spring(pos, vel, x[get_idx(row-1, col+1)], v[get_idx(row-1, col+1)], params.shearRest, params.ks, params.kd);
    //if (col > 0 && row < N-1)   force += calc_spring(pos, vel, x[get_idx(row+1, col-1)], v[get_idx(row+1, col-1)], params.shearRest, params.ks, params.kd);
    //if (col < N-1 && row < N-1) force += calc_spring(pos, vel, x[get_idx(row+1, col+1)], v[get_idx(row+1, col+1)], params.shearRest, params.ks, params.kd);

    //// [Bend] 2칸 너머
    //if (col > 1)   force += calc_spring(pos, vel, x[get_idx(row, col-2)], v[get_idx(row, col-2)], params.bendRest, params.ks, params.kd);
    //if (col < N-2) force += calc_spring(pos, vel, x[get_idx(row, col+2)], v[get_idx(row, col+2)], params.bendRest, params.ks, params.kd);
    //if (row > 1)   force += calc_spring(pos, vel, x[get_idx(row-2, col)], v[get_idx(row-2, col)], params.bendRest, params.ks, params.kd);
    //if (row < N-2) force += calc_spring(pos, vel, x[get_idx(row+2, col)], v[get_idx(row+2, col)], params.bendRest, params.ks, params.kd);

    // 내 메모리에만 기록!
    f[id] = force;
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
    if (id >= params.particleNum1D * params.particleNum1D) return;

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
```
지금 이런 구조인데, 너 말대로 force를 atomic add할 때 element-wise로 넣으려면 버퍼를 또 새로 지정해야 하잖아. 그렇게 하기보단 우선은 3vector atomic add가 안돼?
