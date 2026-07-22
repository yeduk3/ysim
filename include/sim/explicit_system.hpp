#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

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
        // CPU backend is not v1-shipping (ARCHITECTURE §4.5); GL upload here
        // would now go through MeshRenderState, but the path is uninstantiated.
        (void)sceneObjects;
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
    MTL::ComputePipelineState* refCopyPosPSO = nullptr;
    MTL::ComputePipelineState* refCopyForcePSO = nullptr;
    MTL::ComputePipelineState* snapshotXPrevPSO = nullptr;
    // Dedicated shared buffer of pre-resolved {queryGlobalVid,
    // targetGlobalVid} uint2 pairs for the reference-point constraint
    // kernels. Grown on demand; never freed (lives as long as the
    // System). Not pool-allocated so it survives pool resets.
    MTL::Buffer* refPairBuf = nullptr;
    size_t refPairCap = 0;

    // Anomaly flag (single uint, shared storage). The integrate kernels'
    // world-bounds guard stores 1 here whenever any vertex had to be
    // sanitized (NaN/Inf or escaped the world box). Simulator::update
    // polls it once per frame and auto-pauses — one GPU bool, one host if,
    // no extra sync (worst case the pause lands a frame late). Allocated
    // lazily in update() so construction order vs the Metal pool doesn't
    // matter.
    VectorBase<METAL, uint32_t> anomalyFlag;

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
        refCopyPosPSO   = MetalKernelContext::getPSO("ref_constraint_copy_pos");
        refCopyForcePSO = MetalKernelContext::getPSO("ref_constraint_copy_force");
        snapshotXPrevPSO = MetalKernelContext::getPSO("snapshot_xprev_range");
    }

    // GPU replacement for the per-substep CPU xPrev memcpy loop. For each
    // non-Float/non-Kinematic mesh, dispatch a range copy xPrev := x over its
    // contiguous packed-vertex range [statesOffsets[i], +count). All dispatches
    // ride the current encoder (no commitAndWait) so the substep loop stays
    // async under None/PerFrame. m.state.x/xPrev are subspan views into the
    // global packed buffers, so a global-range copy IS the per-mesh copy.
    void snapshotXPrev(Scene<METAL, PR>& scene) {
        auto& off = Scene<METAL, PR>::packedMeshData.statesOffsets;
        if (!off.ptr || off.size == 0 || !snapshotXPrevPSO) return;
        auto& gx     = Scene<METAL, PR>::packedMeshData.x;
        auto& gxprev = Scene<METAL, PR>::packedMeshData.xPrev;
        for (Index i = 0; i < (Index)scene.meshes.size(); ++i) {
            auto& m = scene.meshes[i];
            // Mirror the old CPU loop's exclusions exactly.
            if (m.behaviorType == BehaviorType::Float) continue;
            if (m.behaviorType == BehaviorType::Kinematic) continue;
            if (!m.state.x.ptr || !m.state.xPrev.ptr) continue;
            uint base  = (uint)off.ptr[i];                  // float3 units
            uint count = (uint)off.ptr[i + 1] - base;
            if (count == 0) continue;
            MetalGlobalContext::setBuffer(gx, 0);
            MetalGlobalContext::setBuffer(gxprev, 1);
            MetalGlobalContext::setBytes(base, 2);
            MetalGlobalContext::setBytes(count, 3);
            MetalGlobalContext::dispatchThreads(snapshotXPrevPSO, count);
        }
    }

    // Resolve Scene::referenceConstraints into global vertex-index pairs
    // and upload them into refPairBuf. Returns the valid pair count.
    // objId == compacted mesh id == statesOffsets subscript (same key
    // integrate_cloth binds as `oid`); a global vid is
    // statesOffsets[objId] + localPhysicsVid. Stale/out-of-range
    // constraints (object deleted, vid past the mesh) are skipped.
    uint buildRefPairs(Scene<METAL, PR>& scene) {
        auto& cons = Scene<METAL, PR>::referenceConstraints;
        if (cons.empty()) return 0;
        auto& off = Scene<METAL, PR>::packedMeshData.statesOffsets;
        if (!off.ptr || off.size == 0) return 0;
        const uint32_t numMeshes = (uint32_t)Scene<METAL, PR>::numMeshes;

        static std::vector<uint32_t> scratch;  // 2 entries per pair
        scratch.clear();
        auto resolve = [&](Index obj, Index vid, uint32_t& out) -> bool {
            if (obj + 1 >= off.size || obj >= numMeshes) return false;
            uint32_t base = (uint32_t)off.ptr[obj];
            uint32_t cnt  = (uint32_t)off.ptr[obj + 1] - base;
            if ((uint32_t)vid >= cnt) return false;
            out = base + (uint32_t)vid;
            return true;
        };
        for (const auto& c : cons) {
            uint32_t gq, gt;
            if (!resolve(c.objPair.query,  c.vertexPair.query,  gq)) continue;
            if (!resolve(c.objPair.target, c.vertexPair.target, gt)) continue;
            scratch.push_back(gq);
            scratch.push_back(gt);
        }
        uint count = (uint)(scratch.size() / 2);
        if (count == 0) return 0;

        size_t bytes = scratch.size() * sizeof(uint32_t);
        if (!refPairBuf || refPairCap < bytes) {
            if (refPairBuf) refPairBuf->release();
            refPairBuf = MetalGlobalContext::getDevice()->newBuffer(
                bytes, MTL::ResourceStorageModeShared);
            refPairCap = bytes;
        }
        std::memcpy(refPairBuf->contents(), scratch.data(), bytes);
        return count;
    }

    // Dispatch one of the two constraint kernels over `count` pairs,
    // binding the two global packed buffers it mutates at slots 0/1.
    void dispatchRefKernel(MTL::ComputePipelineState* pso,
                           VectorBase<METAL, PR>& b0,
                           VectorBase<METAL, PR>& b1,
                           uint count) {
        MetalGlobalContext::setBuffer(b0, 0);
        MetalGlobalContext::setBuffer(b1, 1);
        MetalGlobalContext::getComputeCommandEncoder()->setBuffer(
            refPairBuf, 0, 2);
        MetalGlobalContext::setBytes(count, 3);
        MetalGlobalContext::dispatchThreads(pso, count);
    }
    
    struct SimParams {
        float subh, G, kair, kd;
        uint vertexNum; 
        float acctime;
    };

    struct ClothGridParams {
        uint particleNum1D;
        float stretchRestX, stretchRestY;
        float shearRestA,   shearRestB;
        float bendRestX,    bendRestY;
        float kstretch, kshear, kbend;
        float thickness;
    };
    struct ClothParams {
        float kstretch, kshear, kbend;
        float thickness;
    };

    // 2. update() 함수 수정
    //
    // Restructured into two passes so a reference-point constraint can
    // sit between them per the design's 1-1..1-4 steps. The Metal
    // compute encoder dispatches serially, so each step below completes
    // before the next begins — that ordering IS the "fence":
    //   (1-1) ref_constraint_copy_pos   — snap follower x onto leader
    //   (... ) all cloth force kernels
    //   (1-3) ref_constraint_copy_force — follower f/v := leader's
    //   (1-4) all cloth integrate kernels
    // With no constraints the two passes are exactly the old per-mesh
    // force-then-integrate sequence (pairCount == 0 skips both copies).
    void update(Scene<METAL, PR>& sceneObjects) {
        if (!anomalyFlag.ptr) {
            anomalyFlag = VectorBase<METAL, uint32_t>(1);
            anomalyFlag[0] = 0u;
        }
        uint pairCount = buildRefPairs(sceneObjects);

        // Step 1-1: position snap before any force computation.
        if (pairCount > 0) {
            dispatchRefKernel(refCopyPosPSO,
                              Scene<METAL, PR>::packedMeshData.x,
                              Scene<METAL, PR>::packedMeshData.xPrev,
                              pairCount);
        }

        // Pass 1: forces only.
        for(auto& mesh : sceneObjects.meshes) {
            SimParams params = { subh, G, kair, kd, (uint)mesh.state.x.size/3, acctime };
            switch(mesh.behaviorType) {
                case BehaviorType::TriangularCloth:
                    TriangularClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    TriangularClothBehavior<METAL, PR>::update(mesh.state);
                    break;
                case BehaviorType::FastGridCloth:
                    FastGridClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    FastGridClothBehavior<METAL, PR>::update(mesh.state);
                    break;
                case BehaviorType::Float:
                case BehaviorType::Elastic:
                case BehaviorType::Rigid:
                case BehaviorType::Fluid:
                case BehaviorType::Generator:
                default: break;
            }
        }

        // Step 1-3: force/velocity copy after every force kernel.
        if (pairCount > 0) {
            dispatchRefKernel(refCopyForcePSO,
                              Scene<METAL, PR>::packedMeshData.f,
                              Scene<METAL, PR>::packedMeshData.v,
                              pairCount);
        }

        // Pass 2 (step 1-4): integrate. setBuffer must be re-bound per
        // mesh — pass 1's later meshes and the constraint dispatch
        // overwrote the encoder's buffer table.
        for(auto& mesh : sceneObjects.meshes) {
            SimParams params = { subh, G, kair, kd, (uint)mesh.state.x.size/3, acctime };
            switch(mesh.behaviorType) {
                case BehaviorType::TriangularCloth:
                    TriangularClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    MetalGlobalContext::setBuffer(anomalyFlag, 20);
                    MetalGlobalContext::dispatchThreads(integrateClothPSO, mesh.state.x.size/3);
                    break;
                case BehaviorType::FastGridCloth:
                    FastGridClothBehavior<METAL, PR>::setBuffer(mesh, params);
                    MetalGlobalContext::setBuffer(anomalyFlag, 12);
                    MetalGlobalContext::dispatchThreads(integrateClothGridPSO, mesh.state.x.size/3);
                    break;
                case BehaviorType::Float:
                case BehaviorType::Elastic:
                case BehaviorType::Rigid:
                case BehaviorType::Fluid:
                case BehaviorType::Generator:
                default: break;
            }
        }
    }
};

// TODO: BVH


// TODO: Spatial Hash


// simulation mamage



// Headless self-test (D-012). Exercises Simulator::initialize / saveScene /
// loadScene / update against a real Metal device WITHOUT a GLFW window —
// the prior render-state decoupling slice (D-011) made this reachable. Each
// assertion block guards a previously-escaped runtime bug or a parked
// BDD sim-step clause:
//   - CM-002 regression: re-running pack() must not double-free initializers.
//   - CM-003 regression: BVH must re-allocate when numMeshes grows.
//   - CM-004 / BDD-009 / BDD-011: gravity direction actually moves cloth in
//     the expected direction; Float is exempt.
//   - BDD-015: saveScene → loadScene round-trips numMeshes + env, then init
//     and a sim step are stable.
//
// Pass/fail via stderr lines + exit code (0 == all-pass). Runs from any cwd:
// default.metallib + assets resolve via ysim_paths (exe dir + project root).
// D-031: BVH refit benchmarking harness. Times `broad_refit` across four
// refit methods x four cloth resolutions x ten frames on a cloth-only
// FastGridCloth scene, writes per-frame CSV rows. Method labels map to
// values of D-030's runtime `bottomUpHybridDepth` knob; "FullGPU" routes
// through `bottomUpBoxesPartialGPU(30)` (kernel walks to root because the
// depth check never fires at the cutoff) rather than the strict D-029
// `bottomUpBoxesGPU` direct dispatch — the depth-counter overhead is one
// register-read + one compare per iteration, dwarfed by the atomic + 2
// seq_cst fences in the same iteration; treated as sub-noise. A future
// follow-up slice can add a strict-D-029 column if measurement-vs-noise
// becomes a question (recorded as a candidate in PROJECT_STATE).
//
// Production paths untouched: this is all NEW symbols per the
// make-means-add-new rule baked into `.claude/skills/slice/SKILL.md`.
//
// Lives next to `runSelfTest` for now per the user's "tests-in-main"
// convention; a future source-file split slice will move both out.
