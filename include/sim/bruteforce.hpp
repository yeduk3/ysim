#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

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
    MTL::ComputePipelineState* analyticPSO;   // slice (c-1)
    // P4 GPU per-vertex bucketing (replaces the CPU CSR-build loops).
    MTL::ComputePipelineState* bucketZeroPSO;
    MTL::ComputePipelineState* bucketCountPSO;
    MTL::ComputePipelineState* bucketScanPSO;
    MTL::ComputePipelineState* bucketScatterPSO;
    MTL::ComputePipelineState* resetCounterPSO;
    // update() sync refactor: InFrame commits after narrow (so the CPU can
    // read counts) ; None/PerFrame keep the bucketing on the GPU and never
    // sync here. Set by Simulator::update each frame.
    bool syncEachPhase = true;
    BruteForce() {
        bruteForcePSO = MetalKernelContext::getPSO("narrow_pt_tri");
        analyticPSO   = MetalKernelContext::getPSO("narrow_pt_analytic");
        bucketZeroPSO    = MetalKernelContext::getPSO("narrow_bucket_zero");
        bucketCountPSO   = MetalKernelContext::getPSO("narrow_bucket_count");
        bucketScanPSO    = MetalKernelContext::getPSO("narrow_bucket_scan");
        bucketScatterPSO = MetalKernelContext::getPSO("narrow_bucket_scatter");
        resetCounterPSO  = MetalKernelContext::getPSO("reset_counter");
    }

    // GPU replacement for the per-vertex CSR build in narrowAndSortByVertices.
    // Encodes zero -> count -> scan -> scatter into the current encoder; the
    // serial order supplies the barriers, so no commitAndWait is needed and
    // the integrator reads vertColFacets/Offsets straight off the GPU.
    void bucketByVerticesGPU() {
        auto& packedMesh = Scene<METAL, PR>::packedMeshData;
        auto& packedCol  = Scene<METAL, PR>::packedCollisionData;
        const uint nOffsets = (uint)packedCol.vertColFacetsOffsets.size;   // numPoints+1
        const uint maxN     = (uint)packedCol.maxNumCollisions;
        if (nOffsets == 0) return;

        // Step 0: zero offsets (n) + per-vertex counter (n-1).
        MetalGlobalContext::setBuffer(packedCol.vertColFacetsOffsets, 0);
        MetalGlobalContext::setBuffer(packedCol.vertColBucketCounter, 1);
        MetalGlobalContext::setBytes(nOffsets, 2);
        MetalGlobalContext::dispatchThreads(bucketZeroPSO, nOffsets);

        // Step 1: histogram into offsets[ppid+1].
        MetalGlobalContext::setBuffer(packedCol.narrowCollisions, 0);
        MetalGlobalContext::setBuffer(packedCol.numNarrowCollisions, 1);
        MetalGlobalContext::setBuffer(packedMesh.statesOffsets, 2);
        MetalGlobalContext::setBuffer(packedCol.vertColFacetsOffsets, 3);
        MetalGlobalContext::setBytes(maxN, 4);
        MetalGlobalContext::dispatchThreads(bucketCountPSO, maxN);

        // Step 2: in-place inclusive prefix sum (single thread).
        MetalGlobalContext::setBuffer(packedCol.vertColFacetsOffsets, 0);
        MetalGlobalContext::setBytes(nOffsets, 1);
        MetalGlobalContext::dispatchThreads(bucketScanPSO, 1);

        // Step 3: scatter contacts into per-vertex buckets.
        MetalGlobalContext::setBuffer(packedCol.narrowCollisions, 0);
        MetalGlobalContext::setBuffer(packedCol.numNarrowCollisions, 1);
        MetalGlobalContext::setBuffer(packedMesh.statesOffsets, 2);
        MetalGlobalContext::setBuffer(packedCol.vertColFacetsOffsets, 3);
        MetalGlobalContext::setBuffer(packedCol.vertColBucketCounter, 4);
        MetalGlobalContext::setBuffer(packedCol.vertColFacets, 5);
        MetalGlobalContext::setBytes(maxN, 6);
        MetalGlobalContext::dispatchThreads(bucketScatterPSO, maxN);
    }
    struct NarrowParams {
        uint32_t numBroadCollisions;
        uint32_t maxNumCollisions;
        float radius;
        float thickness;
        // Slice (c) A6: 1 ⇒ skip Sphere pairs (analytic path handles
        // them). 0 ⇒ original behavior (sphere via triangle soup).
        uint32_t skipSphere;
    };
    // Mirrors AnalyticNarrowParams in common_types.metalh (field order
    // and types MUST match — bound via setBytes).
    struct AnalyticNarrowParams {
        uint32_t oid;
        uint32_t numVerts;
        uint32_t shapeIndex;
        uint32_t maxNumCollisions;
        uint32_t clothBehavior;
        float    radius;
        float    thickness;
    };
    bool narrow(PR radius, PR thickness, bool analyticEnabled = false) {
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
        // Slow-touch band is radius + thickness; integrator gates vn-zero
        // and position-push on (distance < thickness) per D-016.
        nparams.thickness = static_cast<float>(thickness);
        // Only skip sphere pairs here when the analytic path is on.
        nparams.skipSphere = analyticEnabled ? 1u : 0u;

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
        // xPrev (start-of-substep position) at slot 10 — D-013 swept CCD.
        MetalGlobalContext::setBuffer(packedMesh.xPrev, 10);
        // numBroad count (slot 11) — bound for the P4 async path; redundant
        // here (InFrame dispatches over the tight count).
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 11);

        MetalGlobalContext::dispatchThreads(bruteForcePSO, nparams.numBroadCollisions);
        return true;
    }

    // Async narrow (None/PerFrame): no CPU read of any GPU counter. GPU-reset
    // numNarrow, then dispatch narrow_pt_tri over maxNumCollisions with the
    // real bound applied in-kernel from the numBroad buffer. Encodes into the
    // current encoder (no commitAndWait). Triangle path only — the analytic
    // path is CPU-marker driven, so callers fall back to the sync path when
    // analytic is enabled.
    void narrowGPU(PR radius, PR thickness) {
        typename Scene<METAL, PR>::PackedMeshData& packedMesh = Scene<METAL, PR>::packedMeshData;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;
        const uint maxN = (uint)packedCol.maxNumCollisions;

        // GPU-reset the narrow counter (atomic base) — no CPU touch.
        MetalGlobalContext::setBuffer(packedCol.numNarrowCollisions, 0);
        MetalGlobalContext::dispatchThreads(resetCounterPSO, 1);

        NarrowParams nparams{};
        nparams.numBroadCollisions = maxN;        // gate is the GPU numBroad buf
        nparams.maxNumCollisions   = (uint32_t)packedCol.maxNumCollisions;
        nparams.radius    = radius;
        nparams.thickness = static_cast<float>(thickness);
        nparams.skipSphere = 0u;

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
        MetalGlobalContext::setBuffer(packedMesh.xPrev, 10);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 11);
        MetalGlobalContext::dispatchThreads(bruteForcePSO, maxN);
    }

    // Slice (c-1) — analytic cloth-vs-primitive narrow phase. Appends
    // into the SAME shared narrowCollisions / numNarrowCollisions the
    // triangle path uses (does NOT resetNarrow — narrow() already did),
    // so the existing CPU sort + unchanged integrators consume it.
    // Per-cloth-mesh dispatch (mirrors the integrator dispatch loop):
    // CPU knows the mesh is cloth, so no GPU behavior buffer needed.
    // Returns true if at least one dispatch was issued.
    // Slice (c-2) — broad-marker-driven analytic narrow phase. Consumes
    // the (cloth, sphere) pairs the BVH broad phase left in
    // packedCol.analyticPairs (object AABBs overlapped). One dispatch per
    // pair, each testing a single AnalyticShape — so nothing fires while
    // the cloth is still falling (no overlap ⇒ no marker ⇒ no dispatch),
    // and a cloth that overlaps multiple spheres gets one dispatch each.
    // Appends into the SAME shared narrowCollisions / numNarrowCollisions
    // the triangle path uses (narrow() already resetNarrow'd), so the CPU
    // sort + unchanged integrators consume it. No commit here — the caller
    // (narrowAndSortByVertices) encodes this into the SAME command buffer
    // as narrow_pt_tri and commits ONCE (kills the c-1 per-substep extra
    // commitAndWait). Returns true if at least one dispatch was issued.
    bool narrowAnalytic(PR radius, PR thickness) {
        if (Scene<METAL, PR>::numAnalytic == 0
            || !Scene<METAL, PR>::meshAnalytic.ptr) return false;
        auto& packedMesh = Scene<METAL, PR>::packedMeshData;
        auto& packedCol  = Scene<METAL, PR>::packedCollisionData;
        if (packedCol.analyticPairs.empty()) return false;
        bool dispatched = false;
        for (auto& pr : packedCol.analyticPairs) {
            // Resolve the sphere's mesh id → compact AnalyticShape[] index
            // (numAnalytic is tiny; linear scan is cheaper than a map).
            Index sIdx = -1;
            for (Index k = 0; k < Scene<METAL, PR>::numAnalytic; ++k) {
                if (Scene<METAL, PR>::meshAnalytic[k].objId
                        == (uint32_t)pr.shapeObjId) { sIdx = k; break; }
            }
            if (sIdx < 0) continue;
            if ((Scene<METAL, PR>::meshAnalytic[sIdx].flags & 1u) == 0u)
                continue;   // not collidable — kernel would no-op anyway

            auto& clothMesh = Scene<METAL, PR>::meshes[pr.clothObj];
            uint32_t numVerts = (uint32_t)(clothMesh.state.x.size / 3);
            if (numVerts == 0) continue;

            AnalyticNarrowParams ap{};
            ap.oid              = (uint32_t)pr.clothObj;  // statesOffsets index
            ap.numVerts         = numVerts;
            ap.shapeIndex       = (uint32_t)sIdx;
            ap.maxNumCollisions = (uint32_t)packedCol.maxNumCollisions;
            ap.clothBehavior    = (uint32_t)clothMesh.behaviorType;
            ap.radius           = (float)radius;
            ap.thickness        = (float)thickness;

            MetalGlobalContext::setBuffer(packedCol.numNarrowCollisions, 0);
            MetalGlobalContext::setBuffer(packedCol.narrowCollisions, 1);
            MetalGlobalContext::setBuffer(packedMesh.x, 2);
            MetalGlobalContext::setBuffer(packedMesh.statesOffsets, 3);
            MetalGlobalContext::setBuffer(Scene<METAL, PR>::meshAnalytic, 4);
            MetalGlobalContext::setBytes(ap, 5);
            MetalGlobalContext::dispatchThreads(analyticPSO, numVerts);
            dispatched = true;
        }
        return dispatched;
    }

    void narrowAndSortByVertices(PR radius, PR thickness,
                                  bool analyticEnabled = false) {

        // P4 async path (None/PerFrame, non-analytic): narrow + per-vertex
        // CSR build entirely on the GPU, no commitAndWait and no CPU read of
        // any GPU counter. The integrator reads vertColFacets/Offsets straight
        // off the GPU next. InFrame (and any analytic frame) falls through to
        // the historical CPU-bucketing path below, bit-identical.
        if (!syncEachPhase && !analyticEnabled) {
            Scene<METAL, PR>::packedCollisionData.analyticPairs.clear();
            narrowGPU(radius, thickness);
            bucketByVerticesGPU();
            return;
        }

        // narrow() resets the shared narrow buffers (always), then
        // dispatches the triangle path (only when broad pairs exist).
        // When the analytic toggle is on, narrowAnalytic appends afterward
        // into the same buffers (no reset) from the broad markers. Toggle
        // OFF (default) = original pipeline: narrow_pt_tri handles spheres,
        // analytic not dispatched.
        //
        // Slice (c-2): both dispatches are encoded into the SAME command
        // buffer and committed ONCE here. A compute encoder runs its
        // dispatches serially with an implicit barrier between them, so the
        // shared atomic counter / array stay coherent (tri appends first,
        // analytic continues). This removes the per-substep second
        // commitAndWait the c-1 path paid (the dominant analytic-ON
        // overhead the bvh-vs-analytic experiment measured).
        bool tri = narrow(radius, thickness, analyticEnabled);
        bool ana = analyticEnabled && narrowAnalytic(radius, thickness);
        if (tri || ana) MetalGlobalContext::commitAndWait();
        // Markers consumed; clear so a later SH-broad frame (which does not
        // populate analyticPairs) can't re-dispatch this frame's pairs.
        Scene<METAL, PR>::packedCollisionData.analyticPairs.clear();
        if (!tri && !ana) return;
        // Cumulative narrow-contact counter for the harness — `numNarrowCollisions`
        // resets between substeps, so a per-frame harness sample misses contacts
        // that fired in earlier substeps. This static accumulates across the run
        // and is read by `runSelfTest` to assert "contacts ever fired" (BDD-007).
        Scene<METAL, PR>::packedCollisionData.cumulativeNarrowCollisions +=
            Scene<METAL, PR>::packedCollisionData.numNarrowCollisions[0];

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

template <typename BroadPhase, typename NarrowPhase>
struct CollisionPipeline {
    BroadPhase broadPhase;
    BroadPhase broadPhaseTest;
    NarrowPhase narrowPhase;



};

template <typename BE, typename PR, typename System>
