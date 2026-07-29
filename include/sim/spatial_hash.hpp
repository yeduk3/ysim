#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

struct RadixSorter {};

template <typename Element>
struct RadixSorter<METAL, Element> {
    uint32_t BITS_PER_PASS = 8;
    uint32_t NUM_BUCKETS = 1 << BITS_PER_PASS;
    uint32_t BLOCK_SIZE = 1024; // range limit: 256-1024
    VectorBase<METAL, Element> dst;
    VectorBase<METAL, Index> blockHist; // NumBlocks * 2^{BITS_PER_PASS}
    VectorBase<METAL, Index> hist; // 2^{BITS_PER_PASS}
    VectorBase<METAL, Index> blockOffset; // NumBlocks * 2^{BITS_PER_PASS}

    MTL::ComputePipelineState* radixCountBlockPSO;
    MTL::ComputePipelineState* radixScanOffsetPSO;
    MTL::ComputePipelineState* radixScatterPSO;


    struct RadixParams {
        uint numBlocks;
        uint numElements;
        uint blockSize;
        uint shift;
    };

    inline uint32_t getNumBlocks(const VectorBase<METAL, Element>& src) {
        return (src.size + BLOCK_SIZE - 1) / BLOCK_SIZE;
    }

    RadixSorter() {
        radixCountBlockPSO = MetalKernelContext::getPSO("radixCountBlock_8Bits");
        radixScanOffsetPSO = MetalKernelContext::getPSO("radixScanOffset_8Bits");
        radixScatterPSO = MetalKernelContext::getPSO("radixScatter_8Bits");
    }

    inline void memoryAllocation(const VectorBase<METAL, Element>& src) {
        uint32_t numBlocks = getNumBlocks(src);
        if(!dst.ptr || dst.size < src.size) dst = VectorBase<METAL, Element>(src.size);
        if(!hist.ptr || hist.size < NUM_BUCKETS) hist = VectorBase<METAL, Index>(NUM_BUCKETS);
        if(!blockHist.ptr || blockHist.size < NUM_BUCKETS*numBlocks) blockHist = VectorBase<METAL, Index>(NUM_BUCKETS*numBlocks);
        if(!blockOffset.ptr || blockOffset.size < NUM_BUCKETS*numBlocks) blockOffset = VectorBase<METAL, Index>(NUM_BUCKETS*numBlocks);
    }
    void sort(VectorBase<METAL, Element>& src) {
        memoryAllocation(src);

        RadixParams params;
        params.numBlocks = getNumBlocks(src);
        params.numElements = src.size;
        params.blockSize = BLOCK_SIZE;
        for(uint32_t r = 0; r < 32; r+=BITS_PER_PASS) {
            params.shift = r;
            MetalGlobalContext::setBuffer(src, 0);
            MetalGlobalContext::setBuffer(blockHist, 1);
            MetalGlobalContext::setBytes(params, 2);
            MetalGlobalContext::setBuffer(hist, 3);
            MetalGlobalContext::setBuffer(blockOffset, 4);
            MetalGlobalContext::setBuffer(dst, 5);

            MetalGlobalContext::dispatchThreads(radixCountBlockPSO, params.numBlocks*BLOCK_SIZE, BLOCK_SIZE);
            MetalGlobalContext::dispatchThreads(radixScanOffsetPSO, NUM_BUCKETS, NUM_BUCKETS);
            MetalGlobalContext::dispatchThreads(radixScatterPSO, params.numBlocks*BLOCK_SIZE, BLOCK_SIZE);

            std::swap(src, dst);
        }
    }
};


// TODO: BroadPhase, SpatialHashing
template <typename BE, typename PR>
struct SpatialHashing {};

// Spatial-hashing broadphase (Pabst-style uniform-grid) for the METAL backend.
//
// Implemented step-by-step alongside the existing BVH broadphase; both share
// the BroadCollision output convention so the rest of the pipeline
// (narrow_pt_tri, narrowAndSortByVertices) is unchanged.
//
// Surface mirrors BVH<METAL, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT> so the
// `Simulator::BroadPhase` typedef can be swapped at compile time. All methods
// in Step 0 are no-ops; later steps fill them in.
//
// Pipeline (target end state):
//   1. per-triangle bounding sphere (centroid, radius)         [Step 1]
//   2. max-radius reduction -> cellSize                        [Step 2]
//   3. scene AABB + grid resolution (host)                     [Step 3]
//   4. cell assignment (home + up to 7 phantoms per face)      [Step 4]
//   5. radix sort entries by cellID                            [Step 5]
//   6. cell ranges + nH/nP per cell                            [Step 6]
//   7. pair-count + prefix sum -> P (#candidate pairs)         [Step 7]
//   8. per-pair broad-phase -> BroadCollision[6 per tri pair]  [Step 8]
template<typename PR>
struct SpatialHashing<METAL, PR> {
    // Host-side mirror of metal's SHParams. Layout must match exactly because
    // setBytes copies raw bytes to constant memory.
    struct SHParamsHost {
        uint32_t numFaces;
        float    cellSize;
        float    epsilon;
        uint32_t gridRes[3];     // packed_uint3 = 3 contiguous uints
        float    originMin[3];   // packed_float3 = 3 contiguous floats
    };
    static_assert(sizeof(SHParamsHost) == 36,
                  "SHParamsHost must match metal SHParams layout (36 bytes)");

    // 8-byte hash-table entry, layout-compatible with SortPair so the radix
    // sorter (Step 5) can swap it directly. cellID == 0xFFFFFFFF marks an
    // unused slot and sorts to the tail.
    struct SHEntry {
        uint32_t cellID;
        uint32_t value;
    };
    static_assert(sizeof(SHEntry) == 8, "SHEntry must be 8 bytes");

    // ----- Buffers (allocated lazily; Step 0 leaves them empty) -----
    VectorBase<METAL, PR>          centroid;             // 3*m floats, packed_float3
    VectorBase<METAL, PR>          radius;               // m
    VectorBase<METAL, PR>          maxRadius;            // 1
    VectorBase<METAL, PR>          radiusReducePartial;  // ceil(m/256)
    VectorBase<METAL, PR>          radiusReducePartial2; // ceil(m/256/256), ping-pong
    VectorBase<METAL, Index>       faceObj;              // m, owning object id per global face
    VectorBase<METAL, uint32_t>    meshBehaviors;        // numMeshes, BehaviorType per mesh
    VectorBase<METAL, uint32_t>    meshShapes;           // numMeshes, ColliderKind per mesh
    VectorBase<METAL, uint8_t>     faceCB;               // m, 8-bit cell-type bitmask
    VectorBase<METAL, SHEntry>     entries;              // m*8, cellID=0xFFFFFFFF sentinel
    VectorBase<METAL, uint32_t>    cellStartFlag;        // m*8, 1 if entry starts a new cell
    VectorBase<METAL, uint32_t>    cellStartScan;        // m*8, exclusive scan of cellStartFlag
    // CellProp lives in metal-side struct; on host we mirror it as 5 uints.
    struct CellPropHost {
        uint32_t start;
        uint32_t nH;
        uint32_t nP;
        uint32_t pairCnt;
        uint32_t cellType;
    };
    VectorBase<METAL, CellPropHost> cellProp;            // C entries
    VectorBase<METAL, uint32_t>    pairPrefix;           // C+1, last = P
    VectorBase<METAL, uint32_t>    numCellsBuf;          // 1
    VectorBase<METAL, uint32_t>    numCandidatePairsBuf; // 1
    // sceneBox is small POD; passed via setBytes at Step 3 (no buffer needed).

    // ----- PSO handles (looked up per step as kernels are added) -----
    // Step 1+ will populate these; Step 0 keeps them null.
    MTL::ComputePipelineState* buildBVPSO          = nullptr;
    MTL::ComputePipelineState* reduceMaxRadiusPSO  = nullptr;
    MTL::ComputePipelineState* reduceFinalPSO      = nullptr;
    MTL::ComputePipelineState* assignCellsPSO      = nullptr;
    MTL::ComputePipelineState* markStartsPSO       = nullptr;
    MTL::ComputePipelineState* fillCellPropPSO     = nullptr;
    MTL::ComputePipelineState* computePairCountPSO = nullptr;
    MTL::ComputePipelineState* broadPhasePSO       = nullptr;

    // ----- Scene context cached at build() time -----
    Scene<METAL, PR>* scenePtr = nullptr;
    Index numFaces = 0;
    Index numValidEntries = 0;   // populated after Step 5 (radix sort + scan)
    Index numCells = 0;          // populated after Step 6 (mark + scan + fill)
    Index numCandidatePairs = 0; // populated after Step 7 (pair-count + scan)

    // Reused 4-pass 8-bit radix sorter from RadixSorter<METAL, MortonNode>;
    // SHEntry has the same {uint key=cellID, uint value} byte layout, so the
    // metal kernels work without modification.
    RadixSorter<METAL, SHEntry> sorter;

    // Filled in detectCollisions() once per frame; consumed by Steps 4+ via
    // setBytes. Matches metal SHParams layout exactly (36 bytes).
    SHParamsHost params{};

    // Optional. If non-null, detectCollisions() emits sh_* timing sections
    // that line up with the existing bvh_* labels for side-by-side compare.
    profiler::FrameProfiler* profiler = nullptr;

    // Per-stage timings + workload counters from the most recent
    // detectCollisions(). Filled every call (cheap), so the simulator can dump
    // a one-line log per substep without extra GPU work.  Note: when `verbose`
    // is false, `ms_broad` only times the dispatch (no commit) and `numBroadOut`
    // is whatever was visible at the last commitAndWait inside the pipeline.
    struct LastRunStats {
        Index  numFaces        = 0;
        Index  numValidEntries = 0;
        Index  numCells        = 0;
        Index  numCandidatePairs = 0;
        Index  numBroadOut     = 0;
        Index  maxNH           = 0;
        Index  maxNP           = 0;
        Index  maxPairCnt      = 0;
        Index  numPoints       = 0;
        float    cellSize      = 0.f;
        float    maxRadius     = 0.f;
        float    extent[3]     = {0.f, 0.f, 0.f};
        float    aabbMin[3]    = {0.f, 0.f, 0.f};
        uint32_t gridRes[3]    = {0, 0, 0};
        double ms_buildBV    = 0.0;
        double ms_reduce     = 0.0;
        double ms_grid       = 0.0;
        double ms_assign     = 0.0;
        double ms_sort       = 0.0;
        double ms_cellprop   = 0.0;
        double ms_pairprefix = 0.0;
        double ms_broad      = 0.0;
        double ms_total      = 0.0;
    };
    LastRunStats lastStats{};

    // When true, detectCollisions() commits after the broad-phase dispatch so
    // ms_broad and numBroadOut reflect actual GPU work (instead of just the
    // dispatch cost). Adds a sync point — keep off in steady-state.
    bool verbose = false;

    // Multiplier on the Pabst cell-size rule: cellSize = 2·maxRadius·factor + margin.
    //   factor = 1.0 (default): exact Pabst rule. Every face's BV fits in a
    //                            2x2x2 home+phantom cluster, so the 8 entry
    //                            slots/face in sh_assignCells are sufficient.
    //   factor < 1.0           : finer grid, but a face's BV may now span more
    //                            than 2 cells per axis. The current
    //                            sh_assignCells only emits 8 cells/face, so
    //                            some overlapped cells will be missed and the
    //                            large BVs lose collision pairs. Useful as a
    //                            quick experiment to see whether finer cells
    //                            relieve a hot cell; if it helps materially,
    //                            widen the slot budget and switch to the
    //                            general gMin..gMax cell-range assignment.
    //   factor > 1.0           : coarser grid, slot budget remains safe.
    float cellSizeFactor = 1.0f;

    SpatialHashing() = default;

    void memoryAllocation() {
        if (numFaces == 0) return;
        if (centroid.ptr && centroid.size == numFaces * 3) return;
        centroid = VectorBase<METAL, PR>(numFaces * 3);
        radius   = VectorBase<METAL, PR>(numFaces);

        constexpr Index TG = 256;
        Index ng1 = (numFaces + TG - 1) / TG;
        Index ng2 = std::max<Index>((ng1 + TG - 1) / TG, 1);
        radiusReducePartial  = VectorBase<METAL, PR>(std::max<Index>(ng1, 1));
        radiusReducePartial2 = VectorBase<METAL, PR>(ng2);
        if (!maxRadius.ptr) maxRadius = VectorBase<METAL, PR>(1);

        entries = VectorBase<METAL, SHEntry>(numFaces * 8);
        faceCB  = VectorBase<METAL, uint8_t>(numFaces);

        cellStartFlag = VectorBase<METAL, uint32_t>(numFaces * 8);
        cellStartScan = VectorBase<METAL, uint32_t>(numFaces * 8);
    }

    // faceObj[gFace] = owning object id. Walks facetsOffsets once; only
    // rebuilt when scene size changes.
    void rebuildFaceObj() {
        if (!scenePtr) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index m = packed.facets.size / 3;
        if (faceObj.ptr && faceObj.size == m) return;
        faceObj = VectorBase<METAL, Index>(m);
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        for (Index obj = 0; obj < numMeshes; ++obj) {
            Index begin = packed.facetsOffsets[obj];
            Index end   = packed.facetsOffsets[obj + 1];
            for (Index f = begin; f < end; ++f) faceObj[f] = obj;
        }
    }

    // meshBehaviors[obj] / meshShapes[obj] mirror the per-mesh enums into
    // GPU-readable arrays. Step 8 reads them per pair. P1: meshShapes
    // carries ColliderKind (the shapePair namespace), not ShapeType.
    void rebuildMeshKinds() {
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        if (meshBehaviors.ptr && meshBehaviors.size == numMeshes) return;
        meshBehaviors = VectorBase<METAL, uint32_t>(numMeshes);
        meshShapes    = VectorBase<METAL, uint32_t>(numMeshes);
        auto& meshes = Scene<METAL, PR>::meshes;
        for (Index i = 0; i < numMeshes; ++i) {
            meshBehaviors[i] = (uint32_t)meshes[i].behaviorType;
            meshShapes[i]    = (uint32_t)meshes[i].colliderKind;
        }
    }

    // Match BVH<SCENE,OBJECT> surface so CollisionPipeline / Simulator can
    // typedef to either implementation.
    void build(Scene<METAL, PR>& scene) {
        scenePtr = &scene;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        numFaces = packed.facets.size / 3;
        rebuildFaceObj();
        rebuildMeshKinds();
        memoryAllocation();
        if (!buildBVPSO)         buildBVPSO         = MetalKernelContext::getPSO("sh_buildBV");
        if (!reduceMaxRadiusPSO) reduceMaxRadiusPSO = MetalKernelContext::getPSO("sh_reduceMaxRadius");
        if (!assignCellsPSO)     assignCellsPSO     = MetalKernelContext::getPSO("sh_assignCells");
        if (!markStartsPSO)      markStartsPSO      = MetalKernelContext::getPSO("sh_markStarts");
        if (!fillCellPropPSO)    fillCellPropPSO    = MetalKernelContext::getPSO("sh_fillCellProp");
        if (!computePairCountPSO)computePairCountPSO= MetalKernelContext::getPSO("sh_computePairCount");
        if (!broadPhasePSO)      broadPhasePSO      = MetalKernelContext::getPSO("sh_broadPhase");
    }

    // Step 1 dispatch. Uses `params` (filled by computeGrid in Step 3); only
    // params.numFaces matters to sh_buildBV but we set the whole struct so
    // the same layout flows into later stages.
    void runBuildBV() {
        if (numFaces == 0 || !buildBVPSO) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        params.numFaces = (uint32_t)numFaces;

        MetalGlobalContext::setBuffer(packed.x, 0);
        MetalGlobalContext::setBuffer(packed.statesOffsets, 1);
        MetalGlobalContext::setBuffer(packed.facets, 2);
        MetalGlobalContext::setBuffer(faceObj, 3);
        MetalGlobalContext::setBytes(params, 4);
        MetalGlobalContext::setBuffer(centroid, 5);
        MetalGlobalContext::setBuffer(radius, 6);
        MetalGlobalContext::dispatchThreads(buildBVPSO, numFaces);
    }

    // Iteratively reduce `radius[numFaces]` to a single max in `maxRadius[0]`.
    // Each pass reduces by a factor of 256 (threadgroup size). Output buffer
    // alternates between `radiusReducePartial` / `radiusReducePartial2`; the
    // pass that brings count down to 1 writes directly to `maxRadius`.
    void runReduceMaxRadius() {
        if (numFaces == 0 || !reduceMaxRadiusPSO) return;
        constexpr Index TG = 256;

        auto dispatch = [&](VectorBase<METAL, PR>& in,
                            VectorBase<METAL, PR>& out,
                            uint32_t cnt) {
            Index ng = (cnt + TG - 1) / TG;
            MetalGlobalContext::setBuffer(in, 0);
            MetalGlobalContext::setBuffer(out, 1);
            MetalGlobalContext::setBytes(cnt, 2);
            MetalGlobalContext::dispatchThreads(reduceMaxRadiusPSO, ng * TG, TG);
            return ng;
        };

        // Pass 1: radius -> partial (or maxRadius if it already fits in one group).
        uint32_t cnt = (uint32_t)numFaces;
        Index ng = (cnt + TG - 1) / TG;
        VectorBase<METAL, PR>* dst = (ng == 1) ? &maxRadius : &radiusReducePartial;
        dispatch(radius, *dst, cnt);

        // Subsequent passes: ping-pong until we drive count to 1.
        VectorBase<METAL, PR>* src = dst;
        VectorBase<METAL, PR>* alt = (src == &radiusReducePartial)
                                     ? &radiusReducePartial2
                                     : &radiusReducePartial;
        while (ng > 1) {
            cnt = (uint32_t)ng;
            Index ng2 = (cnt + TG - 1) / TG;
            VectorBase<METAL, PR>* nextDst = (ng2 == 1) ? &maxRadius : alt;
            dispatch(*src, *nextDst, cnt);
            ng = ng2;
            src = nextDst;
            alt = (alt == &radiusReducePartial) ? &radiusReducePartial2
                                                : &radiusReducePartial;
        }
    }

    // ----- Step 3: scene AABB + grid resolution (host-side) -----
    //
    // CPU scan over packedMeshData.x to compute scene min/max, then derive
    // cellSize and per-axis gridRes. Result is stashed into `params` so the
    // same struct flows into all subsequent kernel dispatches via setBytes.
    //
    // Caller must ensure GPU writes to packedMeshData.x and maxRadius[0]
    // have completed (commitAndWait) before this runs.
    void computeGrid(PR margin) {
        if (numFaces == 0) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        // Walk via statesOffsets so the per-mesh range is explicit. Equivalent
        // to (0 .. packed.x.size/3) when the scene is well-packed, but makes
        // any offset/coverage bug observable.
        Index totalPoints = (numMeshes > 0) ? packed.statesOffsets[numMeshes] : 0;
        if (totalPoints == 0) return;

        tinym::vec3 mn(packed.x.ptr[0], packed.x.ptr[1], packed.x.ptr[2]);
        tinym::vec3 mx = mn;
        Index visited = 0;
        for (Index obj = 0; obj < numMeshes; ++obj) {
            Index begin = packed.statesOffsets[obj];
            Index end   = packed.statesOffsets[obj + 1];
            for (Index v = begin; v < end; ++v) {
                tinym::vec3 p(packed.x.ptr[v*3 + 0],
                              packed.x.ptr[v*3 + 1],
                              packed.x.ptr[v*3 + 2]);
                mn = tinym::min(mn, p);
                mx = tinym::max(mx, p);
            }
            visited += (end - begin);
        }
        // Sanity: every vertex was visited and the buffer holds nothing extra.
        // If this fires, statesOffsets is out of sync with packed.x.size.
        if (visited != totalPoints || totalPoints != packed.x.size / 3) {
            std::cout << "[SH computeGrid] coverage mismatch: visited="
                      << visited
                      << " statesOffsets[numMeshes]=" << totalPoints
                      << " packed.x.size/3=" << (packed.x.size / 3)
                      << "\n";
        }

        float cellSize = 2.0f * (float)maxRadius[0] * cellSizeFactor + (float)margin;
        // Degenerate scene (single point) or zero radius: fall back to margin.
        if (cellSize <= 0.f) cellSize = std::max((float)margin, 1e-6f);
        // One-shot warning when the user goes finer than Pabst — sh_assignCells
        // only emits 8 cells/face, so larger BVs will miss overlapped cells.
        static bool warnedFineGrid = false;
        if (cellSizeFactor < 1.0f && !warnedFineGrid) {
            std::cout << "[SH computeGrid] cellSizeFactor=" << cellSizeFactor
                      << " < 1.0 — BVs larger than cellSize/2 along an axis"
                         " will overflow the 8-slot assignment budget and"
                         " miss overlapped cells. Use as a diagnostic only.\n";
            warnedFineGrid = true;
        }

        params.numFaces    = (uint32_t)numFaces;
        params.cellSize    = cellSize;
        params.epsilon     = (float)margin;
        params.originMin[0] = mn.x;
        params.originMin[1] = mn.y;
        params.originMin[2] = mn.z;
        for (int k = 0; k < 3; ++k) {
            float extent = mx[k] - mn[k];
            uint32_t r = (uint32_t)std::ceil(extent / cellSize);
            if (r == 0) r = 1;
            params.gridRes[k] = r;
        }
        // Keep extent/numPoints/maxRadius for the per-substep log so the user
        // can correlate "grid is small" with "extent is small" or
        // "maxRadius (-> cellSize) is huge".
        lastStats.extent[0]   = mx.x - mn.x;
        lastStats.extent[1]   = mx.y - mn.y;
        lastStats.extent[2]   = mx.z - mn.z;
        lastStats.aabbMin[0]  = mn.x;
        lastStats.aabbMin[1]  = mn.y;
        lastStats.aabbMin[2]  = mn.z;
        lastStats.maxRadius   = (float)maxRadius[0];
        lastStats.numPoints   = totalPoints;
    }

    // Step 4 dispatch. Assumes computeGrid() filled `params` and the GPU has
    // already produced centroid[] and radius[].
    void runAssignCells() {
        if (numFaces == 0 || !assignCellsPSO) return;
        MetalGlobalContext::setBuffer(centroid, 0);
        MetalGlobalContext::setBuffer(radius,   1);
        MetalGlobalContext::setBytes(params,    2);
        MetalGlobalContext::setBuffer(entries,  3);
        MetalGlobalContext::setBuffer(faceCB,   4);
        MetalGlobalContext::dispatchThreads(assignCellsPSO, numFaces);
    }

    // ----- Step 5: radix sort entries by cellID -----
    //
    // Uses the 4-pass 8-bit radix sorter that already exists for the BVH's
    // morton codes. After 4 passes the sorted data is back in `entries`.
    // Sentinels (cellID=0xFFFFFFFF) sort to the tail.
    void runRadixSort() {
        if (numFaces == 0) return;
        sorter.sort(entries);
    }

    // Sentinel boundary via binary search. After Step 5 + commitAndWait the
    // unified-memory `entries.ptr` is coherent on the host.
    Index findNumValid() {
        Index n = entries.size;
        if (n == 0) return 0;
        if (entries[n - 1].cellID != 0xFFFFFFFFu) return n;
        if (entries[0].cellID == 0xFFFFFFFFu)     return 0;
        Index lo = 0, hi = n;
        while (lo < hi) {
            Index mid = lo + (hi - lo) / 2;
            if (entries[mid].cellID == 0xFFFFFFFFu) hi = mid;
            else                                    lo = mid + 1;
        }
        return lo;
    }

    // ----- Step 6: cell ranges + nH/nP per cell -----
    //
    // Three-stage: GPU markStarts -> host inclusive prefix-sum -> GPU
    // fillCellProp. The host scan is the bottleneck (~m*8 sequential adds);
    // it can be moved to a Blelloch scan kernel later but is fine on M3 for
    // v1 because we already commitAndWait between Step 5 and Step 7.

    void runMarkStarts() {
        if (numValidEntries == 0 || !markStartsPSO) return;
        uint32_t nv = (uint32_t)numValidEntries;
        MetalGlobalContext::setBuffer(entries,       0);
        MetalGlobalContext::setBytes(nv,             1);
        MetalGlobalContext::setBuffer(cellStartFlag, 2);
        MetalGlobalContext::dispatchThreads(markStartsPSO, numValidEntries);
    }

    // Inclusive prefix sum minus 1: `cellStartScan[i] = (sum flag[0..=i]) - 1`,
    // i.e. the cellIdx that entry `i` belongs to. Final running total is
    // numCells. Host-side; caller must commitAndWait first.
    void hostScanCellStarts() {
        numCells = 0;
        if (numValidEntries == 0) return;
        uint32_t running = 0;
        for (Index i = 0; i < numValidEntries; ++i) {
            running += cellStartFlag[i];
            cellStartScan[i] = running - 1;
        }
        numCells = running;
    }

    void runFillCellProp() {
        if (numCells == 0 || !fillCellPropPSO) return;
        // (Re-)allocate cellProp sized to numCells, zero-initialised.
        if (cellProp.size < numCells) {
            cellProp = VectorBase<METAL, CellPropHost>(numCells);
        }
        std::memset(cellProp.ptr, 0, sizeof(CellPropHost) * numCells);

        uint32_t nv = (uint32_t)numValidEntries;
        MetalGlobalContext::setBuffer(entries,       0);
        MetalGlobalContext::setBuffer(cellStartFlag, 1);
        MetalGlobalContext::setBuffer(cellStartScan, 2);
        MetalGlobalContext::setBytes(nv,             3);
        MetalGlobalContext::setBytes(params,         4);
        MetalGlobalContext::setBuffer(cellProp,      5);
        MetalGlobalContext::dispatchThreads(fillCellPropPSO, numValidEntries);
    }

    void validateCellProp(Index N = 8) {
        if (numCells == 0) {
            std::cout << "[SH Step6] no active cells\n";
            return;
        }
        uint64_t sumH = 0, sumP = 0;
        Index    badStart = numCells; // sentinel meaning "none"
        for (Index i = 0; i < numCells; ++i) {
            sumH += cellProp[i].nH;
            sumP += cellProp[i].nP;
            if (i > 0 && cellProp[i].start <= cellProp[i - 1].start
                && badStart == numCells) {
                badStart = i;
            }
        }
        bool sumOk     = (sumH + sumP == (uint64_t)numValidEntries);
        bool startOk   = (badStart == numCells);
        std::cout << "[SH Step6] cells=" << numCells
                  << " sum(nH)=" << sumH
                  << " sum(nP)=" << sumP
                  << " total=" << (sumH + sumP)
                  << "/" << numValidEntries
                  << (sumOk ? " match" : " MISMATCH")
                  << " starts=" << (startOk ? "increasing" : "BROKEN")
                  << "\n";
        if (!startOk) {
            std::cout << "  first non-increasing start at i=" << badStart
                      << " start[" << (badStart - 1) << "]="
                      << cellProp[badStart - 1].start
                      << " start[" << badStart << "]="
                      << cellProp[badStart].start << "\n";
        }
        Index n = std::min<Index>(N, numCells);
        for (Index i = 0; i < n; ++i) {
            const auto& cp = cellProp[i];
            std::cout << "  cell " << i
                      << " start=" << cp.start
                      << " nH=" << cp.nH
                      << " nP=" << cp.nP
                      << " type=" << cp.cellType
                      << " (cellID=" << entries[cp.start].cellID << ")"
                      << "\n";
        }
    }

    // ----- Step 7: per-cell pairCnt + exclusive prefix -> P -----
    //
    // pairCnt = nH*(nH-1)/2 + nH*nP. Phantom-only cells (nH==0) produce 0
    // pairs and are silently dropped (no fictitious-phantom promotion in
    // v1).  pairPrefix[C+1] is exclusive-scan of pairCnt; the last element
    // is the total candidate-pair count P consumed by Step 8.

    void runComputePairCount() {
        if (numCells == 0 || !computePairCountPSO) return;
        uint32_t nc = (uint32_t)numCells;
        MetalGlobalContext::setBuffer(cellProp, 0);
        MetalGlobalContext::setBytes(nc,        1);
        MetalGlobalContext::dispatchThreads(computePairCountPSO, numCells);
    }

    void hostScanPairPrefix() {
        numCandidatePairs = 0;
        if (numCells == 0) return;
        if (pairPrefix.size < numCells + 1) {
            pairPrefix = VectorBase<METAL, uint32_t>(numCells + 1);
        }
        uint64_t running = 0;
        for (Index i = 0; i < numCells; ++i) {
            pairPrefix[i] = (uint32_t)running;
            running += cellProp[i].pairCnt;
        }
        pairPrefix[numCells] = (uint32_t)running;
        numCandidatePairs = (Index)running;
    }

    void validatePairPrefix(Index N = 8) {
        // Independent recompute of pairCnt from nH/nP, summed.
        uint64_t cpuTotal = 0;
        Index    badCnt   = numCells;
        Index    nonzeroCells = 0;
        for (Index i = 0; i < numCells; ++i) {
            uint32_t h = cellProp[i].nH;
            uint32_t p = cellProp[i].nP;
            uint32_t expected = (h * (h - 1u)) / 2u + h * p;
            if (cellProp[i].pairCnt != expected && badCnt == numCells) {
                badCnt = i;
            }
            cpuTotal += expected;
            if (expected > 0) ++nonzeroCells;
        }
        bool totalOk = (cpuTotal == (uint64_t)numCandidatePairs);
        std::cout << "[SH Step7] P=" << numCandidatePairs
                  << " (cpu sum=" << cpuTotal
                  << (totalOk ? " match" : " MISMATCH") << ")"
                  << " active=" << nonzeroCells << "/" << numCells
                  << " pairCnt=" << ((badCnt == numCells) ? "ok" : "BAD")
                  << "\n";
        if (badCnt != numCells) {
            uint32_t h = cellProp[badCnt].nH;
            uint32_t p = cellProp[badCnt].nP;
            std::cout << "  first bad pairCnt at cell " << badCnt
                      << " nH=" << h << " nP=" << p
                      << " expected=" << ((h*(h-1u))/2u + h*p)
                      << " got=" << cellProp[badCnt].pairCnt << "\n";
        }
        Index n = std::min<Index>(N, numCells);
        for (Index i = 0; i < n; ++i) {
            std::cout << "  cell " << i
                      << " nH=" << cellProp[i].nH
                      << " nP=" << cellProp[i].nP
                      << " pairCnt=" << cellProp[i].pairCnt
                      << " pairPrefix=" << pairPrefix[i] << "\n";
        }
        std::cout << "  pairPrefix[" << numCells << "]=" << pairPrefix[numCells]
                  << "\n";
    }

    // ----- Step 8: per-pair broad-phase emit -----
    //
    // Dispatches P threads, one per candidate pair. Output is the same
    // BroadCollision buffer the BVH path writes to (Scene::packedCollisionData),
    // so the existing narrow_pt_tri kernel consumes SH output unchanged.

    struct SHBroadParamsHost {
        uint32_t numCandidatePairs;
        uint32_t numCells;
        uint32_t maxNumCollisions;
        uint32_t enableSelfCollisions;
        float    epsilon;
    };
    static_assert(sizeof(SHBroadParamsHost) == 20,
                  "SHBroadParamsHost must match metal SHBroadParams (20 bytes)");

    void runBroadPhase(PR margin, bool enableSelfCollisions) {
        if (numCandidatePairs == 0 || !broadPhasePSO) return;
        auto& packed    = Scene<METAL, PR>::packedMeshData;
        auto& packedCol = Scene<METAL, PR>::packedCollisionData;

        SHBroadParamsHost bp{};
        bp.numCandidatePairs    = (uint32_t)numCandidatePairs;
        bp.numCells             = (uint32_t)numCells;
        bp.maxNumCollisions     = (uint32_t)packedCol.maxNumCollisions;
        bp.enableSelfCollisions = enableSelfCollisions ? 1u : 0u;
        bp.epsilon              = (float)margin;

        MetalGlobalContext::setBuffer(entries,                   0);
        MetalGlobalContext::setBuffer(cellProp,                  1);
        MetalGlobalContext::setBuffer(pairPrefix,                2);
        MetalGlobalContext::setBuffer(faceCB,                    3);
        MetalGlobalContext::setBuffer(faceObj,                   4);
        MetalGlobalContext::setBuffer(packed.facets,             5);
        MetalGlobalContext::setBuffer(packed.facetsOffsets,      6);
        MetalGlobalContext::setBuffer(centroid,                  7);
        MetalGlobalContext::setBuffer(radius,                    8);
        MetalGlobalContext::setBuffer(meshBehaviors,             9);
        MetalGlobalContext::setBuffer(meshShapes,               10);
        MetalGlobalContext::setBytes(bp,                        11);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 12);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions,    13);
        MetalGlobalContext::dispatchThreads(broadPhasePSO, numCandidatePairs);
    }

    void validateBroadPhase() {
        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        uint32_t numOut = (uint32_t)packedCol.numBroadCollisions[0];
        bool overflow   = numOut > (uint32_t)packedCol.maxNumCollisions;
        std::cout << "[SH Step8] candidatePairs=" << numCandidatePairs
                  << " broadCollisions=" << numOut
                  << "/" << packedCol.maxNumCollisions
                  << (overflow ? " OVERFLOW" : "")
                  << "\n";
    }

    void validateRadixSort() {
        if (numFaces == 0) return;
        numValidEntries = findNumValid();

        // Cross-check vs Step 4's per-element count.
        Index step4Valid = 0;
        for (Index i = 0; i < entries.size; ++i)
            if (entries[i].cellID != 0xFFFFFFFFu) ++step4Valid;

        // Sortedness on the valid prefix.
        Index firstViolation = numValidEntries; // sentinel value
        for (Index i = 1; i < numValidEntries; ++i) {
            if (entries[i - 1].cellID > entries[i].cellID) {
                firstViolation = i;
                break;
            }
        }
        std::cout << "[SH Step5] numValid=" << numValidEntries
                  << "/" << entries.size
                  << " (step4 count=" << step4Valid
                  << ((step4Valid == numValidEntries) ? " match" : " MISMATCH")
                  << ") sorted="
                  << ((firstViolation == numValidEntries) ? "yes" : "NO")
                  << "\n";
        if (firstViolation != numValidEntries) {
            std::cout << "  first violation at i=" << firstViolation
                      << " cellID[" << (firstViolation - 1) << "]="
                      << entries[firstViolation - 1].cellID
                      << " > cellID[" << firstViolation << "]="
                      << entries[firstViolation].cellID << "\n";
        }
    }

    // Spot-check: per-face entry counts, CB popcount = entry count, and
    // the home cell hash recomputed on the CPU.
    void validateAssignCells(Index N = 5) {
        if (numFaces == 0) return;
        Index totalValid = 0;
        Index minSlots = 8, maxSlots = 0;
        for (Index i = 0; i < numFaces; ++i) {
            Index v = 0;
            for (uint k = 0; k < 8; ++k)
                if (entries[i * 8 + k].cellID != 0xFFFFFFFFu) ++v;
            totalValid += v;
            minSlots = std::min<Index>(minSlots, v);
            maxSlots = std::max<Index>(maxSlots, v);
        }
        std::cout << "[SH Step4] entries: " << totalValid
                  << " valid / " << (numFaces * 8) << " total ("
                  << (100.0 * totalValid / (numFaces * 8)) << "%)"
                  << " per-face min=" << minSlots
                  << " max=" << maxSlots << "\n";

        Index n = std::min<Index>(N, numFaces);
        for (Index i = 0; i < n; ++i) {
            tinym::vec3_view cView(centroid.ptr + i * 3);
            tinym::vec3 c(cView[0], cView[1], cView[2]);
            uint32_t gx = (uint32_t)std::floor((c.x - params.originMin[0]) / params.cellSize);
            uint32_t gy = (uint32_t)std::floor((c.y - params.originMin[1]) / params.cellSize);
            uint32_t gz = (uint32_t)std::floor((c.z - params.originMin[2]) / params.cellSize);
            gx = std::min(gx, params.gridRes[0] - 1);
            gy = std::min(gy, params.gridRes[1] - 1);
            gz = std::min(gz, params.gridRes[2] - 1);
            uint32_t cpuHomeHash = gz * params.gridRes[1] * params.gridRes[0]
                                 + gy * params.gridRes[0]
                                 + gx;
            uint32_t cpuHomeT = (gx & 1u) | ((gy & 1u) << 1) | ((gz & 1u) << 2);

            uint32_t gpuHomeHash = entries[i * 8].cellID;
            uint32_t gpuValue    = entries[i * 8].value;
            uint32_t gpuHomeT    = gpuValue & 0x7u;
            uint32_t gpuPhantom  = (gpuValue >> 3) & 0x1u;
            uint32_t gpuFaceId   = gpuValue >> 4;

            int valid = 0;
            for (uint k = 0; k < 8; ++k)
                if (entries[i * 8 + k].cellID != 0xFFFFFFFFu) ++valid;

            uint8_t cb = faceCB[i];
            int popcount = __builtin_popcount((unsigned)cb);

            std::cout << "  face " << i
                      << " home cpu/gpu hash=" << cpuHomeHash << "/" << gpuHomeHash
                      << " homeT cpu/gpu=" << cpuHomeT << "/" << gpuHomeT
                      << " phantom=" << gpuPhantom
                      << " faceId=" << gpuFaceId
                      << " entries=" << valid
                      << " CB=0x" << std::hex << (int)cb << std::dec
                      << " popcount=" << popcount
                      << ((popcount == valid) ? " [ok]" : " [MISMATCH]")
                      << "\n";
        }
    }

    void validateGrid() {
        std::cout << "[SH Step3] origin=("
                  << params.originMin[0] << ","
                  << params.originMin[1] << ","
                  << params.originMin[2] << ")"
                  << " cellSize=" << params.cellSize
                  << " gridRes=("
                  << params.gridRes[0] << ","
                  << params.gridRes[1] << ","
                  << params.gridRes[2] << ")"
                  << " totalCells=" << ((uint64_t)params.gridRes[0]
                                       * params.gridRes[1]
                                       * params.gridRes[2])
                  << "\n";
    }

    // Compare GPU `maxRadius[0]` against CPU std::max over the same array.
    void validateMaxRadius() {
        if (numFaces == 0) return;
        float gpuMax = (float)maxRadius[0];
        float cpuMax = 0.0f;
        for (Index i = 0; i < numFaces; ++i)
            cpuMax = std::max(cpuMax, (float)radius[i]);
        std::cout << "[SH Step2] maxRadius cpu=" << cpuMax
                  << " gpu=" << gpuMax
                  << " |diff|=" << std::abs(cpuMax - gpuMax) << "\n";
    }

    // One-shot sanity check: GPU result vs CPU recompute on the first N
    // faces. Called manually from Simulator::initialize() while Step 1 is
    // being verified; remove the call once trust is established.
    void validateBuildBV(Index N = 5) {
        if (numFaces == 0) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index n = std::min<Index>(N, numFaces);
        std::cout << "[SH Step1] verifying BV for first " << n
                  << " of " << numFaces << " faces:\n";
        float maxRelErr = 0.0f;
        for (Index i = 0; i < n; ++i) {
            uint32_t obj   = (uint32_t)faceObj[i];
            uint32_t vbase = (uint32_t)packed.statesOffsets[obj];
            uint32_t f0 = (uint32_t)packed.facets[i * 3 + 0];
            uint32_t f1 = (uint32_t)packed.facets[i * 3 + 1];
            uint32_t f2 = (uint32_t)packed.facets[i * 3 + 2];
            tinym::vec3_view v0(packed.x.ptr + (f0 + vbase) * 3);
            tinym::vec3_view v1(packed.x.ptr + (f1 + vbase) * 3);
            tinym::vec3_view v2(packed.x.ptr + (f2 + vbase) * 3);
            tinym::vec3 c = (v0 + v1 + v2) * (1.0f / 3.0f);
            float d0 = (c - v0).norm();
            float d1 = (c - v1).norm();
            float d2 = (c - v2).norm();
            float rCpu = std::max({d0, d1, d2});

            tinym::vec3_view gC(centroid.ptr + i * 3);
            float rGpu = radius[i];

            float dC = (c - gC).norm();
            float dR = std::abs(rCpu - rGpu);
            float rel = (rCpu > 1e-6f) ? (dR / rCpu) : dR;
            maxRelErr = std::max(maxRelErr, rel);
            std::cout << "  face " << i << " obj=" << obj
                      << " cpuC=(" << c.x << "," << c.y << "," << c.z << ")"
                      << " gpuC=(" << gC[0] << "," << gC[1] << "," << gC[2] << ")"
                      << " cpuR=" << rCpu << " gpuR=" << rGpu
                      << " |dC|=" << dC << " |dR|=" << dR << "\n";
        }
        std::cout << "[SH Step1] max relative radius error: "
                  << maxRelErr << "\n";
    }

    void refit() {
        // The grid is rebuilt every frame inside detectCollisions(); refit()
        // is intentionally a no-op so the call site mirrors the BVH path.
    }

    void enlargeTrajectory(PR /*dt*/) {
        // CCD swept BVs are out of scope for v1.
    }

    void queryBegin() {
        Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] = 0;
    }

    void detectCollisions(PR margin, bool enableSelfCollisions = true) {
        // Each stage is wall-time-measured once, and that single number feeds
        // both `lastStats` (for stdout per-substep logging) and the
        // FrameProfiler (for the GUI timing window). Avoids two redundant
        // clock reads per stage.
        using Clock = std::chrono::steady_clock;
        auto run = [&](const char* name, auto&& fn, double& dst) {
            auto a = Clock::now();
            fn();
            auto b = Clock::now();
            dst = std::chrono::duration<double, std::milli>(b - a).count();
            if (profiler) profiler->addSample(std::string(name), dst);
        };

        auto t0 = Clock::now();
        queryBegin();

        run("sh_buildBV",    [&]{ runBuildBV(); },                           lastStats.ms_buildBV);
        run("sh_reduce",     [&]{
            runReduceMaxRadius();
            // Step 3 reads maxRadius[0] and packedMeshData.x on the host, so
            // we must wait for the kernels above to finish writing.
            MetalGlobalContext::commitAndWait();
        }, lastStats.ms_reduce);
        run("sh_grid",       [&]{ computeGrid(margin); },                    lastStats.ms_grid);
        run("sh_assign",     [&]{ runAssignCells(); },                       lastStats.ms_assign);
        run("sh_sort",       [&]{
            runRadixSort();
            // Step 6 needs numValidEntries on host, which requires commit + scan.
            MetalGlobalContext::commitAndWait();
            numValidEntries = findNumValid();
        }, lastStats.ms_sort);
        run("sh_cellprop",   [&]{
            runMarkStarts();
            MetalGlobalContext::commitAndWait();
            hostScanCellStarts();
            runFillCellProp();
        }, lastStats.ms_cellprop);
        run("sh_pairprefix", [&]{
            runComputePairCount();
            // Step 7's host scan needs cellProp[].pairCnt visible.
            MetalGlobalContext::commitAndWait();
            hostScanPairPrefix();
        }, lastStats.ms_pairprefix);
        run("sh_broad",      [&]{
            runBroadPhase(margin, enableSelfCollisions);
            // Force a commit when verbose so the host-side ms_broad reflects
            // actual GPU work and numBroadOut is observable. Steady-state mode
            // skips this so the broad-phase output stays pipelined into the
            // narrow phase via the same command buffer.
            if (verbose) MetalGlobalContext::commitAndWait();
        }, lastStats.ms_broad);

        auto t1 = Clock::now();
        lastStats.ms_total = std::chrono::duration<double, std::milli>(t1 - t0).count();
        lastStats.numFaces          = numFaces;
        lastStats.numValidEntries   = numValidEntries;
        lastStats.numCells          = numCells;
        lastStats.numCandidatePairs = numCandidatePairs;
        lastStats.cellSize          = params.cellSize;
        lastStats.gridRes[0]        = params.gridRes[0];
        lastStats.gridRes[1]        = params.gridRes[1];
        lastStats.gridRes[2]        = params.gridRes[2];

        if (verbose) {
            // Scan cellProp host-side for the heavy-cell signal — only when
            // verbose so we don't pay the O(numCells) sweep every substep.
            Index maxH = 0, maxP = 0, maxC = 0;
            for (Index i = 0; i < numCells; ++i) {
                const auto& cp = cellProp[i];
                if ((Index)cp.nH      > maxH) maxH = cp.nH;
                if ((Index)cp.nP      > maxP) maxP = cp.nP;
                if ((Index)cp.pairCnt > maxC) maxC = cp.pairCnt;
            }
            lastStats.maxNH      = maxH;
            lastStats.maxNP      = maxP;
            lastStats.maxPairCnt = maxC;
            lastStats.numBroadOut =
                Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0];
        }
    }

    // One compact line per call. Caller picks frame/substep labels.
    void printLastStats(std::ostream& os, Index frame, Index substep) const {
        os << "[F=" << frame << " S=" << substep << "] sh:"
           << " faces=" << lastStats.numFaces
           << " valid=" << lastStats.numValidEntries
           << " cells=" << lastStats.numCells
           << " pairs=" << lastStats.numCandidatePairs
           << " broadOut=" << lastStats.numBroadOut
           << " maxNH=" << lastStats.maxNH
           << " maxNP=" << lastStats.maxNP
           << " maxPairCnt=" << lastStats.maxPairCnt
           << " pts=" << lastStats.numPoints
           << " ext=" << lastStats.extent[0]
           << "x" << lastStats.extent[1]
           << "x" << lastStats.extent[2]
           << " maxR=" << lastStats.maxRadius
           << " csF=" << cellSizeFactor
           << " cellSize=" << lastStats.cellSize
           << " grid=" << lastStats.gridRes[0]
           << "x" << lastStats.gridRes[1]
           << "x" << lastStats.gridRes[2]
           << " | bv=" << lastStats.ms_buildBV
           << " red=" << lastStats.ms_reduce
           << " grid=" << lastStats.ms_grid
           << " asgn=" << lastStats.ms_assign
           << " srt=" << lastStats.ms_sort
           << " cell=" << lastStats.ms_cellprop
           << " pp=" << lastStats.ms_pairprefix
           << " brd=" << lastStats.ms_broad
           << " tot=" << lastStats.ms_total << "ms\n";
    }

    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        if (packedCol.numBroadCollisions[0] > packedCol.maxNumCollisions) {
            std::cout << "[SH] broad-phase overflow: "
                      << packedCol.numBroadCollisions[0]
                      << "/" << packedCol.maxNumCollisions << "\n";
        }
    }

    void showBox()      { /* debug-only; defer */ }
    void showSceneBox() { /* debug-only; defer */ }

    void queryClickRay(const Ray& /*ray*/) {
        // Click ray-pick continues to use BVH; SpatialHashing is broadphase-only.
    }
};


// ============================================================================
// Multi-level (hierarchical hash-grid / hgrid) spatial-hashing broad phase.
//
// ADDITIVE alongside SpatialHashing<METAL,PR> (single-level) — both share the
// BroadCollision output convention and the BVH<SCENE,OBJECT> surface, so the
// Simulator can flip the active broad phase at runtime. Algorithm + kernels:
// src/metal/mlspatialhashing.metal (ml_buildBV / ml_assignCells /
// ml_broadPhase) reusing the single-level radix sort and sh_reduceMaxRadius.
//
//   * L geometric grid levels: cellSize[k] = c0*2^k, cellSize[L-1] = 2*maxR.
//   * each face -> ONE home cell at its fitting level Lf (finest cell that fits
//     its inflated BV). No phantom replication. Key = (level<<28)|linearCellId,
//     value = global face id; entries (one per face) radix-sorted by key.
//   * the per-face broad phase walks levels Lf..L-1, binary-searching the sorted
//     entries for each overlapped cell and colliding against the faces HOME
//     there. Dedup: own level emits faceB>faceA; coarser levels are owned by the
//     (strictly finer) querying face. Cells stay sparse -> bounded pair work,
//     instead of the O(N^3) packed-coarse-cell blowup phantom replication caused.
//   * oversized primitives (the floor) are EXCLUDED via a per-mesh
//     AABB-diagonal threshold: excluded faces get radius 0 and emit no entries,
//     so they leave neither the max-radius reduction nor the scene AABB,
//     keeping cell size and per-cell occupancy bounded (the goal's
//     "바닥 오브젝트는 너무 큰 primitive로 테스트에서 제외").
template <typename BE, typename PR>
struct MultiLevelSpatialHashing {};

template<typename PR>
struct MultiLevelSpatialHashing<METAL, PR> {
    static constexpr uint32_t ML_MAX_LEVELS = 4;

    // Host mirror of metal MLParams. Plain 4-byte scalars/arrays only — no
    // packed_* — so the layout is padding-free and byte-matches the kernel.
    struct MLParamsHost {
        uint32_t numFaces;
        uint32_t numLevels;
        float    epsilon;
        float    cellSize[ML_MAX_LEVELS];     // 4 floats
        uint32_t gridRes[ML_MAX_LEVELS * 3];  // 12 uints, [level*3 + axis]
        float    originMin[3];
    };
    static_assert(sizeof(MLParamsHost) == 88,
                  "MLParamsHost must match metal MLParams layout (88 bytes)");

    struct MLBroadParamsHost {
        uint32_t numFaces;
        uint32_t numValid;
        uint32_t maxNumCollisions;
        uint32_t enableSelfCollisions;
        uint32_t numLevels;
        float    epsilon;
    };
    static_assert(sizeof(MLBroadParamsHost) == 24,
                  "MLBroadParamsHost must match metal MLBroadParams (24 bytes)");

    struct SHEntry { uint32_t cellID; uint32_t value; };
    static_assert(sizeof(SHEntry) == 8, "SHEntry must be 8 bytes");

    // ----- Buffers -----
    // Home-only insertion: one entry per face (its home cell), so `entries` is
    // numFaces — no phantom replication, no per-cell property / pair-prefix
    // arrays. The per-face broad phase walks levels and binary-searches this
    // sorted buffer directly.
    VectorBase<METAL, PR>          centroid;             // 3*m
    VectorBase<METAL, PR>          radius;               // m
    VectorBase<METAL, PR>          maxRadius;            // 1
    VectorBase<METAL, PR>          radiusReducePartial;  // ceil(m/256)
    VectorBase<METAL, PR>          radiusReducePartial2;
    VectorBase<METAL, Index>       faceObj;              // m
    VectorBase<METAL, uint8_t>     faceExclude;          // m, 1 == drop from grid
    VectorBase<METAL, uint32_t>    meshBehaviors;        // numMeshes
    VectorBase<METAL, uint32_t>    meshShapes;           // numMeshes
    VectorBase<METAL, SHEntry>     entries;              // m, cellID-sorted, sentinel tail

    MTL::ComputePipelineState* buildBVPSO          = nullptr;
    MTL::ComputePipelineState* reduceMaxRadiusPSO  = nullptr;
    MTL::ComputePipelineState* assignCellsPSO      = nullptr;
    MTL::ComputePipelineState* broadPhasePSO       = nullptr;

    Scene<METAL, PR>* scenePtr = nullptr;
    Index numFaces = 0;
    Index numValidEntries = 0;
    Index numCells = 0;
    Index numCandidatePairs = 0;
    Index numExcludedFaces = 0;

    RadixSorter<METAL, SHEntry> sorter;
    MLParamsHost params{};
    profiler::FrameProfiler* profiler = nullptr;

    // Tunables.
    int   numLevels       = 4;      // MAX levels (cap); active count auto-derived
                                    // per frame from the min/max face-size ratio.
    float cellSizeFactor  = 1.0f;   // multiplier on the finest (min-radius) cell.
    float floorExcludeDiag = 8.0f;  // mesh AABB-diagonal above this -> excluded.
    bool  verbose = false;

    struct LastRunStats {
        Index  numFaces = 0, numValidEntries = 0, numCells = 0,
               numCandidatePairs = 0, numBroadOut = 0, numExcluded = 0;
        Index  numPoints = 0;
        int    numLevels = 0;
        float  maxRadius = 0.f, cellSizeTop = 0.f;
        float  extent[3] = {0,0,0};
        double ms_buildBV=0, ms_reduce=0, ms_grid=0, ms_assign=0,
               ms_sort=0, ms_cellprop=0, ms_pairprefix=0, ms_broad=0, ms_total=0;
    };
    LastRunStats lastStats{};

    MultiLevelSpatialHashing() = default;

    int effectiveLevels() const {
        int L = numLevels;
        if (L < 1) L = 1;
        if (L > (int)ML_MAX_LEVELS) L = (int)ML_MAX_LEVELS;
        return L;
    }

    void memoryAllocation() {
        if (numFaces == 0) return;
        // Home-only insertion: exactly one entry per face (its home cell). No
        // phantom replication, so the entries buffer is numFaces — not the old
        // numFaces*MAXL*8 — which is what kept the radix sort and pair work
        // bounded. The cross-level query walk reads this same sorted buffer.
        if (centroid.ptr && centroid.size == numFaces * 3
            && entries.size == numFaces) return;
        centroid = VectorBase<METAL, PR>(numFaces * 3);
        radius   = VectorBase<METAL, PR>(numFaces);

        constexpr Index TG = 256;
        Index ng1 = (numFaces + TG - 1) / TG;
        Index ng2 = std::max<Index>((ng1 + TG - 1) / TG, 1);
        radiusReducePartial  = VectorBase<METAL, PR>(std::max<Index>(ng1, 1));
        radiusReducePartial2 = VectorBase<METAL, PR>(ng2);
        if (!maxRadius.ptr) maxRadius = VectorBase<METAL, PR>(1);

        entries = VectorBase<METAL, SHEntry>(numFaces);
    }

    void rebuildFaceObj() {
        if (!scenePtr) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index m = packed.facets.size / 3;
        if (faceObj.ptr && faceObj.size == m) return;
        faceObj = VectorBase<METAL, Index>(m);
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        for (Index obj = 0; obj < numMeshes; ++obj) {
            Index begin = packed.facetsOffsets[obj];
            Index end   = packed.facetsOffsets[obj + 1];
            for (Index f = begin; f < end; ++f) faceObj[f] = obj;
        }
    }

    void rebuildMeshKinds() {
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        if (meshBehaviors.ptr && meshBehaviors.size == numMeshes) return;
        meshBehaviors = VectorBase<METAL, uint32_t>(numMeshes);
        meshShapes    = VectorBase<METAL, uint32_t>(numMeshes);
        auto& meshes = Scene<METAL, PR>::meshes;
        for (Index i = 0; i < numMeshes; ++i) {
            meshBehaviors[i] = (uint32_t)meshes[i].behaviorType;
            meshShapes[i]    = (uint32_t)meshes[i].colliderKind;
        }
    }

    // Per-mesh AABB-diagonal exclusion. Oversized meshes (the floor) are
    // dropped from the grid. Rebuilt every build() because positions move; the
    // exclusion is geometric (current AABB) so a mesh that shrinks/grows past
    // the threshold re-classifies automatically.
    void rebuildFaceExclude() {
        if (numFaces == 0) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        if (!faceExclude.ptr || faceExclude.size != numFaces)
            faceExclude = VectorBase<METAL, uint8_t>(numFaces);
        numExcludedFaces = 0;
        for (Index obj = 0; obj < numMeshes; ++obj) {
            Index vb = packed.statesOffsets[obj];
            Index ve = packed.statesOffsets[obj + 1];
            bool excluded = false;
            if (ve > vb) {
                tinym::vec3 mn(packed.x.ptr[vb*3+0], packed.x.ptr[vb*3+1], packed.x.ptr[vb*3+2]);
                tinym::vec3 mx = mn;
                for (Index v = vb; v < ve; ++v) {
                    tinym::vec3 p(packed.x.ptr[v*3+0], packed.x.ptr[v*3+1], packed.x.ptr[v*3+2]);
                    mn = tinym::min(mn, p);
                    mx = tinym::max(mx, p);
                }
                float diag = (mx - mn).norm();
                excluded = (diag > floorExcludeDiag);
            }
            Index fb = packed.facetsOffsets[obj];
            Index fe = packed.facetsOffsets[obj + 1];
            for (Index f = fb; f < fe; ++f) {
                faceExclude[f] = excluded ? 1u : 0u;
                if (excluded) ++numExcludedFaces;
            }
        }
    }

    void build(Scene<METAL, PR>& scene) {
        scenePtr = &scene;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        numFaces = packed.facets.size / 3;
        rebuildFaceObj();
        rebuildMeshKinds();
        memoryAllocation();
        rebuildFaceExclude();
        if (!buildBVPSO)          buildBVPSO          = MetalKernelContext::getPSO("ml_buildBV");
        if (!reduceMaxRadiusPSO)  reduceMaxRadiusPSO  = MetalKernelContext::getPSO("sh_reduceMaxRadius");
        if (!assignCellsPSO)      assignCellsPSO      = MetalKernelContext::getPSO("ml_assignCells");
        if (!broadPhasePSO)       broadPhasePSO       = MetalKernelContext::getPSO("ml_broadPhase");
    }

    void runBuildBV() {
        if (numFaces == 0 || !buildBVPSO) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        params.numFaces  = (uint32_t)numFaces;
        params.numLevels = (uint32_t)effectiveLevels();
        MetalGlobalContext::setBuffer(packed.x, 0);
        MetalGlobalContext::setBuffer(packed.statesOffsets, 1);
        MetalGlobalContext::setBuffer(packed.facets, 2);
        MetalGlobalContext::setBuffer(faceObj, 3);
        MetalGlobalContext::setBuffer(faceExclude, 4);
        MetalGlobalContext::setBytes(params, 5);
        MetalGlobalContext::setBuffer(centroid, 6);
        MetalGlobalContext::setBuffer(radius, 7);
        MetalGlobalContext::dispatchThreads(buildBVPSO, numFaces);
    }

    void runReduceMaxRadius() {
        if (numFaces == 0 || !reduceMaxRadiusPSO) return;
        constexpr Index TG = 256;
        auto dispatch = [&](VectorBase<METAL, PR>& in, VectorBase<METAL, PR>& out, uint32_t cnt) {
            Index ng = (cnt + TG - 1) / TG;
            MetalGlobalContext::setBuffer(in, 0);
            MetalGlobalContext::setBuffer(out, 1);
            MetalGlobalContext::setBytes(cnt, 2);
            MetalGlobalContext::dispatchThreads(reduceMaxRadiusPSO, ng * TG, TG);
            return ng;
        };
        uint32_t cnt = (uint32_t)numFaces;
        Index ng = (cnt + TG - 1) / TG;
        VectorBase<METAL, PR>* dst = (ng == 1) ? &maxRadius : &radiusReducePartial;
        dispatch(radius, *dst, cnt);
        VectorBase<METAL, PR>* src = dst;
        VectorBase<METAL, PR>* alt = (src == &radiusReducePartial)
                                     ? &radiusReducePartial2 : &radiusReducePartial;
        while (ng > 1) {
            cnt = (uint32_t)ng;
            Index ng2 = (cnt + TG - 1) / TG;
            VectorBase<METAL, PR>* nextDst = (ng2 == 1) ? &maxRadius : alt;
            dispatch(*src, *nextDst, cnt);
            ng = ng2;
            src = nextDst;
            alt = (alt == &radiusReducePartial) ? &radiusReducePartial2 : &radiusReducePartial;
        }
    }

    // Host: scene AABB over NON-excluded meshes, then geometric per-level cell
    // sizes / grid resolutions from maxRadius. Caller must commitAndWait first.
    void computeGrid(PR margin) {
        if (numFaces == 0) return;
        auto& packed = Scene<METAL, PR>::packedMeshData;
        Index numMeshes = Scene<METAL, PR>::numMeshes;
        params.numFaces  = (uint32_t)numFaces;
        params.epsilon   = (float)margin;

        // AABB over included meshes only (so the 50-wide floor doesn't blow up
        // grid resolution even if some excluded face slipped through).
        bool have = false;
        tinym::vec3 mn(0,0,0), mx(0,0,0);
        for (Index obj = 0; obj < numMeshes; ++obj) {
            Index vb = packed.statesOffsets[obj];
            Index ve = packed.statesOffsets[obj + 1];
            if (ve <= vb) continue;
            Index ff = packed.facetsOffsets[obj];
            if (ff < numFaces && faceExclude.ptr && faceExclude[ff]) continue;
            for (Index v = vb; v < ve; ++v) {
                tinym::vec3 p(packed.x.ptr[v*3+0], packed.x.ptr[v*3+1], packed.x.ptr[v*3+2]);
                if (!have) { mn = mx = p; have = true; }
                else { mn = tinym::min(mn, p); mx = tinym::max(mx, p); }
            }
        }
        if (!have) { mn = tinym::vec3(0,0,0); mx = tinym::vec3(0,0,0); }

        // Min face radius over INCLUDED faces (radius[]==0 for excluded). The
        // radius buffer is host-coherent here (caller commitAndWaits after the
        // reduce). Drives the FINEST cell so the smallest face maps ~1/cell.
        float maxR = (float)maxRadius[0];
        float minR = maxR;
        for (Index f = 0; f < numFaces; ++f) {
            float rf = (float)radius[f];
            if (rf > 1e-7f && rf < minR) minR = rf;
        }
        if (minR <= 0.f) minR = std::max(maxR, 1e-6f);
        // Clamp the finest cell so the dynamic range never exceeds what the
        // level budget spans (2^(MAXL-1) = 8x). Without this, a single
        // degenerate (near-zero-area) draped-cloth triangle collapses c0 -> 0,
        // which caps every level's resolution and re-creates the pile-up.
        float minFloor = maxR / (float)(1u << (ML_MAX_LEVELS - 1));
        if (minR < minFloor) minR = minFloor;

        // c0 = finest cell = 2*minR (a smallest face fits in one cell).
        float c0 = 2.0f * minR * cellSizeFactor;
        if (c0 <= 0.f) c0 = std::max((float)margin, 1e-6f);
        // Active levels: enough that cellSize[L-1] = c0*2^(L-1) covers the
        // largest margin-inflated BV (2*(maxR+margin)). Capped by the user's
        // numLevels and the hard ML_MAX_LEVELS buffer budget.
        float topNeeded = 2.0f * (maxR + (float)margin);
        int L = 1;
        while (L < effectiveLevels() && c0 * (float)(1u << (L - 1)) < topNeeded) ++L;
        params.numLevels = (uint32_t)L;

        params.originMin[0] = mn.x;
        params.originMin[1] = mn.y;
        params.originMin[2] = mn.z;
        float ext[3] = { mx.x - mn.x, mx.y - mn.y, mx.z - mn.z };
        const uint32_t RES_CAP = 640;   // 640^3 < 2^28 linear-id budget
        for (int k = 0; k < (int)ML_MAX_LEVELS; ++k) {
            float cs = c0 * (float)(1u << k);
            params.cellSize[k] = cs;
            for (int a = 0; a < 3; ++a) {
                uint32_t r = (uint32_t)std::ceil(ext[a] / cs);
                if (r == 0) r = 1;
                if (r > RES_CAP) r = RES_CAP;
                params.gridRes[k*3 + a] = r;
            }
        }

        lastStats.extent[0] = ext[0];
        lastStats.extent[1] = ext[1];
        lastStats.extent[2] = ext[2];
        lastStats.maxRadius = maxR;
        lastStats.cellSizeTop = params.cellSize[L-1];
        lastStats.numPoints = (numMeshes > 0) ? packed.statesOffsets[numMeshes] : 0;
    }

    void runAssignCells() {
        if (numFaces == 0 || !assignCellsPSO) return;
        MetalGlobalContext::setBuffer(centroid, 0);
        MetalGlobalContext::setBuffer(radius,   1);
        MetalGlobalContext::setBuffer(faceExclude, 2);
        MetalGlobalContext::setBytes(params,    3);
        MetalGlobalContext::setBuffer(entries,  4);
        MetalGlobalContext::dispatchThreads(assignCellsPSO, numFaces);
    }

    void runRadixSort() {
        if (numFaces == 0) return;
        sorter.sort(entries);
    }

    Index findNumValid() {
        Index n = entries.size;
        if (n == 0) return 0;
        if (entries[n - 1].cellID != 0xFFFFFFFFu) return n;
        if (entries[0].cellID == 0xFFFFFFFFu)     return 0;
        Index lo = 0, hi = n;
        while (lo < hi) {
            Index mid = lo + (hi - lo) / 2;
            if (entries[mid].cellID == 0xFFFFFFFFu) hi = mid;
            else                                    lo = mid + 1;
        }
        return lo;
    }


    void runBroadPhase(PR margin, bool enableSelfCollisions) {
        if (numFaces == 0 || !broadPhasePSO) return;
        auto& packed    = Scene<METAL, PR>::packedMeshData;
        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        MLBroadParamsHost bp{};
        bp.numFaces             = (uint32_t)numFaces;
        bp.numValid             = (uint32_t)numValidEntries;
        bp.maxNumCollisions     = (uint32_t)packedCol.maxNumCollisions;
        bp.enableSelfCollisions = enableSelfCollisions ? 1u : 0u;
        bp.numLevels            = params.numLevels;   // dynamic active level count
        bp.epsilon              = (float)margin;
        MetalGlobalContext::setBuffer(entries,                      0);
        MetalGlobalContext::setBytes(params,                        1);  // MLParams grid
        MetalGlobalContext::setBuffer(faceObj,                      2);
        MetalGlobalContext::setBuffer(packed.facets,                3);
        MetalGlobalContext::setBuffer(packed.facetsOffsets,         4);
        MetalGlobalContext::setBuffer(centroid,                     5);
        MetalGlobalContext::setBuffer(radius,                       6);
        MetalGlobalContext::setBuffer(meshBehaviors,                7);
        MetalGlobalContext::setBuffer(meshShapes,                   8);
        MetalGlobalContext::setBytes(bp,                            9);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 10);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions,    11);
        MetalGlobalContext::dispatchThreads(broadPhasePSO, numFaces);
    }

    void refit() { /* grid rebuilt each detectCollisions(); mirror BVH surface */ }
    void enlargeTrajectory(PR /*dt*/) { /* CCD swept BVs out of scope */ }
    void queryBegin() {
        Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] = 0;
    }

    void detectCollisions(PR margin, bool enableSelfCollisions = true) {
        using Clock = std::chrono::steady_clock;
        auto run = [&](const char* name, auto&& fn, double& dst) {
            auto a = Clock::now();
            fn();
            auto b = Clock::now();
            dst = std::chrono::duration<double, std::milli>(b - a).count();
            if (profiler) profiler->addSample(std::string(name), dst);
        };
        auto t0 = Clock::now();
        queryBegin();

        run("mlsh_buildBV", [&]{ runBuildBV(); }, lastStats.ms_buildBV);
        run("mlsh_reduce",  [&]{
            runReduceMaxRadius();
            MetalGlobalContext::commitAndWait();
        }, lastStats.ms_reduce);
        run("mlsh_grid",    [&]{ computeGrid(margin); }, lastStats.ms_grid);
        run("mlsh_assign",  [&]{ runAssignCells(); }, lastStats.ms_assign);
        run("mlsh_sort",    [&]{
            runRadixSort();
            MetalGlobalContext::commitAndWait();
            numValidEntries = findNumValid();
        }, lastStats.ms_sort);
        // Home-only insertion + cross-level query walk: no cell-property /
        // pair-prefix passes (and their two host syncs) — the per-face broad
        // phase binary-searches the sorted entries directly.
        lastStats.ms_cellprop   = 0.0;
        lastStats.ms_pairprefix = 0.0;
        numCells = 0; numCandidatePairs = 0;
        run("mlsh_broad",   [&]{
            runBroadPhase(margin, enableSelfCollisions);
            if (verbose) MetalGlobalContext::commitAndWait();
        }, lastStats.ms_broad);

        auto t1 = Clock::now();
        lastStats.ms_total          = std::chrono::duration<double, std::milli>(t1 - t0).count();
        lastStats.numFaces          = numFaces;
        lastStats.numValidEntries   = numValidEntries;
        lastStats.numCells          = numCells;
        lastStats.numCandidatePairs = numCandidatePairs;
        lastStats.numExcluded       = numExcludedFaces;
        lastStats.numLevels         = effectiveLevels();
        if (verbose) {
            lastStats.numBroadOut =
                Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0];
        }
    }

    void printLastStats(std::ostream& os, Index frame, Index substep) const {
        os << "[F=" << frame << " S=" << substep << "] mlsh:"
           << " L=" << lastStats.numLevels
           << " faces=" << lastStats.numFaces
           << " excl=" << lastStats.numExcluded
           << " valid=" << lastStats.numValidEntries
           << " cells=" << lastStats.numCells
           << " pairs=" << lastStats.numCandidatePairs
           << " broadOut=" << lastStats.numBroadOut
           << " maxR=" << lastStats.maxRadius
           << " cellTop=" << lastStats.cellSizeTop
           << " ext=" << lastStats.extent[0] << "x" << lastStats.extent[1]
           << "x" << lastStats.extent[2]
           << " | bv=" << lastStats.ms_buildBV
           << " red=" << lastStats.ms_reduce
           << " grid=" << lastStats.ms_grid
           << " asgn=" << lastStats.ms_assign
           << " srt=" << lastStats.ms_sort
           << " cell=" << lastStats.ms_cellprop
           << " pp=" << lastStats.ms_pairprefix
           << " brd=" << lastStats.ms_broad
           << " tot=" << lastStats.ms_total << "ms\n";
    }

    void queryEnd() {
        MetalGlobalContext::commitAndWait();
        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        if (packedCol.numBroadCollisions[0] > packedCol.maxNumCollisions) {
            std::cout << "[MLSH] broad-phase overflow: "
                      << packedCol.numBroadCollisions[0]
                      << "/" << packedCol.maxNumCollisions << "\n";
        }
    }

    void showBox()      {}
    void showSceneBox() {}
    void queryClickRay(const Ray& /*ray*/) {}
};


// ─────────────────────────────────────────────────────────────────────────
// CpuSpatialHash — CPU uniform-grid broad phase (third sibling path).
//
// Why a CPU broad phase at all:
//   * The GPU single-level SH pays 4+ commitAndWait per detect and emits
//     SIX rows per cell-coincident face PAIR, which routinely blows past
//     packedCollisionData.maxNumCollisions (= numPoints * 15) and silently
//     drops real contacts.
//   * Worse, the GPU SH leaves its broad kernel UNCOMMITTED, so the sync
//     narrow consumer (NarrowPhase::narrow) — which early-outs on a CPU
//     read of numBroadCollisions[0] — sees the queryBegin() reset value 0
//     under the CPU solvers (PbdSystem::step commitAndWaits at its top,
//     i.e. AFTER narrow already bailed). Narrow then never runs.
// Writing the counter and the rows CPU-side removes both problems by
// construction: no dispatch, no sync, and the count is already visible to
// the very next CPU read.
//
// Output convention: VERTEX-major. One BroadCollision row per
// (query vertex, target face) candidate, deduped — not the GPU path's 6
// rows per face pair — so the row count stays near the true contact count
// and the fixed buffer is not the binding constraint.
//
// Namespaces (mirror src/metal/spatialhashing.metal:446-478 exactly):
//   indexPair.point    = LOCAL vertex index inside the query mesh
//   indexPair.triangle = LOCAL face index inside the target mesh
//   objPair            = mesh ARRAY indices (statesOffsets/facetsOffsets subs)
//   behaviorPair       = (uint)BehaviorType of query / target
//   shapePair          = (uint)GeneralMesh::colliderKind of query / target
//
// LIMITATION — CPU-solver path only. Under the GPU symplectic solver the
// mid-frame vertex positions are still GPU-pending when this runs, so it
// would read frame-stale `x` between syncs. It is intended for usePbd /
// usePd, where `x` is CPU-fresh every substep. (It stays correct-but-lagged
// under the symplectic path, not incorrect-and-silent.)
//
// LIMITATION — single level. cellSize is the MAX inflated face extent in
// the scene, so one giant triangle (e.g. a 2-triangle ground plane) makes
// the grid degenerate into near-brute-force. Tessellated colliders (the
// addPlane(24, 3.0) idiom) are the intended input; the multi-level GPU
// hgrid is the answer for mixed scales.
// ─────────────────────────────────────────────────────────────────────────
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>

template <typename PR>
struct CpuSpatialHash {
    // ---- Stats (per detectCollisions call + cumulative) -----------------
    struct Stats {
        uint32_t rows        = 0;   // rows actually written this call
        uint32_t droppedRows = 0;   // candidates lost to maxNumCollisions
        uint32_t targetFaces = 0;   // faces inserted into the grid
        uint32_t queryVerts  = 0;   // vertices probed
        uint32_t candidates  = 0;   // post-dedup, pre-precull candidates
        uint32_t cellInserts = 0;   // face→cell insertions
        double   cellSize    = 0.0;
        double   ms_total    = 0.0;
    };
    Stats    lastStats;
    uint64_t totalRows    = 0;
    uint64_t totalDropped = 0;
    uint64_t totalCalls   = 0;
    bool     verbose      = false;

    // ---- Scratch (members: reused across calls, no per-call churn) ------
    // Face key box = face AABB inflated by `margin`. The precull below is
    // the box form of the spec's sphere test (strictly TIGHTER, and still a
    // conservative superset of every true within-margin contact).
    struct FaceRec {
        float    bmin[3];
        float    bmax[3];
        uint32_t obj;        // mesh array index
        uint32_t localFace;  // face index inside that mesh
    };
    struct Cell {
        uint32_t              gen = 0;   // generation stamp; != gridGen ⇒ stale
        std::vector<uint32_t> items;     // indices into `faces`
    };
    std::vector<FaceRec>                faces;
    std::unordered_map<uint64_t, Cell>  grid;
    uint32_t                            gridGen = 0;
    std::vector<uint32_t>               seen;    // per-vertex dedup scratch

    // Exact 21-bit-per-axis cell key (biased so negatives stay in range).
    // Coordinates beyond ±2^20 alias, which can only MERGE cells → extra
    // candidates that the precull rejects. Never causes a miss.
    static inline uint64_t cellKey(int32_t x, int32_t y, int32_t z) {
        const uint64_t ux = (uint64_t)(uint32_t)(x + 0x100000) & 0x1FFFFFull;
        const uint64_t uy = (uint64_t)(uint32_t)(y + 0x100000) & 0x1FFFFFull;
        const uint64_t uz = (uint64_t)(uint32_t)(z + 0x100000) & 0x1FFFFFull;
        return ux | (uy << 21) | (uz << 42);
    }

    // ---- BVH<SCENE,OBJECT> surface mirrors (all no-ops) -----------------
    void build(Scene<METAL, PR>& /*scene*/) {}
    void refit()                        {}   // grid rebuilt each detect
    void enlargeTrajectory(PR /*dt*/)   {}   // swept box built inline
    void queryBegin() {
        Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] = 0;
    }
    void queryEnd()                     {}   // nothing to sync: all CPU
    void showBox()                      {}
    void showSceneBox()                 {}
    void queryClickRay(const Ray& /*ray*/) {}

    // ---- shared guts (used by BOTH the full and the self-only pass) -----
    static constexpr Index kAllMeshes = ~Index(0);

    // Fill `faces` with the margin-inflated face boxes of every collidable
    // mesh (onlyMesh == kAllMeshes) or of exactly one mesh. Returns the
    // largest axis extent over the collected boxes (0 if none).
    float collectFaceBoxes(float m, Index onlyMesh) {
        auto& meshes = Scene<METAL, PR>::meshes;
        const Index numMeshes = Scene<METAL, PR>::numMeshes;
        faces.clear();
        float maxExtent = 0.f;
        const Index tBegin = (onlyMesh == kAllMeshes) ? Index(0) : onlyMesh;
        const Index tEnd   = (onlyMesh == kAllMeshes) ? numMeshes : onlyMesh + 1;
        for (Index t = tBegin; t < tEnd && t < numMeshes; ++t) {
            auto& tm = meshes[t];
            if (!tm.collidable) continue;
            if (!tm.state.x.ptr || !tm.adjacency.facets.ptr) continue;
            const Index nf = tm.adjacency.facets.size / 3;
            if (nf == 0) continue;
            const Index* F = tm.adjacency.facets.ptr;
            const PR*    X = tm.state.x.ptr;
            for (Index f = 0; f < nf; ++f) {
                const Index i0 = F[f * 3 + 0];
                const Index i1 = F[f * 3 + 1];
                const Index i2 = F[f * 3 + 2];
                FaceRec rec;
                bool ok = true;
                for (int a = 0; a < 3; ++a) {
                    const float p0 = (float)X[i0 * 3 + a];
                    const float p1 = (float)X[i1 * 3 + a];
                    const float p2 = (float)X[i2 * 3 + a];
                    if (!std::isfinite(p0) || !std::isfinite(p1)
                        || !std::isfinite(p2)) { ok = false; break; }
                    const float lo = std::min(p0, std::min(p1, p2));
                    const float hi = std::max(p0, std::max(p1, p2));
                    rec.bmin[a] = lo - m;
                    rec.bmax[a] = hi + m;
                }
                if (!ok) continue;   // NaN vertex: drop the face, keep going
                rec.obj       = (uint32_t)t;
                rec.localFace = (uint32_t)f;
                for (int a = 0; a < 3; ++a)
                    maxExtent = std::max(maxExtent, rec.bmax[a] - rec.bmin[a]);
                faces.push_back(rec);
            }
        }
        lastStats.targetFaces += (uint32_t)faces.size();
        return maxExtent;
    }

    // Squared distance from a point to a triangle (Ericson, Real-Time
    // Collision Detection §5.1.5). Used ONLY by the self-row precull below.
    static inline float pointTriSq(const float p[3], const float a[3],
                                   const float b[3], const float c[3]) {
        auto dot = [](const float* u, const float* v) {
            return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
        };
        float ab[3], ac[3], ap[3], q[3];
        for (int i = 0; i < 3; ++i) {
            ab[i] = b[i] - a[i]; ac[i] = c[i] - a[i]; ap[i] = p[i] - a[i];
        }
        const float d1 = dot(ab, ap), d2 = dot(ac, ap);
        if (d1 <= 0.f && d2 <= 0.f) { for (int i=0;i<3;++i) q[i] = a[i]; }
        else {
            float bp[3]; for (int i=0;i<3;++i) bp[i] = p[i] - b[i];
            const float d3 = dot(ab, bp), d4 = dot(ac, bp);
            if (d3 >= 0.f && d4 <= d3) { for (int i=0;i<3;++i) q[i] = b[i]; }
            else {
                const float vc = d1*d4 - d3*d2;
                if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
                    const float den = d1 - d3;
                    const float v = (den != 0.f) ? d1/den : 0.f;
                    for (int i=0;i<3;++i) q[i] = a[i] + v*ab[i];
                } else {
                    float cp[3]; for (int i=0;i<3;++i) cp[i] = p[i] - c[i];
                    const float d5 = dot(ab, cp), d6 = dot(ac, cp);
                    if (d6 >= 0.f && d5 <= d6) { for (int i=0;i<3;++i) q[i] = c[i]; }
                    else {
                        const float vb = d5*d2 - d1*d6;
                        if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
                            const float den = d2 - d6;
                            const float w = (den != 0.f) ? d2/den : 0.f;
                            for (int i=0;i<3;++i) q[i] = a[i] + w*ac[i];
                        } else {
                            const float va = d3*d6 - d5*d4;
                            if (va <= 0.f && (d4-d3) >= 0.f && (d5-d6) >= 0.f) {
                                const float den = (d4-d3) + (d5-d6);
                                const float w = (den != 0.f) ? (d4-d3)/den : 0.f;
                                for (int i=0;i<3;++i) q[i] = b[i] + w*(c[i]-b[i]);
                            } else {
                                const float den = va + vb + vc;
                                const float v = (den != 0.f) ? vb/den : 0.f;
                                const float w = (den != 0.f) ? vc/den : 0.f;
                                for (int i=0;i<3;++i) q[i] = a[i] + ab[i]*v + ac[i]*w;
                            }
                        }
                    }
                }
            }
        }
        float s = 0.f;
        for (int i = 0; i < 3; ++i) { const float e = p[i]-q[i]; s += e*e; }
        return s;
    }

    // Insert the current `faces` into the generation-stamped grid. Stamping
    // keeps the map's buckets and each cell vector's capacity warm across
    // calls (map.clear() would free every one of them).
    void rebuildGridFromFaces(float invCs) {
        if (++gridGen == 0) { grid.clear(); gridGen = 1; }
        for (uint32_t fi = 0; fi < (uint32_t)faces.size(); ++fi) {
            const FaceRec& fr = faces[fi];
            int lo[3], hi[3];
            bool sane = true;
            for (int a = 0; a < 3; ++a) {
                lo[a] = (int)std::floor(fr.bmin[a] * invCs);
                hi[a] = (int)std::floor(fr.bmax[a] * invCs);
                if (hi[a] - lo[a] > 64) { sane = false; break; }
            }
            if (!sane) continue;   // pathological span (shouldn't happen)
            for (int z = lo[2]; z <= hi[2]; ++z)
              for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x) {
                    Cell& c = grid[cellKey(x, y, z)];
                    if (c.gen != gridGen) { c.gen = gridGen; c.items.clear(); }
                    c.items.push_back(fi);
                    ++lastStats.cellInserts;
                }
        }
    }

    // Probe one query mesh's vertices against the current grid and APPEND
    // rows at [count, cap). `selfOnly` keeps q == t rows only (hybrid pass).
    void probeMeshVertices(Index q, float m, float invCs,
                           bool enableSelfCollisions,
                           bool selfOnly, BroadCollision* out, Index cap,
                           Index& count, uint32_t& dropped) {
        auto& meshes = Scene<METAL, PR>::meshes;
        auto& qm = meshes[q];
        if (!qm.state.x.ptr) return;
        const Index nv = qm.state.x.size / 3;
        if (nv == 0) return;
        const PR* X  = qm.state.x.ptr;
        // xPrev is seeded to x at Scene::pack and re-snapshotted every
        // substep, so the swept box is the same segment the CCD narrow
        // phase (D-013) tests — one substep lagged, by construction.
        const PR* XP = qm.state.xPrev.ptr ? qm.state.xPrev.ptr : X;
        const uint32_t behQ = (uint32_t)qm.behaviorType;
        const uint32_t shpQ = (uint32_t)qm.colliderKind;

        for (Index v = 0; v < nv; ++v) {
            float vlo[3], vhi[3], pCur[3], pPrev[3];
            bool ok = true;
            for (int a = 0; a < 3; ++a) {
                const float pc = (float)X[v * 3 + a];
                float       pp = (float)XP[v * 3 + a];
                if (!std::isfinite(pc)) { ok = false; break; }
                // Guard a garbage / teleported xPrev: a sweep longer than
                // a metre is a pin snap, not motion — use the point.
                if (!std::isfinite(pp) || std::abs(pp - pc) > 1.f) pp = pc;
                pCur[a] = pc; pPrev[a] = pp;
                vlo[a] = std::min(pc, pp);
                vhi[a] = std::max(pc, pp);
            }
            if (!ok) continue;
            ++lastStats.queryVerts;

            int lo[3], hi[3];
            bool sane = true;
            for (int a = 0; a < 3; ++a) {
                lo[a] = (int)std::floor(vlo[a] * invCs);
                hi[a] = (int)std::floor(vhi[a] * invCs);
                if (hi[a] - lo[a] > 64) { sane = false; break; }
            }
            if (!sane) continue;
            const bool multiCell = (lo[0] != hi[0]) || (lo[1] != hi[1])
                                || (lo[2] != hi[2]);
            if (multiCell) seen.clear();

            for (int z = lo[2]; z <= hi[2]; ++z)
              for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x) {
                    auto it = grid.find(cellKey(x, y, z));
                    if (it == grid.end() || it->second.gen != gridGen)
                        continue;
                    for (uint32_t fi : it->second.items) {
                        // Dedup: a face straddling two probed cells must
                        // yield ONE row. Single-cell probes can't repeat.
                        if (multiCell) {
                            bool dup = false;
                            for (uint32_t s : seen)
                                if (s == fi) { dup = true; break; }
                            if (dup) continue;
                            seen.push_back(fi);
                        }
                        const FaceRec& fr = faces[fi];
                        // Distance precull: swept-vertex box vs the
                        // margin-inflated face box.
                        if (vhi[0] < fr.bmin[0] || vlo[0] > fr.bmax[0]
                         || vhi[1] < fr.bmin[1] || vlo[1] > fr.bmax[1]
                         || vhi[2] < fr.bmin[2] || vlo[2] > fr.bmax[2])
                            continue;
                        const Index t = (Index)fr.obj;
                        if (selfOnly && t != q) continue;
                        if (t == q) {
                            // Self rows only when asked (mirrors
                            // spatialhashing.metal:423).
                            if (!enableSelfCollisions) continue;
                            // Cheap early-out on the vertex's OWN face;
                            // the narrow kernel drops the rest of ring-1
                            // via sceneVertexAdjFacets.
                            const Index* TF = meshes[t].adjacency.facets.ptr;
                            const uint32_t b = fr.localFace * 3;
                            if (TF[b] == v || TF[b + 1] == v
                                || TF[b + 2] == v) continue;
                            // EXACT point-triangle precull, self rows only.
                            // The AABB test above cannot reject the
                            // diagonally-opposite triangle of the vertex's
                            // OWN 1-ring quad: on a regular grid the vertex
                            // sits exactly ON that triangle's un-inflated
                            // AABB corner, so every flat sheet emitted ~1
                            // junk row PER FACE forever. True distance there
                            // is a full edge length. Those rows never became
                            // contacts, but they consumed the row budget and
                            // — because they shift the order of
                            // narrowCollisions → vertColFacets → PBD's
                            // Gauss-Seidel projection — perturbed marginal
                            // scenes (they flipped PBD-7 to zero contacts).
                            // Both swept endpoints are tested: per-substep
                            // cloth motion is orders of magnitude below an
                            // edge length, so this keeps the conservatism
                            // that matters. Inter-object rows are NOT
                            // touched — only q == t takes this branch.
                            {
                                const PR* TX = meshes[t].state.x.ptr;
                                float A[3], B[3], C[3];
                                for (int a2 = 0; a2 < 3; ++a2) {
                                    A[a2] = (float)TX[TF[b    ]*3 + a2];
                                    B[a2] = (float)TX[TF[b + 1]*3 + a2];
                                    C[a2] = (float)TX[TF[b + 2]*3 + a2];
                                }
                                const float m2 = m * m;
                                if (pointTriSq(pCur,  A, B, C) > m2
                                 && pointTriSq(pPrev, A, B, C) > m2) continue;
                            }
                        }
                        ++lastStats.candidates;
                        if (count >= cap) { ++dropped; continue; }
                        BroadCollision& row = out[count++];
                        row.indexPair.point    = v;
                        row.indexPair.triangle = fr.localFace;
                        row.objPair.query      = q;
                        row.objPair.target     = t;
                        row.behaviorPair.query  = behQ;
                        row.behaviorPair.target =
                            (uint32_t)meshes[t].behaviorType;
                        row.shapePair.query  = shpQ;
                        row.shapePair.target =
                            (uint32_t)meshes[t].colliderKind;
                    }
                }
        }
    }

    // cellSize = largest inflated face extent ⇒ every face spans at most 2
    // cells per axis (≤8 inserts), which bounds the grid build.
    static inline float cellSizeFrom(float maxExtent) {
        return (maxExtent > 1e-5f) ? maxExtent : 1e-5f;
    }

    void detectCollisions(PR margin, bool enableSelfCollisions = true) {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();

        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        auto& meshes    = Scene<METAL, PR>::meshes;
        const Index numMeshes = Scene<METAL, PR>::numMeshes;

        lastStats = Stats{};
        // 1) Reset the counter CPU-side. This IS the narrow-phase gate.
        if (!packedCol.numBroadCollisions.ptr) return;
        packedCol.numBroadCollisions[0] = 0;
        if (numMeshes == 0 || !packedCol.broadCollisions.ptr) return;

        // 2) Collect target-face key boxes from the live state.x.
        const float m = (float)margin;
        const float maxExtent = collectFaceBoxes(m, kAllMeshes);
        if (faces.empty()) {
            lastStats.ms_total = std::chrono::duration<double, std::milli>(
                Clock::now() - t0).count();
            ++totalCalls;
            return;
        }

        // 3) + 4) Grid.
        const float cs = cellSizeFrom(maxExtent);
        const float invCs = 1.f / cs;
        lastStats.cellSize = (double)cs;
        rebuildGridFromFaces(invCs);

        // 5) Query. Skip Float / Kinematic (target-only behaviors) and any
        //    mesh whose collidable master switch is off.
        BroadCollision* out = packedCol.broadCollisions.ptr;
        const Index     cap = packedCol.maxNumCollisions;
        Index    count   = 0;
        uint32_t dropped = 0;

        for (Index q = 0; q < numMeshes; ++q) {
            auto& qm = meshes[q];
            if (!qm.collidable) continue;
            if (qm.behaviorType == BehaviorType::Float
                || qm.behaviorType == BehaviorType::Kinematic) continue;
            probeMeshVertices(q, m, invCs, enableSelfCollisions,
                              /*selfOnly*/false, out, cap, count, dropped);
        }

        // 6) Publish the count CPU-side — the sync narrow phase reads this
        //    directly and now sees a truthful non-zero value.
        packedCol.numBroadCollisions[0] = count;
        finishStats(t0, count, dropped, cap);
    }

    // ── Hybrid SELF pass (Simulator::useCpuShSelf) ───────────────────────
    // Emits ONLY q == t rows, for cloth meshes, starting from row 0 — it
    // OWNS the counter reset. The caller must run this BEFORE the BVH
    // detect and then call that detect with self disabled and
    // resetCounter == false, so the BVH's GPU atomics append after these
    // rows. The reverse order would race: the BVH's device atomics are
    // still pending when the CPU would write.
    //
    // Same CPU-solver caveat as detectCollisions: `x` is read host-side, so
    // under the GPU symplectic solver mid-frame positions are GPU-pending
    // and this reads frame-stale x between syncs. Intended for usePbd/usePd.
    // `globalSelf` = Simulator::enableSelfCollisions (the scene-wide switch).
    // A mesh contributes self rows when the global switch is on OR when its
    // own inspector toggle (GeneralMesh::selfCollide, "자기 충돌") is set —
    // this is what makes the per-object checkbox load-bearing instead of the
    // save/restore-only field it used to be.
    void detectSelfCollisions(PR margin, bool globalSelf) {
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();

        auto& packedCol = Scene<METAL, PR>::packedCollisionData;
        auto& meshes    = Scene<METAL, PR>::meshes;
        const Index numMeshes = Scene<METAL, PR>::numMeshes;

        lastStats = Stats{};
        if (!packedCol.numBroadCollisions.ptr) return;
        packedCol.numBroadCollisions[0] = 0;
        if (numMeshes == 0 || !packedCol.broadCollisions.ptr) return;

        BroadCollision* out = packedCol.broadCollisions.ptr;
        const Index     cap = packedCol.maxNumCollisions;
        Index    count   = 0;
        uint32_t dropped = 0;

        for (Index q = 0; q < numMeshes; ++q) {
            auto& qm = meshes[q];
            if (!qm.collidable) continue;
            // Self rows on a rigid / float / kinematic body are meaningless
            // row volume — cloth only.
            if (qm.behaviorType != BehaviorType::TriangularCloth
                && qm.behaviorType != BehaviorType::FastGridCloth) continue;
            if (!globalSelf && !qm.selfCollide) continue;
            if (!qm.state.x.ptr || !qm.adjacency.facets.ptr) continue;
            if (qm.adjacency.facets.size < 3) continue;

            // One grid per mesh: only that mesh's own faces are candidates,
            // and its own face size sets the cell size.
            const float m = (float)margin;
            const float maxExtent = collectFaceBoxes(m, q);
            if (faces.empty()) continue;
            const float cs = cellSizeFrom(maxExtent);
            lastStats.cellSize = (double)cs;
            const float invCs = 1.f / cs;
            rebuildGridFromFaces(invCs);
            probeMeshVertices(q, m, invCs, /*enableSelfCollisions*/true,
                              /*selfOnly*/true, out, cap, count, dropped);
        }

        packedCol.numBroadCollisions[0] = count;
        finishStats(t0, count, dropped, cap);
    }

    void finishStats(std::chrono::steady_clock::time_point t0,
                     Index count, uint32_t dropped, Index cap) {
        lastStats.rows        = (uint32_t)count;
        lastStats.droppedRows = dropped;
        lastStats.ms_total    = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        totalRows    += lastStats.rows;
        totalDropped += lastStats.droppedRows;
        ++totalCalls;
        if (verbose && dropped)
            std::cout << "[CPSH] broad-phase overflow: dropped " << dropped
                      << " rows (cap " << cap << ")\n";
    }

    void printLastStats(std::ostream& os, Index frame, Index substep) const {
        os << "[F=" << frame << " S=" << substep << "] cpsh:"
           << " tgtFaces=" << lastStats.targetFaces
           << " qVerts="   << lastStats.queryVerts
           << " cellIns="  << lastStats.cellInserts
           << " cand="     << lastStats.candidates
           << " rows="     << lastStats.rows
           << " dropped="  << lastStats.droppedRows
           << " cs="       << lastStats.cellSize
           << " tot="      << lastStats.ms_total << "ms\n";
    }
};


// TODO: BroadPhase, BVH
template <typename BE, typename PR, Index MODE, Index PRIMITIVE>
