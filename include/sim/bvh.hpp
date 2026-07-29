#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

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

// D-024: ray-vs-triangle (Möller–Trumbore). Used by BVH::queryClickRay's
// leaf to write the actual triangle intersection's t into
// clickRayCollisions, so the smallest-tmin walk on the consumer side
// (production callback + harness) ranks by triangle hits, not by leaf
// AABB hits. Returns true and writes outT (= ray.origin + outT * ray.dir
// is the intersection point) when the ray hits the triangle (p0, p1, p2)
// at outT > 0 with valid barycentric coords. 1e-6 epsilon for the
// determinant test rejects parallel/grazing cases that would divide by
// near-zero; cube triangles in the harness are not grazing the test ray
// so the choice doesn't load-bear there.
inline bool rayTriangleIntersect(const Ray& ray,
                                 const tinym::vec3& p0,
                                 const tinym::vec3& p1,
                                 const tinym::vec3& p2,
                                 float& outT) {
    const float kEps = 1e-6f;
    tinym::vec3 e1 = p1 - p0;
    tinym::vec3 e2 = p2 - p0;
    tinym::vec3 pvec = ray.dir.cross(e2);
    float det = e1.dot(pvec);
    if (std::abs(det) < kEps) return false;
    float invDet = 1.0f / det;
    tinym::vec3 tvec = ray.origin - p0;
    float u = tvec.dot(pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    tinym::vec3 qvec = tvec.cross(e1);
    float v = ray.dir.dot(qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = e2.dot(qvec) * invDet;
    if (t <= 0.0f) return false;
    outT = t;
    return true;
}
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

    // D-029: per-interior-node atomic counter used by the
    // single-dispatch bottomUpBoxes kernel. Zeroed before each
    // dispatch via the zeroVisitCounts kernel. Replaces the
    // retired bottomUpReadyA/B/Progress triple from the prior
    // multi-pass approach.
    VectorBase<METAL, uint32_t> treeVisitCounts;      // numNodes

    // Apetrei (2014) agglomerative LBVH path: temporaries for the
    // single-kernel build (`agglomerativeBuild_*`).
    //
    //   nodeVisitFlags : per-internal-node atomic gate (numInternals = N-1).
    //                    Zeroed before each agglomerative dispatch via the
    //                    existing `zeroVisitCounts` kernel.
    //   nodeRangeLeft  : per-internal-node left  endpoint of covered key
    //   nodeRangeRight :                    right endpoint
    //                    Written by the first/second arrivals on opposite
    //                    sides; second arrival reads both to recover the
    //                    parent's full range when walking up.
    //   rootIndexBuf   : single-int buffer; the agglomerative kernel
    //                    writes the natural slot of the root (which may
    //                    be any internal slot in [0, N-2]). The follow-up
    //                    `agglomerativeSwapRoot` kernel reads it and
    //                    relocates the root to slot 0 so the rest of the
    //                    code (queryPoints / queryClickRay / SCENE reads)
    //                    keeps its `tree[0] is root` invariant.
    //
    // Only allocated/used when `useAgglomerative == true`. Karras path
    // (the default) leaves these buffers idle so the original behavior
    // is preserved bit-for-bit.
    VectorBase<METAL, uint32_t> nodeVisitFlags;       // numInternals
    VectorBase<METAL, int> nodeRangeLeft;             // numInternals
    VectorBase<METAL, int> nodeRangeRight;            // numInternals
    VectorBase<METAL, int> rootIndexBuf;              // 1

    // ---- Sub-object (multi-root) LBVH — square-cloth experiment (Phase 1) ----
    // Partition the primitives into k = 4^s groups by MATERIAL-space tile
    // (fixed for the cloth's lifetime). Each group gets an independent Karras
    // tree in its own slot window of `tree`; there are k roots. Build/refit
    // combine into k roots in ONE dispatch (bottomUpBoxesMultiRoot), which is
    // the divergence experiment. Query integration (scene TLAS over the k
    // roots) is Phase 2 — with this toggle ON but Phase 2 absent, the broad
    // phase still traverses from slot 0 only, so collision is INCOMPLETE.
    // Default OFF ⇒ every existing path is bit-identical. Triangle prim only.
    bool useSubObjectBVH = false;
    int  subBvhSplitS = 1;        // s: k = 4^s groups
    int  subBvhP = 0;             // particleNum1D of the source grid cloth
    int  numGroups = 0;           // k = 4^s (logical; some may be empty/ragged)
    int  subBvhBuiltS = -1;       // s the current group tables were built for

    VectorBase<METAL, uint32_t> groupOfPrim;       // [N] original prim -> group
    VectorBase<METAL, uint32_t> sortedPosToGroup;  // [N] sorted pos -> group (static)
    VectorBase<METAL, uint32_t> groupSize;         // [k] M_g
    VectorBase<METAL, uint32_t> groupPrimBase;     // [k] Σ M_h  (h<g)
    VectorBase<METAL, uint32_t> groupNodeBase;     // [k] Σ (2 M_h - 1)  (h<g)
    uint32_t subBvhNumNodes = 0;                   // 2N - (#nonempty groups)

    // Connectivity clustering (mesh_cluster.hpp) as the grouping source instead
    // of the cloth grid tile-split. Set for ARBITRARY meshes (e.g. the static
    // Human) where deriveSquareClothP() is 0 so the tile-split can't apply. When
    // on, computeSubObjectGroupsClustered() fills groupOfPrim via face dual-graph
    // flood fill and also records each cluster's single-owned vertex list (CSR)
    // for the bidirectional VF phase (Phase 4).
    bool useClusterBVH = false;
    VectorBase<METAL, uint32_t> clusterVertOffsets; // [k+1] CSR offsets
    VectorBase<METAL, uint32_t> clusterVerts;       // [<=V] vertices, single-owned

    // ---- Sub-object top phase: which group subtree(s) does a query point
    // descend? The k group subtrees occupy node slots [0, 2N-k); the top level
    // (point → candidate groups) is a flat cull over the k group root boxes —
    // NOT a tree. (The old binary mini-TLAS was removed: experiments showed it
    // gives no win over a flat cull, and a tree over k boxes has no
    // justification — profiles/experiment/sap-topphase-2026-06-23/FINDINGS.md.)
    //   0 = CPU SAP (default, queryPointsSAP): CPU sweep-and-prune over the k
    //       group root boxes emits candidate (point, groupRoot) pairs, GPU
    //       descends one subtree per pair. Point boxes have constant width
    //       2·margin, so "overlaps group g on X" is a contiguous slice of
    //       x-sorted points → two binary searches/group.
    //   1 = GPU brute (queryPointsGPUTop): each point thread brute-tests the k
    //       group roots on the GPU and descends overlapping subtrees inline —
    //       no CPU sort, no pair buffer. Parity with SAP in the experiment.
    // Mode is live (read at query time).
    int subTopMode = 0;
    // update() sync refactor: pushed from BroadPhase each detect. When false
    // (None/PerFrame) a grouped query MUST use the GPU-brute top phase — the
    // CPU-SAP path reads group roots + query positions on the CPU and would
    // force a sync. Default true ⇒ honor subTopMode as authored.
    bool syncEachPhase = true;
    struct SAPPair { uint32_t pointId; uint32_t entryRoot; };
    VectorBase<METAL, SAPPair> sapPairs;           // [cap] candidate pairs (GPU)
    uint32_t sapPairsCap = 0;                       // capacity in pairs
    // Per query-mesh persistent x-order. Two-mesh broad phase is bidirectional ⇒
    // qIndex alternates each substep; a single shared order would thrash, so key
    // by qIndex. Retained across calls so the previous (near-sorted) order feeds
    // an insertion sort instead of a full std::sort every frame.
    std::unordered_map<Index, std::vector<uint32_t>> sapOrderByQ;
    std::vector<SAPPair>  sapBuild_;                 // CPU pair build scratch (reused)

    // Runtime toggle between the two BVH construction paths. Default is
    // the existing Karras pipeline (`buildTree_*` + `bottomUpBoxes`); set
    // to true to use the Apetrei agglomerative single-kernel build
    // followed by the swap-root post-pass. Designed so either side can be
    // deprecated later by dropping the relevant code paths only.
    bool useAgglomerative = false;

    // D-030: hybrid bottom-up depth knob. GPU combines the first
    // `bottomUpHybridDepth` levels (from leaf side); CPU finishes
    // the remaining top-of-tree. Runtime-tunable so the user can
    // measure the sweet spot. Values: 0 = pure CPU (skips GPU
    // dispatch, calls bottomUpCombine() directly); 1 or 2 = hybrid;
    // large value (>= log2(numPrimitives), e.g., 30) = pure GPU
    // (recovers D-029 walk-to-root behavior). The CPU side reads
    // `treeVisitCounts` as a frontier marker: nodes with count == 2
    // were combined by GPU and their subtrees are skipped by the
    // CPU walk.
    //
    // D-033: default raised from 3 to 30 based on D-031's measured
    // data (profiles/experiment/bvh-refit-2026-05-12/refit_chart_line.png).
    // On Apple Silicon Metal 3.2, FullGPU wins ~1.8x at 100k vertices
    // and ~2.0x at 500k; hybrid values (1, 2, 3) lose to FullGPU at
    // 100k+ and are noise-equivalent at smaller sizes. 30 >= log2 of
    // any realistic tree depth (~21 for 1M leaves), so the kernel
    // walks to root for any mesh. The runtime knob stays — set to a
    // hybrid value (1..log2(N)-1) for measurement/debug.
    int bottomUpHybridDepth = 30;

    int objid; // who made this tree (mesh.id; stable across remove+add)
    // D-041 turn-3 (2026-05-14): mesh INDEX into Scene::meshes — what
    // gets written into broadCollisions.objPair so the Metal narrow
    // kernel can use it as a statesOffsets[] subscript. Distinct from
    // objid (mesh.id) because D-041 turn-2 decoupled id from index;
    // after a removeMesh + add cycle, mesh.id ≠ array index. The narrow
    // kernel reads scenePackedPositionsOffsets[objPair.x/y] as offsets;
    // writing mesh.id (the prior semantic) would mis-index into the
    // offsets array and produce zero contacts post-remove. Set by
    // BroadPhase::build alongside objid. SpatialHashing's faceObj[]
    // already wrote index; this aligns the BVH path with that semantic.
    int objIndex = -1;
    // D-026: cached at build() time from mesh->lifetimeId so
    // BroadPhase::build's Float-mesh skip can verify the slot still
    // refers to the same mesh. Distinct from objid (which is mesh.id
    // and resets on resetScene). See CM-008 (graduated).
    int builtForLifetimeId = -1;
    VectorBase<METAL, int> objIds;
    // Cached at build() from mesh->isStatic. When true the BroadPhase refit
    // loops skip this object's per-leaf AABB recompute (the AABB can't move).
    // Live-updated by the inspector static toggle, same pattern as objBehavior.
    bool objStatic = false;
    // One-time correctness guard for the static fast-path. build()/refit() use
    // the GPU bottom-up combine, which can leave a LARGE tree incompletely
    // combined (root AABB minY=0 vs true geometry); the per-substep CPU enlarge
    // normally masks this. A static mesh skips that enlarge, so the BroadPhase
    // loops run ONE CPU re-combine (combineStaticOnce) the first time they see
    // an uncombined static tree, then freeze. Reset on rebuild / when the mesh
    // goes dynamic again so a re-declared static mesh re-corrects.
    bool staticCombined = false;
    BehaviorType objBehavior;
    VectorBase<METAL, BehaviorType> objBehaviors;
    // P1: the COLLISION identity of this object (collider_pipeline_rework.md
    // decision 1) — not the initializer's ShapeType. This is what the broad
    // phase branches on and what lands in BroadCollision::shapePair, so
    // every downstream shape comparison is in the ColliderKind namespace.
    ColliderKind objCollider;
    VectorBase<METAL, ShapeType> objShapes;

    MTL::ComputePipelineState* fillMortonsPSO;
    MTL::ComputePipelineState* buildTreePSO;
    MTL::ComputePipelineState* buildLeafPSO;
    MTL::ComputePipelineState* bottomUpBoxesPSO;
    MTL::ComputePipelineState* bottomUpBoxesPartialPSO;   // D-030
    MTL::ComputePipelineState* zeroVisitCountsPSO;    // D-029
    MTL::ComputePipelineState* agglomerativeBuildPSO;     // Apetrei 2014
    MTL::ComputePipelineState* agglomerativeSwapRootPSO;  // Apetrei 2014
    MTL::ComputePipelineState* buildTreeGroupedPSO = nullptr;     // sub-object
    MTL::ComputePipelineState* buildLeafGroupedPSO = nullptr;     // sub-object
    MTL::ComputePipelineState* bottomUpBoxesMultiRootPSO = nullptr; // sub-object
    MTL::ComputePipelineState* enlargeLeafPSO = nullptr;          // GPU enlargeTrajectory
    MTL::ComputePipelineState* enlargeLeafGroupedPSO = nullptr;   // GPU enlarge (multi-root)
    MTL::ComputePipelineState* buildSweptLeafPSO = nullptr;        // fused refit+enlarge
    MTL::ComputePipelineState* buildSweptLeafGroupedPSO = nullptr; // fused (multi-root)
    MTL::ComputePipelineState* queryPointsPSO;
    MTL::ComputePipelineState* queryPointsPairsPSO = nullptr;  // SAP top-phase descent
    MTL::ComputePipelineState* queryPointsGroupedPSO = nullptr; // GPU brute top-phase
    MTL::ComputePipelineState* clusterAABBPSO = nullptr;        // per-cluster AABB (cluster VF)
    MTL::ComputePipelineState* queryPointsPairsBoundedPSO = nullptr;  // async per-pair descent
    // Segmented (per-threadgroup) detect+reduce variant — three PSOs
    // matching bvh.metal's queryPointsSegmented → scanReserveSegmented
    // → compactSegmented pipeline. Loaded unconditionally so a runtime
    // A/B toggle (Simulator::useSegmentedBVHQuery) can flip paths
    // without rebuilding pipelines.
    MTL::ComputePipelineState* queryPointsSegmentedPSO;
    MTL::ComputePipelineState* scanReserveSegmentedPSO;
    MTL::ComputePipelineState* compactSegmentedPSO;

    //MTL::ComputePipelineState* radixCountBlocksPSO;
    //MTL::ComputePipelineState* radixComputeOffsetsPSO;
    //MTL::ComputePipelineState* radixScatterBlocksPSO;

    RadixSorter<METAL, MortonNode> sorter;

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
            agglomerativeBuildPSO = MetalKernelContext::getPSO("agglomerativeBuild_Tri");
            // Sub-object (multi-root) LBVH — triangle-only Phase 1 kernels.
            buildTreeGroupedPSO = MetalKernelContext::getPSO("buildTree_Tri_Grouped");
            buildLeafGroupedPSO = MetalKernelContext::getPSO("buildLeaf_Tri_Grouped");
            bottomUpBoxesMultiRootPSO = MetalKernelContext::getPSO("bottomUpBoxesMultiRoot");
            // GPU enlargeTrajectory (triangle-only; edge BVH keeps the CPU loop).
            enlargeLeafPSO = MetalKernelContext::getPSO("enlargeLeaf_Tri");
            enlargeLeafGroupedPSO = MetalKernelContext::getPSO("enlargeLeaf_Tri_Grouped");
            // Fused refit+enlarge (triangle-only).
            buildSweptLeafPSO = MetalKernelContext::getPSO("buildSweptLeaf_Tri");
            buildSweptLeafGroupedPSO = MetalKernelContext::getPSO("buildSweptLeaf_Tri_Grouped");
        } else if constexpr (PRIMITIVE == BVHPRIMITIVE::EDGE) {
            fillMortonsPSO = MetalKernelContext::getPSO("fillMortons_Edge");
            buildTreePSO = MetalKernelContext::getPSO("buildTree_Edge");
            buildLeafPSO = MetalKernelContext::getPSO("buildLeaf_Edge");
            agglomerativeBuildPSO = MetalKernelContext::getPSO("agglomerativeBuild_Edge");
        }
        bottomUpBoxesPSO = MetalKernelContext::getPSO("bottomUpBoxes");
        bottomUpBoxesPartialPSO = MetalKernelContext::getPSO("bottomUpBoxesPartial"); // D-030
        zeroVisitCountsPSO = MetalKernelContext::getPSO("zeroVisitCounts"); // D-029
        agglomerativeSwapRootPSO = MetalKernelContext::getPSO("agglomerativeSwapRoot");
        queryPointsPSO = MetalKernelContext::getPSO("queryPoints");
        queryPointsPairsPSO = MetalKernelContext::getPSO("queryPointsPairs");  // SAP top phase
        queryPointsGroupedPSO = MetalKernelContext::getPSO("queryPointsGrouped");  // GPU top phase
        clusterAABBPSO = MetalKernelContext::getPSO("cluster_aabb");           // cluster pipeline
        queryPointsPairsBoundedPSO = MetalKernelContext::getPSO("queryPointsPairsBounded");
        queryPointsSegmentedPSO = MetalKernelContext::getPSO("queryPointsSegmented");
        scanReserveSegmentedPSO = MetalKernelContext::getPSO("scanReserveSegmented");
        compactSegmentedPSO     = MetalKernelContext::getPSO("compactSegmented");

        //radixCountBlocksPSO     = MetalKernelContext::getPSO("radixCountMortonBlocks");
        //radixComputeOffsetsPSO  = MetalKernelContext::getPSO("radixComputeOffsets");
        //radixScatterBlocksPSO   = MetalKernelContext::getPSO("radixScatterMortonBlocks");
    }

    void memoryAllocation() {
        Index numPrimitives = primitives.size / PRIMITIVE;
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

        treeVisitCounts      = VectorBase<METAL, uint32_t>(numNodes); // D-029

        // Apetrei agglomerative path. Sized by numInternals = N-1; for
        // N == 1 (single-leaf degenerate) numInternals is 0 and we allocate
        // length 1 to keep VectorBase happy / kernel buffer binding valid.
        Index numInternals = (numPrimitives > 1) ? (numPrimitives - 1) : 1;
        nodeVisitFlags = VectorBase<METAL, uint32_t>(numInternals);
        nodeRangeLeft  = VectorBase<METAL, int>(numInternals);
        nodeRangeRight = VectorBase<METAL, int>(numInternals);
        rootIndexBuf   = VectorBase<METAL, int>(1);
    }

    void build(GeneralMesh<METAL, PR>& mesh) {
        build(mesh.id, mesh.state.x, mesh.adjacency.facets);
    }

    struct RadixSortParamsCPU {
        uint32_t numElements;
        uint32_t shift;
        uint32_t numBlocks;
    };

    // D-029: lock-free single-dispatch bottom-up AABB combine.
    // Zeroes treeVisitCounts, then dispatches bottomUpBoxes over
    // numPrimitives threads (one per leaf). Each thread walks up
    // via treeParent using atomic_fetch_add(acq_rel) + release/
    // acquire fences (see bvh.metal::bottomUpBoxes for the full
    // memory-ordering contract). Replaces the retired multi-pass
    // bottomUpCombineGPU driver (initBottomUpReady +
    // clearBottomUpProgress + bottomUpCombineStep). Used by build()
    // and refit(); enlargeTrajectory() stays on CPU for now.
    //
    // Pre-condition: leaf AABBs already written (buildLeaf*
    // dispatched before this call); treeParent populated (build*
    // dispatched, or carried from prior build for refit).
    //
    // N=1 guard (D-029 fix-turn): the tree's single node IS the
    // leaf-root; buildTree_* never writes treeParent[0] for that
    // case. Skipping the dispatch avoids reading uninitialized
    // parent index.
    void bottomUpBoxesGPU(const AABB4& sceneBox) {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives <= 1) return;
        Index numNodes = 2 * numPrimitives - 1;

        // Zero treeVisitCounts before the bottom-up walk.
        uint32_t numNodesU = (uint32_t)numNodes;
        MetalGlobalContext::setBuffer(treeVisitCounts, 0);
        MetalGlobalContext::setBytes(numNodesU, 1);
        MetalGlobalContext::dispatchThreads(zeroVisitCountsPSO, numNodes);

        // Bottom-up combine. sceneBox._pad0 carries numPrimitives
        // (existing kernel convention; see bvh.metal::bottomUpBoxes).
        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(treeVisitCounts, 6);
        MetalGlobalContext::dispatchThreads(bottomUpBoxesPSO, numPrimitives);
    }

    // D-030: GPU partial-depth variant kept SEPARATE from the D-029
    // path so the original `bottomUpBoxesGPU` remains intact and
    // callable (e.g., by callers that want pure-GPU walk-to-root).
    // The two methods share scaffolding (zero visit-counts + same
    // buffer bindings) but dispatch DIFFERENT kernels and live as
    // parallel symbols rather than a widened single one.
    //
    //   maxDepth >= 1: GPU combines up to `maxDepth` levels from the
    //     leaf side; threads exit BEFORE the atomic at subsequent
    //     levels, so `treeVisitCounts == 2` at the GPU frontier and
    //     `== 0` above it. CPU completion (`bottomUpCombineWithSkip`)
    //     uses `== 2` as the frontier marker.
    //   maxDepth >= log2(N): kernel walks all the way to root and is
    //     functionally equivalent to `bottomUpBoxesGPU`, just with a
    //     redundant per-thread depth counter.
    //   maxDepth == 0: caller should NOT invoke this — route through
    //     `bottomUpCombine` (pure CPU) instead.
    //
    // N=1 guard inherited from `bottomUpBoxesGPU`.
    void bottomUpBoxesPartialGPU(const AABB4& sceneBox, uint maxDepth) {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives <= 1) return;
        Index numNodes = 2 * numPrimitives - 1;

        uint32_t numNodesU = (uint32_t)numNodes;
        MetalGlobalContext::setBuffer(treeVisitCounts, 0);
        MetalGlobalContext::setBytes(numNodesU, 1);
        MetalGlobalContext::dispatchThreads(zeroVisitCountsPSO, numNodes);

        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(treeVisitCounts, 6);
        MetalGlobalContext::setBytes(maxDepth, 7);
        MetalGlobalContext::dispatchThreads(bottomUpBoxesPartialPSO, numPrimitives);
    }

    // D-030: top-down recursive CPU walk that skips subtrees the
    // GPU already combined (frontier marker: treeVisitCounts == 2).
    // Used by `bottomUpHybrid` after the GPU partial-depth dispatch
    // commits. Mirrors `bottomUpCombine`'s shape with the extra
    // skip check; the existing `bottomUpCombine` stays as the
    // pure-CPU path + Block 21's reference.
    //
    // Pre-condition: GPU dispatch has been commitAndWait'd so
    // tree[] and treeVisitCounts are CPU-visible. For pure-CPU
    // mode (maxDepth == 0) the caller goes through `bottomUpCombine`
    // instead — treeVisitCounts is undefined there.
    // Iterative form of the recursive skip-walk (same stack-overflow
    // rationale as bottomUpCombine). A node whose subtree the GPU
    // already combined (treeVisitCounts == 2) is treated as a leaf:
    // not descended and not recombined — identical to the old early
    // `return`.
    void bottomUpCombineWithSkip() {
        if (tree.size == 0) return;
        std::vector<std::pair<int,bool>> stk;
        stk.emplace_back(0, false);
        while (!stk.empty()) {
            auto [id, done] = stk.back();
            stk.pop_back();
            BVHNode& node = tree[id];
            if (node.childA < 0) continue;            // leaf
            if (treeVisitCounts[id] == 2u) continue;  // GPU-combined subtree
            if (!done) {
                stk.emplace_back(id, true);
                stk.emplace_back(node.childA, false);
                stk.emplace_back(node.childB, false);
            } else {
                node.aabb.min = tree[node.childA].aabb.min;
                node.aabb.max = tree[node.childA].aabb.max;
                node.aabb.combine(tree[node.childB].aabb);
            }
        }
    }

    // D-030: hybrid bottom-up driver. GPU partial-depth combine
    // (up to maxDepth levels from leaf) followed by CPU completion
    // for the remaining top-of-tree. maxDepth == 0 routes through
    // the pure-CPU path (`bottomUpCombine`). maxDepth large value
    // recovers D-029's walk-to-root behavior. N=1 guard inherited
    // from `bottomUpBoxesGPU`.
    //
    // Pre-condition: leaf AABBs already written (buildLeaf* /
    // buildTree* dispatched before this call); treeParent populated
    // (build* dispatched, or carried from prior build for refit).
    void bottomUpHybrid(const AABB4& sceneBox, int maxDepth) {
        Index numPrimitives = primitives.size / PRIMITIVE;
        // All paths must end with the encoder flushed so callers
        // (BroadPhase::refit / build / harness) see committed
        // tree.ptr — prior GPU dispatches (buildLeaf*, buildTree*,
        // radix sort, etc.) sit in the encoder when bottomUpHybrid
        // is entered.
        if (numPrimitives <= 1) {
            // Tree's single node IS the leaf-root; nothing to
            // combine. Flush so callers see the committed leaf.
            MetalGlobalContext::commitAndWait();
            return;
        }

        if (maxDepth == 0) {
            // Pure CPU — flush prior dispatches first so
            // bottomUpCombine reads up-to-date tree.ptr.
            MetalGlobalContext::commitAndWait();
            bottomUpCombine();
            return;
        }

        // GPU partial-depth combine; CPU finishes the rest. Uses
        // the D-030 parallel-symbol variant so the D-029 path
        // (`bottomUpBoxesGPU`, walk-to-root) stays unmodified.
        bottomUpBoxesPartialGPU(sceneBox, (uint)maxDepth);
        MetalGlobalContext::commitAndWait();
        bottomUpCombineWithSkip();
    }

    // Apetrei (2014) agglomerative LBVH path. Replaces the
    // `buildTreeGPU` + `bottomUpHybrid` pair in one kernel launch
    // (plus a 1-thread root-swap follow-up). See bvh.metal's
    // `agglomerativeBuild_*` and `agglomerativeSwapRoot` doc-blocks
    // for the algorithm and synchronization details.
    //
    // Pre-condition: `mortons` sorted by code (Stages 1-3 done).
    // Post-condition: `tree[]` + `treeParent[]` fully populated with
    // valid AABBs, root at slot 0, same shape contract as the Karras
    // path so downstream readers (queryPoints, queryClickRay,
    // SCENE-level reads) don't need any change.
    //
    // N == 1 guard mirrors the Karras path: the single leaf at slot 0
    // is itself the root; the kernel writes the leaf AABB and exits
    // early, and `agglomerativeSwapRootGPU` is a no-op for that case.
    void agglomerativeBuildGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;

        Index numInternals = (numPrimitives > 1) ? (numPrimitives - 1) : 1;

        // Zero the visit-flag gate before the single-pass build.
        // Reuses the existing `zeroVisitCounts` kernel — it just
        // zeroes a uint buffer of a caller-given length.
        uint32_t flagCount = (uint32_t)numInternals;
        MetalGlobalContext::setBuffer(nodeVisitFlags, 0);
        MetalGlobalContext::setBytes(flagCount, 1);
        MetalGlobalContext::dispatchThreads(zeroVisitCountsPSO, numInternals);

        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(numPrimitives, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(nodeVisitFlags, 6);
        MetalGlobalContext::setBuffer(nodeRangeLeft, 7);
        MetalGlobalContext::setBuffer(nodeRangeRight, 8);
        MetalGlobalContext::setBuffer(rootIndexBuf, 9);

        MetalGlobalContext::dispatchThreads(agglomerativeBuildPSO, numPrimitives);
    }

    // Relocate the root to slot 0 after `agglomerativeBuildGPU`. Kept
    // as a separate dispatch so cross-dispatch memory visibility is
    // handled by Metal's command-queue barrier; the kernel itself is
    // single-threaded and reads `rootIndexBuf[0]` written by the
    // preceding agglomerative build. No-op if the root already lands
    // at slot 0 (kernel checks internally) or for N <= 1.
    void agglomerativeSwapRootGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives <= 1) return;

        MetalGlobalContext::setBuffer(tree, 0);
        MetalGlobalContext::setBuffer(treeParent, 1);
        MetalGlobalContext::setBuffer(rootIndexBuf, 2);
        MetalGlobalContext::setBytes(numPrimitives, 3);

        // Single-threaded swap — dispatch with 1 thread.
        MetalGlobalContext::dispatchThreads(agglomerativeSwapRootPSO, 1);
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
        sorter.sort(mortons);
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

    // ---- Sub-object (multi-root) LBVH helpers (Phase 1) ----

    // Square-cloth detection: a grid cloth of P×P verts emits N = 2(P-1)²
    // triangles. Recover P from N so the toggle can be pushed to every
    // objTree and non-square / non-triangle meshes self-fall-back to the
    // single-root path. Returns 0 when N is not a square-cloth count.
    int deriveSquareClothP() const {
        if (PRIMITIVE != BVHPRIMITIVE::TRIANGLE) return 0;
        Index N = primitives.size / PRIMITIVE;
        if (N < 2 || (N & 1)) return 0;          // 2 tris per quad
        Index half = N / 2;                      // (P-1)²
        int q = (int)std::lround(std::sqrt((double)half));
        if (q < 1 || (Index)q * q != half) return 0;
        return q + 1;                            // P
    }
    // Phase-1 gate: sub-object path is active only for triangle BVHs over a
    // detectable square cloth with the toggle on and a built grouped PSO.
    bool subObjectActive() const {
        return useSubObjectBVH && buildTreeGroupedPSO
               && (primitives.size / PRIMITIVE) > 1
               && (deriveSquareClothP() >= 2 || useClusterBVH);   // cluster mode: any mesh
    }

    // Assign each triangle prim to a material-space tile group and build the
    // static offset tables. Grid topology (MeshGridInitializer): facets are
    // emitted 2 per quad, quads row-major, so prim t -> quad q=t/2 ->
    // (quadRow, quadCol) = (q/Q, q%Q), Q = P-1. Tile = (quadRow/span,
    // quadCol/span), span = ceil(Q / 2^s); ragged last tile absorbs the
    // remainder. Group windows are packed; empty groups contribute 0 nodes.
    void computeSubObjectGroups() {
        Index N = primitives.size / PRIMITIVE;
        int P = deriveSquareClothP();
        subBvhP = P;
        int Q = P - 1;                       // quads per side
        int T = 1 << subBvhSplitS;           // requested tiles/side (s up to ~16 ok)
        int span = (Q + T - 1) / T;          // ceil(Q/T) = quads per tile
        if (span < 1) span = 1;
        // COMPACT tile grid: the requested 4^s tiles are mostly empty once
        // T > Q (each quad becomes its own tile and the partition saturates at
        // Q²). Index by the EFFECTIVE TR×TR grid instead — TR = ceil(Q/span) ≤
        // Q — so the group tables stay ≤ Q² regardless of s. Without this, s≥14
        // would need multi-GB tables and s=16 overflows 4^s past int32.
        int TR = (Q + span - 1) / span;      // effective tiles per side
        if (TR < 1) TR = 1;
        int k = TR * TR;                     // compact group count, no empties
        numGroups = k;

        groupOfPrim      = VectorBase<METAL, uint32_t>(N);
        sortedPosToGroup = VectorBase<METAL, uint32_t>(N);
        groupSize        = VectorBase<METAL, uint32_t>(k);
        groupPrimBase    = VectorBase<METAL, uint32_t>(k);
        groupNodeBase    = VectorBase<METAL, uint32_t>(k);

        for (int g = 0; g < k; ++g) groupSize[g] = 0u;
        for (Index t = 0; t < N; ++t) {
            Index q = t / 2;
            int tr = (int)(q / Q) / span;    // tr < TR (qr ≤ Q-1)
            int tc = (int)(q % Q) / span;
            int g = tr * TR + tc;
            groupOfPrim[t] = (uint32_t)g;
            groupSize[g]++;
        }
        uint32_t pacc = 0, nacc = 0;
        for (int g = 0; g < k; ++g) {
            groupPrimBase[g] = pacc;
            groupNodeBase[g] = nacc;
            uint32_t M = groupSize[g];       // every compact tile is non-empty
            pacc += M;
            nacc += (M > 0) ? (2u * M - 1u) : 0u;
        }
        subBvhNumNodes = nacc;               // = 2N - k
        subBvhBuiltS = subBvhSplitS;
        for (int g = 0; g < k; ++g) {
            uint32_t base = groupPrimBase[g], M = groupSize[g];
            for (uint32_t i = 0; i < M; ++i) sortedPosToGroup[base + i] = (uint32_t)g;
        }
        buildClusterVertexCSR();             // per-cluster vertex CSR (Phase 4 bidirectional VF)
        std::cout << "[SubObjectBVH] s=" << subBvhSplitS << " tiles/side=" << TR
                  << " groups=" << k << " nodes=" << subBvhNumNodes
                  << " (vs single-root " << (2 * N - 1) << ")" << std::endl;
    }

    // Connectivity-clustering grouping (cluster mode): assign each triangle prim
    // to a connected balanced cluster via the face dual graph (mesh_cluster.hpp),
    // then build the SAME offset tables computeSubObjectGroups() produces (the
    // tables are grouping-source-agnostic). Also records each cluster's
    // single-owned vertex list as a GPU CSR for the bidirectional VF phase.
    // Runs on the CPU once at build time (positions/primitives are CPU-visible);
    // flood fill is sequential, so a one-shot CPU build is the right tool.
    void computeSubObjectGroupsClustered() {
        Index N = primitives.size / PRIMITIVE;
        Index V = positions.size / 3;
        meshcluster::ClusterResult R = meshcluster::clusterMeshDualGraph(
            primitives.ptr, (size_t)N, positions.ptr, (size_t)V, subBvhSplitS);
        int k = R.numClusters;
        numGroups = k;

        groupOfPrim      = VectorBase<METAL, uint32_t>(N);
        sortedPosToGroup = VectorBase<METAL, uint32_t>(N);
        groupSize        = VectorBase<METAL, uint32_t>(k);
        groupPrimBase    = VectorBase<METAL, uint32_t>(k);
        groupNodeBase    = VectorBase<METAL, uint32_t>(k);

        for (int g = 0; g < k; ++g) groupSize[g] = 0u;
        for (Index t = 0; t < N; ++t) {
            uint32_t g = (uint32_t)R.faceCluster[t];
            groupOfPrim[t] = g;
            groupSize[g]++;
        }
        uint32_t pacc = 0, nacc = 0;
        for (int g = 0; g < k; ++g) {
            groupPrimBase[g] = pacc;
            groupNodeBase[g] = nacc;
            uint32_t M = groupSize[g];
            pacc += M;
            nacc += (M > 0) ? (2u * M - 1u) : 0u;
        }
        subBvhNumNodes = nacc;               // = 2N - (#nonempty groups)
        subBvhBuiltS = subBvhSplitS;
        for (int g = 0; g < k; ++g) {
            uint32_t base = groupPrimBase[g], M = groupSize[g];
            for (uint32_t i = 0; i < M; ++i) sortedPosToGroup[base + i] = (uint32_t)g;
        }
        buildClusterVertexCSR();             // per-cluster single-owned vertex CSR (Phase 4)

        uint32_t mnF = N, mxF = 0;
        for (int g = 0; g < k; ++g) { uint32_t M = groupSize[g]; mnF = std::min(mnF, M); mxF = std::max(mxF, M); }
        std::cout << "[ClusterBVH] s=" << subBvhSplitS << " clusters=" << k
                  << " nodes=" << subBvhNumNodes << " faces/cluster=[" << mnF << ',' << mxF
                  << "] ownedV=" << clusterVertOffsets[k]
                  << " (vs single-root " << (2 * N - 1) << ")" << std::endl;
    }

    // Build per-group single-owned vertex CSR (clusterVertOffsets/clusterVerts)
    // from the current groupOfPrim — works for ANY grouping source (cloth grid
    // tile-split OR connectivity clustering). Each USED vertex → the group with
    // the majority of its incident faces (ties → lowest group id), so a vertex is
    // owned by exactly one group. Feeds the bidirectional VF phase: one object's
    // cluster vertices query the other object's cluster triangle subtrees. CPU,
    // once per (re)build (positions/primitives are CPU-visible by then).
    void buildClusterVertexCSR() {
        Index N = primitives.size / PRIMITIVE;
        Index V = positions.size / 3;
        int k = numGroups;
        if (k <= 0) return;
        std::vector<std::unordered_map<int,int>> tally(V);
        for (Index t = 0; t < N; ++t) {
            int g = (int)groupOfPrim[t];
            Index b = t * PRIMITIVE;
            for (int p = 0; p < PRIMITIVE; ++p) tally[primitives[b + p]][g]++;
        }
        std::vector<int> vg(V, -1);
        std::vector<uint32_t> cnt(k, 0);
        for (Index v = 0; v < V; ++v) {
            if (tally[v].empty()) continue;
            int bg = -1, bc = -1;
            for (auto& [g, c] : tally[v]) if (c > bc || (c == bc && g < bg)) { bc = c; bg = g; }
            vg[v] = bg; cnt[bg]++;
        }
        clusterVertOffsets = VectorBase<METAL, uint32_t>((Index)(k + 1));
        clusterVertOffsets[0] = 0;
        for (int g = 0; g < k; ++g) clusterVertOffsets[g+1] = clusterVertOffsets[g] + cnt[g];
        uint32_t tot = clusterVertOffsets[k];
        clusterVerts = VectorBase<METAL, uint32_t>((Index)std::max<uint32_t>(1, tot));
        std::vector<uint32_t> w(k);
        for (int g = 0; g < k; ++g) w[g] = clusterVertOffsets[g];
        for (Index v = 0; v < V; ++v) if (vg[v] >= 0) clusterVerts[w[vg[v]]++] = (uint32_t)v;
    }

    // Stable partition of the Morton-sorted `mortons` so each group's prims
    // are contiguous in [primBase[g], primBase[g]+M_g) while KEEPING their
    // Morton order (LSD radix: sort minor key = Morton, then stable-sort
    // major key = group). CPU because build is rare and unified memory makes
    // the scatter trivial. Pre: `mortons` is Morton-sorted and CPU-visible
    // (caller commitAndWait'd after radixSortGPU).
    void cpuStableGroupPartition() {
        Index N = primitives.size / PRIMITIVE;
        std::vector<uint32_t> off(numGroups);
        for (int g = 0; g < numGroups; ++g) off[g] = groupPrimBase[g];
        for (Index i = 0; i < N; ++i) {
            uint32_t prim = mortons[i].index;
            uint32_t g = groupOfPrim[prim];
            mortonsTemp[off[g]++] = mortons[i];
        }
        std::copy(mortonsTemp.ptr, mortonsTemp.ptr + N, mortons.ptr);
    }

    void buildTreeGroupedGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(numPrimitives, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(sortedPosToGroup, 6);
        MetalGlobalContext::setBuffer(groupSize, 7);
        MetalGlobalContext::setBuffer(groupPrimBase, 8);
        MetalGlobalContext::setBuffer(groupNodeBase, 9);
        MetalGlobalContext::dispatchThreads(buildTreeGroupedPSO, numPrimitives);
    }
    void buildLeafGroupedGPU() {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBytes(numPrimitives, 2);
        MetalGlobalContext::setBuffer(mortons, 3);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(sortedPosToGroup, 6);
        MetalGlobalContext::setBuffer(groupSize, 7);
        MetalGlobalContext::setBuffer(groupPrimBase, 8);
        MetalGlobalContext::setBuffer(groupNodeBase, 9);
        MetalGlobalContext::dispatchThreads(buildLeafGroupedPSO, numPrimitives);
    }
    // GPU enlargeTrajectory leaf pass (single-root). Expands each leaf box by
    // the swept volume; positions+velocities buffers + dt go in, tree updated
    // in place. Caller follows with bottomUpBoxesGPU + commitAndWait.
    void enlargeLeafGPU(PR dt) {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        float dtf = (float)dt;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(velocities, 1);
        MetalGlobalContext::setBuffer(primitives, 2);
        MetalGlobalContext::setBytes(numPrimitives, 3);
        MetalGlobalContext::setBytes(dtf, 4);
        MetalGlobalContext::setBuffer(mortons, 5);
        MetalGlobalContext::setBuffer(tree, 6);
        MetalGlobalContext::dispatchThreads(enlargeLeafPSO, numPrimitives);
    }
    // GPU enlargeTrajectory leaf pass (multi-root). Grouped index buffers at
    // 7..10 mirror buildLeafGroupedGPU; caller follows with
    // bottomUpBoxesMultiRootGPU + commitAndWait.
    void enlargeLeafGroupedGPU(PR dt) {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        float dtf = (float)dt;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(velocities, 1);
        MetalGlobalContext::setBuffer(primitives, 2);
        MetalGlobalContext::setBytes(numPrimitives, 3);
        MetalGlobalContext::setBytes(dtf, 4);
        MetalGlobalContext::setBuffer(mortons, 5);
        MetalGlobalContext::setBuffer(tree, 6);
        MetalGlobalContext::setBuffer(sortedPosToGroup, 7);
        MetalGlobalContext::setBuffer(groupSize, 8);
        MetalGlobalContext::setBuffer(groupPrimBase, 9);
        MetalGlobalContext::setBuffer(groupNodeBase, 10);
        MetalGlobalContext::dispatchThreads(enlargeLeafGroupedPSO, numPrimitives);
    }
    // Fused refit+enlarge leaf pass (single-root): SETS each leaf to the swept
    // hull in one dispatch (no prior refit needed). Same bindings as
    // enlargeLeafGPU; the kernel writes the full leaf instead of expanding.
    void buildSweptLeafGPU(PR dt) {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        float dtf = (float)dt;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(velocities, 1);
        MetalGlobalContext::setBuffer(primitives, 2);
        MetalGlobalContext::setBytes(numPrimitives, 3);
        MetalGlobalContext::setBytes(dtf, 4);
        MetalGlobalContext::setBuffer(mortons, 5);
        MetalGlobalContext::setBuffer(tree, 6);
        MetalGlobalContext::dispatchThreads(buildSweptLeafPSO, numPrimitives);
    }
    void buildSweptLeafGroupedGPU(PR dt) {
        int numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        float dtf = (float)dt;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(velocities, 1);
        MetalGlobalContext::setBuffer(primitives, 2);
        MetalGlobalContext::setBytes(numPrimitives, 3);
        MetalGlobalContext::setBytes(dtf, 4);
        MetalGlobalContext::setBuffer(mortons, 5);
        MetalGlobalContext::setBuffer(tree, 6);
        MetalGlobalContext::setBuffer(sortedPosToGroup, 7);
        MetalGlobalContext::setBuffer(groupSize, 8);
        MetalGlobalContext::setBuffer(groupPrimBase, 9);
        MetalGlobalContext::setBuffer(groupNodeBase, 10);
        MetalGlobalContext::dispatchThreads(buildSweptLeafGroupedPSO, numPrimitives);
    }
    // Per-tree fused refit+enlarge: ONE swept-leaf pass + ONE bottom-up + ONE
    // sync, replacing the refit()+enlargeTrajectory() pair (which each did a
    // leaf pass + a propagate). Topology (treeParent/childA/childB + group
    // partition) carries from build(), same as refit(). Triangle-only; the
    // caller (multi-tree refitSwept) only routes here when the swept PSO loaded.
    void refitSwept(PR dt) {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives == 0) return;
        AABB4 sceneBox; sceneBox.i0 = numPrimitives;
        bool grouped = subObjectActive() && groupOfPrim.size == numPrimitives;
        if (grouped) {
            buildSweptLeafGroupedGPU(dt);
            bottomUpBoxesMultiRootGPU(sceneBox);
        } else {
            buildSweptLeafGPU(dt);
            bottomUpBoxesGPU(sceneBox);
        }
        // None/PerFrame keep the refit dispatches in flight; the combined root
        // boxes have no CPU consumer this substep (detect culls on the GPU).
        if (syncEachPhase) MetalGlobalContext::commitAndWait();
    }
    // Multi-root combine. Mirrors bottomUpBoxesGPU (zero visit-counts +
    // dispatch) but the kernel stops each leaf walk at its group root.
    void bottomUpBoxesMultiRootGPU(const AABB4& sceneBox) {
        Index numPrimitives = primitives.size / PRIMITIVE;
        if (numPrimitives <= 1) return;
        Index numNodes = 2 * numPrimitives - 1;   // zero whole alloc (harmless)

        uint32_t numNodesU = (uint32_t)numNodes;
        MetalGlobalContext::setBuffer(treeVisitCounts, 0);
        MetalGlobalContext::setBytes(numNodesU, 1);
        MetalGlobalContext::dispatchThreads(zeroVisitCountsPSO, numNodes);

        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(treeVisitCounts, 6);
        MetalGlobalContext::setBuffer(sortedPosToGroup, 7);
        MetalGlobalContext::setBuffer(groupSize, 8);
        MetalGlobalContext::setBuffer(groupPrimBase, 9);
        MetalGlobalContext::setBuffer(groupNodeBase, 10);
        MetalGlobalContext::dispatchThreads(bottomUpBoxesMultiRootPSO, numPrimitives);
    }

    // CPU correctness check (no query needed): every prim's AABB must sit
    // inside its group's root AABB, and the union of all group roots must
    // equal the scene box. Call after a sub-object build/refit + commitAndWait.
    // Returns true on pass; logs the first failure.
    bool validateSubObjectBVH() {
        Index N = primitives.size / PRIMITIVE;
        if (!useSubObjectBVH || groupOfPrim.size != N) return false;
        AABB4 unionBox; bool unionInit = false;
        for (int g = 0; g < numGroups; ++g) {
            uint32_t M = groupSize[g];
            if (M == 0) continue;
            BVHNode& root = tree[groupNodeBase[g]];
            // enclosure: each prim of this group inside root
            for (uint32_t i = 0; i < M; ++i) {
                uint32_t sp = groupPrimBase[g] + i;
                uint32_t fid = mortons[sp].index;
                Index base = (Index)fid * PRIMITIVE;
                for (int p = 0; p < PRIMITIVE; ++p) {
                    Index vid = primitives[base + p];
                    tinym::vec3_view v(positions.ptr + vid * 3);
                    for (int c = 0; c < 3; ++c) {
                        if (v[c] < root.min[c] - 1e-4f || v[c] > root.max[c] + 1e-4f) {
                            std::cout << "[SubObjectBVH] VALIDATE FAIL g=" << g
                                      << " prim=" << fid << " axis=" << c
                                      << " v=" << v[c] << " root=[" << root.min[c]
                                      << "," << root.max[c] << "]" << std::endl;
                            return false;
                        }
                    }
                }
            }
            if (!unionInit) { unionBox = root.aabb; unionInit = true; }
            else unionBox.combine(root.aabb);
        }
        std::cout << "[SubObjectBVH] VALIDATE OK — " << numGroups
                  << " groups, union=[" << unionBox.min << " .. "
                  << unionBox.max << "]" << std::endl;
        return true;
    }

    // Object-level AABB for the scene TLAS. Single-root: slot 0. Multi-root:
    // union of the k group roots (slot 0 is only group 0's root, so reading
    // tree[0] alone would under-cull this object in the broad phase). Group
    // roots (slot groupNodeBase[g]) are CPU-visible after the GPU multi-root
    // combine + commitAndWait, so this k-loop runs in the BroadPhase refit's
    // post-sync pass. k is tiny vs the per-substep work.
    AABB4 objectRootAABB() {
        if (!subObjectActive() || groupOfPrim.size != primitives.size / PRIMITIVE)
            return tree[0].aabb;
        AABB4 box = tree[groupNodeBase[0]].aabb;
        for (int g = 1; g < numGroups; ++g)
            if (groupSize[g] > 0) box.combine(tree[groupNodeBase[g]].aabb);
        return box;
    }

    void build(int oid, VectorBase<METAL, PR>& pos, VectorBase<METAL, Index>& prim) {
        //std::cout << "[BVH Build] Memory allocated, BVH build start" << std::endl;
        objid = oid;

        auto* mesh = Scene<METAL, PR>::findById(objid);
        positions = pos;
        primitives = prim;
        objCollider = ColliderKind::Mesh;
        if(mesh) {
            velocities = mesh->state.v;
            objBehavior = mesh->behaviorType;
            objStatic = mesh->isStatic;
            staticCombined = false;   // fresh GPU-combined tree → needs one CPU correction if static
            builtForLifetimeId = mesh->lifetimeId;  // D-026
            // P1: cache the per-mesh COLLIDER kind so the broad phase can
            // route any analytic collider (Sphere/Box/Cylinder/Plane) to
            // the analytic narrow path — skipping the triangle-BVH descent
            // — instead of always treating every object as a triangle soup.
            objCollider = mesh->colliderKind;
        }
        //std::cout << "[BVH Build] positions and primitives are assigned" << std::endl;
        Index numPrimitives = primitives.size/PRIMITIVE;
        // Reallocate when buffers are missing OR sized for a different
        // primitive count. The persisted `tree` buffer carries over the
        // size from the prior build; without the size check, a re-build
        // with a different N writes past the old allocation. This bites
        // the SCENE-level BVH whenever scene.numMeshes changes (e.g.
        // every `Create > Sphere/Cube`).
        Index expectedNumNodes = (numPrimitives > 0) ? (2 * numPrimitives - 1) : 0;
        if (!tree.ptr || tree.size != expectedNumNodes) memoryAllocation();

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

        // [stage 4 + 5] tree build + bottom-up AABB combine.
        //
        // Two paths share stages 1–3 (scene box + Morton + sort) and
        // diverge here. The default Karras path runs `buildTreeGPU`
        // (hierarchy) followed by `bottomUpHybrid` (AABB reduction);
        // the Apetrei agglomerative path collapses both into a single
        // kernel launch plus a 1-thread root-swap. Either side can be
        // deprecated independently by removing its branch.
        if (subObjectActive()) {
            // Sub-object multi-root build. Stable group-partition of the
            // Morton-sorted prims (CPU; build is rare) then grouped Karras
            // hierarchy + multi-root combine. Tables are (re)built lazily
            // when the prim count changes.
            // Cluster mode applies to NON-grid meshes (the Human); a square cloth
            // grid keeps its tile-split even with useClusterBVH propagated on.
            bool clusterMode = useClusterBVH && deriveSquareClothP() < 2;
            if (groupOfPrim.size != numPrimitives || subBvhBuiltS != subBvhSplitS) {
                if (clusterMode) computeSubObjectGroupsClustered();    // connectivity (any mesh)
                else             computeSubObjectGroups();             // cloth grid tile-split
            }
            MetalGlobalContext::commitAndWait();   // Morton sort visible to CPU
            cpuStableGroupPartition();
            buildTreeGroupedGPU();
            bottomUpBoxesMultiRootGPU(sceneBox);
            MetalGlobalContext::commitAndWait();
        } else if (useAgglomerative) {
            agglomerativeBuildGPU();
            agglomerativeSwapRootGPU();
            // **프로파일링 공정성**을 위한 강제 sync — 원래 agglomerative
            // 경로는 commitAndWait 없이 pure-GPU 라 dispatch 만 호스트에서
            // 리턴하고 실제 GPU 워크는 다음 sync(브로드 쿼리 등)에서 흡수돼
            // 이 스코프(`bvh_build` / `broad_refit`)에 계산 비용이 잡히지
            // 않았다 — 다른 케이스(Karras `bottomUpHybrid` 는 CPU 단계에서
            // 암시적 commit) 와 같은 sync 기준으로 측정하기 위해 명시 commit.
            // (전체 파이프라인 효율을 측정하려면 이 줄을 빼야 함.)
            MetalGlobalContext::commitAndWait();
        } else {
            buildTreeGPU();

            // Pure-GPU walk-to-root. 이전엔 `bottomUpHybrid` 의
            // bottomUpBoxesPartialGPU + bottomUpCombineWithSkip(CPU)
            // 조합을 썼는데 depth=30 으로 사실상 풀-GPU 임에도 CPU
            // 마무리 단계의 short-circuit 루프가 substep 누적시
            // stall 을 만들어 매-substep rebuild 가 refit 보다 가벼워지는
            // 측정 아티팩트를 만들었음 (profiles/experiment/bvh-build-modes-fixed-2026-05-26).
            // bottomUpBoxesGPU 직접 호출로 D-029 walk-to-root 경로 복귀.
            // (`bottomUpHybrid` 자체는 bench 에서 depth 스윕에 사용되므로 유지.)
            bottomUpBoxesGPU(sceneBox);
            MetalGlobalContext::commitAndWait();
        }
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

    // Iterative post-order combine. Was a recursive lambda whose depth
    // equaled the BVH height — a large / unbalanced LBVH (big mesh, e.g.
    // a high-tessellation cloth or imported Human.obj) recursed deep
    // enough to overflow the main-thread stack → EXC_BAD_ACCESS. An
    // explicit heap stack removes the depth limit; the (id,childrenDone)
    // marker preserves exact post-order so a parent AABB is combined
    // only after both children are final.
    void bottomUpCombine() {
        if (tree.size == 0) return;
        std::vector<std::pair<int,bool>> stk;
        stk.emplace_back(0, false);
        while (!stk.empty()) {
            auto [id, done] = stk.back();
            stk.pop_back();
            BVHNode& node = tree[id];
            if (node.childA < 0) continue;            // leaf
            if (!done) {
                stk.emplace_back(id, true);
                stk.emplace_back(node.childA, false);
                stk.emplace_back(node.childB, false);
            } else {
                node.aabb.min = tree[node.childA].aabb.min;
                node.aabb.max = tree[node.childA].aabb.max;
                node.aabb.combine(tree[node.childB].aabb);
            }
        }
    }

    void refit() {
        // D-029: GPU refit. Topology (treeParent + childA/B) is unchanged
        // across refits, so reuse the existing tree structure. Leaves
        // are recomputed from current positions via buildLeafGPU;
        // bottomUpBoxesGPU then propagates the updated AABBs to the
        // root.
        //
        // **Substep-loop sync caveat (CM-011).** Adding a commit here
        // forces the outer substep loop's in-flight integrator
        // dispatches to flush before this refit's reads. The OLD CPU
        // refit was implicitly reading positions 2 substeps stale
        // (deferred-commit artifact: last commit was prior substep's
        // narrow-phase commit at line 4209; integrator from substep
        // N-1 sat uncommitted when substep N's refit ran). NEW GPU
        // refit is only 1 substep stale (post-prior-substep-integrator),
        // which is more physically standard but changes when contact
        // response visibility propagates. BDD-010's positive case was
        // historically passing because OLD's deeper lag kept the BVH
        // showing cloth-at-ground even after contact response had
        // pushed cloth above; D-029's mechanization update changes
        // the assertion to "cumNarrow > 0" (any inter-object contact
        // across the frame) which is honest under either lag depth.
        Index numPrimitives = primitives.size/PRIMITIVE;
        if (numPrimitives == 0) return;

        AABB4 sceneBox;  // only sceneBox._pad0 (= numPrimitives) is read by the kernel
        sceneBox.i0 = numPrimitives;

        // Sub-object refit: topology + group partition carried from build();
        // recompute leaf AABBs into the grouped slots, combine into k roots.
        if (subObjectActive() && groupOfPrim.size == numPrimitives) {
            buildLeafGroupedGPU();
            bottomUpBoxesMultiRootGPU(sceneBox);
            // No commitAndWait — only the GPU multi-root combine is dispatched.
            // The caller batches ONE commitAndWait, after which the group roots
            // are CPU-visible for objectRootAABB() and the SAP top phase (see
            // BroadPhase::refit). Batching this sync cut refit ~13% / frame ~19%
            // (vs the old inline) on the sub-object scene.
            return;
        }

        buildLeafGPU();
        // Pure-GPU walk-to-root. build() 의 같은 swap 참고.
        bottomUpBoxesGPU(sceneBox);
        // NOTE: no commitAndWait here. The leaf+combine are GPU-only and the
        // result is read later (the TLAS reads each root AABB on the CPU). The
        // sync is the CALLER's responsibility — it batches ONE commitAndWait
        // after dispatching every object's refit, so N objects share a single
        // GPU round-trip instead of paying N per-substep syncs. Any caller that
        // reads this tree on the CPU (objectRootAABB, the CPU enlarge leaf loop,
        // a timing bench) must commitAndWait first.
    }

    void enlargeTrajectory(PR dt) {
        Index numPrimitives = primitives.size/PRIMITIVE;
        if (numPrimitives == 0) return;

        // GPU path (triangle BVH only): the leaf swept-expand runs as a kernel
        // and the propagate reuses the GPU bottom-up (same as refit), turning
        // the old O(N) CPU leaf loop + CPU bottomUpCombine into one dispatch +
        // one GPU walk + one sync. enlargeLeafPSO is non-null only for TRIANGLE
        // (edge BVHs fall through to the CPU loop).
        //
        // OPT-IN (default OFF = legacy CPU path, the known-good behavior). The
        // GPU path is marginal on its own (the bottom-up dominates, not the
        // leaf loop — use the fused refitSwept toggle for the real win) and was
        // implicated in a GPU-wedge report, so it stays behind YSIM_GPU_ENLARGE
        // until root-caused. Read once via static init.
        static const bool gpuEnlarge =
            (std::getenv("YSIM_GPU_ENLARGE") != nullptr);
        if (gpuEnlarge && enlargeLeafPSO) {
            AABB4 sceneBox; sceneBox.i0 = numPrimitives;
            bool grouped = subObjectActive() && groupOfPrim.size == numPrimitives;
            if (grouped) {
                enlargeLeafGroupedGPU(dt);
                bottomUpBoxesMultiRootGPU(sceneBox);
            } else {
                enlargeLeafGPU(dt);
                bottomUpBoxesGPU(sceneBox);
            }
            // No commitAndWait here — the GPU combine is only dispatched. The
            // caller batches ONE commitAndWait (group roots then CPU-visible for
            // objectRootAABB / SAP; see BroadPhase::enlargeTrajectory).
            return;
        }

        // Multi-root: leaves live in per-group windows (NOT [N-1, 2N-2]), so
        // enlarge the grouped leaf slots and propagate via the multi-root
        // combine. Reading tree[N+i-1] here would hit internal/unused slots
        // and dereference a garbage childB.
        if (subObjectActive() && groupOfPrim.size == numPrimitives) {
            for (Index sp = 0; sp < numPrimitives; ++sp) {
                uint32_t g = sortedPosToGroup[sp];
                uint32_t M = groupSize[g];
                uint32_t leafSlot = groupNodeBase[g] + (M - 1) + (sp - groupPrimBase[g]);
                BVHNode& t = tree[leafSlot];
                Index pbase = (Index)t.childB * PRIMITIVE;
                for (Index p = 0; p < PRIMITIVE; ++p) {
                    Index posid = primitives[pbase+p];
                    tinym::vec3_view pos (positions.ptr + posid*3);
                    tinym::vec3_view v (velocities.ptr + posid*3);
                    t.min = tinym::min(t.min, pos+v*dt);
                    t.max = tinym::max(t.max, pos+v*dt);
                }
            }
            AABB4 sceneBox; sceneBox.i0 = numPrimitives;
            bottomUpBoxesMultiRootGPU(sceneBox);
            // No commitAndWait here — only the GPU multi-root combine is
            // dispatched. The caller batches ONE commitAndWait (group roots then
            // CPU-visible for objectRootAABB / SAP; see BroadPhase). This is the
            // per-substep sync that made sub-object enlarge +67%; batching it cut
            // frame time ~11% (vs the old inline sync) on the sub-object scene.
            return;
        }
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

    // One-time CPU re-combine for a static tree (see `staticCombined`). build()
    // and refit() combine the tree on the GPU (`bottomUpBoxesGPU`), which can
    // leave a large tree incompletely combined — root AABB minY pinned at the
    // init 0 instead of the true geometry, so the broad-phase traversal prunes
    // wrong and under-detects contacts (cloth tunnels through a static collider).
    // The per-substep CPU enlarge masks this for dynamic meshes; a static mesh
    // skips it, so correct the tree ONCE here on the CPU, then it stays frozen.
    // Reuses enlargeTrajectory's CPU bottom-up combine (dt=0 ⇒ pure re-combine,
    // velocity term vanishes — a static mesh has zero velocity anyway). A leading
    // sync makes the GPU-written leaf AABBs visible to the CPU combine.
    void combineStaticOnce() {
        if (primitives.size/PRIMITIVE == 0) return;
        MetalGlobalContext::commitAndWait();
        enlargeTrajectory(0);
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
        uint32_t numNodes;   // queried tree's node count → traversal index bound
        uint32_t entryRoot;  // single-root entry slot (0); repurposed by SAP/GPU kernels
    };

    void queryBegin() {
        qFlag[0].stackOverflow = 0;
        qFlag[0].collisionOverflow = 0;
    }
    // D-041 turn-3 (2026-05-14): qObjId / tObjId in QueryPointsParams (which
    // the kernel writes into broadCollisions.objPair) are mesh INDICES
    // into Scene::meshes, NOT mesh.id. The narrow kernel uses them as
    // scenePackedPositionsOffsets[] subscripts; that array is indexed by
    // position (0..numMeshes-1). Callers pass `qIndex` (their loop counter
    // in BroadPhase::detectCollisions). The target index lives on the
    // tree itself as `this->objIndex` (also set by BroadPhase::build).
    // Pre-D-041, mesh.id == array index always, so the prior signature
    // worked by coincidence; after D-041 turn-2 decoupled id from index
    // the prior signature produced empty narrow-phase output after any
    // mesh removal (a stable mesh's id ≠ its new index).
    void queryPoints(Index qIndex, PR queryMargin) {
        if (qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos = qmesh.state.x;
        Index qnumPoints = qpos.size/3;
        //auto& constraints = qmesh->constraints;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol = Scene<METAL, PR>::packedCollisionData;
        // Sub-object top phase (no mini-TLAS): a grouped query culls point→group
        // via a flat top phase, then descends only the overlapping subtrees.
        // subTopMode 0 = CPU SAP (default), 1 = GPU brute. Single-root meshes
        // (non-grouped) fall through to the slot-0 descent below.
        const bool grouped = subObjectActive()
                          && groupOfPrim.size == primitives.size / PRIMITIVE;
        if (grouped) {
            // None/PerFrame (!syncEachPhase) force GPU-brute: the CPU-SAP top
            // phase reads roots + positions on the CPU and would force a sync.
            if ((subTopMode == 1 || !syncEachPhase) && queryPointsGroupedPSO)
                queryPointsGPUTop(qIndex, queryMargin);
            else
                queryPointsSAP(qIndex, queryMargin);
            return;
        }
        QueryPointsParams qParams = {
            queryMargin, qnumPoints, qIndex, (Index)objIndex, packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior, (Index)qmesh.colliderKind, (Index)objCollider,
            (uint32_t)tree.size,
            0u   // single-root: enter at slot 0
        };

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

        // D-041 turn-3: self-pair uses this tree's INDEX (not mesh.id).
        queryPoints(objIndex, queryMargin);
    }

    // SAP top phase (experiment). CPU sweep-and-prune over the k group root
    // boxes vs the qnumPoints query boxes emits candidate (point, groupRoot)
    // pairs; the GPU then descends one group subtree per pair. Replaces the
    // mini-TLAS GPU descent (Phase 2b). Pre: refit committed (group root boxes
    // CPU-visible) and the querying mesh's positions are current on the CPU.
    void queryPointsSAP(Index qIndex, PR queryMargin) {
        if (qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        Index qnumPoints = qpos.size / 3;
        if (qnumPoints == 0 || numGroups <= 0) return;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;

        const float m = (float)queryMargin;
        const float* xp = qpos.ptr;   // unified memory: CPU-visible positions

        // Sort point ids by x.min (= x - m, monotone in x ⇒ sort by x).
        // The order is RETAINED per query mesh: frame-to-frame (and substep-to-
        // substep) points drift only a little, so last call's order is nearly
        // sorted. First call / size change → identity + full sort; otherwise
        // insertion sort the previous order = O(n + inversions), ~O(n) on the
        // coherent case (a static query mesh is already exactly sorted) vs
        // std::sort's O(n log n) every call.
        std::vector<uint32_t>& sapOrder = sapOrderByQ[qIndex];
        if ((Index)sapOrder.size() != qnumPoints) {
            sapOrder.resize((size_t)qnumPoints);
            for (Index i = 0; i < qnumPoints; ++i) sapOrder[i] = (uint32_t)i;
            std::sort(sapOrder.begin(), sapOrder.end(),
                      [xp](uint32_t a, uint32_t b) { return xp[3*a] < xp[3*b]; });
        } else {
            for (Index i = 1; i < qnumPoints; ++i) {
                uint32_t v = sapOrder[i];
                float vk = xp[3*v];
                Index j = i;
                while (j > 0 && xp[3*sapOrder[j-1]] > vk) { sapOrder[j] = sapOrder[j-1]; --j; }
                sapOrder[j] = v;
            }
        }

        // Per group: X-slice of points whose box overlaps the group box on X.
        // Point box = [x-m, x+m] (constant width 2m) ⇒ overlap on X iff
        //   x - m <= gxmax  &&  x + m >= gxmin  ⇔  gxmin-2m <= (x-m) <= gxmax
        // i.e. a contiguous run in sapOrder (sorted by x). Within it, confirm
        // Y/Z overlap, then emit (pointId, groupRoot).
        auto xkey = [xp](uint32_t id) { return xp[3*id]; };  // sort key (x)
        std::vector<SAPPair>& out = sapBuild_;
        out.clear();
        for (int g = 0; g < numGroups; ++g) {
            if (groupSize[g] == 0) continue;
            const BVHNode& gb = tree[groupNodeBase[g]];
            const float gxmin = gb.min.x, gxmax = gb.max.x;
            const float gymin = gb.min.y, gymax = gb.max.y;
            const float gzmin = gb.min.z, gzmax = gb.max.z;
            // run = [lo, hi) over sapOrder where x in [gxmin - m, gxmax + m]
            // (x-m <= gxmax ⇒ x <= gxmax+m ; x+m >= gxmin ⇒ x >= gxmin-m).
            auto lo = std::lower_bound(sapOrder.begin(), sapOrder.end(), gxmin - m,
                          [&](uint32_t id, float v) { return xkey(id) < v; });
            auto hi = std::upper_bound(sapOrder.begin(), sapOrder.end(), gxmax + m,
                          [&](float v, uint32_t id) { return v < xkey(id); });
            const uint32_t root = groupNodeBase[g];
            for (auto it = lo; it != hi; ++it) {
                uint32_t id = *it;
                float y = xp[3*id+1], z = xp[3*id+2];
                if (y + m < gymin || y - m > gymax) continue;
                if (z + m < gzmin || z - m > gzmax) continue;
                out.push_back({ id, root });
            }
        }

        const uint32_t numPairs = (uint32_t)out.size();
        if (numPairs == 0) return;

        // Grow the GPU pair buffer geometrically; refill each call.
        if (numPairs > sapPairsCap) {
            sapPairsCap = numPairs + numPairs / 2 + 64;
            sapPairs = VectorBase<METAL, SAPPair>(sapPairsCap);
        }
        std::copy(out.begin(), out.end(), sapPairs.ptr);

        QueryPointsParams qParams = {
            queryMargin, numPairs, (Index)qIndex, (Index)objIndex,
            packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior,
            (Index)qmesh.colliderKind, (Index)objCollider,
            (uint32_t)tree.size, 0u  // entryRoot unused (per-pair)
        };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::setBuffer(sapPairs, 7);
        MetalGlobalContext::dispatchThreads(queryPointsPairsPSO, numPairs);
    }

    // Phase 4: dispatch a SUPPLIED set of (point, entryRoot) pairs through the
    // existing per-pair descent kernel (queryPointsPairs) — same machinery as
    // queryPointsSAP's tail, but the pairs come from the cluster-pair top phase
    // (this object is the TARGET tree; the pairs carry the querying mesh's point
    // ids + this tree's cluster subtree roots) rather than CPU-SAP. qIndex is the
    // querying mesh index (point positions read from its state.x).
    void dispatchPointPairs(Index qIndex, PR queryMargin, const std::vector<SAPPair>& pairs) {
        if (pairs.empty() || qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;
        uint32_t numPairs = (uint32_t)pairs.size();
        if (numPairs > sapPairsCap) {
            sapPairsCap = numPairs + numPairs / 2 + 64;
            sapPairs = VectorBase<METAL, SAPPair>(sapPairsCap);
        }
        std::copy(pairs.begin(), pairs.end(), sapPairs.ptr);
        QueryPointsParams qParams = {
            queryMargin, numPairs, (Index)qIndex, (Index)objIndex,
            packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior,
            (Index)qmesh.colliderKind, (Index)objCollider,
            (uint32_t)tree.size, 0u
        };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::setBuffer(sapPairs, 7);
        MetalGlobalContext::dispatchThreads(queryPointsPairsPSO, numPairs);
    }

    // Cluster pipeline (GPU): compute per-cluster AABB (tight box over current
    // face verts ∪ swept group root) into `out` (6 floats/cluster). One thread per
    // cluster over its Morton-partitioned prim slice. Pre: grouped tree built/refit.
    void computeClusterAABBGPU(VectorBase<METAL, float>& out) {
        if (numGroups <= 0 || !clusterAABBPSO) return;
        if (out.size != (Index)(6 * numGroups)) out = VectorBase<METAL, float>(6 * numGroups);
        uint32_t k = (uint32_t)numGroups;
        MetalGlobalContext::setBuffer(positions, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(mortons, 2);
        MetalGlobalContext::setBuffer(groupPrimBase, 3);
        MetalGlobalContext::setBuffer(groupSize, 4);
        MetalGlobalContext::setBuffer(tree, 5);
        MetalGlobalContext::setBuffer(groupNodeBase, 6);
        MetalGlobalContext::setBuffer(out, 7);
        MetalGlobalContext::setBytes(k, 8);
        MetalGlobalContext::dispatchThreads(clusterAABBPSO, k);
    }

    // Like dispatchPointPairs but the SAPPair list is already a GPU buffer (from
    // the cluster-pair expansion kernel); `count` is the CPU-read pair count.
    void dispatchPointPairsGPU(Index qIndex, PR queryMargin,
                               VectorBase<METAL, SAPPair>& sapBuf, uint32_t count) {
        if (count == 0 || qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;
        QueryPointsParams qParams = {
            queryMargin, count, (Index)qIndex, (Index)objIndex, packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior,
            (Index)qmesh.colliderKind, (Index)objCollider, (uint32_t)tree.size, 0u };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::setBuffer(sapBuf, 7);
        MetalGlobalContext::dispatchThreads(queryPointsPairsPSO, count);
    }

    // Async per-pair descent: the pair count is a GPU buffer (countBuf), so we
    // over-dispatch `maxThreads` (the SAPPair buffer capacity) and each thread
    // bounds itself — no CPU readback ⇒ no extra sync in the live loop.
    void dispatchPointPairsGPUBounded(Index qIndex, PR queryMargin,
                                      VectorBase<METAL, SAPPair>& sapBuf,
                                      VectorBase<METAL, uint32_t>& countBuf, uint32_t maxThreads) {
        if (maxThreads == 0 || qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;
        QueryPointsParams qParams = {
            queryMargin, maxThreads, (Index)qIndex, (Index)objIndex, packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior,
            (Index)qmesh.colliderKind, (Index)objCollider, (uint32_t)tree.size, 0u };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::setBuffer(sapBuf, 7);
        MetalGlobalContext::setBuffer(countBuf, 8);
        MetalGlobalContext::dispatchThreads(queryPointsPairsBoundedPSO, maxThreads);
    }

    // GPU top phase (mode 2). One thread per query point brute-tests the k
    // group root boxes on the GPU and descends overlapping subtrees inline —
    // no CPU sort, no pair buffer (vs queryPointsSAP). qParams.entryRoot is
    // repurposed as the group count; groupNodeBase[] holds the k root slots.
    void queryPointsGPUTop(Index qIndex, PR queryMargin) {
        if (qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        Index qnumPoints = qpos.size / 3;
        if (qnumPoints == 0 || numGroups <= 0) return;
        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;

        QueryPointsParams qParams = {
            queryMargin, (uint32_t)qnumPoints, (Index)qIndex, (Index)objIndex,
            packedCol.maxNumCollisions,
            (Index)qmesh.behaviorType, (Index)objBehavior,
            (Index)qmesh.colliderKind, (Index)objCollider,
            (uint32_t)tree.size, (uint32_t)numGroups  // entryRoot repurposed = group count
        };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::setBuffer(groupNodeBase, 7);
        MetalGlobalContext::dispatchThreads(queryPointsGroupedPSO, qnumPoints);
    }

    // ===== Segmented (per-threadgroup) detect+reduce variant =====
    // Paper-inspired alternative to `queryPoints` above. See bvh.metal
    // doc-block (queryPointsSegmented) for the algorithmic motivation:
    // replaces per-leaf-hit device-global atomicAdd with a threadgroup-
    // local atomic + per-TG private slice in device memory, then a
    // single device atomic to reserve the dispatch's `globalBase`
    // during the reduce step.
    //
    // Layout-compatible host struct for QuerySegParams in bvh.metal —
    // same 9 32-bit fields, only field [4] differs in meaning (perTGCap
    // vs maxNumCollisions). Kept as a separate type to make the wiring
    // intent obvious at call sites and to avoid silently reusing the
    // baseline QueryPointsParams.
    struct QuerySegParams {
        float    queryMargin;
        uint32_t numPoints;
        uint32_t qObjId;
        uint32_t tObjId;
        uint32_t perTGCap;
        uint32_t qBehavior;
        uint32_t tBehavior;
        uint32_t qShape;
        uint32_t tShape;
        uint32_t numNodes;   // queried tree's node count → traversal index bound
        uint32_t entryRoot;  // single-root entry slot (0); repurposed by SAP/GPU kernels
    };
    struct SegScanParams {
        uint32_t numTGs;
        uint32_t maxNumCollisions;
    };
    struct SegCompactParams {
        uint32_t numTGs;
        uint32_t perTGCap;
        uint32_t tgSize;
        uint32_t maxNumCollisions;
    };

    void queryPointsSegmented(Index qIndex, PR queryMargin) {
        if (qIndex < 0 || qIndex >= (Index)Scene<METAL, PR>::meshes.size()) return;
        auto& qmesh = Scene<METAL, PR>::meshes[qIndex];
        auto& qpos  = qmesh.state.x;
        Index qnumPoints = qpos.size / 3;
        if (qnumPoints == 0) return;

        typename Scene<METAL, PR>::PackedCollisionData& packedCol =
            Scene<METAL, PR>::packedCollisionData;

        const uint32_t tgSize   = (uint32_t)packedCol.segTGSize;
        const uint32_t perTGCap = (uint32_t)packedCol.segPerTGCap;
        const uint32_t numTGs   = ((uint32_t)qnumPoints + tgSize - 1) / tgSize;
        if (numTGs > (uint32_t)packedCol.segMaxTGs) {
            std::cerr << "[queryPointsSegmented] numTGs=" << numTGs
                      << " exceeds segMaxTGs=" << packedCol.segMaxTGs
                      << " (skip; re-pack needed)\n";
            return;
        }

        // Sub-object has no mini-TLAS to descend, so a grouped query can't use
        // the segmented single-tree pipeline. Delegate to the flat top phase
        // (SAP/GPU), which writes straight to the global broadCollisions (the
        // same counter the segmented compact biases from) and skips the
        // per-TG reduce/compact below.
        const bool grouped = subObjectActive()
                          && groupOfPrim.size == primitives.size / PRIMITIVE;
        if (grouped) {
            // None/PerFrame (!syncEachPhase) force GPU-brute: the CPU-SAP top
            // phase reads roots + positions on the CPU and would force a sync.
            if ((subTopMode == 1 || !syncEachPhase) && queryPointsGroupedPSO)
                queryPointsGPUTop(qIndex, queryMargin);
            else
                queryPointsSAP(qIndex, queryMargin);
            return;
        }
        // (1) detection: per-TG private writes via threadgroup atomics.
        QuerySegParams qParams = {
            (float)queryMargin, (uint32_t)qnumPoints,
            (uint32_t)qIndex, (uint32_t)objIndex,
            perTGCap,
            (uint32_t)qmesh.behaviorType, (uint32_t)objBehavior,
            (uint32_t)qmesh.colliderKind, (uint32_t)objCollider,
            (uint32_t)tree.size,
            0u   // single-root: enter at slot 0
        };
        MetalGlobalContext::setBuffer(qpos, 0);
        MetalGlobalContext::setBuffer(primitives, 1);
        MetalGlobalContext::setBuffer(tree, 2);
        MetalGlobalContext::setBytes(qParams, 3);
        MetalGlobalContext::setBuffer(packedCol.segPrivateCollisions, 4);
        MetalGlobalContext::setBuffer(packedCol.segPrivateCount, 5);
        MetalGlobalContext::setBuffer(qFlag, 6);
        MetalGlobalContext::dispatchThreads(
            queryPointsSegmentedPSO, numTGs * tgSize, tgSize);

        // (2) reduce: serial exclusive scan of per-TG counts + one
        // device atomic on the shared numBroadCollisions for globalBase.
        SegScanParams scanParams = { numTGs, (uint32_t)packedCol.maxNumCollisions };
        MetalGlobalContext::setBuffer(packedCol.segPrivateCount, 0);
        MetalGlobalContext::setBuffer(packedCol.segPrivateOffset, 1);
        MetalGlobalContext::setBuffer(packedCol.numBroadCollisions, 2);
        MetalGlobalContext::setBytes(scanParams, 3);
        MetalGlobalContext::setBuffer(qFlag, 4);
        MetalGlobalContext::dispatchThreads(scanReserveSegmentedPSO, 1);

        // (3) compact: scatter per-TG slices into the global
        // broadCollisions at biased offsets.
        SegCompactParams cmpParams = {
            numTGs, perTGCap, tgSize, (uint32_t)packedCol.maxNumCollisions };
        MetalGlobalContext::setBuffer(packedCol.segPrivateCollisions, 0);
        MetalGlobalContext::setBuffer(packedCol.segPrivateCount, 1);
        MetalGlobalContext::setBuffer(packedCol.segPrivateOffset, 2);
        MetalGlobalContext::setBytes(cmpParams, 3);
        MetalGlobalContext::setBuffer(packedCol.broadCollisions, 4);
        MetalGlobalContext::dispatchThreads(
            compactSegmentedPSO, numTGs * tgSize, tgSize);
    }
    void checkSelfCollisionsSegmented(PR queryMargin) {
        queryPointsSegmented(objIndex, queryMargin);
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

        if(node.childA < 0) { // leaf — childB is the primitive id (not a node).
            // D-024: write the actual ray-vs-triangle hit, not the leaf
            // AABB intersect. A rotated mesh's leaf AABB can extend far
            // beyond the triangle itself (the AABB grows with the
            // triangle's diagonal extent under rotation); writing AABB-
            // tmin would let a tilted Plane envelop the camera frustum
            // and steal every click. The smallest-tmin walk on the
            // consumer side (production callback at ~src/main.cpp:6996,
            // harness pickClosest in Block 14) ranks by the values we
            // write here, so triangle-precision must land at the leaf.
            Index triId = static_cast<Index>(node.childB);
            // PRIMITIVE = 3 (triangle) for TRI_LBVH; primitives.ptr holds
            // 3*M facet indices, positions.ptr holds 3*N float coords.
            Index v0 = primitives.ptr[3 * triId + 0];
            Index v1 = primitives.ptr[3 * triId + 1];
            Index v2 = primitives.ptr[3 * triId + 2];
            tinym::vec3 p0(positions.ptr[3*v0+0],
                           positions.ptr[3*v0+1],
                           positions.ptr[3*v0+2]);
            tinym::vec3 p1(positions.ptr[3*v1+0],
                           positions.ptr[3*v1+1],
                           positions.ptr[3*v1+2]);
            tinym::vec3 p2(positions.ptr[3*v2+0],
                           positions.ptr[3*v2+1],
                           positions.ptr[3*v2+2]);
            float triT;
            if (!rayTriangleIntersect(ray, p0, p1, p2, triT)) return;

            auto& rayTracedData = Scene<BE, PR>::rayTracedData;
            auto& rayTraced = rayTracedData.clickRayCollisions;
            auto& numTraced = rayTracedData.numClickRayCollisions;
            if(numTraced[0] >= rayTracedData.approxColsPerRay) return;
            rayTraced[numTraced[0]] = {
                (Index)objid, triId, triT, triT};
            numTraced[0]++;
            return; // D-020: must not recurse from a leaf — childA/childB
                    // here are NOT child node indices (childA == -1 marks
                    // leaf, childB is the primitive id).
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


template <typename BE, typename PR>
struct BVH<BE, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT> {
    using TRI_LBVH = BVH<BE, PR, BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE>;
    using EDGE_LBVH = BVH<BE, PR, BVHMODE::LINEAR, BVHPRIMITIVE::EDGE>;
    std::vector<TRI_LBVH> objTrees;

    // BVH for each object's BV
    VectorBase<METAL, PR> positions;
    VectorBase<METAL, Index> indices;
    EDGE_LBVH tree;

    // SCENE-level A/B 토글. Simulator::useAgglomerativeBVH가 매 빌드/리핏
    // 직전에 이 값을 덮어쓰고, 여기에서 per-mesh TRI_LBVH와 top-level EDGE_LBVH
    // 모두에게 푸시한다. 기본 false = 기존 Karras 경로 그대로 유지.
    bool useAgglomerative = false;

    // Refit ON/OFF 토글. 기본 true = 기존 동작(증분 AABB 갱신).
    // false 일 때 refit() 호출이 자동으로 build()로 폴백되어 토폴로지부터
    // 다시 만든다 — 큰 변형 후 트리 품질 회복용 비교 모드.
    bool enableRefit = true;

    // Fused refit+enlarge A/B toggle (default OFF = legacy two-pass:
    // refit() then enlargeTrajectory()). When ON the substep loop calls
    // refitSwept() once — a single swept-leaf pass + one bottom-up + one TLAS
    // build, dropping the separate enlarge pass. Triangle-only (per-tree
    // refitSwept routes to GPU swept kernels); other primitives keep two-pass.
    bool fusedRefitEnlarge = false;

    // Sub-object (multi-root) LBVH A/B toggle — Phase 1 divergence
    // experiment. Pushed to every per-mesh TRI_LBVH; only triangle BVHs over
    // a detectable square cloth actually switch (subObjectActive()), so
    // spheres/cubes/edge-TLAS self-fall-back. Object-level AABB for the scene
    // TLAS comes from objectRootAABB() (union of group roots). Default false.
    bool useSubObjectBVH = false;
    int  subBvhSplitS = 1;       // s: k = 4^s groups
    bool validateSubObject = false;  // one-shot CPU correctness check on build
    // Connectivity clustering for NON-grid meshes (the static Human): when ON,
    // every tree gets useClusterBVH; grid cloth ignores it (keeps tile-split),
    // non-grid meshes group by face dual-graph flood fill. Default false ⇒
    // existing sub-object paths bit-identical. Pushed to every per-mesh tree.
    bool clusterNonGridBVH = false;

    // Cluster VF pipeline (live): cluster-pair broad (grid over cluster AABBs) +
    // bidirectional per-pair VF, replacing the per-(q,t) full broad phase. Two-mesh
    // only: cloth=objTrees[0] (grouped), static cluster Human=objTrees[1]. Default
    // OFF ⇒ detectCollisionsTwoMesh path unchanged.
    bool clusterVFPipeline = false;
    VectorBase<METAL, float>    clothClusterAABB;      // [6*kc] recomputed each frame (GPU)
    VectorBase<METAL, float>    humanClusterAABB;      // [6*ks] static, built once (CPU→GPU)
    VectorBase<METAL, uint32_t> gridCellStart, gridCellClusters;  // human cell CSR (static)
    ClusterGridParams           clusterGrid{};
    bool clusterGridBuilt = false;
    VectorBase<METAL, ClusterPair>       clusterPairBuf;  VectorBase<METAL, uint32_t> clusterPairCount;
    uint32_t clusterPairCap = 0;
    VectorBase<METAL, typename TRI_LBVH::SAPPair> clusterDirA, clusterDirB;
    VectorBase<METAL, uint32_t>          clusterDirACount, clusterDirBCount;
    uint32_t clusterDirCapA = 0, clusterDirCapB = 0;
    // Sub-object top-phase mode (with useSubObjectBVH): 0 CPU SAP (default),
    // 1 GPU brute. Pushed to every per-mesh tree. See BVH::subTopMode.
    int subTopMode = 0;

    // Two-mesh experiment (human + cloth only). When ON: (1) the scene-level
    // TLAS (`tree`, the EDGE_LBVH over object AABBs) is NOT built/refit — the
    // broad phase is a direct pairwise objectRootAABB overlap test, which is
    // all detectCollisions ever uses anyway; and (2) detectCollisionsTwoMesh
    // queries BIDIRECTIONALLY (a Float mesh — the static Human — is not
    // skipped as a query source). Set by --bench-twomesh. Default false ⇒
    // every existing path is bit-identical.
    bool twoMeshExperiment = false;

    // update() sync refactor: when true (ProfileLevel::InFrame) each broad
    // phase commits+waits per substep and the CPU-side overflow/flag reads
    // run; when false (None/PerFrame) the broad phase stays async and those
    // CPU reads are skipped. Set by Simulator::update each frame. Default
    // true ⇒ historical per-section-sync behavior.
    bool syncEachPhase = true;

    // P2 (collider_pipeline_rework.md §5-P2, decision D1): when true, a mesh
    // whose colliderKind is analytic gets NO per-substep triangle-BVH work
    // (refit / enlargeTrajectory / refitSwept) and its object-level AABB is
    // computed from the SHAPE PARAMS instead of the tree. Set per frame by
    // Simulator::update from analyticColliderActive(), so it is on exactly
    // when the broad phase emits analytic markers — under SH / ML-SH / the
    // two-mesh experiment the colliders fall back to the triangle path and
    // therefore still need their trees refit (member stays false).
    //
    // Deliberately NOT applied to build(): objTrees[i] is what
    // BroadPhase::queryClickRay and showBox() read, and Simulator rebuilds
    // every 10 frames precisely for them ("BVH is always rebuilt — click-ray
    // and showBox/showSceneBox use it"). Production picking uses the GL
    // id-buffer pass, but the harness ray-picks primitives through
    // queryClickRay (BDD-017 uses two cubes = Box colliders), so the built
    // tree stays. Colliders are rigid, so a tree refreshed every 10 frames is
    // exact for them; the per-substep refit is the pure waste P2 removes.
    bool analyticBroadSkip = false;
    // Broad-side inflation band for the analytic swept AABB: radius +
    // thickness, matching what narrow_pt_analytic gates on. Pushed by
    // Simulator::update (radius + margin) like syncEachPhase.
    PR analyticInflate = PR(0.03);

    // ---- P2/T2: analytic object-level AABB (no triangle BVH involved) ----

    // Compact AnalyticShape slot owning mesh ARRAY INDEX `objIndex`, or
    // nullptr. numAnalytic is tiny (a handful of props), so a linear scan
    // beats a map — same idiom as BruteForce::narrowAnalytic.
    static const AnalyticShape* analyticShapeOf(Index objIndex) {
        if (Scene<METAL, PR>::numAnalytic == 0
            || !Scene<METAL, PR>::meshAnalytic.ptr) return nullptr;
        for (Index k = 0; k < Scene<METAL, PR>::numAnalytic; ++k)
            if (Scene<METAL, PR>::meshAnalytic[k].objIndex == (uint32_t)objIndex)
                return &Scene<METAL, PR>::meshAnalytic[k];
        return nullptr;
    }

    // World half-extents of the collider's bounding box at orientation `qv`
    // (AnalyticShape quat layout: (w, x, y, z) in .x/.y/.z/.w).
    //   Sphere/Ellipsoid — conservative isotropic bound (max semi-axis), so
    //                      the box is rotation-invariant and never too small.
    //   Box              — exact OBB→AABB via the |R|·h trick.
    //   Cylinder         — the OBB (r, halfHeight, r) through the same trick.
    static tinym::vec3 analyticWorldHalf(const AnalyticShape& s,
                                         const tinym::vec4& qv) {
        tinym::vec3 h(0.0f, 0.0f, 0.0f);
        switch ((ColliderKind)s.kind) {
            case ColliderKind::Sphere: {
                const float r = std::max(std::max(s.halfExtHeight.x,
                                                  s.halfExtHeight.y),
                                         s.halfExtHeight.z);
                return tinym::vec3(r, r, r);   // rotation-invariant bound
            }
            case ColliderKind::Box:
                h = tinym::vec3(s.halfExtHeight.x, s.halfExtHeight.y,
                                s.halfExtHeight.z);
                break;
            case ColliderKind::Cylinder:
                h = tinym::vec3(s.centerRadius.w, s.halfExtHeight.w,
                                s.centerRadius.w);
                break;
            default:
                return h;   // Plane / Mesh: no finite extent (handled above)
        }
        Quat q; q.w = qv.x; q.x = qv.y; q.y = qv.z; q.z = qv.w;
        const tinym::vec3 cx = rotateVector(q, tinym::vec3(1.0f, 0.0f, 0.0f));
        const tinym::vec3 cy = rotateVector(q, tinym::vec3(0.0f, 1.0f, 0.0f));
        const tinym::vec3 cz = rotateVector(q, tinym::vec3(0.0f, 0.0f, 1.0f));
        return tinym::vec3(
            std::fabs(cx.x)*h.x + std::fabs(cy.x)*h.y + std::fabs(cz.x)*h.z,
            std::fabs(cx.y)*h.x + std::fabs(cy.y)*h.y + std::fabs(cz.y)*h.z,
            std::fabs(cx.z)*h.x + std::fabs(cy.z)*h.y + std::fabs(cz.z)*h.z);
    }

    // Rotation-linearization band (§3): ‖Δθ‖ · r_max between the prev and
    // current orientations. Mirrors BruteForce::analyticRotMargin's intent
    // on the broad side; 0 for a non-rotating collider.
    static float analyticRotBand(const AnalyticShape& s) {
        const tinym::vec4& a = s.rotQuat;
        const tinym::vec4& b = s.prevRotQuat;
        double dot = (double)a.x*b.x + (double)a.y*b.y
                   + (double)a.z*b.z + (double)a.w*b.w;
        dot = std::fabs(dot);
        if (dot > 1.0) dot = 1.0;
        const double theta = 2.0 * std::acos(dot);
        if (theta <= 0.0) return 0.0f;
        const double rMax = std::sqrt(
            (double)s.halfExtHeight.x*s.halfExtHeight.x
          + (double)s.halfExtHeight.y*s.halfExtHeight.y
          + (double)s.halfExtHeight.z*s.halfExtHeight.z)
          + (double)s.centerRadius.w;
        return (float)(theta * rMax);
    }

    // Object-level broad AABB for mesh index `i`.
    //
    // Returns FALSE when the object has no finite bounding box — i.e. a
    // Plane collider, which is an infinite half-space (§1). Callers must
    // then treat every pair involving it as OVERLAPPING; `out` is filled
    // with a degenerate point box at the collider origin so the scene TLAS
    // input stays finite/well-formed (the TLAS is display/click-ray only —
    // the broad pair test below is a direct pairwise overlap).
    //
    // Otherwise: the analytic swept box AABB(prev pose) ∪ AABB(cur pose)
    // inflated by (radius + thickness + rotation margin), or — for a Mesh
    // collider, or whenever the broad skip is off — the tree's root AABB.
    bool objectBroadAABB(Index i, AABB4& out) {
        const AnalyticShape* s =
            (analyticBroadSkip && i < (Index)objTrees.size()
             && isAnalyticCollider(objTrees[i].objCollider))
            ? analyticShapeOf(i) : nullptr;
        if (!s) { out = objTrees[i].objectRootAABB(); return true; }

        const tinym::vec3 c(s->centerRadius.x, s->centerRadius.y,
                            s->centerRadius.z);
        if ((ColliderKind)s->kind == ColliderKind::Plane) {
            out.min = c; out.max = c; out.i0 = 0; out.i1 = 0;
            return false;   // infinite half-space: no finite box exists
        }
        const tinym::vec3 p(s->prevCenterPad.x, s->prevCenterPad.y,
                            s->prevCenterPad.z);
        const tinym::vec3 hc = analyticWorldHalf(*s, s->rotQuat);
        const tinym::vec3 hp = analyticWorldHalf(*s, s->prevRotQuat);
        const float pad = (float)analyticInflate + analyticRotBand(*s);
        const tinym::vec3 padv(pad, pad, pad);
        out.min = tinym::min(c - hc, p - hp) - padv;
        out.max = tinym::max(c + hc, p + hp) + padv;
        out.i0  = 0; out.i1 = 0;
        return true;
    }

    // P2/T1 predicate: this mesh's triangle BVH is dead this frame.
    bool analyticSkip(Index i) const {
        return analyticBroadSkip && i < (Index)objTrees.size()
            && isAnalyticCollider(objTrees[i].objCollider);
    }

    // P2/T3: `collidable == false` drops the mesh from the BVH broad phase
    // entirely — as query, as target, and as a self-collision source. The
    // analytic side keeps its own gate (AnalyticShape::flags bit0), so a
    // marker that somehow survives still no-ops in the kernel.
    static bool objCollidable(Index i) {
        auto& ms = Scene<METAL, PR>::meshes;
        return i < (Index)ms.size() ? ms[i].collidable : true;
    }

    //BVH(SceneObject<METAL, PR>& scene)
    //    : objTrees(scene.numMeshes), positions(scene.numMeshes*3), indices(scene.numMeshes*2) {}

    void build(Scene<METAL, PR>& scene) {
        // allocations
        if(objTrees.size() != scene.numMeshes) {
            objTrees = std::vector<TRI_LBVH>(scene.numMeshes);
            positions = VectorBase<METAL, PR>(scene.numMeshes*6);
            indices = VectorBase<METAL, Index>(scene.numMeshes*2);
        }

        for(Index i = 0; i < scene.numMeshes; ++i) {
            // D-026: gate the Float-mesh skip on lifetime identity in
            // addition to behavior. Without the lifetimeId clause, a
            // resetScene + new addCube×N at the same numMeshes count
            // silently reuses the prior block's stale tree because
            // both old and new are Float — see CM-008 (graduated).
            if(objTrees[i].tree.ptr
               && objTrees[i].objBehavior == BehaviorType::Float
               && objTrees[i].builtForLifetimeId == scene.meshes[i].lifetimeId) {
                // D-041 turn-3: even on skip, refresh objIndex — the same
                // mesh.id can sit at a different array INDEX after a
                // removeMesh + add. Index is the kernel's offsets[] key.
                objTrees[i].objIndex = (int)i;
                continue;
            }
            objTrees[i].useAgglomerative = useAgglomerative;
            objTrees[i].useSubObjectBVH = useSubObjectBVH;
            objTrees[i].subTopMode = subTopMode;
            // Per-object split s: each mesh divides to suit its own size (the
            // GUI sets GeneralMesh::clusterSplitS per object). -1 = inherit the
            // global subBvhSplitS (headless benches set the global, leave the
            // per-object sentinel, and keep their requested s).
            objTrees[i].subBvhSplitS = scene.meshes[i].clusterSplitS >= 1
                                     ? scene.meshes[i].clusterSplitS : subBvhSplitS;
            objTrees[i].useClusterBVH = clusterNonGridBVH;   // non-grid → connectivity cluster
            objTrees[i].build(scene.meshes[i]);
            objTrees[i].objIndex = (int)i;  // D-041 turn-3
            if (validateSubObject && objTrees[i].subObjectActive())
                objTrees[i].validateSubObjectBVH();
            Index pbase = i*6;
            AABB4 objBox; objectBroadAABB(i, objBox);   // P2/T2 for analytic
            positions[pbase  ] = objBox.min.x;
            positions[pbase+1] = objBox.min.y;
            positions[pbase+2] = objBox.min.z;
            positions[pbase+3] = objBox.max.x;
            positions[pbase+4] = objBox.max.y;
            positions[pbase+5] = objBox.max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        if (!twoMeshExperiment && syncEachPhase) {
            tree.useAgglomerative = useAgglomerative;
            tree.build(-1, positions, indices);
        }
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
        // Refit OFF → full rebuild. Scene<>의 모든 필드가 inline static
        // 이므로 더미 인스턴스로 build() 본체를 재실행해도 비용이 없다.
        if (!enableRefit) {
            Scene<METAL, PR> sceneRef;
            build(sceneRef);
            return;
        }
        // Pass 1 — dispatch each dynamic object's refit (GPU leaf+combine, no
        // per-object sync now that refit() doesn't commit) so all objects'
        // GPU work is in flight, then a SINGLE commitAndWait below batches the
        // N round-trips into one. Static objects reuse the cached tree (one CPU
        // correction via combineStaticOnce, which syncs itself).
        for(Index i = 0; i < objTrees.size(); ++i) {
            // P2/T1: an analytic collider's triangle BVH is never descended
            // (the broad phase emits a marker instead), so refitting it is
            // pure waste. Skip the whole per-substep tree pass; the object
            // AABB comes from the shape params in Pass 2. The slot is NOT
            // compacted and the tree is NOT freed (D1) — array index ==
            // objPair == statesOffsets subscript must keep holding, and
            // build() keeps the tree valid for click-ray. combineStaticOnce
            // is skipped with it: nothing reads this tree's AABBs any more.
            if (analyticSkip(i)) continue;
            // Static 메시는 리프 AABB가 불변 → 무거운 GPU refit을 건너뛰고
            // build() 때 쓴 캐시 트리/positions를 그대로 재사용한다. TLAS
            // (tree.build 아래)에는 계속 참여 → 충돌 타깃으로 유효.
            if (!objTrees[i].objStatic) {
                objTrees[i].useAgglomerative = useAgglomerative;
                objTrees[i].useSubObjectBVH = useSubObjectBVH;
                objTrees[i].subTopMode = subTopMode;
                objTrees[i].subBvhSplitS = subBvhSplitS;
                objTrees[i].refit();
                objTrees[i].staticCombined = false;  // dynamic frame re-breaks the GPU combine
            } else if (!objTrees[i].staticCombined) {
                // Static skip is unsafe until the GPU-built tree is corrected
                // once on the CPU (else the broad phase under-detects and the
                // cloth tunnels through this collider). Correct once, then skip.
                objTrees[i].combineStaticOnce();
                objTrees[i].staticCombined = true;
            }
        }
        // External sync (moved out of refit()): make every dispatched leaf+combine
        // visible before the CPU reads each object root AABB below. ONE round-trip
        // for all dynamic objects — measured ~34% faster than the old per-object
        // commit (refit 58.9→38.9 ms, frame 190→169 ms on the static-Human scene),
        // even with a single dynamic mesh. (Grouped/static objects sync inline.)
        MetalGlobalContext::commitAndWait();
        // Pass 2 — read each object's now-synced root AABB into the TLAS input.
        // Grouped trees: objectRootAABB() unions the k group roots on the CPU
        // (visible after the batched sync above). Skipped in the two-mesh
        // experiment — no scene-level TLAS, broad phase reads roots directly.
        if (!twoMeshExperiment) {
        for(Index i = 0; i < objTrees.size(); ++i) {
            Index pbase = i*6;
            AABB4 objBox; objectBroadAABB(i, objBox);   // P2/T2 for analytic
            positions[pbase  ] = objBox.min.x;
            positions[pbase+1] = objBox.min.y;
            positions[pbase+2] = objBox.min.z;
            positions[pbase+3] = objBox.max.x;
            positions[pbase+4] = objBox.max.y;
            positions[pbase+5] = objBox.max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        tree.useAgglomerative = useAgglomerative;
        tree.build(-1, positions, indices);
        }
        //    std::cout << "[tree root before scene build] scene tree "
        //      << " min=" << tree.tree[0].min
        //      << " max=" << tree.tree[0].max << std::endl;
    }

    void enlargeTrajectory(PR dt) {
        // Pass 1 — dispatch each dynamic object's enlarge. Grouped (sub-object)
        // and the opt-in GPU path only DISPATCH a GPU combine now (no inline
        // commit/topCombine — that per-substep sync is what made sub-object
        // enlarge +67%); the regular path is pure CPU and needs no sync at all.
        static const bool gpuEnlarge = std::getenv("YSIM_GPU_ENLARGE") != nullptr;
        bool needSync = gpuEnlarge;   // GPU path syncs even when non-grouped
        for(Index i = 0; i < objTrees.size(); ++i) {
            // P2/T1: analytic collider ⇒ dead triangle BVH, skip (see refit).
            if (analyticSkip(i)) continue;
            // Static 메시는 속도 0 → 궤적 확장이 no-op이므로 GPU dispatch 생략.
            if (!objTrees[i].objStatic) {
                objTrees[i].useAgglomerative = useAgglomerative;
                objTrees[i].useSubObjectBVH = useSubObjectBVH;
                objTrees[i].subTopMode = subTopMode;
                objTrees[i].subBvhSplitS = subBvhSplitS;
                objTrees[i].enlargeTrajectory(dt);
                needSync |= objTrees[i].subObjectActive();   // grouped → GPU combine pending
                objTrees[i].staticCombined = false;
            } else if (!objTrees[i].staticCombined) {
                objTrees[i].combineStaticOnce();   // one-time CPU correction (see refit())
                objTrees[i].staticCombined = true;
            }
        }
        // External sync (moved out of enlargeTrajectory): one round-trip for all
        // dispatched grouped/GPU combines. Skipped entirely when every dynamic
        // object took the pure-CPU path → the regular scene pays NO new sync.
        if (needSync) MetalGlobalContext::commitAndWait();
        // Pass 2 — read each root AABB into the TLAS (grouped: objectRootAABB()
        // unions the k group roots on the CPU, visible after the sync above).
        // Skipped in the two-mesh experiment (no scene-level TLAS).
        if (!twoMeshExperiment) {
        for(Index i = 0; i < objTrees.size(); ++i) {
            Index pbase = i*6;
            AABB4 objBox; objectBroadAABB(i, objBox);   // P2/T2 for analytic
            positions[pbase  ] = objBox.min.x;
            positions[pbase+1] = objBox.min.y;
            positions[pbase+2] = objBox.min.z;
            positions[pbase+3] = objBox.max.x;
            positions[pbase+4] = objBox.max.y;
            positions[pbase+5] = objBox.max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
        }

        tree.useAgglomerative = useAgglomerative;
        tree.build(-1, positions, indices);
        }
    }

    // Authoring-path refit (D-023): ALSO refreshes the analytic colliders'
    // triangle trees, which the per-substep refit() skips (P2/T1). Those
    // trees are what BroadPhase::queryClickRay and showBox() read, so an
    // inspector translate / rotate / scale must not leave a Sphere or Cube
    // collider's tree parked on the old pose — FR-003 / FR-004 assert exactly
    // that. Rare (one call per edit), so the extra refit costs nothing.
    void refitIncludingAnalytic() {
        const bool saved = analyticBroadSkip;
        analyticBroadSkip = false;
        refit();
        analyticBroadSkip = saved;
    }

    // Fused refit+enlarge (NEW, parallel to refit()+enlargeTrajectory()). One
    // per-tree swept pass that both refits AND inflates by velocity, then one
    // TLAS build — replacing the two-pass sequence's two objTree passes + two
    // TLAS builds. enableRefit==false falls back to a full rebuild like
    // refit(). A per-tree without the swept PSO (edge BVH) self-falls-back to
    // refit()+enlargeTrajectory() so correctness is preserved for all shapes.
    void refitSwept(PR dt) {
        if (!enableRefit) {
            Scene<METAL, PR> sceneRef;
            build(sceneRef);
            return;
        }
        for(Index i = 0; i < objTrees.size(); ++i) {
            // P2/T1: analytic collider ⇒ dead triangle BVH, skip the swept
            // pass (see refit). NOT a `continue` — this loop also writes the
            // TLAS input below, which the analytic path still needs.
            const bool anaSkip = analyticSkip(i);
            // Static 메시: swept refit+enlarge 전체 생략 (AABB 불변, 속도 0).
            if (anaSkip) {
                // nothing to refit
            } else if (!objTrees[i].objStatic) {
                objTrees[i].useAgglomerative = useAgglomerative;
                objTrees[i].useSubObjectBVH = useSubObjectBVH;
                objTrees[i].subTopMode = subTopMode;
                objTrees[i].subBvhSplitS = subBvhSplitS;
                objTrees[i].syncEachPhase = syncEachPhase;   // gate the refit sync
                if (objTrees[i].buildSweptLeafPSO) {
                    objTrees[i].refitSwept(dt);
                } else {
                    objTrees[i].refit();
                    // refit() no longer commits; enlargeTrajectory's CPU leaf
                    // loop reads the GPU-written leaf AABBs, so sync between.
                    MetalGlobalContext::commitAndWait();
                    objTrees[i].enlargeTrajectory(dt);
                }
                objTrees[i].staticCombined = false;
            } else if (!objTrees[i].staticCombined) {
                objTrees[i].combineStaticOnce();   // one-time CPU correction (see refit())
                objTrees[i].staticCombined = true;
            }
            // Skip the scene-level TLAS when it has no consumer this frame: the
            // experiment never builds it, and None/PerFrame (!syncEachPhase)
            // can't afford its objectRootAABB() CPU read (it's dead in the
            // query path anyway — detect culls pairwise / GPU-brute). The
            // frame%10 build() still constructs it for click-ray.
            if (!twoMeshExperiment && syncEachPhase) {
            Index pbase = i*6;
            AABB4 objBox; objectBroadAABB(i, objBox);   // P2/T2 for analytic
            positions[pbase  ] = objBox.min.x;
            positions[pbase+1] = objBox.min.y;
            positions[pbase+2] = objBox.min.z;
            positions[pbase+3] = objBox.max.x;
            positions[pbase+4] = objBox.max.y;
            positions[pbase+5] = objBox.max.z;
            Index ibase = i*2;
            indices[ibase  ] = ibase;
            indices[ibase+1] = ibase+1;
            }
        }
        if (!twoMeshExperiment && syncEachPhase) {
            tree.useAgglomerative = useAgglomerative;
            tree.build(-1, positions, indices);
        }
    }

    void checkSelfCollisions(PR margin) {
        for(Index i = 0; i < objTrees.size(); ++i) {
            auto& tree = objTrees[i];
            if(tree.objBehavior == BehaviorType::Float
            || tree.objBehavior == BehaviorType::Kinematic) continue;
            if(!objCollidable(i)) continue;   // P2/T3
            if(analyticSkip(i)) continue;     // P2/T1: no live tree to query
            tree.checkSelfCollisions(margin);
        }
    }

    // `resetCounter == false`: reset the per-tree overflow diagnostics but
    // LEAVE numBroadCollisions[0] alone, so this detect appends to rows a
    // previous producer already wrote (the useCpuShSelf hybrid). See
    // detectCollisions' comment for why the CPU producer must go first.
    void queryBegin(bool resetCounter = true) {
        if (syncEachPhase) {
            for(auto& tree : objTrees) {
                tree.qFlag[0].stackOverflow = 0;
                tree.qFlag[0].collisionOverflow = 0;
            }
            if (resetCounter)
                Scene<METAL, PR>::packedCollisionData.numBroadCollisions[0] = 0;
        } else if (resetCounter) {
            // Async (None/PerFrame): the broad-pair counter MUST be reset on
            // the GPU, encoded into the stream — a CPU write here would not
            // order with the in-flight detect dispatches, so all 60 substeps'
            // resets would land before the GPU runs any query and the pairs
            // would accumulate across the whole frame. (qFlag is never read
            // off InFrame, so its reset is skipped.)
            static MTL::ComputePipelineState* resetPSO =
                MetalKernelContext::getPSO("reset_counter");
            MetalGlobalContext::setBuffer(
                Scene<METAL, PR>::packedCollisionData.numBroadCollisions, 0);
            MetalGlobalContext::dispatchThreads(resetPSO, 1);
        }
    }
    // `resetCounter == false` keeps queryBegin()'s counter reset OFF, i.e.
    // this detect APPENDS to whatever rows numBroadCollisions[0] already
    // holds instead of starting from 0. Sole consumer: the CPU-hash SELF hybrid
    // (Simulator::useCpuShSelf), which writes its host-side self rows FIRST
    // and then lets these GPU query kernels' device atomics continue from
    // that count. Order matters — a CPU append AFTER this call would race the
    // still-pending GPU atomics under the async tiers. Default true ⇒ byte-
    // identical behavior for every existing caller.
    void detectCollisions(PR margin, bool enableSelfCollisions=true,
                          bool analyticEnabled=false,
                          bool resetCounter=true) {
        queryBegin(resetCounter);
        // Push the sync tier so grouped queries pick GPU-brute off InFrame.
        for (auto& tr : objTrees) tr.syncEachPhase = syncEachPhase;
        // Slice (c-2): fresh analytic markers each broad detect.
        Scene<METAL, PR>::packedCollisionData.beginAnalyticPairs();

        // Bidirectional registration. The old `std::set<IndexPair> checked`
        // deduped on the SORTED pair {min,max}, so a colliding pair {A,B}
        // was queried in ONE direction only — points from whichever mesh
        // the outer loop reached first (== the lower mesh index == the
        // EARLIER-created mesh). narrowAndSortByVertices keys collision
        // response solely off objPair.query, so the other mesh (target /
        // facet provider) was detected-but-never-responded. That made
        // collision response depend on creation order (import-then-plane
        // vs plane-then-import). The double loop already visits every
        // ordered (q,t) exactly once, so simply NOT deduping yields both
        // directions when both meshes are non-Float; a Float mesh is
        // still skipped as a query (line above) so Float/non-Float pairs
        // keep their single valid direction — no extra work, no change.
        for(Index q = 0; q < objTrees.size(); ++q) {
            auto& queryTree = objTrees[q];
            if(queryTree.objBehavior == BehaviorType::Float
            || queryTree.objBehavior == BehaviorType::Kinematic) continue;
            // P2/T3: collidable == false leaves the broad phase entirely —
            // as query, as target, and as a self-collision source.
            if(!objCollidable(q)) continue;

            for(Index t = 0; t < objTrees.size(); ++t) {
                if(q == t) {
                    if(!enableSelfCollisions) continue;
                    if(analyticSkip(q)) continue;   // P2/T1: no live tree
                    queryTree.checkSelfCollisions(margin);
                    continue;
                }
                if(!objCollidable(t)) continue;   // P2/T3
                // Phase 2: union of k group roots, not slot 0 (= group 0
                // only), else multi-root objects under-cull and miss pairs.
                // None/PerFrame (!syncEachPhase) skip this CPU cull —
                // objectRootAABB() reads the GPU-built tree nodes on the CPU,
                // which would force a sync; the GPU query kernel culls per
                // node internally, so always dispatch.
                bool hit = true;
                if (syncEachPhase) {
                    // P2/T2: an analytic collider's box comes from the shape
                    // params (its triangle BVH is dead). A Plane returns
                    // false = "infinite half-space, no finite box" and the
                    // pair is then ALWAYS treated as overlapping — one cheap
                    // analytic dispatch per cloth per substep, and the fix
                    // for P1's "Plane only acts where the source grid's AABB
                    // reaches".
                    AABB4 qa, ta;
                    const bool qFinite = objectBroadAABB(q, qa);
                    const bool tFinite = objectBroadAABB(t, ta);
                    hit = (!qFinite || !tFinite) ? true : ta.intersect(qa);
                }
                if(hit) {
                    // P1 (decision 2): when EITHER side is an analytic
                    // collider (colliderKind != Mesh), DON'T descend its
                    // triangle BVH — that traversal + the vertex×triangle
                    // BroadCollisions it emits are pure waste because
                    // narrow_pt_tri drops every analytic row anyway
                    // (skipAnalytic). Instead record one analytic marker, and
                    // only for the (cloth query → analytic target) direction:
                    // the reciprocal ordered visit (analytic q → cloth t) is
                    // the SAME object pair, so marking once avoids a double
                    // dispatch. analytic→analytic is dropped outright (§2:
                    // Bullet owns rigid↔rigid).
                    //
                    // §4: BOTH marker fields are ARRAY INDICES (q, t), the
                    // same namespace queryPoints puts in objPair.
                    if (analyticEnabled
                        && (isAnalyticCollider(objTrees[t].objCollider)
                         || isAnalyticCollider(queryTree.objCollider))) {
                        bool qCloth =
                            queryTree.objBehavior == BehaviorType::TriangularCloth
                         || queryTree.objBehavior == BehaviorType::FastGridCloth;
                        if (isAnalyticCollider(objTrees[t].objCollider) && qCloth)
                            Scene<METAL, PR>::packedCollisionData
                                .pushAnalyticPair(q, t);
                    } else {
                        // D-041 turn-3: pass query INDEX (q, not queryTree.objid)
                        // so broadCollisions.objPair stores indices that the
                        // narrow kernel can directly use as statesOffsets[]
                        // subscripts.
                        objTrees[t].queryPoints(q, margin);
                    }
                }
            }
        }
        // Publish the deduped fresh-union-held analytic marker set
        // (PackedCollisionData::endAnalyticPairs explains the carry-over).
        Scene<METAL, PR>::packedCollisionData.endAnalyticPairs();
        queryEnd();
    }

    // Segmented (per-threadgroup) variant: same broad-phase loop shape
    // as detectCollisions above, but each per-tree dispatch goes through
    // queryPointsSegmented (paper Alg.2 stream registration). Kept as a
    // SEPARATE method so the baseline path stays bench-compatible and
    // can be deprecated cleanly once the segmented path is proven.
    // `resetCounter`: same append semantics as detectCollisions above.
    void detectCollisionsSegmented(PR margin, bool enableSelfCollisions=true,
                                   bool analyticEnabled=false,
                                   bool resetCounter=true) {
        queryBegin(resetCounter);
        // Slice (c-2): fresh analytic markers each broad detect.
        Scene<METAL, PR>::packedCollisionData.beginAnalyticPairs();

        std::set<IndexPair> checked;
        for(Index q = 0; q < objTrees.size(); ++q) {
            auto& queryTree = objTrees[q];
            if(queryTree.objBehavior == BehaviorType::Float
            || queryTree.objBehavior == BehaviorType::Kinematic) continue;
            if(!objCollidable(q)) continue;   // P2/T3

            for(Index t = 0; t < objTrees.size(); ++t) {
                Index a = std::min(q, t);
                Index b = std::max(q, t);
                if(q == t) {
                    if(!enableSelfCollisions) continue;
                    if(analyticSkip(q)) continue;   // P2/T1: no live tree
                    queryTree.checkSelfCollisionsSegmented(margin);
                    checked.insert({a, b});
                    continue;
                }
                if(!objCollidable(t)) continue;   // P2/T3
                if(checked.find({a, b}) != checked.end()) continue;
                // Phase 2: union of k group roots (see detectCollisions).
                // P2/T2: analytic colliders answer from shape params; a
                // Plane has no finite box ⇒ the pair always overlaps.
                AABB4 qa, ta;
                const bool qFinite = objectBroadAABB(q, qa);
                const bool tFinite = objectBroadAABB(t, ta);
                if(!qFinite || !tFinite || ta.intersect(qa)) {
                    // P1: mirror detectCollisions — skip the analytic
                    // collider's triangle-BVH descent, record a marker
                    // instead. Unlike the non-segmented path this loop DEDUPS
                    // each unordered pair (`checked`), so it visits a (cloth,
                    // collider) pair in only ONE ordering — handle BOTH so the
                    // marker doesn't depend on which index is smaller.
                    // Marker fields are ARRAY INDICES (§4).
                    bool tAnalytic = isAnalyticCollider(objTrees[t].objCollider);
                    bool qAnalytic = isAnalyticCollider(queryTree.objCollider);
                    if (analyticEnabled && (tAnalytic || qAnalytic)) {
                        auto isCloth = [](BehaviorType bt) {
                            return bt == BehaviorType::TriangularCloth
                                || bt == BehaviorType::FastGridCloth;
                        };
                        auto& pc = Scene<METAL, PR>::packedCollisionData;
                        if (tAnalytic && isCloth(queryTree.objBehavior))
                            pc.pushAnalyticPair(q, t);
                        else if (qAnalytic && isCloth(objTrees[t].objBehavior))
                            pc.pushAnalyticPair(t, q);
                        // analytic↔analytic: skip (§2, Bullet owns rigid↔rigid)
                    } else {
                        objTrees[t].queryPointsSegmented(q, margin);
                    }
                    checked.insert({a, b});
                }
            }
        }
        // Publish the deduped fresh-union-held analytic marker set
        // (PackedCollisionData::endAnalyticPairs explains the carry-over).
        Scene<METAL, PR>::packedCollisionData.endAnalyticPairs();
        queryEnd();
    }

    // Two-mesh experiment broad phase (human + cloth). No scene-level TLAS —
    // a direct pairwise objectRootAABB overlap test (exactly what
    // detectCollisions already culls with). BIDIRECTIONAL: unlike
    // detectCollisions, a Float/Kinematic mesh is NOT skipped as a query
    // source, so when the two meshes' root AABBs overlap we run BOTH ordered
    // directions — (q,t) visits objTrees[t].queryPoints(q), so cloth→human
    // (cloth points vs Human tris) AND human→cloth (Human points vs cloth
    // tris) are both emitted. The static Human contributes contact DATA in its
    // direction without responding (Float meshes don't integrate). Grouped
    // (sub-object) cloth trees route through queryPoints' grouped top phase.
    void detectCollisionsTwoMesh(PR margin, bool enableSelfCollisions=false) {
        queryBegin();
        // Push the sync tier so grouped queries pick GPU-brute off InFrame.
        for (auto& tr : objTrees) tr.syncEachPhase = syncEachPhase;
        Scene<METAL, PR>::packedCollisionData.clearAnalyticPairs();
        for(Index q = 0; q < objTrees.size(); ++q) {
            auto& queryTree = objTrees[q];
            if(!objCollidable(q)) continue;   // P2/T3
            for(Index t = 0; t < objTrees.size(); ++t) {
                if(q == t) {
                    if(enableSelfCollisions) queryTree.checkSelfCollisions(margin);
                    continue;
                }
                if(!objCollidable(t)) continue;   // P2/T3
                // None/PerFrame skip the CPU root-AABB cull (objectRootAABB
                // reads GPU tree nodes on the CPU → would force a sync); the
                // GPU query kernel culls per node, so always dispatch.
                bool hit = true;
                if (syncEachPhase) {
                    AABB4 qa = queryTree.objectRootAABB();
                    AABB4 ta = objTrees[t].objectRootAABB();
                    hit = ta.intersect(qa);
                }
                if(hit) objTrees[t].queryPoints(q, margin);
            }
        }
        queryEnd();
    }

    // Static Human cluster grid: insert each Human cluster's AABB (tight ∪ swept
    // root, CPU union gather) into uniform grid cells (CSR). cell = max cluster
    // extent. Built ONCE (Human static); also uploads the Human cluster AABBs to
    // GPU for the pair-query overlap test.
    void buildHumanClusterGrid(TRI_LBVH& human, PR margin) {
        int ks = human.numGroups; if (ks <= 0) return;
        std::vector<float> hA(6 * ks);
        for (int g = 0; g < ks; ++g) { for (int c=0;c<3;++c){ hA[6*g+c]=1e30f; hA[6*g+3+c]=-1e30f; } }
        Index N = human.primitives.size / 3;
        for (Index t = 0; t < N; ++t) { int g = (int)human.groupOfPrim[t];
            for (int p = 0; p < 3; ++p) { uint32_t v = human.primitives[t*3+p];
                const float* q = human.positions.ptr + 3*v;
                for (int c=0;c<3;++c){ hA[6*g+c]=std::min(hA[6*g+c],q[c]); hA[6*g+3+c]=std::max(hA[6*g+3+c],q[c]); } } }
        for (int g = 0; g < ks; ++g) { auto& r = human.tree[human.groupNodeBase[g]];
            for (int c=0;c<3;++c){ hA[6*g+c]=std::min(hA[6*g+c],(float)r.min[c]); hA[6*g+3+c]=std::max(hA[6*g+3+c],(float)r.max[c]); } }

        float bmin[3]={1e30f,1e30f,1e30f}, bmax[3]={-1e30f,-1e30f,-1e30f}, maxExt=0.0f;
        for (int g=0; g<ks; ++g) for (int c=0;c<3;++c){ bmin[c]=std::min(bmin[c],hA[6*g+c]); bmax[c]=std::max(bmax[c],hA[6*g+3+c]); maxExt=std::max(maxExt,hA[6*g+3+c]-hA[6*g+c]); }
        float cell = maxExt > 1e-6f ? maxExt : 1.0f;
        // Floor the cell so the grid resolution stays sane: at very high s the
        // clusters are ~single triangles, so cell→tiny would explode dims/numCells.
        int dims[3]; size_t numCells = 0;
        for (int it = 0; it < 64; ++it) {
            size_t nc = 1; for (int c=0;c<3;++c){ dims[c]=std::max(1,(int)std::ceil((bmax[c]-bmin[c])/cell)); nc*=(size_t)dims[c]; }
            numCells = nc; if (nc <= (size_t(1)<<20)) break; cell *= 1.6f;   // cap ~1M cells
        }
        auto cellRange = [&](int g, int lo[3], int hi[3]) { for (int c=0;c<3;++c){
            lo[c]=std::min(std::max(0,(int)std::floor((hA[6*g+c]-bmin[c])/cell)),dims[c]-1);
            hi[c]=std::min(std::max(0,(int)std::floor((hA[6*g+3+c]-bmin[c])/cell)),dims[c]-1); } };
        std::vector<uint32_t> cs(numCells+1, 0);
        for (int g=0; g<ks; ++g){ int lo[3],hi[3]; cellRange(g,lo,hi);
            for (int z=lo[2];z<=hi[2];++z) for (int y=lo[1];y<=hi[1];++y) for (int x=lo[0];x<=hi[0];++x) cs[(size_t)((z*dims[1]+y)*dims[0]+x)+1]++; }
        for (size_t i=0;i<numCells;++i) cs[i+1]+=cs[i];
        std::vector<uint32_t> cc(cs[numCells], 0);
        { std::vector<uint32_t> w(cs.begin(), cs.end()-1);
          for (int g=0; g<ks; ++g){ int lo[3],hi[3]; cellRange(g,lo,hi);
            for (int z=lo[2];z<=hi[2];++z) for (int y=lo[1];y<=hi[1];++y) for (int x=lo[0];x<=hi[0];++x) cc[w[(size_t)((z*dims[1]+y)*dims[0]+x)]++]=(uint32_t)g; } }

        gridCellStart = VectorBase<METAL, uint32_t>((Index)cs.size()); std::copy(cs.begin(), cs.end(), gridCellStart.ptr);
        gridCellClusters = VectorBase<METAL, uint32_t>((Index)std::max<size_t>(1, cc.size())); std::copy(cc.begin(), cc.end(), gridCellClusters.ptr);
        humanClusterAABB = VectorBase<METAL, float>((Index)hA.size()); std::copy(hA.begin(), hA.end(), humanClusterAABB.ptr);
        clusterGrid = ClusterGridParams{ bmin[0],bmin[1],bmin[2], cell, dims[0],dims[1],dims[2], 0u, 0u, (float)margin };
        clusterGridBuilt = true;
    }

    // Live cluster VF pipeline (replaces detectCollisionsTwoMesh when enabled):
    // cloth cluster AABBs (GPU) → grid pair query (GPU) → pair expansion (GPU) →
    // bidirectional per-pair descent (GPU). Two reads of GPU counts (pair count;
    // dir counts) drive tight dispatches — InFrame-style; an async (bounded
    // over-dispatch) variant is a follow-up. Requires objTrees[0]=cloth grouped,
    // objTrees[1]=static cluster Human.
    void detectCollisionsCluster(PR margin) {
        if (objTrees.size() < 2) { detectCollisionsTwoMesh(margin); return; }
        TRI_LBVH& cloth = objTrees[0];
        TRI_LBVH& human = objTrees[1];
        int kc = cloth.numGroups, ks = human.numGroups;
        if (kc <= 0 || ks <= 0) { detectCollisionsTwoMesh(margin); return; }
        for (auto& tr : objTrees) tr.syncEachPhase = true;
        Scene<METAL, PR>::packedCollisionData.clearAnalyticPairs();
        queryBegin();

        // FULLY ASYNC: no count readbacks. GPU counters bound over-dispatched
        // kernels, so the whole pipeline is one command stream with a single sync
        // at queryEnd (matching detectCollisionsTwoMesh's 1 sync/substep) — the 2
        // readback syncs of a tight pipeline would explode the per-substep sync
        // floor and lose to the full path.
        static MTL::ComputePipelineState* resetPSO = MetalKernelContext::getPSO("reset_counter");
        cloth.computeClusterAABBGPU(clothClusterAABB);          // (1) cloth AABBs (GPU)
        if (!clusterGridBuilt) { MetalGlobalContext::commitAndWait(); buildHumanClusterGrid(human, margin); }

        // (2) grid pair query (GPU). Reset count on GPU, dispatch over kc. Cap the
        // pair buffer / over-dispatch: kc*ks explodes at very high s (s8 ~117M), but
        // the actual overlaps are few thousand, so a 4M cap never clamps in practice.
        uint32_t maxPairs = (uint32_t)std::min<uint64_t>((uint64_t)kc * (uint64_t)ks, 4u*1024u*1024u);
        if (clusterPairCap < maxPairs) { clusterPairCap = maxPairs; clusterPairBuf = VectorBase<METAL, ClusterPair>((Index)maxPairs); }
        if (!clusterPairCount.ptr) clusterPairCount = VectorBase<METAL, uint32_t>(1, 0u);
        MetalGlobalContext::setBuffer(clusterPairCount, 0); MetalGlobalContext::dispatchThreads(resetPSO, 1);
        ClusterGridParams gp = clusterGrid; gp.numQuery = (uint32_t)kc; gp.maxPairs = maxPairs; gp.margin = (float)margin;
        MTL::ComputePipelineState* qpso = MetalKernelContext::getPSO("clusterpair_query");
        MetalGlobalContext::setBuffer(clothClusterAABB, 0);
        MetalGlobalContext::setBuffer(humanClusterAABB, 1);
        MetalGlobalContext::setBuffer(gridCellStart, 2);
        MetalGlobalContext::setBuffer(gridCellClusters, 3);
        MetalGlobalContext::setBytes(gp, 4);
        MetalGlobalContext::setBuffer(clusterPairBuf, 5);
        MetalGlobalContext::setBuffer(clusterPairCount, 6);
        MetalGlobalContext::dispatchThreads(qpso, (Index)kc);

        // (3) expand pairs → dirA/dirB SAPPairs (GPU), over-dispatched maxPairs and
        // bounded by the GPU pair count. A vertex can appear in multiple pairs, so
        // size the dir buffers by vertCount × (a safe partner-cluster fanout).
        uint32_t capA = (uint32_t)(cloth.positions.size/3) * 16u + 64u;
        uint32_t capB = (uint32_t)(human.positions.size/3) * 16u + 64u;
        if (clusterDirCapA < capA) { clusterDirCapA = capA; clusterDirA = VectorBase<METAL, typename TRI_LBVH::SAPPair>((Index)capA); }
        if (clusterDirCapB < capB) { clusterDirCapB = capB; clusterDirB = VectorBase<METAL, typename TRI_LBVH::SAPPair>((Index)capB); }
        if (!clusterDirACount.ptr) clusterDirACount = VectorBase<METAL, uint32_t>(1, 0u);
        if (!clusterDirBCount.ptr) clusterDirBCount = VectorBase<METAL, uint32_t>(1, 0u);
        MetalGlobalContext::setBuffer(clusterDirACount, 0); MetalGlobalContext::dispatchThreads(resetPSO, 1);
        MetalGlobalContext::setBuffer(clusterDirBCount, 0); MetalGlobalContext::dispatchThreads(resetPSO, 1);
        MTL::ComputePipelineState* epso = MetalKernelContext::getPSO("clusterpair_expand");
        MetalGlobalContext::setBuffer(clusterPairBuf, 0);
        MetalGlobalContext::setBuffer(clusterPairCount, 1);     // GPU bound (no readback)
        MetalGlobalContext::setBuffer(cloth.clusterVertOffsets, 2);
        MetalGlobalContext::setBuffer(cloth.clusterVerts, 3);
        MetalGlobalContext::setBuffer(cloth.groupNodeBase, 4);
        MetalGlobalContext::setBuffer(human.clusterVertOffsets, 5);
        MetalGlobalContext::setBuffer(human.clusterVerts, 6);
        MetalGlobalContext::setBuffer(human.groupNodeBase, 7);
        MetalGlobalContext::setBuffer(clusterDirA, 8);
        MetalGlobalContext::setBuffer(clusterDirACount, 9);
        MetalGlobalContext::setBuffer(clusterDirB, 10);
        MetalGlobalContext::setBuffer(clusterDirBCount, 11);
        MetalGlobalContext::setBytes(capA, 12);
        MetalGlobalContext::setBytes(capB, 13);
        MetalGlobalContext::dispatchThreads(epso, (Index)maxPairs);

        // (4) bidirectional per-pair descent (GPU), over-dispatched + GPU-bounded.
        human.dispatchPointPairsGPUBounded(0, margin, clusterDirA, clusterDirACount, capA);  // cloth V → human tree
        cloth.dispatchPointPairsGPUBounded(1, margin, clusterDirB, clusterDirBCount, capB);  // human V → cloth tree
        queryEnd();
    }
    void queryEnd() {
        // None/PerFrame: leave the query dispatches in flight (narrow reads
        // broadCollisions on the GPU) and skip the diagnostic flag reads — no
        // sync at all. InFrame syncs so the CPU flag reads below are coherent.
        if (!syncEachPhase) return;
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
