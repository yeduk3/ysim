#pragma once
#include "collision/IBVH.hpp"
#include "collision/AABB4.hpp"
#include "collision/RadixSorter.hpp"
#include "collision/BroadBuffers.hpp"   // QueryFlag mirror (per-tree flag buffer)
#include "backend/MetalContext.hpp"
#include "core/Types.hpp"               // BehaviorType / ShapeType

#include <cmath>
#include <iostream>
#include <type_traits>
#include <vector>

// DEFAULT IBVH concrete: Karras (2012) linear BVH over triangle primitives.
// Real GPU build/refit driving bvh.metal kernels over Scene.topology[0].facets
// + SimState.x. Ported from src/main.cpp 5101-6934 (KARRAS path only).
//
// Contracts preserved verbatim (enforced by .metal reuse + AABB4.hpp mirrors):
//   MortonNode 8B / BVHNode 32B leaf sentinel (childA==-1, childB=facet id) /
//   AABB4 32B with _pad0==numPrimitives / 10-bit Morton parity / Karras
//   findSplit+determineRange / buildTree_Tri treeParent fill / bottomUpBoxes
//   atomic+seq_cst-fence + WEDGE/INDEX guards / leaf slot = N+id-1 / refit
//   does NOT commit (Simulator batches the single per-frame commit).
//
// METAL-first this pass (DECISIONS A6). For BE!=METAL the body is a no-op so
// the template still compiles; the factory only ever instantiates METAL here.
template <typename BE, typename PR>
struct Karras12BVH : IBVH<BE, PR> {
    BVHPartConfig parts;
    explicit Karras12BVH(const BVHPartConfig& p) : parts(p) { initPSOs(); }

    const char* name() const override { return "Karras12 LBVH"; }

    // ---- owned buffers (sized from numFacets in build()) ----
    RadixSorter<METAL, MortonNode> sorter;
    VectorBase<METAL, MortonNode> mortons;        // numFacets
    VectorBase<METAL, BVHNode>    tree;           // numNodes = 2*numFacets-1
    VectorBase<METAL, int>        treeParent;     // numNodes
    VectorBase<METAL, uint32_t>   treeVisitCounts;// numNodes  (D-029)
    // cached views (not owned), set in build()/buildForObject(). posView is a
    // SUB-VIEW of state.x starting at this object's vertex slice (§3.1): for
    // object i>0 it advances both ptr (+vertexBase*3) and the GPU buffer offset
    // (+vertexBase*3*sizeof(PR)), so the kernels' x[localId] read THIS object's
    // vertices even though every mesh shares one packed state.x. fillMortons /
    // buildTree / buildLeaf / queryPoints all bind posView (NOT raw state.x).
    VectorBase<METAL, PR>    posView;             // = state.x sub-view at vertexBase
    VectorBase<METAL, Index> facetView;           // = scene.topology[obj].facets (mesh-local ids)
    Index numFacets = 0, numNodes = 0, vertexBase = 0, vertexCount = 0;

    // Per-object identity (mirrored from the original LINEAR BVH struct). Written
    // by buildForObject; objIndex flows into broadCollisions.objPair (D-041
    // turn-3: SCENE object INDEX, the narrow kernel's statesOffsets subscript).
    int          objIndex   = -1;
    BehaviorType objBehavior = BehaviorType::Float;
    ShapeType    objShape    = ShapeType::Mesh;
    bool         objStatic   = false;             // collider (m=0) → build-once + CPU combine
    bool         staticCombined = false;          // combineStaticOnce ran (anti-tunneling §6)

    // Per-tree query overflow flag (queryPoints buffer(6)). Host-visible; reset
    // before each detect, read after the per-frame commit.
    VectorBase<METAL, QueryFlag> qFlag;           // [1]

    MTL::ComputePipelineState *fillMortonsPSO = nullptr, *buildTreePSO = nullptr,
        *buildLeafPSO = nullptr, *bottomUpBoxesPSO = nullptr,
        *zeroVisitCountsPSO = nullptr, *enlargeLeafPSO = nullptr;

    void initPSOs() {
        if constexpr (std::is_same_v<BE, METAL>) {
            fillMortonsPSO     = MetalKernelContext::getPSO("fillMortons_Tri");
            buildTreePSO       = MetalKernelContext::getPSO("buildTree_Tri");
            buildLeafPSO       = MetalKernelContext::getPSO("buildLeaf_Tri");
            enlargeLeafPSO     = MetalKernelContext::getPSO("enlargeLeaf_Tri");
            zeroVisitCountsPSO = MetalKernelContext::getPSO("zeroVisitCounts");
            bottomUpBoxesPSO   = MetalKernelContext::getPSO("bottomUpBoxes");
        }
    }

    // PART1 sceneBox (CPU reduce over THIS object's vertex slice). _pad0 :=
    // numFacets is the only field bottomUpBoxes needs; fillMortons also reads
    // min/max. posView already points at the object's slice (§3.1), so no
    // vertexBase math here. Returns sceneBox with i0 set to numFacets.
    AABB4 sceneBoxFromPositions(SimState<METAL, PR>& /*state*/) const {
        PR* xp = posView.ptr;
        AABB4 box(tinym::vec3_view(xp), tinym::vec3_view(xp + 3));
        for (Index i = 6; i < vertexCount * 3; i += 3)
            box.combine(tinym::vec3_view(xp + i));
        box.i0 = (int)numFacets;
        return box;
    }

    // Single-tree path preserved verbatim: builds over object 0 + commits once
    // (self-contained at init). SceneBVH does NOT call this — it calls the
    // non-committing buildForObject and batches one commit (§7).
    void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            buildForObject(scene, state, 0);
            if (numFacets == 0) return;
            MetalGlobalContext::commitAndWait();         // self-contained at init
            sanityPrint(state);
        }
    }

    // NEW (§3): per-object build target. Builds a tree over object `obj`'s
    // facets + its vertex slice (posView = state.x sub-view at vertexBase).
    // Does NOT commit — the caller (SceneBVH) batches one commit across all
    // objects (TLAS sync, §7). For obj==0 this reproduces the old build body.
    void buildForObject(Scene<BE, PR>& scene, SimState<BE, PR>& state, Index obj) {
        if constexpr (std::is_same_v<BE, METAL>) {
            objIndex = (int)obj;
            if (obj >= (Index)scene.topology.size() || !scene.topology[obj].built) {
                numFacets = 0; return;
            }
            vertexBase  = scene.statesOffsets[obj];
            vertexCount = scene.objects[obj].vertexCount;
            facetView   = scene.topology[obj].facets;
            // §3.1: sub-view of state.x at this object's slice (3 PR / vertex).
            posView     = VectorBase<METAL, PR>(state.x, (size_t)vertexBase * 3,
                                                (size_t)vertexCount * 3);
            objBehavior = scene.objects[obj].behavior;
            objShape    = scene.objects[obj].shape;
            // Static for the broad phase = explicitly static OR a fixed (m=0)
            // collider: Float/Kinematic never move, so their tree is built once
            // and gets the one-time CPU re-combine (§6 anti-tunneling). Cloth
            // (dynamic) refits every substep.
            objBehavior == BehaviorType::Float ||
            objBehavior == BehaviorType::Kinematic
                ? (objStatic = true) : (objStatic = scene.objects[obj].isStatic);
            staticCombined = false;

            if (!qFlag.ptr) qFlag = VectorBase<METAL, QueryFlag>(1);

            numFacets = scene.topology[obj].numFacets;
            numNodes  = (numFacets > 0) ? 2 * numFacets - 1 : 0;
            if (!tree.ptr || tree.size != numNodes) {
                mortons         = VectorBase<METAL, MortonNode>(numFacets);
                tree            = VectorBase<METAL, BVHNode>(numNodes);
                treeParent      = VectorBase<METAL, int>(numNodes);
                treeVisitCounts = VectorBase<METAL, uint32_t>(numNodes);
            }
            if (numFacets == 0) return;

            AABB4 sceneBox = sceneBoxFromPositions(state);  // PART1 (CPU, posView slice)

            // PART2 fillMortons_Tri  (slot 0 = posView, NOT raw state.x — §3.1)
            MetalGlobalContext::setBuffer(posView, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(sceneBox, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::dispatchThreads(fillMortonsPSO, numFacets);

            // PART3 radix sort (in place by code)
            sorter.sort(mortons);

            // PART4 buildTree_Tri (leaf AABBs + internal childA/B + treeParent)
            int nf = (int)numFacets;
            MetalGlobalContext::setBuffer(posView, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(nf, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::setBuffer(tree, 4);
            MetalGlobalContext::setBuffer(treeParent, 5);
            MetalGlobalContext::dispatchThreads(buildTreePSO, numFacets);

            // PART5 combine (zeroVisitCounts + bottomUpBoxes); single leaf = root
            if (numFacets > 1) bottomUpCombine(sceneBox);
            // NO commitAndWait here — caller batches (build() wrapper / SceneBVH).
        }
    }

    void refit(SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            if (numFacets == 0) return;
            AABB4 sceneBox; sceneBox.i0 = (int)numFacets;  // bottomUpBoxes reads _pad0 only
            int nf = (int)numFacets;
            // buildLeaf_Tri — leaf AABBs from current positions (no commit).
            // slot 0 = posView (object's slice), NOT raw state.x — §3.1.
            MetalGlobalContext::setBuffer(posView, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(nf, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::setBuffer(tree, 4);
            MetalGlobalContext::dispatchThreads(buildLeafPSO, numFacets);
            if (numFacets > 1) bottomUpCombine(sceneBox);
            // NO commitAndWait — Simulator does the single per-frame commit.
        }
    }

    // Root AABB of this BLAS (single-root Karras → slot 0). Read on the CPU
    // for the TLAS object-vs-object cull; valid only after a commit (build's
    // batched commit or Simulator's per-frame commit).
    AABB4 rootAABB() const {
        if (numNodes == 0 || !tree.ptr) return AABB4();
        return tree.ptr[0].aabb;
    }

    // CPU bottom-up re-combine (anti-tunneling §6). The GPU bottomUpBoxes can
    // leave a LARGE static tree under-combined (root minY pinned at init 0),
    // so the broad query prunes wrong and the cloth tunnels through. A static
    // tree skips the per-substep GPU re-combine, so correct it ONCE on the CPU:
    // a leading commit makes the GPU leaf AABBs host-visible, then walk childA/B
    // bottom-up from the root. Verbatim algorithm from main.cpp 6413 (LINEAR).
    void combineStaticOnce() {
        if constexpr (std::is_same_v<BE, METAL>) {
            if (numFacets <= 1 || !tree.ptr) return;
            MetalGlobalContext::commitAndWait();         // flush GPU leaf writes
            std::vector<std::pair<int, bool>> stk;
            stk.emplace_back(0, false);
            while (!stk.empty()) {
                auto [id, done] = stk.back();
                stk.pop_back();
                BVHNode& node = tree.ptr[id];
                if (node.childA < 0) continue;           // leaf
                if (!done) {
                    stk.emplace_back(id, true);
                    stk.emplace_back(node.childA, false);
                    stk.emplace_back(node.childB, false);
                } else {
                    node.aabb.min = tree.ptr[node.childA].aabb.min;
                    node.aabb.max = tree.ptr[node.childA].aabb.max;
                    node.aabb.combine(tree.ptr[node.childB].aabb);
                }
            }
        }
    }

    // margin-isotropic: folded into queryPoints elsewhere; valid no-op per
    // IBVH contract (BVH_VERSIONS.md §3.2). enlargeLeaf_Tri PSO kept loaded
    // for the swept follow-up (needs state.v plumbing).
    void enlargeTrajectory(SimState<BE, PR>&, PR) override {}

    // STRETCH: arch engine has no packedCollisionData yet; valid-tree is the
    // goal this pass. No-op until the narrow-phase port adds broad buffers.
    void detectCollisions(PR, bool) override {}

private:
    // zeroVisitCounts + bottomUpBoxes (sparse slots 2,4,5,6). No commit.
    void bottomUpCombine(const AABB4& sceneBox) {
        uint32_t nn = (uint32_t)numNodes;
        MetalGlobalContext::setBuffer(treeVisitCounts, 0);
        MetalGlobalContext::setBytes(nn, 1);
        MetalGlobalContext::dispatchThreads(zeroVisitCountsPSO, numNodes);

        MetalGlobalContext::setBytes(sceneBox, 2);
        MetalGlobalContext::setBuffer(tree, 4);
        MetalGlobalContext::setBuffer(treeParent, 5);
        MetalGlobalContext::setBuffer(treeVisitCounts, 6);
        MetalGlobalContext::dispatchThreads(bottomUpBoxesPSO, numFacets);
    }

    // Tree sanity after build (build() already commitAndWait'd, so .ptr is
    // host-readable). Root must enclose all cloth verts + all boxes finite.
    void sanityPrint(SimState<METAL, PR>& state) {
        if (numFacets == 0) return;
        int leaves = 0, internals = 0;
        bool finite = true;
        for (Index i = 0; i < numNodes; ++i) {
            const BVHNode& n = tree.ptr[i];
            (n.childA < 0 ? leaves : internals)++;
            const float* c[6] = {&n.min.x,&n.min.y,&n.min.z,&n.max.x,&n.max.y,&n.max.z};
            for (int k = 0; k < 6; ++k) if (!std::isfinite(*c[k])) finite = false;
            if (n.min.x > n.max.x || n.min.y > n.max.y || n.min.z > n.max.z) finite = false;
        }
        AABB4 ref = sceneBoxFromPositions(state);
        const BVHNode& r = tree.ptr[0];
        const float eps = 1e-4f;
        bool encloses = r.min.x <= ref.min.x + eps && r.max.x >= ref.max.x - eps &&
                        r.min.y <= ref.min.y + eps && r.max.y >= ref.max.y - eps &&
                        r.min.z <= ref.min.z + eps && r.max.z >= ref.max.z - eps;
        std::cout << "[Karras12] nodes=" << numNodes
                  << " leaves=" << leaves << "(exp " << numFacets << ")"
                  << " internals=" << internals << "(exp " << (numFacets - 1) << ")"
                  << " root[(" << r.min.x << "," << r.min.y << "," << r.min.z << ")-("
                  << r.max.x << "," << r.max.y << "," << r.max.z << ")]"
                  << " enclosesAll=" << (encloses ? "yes" : "no")
                  << " finite=" << (finite ? "yes" : "no")
                  << " " << ((encloses && finite &&
                             leaves == (int)numFacets &&
                             internals == (int)numFacets - 1) ? "OK" : "BAD") << "\n";
    }
};
