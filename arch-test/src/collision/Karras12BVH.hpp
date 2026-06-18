#pragma once
#include "collision/IBVH.hpp"
#include "collision/AABB4.hpp"
#include "collision/RadixSorter.hpp"
#include "backend/MetalContext.hpp"

#include <cmath>
#include <iostream>
#include <type_traits>

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
    // cached views (not owned), set in build()
    VectorBase<METAL, PR>    posView;             // = state.x (vertexBase==0 §0)
    VectorBase<METAL, Index> facetView;           // = scene.topology[0].facets
    Index numFacets = 0, numNodes = 0, vertexBase = 0, vertexCount = 0;

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

    // PART1 sceneBox (CPU reduce over the cloth's vertex slice). _pad0 :=
    // numFacets is the only field bottomUpBoxes needs; fillMortons also reads
    // min/max. Returns sceneBox with i0 set to numFacets.
    AABB4 sceneBoxFromPositions(SimState<METAL, PR>& state) const {
        PR* xp = state.x.ptr + vertexBase * 3;
        AABB4 box(tinym::vec3_view(xp), tinym::vec3_view(xp + 3));
        for (Index i = 6; i < vertexCount * 3; i += 3)
            box.combine(tinym::vec3_view(xp + i));
        box.i0 = (int)numFacets;
        return box;
    }

    void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            if (scene.topology.empty() || !scene.topology[0].built) { numFacets = 0; return; }
            vertexBase  = scene.statesOffsets[0];
            vertexCount = scene.objects[0].vertexCount;
            facetView   = scene.topology[0].facets;
            posView     = state.x;                       // §0: relies on vertexBase==0

            numFacets = scene.topology[0].numFacets;
            numNodes  = (numFacets > 0) ? 2 * numFacets - 1 : 0;
            if (!tree.ptr || tree.size != numNodes) {
                mortons         = VectorBase<METAL, MortonNode>(numFacets);
                tree            = VectorBase<METAL, BVHNode>(numNodes);
                treeParent      = VectorBase<METAL, int>(numNodes);
                treeVisitCounts = VectorBase<METAL, uint32_t>(numNodes);
            }
            if (numFacets == 0) return;

            AABB4 sceneBox = sceneBoxFromPositions(state);  // PART1 (CPU)

            // PART2 fillMortons_Tri
            MetalGlobalContext::setBuffer(state.x, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(sceneBox, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::dispatchThreads(fillMortonsPSO, numFacets);

            // PART3 radix sort (in place by code)
            sorter.sort(mortons);

            // PART4 buildTree_Tri (leaf AABBs + internal childA/B + treeParent)
            int nf = (int)numFacets;
            MetalGlobalContext::setBuffer(state.x, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(nf, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::setBuffer(tree, 4);
            MetalGlobalContext::setBuffer(treeParent, 5);
            MetalGlobalContext::dispatchThreads(buildTreePSO, numFacets);

            // PART5 combine (zeroVisitCounts + bottomUpBoxes); single leaf = root
            if (numFacets > 1) bottomUpCombine(sceneBox);

            MetalGlobalContext::commitAndWait();         // self-contained at init
            sanityPrint(state);
        }
    }

    void refit(SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            if (numFacets == 0) return;
            AABB4 sceneBox; sceneBox.i0 = (int)numFacets;  // bottomUpBoxes reads _pad0 only
            int nf = (int)numFacets;
            // buildLeaf_Tri — leaf AABBs from current positions (no commit)
            MetalGlobalContext::setBuffer(state.x, 0);
            MetalGlobalContext::setBuffer(facetView, 1);
            MetalGlobalContext::setBytes(nf, 2);
            MetalGlobalContext::setBuffer(mortons, 3);
            MetalGlobalContext::setBuffer(tree, 4);
            MetalGlobalContext::dispatchThreads(buildLeafPSO, numFacets);
            if (numFacets > 1) bottomUpCombine(sceneBox);
            // NO commitAndWait — Simulator does the single per-frame commit.
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
