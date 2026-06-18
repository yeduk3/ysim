#pragma once
#include "collision/ICDPipeline.hpp"
#include "collision/IBVH.hpp"
#include "collision/BVHFactory.hpp"
#include "backend/MetalContext.hpp"
#include "core/Types.hpp"

#include <algorithm>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

// Composable concrete: an IBVH broad phase + the BruteForce<METAL> narrow phase.
// dcd = refit -> enlargeTrajectory -> detectCollisions (⇒ BroadCollision[]) ->
//       narrow_pt_tri (⇒ compact NarrowCollision[]) -> CPU per-vertex counting
//       sort (⇒ SimState.vertColFacets/Offsets CSR). ccd is a no-op: CCD is
// fused into the narrow swept test in the original engine (DECISIONS A9 / D-A1).
//
// The integrate_cloth kernel already reads vertColFacets[5] + vertColFacetsOffsets[6]
// as WHOLE buffers, indexing the CSR by GLOBAL vertex id (obase = statesOffsets[oid];
// begin = Offsets[obase+id]). So the only narrow contract is: fill those two
// SimState buffers with the right contents/sizes. No integrator change.
template <typename BE, typename PR>
struct DefaultCDPipeline : ICDPipeline<BE, PR> {
    std::unique_ptr<IBVH<BE, PR>> broad;

    // narrow_pt_tri scratch (the arch analog of packedCollisionData narrow side).
    // Sized off the broad concrete's maxNumCollisions in build().
    Index maxCollisions = 0;
    VectorBase<METAL, NarrowCollision> narrowCollisions;    // maxCollisions
    VectorBase<METAL, Index>           numNarrowCollisions; // [1], host-visible
    // Self-skip CSR dummies (slots 7/8; unused for cloth-vs-collider where
    // qObjId != tObjId, so narrow_pt_tri's same-object branch is never taken).
    VectorBase<METAL, Index>           dummyAdj, dummyAdjOff;

    MTL::ComputePipelineState* narrowPSO = nullptr;

    // Verification: contacts emitted by the most recent dcd (Runner reads this).
    uint32_t lastNarrowContacts = 0;

    explicit DefaultCDPipeline(std::unique_ptr<IBVH<BE, PR>> b) : broad(std::move(b)) {
        if constexpr (std::is_same_v<BE, METAL>)
            narrowPSO = MetalKernelContext::getPSO("narrow_pt_tri");
    }

    const char* name() const override { return "DefaultCDPipeline(broad+narrow)"; }

    void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) override {
        broad->build(scene, state);
        if constexpr (std::is_same_v<BE, METAL>) {
            // Size narrow scratch off the broad pair budget. If the broad concrete
            // owns no pair buffer (broad-less), fall back to a small slab so the
            // buffers stay valid (narrow then sees numBroad==0 -> free-fall).
            auto* bcnt = broad->broadPairCount();
            (void)bcnt;
            // The contact slab can be at most the broad pair count; reuse the
            // broad concrete's max via lastBroadPairCount sizing is unknown at
            // build, so mirror the original engine default budget.
            maxCollisions = 1u << 20;   // 1,048,576 (original packedCollisionData)
            narrowCollisions    = VectorBase<METAL, NarrowCollision>(maxCollisions);
            numNarrowCollisions = VectorBase<METAL, Index>(1, Index(0));
            dummyAdj    = VectorBase<METAL, Index>(1, Index(0));
            dummyAdjOff = VectorBase<METAL, Index>(2, Index(0));   // CSR [0,0]
            // Grow the SimState contact slab from the realize() dummy(1) to the
            // contact budget so the CSR scatter has room. Offsets stays numPoints+1
            // (whole-scene global ids — integrate indexes it by obase+id).
            state.vertColFacets = VectorBase<METAL, NarrowCollision>(maxCollisions);
            std::cout << "[Narrow] contact slab=" << maxCollisions
                      << " vertColFacets resized; CSR size=" << state.vertColFacetsOffsets.size
                      << " (numPoints+1)\n";
        }
    }

    void dcd(Scene<BE, PR>& scene, SimState<BE, PR>& state, PR margin) override {
        if constexpr (!std::is_same_v<BE, METAL>) return;
        broad->refit(state);
        broad->enlargeTrajectory(state, margin);
        // detectCollisions fills the broad concrete's broadCollisions[] +
        // numBroadCollisions[0] and does its own commitAndWait (reads root AABBs
        // / count on the CPU). After it returns the encoder is closed.
        broad->detectCollisions(margin, /*selfCollision=*/false);
        narrow(scene, state);
    }
    void ccd(Scene<BE, PR>&, SimState<BE, PR>&, PR) override {}  // fused into dcd

    // narrow_pt_tri is a SWEPT test (reads xPrev at slot 10). Without a per-
    // substep xPrev snapshot it would use the stale t0 position as the segment
    // start, making every substep's segment huge → spurious crossings + a
    // violent landing bounce. Request the Simulator's per-substep snapshot.
    bool needsXPrev() const override { return true; }

    uint32_t lastBroadPairCount() const override {
        return broad ? broad->lastBroadPairCount() : 0u;
    }
    uint32_t lastNarrowContactCount() const override { return lastNarrowContacts; }

private:
    // narrow_pt_tri over the broad pairs -> compact NarrowCollision[] -> CPU
    // counting sort -> SimState CSR. Falls back to an empty CSR (begin==end ⇒
    // recovery no-op ⇒ free-fall) whenever broad produced nothing OR the broad
    // concrete owns no pair buffer (degrade, never crash — TASK contract).
    void narrow(Scene<METAL, PR>& scene, SimState<METAL, PR>& state) {
        lastNarrowContacts = 0;
        // Always re-zero the CSR first: an empty CSR is the correct "no contact"
        // contract (begin==end per vertex). Done on the host (Shared storage).
        zeroOffsets(state);

        auto* pairBuf = broad->broadPairBuffer();
        auto* pairCnt = broad->broadPairCount();
        if (!pairBuf || !pairCnt || !pairBuf->ptr || !pairCnt->ptr) return;  // broad-less

        const Index numBroad = pairCnt->ptr[0];
        numNarrowCollisions.ptr[0] = 0;
        if (numBroad == 0) return;

        // radius/thickness: arch carries radius on Simulator (0.012) — passed as
        // the default here for the basic scene; thickness from the sole cloth.
        const PR radius    = PR(0.012);
        const PR thickness = scene.objects[0].cloth.thickness;   // 0.01

        NarrowParams p{};
        p.numBroadCollisions = (uint32_t)std::min<Index>(numBroad, maxCollisions);
        p.maxNumCollisions   = (uint32_t)maxCollisions;
        p.radius             = (float)radius;
        p.thickness          = (float)thickness;
        p.skipSphere         = 0u;   // no analytic path in arch — tri soup only

        // narrow_pt_tri binding table (bruteforce.metal:41). state.x IS the
        // scene-wide packed positions; statesOffsetsGPU IS the position offsets;
        // scene.packedFacets/Offsets are the scene-wide packed target facets.
        MetalGlobalContext::setBuffer(*pairBuf,                   0);
        MetalGlobalContext::setBuffer(numNarrowCollisions,        1);
        MetalGlobalContext::setBuffer(narrowCollisions,           2);
        MetalGlobalContext::setBuffer(state.x,                    3);
        MetalGlobalContext::setBuffer(state.statesOffsetsGPU,     4);
        MetalGlobalContext::setBuffer(scene.packedFacets,         5);
        MetalGlobalContext::setBuffer(scene.packedFacetsOffsets,  6);
        MetalGlobalContext::setBuffer(dummyAdj,                   7);
        MetalGlobalContext::setBuffer(dummyAdjOff,                8);
        MetalGlobalContext::setBytes(p,                           9);
        MetalGlobalContext::setBuffer(state.xPrev,              10);
        MetalGlobalContext::dispatchThreads(narrowPSO, p.numBroadCollisions);

        // The CPU sort below READS the GPU-written narrowCollisions, so flush.
        // (First cut = §6 option A: one extra commit/substep. The GPU sort
        //  fill_vf_offsets + prefix + scatter would remove it; A7 follow-up.)
        MetalGlobalContext::commitAndWait();

        countingSortByVertex(state);
    }

    // Zero the CSR offsets (numPoints+1). Host write (Shared storage) so the
    // later integrate dispatch sees it. setZero via the Eigen map.
    void zeroOffsets(SimState<METAL, PR>& state) {
        Index n = state.vertColFacetsOffsets.size;
        Index* off = state.vertColFacetsOffsets.ptr;
        for (Index i = 0; i < n; ++i) off[i] = Index(0);
    }

    // CPU per-vertex counting sort (verbatim port of main.cpp:7555-7580, arch
    // types). CSR keyed by GLOBAL vertex id = statesOffsets[query] + point, so
    // integrate's obase+id indexing reads the right slice. Three passes over the
    // compact narrowCollisions[0..numNarrow).
    void countingSortByVertex(SimState<METAL, PR>& state) {
        const Index numNarrow = std::min<Index>(numNarrowCollisions.ptr[0], maxCollisions);
        lastNarrowContacts = (uint32_t)numNarrow;
        if (numNarrow == 0) return;

        Index*           off  = state.vertColFacetsOffsets.ptr;   // numPoints+1, zeroed
        NarrowCollision* dst  = state.vertColFacets.ptr;          // contact slab
        const Index      offN = state.vertColFacetsOffsets.size;
        const Index*     soff = state.statesOffsetsGPU.ptr;       // global vert base
        const NarrowCollision* src = narrowCollisions.ptr;

        auto ppidOf = [&](const NarrowCollision& nc) -> Index {
            return soff[nc.objPair.query] + nc.indexPair.point;
        };
        // (1) count -> off[ppid+1]++
        for (Index i = 0; i < numNarrow; ++i) {
            Index ppid = ppidOf(src[i]);
            if (ppid + 1 < offN) off[ppid + 1]++;
        }
        // (2) prefix sum -> true CSR; off[offN-1] == numNarrow
        for (Index i = 1; i < offN; ++i) off[i] += off[i - 1];
        // (3) scatter via temp cursor
        std::vector<Index> cursor(offN > 0 ? offN - 1 : 0, 0);
        for (Index i = 0; i < numNarrow; ++i) {
            Index ppid  = ppidOf(src[i]);
            if (ppid >= (Index)cursor.size()) continue;          // guard malformed
            Index colid = off[ppid] + cursor[ppid]++;
            if (colid < maxCollisions) dst[colid] = src[i];
        }
    }
};

// Default broad phase is the SCENE TLAS (SceneBVH): the basic scene has 3
// collidable meshes, so cloth-vs-collider needs the multi-tree query, not the
// single-mesh Karras12 BLAS. Pass an explicit cfg to override.
template <typename BE, typename PR>
inline std::unique_ptr<ICDPipeline<BE, PR>> makeDefaultCDPipeline(
        const BVHConfig& cfg = BVHConfig{ BVHVersion::Scene, {} }) {
    return std::make_unique<DefaultCDPipeline<BE, PR>>(BVHFactory<BE, PR>::make(cfg));
}
