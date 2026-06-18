#pragma once
#include "collision/IBVH.hpp"
#include "collision/Karras12BVH.hpp"
#include "collision/BroadBuffers.hpp"
#include "collision/AABB4.hpp"
#include "backend/MetalContext.hpp"
#include "core/Types.hpp"

#include <algorithm>
#include <iostream>
#include <type_traits>
#include <vector>

// SCENE TLAS broad phase (Stage 2). A NEW IBVH concrete that owns one
// Karras12BVH BLAS per collidable object + the BroadCollision scratch. Mirrors
// the original src/main.cpp BVH<SCENE,OBJECT> (6935-7361): per-object trees +
// an ordered double loop that queries each cloth (dynamic) mesh's vertices
// against each Float collider's tree, emitting (point, triangle) BroadCollision
// pairs the future narrow phase will consume.
//
// Composition over inheritance (§1): Karras12BVH stays the unchanged single
// BLAS; SceneBVH composes N of them via buildForObject(scene,state,i).
//
// Top tree: deferred. For the basic 3-object scene the object-vs-object overlap
// is O(K^2) on the CPU (cheap root-AABB intersect). An EDGE_LBVH super-tree is
// only worth it past tens of objects (§8.5).
template <typename BE, typename PR>
struct SceneBVH : IBVH<BE, PR> {
    // QueryPointsParams host mirror (bvh.metal 1292, 11 fields, byte-exact).
    struct QueryPointsParams {
        float    queryMargin;
        uint32_t numPoints;
        uint32_t qObjId;
        uint32_t tObjId;
        uint32_t maxNumCollisions;
        uint32_t qBehavior;
        uint32_t tBehavior;
        uint32_t qShape;
        uint32_t tShape;
        uint32_t numNodes;
        uint32_t entryRoot;
    };

    BVHPartConfig parts;
    std::vector<Karras12BVH<BE, PR>> objTrees;   // one BLAS per object
    BroadBuffers<BE, PR> broad;                   // owned broad-output scratch
    MTL::ComputePipelineState* queryPointsPSO = nullptr;
    Scene<BE, PR>* scenePtr = nullptr;            // cached for vertexCount lookups

    static constexpr Index APPROX_PER_VERTEX = 32; // per-vertex contact budget

    // Last-detect stats (read by the Runner for the broad-pair-count print).
    uint32_t lastBroadPairs = 0;
    bool     lastOverflow   = false;
    bool     sanityDumped   = false;   // one-shot pair-sanity dump guard

    explicit SceneBVH(const BVHPartConfig& p) : parts(p) {
        if constexpr (std::is_same_v<BE, METAL>)
            queryPointsPSO = MetalKernelContext::getPSO("queryPoints");
    }
    const char* name() const override { return "SceneBVH (TLAS over Karras12)"; }

    // Query meshes (cloth) are the only ones we query FROM; everything that
    // carries facets is a valid TARGET (Float Human + Float Ground here).
    static bool isQueryMesh(BehaviorType b) {
        return b == BehaviorType::TriangularCloth || b == BehaviorType::FastGridCloth;
    }

    // ---- build: one BLAS per object that has triangle topology ----
    void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            scenePtr = &scene;
            objTrees.assign(scene.objects.size(), Karras12BVH<BE, PR>(parts));
            for (Index i = 0; i < (Index)scene.objects.size(); ++i)
                objTrees[i].buildForObject(scene, state, i);

            // Size the broad scratch off the cloth's vertex count (sum of all
            // query meshes — basic scene has one).
            Index queryVerts = 0;
            for (Index i = 0; i < (Index)scene.objects.size(); ++i)
                if (isQueryMesh(objTrees[i].objBehavior))
                    queryVerts += objTrees[i].vertexCount;
            broad.ensure(queryVerts, APPROX_PER_VERTEX);

            // ONE commit for all the per-object GPU builds (§7), then correct
            // static trees on the CPU (§6 anti-tunneling). build() is the only
            // place that commits for static trees; refit() never re-touches them.
            MetalGlobalContext::commitAndWait();
            for (auto& t : objTrees) {
                if (t.objStatic && t.numFacets > 1) {
                    t.combineStaticOnce();
                    t.staticCombined = true;
                }
            }

            std::cout << "[SceneBVH] trees=" << objTrees.size()
                      << " queryVerts=" << queryVerts
                      << " maxBroad=" << broad.maxNumCollisions << "\n";
            for (Index i = 0; i < (Index)objTrees.size(); ++i) {
                AABB4 r = objTrees[i].rootAABB();
                std::cout << "[SceneBVH]  obj" << i
                          << " facets=" << objTrees[i].numFacets
                          << " static=" << (objTrees[i].objStatic ? "yes" : "no")
                          << " query=" << (isQueryMesh(objTrees[i].objBehavior) ? "yes" : "no")
                          << " root[(" << r.min.x << "," << r.min.y << "," << r.min.z
                          << ")-(" << r.max.x << "," << r.max.y << "," << r.max.z << ")]\n";
            }
        }
    }

    // ---- refit: dynamic (cloth) trees only; static colliders stay frozen ----
    void refit(SimState<BE, PR>& state) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            for (auto& t : objTrees)
                if (!t.objStatic) t.refit(state);     // refit does NOT commit
            // Static trees: combined once in build(); leave frozen. No commit
            // here — Simulator's per-frame commit covers the dynamic refit.
        }
    }

    // margin folded into the per-point query (§5); valid no-op per IBVH.
    void enlargeTrajectory(SimState<BE, PR>&, PR) override {}

    uint32_t lastBroadPairCount() const override { return lastBroadPairs; }

    // Broad-output handles for the narrow phase (Stage 3). The narrow_pt_tri
    // dispatch in DefaultCDPipeline binds these directly — SceneBVH owns the
    // BroadBuffers, narrow only reads them.
    VectorBase<BE, BroadCollision>* broadPairBuffer() override {
        return &broad.broadCollisions;
    }
    VectorBase<BE, uint32_t>* broadPairCount() override {
        return &broad.numBroadCollisions;
    }

    // ---- detectCollisions: each cloth's verts vs each Float collider tree ----
    void detectCollisions(PR margin, bool selfCollision) override {
        if constexpr (std::is_same_v<BE, METAL>) {
            // queryBegin: reset the broad counter + per-tree flags on the host.
            // Safe to write now — the prior substep's commit has completed (the
            // dcd seam runs after Simulator's per-frame commit / before the next).
            broad.numBroadCollisions.ptr[0] = 0u;
            for (auto& t : objTrees)
                if (t.qFlag.ptr) { t.qFlag.ptr[0].stackOverflow = 0u;
                                   t.qFlag.ptr[0].collisionOverflow = 0u; }

            // Ordered double loop, non-dedup (DECISIONS C21 / original 7227):
            // a Float target is never a query mesh, so each Float/cloth pair is
            // emitted in its single valid direction (cloth point vs target tri).
            for (Index q = 0; q < (Index)objTrees.size(); ++q) {
                auto& qt = objTrees[q];
                if (!isQueryMesh(qt.objBehavior)) continue;   // Float/Kinematic skip as query
                if (qt.numFacets == 0) continue;
                AABB4 qa = qt.rootAABB();
                for (Index t = 0; t < (Index)objTrees.size(); ++t) {
                    if (q == t) {
                        if (selfCollision) queryPoints(q, q, margin);
                        continue;
                    }
                    auto& tt = objTrees[t];
                    if (tt.numFacets == 0) continue;          // no facets → nothing to hit
                    AABB4 ta = tt.rootAABB();
                    if (qa.intersect(ta))                     // cheap CPU object-vs-object cull
                        queryPoints(q, t, margin);
                }
            }

            // queryEnd: the broad-output count read for the (future) narrow
            // dispatch needs the GPU work flushed. For this broad-only slice the
            // single commit here makes the count + flags host-readable so the
            // Runner can validate the pair buffer. When narrow lands this folds
            // into the broad→narrow shared-commit seam (§7a) owned by the
            // pipeline; SceneBVH would then only ENCODE the queries.
            MetalGlobalContext::commitAndWait();

            lastBroadPairs = broad.numBroadCollisions.ptr[0];
            lastOverflow = false;
            for (auto& t : objTrees)
                if (t.qFlag.ptr && t.qFlag.ptr[0].collisionOverflow) lastOverflow = true;
            if (lastOverflow)
                std::cout << "[SceneBVH] broad buffer overflow ("
                          << lastBroadPairs << "/" << broad.maxNumCollisions << ")\n";

            // One-shot pair sanity (Stage 2 validation, fires once when broad
            // first emits pairs). Asserts the cross-mesh contract every entry
            // must satisfy for the future narrow phase: objPair.x == a query
            // mesh index, objPair.y == a valid target index, point < that query
            // mesh's vertexCount, triangle < that target's facet count.
            if (!sanityDumped && lastBroadPairs > 0) {
                sanityDumped = true;
                uint32_t n = std::min<uint32_t>(lastBroadPairs, broad.maxNumCollisions);
                uint32_t bad = 0, show = std::min<uint32_t>(n, 4u);
                for (uint32_t k = 0; k < n; ++k) {
                    const BroadCollision& bc = broad.broadCollisions.ptr[k];
                    Index qo = (Index)bc.objPair.query, to = (Index)bc.objPair.target;
                    bool ok = qo >= 0 && qo < (Index)objTrees.size() &&
                              to >= 0 && to < (Index)objTrees.size() &&
                              isQueryMesh(objTrees[qo].objBehavior) &&
                              (Index)bc.indexPair.point    < objTrees[qo].vertexCount &&
                              (Index)bc.indexPair.triangle < objTrees[to].numFacets;
                    if (!ok) ++bad;
                }
                std::cout << "[SceneBVH] pair sanity: " << n << " pairs, " << bad
                          << " invalid -> " << (bad == 0 ? "VALID" : "BAD") << "; sample:";
                for (uint32_t k = 0; k < show; ++k) {
                    const BroadCollision& bc = broad.broadCollisions.ptr[k];
                    std::cout << " {pt=" << bc.indexPair.point
                              << ",tri=" << bc.indexPair.triangle
                              << ",obj(" << bc.objPair.query << "->" << bc.objPair.target << ")}";
                }
                std::cout << "\n";
            }
        }
    }

    // ---- per-tree query dispatch (§4.1). q = query mesh (cloth), target = ----
    // collider. objPair carries SCENE OBJECT INDICES (D-041 turn-3) so the
    // narrow kernel can use them as statesOffsets[] subscripts.
    void queryPoints(Index q, Index target, PR margin) {
        if constexpr (std::is_same_v<BE, METAL>) {
            auto& qt = objTrees[q];        // query positions (cloth slice)
            auto& tt = objTrees[target];   // target facets + tree
            if (qt.numFacets == 0 || tt.numFacets == 0) return;
            uint32_t qVerts = (uint32_t)qt.vertexCount;
            if (qVerts == 0) return;

            QueryPointsParams p;
            p.queryMargin      = (float)margin;
            p.numPoints        = qVerts;
            p.qObjId           = (uint32_t)q;          // query OBJECT INDEX  → objPair.x
            p.tObjId           = (uint32_t)target;     // target OBJECT INDEX → objPair.y
            p.maxNumCollisions = (uint32_t)broad.maxNumCollisions;
            p.qBehavior        = (uint32_t)qt.objBehavior;
            p.tBehavior        = (uint32_t)tt.objBehavior;
            p.qShape           = (uint32_t)qt.objShape;
            p.tShape           = (uint32_t)tt.objShape;
            p.numNodes         = (uint32_t)tt.numNodes; // traversal bound = TARGET tree
            p.entryRoot        = 0u;                     // single-root Karras

            // slot 0: QUERY positions (cloth slice — posView already offset §3.1)
            MetalGlobalContext::setBuffer(qt.posView, 0);
            // slot 1: TARGET facets (mesh-local triangle vertex ids)
            MetalGlobalContext::setBuffer(tt.facetView, 1);
            // slot 2: TARGET tree
            MetalGlobalContext::setBuffer(tt.tree, 2);
            MetalGlobalContext::setBytes(p, 3);
            MetalGlobalContext::setBuffer(broad.broadCollisions, 4);
            MetalGlobalContext::setBuffer(broad.numBroadCollisions, 5);  // atomic
            MetalGlobalContext::setBuffer(qt.qFlag, 6);
            MetalGlobalContext::dispatchThreads(queryPointsPSO, qVerts);
        }
    }
};
