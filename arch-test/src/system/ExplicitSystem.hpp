#pragma once
#include "system/ISystem.hpp"
#include "system/KernelParams.hpp"
#include "backend/MetalContext.hpp"

#include <cstdint>

// Symplectic (semi-implicit) Euler integrator.
//
// Generic template = CPU reference path: integrate step on the shared buffers
// with env forces only (no springs). The METAL specialization below is the
// REAL 2-pass GPU pipeline ported from src/main.cpp 11172-11389 +
// physics.metal: accumulate = compute_tri_spring_forces, integration =
// integrate_cloth (with fused penetration recovery), recoveryPenetration =
// no-op. m == 0 marks fixed/kinematic vertices.
template <typename BE, typename PR>
struct ExplicitSystem : ISystem<BE, PR> {
    const char* name() const override { return "ExplicitSystem (placeholder integrate)"; }

    // f = external forces. (spring forces: CPU path has none)
    void accumulate(Scene<BE, PR>&, SimState<BE, PR>& s) override {
        for (Index i = 0; i < s.numPoints * 3; ++i) s.f[i] = s.externalForces[i];
    }
    // v += (f/m) dt ; x += v dt   (fixed: m == 0)
    void integration(SimState<BE, PR>& s, PR dt) override {
        for (Index i = 0; i < s.numPoints; ++i) {
            PR m = s.m[i];
            if (m <= PR(0)) continue;
            for (int k = 0; k < 3; ++k) {
                Index j = i * 3 + k;
                s.v[j] += (s.f[j] / m) * dt;
                s.x[j] += s.v[j] * dt;
            }
        }
    }
    void recoveryPenetration(SimState<BE, PR>&) override {}  // fused in real integrate
};

// ============================================================================
// METAL specialization — the real GPU 2-pass.
//
// accumulate  -> compute_tri_spring_forces (reads externalForces + springs -> f)
// integration -> integrate_cloth (f,m,mask,contacts,params -> v,x; recovery fused)
// recoveryPenetration -> no-op (fused into integrate, DECISIONS C13/C21)
//
// One shared bindClothTable() binds the full slot 0-19 table (a SUPERSET of
// what each kernel reads) so the integrate pass is pre-staged exactly as the
// original (main.cpp:11371 re-binds the same setBuffer). NO commit inside —
// the Simulator batches one commitAndWait per frame; the serial compute
// encoder supplies the force->integrate ordering fence.
template <typename PR>
struct ExplicitSystem<METAL, PR> : ISystem<METAL, PR> {
    const char* name() const override { return "ExplicitSystem (GPU 2-pass)"; }

    // Sim params (defaults match the original, main.cpp:11201-11207). subh is
    // (re)derived from the dt the Simulator passes to integration().
    PR h = PR(1.0 / 60.0);
    PR G = PR(-9.8);          // ignored on tri path (gravity via externalForces)
    PR kair = PR(-0.001);     // air drag
    PR kd = PR(0.5);          // spring damping
    PR acctime = PR(0);
    PR clothStiffnessScale = PR(1);  // "팽팽함": scales the bound cloth-params copy

    // World-bounds guard target. Lazy-alloc (slot 20). The integrate kernel
    // stores 1 here if any vertex was sanitized (NaN/Inf/escaped).
    VectorBase<METAL, uint32_t> anomalyFlag;

    // Cached across substeps so integration() can reach topology/objects (its
    // signature carries only SimState). Set every accumulate(), same frame.
    Scene<METAL, PR>* sceneRef = nullptr;
    PR lastSubh = PR(1.0 / 60.0 / 60.0);

    // Bind the full slot 0-19 table for object `oid` (vertex count `vc`,
    // base offset `base`). SUPERSET: compute_tri_spring_forces ignores
    // 4/5/6/18/19; integrate_cloth ignores 7/10-17. Binding both pre-stages
    // the integrate pass with the identical call (original main.cpp:2024-2067).
    void bindClothTable(Scene<METAL, PR>& scene, SimState<METAL, PR>& s,
                        Index oid, Index base, Index vc, const SimParams& sp) {
        using V = VectorBase<METAL, PR>;
        using VI = VectorBase<METAL, Index>;
        const auto& topo = scene.topology[oid];

        // state 0-3 (per-mesh slices of the global SimState slabs)
        MetalGlobalContext::setBuffer(V(s.x, base * 3, vc * 3), 0);
        MetalGlobalContext::setBuffer(V(s.v, base * 3, vc * 3), 1);
        MetalGlobalContext::setBuffer(V(s.f, base * 3, vc * 3), 2);
        MetalGlobalContext::setBuffer(V(s.m, base, vc), 3);
        // constraints / contacts 4-6
        MetalGlobalContext::setBuffer(V(s.fixedParticles, base, vc), 4);
        MetalGlobalContext::setBuffer(s.vertColFacets, 5);             // empty (whole)
        MetalGlobalContext::setBuffer(s.vertColFacetsOffsets, 6);      // zero CSR (whole)
        // external forces 7
        MetalGlobalContext::setBuffer(V(s.externalForces, base * 3, vc * 3), 7);
        // sim params 8-9
        MetalGlobalContext::setBytes(sp, 8);
        {
            ClothBehaviorParams<PR> cp = scene.objects[oid].cloth;   // copy; never mutate stored
            cp.stretch *= clothStiffnessScale;
            cp.shear   *= clothStiffnessScale;
            cp.bend    *= clothStiffnessScale;
            MetalGlobalContext::setBytes(cp, 9);
        }
        // adjacency 10-11 (mesh-local indices; bind whole)
        MetalGlobalContext::setBuffer(topo.edges, 10);
        MetalGlobalContext::setBuffer(topo.facets, 11);
        // stretch springs 12-14
        MetalGlobalContext::setBuffer(topo.vertexAdjEdges, 12);
        MetalGlobalContext::setBuffer(topo.vertexAdjEdgesOffsets, 13);
        MetalGlobalContext::setBuffer(topo.restEdgeLengths, 14);
        // bend springs 15-17
        MetalGlobalContext::setBuffer(topo.vertexOppVertices, 15);
        MetalGlobalContext::setBuffer(topo.vertexOppVerticesOffsets, 16);
        MetalGlobalContext::setBuffer(topo.restOppLengths, 17);
        // packed data 18-19 (pre-staged for integrate)
        MetalGlobalContext::setBuffer(s.statesOffsetsGPU, 18);
        MetalGlobalContext::setBytes(oid, 19);
    }

    static bool isCloth(const ObjectDesc& o) {
        return o.behavior == BehaviorType::TriangularCloth &&
               o.kind == ObjectDesc::Kind::GridCloth && o.vertexCount > 0;
    }

    // Force pass: compute_tri_spring_forces over every deformable cloth.
    void accumulate(Scene<METAL, PR>& scene, SimState<METAL, PR>& s) override {
        sceneRef = &scene;
        auto* pso = MetalKernelContext::getPSO("compute_tri_spring_forces");
        for (size_t oid = 0; oid < scene.objects.size(); ++oid) {
            const auto& o = scene.objects[oid];
            if (!isCloth(o) || !scene.topology[oid].built) continue;
            Index base = scene.statesOffsets[oid];
            Index vc = o.vertexCount;
            SimParams sp{ (float)lastSubh, (float)G, (float)kair, (float)kd,
                          (uint32_t)vc, (float)acctime };
            bindClothTable(scene, s, (Index)oid, base, vc, sp);
            MetalGlobalContext::dispatchThreads(pso, vc);
        }
    }

    // Integrate pass: integrate_cloth (fused penetration recovery) over every
    // deformable cloth. Re-binds the full table (pass 1's later meshes / future
    // narrow dispatches clobber the encoder table) + anomalyFlag at slot 20.
    void integration(SimState<METAL, PR>& s, PR dt) override {
        if (!sceneRef) return;
        if (!anomalyFlag.ptr) { anomalyFlag = VectorBase<METAL, uint32_t>(1); anomalyFlag[0] = 0u; }
        lastSubh = dt;  // Simulator passes subh (= h/subSteps) as dt
        auto& scene = *sceneRef;
        auto* pso = MetalKernelContext::getPSO("integrate_cloth");
        for (size_t oid = 0; oid < scene.objects.size(); ++oid) {
            const auto& o = scene.objects[oid];
            if (!isCloth(o) || !scene.topology[oid].built) continue;
            Index base = scene.statesOffsets[oid];
            Index vc = o.vertexCount;
            SimParams sp{ (float)dt, (float)G, (float)kair, (float)kd,
                          (uint32_t)vc, (float)acctime };
            bindClothTable(scene, s, (Index)oid, base, vc, sp);
            MetalGlobalContext::setBuffer(anomalyFlag, 20);
            MetalGlobalContext::dispatchThreads(pso, vc);
        }
    }

    void recoveryPenetration(SimState<METAL, PR>&) override {}  // fused into integrate_cloth

    uint32_t anomaly() const override { return anomalyFlag.ptr ? anomalyFlag.ptr[0] : 0u; }
};
