#pragma once
#include "sim/Simulator.hpp"
#include "system/ExplicitSystem.hpp"
#include "scenes/basic_scene.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

// Headless driver (blueprint goal: no GL). Builds System -> Simulator ->
// setupBasicScene -> initialize -> loop update(). Verifies the basic scene
// actually steps (PORT_MAP.md §4): positions finite, cloth moved under
// gravity. Returns true on pass.
//
// run()      — the single-sim PASS path (unchanged behavior).
// runMulti() — DEMONSTRATES N coexisting Simulators (O2): builds 2+ sims with
//              DIFFERENT setup, steps each SEQUENTIALLY, reads each one's
//              result PURELY through its own LUT (caches keys, not pointers).
template <typename BE, typename PR>
struct Runner {
    int frames;
    explicit Runner(int f = 300) : frames(f) {}

    // read one finished sim via its data bus — caches keys, not pointers
    static PR clothY(const Simulator<BE, PR>& sim) {
        const Precision* pos = sim.lut.template get<Precision>("pos");
        const Index* np = sim.lut.template get<Index>("numPoints");
        Index clothBase = sim.scene.statesOffsets.empty() ? 0 : sim.scene.statesOffsets[0];
        return (pos && np && *np) ? pos[clothBase * 3 + 1] : PR(0);
    }

    bool run() {
        auto sys = std::make_unique<ExplicitSystem<BE, PR>>();
        Simulator<BE, PR> sim(std::move(sys));
        sim.targetFrames = frames;

        setupBasicScene(sim);
        sim.initialize();

        std::cout << "[Runner] objects=" << sim.scene.objects.size()
                  << " verts=" << sim.state.numPoints
                  << " cd=" << (sim.cdPipeline ? sim.cdPipeline->name() : "none")
                  << " sys=" << sim.system->name() << "\n";

        // topology sanity (id 0 grid cloth): facets=2*(N-1)^2, edges>0, restLen>0
        if (!sim.scene.topology.empty() && sim.scene.topology[0].built) {
            const auto& t = sim.scene.topology[0];
            const PR* rest = t.restEdgeLengths.ptr;
            PR minRest = (t.numEdges ? rest[0] : PR(0));
            for (Index e = 1; e < t.numEdges; ++e)
                if (rest[e] < minRest) minRest = rest[e];
            std::cout << "[Topology] cloth(id0) facets=" << t.numFacets
                      << " uniqueEdges=" << t.numEdges
                      << " minRestEdgeLen=" << minRest << "\n";
        }

        // Stage 1 (OBJ + Float topology) sanity: human (id1, FileMesh) +
        // ground (id2) now carry facets-only topology. Report counts so the
        // narrow-phase target set is visibly populated.
        {
            const auto& objs = sim.scene.objects;
            const auto& topo = sim.scene.topology;
            for (size_t i = 1; i < objs.size(); ++i) {
                const char* kind =
                    (objs[i].kind == ObjectDesc::Kind::FileMesh) ? "FileMesh" :
                    (objs[i].kind == ObjectDesc::Kind::Ground)   ? "Ground"   : "GridCloth";
                Index nf = (i < topo.size()) ? topo[i].numFacets : 0;
                bool built = (i < topo.size()) && topo[i].built;
                std::cout << "[Collider] id" << i << " (" << kind << ") verts="
                          << objs[i].vertexCount << " facets=" << nf
                          << " built=" << (built ? "yes" : "no") << "\n";
            }
            std::cout << "[Collider] statesOffsets={";
            for (size_t i = 0; i < sim.scene.statesOffsets.size(); ++i)
                std::cout << sim.scene.statesOffsets[i]
                          << (i + 1 < sim.scene.statesOffsets.size() ? "," : "");
            std::cout << "}\n";
        }

        // cloth (id 0) starting height — read via LUT
        PR y0 = clothY(sim);

        // Stage 2 broad-phase verification: the cloth free-falls (no narrow yet)
        // so it passes THROUGH the human (y∈[0.35,1.18]) and the ground (y≈0).
        // Track the peak broad pair count + the frame it occurred — proves the
        // cross-mesh queryPoints produces (cloth point, target triangle) pairs.
        uint32_t maxBroad = 0, maxNarrow = 0;
        int      maxBroadFrame = -1, maxNarrowFrame = -1;
        PR       minClothY = y0;     // lowest the cloth ever reaches (drape proof)
        while (sim.update()) {
            uint32_t bp = sim.cdPipeline ? sim.cdPipeline->lastBroadPairCount() : 0u;
            if (bp > maxBroad) { maxBroad = bp; maxBroadFrame = sim.frame; }
            uint32_t nc = sim.cdPipeline ? sim.cdPipeline->lastNarrowContactCount() : 0u;
            if (nc > maxNarrow) { maxNarrow = nc; maxNarrowFrame = sim.frame; }
            PR yc = clothY(sim);
            if (yc < minClothY) minClothY = yc;
        }
        std::cout << "[Broad] peakPairs=" << maxBroad
                  << " atFrame=" << maxBroadFrame
                  << " -> " << (maxBroad > 0 ? "PRODUCED" : "EMPTY") << "\n";
        std::cout << "[Narrow] peakContacts=" << maxNarrow
                  << " atFrame=" << maxNarrowFrame
                  << " -> " << (maxNarrow > 0 ? "CONTACTS" : "EMPTY") << "\n";

        // verify via LUT
        const Precision* pos = sim.lut.template get<Precision>("pos");
        const Index* np = sim.lut.template get<Index>("numPoints");
        const int* fr = sim.lut.template get<int>("frame");

        bool finite = true;
        if (pos && np)
            for (Index i = 0; i < *np * 3; ++i)
                if (!std::isfinite((double)pos[i])) { finite = false; break; }
        PR y1 = clothY(sim);

        std::cout << "[Runner] frames=" << (fr ? *fr : -1)
                  << " clothY " << y0 << " -> " << y1
                  << " finite=" << (finite ? "yes" : "no") << "\n";

        // DRAPE proof (Stage 3): the free-fall baseline reaches clothY≈-121 by
        // frame 300. With narrow contacts pushing the cloth out of penetration it
        // must SETTLE on the human/ground instead — clothY stays well above the
        // free-fall floor (human top ≈1.18, ground ≈0; we require > -10 as a wide
        // margin that the cloth did NOT tunnel to the free-fall depth).
        const PR FREEFALL_FLOOR = PR(-10);
        bool draped = (minClothY > FREEFALL_FLOOR);
        std::cout << "[Drape] minClothY=" << minClothY
                  << " finalClothY=" << y1
                  << " (free-fall floor ~ -121) -> "
                  << (draped ? "SETTLED" : "FELL-THROUGH") << "\n";

        // Edge-coherence: |len-rest|/rest over all unique cloth(id0) edges.
        // Proves the sheet fell as a COHERENT sheet (springs working), not as
        // independent free-falling points (which would also drop y). A working
        // stiff solver keeps edges tight; an exploding one yields huge/NaN.
        PR maxStretch = 0, meanStretch = 0;
        uint32_t anomaly = sim.system ? sim.system->anomaly() : 0u;
        bool stretchFinite = true;
        uint32_t edgesOver50 = 0;       // edges stretched >50% (drape hot-spots)
        if (!sim.scene.topology.empty() && sim.scene.topology[0].built && pos) {
            const auto& t = sim.scene.topology[0];
            Index base = sim.scene.statesOffsets.empty() ? 0 : sim.scene.statesOffsets[0];
            PR sum = 0;
            for (Index e = 0; e < t.numEdges; ++e) {
                Index a = t.edges.ptr[e * 2], b = t.edges.ptr[e * 2 + 1];
                const Precision* pa = pos + (base + a) * 3;
                const Precision* pb = pos + (base + b) * 3;
                PR dx = pb[0] - pa[0], dy = pb[1] - pa[1], dz = pb[2] - pa[2];
                PR len = std::sqrt(dx * dx + dy * dy + dz * dz);
                PR rest = t.restEdgeLengths.ptr[e];
                if (!std::isfinite((double)len)) { stretchFinite = false; break; }
                PR st = (rest > PR(0)) ? std::abs(len - rest) / rest : PR(0);
                if (st > maxStretch) maxStretch = st;
                if (st > PR(0.5)) ++edgesOver50;
                sum += st;
            }
            if (t.numEdges) meanStretch = sum / PR(t.numEdges);
        }
        // Coherence for a DRAPED cloth: distinct from the free-fall test. A 1m
        // sheet draped over a ~0.47m human (with the overhang hanging to the
        // ground) legitimately stretches more than a flat free-fall (where
        // meanStretch≈1e-5). The signal that matters is "stable sheet, not an
        // explosion": all edges finite, anomaly clear, MEAN stretch small (the
        // bulk of the sheet stays near rest), and stretch hot-spots confined to
        // a handful of edges bridging the human's sharp features (not a global
        // blow-up). Drape thresholds: mean<0.15 (bulk coherent), few-edge tail.
        bool coherent = stretchFinite && (anomaly == 0u) &&
                        (meanStretch < PR(0.15)) && std::isfinite((double)maxStretch);
        std::cout << "[Verify] meanStretch=" << meanStretch
                  << " maxStretch=" << maxStretch
                  << " edgesOver50%=" << edgesOver50 << "/" << (sim.scene.topology.empty() ? 0 : (int)sim.scene.topology[0].numEdges)
                  << " anomaly=" << anomaly
                  << " -> " << (coherent ? "COHERENT" : "INCOHERENT") << "\n";

        bool clothFell = (!np || *np == 0) || (y1 < y0);
        bool ok = finite && clothFell && coherent && draped;
        std::cout << "[Runner] " << (ok ? "PASS" : "FAIL") << "\n";
        return ok;
    }

    // Build a sim with a custom cloth grid + origin (the rest = basic scene).
    static std::unique_ptr<Simulator<BE, PR>> makeSim(int gridN, tinym::vec3 origin, int tgt) {
        auto sys = std::make_unique<ExplicitSystem<BE, PR>>();
        auto sim = std::make_unique<Simulator<BE, PR>>(std::move(sys));
        sim->targetFrames = tgt;
        setupBasicScene(*sim);
        // override the cloth (id 0) so the two sims diverge
        sim->scene.objects[0].gridN = gridN;
        sim->scene.objects[0].origin = origin;
        return sim;
    }

    // N coexisting sims (O2): all alive at once, stepped one-at-a-time.
    bool runMulti() {
        std::vector<std::unique_ptr<Simulator<BE, PR>>> sims;
        sims.push_back(makeSim(50, tinym::vec3(0.0f, 1.25f, 0.0f), frames));
        sims.push_back(makeSim(30, tinym::vec3(0.5f, 2.00f, 0.0f), frames));

        // initialize ALL first (proves N pools coexist: each owns its memory,
        // and re-activating later does not invalidate the others' buffers).
        for (auto& sim : sims) sim->initialize();

        bool ok = true;
        std::vector<PR> finals;
        for (size_t s = 0; s < sims.size(); ++s) {
            auto& sim = *sims[s];
            PR y0 = clothY(sim);

            while (sim.update()) {}  // only one steps at a time — O2

            const Precision* pos = sim.lut.template get<Precision>("pos");
            const Index* np = sim.lut.template get<Index>("numPoints");
            const int* fr = sim.lut.template get<int>("frame");

            bool finite = true;
            if (pos && np)
                for (Index i = 0; i < *np * 3; ++i)
                    if (!std::isfinite((double)pos[i])) { finite = false; break; }
            PR y1 = clothY(sim);
            bool clothFell = (!np || *np == 0) || (y1 < y0);
            bool simOk = finite && clothFell;

            std::cout << "[Multi] sim" << s
                      << " gridN=" << sim.scene.objects[0].gridN
                      << " verts=" << (np ? *np : 0)
                      << " frames=" << (fr ? *fr : -1)
                      << " clothY " << y0 << " -> " << y1
                      << " finite=" << (finite ? "yes" : "no")
                      << " fell=" << (clothFell ? "yes" : "no")
                      << " " << (simOk ? "PASS" : "FAIL") << "\n";
            finals.push_back(y1);
            ok = ok && simOk;
        }

        // results must be DISTINCT across sims (different setup -> different y)
        bool distinct = true;
        for (size_t i = 0; i < finals.size() && distinct; ++i)
            for (size_t j = i + 1; j < finals.size(); ++j)
                if (finals[i] == finals[j]) { distinct = false; break; }
        std::cout << "[Multi] distinct results=" << (distinct ? "yes" : "no")
                  << " -> " << (ok && distinct ? "PASS" : "FAIL") << "\n";

        // confirm sim0's buffers survived sim1 becoming active (O2 lifetime):
        // re-read sim0 via its cached LUT key — must still resolve & be finite.
        const Precision* re0 = sims[0]->lut.template get<Precision>("pos");
        bool coexist = re0 && std::isfinite((double)re0[0]);
        std::cout << "[Multi] sim0 buffers valid after sim1 ran="
                  << (coexist ? "yes" : "no") << "\n";

        return ok && distinct && coexist;
    }
};
