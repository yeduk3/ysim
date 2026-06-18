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

        // cloth (id 0) starting height — read via LUT
        PR y0 = clothY(sim);

        while (sim.update()) {}

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

        bool clothFell = (!np || *np == 0) || (y1 < y0);
        bool ok = finite && clothFell;
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
