#pragma once
#include "sim/Simulator.hpp"
#include "system/ExplicitSystem.hpp"
#include "scenes/basic_scene.hpp"

#include <cmath>
#include <iostream>
#include <memory>

// Headless driver (blueprint goal: no GL). Builds System -> Simulator ->
// setupBasicScene -> initialize -> loop update(). Verifies the basic scene
// actually steps (PORT_MAP.md §4): positions finite, cloth moved under
// gravity. Returns true on pass.
template <typename BE, typename PR>
struct Runner {
    int frames;
    explicit Runner(int f = 300) : frames(f) {}

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

        // cloth (id 0) starting height
        Index clothBase = sim.scene.statesOffsets.empty() ? 0 : sim.scene.statesOffsets[0];
        PR y0 = sim.state.numPoints ? sim.state.x[clothBase * 3 + 1] : PR(0);

        while (sim.update()) {}

        // verify
        bool finite = true;
        for (Index i = 0; i < sim.state.numPoints * 3; ++i)
            if (!std::isfinite((double)sim.state.x[i])) { finite = false; break; }
        PR y1 = sim.state.numPoints ? sim.state.x[clothBase * 3 + 1] : PR(0);

        std::cout << "[Runner] frames=" << sim.frame
                  << " clothY " << y0 << " -> " << y1
                  << " finite=" << (finite ? "yes" : "no") << "\n";

        bool clothFell = (sim.state.numPoints == 0) || (y1 < y0);
        bool ok = finite && clothFell;
        std::cout << "[Runner] " << (ok ? "PASS" : "FAIL") << "\n";
        return ok;
    }
};
