#ifndef BASIC_SIMULATOR_HPP
#define BASIC_SIMULATOR_HPP


#include "Solver.hpp"

template <class SimConfigT, class SolverT, class CDT, class SceneT>
requires Solver<SolverT, CDT, SceneT>
struct SimulatorBasic {
    SimConfigT simConfig;
    SolverT solver;
    CDT cd;
    SceneT scene;

    SimulatorBasic(
        SimConfigT simConfig,
        SolverT solver,
        CDT cd,
        SceneT scene
    ) : simConfig(simConfig),
        solver(solver),
        cd(cd),
        scene(scene)
    {}

    void step() {
        Real subDt = simConfig.dt / simConfig.substeps;
        for(int substep = 0; substep < simConfig.substeps; substep++) {
            cd.dcd();
            solver.accumulate(cd, scene);
            solver.integrate(scene, subDt);
            cd.ccd();
        }
    }
};


#endif // !BASIC_SIMULATOR_HPP
