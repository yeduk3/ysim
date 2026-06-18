#ifndef BASIC_SIMULATOR_HPP
#define BASIC_SIMULATOR_HPP


#include "Solver.hpp"

template <class SimConfigT, class SolverT, class CDPT, class SceneT>
requires Solver<SolverT, CDPT, SceneT>
class BasicSimulator {
    SimConfigT simConfig;
    SolverT solver;
    CDPT cd;
    SceneT scene;

    BasicSimulator(
        SimConfigT simConfig,
        SolverT solver,
        CDPT cd,
        SceneT scene
    ) : simConfig(simConfig),
        solver(solver),
        cd(cd),
        scene(scene)
    {}

    void step(SolverT& solver, CDPT& cd, SceneT& scene, const SimConfigT& simConfig) {
        Real subDt = simConfig.dt / simConfig.substeps;
        for(int substep = 0; substep < simConfig.substeps; substep++) {
            cd.dcd();
            solver.accumulate();
            solver.integrate(scene, subDt);
        }
    }
};


#endif // !BASIC_SIMULATOR_HPP
