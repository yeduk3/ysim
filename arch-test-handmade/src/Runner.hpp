#ifndef RUNNER_HPP
#define RUNNER_HPP


#include "SimConfig.hpp"
#include "SimulatorBasic.hpp"
#include "SolverExplicit.hpp"

struct Runner {
    void run() {
        auto sim = SimConfig{}
                    .setDt(Real(1)/60)
                    .setSubsteps(60)
                    .setSimulatorType<SimulatorBasic>();

        auto solver = SolverConfig{}
                        .setSolverType<SolverExplicit>();

        auto cd = CDConfig{}
                    .setMargin(0.002)
                    .setRadius(0.001)
                    .setPhase1<LBVH_Karras12>({})
                    .setPhase2<ExhaustiveSearch>({});
        SimBuilder builder;
        auto simul = builder.create(sim, solver, cd);

        simul.step();
    }
};



#endif // !RUNNER_HPP
