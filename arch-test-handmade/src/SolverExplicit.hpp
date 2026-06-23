#ifndef SOLVER_EXPLICIT_HPP
#define SOLVER_EXPLICIT_HPP

#include "DefaultTypes.hpp"

#include <iostream>

template <class CDT>
struct SolverExplicit {
    SolverExplicit(CDT cd) {
    }
    template <class SceneT>
    void accumulate(CDT& cd, const SceneT& scene) {
        std::cout << "SolverExplicit accumulated." << std::endl;
    }

    template <class SceneT>
    void integrate(SceneT& scene, Real dt) {
        std::cout << "SolverExplicit integrated." << std::endl;
    }
};

#endif // !SOLVER_EXPLICIT_HPP
