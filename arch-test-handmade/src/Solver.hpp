#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <concepts>
#include "DefaultTypes.hpp"

template <class SolverT, class CDPT, class SceneT>
concept Solver = requires(SolverT& solver, CDPT& cd, SceneT& scene, Real dt) {
    { solver.accumulate(cd, scene) } -> std::same_as<void>;
    { solver.integrate(scene, dt) } -> std::same_as<void>;
};

#endif // !SOLVER_HPP
