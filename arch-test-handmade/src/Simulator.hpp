#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <concepts>
#include "DefaultTypes.hpp"

// simulator have belows
// - system
// - collision detection pipeline
// - scene
// - simconfig

template <class SimulatorT, class SolverT, class CDPT, class SceneT>
concept Simulator = requires(SimulatorT& simulator, SolverT& solver, CDPT& cd, SceneT& scene, Real dt) {
    { simulator.step(solver, scene, cd, dt) } -> std::same_as<void>;
};


#endif // !SIMULATOR_HPP
