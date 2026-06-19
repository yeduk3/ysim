#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP


#include <concepts>
#include "DefaultTypes.hpp"

// simulator have belows
// - system
// - collision detection pipeline
// - scene
// - simconfig

template <class SimulatorT>
concept Simulator = requires(SimulatorT& simulator) {
    { simulator.step() } -> std::same_as<void>;
};


#endif // !SIMULATOR_HPP
