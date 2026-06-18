#ifndef SIMCONFIG_HPP
#define SIMCONFIG_HPP

/* > Blue print of the configuration-builder pattern.
 * 
 *
 * SimConfig simConfig = SimConfig{
 *      .dt = dt,
 *      .substeps = substeps
 * };
 *
 * LBVH_Karras12_Config broadConfig = LBVH_Karras12_Config{
 *      .margin = margin,
 *      .radius = radius
 * };
 *
 * ExhaustiveSearch narrowConfig = ExhaustiveSearchConfig{};
 *
 * CDConfig.setBroadPhase(broadConfig)
 *         .setNarrowPhase(narrowConfig)
 * 
 * ExplicitSolverConfig esConfig = ExplicitSolverConfig{
 *      .?
 * }
 *
 * SimBuilder builder;
 * builder.setSimConfig(simConfig)
 *        .setCDConfig(CDConfig)
 *        .setSolver(esConfig)
 *
 */

#include "DefaultTypes.hpp"
#include "Simulator.hpp"
#include "BasicSimulator.hpp"

template <class ConfigT>
concept TypedConfig = requires(ConfigT& config) {
    typename ConfigT::Type;
};

struct SimConfigCommon {
    Real dt{};
    UInt substeps{};
};
struct SimConfigBasic {
    SimConfigCommon data;
    template <class SolverT, class CDPT, class SceneT>
    using SimulatorType = BasicSimulator<SimConfigBasic, SolverT, CDPT, SceneT>;
};



struct SceneConfig {

};

// builder pattern (kind of)
struct SimBuilder {
    SceneConfig sceneConfig;



    // finalize //

    template <class SimConfigT, class SolverConfigT, class CDPConfigT> 
    requires TypedConfig<SolverConfigT> && TypedConfig<CDPConfigT>
    auto create(SimConfigT sim, SolverConfigT sol, CDPConfigT cd) {
        using SolverT = SolverConfigT::Type;
        using CDPT = CDPConfigT::Type;
        using SimulatorT = typename SimConfigT::template Type<SolverT, CDPT, SceneConfig>;

        static_assert(Simulator<SimulatorT, SolverT, CDPT, SceneConfig>);

        return SimulatorT(sim, sol, cd, sceneConfig);
    }
};


#endif // !SIMCONFIG_HPP
