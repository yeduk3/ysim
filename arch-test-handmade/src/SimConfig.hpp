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
#include "SimulatorBasic.hpp"
#include "CollisionDetector.hpp"


// Simulation configurations //

template <template <class, class, class, class> class SimulatorTemplate>
struct SimConfigTyped;

struct SimConfig {
    Real dt{};
    UInt substeps{};
    constexpr SimConfig& setDt(Real dt) { 
        this->dt = dt;
        return *this;
    }
    constexpr SimConfig& setSubsteps(UInt substeps) {
        this->substeps = substeps;
        return *this;
    }

    template <template <class, class, class, class> class SimulatorTemplate>
    constexpr auto setSimulatorType() { return SimConfigTyped<SimulatorTemplate>(*this); }
};
template <template <class, class, class, class> class SimulatorTemplate>
struct SimConfigTyped {
    const Real dt;
    const UInt substeps;

    template <class SimConfigT, class SolverT, class CDT, class SceneT>
    using Type = SimulatorTemplate<SimConfigT, SolverT, CDT, SceneT>;

    constexpr SimConfigTyped(const SimConfig& config) : dt(config.dt), substeps(config.substeps) {}
};

// Solver configurations //

template <template <class> class SolverTemplate>
struct SolverConfigTyped;

struct SolverConfig {
    template <template <class> class SolverTemplate>
    constexpr auto setSolverType() { return SolverConfigTyped<SolverTemplate>(*this); }
};

template <template <class> class SolverTemplate>
struct SolverConfigTyped {
    template <class CDT>
    using Type = SolverTemplate<CDT>;

    constexpr SolverConfigTyped(const SolverConfig& config) {}

    template <class CDT>
    Type<CDT> create(CDT cd) {
        return Type<CDT>(cd);
    }
};


// CD configurations //

template <template <class, class> class CDTemplate>
struct CDConfigTyped;

template <class Phase1>
struct CDConfigPhase1Typed;

struct CDConfig {
    Real margin;
    Real radius;

    constexpr CDConfig& setMargin(Real margin) {
        this->margin = margin;
        return *this;
    }
    constexpr CDConfig& setRadius(Real radius) {
        this->radius = radius;
        return *this;
    }

    template <template <class, class> class CDTemplate>
    constexpr auto setCDType() { return CDConfigTyped<CDTemplate>(*this); }

    template <class Phase1>
    constexpr auto setPhase1(const typename Phase1::Config& p1Config) {
        return CDConfigPhase1Typed<Phase1>(*this, p1Config);
    }
};

template <class Phase1, class Phase2> 
struct CDConfigPhase2Typed;

template <class Phase1>
struct CDConfigPhase1Typed {
    const Real margin;
    const Real radius;
    const Phase1::Config p1Config;
    using Type = Phase1;
    constexpr CDConfigPhase1Typed(const CDConfig& config, const Phase1::Config& p1Config) 
        : margin(config.margin), radius(config.radius), p1Config(p1Config) {}

    template <class Phase2>
    constexpr auto setPhase2(const Phase2::Config& p2Config) { 
        return CDConfigPhase2Typed<Phase1, Phase2>(*this, p2Config); 
    }
};

template <class Phase1, class Phase2> 
struct CDConfigPhase2Typed {
    const Real margin;
    const Real radius;
    const Phase1::Config p1Config;
    const Phase2::Config p2Config;
    using Type = CollisionDetector<Phase1, Phase2>;

    constexpr CDConfigPhase2Typed(const CDConfigPhase1Typed<Phase1>& config, const Phase2::Config& p2Config)
        : margin(config.margin), radius(config.radius), p1Config(config.p1Config), p2Config(p2Config) {}

    
    Type create() {
        return Type(p1Config, p2Config);
    }
};

template <template <class, class> class CDTemplate>
struct CDConfigTyped {
    template <class Phase1, class Phase2>
    using Type = CDTemplate<Phase1, Phase2>;   

    constexpr CDConfigTyped(const CDConfig& config) {}
};

struct LBVH_Karras12_Config {

};
struct LBVH_Karras12 {
    using Config = LBVH_Karras12_Config;
    LBVH_Karras12(const Config& config) {
        std::cout << "LBVH_Karras12 created." << std::endl;
    }
    void invoke() {
        std::cout << "LBVH_Karras12 invoked." << std::endl;
    }
};


struct ExhaustiveSearchConfig {

};
struct ExhaustiveSearch {
    using Config = ExhaustiveSearchConfig;
    ExhaustiveSearch(const Config& config) {
        std::cout << "ExhaustiveSearch created." << std::endl;
    }
    void invoke() {
        std::cout << "ExhaustiveSearch invoked." << std::endl;
    }
};

struct Scene {};
struct SceneConfig {
    Scene create() {
        return Scene();
    }
};

// builder pattern (kind of)
struct SimBuilder {
    SceneConfig sceneConfig;



    // finalize //

    template <class SimConfigT, class SolverConfigT, class CDPConfigT> 
    auto create(SimConfigT simConfig, SolverConfigT solverConfig, CDPConfigT cdConfig) {
        using CDT = CDPConfigT::Type;
        using SolverT = SolverConfigT::template Type <CDT>;
        using SimulatorT = typename SimConfigT::template Type<SimConfigT, SolverT, CDT, Scene>;

        static_assert(Simulator<SimulatorT>);

        CDT cd = cdConfig.create();
        SolverT solver = solverConfig.create(cd);
        return SimulatorT(simConfig, solver, cd, sceneConfig.create());
    }
};


#endif // !SIMCONFIG_HPP
