#pragma once
#include "core/Scene.hpp"
#include "core/SimState.hpp"

// Integrator interface (blueprint goal 1). Swap Explicit/Implicit/XPBD.
// The three methods name the real data flow (DECISIONS C13): accumulate =
// force pass (env forces + spring kernel), integration = integrate pass,
// recoveryPenetration = penetration push (fused into integrate in the
// original engine).
template <typename BE, typename PR>
struct ISystem {
    virtual ~ISystem() = default;
    virtual const char* name() const = 0;
    virtual void accumulate(Scene<BE, PR>& scene, SimState<BE, PR>& state) = 0;
    virtual void integration(SimState<BE, PR>& state, PR dt) = 0;
    virtual void recoveryPenetration(SimState<BE, PR>& state) = 0;
};
