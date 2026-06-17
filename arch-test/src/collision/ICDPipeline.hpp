#pragma once
#include "core/Scene.hpp"
#include "core/SimState.hpp"

// The collision swap unit (blueprint goal 1). Virtual dcd/ccd; the whole
// pipeline is swapped, not broad/narrow separately. Simulator holds a
// unique_ptr<ICDPipeline>.
template <typename BE, typename PR>
struct ICDPipeline {
    virtual ~ICDPipeline() = default;
    virtual const char* name() const = 0;
    virtual void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) = 0;
    virtual void dcd(Scene<BE, PR>& scene, SimState<BE, PR>& state, PR margin) = 0;
    virtual void ccd(Scene<BE, PR>& scene, SimState<BE, PR>& state, PR dt) = 0;
};
