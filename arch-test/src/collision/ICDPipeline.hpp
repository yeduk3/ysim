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
    // True only when a CCD swept test needs the per-substep xPrev snapshot.
    // While false the Simulator skips the CPU xPrev copy (would read GPU-stale
    // x under deferred kernels anyway; xPrev is unused with no CCD). When CCD
    // lands this becomes a GPU copy kernel, NOT a CPU loop. (design §6)
    virtual bool needsXPrev() const { return false; }
};
