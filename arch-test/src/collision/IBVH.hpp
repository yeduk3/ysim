#pragma once
#include "core/Scene.hpp"
#include "core/SimState.hpp"

// ── BVH version + per-part backend selection (DECISIONS C5/C6) ──────────
// karras12 is the production default; the other versions are tangled into
// one struct in src/main.cpp today and become separate concretes (see
// BVH_VERSIONS.md). Each karras12 build PART has a CPU and a GPU impl in
// the original engine — the factory picks per part.
enum struct BVHVersion { Karras12, Apetrei, SubObject, SpatialHash, MultiLevelHash };
enum struct PartBackend { CPU, GPU };

struct BVHPartConfig {
    PartBackend morton  = PartBackend::GPU; // fillMortons{CPU,GPU}
    PartBackend sort    = PartBackend::GPU; // radixSort{CPU,GPU}
    PartBackend build   = PartBackend::GPU; // buildCPU(Karras) vs buildTreeGPU
    PartBackend combine = PartBackend::GPU; // bottomUpCombine vs bottomUpBoxesGPU
    PartBackend query   = PartBackend::GPU; // queryAABB vs queryPoints
};
struct BVHConfig { BVHVersion version = BVHVersion::Karras12; BVHPartConfig parts; };

// Virtual broad-phase interface. Surface grounded in the mapped BVH public
// methods (BVH_VERSIONS.md §3). Non-uniform extras (segmented / self-
// collision / analytic) folded into flags — DECISIONS A11 left open.
template <typename BE, typename PR>
struct IBVH {
    virtual ~IBVH() = default;
    virtual const char* name() const = 0;
    virtual void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) = 0;
    virtual void refit(SimState<BE, PR>& state) = 0;
    virtual void enlargeTrajectory(SimState<BE, PR>& state, PR margin) = 0;
    virtual void detectCollisions(PR margin, bool selfCollision) = 0;
};
