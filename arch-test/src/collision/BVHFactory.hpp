#pragma once
#include "collision/IBVH.hpp"
#include "collision/Karras12BVH.hpp"
#include "collision/SceneBVH.hpp"

#include <memory>
#include <stdexcept>

// Picks the BVH VERSION concrete and threads per-part cpu/gpu config into
// it (DECISIONS C5/C6). Karras12 (single BLAS) + Scene (TLAS) are constructible;
// the other versions are deferred concretes (BVH_VERSIONS.md §5) and throw.
// The factory fixes the version ONCE (no per-frame toggle push-down, §1).
template <typename BE, typename PR>
struct BVHFactory {
    static std::unique_ptr<IBVH<BE, PR>> make(const BVHConfig& cfg = {}) {
        switch (cfg.version) {
            case BVHVersion::Karras12:
                return std::make_unique<Karras12BVH<BE, PR>>(cfg.parts);
            case BVHVersion::Scene:
                return std::make_unique<SceneBVH<BE, PR>>(cfg.parts);
            default:
                throw std::runtime_error(
                    "BVHFactory: version not yet ported (see BVH_VERSIONS.md)");
        }
    }
};
