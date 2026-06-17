#pragma once
#include "collision/IBVH.hpp"
#include "collision/Karras12BVH.hpp"

#include <memory>
#include <stdexcept>

// Picks the BVH VERSION concrete and threads per-part cpu/gpu config into
// it (DECISIONS C5/C6). Only Karras12 is constructible this pass; the other
// versions are deferred concretes (BVH_VERSIONS.md §5) and throw if asked.
template <typename BE, typename PR>
struct BVHFactory {
    static std::unique_ptr<IBVH<BE, PR>> make(const BVHConfig& cfg = {}) {
        switch (cfg.version) {
            case BVHVersion::Karras12:
                return std::make_unique<Karras12BVH<BE, PR>>(cfg.parts);
            default:
                throw std::runtime_error(
                    "BVHFactory: version not yet ported (see BVH_VERSIONS.md)");
        }
    }
};
