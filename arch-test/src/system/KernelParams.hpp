#pragma once
#include <cstdint>

// Host mirror of physics.metal `SimParams` (bound via setBytes at slot 8 by
// both compute_tri_spring_forces and integrate_cloth). Byte layout MUST match
// the metal struct exactly — DECISIONS C12. ClothParams is NOT redefined here:
// ClothBehaviorParams<float> (core/BehaviorParams.hpp) is byte-identical and is
// bound directly (DECISIONS C22).
struct SimParams {
    float subh;       // h / subSteps
    float G;          // unused by tri path (gravity arrives via externalForces)
    float kair;       // air drag coeff
    float kd;         // spring damping
    uint32_t vertexNum;
    float acctime;    // unused by tri path
};
static_assert(sizeof(SimParams) == 24, "SimParams must match metal SimParams (24 bytes)");
