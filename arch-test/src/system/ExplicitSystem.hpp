#pragma once
#include "system/ISystem.hpp"

// Symplectic (semi-implicit) Euler integrator.
//
// ponytail: the real engine runs a 2-pass GPU pipeline (compute_tri_spring_
// forces -> integrate_cloth with fused penetration recovery, src/main.cpp
// 11172-11389 + physics.metal). This pass implements the integrate step on
// the CPU over the shared Metal buffers WITHOUT spring forces or contacts,
// so the Runner exercises the full virtual seam with honest free-fall
// motion. Spring force pass + fused recovery are the next ports (PORT_MAP.md
// system rows). m == 0 marks fixed/kinematic vertices (skipped).
template <typename BE, typename PR>
struct ExplicitSystem : ISystem<BE, PR> {
    const char* name() const override { return "ExplicitSystem (placeholder integrate)"; }

    // f = external forces. (spring forces: TODO)
    void accumulate(Scene<BE, PR>&, SimState<BE, PR>& s) override {
        for (Index i = 0; i < s.numPoints * 3; ++i) s.f[i] = s.externalForces[i];
    }
    // v += (f/m) dt ; x += v dt   (fixed: m == 0)
    void integration(SimState<BE, PR>& s, PR dt) override {
        for (Index i = 0; i < s.numPoints; ++i) {
            PR m = s.m[i];
            if (m <= PR(0)) continue;
            for (int k = 0; k < 3; ++k) {
                Index j = i * 3 + k;
                s.v[j] += (s.f[j] / m) * dt;
                s.x[j] += s.v[j] * dt;
            }
        }
    }
    void recoveryPenetration(SimState<BE, PR>&) override {}  // fused in real integrate
};
