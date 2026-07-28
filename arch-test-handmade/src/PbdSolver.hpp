#ifndef PBD_SOLVER_HPP
#define PBD_SOLVER_HPP

// The first *real* solver in this prototype: Position Based Dynamics
// (Mueller et al. 2007), CPU, hand-rolled. Every other solver in Engine.hpp is
// a printing mock; this one owns particle state and actually moves it.
//
// It satisfies the same Solver concept, so the engine needed only one addition:
// SimulatorBasic::setup() calls solver.setup(objects) when the solver defines
// one (detected with `if constexpr (requires ...)`, mocks are unaffected).
//
// Per substep, integrate() runs the whole PBD loop:
//   1. v += a*dt, predict p = x + v*dt
//   2. `iterations` Gauss-Seidel passes over distance constraints + floor
//   3. v = (p - x)/dt, x = p, damping
// accumulate() only gathers external acceleration (gravity) -- the concept's
// accumulate() has no dt, so all time-stepping belongs in integrate().
//
// Geometry comes from the existing SceneObject vocabulary, so cpp scenes and
// JSON scenes both work unchanged: Cloth -> an n x n grid (n = round(sqrt(
// vertices))) of unit-mass particles with structural + shear constraints,
// Floor -> a y = floorY ground plane, Rigid -> not simulated yet.

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#include "CommonTypes.h"
#include "Engine.hpp"

// --------------------------------------------------------------------------
// Minimal vector type. ponytail: 3 floats and 6 operators beat pulling Eigen
// into a self-contained prototype; swap it out if this ever grows a real mesh.
// --------------------------------------------------------------------------

struct Vec3 {
    Real x{}, y{}, z{};
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline Vec3 operator*(Vec3 a, Real s) { return { a.x * s, a.y * s, a.z * s }; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
inline Vec3& operator-=(Vec3& a, Vec3 b) { a = a - b; return a; }
inline Real  dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Real  length(Vec3 a) { return std::sqrt(dot(a, a)); }

// --------------------------------------------------------------------------
// Particle state
// --------------------------------------------------------------------------

struct DistanceConstraint {
    UInt a = 0, b = 0;
    Real rest = 0;
};

struct ParticleSystem {
    std::vector<Vec3> x;   // positions
    std::vector<Vec3> p;   // predicted positions
    std::vector<Vec3> v;   // velocities
    std::vector<Real> w;   // inverse mass; 0 == pinned
    std::vector<DistanceConstraint> springs;

    UInt count() const { return static_cast<UInt>(x.size()); }

    UInt add(Vec3 pos, Real invMass) {
        const UInt i = count();
        x.push_back(pos);
        p.push_back(pos);
        v.push_back({});
        w.push_back(invMass);
        return i;
    }
    void connect(UInt a, UInt b) {
        springs.push_back({ a, b, length(x[b] - x[a]) });
    }
};

// --------------------------------------------------------------------------
// The solver
// --------------------------------------------------------------------------

struct SolverPBD {
    // Tunables. Left as plain fields on purpose: real cloth needs calibration
    // and this is the knob panel until a scene format carries material params.
    Vec3 gravity{ 0, Real(-9.81), 0 };
    Real stiffness  = Real(0.9);    // [0,1], iteration-count compensated
    UInt iterations = 4;
    // Per-substep velocity damping. 0 by default: constraint projection is
    // already dissipative, and a per-substep factor compounds hard (0.002 at
    // 600 substeps/s leaves only 30% of the velocity after one second).
    Real damping    = Real(0);
    Real clothSize  = Real(1);      // side length of a generated cloth patch
    Real clothDrop  = Real(1);      // height the first cloth patch starts at
    Real clothTilt  = Real(0.35);   // radians about Z; a perfectly flat sheet
                                    // lands flat and no constraint ever fires

    ParticleSystem ps;
    bool hasFloor = false;
    Real floorY   = 0;
    Vec3 accel{};                   // external acceleration for this substep
    bool quiet    = false;          // self-test mutes the per-substep line

    SolverPBD() { std::cout << "    [solver] SolverPBD created (real CPU physics)\n"; }

    // --- setup: turn the scene's object list into particles ---------------

    void setup(const std::vector<SceneObject>& objects) {
        UInt clothIdx = 0;
        for (const auto& o : objects) {
            switch (o.kind) {
                case ObjectKind::Cloth: {
                    const Vec3 center{ 0,
                                       clothDrop + Real(0.3) * static_cast<Real>(clothIdx),
                                       0 };
                    const UInt n = addClothGrid(o.vertexCount, center, clothSize);
                    std::cout << "    [pbd   ] cloth '" << o.name << "': " << n << "x" << n
                              << " grid, " << (n * n) << " particles\n";
                    ++clothIdx;
                    break;
                }
                case ObjectKind::Floor:
                    hasFloor = true;
                    floorY   = 0;
                    std::cout << "    [pbd   ] floor '" << o.name << "': plane y=" << floorY << "\n";
                    break;
                case ObjectKind::Rigid:
                    // ponytail: rigid bodies are not simulated yet; add a shape
                    // constraint (or a static SDF) here when a scene needs one.
                    std::cout << "    [pbd   ] rigid '" << o.name << "': skipped (not simulated)\n";
                    break;
            }
        }
        std::cout << "    [pbd   ] total " << ps.count() << " particles, "
                  << ps.springs.size() << " distance constraints\n";
    }

    // Builds an n x n grid centred on `center`, tilted by clothTilt about Z;
    // n = round(sqrt(vertexCount)), >= 2. Structural (axis) + shear (diagonal)
    // constraints; no bend constraints.
    UInt addClothGrid(UInt vertexCount, Vec3 center, Real size) {
        UInt n = static_cast<UInt>(std::lround(std::sqrt(static_cast<double>(vertexCount))));
        if (n < 2) n = 2;
        const Real step = size / static_cast<Real>(n - 1);
        const Real half = size * Real(0.5);
        const Real c = std::cos(clothTilt), s = std::sin(clothTilt);
        const UInt base = ps.count();

        for (UInt i = 0; i < n; ++i) {
            const Real lx = static_cast<Real>(i) * step - half;
            for (UInt j = 0; j < n; ++j) {
                const Real lz = static_cast<Real>(j) * step - half;
                ps.add({ center.x + lx * c, center.y + lx * s, center.z + lz }, Real(1));
            }
        }

        const auto id = [&](UInt i, UInt j) { return base + i * n + j; };
        for (UInt i = 0; i < n; ++i) {
            for (UInt j = 0; j < n; ++j) {
                if (i + 1 < n) ps.connect(id(i, j), id(i + 1, j));            // structural
                if (j + 1 < n) ps.connect(id(i, j), id(i, j + 1));            // structural
                if (i + 1 < n && j + 1 < n) {
                    ps.connect(id(i, j), id(i + 1, j + 1));                   // shear
                    ps.connect(id(i + 1, j), id(i, j + 1));                   // shear
                }
            }
        }
        return n;
    }

    // --- the Solver concept ----------------------------------------------

    template <class CDT, class SceneT>
    void accumulate(CDT&, const SceneT&) {
        accel = gravity;   // contact impulses from `cd` would be added here
    }

    template <class SceneT>
    void integrate(SceneT&, Real dt) {
        const UInt n = ps.count();
        if (n == 0 || !(dt > Real(0))) {
            if (!quiet) std::cout << "      SolverPBD.integrate        (no particles)\n";
            return;
        }

        // 1. predict
        for (UInt i = 0; i < n; ++i) {
            if (ps.w[i] == Real(0)) { ps.p[i] = ps.x[i]; continue; }
            ps.v[i] += accel * dt;
            ps.p[i]  = ps.x[i] + ps.v[i] * dt;
        }

        // 2. project. Iteration-count-compensated stiffness so the material
        //    does not stiffen when `iterations` changes (Mueller 2007, eq. 11).
        const Real k = (iterations == 0)
            ? stiffness
            : Real(1) - std::pow(Real(1) - stiffness, Real(1) / static_cast<Real>(iterations));
        for (UInt it = 0; it < iterations; ++it) {
            projectDistance(k);
            projectFloor();
        }

        // 3. velocity update + damping
        const Real invDt = Real(1) / dt;
        const Real decay = Real(1) - damping;
        for (UInt i = 0; i < n; ++i) {
            if (ps.w[i] == Real(0)) { ps.v[i] = {}; continue; }
            ps.v[i] = (ps.p[i] - ps.x[i]) * (invDt * decay);
            ps.x[i] = ps.p[i];
        }

        if (!quiet) reportState(dt);
    }

    void projectDistance(Real k) {
        for (const auto& c : ps.springs) {
            const Real wa = ps.w[c.a], wb = ps.w[c.b];
            const Real wsum = wa + wb;
            if (wsum == Real(0)) continue;
            const Vec3 d   = ps.p[c.b] - ps.p[c.a];
            const Real len = length(d);
            if (len < Real(1e-9)) continue;
            const Vec3 corr = d * ((len - c.rest) / len / wsum * k);
            ps.p[c.a] += corr * wa;
            ps.p[c.b] -= corr * wb;
        }
    }

    void projectFloor() {
        if (!hasFloor) return;
        // ponytail: pure position clamp -- no friction, no restitution.
        // Add a tangential velocity term here once a scene needs sliding.
        for (UInt i = 0; i < ps.count(); ++i)
            if (ps.w[i] > Real(0) && ps.p[i].y < floorY) ps.p[i].y = floorY;
    }

    void reportState(Real dt) const {
        Real minY = ps.x[0].y, maxY = ps.x[0].y, sumSpeed = 0;
        for (UInt i = 0; i < ps.count(); ++i) {
            minY = std::fmin(minY, ps.x[i].y);
            maxY = std::fmax(maxY, ps.x[i].y);
            sumSpeed += length(ps.v[i]);
        }
        std::printf("      SolverPBD.integrate   dt=%.5f  y=[%.4f, %.4f]  |v|avg=%.4f\n",
                    static_cast<double>(dt), static_cast<double>(minY),
                    static_cast<double>(maxY),
                    static_cast<double>(sumSpeed / static_cast<Real>(ps.count())));
    }
};

// --------------------------------------------------------------------------
// Self-test: the smallest set of checks that fails if the PBD loop breaks.
// Run with `arch_test_handmade --selftest`; returns a process exit code.
// --------------------------------------------------------------------------

namespace pbd_selftest {

struct NullCD {};
struct NullScene {};

inline bool check(bool ok, const char* what) {
    std::cout << (ok ? "  ok   " : "  FAIL ") << what << "\n";
    return ok;
}

// Steps a solver for `steps` substeps of `dt`, gravity applied each substep.
inline void march(SolverPBD& s, UInt steps, Real dt) {
    NullCD cd; NullScene sc;
    for (UInt i = 0; i < steps; ++i) { s.accumulate(cd, sc); s.integrate(sc, dt); }
}

inline int run() {
    std::cout << "== PBD self-test ==\n";
    bool ok = true;
    const Real dt = Real(1) / 600;   // 10 substeps at 60 Hz

    {   // 1. free fall matches 1/2 g t^2 within the scheme's O(dt) error
        SolverPBD s; s.quiet = true;
        s.ps.add({ 0, 0, 0 }, Real(1));
        march(s, 600, dt);                       // 1 second
        const Real y = s.ps.x[0].y;
        const Real want = Real(-0.5) * Real(9.81);
        ok &= check(std::fabs(y - want) < Real(0.05),
                    "free fall reaches -0.5*g*t^2 after 1 s");
        ok &= check(s.ps.v[0].y < Real(-9), "free-fall velocity is about -g*t");
    }
    {   // 2. the floor is never penetrated
        SolverPBD s; s.quiet = true; s.hasFloor = true; s.floorY = 0;
        s.ps.add({ 0, Real(1), 0 }, Real(1));
        Real worst = Real(1);
        NullCD cd; NullScene sc;
        for (UInt i = 0; i < 1200; ++i) {
            s.accumulate(cd, sc); s.integrate(sc, dt);
            worst = std::fmin(worst, s.ps.x[0].y);
        }
        ok &= check(worst >= Real(-1e-6), "particle never falls below the floor");
        ok &= check(s.ps.x[0].y < Real(1e-3), "particle comes to rest on the floor");
    }
    {   // 3. a stretched constraint relaxes back to its rest length
        SolverPBD s; s.quiet = true; s.gravity = {};
        s.ps.add({ 0, 0, 0 }, Real(1));
        s.ps.add({ Real(1), 0, 0 }, Real(1));
        s.ps.springs.push_back({ 0, 1, Real(1) });   // rest 1, current 1
        s.ps.x[1].x = Real(2);                       // stretch to 2
        s.ps.p[1].x = Real(2);
        march(s, 200, dt);
        const Real len = length(s.ps.x[1] - s.ps.x[0]);
        ok &= check(std::fabs(len - Real(1)) < Real(1e-3),
                    "stretched distance constraint returns to rest length");
    }
    {   // 4. a pinned particle (w = 0) does not move, and a real cloth is finite
        SolverPBD s; s.quiet = true; s.hasFloor = true;
        s.setup({ { "sheet", ObjectKind::Cloth, 100 }, { "floor", ObjectKind::Floor, 4 } });
        s.ps.w[0] = Real(0);
        const Vec3 pinned = s.ps.x[0];
        march(s, 600, dt);
        ok &= check(length(s.ps.x[0] - pinned) < Real(1e-6), "pinned particle stays put");
        bool finite = true, above = true;
        for (UInt i = 0; i < s.ps.count(); ++i) {
            finite &= std::isfinite(s.ps.x[i].x) && std::isfinite(s.ps.x[i].y) &&
                      std::isfinite(s.ps.x[i].z);
            if (s.ps.w[i] > Real(0)) above &= s.ps.x[i].y >= Real(-1e-6);
        }
        ok &= check(finite, "10x10 cloth stays finite after 1 s (no blow-up)");
        ok &= check(above, "no cloth particle sinks through the floor");
    }

    std::cout << (ok ? "== PBD self-test PASSED ==\n" : "== PBD self-test FAILED ==\n");
    return ok ? 0 : 1;
}

} // namespace pbd_selftest

#endif // PBD_SOLVER_HPP
