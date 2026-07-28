#ifndef SCENE_SETTINGS_HPP
#define SCENE_SETTINGS_HPP

// User-editable "scene settings" defined directly in C++. Each scene is the
// single source of its own configuration: it names the solver, the broad/narrow
// collision phases (compile-time type aliases), the time-step, and the objects
// to create. This file knows nothing about the dispatch registry -- add a scene
// here, then register it in Scenes.hpp.
//
// name()/dt()/substeps() are constexpr instance accessors: fixed here, but the
// same SceneConfig interface is also met by the JSON-backed SceneRuntime.

#include <string_view>
#include <vector>

#include "Engine.hpp"
#include "PbdSolver.hpp"

// Default scene: floor only, simplest solver/CD combo. Runs when no scene
// argument is given.
struct SceneDefault {
    using SolverType  = SolverExplicit;
    using BroadPhase  = LBVH_Karras12;
    using NarrowPhase = ExhaustiveSearch;

    constexpr std::string_view name() const { return "default"; }
    constexpr Real dt() const       { return Real(1) / 60; }
    constexpr UInt substeps() const { return 1; }

    std::vector<SceneObject> build() const {
        return {
            { "floor", ObjectKind::Floor, 4 },
        };
    }
};

struct SceneBasic {
    using SolverType  = SolverExplicit;
    using BroadPhase  = LBVH_Karras12;
    using NarrowPhase = ExhaustiveSearch;

    constexpr std::string_view name() const { return "basic"; }
    constexpr Real dt() const       { return Real(1) / 60; }
    constexpr UInt substeps() const { return 2; }

    std::vector<SceneObject> build() const {
        return {
            { "cloth",  ObjectKind::Cloth, 1024 },
            { "sphere", ObjectKind::Rigid,  482 },
        };
    }
};

struct SceneClothDrop {
    using SolverType  = SolverXPBD;
    using BroadPhase  = LBVH_Karras12;
    using NarrowPhase = ProximityQuery;

    constexpr std::string_view name() const { return "cloth_drop"; }
    constexpr Real dt() const       { return Real(1) / 60; }
    constexpr UInt substeps() const { return 8; }

    std::vector<SceneObject> build() const {
        return {
            { "banner", ObjectKind::Cloth, 4096 },
            { "floor",  ObjectKind::Floor,    4 },
        };
    }
};

// Sample scene for the real CPU PBD solver: a 20x20 sheet dropped onto a
// floor. Unlike the mock scenes this one actually moves particles -- watch the
// y range in the per-substep line converge to the floor.
struct SceneClothPbd {
    using SolverType  = SolverPBD;
    using BroadPhase  = LBVH_Karras12;
    using NarrowPhase = ProximityQuery;

    constexpr std::string_view name() const { return "cloth_pbd"; }
    constexpr Real dt() const       { return Real(1) / 60; }
    constexpr UInt substeps() const { return 10; }

    std::vector<SceneObject> build() const {
        return {
            { "sheet", ObjectKind::Cloth, 400 },
            { "floor", ObjectKind::Floor,   4 },
        };
    }
};

struct SceneRigidPile {
    using SolverType  = SolverExplicit;
    using BroadPhase  = SpatialHashing;
    using NarrowPhase = ExhaustiveSearch;

    constexpr std::string_view name() const { return "rigid_pile"; }
    constexpr Real dt() const       { return Real(1) / 120; }
    constexpr UInt substeps() const { return 4; }

    std::vector<SceneObject> build() const {
        return {
            { "box_0", ObjectKind::Rigid, 8 },
            { "box_1", ObjectKind::Rigid, 8 },
            { "box_2", ObjectKind::Rigid, 8 },
            { "floor", ObjectKind::Floor, 4 },
        };
    }
};

#endif // SCENE_SETTINGS_HPP
