#ifndef SCENE_JSON_HPP
#define SCENE_JSON_HPP

// JSON scene path. Parses a scene file (nlohmann/json) into a SceneRuntime<...>
// value and runs it through the SAME generic runScene / SimulatorBasic as the
// cpp scenes. The solver / broad_phase / narrow_phase strings pick the three
// compile-time types via a 2x2x2 nested dispatch; dt / substeps / name /
// objects are ordinary runtime fields.
//
// Schema:
//   { "name": str, "solver": "explicit"|"xpbd"|"pbd",
//     "broad_phase": "lbvh"|"spatial_hash",
//     "narrow_phase": "exhaustive"|"proximity",
//     "dt": num (default 1/60), "substeps": uint (default 1),
//     "objects": [ { "name": str, "kind": "cloth"|"rigid"|"floor",
//                    "vertices": uint } ] }

#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "Scenes.hpp"

// A scene whose configuration is filled in at runtime. Type aliases stay
// compile-time (chosen by the dispatch below); everything else is a data field.
template <class SolverT, class BroadT, class NarrowT>
struct SceneRuntime {
    using SolverType  = SolverT;
    using BroadPhase  = BroadT;
    using NarrowPhase = NarrowT;

    std::string              nameStr;
    Real                     dtVal       = Real(1) / 60;
    UInt                     substepsVal = 1;
    std::vector<SceneObject> objs;

    std::string_view name() const     { return nameStr; }
    Real dt() const                   { return dtVal; }
    UInt substeps() const             { return substepsVal; }
    std::vector<SceneObject> build() const { return objs; }
};

namespace scene_json {

// Parsed-and-validated JSON fields, before type dispatch.
struct SceneData {
    std::string              name;
    std::string              solver;   // validated: explicit | xpbd | pbd
    std::string              broad;    // validated: lbvh | spatial_hash
    std::string              narrow;   // validated: exhaustive | proximity
    Real                     dt       = Real(1) / 60;
    UInt                     substeps = 1;
    std::vector<SceneObject> objects;
};

inline bool parseKind(std::string_view s, ObjectKind& out) {
    if (s == "cloth") { out = ObjectKind::Cloth; return true; }
    if (s == "rigid") { out = ObjectKind::Rigid; return true; }
    if (s == "floor") { out = ObjectKind::Floor; return true; }
    return false;
}

// --- 3x2x2 string -> type dispatch. All 12 combos reachable. --------------

template <class S, class B, class N>
void runCombo(const SceneData& d, UInt frames) {
    runScene(SceneRuntime<S, B, N>{ d.name, d.dt, d.substeps, d.objects }, frames);
}

template <class S, class B>
void pickNarrow(const SceneData& d, UInt frames) {
    if (d.narrow == "exhaustive") runCombo<S, B, ExhaustiveSearch>(d, frames);
    else                          runCombo<S, B, ProximityQuery>(d, frames);
}

template <class S>
void pickBroad(const SceneData& d, UInt frames) {
    if (d.broad == "lbvh") pickNarrow<S, LBVH_Karras12>(d, frames);
    else                   pickNarrow<S, SpatialHashing>(d, frames);
}

inline void dispatch(const SceneData& d, UInt frames) {
    if (d.solver == "explicit")  pickBroad<SolverExplicit>(d, frames);
    else if (d.solver == "pbd")  pickBroad<SolverPBD>(d, frames);
    else                         pickBroad<SolverXPBD>(d, frames);
}

// --- load + validate + run. Returns a process exit code. ------------------

inline int loadAndRun(const std::string& path, UInt frames) {
    using nlohmann::json;

    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open scene file: '" << path << "'\n";
        return 1;
    }

    json j;
    try {
        j = json::parse(in);
    } catch (const json::parse_error& e) {
        std::cerr << "malformed JSON in '" << path << "': " << e.what() << "\n";
        return 1;
    }

    SceneData d;
    try {
        d.name     = j.value("name", std::string("unnamed"));
        d.solver   = j.value("solver", std::string());
        d.broad    = j.value("broad_phase", std::string());
        d.narrow   = j.value("narrow_phase", std::string());
        d.dt       = j.value("dt", Real(1) / 60);
        // Read substeps signed: nlohmann static_casts a negative JSON integer
        // straight into UInt (no range check), which would wrap to ~4e9.
        const long long ss = j.value("substeps", 1LL);
        if (ss < 1 || ss > 1000000) {
            std::cerr << "invalid substeps: " << ss << " (must be 1..1000000)\n";
            return 1;
        }
        d.substeps = static_cast<UInt>(ss);
    } catch (const json::exception& e) {
        std::cerr << "invalid scene field in '" << path << "': " << e.what() << "\n";
        return 1;
    }

    if (d.solver != "explicit" && d.solver != "xpbd" && d.solver != "pbd") {
        std::cerr << "unknown solver: '" << d.solver << "' (expected explicit|xpbd|pbd)\n";
        return 1;
    }
    if (d.broad != "lbvh" && d.broad != "spatial_hash") {
        std::cerr << "unknown broad_phase: '" << d.broad << "' (expected lbvh|spatial_hash)\n";
        return 1;
    }
    if (d.narrow != "exhaustive" && d.narrow != "proximity") {
        std::cerr << "unknown narrow_phase: '" << d.narrow << "' (expected exhaustive|proximity)\n";
        return 1;
    }
    if (!(d.dt > Real(0))) {
        std::cerr << "invalid dt: " << d.dt << " (must be > 0)\n";
        return 1;
    }

    if (!j.contains("objects") || !j["objects"].is_array()) {
        std::cerr << "scene '" << path << "': missing or non-array 'objects'\n";
        return 1;
    }
    for (const auto& o : j["objects"]) {
        std::string oname;
        std::string kindStr;
        UInt        verts = 0;
        try {
            oname   = o.value("name", std::string("obj"));
            kindStr = o.value("kind", std::string());
            verts   = o.value("vertices", UInt(0));
        } catch (const json::exception& e) {
            std::cerr << "invalid object field in '" << path << "': " << e.what() << "\n";
            return 1;
        }
        ObjectKind kind;
        if (!parseKind(kindStr, kind)) {
            std::cerr << "object '" << oname << "': bad kind '" << kindStr
                      << "' (expected cloth|rigid|floor)\n";
            return 1;
        }
        d.objects.push_back({ std::move(oname), kind, verts });
    }

    std::cout << "== load scene '" << d.name << "' from " << path << " ==\n"
              << "    solver=" << d.solver << " broad=" << d.broad
              << " narrow=" << d.narrow << "\n";
    dispatch(d, frames);
    return 0;
}

} // namespace scene_json

#endif // SCENE_JSON_HPP
