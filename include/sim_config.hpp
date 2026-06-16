#ifndef YSIM_SIM_CONFIG_HPP
#define YSIM_SIM_CONFIG_HPP

// RunConfig — a single self-describing JSON config that drives one simulator
// run: scene + solver timing + engine (backend/system) + profiling output.
//
// It is a *superset* of scene_format::SceneSnapshot read from the SAME JSON
// root: the existing "objects"/"environment"/"simulation" blocks are parsed
// by scene_format unchanged, and two additive, fully-optional blocks are
// layered on top:
//
//   "engine":  { "backend": "METAL", "system": "Explicit" }   (req 6)
//   "profile": { "enabled": false, "frames": 30,
//                "realtime_sync": false, "output_path": "" }   (req 2)
//
// Because both blocks are optional with engine-default fallbacks, every
// pre-existing *.ysim.json scene loads as a valid RunConfig (req 1). The
// scene_format parser ignores unknown keys, so adding them does not bump the
// format version or break old tooling.
//
// Backend-independent (no Metal/GL/tinym) — same constraint as
// scene_format.hpp, so the test harness can parse/validate configs without a
// GPU device. The actual backend/system *match* check (does this build wire
// the requested backend?) lives in the builder, which knows the compiled-in
// types; here we only reject names that are not recognized at all.

#include <string>

#include <nlohmann/json.hpp>

#include "scene_format.hpp"

namespace sim_config {

using scene_format::LoadWarnings;
using scene_format::Result;

// Compile-time engine selection (req 6). Backend/system are still chosen at
// compile time in this engine — these strings record the *intent* in the
// config so a run is self-describing and a future runtime selector has a
// hook. The builder fails loud if a config asks for a backend/system this
// build did not wire.
struct EngineConfig {
    std::string backend = "METAL";    // METAL | CPU | CUDA
    std::string system  = "Explicit"; // Explicit (only system wired today)
};

// Profiling output (req 2). When enabled, a `--scene` run collects `frames`
// profiled frames then writes the CSV to `outputPath` (or, when empty, a
// default path derived from the scene file stem — see defaultProfilePath).
// realtimeSync mirrors the GUI "실시간 동기화" toggle: false → one fixed step
// per render frame (pure compute, the profiling default).
struct ProfileConfig {
    bool enabled = false;
    int  frames = 30;
    bool realtimeSync = false;
    std::string outputPath;  // empty → defaultProfilePath(stem, frames)
};

struct RunConfig {
    scene_format::SceneSnapshot scene;
    EngineConfig engine;
    ProfileConfig profile;
    LoadWarnings warnings;  // mirrors scene.warnings + engine/profile warnings
};

inline bool isKnownBackend(const std::string& b) {
    return b == "METAL" || b == "CPU" || b == "CUDA";
}

inline bool isKnownSystem(const std::string& s) {
    return s == "Explicit";
}

// ----------------------------------------------------------------- encode ---
inline nlohmann::json toJson(const EngineConfig& e) {
    nlohmann::json j;
    j["backend"] = e.backend;
    j["system"] = e.system;
    return j;
}

inline nlohmann::json toJson(const ProfileConfig& p) {
    nlohmann::json j;
    j["enabled"] = p.enabled;
    j["frames"] = p.frames;
    j["realtime_sync"] = p.realtimeSync;
    j["output_path"] = p.outputPath;
    return j;
}

inline nlohmann::json toJson(const RunConfig& c) {
    // Start from the scene snapshot so objects/environment/simulation are
    // byte-for-byte what scene_format would emit, then layer the two new
    // blocks on. A RunConfig saved this way round-trips through BOTH
    // scene_format::fromJson (sees a valid scene, ignores engine/profile)
    // and sim_config::fromJson (sees the full config).
    nlohmann::json j = scene_format::toJson(c.scene);
    j["engine"] = toJson(c.engine);
    j["profile"] = toJson(c.profile);
    return j;
}

inline std::string toString(const RunConfig& c) { return toJson(c).dump(2); }

// ----------------------------------------------------------------- decode ---
inline Result<EngineConfig> engineFromJson(const nlohmann::json& j) {
    using R = Result<EngineConfig>;
    EngineConfig e;
    if (auto it = j.find("backend"); it != j.end()) {
        if (!it->is_string()) return R::fail("engine.backend must be a string");
        e.backend = it->get<std::string>();
        if (!isKnownBackend(e.backend))
            return R::fail("unknown engine.backend '" + e.backend +
                           "' (expected METAL | CPU | CUDA)");
    }
    if (auto it = j.find("system"); it != j.end()) {
        if (!it->is_string()) return R::fail("engine.system must be a string");
        e.system = it->get<std::string>();
        if (!isKnownSystem(e.system))
            return R::fail("unknown engine.system '" + e.system +
                           "' (expected Explicit)");
    }
    return R::success(e);
}

inline Result<ProfileConfig> profileFromJson(const nlohmann::json& j) {
    using R = Result<ProfileConfig>;
    ProfileConfig p;
    if (auto it = j.find("enabled"); it != j.end()) {
        if (!it->is_boolean()) return R::fail("profile.enabled must be a boolean");
        p.enabled = it->get<bool>();
    }
    if (auto it = j.find("frames"); it != j.end()) {
        if (!it->is_number_integer()) return R::fail("profile.frames must be an integer");
        int v = it->get<int>();
        if (v < 1) return R::fail("profile.frames must be >= 1");
        p.frames = v;
    }
    if (auto it = j.find("realtime_sync"); it != j.end()) {
        if (!it->is_boolean()) return R::fail("profile.realtime_sync must be a boolean");
        p.realtimeSync = it->get<bool>();
    }
    if (auto it = j.find("output_path"); it != j.end()) {
        if (!it->is_string()) return R::fail("profile.output_path must be a string");
        p.outputPath = it->get<std::string>();
    }
    return R::success(p);
}

inline Result<RunConfig> fromJson(const nlohmann::json& j) {
    using R = Result<RunConfig>;
    RunConfig c;
    // Scene block: delegate to scene_format (strict format-version + object
    // validation). Engine/profile keys are simply unseen by it.
    auto sr = scene_format::fromJson(j);
    if (!sr.ok) return R::fail(sr.error.message);
    c.scene = std::move(sr.value);
    c.warnings = c.scene.warnings;

    if (auto it = j.find("engine"); it != j.end()) {
        if (!it->is_object()) return R::fail("'engine' must be a JSON object");
        auto er = engineFromJson(*it);
        if (!er.ok) return R::fail(er.error.message);
        c.engine = er.value;
    }
    if (auto it = j.find("profile"); it != j.end()) {
        if (!it->is_object()) return R::fail("'profile' must be a JSON object");
        auto pr = profileFromJson(*it);
        if (!pr.ok) return R::fail(pr.error.message);
        c.profile = pr.value;
    }
    return R::success(std::move(c));
}

inline Result<RunConfig> parseString(const std::string& text) {
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch (const nlohmann::json::parse_error& e) {
        return Result<RunConfig>::fail(std::string("JSON parse error: ") + e.what());
    }
    return fromJson(j);
}

inline Result<RunConfig> readFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return Result<RunConfig>::fail("cannot open '" + path + "' for reading");
    std::stringstream ss;
    ss << in.rdbuf();
    return parseString(ss.str());
}

inline bool writeToFile(const RunConfig& c, const std::string& path,
                        std::string* error = nullptr) {
    std::ofstream out(path);
    if (!out) {
        if (error) *error = "cannot open '" + path + "' for writing";
        return false;
    }
    out << toJson(c).dump(2);
    out.close();
    if (!out) {
        if (error) *error = "write to '" + path + "' failed";
        return false;
    }
    return true;
}

// Strip directory + extension from a path → bare stem. "/a/b/flag.json" →
// "flag"; "scene.ysim.json" → "scene.ysim" (only the LAST extension is cut,
// matching the convention that profiles are named after the whole scene file).
inline std::string pathStem(const std::string& path) {
    auto slash = path.find_last_of('/');
    std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
    auto dot = base.find_last_of('.');
    return (dot == std::string::npos) ? base : base.substr(0, dot);
}

// Default profile CSV path when profile.outputPath is empty (req 2 — "기본
// 템플릿"). Mirrors the convention already used by ad-hoc profiling runs
// (profiles/<name>-<N>f.csv). `projectRoot` is the YSIM_PROJECT_ROOT prefix
// (may be empty → relative path).
inline std::string defaultProfilePath(const std::string& projectRoot,
                                      const std::string& sceneStem,
                                      int frames) {
    std::string dir = projectRoot.empty() ? "profiles"
                                          : (projectRoot + "/profiles");
    std::string stem = sceneStem.empty() ? "scene" : sceneStem;
    return dir + "/" + stem + "-" + std::to_string(frames) + "f.csv";
}

// Sidecar scene-config path next to a profile CSV (req 4). "<dir>/<stem>.csv"
// → "<dir>/<stem>.scene.json". A non-.csv path just gets ".scene.json"
// appended after its last extension is stripped.
inline std::string sidecarScenePath(const std::string& csvPath) {
    auto dot = csvPath.find_last_of('.');
    auto slash = csvPath.find_last_of('/');
    std::string base =
        (dot != std::string::npos && (slash == std::string::npos || dot > slash))
            ? csvPath.substr(0, dot)
            : csvPath;
    return base + ".scene.json";
}

}  // namespace sim_config

#endif  // YSIM_SIM_CONFIG_HPP
