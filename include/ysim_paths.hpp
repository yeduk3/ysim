#ifndef YSIM_PATHS_HPP
#define YSIM_PATHS_HPP

// Centralized runtime path resolution so the binary is cwd-independent.
//
// Two distinct roots:
//   · runtimeDir()  — directory of the running executable. Compiled runtime
//     artifacts (default.metallib, GLSL shaders) live NEXT TO the binary and
//     are loaded relative to it, never the cwd. This is what lets `build/ysim`
//     run from any working directory (and makes the build folder relocatable).
//   · assetRoot()   — external assets (models, icons, BVH cache) referenced
//     from the project source tree directly (NOT copied into the build).
//     Compiled in as YSIM_PROJECT_ROOT; overridable by env for relocated use.
//
// Header-only; values resolved once (function-local statics).

#include <cstdlib>
#include <filesystem>
#include <string>

#ifdef __APPLE__
#include <climits>
#include <cstdint>
#include <mach-o/dyld.h>
#include <stdlib.h>  // realpath
#endif

namespace ysim_paths {

inline bool isAbsolute(const std::string& p) {
    return !p.empty() && p[0] == '/';
}

// Directory of the running executable (no trailing slash). Env override
// YSIM_RUNTIME_DIR wins; falls back to "." (legacy cwd) if it cannot be
// resolved on this platform.
inline const std::string& runtimeDir() {
    static const std::string dir = []() -> std::string {
        if (const char* o = std::getenv("YSIM_RUNTIME_DIR"))
            return std::string(o);
#ifdef __APPLE__
        char buf[PATH_MAX];
        uint32_t sz = sizeof(buf);
        if (_NSGetExecutablePath(buf, &sz) == 0) {
            char real[PATH_MAX];
            const char* exe = realpath(buf, real) ? real : buf;
            std::string s(exe);
            auto slash = s.find_last_of('/');
            if (slash != std::string::npos) return s.substr(0, slash);
        }
#endif
        return ".";
    }();
    return dir;
}

// Resolve a runtime-data file (metallib, shader). Absolute paths pass
// through; relative names anchor at runtimeDir().
inline std::string runtimeFile(const std::string& name) {
    if (isAbsolute(name)) return name;
    return runtimeDir() + "/" + name;
}

// Project source root. Env override YSIM_PROJECT_ROOT_DIR wins; otherwise the
// compiled-in YSIM_PROJECT_ROOT; "." as a last resort.
inline const std::string& projectRoot() {
    static const std::string root = []() -> std::string {
        if (const char* o = std::getenv("YSIM_PROJECT_ROOT_DIR"))
            return std::string(o);
#ifdef YSIM_PROJECT_ROOT
        return std::string(YSIM_PROJECT_ROOT);
#else
        return ".";
#endif
    }();
    return root;
}

// External-asset root (<projectRoot>/assets). Env override YSIM_ASSET_ROOT
// wins, so a relocated install can point assets elsewhere.
inline std::string assetRoot() {
    if (const char* o = std::getenv("YSIM_ASSET_ROOT")) return std::string(o);
    return projectRoot() + "/assets";
}

// Resolve an asset file relative to assetRoot(). Absolute paths pass through.
inline std::string assetFile(const std::string& rel) {
    if (isAbsolute(rel)) return rel;
    return assetRoot() + "/" + rel;
}

// Resolve a user-supplied INPUT path (e.g. `--scene foo.json`). Tries the
// path as-given first (cwd-relative or absolute, the standard CLI behavior),
// then — for relative paths — anchored at the project root, so a repo-relative
// path like `scenes/x.json` works whether ysim is launched from the repo root
// or from build/. Returns the first that exists; otherwise the original path
// (so a not-found error reports exactly what the user typed).
inline std::string resolveInput(const std::string& path) {
    namespace fs = std::filesystem;
    std::error_code ec;
    if (fs::exists(path, ec)) return path;
    if (!isAbsolute(path)) {
        std::string atRoot = projectRoot() + "/" + path;
        if (fs::exists(atRoot, ec)) return atRoot;
    }
    return path;
}

}  // namespace ysim_paths

#endif  // YSIM_PATHS_HPP
