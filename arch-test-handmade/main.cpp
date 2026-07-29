// arch_test_handmade -- scene-driven architecture experiment.
//
//   ./arch_test_handmade [--scene] [scene | path.json] [frames]
//   ./arch_test_handmade --list          (also: list, help, --help, -h)
//   ./arch_test_handmade --selftest      (PBD solver checks)
//
// A scene comes from one of two sources, both consumed by the SAME generic
// SimulatorBasic:
//   - a registry name (cpp scene, SceneSettings.hpp), or
//   - a *.json file (SceneJson.hpp), parsed at runtime into a SceneRuntime<...>.
// Either way the scene alone decides the solver, the broad/narrow collision
// phases, and the objects. With no argument, the floor-only "default" runs.

#include <charconv>
#include <iostream>
#include <string>
#include <string_view>

#include "PbdSolver.hpp"
#include "SceneJson.hpp"
#include "Scenes.hpp"

namespace {

constexpr std::string_view kDefaultScene = "default";
constexpr UInt             kDefaultFrames = 2;

void printScenes(std::ostream& os) {
    os << "available scenes:\n";
    for (const auto& e : sceneRegistry)
        os << "  " << e.name << "  -  " << e.description << "\n";
}

bool isListFlag(std::string_view a) {
    return a == "--list" || a == "list" || a == "--help" || a == "-h" || a == "help";
}

bool hasJsonExt(std::string_view a) {
    return a.size() >= 5 && a.substr(a.size() - 5) == ".json";
}

#ifndef ATH_SOURCE_DIR
#define ATH_SOURCE_DIR "."
#endif

// Registry JSON paths are source-relative so the binary works from any cwd.
std::string resolveScenePath(std::string_view rel) {
    return std::string(ATH_SOURCE_DIR) + "/" + std::string(rel);
}

// Run the scene registered under `name`. `found` reports whether the name
// exists at all; the return value is the process exit code.
int runByName(std::string_view name, UInt frames, bool& found) {
    for (const auto& e : sceneRegistry) {
        if (e.name != name) continue;
        found = true;
        if (e.run) { e.run(frames); return 0; }
        return scene_json::loadAndRun(resolveScenePath(e.jsonPath), frames);
    }
    found = false;
    return 1;
}

} // namespace

int main(int argc, char** argv) {
    bool found = false;

    // `--scene <x>` is accepted alongside a bare `<x>`, matching ysim's CLI.
    int argi = 1;
    if (argc >= 2 && std::string_view(argv[1]) == "--scene") {
        if (argc < 3) {
            std::cerr << "--scene requires a value\n";
            return 1;
        }
        argi = 2;
    }

    // No scene argument -> run the floor-only default scene.
    if (argc <= argi) {
        const int rc = runByName(kDefaultScene, kDefaultFrames, found);
        if (!found) {
            std::cerr << "default scene '" << kDefaultScene << "' is not registered\n";
            return 1;
        }
        return rc;
    }

    const std::string_view arg1 = argv[argi];

    // Explicit list/help flag -> print the scene listing and exit 0.
    if (isListFlag(arg1)) {
        printScenes(std::cout);
        return 0;
    }

    // Self-test for the one solver that does real physics (SolverPBD).
    if (arg1 == "--selftest") return pbd_selftest::run();

    // Optional frame count.
    UInt frames = kDefaultFrames;
    if (argc > argi + 1) {
        const std::string_view f = argv[argi + 1];
        const auto [ptr, ec] = std::from_chars(f.data(), f.data() + f.size(), frames);
        if (ec != std::errc{} || ptr != f.data() + f.size()) {
            std::cerr << "invalid frame count: '" << f << "'\n";
            return 1;
        }
    }

    // A *.json argument is a JSON scene file; anything else is a registry name.
    if (hasJsonExt(arg1))
        return scene_json::loadAndRun(std::string(arg1), frames);

    const int rc = runByName(arg1, frames, found);
    if (found) return rc;

    std::cerr << "unknown scene: '" << arg1 << "'\n";
    printScenes(std::cerr);
    return 1;
}
