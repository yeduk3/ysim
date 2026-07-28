#ifndef SCENES_HPP
#define SCENES_HPP

// Dispatch machinery (engine side). Turns a scene value into a fully
// compile-time-typed simulator run via the single generic SimulatorBasic.
//
// Two front doors funnel into the one runScene<SceneT>(scene, frames):
//   - the sceneRegistry below, keyed by name, for cpp scenes (SceneSettings.hpp)
//   - the JSON loader (SceneJson.hpp), which builds a SceneRuntime<...> value
// The cpp scene definitions live in SceneSettings.hpp -- to add one, define it
// there and append one entry<...>() to sceneRegistry.

#include <array>
#include <string_view>
#include <utility>

#include "SceneSettings.hpp"

// The single generic run path: any SceneConfig-satisfying scene value.
template <class SceneT>
void runScene(SceneT scene, UInt frames) {
    static_assert(SceneConfig<SceneT>, "scene must satisfy SceneConfig");
    std::cout << "== build components for scene '" << scene.name() << "' ==\n";
    SimulatorBasic<SceneT> sim{ std::move(scene) };
    static_assert(Simulator<SimulatorBasic<SceneT>>, "simulator must satisfy Simulator");
    sim.setup();
    sim.run(frames);
}

// Registry adapter: default-construct a cpp scene, then run it.
template <class SceneT>
void runRegisteredScene(UInt frames) {
    runScene(SceneT{}, frames);
}

using RunFn = void (*)(UInt frames);

// A registry entry is either a cpp scene (`run` set) or a JSON scene file
// (`jsonPath` set). The registry stays constexpr either way -- a JSON entry
// carries only a path, and main.cpp hands it to scene_json::loadAndRun. That
// keeps Scenes.hpp free of any dependency on SceneJson.hpp (which includes
// this header).
struct SceneEntry {
    std::string_view name;
    std::string_view description;
    RunFn            run      = nullptr;   // cpp scene
    std::string_view jsonPath;             // JSON scene, relative to ATH_SOURCE_DIR
};

template <class SceneT>
constexpr SceneEntry entry(std::string_view description) {
    return { SceneT{}.name(), description, &runRegisteredScene<SceneT>, {} };
}

constexpr SceneEntry jsonEntry(std::string_view name, std::string_view description,
                               std::string_view path) {
    return { name, description, nullptr, path };
}

inline constexpr std::array<SceneEntry, 6> sceneRegistry = {{
    entry<SceneDefault>("explicit + LBVH + exhaustive; floor only (no-arg default)"),
    entry<SceneBasic>("explicit + LBVH + exhaustive; 1 cloth, 1 rigid"),
    entry<SceneClothDrop>("XPBD + LBVH + proximity; cloth onto a floor"),
    entry<SceneClothPbd>("PBD (REAL cpu physics) + LBVH + proximity; 20x20 sheet onto a floor"),
    entry<SceneRigidPile>("explicit + spatial-hash + exhaustive; 3 boxes"),
    jsonEntry("cloth_pbd_json", "PBD; 30x30 sheet -- loaded from a JSON file",
              "scenes/cloth_pbd.json"),
}};

#endif // SCENES_HPP
