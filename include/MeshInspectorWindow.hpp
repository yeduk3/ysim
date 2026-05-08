#pragma once

#include "tinym.hpp"

#include <functional>
#include <string>

namespace mesh_inspector {

struct MeshInspectorWindowState {
    bool open = true;
    std::string status_message;
};

struct MeshInspectorTarget {
    int mesh_id = -1;
    const char* behavior_label = nullptr;
    const char* shape_label = nullptr;
    tinym::vec3* base_color = nullptr;
    // BDD-003 inspector translate path. The pointer reads the mesh's current
    // transformPosition; the callback applies a finished edit through
    // Simulator::translateObject (which mutates state.x and state.xPrev in
    // tandem). Both must be set together to enable the InputFloat3 row.
    tinym::vec3* transform_position = nullptr;
    std::function<void(int, tinym::vec3)> on_translate;
};

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
);

} // namespace mesh_inspector
