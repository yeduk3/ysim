#pragma once

#include "tinym.hpp"

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
};

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
);

} // namespace mesh_inspector
