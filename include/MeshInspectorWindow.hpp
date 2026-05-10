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
    // FR-004 / D-021 inspector rotate path. The 4-float pointer reads the
    // mesh's rotationQuat as (w, x, y, z) — Quat is a {w, x, y, z} POD per
    // D-019 so &mesh.rotationQuat.w aliases a contiguous float[4]. The
    // callback applies a finished edit through Simulator::rotateObject
    // (which rotates state.x / state.xPrev around the transformPosition
    // pivot and writes the new absolute rotationQuat). Raw float pointer
    // avoids coupling this header to main.cpp's Quat type. Both fields
    // must be set together to enable the InputFloat4 row.
    float* rotation_wxyz = nullptr;
    std::function<void(int, float, float, float, float)> on_rotate;
};

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
);

} // namespace mesh_inspector
