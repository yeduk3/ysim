#pragma once

#include "tinym.hpp"

#include <string>

namespace mesh_inspector {

struct MeshInspectorWindowState {
    bool open = true;
    std::string status_message;
    bool behavior_change_requested = false;
    int pending_behavior_mesh_id = -1;
    int pending_behavior_index = -1;
};

struct MeshInspectorTarget {
    int mesh_id = -1;
    int behavior_index = -1;
    bool fast_grid_supported = false;
    const char* behavior_label = nullptr;
    const char* shape_label = nullptr;
    tinym::vec3* base_color = nullptr;
    float* cloth_stretch = nullptr;
    float* cloth_shear = nullptr;
    float* cloth_bend = nullptr;
    float* cloth_thickness = nullptr;
    float* fast_stretch_rest = nullptr;
    float* fast_shear_rest = nullptr;
    float* fast_bend_rest = nullptr;
    float* fast_kstretch = nullptr;
    float* fast_kshear = nullptr;
    float* fast_kbend = nullptr;
    float* fast_thickness = nullptr;
};

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
);

} // namespace mesh_inspector
