#include "MeshInspectorWindow.hpp"

#include "imgui.h"

namespace mesh_inspector {

namespace {

constexpr const char* kBehaviorLabels[] = {
    "TriangularCloth",
    "FastGridCloth",
    "Float",
};

constexpr int kBehaviorCount = static_cast<int>(sizeof(kBehaviorLabels) / sizeof(kBehaviorLabels[0]));
constexpr int kFastGridBehaviorIndex = 1;

} // namespace

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
) {
    if (!state.open) return;

    ImGui::SetNextWindowSize(ImVec2(420, 540), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Mesh Inspector", &state.open)) {
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Selected mesh controls");

    if (target.mesh_id < 0 || target.base_color == nullptr) {
        ImGui::Separator();
        ImGui::TextDisabled("No mesh is currently selected.");
        ImGui::TextDisabled("Click a mesh in the viewport to edit its color.");
        if (!state.status_message.empty()) {
            ImGui::Spacing();
            ImGui::TextDisabled("%s", state.status_message.c_str());
        }
        ImGui::End();
        return;
    }

    ImGui::Separator();
    ImGui::Text("Mesh ID: %d", target.mesh_id);
    if (target.behavior_label) ImGui::Text("Behavior: %s", target.behavior_label);
    if (target.shape_label) ImGui::Text("Shape: %s", target.shape_label);

    if (target.behavior_index >= 0 && target.behavior_index < kBehaviorCount) {
        if (ImGui::BeginCombo("Behavior", kBehaviorLabels[target.behavior_index])) {
            for (int i = 0; i < kBehaviorCount; ++i) {
                const bool is_fast_grid = (i == kFastGridBehaviorIndex);
                const bool available = !is_fast_grid || target.fast_grid_supported;
                if (!available) ImGui::BeginDisabled();

                const bool selected = (i == target.behavior_index);
                if (ImGui::Selectable(kBehaviorLabels[i], selected)) {
                    state.behavior_change_requested = true;
                    state.pending_behavior_mesh_id = target.mesh_id;
                    state.pending_behavior_index = i;
                    state.status_message = "Behavior change queued.";
                }

                if (selected) ImGui::SetItemDefaultFocus();
                if (!available) ImGui::EndDisabled();
            }
            ImGui::EndCombo();
        }

        if (!target.fast_grid_supported) {
            ImGui::TextDisabled("FastGridCloth is only available for grid-based meshes.");
        }
    } else if (target.behavior_label) {
        ImGui::TextDisabled("Behavior editing is not available for this mesh.");
    }

    if (ImGui::CollapsingHeader("Behavior Parameters", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (target.behavior_index == 0) {
            ImGui::TextDisabled("Cloth spring tuning");
            if (target.cloth_stretch) ImGui::InputFloat("Stretch", target.cloth_stretch, 0.0f, 0.0f, "%.6g");
            if (target.cloth_shear) ImGui::InputFloat("Shear", target.cloth_shear, 0.0f, 0.0f, "%.6g");
            if (target.cloth_bend) ImGui::InputFloat("Bend", target.cloth_bend, 0.0f, 0.0f, "%.6g");
            if (target.cloth_thickness) ImGui::InputFloat("Thickness", target.cloth_thickness, 0.0f, 0.0f, "%.6g");
        } else if (target.behavior_index == 1) {
            ImGui::TextDisabled("Fast grid cloth tuning");
            if (target.fast_stretch_rest) ImGui::InputFloat("Stretch Rest", target.fast_stretch_rest, 0.0f, 0.0f, "%.6g");
            if (target.fast_shear_rest) ImGui::InputFloat("Shear Rest", target.fast_shear_rest, 0.0f, 0.0f, "%.6g");
            if (target.fast_bend_rest) ImGui::InputFloat("Bend Rest", target.fast_bend_rest, 0.0f, 0.0f, "%.6g");
            if (target.fast_kstretch) ImGui::InputFloat("Stretch Stiffness", target.fast_kstretch, 0.0f, 0.0f, "%.6g");
            if (target.fast_kshear) ImGui::InputFloat("Shear Stiffness", target.fast_kshear, 0.0f, 0.0f, "%.6g");
            if (target.fast_kbend) ImGui::InputFloat("Bend Stiffness", target.fast_kbend, 0.0f, 0.0f, "%.6g");
            if (target.fast_thickness) ImGui::InputFloat("Thickness", target.fast_thickness, 0.0f, 0.0f, "%.6g");
            ImGui::TextDisabled("Particle count is preserved by rebuilds and is not live-editable yet.");
        } else if (target.behavior_index == 2) {
            ImGui::TextDisabled("Float meshes have no spring parameters.");
            ImGui::TextDisabled("They are skipped by cloth collision resolution.");
        } else {
            ImGui::TextDisabled("No editable parameters for this behavior.");
        }
    }

    if (ImGui::CollapsingHeader("Appearance", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::ColorEdit3("Base Color", target.base_color->v);

        if (ImGui::Button("Reset Color")) {
            *target.base_color = tinym::vec3(1.0f);
            state.status_message = "Base color reset to white.";
        }
        ImGui::SameLine();
        ImGui::TextDisabled("More parameters can be added here later.");
    }

    if (!state.status_message.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("%s", state.status_message.c_str());
    }

    ImGui::End();
}

} // namespace mesh_inspector
