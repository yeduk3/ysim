#include "MeshInspectorWindow.hpp"

#include "imgui.h"

namespace mesh_inspector {

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
) {
    if (!state.open) return;

    ImGui::SetNextWindowSize(ImVec2(360, 220), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Mesh Inspector", &state.open)) {
        ImGui::End();
        return;
    }

    ImGui::TextUnformatted("Selected mesh controls");

    if (target.mesh_id < 0 || target.base_color == nullptr) {
        ImGui::Separator();
        ImGui::TextDisabled("No mesh is currently selected.");
        ImGui::TextDisabled("Click a mesh in the viewport to edit its color.");
        ImGui::End();
        return;
    }

    ImGui::Separator();
    ImGui::Text("Mesh ID: %d", target.mesh_id);
    if (target.behavior_label) ImGui::Text("Behavior: %s", target.behavior_label);
    if (target.shape_label) ImGui::Text("Shape: %s", target.shape_label);

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
