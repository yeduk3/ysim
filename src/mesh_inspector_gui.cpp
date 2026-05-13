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
        // D-027: material edits write through *target.base_color / metallic /
        // ... in place for live preview (renderer reads mesh.material.*
        // every frame), then fire on_material_edit so Simulator::setMaterial
        // also writes pendingMaterials[id] for re-pack survival via D-025.
        bool haveMaterial = target.metallic && target.roughness
                         && target.specular_weight && target.emission_color
                         && target.on_material_edit;

        auto fireMaterialEdit = [&]() {
            if (!haveMaterial) return;
            target.on_material_edit(target.mesh_id,
                                    *target.base_color,
                                    *target.metallic,
                                    *target.roughness,
                                    *target.specular_weight,
                                    *target.emission_color);
        };

        if (ImGui::ColorEdit3("Base Color", target.base_color->v)) fireMaterialEdit();

        if (haveMaterial) {
            if (ImGui::SliderFloat("Metallic", target.metallic, 0.0f, 1.0f))         fireMaterialEdit();
            if (ImGui::SliderFloat("Roughness", target.roughness, 0.0f, 1.0f))       fireMaterialEdit();
            if (ImGui::SliderFloat("Specular Weight", target.specular_weight, 0.0f, 1.0f)) fireMaterialEdit();
            if (ImGui::ColorEdit3("Emission", target.emission_color->v))             fireMaterialEdit();
        }

        if (ImGui::Button("Reset Color")) {
            *target.base_color = tinym::vec3(1.0f);
            fireMaterialEdit();
            state.status_message = "Base color reset to white.";
        }
    }

    if (target.transform_position && target.on_translate) {
        if (ImGui::CollapsingHeader("Transform", ImGuiTreeNodeFlags_DefaultOpen)) {
            float p[3] = { target.transform_position->x,
                           target.transform_position->y,
                           target.transform_position->z };
            ImGui::InputFloat3("Position", p);
            // IsItemDeactivatedAfterEdit fires once on commit (Enter / Tab /
            // focus loss), not per keystroke — keeps state.x mutation off the
            // typing critical path.
            if (ImGui::IsItemDeactivatedAfterEdit()) {
                target.on_translate(target.mesh_id,
                                    tinym::vec3(p[0], p[1], p[2]));
                state.status_message = "Position updated.";
            }
        }
    }

    if (target.rotation_wxyz && target.on_rotate) {
        if (ImGui::CollapsingHeader("Rotation", ImGuiTreeNodeFlags_DefaultOpen)) {
            // D-035: mode toggle selects how the rotation is shown / edited.
            // Canonical storage is still Quat on the mesh; Euler XYZ and
            // Axis-Angle are display-time conversions via the helpers in
            // MeshInspectorWindow.hpp. Mode state is per-window
            // (state.rotation_input_mode); the mesh's `*target.rotation_wxyz`
            // is the source of truth that all three widgets read from.
            ImGui::Combo("Mode", &state.rotation_input_mode,
                         "Quat\0Euler XYZ (deg)\0Axis-Angle (deg)\0\0");

            const float currWxyz[4] = {
                target.rotation_wxyz[0], target.rotation_wxyz[1],
                target.rotation_wxyz[2], target.rotation_wxyz[3]
            };

            if (state.rotation_input_mode == 0) {
                // Quat mode — direct 4-float edit, pre-D-035 default.
                float q[4] = { currWxyz[0], currWxyz[1], currWxyz[2], currWxyz[3] };
                ImGui::InputFloat4("Quat (w,x,y,z)", q);
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    target.on_rotate(target.mesh_id, q[0], q[1], q[2], q[3]);
                    state.status_message = "Rotation updated (quat).";
                }
            } else if (state.rotation_input_mode == 1) {
                // Euler XYZ mode — display radians-derived degrees;
                // on commit convert back to wxyz.
                float xyzDeg[3];
                quatWxyzToEulerXYZDeg(currWxyz, xyzDeg);
                ImGui::InputFloat3("Euler XYZ (deg)", xyzDeg);
                if (ImGui::IsItemDeactivatedAfterEdit()) {
                    float newWxyz[4];
                    eulerXYZDegToQuatWxyz(xyzDeg, newWxyz);
                    target.on_rotate(target.mesh_id,
                                     newWxyz[0], newWxyz[1], newWxyz[2], newWxyz[3]);
                    state.status_message = "Rotation updated (Euler XYZ deg).";
                }
            } else {
                // Axis-Angle mode — axis (vec3) + angle (degrees).
                // Edits to either widget commit a fresh wxyz on commit.
                float axis[3];
                float angleDeg = 0.0f;
                quatWxyzToAxisAngleDeg(currWxyz, axis, angleDeg);
                bool axisCommit = false;
                bool angleCommit = false;
                ImGui::InputFloat3("Axis (x,y,z)", axis);
                if (ImGui::IsItemDeactivatedAfterEdit()) axisCommit = true;
                ImGui::InputFloat("Angle (deg)", &angleDeg);
                if (ImGui::IsItemDeactivatedAfterEdit()) angleCommit = true;
                if (axisCommit || angleCommit) {
                    float newWxyz[4];
                    axisAngleDegToQuatWxyz(axis, angleDeg, newWxyz);
                    target.on_rotate(target.mesh_id,
                                     newWxyz[0], newWxyz[1], newWxyz[2], newWxyz[3]);
                    state.status_message = "Rotation updated (axis-angle).";
                }
            }
        }
    }

    if (!state.status_message.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("%s", state.status_message.c_str());
    }

    ImGui::End();
}

} // namespace mesh_inspector
