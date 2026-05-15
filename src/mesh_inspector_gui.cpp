#include "MeshInspectorWindow.hpp"

#include "imgui.h"

namespace mesh_inspector {

// Behavior combo for the right-panel: only two user-visible choices —
// 옷감 (Cloth) and 강체 (Rigid). The dropdown index passed back to
// on_behavior_change is the canonical 4-entry index defined by
// production:
//   0 = Float, 1 = TriangularCloth, 2 = FastGridCloth, 3 = Rigid.
// "옷감" auto-picks FastGridCloth when the mesh has a square-regular
// grid topology (target.grid_eligible), else TriangularCloth.
static int clothBehaviorIndex(bool gridEligible) {
    return gridEligible ? 2 : 1;
}

// Map the canonical 4-entry index back to a 2-entry combo position:
//   옷감 = 0 (both TriangularCloth and FastGridCloth collapse here)
//   강체 = 1
//   anything else (Float, etc.) → -1 = "(미설정)"
static int simpleComboFromCanonical(int canonical) {
    if (canonical == 1 || canonical == 2) return 0;
    if (canonical == 3) return 1;
    return -1;
}

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
) {
    if (!state.open) return;

    if (!ImGui::Begin("물체", nullptr,
                      ImGuiWindowFlags_NoMove |
                      ImGuiWindowFlags_NoResize |
                      ImGuiWindowFlags_NoCollapse |
                      ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::End();
        return;
    }

    // No-selection branch: surface the three Add-Object buttons (they
    // open the same modals that used to hang off the File / Create menus).
    if (target.mesh_id < 0 || target.base_color == nullptr) {
        ImGui::TextDisabled("선택된 물체가 없습니다.");
        ImGui::Separator();
        ImGui::TextUnformatted("물체 추가");
        ImGui::Spacing();
        const ImVec2 btn(-FLT_MIN, 0);
        if (ImGui::Button("정육면체 추가...", btn) && target.on_request_add_cube) {
            target.on_request_add_cube();
        }
        if (ImGui::Button("구 추가...", btn) && target.on_request_add_sphere) {
            target.on_request_add_sphere();
        }
        if (ImGui::Button("OBJ 파일 추가...", btn) && target.on_request_add_import) {
            target.on_request_add_import();
        }
        ImGui::End();
        return;
    }

    ImGui::Text("물체 ID: %d", target.mesh_id);
    if (target.shape_label) ImGui::Text("형상: %s", target.shape_label);
    ImGui::Separator();

    // ─── Material (옷감 / 강체) ─────────────────────────────────────
    if (target.current_behavior_index >= 0 && target.on_behavior_change) {
        static const char* kSimpleLabels[] = { "옷감", "강체" };
        int simpleIdx = simpleComboFromCanonical(target.current_behavior_index);
        const char* preview = (simpleIdx >= 0) ? kSimpleLabels[simpleIdx] : "(미설정)";
        if (ImGui::BeginCombo("소재", preview)) {
            // 옷감 row
            bool clothSelected = (simpleIdx == 0);
            if (ImGui::Selectable(kSimpleLabels[0], clothSelected)) {
                int newCanonical = clothBehaviorIndex(target.grid_eligible);
                bool ok = target.on_behavior_change(target.mesh_id, newCanonical);
                state.status_message = ok
                    ? std::string("소재가 옷감으로 변경됨.")
                    : std::string("옷감 변경이 거부됨.");
            }
            if (clothSelected) ImGui::SetItemDefaultFocus();
            // 강체 row
            bool rigidSelected = (simpleIdx == 1);
            if (ImGui::Selectable(kSimpleLabels[1], rigidSelected)) {
                bool ok = target.on_behavior_change(target.mesh_id, 3);
                state.status_message = ok
                    ? std::string("소재가 강체로 변경됨.")
                    : std::string("강체 변경이 거부됨.");
            }
            if (rigidSelected) ImGui::SetItemDefaultFocus();
            ImGui::EndCombo();
        }
    }

    // ─── Transform: position (Vec3 input, commit-on-deactivate) ───────
    if (target.transform_position && target.on_translate) {
        float p[3] = { target.transform_position->x,
                       target.transform_position->y,
                       target.transform_position->z };
        ImGui::InputFloat3("위치", p);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            target.on_translate(target.mesh_id,
                                tinym::vec3(p[0], p[1], p[2]));
            state.status_message = "위치가 갱신됨.";
        }
    }

    // ─── Rotation: Euler XYZ (degrees) ────────────────────────────────
    if (target.rotation_wxyz && target.on_rotate) {
        const float currWxyz[4] = {
            target.rotation_wxyz[0], target.rotation_wxyz[1],
            target.rotation_wxyz[2], target.rotation_wxyz[3]
        };
        float xyzDeg[3];
        quatWxyzToEulerXYZDeg(currWxyz, xyzDeg);
        ImGui::InputFloat3("회전 (도)", xyzDeg);
        if (ImGui::IsItemDeactivatedAfterEdit()) {
            float newWxyz[4];
            eulerXYZDegToQuatWxyz(xyzDeg, newWxyz);
            target.on_rotate(target.mesh_id,
                             newWxyz[0], newWxyz[1], newWxyz[2], newWxyz[3]);
            state.status_message = "회전이 갱신됨.";
        }
    }

    // ─── 외관 (v1 OpenPBR subset) ─────────────────────────────────────
    // 모든 위젯은 in-place로 mesh.material.*을 갱신하고 on_material_edit
    // 콜백을 통해 Simulator::setMaterial로 라우팅되어 pendingMaterials
    // 까지 써둠 → Scene::pack 재구성 시 재적용 (D-025 / D-027).
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
    if (ImGui::CollapsingHeader("외관", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::ColorEdit3("기본 색상", target.base_color->v)) fireMaterialEdit();
        if (haveMaterial) {
            if (ImGui::SliderFloat("금속성", target.metallic, 0.0f, 1.0f, "%.2f"))
                fireMaterialEdit();
            if (ImGui::SliderFloat("거칠기", target.roughness, 0.0f, 1.0f, "%.2f"))
                fireMaterialEdit();
            if (ImGui::SliderFloat("스페큘러", target.specular_weight, 0.0f, 1.0f, "%.2f"))
                fireMaterialEdit();
            if (ImGui::ColorEdit3("발광 색상", target.emission_color->v))
                fireMaterialEdit();
        }
        if (ImGui::Button("기본 색상 초기화", ImVec2(-FLT_MIN, 0))) {
            *target.base_color = tinym::vec3(1.0f);
            fireMaterialEdit();
            state.status_message = "기본 색상이 흰색으로 초기화됨.";
        }
    }

    // ─── Per-object environment-force toggles ─────────────────────────
    // Checkbox writes through the aliased GeneralMesh field for live
    // effect; the on_env_toggle_change callback mirrors the new value
    // into the request side so it survives Simulator::reset().
    auto fireEnvToggle = [&]() {
        if (!target.on_env_toggle_change) return;
        bool g = target.apply_gravity ? *target.apply_gravity : true;
        bool w = target.apply_wind    ? *target.apply_wind    : true;
        target.on_env_toggle_change(target.mesh_id, g, w);
    };
    if (target.apply_gravity) {
        if (ImGui::Checkbox("중력 적용", target.apply_gravity)) fireEnvToggle();
    }
    if (target.apply_wind) {
        if (ImGui::Checkbox("바람 적용", target.apply_wind)) fireEnvToggle();
    }

    // ─── Delete ───────────────────────────────────────────────────────
    if (target.on_delete) {
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.18f, 0.18f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.80f, 0.25f, 0.25f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.55f, 0.10f, 0.10f, 1.0f));
        if (ImGui::Button("제거", ImVec2(-FLT_MIN, 0))) {
            target.on_delete(target.mesh_id);
            state.status_message = "물체가 제거됨.";
        }
        ImGui::PopStyleColor(3);
    }

    if (!state.status_message.empty()) {
        ImGui::Spacing();
        ImGui::TextDisabled("%s", state.status_message.c_str());
    }

    ImGui::End();
}

} // namespace mesh_inspector
