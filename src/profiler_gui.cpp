#include "ProfilerWindow.hpp"

#include "imgui.h"

#include <algorithm>
#include <cstdio>
#include <vector>

namespace profiler {

namespace {

void drawSeries(const char* label, const std::vector<float>& values, float min_value = 0.0f) {
    if (values.empty()) {
        ImGui::TextDisabled("%s: no samples yet", label);
        return;
    }

    float max_value = *std::max_element(values.begin(), values.end());
    max_value = std::max(max_value, min_value + 0.001f);
    ImGui::PlotLines(label, values.data(), static_cast<int>(values.size()), 0, nullptr, min_value, max_value, ImVec2(0, 70));
}

} // namespace

void drawProfilerWindow(
    ProfilerWindowState& state,
    FrameProfiler& profiler,
    bool* pause,
    bool* debug_each_boxes,
    bool* debug_scene_box,
    bool* debug_collisions,
    bool* mesh_inspector_open,
    SceneCounts scene_counts,
    bool* use_segmented_bvh_query,
    bool* use_agglomerative_bvh,
    bool* enable_refit,
    bool* use_spatial_hashing,
    std::uint32_t* refit_substep_period,
    std::uint32_t* cd_substep_period,
    int* profile_level,
    bool* fused_refit_enlarge,
    bool* use_subobject_bvh,
    int*  subbvh_split_s,
    bool* use_multilevel_sh
) {
    if (!state.open) return;

    auto& history = profiler.history();
    const FrameSnapshot* latest = history.latestFrame();
    const auto& section_names = history.sectionNames();

    if (state.selected_section >= static_cast<int>(section_names.size()))
        state.selected_section = section_names.empty() ? 0 : static_cast<int>(section_names.size()) - 1;

    ImGui::SetNextWindowSize(ImVec2(520, 640), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Profiler", &state.open)) {
        ImGui::End();
        return;
    }

    if (latest) {
        ImGui::Text("FPS %.1f", latest->fps);
        ImGui::Text("Frame %.3f ms", latest->frame_ms);
        ImGui::Text("Sequence %llu", static_cast<unsigned long long>(latest->sequence));
    } else {
        ImGui::TextDisabled("No frame samples yet");
    }

    if (profile_level) {
        // None → no profiling (max sim loop). PerFrame → frame_ms +
        // physics/render split only. InFrame → full per-section breakdown.
        static const char* kLevels[] = {"None", "Per-Frame", "In-Frame"};
        if (*profile_level < 0) *profile_level = 0;
        if (*profile_level > 2) *profile_level = 2;
        ImGui::Combo("Profile Level", profile_level, kLevels, 3);
        ImGui::SameLine();
        ImGui::TextDisabled(
            *profile_level == 0 ? "[no overhead]"
          : *profile_level == 1 ? "[frame + phys/render]"
                                : "[all sections]");
    }

    ImGui::Separator();
    ImGui::Text("Scene: %d meshes / %d points / %d triangles",
                scene_counts.meshes,
                scene_counts.points,
                scene_counts.triangles);

    ImGui::Separator();
    ImGui::SliderFloat("History Seconds", &state.history_seconds, 1.0f, 6.0f, "%.1f");

    if (pause) ImGui::Checkbox("Pause", pause);
    if (debug_each_boxes) ImGui::Checkbox("Debug Each Boxes", debug_each_boxes);
    if (debug_scene_box) ImGui::Checkbox("Debug Scene Box", debug_scene_box);
    if (debug_collisions) ImGui::Checkbox("Debug Collisions", debug_collisions);
    if (mesh_inspector_open) ImGui::Checkbox("Mesh Inspector", mesh_inspector_open);
    if (use_spatial_hashing) {
        ImGui::Checkbox("Spatial Hashing", use_spatial_hashing);
        ImGui::SameLine();
        ImGui::TextDisabled(*use_spatial_hashing ? "[uniform grid]" : "[BVH broadphase]");
    }
    if (use_multilevel_sh) {
        ImGui::Checkbox("Multi-level Spatial Hash", use_multilevel_sh);
        ImGui::SameLine();
        ImGui::TextDisabled(*use_multilevel_sh ? "[hgrid, floor excluded]" : "[off]");
    }
    if (use_segmented_bvh_query) {
        ImGui::Checkbox("BVH Segmented Query (G)", use_segmented_bvh_query);
        ImGui::SameLine();
        ImGui::TextDisabled(*use_segmented_bvh_query ? "[per-TG + scan]" : "[baseline atomicAdd]");
    }
    if (use_agglomerative_bvh) {
        ImGui::Checkbox("BVH Agglomerative Build", use_agglomerative_bvh);
        ImGui::SameLine();
        ImGui::TextDisabled(*use_agglomerative_bvh ? "[Apetrei 2014]" : "[Karras 2012]");
    }
    if (enable_refit) {
        ImGui::Checkbox("BVH Refit", enable_refit);
        ImGui::SameLine();
        ImGui::TextDisabled(*enable_refit ? "[incremental]" : "[full rebuild]");
    }
    if (fused_refit_enlarge) {
        ImGui::Checkbox("Fused Refit+Enlarge", fused_refit_enlarge);
        ImGui::SameLine();
        ImGui::TextDisabled(*fused_refit_enlarge ? "[1 swept pass]" : "[2-pass legacy]");
    }
    if (use_subobject_bvh) {
        ImGui::Checkbox("Cluster mode", use_subobject_bvh);
        ImGui::SameLine();
        ImGui::TextDisabled(*use_subobject_bvh ? "[cluster sub-obj BVH — per-object s in 물체 panel]"
                                               : "[off]");
        if (subbvh_split_s) {
            // SliderInt clamps to [1,16]; k = tiles² saturates at full split.
            ImGui::SliderInt("Split s (N)", subbvh_split_s, 1, 16, "s=%d");
            ImGui::SameLine();
            ImGui::TextDisabled("[applies on next BVH rebuild]");
        }
    }
    if (refit_substep_period || cd_substep_period) {
        ImGui::Separator();
        ImGui::TextDisabled("Collision cadence (per substep, 1 = every substep)");
        const ImU32 lo = 1, hi = 60;
        if (refit_substep_period) {
            if (*refit_substep_period < 1) *refit_substep_period = 1;
            ImGui::SliderScalar("Refit period", ImGuiDataType_U32,
                                refit_substep_period, &lo, &hi, "%u");
            ImGui::SameLine();
            ImGui::TextDisabled(*refit_substep_period <= 1 ? "[every substep]"
                                                           : "[BVH AABB refresh]");
        }
        if (cd_substep_period) {
            if (*cd_substep_period < 1) *cd_substep_period = 1;
            ImGui::SliderScalar("CD period", ImGuiDataType_U32,
                                cd_substep_period, &lo, &hi, "%u");
            ImGui::SameLine();
            ImGui::TextDisabled(*cd_substep_period <= 1 ? "[every substep]"
                                                        : "[broad pair refresh; narrow every substep]");
        }
    }

    ImGui::Separator();

    drawSeries("Frame Time (ms)", history.makeRecentSeries(-1, state.history_seconds));

    int physics_index = history.sectionIndex("physics_total");
    if (physics_index >= 0)
        drawSeries("Physics (ms)", history.makeRecentSeries(physics_index, state.history_seconds));

    int render_index = history.sectionIndex("render_total");
    if (render_index >= 0)
        drawSeries("Render (ms)", history.makeRecentSeries(render_index, state.history_seconds));

    if (!section_names.empty()) {
        std::vector<const char*> items;
        items.reserve(section_names.size());
        for (const auto& name : section_names) items.push_back(name.c_str());

        ImGui::Separator();
        ImGui::Combo("Section Graph", &state.selected_section, items.data(), static_cast<int>(items.size()));
        drawSeries(section_names[state.selected_section].c_str(), history.makeRecentSeries(state.selected_section, state.history_seconds));
    }

    if (ImGui::CollapsingHeader("Section Table", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::BeginTable("profiler_sections", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("Section");
            ImGui::TableSetupColumn("Latest (ms)");
            ImGui::TableSetupColumn("Avg (ms)");
            ImGui::TableSetupColumn("Max (ms)");
            ImGui::TableHeadersRow();

            for (std::size_t i = 0; i < section_names.size(); ++i) {
                SectionStats stats = history.computeRecentStats(i, state.history_seconds);
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(section_names[i].c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("%.3f", stats.latest_ms);
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.3f", stats.average_ms);
                ImGui::TableSetColumnIndex(3);
                ImGui::Text("%.3f", stats.max_ms);
            }
            ImGui::EndTable();
        }
    }

    ImGui::Separator();
    static char export_path_buffer[512];
    static bool export_path_initialized = false;
    if (!export_path_initialized) {
        std::snprintf(export_path_buffer, sizeof(export_path_buffer), "%s", state.export_path.c_str());
        export_path_initialized = true;
    }

    ImGui::InputText("Export CSV", export_path_buffer, sizeof(export_path_buffer));
    state.export_path = export_path_buffer;
    if (ImGui::Button("Write CSV")) {
        state.status_message = history.exportCsv(state.export_path)
            ? "Exported profiler history"
            : "Failed to export profiler history";
    }
    if (!state.status_message.empty()) ImGui::TextUnformatted(state.status_message.c_str());

    ImGui::End();
}

} // namespace profiler
