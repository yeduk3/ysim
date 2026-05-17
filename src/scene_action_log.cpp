#include "SceneActionLog.hpp"

#include "imgui.h"

namespace scene_log {

namespace {

const char* categoryTag(Category c) {
    switch (c) {
        case Category::Object:     return "[오브젝트]";
        case Category::Constraint: return "[구속]";
        case Category::SceneIO:    return "[씬]";
    }
    return "[?]";
}

ImVec4 categoryColor(Category c) {
    switch (c) {
        case Category::Object:     return ImVec4(0.55f, 0.78f, 1.00f, 1.0f); // blue
        case Category::Constraint: return ImVec4(0.70f, 0.85f, 0.55f, 1.0f); // green
        case Category::SceneIO:    return ImVec4(0.90f, 0.78f, 0.45f, 1.0f); // amber
    }
    return ImVec4(1, 1, 1, 1);
}

} // namespace

void drawSceneActionLogWindow(SceneActionLogWindowState& state) {
    if (!state.open) return;

    auto& log = SceneActionLog::instance();

    ImGui::SetNextWindowSize(ImVec2(460, 340), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("씬 동작 로그", &state.open)) {
        ImGui::End();
        return;
    }

    if (ImGui::Button("지우기")) log.clear();
    ImGui::SameLine();
    ImGui::Checkbox("자동 스크롤", &state.autoScroll);
    ImGui::SameLine();
    ImGui::TextDisabled("(%d개 · 런타임 전용)",
                        (int)log.entries().size());

    ImGui::Separator();

    ImGui::BeginChild("scene_log_scroll", ImVec2(0, 0), false,
                      ImGuiWindowFlags_HorizontalScrollbar);
    if (log.entries().empty()) {
        ImGui::TextDisabled("아직 기록된 동작이 없습니다.");
    } else {
        for (const auto& e : log.entries()) {
            ImGui::TextDisabled("%s", e.time.c_str());
            ImGui::SameLine();
            ImGui::TextColored(categoryColor(e.category), "%s",
                               categoryTag(e.category));
            ImGui::SameLine();
            if (e.ok) {
                ImGui::TextUnformatted(e.message.c_str());
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f),
                                   "%s", e.message.c_str());
            }
        }
    }

    // Auto-scroll to the newest line once per append, only when the
    // user left auto-scroll on. takeScrollPending() clears the flag so
    // a user who scrolled up to read history is not yanked back down
    // every frame — only when a genuinely new action lands.
    if (state.autoScroll && log.takeScrollPending()) {
        ImGui::SetScrollHereY(1.0f);
    } else {
        log.takeScrollPending();
    }
    ImGui::EndChild();

    ImGui::End();
}

} // namespace scene_log
