#pragma once

#include "FrameProfiler.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <string>

namespace profiler {

inline std::string makeDefaultProfileExportPath() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &now_time);
#else
    localtime_r(&now_time, &local_time);
#endif

    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &local_time);

#ifdef YSIM_PROJECT_ROOT
    return std::string(YSIM_PROJECT_ROOT) + "/profiles/profile_" + timestamp + ".csv";
#else
    return std::string("profiles/profile_") + timestamp + ".csv";
#endif
}

struct ProfilerWindowState {
    // Default hidden — the profiler window is opt-in via the "보기"
    // menu's "프로파일러 윈도우 열기" item. drawProfilerWindow
    // early-returns when this is false, so leaving the call site
    // unconditional costs nothing.
    bool open = false;
    float history_seconds = 3.0f;
    int selected_section = 0;
    std::string export_path = makeDefaultProfileExportPath();
    std::string status_message;
};

// Scene aggregate counts shown alongside FPS/frame time. Producer
// (main.cpp) sums GeneralMesh::state.x and adjacency.facets over
// Scene::meshes once per frame and passes the totals in.
struct SceneCounts {
    int meshes = 0;
    int points = 0;
    int triangles = 0;
};

void drawProfilerWindow(
    ProfilerWindowState& state,
    FrameProfiler& profiler,
    bool* pause,
    bool* debug_each_boxes,
    bool* debug_scene_box,
    bool* debug_collisions,
    bool* mesh_inspector_open,
    SceneCounts scene_counts = {},
    bool* use_segmented_bvh_query = nullptr,
    bool* use_agglomerative_bvh = nullptr,
    bool* enable_refit = nullptr,
    bool* use_analytic_primitive = nullptr,
    bool* use_spatial_hashing = nullptr,
    // Substep cadence knobs (Simulator::refitSubstepPeriod / cdSubstepPeriod,
    // both Index == uint32_t). 1 = every substep. Live-editable sliders.
    std::uint32_t* refit_substep_period = nullptr,
    std::uint32_t* cd_substep_period = nullptr,
    // Profiling detail tier (0=None, 1=PerFrame, 2=InFrame), matching
    // sim_config::ProfileLevel's enumerator order. When non-null a combo lets
    // the user switch tiers live; main.cpp re-wires the profiler pointers from
    // the edited value on the next frame. Kept as an int so this header stays
    // free of a sim_config dependency.
    int* profile_level = nullptr,
    // Fused refit+enlarge toggle (single-pass broad-phase maintenance). When
    // non-null, a checkbox lets the user A/B it live against the legacy
    // two-pass path.
    bool* fused_refit_enlarge = nullptr
);

} // namespace profiler
