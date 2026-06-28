#pragma once

#include "tinym.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace mesh_inspector {

// D-035: inspector-side float-array conversion helpers between the
// canonical Quat representation (the 4-float wxyz storage at
// target.rotation_wxyz) and user-facing input modes. These mirror
// the Quat-based helpers in main.cpp (quatFromAxisAngle / quatToAxisAngle
// / quatFromEulerXYZ / quatToEulerXYZ) but operate on raw float arrays
// so the inspector TU doesn't need access to main.cpp's bare Quat
// struct. Implementations are kept lockstep with main.cpp's; if either
// side changes, the other must follow. (Source-file split, when it
// lands, will consolidate to a single shared implementation.)
//
// Conventions (load-bearing — see D-035):
//   - Euler XYZ is intrinsic Tait-Bryan: rotation = R_z * R_y * R_x.
//     Apply X first in body frame, then Y, then Z. Matches Blender.
//   - All angles in this header are in DEGREES (user-facing). The
//     widget displays degrees; conversion to radians happens inside
//     these helpers.
//   - Axis-angle output is in [0, 180] degrees with auto-normalized
//     axis; identity-quat fallback returns angle=0 axis=(1, 0, 0).
//   - Gimbal-lock fallback for Euler extraction is lossy by design.

namespace detail {
inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kDegToRad = kPi / 180.0f;
inline constexpr float kRadToDeg = 180.0f / kPi;
}

// Build a unit-norm wxyz quaternion from an axis (any non-zero length)
// and an angle in degrees. wxyz layout matches GeneralMesh::rotationQuat
// (the renderer / simulator read q.w / q.x / q.y / q.z in that order).
inline void axisAngleDegToQuatWxyz(const float axis[3], float angleDeg,
                                   float outWxyz[4]) {
    float n = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (n < 1e-12f) {
        outWxyz[0] = 1.0f; outWxyz[1] = 0.0f; outWxyz[2] = 0.0f; outWxyz[3] = 0.0f;
        return;
    }
    float nx = axis[0] / n;
    float ny = axis[1] / n;
    float nz = axis[2] / n;
    float half = (angleDeg * detail::kDegToRad) * 0.5f;
    float s = std::sin(half);
    outWxyz[0] = std::cos(half);
    outWxyz[1] = nx * s;
    outWxyz[2] = ny * s;
    outWxyz[3] = nz * s;
}

// Extract axis (unit-norm) and angle (in degrees, [0, 180]) from a
// unit-norm wxyz quaternion. Identity-quat fallback: angle=0,
// axis=(1, 0, 0).
// Turn-30 fix (BLOCK closure for D-035): input wxyz is antipodally
// canonicalized at entry (negate all 4 components when wxyz[0] < 0)
// so qw ∈ [0, 1] always and the returned angle stays in [0, 180°] by
// construction. Without this, wxyz = (-1, 0, 0, 0) (antipodal identity)
// produced s = sqrt(1 - qw²) = 0 and axis = wxyz[1..3] / s = NaN/inf —
// see D-035's turn-30 addendum. Lockstep with main.cpp::quatToAxisAngle.
inline void quatWxyzToAxisAngleDeg(const float wxyz[4], float outAxis[3],
                                   float& outAngleDeg) {
    // Antipodal canonicalization: wxyz and -wxyz represent the same
    // rotation; pick the qw >= 0 representative.
    float qw = wxyz[0], qx = wxyz[1], qy = wxyz[2], qz = wxyz[3];
    if (qw < 0.0f) {
        qw = -qw; qx = -qx; qy = -qy; qz = -qz;
    }
    if (qw > 1.0f) qw = 1.0f;  // tiny rounding overshoot guard
    float angleRad = 2.0f * std::acos(qw);
    if (angleRad < 1e-6f) {
        outAxis[0] = 1.0f; outAxis[1] = 0.0f; outAxis[2] = 0.0f;
        outAngleDeg = 0.0f;
        return;
    }
    float s = std::sqrt(1.0f - qw * qw);
    outAxis[0] = qx / s;
    outAxis[1] = qy / s;
    outAxis[2] = qz / s;
    outAngleDeg = angleRad * detail::kRadToDeg;
}

// Build wxyz from Euler XYZ (intrinsic Tait-Bryan, in degrees). Uses
// the same compose-via-axis-angle path as main.cpp's quatFromEulerXYZ.
inline void eulerXYZDegToQuatWxyz(const float xyzDeg[3], float outWxyz[4]) {
    float qx[4], qy[4], qz[4];
    float ax[3] = {1.0f, 0.0f, 0.0f};
    float ay[3] = {0.0f, 1.0f, 0.0f};
    float az[3] = {0.0f, 0.0f, 1.0f};
    axisAngleDegToQuatWxyz(ax, xyzDeg[0], qx);
    axisAngleDegToQuatWxyz(ay, xyzDeg[1], qy);
    axisAngleDegToQuatWxyz(az, xyzDeg[2], qz);
    // qz * qy * qx via Hamilton (a * b = apply b first then a).
    auto hamilton = [](const float a[4], const float b[4], float out[4]) {
        out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
        out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
        out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
        out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    };
    float zy[4];
    hamilton(qz, qy, zy);
    hamilton(zy, qx, outWxyz);
}

// Extract Euler XYZ (intrinsic Tait-Bryan, in degrees) from a unit-norm
// wxyz quaternion. Gimbal-lock fallback: when |sinp| ≥ 1 - 1e-6, set
// roll (X) = 0 and derive yaw (Z) from the residual.
inline void quatWxyzToEulerXYZDeg(const float wxyz[4], float outXyzDeg[3]) {
    float qw = wxyz[0], qx = wxyz[1], qy = wxyz[2], qz = wxyz[3];
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    float xRad, yRad, zRad;
    if (std::abs(sinp) >= 1.0f - 1e-6f) {
        yRad = (sinp > 0.0f) ? (detail::kPi * 0.5f) : (-detail::kPi * 0.5f);
        xRad = 0.0f;
        zRad = 2.0f * std::atan2(qz, qw);
    } else {
        yRad = std::asin(sinp);
        xRad = std::atan2(2.0f * (qw * qx + qy * qz),
                          1.0f - 2.0f * (qx * qx + qy * qy));
        zRad = std::atan2(2.0f * (qw * qz + qx * qy),
                          1.0f - 2.0f * (qy * qy + qz * qz));
    }
    outXyzDeg[0] = xRad * detail::kRadToDeg;
    outXyzDeg[1] = yRad * detail::kRadToDeg;
    outXyzDeg[2] = zRad * detail::kRadToDeg;
}


struct MeshInspectorWindowState {
    bool open = true;
    std::string status_message;
    // D-035: pre-existing rotation_input_mode (Quat / Euler / Axis-Angle)
    // dropped from the right-panel UI; Euler XYZ (deg) is the only mode
    // surfaced now. Field preserved as a no-op placeholder so the struct
    // layout stays binary-compatible for any other consumer.
    int rotation_input_mode = 1;
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
    // Inspector scale path, mirrors the translate path. The pointer reads
    // the mesh's current per-axis scale; the callback applies a finished
    // edit through Simulator::scaleObject (which scales state.x /
    // state.xPrev / preview about the transformPosition pivot and writes
    // the new absolute scale onto the mesh + request for pack survival).
    // Both must be set together to enable the InputFloat3 row.
    tinym::vec3* scale = nullptr;
    std::function<void(int, tinym::vec3)> on_scale;
    // "팽팽함" — cloth-only uniform stiffness multiplier. Non-null only
    // when the selected mesh is a cloth behavior; the slider is hidden
    // otherwise. Reads the live value via the pointer; commits through
    // the callback (which writes the live mesh + request mirror).
    float* cloth_stiffness_scale = nullptr;
    std::function<void(int, float)> on_cloth_stiffness_scale;
    // Per-type cloth stiffness coefficients shown alongside "팽팽함".
    // Each pointer reads the live behavior-param field; the callback
    // commits to the mesh + request mirror. A null pointer hides that
    // slider (same convention as cloth_stiffness_scale): FastGridCloth
    // binds stretch+bend only, TriangularCloth binds stretch+shear+bend.
    float* cloth_stretch = nullptr;
    std::function<void(int, float)> on_cloth_stretch;
    float* cloth_shear = nullptr;
    std::function<void(int, float)> on_cloth_shear;
    float* cloth_bend = nullptr;
    std::function<void(int, float)> on_cloth_bend;
    // FR-005 / D-027 inspector material path. The 4 scalar/vec3 pointers
    // read the mesh's material fields (D-005's v1 OpenPBR subset: base_color,
    // metallic, roughness, specular_weight, emission_color). Widgets mutate
    // these in place for live preview (the renderer reads mesh.material.*
    // every frame). On every change the callback fires with the full Material
    // snapshot so Simulator::setMaterial can also write pendingMaterials[id]
    // for re-pack survival via D-025. The 5 fields + callback must all be
    // set together to enable the material rows. The callback signature
    // takes individual primitives to avoid coupling this header to main.cpp's
    // Material struct (same pattern as on_rotate).
    float* metallic = nullptr;
    float* roughness = nullptr;
    float* specular_weight = nullptr;
    tinym::vec3* emission_color = nullptr;
    std::function<void(int /*meshId*/,
                       tinym::vec3 /*baseColor*/, float /*metallic*/,
                       float /*roughness*/, float /*specularWeight*/,
                       tinym::vec3 /*emissionColor*/)> on_material_edit;
    // FR-006 / BDD-006 / D-036 inspector behavior-tag editing path.
    // current_behavior_index is set by production each frame to the
    // mesh's current behavior tag mapped to a dropdown index:
    //   0 = Float, 1 = TriangularCloth, 2 = FastGridCloth, 3 = Rigid.
    // Reserved-not-shipped (Elastic / Fluid / Generator) are absent
    // from the dropdown — hard-coded 4-entry list. -1 means the field
    // is uninitialized / behavior editing is disabled for this target.
    // grid_eligible is true when the mesh's initializer is a
    // MeshGridInitializer (the only producer of a square-regular
    // grid topology); FastGridCloth combo entry is selectable only
    // when this is true. on_behavior_change applies the user's
    // dropdown selection by mapping the index back to BehaviorType
    // and invoking Simulator::changeBehavior at the production
    // boundary. The callback returns false when the setter rejects
    // (e.g., FastGridCloth on a non-grid shape) so the widget can
    // surface a status message. The int-pointer / int-index pattern
    // avoids coupling this header to main.cpp's BehaviorType enum;
    // production owns the mapping table.
    int current_behavior_index = -1;
    bool grid_eligible = false;
    std::function<bool(int /*meshId*/, int /*newBehaviorIndex*/)> on_behavior_change;

    // D-041: remove the currently-selected mesh from the scene. Production
    // wires this to Simulator::removeMesh which erases the request, frees
    // the initializer, decrements numMeshes, and marks dirty so the next
    // Simulator::update re-initializes. The widget renders a Delete button
    // next to other transform/material controls; clicking fires this
    // callback with the selected mesh id.
    std::function<void(int /*meshId*/)> on_delete;

    // Per-object environment-force gates. Pointers alias
    // GeneralMesh::applyGravity / applyWind; widget checkboxes mutate in
    // place. Both must be set together to enable the row.
    // The on_env_toggle_change callback fires AFTER the in-place mutation
    // so production can propagate the new value to the RequestGeneralMesh
    // mirror — without that, Simulator::reset() (which rebuilds meshes
    // from requests via Scene::pack) would clobber the user's toggle
    // back to its default. Optional.
    bool* apply_gravity = nullptr;
    bool* apply_wind = nullptr;
    std::function<void(int /*meshId*/, bool /*applyGravity*/, bool /*applyWind*/)>
        on_env_toggle_change;

    // Static(고정) declaration toggle. Pointer aliases GeneralMesh::isStatic;
    // the widget mutates in place, then fires on_static_change so production
    // can mirror to the request, update the BVH objStatic cache, and — when
    // turning static ON — auto-clear applyGravity/applyWind (the apply_*
    // pills reflect the change next frame since they alias the live fields).
    // Optional; the Environment row renders the pill only when set.
    bool* is_static = nullptr;
    std::function<void(int /*meshId*/, bool /*isStatic*/)> on_static_change;

    // ── Sub-object BVH (per-object; shown only in master cluster mode) ─
    // Per-object controls for the cluster sub-object BVH. subobj_split_s
    // aliases this mesh's split s (1..8, k=4^s); on_subobj_split commits +
    // rebuilds this mesh's tree. subobj_render aliases "this mesh's
    // triangles are color-coded by cluster"; on_subobj_render commits it.
    // The master on/off lives in the profiler window (global) — production
    // only sets these pointers when master cluster mode is on, so the
    // section is hidden otherwise (null-pointer gating). Both must be set.
    int*  subobj_split_s = nullptr;
    bool* subobj_render  = nullptr;
    std::function<void(int /*meshId*/, int  /*s*/)>      on_subobj_split;
    std::function<void(int /*meshId*/, bool /*render*/)> on_subobj_render;

    // Checkerboard render option — non-null only for plane (grid) meshes.
    // The checkbox aliases GeneralMesh::checkerboard (renderer reads it each
    // frame); on_checkerboard mirrors the new value onto the request so
    // Scene::pack rebuilds preserve it. The pattern is a world-space
    // black/white checker (1 world unit per cell) derived in the plane's
    // local frame, so it stays fixed to the plane when it translates and
    // densifies under scale. Both must be set together to enable the row.
    bool* checkerboard = nullptr;
    std::function<void(int /*meshId*/, bool /*on*/)> on_checkerboard;

    // ── Kinematic body (BVH motion) panel ────────────────────────────
    // kin_panel=true renders the playback section: play/pause, speed,
    // loop, a time scrub over [0, kin_duration], and a motion-file combo
    // fed by kin_file_list (file names relative to the BVH asset dir).
    // Values are snapshots rebuilt each frame by production; edits commit
    // through the callbacks. The behavior segment tabs are hidden for
    // kinematic targets (production passes current_behavior_index = -1).
    bool kin_panel = false;
    bool kin_playing = false;
    float kin_speed = 1.0f;
    bool kin_loop = true;
    float kin_time = 0.0f;      // seconds
    float kin_duration = 0.0f;  // seconds
    std::string kin_file;       // currently loaded file (display name)
    std::vector<std::string> kin_file_list;
    std::function<void(int /*meshId*/, bool /*playing*/)> on_kin_play;
    std::function<void(int /*meshId*/, float /*speed*/)> on_kin_speed;
    std::function<void(int /*meshId*/, bool /*loop*/)> on_kin_loop;
    std::function<void(int /*meshId*/, float /*timeSec*/)> on_kin_scrub;
    std::function<void(int /*meshId*/, const std::string& /*file*/)> on_kin_file;
    // Camera-follow toggle: when on, the viewport orbit pivot tracks this
    // kinematic body's animated root each frame (production reads the live
    // root world position from the proxy). kin_camera_follow is a per-frame
    // snapshot of whether THIS mesh is the followed one; the callback sets
    // the follow target (true) or clears it (false). Rendered in the 모션
    // accordion, so it only appears for kinematic targets.
    bool kin_camera_follow = false;
    std::function<void(int /*meshId*/, bool /*follow*/)> on_kin_camera_follow;

    // ── Motion-graph sub-panel (Kovar 2002) ──────────────────────────
    // kin_mode picks which widget set renders inside the 모션 accordion
    // (mode-exclusive): 0 = single clip (the original playback widgets),
    // 1 = graph random walk, 2 = graph transition. kin_graph_ready is true
    // when the active mode's graph build succeeded — gates the playback
    // widgets of graph modes. kin_status carries the last build report
    // verbatim; kin_label names what the walk/transition is playing now.
    // kin_graph_selected parallels kin_file_list (1 = clip in the walk's
    // graph set). All values are per-frame snapshots like the rest of the
    // kin_* fields; edits commit through the callbacks only.
    int kin_mode = 0;
    bool kin_graph_ready = false;
    float kin_threshold = 0.10f;
    float kin_marker_frac = 0.10f;  // joint-axis marker length / body height
    std::string kin_status;
    std::string kin_label;
    std::vector<unsigned char> kin_graph_selected;
    std::function<void(int /*meshId*/, int /*mode*/)> on_kin_mode;
    std::function<void(int /*meshId*/, float /*threshold*/)> on_kin_threshold;
    std::function<void(int /*meshId*/, float /*markerFrac*/)> on_kin_marker_frac;
    std::function<void(int /*meshId*/, const std::string& /*file*/, bool /*on*/)>
        on_kin_graph_toggle;
    std::function<void(int /*meshId*/, bool /*selectAll*/)> on_kin_graph_all;
    std::function<void(int /*meshId*/)> on_kin_walk_build;
    std::function<void(int /*meshId*/)> on_kin_walk_reseed;
    std::function<void(int /*meshId*/)> on_kin_trans_build;
    std::function<void(int /*meshId*/)> on_kin_blend_build;

    // ── Reusable motion-clip selector (common to every clip-picking mode) ──
    // Each kin_clip_slots entry is one selectable clip: a file combo, a preview
    // strobe toggle, an in-place color swatch, and a one-shot play button. The
    // GUI renders a single uniform row per slot (clipSlotRow), so transition,
    // DTW, and blend-space modes all pick clips with identical widgets. Modes
    // index slots 0,1,…; the slot callbacks carry the slot index so one
    // component drives them all. Per-frame snapshot like the other kin_* fields.
    struct MotionClipSlot {
        std::string label;       // row heading (모션 1, Walk, …); blank = none
        std::string file;        // currently selected file name
        bool preview = false;    // strobe-ghost toggle state
        float* color = nullptr;  // 3 floats aliasing the body (ColorEdit3), or null
        // Active-frame window [range_start, range_end] over a clip of frame_count
        // frames. The 2-handle slider edits it; preview + the object anchor use
        // only this window. frame_count 0 = not cached yet (slider hidden).
        int range_start = 0;
        int range_end = 0;
        int frame_count = 0;
    };
    std::vector<MotionClipSlot> kin_clip_slots;
    std::function<void(int /*meshId*/, int /*slot*/, const std::string& /*file*/)>
        on_kin_slot_file;
    std::function<void(int /*meshId*/, int /*slot*/, bool /*on*/)> on_kin_slot_preview;
    std::function<void(int /*meshId*/, int /*slot*/)> on_kin_slot_play;
    std::function<void(int /*meshId*/, int /*slot*/, int /*start*/, int /*end*/)>
        on_kin_slot_range;
    // Which one-shot preview is currently playing: 0 none, slotIdx+1 a clip
    // slot, -1 the blended result. Lets the play button flip to "중단" + stop.
    int kin_preview_playing = 0;
    std::function<void(int /*meshId*/)> on_kin_preview_stop;
    // Play buttons are paused-only — disabled while the sim runs (kin_sim_paused
    // mirrors simulator.pause).
    bool kin_sim_paused = false;
    // Blend playback: tint the live body by the current blend source weight
    // (mixes slot 0/1 colors). DTW mode only; per-frame snapshot.
    bool kin_blend_colorize = false;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_blend_colorize;
    // Blend-result preview: translucent strobe of the live blended cycle
    // (blend-space mode); morphs as the pad cursor moves. Per-frame snapshot.
    bool kin_blend_preview = false;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_blend_preview;
    // Absolute vs relative root for the blend space (build-time). On = blend the
    // real per-clip root motion (anchored frame-0→object); off = pin + integrate
    // heading-relative velocity. Toggling rebuilds. Per-frame snapshot.
    bool kin_blend_absroot = false;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_blend_absroot;
    // One-shot opaque playback of the blended result through one gait cycle
    // (paused-only). Honors the live cursor as it plays.
    std::function<void(int /*meshId*/)> on_kin_blend_play;

    // ── Interactive blend space (kin_mode 4) ──────────────────────────────
    // A 2D pad blends N locomotion clips placed at kin_blend_coords (in
    // [-1,1]²); dragging the cursor sets inverse-distance weights live.
    // Populated only after a blend space is built (kin_graph_ready).
    float kin_blend_cursor[2] = {0.0f, 0.0f};
    std::vector<std::array<float, 2>> kin_blend_coords;  // clip sample points
    std::vector<std::string> kin_blend_labels;           // short clip labels
    std::vector<float> kin_blend_weights;                // current mix (Σ=1)
    std::function<void(int /*meshId*/)> on_kin_blendspace_build;
    std::function<void(int /*meshId*/, float /*x*/, float /*y*/)>
        on_kin_blend_cursor;
    // Preset dropdown (above the per-clip combos): pick a curated 4-clip set or
    // 자율선택(manual). kin_blend_preset = -1 manual, else index into
    // kin_blend_presets. Choosing one fills the 4 clips; editing any clip reverts.
    std::vector<std::string> kin_blend_presets;
    int kin_blend_preset = -1;
    std::function<void(int /*meshId*/, int /*presetIdx, -1=manual*/)>
        on_kin_blend_preset;

    // ── Two-motion keytime blend (kin_mode 5; Verbs & Adverbs) ────────────
    // Two clips registered by editable foot keytimes and mixed by named adverb
    // "tags" (1..2), each with a per-motion percentage; a query slider (1 tag)
    // or pad (2 tags) sets the live mix. The kin_verb_* snapshots are populated
    // after a build (kin_verb_ready); keytimes are LFD,RFU,RFD,LFU,cycleEnd in
    // clip-frame coordinates, frame_count[i] is clip i's length (slider max).
    // The two files reuse kin_clip_slots 0/1 (combo + preview + frame window).
    bool kin_verb_ready = false;
    std::vector<std::array<int, 5>> kin_verb_keys;      // per motion (2)
    std::vector<int> kin_verb_frame_count;              // clip length per motion
    std::vector<std::string> kin_verb_tags;             // 1..2 adverb names
    std::vector<std::array<float, 2>> kin_verb_adverb;  // [motion][tag] percent
    float kin_verb_query[2] = {50.0f, 0.0f};            // adverb query percent
    std::vector<float> kin_verb_weights;                // current mix (2), display
    std::function<void(int /*meshId*/)> on_kin_verb_build;
    std::function<void(int /*meshId*/, int /*ex*/, int /*which*/, int /*frame*/)>
        on_kin_verb_keytime;
    std::function<void(int /*meshId*/, const std::string& /*name*/)>
        on_kin_verb_add_tag;
    std::function<void(int /*meshId*/, int /*tagIdx*/)> on_kin_verb_remove_tag;
    std::function<void(int /*meshId*/, int /*tagIdx*/, const std::string& /*name*/)>
        on_kin_verb_tag_name;
    std::function<void(int /*meshId*/, int /*ex*/, int /*tag*/, float /*pct*/)>
        on_kin_verb_adverb;
    std::function<void(int /*meshId*/, int /*tag*/, float /*pct*/)> on_kin_verb_query;
    // Blended-result preview: translucent strobe of the live 2-motion blend
    // cycle (toggle), plus a one-shot 재생 through one cycle (paused-only; reuses
    // kin_preview_playing == -1). Both morph as the adverb slider moves.
    bool kin_verb_preview = false;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_verb_preview;
    std::function<void(int /*meshId*/)> on_kin_verb_play;
    // Extrapolation toggle: on = signed RBF weights (over-driving the adverb past
    // a motion exaggerates it); off = convex (clamped between the two motions).
    bool kin_verb_extrapolate = true;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_verb_extrapolate;
    // Root mode toggle: off = absolute (blend the real root trajectories); on =
    // relative velocity (integrate blended heading-relative velocity). Both
    // travel — neither is a treadmill.
    bool kin_verb_root_relative = false;
    std::function<void(int /*meshId*/, bool /*on*/)> on_kin_verb_root_relative;

    // No-selection branch: when mesh_id < 0 the right panel renders these
    // Add-Object buttons instead of the per-mesh editors. Callbacks
    // are owned by main.cpp; they open the same Sphere / Cube / Import
    // modals that used to live behind the Create + File menus.
    // on_request_add_kinematic adds a BVH kinematic body directly (no
    // modal — the file is switchable in the inspector afterwards).
    std::function<void()> on_request_add_cube;
    std::function<void()> on_request_add_sphere;
    std::function<void()> on_request_add_cylinder;
    std::function<void()> on_request_add_plane;
    std::function<void()> on_request_add_import;
    std::function<void()> on_request_add_kinematic;

    // No-selection branch: the scene's objects, one entry per
    // RequestGeneralMesh (the canonical, pack-surviving list).
    // Rebuilt each frame by production. Clicking a row calls
    // on_select_object(id) → production sets selectedObj, so the
    // panel switches to that object's editor next frame.
    struct ObjectListEntry {
        int id = -1;
        std::string label;
    };
    std::vector<ObjectListEntry> object_list;
    std::function<void(int /*meshId*/)> on_select_object;

    // ── Point panel ──────────────────────────────────────────────────
    // When point_panel is true the inspector renders the per-vertex
    // panel (pin toggle, position, reference-copy) for the selected
    // (point_obj, point_vert) and ignores the object editors. When it
    // is false the existing object / no-selection logic applies. In
    // Point selection mode with NO vertex selected, production builds a
    // target with point_panel=false AND mesh_id=-1 so the same
    // "nothing selected" add-buttons panel shows.
    bool point_panel = false;
    int point_obj = -1;
    int point_vert = -1;
    bool point_fixed = false;          // current pinned state (read each frame)
    float point_position[3] = {0, 0, 0};  // current world pos (read each frame)
    bool point_ref_active = false;     // reference-pick mode on?
    std::function<void(bool)> on_point_set_fixed;
    std::function<void(float, float, float)> on_point_move;
    std::function<void()> on_point_ref_toggle;

    // Reference-point constraints touching the selected vertex. One
    // entry per constraint where the selected point is either the
    // follower (selected_is_follower = true: this point tracks
    // other_obj/other_vert) or the leader (false: other_obj/other_vert
    // tracks this point). Rebuilt each frame by production. The remove
    // callback takes the entry's index in this vector; production
    // re-resolves the same ordered match list and erases that one.
    struct PointRefEntry {
        bool selected_is_follower = true;
        int other_obj = -1;
        int other_vert = -1;
    };
    std::vector<PointRefEntry> point_ref_constraints;
    std::function<void(int /*entryIndex*/)> on_point_ref_remove;
};

void drawMeshInspectorWindow(
    MeshInspectorWindowState& state,
    const MeshInspectorTarget& target
);

} // namespace mesh_inspector
