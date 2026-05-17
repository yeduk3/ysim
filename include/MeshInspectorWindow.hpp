#pragma once

#include "tinym.hpp"

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

    // No-selection branch: when mesh_id < 0 the right panel renders these
    // three Add-Object buttons instead of the per-mesh editors. Callbacks
    // are owned by main.cpp; they open the same Sphere / Cube / Import
    // modals that used to live behind the Create + File menus.
    std::function<void()> on_request_add_cube;
    std::function<void()> on_request_add_sphere;
    std::function<void()> on_request_add_cylinder;
    std::function<void()> on_request_add_plane;
    std::function<void()> on_request_add_import;

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
