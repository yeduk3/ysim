#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

struct Material {
    tinym::vec3 baseColor = tinym::vec3(1.0f);
    float metallic = 0.0f;
    float roughness = 0.5f;
    float specularWeight = 1.0f;
    tinym::vec3 emissionColor = tinym::vec3(0.0f);
};

// D-037: `struct Quat` definition lives in `include/Quat.hpp` (included near
// the top of this file). Body unchanged; only the physical location moved so
// `include/RigidPhysicsTypes.hpp` (RigidInitial holds Quat by value) can see
// the complete type. Helper functions (operator*, quatNormalize, etc.) stay
// in this file below.

// D-019: canonical quaternion math. Hamilton product convention is `a * b
// = apply b first, then a` — i.e., to rotate by R1 then R2, write R2 * R1.
// Free functions so they pair with the bare struct without changing its
// aggregate-init shape (the on-disk schema relies on the {w, x, y, z}
// member order). Future rotation consumers (FR-004 inspector wiring,
// FR-008 rigid body, eventual renderer-side rotation) should use these
// rather than reimplementing the math.
inline Quat operator*(const Quat& a, const Quat& b) {
    Quat r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

inline float quatNorm(const Quat& q) {
    return std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

inline Quat quatNormalize(const Quat& q) {
    float n = quatNorm(q);
    if (n < 1e-12f) return Quat{};
    return Quat{q.w/n, q.x/n, q.y/n, q.z/n};
}

// D-022: extends D-019's canonical Quat math family.
// quatConjugate flips the sign of the imaginary parts; for unit
// quaternions it equals the inverse, which is what rotation
// composition uses. rotateVector applies the canonical Hamilton
// sandwich q * v_pure * conjugate(q) — D-019's operator* left-
// associates so the expression reads as written. Caller must pass a
// unit-norm q (callers that build rotations via quatAxisAngle or
// quatNormalize satisfy this).
inline Quat quatConjugate(const Quat& q) {
    return Quat{q.w, -q.x, -q.y, -q.z};
}

inline tinym::vec3 rotateVector(const Quat& q, const tinym::vec3& v) {
    Quat vp{0.0f, v.x, v.y, v.z};
    Quat r = q * vp * quatConjugate(q);
    return tinym::vec3(r.x, r.y, r.z);
}

// D-035: inspector rotation ergonomics — conversion helpers between
// the canonical Quat representation and user-facing Euler / axis-angle
// forms. Free functions next to D-019/D-022's quat family, by the same
// convention. Inspector widgets display values in degrees; these
// helpers operate in radians internally — degree conversion happens
// at the inspector boundary.
//
// quatFromAxisAngle: auto-normalizes the axis. If axis-norm < 1e-12,
// returns identity (caller intent is ambiguous; identity is the safe
// fallback). Angle is in radians.
inline Quat quatFromAxisAngle(tinym::vec3 axis, float angleRadians) {
    float n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (n < 1e-12f) return Quat{};
    float nx = axis.x / n;
    float ny = axis.y / n;
    float nz = axis.z / n;
    float half = angleRadians * 0.5f;
    float s = std::sin(half);
    return Quat{std::cos(half), nx * s, ny * s, nz * s};
}

// quatToAxisAngle: inverse of quatFromAxisAngle. Returns angle in
// [0, π] and a unit-norm axis. Identity-quat fallback: angle=0,
// axis=(1, 0, 0) canonical. Caller passes a unit-norm q.
//
// Turn-30 fix (BLOCK closure for D-035): input is antipodally
// canonicalized at entry (negate all 4 components when q.w < 0) so
// qw ∈ [0, 1] always, acos(qw) ∈ [0, π/2], and the returned angle
// stays in [0, π] by construction. Without this, q = (-1, 0, 0, 0)
// (the antipodal identity / 360° rotation case) produced
// s = sqrt(1 - qw²) = 0 and axis = q.x/s = NaN/inf — the BLOCK
// surfaced by Estimator turn 30. After canonicalization, qw = +1 →
// acos(1) = 0 → the identity-fallback branch catches it cleanly.
inline void quatToAxisAngle(const Quat& q, tinym::vec3& outAxis,
                            float& outAngleRadians) {
    // Antipodal canonicalization: q and -q represent the same
    // rotation; pick the q.w >= 0 representative.
    float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    if (qw < 0.0f) {
        qw = -qw; qx = -qx; qy = -qy; qz = -qz;
    }
    // Clamp qw to [0, 1] to guard against tiny rounding overshoot
    // (acos is undefined outside [-1, 1]; after canonicalization the
    // valid range is [0, 1]).
    if (qw > 1.0f) qw = 1.0f;
    outAngleRadians = 2.0f * std::acos(qw);
    if (outAngleRadians < 1e-6f) {
        outAxis = tinym::vec3(1.0f, 0.0f, 0.0f);
        outAngleRadians = 0.0f;
        return;
    }
    float s = std::sqrt(1.0f - qw * qw);
    outAxis = tinym::vec3(qx / s, qy / s, qz / s);
}

// quatFromEulerXYZ: intrinsic Tait-Bryan XYZ.
//   rotation = R_z(zRad) * R_y(yRad) * R_x(xRad)
// "Apply X first in body frame, then Y, then Z" — matches Blender's
// default Euler convention. Compose via three axis-angle quaternions
// using D-019's Hamilton product (`a * b = apply b first then a`).
inline Quat quatFromEulerXYZ(float xRad, float yRad, float zRad) {
    Quat qx = quatFromAxisAngle(tinym::vec3(1.0f, 0.0f, 0.0f), xRad);
    Quat qy = quatFromAxisAngle(tinym::vec3(0.0f, 1.0f, 0.0f), yRad);
    Quat qz = quatFromAxisAngle(tinym::vec3(0.0f, 0.0f, 1.0f), zRad);
    return qz * qy * qx;
}

// quatToEulerXYZ: extract Tait-Bryan XYZ from a unit-norm Quat using
// the standard formulas (see e.g., Diebel 2006 "Representing Attitude").
// Output is in radians, with pitch (Y) in [-π/2, π/2]. Gimbal-lock
// fallback: when |sinp| ≥ 1 - 1e-6 (within 0.0257° of pitch=±90°),
// set roll (X) = 0 and derive yaw (Z) from the residual. The
// round-trip through gimbal lock is lossy by design — Euler cannot
// uniquely represent gimbal-locked rotations. Documented in D-035.
inline void quatToEulerXYZ(const Quat& q, float& outX, float& outY,
                           float& outZ) {
    // sinp = 2 * (q.w * q.y - q.z * q.x) corresponds to sin(pitch)
    // in this Tait-Bryan XYZ convention.
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (sinp > 1.0f) sinp = 1.0f;
    else if (sinp < -1.0f) sinp = -1.0f;
    if (std::abs(sinp) >= 1.0f - 1e-6f) {
        // Gimbal lock. Pitch is ±π/2; collapse roll into yaw.
        outY = (sinp > 0.0f) ? (3.14159265358979323846f * 0.5f)
                             : (-3.14159265358979323846f * 0.5f);
        outX = 0.0f;
        outZ = 2.0f * std::atan2(q.z, q.w);
    } else {
        outY = std::asin(sinp);
        outX = std::atan2(2.0f * (q.w * q.x + q.y * q.z),
                          1.0f - 2.0f * (q.x * q.x + q.y * q.y));
        outZ = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                          1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    }
}

// Row-major 3x3 from a unit quaternion — the body-rotation input of
// kinematic::BodyProxy::writeVertices (which is Quat/tinym-free).
inline std::array<float, 9> quatToMat3(const Quat& q) {
    const float w = q.w, x = q.x, y = q.y, z = q.z;
    return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
            2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
            2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
}

// ---- BVH kinematic body ----------------------------------------------------

template <typename PR>
struct MeshKinematicInitializerParams : InitializerParams<PR> {
    std::string filePath;   // source .bvh
    tinym::vec3 center;
    // Normalized body height in scene units; the FK pose (whose raw units
    // vary ~7x across assets/BVH) is uniformly scaled so its rest height
    // equals this BEFORE the user transform applies.
    PR targetHeight;

    MeshKinematicInitializerParams(std::string filePath, tinym::vec3 center,
                                   Index numPoints, Index numFacets,
                                   Index numEdges, PR targetHeight, PR mass)
        : InitializerParams<PR>(numPoints, numFacets, numEdges, mass),
          filePath(std::move(filePath)), center(center),
          targetHeight(targetHeight) {}
};

// One GeneralMesh per kinematic body: the concatenated sphere(joint) +
// cylinder(link) proxy carries a single mesh id, so picking any part
// selects the whole body and its triangles ride the existing collision
// pipeline. The initializer owns motion + proxy + playback state because
// the initializer object (held by the request) is the only per-mesh
// state that survives Scene::pack rebuilds.
template <typename BE, typename PR>
struct MeshKinematicInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshKinematicInitializerParams<PR>;
    ParamsType params;

    bvh::Motion motion;
    kinematic::BodyProxy proxy;

    // Playback state (driven by Simulator::update with the sim step h, so
    // motion time IS simulation time — pausing the sim pauses the body).
    bool playing = true;
    float playSpeed = 1.0f;
    bool loop = true;
    double localTime = 0.0;

    // ── Motion-graph playback (Kovar 2002, include/motion_graph.hpp) ──
    // motionMode picks the pose source: 0 = single clip (the original path,
    // untouched), 1 = graph random walk, 2 = graph transition. A graph mode
    // only engages once its session build succeeded (graphActive()); until
    // then the single clip keeps playing, so a half-configured panel never
    // blanks the body. The staging fields are inspector state that must
    // survive re-packs, which is why they live here with the playback state.
    int motionMode = 0;
    mograph::Session graphSession;
    std::vector<std::string> graphSelFiles;  // random-walk clip set (file names)
    std::string transFileA, transFileB;      // transition endpoints (file names)
    // Interactive blend space (motionMode 4): N locomotion clips placed in a 2D
    // pad, blended live by a draggable cursor. Default files sit at a diamond
    // (bottom/top/left/right); all clips just mix — no fixed role names, they
    // are shown by file. Files user-editable. The live cursor lives on graphSession.
    std::vector<std::string> blendSpaceFiles{
        "WalkLoopA.bvh", "jogCurve.bvh", "SneakLoopA.bvh", "StrutLoopA.bvh"};
    std::vector<std::array<float, 2>> blendSpaceCoords{
        {0.0f, -1.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {1.0f, 0.0f}};
    // Active blend preset: -1 = 자율선택(manual), else index into blendPresets().
    // Selecting a preset fills blendSpaceFiles; changing any file reverts to -1.
    int blendPreset = -1;
    // Blend-space root handling (build-time): false = relative (pin + integrate
    // heading-relative velocity → curving travel-in-place); true = absolute
    // (blend the real per-clip root pos/orientation, anchored frame-0→object).
    bool blendAbsoluteRoot = false;
    float graphThreshold = 0.10f;            // cost threshold, fraction of height
    float graphMarkerFrac = 0.10f;           // joint-axis marker length / height
    uint32_t walkSeed = 12345;
    std::string graphStatus;                 // last build report for the GUI

    // ── Two-motion keytime blend (motionMode 5; Verbs & Adverbs) ──
    // A simpler sibling of the blend space (include/motion_verb.hpp): exactly
    // two clips, registered by EXPLICIT foot keytimes (auto-detected, then
    // editable) instead of DTW, and mixed by named "adverb" tags through an
    // RBF. The staging below survives re-packs; buildKinematicVerb copies it
    // into verbBlend, and the per-edit setters keep verbBlend live. Keytimes
    // live in verbBlend.ex[i].key (editable post-build); the two files reuse
    // motionSlots 0/1 for the file combo + preview + active-frame window.
    mograph::VerbBlend verbBlend;
    std::vector<std::string> verbFiles{"WalkLoopA.bvh", "SneakLoopA.bvh"};
    std::vector<std::string> verbTags{"슬픔"};                  // 1..2 adverb names
    std::vector<std::array<float, 2>> verbAdverb{{0, 0}, {100, 0}};  // [clip][tag]%
    std::array<float, 2> verbQuery{50.0f, 0.0f};               // live adverb query %
    std::string verbStatus;                  // last build report for the GUI
    // Blended-result preview: translucent strobe of the live 2-motion blend
    // cycle (morphs as the adverb slider / keytimes change). One-shot opaque
    // playback reuses previewPlay = -1 (shared with the blend space).
    bool verbPreview = false;
    // Keytime-tuning preview (mode 5/6): which motion's keytime "filmstrip" to
    // render — a static translucent ghost of that clip at EACH of its 5 keytime
    // frames, spread along X so the gait registration is readable and re-poses
    // live as a keytime slider is dragged (tune timing by watching the pose).
    // -1 = off; one motion at a time (the strips would overlap otherwise).
    int verbKtPreview = -1;
    // Active N-blend preset (mode 6): -1 = 자율선택(manual), else index into
    // verbPresets(). A manual file/motion edit reverts to -1.
    int verbPreset = -1;
    // Allow extrapolation past an example (signed RBF weights). On = over-driving
    // the adverb slider past a motion exaggerates it; off = convex (clamped to
    // the two motions). Mirrors verbBlend.convexWeights (= !verbExtrapolate).
    bool verbExtrapolate = true;
    // Mode 5 = exactly two motions; mode 6 = the N-motion variant. Both drive
    // the same verbBlend (already N-way) and share every sample/preview/rebase
    // path — they differ only in how many motions the GUI exposes.
    bool verbActive() const {
        return (motionMode == 5 || motionMode == 6) && verbBlend.ready();
    }

    // ── Per-clip motion-selection slots (backs the reusable selector) ──
    // One slot per selectable clip. Modes that pick clips index into these:
    // transition/DTW use 0,1; the blend space uses 0..N-1. The file a slot
    // *shows* comes from the mode's own storage (slotFile) — transFileA/B for
    // 2/3, blendSpaceFiles[i] for 4 — so there is one source of truth for the
    // selected files; the slot owns only the preview toggle, color, and cache.
    // Available before any build (samples the selected files directly), exactly
    // like the old A/B preview.
    struct MotionSlot {
        bool preview = false;                          // strobe-ghost toggle
        std::array<float, 3> color{0.6f, 0.6f, 0.65f};  // strobe / play / tint
        mograph::Clip cachedClip;                      // sampled ghost cache
        std::string cachedFile;                        // which file the cache holds
        // Active-frame window [rangeStart, rangeEnd] over cachedClip: the only
        // frames the effect + preview use, and whose FIRST frame anchors to the
        // kinematic object. rangeEnd < 0 = clip end (resolved once cached); a
        // file change resets the window to full (see on_kin_slot_file).
        int rangeStart = 0;
        int rangeEnd = -1;
        // Keytime source loop (verb modes): which detected gait cycle in the
        // window the footstep keytimes come from. 0 = first, 1 = next loop.
        int loopSel = 0;
    };
    std::vector<MotionSlot> motionSlots{
        {false, {0.95f, 0.25f, 0.25f}, {}, ""},  // slot 0 — red
        {false, {0.30f, 0.45f, 0.95f}, {}, ""},  // slot 1 — blue
        {false, {0.30f, 0.85f, 0.40f}, {}, ""},  // slot 2 — green
        {false, {0.95f, 0.78f, 0.25f}, {}, ""},  // slot 3 — amber
    };
    // Blend playback tint: when on (DTW mode only), the live body is colored by
    // mixing slot 0/1 colors by the current blend source weight.
    bool blendColorize = false;
    // Blend-result preview (blend-space mode only): a translucent strobe of the
    // live N-way blended gait cycle, recomputed every frame from the session's
    // current cursor — so dragging the pad morphs the ghost in real time.
    bool blendPreview = false;
    // One-shot opaque playback (paused-only, render-loop driven): 0 none, else
    // (slot index + 1). Clears itself at clip end.
    int previewPlay = 0;
    double previewPlayTime = 0.0;

    // Slots the current mode exposes, and the file each one selects (from the
    // mode's own storage). The single place that knows the mode→slot mapping.
    int numSlots() const {
        if (motionMode == 4) return int(blendSpaceFiles.size());
        if (motionMode == 6) return int(verbFiles.size());  // N-motion blend
        if (motionMode == 2 || motionMode == 3 || motionMode == 5) return 2;
        return 0;
    }
    // Resolve slot i's active-frame window against its cached clip length.
    // rangeEnd<0 / stale ⇒ full clip; clamped, start ≤ end. {0,0} if not cached.
    std::array<int, 2> slotRange(int i) const {
        if (i < 0 || i >= int(motionSlots.size())) return {0, 0};
        const int nf = int(motionSlots[i].cachedClip.frames.size());
        if (nf <= 0) return {0, 0};
        int a = motionSlots[i].rangeStart;
        int b = motionSlots[i].rangeEnd < 0 ? nf - 1 : motionSlots[i].rangeEnd;
        a = a < 0 ? 0 : (a > nf - 1 ? nf - 1 : a);
        b = b < 0 ? 0 : (b > nf - 1 ? nf - 1 : b);
        if (b < a) b = a;
        return {a, b};
    }
    std::string slotFile(int i) const {
        if (motionMode == 5 || motionMode == 6)
            return i >= 0 && i < int(verbFiles.size()) ? verbFiles[i]
                                                       : std::string();
        if (motionMode == 4)
            return i >= 0 && i < int(blendSpaceFiles.size()) ? blendSpaceFiles[i]
                                                             : std::string();
        if (motionMode == 2 || motionMode == 3) {
            const std::string& f = (i == 0) ? transFileA
                                            : (i == 1 ? transFileB : transFileA);
            if (!f.empty()) return f;
            return std::filesystem::path(params.filePath).filename().string();
        }
        return std::string();
    }

    // Reference skeleton (the loaded motion's), cached for preview FK + clip
    // sampling. Invalidated on reloadMotion.
    mograph::Skeleton skelCache_;
    bool skelValid_ = false;
    const mograph::Skeleton& skel() {
        if (!skelValid_) {
            skelCache_ = mograph::Skeleton::extract(motion);
            skelValid_ = true;
        }
        return skelCache_;
    }

    bool graphActive() const {
        if (!graphSession.ready()) return false;
        if (motionMode == 1)
            return graphSession.mode == mograph::Session::Mode::RandomWalk;
        if (motionMode == 2)
            return graphSession.mode == mograph::Session::Mode::Transition;
        if (motionMode == 3)
            return graphSession.mode == mograph::Session::Mode::Blend;
        if (motionMode == 4)
            return graphSession.mode == mograph::Session::Mode::BlendSpace;
        return false;
    }

    // Playback length of whatever the current mode plays (walks are
    // effectively unbounded).
    double activeDuration() const {
        if (verbActive()) return verbBlend.cycleSec;  // loops one gait cycle
        return graphActive() ? graphSession.duration()
                             : (double)motion.duration();
    }

    explicit MeshKinematicInitializer(ParamsType p) : params(std::move(p)) {}

    // Factory: loads + builds first because InitializerParams needs the
    // vertex/facet/edge counts at construction. Returns nullptr (with
    // `err` filled) on parse failure.
    static MeshKinematicInitializer* create(const std::string& path,
                                            tinym::vec3 center,
                                            PR targetHeight, PR mass,
                                            std::string* err = nullptr) {
        bvh::Motion m = bvh::load(path, err);
        if (!m.valid()) return nullptr;
        kinematic::BodyProxy proxy;
        proxy.build(m);
        auto* init = new MeshKinematicInitializer(ParamsType(
            path, center, proxy.numVerts, proxy.numFacets(), proxy.numEdges,
            targetHeight, mass));
        init->motion = std::move(m);
        init->proxy = std::move(proxy);
        return init;
    }

    // Swap the motion source in place. Topology may change (different
    // skeleton), so the caller must mark the scene dirty for a re-pack.
    bool reloadMotion(const std::string& path, std::string* err = nullptr) {
        bvh::Motion m = bvh::load(path, err);
        if (!m.valid()) return false;
        kinematic::BodyProxy p;
        p.build(m);
        motion = std::move(m);
        proxy = std::move(p);
        params.filePath = path;
        params.numPoints = proxy.numVerts;
        params.numFacets = proxy.numFacets();
        params.numEdges = proxy.numEdges;
        localTime = 0.0;
        // A built graph session references the OLD motion's skeleton/units;
        // keep it from driving the new proxy. Builders reload first, then
        // rebuild the session, so this only ever drops stale state.
        graphSession.clear();
        verbBlend = mograph::VerbBlend{};  // 2-blend references old skeleton too
        verbStatus.clear();
        // Skeleton + preview caches reference the old motion — drop them.
        skelValid_ = false;
        for (auto& s : motionSlots) {
            s.cachedFile.clear();
            s.cachedClip.frames.clear();
        }
        previewPlay = 0;
        invalidateRebase();
        return true;
    }

    float normScale() const {
        const float h = motion.restHeight();
        return h > 1e-6f ? float(params.targetHeight) / h : 1.0f;
    }

    // FK pose at `timeSec` → `out` (3*numPoints PR). Base variant used by
    // initialize/populatePreview: pose normalized + translated to center,
    // NO user scale/rotation — Scene::pack bakes those afterwards, same
    // as every other initializer.
    void writePoseBase(double timeSec, PR* out) {
        writePose(timeSec, tinym::vec3(1, 1, 1), Quat{}, params.center, out);
    }

    // Full variant used by the per-frame update, which overwrites the
    // pack-baked geometry and therefore must apply the user transform
    // itself (scale → rotate → translate around the FK origin).
    void writePose(double timeSec, tinym::vec3 userScale, const Quat& rot,
                   tinym::vec3 position, PR* out) {
        bvh::Pose pose;
        sampleWorldPose(timeSec, pose);
        applyRootRebase(pose);
        scratch_.resize(size_t(proxy.numVerts) * 3);
        proxy.writeVertices(
            pose, normScale(),
            {float(userScale.x), float(userScale.y), float(userScale.z)},
            quatToMat3(rot),
            {float(position.x), float(position.y), float(position.z)},
            scratch_.data());
        for (size_t i = 0; i < scratch_.size(); ++i) out[i] = PR(scratch_[i]);
    }

    // Sample the active source (graph session when built, else the raw clip)
    // into a world-space pose. Shared by playback and the frame-0 rebase.
    void sampleWorldPose(double timeSec, bvh::Pose& pose) {
        if (verbActive()) {
            verbBlend.sample(timeSec, pose);
            // Skeleton mismatch ⇒ a blend built against a different file than
            // the proxy — fall back rather than feed writeVertices a wrong-size
            // pose (the builder gates on compatible(); belt + braces).
            if (pose.world.size() != motion.joints.size())
                motion.evaluate(float(timeSec), loop, pose);
            return;
        }
        if (graphActive()) {
            graphSession.samplePose(timeSec, pose);
            // Joint-count mismatch would mean a session built against a
            // different skeleton than the proxy — fall back rather than
            // index out of bounds (builders prevent this; belt+braces).
            if (pose.world.size() != motion.joints.size())
                motion.evaluate(float(timeSec), loop, pose);
        } else {
            motion.evaluate(float(timeSec), loop, pose);
        }
    }

    // Cached live-playback rebase (frame-0 xz + yaw). Cached because for long
    // random walks WalkBaker evicts old frames, so re-sampling frame 0 later
    // would shift the origin and jump the body. Invalidated when the playback
    // source changes (mode/build/file/seed).
    bool rebaseValid_ = false;
    float rebaseTh_ = 0.0f, rebaseX0_ = 0.0f, rebaseZ0_ = 0.0f;
    void invalidateRebase() { rebaseValid_ = false; }

    // Re-root: the BVH frame-0 root xz position + heading (yaw) are removed so
    // the kinematic object's transform alone places the body (frame-0 root
    // sits at the object origin). Y is kept so feet stay grounded. Gravity-
    // preserving — same family as the motion-graph XformXZ.
    void applyRootRebase(bvh::Pose& pose) {
        if (pose.world.empty()) return;
        // Blend space (both root modes): clips are zeroed at the root and travel
        // is integrated into samplePose, so there is nothing to re-root — and we
        // must NOT sample at t=0 here, which would corrupt the travel dt clock.
        if (motionMode == 4 && graphActive()) return;
        // 2-motion keytime blend: clips are pinned in place (root xz=0, yaw=0),
        // so there is nothing to re-root — the object transform alone places it.
        if (verbActive()) return;
        if (!rebaseValid_) {
            bvh::Pose p0;
            sampleWorldPose(0.0, p0);
            if (p0.world.empty()) return;
            const auto& R0 = p0.world[0].R;
            rebaseTh_ = std::atan2(R0[2], R0[0]);
            rebaseX0_ = p0.world[0].t[0];
            rebaseZ0_ = p0.world[0].t[2];
            rebaseValid_ = true;
        }
        const float c = std::cos(rebaseTh_), s = std::sin(rebaseTh_);
        for (auto& jx : pose.world) {
            const float dx = jx.t[0] - rebaseX0_, dz = jx.t[2] - rebaseZ0_;
            jx.t[0] = c * dx - s * dz;
            jx.t[2] = s * dx + c * dz;
        }
    }

    // In-place: rotate the whole pose by -yaw(root0) about Y and subtract
    // root0's xz, leaving Y untouched. (writeVertices only reads joint
    // positions, so re-rooting positions re-roots the visible body.)
    static void rebaseXZYaw(bvh::Pose& pose, const bvh::JointXform& root0) {
        const float th = std::atan2(root0.R[2], root0.R[0]);
        const float c = std::cos(th), s = std::sin(th);
        const float x0 = root0.t[0], z0 = root0.t[2];
        for (auto& jx : pose.world) {
            const float dx = jx.t[0] - x0, dz = jx.t[2] - z0;
            jx.t[0] = c * dx - s * dz;
            jx.t[2] = s * dx + c * dz;
        }
    }

    // Motion-preview ghost: proxy world verts for clip `c` frame `f`, re-rooted
    // exactly like the live body (frame-0 xz+yaw removed, Y kept) and placed by
    // the body transform — so it rests at the object's position, feet grounded.
    void writeGhost(const mograph::Clip& c, int f, tinym::vec3 userScale,
                    const Quat& rot, tinym::vec3 position,
                    std::vector<float>& out, int anchorFrame = 0) {
        if (c.frames.empty()) return;
        const int nf = int(c.frames.size());
        const int fi = f < 0 ? 0 : (f >= nf ? nf - 1 : f);
        const int af = anchorFrame < 0 ? 0 : (anchorFrame >= nf ? nf - 1 : anchorFrame);
        bvh::Pose pose, pose0;
        mograph::fk(skel(), c.frames[fi], pose);
        mograph::fk(skel(), c.frames[af], pose0);  // anchor = range-start frame
        rebaseXZYaw(pose, pose0.world[0]);
        out.resize(size_t(proxy.numVerts) * 3);
        proxy.writeVertices(
            pose, normScale(),
            {float(userScale.x), float(userScale.y), float(userScale.z)},
            quatToMat3(rot),
            {float(position.x), float(position.y), float(position.z)},
            out.data());
    }

    // Like writeGhost but samples the clip at a continuous time (lerp between
    // frames) — for the smooth one-shot preview playback.
    void writeGhostAtTime(const mograph::Clip& c, double timeSec,
                          tinym::vec3 userScale, const Quat& rot,
                          tinym::vec3 position, std::vector<float>& out,
                          int anchorFrame = 0) {
        if (c.frames.empty()) return;
        const int nf = int(c.frames.size());
        const int af = anchorFrame < 0 ? 0 : (anchorFrame >= nf ? nf - 1 : anchorFrame);
        double ff = timeSec / (c.dt > 0 ? c.dt : 1.0);
        if (ff < 0) ff = 0;
        if (ff > nf - 1) ff = nf - 1;
        const int f0 = int(ff);
        const int f1 = f0 + 1 < nf ? f0 + 1 : f0;
        const float frac = float(ff - f0);
        mograph::LocalPose mid;
        mograph::blendPose(c.frames[f0], c.frames[f1], 1.0f - frac, mid);
        bvh::Pose pose, pose0;
        mograph::fk(skel(), mid, pose);
        mograph::fk(skel(), c.frames[af], pose0);  // anchor = range-start frame
        rebaseXZYaw(pose, pose0.world[0]);
        out.resize(size_t(proxy.numVerts) * 3);
        proxy.writeVertices(
            pose, normScale(),
            {float(userScale.x), float(userScale.y), float(userScale.z)},
            quatToMat3(rot),
            {float(position.x), float(position.y), float(position.z)},
            out.data());
    }

    // Strobe-frame of the live N-way blended result at normalized phase
    // [0,1), re-rooted like the live body. Samples the session at the CURRENT
    // cursor every call (samplePose reads graphSession.cursor), so the blend
    // preview ghost morphs in real time as the pad is dragged. No-op unless a
    // blend space is built.
    void writeBlendGhost(double phase01, bool loop, tinym::vec3 userScale,
                         const Quat& rot, tinym::vec3 position,
                         std::vector<float>& out) {
        out.clear();
        if (!graphActive() ||
            graphSession.mode != mograph::Session::Mode::BlendSpace)
            return;
        bvh::Pose pose, pose0;
        // Rebase reference = phase-0 pose (always looped) so the ghost root
        // sits at the object origin regardless of the play/clamp mode.
        if (!graphSession.sampleBlendPhase(float(phase01), loop, pose)) return;
        graphSession.sampleBlendPhase(0.0f, true, pose0);
        if (pose.world.empty() || pose0.world.empty()) return;
        rebaseXZYaw(pose, pose0.world[0]);
        // Show the curving travel too, so the preview matches the live body:
        // integrate the blended root velocity over ONE cycle from phase 0 to
        // phase01, then rotate (heading) + translate the ghost. Strobe frames
        // then spread along the curved path; the one-shot ghost follows it.
        {
            const auto& ses = graphSession;
            std::vector<float> w;
            ses.blendWeights(ses.cursor, w);
            const int STEPS = 24;
            const double subDt = (double(phase01) / STEPS) * ses.blendCycleSec;
            double yaw = 0.0, px = 0.0, pz = 0.0;
            for (int s = 0; s < STEPS; ++s) {
                const float ph = float(double(phase01) * (s + 0.5) / STEPS);
                const auto v = ses.blendRootVel(ph, w);
                yaw += v[2] * subDt;
                const double cy = std::cos(yaw), sy = std::sin(yaw);
                px += (cy * v[0] - sy * v[1]) * subDt;
                pz += (sy * v[0] + cy * v[1]) * subDt;
            }
            const float cyf = std::cos(float(yaw)), syf = std::sin(float(yaw));
            for (auto& jx : pose.world) {
                const float x = jx.t[0], z = jx.t[2];
                jx.t[0] = cyf * x - syf * z + float(px);  // re-head + translate
                jx.t[2] = syf * x + cyf * z + float(pz);
            }
        }
        out.resize(size_t(proxy.numVerts) * 3);
        proxy.writeVertices(
            pose, normScale(),
            {float(userScale.x), float(userScale.y), float(userScale.z)},
            quatToMat3(rot),
            {float(position.x), float(position.y), float(position.z)},
            out.data());
    }

    // Strobe-frame of the live 2-motion keytime blend at normalized phase
    // [0,1). Samples verbBlend.sampleMixed at the CURRENT adverb query +
    // keytimes every call, so the preview morphs in real time as the
    // slider/keytimes change. sampleMixed re-roots each clip exactly like the
    // live body (xz + yaw removed) but omits the per-cycle travel, so the ghost
    // faces the kinematic forward and sits at the object origin (in place).
    // No-op unless a 2-motion blend is built.
    void writeVerbGhost(double phase01, tinym::vec3 userScale, const Quat& rot,
                        tinym::vec3 position, std::vector<float>& out) {
        out.clear();
        if (!verbActive()) return;
        mograph::LocalPose lp;
        verbBlend.sampleMixed(float(phase01), lp);
        bvh::Pose pose;
        mograph::fk(verbBlend.skel, lp, pose);
        if (pose.world.empty() || pose.world.size() != motion.joints.size())
            return;
        out.resize(size_t(proxy.numVerts) * 3);
        proxy.writeVertices(
            pose, normScale(),
            {float(userScale.x), float(userScale.y), float(userScale.z)},
            quatToMat3(rot),
            {float(position.x), float(position.y), float(position.z)},
            out.data());
    }

    void initialize(MeshState<BE, PR>& state,
                    MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params);
        adjacency.memoryAllocation(params);

        // Current localTime (not 0) so a structural re-pack mid-playback
        // realizes the frame the user is looking at.
        writePoseBase(localTime, state.x.ptr);

        if (adjacency.vertexAdjFacets.ptr) return;
        for (Index f = 0; f < params.numFacets * 3; ++f)
            adjacency.facets[f] = proxy.facets[f];

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        std::vector<PR> x(size_t(params.numPoints) * 3);
        writePoseBase(localTime, x.data());
        preview.x.assign(x.begin(), x.end());
        preview.facets.assign(proxy.facets.begin(), proxy.facets.end());
        preview.recomputeNormals();
    }

    InitializerParams<PR>* getParams() override { return &params; }

  private:
    std::vector<float> scratch_;
};

template <typename BE, typename PR>
