#ifndef YSIM_MOTION_CLIP_HPP
#define YSIM_MOTION_CLIP_HPP

// CPU-pure pose-track representation on top of bvh_motion.hpp, following the
// same header-only / no-Metal / no-GLFW / no-Eigen convention.
//
// bvh::Motion stores raw Euler channel rows — fine for single-clip playback,
// wrong for anything that mixes motions: Euler channels can't be blended
// across clips safely and per-file units vary ~7x across assets/BVH. This
// header re-expresses a motion as a track of LocalPose (root position in
// *reference* units + one local quaternion per joint) against a single
// reference Skeleton, which is exactly the form motion graphs, transitions,
// and future motion *blending* operate on:
//
//   bvh::Motion --sampleClip()--> Clip{LocalPose[]} --fk()--> bvh::Pose
//                                       |
//                              blendPose()/XformXZ      (graph/blend layer)
//
// Retargeting model: BVH rest poses carry no joint rotations (offsets are
// translation-only), so local rotations transfer between same-structure
// skeletons directly; only the root translation needs the unit fix
// (refHeight/srcHeight). Skeleton::compatible() gates this — same joint
// names + parents + height-normalized offset directions within tolerance.

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "bvh_motion.hpp"

namespace mograph {

// ---- quaternion -------------------------------------------------------------

// Minimal unit quaternion (w, x, y, z), Hamilton convention; mat3 layout is
// row-major to match bvh::JointXform::R.
struct Quatf {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    static Quatf identity() { return {}; }

    static Quatf axisAngle(float ax, float ay, float az, float rad) {
        const float h = 0.5f * rad, s = std::sin(h);
        return {std::cos(h), ax * s, ay * s, az * s};
    }

    // Rotation about +Y by `rad`; matches bvh::detail::axisRotation(Yrot).
    static Quatf yaw(float rad) { return axisAngle(0.0f, 1.0f, 0.0f, rad); }

    Quatf operator*(const Quatf& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w};
    }

    float dot(const Quatf& b) const { return w * b.w + x * b.x + y * b.y + z * b.z; }

    Quatf normalized() const {
        const float n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < 1e-12f) return identity();
        return {w / n, x / n, y / n, z / n};
    }

    void toMat3(std::array<float, 9>& m) const {
        const float xx = x * x, yy = y * y, zz = z * z;
        const float xy = x * y, xz = x * z, yz = y * z;
        const float wx = w * x, wy = w * y, wz = w * z;
        m = {1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
             2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
             2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)};
    }

    // Shepperd's method on a row-major rotation matrix.
    static Quatf fromMat3(const std::array<float, 9>& m) {
        Quatf q;
        const float tr = m[0] + m[4] + m[8];
        if (tr > 0.0f) {
            const float s = std::sqrt(tr + 1.0f) * 2.0f;
            q.w = 0.25f * s;
            q.x = (m[7] - m[5]) / s;
            q.y = (m[2] - m[6]) / s;
            q.z = (m[3] - m[1]) / s;
        } else if (m[0] > m[4] && m[0] > m[8]) {
            const float s = std::sqrt(1.0f + m[0] - m[4] - m[8]) * 2.0f;
            q.w = (m[7] - m[5]) / s;
            q.x = 0.25f * s;
            q.y = (m[1] + m[3]) / s;
            q.z = (m[2] + m[6]) / s;
        } else if (m[4] > m[8]) {
            const float s = std::sqrt(1.0f + m[4] - m[0] - m[8]) * 2.0f;
            q.w = (m[2] - m[6]) / s;
            q.x = (m[1] + m[3]) / s;
            q.y = 0.25f * s;
            q.z = (m[5] + m[7]) / s;
        } else {
            const float s = std::sqrt(1.0f + m[8] - m[0] - m[4]) * 2.0f;
            q.w = (m[3] - m[1]) / s;
            q.x = (m[2] + m[6]) / s;
            q.y = (m[5] + m[7]) / s;
            q.z = 0.25f * s;
        }
        return q.normalized();
    }
};

// Shortest-arc spherical interpolation (nlerp fallback when nearly parallel).
inline Quatf slerp(const Quatf& a, Quatf b, float t) {
    float d = a.dot(b);
    if (d < 0.0f) { b = {-b.w, -b.x, -b.y, -b.z}; d = -d; }
    if (d > 0.9995f) {
        return Quatf{a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t,
                     a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t}
            .normalized();
    }
    const float th = std::acos(d > 1.0f ? 1.0f : d);
    const float sa = std::sin((1.0f - t) * th) / std::sin(th);
    const float sb = std::sin(t * th) / std::sin(th);
    return {a.w * sa + b.w * sb, a.x * sa + b.x * sb,
            a.y * sa + b.y * sb, a.z * sa + b.z * sb};
}

// ---- skeleton ---------------------------------------------------------------

// Joint topology + rest offsets extracted from a bvh::Motion, in that file's
// units. One Skeleton per session acts as the reference every clip is
// retargeted onto, so the proxy mesh topology never changes mid-session.
struct Skeleton {
    struct J {
        std::string name;
        int parent = -1;
        std::array<float, 3> offset{};
        bool isEndSite = false;
    };
    std::vector<J> joints;
    float height = 0.0f;  // restHeight() of the source motion

    static Skeleton extract(const bvh::Motion& m) {
        Skeleton s;
        s.joints.reserve(m.joints.size());
        for (const auto& j : m.joints)
            s.joints.push_back({j.name, j.parent, j.offset, j.isEndSite});
        s.height = m.restHeight();
        return s;
    }

    // Retarget gate: identical joint names + parent structure, and rest
    // offsets that agree in height-normalized space within `tol` (fraction
    // of body height). Catches same-family files that only differ in units
    // (WalkLoopA vs walkCurve ≈ 0.6% residual) while rejecting genuinely
    // different rest poses (j_Uber offsets point elsewhere entirely).
    bool compatible(const Skeleton& o, float tol = 0.08f) const {
        if (joints.size() != o.joints.size()) return false;
        if (height <= 1e-6f || o.height <= 1e-6f) return false;
        for (size_t i = 0; i < joints.size(); ++i) {
            const J& a = joints[i];
            const J& b = o.joints[i];
            if (a.parent != b.parent || a.isEndSite != b.isEndSite ||
                a.name != b.name)
                return false;
            float d2 = 0.0f;
            for (int k = 0; k < 3; ++k) {
                const float d = a.offset[k] / height - b.offset[k] / o.height;
                d2 += d * d;
            }
            if (d2 > tol * tol) return false;
        }
        return true;
    }
};

// ---- local pose -------------------------------------------------------------

// Root world translation (reference units) + per-joint local rotation.
// End Sites carry identity rotations; interior position channels (absent in
// all assets/BVH files) are not represented.
struct LocalPose {
    std::array<float, 3> rootPos{};
    std::vector<Quatf> rot;  // size == Skeleton::joints.size()
};

// out = a*(wa) + b*(1-wa); roots lerp, rotations slerp. The future N-way
// blend extends this signature (poses[], weights[]).
inline void blendPose(const LocalPose& a, const LocalPose& b, float wa,
                      LocalPose& out) {
    const float wb = 1.0f - wa;
    out.rot.resize(a.rot.size());
    for (int k = 0; k < 3; ++k)
        out.rootPos[k] = a.rootPos[k] * wa + b.rootPos[k] * wb;
    for (size_t j = 0; j < a.rot.size(); ++j)
        out.rot[j] = slerp(a.rot[j], b.rot[j], wb);
}

// Forward kinematics against the reference skeleton. Parent always precedes
// child (bvh::Motion guarantee, preserved by Skeleton::extract).
inline void fk(const Skeleton& s, const LocalPose& p, bvh::Pose& out) {
    out.world.resize(s.joints.size());
    std::array<float, 9> Rl;
    for (size_t j = 0; j < s.joints.size(); ++j) {
        const auto& jt = s.joints[j];
        if (jt.parent < 0) {
            p.rot[j].toMat3(out.world[j].R);
            out.world[j].t = p.rootPos;
            continue;
        }
        const bvh::JointXform& pa = out.world[jt.parent];
        p.rot[j].toMat3(Rl);
        bvh::detail::mul33(pa.R, Rl, out.world[j].R);
        const auto t = bvh::detail::mulV(pa.R, jt.offset);
        out.world[j].t = {pa.t[0] + t[0], pa.t[1] + t[1], pa.t[2] + t[2]};
    }
}

// ---- ground-plane rigid transform -------------------------------------------

// Rotation about +Y followed by translation in the floor plane — the only
// alignment family motion graphs may apply (Kovar §3.2): it preserves
// gravity. apply(): p' = R_yaw p + (tx, 0, tz).
struct XformXZ {
    float theta = 0.0f, tx = 0.0f, tz = 0.0f;

    void applyPoint(std::array<float, 3>& p) const {
        const float c = std::cos(theta), s = std::sin(theta);
        const float x = c * p[0] + s * p[2];
        const float z = -s * p[0] + c * p[2];
        p[0] = x + tx;
        p[2] = z + tz;
    }

    void applyPose(LocalPose& p) const {
        applyPoint(p.rootPos);
        if (!p.rot.empty()) p.rot[0] = (Quatf::yaw(theta) * p.rot[0]).normalized();
    }

    // Composition (this ∘ inner): apply `inner` first, then this.
    XformXZ compose(const XformXZ& inner) const {
        const float c = std::cos(theta), s = std::sin(theta);
        XformXZ r;
        r.theta = theta + inner.theta;
        r.tx = c * inner.tx + s * inner.tz + tx;
        r.tz = -s * inner.tx + c * inner.tz + tz;
        return r;
    }
};

// ---- clip sampling / retargeting ---------------------------------------------

// A motion resampled onto the reference skeleton at fixed dt.
struct Clip {
    std::string name;
    std::vector<LocalPose> frames;
    float dt = 1.0f / 30.0f;

    float duration() const { return float(frames.size()) * dt; }
};

namespace detail {

// One frame row (optionally channel-lerped with `b` toward row `rowB`) →
// LocalPose. `rootScale` converts source units to reference units.
inline void poseFromRows(const bvh::Motion& m, const float* rowA,
                         const float* rowB, float b, float rootScale,
                         LocalPose& out) {
    out.rot.assign(m.joints.size(), Quatf::identity());
    for (size_t j = 0; j < m.joints.size(); ++j) {
        const bvh::Joint& jt = m.joints[j];
        Quatf q = Quatf::identity();
        std::array<float, 3> pos{};
        for (size_t k = 0; k < jt.channels.size(); ++k) {
            const size_t col = jt.channelStart + k;
            float v;
            const bool isPos = jt.channels[k] == bvh::Channel::Xpos ||
                               jt.channels[k] == bvh::Channel::Ypos ||
                               jt.channels[k] == bvh::Channel::Zpos;
            if (isPos) {
                v = rowA[col] + (rowB[col] - rowA[col]) * b;
            } else {
                // Shortest-arc channel lerp, same rule as Motion::evaluate.
                float d = std::fmod(rowB[col] - rowA[col], 360.0f);
                if (d > 180.0f) d -= 360.0f;
                if (d < -180.0f) d += 360.0f;
                v = rowA[col] + d * b;
            }
            const float rad = v * 3.14159265358979323846f / 180.0f;
            switch (jt.channels[k]) {
                case bvh::Channel::Xpos: pos[0] += v; break;
                case bvh::Channel::Ypos: pos[1] += v; break;
                case bvh::Channel::Zpos: pos[2] += v; break;
                case bvh::Channel::Xrot: q = q * Quatf::axisAngle(1, 0, 0, rad); break;
                case bvh::Channel::Yrot: q = q * Quatf::axisAngle(0, 1, 0, rad); break;
                case bvh::Channel::Zrot: q = q * Quatf::axisAngle(0, 0, 1, rad); break;
            }
        }
        out.rot[j] = q.normalized();
        if (jt.parent < 0) {
            out.rootPos = {(jt.offset[0] + pos[0]) * rootScale,
                           (jt.offset[1] + pos[1]) * rootScale,
                           (jt.offset[2] + pos[2]) * rootScale};
        }
        // Interior position channels (none in assets/BVH) are dropped: the
        // reference skeleton's offsets are authoritative below the root.
    }
}

}  // namespace detail

// Samples `m` as a Clip in `ref` units at `ref`-side dt. Caller must have
// checked Skeleton::compatible. Returns false for degenerate motions.
inline bool sampleClip(const bvh::Motion& m, const Skeleton& ref, float refDt,
                       std::string name, Clip& out) {
    if (!m.valid() || refDt <= 0.0f) return false;
    const float srcH = m.restHeight();
    if (srcH <= 1e-6f || ref.height <= 1e-6f) return false;
    const float rootScale = ref.height / srcH;

    out.name = std::move(name);
    out.dt = refDt;
    out.frames.clear();

    if (std::fabs(m.frameTime - refDt) < 1e-5f) {
        out.frames.resize(m.numFrames);
        for (bvh::Index f = 0; f < m.numFrames; ++f) {
            const float* row = m.data.data() + size_t(f) * m.numChannels;
            detail::poseFromRows(m, row, row, 0.0f, rootScale, out.frames[f]);
        }
    } else {
        // Resample by channel lerp at refDt over the source duration.
        const int n = int(m.duration() / refDt) + 1;
        out.frames.resize(size_t(n));
        for (int f = 0; f < n; ++f) {
            float ff = (float(f) * refDt) / m.frameTime;
            if (ff > float(m.numFrames - 1)) ff = float(m.numFrames - 1);
            const bvh::Index f0 = bvh::Index(ff);
            const bvh::Index f1 = f0 + 1 < m.numFrames ? f0 + 1 : f0;
            const float* r0 = m.data.data() + size_t(f0) * m.numChannels;
            const float* r1 = m.data.data() + size_t(f1) * m.numChannels;
            detail::poseFromRows(m, r0, r1, ff - std::floor(ff), rootScale,
                                 out.frames[f]);
        }
    }
    return !out.frames.empty();
}

}  // namespace mograph

#endif  // YSIM_MOTION_CLIP_HPP
