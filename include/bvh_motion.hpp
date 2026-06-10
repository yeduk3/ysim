#ifndef YSIM_BVH_MOTION_HPP
#define YSIM_BVH_MOTION_HPP

// CPU-pure BVH (Biovision Hierarchy) motion-capture loader + forward
// kinematics evaluator. Header-only and free of Metal / GLFW / Eigen
// dependencies, following the primitive_geometry.hpp convention, so a test
// harness can exercise parsing and FK without a GPU device.
//
// The runtime-side kinematic body (src/main.cpp) wraps this: it evaluates
// world-space joint positions for the current simulation time and rebuilds
// a sphere/cylinder proxy mesh from them. Keeping pose evaluation separate
// from the proxy mesh is deliberate — a future skinned model (e.g. SMPL)
// consumes the same per-joint transforms.
//
// Format notes (verified against assets/BVH/*.bvh):
//   - ROOT has 6 channels (Xposition..Zrotation in arbitrary listed order),
//     interior joints typically 3 rotation channels.
//   - Rotations are degrees, applied in the *listed channel order*.
//   - "End Site" blocks may or may not carry a name token.
//   - Units vary wildly across files (WalkLoopA hip offset ~0.5,
//     j_Uber ~180) — callers should normalize via restHeight().

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace bvh {

using Index = uint32_t;

enum class Channel : uint8_t { Xpos, Ypos, Zpos, Xrot, Yrot, Zrot };

struct Joint {
    std::string name;
    int parent = -1;                 // index into Motion::joints, -1 for root
    std::array<float, 3> offset{};   // rest offset from parent, file units
    std::vector<Channel> channels;   // empty for End Sites
    Index channelStart = 0;          // first column of this joint in a frame row
    bool isEndSite = false;
};

// Rigid transform accumulated down the joint chain. Column vectors:
// world = R * local + t.
struct JointXform {
    std::array<float, 9> R{1, 0, 0, 0, 1, 0, 0, 0, 1};  // row-major 3x3
    std::array<float, 3> t{};
};

struct Pose {
    // One entry per Motion::joints element (End Sites included).
    std::vector<JointXform> world;

    const std::array<float, 3>& position(size_t j) const { return world[j].t; }
};

struct Motion {
    std::vector<Joint> joints;   // pre-order; parent always precedes child
    Index numChannels = 0;       // columns per motion row
    Index numFrames = 0;
    float frameTime = 1.0f / 30.0f;  // seconds per frame
    std::vector<float> data;     // numFrames * numChannels, row-major

    bool valid() const {
        return !joints.empty() && numFrames > 0 && frameTime > 0.0f &&
               data.size() == size_t(numFrames) * numChannels;
    }

    float duration() const { return float(numFrames) * frameTime; }

    // World-space pose at `timeSec`. Channel values are linearly
    // interpolated between the two neighboring frames (fine for mocap's
    // small per-frame deltas; avoids stair-stepping at slow playback).
    // With loop=true time wraps over duration(); across the last->first
    // seam ROTATIONS interpolate (shortest arc) but POSITIONS snap —
    // locomotion clips translate the root over the clip (WalkLoopA drifts
    // ~11 units in Z), and lerping that across the seam sweeps the body
    // backward over one frame interval. Otherwise time clamps to the
    // final frame.
    void evaluate(float timeSec, bool loop, Pose& out) const;

    // Vertical extent (Y) of the rest skeleton (frame-0 translation,
    // identity rotations is NOT used — offsets only), for scale
    // normalization across files with different units.
    float restHeight() const;

  private:
    // seamWrap=true marks the looped last->first interval: position
    // channels hold f0's value instead of lerping.
    void evaluateFrameLerp(Index f0, Index f1, float a, Pose& out,
                           bool seamWrap = false) const;
};

// Parses `path`. On failure returns a Motion with valid() == false and, if
// `err` is non-null, a human-readable reason.
Motion load(const std::string& path, std::string* err = nullptr);

// ---- implementation --------------------------------------------------------

namespace detail {

inline void mul33(const std::array<float, 9>& A, const std::array<float, 9>& B,
                  std::array<float, 9>& out) {
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r * 3 + c] = A[r * 3 + 0] * B[0 + c] +
                             A[r * 3 + 1] * B[3 + c] +
                             A[r * 3 + 2] * B[6 + c];
}

inline std::array<float, 3> mulV(const std::array<float, 9>& A,
                                 const std::array<float, 3>& v) {
    return {A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
            A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
            A[6] * v[0] + A[7] * v[1] + A[8] * v[2]};
}

inline void axisRotation(Channel ch, float deg, std::array<float, 9>& out) {
    const float rad = deg * 3.14159265358979323846f / 180.0f;
    const float c = std::cos(rad), s = std::sin(rad);
    switch (ch) {
        case Channel::Xrot: out = {1, 0, 0, 0, c, -s, 0, s, c}; break;
        case Channel::Yrot: out = {c, 0, s, 0, 1, 0, -s, 0, c}; break;
        case Channel::Zrot: out = {c, -s, 0, s, c, 0, 0, 0, 1}; break;
        default:            out = {1, 0, 0, 0, 1, 0, 0, 0, 1}; break;
    }
}

}  // namespace detail

inline void Motion::evaluateFrameLerp(Index f0, Index f1, float a, Pose& out,
                                      bool seamWrap) const {
    out.world.resize(joints.size());
    const float* row0 = data.data() + size_t(f0) * numChannels;
    const float* row1 = data.data() + size_t(f1) * numChannels;

    std::array<float, 9> Rch, tmp;
    for (size_t j = 0; j < joints.size(); ++j) {
        const Joint& jt = joints[j];
        JointXform local;
        local.t = jt.offset;
        for (size_t k = 0; k < jt.channels.size(); ++k) {
            const size_t col = jt.channelStart + k;
            const bool isPos = jt.channels[k] == Channel::Xpos ||
                               jt.channels[k] == Channel::Ypos ||
                               jt.channels[k] == Channel::Zpos;
            float v;
            if (isPos) {
                // Across the loop seam a clip's accumulated root drift
                // (last vs first frame) is not motion — hold f0.
                v = seamWrap ? row0[col]
                             : row0[col] + (row1[col] - row0[col]) * a;
            } else {
                // Shortest-arc lerp: a 358° -> 2° pair must rotate +4°,
                // not -356° (Euler channels can wrap mid-clip too).
                float d = std::fmod(row1[col] - row0[col], 360.0f);
                if (d > 180.0f) d -= 360.0f;
                if (d < -180.0f) d += 360.0f;
                v = row0[col] + d * a;
            }
            switch (jt.channels[k]) {
                case Channel::Xpos: local.t[0] += v; break;
                case Channel::Ypos: local.t[1] += v; break;
                case Channel::Zpos: local.t[2] += v; break;
                default:
                    detail::axisRotation(jt.channels[k], v, Rch);
                    detail::mul33(local.R, Rch, tmp);
                    local.R = tmp;
                    break;
            }
        }
        if (jt.parent < 0) {
            out.world[j] = local;
        } else {
            const JointXform& p = out.world[jt.parent];
            JointXform w;
            detail::mul33(p.R, local.R, w.R);
            const auto pt = detail::mulV(p.R, local.t);
            w.t = {p.t[0] + pt[0], p.t[1] + pt[1], p.t[2] + pt[2]};
            out.world[j] = w;
        }
    }
}

inline void Motion::evaluate(float timeSec, bool loop, Pose& out) const {
    if (!valid()) {
        out.world.assign(joints.size(), JointXform{});
        return;
    }
    const float dur = duration();
    float t = timeSec;
    if (loop) {
        t = std::fmod(t, dur);
        if (t < 0.0f) t += dur;
        const float ff = t / frameTime;
        const Index f0 = Index(ff) % numFrames;
        const Index f1 = (f0 + 1) % numFrames;  // wrap last -> first
        evaluateFrameLerp(f0, f1, ff - std::floor(ff), out,
                          /*seamWrap=*/f1 < f0);
    } else {
        if (t <= 0.0f) { evaluateFrameLerp(0, 0, 0.0f, out); return; }
        const float ff = t / frameTime;
        if (ff >= float(numFrames - 1)) {
            evaluateFrameLerp(numFrames - 1, numFrames - 1, 0.0f, out);
            return;
        }
        const Index f0 = Index(ff);
        evaluateFrameLerp(f0, f0 + 1, ff - std::floor(ff), out);
    }
}

inline float Motion::restHeight() const {
    if (joints.empty()) return 0.0f;
    // Accumulate offsets down the hierarchy (identity rotations).
    std::vector<float> y(joints.size(), 0.0f);
    float lo = 0.0f, hi = 0.0f;
    for (size_t j = 0; j < joints.size(); ++j) {
        const float base = joints[j].parent < 0 ? 0.0f : y[joints[j].parent];
        y[j] = base + joints[j].offset[1];
        lo = std::min(lo, y[j]);
        hi = std::max(hi, y[j]);
    }
    return hi - lo;
}

inline Motion load(const std::string& path, std::string* err) {
    Motion m;
    auto fail = [&](const std::string& why) {
        if (err) *err = path + ": " + why;
        m = Motion{};
        return m;
    };

    std::ifstream file(path);
    if (!file) return fail("cannot open");
    std::stringstream ss;
    ss << file.rdbuf();

    std::string tok;
    if (!(ss >> tok) || tok != "HIERARCHY") return fail("missing HIERARCHY");

    std::vector<int> stack;  // open joint scopes
    bool sawRoot = false;
    while (ss >> tok) {
        if (tok == "ROOT" || tok == "JOINT" || tok == "End") {
            Joint jt;
            jt.parent = stack.empty() ? -1 : stack.back();
            if (tok == "End") {
                std::string site;
                if (!(ss >> site) || site != "Site") return fail("malformed End Site");
                jt.isEndSite = true;
                // Optional name before '{'.
                if (!(ss >> tok)) return fail("truncated End Site");
                if (tok != "{") {
                    jt.name = tok;
                    if (!(ss >> tok) || tok != "{") return fail("End Site missing {");
                } else {
                    jt.name = (jt.parent >= 0 ? m.joints[jt.parent].name
                                              : std::string("Root")) + "_end";
                }
            } else {
                if (jt.parent < 0 && tok != "ROOT") return fail("JOINT before ROOT");
                if (jt.parent >= 0 && tok == "ROOT") return fail("nested ROOT");
                sawRoot = true;
                if (!(ss >> jt.name)) return fail("joint missing name");
                if (!(ss >> tok) || tok != "{") return fail("joint missing {");
            }
            m.joints.push_back(jt);
            stack.push_back(int(m.joints.size()) - 1);
        } else if (tok == "OFFSET") {
            if (stack.empty()) return fail("OFFSET outside joint");
            Joint& jt = m.joints[stack.back()];
            if (!(ss >> jt.offset[0] >> jt.offset[1] >> jt.offset[2]))
                return fail("malformed OFFSET");
        } else if (tok == "CHANNELS") {
            if (stack.empty()) return fail("CHANNELS outside joint");
            Joint& jt = m.joints[stack.back()];
            int n = 0;
            if (!(ss >> n) || n < 0 || n > 6) return fail("malformed CHANNELS");
            jt.channelStart = m.numChannels;
            for (int i = 0; i < n; ++i) {
                if (!(ss >> tok)) return fail("truncated CHANNELS");
                if (tok == "Xposition") jt.channels.push_back(Channel::Xpos);
                else if (tok == "Yposition") jt.channels.push_back(Channel::Ypos);
                else if (tok == "Zposition") jt.channels.push_back(Channel::Zpos);
                else if (tok == "Xrotation") jt.channels.push_back(Channel::Xrot);
                else if (tok == "Yrotation") jt.channels.push_back(Channel::Yrot);
                else if (tok == "Zrotation") jt.channels.push_back(Channel::Zrot);
                else return fail("unknown channel " + tok);
            }
            m.numChannels += Index(n);
        } else if (tok == "}") {
            if (stack.empty()) return fail("unbalanced }");
            stack.pop_back();
        } else if (tok == "MOTION") {
            break;
        } else {
            return fail("unexpected token " + tok);
        }
    }
    if (!sawRoot) return fail("no ROOT joint");
    if (!stack.empty()) return fail("unclosed joint scope");

    // MOTION section: "Frames: N" then "Frame Time: t" then rows.
    if (!(ss >> tok) || tok.rfind("Frames", 0) != 0) return fail("missing Frames");
    if (tok == "Frames") { if (!(ss >> tok) || tok != ":") return fail("malformed Frames"); }
    long frames = 0;
    if (!(ss >> frames) || frames <= 0) return fail("bad frame count");
    if (!(ss >> tok) || tok != "Frame") return fail("missing Frame Time");
    if (!(ss >> tok) || tok.rfind("Time", 0) != 0) return fail("missing Frame Time");
    if (tok == "Time") { if (!(ss >> tok) || tok != ":") return fail("malformed Frame Time"); }
    if (!(ss >> m.frameTime) || m.frameTime <= 0.0f) return fail("bad frame time");

    m.numFrames = Index(frames);
    m.data.resize(size_t(m.numFrames) * m.numChannels);
    for (size_t i = 0; i < m.data.size(); ++i) {
        if (!(ss >> m.data[i])) return fail("truncated motion data");
    }
    if (err) err->clear();
    return m;
}

}  // namespace bvh

#endif  // YSIM_BVH_MOTION_HPP
