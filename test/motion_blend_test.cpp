// Motion-blend math unit tests — backend-independent, no GPU/GL.
// Shares the ysim_tests doctest main (scene_io_test.cpp owns IMPLEMENT_WITH_MAIN),
// so this TU includes doctest.h WITHOUT that macro.
//
// Covers item D of the motion-blend rework: Quatf log/exp roundtrip and the
// intrinsic (Karcher) mean blendPoseNMean() — its 2-clip case must equal slerp,
// it must be independent of clip order, and a weight collapsing onto one clip
// must return that clip's pose verbatim.

#include "doctest.h"

#include "motion_clip.hpp"
#include "motion_graph.hpp"

#include <array>
#include <cmath>
#include <vector>

using mograph::LocalPose;
using mograph::Quatf;

namespace {

// Orientation distance: 0 when a, b are the same rotation (q ≡ -q aware).
float quatAngle(const Quatf& a, const Quatf& b) {
    float d = std::fabs(a.normalized().dot(b.normalized()));
    if (d > 1.0f) d = 1.0f;
    return 2.0f * std::acos(d);  // radians between the two orientations
}

LocalPose pose1(const Quatf& q, std::array<float, 3> root = {0, 0, 0}) {
    LocalPose p;
    p.rootPos = root;
    p.rot = {q};
    return p;
}

}  // namespace

TEST_CASE("Quatf: exp(log(q)) roundtrips on the w>=0 hemisphere") {
    const Quatf qs[] = {
        Quatf::identity(),
        Quatf::axisAngle(1, 0, 0, 0.3f),
        Quatf::axisAngle(0, 1, 0, 1.1f),
        Quatf::axisAngle(0.577f, 0.577f, 0.577f, 2.0f),
        Quatf::yaw(-0.7f),
    };
    for (Quatf q : qs) {
        q = q.normalized();
        if (q.w < 0.0f) q = {-q.w, -q.x, -q.y, -q.z};  // log assumes w>=0 arc
        const Quatf r = Quatf::exp(q.log());
        CHECK(quatAngle(q, r) < 1e-5f);
    }
}

TEST_CASE("blendPoseNMean: 2-clip mean equals slerp") {
    const Quatf a = Quatf::axisAngle(0, 1, 0, 0.2f).normalized();
    const Quatf b = Quatf::axisAngle(1, 0, 0, 1.3f).normalized();
    for (float wb : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f}) {
        std::vector<LocalPose> poses = {pose1(a, {1, 2, 3}), pose1(b, {7, 0, -1})};
        std::vector<float> w = {1.0f - wb, wb};
        LocalPose out;
        mograph::blendPoseNMean(poses, w, out);
        // Rotation: matches shortest-arc slerp at fraction wb.
        CHECK(quatAngle(out.rot[0], mograph::slerp(a, b, wb)) < 1e-4f);
        // Root: weighted (here convex) mean.
        for (int k = 0; k < 3; ++k)
            CHECK(out.rootPos[k] ==
                  doctest::Approx(poses[0].rootPos[k] * (1.0f - wb) +
                                  poses[1].rootPos[k] * wb)
                      .epsilon(1e-4));
    }
}

TEST_CASE("blendPoseNMean: independent of clip order") {
    const Quatf a = Quatf::axisAngle(0, 1, 0, 0.4f).normalized();
    const Quatf b = Quatf::axisAngle(1, 0, 0, 0.9f).normalized();
    const Quatf c = Quatf::axisAngle(0, 0, 1, 1.4f).normalized();

    std::vector<LocalPose> p1 = {pose1(a), pose1(b), pose1(c)};
    std::vector<float> w1 = {0.2f, 0.3f, 0.5f};
    LocalPose o1;
    mograph::blendPoseNMean(p1, w1, o1);

    std::vector<LocalPose> p2 = {pose1(c), pose1(a), pose1(b)};
    std::vector<float> w2 = {0.5f, 0.2f, 0.3f};
    LocalPose o2;
    mograph::blendPoseNMean(p2, w2, o2);

    CHECK(quatAngle(o1.rot[0], o2.rot[0]) < 1e-4f);
}

TEST_CASE("blendPoseNMean: weight collapsing onto one clip returns it") {
    const Quatf a = Quatf::axisAngle(0, 1, 0, 0.4f).normalized();
    const Quatf b = Quatf::axisAngle(1, 0, 0, 0.9f).normalized();
    std::vector<LocalPose> poses = {pose1(a, {1, 2, 3}), pose1(b, {7, 0, -1})};
    std::vector<float> w = {1.0f, 0.0f};
    LocalPose out;
    mograph::blendPoseNMean(poses, w, out);
    CHECK(quatAngle(out.rot[0], a) < 1e-5f);
    for (int k = 0; k < 3; ++k)
        CHECK(out.rootPos[k] == doctest::Approx(poses[0].rootPos[k]).epsilon(1e-5));
}

// ── Item B: thin-plate RBF blend weights ─────────────────────────────────
namespace {
mograph::Session rbfSession(const std::vector<std::array<float, 2>>& pts) {
    mograph::Session s;
    s.clipCoords = pts;
    s.useRbf = true;
    s.buildRbf();
    return s;
}
}  // namespace

TEST_CASE("RBF: cardinal property w_i(x_j) = delta_ij at every sample") {
    // Diamond (Walk/Run/Sneak/Limp preset) + an off-axis 5th to exercise n>4.
    const std::vector<std::array<float, 2>> pts = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {0.5f, 0.5f}};
    auto s = rbfSession(pts);
    REQUIRE(s.rbfLambda.size() == pts.size() * pts.size());  // system non-singular
    std::vector<float> w;
    for (size_t j = 0; j < pts.size(); ++j) {
        s.blendWeights(pts[j], w);
        for (size_t i = 0; i < pts.size(); ++i)
            CHECK(w[i] == doctest::Approx(i == j ? 1.0f : 0.0f).epsilon(1e-3));
    }
}

TEST_CASE("RBF: weights sum to one at interior + exterior points") {
    const std::vector<std::array<float, 2>> pts = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    auto s = rbfSession(pts);
    std::vector<float> w;
    for (std::array<float, 2> q : {std::array<float, 2>{0.1f, 0.2f},
                                   std::array<float, 2>{0.6f, -0.3f},
                                   std::array<float, 2>{2.0f, 2.0f}}) {
        s.blendWeights(q, w);
        float sum = 0.0f;
        for (float x : w) sum += x;
        CHECK(sum == doctest::Approx(1.0f).epsilon(1e-4));
    }
}

TEST_CASE("RBF: useRbf=false falls back to inverse-distance (snaps to sample)") {
    const std::vector<std::array<float, 2>> pts = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    auto s = rbfSession(pts);
    s.useRbf = false;
    std::vector<float> w;
    s.blendWeights(pts[2], w);  // exactly on sample 2
    CHECK(w[2] == doctest::Approx(1.0f).epsilon(1e-2));
}

// ── Item A: rebaseClipToFrame0 is a RIGID transform (not a reflection) ────
// Regression guard for the position/orientation sign bug: position and
// orientation must both rebase by yaw(-th0). frame 0 → canonical (xz=0, yaw=0),
// and frame 1 must match yaw(-th0) applied to BOTH its delta and its rotation.
TEST_CASE("rebaseClipToFrame0: rigid (position and orientation rotate alike)") {
    const float th0 = 0.7f, th1 = 1.2f;
    mograph::Clip c;
    c.frames.resize(2);
    c.frames[0].rootPos = {5.0f, 0.3f, -2.0f};
    c.frames[0].rot = {Quatf::yaw(th0)};
    c.frames[1].rootPos = {6.5f, 0.4f, 0.5f};
    c.frames[1].rot = {Quatf::yaw(th1)};

    const float dx = c.frames[1].rootPos[0] - c.frames[0].rootPos[0];
    const float dz = c.frames[1].rootPos[2] - c.frames[0].rootPos[2];
    std::array<float, 9> Rm;
    Quatf::yaw(-th0).toMat3(Rm);                    // the house rebase rotation
    const float ex = Rm[0] * dx + Rm[2] * dz;       // yaw(-th0) · (dx,dz)
    const float ez = Rm[6] * dx + Rm[8] * dz;

    mograph::Session::rebaseClipToFrame0(c);

    // frame 0 → origin (xz), canonical heading.
    CHECK(c.frames[0].rootPos[0] == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(c.frames[0].rootPos[2] == doctest::Approx(0.0f).epsilon(1e-4));
    CHECK(quatAngle(c.frames[0].rot[0], Quatf::identity()) < 1e-4f);
    // Y is preserved.
    CHECK(c.frames[0].rootPos[1] == doctest::Approx(0.3f).epsilon(1e-5));
    // frame 1 position = yaw(-th0)·delta; orientation = yaw(th1-th0). Same angle.
    CHECK(c.frames[1].rootPos[0] == doctest::Approx(ex).epsilon(1e-4));
    CHECK(c.frames[1].rootPos[2] == doctest::Approx(ez).epsilon(1e-4));
    CHECK(quatAngle(c.frames[1].rot[0], Quatf::yaw(th1 - th0)) < 1e-4f);
}
