#ifndef YSIM_MOTION_VERB_HPP
#define YSIM_MOTION_VERB_HPP

// Two-motion keytime blend — a deliberately simple take on Rose, Cohen &
// Bodenheimer, "Verbs and Adverbs: Multidimensional Motion Interpolation"
// (IEEE CG&A 1998). It is a sibling to motion_graph.hpp's blend space, not a
// layer on it: where the blend space registers clips by DTW and travels by
// integrating velocity, this mode registers two clips by EXPLICIT foot
// keytimes and plays them in place. The pieces:
//
//   * Keytimes. Four gait events — left-foot-down, right-foot-up,
//     right-foot-down, left-foot-up — in that fixed canonical order define one
//     cycle. Each example supplies the FRAME at which each event happens, so a
//     shared phase p∈[0,1] maps (piecewise-linearly) to the same structural
//     moment in both clips: A's left-foot-down blends with B's left-foot-down.
//     Auto-detected from foot height, then user-editable (the detector only
//     seeds the fields).
//
//   * Adverbs. Named tags ("sad", …), 1 or 2 of them. Each example carries a
//     scalar per tag (a percentage). A query point in this tag space is turned
//     into per-example blend weights by a thin-plate RBF with a linear
//     polynomial term (the paper's cardinal-function scheme) — for the 1-tag /
//     2-example target case this is exactly linear interpolation, but the same
//     code extends to a 2-tag plane.
//
// The blended pose at phase p is the Karcher mean of the two keytime-warped
// poses under those weights (blendPoseNMean). Root is pinned in place
// (pinClipInPlace) — the body walks on the spot, which is what a cloth-driving
// kinematic body wants; ground travel is intentionally out of scope here.

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string>
#include <vector>

#include "motion_clip.hpp"

namespace mograph {

// Canonical gait-event order over one cycle (defines the phase axis).
enum VerbEvent { VE_LFD = 0, VE_RFU = 1, VE_RFD = 2, VE_LFU = 3 };

// Left/right foot joint indices by name, most-distal first (toe > foot >
// ankle). {-1,-1} when a side is missing → caller falls back to even keytimes.
inline std::array<int, 2> findFeet(const Skeleton& s) {
    auto lower = [](std::string t) {
        for (auto& c : t) c = char(std::tolower((unsigned char)c));
        return t;
    };
    int best[2] = {-1, -1}, rank[2] = {-1, -1};  // rank: toe=2, foot=1, ankle=0
    for (int j = 0; j < int(s.joints.size()); ++j) {
        const std::string n = lower(s.joints[j].name);
        int r = -1;
        if (n.find("toe") != std::string::npos) r = 2;
        else if (n.find("foot") != std::string::npos) r = 1;
        else if (n.find("ankle") != std::string::npos) r = 0;
        if (r < 0) continue;
        int side = -1;
        if (n.find("left") != std::string::npos) side = 0;
        else if (n.find("right") != std::string::npos) side = 1;
        if (side < 0) continue;
        if (r > rank[side]) { rank[side] = r; best[side] = j; }
    }
    return {best[0], best[1]};
}

// Detect the five cycle keytimes (LFD, RFU, RFD, LFU, cycleEnd=next-LFD) inside
// the frame window [a,b], in full-clip frame coordinates. A foot is "down" when
// its world Y sits in the lowest `band` fraction of its own range; events are
// the up→down / down→up crossings, picked greedily in canonical order. Sets
// *ok=true only when a full ordered cycle was found; otherwise returns an
// even four-way split of the window (still a usable, editable seed) with
// *ok=false. Y is invariant under the pin (yaw+xz removal), so it works the
// same on pinned or raw clips.
inline std::array<int, 5> detectKeytimes(const Skeleton& skel, const Clip& c,
                                         int a, int b, bool* ok = nullptr,
                                         float band = 0.25f) {
    if (ok) *ok = false;
    const int nf = int(c.frames.size());
    std::array<int, 5> kt{0, 0, 0, 0, nf > 0 ? nf - 1 : 0};
    if (nf < 5) return kt;
    if (a < 0) a = 0;
    if (b < 0 || b > nf - 1) b = nf - 1;
    if (b < a + 4) { a = 0; b = nf - 1; }
    auto evenFallback = [&] {
        const int span = std::max(4, b - a);
        return std::array<int, 5>{a, a + span / 4, a + span / 2,
                                  a + 3 * span / 4, b};
    };
    const auto feet = findFeet(skel);
    if (feet[0] < 0 || feet[1] < 0) return evenFallback();

    // Foot world Y per frame over [a,b].
    const int w = b - a + 1;
    std::vector<float> yL(w), yR(w);
    for (int f = a; f <= b; ++f) {
        bvh::Pose p;
        fk(skel, c.frames[f], p);
        if (int(p.world.size()) <= std::max(feet[0], feet[1]))
            return evenFallback();
        yL[f - a] = p.world[feet[0]].t[1];
        yR[f - a] = p.world[feet[1]].t[1];
    }
    auto contact = [&](const std::vector<float>& y) {
        float mn = 1e30f, mx = -1e30f;
        for (float v : y) { mn = std::min(mn, v); mx = std::max(mx, v); }
        const float th = mn + band * (mx - mn);
        std::vector<char> ct(y.size());
        for (size_t i = 0; i < y.size(); ++i) ct[i] = y[i] <= th ? 1 : 0;
        return ct;
    };
    const std::vector<char> cL = contact(yL), cR = contact(yR);
    // First up→down crossing at/after absolute frame `from` (counts a foot
    // already planted at the window start as a down there).
    auto downAt = [&](const std::vector<char>& ct, int from) {
        for (int i = std::max(from, a); i <= b; ++i) {
            const int k = i - a;
            if (ct[k] && (k == 0 || !ct[k - 1])) return i;
        }
        return -1;
    };
    auto upAt = [&](const std::vector<char>& ct, int from) {
        for (int i = std::max(from, a); i <= b; ++i) {
            const int k = i - a;
            if (!ct[k] && k > 0 && ct[k - 1]) return i;
        }
        return -1;
    };
    const int lfd = downAt(cL, a);
    if (lfd < 0) return evenFallback();
    const int rfu = upAt(cR, lfd + 1);
    const int rfd = downAt(cR, (rfu < 0 ? lfd : rfu) + 1);
    const int lfu = upAt(cL, (rfd < 0 ? lfd : rfd) + 1);
    int end = downAt(cL, (lfu < 0 ? lfd : lfu) + 1);
    if (rfu < 0 || rfd < 0 || lfu < 0) return evenFallback();
    if (end < 0) end = b;
    if (!(lfd < rfu && rfu < rfd && rfd < lfu && lfu < end))
        return evenFallback();
    if (ok) *ok = true;
    return {lfd, rfu, rfd, lfu, end};
}

// Map a normalized phase to a fractional frame through the keytime warp:
// canonical breakpoints `canon` (phase) ↔ `key` (frame), piecewise-linear.
// canon[k] is where event k lands on the phase axis; key[k] its frame. At
// canon[k] this returns exactly key[k] — the structural-alignment guarantee.
inline float verbWarpFrame(const std::array<int, 5>& key,
                           const std::array<float, 5>& canon, float phase01) {
    float p = phase01 - std::floor(phase01);
    int s = 0;
    while (s < 3 && p >= canon[s + 1]) ++s;  // segment [canon[s], canon[s+1])
    const float w0 = canon[s], w1 = canon[s + 1];
    float u = (w1 > w0 + 1e-9f) ? (p - w0) / (w1 - w0) : 0.0f;
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;
    return float(key[s]) + u * float(key[s + 1] - key[s]);
}

// World heading (yaw about +Y) of a root quaternion — matches pinClipInPlace.
inline float verbHeading(const Quatf& q) {
    std::array<float, 9> R;
    q.toMat3(R);
    return std::atan2(R[2], R[0]);
}

// Sample a clip at a fractional frame index (lerp f0→f1, clamped to [0,nf-1]).
inline void sampleClipFrameF(const Clip& c, float ff, LocalPose& out) {
    const int nf = int(c.frames.size());
    if (nf == 0) { out = LocalPose{}; return; }
    if (ff < 0.0f) ff = 0.0f;
    if (ff > float(nf - 1)) ff = float(nf - 1);
    const int f0 = int(ff);
    const int f1 = std::min(f0 + 1, nf - 1);
    blendPose(c.frames[f0], c.frames[f1], 1.0f - (ff - float(f0)), out);
}

// One example motion in the blend (exactly two are used).
struct VerbExample {
    Clip clip;                          // retargeted + pinned in place
    std::array<int, 5> key{0, 0, 0, 0, 1};  // LFD,RFU,RFD,LFU,cycleEnd (frames)
    std::vector<float> adverb;          // value per tag (percent); size == tags
    std::array<int, 2> range{0, -1};    // window the keytimes were detected in
    std::string name;
};

// In-place Gauss-Jordan solve of A·X=B (A m×m, B m×rhs, row-major; B←X).
// Partial pivoting; false if singular. Local copy so this header stays
// independent of motion_graph.hpp.
inline bool verbSolveDense(std::vector<double>& A, std::vector<double>& B,
                           int m, int rhs) {
    auto a = [&](int r, int c) -> double& { return A[size_t(r) * m + c]; };
    auto b = [&](int r, int c) -> double& { return B[size_t(r) * rhs + c]; };
    for (int col = 0; col < m; ++col) {
        int piv = col;
        double best = std::fabs(a(col, col));
        for (int r = col + 1; r < m; ++r) {
            const double v = std::fabs(a(r, col));
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-12) return false;
        if (piv != col) {
            for (int c = 0; c < m; ++c) std::swap(a(col, c), a(piv, c));
            for (int c = 0; c < rhs; ++c) std::swap(b(col, c), b(piv, c));
        }
        const double d = a(col, col);
        for (int r = 0; r < m; ++r) {
            if (r == col) continue;
            const double f = a(r, col) / d;
            if (f == 0.0) continue;
            for (int c = col; c < m; ++c) a(r, c) -= f * a(col, c);
            for (int c = 0; c < rhs; ++c) b(r, c) -= f * b(col, c);
        }
    }
    for (int col = 0; col < m; ++col) {
        const double d = a(col, col);
        for (int c = 0; c < rhs; ++c) b(col, c) /= d;
    }
    return true;
}

// Verbs & Adverbs two-motion keytime blend. Build it from two pinned clips +
// their keytimes + tag values, set `query`, then sample(t) each frame.
struct VerbBlend {
    Skeleton skel;
    float dt = 1.0f / 30.0f;
    std::vector<VerbExample> ex;      // exactly two
    std::vector<std::string> tags;    // 1..2 adverb axes
    std::vector<float> query;         // adverb query, size == tags (percent)
    bool useIntrinsicMean = true;     // Karcher mean vs incremental slerp
    // Convex weights clamp the RBF cardinals ≥0 (renormalize Σ=1) → the blend
    // stays inside the two examples (no extrapolation). OFF (default) keeps the
    // raw signed partition (Σ=1, may go <0 or >1), so over-driving the adverb
    // past an example EXTRAPOLATES along the joint geodesics — the Verbs &
    // Adverbs "150% sad" case. Needs the n==2 blendPose path below: the N-way
    // blendPoseNMean drops w≤0, which would defeat the signed weights.
    bool convexWeights = false;

    // Built from the examples (query-INDEPENDENT, so the phase clock never
    // jumps when the adverb slider moves):
    std::array<float, 5> canon{0.0f, 0.25f, 0.5f, 0.75f, 1.0f};  // phase axis
    double cycleSec = 1.0;            // gait period — the phase clock divisor
    int dim = 1;                      // == max(1, tags.size())
    std::vector<float> rbfLambda;     // n*n  radial coeffs (cardinal i, basis j)
    std::vector<float> rbfPoly;       // n*(dim+1)  poly coeffs {a_i, b_i…}

    // Root motion (neither is a treadmill — the body travels). Absolute (default)
    // blends the clips' real root trajectories, anchored per cycle so it loops
    // forward along the blended path. Relative pins + integrates the blended
    // heading-relative velocity (curving travel; robust when the two clips
    // face/curve differently). Built per clip in rebuild() from the UNPINNED
    // clips + keytimes.
    enum class Root { Absolute = 0, Relative = 1 };
    Root rootMode = Root::Absolute;
    std::vector<std::array<float, 2>> exRoot0;  // [clip] XZ at key[0] (cycle start)
    std::vector<std::array<float, 2>> exDelta;  // [clip] XZ key[0]→key[4] (1 cycle)
    std::vector<std::vector<std::array<float, 3>>> exVel;  // [clip][f] {lvx,lvz,ω}
    // Relative-mode integrator (world). Reset on seek / mode switch, NOT on a
    // weight/keytime edit (that must not teleport a body mid-travel).
    double travYaw = 0.0, travX = 0.0, travZ = 0.0, travLastT = -1.0;
    void resetTravel() { travYaw = 0.0; travX = 0.0; travZ = 0.0; travLastT = -1.0; }

    bool ready() const {
        return ex.size() >= 2 && !skel.joints.empty() &&
               !ex[0].clip.frames.empty() && !ex[1].clip.frames.empty();
    }

    // Thin-plate kernel φ(r²)=½·r²·ln(r²); φ(0)=0.
    static double phi(double r2) {
        return r2 < 1e-12 ? 0.0 : 0.5 * r2 * std::log(r2);
    }
    double coord(int i, int k) const {
        return double(i < int(ex.size()) && k < int(ex[i].adverb.size())
                          ? ex[i].adverb[k]
                          : 0.0f);
    }

    // Recompute the canonical breakpoints, cycle period, and RBF cardinals from
    // the current examples/tags. Cheap (≤4×4 solve) — call after any keytime,
    // tag, or adverb edit.
    void rebuild() {
        dim = std::max(1, int(tags.size()));
        // Canonical phase spacing = mean of the examples' normalized per-segment
        // frame durations, so the blended cycle's internal tempo is their
        // average (and is query-independent).
        std::array<double, 4> seg{0, 0, 0, 0};
        int cnt = 0;
        for (const auto& e : ex) {
            double tot = 0;
            std::array<double, 4> d{};
            for (int s = 0; s < 4; ++s) {
                d[s] = std::max(1, e.key[s + 1] - e.key[s]);
                tot += d[s];
            }
            if (tot <= 0) continue;
            for (int s = 0; s < 4; ++s) seg[s] += d[s] / tot;
            ++cnt;
        }
        canon[0] = 0.0f;
        if (cnt > 0) {
            double acc = 0;
            for (int s = 0; s < 4; ++s) { acc += seg[s] / cnt; canon[s + 1] = float(acc); }
        } else {
            canon = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
        }
        canon[4] = 1.0f;
        // Cycle clock = mean cycle duration of the examples.
        double cs = 0;
        int n = 0;
        for (const auto& e : ex) {
            cs += double(e.key[4] - e.key[0]) * double(e.clip.dt);
            ++n;
        }
        cycleSec = (n > 0 && cs > 1e-6) ? cs / n : 1.0;
        buildRbf();
        buildRootData();
    }

    // Per-clip root trajectory data from the (unpinned) clips: cycle-start XZ,
    // one-cycle XZ travel, and per-frame heading-relative velocity {lvx,lvz,ω}.
    void buildRootData() {
        exRoot0.assign(ex.size(), {0.0f, 0.0f});
        exDelta.assign(ex.size(), {0.0f, 0.0f});
        exVel.assign(ex.size(), {});
        for (size_t i = 0; i < ex.size(); ++i) {
            const auto& fr = ex[i].clip.frames;
            const int nf = int(fr.size());
            if (nf == 0) continue;
            auto cl = [&](int f) { return f < 0 ? 0 : (f > nf - 1 ? nf - 1 : f); };
            const int k0 = cl(ex[i].key[0]), k4 = cl(ex[i].key[4]);
            exRoot0[i] = {fr[k0].rootPos[0], fr[k0].rootPos[2]};
            exDelta[i] = {fr[k4].rootPos[0] - exRoot0[i][0],
                          fr[k4].rootPos[2] - exRoot0[i][1]};
            const float dt = ex[i].clip.dt > 1e-6f ? ex[i].clip.dt : 1.0f / 30.0f;
            std::vector<std::array<float, 3>> vel(size_t(nf), {0, 0, 0});
            for (int f = 0; f < nf; ++f) {
                const int g = f + 1 < nf ? f + 1 : f;
                const float wvx = (fr[g].rootPos[0] - fr[f].rootPos[0]) / dt;
                const float wvz = (fr[g].rootPos[2] - fr[f].rootPos[2]) / dt;
                const float h0 = fr[f].rot.empty() ? 0.0f : verbHeading(fr[f].rot[0]);
                const float h1 = fr[g].rot.empty() ? 0.0f : verbHeading(fr[g].rot[0]);
                float dh = h1 - h0;
                const float kPi = 3.14159265358979323846f;
                while (dh > kPi) dh -= 2.0f * kPi;
                while (dh < -kPi) dh += 2.0f * kPi;
                const float c = std::cos(-h0), s = std::sin(-h0);  // world→local
                vel[size_t(f)] = {c * wvx - s * wvz, s * wvx + c * wvz, dh / dt};
            }
            exVel[i] = std::move(vel);
        }
    }

    // Blended heading-relative root velocity {lvx, lvz, ω} at a normalized phase.
    std::array<double, 3> blendVel(float phase, const std::vector<float>& w) const {
        double vx = 0, vz = 0, om = 0;
        for (size_t i = 0; i < ex.size() && i < exVel.size() && i < w.size(); ++i) {
            const auto& vel = exVel[i];
            const int nf = int(vel.size());
            if (nf == 0) continue;
            const float ff = verbWarpFrame(ex[i].key, canon, phase);
            int f0 = int(ff);
            if (f0 < 0) f0 = 0;
            if (f0 > nf - 1) f0 = nf - 1;
            const int f1 = f0 + 1 < nf ? f0 + 1 : f0;
            const float a = ff - std::floor(ff);
            vx += double(w[i]) * (vel[f0][0] * (1 - a) + vel[f1][0] * a);
            vz += double(w[i]) * (vel[f0][1] * (1 - a) + vel[f1][1] * a);
            om += double(w[i]) * (vel[f0][2] * (1 - a) + vel[f1][2] * a);
        }
        return {vx, vz, om};
    }

    // Solve the thin-plate cardinal coefficients over adverb space once.
    // System [[Φ P],[Pᵀ 0]]·[Λ;C]=[I;0], P row i = [1, x_i…]. Leaves the
    // buffers empty on n<2 or a singular system (collinear samples in ≥2D,
    // e.g. two tags but two motions) → weights() falls back to inverse-distance.
    void buildRbf() {
        rbfLambda.clear();
        rbfPoly.clear();
        const int n = int(ex.size());
        const int d = dim;
        if (n < 2) return;
        const int P = d + 1;
        const int m = n + P;
        std::vector<double> M(size_t(m) * m, 0.0), B(size_t(m) * n, 0.0);
        auto A = [&](int r, int c) -> double& { return M[size_t(r) * m + c]; };
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                double r2 = 0;
                for (int k = 0; k < d; ++k) {
                    const double dd = coord(i, k) - coord(j, k);
                    r2 += dd * dd;
                }
                A(i, j) = phi(r2);
            }
            A(i, n) = 1.0;
            for (int k = 0; k < d; ++k) A(i, n + 1 + k) = coord(i, k);
            A(n, i) = 1.0;
            for (int k = 0; k < d; ++k) A(n + 1 + k, i) = coord(i, k);
        }
        for (int i = 0; i < n; ++i) B[size_t(i) * n + i] = 1.0;  // RHS = [I;0]
        if (!verbSolveDense(M, B, m, n)) return;
        rbfLambda.assign(size_t(n) * n, 0.0f);
        rbfPoly.assign(size_t(n) * P, 0.0f);
        for (int i = 0; i < n; ++i) {  // column i of X = cardinal i's coeffs
            for (int j = 0; j < n; ++j)
                rbfLambda[size_t(i) * n + j] = float(B[size_t(j) * n + i]);
            for (int k = 0; k < P; ++k)
                rbfPoly[size_t(i) * P + k] = float(B[size_t(n + k) * n + i]);
        }
    }

    // Per-example blend weights for adverb query `q` (Σ=1). RBF cardinals when
    // built (w_i(adverb_j)=δ_ij), else inverse-distance in adverb space.
    void weights(const std::vector<float>& q, std::vector<float>& w) const {
        const int n = int(ex.size());
        const int d = dim;
        const int P = d + 1;
        w.assign(size_t(n), 0.0f);
        if (n == 0) return;
        if (n == 1) { w[0] = 1.0f; return; }
        auto qk = [&](int k) { return double(k < int(q.size()) ? q[k] : 0.0f); };
        if (int(rbfLambda.size()) == n * n && int(rbfPoly.size()) == n * P) {
            float sum = 0.0f;
            for (int i = 0; i < n; ++i) {
                double acc = double(rbfPoly[size_t(i) * P + 0]);
                for (int k = 0; k < d; ++k)
                    acc += double(rbfPoly[size_t(i) * P + 1 + k]) * qk(k);
                for (int j = 0; j < n; ++j) {
                    double r2 = 0;
                    for (int k = 0; k < d; ++k) {
                        const double dd = qk(k) - coord(j, k);
                        r2 += dd * dd;
                    }
                    acc += double(rbfLambda[size_t(i) * n + j]) * phi(r2);
                }
                w[i] = float(acc);
                sum += w[i];
            }
            if (convexWeights) {
                float s2 = 0.0f;
                for (auto& x : w) { if (x < 0.0f) x = 0.0f; s2 += x; }
                if (s2 > 1e-6f) for (auto& x : w) x /= s2;
                else for (auto& x : w) x = 1.0f / float(n);
            } else if (std::fabs(sum) > 1e-6f) {
                for (auto& x : w) x /= sum;
            }
            return;
        }
        float sum = 0.0f;
        for (int i = 0; i < n; ++i) {
            double r2 = 0;
            for (int k = 0; k < d; ++k) {
                const double dd = qk(k) - coord(i, k);
                r2 += dd * dd;
            }
            const float wi = 1.0f / float(r2 + 1e-4);
            w[i] = wi;
            sum += wi;
        }
        if (sum > 1e-12f) for (auto& x : w) x /= sum;
        else for (auto& x : w) x = 1.0f / float(n);
    }

    // Blended LOCAL pose at a normalized phase (no fk) — each example keytime-
    // warped to the common phase, then Karcher-mean-mixed by the adverb weights.
    void sampleMixed(float phase01, LocalPose& mixed) const {
        std::vector<float> w;
        weights(query, w);
        std::vector<LocalPose> poses(ex.size());
        for (size_t i = 0; i < ex.size(); ++i) {
            const float ff = verbWarpFrame(ex[i].key, canon, phase01);
            sampleClipFrameF(ex[i].clip, ff, poses[i]);
        }
        // Exactly two motions (the target case): a direct signed two-pose blend
        // so a weight outside [0,1] EXTRAPOLATES — slerp extends along the joint
        // geodesic and the root lerp extends linearly. blendPoseNMean would drop
        // the negative weight (w≤0 skipped) and collapse onto the near endpoint,
        // so it cannot extrapolate. For n==2 blendPose is identical to the mean
        // inside [0,1] and the correct extension outside it.
        if (poses.size() == 2 && w.size() == 2) {
            blendPose(poses[0], poses[1], w[0], mixed);  // wb = 1-w0 = w1
            return;
        }
        if (useIntrinsicMean) blendPoseNMean(poses, w, mixed);
        else blendPoseN(poses, w, mixed);
    }

    // World pose at time `tSec` (the per-frame entry). Phase = fmod over the
    // fixed cycle clock; the root TRAVELS (absolute per-cycle accumulate, or
    // relative velocity integration) — no treadmill. Non-const: relative mode
    // advances the integrator. `tSec` is the open-ended play clock (NOT wrapped
    // by the caller for this mode), so the cycle counter / dt are monotone.
    bool sample(double tSec, bvh::Pose& out) {
        if (!ready()) return false;
        const double cyc = cycleSec > 1e-6 ? cycleSec : 1.0;
        const double tt = std::max(0.0, tSec);
        const float phase = float(std::fmod(tt, cyc) / cyc);
        std::vector<float> w;
        weights(query, w);
        std::vector<LocalPose> poses(ex.size());
        for (size_t i = 0; i < ex.size(); ++i) {
            const float ff = verbWarpFrame(ex[i].key, canon, phase);
            sampleClipFrameF(ex[i].clip, ff, poses[i]);
        }
        LocalPose mixed;
        if (poses.size() == 2 && w.size() == 2)
            blendPose(poses[0], poses[1], w[0], mixed);
        else if (useIntrinsicMean)
            blendPoseNMean(poses, w, mixed);
        else
            blendPoseN(poses, w, mixed);

        if (rootMode == Root::Relative) {
            double dt = travLastT < 0.0 ? 0.0 : (tSec - travLastT);
            if (dt < 0.0 || dt > 0.5) dt = 0.0;  // seek / first frame ⇒ no step
            travLastT = tSec;
            const auto v = blendVel(phase, w);
            travYaw += v[2] * dt;
            const double cy = std::cos(travYaw), sy = std::sin(travYaw);
            travX += (cy * v[0] - sy * v[1]) * dt;  // local → world
            travZ += (sy * v[0] + cy * v[1]) * dt;
            if (!mixed.rot.empty()) {  // strip the blended heading, face travYaw
                const float h0 = verbHeading(mixed.rot[0]);
                mixed.rot[0] =
                    (Quatf::yaw(float(travYaw) - h0) * mixed.rot[0]).normalized();
            }
            mixed.rootPos[0] = float(travX);
            mixed.rootPos[2] = float(travZ);  // keep blended Y (bob)
        } else {
            // Absolute: per-cycle accumulate XZ so the loop travels forward with
            // no seam; Y + heading stay as blended above.
            const double cycleIdx = std::floor(tt / cyc);
            double dx = 0, dz = 0, gx = 0, gz = 0;
            for (size_t i = 0; i < ex.size() && i < exRoot0.size() &&
                               i < w.size(); ++i) {
                dx += double(w[i]) * (poses[i].rootPos[0] - exRoot0[i][0]);
                dz += double(w[i]) * (poses[i].rootPos[2] - exRoot0[i][1]);
                gx += double(w[i]) * exDelta[i][0];
                gz += double(w[i]) * exDelta[i][1];
            }
            mixed.rootPos[0] = float(cycleIdx * gx + dx);
            mixed.rootPos[2] = float(cycleIdx * gz + dz);
        }
        fk(skel, mixed, out);
        return !out.world.empty();
    }
};

}  // namespace mograph

#endif  // YSIM_MOTION_VERB_HPP
