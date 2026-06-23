#ifndef YSIM_MOTION_GRAPH_HPP
#define YSIM_MOTION_GRAPH_HPP

// CPU-pure motion-graph construction and playback over motion_clip.hpp
// tracks, after Kovar, Gleicher & Pighin, "Motion Graphs" (SIGGRAPH 2002).
// Header-only, no Metal / GLFW / Eigen (std::thread only, for graph builds).
//
// Pipeline:
//   1. Similarity — D(i, j) compares the k-frame windows A[i..i+k-1] and
//      B[j-k+1..j] as joint point clouds under the *optimal* ground-plane
//      rigid alignment (yaw + xz translation; closed form, paper §3.2). Each
//      joint contributes its origin plus three markers offset along its local
//      axes (paper §4.1), so full joint orientation — not just position —
//      drives the metric.
//      Cost is normalized to sqrt(SSD / Σw) / restHeight, i.e. "average
//      joint deviation as a fraction of body height" — so one threshold
//      works across files whose raw units differ ~7x.
//   2. Transition candidates — local minima of D below the threshold.
//   3. Transitions — k blended frames with the paper's ease curve
//      2u³-3u²+1; root positions lerp, joint rotations slerp, B aligned.
//   4. Graph — nodes are cut points, edges are clip segments or baked
//      transitions; pruned to the largest SCC so a walk never dead-ends.
//   5. Playback — WalkBaker streams a random walk into a LocalPose track
//      (cumulative XformXZ keeps the body continuous through transitions);
//      two-clip transitions bake one finite, scrubbable track.
//
// Session::buildBlend adds a DTW timewarp blend (Kovar & Gleicher, "Flexible
// Automatic Motion Blending with Registration Curves", 2003): a slope-limited
// dynamic-time-warp over D gives a frame correspondence, and the crossfade
// follows it so B's timeline is time-scaled to match A's phase.
//
// The brute-force evaluators (costDirect / optimalAlign) double as the
// self-test reference for the streaming cost matrix.
//
// Performance: D over a clip pair is O(Na·Nb·J) via per-diagonal sliding
// windows (window sums reuse k-1 of k frame pairs along a diagonal), not
// the naive O(Na·Nb·J·k). Graph builds parallelize across clip pairs.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "motion_clip.hpp"

namespace mograph {

// ---- frame point clouds ------------------------------------------------------

// Per-frame world point cloud of a clip plus prefix sums of the single-frame
// scalar terms the metric needs (double accumulators: prefix sums over ~1000
// frames of squared coordinates overflow float precision).
//
// Each joint contributes either its world origin alone (markerScale <= 0) or,
// following Kovar §4.1 ("each frame is converted to a point cloud by attaching
// markers to the joints"), the origin plus three markers rigidly offset along
// the joint's local axes (t + R·(s,0,0), R·(0,s,0), R·(0,0,s)). The markers
// make a joint's full orientation count — without them a distal/leaf joint's
// rotation is invisible to the metric (it moves no descendant). `numJoints` is
// the resulting points-per-frame (J or 4·J); every downstream consumer treats
// it as an opaque point count, so the closed-form alignment is unchanged.
struct FrameCloud {
    int numFrames = 0, numJoints = 0;  // numJoints == points per frame
    std::vector<float> pos;            // numFrames * numJoints * 3
    // Prefix arrays, size numFrames+1: per-frame sums over points.
    std::vector<double> px, pz, pxz2, py2;

    void build(const Skeleton& skel, const Clip& clip, float markerScale = 0.0f) {
        numFrames = int(clip.frames.size());
        const int J = int(skel.joints.size());
        const int perJoint = markerScale > 0.0f ? 4 : 1;  // origin + 3 axis markers
        numJoints = J * perJoint;
        pos.assign(size_t(numFrames) * numJoints * 3, 0.0f);
        px.assign(numFrames + 1, 0.0);
        pz.assign(numFrames + 1, 0.0);
        pxz2.assign(numFrames + 1, 0.0);
        py2.assign(numFrames + 1, 0.0);
        bvh::Pose pose;
        for (int f = 0; f < numFrames; ++f) {
            fk(skel, clip.frames[f], pose);
            double sx = 0, sz = 0, sxz2 = 0, sy2 = 0;
            float* dst = pos.data() + size_t(f) * numJoints * 3;
            int w = 0;  // point cursor within the frame
            auto emit = [&](const std::array<float, 3>& p) {
                dst[w * 3 + 0] = p[0];
                dst[w * 3 + 1] = p[1];
                dst[w * 3 + 2] = p[2];
                ++w;
                sx += p[0];
                sz += p[2];
                sxz2 += double(p[0]) * p[0] + double(p[2]) * p[2];
                sy2 += double(p[1]) * p[1];
            };
            for (int m = 0; m < J; ++m) {
                const bvh::JointXform& jx = pose.world[m];
                emit(jx.t);
                if (perJoint == 4) {
                    for (int ax = 0; ax < 3; ++ax) {
                        std::array<float, 3> local{0.0f, 0.0f, 0.0f};
                        local[ax] = markerScale;
                        const auto d = bvh::detail::mulV(jx.R, local);
                        emit({jx.t[0] + d[0], jx.t[1] + d[1], jx.t[2] + d[2]});
                    }
                }
            }
            px[f + 1] = px[f] + sx;
            pz[f + 1] = pz[f] + sz;
            pxz2[f + 1] = pxz2[f] + sxz2;
            py2[f + 1] = py2[f] + sy2;
        }
    }

    const float* frame(int f) const { return pos.data() + size_t(f) * numJoints * 3; }
};

// ---- windowed point-cloud distance -------------------------------------------

// D(i, jj) where i = first A-frame of the window and jj + k - 1 = j = last
// B-frame of the window. Valid ranges: i in [0, Na-k], j in [k-1, Nb-1].
struct CostMatrix {
    int k = 0;
    int ra = 0, rb = 0;  // rows (i count), cols (jj count)
    std::vector<float> d;

    float at(int i, int jj) const { return d[size_t(i) * rb + jj]; }
    bool empty() const { return ra <= 0 || rb <= 0; }
};

namespace detail {

// Optimal yaw+xz alignment of centered cross sums (paper eq. 2-3, derived
// for our Y-rotation convention x' = c·x + s·z, z' = -s·x + c·z):
// maximize c·A + s·B  →  theta = atan2(B, A).
struct CellSums {
    double sxA = 0, szA = 0, sxz2A = 0, sy2A = 0;
    double sxB = 0, szB = 0, sxz2B = 0, sy2B = 0;
    double paa = 0;  // Σ (xa·xb + za·zb)
    double pb = 0;   // Σ (xa·zb - za·xb)
    double py = 0;   // Σ ya·yb
};

inline void solveCell(const CellSums& s, double W, double height,
                      XformXZ* xfOut, float* costOut) {
    const double Ac = s.paa - (s.sxA * s.sxB + s.szA * s.szB) / W;
    const double Bc = s.pb - (s.sxA * s.szB - s.szA * s.sxB) / W;
    const double theta = std::atan2(Bc, Ac);
    if (xfOut) {
        const double c = std::cos(theta), sn = std::sin(theta);
        xfOut->theta = float(theta);
        xfOut->tx = float((s.sxA - c * s.sxB - sn * s.szB) / W);
        xfOut->tz = float((s.szA + sn * s.sxB - c * s.szB) / W);
    }
    if (costOut) {
        const double exz = (s.sxz2A - (s.sxA * s.sxA + s.szA * s.szA) / W) +
                           (s.sxz2B - (s.sxB * s.sxB + s.szB * s.szB) / W) -
                           2.0 * std::sqrt(Ac * Ac + Bc * Bc);
        const double ey = s.sy2A + s.sy2B - 2.0 * s.py;
        const double e = std::max(exz + ey, 0.0);
        *costOut = float(std::sqrt(e / W) / height);
    }
}

}  // namespace detail

// Streaming cost matrix: per-diagonal sliding window over per-frame-pair
// cross terms; single-clip terms come from FrameCloud prefix sums.
inline void computeCostMatrix(const FrameCloud& A, const FrameCloud& B, int k,
                              float height, CostMatrix& out) {
    out.k = k;
    out.ra = A.numFrames - k + 1;
    out.rb = B.numFrames - k + 1;
    out.d.clear();
    if (out.empty() || A.numJoints != B.numJoints || height <= 0.0f) {
        out.ra = out.rb = 0;
        return;
    }
    const int J = A.numJoints;
    const double W = double(k) * J;
    out.d.assign(size_t(out.ra) * out.rb, std::numeric_limits<float>::max());

    std::vector<double> ringA(k), ringB(k), ringY(k);
    for (int dOff = -(A.numFrames - 1); dOff <= B.numFrames - 1; ++dOff) {
        const int a0 = std::max(0, -dOff);
        const int aEnd = std::min(A.numFrames, B.numFrames - dOff);
        double sumA = 0, sumB = 0, sumY = 0;
        int filled = 0;
        for (int a = a0; a < aEnd; ++a) {
            const int b = a + dOff;
            const float* pa = A.frame(a);
            const float* pb = B.frame(b);
            double paa = 0, pcr = 0, py = 0;
            for (int m = 0; m < J; ++m) {
                const double xa = pa[m * 3 + 0], ya = pa[m * 3 + 1], za = pa[m * 3 + 2];
                const double xb = pb[m * 3 + 0], yb = pb[m * 3 + 1], zb = pb[m * 3 + 2];
                paa += xa * xb + za * zb;
                pcr += xa * zb - za * xb;
                py += ya * yb;
            }
            const int slot = (a - a0) % k;
            if (filled == k) {
                sumA -= ringA[slot];
                sumB -= ringB[slot];
                sumY -= ringY[slot];
            } else {
                ++filled;
            }
            ringA[slot] = paa;
            ringB[slot] = pcr;
            ringY[slot] = py;
            sumA += paa;
            sumB += pcr;
            sumY += py;
            if (filled < k) continue;

            const int i = a - k + 1;       // A window start
            const int jj = b - k + 1;      // = j - (k-1)
            detail::CellSums s;
            s.sxA = A.px[i + k] - A.px[i];
            s.szA = A.pz[i + k] - A.pz[i];
            s.sxz2A = A.pxz2[i + k] - A.pxz2[i];
            s.sy2A = A.py2[i + k] - A.py2[i];
            s.sxB = B.px[jj + k] - B.px[jj];
            s.szB = B.pz[jj + k] - B.pz[jj];
            s.sxz2B = B.pxz2[jj + k] - B.pxz2[jj];
            s.sy2B = B.py2[jj + k] - B.py2[jj];
            s.paa = sumA;
            s.pb = sumB;
            s.py = sumY;
            float cost;
            detail::solveCell(s, W, height, nullptr, &cost);
            out.d[size_t(i) * out.rb + jj] = cost;
        }
    }
}

// Brute single-cell evaluation: optimal alignment + cost for window
// (A[i..i+k-1], B[j-k+1..j]). Independent of the streaming path — used to
// fetch alignments for chosen transitions and as the self-test reference.
struct AlignResult {
    XformXZ xf;
    float cost = 0.0f;
};

inline AlignResult optimalAlign(const FrameCloud& A, int i, const FrameCloud& B,
                                int j, int k, float height) {
    const int J = A.numJoints;
    detail::CellSums s;
    for (int p = 0; p < k; ++p) {
        const float* pa = A.frame(i + p);
        const float* pb = B.frame(j - k + 1 + p);
        for (int m = 0; m < J; ++m) {
            const double xa = pa[m * 3 + 0], ya = pa[m * 3 + 1], za = pa[m * 3 + 2];
            const double xb = pb[m * 3 + 0], yb = pb[m * 3 + 1], zb = pb[m * 3 + 2];
            s.sxA += xa; s.szA += za;
            s.sxz2A += xa * xa + za * za;
            s.sy2A += ya * ya;
            s.sxB += xb; s.szB += zb;
            s.sxz2B += xb * xb + zb * zb;
            s.sy2B += yb * yb;
            s.paa += xa * xb + za * zb;
            s.pb += xa * zb - za * xb;
            s.py += ya * yb;
        }
    }
    AlignResult r;
    detail::solveCell(s, double(k) * J, height, &r.xf, &r.cost);
    return r;
}

// Literal point-cloud SSD under an explicit transform (test oracle).
inline float costDirect(const FrameCloud& A, int i, const FrameCloud& B, int j,
                        int k, float height, const XformXZ& xf) {
    const int J = A.numJoints;
    double e = 0;
    for (int p = 0; p < k; ++p) {
        const float* pa = A.frame(i + p);
        const float* pb = B.frame(j - k + 1 + p);
        for (int m = 0; m < J; ++m) {
            std::array<float, 3> q{pb[m * 3 + 0], pb[m * 3 + 1], pb[m * 3 + 2]};
            xf.applyPoint(q);
            const double dx = pa[m * 3 + 0] - q[0];
            const double dy = pa[m * 3 + 1] - q[1];
            const double dz = pa[m * 3 + 2] - q[2];
            e += dx * dx + dy * dy + dz * dz;
        }
    }
    return float(std::sqrt(e / (double(k) * J)) / height);
}

// ---- transition candidates ----------------------------------------------------

struct TransitionCand {
    int i = 0, j = 0;  // A window start frame, B window end frame
    float cost = 0.0f;
};

// Local minima of D below `threshold` (8-neighborhood; plateau ties keep the
// first cell in scan order).
inline void findLocalMinima(const CostMatrix& D, float threshold,
                            std::vector<TransitionCand>& out) {
    out.clear();
    for (int i = 0; i < D.ra; ++i) {
        for (int jj = 0; jj < D.rb; ++jj) {
            const float c = D.at(i, jj);
            if (c >= threshold) continue;
            bool minimal = true;
            for (int di = -1; di <= 1 && minimal; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    if (di == 0 && dj == 0) continue;
                    const int ni = i + di, nj = jj + dj;
                    if (ni < 0 || nj < 0 || ni >= D.ra || nj >= D.rb) continue;
                    const float n = D.at(ni, nj);
                    const bool before = di < 0 || (di == 0 && dj < 0);
                    if (before ? n <= c : n < c) { minimal = false; break; }
                }
            }
            if (minimal) out.push_back({i, jj + D.k - 1, c});
        }
    }
}

// k blended frames from A@i to B@j (B pre-aligned by `align`), ease weights
// w_A(p) = 2u³ - 3u² + 1, u = (p+1)/k: starts ≈A[i], ends exactly at
// aligned B[j], so frame f=i-1 before and j+1 after are both continuous.
inline void bakeTransitionFrames(const Clip& A, int i, const Clip& B, int j,
                                 int k, const XformXZ& align,
                                 std::vector<LocalPose>& out) {
    out.resize(k);
    LocalPose tb;
    for (int p = 0; p < k; ++p) {
        const float u = float(p + 1) / float(k);
        const float wa = 2.0f * u * u * u - 3.0f * u * u + 1.0f;
        tb = B.frames[j - k + 1 + p];
        align.applyPose(tb);
        blendPose(A.frames[i + p], tb, wa, out[p]);
    }
}

// ---- dynamic time warping (Kovar & Gleicher, "Registration Curves", 2003) ----

// A monotonic frame correspondence between clip A and clip B, expressed over
// the windowed cost matrix: each cell is a (Awindow, Bwindow) start-index pair.
// Consecutive cells differ by (1,0), (0,1) or (1,1), so the path doubles as a
// strictly non-decreasing timewarp S(u) — runs of (1,0)/(0,1) are exactly the
// places one clip's timeline is stretched against the other's.
struct WarpPath {
    std::vector<std::pair<int, int>> cells;  // (i, jj) along the warp
    float cost = std::numeric_limits<float>::max();  // average cell cost
};

// Slope-limited DTW over the cost matrix D (RegCurves §4.2.1). Free boundaries
// (Kovar's min-average-cost rule when the clips are not globally time-aligned):
// the path may start on the bottom/left border and end on the top/right border.
// Step set advances both axes at least once per move and bounds the warp slope
// to [1/L, L] via steps {(1,1),(1,2..L),(2..L,1)} — the continuity, causality
// and slope-limit properties of §4.2.1. Returns false if no path spans the grid.
inline bool dtwPath(const CostMatrix& D, int slopeLimit, WarpPath& out) {
    out.cells.clear();
    out.cost = std::numeric_limits<float>::max();
    const int ra = D.ra, rb = D.rb;
    if (ra < 2 || rb < 2) return false;
    const int L = std::max(2, slopeLimit);
    const float INF = std::numeric_limits<float>::max();
    const size_t N = size_t(ra) * rb;
    std::vector<float> acc(N, INF);   // min total cell cost reaching this cell
    std::vector<int> cnt(N, 0);       // cells on that best path (for averaging)
    std::vector<int> pred(N, -1);     // packed predecessor, -1 = border start
    auto id = [&](int i, int j) { return size_t(i) * rb + j; };

    for (int j = 0; j < rb; ++j) { acc[id(0, j)] = D.at(0, j); cnt[id(0, j)] = 1; }
    for (int i = 0; i < ra; ++i) { acc[id(i, 0)] = D.at(i, 0); cnt[id(i, 0)] = 1; }

    // Row-major sweep: every predecessor has strictly smaller i and j, so a
    // cell is final before it relaxes its successors.
    for (int i = 0; i < ra; ++i) {
        for (int j = 0; j < rb; ++j) {
            const float a = acc[id(i, j)];
            if (a == INF) continue;
            const int c0 = cnt[id(i, j)];
            auto relax = [&](int di, int dj) {
                const int ni = i + di, nj = j + dj;
                if (ni >= ra || nj >= rb) return;
                // Sum the cells the L-shaped step passes through (Kovar sums
                // path cells); the path stays dense for blending.
                float add = 0.0f;
                int addc = 0;
                if (di == 1)
                    for (int c = 1; c <= dj; ++c) { add += D.at(i + 1, j + c); ++addc; }
                else
                    for (int r = 1; r <= di; ++r) { add += D.at(i + r, j + 1); ++addc; }
                const float na = a + add;
                if (na < acc[id(ni, nj)]) {
                    acc[id(ni, nj)] = na;
                    cnt[id(ni, nj)] = c0 + addc;
                    pred[id(ni, nj)] = int(id(i, j));
                }
            };
            relax(1, 1);
            for (int dj = 2; dj <= L; ++dj) relax(1, dj);
            for (int di = 2; di <= L; ++di) relax(di, 1);
        }
    }

    // Free end: lowest average-cost cell on the far border, long enough that it
    // is a real warp and not a one-cell corner artifact.
    const int minLen = std::max(2, std::min(ra, rb) / 2);
    size_t best = N;
    float bestAvg = INF;
    auto consider = [&](int i, int j) {
        const size_t e = id(i, j);
        if (acc[e] == INF || cnt[e] < minLen) return;
        const float avg = acc[e] / float(cnt[e]);
        if (avg < bestAvg) { bestAvg = avg; best = e; }
    };
    for (int j = 0; j < rb; ++j) consider(ra - 1, j);
    for (int i = 0; i < ra; ++i) consider(i, rb - 1);
    if (best == N) return false;

    // Backtrack, densifying each L-shaped step into unit cells.
    std::vector<std::pair<int, int>> rev;
    size_t e = best;
    for (;;) {
        const int ci = int(e / rb), cj = int(e % rb);
        rev.push_back({ci, cj});
        const int p = pred[e];
        if (p < 0) break;
        const int pi = p / rb, pj = p % rb;
        const int di = ci - pi, dj = cj - pj;
        if (di == 1)
            for (int c = dj - 1; c >= 1; --c) rev.push_back({ci, pj + c});
        else
            for (int r = di - 1; r >= 1; --r) rev.push_back({pi + r, cj});
        e = size_t(p);
    }
    out.cells.assign(rev.rbegin(), rev.rend());
    out.cost = bestAvg;
    return out.cells.size() >= 2;
}

// ---- the graph -----------------------------------------------------------------

struct GraphParams {
    int windowFrames = 10;          // ≈1/3 s at 30 fps (paper)
    float thresholdFrac = 0.10f;    // avg point deviation / body height
    float markerScaleFrac = 0.10f;  // joint-axis marker length / body height (0 → origins only)
    int maxPerPair = 40;            // lowest-cost cap per ordered clip pair
    int numThreads = 0;             // 0 → hardware_concurrency
};

struct MotionGraph {
    struct Transition {
        int clipA = 0, clipB = 0;
        int i = 0, j = 0;
        float cost = 0.0f;
        XformXZ align;
        std::vector<LocalPose> frames;  // k baked blend frames
    };
    // Node (clip, frame) = "about to play clips[clip].frames[frame]".
    // Segment edge plays frames [f0, f1); transition edge plays its baked
    // frames and the walker then composes `align` into its cumulative
    // transform (landing node is (clipB, j+1)).
    struct Node {
        int clip = 0, frame = 0;
        std::vector<int> out;
    };
    struct Edge {
        int from = -1, to = -1;
        int clip = -1, f0 = 0, f1 = 0;  // segment payload
        int trans = -1;                 // >=0 → transition payload
    };

    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Transition> transitions;

    struct Stats {
        int rawCandidates = 0;     // local minima below threshold, pre-cap
        int transitionsKept = 0;   // post-cap, post-SCC
        int nodesTotal = 0, nodesScc = 0;
        int edgesKept = 0;
        double buildMs = 0.0;
        std::vector<int> clipsDroppedBySCC;  // clip indices with no surviving node
        float minCostSeen = std::numeric_limits<float>::max();  // over all pairs
    } stats;

    bool build(const std::vector<Clip>& clips, const Skeleton& skel,
               const GraphParams& gp, std::string* err) {
        nodes.clear();
        edges.clear();
        transitions.clear();
        stats = Stats{};
        const auto t0 = std::chrono::steady_clock::now();
        const int nc = int(clips.size());
        if (nc == 0) {
            if (err) *err = "no clips";
            return false;
        }
        const int k = gp.windowFrames;
        for (const Clip& c : clips) {
            if (int(c.frames.size()) < k + 1) {
                if (err) *err = "clip '" + c.name + "' shorter than the blend window";
                return false;
            }
        }

        // 1. Point clouds (shared across pairs).
        const float markerScale = gp.markerScaleFrac * skel.height;
        std::vector<FrameCloud> clouds(nc);
        for (int c = 0; c < nc; ++c) clouds[c].build(skel, clips[c], markerScale);

        // 2. Transition candidates per ordered pair, in parallel.
        struct Pair { int a, b; };
        std::vector<Pair> pairs;
        for (int a = 0; a < nc; ++a)
            for (int b = 0; b < nc; ++b) pairs.push_back({a, b});
        std::vector<std::vector<Transition>> perPair(pairs.size());
        std::vector<float> perPairMin(pairs.size(),
                                      std::numeric_limits<float>::max());
        std::vector<int> perPairRaw(pairs.size(), 0);
        std::atomic<size_t> cursor{0};
        auto worker = [&]() {
            CostMatrix D;
            std::vector<TransitionCand> cands;
            for (;;) {
                const size_t pi = cursor.fetch_add(1);
                if (pi >= pairs.size()) return;
                const int a = pairs[pi].a, b = pairs[pi].b;
                computeCostMatrix(clouds[a], clouds[b], k, skel.height, D);
                if (D.empty()) continue;
                for (float v : D.d) perPairMin[pi] = std::min(perPairMin[pi], v);
                findLocalMinima(D, gp.thresholdFrac, cands);
                if (a == b) {
                    // Drop overlapping-window self matches (the trivial
                    // identity diagonal and its shoulder).
                    cands.erase(std::remove_if(cands.begin(), cands.end(),
                                               [&](const TransitionCand& t) {
                                                   return std::abs((t.j - k + 1) - t.i) < 2 * k;
                                               }),
                                cands.end());
                }
                perPairRaw[pi] = int(cands.size());
                std::sort(cands.begin(), cands.end(),
                          [](const TransitionCand& x, const TransitionCand& y) {
                              return x.cost < y.cost;
                          });
                if (int(cands.size()) > gp.maxPerPair) cands.resize(gp.maxPerPair);
                for (const TransitionCand& t : cands) {
                    Transition tr;
                    tr.clipA = a;
                    tr.clipB = b;
                    tr.i = t.i;
                    tr.j = t.j;
                    tr.cost = t.cost;
                    tr.align = optimalAlign(clouds[a], t.i, clouds[b], t.j, k,
                                            skel.height).xf;
                    perPair[pi].push_back(std::move(tr));
                }
            }
        };
        int nt = gp.numThreads > 0 ? gp.numThreads
                                   : int(std::thread::hardware_concurrency());
        nt = std::max(1, std::min<int>(nt, int(pairs.size())));
        std::vector<std::thread> threads;
        for (int t = 1; t < nt; ++t) threads.emplace_back(worker);
        worker();
        for (auto& th : threads) th.join();

        for (size_t pi = 0; pi < pairs.size(); ++pi) {
            stats.rawCandidates += perPairRaw[pi];
            stats.minCostSeen = std::min(stats.minCostSeen, perPairMin[pi]);
            for (auto& tr : perPair[pi]) transitions.push_back(std::move(tr));
        }
        if (transitions.empty()) {
            if (err)
                *err = "no transitions under threshold (best cost seen: " +
                       std::to_string(stats.minCostSeen) + ")";
            return false;
        }

        // 3. Nodes from cut points; segment + transition edges.
        std::vector<std::vector<int>> cuts(nc);
        for (int c = 0; c < nc; ++c) {
            cuts[c].push_back(0);
            cuts[c].push_back(int(clips[c].frames.size()));
        }
        for (const Transition& tr : transitions) {
            cuts[tr.clipA].push_back(tr.i);
            cuts[tr.clipB].push_back(tr.j + 1);
        }
        std::vector<int> nodeBase(nc + 1, 0);
        for (int c = 0; c < nc; ++c) {
            auto& cs = cuts[c];
            std::sort(cs.begin(), cs.end());
            cs.erase(std::unique(cs.begin(), cs.end()), cs.end());
            nodeBase[c + 1] = nodeBase[c] + int(cs.size());
        }
        auto nodeId = [&](int clip, int frame) {
            const auto& cs = cuts[clip];
            const int off = int(std::lower_bound(cs.begin(), cs.end(), frame) -
                                cs.begin());
            return nodeBase[clip] + off;
        };
        nodes.assign(nodeBase[nc], Node{});
        for (int c = 0; c < nc; ++c)
            for (size_t s = 0; s < cuts[c].size(); ++s)
                nodes[nodeBase[c] + s] = Node{c, cuts[c][s], {}};
        auto addEdge = [&](Edge e) {
            const int id = int(edges.size());
            nodes[e.from].out.push_back(id);
            edges.push_back(e);
        };
        for (int c = 0; c < nc; ++c) {
            for (size_t s = 0; s + 1 < cuts[c].size(); ++s) {
                Edge e;
                e.from = nodeBase[c] + int(s);
                e.to = nodeBase[c] + int(s) + 1;
                e.clip = c;
                e.f0 = cuts[c][s];
                e.f1 = cuts[c][s + 1];
                addEdge(e);
            }
        }
        for (size_t t = 0; t < transitions.size(); ++t) {
            Edge e;
            e.from = nodeId(transitions[t].clipA, transitions[t].i);
            e.to = nodeId(transitions[t].clipB, transitions[t].j + 1);
            e.trans = int(t);
            addEdge(e);
        }
        stats.nodesTotal = int(nodes.size());

        // 4. Largest SCC (iterative Tarjan), then prune.
        const int n = int(nodes.size());
        std::vector<int> idx(n, -1), low(n, 0), comp(n, -1);
        std::vector<char> onStk(n, 0);
        std::vector<int> stk;
        int counter = 0, ncomp = 0;
        struct Fr { int v; size_t ei; };
        std::vector<Fr> call;
        for (int root = 0; root < n; ++root) {
            if (idx[root] >= 0) continue;
            call.push_back({root, 0});
            idx[root] = low[root] = counter++;
            stk.push_back(root);
            onStk[root] = 1;
            while (!call.empty()) {
                Fr& f = call.back();
                if (f.ei < nodes[f.v].out.size()) {
                    const int w = edges[nodes[f.v].out[f.ei++]].to;
                    if (idx[w] < 0) {
                        idx[w] = low[w] = counter++;
                        stk.push_back(w);
                        onStk[w] = 1;
                        call.push_back({w, 0});
                    } else if (onStk[w]) {
                        low[f.v] = std::min(low[f.v], idx[w]);
                    }
                } else {
                    const int v = f.v;
                    if (low[v] == idx[v]) {
                        for (;;) {
                            const int w = stk.back();
                            stk.pop_back();
                            onStk[w] = 0;
                            comp[w] = ncomp;
                            if (w == v) break;
                        }
                        ++ncomp;
                    }
                    call.pop_back();
                    if (!call.empty())
                        low[call.back().v] = std::min(low[call.back().v], low[v]);
                }
            }
        }
        std::vector<int> compSize(ncomp, 0);
        for (int v = 0; v < n; ++v) ++compSize[comp[v]];
        const int best = int(std::max_element(compSize.begin(), compSize.end()) -
                             compSize.begin());
        if (compSize[best] < 2) {
            if (err) *err = "graph has no strongly connected walk (raise the threshold or add clips)";
            nodes.clear();
            edges.clear();
            transitions.clear();
            return false;
        }

        // Compact nodes/edges/transitions to the winning SCC.
        std::vector<int> nodeMap(n, -1);
        std::vector<Node> keptNodes;
        for (int v = 0; v < n; ++v) {
            if (comp[v] != best) continue;
            nodeMap[v] = int(keptNodes.size());
            Node nn = nodes[v];
            nn.out.clear();
            keptNodes.push_back(nn);
        }
        std::vector<Edge> keptEdges;
        std::vector<int> transMap(transitions.size(), -1);
        std::vector<Transition> keptTrans;
        for (const Edge& e : edges) {
            if (nodeMap[e.from] < 0 || nodeMap[e.to] < 0) continue;
            Edge ne = e;
            ne.from = nodeMap[e.from];
            ne.to = nodeMap[e.to];
            if (e.trans >= 0) {
                if (transMap[e.trans] < 0) {
                    transMap[e.trans] = int(keptTrans.size());
                    keptTrans.push_back(std::move(transitions[e.trans]));
                }
                ne.trans = transMap[e.trans];
            }
            keptNodes[ne.from].out.push_back(int(keptEdges.size()));
            keptEdges.push_back(ne);
        }
        nodes = std::move(keptNodes);
        edges = std::move(keptEdges);
        transitions = std::move(keptTrans);

        // 5. Bake the surviving transitions' blend frames.
        std::vector<FrameCloud>().swap(clouds);
        for (Transition& tr : transitions)
            bakeTransitionFrames(clips[tr.clipA], tr.i, clips[tr.clipB], tr.j, k,
                                 tr.align, tr.frames);

        std::vector<char> clipSeen(nc, 0);
        for (const Node& nd : nodes) clipSeen[nd.clip] = 1;
        for (int c = 0; c < nc; ++c)
            if (!clipSeen[c]) stats.clipsDroppedBySCC.push_back(c);
        stats.nodesScc = int(nodes.size());
        stats.edgesKept = int(edges.size());
        stats.transitionsKept = int(transitions.size());
        stats.buildMs = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t0)
                            .count();

        // Sanity: every kept node must have an out edge (SCC size >= 2
        // guarantees it, but a violated invariant here would hang the walk).
        for (const Node& nd : nodes) {
            if (nd.out.empty()) {
                if (err) *err = "internal: SCC node without out-edge";
                nodes.clear();
                edges.clear();
                transitions.clear();
                return false;
            }
        }
        return true;
    }
};

// ---- random walk baker ----------------------------------------------------------

// Streams a random graph walk into a LocalPose track. The cumulative
// XformXZ G maps clip-local coordinates into the walk's world frame; taking
// a transition composes its alignment into G, which is exactly what keeps
// the body continuous (no teleports) across clips.
struct WalkBaker {
    std::mt19937 rng;
    int edge = -1;
    int posInEdge = 0;
    XformXZ G;
    std::vector<LocalPose> baked;
    long long bakedStart = 0;
    bool stuck = false;
    // P(take a transition) at nodes offering both choices. Uniform edge
    // choice switches clips every ~0.5 s on dense graphs — visually jittery.
    float transitionBias = 0.3f;
    // Frames to stay in a clip before transitions are considered again
    // (dense graphs put a decision node every few frames).
    int minDwellFrames = 45;
    int sinceTransition = 1 << 28;

    struct LabelSpan {
        long long start = 0;
        int clip = -1;      // -1 → transition
        int toClip = -1;    // transition target
    };
    std::vector<LabelSpan> labels;

    static constexpr size_t kMaxBaked = 36000;  // ~20 min at 30 fps

    void start(const MotionGraph& g, const std::vector<Clip>& clips,
               uint32_t seed) {
        baked.clear();
        labels.clear();
        bakedStart = 0;
        stuck = g.nodes.empty();
        G = XformXZ{};
        sinceTransition = 1 << 28;
        rng.seed(seed);
        if (stuck) return;
        const int node = int(rng() % g.nodes.size());
        edge = g.nodes[node].out[rng() % g.nodes[node].out.size()];
        posInEdge = 0;
        // Recenter so the walk starts at the body's placed xz position.
        const MotionGraph::Edge& e = g.edges[edge];
        const LocalPose& p0 = e.trans >= 0 ? g.transitions[e.trans].frames[0]
                                           : clips[e.clip].frames[e.f0];
        G.tx = -p0.rootPos[0];
        G.tz = -p0.rootPos[2];
        pushLabel(g, 0);
    }

    void pushLabel(const MotionGraph& g, long long frame) {
        const MotionGraph::Edge& e = g.edges[edge];
        LabelSpan s;
        s.start = frame;
        if (e.trans >= 0) {
            s.clip = -1;
            s.toClip = g.transitions[e.trans].clipB;
        } else {
            s.clip = e.clip;
        }
        if (!labels.empty() && labels.back().clip == s.clip &&
            labels.back().toClip == s.toClip)
            return;
        labels.push_back(s);
    }

    int pickEdge(const MotionGraph& g, const MotionGraph::Node& nd) {
        int nSeg = 0, nTr = 0;
        for (int eid : nd.out) (g.edges[eid].trans >= 0 ? nTr : nSeg)++;
        const bool dwellOk = sinceTransition >= minDwellFrames;
        const bool wantTr =
            nTr > 0 &&
            (nSeg == 0 ||
             (dwellOk && float(rng() % 10000) / 10000.0f < transitionBias));
        int pick = int(rng() % (wantTr ? nTr : nSeg));
        for (int eid : nd.out) {
            const bool isTr = g.edges[eid].trans >= 0;
            if (isTr != wantTr) continue;
            if (pick-- == 0) return eid;
        }
        return nd.out[0];  // unreachable
    }

    void ensureBaked(const MotionGraph& g, const std::vector<Clip>& clips,
                     long long frame) {
        if (stuck || edge < 0) return;
        while (bakedStart + (long long)baked.size() <= frame) {
            const MotionGraph::Edge& e = g.edges[edge];
            LocalPose p;
            int len;
            if (e.trans >= 0) {
                const auto& tr = g.transitions[e.trans];
                len = int(tr.frames.size());
                p = tr.frames[posInEdge];
            } else {
                len = e.f1 - e.f0;
                p = clips[e.clip].frames[e.f0 + posInEdge];
            }
            G.applyPose(p);
            baked.push_back(std::move(p));
            ++posInEdge;
            ++sinceTransition;
            if (posInEdge >= len) {
                if (e.trans >= 0) {
                    G = G.compose(g.transitions[e.trans].align);
                    sinceTransition = 0;
                }
                const MotionGraph::Node& nd = g.nodes[e.to];
                if (nd.out.empty()) { stuck = true; return; }
                edge = pickEdge(g, nd);
                posInEdge = 0;
                pushLabel(g, bakedStart + (long long)baked.size());
            }
            if (baked.size() > kMaxBaked) {
                const size_t drop = kMaxBaked / 2;
                baked.erase(baked.begin(), baked.begin() + drop);
                bakedStart += (long long)drop;
                // Keep the newest label at or before the new start plus all after.
                size_t firstKeep = 0;
                for (size_t l = 0; l < labels.size(); ++l)
                    if (labels[l].start <= bakedStart) firstKeep = l;
                labels.erase(labels.begin(), labels.begin() + firstKeep);
            }
        }
    }
};

// ---- session facade --------------------------------------------------------------

struct SessionParams {
    float thresholdFrac = 0.10f;
    float markerScaleFrac = 0.10f;  // joint-axis marker length / body height
    float windowSec = 1.0f / 3.0f;
    int maxPerPair = 40;
    uint32_t seed = 12345;
    float transitionBias = 0.3f;  // walk: P(transition) vs continue
    // DTW blend (Mode::Blend) knobs.
    int slopeLimit = 2;       // warp slope bound [1/L, L]
    float blendSec = 0.5f;    // crossfade half-length, each side of the anchor
};

// One graph-playback context per kinematic body. Owns the reference
// skeleton, retargeted clips, and either a finite baked transition track or
// a streaming random walk. samplePose() is the single entry the per-frame
// FK update consumes; future blend modes plug in as new build*()/sample
// paths over the same Clip/LocalPose data.
struct Session {
    enum class Mode : uint8_t { Off = 0, Transition = 1, RandomWalk = 2, Blend = 3 };

    // Transition and Blend both bake one finite, scrubbable LocalPose track.
    bool trackMode() const { return mode == Mode::Transition || mode == Mode::Blend; }

    Mode mode = Mode::Off;
    Skeleton skel;
    float dt = 1.0f / 30.0f;
    std::vector<Clip> clips;
    SessionParams params;

    // Transition payload.
    std::vector<LocalPose> track;
    struct TransInfo {
        int i = 0, j = 0, blendFrames = 0;
        int prefixFrames = 0;  // composite frame where the blend starts
        float cost = 0.0f;
        bool aboveThreshold = false;
    } trans;

    // Random-walk payload.
    MotionGraph graph;
    WalkBaker walker;

    std::string note;      // build summary for the GUI status line
    double buildMs = 0.0;

    bool ready() const {
        if (trackMode()) return !track.empty();
        if (mode == Mode::RandomWalk) return !graph.nodes.empty() && !clips.empty();
        return false;
    }

    void clear() {
        mode = Mode::Off;
        clips.clear();
        track.clear();
        graph = MotionGraph{};
        walker = WalkBaker{};
        note.clear();
    }

    // Finite for transitions/blends; effectively unbounded for walks.
    double duration() const {
        if (trackMode()) return double(track.size()) * dt;
        if (mode == Mode::RandomWalk) return 1e18;
        return 0.0;
    }

    int windowFrames() const {
        int k = int(std::lround(params.windowSec / dt));
        return std::max(3, k);
    }

    // Best Kovar transition A→B baked into one scrubbable track:
    // A[0..i-1] + k blended frames + aligned B[j+1..]. When no local
    // minimum beats the threshold the global minimum is used and flagged,
    // so the user always gets a playable (if imperfect) result.
    bool buildTransition(const bvh::Motion& A, const std::string& nameA,
                         const bvh::Motion& B, const std::string& nameB,
                         const SessionParams& p, std::string* err) {
        clear();
        params = p;
        if (!A.valid() || !B.valid()) {
            if (err) *err = "invalid motion";
            return false;
        }
        skel = Skeleton::extract(A);
        dt = A.frameTime;
        const bool self = &A == &B || nameA == nameB;
        if (!self && !skel.compatible(Skeleton::extract(B))) {
            if (err) *err = "'" + nameB + "' has an incompatible skeleton";
            return false;
        }
        clips.resize(2);
        if (!sampleClip(A, skel, dt, nameA, clips[0]) ||
            !sampleClip(B, skel, dt, nameB, clips[1])) {
            if (err) *err = "failed to sample clips";
            return false;
        }
        const int k = windowFrames();
        if (int(clips[0].frames.size()) < k + 1 ||
            int(clips[1].frames.size()) < k + 1) {
            if (err) *err = "clip shorter than the blend window";
            return false;
        }
        const auto t0 = std::chrono::steady_clock::now();
        const float markerScale = p.markerScaleFrac * skel.height;
        FrameCloud ca, cb;
        ca.build(skel, clips[0], markerScale);
        cb.build(skel, clips[1], markerScale);
        CostMatrix D;
        computeCostMatrix(ca, cb, k, skel.height, D);
        if (D.empty()) {
            if (err) *err = "cost matrix empty";
            return false;
        }
        std::vector<TransitionCand> cands;
        findLocalMinima(D, p.thresholdFrac, cands);
        if (self) {
            cands.erase(std::remove_if(cands.begin(), cands.end(),
                                       [&](const TransitionCand& t) {
                                           return std::abs((t.j - k + 1) - t.i) < 2 * k;
                                       }),
                        cands.end());
        }
        TransitionCand pick;
        if (!cands.empty()) {
            pick = *std::min_element(cands.begin(), cands.end(),
                                     [](const TransitionCand& x, const TransitionCand& y) {
                                         return x.cost < y.cost;
                                     });
            trans.aboveThreshold = false;
        } else {
            // Global minimum fallback (flagged) — exclude self overlap.
            float bestCost = std::numeric_limits<float>::max();
            int bi = -1, bjj = -1;
            for (int i = 0; i < D.ra; ++i) {
                for (int jj = 0; jj < D.rb; ++jj) {
                    if (self && std::abs(jj - i) < 2 * k) continue;
                    const float c = D.at(i, jj);
                    if (c < bestCost) { bestCost = c; bi = i; bjj = jj; }
                }
            }
            if (bi < 0) {
                if (err) *err = "no usable transition cell";
                return false;
            }
            pick = {bi, bjj + k - 1, bestCost};
            trans.aboveThreshold = true;
        }
        const XformXZ align =
            optimalAlign(ca, pick.i, cb, pick.j, k, skel.height).xf;

        track.clear();
        track.reserve(size_t(pick.i) + k + clips[1].frames.size());
        for (int f = 0; f < pick.i; ++f) track.push_back(clips[0].frames[f]);
        trans.prefixFrames = pick.i;
        std::vector<LocalPose> blendFr;
        bakeTransitionFrames(clips[0], pick.i, clips[1], pick.j, k, align, blendFr);
        for (auto& f : blendFr) track.push_back(std::move(f));
        for (size_t f = size_t(pick.j) + 1; f < clips[1].frames.size(); ++f) {
            LocalPose lp = clips[1].frames[f];
            align.applyPose(lp);
            track.push_back(std::move(lp));
        }
        trans.i = pick.i;
        trans.j = pick.j;
        trans.cost = pick.cost;
        trans.blendFrames = k;
        buildMs = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
        mode = Mode::Transition;
        note = nameA + "[" + std::to_string(pick.i) + "] -> " + nameB + "[" +
               std::to_string(pick.j) + "], cost " + std::to_string(pick.cost);
        return true;
    }

    // DTW timewarp blend A→B (RegCurves 2003) baked into one scrubbable track.
    // Unlike buildTransition's rigid 1:1 frame pairing, the crossfade follows a
    // slope-limited dynamic-time-warp correspondence, so B's timeline is *time
    // scaled* to match A's phase through the blend (e.g. a fast footfall aligned
    // to a slow one). Track = A[0..aStart-1] + crossfade along the warp around
    // the best-matching anchor + aligned B suffix.
    bool buildBlend(const bvh::Motion& A, const std::string& nameA,
                    const bvh::Motion& B, const std::string& nameB,
                    const SessionParams& p, std::string* err) {
        clear();
        params = p;
        if (!A.valid() || !B.valid()) {
            if (err) *err = "invalid motion";
            return false;
        }
        skel = Skeleton::extract(A);
        dt = A.frameTime;
        const bool self = &A == &B || nameA == nameB;
        if (!self && !skel.compatible(Skeleton::extract(B))) {
            if (err) *err = "'" + nameB + "' has an incompatible skeleton";
            return false;
        }
        clips.resize(2);
        if (!sampleClip(A, skel, dt, nameA, clips[0]) ||
            !sampleClip(B, skel, dt, nameB, clips[1])) {
            if (err) *err = "failed to sample clips";
            return false;
        }
        const int k = windowFrames();
        if (int(clips[0].frames.size()) < k + 1 ||
            int(clips[1].frames.size()) < k + 1) {
            if (err) *err = "clip shorter than the blend window";
            return false;
        }
        const auto t0 = std::chrono::steady_clock::now();
        const float markerScale = p.markerScaleFrac * skel.height;
        FrameCloud ca, cb;
        ca.build(skel, clips[0], markerScale);
        cb.build(skel, clips[1], markerScale);
        CostMatrix D;
        computeCostMatrix(ca, cb, k, skel.height, D);
        if (D.empty()) {
            if (err) *err = "cost matrix empty";
            return false;
        }
        WarpPath path;
        if (!dtwPath(D, p.slopeLimit, path)) {
            if (err) *err = "DTW found no warp path";
            return false;
        }
        // Anchor: lowest-cost correspondence on the warp (the blend pivot).
        int anchor = 0;
        float anchorCost = std::numeric_limits<float>::max();
        for (size_t c = 0; c < path.cells.size(); ++c) {
            const float v = D.at(path.cells[c].first, path.cells[c].second);
            if (v < anchorCost) { anchorCost = v; anchor = int(c); }
        }
        const int half = std::max(k, int(std::lround(p.blendSec / dt)));
        const int p0 = std::max(0, anchor - half);
        const int p1 = std::min(int(path.cells.size()) - 1, anchor + half);
        auto aFrame = [&](int c) { return path.cells[c].first; };   // A window start
        auto bFrame = [&](int c) { return path.cells[c].second; };  // B window start
        const XformXZ align =
            optimalAlign(ca, path.cells[anchor].first, cb,
                         path.cells[anchor].second + k - 1, k, skel.height).xf;

        const int aStart = aFrame(p0);
        const int bEnd = bFrame(p1);
        track.clear();
        track.reserve(size_t(aStart) + (p1 - p0 + 1) + clips[1].frames.size());
        for (int f = 0; f < aStart; ++f) track.push_back(clips[0].frames[f]);
        trans.prefixFrames = aStart;
        const int blendLen = p1 - p0 + 1;
        LocalPose bAligned, mixed;
        for (int d = 0; d < blendLen; ++d) {
            const int c = p0 + d;
            const float u = float(d + 1) / float(blendLen);
            const float wa = 2.0f * u * u * u - 3.0f * u * u + 1.0f;
            bAligned = clips[1].frames[bFrame(c)];
            align.applyPose(bAligned);
            blendPose(clips[0].frames[aFrame(c)], bAligned, wa, mixed);
            track.push_back(mixed);
        }
        for (int f = bEnd + 1; f < int(clips[1].frames.size()); ++f) {
            LocalPose lp = clips[1].frames[f];
            align.applyPose(lp);
            track.push_back(std::move(lp));
        }
        trans.i = path.cells[anchor].first;
        trans.j = path.cells[anchor].second + k - 1;
        trans.cost = anchorCost;
        trans.blendFrames = blendLen;
        trans.aboveThreshold = anchorCost >= p.thresholdFrac;
        buildMs = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
        mode = Mode::Blend;
        note = nameA + " ~DTW~ " + nameB + ", " + std::to_string(blendLen) +
               " blend frames, warp cost " + std::to_string(path.cost);
        return true;
    }

    // Motion graph over `motions` (skeleton-compatible subset), random walk.
    // `motions[0]` provides the reference skeleton.
    bool buildRandomWalk(const std::vector<const bvh::Motion*>& motions,
                         const std::vector<std::string>& names,
                         const SessionParams& p, std::string* err,
                         std::vector<std::string>* skipped = nullptr) {
        clear();
        params = p;
        if (motions.empty() || !motions[0]->valid()) {
            if (err) *err = "no motions";
            return false;
        }
        skel = Skeleton::extract(*motions[0]);
        dt = motions[0]->frameTime;
        for (size_t m = 0; m < motions.size(); ++m) {
            if (m > 0 && !skel.compatible(Skeleton::extract(*motions[m]))) {
                if (skipped) skipped->push_back(names[m]);
                continue;
            }
            Clip c;
            if (!sampleClip(*motions[m], skel, dt, names[m], c)) {
                if (skipped) skipped->push_back(names[m]);
                continue;
            }
            clips.push_back(std::move(c));
        }
        if (clips.empty()) {
            if (err) *err = "no compatible clips";
            return false;
        }
        GraphParams gp;
        gp.windowFrames = windowFrames();
        gp.thresholdFrac = p.thresholdFrac;
        gp.markerScaleFrac = p.markerScaleFrac;
        gp.maxPerPair = p.maxPerPair;
        if (!graph.build(clips, skel, gp, err)) return false;
        walker.transitionBias = p.transitionBias;
        walker.start(graph, clips, p.seed);
        buildMs = graph.stats.buildMs;
        mode = Mode::RandomWalk;
        note = std::to_string(graph.stats.nodesScc) + " nodes / " +
               std::to_string(graph.stats.edgesKept) + " edges / " +
               std::to_string(graph.stats.transitionsKept) + " transitions";
        return true;
    }

    void reseed(uint32_t seed) {
        if (mode != Mode::RandomWalk || graph.nodes.empty()) return;
        params.seed = seed;
        walker.start(graph, clips, seed);
    }

    bool samplePose(double t, bvh::Pose& out) {
        if (!ready()) return false;
        if (trackMode()) {
            const double ff = std::max(0.0, t / dt);
            size_t f0 = size_t(ff);
            if (f0 >= track.size() - 1) {
                fk(skel, track.back(), out);
                return true;
            }
            LocalPose mid;
            blendPose(track[f0], track[f0 + 1], 1.0f - float(ff - double(f0)), mid);
            fk(skel, mid, out);
            return true;
        }
        // Random walk: bake on demand, then lerp.
        const long long f = std::max(0LL, (long long)(t / dt));
        walker.ensureBaked(graph, clips, f + 1);
        const long long last =
            walker.bakedStart + (long long)walker.baked.size() - 1;
        if (last < 0) return false;
        const long long c0 = std::min(std::max(f, walker.bakedStart), last);
        const long long c1 = std::min(c0 + 1, last);
        const float a = float(std::min(std::max(t / dt - double(c0), 0.0), 1.0));
        LocalPose mid;
        blendPose(walker.baked[size_t(c0 - walker.bakedStart)],
                  walker.baked[size_t(c1 - walker.bakedStart)], 1.0f - a, mid);
        fk(skel, mid, out);
        return true;
    }

    // Display label for the GUI: which clip (or transition) plays at `t`.
    std::string currentLabel(double t) const {
        if (trackMode()) {
            if (track.empty() || clips.size() < 2) return "";
            const char* arrow = mode == Mode::Blend ? " ~ " : " -> ";
            const double f = t / dt;
            if (f < double(trans.prefixFrames)) return clips[0].name;
            if (f < double(trans.prefixFrames + trans.blendFrames))
                return clips[0].name + arrow + clips[1].name;
            return clips[1].name;
        }
        if (mode == Mode::RandomWalk) {
            const long long f = (long long)(t / dt);
            const WalkBaker::LabelSpan* cur = nullptr;
            for (const auto& s : walker.labels) {
                if (s.start <= f) cur = &s;
                else break;
            }
            if (!cur) return "";
            if (cur->clip < 0)
                return cur->toClip >= 0 && cur->toClip < int(clips.size())
                           ? "-> " + clips[cur->toClip].name
                           : "";
            return cur->clip < int(clips.size()) ? clips[cur->clip].name : "";
        }
        return "";
    }

    // Source mix at `t` for the Blend track: 1 = pure clip A, 0 = pure clip B,
    // smooth crossfade (same w_A = 2u³-3u²+1 curve the bake uses) across the
    // blend region. -1 when not a built blend (caller falls back to no tint).
    float blendWeightA(double t) const {
        if (mode != Mode::Blend || track.empty() || trans.blendFrames <= 0)
            return -1.0f;
        const double f = t / dt;
        if (f <= double(trans.prefixFrames)) return 1.0f;
        const double d = f - double(trans.prefixFrames);
        if (d >= double(trans.blendFrames)) return 0.0f;
        const float u = float((d + 1.0) / double(trans.blendFrames));
        return 2.0f * u * u * u - 3.0f * u * u + 1.0f;
    }
};

}  // namespace mograph

#endif  // YSIM_MOTION_GRAPH_HPP
