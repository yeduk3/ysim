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
    enum class Mode : uint8_t {
        Off = 0, Transition = 1, RandomWalk = 2, Blend = 3, BlendSpace = 4
    };
    // Blend-space root handling (AAT07 "absolute vs relative"). Relative (default)
    // pins each clip + integrates blended heading-relative velocity → curving
    // travel-in-place. Absolute rebases each clip to its own frame 0 and blends
    // the absolute root position/orientation directly (no integrator); the body
    // is then anchored frame-0→object by applyRootRebase. Chosen at build.
    enum class RootMode : uint8_t { Relative = 0, Absolute = 1 };

    // Transition and Blend both bake one finite, scrubbable LocalPose track.
    bool trackMode() const { return mode == Mode::Transition || mode == Mode::Blend; }

    Mode mode = Mode::Off;
    RootMode rootMode = RootMode::Relative;
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

    // Interactive blend-space payload (Mode::BlendSpace): N looping clips, each
    // placed at a 2D coordinate; a live cursor produces inverse-distance weights
    // that blend the clips per frame. No baked track — samplePose blends live.
    std::vector<std::array<float, 2>> clipCoords;  // parallel to clips
    std::array<float, 2> cursor{0.0f, 0.0f};
    // Fixed gait period (seconds), set once at build = mean clip duration. The
    // live phase = fmod(t, blendCycleSec)/blendCycleSec is CURSOR-INDEPENDENT:
    // a weight-dependent period would make fmod(large t, cycle) jump whenever
    // the cursor moves (the modulus shifts under a large dividend) → a pose
    // pop. Clips stay phase-synced on this common normalized timeline.
    double blendCycleSec = 1.0;
    // Joint-rotation blend method for the N-way mix. true = intrinsic (Karcher)
    // mean (order-independent, no spurious acceleration); false = the legacy
    // order-dependent incremental slerp. Default on; the toggle is the fallback.
    bool useIntrinsicMean = true;
    // Thin-plate RBF cursor→weight scheme (Rose "Verbs & Adverbs"). Built once
    // from clipCoords: cardinal functions w_i(x)=Σ_j λ_ij·φ(|x-x_j|)+a_i+b_i·x+
    // c_i·y with φ(r)=r²ln(r), so w_i(x_j)=δ_ij exactly and the affine poly term
    // makes Σ_i w_i≡1. Empty (n<2 or a singular system from coincident samples)
    // ⇒ blendWeights falls back to inverse-distance. Weights may go negative
    // (extrapolation) — accepted, not clamped (matches the paper).
    // ponytail: O(n²) per-frame eval; n = #blend clips is tiny so no hoist.
    bool useRbf = true;
    // Convex weights: clamp the RBF cardinals to ≥0 then renormalise to Σ=1, so
    // the blended pose stays inside the convex hull of the sample poses (no
    // extrapolation overshoot — a blend-space instability source). Off ⇒ the raw
    // signed partition (paper-exact, may overshoot). IDW is already non-negative.
    bool convexWeights = true;
    std::vector<float> rbfLambda;  // n*n, cardinal i basis j → rbfLambda[i*n+j]
    std::vector<float> rbfPoly;    // n*3, cardinal i → {a_i,b_i,c_i}
    // Root motion (velocity-integrated, with turning). The clips are pinned in
    // place (no absolute root motion → no seam teleport); locomotion is added
    // back by blending each clip's per-frame HEADING-RELATIVE root velocity
    // {localVx, localVz, yawRate} and integrating it: blendYaw += Σ w_i·ω_i·dt,
    // blendPos += R(blendYaw)·(Σ w_i·v_i)·dt. Integration (not v·t) keeps the
    // position continuous when the mix changes mid-playback, and the heading
    // term lets the body curve (e.g. blending toward jogCurve turns it).
    std::vector<std::vector<std::array<float, 3>>> clipRootVel;  // [clip][frame]
    // Phase registration (Kovar & Gleicher "Registration Curves" 2003): per-clip
    // cyclic warp LUT mapping the shared phase → that clip's frame, so a common
    // phase hits the SAME gait event in every clip. Without it the shared
    // phase·nf map assumes uniform phase correspondence — false across
    // walk/run/sneak/limp (different stance/swing ratios + arbitrary loop seams)
    // → stance-vs-swing poses get averaged → jitter. Built once vs a reference
    // clip in buildPhaseRegistration(); an empty LUT for a clip ⇒ linear
    // phase·nf fallback. Parallel to clips. registerPhase off ⇒ all linear.
    bool registerPhase = true;
    std::vector<std::vector<float>> clipPhaseLUT;   // parallel to clips; [clip][R]
    static constexpr int kPhaseLUTRes = 256;        // LUT samples over one cycle
    double blendYaw = 0.0;          // integrated heading (rad)
    double blendPosX = 0.0, blendPosZ = 0.0;  // integrated world root position
    double blendTravelLastT = -1.0;  // last samplePose t (dt clock); <0 = uninit

    std::string note;      // build summary for the GUI status line
    double buildMs = 0.0;

    bool ready() const {
        if (trackMode()) return !track.empty();
        if (mode == Mode::RandomWalk) return !graph.nodes.empty() && !clips.empty();
        if (mode == Mode::BlendSpace) return !clips.empty();
        return false;
    }

    void clear() {
        mode = Mode::Off;
        rootMode = RootMode::Relative;
        clips.clear();
        track.clear();
        graph = MotionGraph{};
        walker = WalkBaker{};
        clipCoords.clear();
        clipRootVel.clear();
        clipPhaseLUT.clear();
        rbfLambda.clear();
        rbfPoly.clear();
        resetBlendTravel();
        note.clear();
    }

    // Reset just the root-motion integrator (on seek / time set), keep the build.
    void resetBlendTravel() {
        blendYaw = 0.0;
        blendPosX = 0.0;
        blendPosZ = 0.0;
        blendTravelLastT = -1.0;
    }

    // Blended heading-relative root velocity {vx, vz, yawRate} at normalized
    // phase under weights `w` — per-frame lerp (looping), summed by weight.
    std::array<double, 3> blendRootVel(float phase,
                                       const std::vector<float>& w) const {
        double bvx = 0.0, bvz = 0.0, bom = 0.0;
        for (size_t i = 0; i < clips.size() && i < clipRootVel.size() &&
                           i < w.size(); ++i) {
            const auto& rv = clipRootVel[i];
            const int nf = int(rv.size());
            if (nf == 0) continue;
            // Warp the velocity phase by the SAME LUT as the pose, else travel
            // desyncs from the gait-aligned pose.
            const bool reg = registerPhase && i < clipPhaseLUT.size() &&
                             !clipPhaseLUT[i].empty();
            const float ff = reg ? warpedFrame(clipPhaseLUT[i], phase, nf)
                                 : (phase - std::floor(phase)) * float(nf);
            const int f0 = int(ff) % nf;
            const int f1 = (f0 + 1) % nf;
            const float a = ff - std::floor(ff);
            const float vx = rv[f0][0] * (1 - a) + rv[f1][0] * a;
            const float vz = rv[f0][1] * (1 - a) + rv[f1][1] * a;
            const float om = rv[f0][2] * (1 - a) + rv[f1][2] * a;
            bvx += double(w[i]) * vx;
            bvz += double(w[i]) * vz;
            bom += double(w[i]) * om;
        }
        return {bvx, bvz, bom};
    }

    // Finite for transitions/blends; effectively unbounded for walks + the
    // forever-looping blend space.
    double duration() const {
        if (trackMode()) return double(track.size()) * dt;
        if (mode == Mode::RandomWalk || mode == Mode::BlendSpace) return 1e18;
        return 0.0;
    }

    // ── Blend-space weighting (Mode::BlendSpace) ──────────────────────────
    // Thin-plate radial basis φ(r)=r²ln(r), keyed on squared distance:
    // r²ln(r) = ½·r²·ln(r²). φ(0)=0.
    static double tpsPhi(double r2) {
        return r2 < 1e-12 ? 0.0 : 0.5 * r2 * std::log(r2);
    }

    // In-place Gauss-Jordan solve of A·X=B (A is mxm row-major, B is m×rhs
    // row-major; B holds X on return). Partial pivoting; false if singular.
    static bool solveDense(std::vector<double>& A, std::vector<double>& B,
                           int m, int rhs) {
        auto a = [&](int r, int c) -> double& { return A[r * m + c]; };
        auto b = [&](int r, int c) -> double& { return B[r * rhs + c]; };
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

    // Solve the thin-plate cardinal-function coefficients once (at build).
    // System: [[Φ P],[Pᵀ 0]]·[Λ;C] = [I;0]; P row i = [1,xᵢ,yᵢ]. Leaves
    // rbfLambda/rbfPoly empty on n<2 or a singular system → IDW fallback.
    void buildRbf() {
        rbfLambda.clear();
        rbfPoly.clear();
        const int n = int(clipCoords.size());
        if (n < 2) return;
        const int m = n + 3;
        std::vector<double> M(size_t(m) * m, 0.0), B(size_t(m) * n, 0.0);
        auto AT = [&](int r, int c) -> double& { return M[size_t(r) * m + c]; };
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                const double dx = clipCoords[i][0] - clipCoords[j][0];
                const double dy = clipCoords[i][1] - clipCoords[j][1];
                AT(i, j) = tpsPhi(dx * dx + dy * dy);
            }
            AT(i, n + 0) = 1.0; AT(i, n + 1) = clipCoords[i][0]; AT(i, n + 2) = clipCoords[i][1];
            AT(n + 0, i) = 1.0; AT(n + 1, i) = clipCoords[i][0]; AT(n + 2, i) = clipCoords[i][1];
        }
        for (int i = 0; i < n; ++i) B[size_t(i) * n + i] = 1.0;  // RHS = [I;0]
        if (!solveDense(M, B, m, n)) return;
        rbfLambda.assign(size_t(n) * n, 0.0f);
        rbfPoly.assign(size_t(n) * 3, 0.0f);
        for (int i = 0; i < n; ++i) {  // column i of X = cardinal i's coeffs
            for (int j = 0; j < n; ++j) rbfLambda[size_t(i) * n + j] = float(B[size_t(j) * n + i]);
            for (int k = 0; k < 3; ++k) rbfPoly[size_t(i) * 3 + k] = float(B[size_t(n + k) * n + i]);
        }
    }

    // Blend weights for `cur` over clipCoords, normalized (Σ=1). Uses the
    // thin-plate RBF cardinal functions when built (w_i(x_j)=δ_ij, smooth
    // between samples); otherwise inverse-distance with an ε singularity guard
    // (snaps onto a sample without divide-by-zero; uniform if all coincide).
    void blendWeights(const std::array<float, 2>& cur,
                      std::vector<float>& w) const {
        const int n = int(clipCoords.size());
        w.assign(size_t(n), 0.0f);
        if (n == 0) return;
        if (n == 1) { w[0] = 1.0f; return; }
        if (useRbf && int(rbfLambda.size()) == n * n &&
            int(rbfPoly.size()) == n * 3) {
            float sum = 0.0f;
            for (int i = 0; i < n; ++i) {
                double acc = double(rbfPoly[size_t(i) * 3 + 0]) +
                             double(rbfPoly[size_t(i) * 3 + 1]) * cur[0] +
                             double(rbfPoly[size_t(i) * 3 + 2]) * cur[1];
                for (int j = 0; j < n; ++j) {
                    const double dx = cur[0] - clipCoords[j][0];
                    const double dy = cur[1] - clipCoords[j][1];
                    acc += double(rbfLambda[size_t(i) * n + j]) * tpsPhi(dx * dx + dy * dy);
                }
                w[i] = float(acc);
                sum += w[i];
            }
            if (convexWeights) {
                // Clamp negatives then renormalise ⇒ a convex combination
                // (Σ=1, all ≥0): the blended pose stays in the convex hull of
                // the samples, so no extrapolation overshoot.
                float s2 = 0.0f;
                for (auto& x : w) { if (x < 0.0f) x = 0.0f; s2 += x; }
                if (s2 > 1e-6f) for (auto& x : w) x /= s2;
                else for (auto& x : w) x = 1.0f / float(n);
            } else if (std::fabs(sum) > 1e-6f) {
                for (auto& x : w) x /= sum;  // raw signed partition (Σ=1)
            }
            return;
        }
        float sum = 0.0f;
        for (int i = 0; i < n; ++i) {
            const float dx = cur[0] - clipCoords[i][0];
            const float dy = cur[1] - clipCoords[i][1];
            const float wi = 1.0f / (dx * dx + dy * dy + 1e-4f);
            w[i] = wi;
            sum += wi;
        }
        if (sum > 1e-12f) for (auto& x : w) x /= sum;
        else for (auto& x : w) x = 1.0f / float(n);
    }

    // Mean clip duration — the fixed gait period stored as blendCycleSec at
    // build time. Cursor-independent (see blendCycleSec) so the live phase
    // never jumps when the mix changes.
    double meanClipDuration() const {
        if (clips.empty()) return 1.0;
        double c = 0.0;
        for (const auto& cl : clips) c += cl.duration();
        return c / double(clips.size());
    }

    // N-way pose blend at normalized phase under weights `w` (no fk, no travel).
    void blendLocalPoseW(float phase01, bool loop, const std::vector<float>& w,
                         LocalPose& mixed) const {
        std::vector<LocalPose> poses(clips.size());
        for (size_t i = 0; i < clips.size(); ++i) {
            if (registerPhase && i < clipPhaseLUT.size() && !clipPhaseLUT[i].empty())
                sampleClipPhaseLUT(clips[i], clipPhaseLUT[i], phase01, poses[i], loop);
            else
                sampleClipPhase(clips[i], phase01, poses[i], loop);
        }
        if (useIntrinsicMean) blendPoseNMean(poses, w, mixed);
        else blendPoseN(poses, w, mixed);
    }

    // N-way blend at normalized phase `phase01` under the current cursor →
    // world pose, IN PLACE (no root travel — travel lives in samplePose). loop=
    // true wraps the gait cycle; loop=false clamps to the last frame (one-shot
    // playback). Used by the preview ghosts.
    bool sampleBlendPhase(float phase01, bool loop, bvh::Pose& out) {
        if (mode != Mode::BlendSpace || clips.empty()) return false;
        std::vector<float> w;
        blendWeights(cursor, w);
        LocalPose mixed;
        blendLocalPoseW(phase01, loop, w, mixed);
        fk(skel, mixed, out);
        return true;
    }

    int windowFrames() const {
        int k = int(std::lround(params.windowSec / dt));
        return std::max(3, k);
    }

    // Best Kovar transition A→B baked into one scrubbable track:
    // A[0..i-1] + k blended frames + aligned B[j+1..]. When no local
    // minimum beats the threshold the global minimum is used and flagged,
    // so the user always gets a playable (if imperfect) result.
    // Trim a sampled clip to its active window [start,end] (raw; end<0 = full).
    // The window's first frame becomes frame 0, so a baked track / blend that
    // consumes the clip starts there → it anchors to the kinematic object.
    // Rigidly rebase a clip so its frame 0 sits at the canonical origin (xz=0,
    // yaw=0), keeping the rest of the root trajectory relative to it (Y kept).
    // For absolute-root blending: the clip then carries its real in-cycle travel
    // and the blend averages absolute roots. Uses the file's local-frame
    // convention (matches the velocity extractor's rotation).
    static void rebaseClipToFrame0(Clip& c) {
        if (c.frames.empty() || c.frames[0].rot.empty()) return;
        std::array<float, 9> R0;
        c.frames[0].rot[0].toMat3(R0);
        const float th0 = std::atan2(R0[2], R0[0]);
        const float x0 = c.frames[0].rootPos[0], z0 = c.frames[0].rootPos[2];
        const float cs = std::cos(th0), sn = std::sin(th0);
        for (auto& p : c.frames) {
            const float dx = p.rootPos[0] - x0, dz = p.rootPos[2] - z0;
            // Rigid: position AND orientation both rotate by yaw(-th0). Same
            // form as applyRootRebase / rebaseXZYaw (main.cpp), keyed on the
            // identical th0 = atan2(R[2],R[0]).
            p.rootPos[0] = cs * dx - sn * dz;
            p.rootPos[2] = sn * dx + cs * dz;
            if (!p.rot.empty())
                p.rot[0] = (Quatf::yaw(-th0) * p.rot[0]).normalized();
        }
    }

    static void trimClip(Clip& c, const std::array<int, 2>& rg) {
        const int nf = int(c.frames.size());
        if (nf <= 0) return;
        int a = rg[0];
        int b = rg[1] < 0 ? nf - 1 : rg[1];
        a = a < 0 ? 0 : (a > nf - 1 ? nf - 1 : a);
        b = b < 0 ? 0 : (b > nf - 1 ? nf - 1 : b);
        if (b < a) b = a;
        if (a > 0 || b < nf - 1)
            c.frames = std::vector<LocalPose>(c.frames.begin() + a,
                                              c.frames.begin() + b + 1);
    }

    bool buildTransition(const bvh::Motion& A, const std::string& nameA,
                         const bvh::Motion& B, const std::string& nameB,
                         const SessionParams& p, std::string* err,
                         std::array<int, 2> rangeA = {0, -1},
                         std::array<int, 2> rangeB = {0, -1}) {
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
        trimClip(clips[0], rangeA);  // active windows → track starts at A's start
        trimClip(clips[1], rangeB);
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
                    const SessionParams& p, std::string* err,
                    std::array<int, 2> rangeA = {0, -1},
                    std::array<int, 2> rangeB = {0, -1}) {
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
        trimClip(clips[0], rangeA);  // active windows → track starts at A's start
        trimClip(clips[1], rangeB);
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

    // Convert a DTW warp path (ref window-start a × DOUBLED-clip window-start b)
    // into a cyclic warp LUT: lut[s] = fractional clip frame at the shared phase
    // s/R. A window [start, start+k-1] is represented by its centre start+(k-1)/2.
    // Runs where the ref holds (a constant, clip advances) are collapsed to a
    // mean clip frame → a single-valued monotone v(refPhase). The clip is
    // periodic (period nfClip ⇔ refPhase period 1), so samples are extended ±1
    // cycle to interpolate cleanly across the seam, then folded to [0,nfClip).
    // Pure (FK-free) → unit-testable. Returns empty on a degenerate path.
    static std::vector<float> buildPhaseLUTFromPath(const WarpPath& path,
                                                    int nfRef, int nfClip,
                                                    int k, int R) {
        if (nfRef <= 0 || nfClip <= 0 || R < 2) return {};
        const float half = 0.5f * float(k - 1);
        std::vector<float> xs, ys;  // refPhase (non-decreasing), clipFrame (raw)
        double accY = 0.0; int accN = 0; int curA = -1;
        auto flush = [&]() {
            if (accN == 0) return;
            xs.push_back((float(curA) + half) / float(nfRef));
            ys.push_back(float(accY / accN));
            accY = 0.0; accN = 0;
        };
        for (const auto& cell : path.cells) {
            if (cell.first != curA && curA >= 0) flush();
            curA = cell.first;
            accY += double(cell.second) + half;  // clip frame in [0, 2·nfClip)
            ++accN;
        }
        flush();
        if (xs.size() < 2) return {};
        // Periodic extension ±1 cycle (refPhase ±1 ⇔ clip frame ±nfClip). xs ⊂
        // [0,1) so the three blocks stay globally increasing.
        std::vector<float> ex, ey;
        ex.reserve(xs.size() * 3); ey.reserve(ys.size() * 3);
        for (int rep = -1; rep <= 1; ++rep)
            for (size_t j = 0; j < xs.size(); ++j) {
                ex.push_back(xs[j] + float(rep));
                ey.push_back(ys[j] + float(rep) * float(nfClip));
            }
        std::vector<float> lut(R, 0.0f);
        size_t seg = 0;
        for (int s = 0; s < R; ++s) {
            const float u = float(s) / float(R);
            while (seg + 1 < ex.size() && ex[seg + 1] < u) ++seg;
            const size_t s1 = seg + 1 < ex.size() ? seg + 1 : seg;
            const float x0 = ex[seg], x1 = ex[s1];
            const float y0 = ey[seg], y1 = ey[s1];
            float t = (x1 > x0 + 1e-9f) ? (u - x0) / (x1 - x0) : 0.0f;
            if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
            float ff = std::fmod(y0 + (y1 - y0) * t, float(nfClip));
            if (ff < 0.0f) ff += float(nfClip);
            lut[s] = ff;
        }
        return lut;
    }

    // Build per-clip phase-registration LUTs vs a reference clip so the shared
    // phase is gait-aligned across clips (Kovar & Gleicher 2003). Reference =
    // the clip nearest the blend-space centroid (most representative gait). Each
    // other clip is DOUBLED so the free-boundary DTW recovers its cyclic phase
    // offset; the warp path → LUT. An empty LUT for a clip ⇒ linear fallback.
    // Called at the end of buildBlendSpace. Clips are already pinned in place.
    void buildPhaseRegistration() {
        clipPhaseLUT.assign(clips.size(), {});
        const int n = int(clips.size());
        if (!registerPhase || n < 2 || skel.height <= 0.0f) return;
        std::array<float, 2> ctr{0.0f, 0.0f};
        for (const auto& c : clipCoords) { ctr[0] += c[0]; ctr[1] += c[1]; }
        ctr[0] /= float(n); ctr[1] /= float(n);
        int ref = 0; float bestD = std::numeric_limits<float>::max();
        for (int i = 0; i < n && i < int(clipCoords.size()); ++i) {
            const float dx = clipCoords[i][0] - ctr[0];
            const float dy = clipCoords[i][1] - ctr[1];
            const float d = dx * dx + dy * dy;
            if (d < bestD) { bestD = d; ref = i; }
        }
        const int nfRef = int(clips[ref].frames.size());
        if (nfRef < 4) return;  // ref too short to register against
        const float markerScale = params.markerScaleFrac * skel.height;
        // Reference identity LUT (shared phase ↦ ref frame, linear).
        clipPhaseLUT[ref].resize(kPhaseLUTRes);
        for (int s = 0; s < kPhaseLUTRes; ++s)
            clipPhaseLUT[ref][s] = (float(s) / float(kPhaseLUTRes)) * float(nfRef);
        FrameCloud cref; cref.build(skel, clips[ref], markerScale);
        for (int i = 0; i < n; ++i) {
            if (i == ref) continue;
            const int nfi = int(clips[i].frames.size());
            if (nfi < 4) continue;  // leave empty ⇒ linear
            Clip dbl; dbl.dt = clips[i].dt;
            dbl.frames = clips[i].frames;
            dbl.frames.insert(dbl.frames.end(), clips[i].frames.begin(),
                              clips[i].frames.end());
            int k = std::min({windowFrames(), nfRef - 1, 2 * nfi - 1});
            k = std::max(2, k);
            FrameCloud ci; ci.build(skel, dbl, markerScale);
            CostMatrix D;
            computeCostMatrix(cref, ci, k, skel.height, D);
            WarpPath path;
            if (D.empty() || !dtwPath(D, params.slopeLimit, path)) continue;
            clipPhaseLUT[i] = buildPhaseLUTFromPath(path, nfRef, nfi, k, kPhaseLUTRes);
        }
    }

    // Interactive blend space: retarget each compatible clip onto motions[0]'s
    // skeleton and place it at coords[i]. No graph build, no baked track — the
    // cursor's inverse-distance weights blend the clips live in samplePose.
    // motions[0] is the reference skeleton (clips[0]); later clips are skeleton-
    // gated (incompatible ones reported in `skipped`, dropped from the space).
    // `ranges` (optional, parallel to motions) trims each clip to an active
    // frame window [start,end] BEFORE velocity extraction + pinning, so the
    // window's first frame becomes the clip's frame 0 (→ anchors to the object)
    // and only those frames drive the blend. end<0 or start<0 ⇒ no trim.
    bool buildBlendSpace(const std::vector<const bvh::Motion*>& motions,
                         const std::vector<std::string>& names,
                         const std::vector<std::array<float, 2>>& coords,
                         const SessionParams& p, std::string* err,
                         std::vector<std::string>* skipped = nullptr,
                         const std::vector<std::array<int, 2>>* ranges = nullptr,
                         RootMode rmode = RootMode::Relative) {
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
            // Trim to the active window first → window start = clip frame 0.
            if (ranges && m < ranges->size() && !c.frames.empty()) {
                const int nf = int(c.frames.size());
                int a = (*ranges)[m][0];
                int b = (*ranges)[m][1] < 0 ? nf - 1 : (*ranges)[m][1];
                a = a < 0 ? 0 : (a > nf - 1 ? nf - 1 : a);
                b = b < 0 ? 0 : (b > nf - 1 ? nf - 1 : b);
                if (b < a) b = a;
                if (a > 0 || b < nf - 1)
                    c.frames = std::vector<LocalPose>(c.frames.begin() + a,
                                                      c.frames.begin() + b + 1);
            }
            if (rmode == RootMode::Absolute) {
                // Absolute root: keep the clip's real orientation (frame 0 aligned
                // to canonical, the rest relative), and carry travel as WORLD-frame
                // per-frame velocity {dx/dt, dz/dt, 0} — integrated in samplePose
                // with NO heading re-projection (the body faces its blended actual
                // orientation, not an integrated yaw). Root xz is then zeroed so
                // the per-cycle loop seam can't slide the body back to the origin.
                rebaseClipToFrame0(c);
                const float cdt = c.dt > 1e-6f ? c.dt : 1.0f;
                std::vector<std::array<float, 3>> rv(c.frames.size(), {0.0f, 0.0f, 0.0f});
                for (size_t f = 1; f < c.frames.size(); ++f) {
                    rv[f][0] = (c.frames[f].rootPos[0] - c.frames[f - 1].rootPos[0]) / cdt;
                    rv[f][1] = (c.frames[f].rootPos[2] - c.frames[f - 1].rootPos[2]) / cdt;
                    rv[f][2] = 0.0f;  // no yaw integration (orientation kept)
                }
                if (rv.size() > 1) rv[0] = rv[1];
                clipRootVel.push_back(std::move(rv));
                for (auto& fp : c.frames) { fp.rootPos[0] = 0.0f; fp.rootPos[2] = 0.0f; }
            } else {
                // Per-frame HEADING-RELATIVE root velocity {localVx, localVz,
                // yawRate} — read BEFORE pinning zeroes the root xz/heading.
                // Stored so samplePose can blend + integrate it into curving
                // travel.
                const float cdt = c.dt > 1e-6f ? c.dt : 1.0f;
                const float kTwoPi = 6.28318530718f;
                std::vector<std::array<float, 3>> rv(c.frames.size(), {0.0f, 0.0f, 0.0f});
                for (size_t f = 1; f < c.frames.size(); ++f) {
                    const float dx = c.frames[f].rootPos[0] - c.frames[f - 1].rootPos[0];
                    const float dz = c.frames[f].rootPos[2] - c.frames[f - 1].rootPos[2];
                    std::array<float, 9> R0, R1;
                    c.frames[f - 1].rot[0].toMat3(R0);
                    c.frames[f].rot[0].toMat3(R1);
                    const float yaw0 = std::atan2(R0[2], R0[0]);
                    const float yaw1 = std::atan2(R1[2], R1[0]);
                    const float dyaw = std::remainder(yaw1 - yaw0, kTwoPi);  // [-π,π]
                    const float cs = std::cos(yaw0), sn = std::sin(yaw0);
                    rv[f][0] = (cs * dx + sn * dz) / cdt;   // local forward/back
                    rv[f][1] = (-sn * dx + cs * dz) / cdt;  // local lateral
                    rv[f][2] = dyaw / cdt;                  // yaw rate
                }
                if (rv.size() > 1) rv[0] = rv[1];
                clipRootVel.push_back(std::move(rv));
                pinClipInPlace(c);  // remove root travel/turn → no seam teleport
            }
            clips.push_back(std::move(c));
            clipCoords.push_back(m < coords.size()
                                     ? coords[m]
                                     : std::array<float, 2>{0.0f, 0.0f});
        }
        if (clips.empty()) {
            if (err) *err = "no compatible clips";
            return false;
        }
        blendCycleSec = meanClipDuration();  // fixed gait period (no pop on drag)
        buildRbf();  // thin-plate cardinal coeffs (empty ⇒ IDW fallback)
        buildPhaseRegistration();  // gait-align the shared phase across clips
        resetBlendTravel();
        cursor = {0.0f, 0.0f};
        rootMode = rmode;
        mode = Mode::BlendSpace;
        note = std::to_string(clips.size()) + " clips in blend space";
        return true;
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
        if (mode == Mode::BlendSpace) {
            const double cycle = blendCycleSec;  // fixed — cursor-independent
            const double phase =
                cycle > 1e-6 ? std::fmod(std::max(0.0, t), cycle) / cycle : 0.0;
            std::vector<float> w;
            blendWeights(cursor, w);
            if (rootMode == RootMode::Absolute) {
                // Integrate blended WORLD-frame root velocity (no heading re-
                // projection) → continuous travel with no per-cycle loop pop.
                // Orientation is the blended absolute orientation (kept in pose).
                double dtA = blendTravelLastT < 0.0 ? 0.0 : (t - blendTravelLastT);
                if (dtA < 0.0 || dtA > 0.5) dtA = 0.0;
                blendTravelLastT = t;
                const auto va = blendRootVel(float(phase), w);  // {dx,dz,0} world
                blendPosX += va[0] * dtA;
                blendPosZ += va[1] * dtA;
                LocalPose mixed;
                blendLocalPoseW(float(phase), true, w, mixed);
                mixed.rootPos[0] += float(blendPosX);
                mixed.rootPos[2] += float(blendPosZ);
                fk(skel, mixed, out);
                return true;
            }
            // Integrate the blended heading-relative root velocity into a world
            // heading + position. A seek/scrub (dt < 0 or a big gap) is not
            // integrated, so the body never lurches across a time jump.
            double dtT = blendTravelLastT < 0.0 ? 0.0 : (t - blendTravelLastT);
            if (dtT < 0.0 || dtT > 0.5) dtT = 0.0;
            blendTravelLastT = t;
            const auto v = blendRootVel(float(phase), w);  // {vx, vz, yawRate} local
            blendYaw += v[2] * dtT;
            const double cy = std::cos(blendYaw), sy = std::sin(blendYaw);
            blendPosX += (cy * v[0] - sy * v[1]) * dtT;  // local→world
            blendPosZ += (sy * v[0] + cy * v[1]) * dtT;
            LocalPose mixed;
            blendLocalPoseW(float(phase), true, w, mixed);
            // Face the integrated heading, then place at the integrated position.
            mixed.rot[0] = (Quatf::yaw(float(blendYaw)) * mixed.rot[0]).normalized();
            mixed.rootPos[0] += float(blendPosX);
            mixed.rootPos[2] += float(blendPosZ);
            fk(skel, mixed, out);
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
        if (mode == Mode::BlendSpace) {
            if (clips.empty()) return "";
            std::vector<float> w;
            blendWeights(cursor, w);
            size_t best = 0;
            for (size_t i = 1; i < w.size(); ++i)
                if (w[i] > w[best]) best = i;
            return clips[best].name + " " +
                   std::to_string(int(w[best] * 100.0f + 0.5f)) + "%";
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
