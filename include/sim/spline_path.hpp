#pragma once
// Catmull-Rom spline sampling for the spline-follow dynamic constraint
// (point panel "경로 따라가기"). Standalone header — unlike the sim
// fragment headers this only needs tinym + the STL, so the scene_format
// tests and Scene::pack can both use it.
//
// Two parameterizations:
//   sampleUniform  — plain uniform-parameter Catmull-Rom; interpolates
//                    the control points exactly at u = i/segs. Used by
//                    the self-tests and as the primitive under the
//                    arc-length map.
//   sampleArcLength — near-constant-speed sample via a dense polyline
//                    lookup table, so playback speed does not bunch up
//                    where control points cluster. Used by the runtime
//                    constraint and the viz.

#include <cmath>
#include <vector>

#include "../tinym.hpp"

namespace spline_path {

// Catmull-Rom position at u ∈ [0,1] over the whole path. closed wraps
// the control polygon (segment count == pts.size()); open clamps the
// end tangents by duplicating endpoints (segment count == size()-1).
// Fewer than 2 points degenerates to the sole point / origin.
inline tinym::vec3 sampleUniform(const std::vector<tinym::vec3>& pts,
                                 bool closed, float u) {
    const int n = (int)pts.size();
    if (n == 0) return tinym::vec3(0.0f, 0.0f, 0.0f);
    if (n == 1) return pts[0];
    const int segs = closed ? n : n - 1;
    float t = u * (float)segs;
    if (t < 0.0f) t = 0.0f;
    if (t > (float)segs) t = (float)segs;
    int si = (int)t;
    if (si >= segs) si = segs - 1;
    const float lt = t - (float)si;
    auto P = [&](int i) -> const tinym::vec3& {
        if (closed) return pts[((i % n) + n) % n];
        return pts[i < 0 ? 0 : (i > n - 1 ? n - 1 : i)];
    };
    const tinym::vec3 &p0 = P(si - 1), &p1 = P(si), &p2 = P(si + 1),
                      &p3 = P(si + 2);
    const float t2 = lt * lt, t3 = t2 * lt;
    return (p1 * 2.0f + (p2 - p0) * lt
            + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2
            + ((p1 - p2) * 3.0f + p3 - p0) * t3) * 0.5f;
}

// Dense polyline over the whole path (samplesPerSeg subdivisions per
// control segment, endpoint included — for a closed path the last
// sample equals the first). Shared by the viz and the arc-length map.
inline std::vector<tinym::vec3> samplePolyline(
    const std::vector<tinym::vec3>& pts, bool closed, int samplesPerSeg) {
    std::vector<tinym::vec3> out;
    const int n = (int)pts.size();
    if (n < 2) { if (n == 1) out.push_back(pts[0]); return out; }
    const int segs = closed ? n : n - 1;
    const int N = segs * (samplesPerSeg < 1 ? 1 : samplesPerSeg);
    out.reserve(N + 1);
    for (int i = 0; i <= N; ++i)
        out.push_back(sampleUniform(pts, closed, (float)i / (float)N));
    return out;
}

// Near-constant-speed sample: s01 ∈ [0,1] is the fraction of total arc
// length traveled. ponytail: rebuilds the lookup table per call — the
// constraint runs once per frame over <100 samples, cache when a
// profiler says so.
inline tinym::vec3 sampleArcLength(const std::vector<tinym::vec3>& pts,
                                   bool closed, float s01) {
    const auto poly = samplePolyline(pts, closed, 16);
    if (poly.empty()) return tinym::vec3(0.0f, 0.0f, 0.0f);
    if (poly.size() == 1) return poly[0];
    std::vector<float> cum(poly.size(), 0.0f);
    for (size_t i = 1; i < poly.size(); ++i)
        cum[i] = cum[i - 1] + (poly[i] - poly[i - 1]).norm();
    const float total = cum.back();
    if (!(total > 0.0f)) return poly[0];
    float s = s01;
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    const float target = s * total;
    size_t lo = 0, hi = poly.size() - 1;
    while (lo + 1 < hi) {
        const size_t mid = (lo + hi) / 2;
        if (cum[mid] <= target) lo = mid; else hi = mid;
    }
    const float span = cum[hi] - cum[lo];
    const float f = span > 0.0f ? (target - cum[lo]) / span : 0.0f;
    return poly[lo] + (poly[hi] - poly[lo]) * f;
}

} // namespace spline_path
