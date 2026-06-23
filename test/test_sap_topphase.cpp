// Standalone check for the SAP top-phase candidate-pair logic
// (BVH::queryPointsSAP in src/main.cpp). Replicates the slice math exactly
// and asserts it produces the SAME (point, group) pairs as a brute-force
// O(N*k) box-overlap scan, over randomized boxes.
//
//   c++ -std=c++17 -O2 test/test_sap_topphase.cpp -o /tmp/test_sap && /tmp/test_sap
//
// Fails loudly (assert / nonzero exit) if the binary-search bounds or the
// Y/Z confirm ever diverge from ground truth.

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <set>
#include <vector>

struct Box { float lo[3], hi[3]; };  // group box

// Mirrors queryPointsSAP: point box = [x-m, x+m]^3 (constant width 2m).
static std::set<std::pair<uint32_t, uint32_t>>
sapPairs(const std::vector<float>& xp, int n, const std::vector<Box>& g, float m) {
    std::vector<uint32_t> order(n);
    for (int i = 0; i < n; ++i) order[i] = (uint32_t)i;
    std::sort(order.begin(), order.end(),
              [&](uint32_t a, uint32_t b) { return xp[3*a] < xp[3*b]; });
    auto xkey = [&](uint32_t id) { return xp[3*id]; };

    std::set<std::pair<uint32_t, uint32_t>> out;
    for (uint32_t gi = 0; gi < g.size(); ++gi) {
        const Box& b = g[gi];
        auto lo = std::lower_bound(order.begin(), order.end(), b.lo[0] - m,
                      [&](uint32_t id, float v) { return xkey(id) < v; });
        auto hi = std::upper_bound(order.begin(), order.end(), b.hi[0] + m,
                      [&](float v, uint32_t id) { return v < xkey(id); });
        for (auto it = lo; it != hi; ++it) {
            uint32_t id = *it;
            float y = xp[3*id+1], z = xp[3*id+2];
            if (y + m < b.lo[1] || y - m > b.hi[1]) continue;
            if (z + m < b.lo[2] || z - m > b.hi[2]) continue;
            out.insert({id, gi});
        }
    }
    return out;
}

// Ground truth: every (point, group) box overlap, all three axes.
static std::set<std::pair<uint32_t, uint32_t>>
brute(const std::vector<float>& xp, int n, const std::vector<Box>& g, float m) {
    std::set<std::pair<uint32_t, uint32_t>> out;
    for (int i = 0; i < n; ++i)
        for (uint32_t gi = 0; gi < g.size(); ++gi) {
            const Box& b = g[gi];
            bool ov = true;
            for (int a = 0; a < 3; ++a)
                if (xp[3*i+a] + m < b.lo[a] || xp[3*i+a] - m > b.hi[a]) { ov = false; break; }
            if (ov) out.insert({(uint32_t)i, gi});
        }
    return out;
}

int main() {
    uint32_t s = 12345;
    auto rnd = [&]() { s = s*1664525u + 1013904223u; return (s >> 8) / float(1u<<24); };

    for (int trial = 0; trial < 200; ++trial) {
        int n = 1 + (int)(rnd() * 400);
        int k = 1 + (int)(rnd() * 12);
        float m = 0.001f + rnd() * 0.2f;       // margin (incl. zero-ish and fat)
        std::vector<float> xp(3 * n);
        for (auto& v : xp) v = rnd() * 10.0f - 5.0f;
        std::vector<Box> g(k);
        for (auto& b : g)
            for (int a = 0; a < 3; ++a) {
                float c0 = rnd()*10-5, c1 = rnd()*10-5;
                b.lo[a] = std::min(c0,c1); b.hi[a] = std::max(c0,c1);
            }
        assert(sapPairs(xp, n, g, m) == brute(xp, n, g, m));
    }
    printf("test_sap_topphase: OK (200 trials)\n");
    return 0;
}
