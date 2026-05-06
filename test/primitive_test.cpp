#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "primitive_geometry.hpp"
#include "scene_format.hpp"

#include <array>
#include <cmath>
#include <set>
#include <utility>

using namespace primitive;

namespace {

double distFromCenter(const Geometry& g, Index v, std::array<float, 3> c) {
    float dx = g.positions[3 * v + 0] - c[0];
    float dy = g.positions[3 * v + 1] - c[1];
    float dz = g.positions[3 * v + 2] - c[2];
    return std::sqrt((double)(dx * dx + dy * dy + dz * dz));
}

int countUniqueEdges(const Geometry& g) {
    std::set<std::pair<Index, Index>> edges;
    for (size_t i = 0; i < g.facets.size(); i += 3) {
        Index a = g.facets[i];
        Index b = g.facets[i + 1];
        Index c = g.facets[i + 2];
        auto add = [&](Index x, Index y) {
            if (x > y) std::swap(x, y);
            edges.insert({x, y});
        };
        add(a, b);
        add(b, c);
        add(c, a);
    }
    return (int)edges.size();
}

}  // namespace

// ---- BDD-001 sphere --------------------------------------------------------

TEST_CASE("BDD-001: sphere primitive — vertex/facet counts match closed-form") {
    for (int tess : {3, 4, 8, 16}) {
        CAPTURE(tess);
        auto g = sphere(2.0f, tess, {0.f, 0.f, 0.f});
        CHECK((int)(g.positions.size() / 3) == sphereVertexCount(tess));
        CHECK((int)(g.facets.size() / 3) == sphereFacetCount(tess));
        CHECK(countUniqueEdges(g) == sphereEdgeCount(tess));
    }
}

TEST_CASE("BDD-001: sphere primitive — every vertex lies on the sphere surface") {
    const float size = 1.5f;
    const std::array<float, 3> center{0.25f, -0.5f, 1.0f};
    auto g = sphere(size, 12, center);
    int n = (int)(g.positions.size() / 3);
    REQUIRE(n > 0);
    const double r = (double)size * 0.5;
    for (int v = 0; v < n; ++v) {
        CAPTURE(v);
        CHECK(distFromCenter(g, v, center) == doctest::Approx(r).epsilon(1e-4));
    }
}

TEST_CASE("BDD-001: sphere primitive — facets reference no out-of-bounds indices") {
    auto g = sphere(1.0f, 8, {0.f, 0.f, 0.f});
    Index nv = (Index)(g.positions.size() / 3);
    for (Index f : g.facets) CHECK(f < nv);
}

TEST_CASE("BDD-001: sphere primitive — no degenerate triangle (zero area)") {
    auto g = sphere(1.0f, 8, {0.f, 0.f, 0.f});
    for (size_t i = 0; i < g.facets.size(); i += 3) {
        Index a = g.facets[i], b = g.facets[i + 1], c = g.facets[i + 2];
        CHECK(a != b);
        CHECK(b != c);
        CHECK(a != c);
        // Cross-product magnitude > 0: catches collinear-vertex bugs.
        float ax = g.positions[3*a], ay = g.positions[3*a+1], az = g.positions[3*a+2];
        float bx = g.positions[3*b], by = g.positions[3*b+1], bz = g.positions[3*b+2];
        float cx = g.positions[3*c], cy = g.positions[3*c+1], cz = g.positions[3*c+2];
        float ux = bx - ax, uy = by - ay, uz = bz - az;
        float vx = cx - ax, vy = cy - ay, vz = cz - az;
        float nx = uy*vz - uz*vy;
        float ny = uz*vx - ux*vz;
        float nz = ux*vy - uy*vx;
        float a2 = nx*nx + ny*ny + nz*nz;
        CHECK(a2 > 1e-10f);
    }
}

// ---- BDD-001 cube ----------------------------------------------------------

TEST_CASE("BDD-001: cube primitive — vertex/facet counts match closed-form") {
    for (int tess : {1, 2, 4, 8}) {
        CAPTURE(tess);
        auto g = cube(2.0f, tess, {0.f, 0.f, 0.f});
        CHECK((int)(g.positions.size() / 3) == cubeVertexCount(tess));
        CHECK((int)(g.facets.size() / 3) == cubeFacetCount(tess));
        CHECK(countUniqueEdges(g) == cubeEdgeCount(tess));
    }
}

TEST_CASE("BDD-001: cube primitive — every vertex sits on a face plane") {
    const float size = 1.0f;
    const std::array<float, 3> center{0.f, 0.f, 0.f};
    auto g = cube(size, 3, center);
    const float h = size * 0.5f;
    int n = (int)(g.positions.size() / 3);
    REQUIRE(n > 0);
    for (int v = 0; v < n; ++v) {
        float x = g.positions[3*v + 0] - center[0];
        float y = g.positions[3*v + 1] - center[1];
        float z = g.positions[3*v + 2] - center[2];
        // At least one coordinate must be at ±h within tolerance.
        bool onFace = (std::fabs(std::fabs(x) - h) < 1e-4f) ||
                      (std::fabs(std::fabs(y) - h) < 1e-4f) ||
                      (std::fabs(std::fabs(z) - h) < 1e-4f);
        CAPTURE(v); CAPTURE(x); CAPTURE(y); CAPTURE(z);
        CHECK(onFace);
    }
}

TEST_CASE("BDD-001: cube primitive — center translation moves every vertex") {
    auto a = cube(1.0f, 1, {0.f, 0.f, 0.f});
    auto b = cube(1.0f, 1, {2.0f, 3.0f, 4.0f});
    REQUIRE(a.positions.size() == b.positions.size());
    for (size_t i = 0; i < a.positions.size(); i += 3) {
        CHECK(b.positions[i + 0] == doctest::Approx(a.positions[i + 0] + 2.0f));
        CHECK(b.positions[i + 1] == doctest::Approx(a.positions[i + 1] + 3.0f));
        CHECK(b.positions[i + 2] == doctest::Approx(a.positions[i + 2] + 4.0f));
    }
}

// ---- BDD-001 schema integration -------------------------------------------

TEST_CASE("BDD-001: scene_format accepts sphere and cube as primitive shapes") {
    using namespace scene_format;
    SceneSnapshot s;
    Object o;
    o.id = 0;
    o.name = "ball";
    o.source.kind = Source::Kind::Primitive;
    o.source.primitive.shape = "sphere";
    o.source.primitive.size = 2.0;
    o.source.primitive.tessellation = 16;
    o.transform.position = {0, 1, 0};
    s.objects.push_back(o);

    auto j = toJson(s);
    auto r = fromJson(j);
    REQUIRE(r.ok);
    REQUIRE(r.value.objects.size() == 1);
    CHECK(r.value.objects[0].source.primitive.shape == "sphere");

    s.objects[0].source.primitive.shape = "cube";
    auto j2 = toJson(s);
    auto r2 = fromJson(j2);
    REQUIRE(r2.ok);
    CHECK(r2.value.objects[0].source.primitive.shape == "cube");
}

TEST_CASE("BDD-001: scene_format still rejects unknown primitive shapes") {
    nlohmann::json j = {
        {"format_version", 1},
        {"objects", nlohmann::json::array({{
            {"id", 0}, {"name", "x"},
            {"source", {{"type", "primitive"}, {"shape", "torus"},
                         {"size", 1.0}, {"tessellation", 8}}},
            {"transform", {{"position", {0,0,0}}, {"rotation", {1,0,0,0}}}},
            {"material", {{"base_color", {1,1,1}}, {"metallic", 0.0},
                           {"roughness", 0.5}, {"specular_weight", 1.0},
                           {"emission_color", {0,0,0}}}},
            {"behavior", {{"type", "Float"}, {"params", nlohmann::json::object()}}}
        }})},
        {"environment", {{"gravity", {0,0,0}}, {"wind", {0,0,0}}}}
    };
    auto r = scene_format::fromJson(j);
    CHECK_FALSE(r.ok);
    CHECK(r.error.message.find("torus") != std::string::npos);
}
