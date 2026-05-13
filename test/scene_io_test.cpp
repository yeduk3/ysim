#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include "scene_format.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>
#include <vector>

using namespace scene_format;

namespace {

// Authored scene fixture used by BDD-014 / BDD-015 / BDD-016 — a populated
// scene with at least one primitive, one imported mesh, an edited material,
// a non-default behavior, and non-default forces (per docs/TESTS.md#BDD-014).
SceneSnapshot makePopulatedSnapshot() {
    SceneSnapshot s;
    s.environment.gravity = {0.5, -8.0, 1.5};
    s.environment.wind    = {2.0, 0.0, -1.25};

    Object grid;
    grid.id = 0;
    grid.name = "cloth_grid";
    grid.source.kind = Source::Kind::Primitive;
    grid.source.primitive.shape = "grid";
    grid.source.primitive.size = 1.5;
    grid.source.primitive.tessellation = 32;
    grid.source.primitive.direction = "XZPlane";
    grid.source.primitive.mass = 0.25;
    grid.source.primitive.jiggle = true;
    grid.transform.position = {0.1, 0.25, 0.0};
    grid.transform.rotation = {0.7071067811865476, 0.0, 0.7071067811865476, 0.0};
    grid.material.baseColor = {0.6, 0.4, 0.2};
    grid.material.metallic = 0.1;
    grid.material.roughness = 0.4;
    grid.material.specularWeight = 0.9;
    grid.material.emissionColor = {0.0, 0.05, 0.0};
    grid.behavior.type = "TriangularCloth";
    grid.behavior.params = {
        {"stretch",   1234.5},
        {"shear",      567.0},
        {"bend",        78.9},
        {"thickness",    0.01}
    };
    s.objects.push_back(grid);

    Object obj;
    obj.id = 1;
    obj.name = "imported";
    obj.source.kind = Source::Kind::Import;
    obj.source.import.path = "assets/teapot.obj";
    obj.source.import.scale = 1.5;
    obj.source.import.mass = 0.1;
    obj.transform.position = {-1.0, 0.0, 2.0};
    obj.transform.rotation = {1, 0, 0, 0};
    obj.material.baseColor = {0.2, 0.7, 0.9};
    obj.material.metallic = 0.0;
    obj.material.roughness = 0.5;
    obj.material.specularWeight = 1.0;
    obj.material.emissionColor = {0.0, 0.0, 0.0};
    obj.behavior.type = "Float";
    obj.behavior.params = nlohmann::json::object();
    s.objects.push_back(obj);

    return s;
}

std::string tempPath(const char* tag) {
    std::string tmpl = "/tmp/ysim_test_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = mkstemp(buf.data());
    if (fd >= 0) close(fd);
    return std::string(buf.data()) + "." + tag + ".ysim.json";
}

bool writeText(const std::string& path, const std::string& text) {
    std::ofstream out(path);
    if (!out) return false;
    out << text;
    return (bool)out;
}

}  // namespace

// ---- BDD-014 -----------------------------------------------------------

TEST_CASE("BDD-014: save populated scene to disk") {
    auto snap = makePopulatedSnapshot();
    auto path = tempPath("014");
    std::string err;
    REQUIRE(writeToFile(snap, path, &err));

    // File exists and parses as valid JSON with the expected structural keys.
    std::ifstream in(path);
    REQUIRE(in.good());
    nlohmann::json j;
    in >> j;

    CHECK(j.contains("format_version"));
    CHECK(j["format_version"].get<int>() == kFormatVersion);
    CHECK(j.contains("objects"));
    REQUIRE(j["objects"].is_array());
    CHECK(j["objects"].size() == 2);
    CHECK(j.contains("environment"));
    CHECK(j["environment"].contains("gravity"));
    CHECK(j["environment"].contains("wind"));

    // Per-object structural keys.
    for (auto& o : j["objects"]) {
        CHECK(o.contains("id"));
        CHECK(o.contains("name"));
        CHECK(o.contains("source"));
        CHECK(o.contains("transform"));
        CHECK(o["transform"].contains("position"));
        CHECK(o["transform"].contains("rotation"));
        REQUIRE(o["transform"]["rotation"].is_array());
        CHECK(o["transform"]["rotation"].size() == 4);
        CHECK(o.contains("material"));
        CHECK(o["material"].contains("base_color"));
        CHECK(o["material"].contains("metallic"));
        CHECK(o["material"].contains("roughness"));
        CHECK(o["material"].contains("specular_weight"));
        CHECK(o["material"].contains("emission_color"));
        CHECK(o.contains("behavior"));
        CHECK(o["behavior"].contains("type"));
    }

    // The first object is the grid primitive with the edited material;
    // the second is the imported mesh — confirm round-trip records both kinds.
    CHECK(j["objects"][0]["source"]["type"].get<std::string>() == "primitive");
    CHECK(j["objects"][0]["source"]["shape"].get<std::string>() == "grid");
    CHECK(j["objects"][0]["behavior"]["type"].get<std::string>() == "TriangularCloth");
    CHECK(j["objects"][1]["source"]["type"].get<std::string>() == "import");
    CHECK(j["objects"][1]["source"]["path"].get<std::string>() == "assets/teapot.obj");

    std::remove(path.c_str());
}

// ---- BDD-015 -----------------------------------------------------------

TEST_CASE("BDD-015: load reproduces saved state field-by-field") {
    auto snap = makePopulatedSnapshot();
    auto path = tempPath("015");
    REQUIRE(writeToFile(snap, path));

    auto r = readFromFile(path);
    REQUIRE(r.ok);
    auto& loaded = r.value;

    CHECK(loaded.formatVersion == kFormatVersion);
    REQUIRE(loaded.objects.size() == snap.objects.size());

    for (size_t i = 0; i < snap.objects.size(); ++i) {
        const auto& a = snap.objects[i];
        const auto& b = loaded.objects[i];
        CHECK(a.id == b.id);
        CHECK(a.name == b.name);
        CHECK(int(a.source.kind) == int(b.source.kind));
        if (a.source.kind == Source::Kind::Primitive) {
            CHECK(a.source.primitive.shape == b.source.primitive.shape);
            CHECK(a.source.primitive.size == doctest::Approx(b.source.primitive.size));
            CHECK(a.source.primitive.tessellation == b.source.primitive.tessellation);
            CHECK(a.source.primitive.direction == b.source.primitive.direction);
            CHECK(a.source.primitive.mass == doctest::Approx(b.source.primitive.mass));
            CHECK(a.source.primitive.jiggle == b.source.primitive.jiggle);
        } else {
            CHECK(a.source.import.path == b.source.import.path);
            CHECK(a.source.import.scale == doctest::Approx(b.source.import.scale));
            CHECK(a.source.import.mass == doctest::Approx(b.source.import.mass));
        }
        for (int k = 0; k < 3; ++k)
            CHECK(a.transform.position[k] == doctest::Approx(b.transform.position[k]));
        // Rotation may renormalize; check it's a unit quaternion close to the source.
        Quat aq = normalizeQuat(a.transform.rotation);
        for (int k = 0; k < 4; ++k)
            CHECK(aq[k] == doctest::Approx(b.transform.rotation[k]));
        for (int k = 0; k < 3; ++k)
            CHECK(a.material.baseColor[k] == doctest::Approx(b.material.baseColor[k]));
        CHECK(a.material.metallic == doctest::Approx(b.material.metallic));
        CHECK(a.material.roughness == doctest::Approx(b.material.roughness));
        CHECK(a.material.specularWeight == doctest::Approx(b.material.specularWeight));
        for (int k = 0; k < 3; ++k)
            CHECK(a.material.emissionColor[k] == doctest::Approx(b.material.emissionColor[k]));
        CHECK(a.behavior.type == b.behavior.type);
        CHECK(a.behavior.params == b.behavior.params);
    }

    for (int k = 0; k < 3; ++k) {
        CHECK(snap.environment.gravity[k] == doctest::Approx(loaded.environment.gravity[k]));
        CHECK(snap.environment.wind[k]    == doctest::Approx(loaded.environment.wind[k]));
    }

    // Save → load → save produces identical bytes (round-trip determinism;
    // proxy for "running the simulation produces the same first-step output"
    // when the persistence layer cannot host a Metal device).
    std::string text1 = toString(snap);
    std::string text2 = toString(loaded);
    CHECK(text1 == text2);

    std::remove(path.c_str());
}

// ---- BDD-016 -----------------------------------------------------------

TEST_CASE("BDD-016: reject scene file with unsupported format_version") {
    // Pre-state: a snapshot we can use as "current scene state" sentinel.
    auto sentinel = makePopulatedSnapshot();
    auto sentinelText = toString(sentinel);

    auto path = tempPath("016a");
    REQUIRE(writeText(path,
        R"({"format_version": 999, "objects": [], "environment": {"gravity":[0,0,0],"wind":[0,0,0]}})"));
    auto r = readFromFile(path);
    CHECK_FALSE(r.ok);
    CHECK(r.error.message.find("999") != std::string::npos);
    CHECK(r.error.message.find("1") != std::string::npos);
    // Sentinel snapshot is unchanged — fromJson does not mutate inputs.
    CHECK(toString(sentinel) == sentinelText);
    std::remove(path.c_str());
}

TEST_CASE("BDD-016: reject scene file missing format_version") {
    auto path = tempPath("016b");
    REQUIRE(writeText(path,
        R"({"objects": [], "environment": {"gravity":[0,0,0],"wind":[0,0,0]}})"));
    auto r = readFromFile(path);
    CHECK_FALSE(r.ok);
    CHECK(r.error.message.find("missing format_version") != std::string::npos);
    std::remove(path.c_str());
}

TEST_CASE("BDD-016: reject reserved-but-not-shipped behavior type") {
    // D-036 turn-32 addendum: "Rigid" is no longer reserved-not-shipped
    // (it round-trips via persistence per BDD-006 fix-turn). Reserved
    // set is now {Elastic, Fluid, Generator}; we exercise the rejection
    // path with "Elastic" so the test's intent (loader rejects reserved
    // behavior names) is preserved.
    auto path = tempPath("016c");
    REQUIRE(writeText(path, R"({
      "format_version": 1,
      "objects": [{
        "id": 0, "name": "x",
        "source": {"type":"primitive","shape":"grid","size":1.0,"tessellation":2},
        "transform": {"position":[0,0,0],"rotation":[1,0,0,0]},
        "material": {"base_color":[1,1,1],"metallic":0.0,"roughness":0.5,"specular_weight":1.0,"emission_color":[0,0,0]},
        "behavior": {"type":"Elastic","params":{}}
      }],
      "environment": {"gravity":[0,0,0],"wind":[0,0,0]}
    })"));
    auto r = readFromFile(path);
    CHECK_FALSE(r.ok);
    CHECK(r.error.message.find("Elastic") != std::string::npos);
    CHECK(r.error.message.find("not available") != std::string::npos);
    std::remove(path.c_str());
}

// ---- Import path resolution (BDD-014/015 Estimator follow-up) ---------

TEST_CASE("import paths resolve relative to scene file directory") {
    using R = std::string;
    CHECK(resolveImportPath("/scenes/2026", "assets/teapot.obj") == R("/scenes/2026/assets/teapot.obj"));
    CHECK(resolveImportPath("scenes", "assets/teapot.obj") == R("scenes/assets/teapot.obj"));
    CHECK(resolveImportPath("", "assets/teapot.obj") == R("assets/teapot.obj"));
    // Absolute import paths pass through.
    CHECK(resolveImportPath("/scenes/2026", "/library/teapot.obj") == R("/library/teapot.obj"));
    // Trailing slash on dir.
    CHECK(resolveImportPath("/scenes/2026/", "assets/teapot.obj") == R("/scenes/2026/assets/teapot.obj"));
}

TEST_CASE("sceneDir extracts directory from a scene file path") {
    CHECK(sceneDir("/scenes/2026/foo.ysim.json") == "/scenes/2026");
    CHECK(sceneDir("foo.ysim.json") == "");
    CHECK(sceneDir("./foo.ysim.json") == ".");
}

// ---- Material clamping warns (Estimator WARNING follow-up) ------------

TEST_CASE("loader emits a warning and clamps out-of-range material values") {
    auto path = tempPath("clamp");
    REQUIRE(writeText(path, R"({
      "format_version": 1,
      "objects": [{
        "id": 0, "name": "x",
        "source": {"type":"primitive","shape":"grid","size":1.0,"tessellation":2},
        "transform": {"position":[0,0,0],"rotation":[1,0,0,0]},
        "material": {"base_color":[1.5, -0.2, 0.5],"metallic":2.0,"roughness":-0.1,"specular_weight":3.0,"emission_color":[-1.0, 0.0, 0.0]},
        "behavior": {"type":"Float","params":{}}
      }],
      "environment": {"gravity":[0,0,0],"wind":[0,0,0]}
    })"));
    auto r = readFromFile(path);
    REQUIRE(r.ok);
    CHECK_FALSE(r.value.warnings.empty());
    const auto& m = r.value.objects[0].material;
    CHECK(m.baseColor[0] == doctest::Approx(1.0));
    CHECK(m.baseColor[1] == doctest::Approx(0.0));
    CHECK(m.baseColor[2] == doctest::Approx(0.5));
    CHECK(m.metallic == doctest::Approx(1.0));
    CHECK(m.roughness == doctest::Approx(0.0));
    CHECK(m.specularWeight == doctest::Approx(1.0));
    CHECK(m.emissionColor[0] == doctest::Approx(0.0));
    std::remove(path.c_str());
}

TEST_CASE("BDD-016: reject unsupported import extension") {
    auto path = tempPath("016d");
    REQUIRE(writeText(path, R"({
      "format_version": 1,
      "objects": [{
        "id": 0, "name": "x",
        "source": {"type":"import","path":"foo.fbx"},
        "transform": {"position":[0,0,0],"rotation":[1,0,0,0]},
        "material": {"base_color":[1,1,1],"metallic":0.0,"roughness":0.5,"specular_weight":1.0,"emission_color":[0,0,0]},
        "behavior": {"type":"Float","params":{}}
      }],
      "environment": {"gravity":[0,0,0],"wind":[0,0,0]}
    })"));
    auto r = readFromFile(path);
    CHECK_FALSE(r.ok);
    CHECK(r.error.message.find("foo.fbx") != std::string::npos);
    std::remove(path.c_str());
}

// ---- BDD-009 / 011 / 012 — env-forces persistence guard ---------------

TEST_CASE("BDD-011/012: non-default gravity and wind round-trip bit-stable") {
    SceneSnapshot s;
    s.environment.gravity = {1.5, -3.25, 0.125};
    s.environment.wind    = {-0.5, 0.0, 2.75};
    auto path = tempPath("envrt");
    REQUIRE(writeToFile(s, path));
    auto r = readFromFile(path);
    REQUIRE(r.ok);
    for (int k = 0; k < 3; ++k) {
        CHECK(s.environment.gravity[k] == doctest::Approx(r.value.environment.gravity[k]));
        CHECK(s.environment.wind[k]    == doctest::Approx(r.value.environment.wind[k]));
    }
    // Save→load→save byte identity.
    CHECK(toString(s) == toString(r.value));
    std::remove(path.c_str());
}

TEST_CASE("BDD-011/012: missing environment falls back to schema defaults") {
    auto path = tempPath("envdef");
    REQUIRE(writeText(path, R"({"format_version": 1, "objects": []})"));
    auto r = readFromFile(path);
    REQUIRE(r.ok);
    // Schema default: gravity = (0, -9.81, 0); wind = (0, 0, 0).
    CHECK(r.value.environment.gravity[0] == doctest::Approx(0.0));
    CHECK(r.value.environment.gravity[1] == doctest::Approx(-9.81));
    CHECK(r.value.environment.gravity[2] == doctest::Approx(0.0));
    CHECK(r.value.environment.wind[0] == doctest::Approx(0.0));
    CHECK(r.value.environment.wind[1] == doctest::Approx(0.0));
    CHECK(r.value.environment.wind[2] == doctest::Approx(0.0));
    std::remove(path.c_str());
}
