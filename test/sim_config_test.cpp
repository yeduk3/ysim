// RunConfig (sim_config.hpp) unit tests — backend-independent, no GPU.
// Shares the ysim_tests binary's doctest main (scene_io_test.cpp owns it), so
// this TU includes doctest.h WITHOUT the IMPLEMENT_WITH_MAIN macro.

#include "doctest.h"

#include "sim_config.hpp"

using namespace sim_config;

namespace {

// Minimal valid scene JSON (scene_format requires format_version==1 + an
// objects array; everything else defaults).
const char* kMinimalScene = R"({
  "format_version": 1,
  "objects": []
})";

}  // namespace

TEST_CASE("RunConfig: defaults when engine/profile blocks absent (req 1)") {
    auto r = parseString(kMinimalScene);
    REQUIRE(r.ok);
    CHECK(r.value.engine.backend == "METAL");
    CHECK(r.value.engine.system == "Explicit");
    CHECK(r.value.profile.enabled == false);
    CHECK(r.value.profile.frames == 30);
    CHECK(r.value.profile.realtimeSync == false);
    CHECK(r.value.profile.outputPath.empty());
}

TEST_CASE("RunConfig: a plain scene_format scene loads as a RunConfig (back-compat)") {
    // A snapshot serialized by the OLD scene_format (no engine/profile keys)
    // must still parse as a RunConfig with engine/profile defaults.
    scene_format::SceneSnapshot snap;  // empty, version-1
    std::string text = scene_format::toString(snap);
    auto r = parseString(text);
    REQUIRE(r.ok);
    CHECK(r.value.engine.backend == "METAL");
    CHECK(r.value.profile.enabled == false);
}

TEST_CASE("RunConfig: engine + profile round-trip through JSON") {
    auto base = parseString(kMinimalScene);
    REQUIRE(base.ok);
    RunConfig c = base.value;
    c.engine.backend = "METAL";
    c.engine.system = "Explicit";
    c.profile.enabled = true;
    c.profile.frames = 45;
    c.profile.realtimeSync = true;
    c.profile.outputPath = "profiles/foo-45f.csv";

    std::string text = toString(c);
    auto r = parseString(text);
    REQUIRE(r.ok);
    CHECK(r.value.engine.backend == c.engine.backend);
    CHECK(r.value.engine.system == c.engine.system);
    CHECK(r.value.profile.enabled == true);
    CHECK(r.value.profile.frames == 45);
    CHECK(r.value.profile.realtimeSync == true);
    CHECK(r.value.profile.outputPath == "profiles/foo-45f.csv");
}

TEST_CASE("RunConfig: rejects unknown backend / system") {
    auto bad_backend = parseString(R"({
      "format_version": 1, "objects": [],
      "engine": { "backend": "VULKAN" }
    })");
    CHECK_FALSE(bad_backend.ok);

    auto bad_system = parseString(R"({
      "format_version": 1, "objects": [],
      "engine": { "system": "Implicit" }
    })");
    CHECK_FALSE(bad_system.ok);
}

TEST_CASE("RunConfig: rejects malformed profile") {
    auto bad_frames = parseString(R"({
      "format_version": 1, "objects": [],
      "profile": { "frames": 0 }
    })");
    CHECK_FALSE(bad_frames.ok);

    auto bad_enabled = parseString(R"({
      "format_version": 1, "objects": [],
      "profile": { "enabled": "yes" }
    })");
    CHECK_FALSE(bad_enabled.ok);

    auto bad_block = parseString(R"({
      "format_version": 1, "objects": [],
      "engine": "METAL"
    })");
    CHECK_FALSE(bad_block.ok);
}

TEST_CASE("RunConfig: scene parse errors propagate (bad behavior)") {
    auto r = parseString(R"({
      "format_version": 1,
      "objects": [{
        "id": 0, "name": "x",
        "source": { "type": "primitive", "shape": "grid", "size": 1.0, "tessellation": 8 },
        "transform": { "position": [0,0,0], "rotation": [1,0,0,0] },
        "material": { "base_color": [1,1,1], "metallic": 0, "roughness": 0.5,
                      "specular_weight": 1, "emission_color": [0,0,0] },
        "behavior": { "type": "Wobbly", "params": {} }
      }]
    })");
    CHECK_FALSE(r.ok);
}

TEST_CASE("sim_config path helpers") {
    CHECK(pathStem("/a/b/flag.json") == "flag");
    CHECK(pathStem("scene.ysim.json") == "scene.ysim");
    CHECK(pathStem("noext") == "noext");

    CHECK(defaultProfilePath("", "cloth_drape", 20) == "profiles/cloth_drape-20f.csv");
    CHECK(defaultProfilePath("/root", "s", 30) == "/root/profiles/s-30f.csv");
    CHECK(defaultProfilePath("", "", 5) == "profiles/scene-5f.csv");

    CHECK(sidecarScenePath("profiles/cloth_drape-20f.csv")
          == "profiles/cloth_drape-20f.scene.json");
    CHECK(sidecarScenePath("/x/y/out.csv") == "/x/y/out.scene.json");
}
