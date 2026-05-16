#ifndef YSIM_SCENE_FORMAT_HPP
#define YSIM_SCENE_FORMAT_HPP

// Plain POD scene snapshot + JSON encode/decode. No dependency on Metal,
// OpenGL, GLFW, or tinym — the persistence layer is backend-independent
// (docs/ARCHITECTURE.md §4.1) and these types are reused from the test
// harness (test/scene_io_test.cpp), which has no GPU device.

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace scene_format {

constexpr int kFormatVersion = 1;

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // [w, x, y, z], renormalized on read

// "grid" is the only primitive shape v1 ships (D-003). "sphere"/"cube" are
// recognized as reserved names and rejected on load — see schema doc.
struct PrimitiveSource {
    std::string shape = "grid";
    double size = 1.0;
    int tessellation = 32;
    // grid-specific (carried even for non-grid so the encoder is uniform):
    std::string direction = "XZPlane";  // XYPlane | YZPlane | XZPlane
    double mass = 0.1;
    bool jiggle = false;
};

struct ImportSource {
    std::string path;
    double scale = 1.0;
    double mass = 0.1;
};

struct Source {
    enum class Kind { Primitive, Import };
    Kind kind = Kind::Primitive;
    PrimitiveSource primitive{};
    ImportSource import{};
};

struct Transform {
    Vec3 position{0, 0, 0};
    Quat rotation{1, 0, 0, 0};
};

struct Material {
    Vec3 baseColor{1, 1, 1};
    double metallic = 0.0;
    double roughness = 0.5;
    double specularWeight = 1.0;
    Vec3 emissionColor{0, 0, 0};
};

struct Behavior {
    std::string type = "Float";
    nlohmann::json params = nlohmann::json::object();
};

// A pinned vertex: physics-vertex index + the world position it is
// held at. Persisted per object as the "fixed_particles" array.
struct FixedParticle {
    int vid = 0;
    Vec3 pos{0, 0, 0};
};

struct Object {
    int id = 0;
    std::string name;
    Source source;
    Transform transform;
    Material material;
    Behavior behavior;
    // Per-object environment-force gates (UI toggles). Optional in JSON
    // for backward compat — older snapshots default to "receive both".
    bool applyGravity = true;
    bool applyWind = true;
    // "팽팽함" — uniform multiplier on simulated cloth stiffness.
    // Optional in JSON (default 1.0 for older snapshots / non-cloth).
    double clothStiffnessScale = 1.0;
    // Point-selection panel constraints. Optional in JSON — older
    // snapshots simply have no pinned vertices.
    std::vector<FixedParticle> fixedParticles;
};

struct Environment {
    Vec3 gravity{0.0, -9.81, 0.0};
    Vec3 wind{0.0, 0.0, 0.0};
    // UI viewport clear color. Optional in JSON for backward compat with
    // pre-existing scenes (default mirrors SceneEnvironment::backgroundColor).
    Vec3 backgroundColor{0.05, 0.05, 0.08};
};

// A reference-point coincidence constraint set in the point-selection
// panel: the (queryObject, queryVertex) vertex must occupy the same
// world position as the (targetObject, targetVertex) vertex every
// step. Cross-object capable, so this is a scene-level list (not per
// Object). Indices are physics-vertex ids, matching FixedParticle.vid.
struct ReferenceConstraint {
    int queryObject = 0;
    int queryVertex = 0;
    int targetObject = 0;
    int targetVertex = 0;
};

struct LoadError {
    std::string message;
};

// Loader-emitted warnings (e.g. clamped material values). Distinct from
// LoadError — these do not fail the load (`docs/design/scene_format.md`
// says "logs a warning and clamps").
struct LoadWarnings {
    std::vector<std::string> messages;
    bool empty() const { return messages.empty(); }
};

struct SceneSnapshot {
    int formatVersion = kFormatVersion;
    std::vector<Object> objects;
    Environment environment;
    // Reference-point coincidence constraints (point panel). Optional in
    // JSON — older snapshots simply have none.
    std::vector<ReferenceConstraint> referenceConstraints;
    LoadWarnings warnings;
};

template <typename T>
struct Result {
    bool ok = false;
    T value{};
    LoadError error{};
    static Result success(T v) { return {true, std::move(v), {}}; }
    static Result fail(std::string msg) { return {false, T{}, {std::move(msg)}}; }
};

inline Quat normalizeQuat(const Quat& q) {
    double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n == 0.0) return {1, 0, 0, 0};
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

inline Vec3 clampColor(const Vec3& c) {
    return {clamp01(c[0]), clamp01(c[1]), clamp01(c[2])};
}

// Returns true when any field had to be clamped — caller may surface a warning.
inline bool clampInPlace(Material& m) {
    bool changed = false;
    auto clampVec = [&](Vec3& v) {
        for (int i = 0; i < 3; ++i) {
            double c = clamp01(v[i]);
            if (c != v[i]) { v[i] = c; changed = true; }
        }
    };
    auto clampScalar = [&](double& x) {
        double c = clamp01(x);
        if (c != x) { x = c; changed = true; }
    };
    clampVec(m.baseColor);
    clampScalar(m.metallic);
    clampScalar(m.roughness);
    clampScalar(m.specularWeight);
    for (int i = 0; i < 3; ++i) {
        if (m.emissionColor[i] < 0.0) { m.emissionColor[i] = 0.0; changed = true; }
    }
    return changed;
}

namespace detail {

inline nlohmann::json vec3ToJson(const Vec3& v) {
    return nlohmann::json::array({v[0], v[1], v[2]});
}

inline nlohmann::json quatToJson(const Quat& q) {
    return nlohmann::json::array({q[0], q[1], q[2], q[3]});
}

inline bool readVec3(const nlohmann::json& j, Vec3& out, std::string& err,
                     const std::string& path) {
    if (!j.is_array() || j.size() != 3) {
        err = "expected 3-element array at " + path;
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (!j[i].is_number()) {
            err = "expected number at " + path + "[" + std::to_string(i) + "]";
            return false;
        }
        out[i] = j[i].get<double>();
    }
    return true;
}

inline bool readQuat(const nlohmann::json& j, Quat& out, std::string& err,
                     const std::string& path) {
    if (!j.is_array() || j.size() != 4) {
        err = "expected 4-element [w,x,y,z] array at " + path;
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        if (!j[i].is_number()) {
            err = "expected number at " + path + "[" + std::to_string(i) + "]";
            return false;
        }
        out[i] = j[i].get<double>();
    }
    return true;
}

template <typename T>
inline bool readField(const nlohmann::json& j, const char* key, T& out,
                      std::string& err, const std::string& path) {
    auto it = j.find(key);
    if (it == j.end()) {
        err = "missing '" + std::string(key) + "' at " + path;
        return false;
    }
    try {
        out = it->get<T>();
    } catch (const std::exception& e) {
        err = "wrong type for '" + std::string(key) + "' at " + path + ": " + e.what();
        return false;
    }
    return true;
}

}  // namespace detail

inline nlohmann::json toJson(const Source& s) {
    using nlohmann::json;
    json j;
    if (s.kind == Source::Kind::Primitive) {
        j["type"] = "primitive";
        j["shape"] = s.primitive.shape;
        j["size"] = s.primitive.size;
        j["tessellation"] = s.primitive.tessellation;
        j["direction"] = s.primitive.direction;
        j["mass"] = s.primitive.mass;
        j["jiggle"] = s.primitive.jiggle;
    } else {
        j["type"] = "import";
        j["path"] = s.import.path;
        j["scale"] = s.import.scale;
        j["mass"] = s.import.mass;
    }
    return j;
}

inline nlohmann::json toJson(const Transform& t) {
    nlohmann::json j;
    Quat q = normalizeQuat(t.rotation);
    j["position"] = detail::vec3ToJson(t.position);
    j["rotation"] = detail::quatToJson(q);
    return j;
}

inline nlohmann::json toJson(const Material& m) {
    nlohmann::json j;
    j["base_color"] = detail::vec3ToJson(m.baseColor);
    j["metallic"] = m.metallic;
    j["roughness"] = m.roughness;
    j["specular_weight"] = m.specularWeight;
    j["emission_color"] = detail::vec3ToJson(m.emissionColor);
    return j;
}

inline nlohmann::json toJson(const Behavior& b) {
    nlohmann::json j;
    j["type"] = b.type;
    j["params"] = b.params;
    return j;
}

inline nlohmann::json toJson(const Object& o) {
    nlohmann::json j;
    j["id"] = o.id;
    j["name"] = o.name;
    j["source"] = toJson(o.source);
    j["transform"] = toJson(o.transform);
    j["material"] = toJson(o.material);
    j["behavior"] = toJson(o.behavior);
    j["apply_gravity"] = o.applyGravity;
    j["apply_wind"] = o.applyWind;
    j["cloth_stiffness_scale"] = o.clothStiffnessScale;
    if (!o.fixedParticles.empty()) {
        nlohmann::json fp = nlohmann::json::array();
        for (const auto& f : o.fixedParticles) {
            nlohmann::json e;
            e["vid"] = f.vid;
            e["pos"] = detail::vec3ToJson(f.pos);
            fp.push_back(std::move(e));
        }
        j["fixed_particles"] = std::move(fp);
    }
    return j;
}

inline nlohmann::json toJson(const Environment& e) {
    nlohmann::json j;
    j["gravity"] = detail::vec3ToJson(e.gravity);
    j["wind"] = detail::vec3ToJson(e.wind);
    j["background_color"] = detail::vec3ToJson(e.backgroundColor);
    return j;
}

inline nlohmann::json toJson(const ReferenceConstraint& c) {
    nlohmann::json j;
    j["query"]  = {{"object", c.queryObject},  {"vertex", c.queryVertex}};
    j["target"] = {{"object", c.targetObject}, {"vertex", c.targetVertex}};
    return j;
}

inline nlohmann::json toJson(const SceneSnapshot& s) {
    nlohmann::json j;
    j["format_version"] = s.formatVersion;
    j["objects"] = nlohmann::json::array();
    for (const auto& o : s.objects) j["objects"].push_back(toJson(o));
    j["environment"] = toJson(s.environment);
    // Omit the key entirely when empty so constraint-free scenes stay
    // byte-identical to pre-feature snapshots (mirrors fixed_particles).
    if (!s.referenceConstraints.empty()) {
        nlohmann::json rc = nlohmann::json::array();
        for (const auto& c : s.referenceConstraints) rc.push_back(toJson(c));
        j["reference_constraints"] = std::move(rc);
    }
    return j;
}

inline std::string toString(const SceneSnapshot& s) {
    return toJson(s).dump(2);
}

inline bool isReservedBehavior(const std::string& t) {
    // D-036 turn-32 addendum: "Rigid" moved out of reserved-not-shipped —
    // it has runtime tag-set support per D-036; persistence round-trips
    // the tag. Integrator dispatch stays parked under
    // BDD-006-RIGID-DISPATCH-PARKED until slice B-3.
    return t == "Elastic" || t == "Fluid" || t == "Generator";
}

inline bool isKnownBehavior(const std::string& t) {
    return t == "Float" || t == "TriangularCloth" || t == "FastGridCloth"
        || t == "Rigid";
}

inline bool isReservedShape(const std::string& /*s*/) {
    // v1: no shapes are currently reserved-but-not-shipped. The reserved-shape
    // *pattern* is preserved so future names can opt into the same loud-fail
    // behavior; D-010 amends D-003 to record the membership change.
    return false;
}

inline bool isKnownShape(const std::string& s) {
    return s == "grid" || s == "sphere" || s == "cube";
}

inline Result<Source> sourceFromJson(const nlohmann::json& j, int idx) {
    using R = Result<Source>;
    Source s;
    std::string idxStr = "objects[" + std::to_string(idx) + "].source";
    std::string err;
    std::string type;
    if (!detail::readField(j, "type", type, err, idxStr)) return R::fail(err);
    if (type == "primitive") {
        s.kind = Source::Kind::Primitive;
        if (!detail::readField(j, "shape", s.primitive.shape, err, idxStr)) return R::fail(err);
        if (isReservedShape(s.primitive.shape)) {
            return R::fail("source.shape '" + s.primitive.shape +
                           "' not available in this build (objects[" +
                           std::to_string(idx) + "])");
        }
        if (!isKnownShape(s.primitive.shape)) {
            return R::fail("unknown source.shape '" + s.primitive.shape +
                           "' at objects[" + std::to_string(idx) + "]");
        }
        if (!detail::readField(j, "size", s.primitive.size, err, idxStr)) return R::fail(err);
        if (!detail::readField(j, "tessellation", s.primitive.tessellation, err, idxStr))
            return R::fail(err);
        // direction/mass/jiggle have v1 defaults if omitted.
        if (auto it = j.find("direction"); it != j.end()) s.primitive.direction = it->get<std::string>();
        if (auto it = j.find("mass"); it != j.end()) s.primitive.mass = it->get<double>();
        if (auto it = j.find("jiggle"); it != j.end()) s.primitive.jiggle = it->get<bool>();
    } else if (type == "import") {
        s.kind = Source::Kind::Import;
        if (!detail::readField(j, "path", s.import.path, err, idxStr)) return R::fail(err);
        // v1 supports .obj only.
        auto dot = s.import.path.find_last_of('.');
        std::string ext = (dot == std::string::npos) ? "" : s.import.path.substr(dot);
        for (auto& c : ext) c = (char)std::tolower((unsigned char)c);
        if (ext != ".obj") {
            return R::fail("unsupported source.path extension '" + s.import.path +
                           "' at objects[" + std::to_string(idx) + "]");
        }
        if (auto it = j.find("scale"); it != j.end()) s.import.scale = it->get<double>();
        if (auto it = j.find("mass"); it != j.end()) s.import.mass = it->get<double>();
    } else {
        return R::fail("unknown source.type '" + type + "' at objects[" +
                       std::to_string(idx) + "]");
    }
    return R::success(s);
}

inline Result<Transform> transformFromJson(const nlohmann::json& j, int idx) {
    using R = Result<Transform>;
    Transform t;
    std::string path = "objects[" + std::to_string(idx) + "].transform";
    std::string err;
    auto pit = j.find("position");
    if (pit == j.end()) return R::fail("missing 'position' at " + path);
    if (!detail::readVec3(*pit, t.position, err, path + ".position")) return R::fail(err);
    auto rit = j.find("rotation");
    if (rit == j.end()) return R::fail("missing 'rotation' at " + path);
    if (!detail::readQuat(*rit, t.rotation, err, path + ".rotation")) return R::fail(err);
    t.rotation = normalizeQuat(t.rotation);
    return R::success(t);
}

inline Result<Material> materialFromJson(const nlohmann::json& j, int idx,
                                          LoadWarnings* warnings = nullptr) {
    using R = Result<Material>;
    Material m;
    std::string path = "objects[" + std::to_string(idx) + "].material";
    std::string err;
    auto bc = j.find("base_color");
    if (bc == j.end()) return R::fail("missing 'base_color' at " + path);
    if (!detail::readVec3(*bc, m.baseColor, err, path + ".base_color")) return R::fail(err);
    if (!detail::readField(j, "metallic", m.metallic, err, path)) return R::fail(err);
    if (!detail::readField(j, "roughness", m.roughness, err, path)) return R::fail(err);
    if (!detail::readField(j, "specular_weight", m.specularWeight, err, path)) return R::fail(err);
    auto ec = j.find("emission_color");
    if (ec == j.end()) return R::fail("missing 'emission_color' at " + path);
    if (!detail::readVec3(*ec, m.emissionColor, err, path + ".emission_color")) return R::fail(err);
    if (clampInPlace(m) && warnings) {
        warnings->messages.push_back("clamped out-of-range material values at " + path);
    }
    return R::success(m);
}

inline Result<Behavior> behaviorFromJson(const nlohmann::json& j, int idx) {
    using R = Result<Behavior>;
    Behavior b;
    std::string path = "objects[" + std::to_string(idx) + "].behavior";
    std::string err;
    if (!detail::readField(j, "type", b.type, err, path)) return R::fail(err);
    if (isReservedBehavior(b.type)) {
        return R::fail("behavior '" + b.type +
                       "' not available in this build (objects[" +
                       std::to_string(idx) + "])");
    }
    if (!isKnownBehavior(b.type)) {
        return R::fail("unknown behavior.type '" + b.type + "' at " + path);
    }
    if (auto it = j.find("params"); it != j.end()) b.params = *it;
    return R::success(b);
}

inline Result<Object> objectFromJson(const nlohmann::json& j, int idx,
                                       LoadWarnings* warnings = nullptr) {
    using R = Result<Object>;
    Object o;
    std::string path = "objects[" + std::to_string(idx) + "]";
    std::string err;
    if (!detail::readField(j, "id", o.id, err, path)) return R::fail(err);
    if (!detail::readField(j, "name", o.name, err, path)) return R::fail(err);
    auto sj = j.find("source");
    if (sj == j.end()) return R::fail("missing 'source' at " + path);
    auto sr = sourceFromJson(*sj, idx);
    if (!sr.ok) return R::fail(sr.error.message);
    o.source = std::move(sr.value);
    auto tj = j.find("transform");
    if (tj == j.end()) return R::fail("missing 'transform' at " + path);
    auto tr = transformFromJson(*tj, idx);
    if (!tr.ok) return R::fail(tr.error.message);
    o.transform = std::move(tr.value);
    auto mj = j.find("material");
    if (mj == j.end()) return R::fail("missing 'material' at " + path);
    auto mr = materialFromJson(*mj, idx, warnings);
    if (!mr.ok) return R::fail(mr.error.message);
    o.material = std::move(mr.value);
    auto bj = j.find("behavior");
    if (bj == j.end()) return R::fail("missing 'behavior' at " + path);
    auto br = behaviorFromJson(*bj, idx);
    if (!br.ok) return R::fail(br.error.message);
    o.behavior = std::move(br.value);
    // apply_gravity / apply_wind are optional (backward compat with v1
    // snapshots that pre-date the per-object force-gate toggles).
    if (auto it = j.find("apply_gravity"); it != j.end() && it->is_boolean()) {
        o.applyGravity = it->get<bool>();
    }
    if (auto it = j.find("apply_wind"); it != j.end() && it->is_boolean()) {
        o.applyWind = it->get<bool>();
    }
    if (auto it = j.find("cloth_stiffness_scale");
        it != j.end() && it->is_number()) {
        o.clothStiffnessScale = it->get<double>();
    }
    // fixed_particles is optional (backward compat). Malformed entries
    // are skipped rather than failing the whole load.
    if (auto it = j.find("fixed_particles");
        it != j.end() && it->is_array()) {
        for (const auto& e : *it) {
            FixedParticle f;
            if (auto v = e.find("vid");
                v != e.end() && v->is_number_integer())
                f.vid = v->get<int>();
            else
                continue;
            auto p = e.find("pos");
            if (p == e.end()) continue;
            std::string perr;
            if (!detail::readVec3(*p, f.pos, perr, path)) continue;
            o.fixedParticles.push_back(f);
        }
    }
    return R::success(std::move(o));
}

inline Result<Environment> environmentFromJson(const nlohmann::json& j) {
    using R = Result<Environment>;
    Environment e;
    std::string err;
    auto g = j.find("gravity");
    if (g != j.end() && !detail::readVec3(*g, e.gravity, err, "environment.gravity"))
        return R::fail(err);
    auto w = j.find("wind");
    if (w != j.end() && !detail::readVec3(*w, e.wind, err, "environment.wind"))
        return R::fail(err);
    // background_color is optional for backward compat.
    auto bg = j.find("background_color");
    if (bg != j.end() && !detail::readVec3(*bg, e.backgroundColor, err,
                                           "environment.background_color"))
        return R::fail(err);
    return R::success(e);
}

inline Result<SceneSnapshot> fromJson(const nlohmann::json& j) {
    using R = Result<SceneSnapshot>;
    SceneSnapshot s;
    if (!j.is_object()) return R::fail("scene root must be a JSON object");
    auto fv = j.find("format_version");
    if (fv == j.end()) return R::fail("missing format_version");
    if (!fv->is_number_integer())
        return R::fail("format_version must be an integer (got " +
                       std::string(fv->type_name()) + ")");
    int v = fv->get<int>();
    if (v != kFormatVersion) {
        return R::fail("format_version mismatch: found " + std::to_string(v) +
                       ", expected " + std::to_string(kFormatVersion));
    }
    s.formatVersion = v;
    auto objs = j.find("objects");
    if (objs == j.end() || !objs->is_array())
        return R::fail("missing or invalid 'objects' array");
    for (size_t i = 0; i < objs->size(); ++i) {
        auto r = objectFromJson((*objs)[i], (int)i, &s.warnings);
        if (!r.ok) return R::fail(r.error.message);
        s.objects.push_back(std::move(r.value));
    }
    auto envj = j.find("environment");
    if (envj != j.end()) {
        auto er = environmentFromJson(*envj);
        if (!er.ok) return R::fail(er.error.message);
        s.environment = er.value;
    }
    // reference_constraints is optional (backward compat). Malformed
    // entries are skipped rather than failing the whole load, matching
    // the fixed_particles loader policy.
    if (auto it = j.find("reference_constraints");
        it != j.end() && it->is_array()) {
        for (const auto& e : *it) {
            auto q = e.find("query");
            auto t = e.find("target");
            if (q == e.end() || t == e.end()
                || !q->is_object() || !t->is_object()) continue;
            auto qo = q->find("object"); auto qv = q->find("vertex");
            auto to = t->find("object"); auto tv = t->find("vertex");
            if (qo == q->end() || qv == q->end()
                || to == t->end() || tv == t->end()
                || !qo->is_number_integer() || !qv->is_number_integer()
                || !to->is_number_integer() || !tv->is_number_integer())
                continue;
            ReferenceConstraint c;
            c.queryObject  = qo->get<int>();
            c.queryVertex  = qv->get<int>();
            c.targetObject = to->get<int>();
            c.targetVertex = tv->get<int>();
            s.referenceConstraints.push_back(c);
        }
    }
    return R::success(std::move(s));
}

inline Result<SceneSnapshot> parseString(const std::string& text) {
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(text);
    } catch (const nlohmann::json::parse_error& e) {
        return Result<SceneSnapshot>::fail(std::string("JSON parse error: ") + e.what());
    }
    return fromJson(j);
}

inline bool writeToFile(const SceneSnapshot& s, const std::string& path,
                        std::string* error = nullptr) {
    std::ofstream out(path);
    if (!out) {
        if (error) *error = "cannot open '" + path + "' for writing";
        return false;
    }
    out << toJson(s).dump(2);
    out.close();
    if (!out) {
        if (error) *error = "write to '" + path + "' failed";
        return false;
    }
    return true;
}

inline Result<SceneSnapshot> readFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return Result<SceneSnapshot>::fail("cannot open '" + path + "' for reading");
    std::stringstream ss;
    ss << in.rdbuf();
    return parseString(ss.str());
}

// Returns the directory portion of `scenePath`, with trailing slash stripped.
// Empty string when the path has no directory component.
inline std::string sceneDir(const std::string& scenePath) {
    auto pos = scenePath.find_last_of('/');
    if (pos == std::string::npos) return "";
    return scenePath.substr(0, pos);
}

inline bool isAbsolutePath(const std::string& p) {
    return !p.empty() && p[0] == '/';
}

// Resolve an import path against the directory of the scene file it came
// from. Absolute imports pass through; relative imports are joined with
// `dir`. Anchors `BDD-014`'s "import path round-trips" against the
// schema's "interpreted relative to the scene file's directory" rule.
inline std::string resolveImportPath(const std::string& dir, const std::string& importPath) {
    if (isAbsolutePath(importPath)) return importPath;
    if (dir.empty()) return importPath;
    if (!dir.empty() && dir.back() == '/') return dir + importPath;
    return dir + "/" + importPath;
}

}  // namespace scene_format

#endif  // YSIM_SCENE_FORMAT_HPP
