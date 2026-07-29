#include "Foundation/NSString.hpp"
#include "Metal/MTLBuffer.hpp"
#include "Metal/MTLComputeCommandEncoder.hpp"
#include "Metal/MTLComputePipeline.hpp"
#include "YGLWindow.hpp"
#include "camera.hpp"
#include "FrameProfiler.hpp"
#include "MeshInspectorWindow.hpp"
#include "ProfilerWindow.hpp"
#include "SceneActionLog.hpp"
#include "program.hpp"
#include "objreader.hpp"
#include "assimpreader.hpp"
#include <nfd.hpp>
#include "scene_format.hpp"
#include "sim_config.hpp"
#include "ysim_paths.hpp"
#include "mesh_cluster.hpp"
#include "MeshGL.hpp"
#include "MeshRenderState.hpp"
#include "HiddenGLContext.hpp"
#include "bvh_motion.hpp"
#include "kinematic_body.hpp"
#include "motion_graph.hpp"
#include "motion_verb.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// stb_image: just declare what we need (impl lives in framebuffer.hpp)
extern "C" {
    extern unsigned char* stbi_load(const char*, int*, int*, int*, int);
    extern void stbi_image_free(void*);
}

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <ratio>
#include <string>
#include <type_traits>
#include <typeindex>
#include <set>
#include <array>

YGLWindow* yglwindow;

#include <cstdint>

using Index = uint32_t;

//#include "MemoryPool.hpp"


// --- extracted sim modules (see include/sim/, ARCHITECTURE.md §2.1) ---
#include "sim/core_types.hpp"
#include "sim/mesh_state.hpp"
#include "sim/initializers.hpp"
#include "sim/collision_pod.hpp"
// Must sit here, not next to simulator.hpp: scene/spatial_hash/bvh/bruteforce
// each END with a dangling `template <...>` line that heads the NEXT fragment,
// so nothing can be inserted between them. collision_pod.hpp closes cleanly,
// and PbdSystem only needs Scene/GeneralMesh as declared template names
// (mesh_state.hpp forward-declares both) — its bodies are dependent.
#include "sim/pbd_system.hpp"
// Same slot, same constraint: PdSystem is a sibling CPU solver that only
// needs Scene/GeneralMesh as declared template names.
#include "sim/pd_system.hpp"
#include "sim/material_quat.hpp"
#include "sim/scene.hpp"
#include "sim/spatial_hash.hpp"
#include "sim/bvh.hpp"
#include "sim/bruteforce.hpp"
#include "sim/simulator.hpp"
#include "sim/symplectic_system.hpp"
#include "sim/scene_registry.hpp"

// Compile-time backend name for the req-6 engine check. The build wires
// exactly one backend (METAL today); a config asking for a different one is
// rejected by the builder rather than silently ignored.
template <typename BE> inline const char* backendName();
template <> inline const char* backendName<METAL>() { return "METAL"; }
template <> inline const char* backendName<CPU>()   { return "CPU"; }
template <> inline const char* backendName<CUDA>()  { return "CUDA"; }

// SimulatorBuilder — turns a RunConfig (scene + engine + profile) into a
// configured simulator. The simulator and its System are caller-owned
// (System must outlive the Simulator that references it, and Scene state is
// process-static), so the builder *configures* a provided sim+system rather
// than allocating them: build a scene from config.scene via the shared
// applySnapshot path, after validating the config's requested backend/system
// match this build (req 6). Used by the `--scene` CLI and headless tests.
template <typename BE, typename PR, typename Sys>
struct SimulatorBuilder {
    sim_config::RunConfig config;
    std::string sceneDir;  // resolves relative import paths in config.scene

    static sim_config::Result<SimulatorBuilder> fromFile(const std::string& rawPath) {
        using R = sim_config::Result<SimulatorBuilder>;
        // Accept a cwd-relative / absolute path, falling back to project-root
        // relative so `scenes/x.json` works from the repo root or build/.
        const std::string path = ysim_paths::resolveInput(rawPath);
        auto r = sim_config::readFromFile(path);
        if (!r.ok) return R::fail(r.error.message);
        SimulatorBuilder b;
        b.config = std::move(r.value);
        b.sceneDir = scene_format::sceneDir(path);  // imports resolve here
        return R::success(std::move(b));
    }

    SimulatorBuilder& withConfig(sim_config::RunConfig c) {
        config = std::move(c);
        return *this;
    }
    SimulatorBuilder& withProfile(sim_config::ProfileConfig p) {
        config.profile = std::move(p);
        return *this;
    }

    // Configure `sim`+`system` from this config. Returns false (and fills
    // `error`) on backend/system mismatch with the compiled build.
    bool buildInto(Simulator<BE, PR, Sys>& sim, Sys& /*system*/,
                   std::string* error = nullptr) const {
        const std::string have = backendName<BE>();
        if (config.engine.backend != have) {
            if (error)
                *error = "config requests backend '" + config.engine.backend +
                         "' but this build is '" + have + "'";
            return false;
        }
        if (config.engine.system != "Explicit") {
            if (error)
                *error = "config requests system '" + config.engine.system +
                         "' but this build only wires 'Explicit'";
            return false;
        }
        // applySnapshot sets system timing (h/subSteps), environment, and
        // rebuilds the object set — the exact GUI-load path (req 3).
        sim.applySnapshot(config.scene, sceneDir);
        return true;
    }
};

// Validity self-test (BDD correctness blocks), separated out of this TU.
// Included last: it needs the full engine + GL helpers above in scope.
#include "../test/self_test_inline.hpp"

// Distinguishes a --scene RunConfig file path from a registry scene name.
static bool endsWithJson(const std::string& s) {
    return s.size() >= 5 && s.compare(s.size() - 5, 5, ".json") == 0;
}

// Usage / help text for --list / --help / -h and the unknown-flag error path.
// Lists the accepted flags and the named-scene registry (this build is METAL).
static void printUsage(std::ostream& os) {
    os << "usage: ysim [--scene <name|file.json>] [--demo-uniform] "
          "[--self-test] [--list]\n\n"
       << "flags:\n"
       << "  --scene <name>       run a named scene from the registry (below)\n"
       << "  --scene <file.json>  load a RunConfig scene file\n"
       << "  --demo-uniform       launch the ML-spatial-hash uniform demo scene\n"
       << "  --self-test          run headless self-tests and exit\n"
       << "  --list, --help, -h   print this help and exit\n\n"
       << "named scenes (--scene <name>):\n";
    for (const auto& e : scene_registry::registry<METAL, Precision>())
        os << "  " << e.name << "  -  " << e.description << "\n";
    os << "\nNote: --scene also accepts a RunConfig *.json file path.\n";
}

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--self-test") {
        return runSelfTest();
    }
    // ── CLI parse (before any GL / window creation) ──────────────────────
    //   --scene <name|file.json> : named registry scene, or a RunConfig file
    //                              (functionally identical to GUI File>Load,
    //                              req 3; its profile block can run a headless
    //                              N-frame capture + sidecar, req 2/4).
    //   --demo-uniform           : the ML-spatial-hash uniform demo scene
    //                              (== `--scene demo_uniform`, same registry
    //                              entry). Mirrors runMlDriveDiag("uniform").
    //   --list / --help / -h     : print usage + scenes and exit (no window).
    // Unknown --flags are a hard error (previously silently ignored, launching
    // the GUI anyway). Non-flag positional args keep their prior no-op.
    std::string scenePath;   // --scene value: registry name OR *.json path
    bool demoUniform = false;
    bool wantList = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scene") {
            if (i + 1 >= argc) {
                std::cerr << "--scene requires a value\n";
                return 1;
            }
            scenePath = argv[++i];
        } else if (a == "--demo-uniform") {
            demoUniform = true;
        } else if (a == "--list" || a == "--help" || a == "-h") {
            wantList = true;
        } else if (a == "--self-test") {
            // Handled at the top of main() when first; ignored elsewhere.
        } else if (a.rfind("--", 0) == 0) {
            std::cerr << "unknown option: '" << a << "'\n\n";
            printUsage(std::cerr);
            return 1;
        }
        // else: non-flag positional argument → prior behavior (ignored).
    }

    if (wantList) {
        printUsage(std::cout);
        return 0;   // before window creation: no GL, no GUI.
    }

    // Validate a named scene before any GL/window work (find() is a pure
    // registry lookup) so a typo'd name fails fast instead of flashing a
    // window. METAL matches printUsage; the registry rows are backend-agnostic.
    if (!scenePath.empty() && !endsWithJson(scenePath)
        && !scene_registry::find<METAL, Precision>(scenePath)) {
        std::cerr << "unknown scene: '" << scenePath << "'\n"
                  << "named scenes:\n";
        for (const auto& e : scene_registry::registry<METAL, Precision>())
            std::cerr << "  " << e.name << "\n";
        return 1;
    }

    std::cout << "Run simulator" << std::endl;

    //window = new YGLWindow(640, 480, "ysim");
    yglwindow = new YGLWindow(1600, 900, "ysim");

    // Set macOS Dock icon — done via setDockIcon() in dock_icon.mm
    #ifdef __APPLE__
    extern void setDockIcon(const char* path);
    setDockIcon(ysim_paths::assetFile("icons/app_icon.png").c_str());
    #endif


    // Add Ground
    

    // Add Cloth
#define METAL_SYSTEM

#ifdef METAL_SYSTEM
    using Backend = METAL;
#else
    using Backend = CPU;
#endif

    //ByteMemoryPool<METAL> pool(50*1024*1024*sizeof(Precision));
    Precision h = 1/Precision(60);
    Index subSteps = 1;
    SymplecticSystem<Backend, Precision> system(h, subSteps);
    Simulator<Backend, Precision, SymplecticSystem<Backend, Precision>> simulator(system);
    std::cout << "[Main] simulator created" << std::endl;

    //Index particleNum1D = 20;
    //Precision size1D = 100;
    //Precision kstretch = 2e3;
    //Precision kshear = 2e3;
    //Precision kbend = 4e3;
    // Cloth material constants for the built-in scenes now live with their
    // scene in scene_registry.hpp (demo_uniform); kept here (commented) for
    // reference by the disabled experiments below.
    //Index particleNum1D = 20;
    //Precision size1D = 0.5;
    //Precision kstretch = 1e5;
    //Precision kshear = 1e5;
    //Precision kbend = 2e5;
    //Precision mass = 0.1;
    //Precision thickness = 0.01;
    //simulator.addClothGridFast(particleNum1D, size1D, kstretch, kshear, kbend, thickness, mass);
    //simulator.addClothGridFast(100, 1, kstretch, kshear, kbend, thickness, mass);
    //for(int i = 0; i < 1; i++) 
    //    simulator.addCloth(particleNum1D, size1D, tinym::vec3(0, 0.15+(float)i*0.05f, 0), kstretch, kshear, kbend, thickness, mass);
    // RunConfig (scene + engine + profile) active for this run — set from
    // --scene, else left at defaults (the profile sidecar synthesizes a scene
    // from the live default sim on export).
    sim_config::RunConfig runConfig;
    bool haveRunConfig = false;
    // Set when a named registry scene is chosen (needed again after
    // initialize() for its optional post-init hook, e.g. demo_uniform's ML
    // broad-phase activation). Stays null on the *.json path.
    const scene_registry::Entry<Backend, Precision>* activeScene = nullptr;

    if (!scenePath.empty() && endsWithJson(scenePath)) {
        // --scene <file.json>: RunConfig path (unchanged SimulatorBuilder).
        auto bres = SimulatorBuilder<Backend, Precision,
            SymplecticSystem<Backend, Precision>>::fromFile(scenePath);
        if (!bres.ok) {
            std::cerr << "[--scene] load failed: " << bres.error.message << "\n";
            return 1;
        }
        std::string berr;
        if (!bres.value.buildInto(simulator, system, &berr)) {
            std::cerr << "[--scene] " << berr << "\n";
            return 1;
        }
        runConfig = bres.value.config;
        haveRunConfig = true;
        std::cout << "[--scene] loaded " << scenePath << " ("
                  << runConfig.scene.objects.size() << " objects)\n";
    } else {
        // Named scene: explicit `--scene <name>`, `--demo-uniform`, or the
        // no-arg default. All three resolve to one registry entry so each
        // scene has a single definition (scene_registry.hpp).
        const std::string sceneName =
            !scenePath.empty() ? scenePath
          : demoUniform        ? std::string("demo_uniform")
                               : std::string("default");
        activeScene = scene_registry::find<Backend, Precision>(sceneName);
        if (!activeScene) {
            std::cerr << "unknown scene: '" << sceneName << "'\n"
                      << "named scenes:\n";
            for (const auto& e : scene_registry::registry<Backend, Precision>())
                std::cerr << "  " << e.name << "\n";
            return 1;
        }
        activeScene->setup(simulator, system);
        std::cout << "[scene] built '" << activeScene->name << "'\n";
    }

    std::cout << "[Main] mesh added to scene" << std::endl;

    simulator.initialize();
    std::cout << "[Main] simulator is initialized" << std::endl;

    // Post-initialize scene configuration (e.g. demo_uniform's ML broad-phase
    // activation, which must run after initialize()). Drives off the registry
    // entry so `--demo-uniform` and `--scene demo_uniform` behave identically.
    if (activeScene && activeScene->postInit) {
        activeScene->postInit(simulator);
    }

    // if(Scene<Backend, Precision>::numMeshes > 0) {
    //     std::cout << "Try to pin general meshes\n";
    //     for(auto& mesh: Scene<Backend, Precision>::meshes) {
    //         std::cout << mesh.id << std::endl;
    //     }
    //     auto* mesh = Scene<Backend, Precision>::findById(0);
    //     std::cout << mesh << std::endl;
    //     mesh->constraints.fixParticle(0);
    //     //mesh->constraints.fixParticle(particleNum1D-1);
    // }

    // std::cout << "[Main] particles are pinned" << std::endl;




    Program shader;
    shader.loadShader("shader.vert", "shader.geom", "shader.frag");
    // D-034: after the loader contract change, loadShader no longer
    // calls exit(1) on compile/link failure. Production keeps the
    // loud-failure behavior via this defensive check at the call site.
    // The harness (Block 25 / Block 27) consults `programID == 0`
    // for SKIP semantics instead.
    if (!shader.programID) {
        std::cerr << "[ysim] main shader load failed (shader.vert / "
                     "shader.geom / shader.frag missing or malformed). "
                     "Run from build/ directory.\n";
        std::exit(1);
    }

    // ID-pass shader for hover/outline. id.vert reuses position attrib;
    // id.frag writes meshId to an R32I attachment. Failure is non-fatal
    // — outline visuals just disappear, scene still renders.
    Program idShader;
    idShader.loadShader("id.vert", "id.frag");
    const bool idShaderOk = idShader.programID != 0;

    // Point-selection shaders. idpoint.* writes (meshId, depth,
    // vertexId) for GL_POINTS into the same id FBO; point.* draws the
    // on-screen selectable/hover/select dots. Non-fatal if missing —
    // point mode just won't highlight.
    Program idPointShader;
    idPointShader.loadShader("idpoint.vert", "idpoint.frag");
    const bool idPointShaderOk = idPointShader.programID != 0;
    Program pointShader;
    pointShader.loadShader("point.vert", "point.frag");
    const bool pointShaderOk = pointShader.programID != 0;

    // Directional shadow map. Depth-only program + depth-texture FBO,
    // rendered once per frame from the light's ortho frustum and sampled
    // by shader.frag (sampler2DShadow, unit 2 — unit 1 is the id buffer).
    // Non-fatal if anything fails: shadowsOn stays 0 and lighting is
    // exactly the pre-shadow output.
    Program shadowShader;
    shadowShader.loadShader("shadow.vert", "shadow.frag");
    bool shadowOk = shadowShader.programID != 0;
    GLuint shadowFbo = 0, shadowTex = 0;
    const int kShadowRes = 2048;
    if (shadowOk) {
        glGenTextures(1, &shadowTex);
        glBindTexture(GL_TEXTURE_2D, shadowTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowRes,
                     kShadowRes, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // Border = max depth → fragments outside the map sample as lit.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        const float kBorder[4] = {1.0f, 1.0f, 1.0f, 1.0f};
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, kBorder);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                        GL_COMPARE_REF_TO_TEXTURE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        glGenFramebuffers(1, &shadowFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                               GL_TEXTURE_2D, shadowTex, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "[ysim] shadow FBO incomplete — shadows disabled\n";
            shadowOk = false;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    // Offscreen R32I framebuffer that the id pass writes to, sampled
    // both by the cursor callback (hover read-back) and by shader.frag
    // (5×5 neighborhood outline test). Built inline rather than via the
    // Framebuffer helper because integer textures need GL_NEAREST
    // filtering and the helper assumes GL_LINEAR + RGBA channel layout.
    GLuint idFbo = 0, idTex = 0, idDepth = 0;
    int idFboW = 0, idFboH = 0;
    auto ensureIdFbo = [&](int w, int h) {
        if (!idShaderOk) return;
        if (idFbo != 0 && w == idFboW && h == idFboH) return;
        if (idTex)   { glDeleteTextures(1, &idTex);             idTex = 0; }
        if (idDepth) { glDeleteRenderbuffers(1, &idDepth);      idDepth = 0; }
        if (idFbo)   { glDeleteFramebuffers(1, &idFbo);         idFbo = 0; }
        idFboW = w; idFboH = h;
        glGenFramebuffers(1, &idFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, idFbo);
        glGenTextures(1, &idTex);
        glBindTexture(GL_TEXTURE_2D, idTex);
        // RGBA32F so .r can carry the mesh id as a float and .g can
        // carry window-space depth. R32I was rejected once the depth
        // channel joined — integer-only textures can't represent the
        // [0, 1] depth value without an awkward bit-cast. Sampler in
        // the main shader is sampler2D (float) accordingly.
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, w, h, 0,
                     GL_RGBA, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                               GL_TEXTURE_2D, idTex, 0);
        glGenRenderbuffers(1, &idDepth);
        glBindRenderbuffer(GL_RENDERBUFFER, idDepth);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, w, h);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER, idDepth);
        GLenum drawBuf = GL_COLOR_ATTACHMENT0;
        glDrawBuffers(1, &drawBuf);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            std::cerr << "[ysim] id-pass FBO incomplete\n";
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    };


    bool debugEachBoxes = false;
    bool debugSceneBox = false;
    bool debugCollisions = true;
    profiler::FrameProfiler frameProfiler(360);
    profiler::ProfilerWindowState profilerWindowState;
    mesh_inspector::MeshInspectorWindowState meshInspectorWindowState;
    scene_log::SceneActionLogWindowState sceneLogWindowState;

    std::cout << "[Main] programs are loaded" << std::endl;


    camera.setPosition(tinym::vec3(0, 0, 5));
    // Ground moved to y=0 (scene content sits above it), so raise the
    // orbit/pan pivot by the same +1: curPosition follows via
    // rotatePosition (identity rotation at startup → camera at (0,1,5)).
    camera.look = tinym::vec3(0, 1, 0);
    camera.rotatePosition();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    constexpr float kUiScale = 1.2f;

    // ─── ELDS Light Theme — Figma pixel-exact tokens ───────────────
    {
        ImGuiStyle& s = ImGui::GetStyle();

        // Figma: panels rounded-[16px], inputs rounded-[8px]
        s.WindowRounding    = 16.0f;
        s.ChildRounding     = 16.0f;
        s.FrameRounding     = 8.0f;
        s.PopupRounding     = 16.0f;
        s.ScrollbarRounding = 99.0f;
        s.GrabRounding      = 99.0f;  // pill-shaped slider grab
        s.TabRounding       = 8.0f;

        // Figma: panel inner padding 24px, content gap 4px
        s.WindowPadding     = ImVec2(0, 0);  // managed manually per-section
        s.FramePadding      = ImVec2(12, 10); // inputs: p-[12px], h=40px → 10px vert
        s.ItemSpacing       = ImVec2(4, 4);   // Figma: gap-[4px] between inputs
        s.ItemInnerSpacing  = ImVec2(4, 4);
        s.ScrollbarSize     = 8.0f;
        s.GrabMinSize       = 24.0f;   // Figma: 24x24 slider thumb
        s.WindowBorderSize  = 1.0f;    // Figma: border 1px gray10
        s.FrameBorderSize   = 1.0f;    // Figma: border 1px gray20
        s.PopupBorderSize   = 1.0f;
        s.WindowTitleAlign  = ImVec2(0.04f, 0.50f);
        s.SeparatorTextBorderSize = 0.0f;

        // Figma exact color tokens
        const ImVec4 gray100 = ImVec4(0.063f, 0.078f, 0.102f, 1.0f); // #10141A
        const ImVec4 gray90  = ImVec4(0.098f, 0.122f, 0.157f, 1.0f); // #191F28
        const ImVec4 gray80  = ImVec4(0.200f, 0.239f, 0.294f, 1.0f); // #333D4B
        const ImVec4 gray60  = ImVec4(0.420f, 0.463f, 0.518f, 1.0f); // #6B7684
        const ImVec4 gray50  = ImVec4(0.545f, 0.584f, 0.631f, 1.0f); // #8B95A1
        const ImVec4 gray40  = ImVec4(0.690f, 0.722f, 0.757f, 1.0f); // #B0B8C1
        const ImVec4 gray20  = ImVec4(0.886f, 0.906f, 0.922f, 1.0f); // #E2E7EB
        const ImVec4 gray10  = ImVec4(0.933f, 0.941f, 0.949f, 1.0f); // #EEF0F2
        const ImVec4 gray08  = ImVec4(0.898f, 0.910f, 0.922f, 1.0f); // #E5E8EB
        const ImVec4 gray5   = ImVec4(0.976f, 0.980f, 0.984f, 1.0f); // #F9FAFB
        const ImVec4 white   = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
        const ImVec4 transparent = ImVec4(0, 0, 0, 0);
        const ImVec4 error   = ImVec4(1.0f, 0.369f, 0.369f, 1.0f);

        ImVec4* c = s.Colors;

        c[ImGuiCol_WindowBg]             = white;
        c[ImGuiCol_ChildBg]              = white;
        c[ImGuiCol_PopupBg]              = ImVec4(1, 1, 1, 0.98f);
        c[ImGuiCol_Text]                 = gray90;
        c[ImGuiCol_TextDisabled]         = gray60;  // Figma: labels are gray60
        c[ImGuiCol_Border]               = gray10;  // Figma: window border is gray10
        c[ImGuiCol_BorderShadow]         = transparent;

        // Figma inputs: white bg, gray20 border (FrameBorder handles this)
        c[ImGuiCol_FrameBg]              = white;
        c[ImGuiCol_FrameBgHovered]       = gray5;
        c[ImGuiCol_FrameBgActive]        = gray5;
        // ImGui 1.92's InputText caret uses its own color
        // (ImGuiCol_InputTextCursor), not ImGuiCol_Text. This palette
        // assigns the whole Colors[] array without first calling a
        // StyleColors* preset, so the caret was left at the dark-theme
        // default (near-white) → invisible on the white input bg. Pin
        // it to the dark text color so the caret shows in every field.
        c[ImGuiCol_InputTextCursor]      = gray90;

        c[ImGuiCol_TitleBg]              = white;
        c[ImGuiCol_TitleBgActive]        = white;
        c[ImGuiCol_TitleBgCollapsed]     = white;
        c[ImGuiCol_MenuBarBg]            = white;

        c[ImGuiCol_ScrollbarBg]          = transparent;
        c[ImGuiCol_ScrollbarGrab]        = gray20;
        c[ImGuiCol_ScrollbarGrabHovered] = gray40;
        c[ImGuiCol_ScrollbarGrabActive]  = gray50;

        // Button — grayOutline (transparent bg + border)
        c[ImGuiCol_Button]               = transparent;
        c[ImGuiCol_ButtonHovered]        = gray10;
        c[ImGuiCol_ButtonActive]         = gray20;

        // Figma accordion header: bg gray5, full width
        c[ImGuiCol_Header]               = gray5;
        c[ImGuiCol_HeaderHovered]        = gray10;
        c[ImGuiCol_HeaderActive]         = gray10;

        c[ImGuiCol_Separator]            = gray20;
        c[ImGuiCol_SeparatorHovered]     = gray40;
        c[ImGuiCol_SeparatorActive]      = gray60;

        // Figma slider: gray100 fill, gray08 bg track, white 24px grab
        c[ImGuiCol_SliderGrab]           = white;   // white circle thumb
        c[ImGuiCol_SliderGrabActive]     = white;
        c[ImGuiCol_CheckMark]            = gray90;

        c[ImGuiCol_Tab]                  = gray10;
        c[ImGuiCol_TabHovered]           = gray5;

        c[ImGuiCol_TableHeaderBg]        = gray10;
        c[ImGuiCol_TableBorderStrong]    = gray20;
        c[ImGuiCol_TableBorderLight]     = gray10;
        c[ImGuiCol_TableRowBg]           = white;
        c[ImGuiCol_TableRowBgAlt]        = gray5;

        c[ImGuiCol_ModalWindowDimBg]     = ImVec4(0, 0, 0, 0.35f);

        c[ImGuiCol_ResizeGrip]           = gray20;
        c[ImGuiCol_ResizeGripHovered]    = gray40;
        c[ImGuiCol_ResizeGripActive]     = gray60;

        c[ImGuiCol_PlotLines]            = gray90;
        c[ImGuiCol_PlotLinesHovered]     = error;
        c[ImGuiCol_PlotHistogram]        = gray80;  // progress bar fill = gray80
        c[ImGuiCol_PlotHistogramHovered] = gray60;

        s.ScaleAllSizes(kUiScale);

        // Force exact rounding values after scale (ScaleAllSizes multiplies them)
        s.WindowRounding = 16.0f;
        s.ChildRounding  = 16.0f;
        s.FrameRounding  = 8.0f;
        s.PopupRounding  = 16.0f;
        s.GrabRounding   = 99.0f;
        s.TabRounding    = 8.0f;
        s.ScrollbarRounding = 99.0f;
    }

    // ─── Font: AppleSDGothicNeo (Pretendard 대용) ────────────────────
    {
        const char* fontCandidates[] = {
            "/System/Library/Fonts/AppleSDGothicNeo.ttc",
            "/Library/Fonts/Pretendard-Medium.otf",
            "/System/Library/Fonts/Supplemental/AppleGothic.ttf",
        };
        ImFontConfig cfg;
        cfg.OversampleH = 2;
        cfg.OversampleV = 1;
        bool loaded = false;
        for (const char* path : fontCandidates) {
            ImFont* f = io.Fonts->AddFontFromFileTTF(
                path, 15.0f * kUiScale, &cfg,
                io.Fonts->GetGlyphRangesKorean());
            if (f) { loaded = true; break; }
        }
        if (!loaded) io.Fonts->AddFontDefault();
    }

    ImGui_ImplGlfw_InitForOpenGL(yglwindow->getGLFWWindow(), false);
#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 410");
#else
    ImGui_ImplOpenGL3_Init("#version 330");
#endif

    struct CallbacksDataPack {
        Simulator<Backend, Precision, SymplecticSystem<Backend, Precision>>* simulator;
        bool* debugEachBoxes;
        bool* debugSceneBox;
        bool* debugCollisions;
        profiler::FrameProfiler* frameProfiler;
        // Hover-readback pointers: cursor callback reads from *idFbo via
        // glReadPixels at the cursor position, mapped through the
        // window→framebuffer DPI ratio. Values are owned by main and
        // updated each frame in ensureIdFbo / render.
        GLuint* idFbo;
        int* idFboW;
        int* idFboH;
    };
    CallbacksDataPack pack = {&simulator, &debugEachBoxes, &debugSceneBox,
                              &debugCollisions, &frameProfiler,
                              &idFbo, &idFboW, &idFboH};

    //glfwSetWindowUserPointer(window->getGLFWWindow(), &system);
    glfwSetWindowUserPointer(yglwindow->getGLFWWindow(), &(pack));

    auto cursorCallback = [](GLFWwindow* window, double xpos, double ypos) {
        ImGui_ImplGlfw_CursorPosCallback(window, xpos, ypos);
        auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
        auto* sim = pack->simulator;
        if (ImGui::GetIO().WantCaptureMouse) {
            sim->hoveredObj = -1;
            sim->hoveredVert = -1;
            sim->hoveredVertObj = -1;
            return;
        }
        YGL::cursorPosCallback(window, xpos, ypos);

        // Hover read-back: sample the id FBO at the cursor pixel and
        // store the result on simulator.hoveredObj. The id FBO holds
        // last-frame's geometry — 1-frame latency is acceptable for
        // hover UX. DPI-aware: cursor pos is in window pixels while the
        // FBO is sized to framebuffer pixels, so scale by the ratio.
        if (*pack->idFbo == 0 || *pack->idFboW == 0 || *pack->idFboH == 0) return;
        int winW = 0, winH = 0;
        glfwGetWindowSize(window, &winW, &winH);
        if (winW <= 0 || winH <= 0) return;
        const float fx = (float)*pack->idFboW / (float)winW;
        const float fy = (float)*pack->idFboH / (float)winH;
        const int px = (int)(xpos * fx);
        // GL's origin is bottom-left; GLFW's cursor is top-left.
        const int py = (int)((winH - ypos) * fy);
        if (px < 0 || py < 0 || px >= *pack->idFboW || py >= *pack->idFboH) {
            sim->hoveredObj = -1;
            sim->hoveredVert = -1;
            sim->hoveredVertObj = -1;
            return;
        }
        GLint readBufBackup = 0;
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &readBufBackup);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, *pack->idFbo);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        // RGBA32F read-back. Only .r (the mesh id) is used here; depth
        // (.g) drives the outline pass via the shader sampler, not the
        // cursor logic. Cast the float id back to an integer; negative
        // sentinel (-1.0) passes through cleanly as int.
        float sampled[4] = {-1.0f, 1.0f, -1.0f, 0.0f};
        glReadPixels(px, py, 1, 1, GL_RGBA, GL_FLOAT, sampled);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, (GLuint)readBufBackup);
        sim->hoveredObj = (int)sampled[0];
        if (sim->selectionMode == SelectionMode::Point) {
            // .b = render-vertex id where a point landed; -1 sentinel
            // on background / triangle-interior (depth pre-pass ran
            // with color masked, so only point fragments wrote .b).
            int vid = (int)sampled[2];
            sim->hoveredVert = vid;
            sim->hoveredVertObj = (vid >= 0) ? (int)sampled[0] : -1;
        } else {
            sim->hoveredVert = -1;
            sim->hoveredVertObj = -1;
        }
    };
    auto scrollCallbackWrapped = [](GLFWwindow* window, double xoffset, double yoffset) {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
        if (ImGui::GetIO().WantCaptureMouse) return;
        YGL::scrollCallback(window, xoffset, yoffset);
    };
    auto mouseButtonCallback = [](GLFWwindow* window, int button, int action, int mods) {
        ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

        // 선택 트리거는 "클릭을 뗐을 때" (RELEASE). 단, 카메라 회전
        // 드래그 끝에서 의도치 않게 선택되는 걸 막기 위해 click vs
        // drag 구분: PRESS 시 커서 위치를 기록하고, RELEASE 시 동일
        // 위치 ±5px 이내일 때만 click으로 판정해 ray cast로 진행.
        // 5px 이상 이동했으면 drag로 간주하고 선택 동작 생략.
        static double pressX = 0.0, pressY = 0.0;
        static bool pressOnScene = false;
        constexpr double kClickRadiusPx = 5.0;

        if (button != GLFW_MOUSE_BUTTON_LEFT) return;

        if (action == GLFW_PRESS) {
            if (ImGui::GetIO().WantCaptureMouse) {
                pressOnScene = false;
                return;
            }
            glfwGetCursorPos(window, &pressX, &pressY);
            pressOnScene = true;
            return;
        }

        // GLFW_RELEASE
        if (!pressOnScene) return;
        pressOnScene = false;
        if (ImGui::GetIO().WantCaptureMouse) return;

        double rx = 0.0, ry = 0.0;
        glfwGetCursorPos(window, &rx, &ry);
        const double dx = rx - pressX;
        const double dy = ry - pressY;
        if (dx*dx + dy*dy > kClickRadiusPx * kClickRadiusPx) return;

        auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
        auto* simulator = pack->simulator;

        // 클릭 선택을 BVH ray cast 대신 hover 결과(=ID 버퍼 샘플)로
        // 위임. 이유: 렌더링/호버는 MeshGL → PreviewState 경로라 회전
        // 직후에도 100% 정확하지만, BVH는 Scene::packedMeshData를
        // 가리키는 별도의 positions view를 들고 있어 broadPhase.refit
        // 타이밍/GPU sync 문제로 회전 직후 클릭 픽킹이 stale해질 수
        // 있음. ID 버퍼는 이미 매 프레임 preview 상태로 다시 그려지고
        // 커서 콜백이 cursor 위치의 id를 simulator.hoveredObj에 캐시
        // 해두므로, click-release 시 그 값을 그대로 selectedObj에 옮기면
        // 시각(preview) ↔ 호버 ↔ 선택이 같은 데이터 소스를 공유한다.
        simulator->clearDebugLines();
        if (simulator->selectionMode == SelectionMode::Point) {
            if (simulator->pointRefPickActive) {
                // Reference-pick: register a PERSISTENT coincidence
                // constraint — the already-selected vertex (FOLLOWER /
                // query) must track the clicked vertex (LEADER / target)
                // every step. Selection unchanged; exit ref mode.
                if (simulator->selectedVert >= 0
                    && simulator->selectedVertObj >= 0
                    && simulator->hoveredVert >= 0
                    && simulator->hoveredVertObj >= 0
                    && simulator->setReferenceConstraint(
                           simulator->selectedVertObj, simulator->selectedVert,
                           simulator->hoveredVertObj,  simulator->hoveredVert)) {
                    std::cout << "RefConstraint: obj "
                              << simulator->selectedVertObj << " vert "
                              << simulator->selectedVert << " -> obj "
                              << simulator->hoveredVertObj << " vert "
                              << simulator->hoveredVert << std::endl;
                }
                simulator->pointRefPickActive = false;
            } else {
                simulator->selectedVert    = simulator->hoveredVert;
                simulator->selectedVertObj = simulator->hoveredVertObj;
                // Also surface the owning mesh in the inspector.
                if (simulator->selectedVertObj >= 0)
                    simulator->selectedObj = simulator->selectedVertObj;
                if (simulator->selectedVert >= 0) {
                    std::cout << "SelectedVert: obj " << simulator->selectedVertObj
                              << " vert " << simulator->selectedVert << std::endl;
                }
            }
        } else {
            simulator->selectedObj = simulator->hoveredObj;
            if (simulator->selectedObj >= 0) {
                std::cout << "ClosestObj: " << simulator->selectedObj << std::endl;
            }
        }
    };
    auto charCallback = [](GLFWwindow* window, unsigned int c) {
        ImGui_ImplGlfw_CharCallback(window, c);
    };
    auto keyCallback = [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);

        // ESC = deselect, handled BEFORE the WantCaptureKeyboard guard
        // so it works no matter which window holds ImGui keyboard/nav
        // focus. Selecting an object from the right-panel list focuses
        // that ImGui window → WantCaptureKeyboard becomes true → the
        // guard below would otherwise swallow ESC, so list-selected
        // objects could not be cleared while viewport-picked ones
        // could. One code path now serves both selection routes.
        // Skipped while a text field is being edited (WantTextInput)
        // so ESC keeps its ImGui "cancel edit" meaning there.
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS
            && !ImGui::GetIO().WantTextInput) {
            auto* p = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
            auto* sim = p->simulator;
            sim->selectedObj        = -1;
            sim->selectedVert       = -1;
            sim->selectedVertObj    = -1;
            sim->pointRefPickActive = false;
        }

        if (ImGui::GetIO().WantCaptureKeyboard) return;

        auto* pack = static_cast<CallbacksDataPack*>(glfwGetWindowUserPointer(window));
        auto* simulator = pack->simulator;
        auto* debugEachBoxes = pack->debugEachBoxes;
        auto* debugSceneBox = pack->debugSceneBox;
        auto* debugCollisions = pack->debugCollisions;

        if(key == GLFW_KEY_0 && action == GLFW_PRESS) {
            // D-042 R-8: explicit reset — preview is repopulated from
            // initializer truth before initialize() runs Scene::pack, so
            // R-3's memcpy lands at addX-time geometry instead of the last
            // frame's sim state. initialize() alone no longer suffices
            // because R-5 resync has been continuously writing sim state
            // back into preview.
            simulator->reset();
            // Wipe the accumulated profiler log so each manual re-measure
            // starts from an empty CSV. Section columns are preserved.
            if(simulator->profiler) simulator->profiler->history().clearFrames();
        } else if(key == GLFW_KEY_9 && action == GLFW_PRESS) {
        } else if(key == GLFW_KEY_1 && action == GLFW_PRESS) {
            if(simulator->scene.meshes.size() > 0)
                simulator->scene.meshes[0].constraints.fixedParticles[0] = !((bool)simulator->scene.meshes[0].constraints.fixedParticles[0]);
        } else if(key == GLFW_KEY_2 && action == GLFW_PRESS) {
            if(simulator->scene.meshes.size() > 0)
                simulator->scene.meshes[0].constraints.fixedParticles[200-1] = !((bool)simulator->scene.meshes[0].constraints.fixedParticles[200-1]);
        } else if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
            simulator->pause = !(simulator->pause);
        } else if(key == GLFW_KEY_PERIOD
                  && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
            // '.' = advance one frame, '>' (Shift+.) = advance one substep.
            // Forward only — no history, so no backward variant. While
            // running, the first press just pauses (then step from there).
            // GLFW_REPEAT included so holding the key scrubs.
            if (!simulator->pause) {
                simulator->pause = true;
            } else if (mods & GLFW_MOD_SHIFT) {
                simulator->stepSubstepsPending++;
            } else {
                simulator->stepFramesPending++;
            }
        } else if(key == GLFW_KEY_B && action == GLFW_PRESS) {
            *debugEachBoxes = !(*debugEachBoxes);
        } else if(key == GLFW_KEY_S && action == GLFW_PRESS) {
            *debugSceneBox = !(*debugSceneBox);
        } else if(key == GLFW_KEY_C && action == GLFW_PRESS) {
            *debugCollisions = !(*debugCollisions);
        } else if(key == GLFW_KEY_L && action == GLFW_PRESS) {
            simulator->logSHPerSubstep = !(simulator->logSHPerSubstep);
            std::cout << "[main] logSHPerSubstep = "
                      << (simulator->logSHPerSubstep ? "on" : "off") << "\n";
        } else if(key == GLFW_KEY_G && action == GLFW_PRESS) {
            // BVH detect+reduce A/B toggle: baseline queryPoints
            // (per-leaf-hit global atomicAdd) vs queryPointsSegmented
            // (per-threadgroup private + reduce). Only affects the BVH
            // path; the SpatialHashing path is unchanged.
            simulator->useSegmentedBVHQuery = !(simulator->useSegmentedBVHQuery);
            std::cout << "[main] useSegmentedBVHQuery = "
                      << (simulator->useSegmentedBVHQuery
                          ? "on (segmented per-TG)"
                          : "off (baseline atomicAdd)")
                      << "\n";
        } else if(key == GLFW_KEY_B && action == GLFW_PRESS) {
            // Sub-object (multi-root) LBVH A/B toggle — Phase 1 divergence
            // experiment. Square-cloth triangle BVHs only; others fall back.
            // WARNING: query traversal is single-root (Phase 2 pending), so
            // intra-cloth self-collision is INCOMPLETE while this is on —
            // measurement mode for build/refit combine timing.
            auto& bp = simulator->collisionPipeline.broadPhase;
            bp.useSubObjectBVH = !bp.useSubObjectBVH;
            bp.validateSubObject = bp.useSubObjectBVH;
            std::cout << "[main] useSubObjectBVH = "
                      << (bp.useSubObjectBVH ? "on (multi-root)" : "off (single-root)")
                      << " s=" << bp.subBvhSplitS
                      << " (groups compact <= Q^2; see [SubObjectBVH] log)\n";
        } else if(key == GLFW_KEY_N && action == GLFW_PRESS) {
            // Cycle the sub-object split parameter s in [1,16].
            auto& bp = simulator->collisionPipeline.broadPhase;
            bp.subBvhSplitS = (bp.subBvhSplitS % 16) + 1;
            std::cout << "[main] subBvhSplitS = " << bp.subBvhSplitS << "\n";
        } else if(key == GLFW_KEY_M && action == GLFW_PRESS) {
            // Sub-object top-phase mode toggle. Only bites with useSubObjectBVH
            // on + a grouped square-cloth tree; single-root meshes are unaffected.
            auto& bp = simulator->collisionPipeline.broadPhase;
            bp.subTopMode = (bp.subTopMode + 1) % 2;
            const char* m[] = { "0 CPU sweep-and-prune", "1 GPU brute top" };
            std::cout << "[main] subTopMode = " << m[bp.subTopMode] << "\n";
        }
        // NOTE: ESC-deselect is handled in the early block above
        // (before the WantCaptureKeyboard guard), not here — see the
        // comment there for why list-selected objects need it.
    };
    glfwSetCursorPosCallback(yglwindow->getGLFWWindow(), cursorCallback);
    glfwSetScrollCallback(yglwindow->getGLFWWindow(), scrollCallbackWrapped);
    glfwSetMouseButtonCallback(yglwindow->getGLFWWindow(), mouseButtonCallback);
    glfwSetCharCallback(yglwindow->getGLFWWindow(), charCallback);
    glfwSetKeyCallback(yglwindow->getGLFWWindow(), keyCallback);

    std::cout << "[Main] callbacks are set" << std::endl;

    // simulator.profiler / shBroadPhase.profiler are wired *after* the
    // profiling level is derived below: only the InFrame tier attaches the
    // profiler (the in-update ScopedTimer sections are gated on a non-null
    // pointer). PerFrame/None run with profiler==nullptr — zero per-section
    // Clock::now overhead and no per-frame debug logging.

    // --- Profiling activation (req 2) ---------------------------------------
    // Active when the --scene config's profile block is enabled, OR the
    // YSIM_PROFILE_RUN env override is set (legacy/ad-hoc). When active the
    // app auto-runs unpaused, captures `profileFrames`, then writes the CSV +
    // a scene sidecar (req 4) and quits.
    const bool envProfile = std::getenv("YSIM_PROFILE_RUN") != nullptr;
    const bool cfgProfile = haveRunConfig && runConfig.profile.enabled;
    const bool profileActive = cfgProfile || envProfile;
    const int  profileFrames = cfgProfile ? runConfig.profile.frames
        : (std::getenv("YSIM_PROFILE_FRAMES") ? std::atoi(std::getenv("YSIM_PROFILE_FRAMES")) : 30);
    const bool profileRealtimeSync = cfgProfile ? runConfig.profile.realtimeSync : false;
    std::string profileCsvPath;
    if (cfgProfile && !runConfig.profile.outputPath.empty()) {
        profileCsvPath = runConfig.profile.outputPath;
    } else if (const char* e = std::getenv("YSIM_PROFILE_CSV")) {
        profileCsvPath = e;
    } else {
#ifdef YSIM_PROJECT_ROOT
        const std::string root = YSIM_PROJECT_ROOT;
#else
        const std::string root = "";
#endif
        const std::string stem = scenePath.empty() ? std::string("default-scene")
                                                    : sim_config::pathStem(scenePath);
        profileCsvPath = sim_config::defaultProfilePath(root, stem, profileFrames);
    }
    // Auto-start unpaused so the render loop collects frames without a manual
    // play click.
    if (profileActive) simulator.pause = false;

    // --- Profiling detail level (3 tiers) -----------------------------------
    // Source priority: YSIM_PROFILE_LEVEL env > --scene config > default. The
    // default is InFrame so a build with no flags behaves exactly as before.
    // Mutable: the ProfilerWindow combo edits it at runtime, and applyLevel()
    // re-wires the profiler pointers each frame from this single source.
    sim_config::ProfileLevel activeProfileLevel =
        haveRunConfig ? runConfig.profile.level
                      : sim_config::ProfileLevel::InFrame;
    if (const char* el = std::getenv("YSIM_PROFILE_LEVEL")) {
        sim_config::ProfileLevel parsed;
        if (sim_config::parseProfileLevel(el, parsed))
            activeProfileLevel = parsed;
        else
            std::cout << "[profile] ignoring unknown YSIM_PROFILE_LEVEL='" << el
                      << "' (expected none|per_frame|in_frame)\n";
    }
    // Attach the profiler only at InFrame; PerFrame/None null it so update()'s
    // `if (profiler)` sections take the fast path. Called once now and at the
    // top of every render frame (so the GUI combo takes effect immediately).
    auto applyProfilerLevel = [&]() {
        // Drive the sync tier: profileLevel gates syncEachPhase() inside
        // update(), so None/PerFrame run the substep loop fully async (one
        // boundary sync/frame) and InFrame keeps the per-section commits.
        // Without this the GUI level selector would record nothing extra but
        // still pay the full 181-sync floor.
        simulator.profileLevel = activeProfileLevel;
        profiler::FrameProfiler* p =
            (activeProfileLevel == sim_config::ProfileLevel::InFrame)
                ? &frameProfiler : nullptr;
        simulator.profiler = p;
        simulator.shBroadPhase.profiler = p;
        simulator.mlBroadPhase.profiler = p;
    };
    applyProfilerLevel();
    std::cout << "[profile] level = "
              << sim_config::toString(activeProfileLevel) << "\n";
    // GUI-editable mirror of activeProfileLevel (0/1/2). The ProfilerWindow
    // combo writes this; render() syncs it back to the enum each frame.
    int profileLevelUi = static_cast<int>(activeProfileLevel);

    // Fused refit+enlarge experiment (NEW single-pass path; default OFF =
    // legacy refit()+enlargeTrajectory()). Opt in via env or the Profiler
    // window's "Fused Refit+Enlarge" checkbox.
    if (std::getenv("YSIM_FUSED_REFIT") != nullptr)
        simulator.collisionPipeline.broadPhase.fusedRefitEnlarge = true;

    // Sub-object-vs-regular BVH sweep, replicating runFrameProfile's collision
    // environment INSIDE the real interactive renderer (req: same pipeline, real
    // loop). YSIM_EXP_TWOMESH = master switch (also drops ground + statics the
    // obstacle in the scene block above). YSIM_EXP_SUB_S:
    //   0/unset → regular single-root LBVH (twoMeshExperiment, detectCollisionsTwoMesh)
    //   1..6    → cluster-VF (grid cluster-pair) sub-object BVH, subBvhSplitS=s
    //             applied to BOTH cloth+obstacle (global, per-mesh override -1).
    // fusedRefitEnlarge = swept refit (→ broad_refit_swept section), as in the bench.
    if (std::getenv("YSIM_EXP_TWOMESH")) {
        auto& bp = simulator.collisionPipeline.broadPhase;
        bp.twoMeshExperiment = true;
        bp.fusedRefitEnlarge = true;
        int sv = std::getenv("YSIM_EXP_SUB_S") ? std::atoi(std::getenv("YSIM_EXP_SUB_S")) : 0;
        if (sv > 0) {
            bp.useSubObjectBVH   = true;
            bp.subBvhSplitS      = sv;
            bp.subTopMode        = 1;       // GPU brute top (bench default)
            bp.clusterNonGridBVH = true;
            bp.clusterVFPipeline = true;
            bp.validateSubObject = false;
            if (bp.objTrees.size() > 1) bp.objTrees[1].builtForLifetimeId = -1;
        }
        std::cout << "[exp] twoMesh=1 fused=1 SUB_S=" << sv
                  << " cluster=" << bp.clusterVFPipeline
                  << " s=" << bp.subBvhSplitS << "\n";
    }

    auto init = []() {
        glfwSwapInterval(1);
    };
    auto render = [&]() {
        double currentTime = glfwGetTime();
        // Re-wire the profiler pointers from the (possibly GUI-edited) level
        // before any simulation step this frame. profileLevelUi is the combo's
        // backing int; fold it back into the enum first.
        activeProfileLevel = static_cast<sim_config::ProfileLevel>(profileLevelUi);
        applyProfilerLevel();
        const bool profilingOn =
            activeProfileLevel != sim_config::ProfileLevel::None;
        const bool inFrameLevel =
            activeProfileLevel == sim_config::ProfileLevel::InFrame;
        // Collect a frame snapshot at PerFrame *and* InFrame (None skips it).
        // At PerFrame the snapshot carries frame_ms + the top-level
        // physics_total/render_total scopes only; the in-update sections are
        // off because simulator.profiler is null.
        bool collectProfileFrame = profilingOn && !simulator.pause;
        // The guard pairs begin/endFrame on `collectProfileFrame`. Block 10
        // clause (c) constructs the same guard with `!sim.pause` so the
        // harness drives the production gate predicate (D-017).
        profiler::ProfilerFrameGate frameGate(frameProfiler, collectProfileFrame,
                                              simulator.frame, currentTime);

        // Per-frame modal-open flags. Reset every frame; flipped to true by
        // either the File menu (Save/Load) or the right-panel Add-Object
        // buttons (Cube/Sphere/Import). Checked at the bottom of the ImGui
        // section and routed to ImGui::OpenPopup in one place.
        bool openSaveModal = false;
        bool openLoadModal = false;
        bool openImportModal = false;
        bool openSphereModal = false;
        bool openCubeModal = false;
        bool openCylinderModal = false;
        bool openPlaneModal = false;

        // Adds a BVH kinematic body directly (no modal): default file is
        // WalkLoopA.bvh when present, else the first listing entry. The
        // file is switchable afterwards from the inspector's 모션 combo.
        auto addKinematicFromAssets = [&simulator]() {
            auto files = listBVHFiles();
            if (files.empty()) {
                std::cerr << "[addKinematic] no .bvh files in "
                          << bvhAssetDir() << std::endl;
                return;
            }
            std::string pick = files.front();
            for (const auto& f : files)
                if (f == "WalkLoopA.bvh") { pick = f; break; }
            simulator.addKinematicBody(bvhAssetDir() + "/" + pick,
                                       tinym::vec3(0, 0, 0));
        };

        auto buildSelectedMeshTarget = [&]() {
            mesh_inspector::MeshInspectorTarget target;

            // Object list shown in the no-selection (empty) state.
            // Built from requestsGeneralMeshes — the canonical,
            // pack-surviving object list (ids are the compacted ids the
            // rest of the inspector / picking uses). Built before the
            // point-mode early return so the list is available in both
            // the object-mode and point-mode empty states.
            // Object type from the request's initializer subtype
            // (same dynamic_cast cascade Scene::pack uses). Grid =
            // 평면. Behavior collapses to the two user-facing buckets:
            // cloth variants → 옷감, everything else (Float/Rigid) →
            // 강체. Label: "{타입} {id}: {행동}".
            auto objTypeName = [](GeneralMeshInitializer<Backend, Precision>* in)
                -> const char* {
                if (dynamic_cast<MeshSphereInitializer  <Backend, Precision>*>(in)) return "구";
                if (dynamic_cast<MeshCubeInitializer    <Backend, Precision>*>(in)) return "정육면체";
                if (dynamic_cast<MeshCylinderInitializer<Backend, Precision>*>(in)) return "원기둥";
                if (dynamic_cast<MeshFileInitializer    <Backend, Precision>*>(in)) return "OBJ 파일";
                if (dynamic_cast<AssimpMeshFileInitializer<Backend, Precision>*>(in)) return "모델 파일";
                if (dynamic_cast<MeshGridInitializer    <Backend, Precision>*>(in)) return "평면";
                if (dynamic_cast<MeshKinematicInitializer<Backend, Precision>*>(in)) return "키네마틱";
                return "물체";
            };
            auto behaviorBucket = [](BehaviorType b) -> const char* {
                if (b == BehaviorType::Kinematic) return "모션";
                return (b == BehaviorType::TriangularCloth
                     || b == BehaviorType::FastGridCloth) ? "옷감" : "강체";
            };
            for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes) {
                mesh_inspector::MeshInspectorTarget::ObjectListEntry e;
                e.id = r.id;
                e.label = std::string(objTypeName(r.initializer)) + " "
                        + std::to_string(r.id) + ": "
                        + behaviorBucket(r.behaviorType);
                target.object_list.push_back(std::move(e));
            }
            target.on_select_object = [&simulator](int id) {
                simulator.selectedObj = id;
            };

            // Point selection mode: a selected vertex → point panel; no
            // selected vertex → fall through with mesh_id=-1 so the same
            // "nothing selected" add-buttons panel renders.
            if (simulator.selectionMode == SelectionMode::Point) {
                tinym::vec3 wp;
                if (simulator.selectedVert >= 0
                    && simulator.selectedVertObj >= 0
                    && simulator.vertexWorldPos(simulator.selectedVertObj,
                                                simulator.selectedVert, wp)) {
                    target.point_panel = true;
                    target.point_obj  = simulator.selectedVertObj;
                    target.point_vert = simulator.selectedVert;
                    target.point_fixed = simulator.isVertexFixed(
                        simulator.selectedVertObj, simulator.selectedVert);
                    target.point_position[0] = wp.x;
                    target.point_position[1] = wp.y;
                    target.point_position[2] = wp.z;
                    target.point_ref_active = simulator.pointRefPickActive;
                    int o = simulator.selectedVertObj, v = simulator.selectedVert;
                    target.on_point_set_fixed = [&simulator, o, v](bool f) {
                        simulator.setVertexFixed(o, v, f);
                    };
                    target.on_point_move = [&simulator, o, v](float x, float y, float z) {
                        simulator.translateVertexTo(o, v, tinym::vec3(x, y, z));
                    };
                    target.on_point_ref_toggle = [&simulator]() {
                        simulator.pointRefPickActive = !simulator.pointRefPickActive;
                    };
                    for (const auto& e :
                         simulator.referenceConstraintsForPoint(o, v)) {
                        mesh_inspector::MeshInspectorTarget::PointRefEntry pe;
                        pe.selected_is_follower = e.selectedIsFollower;
                        pe.other_obj  = e.otherObj;
                        pe.other_vert = e.otherVert;
                        target.point_ref_constraints.push_back(pe);
                    }
                    target.on_point_ref_remove = [&simulator, o, v](int i) {
                        simulator.removeReferenceConstraintForPoint(o, v, i);
                    };
                }
                // No vertex selected → mesh_id stays -1 so the same
                // add-buttons panel shows; wire those callbacks too.
                target.on_request_add_cube   = [&openCubeModal]()   { openCubeModal   = true; };
                target.on_request_add_sphere = [&openSphereModal]() { openSphereModal = true; };
                target.on_request_add_cylinder = [&openCylinderModal]() { openCylinderModal = true; };
                target.on_request_add_plane  = [&openPlaneModal]()  { openPlaneModal  = true; };
                target.on_request_add_import = [&openImportModal]() { openImportModal = true; };
                target.on_request_add_kinematic = addKinematicFromAssets;
                return target;
            }

            if (auto* selectedMesh = Scene<Backend, Precision>::findById(simulator.selectedObj)) {
                target.mesh_id = selectedMesh->id;
                target.behavior_label = behaviorTypeName(selectedMesh->behaviorType);
                target.shape_label = shapeTypeName(selectedMesh->shapeType);
                target.base_color = &selectedMesh->material.baseColor;
                target.transform_position = &selectedMesh->transformPosition;
                target.on_translate = [&simulator](int id, tinym::vec3 v) {
                    simulator.translateObject(id, v);
                };
                target.rotation_wxyz = &selectedMesh->rotationQuat.w;
                target.on_rotate = [&simulator](int id, float w, float x, float y, float z) {
                    simulator.rotateObject(id, ::Quat{w, x, y, z});
                };
                target.scale = &selectedMesh->scale;
                target.on_scale = [&simulator](int id, tinym::vec3 s) {
                    simulator.scaleObject(id, s);
                };
                // 질량 — per-vertex mass, shown for EVERY behavior (the
                // cloth solvers and the Bullet body both derive from it).
                // state.m is METAL shared storage and Precision == float,
                // so &m.ptr[0] is a live float the widget can drag; the
                // committed edit goes through Simulator::setObjectMass,
                // which refills all 3N entries + the initializer params.
                if (selectedMesh->state.m.ptr && selectedMesh->state.m.size > 0) {
                    target.mass_per_vertex = &selectedMesh->state.m.ptr[0];
                    target.mass_num_points = (int)(selectedMesh->state.x.size / 3);
                    target.on_mass = [&simulator](int id, float v) {
                        simulator.setObjectMass(id, (Precision)v);
                    };
                }
                // "팽팽함" — only meaningful for cloth behaviors; left
                // null otherwise so the slider stays hidden.
                if (selectedMesh->behaviorType == BehaviorType::TriangularCloth
                 || selectedMesh->behaviorType == BehaviorType::FastGridCloth) {
                    target.cloth_stiffness_scale = &selectedMesh->clothStiffnessScale;
                    target.on_cloth_stiffness_scale =
                        [&simulator](int id, float v) {
                            if (auto* m = Scene<Backend, Precision>::findById(id))
                                m->clothStiffnessScale = (Precision)v;
                            if (auto* r = simulator.findRequest(id))
                                r->clothStiffnessScale = (Precision)v;
                        };
                }
                // Per-type stiffness coefficients next to "팽팽함". The
                // GPU upload reads behaviorParams live each frame (the
                // setBytes paths multiply by clothStiffnessScale), so a
                // slider edit takes effect next frame with no re-pack —
                // we only mirror mesh + request so it survives a rebuild.
                if (auto* cp = std::get_if<ClothBehaviorParams<Precision>>(
                        &selectedMesh->behaviorParams)) {
                    auto setT = [&simulator](int id, Precision ClothBehaviorParams<Precision>::* mem, float v) {
                        if (auto* m = Scene<Backend, Precision>::findById(id))
                            if (auto* p = std::get_if<ClothBehaviorParams<Precision>>(&m->behaviorParams))
                                p->*mem = (Precision)v;
                        if (auto* r = simulator.findRequest(id))
                            if (auto* p = std::get_if<ClothBehaviorParams<Precision>>(&r->behaviorParams))
                                p->*mem = (Precision)v;
                    };
                    target.cloth_stretch = &cp->stretch;
                    target.on_cloth_stretch = [setT](int id, float v){ setT(id, &ClothBehaviorParams<Precision>::stretch, v); };
                    target.cloth_shear = &cp->shear;
                    target.on_cloth_shear = [setT](int id, float v){ setT(id, &ClothBehaviorParams<Precision>::shear, v); };
                    target.cloth_bend = &cp->bend;
                    target.on_cloth_bend = [setT](int id, float v){ setT(id, &ClothBehaviorParams<Precision>::bend, v); };
                } else if (auto* fp = std::get_if<FastGridClothBehaviorParams<Precision>>(
                        &selectedMesh->behaviorParams)) {
                    auto setF = [&simulator](int id, Precision FastGridClothBehaviorParams<Precision>::* mem, float v) {
                        if (auto* m = Scene<Backend, Precision>::findById(id))
                            if (auto* p = std::get_if<FastGridClothBehaviorParams<Precision>>(&m->behaviorParams))
                                p->*mem = (Precision)v;
                        if (auto* r = simulator.findRequest(id))
                            if (auto* p = std::get_if<FastGridClothBehaviorParams<Precision>>(&r->behaviorParams))
                                p->*mem = (Precision)v;
                    };
                    // FastGridCloth: stretch + bend only (shear left null →
                    // hidden), per the requested per-type knob set.
                    target.cloth_stretch = &fp->kstretch;
                    target.on_cloth_stretch = [setF](int id, float v){ setF(id, &FastGridClothBehaviorParams<Precision>::kstretch, v); };
                    target.cloth_bend = &fp->kbend;
                    target.on_cloth_bend = [setF](int id, float v){ setF(id, &FastGridClothBehaviorParams<Precision>::kbend, v); };
                }
                // "두께" — contact thickness. Both variants carry the field
                // under the same name; bind whichever the mesh actually
                // holds. PbdSystem::thicknessOf and the narrow/integrate
                // paths read behaviorParams live, so the edit lands on the
                // next substep; setClothThickness mirrors mesh + request so
                // it also survives Scene::pack.
                {
                    float* thickPtr = nullptr;
                    if (auto* cp = std::get_if<ClothBehaviorParams<Precision>>(
                            &selectedMesh->behaviorParams))
                        thickPtr = &cp->thickness;
                    else if (auto* fp = std::get_if<FastGridClothBehaviorParams<Precision>>(
                            &selectedMesh->behaviorParams))
                        thickPtr = &fp->thickness;
                    if (thickPtr) {
                        target.cloth_thickness = thickPtr;
                        target.on_thickness = [&simulator](int id, float v) {
                            simulator.setClothThickness(id, (Precision)v);
                        };
                    }
                }
                // Solver-dependent slider range. PBD divides the same
                // coefficient by its per-channel reference and clamps to
                // [0,1], so everything above the reference is a dead zone —
                // cap the slider there and give it two decades of travel so
                // the meaningful band is actually reachable. Shear is hidden
                // under PBD: adjacency has no shear constraint set (the grid
                // diagonals are ordinary edges, already projected with the
                // stretch coefficient).
                // PD is deliberately absent from this remap: its weights ARE
                // the spring constants (no [0,1] projection weight, no
                // reference), so it shares the symplectic path's full slider
                // range and its shear entry stays visible for the same reason
                // the symplectic path keeps it.
                if (simulator.usePbd) {
                    const float sRef = std::log10((float)simulator.pbd.stretchRef);
                    const float bRef = std::log10((float)simulator.pbd.bendRef);
                    target.cloth_stretch_log_range[0] = sRef - 2.0f;
                    target.cloth_stretch_log_range[1] = sRef;
                    target.cloth_bend_log_range[0]    = bRef - 2.0f;
                    target.cloth_bend_log_range[1]    = bRef;
                    target.cloth_shear = nullptr;
                    target.on_cloth_shear = nullptr;
                }
                // D-027: material inspector path. base_color is set above;
                // wire the other 4 material fields + the commit callback so
                // each widget change routes through Simulator::setMaterial
                // (which writes both mesh->material and pendingMaterials[id]).
                target.metallic = &selectedMesh->material.metallic;
                target.roughness = &selectedMesh->material.roughness;
                target.specular_weight = &selectedMesh->material.specularWeight;
                target.emission_color = &selectedMesh->material.emissionColor;
                target.on_material_edit = [&simulator](int id,
                                                       tinym::vec3 baseColor,
                                                       float metallic, float roughness,
                                                       float specularWeight,
                                                       tinym::vec3 emissionColor) {
                    ::Material m;
                    m.baseColor = baseColor;
                    m.metallic = metallic;
                    m.roughness = roughness;
                    m.specularWeight = specularWeight;
                    m.emissionColor = emissionColor;
                    simulator.setMaterial(id, m);
                };
                // FR-006 / BDD-006 / D-036: behavior-tag editing.
                // Map enum → dropdown index (reserved-not-shipped
                // entries collapse to -1, disabling the combo).
                auto behaviorToIndex = [](BehaviorType t) -> int {
                    switch (t) {
                        case BehaviorType::Float:           return 0;
                        case BehaviorType::TriangularCloth: return 1;
                        case BehaviorType::FastGridCloth:   return 2;
                        case BehaviorType::Rigid:           return 3;
                        default:                            return -1;
                    }
                };
                target.current_behavior_index = behaviorToIndex(selectedMesh->behaviorType);
                target.grid_eligible = (dynamic_cast<MeshGridInitializer<Backend, Precision>*>(
                    selectedMesh->initializer) != nullptr);
                // Plane checkerboard render option — grid meshes only (the
                // plane primitive). Aliases the live flag for in-place
                // preview; the callback mirrors onto the request so a
                // Scene::pack rebuild preserves it (D-025-style mirror).
                if (target.grid_eligible) {
                    target.checkerboard = &selectedMesh->checkerboard;
                    target.on_checkerboard = [&simulator](int id, bool on) {
                        if (auto* m = Scene<Backend, Precision>::findById(id))
                            m->checkerboard = on;
                        if (auto* r = simulator.findRequest(id))
                            r->checkerboard = on;
                    };
                }
                target.on_behavior_change = [&simulator](int id, int newIndex) -> bool {
                    static const BehaviorType kIndexToType[] = {
                        BehaviorType::Float,
                        BehaviorType::TriangularCloth,
                        BehaviorType::FastGridCloth,
                        BehaviorType::Rigid,
                    };
                    if (newIndex < 0 || newIndex > 3) return false;
                    return simulator.changeBehavior(id, kIndexToType[newIndex]);
                };
                // D-041: wire Delete button → Simulator::removeMesh.
                target.on_delete = [&simulator](int id) {
                    simulator.removeMesh(id);
                };
                // Per-object environment-force toggles (UI checkboxes alias
                // GeneralMesh.applyGravity / applyWind directly).
                target.apply_gravity = &selectedMesh->applyGravity;
                target.apply_wind    = &selectedMesh->applyWind;
                // Mirror the toggle into the matching request so
                // Simulator::reset() (which re-realizes meshes from
                // requests via Scene::pack) preserves the user's choice.
                // For Rigid meshes, also recreate the Bullet body so the
                // new applyGravity choice maps to mass=0 (static, Float-
                // like) or mass=1 (dynamic). Cloth gating already takes
                // effect on the next applyEnvironmentForces frame, so
                // no further work is needed for non-Rigid meshes.
                target.on_env_toggle_change = [&simulator](int id, bool g, bool w) {
                    for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes) {
                        if (r.id == id) {
                            r.applyGravity = g;
                            r.applyWind    = w;
                            break;
                        }
                    }
                    auto& meshes = Scene<Backend, Precision>::meshes;
                    for (int idx = 0; idx < (int)meshes.size(); ++idx) {
                        if (meshes[idx].id != id) continue;
                        if (meshes[idx].behaviorType == BehaviorType::Rigid) {
                            simulator.recreateRigidBackendBody(idx);
                        }
                        break;
                    }
                };
                // Static(고정) declaration toggle. Pill aliases isStatic; the
                // callback mirrors to the request, live-updates the BVH
                // objStatic cache (so the very next refit skips this object),
                // and — when turning static ON — auto-clears gravity/wind so
                // the mesh has no motion source (the apply_* pills follow,
                // since they alias the same live fields). For Rigid meshes,
                // gravity-off ⇒ mass=0 on the recreated Bullet body (static).
                target.is_static = &selectedMesh->isStatic;
                target.on_static_change = [&simulator](int id, bool s) {
                    for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes) {
                        if (r.id == id) {
                            r.isStatic = s;
                            if (s) { r.applyGravity = false; r.applyWind = false; }
                            break;
                        }
                    }
                    auto& meshes = Scene<Backend, Precision>::meshes;
                    for (int idx = 0; idx < (int)meshes.size(); ++idx) {
                        if (meshes[idx].id != id) continue;
                        meshes[idx].isStatic = s;
                        if (s) {
                            meshes[idx].applyGravity = false;
                            meshes[idx].applyWind    = false;
                        }
                        auto& trees =
                            simulator.collisionPipeline.broadPhase.objTrees;
                        if (idx < (int)trees.size()) trees[idx].objStatic = s;
                        if (meshes[idx].behaviorType == BehaviorType::Rigid) {
                            simulator.recreateRigidBackendBody(idx);
                        }
                        break;
                    }
                };
                // Collider panel (collider_pipeline_rework.md §1, P0).
                // The dropdown index is a per-frame snapshot of the live
                // ColliderKind; the two pills alias the live bools. Every
                // edit writes BOTH the mesh (immediate effect / display)
                // and the request (survives Scene::pack — a later
                // translate or add-object would otherwise reset it back
                // to the initializer-derived default).
                target.collider_kind_index = (int)selectedMesh->colliderKind;
                target.on_collider_kind = [&simulator](int id, int kindIndex) {
                    static const ColliderKind kIndexToKind[] = {
                        ColliderKind::Mesh,
                        ColliderKind::Sphere,
                        ColliderKind::Box,
                        ColliderKind::Cylinder,
                        ColliderKind::Plane,
                    };
                    if (kindIndex < 0 || kindIndex > 4) return;
                    const ColliderKind k = kIndexToKind[kindIndex];
                    if (auto* m = Scene<Backend, Precision>::findById(id))
                        m->colliderKind = k;
                    if (auto* r = simulator.findRequest(id))
                        r->colliderKind = k;
                };
                target.collidable   = &selectedMesh->collidable;
                target.self_collide = &selectedMesh->selfCollide;
                target.on_collidable = [&simulator](int id, bool on) {
                    if (auto* m = Scene<Backend, Precision>::findById(id))
                        m->collidable = on;
                    if (auto* r = simulator.findRequest(id))
                        r->collidable = on;
                };
                target.on_self_collide = [&simulator](int id, bool on) {
                    if (auto* m = Scene<Backend, Precision>::findById(id))
                        m->selfCollide = on;
                    if (auto* r = simulator.findRequest(id))
                        r->selfCollide = on;
                };
                // Sub-object BVH panel — shown ONLY when the profiler's
                // master "Cluster mode" is on. Per-object controls (the
                // pointers alias this mesh's own fields): split s (k=4^s) and
                // cluster-color render, so each object divides to suit its
                // size. Both writes mirror to the request (so the '0' reset
                // preserves them) and the s change clears just THIS mesh's
                // tree cache so the next update rebuilds it at the new split.
                if (simulator.clusterModeOn) {
                    // Lazily turn the inherit sentinel (-1) into a concrete
                    // per-object value so the slider shows/edits a real s.
                    if (selectedMesh->clusterSplitS < 1)
                        selectedMesh->clusterSplitS = std::max(1,
                            simulator.collisionPipeline.broadPhase.subBvhSplitS);
                    target.subobj_split_s = &selectedMesh->clusterSplitS;
                    target.subobj_render  = &selectedMesh->clusterRender;
                    target.on_subobj_split = [&simulator](int id, int s) {
                        for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes)
                            if (r.id == id) { r.clusterSplitS = s; break; }
                        auto& meshes = Scene<Backend, Precision>::meshes;
                        auto& trees = simulator.collisionPipeline.broadPhase.objTrees;
                        for (int idx = 0; idx < (int)meshes.size(); ++idx)
                            if (meshes[idx].id == id) {
                                meshes[idx].clusterSplitS = s;
                                if (idx < (int)trees.size()) trees[idx].builtForLifetimeId = -1;
                                break;
                            }
                    };
                    target.on_subobj_render = [](int id, bool on) {
                        for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes)
                            if (r.id == id) { r.clusterRender = on; break; }
                        for (auto& m : Scene<Backend, Precision>::meshes)
                            if (m.id == id) { m.clusterRender = on; break; }
                    };
                }
                // Kinematic body: playback panel. Snapshots are rebuilt
                // each frame; commits go through the Simulator helpers /
                // the request-owned initializer (pack-stable). The
                // gravity/wind toggles and behavior tabs are meaningless
                // for prescribed motion — hide them again.
                if (auto* kin = simulator.kinematicOf(selectedMesh->id)) {
                    target.kin_panel = true;
                    target.kin_playing = kin->playing;
                    target.kin_speed = kin->playSpeed;
                    target.kin_loop = kin->loop;
                    target.kin_time = (float)kin->localTime;
                    target.kin_duration = (float)kin->activeDuration();
                    target.kin_file = std::filesystem::path(
                        kin->params.filePath).filename().string();
                    target.kin_file_list = listBVHFiles();
                    target.apply_gravity = nullptr;
                    target.apply_wind = nullptr;
                    target.current_behavior_index = -1;
                    // Camera-follow toggle: reflect whether THIS body is the
                    // current follow target; the callback sets/clears it.
                    target.kin_camera_follow =
                        (simulator.cameraFollowObjId == selectedMesh->id);
                    target.on_kin_camera_follow =
                        [&simulator](int id, bool follow) {
                            simulator.cameraFollowObjId = follow ? id : -1;
                        };
                    // Motion-graph snapshots (mode-exclusive panel).
                    target.kin_mode = kin->motionMode;
                    target.kin_graph_ready = kin->graphActive();
                    target.kin_threshold = kin->graphThreshold;
                    target.kin_marker_frac = kin->graphMarkerFrac;
                    target.kin_blend_colorize = kin->blendColorize;
                    target.kin_blend_preview = kin->blendPreview;
                    target.kin_blend_absroot = kin->blendAbsoluteRoot;
                    target.kin_preview_playing = kin->previewPlay;  // 0 none, slot+1, -1 blend
                    target.kin_blend_preset = kin->blendPreset;
                    if (target.kin_blend_presets.empty())
                        for (const auto& p : blendPresets())
                            target.kin_blend_presets.push_back(p.name);
                    // Reusable clip-selector slots for the active mode: one row
                    // per selectable clip (file + preview toggle + color + play),
                    // identical across transition / DTW / blend-space modes.
                    {
                        const std::string cur = target.kin_file;
                        auto pushSlot = [&](const std::string& label,
                                            const std::string& file, int idx) {
                            mesh_inspector::MeshInspectorTarget::MotionClipSlot s;
                            s.label = label;
                            s.file = file;
                            if (idx < int(kin->motionSlots.size())) {
                                s.preview = kin->motionSlots[idx].preview;
                                s.color = kin->motionSlots[idx].color.data();
                                // Frame window: known only once the clip is
                                // cached (preview/play loads it) → slider shows
                                // then. Resolved against the cached length.
                                const int fc = int(
                                    kin->motionSlots[idx].cachedClip.frames.size());
                                s.frame_count = fc;
                                s.loop_sel = kin->motionSlots[idx].loopSel;
                                if (fc > 0) {
                                    const auto rg = kin->slotRange(idx);
                                    s.range_start = rg[0];
                                    s.range_end = rg[1];
                                }
                            }
                            target.kin_clip_slots.push_back(std::move(s));
                        };
                        if (kin->motionMode == 2 || kin->motionMode == 3) {
                            pushSlot("모션 1 (시작)",
                                     kin->transFileA.empty() ? cur : kin->transFileA, 0);
                            pushSlot("모션 2 (도착)",
                                     kin->transFileB.empty() ? cur : kin->transFileB, 1);
                        } else if (kin->motionMode == 4) {
                            // No fixed corner names — the file combo already
                            // shows each clip; rows are identified by file.
                            for (size_t i = 0; i < kin->blendSpaceFiles.size(); ++i)
                                pushSlot("", kin->blendSpaceFiles[i], int(i));
                        } else if (kin->motionMode == 5) {
                            if (kin->verbFiles.size() >= 2) {
                                pushSlot("모션 1", kin->verbFiles[0], 0);
                                pushSlot("모션 2", kin->verbFiles[1], 1);
                            }
                        } else if (kin->motionMode == 6) {
                            const int n = kin->numSlots();
                            for (int i = 0; i < n &&
                                            i < int(kin->motionSlots.size());
                                 ++i)
                                pushSlot("모션 " + std::to_string(i + 1),
                                         kin->slotFile(i), i);
                        }
                    }
                    target.kin_sim_paused = simulator.pause;
                    target.kin_status = kin->graphStatus;
                    if (kin->graphActive())
                        target.kin_label =
                            kin->graphSession.currentLabel(kin->localTime);
                    // Blend-space pad snapshot (mode 4, after a build): the
                    // session's clips + coords are authoritative; pad/bar labels
                    // are the clip file names (stem, no .bvh) — no fixed naming.
                    if (kin->motionMode == 4 && kin->graphActive()) {
                        const auto& ses = kin->graphSession;
                        target.kin_blend_cursor[0] = ses.cursor[0];
                        target.kin_blend_cursor[1] = ses.cursor[1];
                        for (size_t i = 0; i < ses.clips.size(); ++i) {
                            target.kin_blend_coords.push_back(ses.clipCoords[i]);
                            target.kin_blend_labels.push_back(
                                std::filesystem::path(ses.clips[i].name)
                                    .stem()
                                    .string());
                        }
                        ses.blendWeights(ses.cursor, target.kin_blend_weights);
                        // Show the EFFECTIVE mix the pose uses: blendPoseNMean
                        // drops negative (extrapolation) weights and renormalizes
                        // over the positives, so mirror that for the pad display.
                        {
                            auto& bw = target.kin_blend_weights;
                            float sp = 0.0f;
                            for (auto& x : bw) { if (x < 0.0f) x = 0.0f; sp += x; }
                            if (sp > 1e-6f) for (auto& x : bw) x /= sp;
                        }
                    }
                    // 2-motion keytime blend (mode 5): editable keytimes +
                    // tags + adverbs + the live mix. Status reuses kin_status;
                    // `target` is rebuilt fresh each frame so push_back is safe.
                    target.kin_verb_ready = kin->verbActive();
                    target.kin_verb_preview = kin->verbPreview;
                    target.kin_verb_kt_preview = kin->verbKtPreview;
                    target.kin_verb_preset = kin->verbPreset;
                    if (target.kin_verb_presets.empty())
                        for (const auto& p : verbPresets())
                            target.kin_verb_presets.push_back(p.name);
                    target.kin_verb_extrapolate = kin->verbExtrapolate;
                    if (kin->motionMode == 5 || kin->motionMode == 6) {
                        target.kin_status = kin->verbStatus;
                        target.kin_verb_tags = kin->verbTags;
                        target.kin_verb_adverb = kin->verbAdverb;
                        target.kin_verb_query[0] = kin->verbQuery[0];
                        target.kin_verb_query[1] = kin->verbQuery[1];
                        if (kin->verbActive()) {
                            for (const auto& e : kin->verbBlend.ex) {
                                target.kin_verb_keys.push_back(e.key);
                                target.kin_verb_frame_count.push_back(
                                    int(e.clip.frames.size()));
                                target.kin_verb_names.push_back(
                                    std::filesystem::path(e.name).stem().string());
                            }
                            kin->verbBlend.weights(kin->verbBlend.query,
                                                   target.kin_verb_weights);
                        }
                    }
                    target.kin_graph_selected.assign(
                        target.kin_file_list.size(), 0);
                    for (size_t fi = 0; fi < target.kin_file_list.size(); ++fi)
                        for (const auto& s : kin->graphSelFiles)
                            if (s == target.kin_file_list[fi]) {
                                target.kin_graph_selected[fi] = 1;
                                break;
                            }
                    target.on_kin_play = [&simulator](int id, bool playing) {
                        if (auto* k = simulator.kinematicOf(id)) k->playing = playing;
                    };
                    target.on_kin_speed = [&simulator](int id, float speed) {
                        if (auto* k = simulator.kinematicOf(id)) k->playSpeed = speed;
                    };
                    target.on_kin_loop = [&simulator](int id, bool loop) {
                        if (auto* k = simulator.kinematicOf(id)) k->loop = loop;
                    };
                    target.on_kin_scrub = [&simulator](int id, float timeSec) {
                        simulator.setKinematicTime(id, (double)timeSec);
                    };
                    target.on_kin_file = [&simulator](int id, const std::string& f) {
                        simulator.setKinematicFile(id, bvhAssetDir() + "/" + f);
                    };
                    target.on_kin_mode = [&simulator](int id, int mode) {
                        simulator.setKinematicMode(id, mode);
                    };
                    target.on_kin_threshold = [&simulator](int id, float v) {
                        if (auto* k = simulator.kinematicOf(id))
                            k->graphThreshold = v;
                    };
                    target.on_kin_marker_frac = [&simulator](int id, float v) {
                        if (auto* k = simulator.kinematicOf(id))
                            k->graphMarkerFrac = v;
                    };
                    target.on_kin_graph_toggle =
                        [&simulator](int id, const std::string& f, bool on) {
                            auto* k = simulator.kinematicOf(id);
                            if (!k) return;
                            auto& v = k->graphSelFiles;
                            auto it = std::find(v.begin(), v.end(), f);
                            if (on && it == v.end()) v.push_back(f);
                            else if (!on && it != v.end()) v.erase(it);
                        };
                    target.on_kin_graph_all = [&simulator](int id, bool all) {
                        if (auto* k = simulator.kinematicOf(id)) {
                            k->graphSelFiles =
                                all ? listBVHFiles() : std::vector<std::string>{};
                        }
                    };
                    target.on_kin_walk_build = [&simulator](int id) {
                        simulator.buildKinematicWalk(id);
                    };
                    target.on_kin_walk_reseed = [&simulator](int id) {
                        simulator.reseedKinematicWalk(id);
                    };
                    target.on_kin_trans_build = [&simulator](int id) {
                        auto* k = simulator.kinematicOf(id);
                        if (!k) return;
                        const std::string cur = std::filesystem::path(
                            k->params.filePath).filename().string();
                        simulator.buildKinematicTransition(
                            id, k->transFileA.empty() ? cur : k->transFileA,
                            k->transFileB.empty() ? cur : k->transFileB);
                    };
                    target.on_kin_blend_build = [&simulator](int id) {
                        auto* k = simulator.kinematicOf(id);
                        if (!k) return;
                        const std::string cur = std::filesystem::path(
                            k->params.filePath).filename().string();
                        simulator.buildKinematicBlend(
                            id, k->transFileA.empty() ? cur : k->transFileA,
                            k->transFileB.empty() ? cur : k->transFileB);
                    };
                    // Reusable slot callbacks (mode-aware file routing): the
                    // common selector component fires these for any clip-picking
                    // mode; the slot index says which clip.
                    target.on_kin_slot_file =
                        [&simulator](int id, int slot, const std::string& f) {
                            auto* k = simulator.kinematicOf(id);
                            if (!k) return;
                            if (k->motionMode == 5 || k->motionMode == 6) {
                                if (slot >= 0 && slot < int(k->verbFiles.size()))
                                    k->verbFiles[slot] = f;
                                k->verbPreset = -1;  // manual edit → 자율선택
                            } else if (k->motionMode == 4) {
                                if (slot >= 0 &&
                                    slot < int(k->blendSpaceFiles.size())) {
                                    k->blendSpaceFiles[slot] = f;
                                    k->blendPreset = -1;  // manual edit → 자율선택
                                }
                            } else {  // transition / DTW
                                if (slot == 0) k->transFileA = f;
                                else if (slot == 1) k->transFileB = f;
                            }
                            if (slot >= 0 && slot < int(k->motionSlots.size())) {
                                k->motionSlots[slot].cachedFile.clear();  // re-cache
                                k->motionSlots[slot].rangeStart = 0;      // reset
                                k->motionSlots[slot].rangeEnd = -1;       // window
                            }
                        };
                    target.on_kin_slot_preview =
                        [&simulator](int id, int slot, bool on) {
                            auto* k = simulator.kinematicOf(id);
                            if (k && slot >= 0 && slot < int(k->motionSlots.size()))
                                k->motionSlots[slot].preview = on;
                        };
                    target.on_kin_slot_play = [&simulator](int id, int slot) {
                        simulator.startPreviewPlayback(id, slot);  // 0-based
                    };
                    target.on_kin_slot_range =
                        [&simulator](int id, int slot, int start, int end) {
                            auto* k = simulator.kinematicOf(id);
                            if (!k || slot < 0 || slot >= int(k->motionSlots.size()))
                                return;
                            if (end < start) end = start;
                            k->motionSlots[slot].rangeStart = start;
                            k->motionSlots[slot].rangeEnd = end;
                        };
                    target.on_kin_slot_loop =
                        [&simulator](int id, int slot, int loopSel) {
                            simulator.verbSetLoopSel(id, slot, loopSel);
                        };
                    target.on_kin_preview_stop = [&simulator](int id) {
                        simulator.stopPreviewPlayback(id);
                    };
                    target.on_kin_blend_colorize = [&simulator](int id, bool on) {
                        if (auto* k = simulator.kinematicOf(id)) k->blendColorize = on;
                    };
                    target.on_kin_blendspace_build = [&simulator](int id) {
                        simulator.buildKinematicBlendSpace(id);
                    };
                    target.on_kin_blend_preview = [&simulator](int id, bool on) {
                        if (auto* k = simulator.kinematicOf(id)) k->blendPreview = on;
                    };
                    target.on_kin_blend_absroot = [&simulator](int id, bool on) {
                        auto* k = simulator.kinematicOf(id);
                        if (!k) return;
                        k->blendAbsoluteRoot = on;
                        if (k->motionMode == 4 && k->graphActive())
                            simulator.buildKinematicBlendSpace(id);  // build-time
                    };
                    target.on_kin_blend_play = [&simulator](int id) {
                        simulator.startBlendPlayback(id);
                    };
                    target.on_kin_blend_preset = [&simulator](int id, int idx) {
                        simulator.setKinematicBlendPreset(id, idx);
                    };
                    target.on_kin_blend_cursor =
                        [&simulator](int id, float x, float y) {
                            simulator.setKinematicBlendCursor(id, x, y);
                        };
                    // 2-motion keytime blend (mode 5).
                    target.on_kin_verb_build = [&simulator](int id) {
                        simulator.buildKinematicVerb(id);
                    };
                    target.on_kin_verb_keytime =
                        [&simulator](int id, int ex, int which, int frame) {
                            simulator.verbSetKeytime(id, ex, which, frame);
                        };
                    target.on_kin_verb_cycle_end = [&simulator](int id, int ex) {
                        simulator.verbSetCycleEndFull(id, ex);
                    };
                    target.on_kin_verb_add_tag =
                        [&simulator](int id, const std::string& name) {
                            simulator.verbAddTag(id, name);
                        };
                    target.on_kin_verb_remove_tag = [&simulator](int id, int i) {
                        simulator.verbRemoveTag(id, i);
                    };
                    target.on_kin_verb_tag_name =
                        [&simulator](int id, int i, const std::string& name) {
                            simulator.verbSetTagName(id, i, name);
                        };
                    target.on_kin_verb_adverb =
                        [&simulator](int id, int ex, int tag, float v) {
                            simulator.verbSetAdverb(id, ex, tag, v);
                        };
                    target.on_kin_verb_query =
                        [&simulator](int id, int tag, float v) {
                            simulator.verbSetQuery(id, tag, v);
                        };
                    target.on_kin_verb_preview = [&simulator](int id, bool on) {
                        if (auto* k = simulator.kinematicOf(id)) k->verbPreview = on;
                    };
                    target.on_kin_verb_kt_preview =
                        [&simulator](int id, int m, bool on) {
                            auto* k = simulator.kinematicOf(id);
                            if (!k) return;
                            // One filmstrip at a time: on → show motion m; off →
                            // clear only if m was the one showing.
                            k->verbKtPreview = on ? m
                                              : (k->verbKtPreview == m ? -1
                                                                       : k->verbKtPreview);
                        };
                    target.on_kin_verb_play = [&simulator](int id) {
                        simulator.startVerbPlayback(id);
                    };
                    target.on_kin_verb_extrapolate =
                        [&simulator](int id, bool on) {
                            simulator.verbSetExtrapolate(id, on);
                        };
                    target.on_kin_verb_add_motion =
                        [&simulator](int id, const std::string& f) {
                            simulator.verbAddMotion(id, f);
                        };
                    target.on_kin_verb_remove_motion =
                        [&simulator](int id, int idx) {
                            simulator.verbRemoveMotion(id, idx);
                        };
                    target.on_kin_verb_preset = [&simulator](int id, int idx) {
                        simulator.setKinematicVerbPreset(id, idx);
                    };
                }
            }
            // Add-Object callbacks are wired regardless of selection — they
            // are only rendered when the no-selection branch fires inside
            // drawMeshInspectorWindow, but setting them unconditionally
            // keeps the call site free of selection-aware branches.
            target.on_request_add_cube   = [&openCubeModal]()   { openCubeModal   = true; };
            target.on_request_add_sphere = [&openSphereModal]() { openSphereModal = true; };
            target.on_request_add_cylinder = [&openCylinderModal]() { openCylinderModal = true; };
            target.on_request_add_plane  = [&openPlaneModal]()  { openPlaneModal  = true; };
            target.on_request_add_import = [&openImportModal]() { openImportModal = true; };
            target.on_request_add_kinematic = addKinematicFromAssets;
            return target;
        };

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Persistent UI state shared between menu bar, panel buttons, and
        // the Save / Load / Cube / Sphere / Import modals. Statics so the
        // text fields retain their contents across frames.
        static char scenePathBuf[512] = "scene.ysim.json";
        static char importPathBuf[512] = "assets/Human.obj";
        static float importScale = 1.0f;
        static std::string sceneIOStatus;
        static float primSize = 1.0f;
        static int primTess = 16;
        static float primPos[3] = {0.f, 0.f, 0.f};
        static float planeSize = 5.0f;
        static float planePos[3] = {0.f, 0.f, 0.f};
        static int planeDirIdx = 2; // 0:XY, 1:YZ, 2:XZ (default ground)
        static int planeTess = 20; // particleNum1D; >=2. higher = cloth-ready
        // Wall-clock ↔ sim-time sync toggle (좌측 시뮬레이션 환경 패널).
        // On: fixed-h steps are paced by the accumulated wall dt; off:
        // legacy 1 step/frame.
        // Profiling runs step once per render frame (pure compute, not
        // wall-clock-throttled) unless the config explicitly asks for
        // real-time sync. Non-profiling launches default to real-time ON.
        static bool realtimeSimSync = profileActive ? profileRealtimeSync : true;
        // Shadow pass toggle (좌측 조명 패널). Gates both the depth pass
        // and the shadowsOn frag uniform; shadowOk (FBO health) still
        // wins when false.
        static bool shadowsEnabled = true;

        // ─── Top bar: h≈56, 파일/보기(px=12) + 우측 선택모드 탭(hug) ──
        {
            // Menu bar frame padding: 12px horizontal for menu items, 18px vertical for h≈56
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 18));
            ImGui::PushStyleColor(ImGuiCol_MenuBarBg, ImVec4(1,1,1,1));
            // Dropdown popup style: more padding
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24, 16));
            if (ImGui::BeginMainMenuBar()) {
                if (ImGui::BeginMenu("파일")) {
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 12));
                    if (ImGui::MenuItem("씬 저장하기...")) openSaveModal = true;
                    if (ImGui::MenuItem("씬 불러오기...")) openLoadModal = true;
                    ImGui::PopStyleVar();
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu("보기")) {
                    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 12));
                    if (ImGui::MenuItem("프로파일러")) profilerWindowState.open = true;
                    if (ImGui::MenuItem("씬 동작 로그")) sceneLogWindowState.open = true;
                    ImGui::PopStyleVar();
                    ImGui::EndMenu();
                }

                // Right-aligned selection mode segment tab (hug content)
                {
                    // Measure text to compute hug width
                    ImVec2 szObj = ImGui::CalcTextSize("오브젝트");
                    ImVec2 szPt  = ImGui::CalcTextSize("점");
                    float pad = 4, segH = 28, segPadX = 12;
                    float segWObj = szObj.x + segPadX * 2;
                    float segWPt  = szPt.x + segPadX * 2;
                    float tabW = pad + segWObj + pad + segWPt + pad;
                    float tabH = segH + pad * 2;

                    float menuBarH = ImGui::GetWindowSize().y;
                    float rightX = ImGui::GetWindowSize().x - tabW - 16;
                    float tabY = (menuBarH - tabH) * 0.5f;
                    ImVec2 wPos = ImGui::GetWindowPos();

                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    ImVec2 tPos = {wPos.x + rightX, wPos.y + tabY};
                    dl->AddRectFilled(tPos, {tPos.x + tabW, tPos.y + tabH},
                                      ImGui::ColorConvertFloat4ToU32(ImVec4(0.933f,0.941f,0.949f,1)), 8);

                    bool objMode = simulator.selectionMode == SelectionMode::Object;
                    bool ptMode  = simulator.selectionMode == SelectionMode::Point;
                    float textH = ImGui::GetFontSize();
                    float fpy = (segH - textH) * 0.5f;

                    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8);
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0);
                    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {segPadX, fpy});

                    ImGui::SetCursorScreenPos({tPos.x + pad, tPos.y + pad});
                    ImGui::PushStyleColor(ImGuiCol_Button, objMode ? ImVec4(1,1,1,1) : ImVec4(0,0,0,0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, objMode ? ImVec4(1,1,1,1) : ImVec4(1,1,1,0.5f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1,1,1,1));
                    ImGui::PushStyleColor(ImGuiCol_Text, objMode ? ImVec4(0.098f,0.122f,0.157f,1) : ImVec4(0.545f,0.584f,0.631f,1));
                    if (ImGui::Button("오브젝트", {segWObj, segH})) {
                        simulator.selectionMode = SelectionMode::Object;
                        simulator.hoveredVert = simulator.selectedVert = -1;
                        simulator.hoveredVertObj = simulator.selectedVertObj = -1;
                        simulator.pointRefPickActive = false;
                    }
                    ImGui::PopStyleColor(4);

                    ImGui::SetCursorScreenPos({tPos.x + pad + segWObj + pad, tPos.y + pad});
                    ImGui::PushStyleColor(ImGuiCol_Button, ptMode ? ImVec4(1,1,1,1) : ImVec4(0,0,0,0));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ptMode ? ImVec4(1,1,1,1) : ImVec4(1,1,1,0.5f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(1,1,1,1));
                    ImGui::PushStyleColor(ImGuiCol_Text, ptMode ? ImVec4(0.098f,0.122f,0.157f,1) : ImVec4(0.545f,0.584f,0.631f,1));
                    if (ImGui::Button("점", {segWPt, segH})) {
                        simulator.selectionMode = SelectionMode::Point;
                        simulator.hoveredObj = -1;
                    }
                    ImGui::PopStyleColor(4);

                    ImGui::PopStyleVar(3);
                }

                ImGui::EndMainMenuBar();
            }
            ImGui::PopStyleVar(2); // FramePadding, WindowPadding
            ImGui::PopStyleColor(); // MenuBarBg
        }

        // ─── Layout geometry ──────────────────────────────────────────
        // Both side panels are pinned: left edge → Scene panel pivoted at
        // (0, 0.5); right edge → Object panel pivoted at (1, 0.5). The
        // menu bar's height is added to the top inset so neither panel
        // sits under it. ImGuiCond_Always + NoMove + NoResize make the
        // pinning hard (the user cannot drag or resize them).
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        const float menuH = ImGui::GetFrameHeight();
        // 패널 폭은 UI 스케일과 비례. 위젯이 1.2배 커졌으므로 폭도
        // 같은 비율로 늘려야 라벨이 잘리지 않음 (300 * 1.2 = 360).
        const float panelW = 400.0f;
        const float panelTop = vp->WorkPos.y;
        const float panelH  = vp->WorkSize.y; // full height, bar floats on top

        // ─── Figma helpers (shared by both panels) ─────────────────────
        const float P = 24.0f;
        const float CW = panelW - P * 2;  // 352px content width

        const ImVec4 cGray100(0.063f, 0.078f, 0.102f, 1.0f);
        const ImVec4 cGray90 (0.098f, 0.122f, 0.157f, 1.0f);
        const ImVec4 cGray60 (0.420f, 0.463f, 0.518f, 1.0f);
        const ImVec4 cGray50 (0.545f, 0.584f, 0.631f, 1.0f);
        const ImVec4 cGray40 (0.690f, 0.722f, 0.757f, 1.0f);
        const ImVec4 cGray20 (0.886f, 0.906f, 0.922f, 1.0f);
        const ImVec4 cGray10 (0.933f, 0.941f, 0.949f, 1.0f);
        const ImVec4 cGray5  (0.976f, 0.980f, 0.984f, 1.0f);
        const ImVec4 cWhite  (1, 1, 1, 1);

        // XYZ input: available width minus right padding (P=24)
        auto InputXYZ = [P](const char* id, float v[3]) -> bool {
            bool changed = false;
            ImGui::PushID(id);
            float w = ImGui::GetContentRegionAvail().x - P;
            float gap = 4.0f;
            float chW = (w - gap * 2.0f) / 3.0f;
            const char* labels[] = {"x", "y", "z"};
            for (int i = 0; i < 3; ++i) {
                if (i > 0) ImGui::SameLine(0, gap);
                ImGui::SetNextItemWidth(chW);
                char fmt[16]; snprintf(fmt, sizeof(fmt), "%s  %%.3f", labels[i]);
                ImGui::PushID(i);
                if (ImGui::DragFloat("##v", &v[i], 0.01f, 0, 0, fmt))
                    changed = true;
                ImGui::PopID();
            }
            ImGui::PopID();
            return changed;
        };

        // RGB input: available width minus right padding (P=24)
        auto InputRGB = [P](const char* id, float col[3]) -> bool {
            bool changed = false;
            ImGui::PushID(id);
            float w = ImGui::GetContentRegionAvail().x - P;
            float gap = 4.0f;
            float swatchW = 40.0f;
            float chW = (w - swatchW - gap * 3.0f) / 3.0f;
            const char* labels[] = {"R", "G", "B"};
            for (int i = 0; i < 3; ++i) {
                if (i > 0) ImGui::SameLine(0, gap);
                ImGui::SetNextItemWidth(chW);
                char fmt[16]; snprintf(fmt, sizeof(fmt), "%s  %%.0f", labels[i]);
                ImGui::PushID(i);
                float v255 = col[i] * 255.0f;
                if (ImGui::DragFloat("##c", &v255, 1.0f, 0.0f, 255.0f, fmt)) {
                    col[i] = v255 / 255.0f;
                    changed = true;
                }
                ImGui::PopID();
            }
            // Color swatch → opens color picker on click
            ImGui::SameLine(0, gap);
            ImVec4 preview(col[0], col[1], col[2], 1.0f);
            if (ImGui::ColorButton("##sw", preview,
                                   ImGuiColorEditFlags_NoTooltip,
                                   ImVec2(swatchW, swatchW))) {
                ImGui::OpenPopup("##picker");
            }
            if (ImGui::BeginPopup("##picker")) {
                if (ImGui::ColorPicker3("##pick", col,
                        ImGuiColorEditFlags_NoSidePreview |
                        ImGuiColorEditFlags_NoSmallPreview |
                        ImGuiColorEditFlags_NoInputs))
                    changed = true;
                ImGui::EndPopup();
            }
            ImGui::PopID();
            return changed;
        };

        // Accordion header: custom draw, h=56, gray5 bg, gray20 top+bottom border
        // Clips to parent window so rounded corners are respected
        static std::unordered_map<ImGuiID, bool> accordionStates;
        auto AccordionHeader = [&](const char* korean, const char* english) -> bool {
            ImGui::PushID(korean);
            ImGuiID aid = ImGui::GetID("##acc");
            if (accordionStates.find(aid) == accordionStates.end())
                accordionStates[aid] = true;

            float fullW = ImGui::GetContentRegionAvail().x;
            float headerH = 56.0f;
            ImVec2 pos = ImGui::GetCursorScreenPos();
            ImDrawList* dl = ImGui::GetWindowDrawList();

            ImU32 bgCol     = ImGui::ColorConvertFloat4ToU32(cGray5);
            ImU32 borderCol = ImGui::ColorConvertFloat4ToU32(cGray20);

            // Background
            dl->AddRectFilled(pos, {pos.x + fullW, pos.y + headerH}, bgCol);
            (void)borderCol; // no borders on accordion

            // Click area
            bool clicked = ImGui::InvisibleButton("##accBtn", {fullW, headerH});
            if (clicked) accordionStates[aid] = !accordionStates[aid];
            bool open = accordionStates[aid];

            // Title text: larger size (1.2x default ≈ 18px equivalent)
            ImFont* font = ImGui::GetFont();
            float titleFontSize = ImGui::GetFontSize() * 1.2f;
            float subFontSize = ImGui::GetFontSize();
            float textY = pos.y + (headerH - titleFontSize) * 0.5f;
            float subY = pos.y + (headerH - subFontSize) * 0.5f;

            ImU32 titleCol = ImGui::ColorConvertFloat4ToU32(cGray100);
            dl->AddText(font, titleFontSize, {pos.x + P, textY}, titleCol, korean);

            if (english && english[0]) {
                ImVec2 titleSize = font->CalcTextSizeA(titleFontSize, FLT_MAX, 0, korean);
                dl->AddText(font, subFontSize, {pos.x + P + titleSize.x + 4.0f, subY},
                            ImGui::ColorConvertFloat4ToU32(cGray60), english);
            }

            // Chevron: vertically centered, right-aligned at fullW - P
            {
                float arrowX = pos.x + fullW - P;
                float arrowY = pos.y + headerH * 0.5f;
                float r = 5.0f;
                ImU32 arrowCol = ImGui::ColorConvertFloat4ToU32(cGray60);
                if (open) {
                    dl->PathLineTo({arrowX - r, arrowY + r * 0.4f});
                    dl->PathLineTo({arrowX,     arrowY - r * 0.4f});
                    dl->PathLineTo({arrowX + r, arrowY + r * 0.4f});
                } else {
                    dl->PathLineTo({arrowX - r, arrowY - r * 0.4f});
                    dl->PathLineTo({arrowX,     arrowY + r * 0.4f});
                    dl->PathLineTo({arrowX + r, arrowY - r * 0.4f});
                }
                dl->PathStroke(arrowCol, 0, 2.0f);
            }

            ImGui::PopID();
            return open;
        };

        // Figma segment tab helper
        auto SegmentTab2 = [](const char* id, const char* labelA, const char* labelB,
                              bool aActive, auto onA, auto onB) {
            ImGui::PushID(id);
            const ImVec4 cGray10(0.933f, 0.941f, 0.949f, 1.0f);
            const ImVec4 cWhite(1, 1, 1, 1);
            const ImVec4 cGray90(0.098f, 0.122f, 0.157f, 1.0f);
            const ImVec4 cGray50(0.545f, 0.584f, 0.631f, 1.0f);

            ImVec2 cPos = ImGui::GetCursorScreenPos();
            float cW = ImGui::GetContentRegionAvail().x;
            float cH = 40.0f, pad = 4.0f, segH = 32.0f;
            float segW = (cW - pad * 3.0f) / 2.0f;

            ImGui::GetWindowDrawList()->AddRectFilled(
                cPos, {cPos.x + cW, cPos.y + cH},
                ImGui::ColorConvertFloat4ToU32(cGray10), 8.0f);

            ImGui::SetCursorScreenPos({cPos.x + pad, cPos.y + pad});
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);

            ImGui::PushStyleColor(ImGuiCol_Button, aActive ? cWhite : ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, aActive ? cWhite : ImVec4(1,1,1,0.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, cWhite);
            ImGui::PushStyleColor(ImGuiCol_Text, aActive ? cGray90 : cGray50);
            if (ImGui::Button(labelA, {segW, segH})) onA();
            ImGui::PopStyleColor(4);

            ImGui::SameLine(0, pad);

            bool bActive = !aActive;
            ImGui::PushStyleColor(ImGuiCol_Button, bActive ? cWhite : ImVec4(0,0,0,0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, bActive ? cWhite : ImVec4(1,1,1,0.5f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, cWhite);
            ImGui::PushStyleColor(ImGuiCol_Text, bActive ? cGray90 : cGray50);
            if (ImGui::Button(labelB, {segW, segH})) onB();
            ImGui::PopStyleColor(4);

            ImGui::PopStyleVar(2);
            ImGui::SetCursorScreenPos({cPos.x, cPos.y + cH});
            ImGui::PopID();
        };

        // ─── Scene panel (left, flush to edge, no margin) ──────────────
        ImGui::SetNextWindowPos(
            ImVec2(vp->WorkPos.x, panelTop),
            ImGuiCond_Always, ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(panelW, panelH), ImGuiCond_Always);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        if (ImGui::Begin("씬", nullptr,
                         ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoTitleBar)) {
            auto& env = Scene<Backend, Precision>::environment;

            // ── 물리 환경 Physical Environment ───────────────────
            float gravity[3] = {(float)env.gravity.x, (float)env.gravity.y, (float)env.gravity.z};
            float wind[3]    = {(float)env.wind.x,    (float)env.wind.y,    (float)env.wind.z};

            if (AccordionHeader("물리 환경", "Physical Environment")) {
                ImGui::Dummy({0, P});
                ImGui::Indent(P);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("중력");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                if (InputXYZ("gravity", gravity))
                    env.gravity = tinym::vec3(gravity[0], gravity[1], gravity[2]);

                ImGui::Dummy({0, 20});

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("바람");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                if (InputXYZ("wind", wind))
                    env.wind = tinym::vec3(wind[0], wind[1], wind[2]);

                ImGui::Unindent(P);
                ImGui::Dummy({0, P});
            }

            // Label-left / pill-right toggle row, mirroring the
            // inspector's PillToggle look (42x24, dark=on).
            auto panelToggleRow = [&](const char* id, const char* label,
                                      bool* v) -> bool {
                bool changed = false;
                ImGui::PushID(id);
                ImGui::PushStyleColor(ImGuiCol_Text,
                                      ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted(label);
                ImGui::PopStyleColor();
                const float w = 42, h = 24, r = h / 2;
                ImGui::SameLine();
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                    ImGui::GetContentRegionAvail().x - w - P);
                ImVec2 pos = ImGui::GetCursorScreenPos();
                ImDrawList* dl = ImGui::GetWindowDrawList();
                if (ImGui::InvisibleButton("##t", {w, h})) { *v = !*v; changed = true; }
                dl->AddRectFilled(pos, {pos.x + w, pos.y + h},
                    ImGui::ColorConvertFloat4ToU32(*v
                        ? ImVec4(0.063f, 0.078f, 0.102f, 1.0f)
                        : ImVec4(0.886f, 0.906f, 0.922f, 1.0f)), r);
                float tr = 10, tx = *v ? pos.x + w - 2 - tr : pos.x + 2 + tr;
                dl->AddCircleFilled({tx, pos.y + h / 2}, tr,
                                    IM_COL32(255, 255, 255, 255), 16);
                ImGui::PopID();
                return changed;
            };

            // ── 시뮬레이션 환경 Simulation ───────────────────────
            // Edits the live SymplecticSystem driving the sim: `h` is
            // the per-frame time step (default 1/60 s), `subSteps` the
            // substep count per frame (default 60). subh = h/subSteps
            // is only computed in the System ctor, so it MUST be
            // recomputed here on every edit — it is the value actually
            // fed to the integrator each substep (SimParams.subh) and
            // to the swept-AABB enlargeTrajectory. Without the resync
            // the UI would change nothing.
            if (AccordionHeader("시뮬레이션 환경", "Simulation")) {
                ImGui::Dummy({0, P});
                ImGui::Indent(P);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("프레임 당 시간 (초)");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                float frameDt = (float)system.h;
                if (ImGui::InputFloat("##simDt", &frameDt, 0.0f, 0.0f, "%.6f")) {
                    if (frameDt > 1e-6f) {
                        system.h = (Precision)frameDt;
                        system.subh = system.h /
                            (Precision)(system.subSteps > 0 ? system.subSteps : 1);
                    }
                }

                ImGui::Dummy({0, 20});

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("한 프레임 당 분할 계산 횟수");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                int subN = (int)system.subSteps;
                if (ImGui::InputInt("##simSub", &subN)) {
                    if (subN < 1) subN = 1;
                    system.subSteps = (size_t)subN;
                    system.subh = system.h / (Precision)system.subSteps;
                }

                ImGui::Dummy({0, 20});
                panelToggleRow("rtSync", "실시간 동기화", &realtimeSimSync);

                // Solver A/B: symplectic (GPU) vs PBD (CPU). Both consume the
                // same subh / substep loop / collision pipeline, so switching
                // is live — no re-init needed. Cloth stiffness is NOT here:
                // both solvers read the per-mesh stretch/shear/bend from the
                // mesh inspector (PBD maps them onto its [0,1] projection
                // weights internally), so there is one place to edit them.
                ImGui::Dummy({0, 20});
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("솔버");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                {
                    // Index order must match the dispatch priority in
                    // Simulator::update (PD > PBD > symplectic); the combo
                    // writes an exclusive pair so the two flags can never
                    // both be set from here.
                    const char* solverNames[] = { "Symplectic (GPU)", "PBD (CPU)",
                                                  "PD (CPU)" };
                    int solverIdx = simulator.usePd ? 2 : (simulator.usePbd ? 1 : 0);
                    if (ImGui::Combo("##solver", &solverIdx, solverNames,
                                     IM_ARRAYSIZE(solverNames))) {
                        simulator.usePbd = (solverIdx == 1);
                        simulator.usePd  = (solverIdx == 2);
                    }
                }

                if (simulator.usePd) {
                    ImGui::Dummy({0, 12});
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                    ImGui::TextUnformatted("PD 반복 횟수");
                    ImGui::PopStyleColor();
                    ImGui::Dummy({0, 4});
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                    if (ImGui::InputInt("##pdIter", &simulator.pd.iterations)
                        && simulator.pd.iterations < 1) {
                        simulator.pd.iterations = 1;
                    }
                    // Damping is a per-substep velocity scale (0 = none); PD's
                    // implicit solve already dissipates, so this is a knob for
                    // killing residual jitter, not the main damping source.
                    ImGui::Dummy({0, 12});
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                    ImGui::TextUnformatted("PD 감쇠");
                    ImGui::PopStyleColor();
                    ImGui::Dummy({0, 4});
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                    {
                        float pdDamping = (float)simulator.pd.damping;
                        if (ImGui::SliderFloat("##pdDamping", &pdDamping, 0.0f, 0.2f))
                            simulator.pd.damping = (Precision)pdDamping;
                    }
                }

                if (simulator.usePbd) {
                    ImGui::Dummy({0, 12});
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                    ImGui::TextUnformatted("PBD 반복 횟수");
                    ImGui::PopStyleColor();
                    ImGui::Dummy({0, 4});
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                    if (ImGui::InputInt("##pbdIter", &simulator.pbd.iterations)
                        && simulator.pbd.iterations < 1) {
                        simulator.pbd.iterations = 1;
                    }
                }

                ImGui::Unindent(P);
                ImGui::Dummy({0, P});
            }

            // ── 조명 Light ───────────────────────────────────────
            if (AccordionHeader("조명", "Light")) {
                ImGui::Dummy({0, P});
                ImGui::Indent(P);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("RGB");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                InputRGB("lightColor", env.lightColor.v);

                ImGui::Dummy({0, 16});

                // "세기" left + "1.6" right
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("세기");
                ImGui::SameLine(ImGui::GetContentRegionAvail().x - P - 18);
                ImGui::Text("%.3f", env.lightIntensity);
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                // Slider: gray10 bg, height=28 (grab 24px circle fits square)
                ImGui::PushStyleColor(ImGuiCol_FrameBg, cGray10);
                ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, cGray10);
                ImGui::PushStyleColor(ImGuiCol_FrameBgActive, cGray10);
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 4)); // (28-20)/2
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - P);
                ImGui::SliderFloat("##lightInt", &env.lightIntensity, 0.0f, 10.0f, "");
                ImGui::PopStyleVar();
                ImGui::PopStyleColor(3);

                ImGui::Dummy({0, 16});
                panelToggleRow("shadows", "그림자", &shadowsEnabled);

                ImGui::Unindent(P);
                ImGui::Dummy({0, P});
            }

            // ── 배경 Background ──────────────────────────────────
            if (AccordionHeader("배경", "Background")) {
                ImGui::Dummy({0, P});
                ImGui::Indent(P);

                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.420f, 0.463f, 0.518f, 1.0f));
                ImGui::TextUnformatted("RGB");
                ImGui::PopStyleColor();
                ImGui::Dummy({0, 4});
                InputRGB("bgColor", env.backgroundColor.v);

                ImGui::Unindent(P);
                ImGui::Dummy({0, P});
            }

        }
        ImGui::PopStyleVar(2);  // WindowRounding, WindowBorderSize
        ImGui::End();

        // ─── Object panel (right, flush to edge, no margin) ─────────────
        ImGui::SetNextWindowPos(
            ImVec2(vp->WorkPos.x + vp->WorkSize.x - panelW, panelTop),
            ImGuiCond_Always, ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(panelW, panelH), ImGuiCond_Always);
        mesh_inspector::drawMeshInspectorWindow(
            meshInspectorWindowState, buildSelectedMeshTarget());

        // ─── 하단 재생 바: 둥둥 떠다니는 바, rounded-20, h=64 ──────────
        // 좌우 패널 사이 24px gap, fill width, max 800px, 하단 24px 위
        {
            float gapFromPanel = 24.0f;
            float leftEdge = vp->WorkPos.x + panelW + gapFromPanel;
            float rightEdge = vp->WorkPos.x + vp->WorkSize.x - panelW - gapFromPanel;
            float barW = rightEdge - leftEdge;
            if (barW > 800.0f) barW = 800.0f;
            float barCenterX = (leftEdge + rightEdge) * 0.5f;
            float barY = vp->WorkPos.y + vp->WorkSize.y - 24.0f; // 24px from bottom

            const float barHH = 64.0f; // bar height
            const float elemH = 36.0f; // all inner elements same height
            // Vertical padding to center elements: (64 - 36) / 2 = 14
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 20);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(P, (barHH - elemH) / 2));
            ImGui::PushStyleColor(ImGuiCol_Border, cGray20);
            ImGui::SetNextWindowPos(
                ImVec2(barCenterX, barY),
                ImGuiCond_Always, ImVec2(0.5f, 1.0f));
            ImGui::SetNextWindowSize(ImVec2(barW, barHH), ImGuiCond_Always);
            if (ImGui::Begin("시뮬레이션 진행", nullptr,
                             ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoCollapse |
                             ImGuiWindowFlags_NoTitleBar |
                             ImGuiWindowFlags_NoScrollbar |
                             ImGuiWindowFlags_NoSavedSettings)) {
                if (simulator.targetFrames < 1) simulator.targetFrames = 1;
                const int   tgt = simulator.targetFrames;
                const Index cur = simulator.frame;

                // All elements: height = elemH, vertically centered by WindowPadding

                // Play/Pause: icon-only square button
                // Paused → gray80 filled (play icon white)
                // Playing → gray outline (pause icon gray80)
                {
                    bool paused = simulator.pause;
                    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8);
                    if (paused) {
                        // Gray80 filled
                        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.200f,0.239f,0.294f,1)); // gray80
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.250f,0.290f,0.340f,1));
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.150f,0.190f,0.240f,1));
                    } else {
                        // Gray outline
                        ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0,0,0,0));
                        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, cGray10);
                        ImGui::PushStyleColor(ImGuiCol_ButtonActive, cGray20);
                    }
                    ImVec2 btnPos = ImGui::GetCursorScreenPos();
                    if (ImGui::Button("##play", ImVec2(elemH, elemH))) {
                        simulator.pause = !simulator.pause;
                    }
                    ImGui::PopStyleColor(3);
                    ImGui::PopStyleVar(2); // FrameRounding + FrameBorderSize

                    // Draw icon centered
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    float cx = btnPos.x + elemH / 2, cy = btnPos.y + elemH / 2;
                    if (paused) {
                        // Play triangle (white)
                        ImU32 ic = IM_COL32(255, 255, 255, 255);
                        dl->AddTriangleFilled({cx - 4, cy - 6}, {cx - 4, cy + 6}, {cx + 6, cy}, ic);
                    } else {
                        // Pause bars (gray80)
                        ImU32 ic = ImGui::ColorConvertFloat4ToU32(ImVec4(0.200f,0.239f,0.294f,1));
                        dl->AddRectFilled({cx - 5, cy - 5}, {cx - 2, cy + 5}, ic, 1);
                        dl->AddRectFilled({cx + 2, cy - 5}, {cx + 5, cy + 5}, ic, 1);
                    }
                }
                ImGui::SameLine();

                // Progress bar: fill remaining minus input width
                const float inputW = 120.0f;
                const float spacing = ImGui::GetStyle().ItemSpacing.x;
                float progW = ImGui::GetContentRegionAvail().x - inputW - spacing;
                if (progW < 60.0f) progW = 60.0f;

                // Match progress bar height to elemH
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {4, (elemH - ImGui::GetFontSize()) / 2});
                float frac = (float)cur / (float)tgt;
                if (frac < 0.0f) frac = 0.0f;
                if (frac > 1.0f) frac = 1.0f;
                char overlay[64];
                std::snprintf(overlay, sizeof(overlay),
                              "%u / %d", (unsigned)cur, tgt);
                ImGui::ProgressBar(frac, ImVec2(progW, elemH), overlay);
                ImGui::PopStyleVar();

                // Frame input: 120px total, no +/- buttons, padding 12px
                ImGui::SameLine();
                ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {12, (elemH - ImGui::GetFontSize()) / 2});
                ImGui::SetNextItemWidth(inputW);
                int step = 0; // step=0 hides +/- buttons
                if (ImGui::InputInt("##tgt", &simulator.targetFrames, step)
                    && simulator.targetFrames < 1) {
                    simulator.targetFrames = 1;
                }
                ImGui::PopStyleVar();
            }
            ImGui::End();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar(3);
        }

        // ─── Modal popups ─────────────────────────────────────────────
        // Triggered by either the File menu (Save/Load) or the right
        // panel's add-object buttons (Sphere/Cube/Import). Centralized
        // here so OpenPopup → BeginPopupModal stay co-located.
        if (openSaveModal) ImGui::OpenPopup("씬 저장하기");
        if (openLoadModal) {
            // 씬 불러오기 — native open dialog (NFD) instead of a text path.
            NFD::Guard _nfdGuard;
            NFD::UniquePath _nfdPath;
            nfdfilteritem_t _nfdFilter[1] = {{"Scene", "json,ysim"}};
            if (NFD::OpenDialog(_nfdPath, _nfdFilter, 1) == NFD_OKAY) {
                auto lr = simulator.loadScene(_nfdPath.get());
                if (lr.ok) { simulator.initialize(); simulator.applyPendingMaterials(); }
            }
        }
        if (openImportModal) {
            // 3D 물체 추가 — pick the model file via NFD, then the modal
            // below still collects the import scale.
            NFD::Guard _nfdGuard;
            NFD::UniquePath _nfdPath;
            nfdfilteritem_t _nfdFilter[1] =
                {{"3D Model", "obj,fbx,gltf,glb,dae,stl,ply,3ds,blend,off,3mf,x"}};
            if (NFD::OpenDialog(_nfdPath, _nfdFilter, 1) == NFD_OKAY) {
                std::snprintf(importPathBuf, sizeof(importPathBuf), "%s",
                              _nfdPath.get());
                ImGui::OpenPopup("3D 모델 파일 가져오기");
            }
        }
        if (openSphereModal) ImGui::OpenPopup("구 생성");
        if (openCubeModal) ImGui::OpenPopup("정육면체 생성");
        if (openCylinderModal) ImGui::OpenPopup("원기둥 생성");
        if (openPlaneModal) ImGui::OpenPopup("평면 생성");
        // ─── Modal style helpers ─────────────────────────────────────
        // Figma: rounded-16, p=24, gray100 filled primary btn, outline cancel btn
        // Modal: 480px fixed width, rounded-16, p=24
        auto modalBegin = [&](const char* title) -> bool {
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 16);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(P, P));
            ImGui::PushStyleColor(ImGuiCol_PopupBg, ImVec4(1,1,1,1));
            ImGui::SetNextWindowSize(ImVec2(480, 0));
            // Center modal in viewport
            ImVec2 center = ImGui::GetMainViewport()->GetCenter();
            ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
            bool opened = ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);
            if (!opened) {
                ImGui::PopStyleColor();
                ImGui::PopStyleVar(2);
            }
            return opened;
        };
        auto modalEnd = [&]() {
            ImGui::EndPopup();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar(2);
        };
        auto modalTitle = [&](const char* title) {
            ImFont* fo = ImGui::GetFont();
            float tFS = ImGui::GetFontSize();
            ImGui::Dummy({0, 0});
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 pos = ImGui::GetCursorScreenPos();
            dl->AddText(fo, tFS, pos, ImGui::ColorConvertFloat4ToU32(cGray100), title);
            ImGui::Dummy({0, tFS + 16});
        };
        auto modalLabel = [&](const char* label) {
            ImGui::PushStyleColor(ImGuiCol_Text, cGray60);
            ImGui::TextUnformatted(label);
            ImGui::PopStyleColor();
            ImGui::Dummy({0, 4});
        };
        auto modalButtons = [&](const char* cancelLabel, const char* confirmLabel) -> int {
            // 0=none, 1=cancel, 2=confirm
            ImGui::Dummy({0, 16});
            float bW = (ImGui::GetContentRegionAvail().x - 8) / 2;
            float bH = 48;
            float tH = ImGui::GetFontSize();
            float padY = (bH - tH) / 2;
            int result = 0;
            // Cancel: outline
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 1);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {0, padY});
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(1,1,1,1));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, cGray5);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, cGray10);
            ImGui::PushStyleColor(ImGuiCol_Text, cGray90);
            ImGui::PushStyleColor(ImGuiCol_Border, cGray20);
            if (ImGui::Button(cancelLabel, {bW, bH})) result = 1;
            ImGui::PopStyleColor(5);
            ImGui::PopStyleVar(3);
            ImGui::SameLine(0, 8);
            // Confirm: gray100 filled
            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 8);
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {0, padY});
            ImGui::PushStyleColor(ImGuiCol_Button, cGray100);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, cGray90);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, cGray90);
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1,1,1,1));
            if (ImGui::Button(confirmLabel, {bW, bH})) result = 2;
            ImGui::PopStyleColor(4);
            ImGui::PopStyleVar(3);
            return result;
        };

        // ─── 씬 저장하기 ────────────────────────────────────────
        if (modalBegin("씬 저장하기")) {
            modalTitle("씬 저장하기");
            modalLabel("경로");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputText("##savePath", scenePathBuf, sizeof(scenePathBuf));
            int r = modalButtons("취소", "저장");
            if (r == 2) {
                std::string err;
                simulator.saveScene(scenePathBuf, &err);
                ImGui::CloseCurrentPopup();
            } else if (r == 1) ImGui::CloseCurrentPopup();
            modalEnd();
        }

        // (씬 불러오기는 NFD 네이티브 다이얼로그로 대체됨 — 위 openLoadModal 처리)

        // ─── 3D 모델 파일 가져오기 ──────────────────────────────────
        if (modalBegin("3D 모델 파일 가져오기")) {
            modalTitle("3D 모델 파일 가져오기");
            modalLabel("파일");
            ImGui::TextWrapped("%s", importPathBuf[0] ? importPathBuf : "(파일 미선택)");
            ImGui::Dummy({0, 12});
            modalLabel("배율");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputFloat("##impScale", &importScale);
            int r = modalButtons("취소", "가져오기");
            if (r == 2) {
                std::string path = importPathBuf, prefix, file;
                auto slash = path.find_last_of('/');
                if (slash != std::string::npos) { prefix = path.substr(0, slash); file = path.substr(slash+1); }
                else file = path;
                std::string err;
                if (simulator.importModel(prefix, file, (Precision)importScale, Precision(0.1), &err, BehaviorType::Rigid)) {
                    simulator.initialize(); simulator.applyPendingMaterials();
                }
                ImGui::CloseCurrentPopup();
            } else if (r == 1) ImGui::CloseCurrentPopup();
            modalEnd();
        }

        // ─── Primitive modals (구/정육면체/원기둥) ──────────────
        auto primitiveModal = [&](const char* title, int kind) {
            if (modalBegin(title)) {
                modalTitle(title);
                modalLabel("크기");
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat("##primSz", &primSize);
                ImGui::Dummy({0, 12});
                modalLabel("분할 수");
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputInt("##primTess", &primTess);
                ImGui::Dummy({0, 12});
                modalLabel("위치");
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat3("##primPos", primPos);
                int r = modalButtons("취소", "생성");
                if (r == 2) {
                    int t = primTess;
                    tinym::vec3 c(primPos[0], primPos[1], primPos[2]);
                    if (kind == 0) { if (t < 3) t = 3; simulator.addSphere(c, (Index)t, (Precision)primSize, (Precision)0.1, BehaviorType::Rigid); }
                    else if (kind == 1) { if (t < 1) t = 1; simulator.addCube(c, (Index)t, (Precision)primSize, (Precision)0.1, BehaviorType::Rigid); }
                    else { if (t < 3) t = 3; simulator.addCylinder(c, (Index)t, (Precision)primSize, (Precision)0.1, BehaviorType::Rigid); }
                    simulator.initialize();
                    ImGui::CloseCurrentPopup();
                } else if (r == 1) ImGui::CloseCurrentPopup();
                modalEnd();
            }
        };
        primitiveModal("구 생성", 0);
        primitiveModal("정육면체 생성", 1);
        primitiveModal("원기둥 생성", 2);

        // ─── 평면 생성 ──────────────────────────────────────────
        if (modalBegin("평면 생성")) {
            modalTitle("평면 생성");
            modalLabel("방향");
            const char* dirNames[] = { "XY 평면", "YZ 평면", "XZ 평면 (바닥)" };
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::Combo("##planeDir", &planeDirIdx, dirNames, 3);
            ImGui::Dummy({0, 12});
            modalLabel("크기");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputFloat("##planeSz", &planeSize);
            ImGui::Dummy({0, 12});
            modalLabel("분할 수");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputInt("##planeTess", &planeTess);
            ImGui::PushStyleColor(ImGuiCol_Text, cGray50);
            ImGui::TextWrapped("분할 수를 높이고 소재를 옷감으로 바꾸면 천처럼 시뮬레이션됩니다.");
            ImGui::PopStyleColor();
            ImGui::Dummy({0, 12});
            modalLabel("위치");
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputFloat3("##planePos", planePos);
            int r = modalButtons("취소", "생성");
            if (r == 2) {
                PlaneDirection dir = PlaneDirection::XZPlane;
                if (planeDirIdx == 0) dir = PlaneDirection::XYPlane;
                else if (planeDirIdx == 1) dir = PlaneDirection::YZPlane;
                float s = planeSize > 0 ? planeSize : 1;
                int pn = planeTess < 2 ? 2 : planeTess;
                simulator.addPlane(dir, tinym::vec3(planePos[0], planePos[1], planePos[2]), (Index)pn, (Precision)s);
                simulator.initialize();
                ImGui::CloseCurrentPopup();
            } else if (r == 1) ImGui::CloseCurrentPopup();
            modalEnd();
        }

        // Real-time sync: the integrator's h is fixed (1/60), but the render
        // loop is vsync-paced (60/120/144Hz displays all occur), so stepping
        // once per render frame makes simulated time run faster or slower
        // than the wall clock. Accumulate wall dt and run floor(acc/h) fixed
        // steps per render frame instead (kinematic playback rides sim time,
        // so motion speed stays correct too). Hitch/debt clamp at 4 steps —
        // beyond that the debt is dropped (slow-motion beats a death spiral).
        // While paused the accumulator drains and a single update() call
        // keeps the dirty-init path responsive (it early-returns on pause).
        static double simStepAccum = 0.0;
        static double lastWallTime = -1.0;
        {
            if (lastWallTime < 0.0) lastWallTime = currentTime;
            double wallDt = currentTime - lastWallTime;
            lastWallTime = currentTime;
            if (wallDt < 0.0) wallDt = 0.0;
            if (wallDt > 0.25) wallDt = 0.25;

            int steps = 1;
            if (realtimeSimSync && !simulator.pause) {
                simStepAccum += wallDt;
                const double hSec = (double)system.h;
                steps = (int)(simStepAccum / hSec);
                if (steps > 4) { steps = 4; simStepAccum = 0.0; }
                else simStepAccum -= (double)steps * hSec;
            } else {
                simStepAccum = 0.0;
            }

            if (collectProfileFrame) {
                auto scope = frameProfiler.scoped("physics_total");
                for (int s = 0; s < steps; ++s) simulator.update();
            } else {
                for (int s = 0; s < steps; ++s) simulator.update();
            }
            // One-shot preview playback advances on render wall time, NOT the
            // physics step — it animates while the sim is paused and dies at
            // the clip end.
            simulator.advancePreviewPlayback(wallDt);
        }
        simulator.uploadMeshes();

        // ─── Pass 0: directional shadow map (offscreen) ───────────────
        // Renders scene depth from the light's view into shadowTex.
        // Light direction matches shader.frag's lightPosition default
        // (50, 50, 30); the ortho box covers the authoring area around
        // the origin — fragments outside it sample as lit (border=1).
        tinym::mat4 lightVP(1);
        if (shadowOk && shadowsEnabled) {
            const tinym::vec3 lightDir =
                tinym::vec3(50.0f, 50.0f, 30.0f).normalize();
            const float R = 8.0f, zn = 1.0f, zf = 80.0f;
            tinym::mat4 lightView = tinym::lookAt(
                lightDir * 40.0f, tinym::vec3(0, 0, 0), tinym::vec3(0, 1, 0));
            tinym::mat4 lightProj(
                tinym::vec4(1.0f / R, 0.0f, 0.0f, 0.0f),
                tinym::vec4(0.0f, 1.0f / R, 0.0f, 0.0f),
                tinym::vec4(0.0f, 0.0f, -2.0f / (zf - zn), 0.0f),
                tinym::vec4(0.0f, 0.0f, -(zf + zn) / (zf - zn), 1.0f));
            lightVP = lightProj * lightView;

            glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo);
            glViewport(0, 0, kShadowRes, kShadowRes);
            glEnable(GL_DEPTH_TEST);
            glClear(GL_DEPTH_BUFFER_BIT);
            shadowShader.use();
            shadowShader.setUniform("LightVP", lightVP);
            simulator.drawDepth();
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }
        auto applyShadowUniforms = [&]() {
            // shadowMap must ALWAYS sit on its own unit: a sampler2DShadow
            // sharing unit 0 with the idBuffer sampler2D makes every draw
            // GL_INVALID_OPERATION on macOS even when neither is sampled.
            shader.setUniform("shadowMap", 2);
            if (!shadowOk || !shadowsEnabled) {
                shader.setUniform("shadowsOn", 0);
                return;
            }
            glActiveTexture(GL_TEXTURE2);
            glBindTexture(GL_TEXTURE_2D, shadowTex);
            shader.setUniform("LightVP", lightVP);
            shader.setUniform("shadowsOn", 1);
        };

        // Camera-follow: lock the orbit pivot to a kinematic body's live
        // animated root before any pass reads camera.lookAt(), so the whole
        // frame (id / shadow / main) shares the followed view. Orbit + zoom
        // still work (theta/phi/fovy untouched); only the pan pivot is
        // overridden. Auto-clears if the followed mesh is gone.
        if (simulator.cameraFollowObjId >= 0) {
            tinym::vec3 followRoot;
            if (simulator.kinematicRootWorldPos(simulator.cameraFollowObjId, followRoot)) {
                camera.look = followRoot;
                camera.rotatePosition();
            } else {
                simulator.cameraFollowObjId = -1;
            }
        }

        // ─── Pass 1: id buffer (offscreen) ────────────────────────────
        // Paints each mesh's id into idTex (R32I). Sampled later by
        // shader.frag's outline check and (asynchronously, last-frame
        // value) by the cursor callback's glReadPixels. Skipped if the
        // id shader failed to load — outline becomes invisible but the
        // scene still renders normally.
        // Exactly ONE off-screen id pass runs, branched on the active
        // SelectionMode (requirement 3):
        //  · Object: triangle pass → .r = meshId, .g = depth. Drives
        //    the whole-mesh hover/outline (unchanged behavior).
        //  · Point: depth-only triangle pre-pass (glColorMask off, so
        //    non-point pixels keep the clear .r=-1/.b=-1 sentinel) then
        //    a GL_POINTS pass (size 9, GL_LEQUAL) → .r = meshId,
        //    .g = depth, .b = vertexId. The depth pre-pass makes the
        //    point pass occlusion-correct ("보이는 것만").
        const int idW = yglwindow->width();
        const int idH = yglwindow->height();
        const bool pointMode = (simulator.selectionMode == SelectionMode::Point);
        if (idShaderOk && idW > 0 && idH > 0) {
            ensureIdFbo(idW, idH);
            glBindFramebuffer(GL_FRAMEBUFFER, idFbo);
            glViewport(0, 0, idW, idH);
            const GLfloat clearObj[4]   = {-1.0f, 1.0f,  0.0f, 0.0f};
            const GLfloat clearPoint[4] = {-1.0f, 1.0f, -1.0f, 0.0f};
            glClearBufferfv(GL_COLOR, 0, pointMode ? clearPoint : clearObj);
            glClear(GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            tinym::mat4 Mid(1);
            tinym::mat4 Vid = camera.lookAt();
            tinym::mat4 Pid = camera.perspective(yglwindow->aspect(), 0.1f, 1000.f);
            if (pointMode && idPointShaderOk) {
                // Depth-only triangle pre-pass: occlude back-facing /
                // hidden vertices without touching the color channels.
                glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
                idShader.use();
                idShader.setUniform("M", Mid);
                idShader.setUniform("V", Vid);
                idShader.setUniform("P", Pid);
                simulator.drawIds(idShader);
                glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
                // Visible-vertex point pass. Points lie exactly on the
                // surface depth → GL_LEQUAL so they are not culled by
                // their own triangles.
                idPointShader.use();
                idPointShader.setUniform("M", Mid);
                idPointShader.setUniform("V", Vid);
                idPointShader.setUniform("P", Pid);
                // Intentionally larger than the on-screen dot
                // (drawSelectablePoints kDot=5): the off-screen id
                // buffer renders fat points so the pick hit-area is
                // forgiving — easier to click a vertex than the small
                // visible dot would suggest.
                glPointSize(20.0f);
                glDepthFunc(GL_LEQUAL);
                simulator.drawPointIds(idPointShader);
                glDepthFunc(GL_LESS);
                glPointSize(1.0f);
            } else {
                idShader.use();
                idShader.setUniform("M", Mid);
                idShader.setUniform("V", Vid);
                idShader.setUniform("P", Pid);
                simulator.drawIds(idShader);
            }
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
        }

        // Outline-uniform application helper, shared by both render
        // branches below. Binds idTex to texture unit 1 (unit 0 is
        // free for any future material texture). hoveredId/selectedId
        // are forwarded straight from Simulator state.
        auto applyOutlineUniforms = [&]() {
            if (!idShaderOk) {
                shader.setUniform("hoveredId", -1);
                shader.setUniform("selectedId", -1);
                return;
            }
            glActiveTexture(GL_TEXTURE1);
            glBindTexture(GL_TEXTURE_2D, idTex);
            shader.setUniform("idBuffer", 1);
            // Whole-mesh outline only in Object mode. In Point mode the
            // id buffer holds vertex ids, not a mesh-silhouette field,
            // so suppress the outline and let the point overlay carry
            // the hover/select feedback instead.
            if (pointMode) {
                shader.setUniform("hoveredId", -1);
                shader.setUniform("selectedId", -1);
            } else {
                shader.setUniform("hoveredId",  simulator.hoveredObj);
                shader.setUniform("selectedId", simulator.selectedObj);
            }
        };

        // Point-overlay helper (shared by both render branches). Drawn
        // AFTER the lit mesh so the dots read on top; depth-tested so
        // dots on hidden geometry are occluded by the solid surface.
        auto drawPointOverlay = [&](const tinym::mat4& Vp,
                                    const tinym::mat4& Pp) {
            if (!pointMode || !pointShaderOk) return;
            pointShader.use();
            tinym::mat4 Mp(1);
            pointShader.setUniform("M", Mp);
            pointShader.setUniform("V", Vp);
            pointShader.setUniform("P", Pp);
            glEnable(GL_DEPTH_TEST);
            simulator.drawSelectablePoints(pointShader);
            shader.use();
        };

        if (collectProfileFrame) {
            auto scope = frameProfiler.scoped("render_total");

            shader.use();
            glViewport(0, 0, yglwindow->width(), yglwindow->height());
            {
                const auto& bg = Scene<Backend, Precision>::environment.backgroundColor;
                glClearColor(bg.x, bg.y, bg.z, 1.0f);
            }
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            tinym::mat4 M(1);
            tinym::mat4 V = camera.lookAt();
            shader.setUniform("M", M);
            shader.setUniform("V", V);
            tinym::mat4 P = camera.perspective(yglwindow->aspect(), 0.1f, 1000.f);
            shader.setUniform("P", P);
            auto w = yglwindow->width()/2;
            auto h = yglwindow->height()/2;
            tinym::mat4 viewport = tinym::mat4(
                    tinym::vec4(w,0.0f,0.0f,0.0f),
                    tinym::vec4(0.0f,h,0.0f,0.0f),
                    tinym::vec4(0.0f,0.0f,1.0f,0.0f),
                    tinym::vec4(w+0, h+0, 0.0f, 1.0f));
            shader.setUniform("ViewportMatrix", viewport);
            shader.setUniform("lightColor",
                Scene<Backend, Precision>::environment.lightColor
                * Scene<Backend, Precision>::environment.lightIntensity);
            applyOutlineUniforms();
            applyShadowUniforms();

            {
                // scene_draw / debug_draw are *in-frame* sub-sections: timed
                // only at InFrame. At PerFrame they fold into render_total (the
                // default-constructed ScopedTimer is an inert no-op).
                profiler::FrameProfiler::ScopedTimer drawScope;
                if (inFrameLevel) drawScope = frameProfiler.scoped("scene_draw");
                simulator.draw(shader);
                drawPointOverlay(V, P);
            }

            {
                profiler::FrameProfiler::ScopedTimer debugScope;
                if (inFrameLevel) debugScope = frameProfiler.scoped("debug_draw");
                if(debugEachBoxes) {
                    simulator.debugEachBoxes(V, P);
                }
                if(debugSceneBox) {
                    simulator.debugSceneBox(V, P);
                }

                if(debugCollisions) {
                    simulator.debugCollisions(V, P);
                }
            }
        } else {
            shader.use();
            glViewport(0, 0, yglwindow->width(), yglwindow->height());
            {
                const auto& bg = Scene<Backend, Precision>::environment.backgroundColor;
                glClearColor(bg.x, bg.y, bg.z, 1.0f);
            }
            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            tinym::mat4 M(1);
            tinym::mat4 V = camera.lookAt();
            shader.setUniform("M", M);
            shader.setUniform("V", V);
            tinym::mat4 P = camera.perspective(yglwindow->aspect(), 0.1f, 1000.f);
            shader.setUniform("P", P);
            auto w = yglwindow->width()/2;
            auto h = yglwindow->height()/2;
            tinym::mat4 viewport = tinym::mat4(
                    tinym::vec4(w,0.0f,0.0f,0.0f),
                    tinym::vec4(0.0f,h,0.0f,0.0f),
                    tinym::vec4(0.0f,0.0f,1.0f,0.0f),
                    tinym::vec4(w+0, h+0, 0.0f, 1.0f));
            shader.setUniform("ViewportMatrix", viewport);
            shader.setUniform("lightColor",
                Scene<Backend, Precision>::environment.lightColor
                * Scene<Backend, Precision>::environment.lightIntensity);
            applyOutlineUniforms();
            applyShadowUniforms();

            simulator.draw(shader);
            drawPointOverlay(V, P);

            if(debugEachBoxes) {
                simulator.debugEachBoxes(V, P);
            }
            if(debugSceneBox) {
                simulator.debugSceneBox(V, P);
            }

            if(debugCollisions) {
                simulator.debugCollisions(V, P);
            }
            simulator.showDebugLines(V, P);
        }

        // mesh_inspector::drawMeshInspectorWindow already ran above as the
        // right-side Object panel. Only the profiler window is drawn here
        // at end-of-frame (it owns its own toggle / floating placement).
        // 씬 카운트(meshes / points / triangles) — 프로파일러 표시용.
        // packedMeshData가 아닌 realized GeneralMesh 컬렉션을 쓰는 이유:
        // 사용자가 본 화면에 실제 떠 있는 메시만 집계하기 위함이다.
        profiler::SceneCounts sceneCounts;
        sceneCounts.meshes = (int)Scene<Backend, Precision>::meshes.size();
        for (const auto& m : Scene<Backend, Precision>::meshes) {
            sceneCounts.points    += (int)(m.state.x.size / 3);
            sceneCounts.triangles += (int)(m.adjacency.facets.size / 3);
        }
        if (inFrameLevel) {
            auto imguiScope = frameProfiler.scoped("imgui_draw");
            profiler::drawProfilerWindow(
                profilerWindowState,
                frameProfiler,
                &simulator.pause,
                &debugEachBoxes,
                &debugSceneBox,
                &debugCollisions,
                &meshInspectorWindowState.open,
                sceneCounts,
                &simulator.useSegmentedBVHQuery,
                &simulator.collisionPipeline.broadPhase.useAgglomerative,
                &simulator.collisionPipeline.broadPhase.enableRefit,
                &simulator.useSpatialHashing,
                &simulator.refitSubstepPeriod,
                &simulator.cdSubstepPeriod,
                &profileLevelUi,
                &simulator.collisionPipeline.broadPhase.fusedRefitEnlarge,
                &simulator.clusterModeOn,   // master cluster mode (per-object s in inspector)
                nullptr,                    // split-s slider removed — now per-object
                &simulator.useMultiLevelSH
            );
            scene_log::drawSceneActionLogWindow(sceneLogWindowState);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        } else {
            profiler::drawProfilerWindow(
                profilerWindowState,
                frameProfiler,
                &simulator.pause,
                &debugEachBoxes,
                &debugSceneBox,
                &debugCollisions,
                &meshInspectorWindowState.open,
                sceneCounts,
                &simulator.useSegmentedBVHQuery,
                &simulator.collisionPipeline.broadPhase.useAgglomerative,
                &simulator.collisionPipeline.broadPhase.enableRefit,
                &simulator.useSpatialHashing,
                &simulator.refitSubstepPeriod,
                &simulator.cdSubstepPeriod,
                &profileLevelUi,
                &simulator.collisionPipeline.broadPhase.fusedRefitEnlarge,
                &simulator.clusterModeOn,   // master cluster mode (per-object s in inspector)
                nullptr,                    // split-s slider removed — now per-object
                &simulator.useMultiLevelSH
            );
            scene_log::drawSceneActionLogWindow(sceneLogWindowState);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        // Apply a just-toggled profiler "Cluster mode" checkbox to the
        // BroadPhase (no-op when unchanged); the next update rebuilds.
        simulator.reconcileClusterMode();

        // Close before the window-title read below — title reads
        // history().latestFrame() and needs endFrame() to have run.
        frameGate.close();

        // Once profileFrames are collected, dump the per-section CSV + a
        // scene sidecar (req 4: the exact RunConfig that produced it) and ask
        // the window to close so the run exits.
        static bool ysimProfileDone = false;
        if (!ysimProfileDone && profileActive
            && simulator.frame >= profileFrames) {
            frameProfiler.history().exportCsv(profileCsvPath);  // creates dirs
            sim_config::RunConfig outCfg = runConfig;
            if (!haveRunConfig) outCfg.scene = simulator.toSnapshot();
            outCfg.profile.enabled      = true;
            outCfg.profile.frames       = profileFrames;
            outCfg.profile.realtimeSync = profileRealtimeSync;
            outCfg.profile.outputPath   = profileCsvPath;
            outCfg.profile.level        = activeProfileLevel;  // record detail tier
            const std::string sidecar = sim_config::sidecarScenePath(profileCsvPath);
            std::string serr;
            bool sok = sim_config::writeToFile(outCfg, sidecar, &serr);
            std::cout << "[profile] wrote " << profileCsvPath << " ("
                      << frameProfiler.history().frames().size() << " frames) + "
                      << (sok ? sidecar : ("sidecar FAILED: " + serr)) << "\n";
            ysimProfileDone = true;
            glfwSetWindowShouldClose(yglwindow->getGLFWWindow(), GLFW_TRUE);
        }

        if (const auto* latest = frameProfiler.history().latestFrame()) {
            char title[256];
            std::snprintf(
                title,
                sizeof(title),
                "ysim | FPS: %.1f | Frame: %.2f ms",
                latest->fps,
                latest->frame_ms
            );
            glfwSetWindowTitle(yglwindow->getGLFWWindow(), title);
        }
    };


    yglwindow->mainLoop(init, render);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
