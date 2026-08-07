#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

enum struct ShapeType : Index {
    Mesh,     // general triangle soup — per-particle/triangle collision
    Plane,    // reserved analytic plane (not yet auto-assigned; see (c))
    Sphere,
    Cube,
    Cylinder,
};

// Per-mesh collider representation (docs/design/collider_pipeline_rework.md
// §1, phase P0). Distinct axis from ShapeType: ShapeType classifies the
// mesh's SOURCE primitive (what the initializer built), colliderKind is
// what the collision pipeline is asked to TREAT it as — the user can point
// a Sphere-sourced mesh at Mesh collision, or declare an imported prop a
// Box. The acceleration axis stays fixed in v1 (Mesh → BVH, self → SH), so
// this enum carries geometry only.
//
// Plain sequential values, no bit packing: §6 reserves an appended Convex
// slot (GJK/EPA or baked SDF) for arbitrary props, so the kind count must
// stay free to grow. Persisted by NAME (colliderKindJson), never by
// ordinal, so appending is schema-safe.
//
// P1: this IS the collision pipeline's shape identity. BVH::objCollider,
// BroadCollision/NarrowCollision::shapePair and AnalyticShape::kind all
// carry ColliderKind values (NOT ShapeType — the two numberings differ).
// Keep the metal-side YSIM_COLLIDER_* defines in sync.
enum struct ColliderKind : uint32_t {
    Mesh = 0,   // triangle soup (BVH) — grid / imported file default
    Sphere,     // analytic sphere / ellipsoid under non-uniform scale
    Box,        // OBB
    Cylinder,
    Plane,      // infinite half-space, local +Y up (finite floor = Box)
};

// Picking mode. Object: id-buffer triangle pass → whole-mesh hover/
// select + outline. Point: id-buffer GL_POINTS pass → per-vertex
// hover/select with on-screen dots. Exactly one off-screen id pass
// runs per frame, branched on this (requirement 3).
enum struct SelectionMode : int {
    Object,
    Point,
};

// A pinned vertex constraint. `vid` is the PHYSICS vertex index within
// its mesh (constraints.fixedParticles[vid] = 0). `pos` is the world
// position the vertex is held at. Stored on RequestGeneralMesh (so it
// survives Scene::pack — like applyGravity/rotationQuat mirrors) and
// round-tripped through scene_format as a per-object constraint list.
struct FixedVertex {
    uint32_t vid;
    tinym::vec3 pos;
};

// A reference-point coincidence constraint (point-selection panel).
// Expressed via IndexPair per the design: the vertex `vertexPair.query`
// of the object whose id is `objPair.query` (the FOLLOWER) must track the
// position of vertex `vertexPair.target` of the object `objPair.target`
// (the LEADER) every integration step. `.query`/`.target` of vertexPair
// are PHYSICS vertex ids (same space as FixedVertex::vid). Stored in a
// Scene-static list that survives Scene::pack/reset and round-trips
// through scene_format.
struct ReferencePointConstraint {
    IndexPair objPair;
    IndexPair vertexPair;
};

// A spline-follow dynamic constraint (point panel "경로 따라가기").
// The vertex `vid` (PHYSICS index, same space as FixedVertex::vid) is
// held (fixedParticles[vid] = 0) and its held position is driven along
// a Catmull-Rom path through `points` (world space, >= 2 to be active).
// closed: the path wraps and playback loops forever; open: playback
// clamps at the last point and stops (`playing` flips false). Stored on
// RequestGeneralMesh (pack-surviving) and round-tripped through
// scene_format; localTime/playing are runtime playback state and reset
// on load.
struct SplineConstraint {
    uint32_t vid = 0;
    bool closed = false;
    float duration = 4.0f;            // seconds per full traversal
    std::vector<tinym::vec3> points;  // control points, world space
    double localTime = 0.0;
    bool playing = true;
};

const char* behaviorTypeName(BehaviorType behaviorType) {
    switch (behaviorType) {
        case BehaviorType::TriangularCloth: return "TriangularCloth";
        case BehaviorType::FastGridCloth: return "FastGridCloth";
        case BehaviorType::Elastic: return "Elastic";
        case BehaviorType::Rigid: return "Rigid";
        case BehaviorType::Float: return "Float";
        case BehaviorType::Fluid: return "Fluid";
        case BehaviorType::Generator: return "Generator";
        case BehaviorType::Kinematic: return "Kinematic";
        default: return "Unknown";
    }
}

// BVH motion asset directory — external asset, referenced from the project
// source tree (no longer copied into the build). ysim_paths::assetFile
// honors the YSIM_ASSET_ROOT / project-root resolution.
inline std::string bvhAssetDir() {
    return ysim_paths::assetFile("BVH");
}

// Sorted *.bvh file names (no directory) in bvhAssetDir().
inline std::vector<std::string> listBVHFiles() {
    namespace fs = std::filesystem;
    std::vector<std::string> out;
    std::error_code ec;
    for (const auto& e : fs::directory_iterator(bvhAssetDir(), ec)) {
        if (!e.is_regular_file()) continue;
        if (e.path().extension() != ".bvh") continue;
        out.push_back(e.path().filename().string());
    }
    std::sort(out.begin(), out.end());
    return out;
}

// Curated blend-space presets, distilled from the `--blend-spaces` ranking
// (opposed axes that are also near-orthogonal). Each is 4 files in
// blendSpaceCoords order {bottom(Y-), top(Y+), left(X-), right(X+)}; all four
// share the WalkLoopA skeleton group. The blend-space mode's preset dropdown
// offers these plus "자율선택" (manual). MBS-7 verifies each builds with all 4.
struct BlendPreset {
    const char* name;
    std::array<const char*, 4> files;
    // Per-clip active frame window {start,end}; end<0 = full. Default = all full
    // (presets that omit it blend whole clips).
    std::array<std::array<int, 2>, 4> ranges{{{0, -1}, {0, -1}, {0, -1}, {0, -1}}};
};
inline const std::vector<BlendPreset>& blendPresets() {
    static const std::vector<BlendPreset> kPresets = {
        {"정지 ↔ 질주 · 살금 ↔ 활보",
         {"standReallyStill.bvh", "jogCurve.bvh", "SneakLoopA.bvh", "StrutLoopA.bvh"}},
        {"걷기 ↔ 질주 · 살금 ↔ 활보",
         {"WalkLoopA.bvh", "jogCurve.bvh", "SneakLoopA.bvh", "StrutLoopA.bvh"}},
        {"걷기 변형 (미세 블렌드)",
         {"smoothWalk.bvh", "walkStraightTwiceAsFast.bvh", "walkCurve.bvh",
          "walkReallySmooth.bvh"},
         {{{36, -1}, {0, -1}, {0, -1}, {35, -1}}}},  // trim the two long walks
        {"댄스 ↔ 정지 · 살금 ↔ 질주",
         {"standReallyStill.bvh", "DNCMODRNA.bvh", "SneakLoopA.bvh", "jogCurve.bvh"}},
    };
    return kPresets;
}

// Curated presets for the N-motion keytime blend (motionMode 6). Each captures
// a full mode-6 setup: the motion files, per-motion colors, loop-extend flags,
// active-frame windows, hand-tuned footstep keytimes (LFD,RFU,RFD,LFU,cycleEnd
// in clip-frame coords, AFTER any loop-extend), the adverb tag, and each
// motion's adverb percent. Applied by setKinematicVerbPreset. Keytimes are
// clamped to the built clip length, so a slightly different resample is safe.
struct VerbPreset {
    const char* name;
    std::vector<const char*> files;
    std::vector<std::array<float, 3>> colors;
    std::vector<int> loopSel;                   // per motion: 1 = loop-extend (2×)
    std::vector<std::array<int, 2>> ranges;     // per motion {start,end}; end<0 full
    std::vector<std::array<int, 5>> keys;       // per motion keytimes
    std::vector<const char*> tags;              // 1..2 adverb names
    std::vector<std::array<float, 2>> adverbs;  // [motion][tag] percent (tag1 = 0)
    bool extrapolate;                           // 외삽 허용
};
inline const std::vector<VerbPreset>& verbPresets() {
    static const std::vector<VerbPreset> kPresets = {
        {"속도: 살금·걷기·조깅",
         {"SneakLoopA.bvh", "WalkLoopA.bvh", "walkToJog.bvh"},
         {{0.95f, 0.25f, 0.25f}, {0.30f, 0.45f, 0.95f}, {0.30f, 0.85f, 0.40f}},
         {1, 1, 0},
         {{30, 175}, {0, -1}, {35, 80}},
         {{54, 67, 97, 106, 142}, {19, 21, 36, 37, 50}, {49, 51, 61, 62, 70}},
         {"speed"},
         {{0, 0}, {50, 0}, {100, 0}},
         true},
        {"곡선 × 조깅 (4모션·2태그)",
         {"jogCurve.bvh", "walkCurve.bvh", "walkToJog.bvh", "WalkLoopA.bvh"},
         {{0.95f, 0.25f, 0.25f},
          {0.30f, 0.45f, 0.95f},
          {0.30f, 0.85f, 0.40f},
          {0.95f, 0.78f, 0.25f}},
         {0, 0, 0, 1},
         {{13, 76}, {54, 165}, {0, -1}, {0, -1}},
         {{20, 21, 32, 33, 44},
          {60, 62, 78, 79, 92},
          {49, 51, 61, 62, 70},
          {19, 21, 36, 37, 50}},
         {"curvy", "jog"},
         {{85, 85}, {85, 15}, {0, 85}, {0, 15}},
         true},
    };
    return kPresets;
}

const char* shapeTypeName(ShapeType shapeType) {
    switch (shapeType) {
        case ShapeType::Mesh: return "Mesh";
        case ShapeType::Plane: return "Plane";
        case ShapeType::Sphere: return "Sphere";
        case ShapeType::Cube: return "Cube";
        case ShapeType::Cylinder: return "Cylinder";
        default: return "Unknown";
    }
}

// P1 pipeline predicate: everything that is NOT a triangle soup goes
// through the analytic narrow kernel (broad marker + skipAnalytic gate).
// Single definition so the broad phase, the pack-time AnalyticShape
// selection and the narrow dispatch can never disagree.
inline bool isAnalyticCollider(ColliderKind kind) {
    return kind != ColliderKind::Mesh;
}

// Display name (inspector combo / logs), mirroring shapeTypeName.
inline const char* colliderKindName(ColliderKind kind) {
    switch (kind) {
        case ColliderKind::Mesh: return "Mesh";
        case ColliderKind::Sphere: return "Sphere";
        case ColliderKind::Box: return "Box";
        case ColliderKind::Cylinder: return "Cylinder";
        case ColliderKind::Plane: return "Plane";
        default: return "Unknown";
    }
}

// Persistence token (scene_format "collider_kind"). Lowercase, stable
// across enum reorderings — the JSON never stores the ordinal.
inline const char* colliderKindJson(ColliderKind kind) {
    switch (kind) {
        case ColliderKind::Mesh: return "mesh";
        case ColliderKind::Sphere: return "sphere";
        case ColliderKind::Box: return "box";
        case ColliderKind::Cylinder: return "cylinder";
        case ColliderKind::Plane: return "plane";
        default: return "mesh";
    }
}

// Parse a persistence token. Returns false for an unknown/empty token so
// the caller keeps the initializer-derived default (backward compat: a
// scene saved before P0 simply has no key).
inline bool colliderKindFromJson(const std::string& name, ColliderKind& out) {
    if (name == "mesh")     { out = ColliderKind::Mesh;     return true; }
    if (name == "sphere")   { out = ColliderKind::Sphere;   return true; }
    if (name == "box")      { out = ColliderKind::Box;      return true; }
    if (name == "cylinder") { out = ColliderKind::Cylinder; return true; }
    if (name == "plane")    { out = ColliderKind::Plane;    return true; }
    return false;
}

struct alignas(32) BroadCollision {
    IndexPair indexPair;
    IndexPair objPair;
    IndexPair behaviorPair;
    IndexPair shapePair;
};

// Cluster-pair broad phase (cluster VF pipeline) — MUST match bvh.metal layout.
struct ClusterPair { uint32_t a; uint32_t b; };   // (query cluster, target cluster)
struct ClusterGridParams {
    float ox, oy, oz;     // grid origin (min corner)
    float cellSize;       // uniform cell size (~ max cluster extent)
    int   dx, dy, dz;     // grid dims
    uint32_t numQuery;    // # query clusters
    uint32_t maxPairs;    // pair buffer capacity
    float margin;         // inflate query AABBs (VF needs margin-expanded pairs)
};

struct NarrowCollision {
    IndexPair indexPair;
    IndexPair objPair;
    tinym::vec4 collisionNormalAndDistance;
    IndexPair behaviorPair;
    IndexPair shapePair;
};

// Per-collider analytic descriptor (collider_pipeline_rework.md P1).
// COMPACT array (count = #meshes whose colliderKind != Mesh, NOT
// numMeshes): each entry self-describes via objIndex. Allocated at
// Scene::pack, contents refilled from the live mesh transform each
// frame (D3). GPU-visible; vec4-packed so the analytic narrow kernel
// reads it without padding surprises — MUST stay byte-identical to
// AnalyticShape in src/metal/common_types.metalh.
//
// prevCenterPad.xyz / prevRotQuat carry the shape transform at the
// START of the step the CCD segment spans (rolled forward by
// Scene::refreshAnalyticShapes). The narrow kernel inverts that motion
// into the vertex's segment — see collider_pipeline_rework.md §3 —
// instead of building swept volumes.
struct alignas(16) AnalyticShape {
    tinym::vec4 centerRadius;   // xyz = world center, w = radius (sphere/cyl)
    tinym::vec4 halfExtHeight;  // xyz = half-extents (box/ellipsoid), w = cyl half-height
    tinym::vec4 rotQuat;        // world orientation (w,x,y,z), local→world
    tinym::vec4 prevCenterPad;  // xyz = prev-step center; w = pad
    tinym::vec4 prevRotQuat;    // prev-step orientation (w,x,y,z)
    uint32_t kind;              // ColliderKind enum value
    uint32_t objIndex;          // owning mesh ARRAY INDEX — the objPair namespace (§4)
    uint32_t behaviorType;      // BehaviorType (cloth-vs-collider gate / future rigid-rigid)
    uint32_t flags;             // bit0 = collidable
};
// 5 float4 + 4 uint, 16-aligned — the metal mirror must agree byte for byte
// or the kernel reads garbage half-extents (silent wrong collisions).
static_assert(sizeof(AnalyticShape) == 96, "AnalyticShape must match the metal mirror");
static_assert(alignof(AnalyticShape) == 16, "AnalyticShape must stay 16-aligned");

template <typename BE, typename PR>
struct Constraints {
    VectorBase<BE, PR> fixedParticles;

    //Index maxNumCollisions = 0;
    ////Index numBroadCollisions = 0;
    //VectorBase<BE, Index> numBroadCollisions;
    //VectorBase<BE, Index> numNarrowCollisions;
    //Index approxColPerVertex = 15;
    //VectorBase<BE, BroadCollision> broadCollisions;
    //VectorBase<BE, NarrowCollision> narrowCollisions;

    //VectorBase<BE, NarrowCollision> vertexColPrims;
    //VectorBase<BE, Index> vertexColPrimsOffsets;

    private:

    public:
    void memoryAllocation(Index numPoints) {
        if(fixedParticles.ptr) return;
        fixedParticles = VectorBase<BE, PR>(numPoints, 1);

        //maxNumCollisions = numPoints * approxColPerVertex;
        //numBroadCollisions = VectorBase<BE, Index>(1);
        //numNarrowCollisions = VectorBase<BE, Index>(1);

        //broadCollisions = VectorBase<BE, BroadCollision>(maxNumCollisions);
        //narrowCollisions = VectorBase<BE, NarrowCollision>(maxNumCollisions);

        //vertexColPrims = VectorBase<BE, NarrowCollision>(maxNumCollisions);
        //vertexColPrimsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
    }

    void fixParticle(Index id) { fixedParticles[id] = PR(0); }
    void releaseParticle(Index id) { fixedParticles[id] = PR(1); }

};

template <typename PR>
struct ClothBehaviorParams {
    PR stretch, shear, bend;
    PR thickness;
    // PD continuum strain limiting band [σmin, σmax] (design doc §3.3):
    // the singular values of each triangle's deformation gradient are
    // clamped into this interval by the PD local step. 1/1 = corotated
    // (no free play — see the tunneling calibration in pd_system.hpp).
    // PER MESH, because it is a fabric property, not a solver setting;
    // the symplectic and PBD paths ignore it. Default member initializers
    // keep the aggregate init sites ({stretch, shear, bend, thickness})
    // compiling unchanged.
    PR sigmaMin = PR(1);
    PR sigmaMax = PR(1);
    // PBD tearing opt-OUT for THIS cloth. The global PbdSystem::tearEnabled
    // stays the master gate; this only decides whether a mesh participates
    // once tearing is on scene-wide, so a scene can hang one tearable rag
    // next to an indestructible curtain. Default true so every pre-existing
    // scene (and the PBT self-tests) behaves exactly as before.
    // PER MESH for the same reason as the sigma band: a fabric property,
    // not a solver setting. Turning it off stops FURTHER tearing only —
    // holes already torn stay torn (the tear state is still maintained).
    bool tearable = true;
};

// FastGridCloth rest lengths are PER DIRECTION (6 values), not 3 scalars.
// A single stretch/shear/bend scalar only holds for a uniformly scaled
// grid; a non-uniform scale (e.g. 3x on X, 1x on Y) gives the column
// edges, row edges, the two cell diagonals, and the two bend spans all
// different rest lengths. Grid layout (MeshGridInitializer, row-major,
// idx = row*pn1D + col):
//   stretchRestX : idx 0  -> idx 1            (col +1, horizontal edge)
//   stretchRestY : idx 0  -> idx pn1D         (row +1, vertical edge)
//   shearRestA   : idx 0  -> idx pn1D+1       ("\" diagonal, +row +col)
//   shearRestB   : idx 1  -> idx pn1D         ("/" diagonal, +row -col)
//   bendRestX    : idx 0  -> idx 2            (col +2, horizontal bend)
//   bendRestY    : idx 0  -> idx 2*pn1D       (row +2, vertical bend)
// Field order MUST stay byte-identical to physics.metal's ClothGridParams
// (sent verbatim via setBytes; all members are 4-byte scalars, no pad).
template <typename PR>
struct FastGridClothBehaviorParams {
    uint particleNum1D;
    PR stretchRestX, stretchRestY;
    PR shearRestA,   shearRestB;
    PR bendRestX,    bendRestY;
    PR kstretch, kshear, kbend;
    PR thickness;
};

template <typename PR>
struct FloatBehaviorParams {};

template <typename PR>
using BehaviorParams = std::variant<
    ClothBehaviorParams<PR>,
    FastGridClothBehaviorParams<PR>,
    FloatBehaviorParams<PR>
>;

// Derive FastGridCloth's 6 directional rest lengths from the LIVE grid
// geometry. FastGridCloth physics reads these scalars directly (the
// metal kernel routes each spring to its matching field), NOT the
// adjacency restEdgeLengths arrays — so recomputeRestLengths() alone
// leaves a scaled FastGrid sheet pre-stressed. Measuring from the
// post-transform coordinates makes a uniformly OR non-uniformly scaled
// sheet's rest config fall out naturally. Each value is only written
// when its sample indices are in range and the measured length is
// non-degenerate, so a collapsed axis can't zero out a rest length and
// a too-small grid leaves the constructed defaults intact.
template <typename PR>
inline void recomputeFastGridRest(const PR* x, Index numPoints,
                                  FastGridClothBehaviorParams<PR>& p) {
    uint pn = p.particleNum1D;
    if (pn < 2 || x == nullptr) return;
    auto dist = [&](Index a, Index b) {
        PR dx = x[a*3+0] - x[b*3+0];
        PR dy = x[a*3+1] - x[b*3+1];
        PR dz = x[a*3+2] - x[b*3+2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    auto set = [&](PR& dst, Index a, Index b) {
        if (a < numPoints && b < numPoints) {
            PR d = dist(a, b);
            if (d > PR(1e-9)) dst = d;
        }
    };
    const Index P = (Index)pn;
    set(p.stretchRestX, 0, 1);        // col +1
    set(p.stretchRestY, 0, P);        // row +1
    set(p.shearRestA,   0, P + 1);    // "\" diagonal
    set(p.shearRestB,   1, P);        // "/" diagonal
    set(p.bendRestX,    0, 2);        // col +2
    set(p.bendRestY,    0, 2 * P);    // row +2
}

//! Force accumulator
template <typename BE, typename PR>
struct TriangularClothBehavior {};

template <typename PR>
struct TriangularClothBehavior<METAL, PR> {

    static MTL::ComputePipelineState* getPSO() {
        static MTL::ComputePipelineState* pso = nullptr;
        if (!pso) {
            pso = MetalKernelContext::getPSO("compute_tri_spring_forces");
        }
        return pso;
    }

    template <typename SimParams>
    static void setBuffer(
            GeneralMesh<METAL, PR>& mesh,
            SimParams& simParams) {
        Index offset = 0;
        // state 0-3
        MetalGlobalContext::setBuffer(mesh.state.x, offset++);
        MetalGlobalContext::setBuffer(mesh.state.v, offset++);
        MetalGlobalContext::setBuffer(mesh.state.f, offset++);
        MetalGlobalContext::setBuffer(mesh.state.m, offset++);
        // constraints 4-6
        MetalGlobalContext::setBuffer(mesh.constraints.fixedParticles, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL,PR>::packedCollisionData.vertColFacets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL,PR>::packedCollisionData.vertColFacetsOffsets, offset++);
        // external forces 7
        MetalGlobalContext::setBuffer(mesh.externalForces.externalForces, offset++);
        //simulation parameters 8-9
        MetalGlobalContext::setBytes(simParams, offset++);
        {
            // "팽팽함": scale a COPY of the cloth stiffness so the
            // stored base params stay intact; only the simulated
            // stretch/shear/bend are multiplied.
            ClothBehaviorParams<PR> cp =
                std::get<ClothBehaviorParams<PR>>(mesh.behaviorParams);
            PR s = mesh.clothStiffnessScale;
            cp.stretch *= s; cp.shear *= s; cp.bend *= s;
            MetalGlobalContext::setBytes(cp, offset++);
        }
        // adjacency 10-11
        MetalGlobalContext::setBuffer(mesh.adjacency.edges, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.facets, offset++);
        // stretch springs 12-14
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdges, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdgesOffsets, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.restEdgeLengths, offset++);
        // bend springs 15-17
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVertices, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVerticesOffsets, offset++);
        MetalGlobalContext::setBuffer(mesh.adjacency.restOppLengths, offset++);
        // packed data
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedMeshData.statesOffsets, offset++);
        MetalGlobalContext::setBytes(mesh.id, offset++);
    }

    static void update(MeshState<METAL, PR>& state) {
        auto* pso = getPSO();
        size_t vertexNum = state.x.size/3;
        MetalGlobalContext::dispatchThreads(pso, vertexNum);
    }
};

template <typename BE, typename PR>
struct FastGridClothBehavior {};

template <typename PR>
struct FastGridClothBehavior<METAL, PR> {
    static MTL::ComputePipelineState* getPSO() {
        static MTL::ComputePipelineState* pso = nullptr;
        if (!pso) {
            pso = MetalKernelContext::getPSO("compute_cloth_grid_forces_fast");
        }
        return pso;
    }

    template <typename SimParams>
    static void setBuffer(
            GeneralMesh<METAL, PR>& mesh,
            SimParams& simParams) {
        Index offset = 0;
        // state 0-3
        MetalGlobalContext::setBuffer(mesh.state.x, offset++);
        MetalGlobalContext::setBuffer(mesh.state.v, offset++);
        MetalGlobalContext::setBuffer(mesh.state.f, offset++);
        MetalGlobalContext::setBuffer(mesh.state.m, offset++);
        // constraints 4-6
        MetalGlobalContext::setBuffer(mesh.constraints.fixedParticles, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrims, offset++);
        //MetalGlobalContext::setBuffer(mesh.constraints.vertexColPrimsOffsets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedCollisionData.vertColFacets, offset++);
        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets, offset++);
        // external forces 7
        MetalGlobalContext::setBuffer(mesh.externalForces.externalForces, offset++);
        //simulation parameters 8-9
        MetalGlobalContext::setBytes(simParams, offset++);
        {
            // "팽팽함": scale the k* stiffness (NOT the rest lengths).
            FastGridClothBehaviorParams<PR> fp =
                std::get<FastGridClothBehaviorParams<PR>>(mesh.behaviorParams);
            PR s = mesh.clothStiffnessScale;
            fp.kstretch *= s; fp.kshear *= s; fp.kbend *= s;
            MetalGlobalContext::setBytes(fp, offset++);
        }
        //// adjacency 10-11
        //MetalGlobalContext::setBuffer(mesh.adjacency.edges, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.facets, offset++);
        //// stretch springs 12-14
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdges, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexAdjEdgesOffsets, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.restEdgeLengths, offset++);
        //// bend springs 15-17
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVertices, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.vertexOppVerticesOffsets, offset++);
        //MetalGlobalContext::setBuffer(mesh.adjacency.restOppLengths, offset++);

        MetalGlobalContext::setBuffer(Scene<METAL, PR>::packedMeshData.statesOffsets, offset++);
        MetalGlobalContext::setBytes(mesh.id, offset++);
    }

    static void update(MeshState<METAL, PR>& state) {
        auto* pso = getPSO();
        size_t vertexNum = state.x.size/3;
        MetalGlobalContext::dispatchThreads(pso, vertexNum);
    }
};

