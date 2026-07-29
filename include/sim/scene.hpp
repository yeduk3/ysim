#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

struct GeneralMesh {
    int id;
    // D-026: never-reused identity for BVH skip-cache invalidation. Set
    // at Scene::pack realization from RequestGeneralMesh::lifetimeId.
    // Distinct from `id` (which is numMeshes-derived and resets on
    // resetScene; D-018 RNG seed). See CM-008 (graduated).
    int lifetimeId = -1;


    MeshState<BE, PR> state;
    MeshAdjacency<BE, PR> adjacency;
    GeneralMeshInitializer<BE, PR>* initializer;
    BehaviorType behaviorType;
    ShapeType shapeType = ShapeType::Mesh;
    BehaviorParams<PR> behaviorParams;
    Material material;
    Quat rotationQuat;
    // World-space center mirror for the inspector translate path (BDD-003).
    // Mutated only by Simulator::translateObject; pack-time seeded from the
    // initializer's center/offset so existing meshes preserve their author
    // intent. Persists through saveScene/loadScene via Simulator::toSnapshot.
    tinym::vec3 transformPosition = tinym::vec3(0);
    // Per-axis world-space scale mirror for the inspector scale path.
    // Mutated only by Simulator::scaleObject; carried through Scene::pack
    // via RequestGeneralMesh::scale (geometry is preserved by the R-3
    // preview memcpy, this preserves the stored factor the inspector
    // shows and scaleObject composes its delta against). Default unit.
    tinym::vec3 scale = tinym::vec3(1.0f, 1.0f, 1.0f);
    // "팽팽함" — uniform multiplier applied to the cloth stiffness
    // (TriangularCloth stretch/shear/bend, FastGridCloth k*) at kernel
    // upload time. Base behaviorParams stay untouched; this scales the
    // SIMULATED stiffness. Default 1. Mirrored on RequestGeneralMesh so
    // it survives Scene::pack; round-trips through scene_format.
    PR clothStiffnessScale = PR(1);
    Constraints<BE, PR> constraints;
    ExternalForces<BE, PR> externalForces;

    // Per-object environment-force gates (UI-driven; default = receive both).
    // Honored by Simulator::applyEnvironmentForces — clearing applyGravity
    // omits the per-particle gravity term; clearing applyWind omits the
    // wind term (cloth only). Persisted via scene_format::Object.
    bool applyGravity = true;
    bool applyWind = true;

    // 고정(Static) 선언 — UI 토글. true면 이 메시는 움직이지 않는다고
    // 약속하는 것으로, BroadPhase가 매 substep BVH refit(리프 AABB 재계산)을
    // 건너뛴다. 한 번 build()된 트리 AABB를 그대로 재사용 → floating/floor
    // 류 정적 메시의 낭비되는 refit 제거. 충돌 '타깃'으로는 여전히 TLAS에
    // 참여한다(천이 정적 바닥 위로 떨어지는 경우). static을 켜면 inspector
    // 콜백이 applyGravity/applyWind를 자동으로 끈다(움직임 소스 제거).
    // Persisted via scene_format::Object; mirrored on RequestGeneralMesh.
    bool isStatic = false;

    // ── Collider data model (collider_pipeline_rework.md §1, P0) ────────
    // colliderKind = how the collision pipeline should REPRESENT this mesh
    // (geometry axis only; the accel axis is fixed in v1). Defaulted ONCE
    // at creation from the initializer subtype (Sphere→Sphere, Cube→Box,
    // Cylinder→Cylinder, Grid/File→Mesh) in Scene::addGeneralMesh — NOT
    // re-derived at pack, so a user override in the inspector survives.
    // collidable = master on/off (P2 will skip it in the broad loop);
    // selfCollide = cloth self-collision gate (P3, SH-based).
    // All three mirrored on RequestGeneralMesh so they survive Scene::pack
    // (translate / rotate / add-object all re-pack); round-tripped through
    // scene_format. INERT in P0 — no consumer yet.
    ColliderKind colliderKind = ColliderKind::Mesh;
    bool collidable = true;
    bool selfCollide = false;

    // Plane checkerboard render option (UI-driven; plane/grid meshes only).
    // When true the renderer overrides the surface albedo with a world-space
    // black/white checker (1 world unit per cell) computed in the plane's
    // local frame — see Simulator::draw and shader.frag's checker* uniforms.
    // Mirrored on RequestGeneralMesh so it survives Scene::pack rebuilds.
    bool checkerboard = false;

    // Sub-object / cluster BVH per-object settings (UI-driven). clusterSplitS
    // = this mesh's split s (k=4^s clusters/tiles) — per-object so each mesh
    // divides to suit its size (maximizes the grid cluster-pair effect). -1
    // = inherit the global BroadPhase::subBvhSplitS (so headless benches that
    // set the global keep working; the GUI writes a concrete 1..8 per object).
    // clusterRender = color this mesh's triangles by cluster in the viewport.
    // Both mirrored on RequestGeneralMesh so they survive Scene::pack (the
    // '0' reset rebuilds BroadPhase from scratch; without the mirror these
    // GUI edits would be lost). The master cluster-mode switch is global
    // (Simulator::clusterModeOn / the profiler window), these are per-object.
    int  clusterSplitS = -1;
    bool clusterRender = false;

    // D-039: Rigid backend wiring. Set by Simulator::ensureRigidBackendBody
    // on Float→Rigid transition (changeBehavior) or initialize-time sweep.
    // kInvalidBodyHandle (-1) means "no backend body yet"; update() skips.
    int32_t rigidBodyHandle = ysim::physics::kInvalidBodyHandle;
    // Cached last backend body position so each frame's Δpos = current - last
    // can be applied to state.x / state.xPrev / transformPosition.
    tinym::vec3 rigidLastBodyPos = {};
    // TOTAL mass the Bullet body was created with (Σ state.m over this
    // mesh's vertices — ysim is the source of truth for mass, see
    // ensureRigidBackendBody). 0 for a static body (applyGravity == false)
    // and for meshes with no backend body. The PBD cloth→rigid coupling
    // reads it as the body's inverse-mass weight, so it must stay in sync
    // with whatever mass the body actually has: every path that changes the
    // Bullet mass goes through recreateRigidBackendBody.
    PR rigidBodyMass = PR(0);

    // Render-side GL state lives in MeshRenderState, keyed by id (D-011).
    // GeneralMesh no longer owns OpenGL handles, so initialize() is safe to
    // call from a non-GL context — that's the precondition for the upcoming
    // Metal-backed test harness.

    GeneralMesh(GeneralMeshInitializer<BE, PR>* initializer, BehaviorType behaviorType, BehaviorParams<PR> behaviorParams)
    : initializer(initializer), behaviorType(behaviorType), behaviorParams(behaviorParams) {}
    GeneralMesh(GeneralMesh&& other) noexcept
        : id(other.id),
          lifetimeId(other.lifetimeId),
          state(std::move(other.state)),
          adjacency(std::move(other.adjacency)),
          initializer(other.initializer),
          behaviorType(other.behaviorType),
          behaviorParams(other.behaviorParams),
          material(std::move(other.material)),
          rotationQuat(other.rotationQuat),
          transformPosition(other.transformPosition),
          scale(other.scale),
          clothStiffnessScale(other.clothStiffnessScale),
          constraints(std::move(other.constraints)),
          externalForces(std::move(other.externalForces)),
          applyGravity(other.applyGravity),
          applyWind(other.applyWind),
          isStatic(other.isStatic),
          colliderKind(other.colliderKind),
          collidable(other.collidable),
          selfCollide(other.selfCollide),
          checkerboard(other.checkerboard),
          clusterSplitS(other.clusterSplitS),
          clusterRender(other.clusterRender)
    {
        other.initializer = nullptr;
    }
    ~GeneralMesh() { delete initializer; }

    void initialize() {
        std::cout << "  - [GeneralMesh initialize] id " << id << " try to initialize\n";
        std::cout << "  - [GeneralMesh initialize] initializer " << initializer << "\n";
        initializer->initialize(state, adjacency);
        std::cout << "  - [GeneralMesh initialize] id " << id << " initializer init\n";
        constraints.memoryAllocation(state.x.size/3);
        std::cout << "  - [GeneralMesh initialize] id " << id << " constraints init. finished.\n";
    }


};






struct Ray {
    tinym::vec3 origin;
    tinym::vec3 dir;
};


struct RayHit {
    Index obj;
    Index primId;
    float tmin, tmax;
};


struct SceneEnvironment {
    tinym::vec3 gravity = tinym::vec3(0.0f, -9.81f, 0.0f);
    tinym::vec3 wind    = tinym::vec3(0.0f, 0.0f, 0.0f);
    // D-028 follow-on: scene-global directional-light tint + magnitude.
    // The shader's `lightColor` uniform is set to (lightColor * lightIntensity)
    // each frame; defaults give white tint at ~1.6 radiance, matching the
    // tuned PBR-preview brightness from the initial D-028 commit. Not
    // currently persisted in scene_format (in-memory only) — promote to
    // schema if/when scene authoring needs lighting setups to survive
    // save/load.
    tinym::vec3 lightColor = tinym::vec3(1.0f, 1.0f, 1.0f);
    float       lightIntensity = 1.6f;
    // Viewport clear color. Read by the render loop each frame and pushed
    // through glClearColor; mirrored into scene_format::Environment so
    // saveScene/loadScene round-trip the choice.
    tinym::vec3 backgroundColor = tinym::vec3(0.886f, 0.906f, 0.922f); // #E2E7EB gray20
};

template <typename BE, typename PR>
struct Scene {
    inline static int numMeshes = 0;
    // D-026: never-resetting monotone counter for per-mesh lifetime
    // identity. Distinct from numMeshes (which resets on resetScene
    // and is the D-018 RNG seed). Used by BroadPhase::build to gate
    // the Float-mesh skip — see CM-008 (graduated).
    inline static int lifetimeMeshCount = 0;
    // Vestigial. id is no longer a monotone counter: it is the compacted
    // [0, numMeshes) array slot, (re)assigned in addGeneralMesh /
    // removeMesh / Scene::pack so it always equals the request's index
    // (== statesOffsets / objTrees / faceObj subscript == objPair). The
    // old monotone scheme avoided MeshRenderState id reuse by never
    // compacting; removeMesh now rebuilds the id-keyed render cache
    // instead. Kept (still zeroed by loadScene) only to avoid churning
    // that reset path; nothing reads it for id assignment anymore.
    inline static int nextMeshId = 0;

    inline static std::vector<GeneralMesh<BE, PR>> meshes;
    inline static SceneEnvironment environment;

    struct RequestGeneralMesh {
        int id;
        int lifetimeId;
        GeneralMeshInitializer<BE, PR>* initializer;
        BehaviorType behaviorType;
        BehaviorParams<PR> behaviorParams;
        // Mirror of GeneralMesh.applyGravity / applyWind kept on the
        // request so per-mesh environment-force toggles survive
        // Scene::pack rebuilds (initialize / reset / loadScene). pack
        // copies these into the realized mesh; the inspector callback
        // writes here in addition to the live mesh field. Both default
        // true (matches GeneralMesh defaults).
        bool applyGravity = true;
        bool applyWind = true;
        // Mirror of GeneralMesh.isStatic kept on the request so the static
        // declaration survives Scene::pack rebuilds (reset / load /
        // changeBehavior). pack copies this onto the realized mesh; the
        // inspector callback writes here in addition to the live field.
        bool isStatic = false;
        // Mirror of GeneralMesh.colliderKind / collidable / selfCollide
        // (collider_pipeline_rework.md §1). Request-owned so a user edit in
        // the inspector survives Scene::pack — without the mirror, the next
        // translate / rotate / add-object silently reset the collider back
        // to its initializer-derived default. colliderKind is SEEDED here
        // once, in addGeneralMesh (defaultColliderKind), and never
        // re-derived; pack only copies request → mesh.
        ColliderKind colliderKind = ColliderKind::Mesh;
        bool collidable = true;
        bool selfCollide = false;
        // Mirror of GeneralMesh.checkerboard (plane render option) kept on
        // the request so the toggle survives Scene::pack rebuilds. pack
        // copies this onto the realized mesh; the inspector callback writes
        // here in addition to the live mesh field. Default off.
        bool checkerboard = false;
        // Mirror of GeneralMesh.clusterSplitS / clusterRender (per-object
        // sub-object BVH split count + cluster-color render). The '0' reset
        // wipes BroadPhase entirely, so these per-object GUI edits must ride
        // the request through pack. Defaults: s=-1 (inherit global), render off.
        int  clusterSplitS = -1;
        bool clusterRender = false;
        // Mirror of GeneralMesh.rotationQuat kept on the request so the
        // user's orientation survives Scene::pack rebuilds (initialize /
        // reset / loadScene). Without this, pack rebuilds meshes from the
        // initializer with a default identity quat — the geometry stays
        // rotated (carried by preview's R-3 memcpy) but the stored
        // quaternion (what the inspector displays and what rotateObject
        // composes deltas against) snapped back to identity. pack copies
        // this into the realized mesh; rotateObject writes here in
        // addition to the live mesh field. Defaults to identity.
        ::Quat rotationQuat{};
        // Mirror of GeneralMesh.scale kept on the request so the user's
        // per-axis scale survives Scene::pack rebuilds, same rationale as
        // rotationQuat above. pack copies this onto the realized mesh;
        // scaleObject writes here in addition to the live mesh field.
        tinym::vec3 scale = tinym::vec3(1.0f, 1.0f, 1.0f);
        // Mirror of GeneralMesh.clothStiffnessScale ("팽팽함") so the
        // multiplier survives Scene::pack rebuilds. Default 1.
        PR clothStiffnessScale = PR(1);
        // S3-2: material owned by the request (single source of truth).
        // pack copies this onto the realized mesh; setMaterial writes
        // here + the live mesh in place (no re-pack — a value
        // overwrite). Replaces the pendingMaterials side-map.
        ::Material material{};
        // Pinned-vertex constraints (point-selection panel). Source of
        // truth for which vertices are fixed and where: Scene::pack
        // re-applies these into constraints.fixedParticles + state.x +
        // preview every rebuild, and scene_format round-trips them.
        // setVertexFixed / translateVertexTo keep this in sync with the
        // live mesh. vid is the PHYSICS vertex index.
        std::vector<FixedVertex> fixedVertices;
        // D-042 R-1: heap-owned vertex/facet/normal preview, populated by
        // initializer->populatePreview() at addGeneralMesh time. Stays
        // alive across Scene::pack / pool-reset cycles. R-2 will point
        // MeshGL here for stable pre-pack rendering; R-5 will sync the
        // packedMeshData → preview after every Simulator::update.
        PreviewState<PR> preview;

        RequestGeneralMesh(int id, int lifetimeId,
                           GeneralMeshInitializer<BE, PR> *initializer,
                           BehaviorType behaviorType,
                           BehaviorParams<PR> behaviorParams)
            : id(id), lifetimeId(lifetimeId), initializer(initializer),
              behaviorType(behaviorType),
              behaviorParams(std::move(behaviorParams)) {}
    };

    // Initializer-derived collider default (collider_pipeline_rework.md §1).
    // Evaluated ONCE per object, at addGeneralMesh time — deliberately NOT
    // in pack's shapeType cascade, which runs on every rebuild and would
    // clobber the user's dropdown choice. Primitive initializers get their
    // matching analytic kind; Grid and File/Assimp/Kinematic stay Mesh (a
    // grid can be deforming cloth, so mesh collision is the honest default
    // — the user opts a static grid into Plane via the inspector).
    static ColliderKind defaultColliderKind(GeneralMeshInitializer<BE, PR>* init) {
        if (dynamic_cast<MeshSphereInitializer  <BE, PR>*>(init)) return ColliderKind::Sphere;
        if (dynamic_cast<MeshCubeInitializer    <BE, PR>*>(init)) return ColliderKind::Box;
        if (dynamic_cast<MeshCylinderInitializer<BE, PR>*>(init)) return ColliderKind::Cylinder;
        return ColliderKind::Mesh;
    }

    inline static std::vector<RequestGeneralMesh> requestsGeneralMeshes;
    // Reference-point coincidence constraints (point panel). Scene-static
    // so it survives Scene::pack and Simulator::reset (neither clears it);
    // loadScene rebuilds the scene and explicitly restores this from the
    // snapshot. The integrator enforces query↔target position coincidence.
    inline static std::vector<ReferencePointConstraint> referenceConstraints;
    inline static bool dirty = true;

    void addGeneralMesh(GeneralMeshInitializer<BE, PR>* initializer, BehaviorType behaviorType, BehaviorParams<PR> behaviorParams) {
        //meshes.emplace_back(initializer, behaviorType, behaviorParams);
        //meshes.back().id = numMeshes++;

        // id is the compacted [0, numMeshes) slot that governs ALL mesh
        // access — findById AND packed-data subscripts (statesOffsets,
        // objTrees, faceObj) AND objPair in collisions. It equals the
        // request's position in requestsGeneralMeshes, which (since we
        // only append here and removeMesh renumbers) is always numMeshes.
        // It is internal logic only, never persisted as an identity:
        // save/load and removeMesh both re-derive it from load/array
        // order. lifetimeId stays the monotone never-reused identity
        // (D-026 BVH Float-skip gate); id and lifetimeId are distinct on
        // purpose — id is volatile, lifetimeId is stable.
        int newId = (int)requestsGeneralMeshes.size();
        requestsGeneralMeshes.emplace_back(newId, lifetimeMeshCount++,
                                           initializer, behaviorType, behaviorParams);
        // Collider default, derived from the initializer subtype exactly
        // once (see defaultColliderKind). From here on the request is the
        // source of truth: the inspector writes it, pack copies it out,
        // loadScene overwrites it when the scene file carries a kind.
        requestsGeneralMeshes.back().colliderKind = defaultColliderKind(initializer);
        numMeshes++;

        // D-042 R-1: populate preview state directly from the initializer
        // (heap-owned vector buffers). After this call, the new request's
        // preview.x / preview.facets / preview.n are ready for renderer
        // consumption — no need to wait for Scene::pack. Subtypes that
        // haven't migrated yet hit the default no-op and leave preview
        // empty (acceptable until R-2 makes preview load-bearing).
        requestsGeneralMeshes.back().initializer->populatePreview(
            requestsGeneralMeshes.back().preview);

        dirty = true;

        //std::cout << "id " << meshes.back().id << " object is created\n";
    }


    struct PackedMeshData {
        // MeshState
        VectorBase<BE, PR> x;
        // xPrev: start-of-substep snapshot of x; consumed by the swept-segment
        // narrow phase (D-013). Sliced into per-mesh state.xPrev.
        VectorBase<BE, PR> xPrev;
        VectorBase<BE, PR> v;
        VectorBase<BE, PR> f;
        VectorBase<BE, PR> m;
        VectorBase<BE, PR> n;
        // External-forces buffer consumed by the cloth kernels
        // (TriangularClothBehavior / FastGridClothBehavior). Sliced into per
        // mesh externalForces.externalForces, filled per frame from
        // Scene::environment by Simulator::applyEnvironmentForces.
        VectorBase<BE, PR> externalForces;
        // MeshAdjacency
        VectorBase<BE, Index> facets;
        VectorBase<BE, Index> edges;
        VectorBase<BE, Index> vertexAdjFacets, vertexAdjFacetsOffsets;
        VectorBase<BE, Index> vertexAdjEdges, vertexAdjEdgesOffsets;

        // offset data by id
        VectorBase<BE, Index> statesOffsets;
        VectorBase<BE, Index> facetsOffsets;
        VectorBase<BE, Index> edgesOffsets;
        // Slice (c) A1: owning mesh index per packed vertex (size
        // numPoints), built once per Scene::pack from statesOffsets.
        // Lets the analytic narrow kernel resolve thread→mesh (and its
        // behavior via meshBehaviors) without an in-kernel binary
        // search over statesOffsets. Inert until c-1 consumes it.
        VectorBase<BE, Index> vertObj;
    };
    inline static PackedMeshData packedMeshData;

    struct PackedCollisionData {
        VectorBase<BE, BroadCollision> broadCollisions;
        VectorBase<BE, Index> numBroadCollisions;
        VectorBase<BE, NarrowCollision> narrowCollisions;
        VectorBase<BE, Index> numNarrowCollisions;
        // Cumulative across substeps; the per-substep `numNarrowCollisions[0]`
        // resets in `resetNarrow()`. The self-test reads this to verify
        // contacts ever fired during a frame loop (BDD-007 acceptance).
        size_t cumulativeNarrowCollisions = 0;
        Index approxColsPerPoints = 15;
        Index maxNumCollisions;
        VectorBase<BE, NarrowCollision> vertColFacets;
        VectorBase<BE, Index> vertColFacetsOffsets;
        // P4 GPU bucketing scratch: per-vertex running offset the scatter
        // kernel atomic-bumps (size numPoints). vertColFacetsOffsets doubles
        // as histogram-then-CSR (scanned in place), so no scan scratch.
        VectorBase<BE, Index> vertColBucketCounter;

        // Segmented (per-threadgroup) detect+reduce scratch — used by
        // BVH::queryPointsSegmented (paper-inspired alternative to the
        // baseline per-leaf-hit global atomic). Allocated alongside
        // broadCollisions in pack(). Worst-case size: ceil(numPoints
        // / segTGSize) threadgroups, each holding up to segPerTGCap
        // BroadCollisions. Reused across all (q,t) calls in one
        // detectCollisionsSegmented frame; the global numBroadCollisions
        // counter (above) accumulates across calls as in the baseline.
        Index segTGSize     = 256;
        // Per-TG private slice budget multiplier on top of the global
        // per-point average (approxColsPerPoints). The baseline path
        // shares one global budget so hot-spot hits "borrow" capacity
        // from cold TGs; the segmented path partitions the budget
        // per-TG so hot TGs (Morton-clustered cloth points piling onto
        // the same ground patch) overflow well before the global cap.
        // 4× covers typical hot/cold ratio in the funnel/cloth-ball
        // benchmarks; raise if [queryPointsSegmented] still warns.
        Index segPerTGCapFactor = 4;
        Index segPerTGCap   = 0;   // = approxColsPerPoints * segTGSize * factor, set in pack
        Index segMaxTGs     = 0;   // = ceil(numPoints / segTGSize),    set in pack
        VectorBase<BE, BroadCollision> segPrivateCollisions;
        VectorBase<BE, uint32_t>       segPrivateCount;
        VectorBase<BE, uint32_t>       segPrivateOffset;

        // Analytic broad markers. The BVH broad phase finds (cloth query,
        // analytic target) object pairs whose top-level AABBs overlap and
        // records them HERE instead of descending the target's triangle
        // BVH (which would emit thousands of vertex×triangle BroadCollisions
        // that narrow_pt_tri then skips). narrowAnalytic() consumes these
        // — one dispatch per pair — so the analytic narrow phase only fires
        // when objects actually overlap (zero cost during the fall) and
        // tests one collider per cloth instead of all cloths × all shapes.
        //
        // objPair namespace (collider_pipeline_rework.md §4): BOTH fields
        // are object ARRAY INDICES (== statesOffsets / objTrees subscript
        // == BroadCollision::objPair), matching the triangle path. They are
        // resolved to a compact AnalyticShape[] slot via
        // AnalyticShape::objIndex in narrowAnalytic.
        //
        // Cleared at the top of every broad detect (all BVH variants), NOT
        // after narrow consumes them: with cdSubstepPeriod > 1 the broad
        // pair set is deliberately HELD across substeps and the analytic
        // markers must be held with it, exactly like broadCollisions.
        struct AnalyticBroadPair { Index clothObj; Index shapeObj; };
        // What narrowAnalytic consumes: this detect's own markers UNION the
        // previous detect's own markers.
        std::vector<AnalyticBroadPair> analyticPairs;
        // This detect's own markers (the union's fresh half).
        std::vector<AnalyticBroadPair> analyticPairsFresh;
        // The previous detect's own markers, carried one detect forward.
        //
        // WHY the carry-over: the broad AABB and the CCD segment point in
        // OPPOSITE time directions. enlargeTrajectory inflates the leaf boxes
        // from the current x FORWARD by v*subh, so at substep n the marker
        // covers [x_n, x_n + v*subh]; but xPrev is snapshotted AFTER the
        // narrow phase (simulator.hpp `system.snapshotXPrev`), so at substep
        // n the narrow segment is the BACKWARD interval [x_{n-1}, x_n]. The
        // two intervals are the same swath of space one substep apart — so a
        // thin collider that the marker catches at substep n is only crossed
        // by the segment at substep n+1, when the marker is already gone, and
        // the swept test never fires. Holding each marker for exactly one
        // extra detect realigns them. Conservative (at worst one extra
        // analytic dispatch per pair per substep, and only while the objects
        // are near each other) and confined to the analytic path.
        std::vector<AnalyticBroadPair> analyticPairsHeld;

        // BVH broad-phase bracket. begin() rotates last detect's own markers
        // into the held slot; the detect loop pushes into the fresh slot;
        // end() publishes the deduped union.
        void beginAnalyticPairs() {
            analyticPairsHeld = analyticPairsFresh;
            analyticPairsFresh.clear();
            analyticPairs.clear();
        }
        void pushAnalyticPair(Index clothObj, Index shapeObj) {
            analyticPairsFresh.push_back({clothObj, shapeObj});
        }
        void endAnalyticPairs() {
            analyticPairs = analyticPairsFresh;
            for (const auto& h : analyticPairsHeld) {
                bool dup = false;
                for (const auto& p : analyticPairs)
                    if (p.clothObj == h.clothObj && p.shapeObj == h.shapeObj) {
                        dup = true; break;
                    }
                if (!dup) analyticPairs.push_back(h);
            }
        }
        // Broad paths that emit no analytic markers (SH / multi-level SH /
        // two-mesh / cluster) drop the whole carry-over chain.
        void clearAnalyticPairs() {
            analyticPairs.clear();
            analyticPairsFresh.clear();
            analyticPairsHeld.clear();
        }

        void resetNarrow() {
            std::memset(narrowCollisions.ptr, 0, sizeof(NarrowCollision)*numNarrowCollisions[0]);
            std::memset(vertColFacets.ptr, 0, sizeof(NarrowCollision)*numNarrowCollisions[0]);
            std::memset(vertColFacetsOffsets.ptr, 0, sizeof(Index)*numNarrowCollisions[0]);
            numNarrowCollisions[0] = 0;
        }
    };
    inline static PackedCollisionData packedCollisionData;

    struct RayTracedData {
        VectorBase<BE, RayHit> clickRayCollisions;
        VectorBase<BE, Index> numClickRayCollisions;
        Index approxColsPerRay = 4096;
    };
    inline static RayTracedData rayTracedData;

    // Compact analytic-collider array. numAnalytic = #meshes whose
    // colliderKind != Mesh (P1: the COLLIDER axis selects, not the
    // initializer's ShapeType — a user can declare an imported prop a
    // Box, or send a Sphere-sourced mesh back through triangle
    // collision). Allocated in packAnalyticShapes() at pack; contents
    // refilled by refreshAnalyticShapes() each update (D3).
    inline static VectorBase<BE, AnalyticShape> meshAnalytic;
    inline static Index numAnalytic = 0;

    // Per-entry canonical half-extents in the collider's LOCAL frame (see
    // fitAnalyticLocalHalf), cached because the arbitrary-mesh fit is
    // O(numPoints) while refresh runs per frame. Fitted LAZILY on the first
    // refresh that runs with live geometry: packAnalyticShapes() executes at
    // the tail of Scene::pack, BEFORE Scene::initialize() has run the mesh
    // initializers, so an AABB fit taken there would read unpopulated
    // positions and collapse the collider to a point. Invalidated by every
    // pack, so a re-scaled / re-authored collider is re-fitted.
    inline static std::vector<tinym::vec3> analyticLocalHalf;
    inline static std::vector<uint8_t> analyticHalfFitted;
    // false until the first refresh has written a transform, so the
    // first fill can seed prev == cur (no phantom motion on frame 0).
    inline static bool analyticPrimed = false;

    // Initializer-classification predicate (ShapeType axis). NOT the
    // collision-pipeline predicate — that is isAnalyticCollider(colliderKind),
    // which is what packAnalyticShapes / the broad phase key off since P1.
    static bool isPrimitiveShape(ShapeType s) {
        return s == ShapeType::Sphere
            || s == ShapeType::Cube
            || s == ShapeType::Cylinder;
    }

    // Canonical LOCAL half-extents for one analytic collider.
    //
    // Fit rule (deliberately simple; no manual param editing in v1 —
    // collider_pipeline_rework.md §1):
    //  * initializer MATCHES the kind (Sphere/Cube/Cylinder built by the
    //    matching primitive::*): half = size*0.5 * per-axis scale. Exact,
    //    because primitive::{sphere,cube,cylinder}(size) all build with
    //    r = size*0.5 and the scale is what got baked into the vertices.
    //  * anything else (imported mesh, grid, mismatched initializer):
    //    REST-POSE LOCAL AABB fit. Vertices are mapped into the collider
    //    frame (inverse rotation about transformPosition) and the half-
    //    extent per axis is max(|min|,|max|) — i.e. the smallest
    //    origin-centred box that contains the geometry. Scale is already
    //    baked into the vertices, so it is NOT applied again.
    //    Box  → those half-extents;  Sphere → same, used as ellipsoid
    //    semi-axes;  Cylinder → radius = max(hx,hz), half-height = hy;
    //    Plane → unused (a half-space needs only center + rotation).
    static tinym::vec3 fitAnalyticLocalHalf(GeneralMesh<BE, PR>& m) {
        PR sz = PR(0);
        const ColliderKind k = m.colliderKind;
        if (k == ColliderKind::Sphere) {
            if (auto* sp = dynamic_cast<MeshSphereInitializer<BE,PR>*>(m.initializer))
                sz = sp->params.size;
        } else if (k == ColliderKind::Box) {
            if (auto* cb = dynamic_cast<MeshCubeInitializer<BE,PR>*>(m.initializer))
                sz = cb->params.size;
        } else if (k == ColliderKind::Cylinder) {
            if (auto* cy = dynamic_cast<MeshCylinderInitializer<BE,PR>*>(m.initializer))
                sz = cy->params.size;
        }
        if (sz > PR(0)) {
            const PR h = sz * PR(0.5);
            return tinym::vec3((float)(h * m.scale.x),
                               (float)(h * m.scale.y),
                               (float)(h * m.scale.z));
        }

        // AABB fit from the live (rest-pose, for a rigid collider)
        // geometry, expressed in the collider's local frame.
        const PR* x = m.state.x.ptr;
        const Index n = x ? m.state.x.size / 3 : 0;
        if (n == 0) return tinym::vec3(0.0f, 0.0f, 0.0f);
        const Quat qInv = quatConjugate(m.rotationQuat);
        const tinym::vec3 c = m.transformPosition;
        float hx = 0.0f, hy = 0.0f, hz = 0.0f;
        for (Index i = 0; i < n; ++i) {
            tinym::vec3 w((float)x[i*3+0] - c.x,
                          (float)x[i*3+1] - c.y,
                          (float)x[i*3+2] - c.z);
            tinym::vec3 l = rotateVector(qInv, w);
            hx = std::max(hx, std::fabs(l.x));
            hy = std::max(hy, std::fabs(l.y));
            hz = std::max(hz, std::fabs(l.z));
        }
        return tinym::vec3(hx, hy, hz);
    }

    // Refill meshAnalytic from the live mesh transform + the cached
    // local half-extents. Iterates meshes in the SAME order as the
    // count pass so entry k always maps to the same mesh across frames.
    //
    // prev{Center,RotQuat} roll: each call moves the PREVIOUS refresh's
    // transform into the prev slots, so `prev` is the collider pose at
    // the start of the interval the vertex segment x_prev→x spans. The
    // caller (Simulator::update) refreshes once per FRAME while the CCD
    // segment is per SUBSTEP, which makes the derived rotation margin
    // conservative (frame delta ≥ substep delta) — never optimistic.
    // v1 reads the authoring transform for non-Rigid colliders (static);
    // Rigid-tagged colliders follow their live vertex centroid so a
    // Bullet-driven body carries its analytic shape with it (full c-4 /
    // A9 motion reconciliation — rotation propagation — still pending).
    // `fitExtents` is false only for the pack-time seeding call, where the
    // mesh initializers have not run yet (see analyticLocalHalf).
    static void refreshAnalyticShapes(bool fitExtents = true) {
        if (numAnalytic == 0 || !meshAnalytic.ptr) return;
        Index k = 0;
        for (Index i = 0; i < (Index)meshes.size(); ++i) {
            auto& m = meshes[i];
            if (!isAnalyticCollider(m.colliderKind)) continue;
            if (k >= numAnalytic) break;

            // Rigid bodies live in state.x (Simulator's D-040 centroid snap
            // keeps the verts on the Bullet body) while transformPosition
            // deliberately stays at the authorial spawn. Reading only the
            // authoring transform left a FALLING Rigid's analytic collider
            // frozen at its spawn point — the rendered ball dropped straight
            // through cloth with zero contacts. Follow the live centroid for
            // Rigid; everything else keeps the authoring transform (v1:
            // static colliders).
            tinym::vec3 c = m.transformPosition;
            if (m.behaviorType == BehaviorType::Rigid
                && m.state.x.ptr && m.state.x.size >= 3) {
                const Index nv = m.state.x.size / 3;
                double sx = 0.0, sy = 0.0, sz = 0.0;
                for (Index vi = 0; vi < nv; ++vi) {
                    sx += (double)m.state.x.ptr[vi*3+0];
                    sy += (double)m.state.x.ptr[vi*3+1];
                    sz += (double)m.state.x.ptr[vi*3+2];
                }
                c = tinym::vec3((float)(sx / nv), (float)(sy / nv),
                                (float)(sz / nv));
            }
            const tinym::vec4 rot(m.rotationQuat.w, m.rotationQuat.x,
                                  m.rotationQuat.y, m.rotationQuat.z);
            if (fitExtents && k < (Index)analyticHalfFitted.size()
                && !analyticHalfFitted[k]) {
                analyticLocalHalf[k]  = fitAnalyticLocalHalf(m);
                analyticHalfFitted[k] = 1;
            }
            const tinym::vec3 h = (k < (Index)analyticLocalHalf.size())
                                ? analyticLocalHalf[k]
                                : tinym::vec3(0.0f, 0.0f, 0.0f);

            AnalyticShape& dst = meshAnalytic[k];
            const tinym::vec4 prevQ = analyticPrimed ? dst.rotQuat : rot;

            AnalyticShape a{};
            a.rotQuat       = rot;
            a.prevCenterPad = analyticPrimed
                ? tinym::vec4(dst.centerRadius.x, dst.centerRadius.y,
                              dst.centerRadius.z, 0.0f)
                : tinym::vec4(c.x, c.y, c.z, 0.0f);
            a.prevRotQuat   = prevQ;
            a.kind          = (uint32_t)m.colliderKind;
            // §4: objPair carries ARRAY INDEX everywhere. pack() makes
            // mesh.id == its array index, so `i` and `m.id` agree — `i`
            // is written because the index is what the contract names.
            a.objIndex      = (uint32_t)i;
            a.behaviorType  = (uint32_t)m.behaviorType;
            a.flags         = m.collidable ? 1u : 0u;

            switch (m.colliderKind) {
                case ColliderKind::Sphere:
                    // Ellipsoid semi-axes; a uniform sphere reduces exactly.
                    a.centerRadius  = tinym::vec4(c.x, c.y, c.z, h.x);
                    a.halfExtHeight = tinym::vec4(h.x, h.y, h.z, h.y);
                    break;
                case ColliderKind::Box:
                    a.centerRadius  = tinym::vec4(c.x, c.y, c.z, 0.0f);
                    a.halfExtHeight = tinym::vec4(h.x, h.y, h.z, 0.0f);
                    break;
                case ColliderKind::Cylinder:
                    // Local +Y axis; radius from the xz extent.
                    a.centerRadius  = tinym::vec4(c.x, c.y, c.z,
                                                  std::max(h.x, h.z));
                    a.halfExtHeight = tinym::vec4(h.x, h.y, h.z, h.y);
                    break;
                case ColliderKind::Plane:
                    // Infinite half-space, local +Y up: center + rotation
                    // only. halfExtHeight unused.
                    a.centerRadius  = tinym::vec4(c.x, c.y, c.z, 0.0f);
                    a.halfExtHeight = tinym::vec4(0.0f, 0.0f, 0.0f, 0.0f);
                    break;
                default:
                    break;
            }
            dst = a;
            ++k;
        }
        analyticPrimed = true;
    }

    // Drop the cached LOCAL half-extents so the next refreshAnalyticShapes()
    // re-fits them from the live geometry / the live mesh scale. Needed by any
    // authoring edit that changes a collider's SIZE without going through
    // Scene::pack — Simulator::scaleObject is deliberately an in-place,
    // no-repack edit, and translate/rotate need nothing because the transform
    // is re-read every refresh while the extents are cached.
    static void invalidateAnalyticFit() {
        std::fill(analyticHalfFitted.begin(), analyticHalfFitted.end(), 0);
    }

    // Count analytic colliders, (re)allocate the compact buffer, seed it.
    // Called at the end of Scene::pack (size can change across packs). The
    // half-extent cache is only INVALIDATED here — the fit itself waits for
    // the first refresh with live geometry (analyticLocalHalf).
    static void packAnalyticShapes() {
        Index count = 0;
        for (auto& m : meshes) if (isAnalyticCollider(m.colliderKind)) ++count;
        numAnalytic = count;
        analyticLocalHalf.assign(count, tinym::vec3(0.0f, 0.0f, 0.0f));
        analyticHalfFitted.assign(count, 0);
        analyticPrimed = false;      // fresh buffer ⇒ seed prev == cur
        // Marker fields are mesh ARRAY INDICES, and a pack can renumber or
        // drop meshes (Simulator::removeMesh compacts ids/indices, resetScene
        // empties the array). The one-detect carry-over (analyticPairsHeld)
        // would then hand narrowAnalytic an index into the OLD array — a
        // wrong-mesh dispatch at best, an out-of-range `meshes[clothObj]` at
        // worst. Indices are only meaningful within one pack, so the whole
        // chain dies with the pack that created it.
        packedCollisionData.clearAnalyticPairs();
        if (count == 0) { meshAnalytic = VectorBase<BE, AnalyticShape>(); return; }
        meshAnalytic = VectorBase<BE, AnalyticShape>(count);
        refreshAnalyticShapes(/*fitExtents=*/false);
        analyticPrimed = false;      // seeding pass is not a real prev sample
    }

    static void pack() {
        if(!dirty) {
            initialize();
            return;
        }

        // meshes[i].initializer is the *same* pointer stored in
        // requestsGeneralMeshes[i].initializer (it was copied, not moved,
        // by addGeneralMesh + emplace_back). Without nulling here,
        // ~GeneralMesh would delete it, leaving requestsGeneralMeshes with
        // dangling pointers; the rebuild loop below would then dereference
        // freed memory. requestsGeneralMeshes is the canonical owner across
        // a re-pack; explicit delete only happens in loadScene.
        for (auto& m : meshes) m.initializer = nullptr;
        meshes.clear();

        // count sizes
        packedMeshData.statesOffsets = VectorBase<BE, Index>(numMeshes+1, 0);
        packedMeshData.facetsOffsets = VectorBase<BE, Index>(numMeshes+1, 0);
        packedMeshData.edgesOffsets  = VectorBase<BE, Index>(numMeshes+1, 0);
        std::vector<PR> masses(numMeshes);

        for(Index i = 0; i < requestsGeneralMeshes.size(); ++i) {
            RequestGeneralMesh& req = requestsGeneralMeshes[i];

            packedMeshData.statesOffsets[i+1] = packedMeshData.statesOffsets[i] + req.initializer->getParams()->numPoints;
            packedMeshData.facetsOffsets[i+1] = packedMeshData.facetsOffsets[i] + req.initializer->getParams()->numFacets;
            packedMeshData.edgesOffsets [i+1] = packedMeshData.edgesOffsets [i] + req.initializer->getParams()->numEdges;
            masses[i] = req.initializer->getParams()->mass;
        }

        // allocate MeshState
        Index numPoints = packedMeshData.statesOffsets[numMeshes];
        // Slice (c) A1: per-vertex owning mesh index. Each mesh i owns
        // the packed-vertex range [statesOffsets[i], statesOffsets[i+1]).
        packedMeshData.vertObj = VectorBase<BE, Index>(numPoints > 0 ? numPoints : 1, 0);
        for (Index i = 0; i < numMeshes; ++i)
            for (Index v = packedMeshData.statesOffsets[i];
                 v < packedMeshData.statesOffsets[i+1]; ++v)
                packedMeshData.vertObj[v] = i;
        Index numStatesData = numPoints*3;
        packedMeshData.x = VectorBase<BE, PR>(numStatesData);
        packedMeshData.xPrev = VectorBase<BE, PR>(numStatesData);
        packedMeshData.v = VectorBase<BE, PR>(numStatesData, 0);
        packedMeshData.f = VectorBase<BE, PR>(numStatesData, 0);
        packedMeshData.m = VectorBase<BE, PR>(numStatesData);
        for(Index i = 0; i < requestsGeneralMeshes.size(); ++i) 
            std::fill(packedMeshData.m.ptr + packedMeshData.statesOffsets[i]*3,
                    packedMeshData.m.ptr + packedMeshData.statesOffsets[i+1]*3,
                    masses[i]);
        packedMeshData.n = VectorBase<BE, PR>(numStatesData);
        packedMeshData.externalForces = VectorBase<BE, PR>(numStatesData, 0);

        // allocate MeshAdjacency
        packedMeshData.facets = VectorBase<BE, Index>(packedMeshData.facetsOffsets[numMeshes]*3);
        packedMeshData.edges  = VectorBase<BE, Index>(packedMeshData.edgesOffsets [numMeshes]*2);
        packedMeshData.vertexAdjFacetsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
        packedMeshData.vertexAdjEdgesOffsets  = VectorBase<BE, Index>(numPoints+1, 0);

        // initialize meshes
        //meshes.resize(numMeshes);
        Index numVertexAdjFacets = 0, numVertexAdjEdges = 0;
        for(Index i = 0; i < numMeshes; ++i) {
            RequestGeneralMesh& req = requestsGeneralMeshes[i];
            meshes.emplace_back(req.initializer, req.behaviorType, req.behaviorParams);
            // pack is the single authoritative compaction point: id == the
            // request's array index i == this mesh's statesOffsets /
            // objTrees / faceObj subscript == objPair value. add/removeMesh
            // keep req.id == i too; writing it back here makes pack robust
            // to any caller (loadScene order, etc.) that didn't pre-compact.
            req.id = (int)i;
            meshes[i].id = (int)i;
            meshes[i].lifetimeId = req.lifetimeId;  // D-026
            // Carry per-mesh environment-force toggles through pack so
            // reset / load / changeBehavior rebuilds preserve the user's
            // Apply Gravity / Apply Wind selections.
            meshes[i].applyGravity = req.applyGravity;
            meshes[i].applyWind    = req.applyWind;
            meshes[i].isStatic     = req.isStatic;
            // Collider data model (§1 P0): request-owned, so the user's
            // dropdown/checkbox edits survive this rebuild. Copied BEFORE
            // the shapeType classification cascade below to make the split
            // explicit — shapeType is re-derived from the initializer every
            // pack, colliderKind deliberately is NOT.
            meshes[i].colliderKind = req.colliderKind;
            meshes[i].collidable   = req.collidable;
            meshes[i].selfCollide  = req.selfCollide;
            meshes[i].checkerboard = req.checkerboard;
            // Per-object sub-object BVH settings survive reset via the request.
            meshes[i].clusterSplitS = req.clusterSplitS;
            meshes[i].clusterRender = req.clusterRender;
            // Carry the user's orientation through pack. The R-3 preview
            // memcpy below restores the rotated *geometry*; this restores
            // the stored *quaternion* so the inspector keeps showing the
            // accumulated rotation and the next rotateObject composes its
            // delta against the true current orientation (not identity).
            meshes[i].rotationQuat = req.rotationQuat;
            // Same as rotationQuat: R-3's preview memcpy restores the
            // scaled geometry, this restores the stored scale factor so
            // the inspector keeps showing it and scaleObject composes its
            // next delta against the true current scale (not unit).
            meshes[i].scale = req.scale;
            meshes[i].clothStiffnessScale = req.clothStiffnessScale;
            // S3-2: material is request-owned; restore it onto the
            // rebuilt mesh (replaces the post-pack pendingMaterials
            // apply).
            meshes[i].material = req.material;
            // Seed transformPosition from the initializer's center/offset so
            // BDD-003's translate path computes deltas against the author
            // intent, not against (0,0,0). Mirrors the dynamic_cast cascade
            // in toSnapshot.
            // Also classifies shapeType from the initializer subtype in
            // the same pass ((a)+(b)). Set in EVERY branch (not relying
            // on the default) so a re-pack of a reused meshes[i] slot
            // can't carry a stale class. Grid/File stay Mesh — a grid
            // can be deforming cloth, so it is honestly mesh-collision;
            // analytic Plane is left for slice (c).
            if (auto* g  = dynamic_cast<MeshGridInitializer  <BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = g->params.center;
                meshes[i].shapeType = ShapeType::Mesh;
            } else if (auto* sp = dynamic_cast<MeshSphereInitializer<BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = sp->params.center;
                meshes[i].shapeType = ShapeType::Sphere;
            } else if (auto* cb = dynamic_cast<MeshCubeInitializer  <BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = cb->params.center;
                meshes[i].shapeType = ShapeType::Cube;
            } else if (auto* cy = dynamic_cast<MeshCylinderInitializer<BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = cy->params.center;
                meshes[i].shapeType = ShapeType::Cylinder;
            } else if (auto* kb = dynamic_cast<MeshKinematicInitializer<BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = kb->params.center;
                meshes[i].shapeType = ShapeType::Mesh;
            } else if (auto* f  = dynamic_cast<MeshFileInitializer  <BE, PR>*>(req.initializer)) {
                meshes[i].transformPosition = f->params.offset;
                meshes[i].shapeType = ShapeType::Mesh;
            } else if (auto* a  = dynamic_cast<AssimpMeshFileInitializer<BE, PR>*>(req.initializer)) {
                // AssimpMeshFileInitializer is a SIBLING of MeshFileInitializer
                // (both derive from GeneralMeshInitializer), so the cast above
                // does not match it. Without this branch a re-pack on reset
                // reseeds transformPosition to (0,0,0) and drops imported-mesh
                // position.
                meshes[i].transformPosition = a->params.offset;
                meshes[i].shapeType = ShapeType::Mesh;
            } else {
                meshes[i].shapeType = ShapeType::Mesh;
            }
            Index prevNumPoints = packedMeshData.statesOffsets[i];
            Index curNumPoints = packedMeshData.statesOffsets[i+1]-prevNumPoints;
            meshes[i].state.x = VectorBase<BE, PR>(packedMeshData.x, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.xPrev = VectorBase<BE, PR>(packedMeshData.xPrev, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.v = VectorBase<BE, PR>(packedMeshData.v, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.f = VectorBase<BE, PR>(packedMeshData.f, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.m = VectorBase<BE, PR>(packedMeshData.m, prevNumPoints*3, curNumPoints*3);
            meshes[i].state.n = VectorBase<BE, PR>(packedMeshData.n, prevNumPoints*3, curNumPoints*3);
            meshes[i].externalForces.externalForces = VectorBase<BE, PR>(packedMeshData.externalForces, prevNumPoints*3, curNumPoints*3);

            meshes[i].adjacency.facets = VectorBase<BE, Index>(packedMeshData.facets, packedMeshData.facetsOffsets[i]*3, (packedMeshData.facetsOffsets[i+1]-packedMeshData.facetsOffsets[i])*3);
            meshes[i].adjacency.edges  = VectorBase<BE, Index>(packedMeshData.edges, packedMeshData.edgesOffsets[i]*2, (packedMeshData.edgesOffsets[i+1]-packedMeshData.edgesOffsets[i])*2);

            meshes[i].initialize();

            // S3-3: pack rebuilds the FULL transformed geometry
            // deterministically from the request alone — no preview
            // carrier. mesh.initialize() above produced the base shape
            // + topology at `center`; bake the request's transform in
            // place over state.x: scale → rotate → translate, pivot =
            // transformPosition (== the initializer center, set just
            // above). Topology (adjacency.facets) and base normals came
            // from the initializer regen and are correct; rotation also
            // rotates the normals so a paused mesh lights correctly
            // pre-sim. Replaces the D-042 R-3 preview→state override.
            {
                const tinym::vec3 pivot = meshes[i].transformPosition;
                auto* ip = req.initializer->getParams();
                const tinym::vec3 s = ip->scale;
                const ::Quat q = ip->rotationQuat;
                const bool unitS = std::abs(s.x-1.f)<1e-7f
                                && std::abs(s.y-1.f)<1e-7f
                                && std::abs(s.z-1.f)<1e-7f;
                const bool idQ = std::abs(q.w-1.f)<1e-7f
                              && std::abs(q.x)<1e-7f
                              && std::abs(q.y)<1e-7f
                              && std::abs(q.z)<1e-7f;
                if (!unitS || !idQ) {
                    const size_t nP = (size_t)curNumPoints;
                    PR* xs = meshes[i].state.x.ptr;
                    PR* ns = meshes[i].state.n.ptr;
                    for (size_t v = 0; v < nP; ++v) {
                        tinym::vec3 p(xs[v*3+0], xs[v*3+1], xs[v*3+2]);
                        tinym::vec3 d = p - pivot;
                        d = tinym::vec3(d.x*s.x, d.y*s.y, d.z*s.z);
                        if (!idQ) d = rotateVector(q, d);
                        xs[v*3+0] = (PR)(pivot.x + d.x);
                        xs[v*3+1] = (PR)(pivot.y + d.y);
                        xs[v*3+2] = (PR)(pivot.z + d.z);
                        if (ns && !idQ) {
                            tinym::vec3 nv = rotateVector(q,
                                tinym::vec3(ns[v*3+0], ns[v*3+1], ns[v*3+2]));
                            ns[v*3+0] = (PR)nv.x;
                            ns[v*3+1] = (PR)nv.y;
                            ns[v*3+2] = (PR)nv.z;
                        }
                    }
                }
            }

            // Behavior-driven cloth jiggle. state.x currently holds the
            // flat preview (R-3 memcpy above). If THIS mesh is currently
            // a cloth, perturb it sub-visibly so the solver/narrow-phase
            // isn't degenerate on a perfectly coplanar sheet; if it is
            // Rigid (or anything non-cloth) leave it flat. Reseeded from
            // the stable mesh id each pack and always run on the just-
            // memcpy'd FLAT buffer, so toggling Rigid<->Cloth back and
            // forth reproduces the identical cloth state every time with
            // zero accumulation. Runs BEFORE recomputeRestLengths so the
            // rest config is measured from the jiggled (== simulated)
            // geometry — keeps the "rest length == what is simulated"
            // invariant intact.
            {
                bool isCloth =
                    meshes[i].behaviorType == BehaviorType::TriangularCloth
                 || meshes[i].behaviorType == BehaviorType::FastGridCloth;
                if (isCloth) {
                    if (auto* g = dynamic_cast<MeshGridInitializer<BE, PR>*>(
                            req.initializer)) {
                        g->applyClothJiggle(meshes[i].state.x.ptr,
                                            curNumPoints,
                                            (uint32_t)meshes[i].id);
                    }
                }
            }

            // Rest length must reflect the geometry the user sees (the
            // preview just memcpy'd into state.x), NOT the initializer's
            // param-regen that mesh.initialize() measured a few lines up.
            // Without this, scale/rotate edits (preview-only mutations)
            // leave every cloth spring pre-stressed → free-fall blow-up.
            // (When cloth, "what the user sees" includes the <=1e-4
            // jiggle applied just above — invisible but consistent.)
            MeshAdjacencyInitializer<BE, PR>::recomputeRestLengths(
                meshes[i].state, meshes[i].adjacency);

            // recomputeRestLengths only fixes the adjacency arrays
            // (TriangularCloth). FastGridCloth's solver instead reads
            // the scalar rest lengths in its behavior params, so a
            // scaled/rotated grid would stay pre-stressed without this.
            // state.x here is the fully transformed geometry, so derive
            // the scalars from it and keep the request copy in lock-step
            // (a later non-dirty re-pack reuses req.behaviorParams).
            if (meshes[i].behaviorType == BehaviorType::FastGridCloth) {
                if (auto* fp = std::get_if<FastGridClothBehaviorParams<PR>>(
                        &meshes[i].behaviorParams)) {
                    recomputeFastGridRest<PR>(meshes[i].state.x.ptr,
                                              meshes[i].state.x.size / 3,
                                              *fp);
                    req.behaviorParams = meshes[i].behaviorParams;
                }
            }

            // Seed xPrev with the initial position so the first substep's
            // swept-CCD narrow check (D-013) sees a degenerate segment
            // (xPrev == x) rather than dangling zeros, which would
            // otherwise trigger spurious crossings. Sits AFTER the R-3
            // memcpy so xPrev mirrors the post-preview x (not the
            // initializer's pre-override regen).
            std::memcpy(meshes[i].state.xPrev.ptr,
                        meshes[i].state.x.ptr,
                        meshes[i].state.x.size * sizeof(PR));

            // Re-apply pinned-vertex constraints from the request (the
            // source of truth). Runs LAST so the pinned position wins
            // over preview/jiggle/recompute: write the held position
            // into state.x + xPrev, set the fixedParticles mask to 0,
            // and mirror into preview (physics + any render copies) so
            // the rendered dot + the next pack's R-3 memcpy agree. This
            // is what makes a pin survive Scene::pack AND loadScene
            // (loadScene rebuilds preview flat from the initializer; the
            // request list restores both the pin and its location).
            for (const auto& fv : req.fixedVertices) {
                if ((Index)fv.vid >= (Index)curNumPoints) continue;
                Index b = (Index)fv.vid * 3;
                meshes[i].state.x.ptr[b+0] = (PR)fv.pos.x;
                meshes[i].state.x.ptr[b+1] = (PR)fv.pos.y;
                meshes[i].state.x.ptr[b+2] = (PR)fv.pos.z;
                meshes[i].state.xPrev.ptr[b+0] = (PR)fv.pos.x;
                meshes[i].state.xPrev.ptr[b+1] = (PR)fv.pos.y;
                meshes[i].state.xPrev.ptr[b+2] = (PR)fv.pos.z;
                if (meshes[i].constraints.fixedParticles.ptr
                    && (Index)fv.vid < meshes[i].constraints.fixedParticles.size)
                    meshes[i].constraints.fixedParticles[fv.vid] = PR(0);
                if (fv.vid * 3 + 2 < req.preview.x.size()) {
                    req.preview.x[b+0] = (PR)fv.pos.x;
                    req.preview.x[b+1] = (PR)fv.pos.y;
                    req.preview.x[b+2] = (PR)fv.pos.z;
                }
                if (req.preview.hasRender()) {
                    for (size_t rv = 0; rv < req.preview.renderToPhysics.size(); ++rv)
                        if (req.preview.renderToPhysics[rv] == fv.vid) {
                            req.preview.renderX[rv*3+0] = (PR)fv.pos.x;
                            req.preview.renderX[rv*3+1] = (PR)fv.pos.y;
                            req.preview.renderX[rv*3+2] = (PR)fv.pos.z;
                        }
                }
            }

            for(int j = 0; j < curNumPoints; ++j) {
                packedMeshData.vertexAdjFacetsOffsets[prevNumPoints+j+1] = meshes[i].adjacency.vertexAdjFacetsOffsets[j+1]+numVertexAdjFacets;
                packedMeshData.vertexAdjEdgesOffsets [prevNumPoints+j+1] = meshes[i].adjacency.vertexAdjEdgesOffsets [j+1]+numVertexAdjEdges;
            }
            numVertexAdjFacets += meshes[i].adjacency.vertexAdjFacets.size;
            numVertexAdjEdges  += meshes[i].adjacency.vertexAdjEdges.size;
        }

        packedMeshData.vertexAdjFacets = VectorBase<BE, Index>(numVertexAdjFacets);
        packedMeshData.vertexAdjEdges  = VectorBase<BE, Index>(numVertexAdjEdges);
        numVertexAdjFacets = numVertexAdjEdges = 0;
        Index numFacets = 0, numEdges = 0;
        for(Index i = 0; i < numMeshes; ++i) {
            auto& vaf = meshes[i].adjacency.vertexAdjFacets;
            auto& vae = meshes[i].adjacency.vertexAdjEdges;
            std::copy(vaf.ptr, vaf.ptr+vaf.size, packedMeshData.vertexAdjFacets.ptr+numVertexAdjFacets);
            std::copy(vae.ptr, vae.ptr+vae.size, packedMeshData.vertexAdjEdges.ptr+numVertexAdjEdges);
            for(Index j = 0; j < vaf.size; ++j)
                if(packedMeshData.vertexAdjFacets[numVertexAdjFacets+j] != vaf[j]) exit(1);
            for(Index j = 0; j < vae.size; ++j)
                if(packedMeshData.vertexAdjEdges[numVertexAdjEdges+j]  != vae[j]) exit(1);
            vaf = VectorBase<BE, Index>(packedMeshData.vertexAdjFacets, numVertexAdjFacets, vaf.size);
            vae = VectorBase<BE, Index>(packedMeshData.vertexAdjEdges,  numVertexAdjEdges,  vae.size);
            numVertexAdjFacets += vaf.size;
            numVertexAdjEdges  += vae.size;
        }

        // allocate collisions
        // YSIM_COLS_PER_POINT raises the per-point broad-pair budget so a
        // densely-draping cloth on a big obstacle (P200 + camel ≈ 6–12M
        // pairs) does NOT overflow the buffer and silently drop pairs —
        // dropped pairs are the source of non-reproducible detect times and
        // patchy tunneling. Default 15 keeps normal-run memory small.
        if (const char* e = std::getenv("YSIM_COLS_PER_POINT")) {
            int v = std::atoi(e);
            if (v > 0) packedCollisionData.approxColsPerPoints = (Index)v;
        }
        packedCollisionData.maxNumCollisions = numPoints*packedCollisionData.approxColsPerPoints;
        packedCollisionData.broadCollisions = VectorBase<BE, BroadCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.numBroadCollisions = VectorBase<BE, Index>(1, 0);
        packedCollisionData.narrowCollisions = VectorBase<BE, NarrowCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.numNarrowCollisions = VectorBase<BE, Index>(1, 0);
        packedCollisionData.vertColFacets = VectorBase<BE, NarrowCollision>(packedCollisionData.maxNumCollisions);
        packedCollisionData.vertColFacetsOffsets = VectorBase<BE, Index>(numPoints+1, 0);
        packedCollisionData.vertColBucketCounter = VectorBase<BE, Index>(numPoints, 0);

        // Segmented variant scratch (paper-inspired alternative path).
        // Sized so a single queryPointsSegmented dispatch over ANY mesh
        // fits: segMaxTGs uses TOTAL numPoints as an upper bound on any
        // single mesh's point count, segPerTGCap mirrors the global
        // per-point estimate scaled to one threadgroup.
        // Decoupled from approxColsPerPoints: the segmented path's per-TG
        // budget keeps its historical base (15) so raising the global broad
        // budget (YSIM_COLS_PER_POINT) does not multiply this scratch by the
        // same factor (would balloon to ~1GB at cols=256). Segmented query is
        // off in the cluster bench anyway; this only sizes its idle scratch.
        packedCollisionData.segPerTGCap = Index(15)
                                        * packedCollisionData.segTGSize
                                        * packedCollisionData.segPerTGCapFactor;
        packedCollisionData.segMaxTGs   = std::max<Index>(1,
            (numPoints + packedCollisionData.segTGSize - 1) / packedCollisionData.segTGSize);
        packedCollisionData.segPrivateCollisions = VectorBase<BE, BroadCollision>(
            packedCollisionData.segMaxTGs * packedCollisionData.segPerTGCap);
        packedCollisionData.segPrivateCount  = VectorBase<BE, uint32_t>(packedCollisionData.segMaxTGs, 0);
        packedCollisionData.segPrivateOffset = VectorBase<BE, uint32_t>(packedCollisionData.segMaxTGs, 0);


        // allocate ray traced data
        rayTracedData.clickRayCollisions = VectorBase<BE, RayHit>(rayTracedData.approxColsPerRay);
        rayTracedData.numClickRayCollisions = VectorBase<BE, Index>(1, 0);


        // adjacency data

        // Slice (c) c-0: build the compact analytic-primitive array
        // from the now-realized meshes (shapeType/transform set by the
        // cascade above). Inert — no kernel consumes it yet.
        packAnalyticShapes();

        dirty = false;
    }

    static void initialize() {
        for(auto& mesh : meshes) {
            std::cout << "  - try to initialize mesh " << mesh.id << "\n";
            mesh.initialize();
            mesh.state.v.map().setZero();
            std::cout << "  - mesh " << mesh.id << " is initialized\n";
        }
        if(meshes.size() > 0) std::cout << "[Simulator Init] general mesh objects are initialized" << std::endl;
    }

    /// postpack() is for data which is able to allocate after the mesh initialization.



    static GeneralMesh<BE, PR>* findById(int id) {
        for(auto& mesh : meshes) {
            if(mesh.id == id) return &mesh;
        }
        return nullptr;
    }
};



// RadixSorter is declared here (above SpatialHashing) so that
// SpatialHashing<METAL, PR> can hold a value-typed RadixSorter member.
// The primary template and METAL specialization were originally further
// down the file alongside BVH; they were moved up unchanged.
template <typename BE, typename Element>
