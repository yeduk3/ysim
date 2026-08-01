#pragma once
// Fragment header: included in order by main.cpp after sim/scene.hpp and
// BEFORE sim/bruteforce.hpp (whose last line is the dangling
// `template <typename BE, typename PR, typename System>` that heads
// simulator.hpp — nothing can be inserted between those two). Relies on the
// preamble (using Index, std includes) and on earlier fragments. Not
// independently compilable by design.
//
// CPU Position Based Dynamics (Müller et al. 2007) as a SIBLING solver to
// SymplecticSystem, selected at runtime by `Simulator::usePbd` (see the
// A-option in docs/design/pbd-system-handoff.md §4). Deliberately NOT a
// System-type swap: main.cpp instantiates exactly one Simulator/System pair,
// and PBD borrows the System's timing (`subh`) rather than duplicating it, so
// there is one source of truth for h / subSteps / acctime.
//
// Runs on the CPU against the METAL shared-storage buffers directly
// (MeshState::x/v/m and Scene::packedCollisionData are all
// StorageModeShared, so `.ptr` is host-addressable).
//
// ponytail: one commitAndWait per substep. The CPU solver must see the
// broad/narrow-phase results encoded earlier in the same substep, which costs
// a GPU sync that the all-GPU symplectic path does not pay. Port the
// projection loop to a Metal kernel if PBD earns its keep.

#include <unordered_map>
#include <algorithm>

template <typename BE, typename PR>
struct PbdSystem {};

template <typename PR>
struct PbdSystem<METAL, PR> {
    // Gauss-Seidel sweeps per substep. More iterations => stiffer cloth.
    int iterations = 8;
    // Fraction of velocity removed per substep. 0 = no artificial damping
    // (the distance projections already dissipate).
    PR damping  = PR(0);

    // PBD stiffness is a dimensionless [0,1] projection weight; the force
    // solver's kstretch/kbend are spring constants — there is no physical
    // conversion between them. Per-channel reference constants give a rough
    // linear map so the SAME per-mesh coefficients drive both solvers (the
    // mesh inspector edits one set of numbers, not two). References are
    // picked so the shipped cloth defaults (kstretch 1e5, kbend 2-3e5) land
    // where the PBD self-test was tuned (~1.0 stretch, ~0.2-0.3 bend).
    //
    // ponytail: deliberately NOT a per-solver material-parameter struct.
    // Storing separate coefficient sets per solver mode needs a real
    // material-params refactor; until PBD earns its keep this stays a
    // two-constant approximation. Refactor task, see
    // docs/design/pbd-system-handoff.md.
    PR stretchRef = PR(1e5);
    PR bendRef    = PR(1e6);

    // ── Tearing (milestone 1: hole tearing via facet degeneration) ────────
    // Default OFF so every existing scene is byte-identical to before.
    bool tearEnabled = false;
    // Break an edge once its PREDICTED length exceeds tearRatio * rest.
    // Measured on `p` BEFORE the projection sweeps: PBD re-satisfies the
    // distance constraint every iteration, so post-solve strain is tiny even
    // under a load that would physically rip the sheet. The overstretch only
    // exists in the prediction, which is exactly where we look.
    PR tearRatio = PR(1.4);
    // Cap on tears per mesh per step. Without it, the first gust that
    // overstretches a whole row breaks every edge of that row in one step and
    // the cloth shatters instead of ripping. Tearing the WORST edges first and
    // letting the rest re-strain next step is what produces a propagating tear.
    int maxTearsPerStep = 4;

    // Cumulative / per-step tear counters (read by the GUI and the tests).
    // `tearsThisStep` counts PRIMARY tears only — the dangling edges killed by
    // the secondary rule are bookkeeping, not new rips.
    uint32_t tearCount     = 0;
    uint32_t tearsThisStep = 0;

    // Predicted positions, one flat 3N block PER MESH, indexed by the
    // Scene::meshes ARRAY INDEX. A two-way contact writes into the PARTNER
    // mesh's predicted positions, so a single reused buffer no longer works:
    // every active cloth's block must stay live for the whole sweep phase.
    // Grown, never shrunk — same idiom as bendCache.
    std::vector<std::vector<PR>> predBlocks;
    // Accumulated two-way contact displacement per vertex (flat 3N per mesh),
    // zeroed each step. The velocity update needs to know how much of a
    // vertex's position delta came from contact projection so depenetration
    // does not become velocity (see the split in phase 3).
    std::vector<std::vector<PR>> cDispBlocks;

    // One bending constraint per interior edge: p1-p2 is the shared edge,
    // p3 / p4 the opposite vertices of the two incident triangles, phi0 the
    // rest dihedral angle. Adjacency does not persist this quad (it stores
    // opposite-vertex PAIRS without the shared edge), so PbdSystem derives
    // it from facets and caches per mesh.
    struct BendQuad { Index p1, p2, p3, p4; PR phi0; };
    static constexpr Index kConsumed = (Index)-1;
    struct BendCache {
        int   lifetimeId = -2;   // -1 is a legal mesh value, so seed elsewhere
        Index numPoints = 0, numFacets = 0;
        std::vector<BendQuad> quads;
        // Set by the tear pass. The topology key above CANNOT catch a tear:
        // degeneration rewrites facet INDICES in place, so numFacets (and
        // numPoints, and lifetimeId) are all unchanged. Without this flag the
        // solver would keep projecting bend quads that span a hole.
        bool  dirty = false;
    };
    std::vector<BendCache> bendCache;   // parallel to Scene::meshes

    // ── Tear bookkeeping, one per mesh (parallel to Scene::meshes) ────────
    // Everything here is derived from the mesh's own topology and is rebuilt
    // whenever that topology is (re)realized. `edgeAlive` is MONOTONIC: a dead
    // edge never comes back, because the geometry it constrained is gone. The
    // only way back to a whole cloth is a full rebuild (scene reset), which
    // resetTearing() / the heal check below handle.
    static constexpr Index kNoFacet = (Index)-1;
    struct TearState {
        int   lifetimeId = -2;   // same seeding convention as BendCache
        Index numPoints = 0, numFacets = 0;
        // Per EDGE SLOT (size == numEdges, the padded upper bound). Padded /
        // out-of-range slots are seeded dead so an "alive" slot is always a
        // real edge.
        std::vector<uint8_t> edgeAlive;
        // The 1-2 facets incident to each edge slot, kNoFacet when absent.
        // A manifold interior edge has both; a boundary edge has only f0; a
        // padded slot has neither.
        std::vector<Index> edgeFacet0, edgeFacet1;
        std::vector<uint8_t> facetAlive;   // size == numFacets
        // Consumed by the NEXT step's BendCache check (bend quads spanning a
        // hole must be dropped) — kept separate from facetsDirty so the two
        // consumers cannot steal each other's edge.
        bool bendDirty   = false;
        // Consumed by phase 2 (render propagation): "this mesh's facet index
        // buffer changed this step". Cleared by consumeTornDirty().
        bool facetsDirty = false;
    };
    std::vector<TearState> tearStates;   // parallel to Scene::meshes

    // Phase-2 hook: did mesh `mi` change its facet indices since the last
    // call? Clears the flag, so exactly one consumer sees each tear event.
    bool consumeTornDirty(Index mi) {
        if (mi >= (Index)tearStates.size()) return false;
        const bool d = tearStates[mi].facetsDirty;
        tearStates[mi].facetsDirty = false;
        return d;
    }
    // Non-consuming peek (tests / debug HUD).
    bool tornDirty(Index mi) const {
        return mi < (Index)tearStates.size() && tearStates[mi].facetsDirty;
    }
    // Which facets of mesh `mi` are still live, nullptr when the mesh has no
    // tear state yet. Phase 2 can walk this instead of re-deriving degeneracy
    // from the index buffer.
    const std::vector<uint8_t>* facetAliveOf(Index mi) const {
        if (mi >= (Index)tearStates.size()) return nullptr;
        const auto& fa = tearStates[mi].facetAlive;
        return fa.empty() ? nullptr : &fa;
    }
    // Forget every tear. Does NOT restore the facet indices — only a topology
    // rebuild (Scene::pack) can do that; this just stops the solver from
    // believing in holes that no longer exist.
    void resetTearing() {
        tearStates.clear();
        for (auto& bc : bendCache) bc.dirty = true;
        tearCount = 0;
        tearsThisStep = 0;
    }

    // Anomaly (NaN/Inf) counter for the frame; Simulator reads nothing from
    // this today — the guard's job is to keep a blown-up vertex from
    // poisoning the whole mesh.
    uint32_t sanitizeCount = 0;

    // Two-way contact statistics for the LAST step() call. `selfContactCount`
    // is the subset whose target mesh is the query mesh itself. Reset at the
    // top of step(); read by the self-tests.
    uint32_t twoWayContactCount = 0;
    uint32_t selfContactCount   = 0;
    // Number of one-sided contact rows that fed a coupled rigid body this
    // step(). Reset at the top of step() like the counters above.
    uint32_t rigidCoupleCount   = 0;

    static bool clothLike(BehaviorType b) {
        return b == BehaviorType::TriangularCloth
            || b == BehaviorType::FastGridCloth;
    }

    // thickness lives on the two cloth param structs; Float has none.
    static PR thicknessOf(const BehaviorParams<PR>& bp) {
        if (auto* c = std::get_if<ClothBehaviorParams<PR>>(&bp)) return c->thickness;
        if (auto* g = std::get_if<FastGridClothBehaviorParams<PR>>(&bp)) return g->thickness;
        return PR(0);
    }

    // Per-mesh spring constants, the SAME fields the force kernels read
    // (including the "팽팽함" clothStiffnessScale multiplier the symplectic
    // path applies at upload time).
    //
    // `shear` has no separate PBD constraint set: adjacency exposes edges
    // (stretch) and opposite-vertex pairs (bend) only. On a triangulated
    // grid the shear diagonals ARE mesh edges, so they are already projected
    // with the stretch coefficient — the shear number is unused here rather
    // than silently folded in somewhere it does not belong.
    static void springConstantsOf(const GeneralMesh<METAL, PR>& mesh,
                                  PR& kstretch, PR& kbend) {
        kstretch = PR(0); kbend = PR(0);
        if (auto* c = std::get_if<ClothBehaviorParams<PR>>(&mesh.behaviorParams)) {
            kstretch = c->stretch; kbend = c->bend;
        } else if (auto* g = std::get_if<FastGridClothBehaviorParams<PR>>(&mesh.behaviorParams)) {
            kstretch = g->kstretch; kbend = g->kbend;
        }
        const PR s = mesh.clothStiffnessScale;
        kstretch *= s; kbend *= s;
    }

    // The packed state arrays are flat 3N scalars; map them as vectors
    // instead of indexing components by hand. vertexAt reads a triple out of
    // a const buffer, vertexRef write-maps a mutable one. Note tinym's
    // norm()/scalar operators are float-typed, which matches the only
    // instantiated Precision.
    using Vec3 = tinym::vec3_base<PR>;
    using Vec3Ref = tinym::vec3_view_base<PR>;
    static Vec3 vertexAt(const PR* base, Index i) {
        return tinym::vec3_at(base + i * 3);
    }
    static Vec3Ref vertexRef(PR* base, Index i) { return Vec3Ref(base + i * 3); }

    // ── Cloth → rigid-body coupling ──────────────────────────────────────
    // Positional correction owed to each RIGID mesh, indexed by the mesh
    // ARRAY INDEX. A one-sided cloth-vs-rigid contact splits its
    // depenetration mass-weighted: the cloth vertex takes its share here and
    // now, the body's share accumulates into this vector.
    //
    // NOT reset per step(): the narrow phase measured its `distance` against
    // the FRAME-FROZEN body (a Rigid's analytic shape and verts only move in
    // the next frame's update() preamble), so the correction has to keep
    // accumulating across every substep of the frame and be pushed to Bullet
    // once, at frame completion. Simulator::update calls beginFrameRigid()
    // once per frame to zero it, and zeroes each entry as it writes it back.
    std::vector<Vec3> rigidDelta;

    // Zero the per-frame rigid coupling accumulator. Called once per FRAME
    // by Simulator::update (not per substep).
    void beginFrameRigid(Index numMeshes) {
        rigidDelta.assign((size_t)numMeshes, Vec3());
    }

    // Dihedral angle of the quad, measured exactly the way the constraint
    // function does — including the DELIBERATELY opposite normal
    // orientations. n1 = (p2-p1)x(p3-p1) and n2 = (p2-p1)x(p4-p1) point to
    // opposite sides of a FLAT sheet, so a flat rest pose gives d = -1 and
    // phi0 = pi. Do NOT "fix" the winding to make the normals agree: the
    // gradients (eq 21-28) are derived for exactly these two expressions.
    // Returns false when either triangle is degenerate.
    static bool dihedral(const PR* pos, Index i1, Index i2, Index i3, Index i4,
                         Vec3& n1, Vec3& n2, PR& l23, PR& l24, PR& d) {
        const Vec3 p1 = vertexAt(pos, i1);
        const Vec3 P2 = vertexAt(pos, i2) - p1;
        const Vec3 P3 = vertexAt(pos, i3) - p1;
        const Vec3 P4 = vertexAt(pos, i4) - p1;
        const Vec3 c23 = P2.cross(P3);
        const Vec3 c24 = P2.cross(P4);
        l23 = c23.norm();
        l24 = c24.norm();
        if (l23 < PR(1e-12) || l24 < PR(1e-12)) return false;
        n1 = c23 / l23;
        n2 = c24 / l24;
        d = n1.dot(n2);
        if (d >  PR(1)) d =  PR(1);
        if (d < PR(-1)) d = PR(-1);
        return true;
    }

    // Undirected edge key: (min,max) packed into 64 bits. The ONE convention
    // shared by buildBendQuads, the tear edge->facet map and the phi0 carry-
    // over below — they must agree or a rebuilt quad looks like a new one.
    static uint64_t edgeKey(Index a, Index b) {
        return a < b ? (((uint64_t)a << 32) | (uint32_t)b)
                     : (((uint64_t)b << 32) | (uint32_t)a);
    }

    // Derive one quad per interior edge from the facet list, and measure
    // phi0 from the CURRENT positions. Called when the cached topology no
    // longer matches the mesh, so phi0 is the pose at that moment — the
    // same "re-measure rest from live geometry" convention
    // MeshAdjacency::recomputeRestLengths uses.
    static void buildBendQuads(const GeneralMesh<METAL, PR>& mesh,
                               std::vector<BendQuad>& out) {
        out.clear();
        const Index* facets = mesh.adjacency.facets.ptr;
        const PR* pos = mesh.state.x.ptr;
        if (!facets || !pos) return;
        const Index numFacets = (Index)mesh.adjacency.facets.size / 3;

        // edge (min,max) -> opposite vertex of the first face seen.
        std::unordered_map<uint64_t, Index> firstOpp;
        firstOpp.reserve((size_t)numFacets * 3);

        for (Index f = 0; f < numFacets; ++f) {
            const Index v[3] = { facets[f*3+0], facets[f*3+1], facets[f*3+2] };
            // TEARING: a torn facet is degenerated in place to (v0,v0,v0).
            // dihedral() would already reject the quad it produces (both
            // cross products are zero), but the (v0,v0) self-key would still
            // be inserted into firstOpp and then marked kConsumed — dead
            // weight, and a trap for anyone who later reuses this map. Drop
            // any duplicate-index facet up front instead.
            if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;
            for (int e = 0; e < 3; ++e) {
                const Index a = v[e], b = v[(e+1)%3], o = v[(e+2)%3];
                const uint64_t key = edgeKey(a, b);
                auto it = firstOpp.find(key);
                if (it == firstOpp.end()) { firstOpp.emplace(key, o); continue; }
                // Second incident face closes the quad. A third (non-manifold
                // geometry) finds the consumed marker and is dropped rather
                // than emitting a constraint against vertex (Index)-1.
                if (it->second == kConsumed) continue;
                const Index lo = a < b ? a : b, hi = a < b ? b : a;
                Vec3 n1, n2; PR l23, l24, d;
                if (dihedral(pos, lo, hi, it->second, o, n1, n2, l23, l24, d))
                    out.push_back(BendQuad{ lo, hi, it->second, o, std::acos(d) });
                it->second = kConsumed;
            }
        }
    }

    // Re-derive the quad TOPOLOGY after a tear while keeping every surviving
    // quad's ORIGINAL phi0.
    //
    // A plain buildBendQuads() re-run would re-measure phi0 from the live
    // pose, i.e. bake the cloth's current drape in as its rest shape — and
    // since a tear propagates over many steps, that re-bake would happen
    // every frame the cloth is ripping, progressively freezing the whole
    // sheet. A quad is uniquely identified by its SHARED EDGE (p1,p2), so the
    // carry-over is a single lookup: same shared edge => same constraint =>
    // same rest angle. Quads that are genuinely new (there are none today —
    // tearing only removes) fall back to the freshly measured angle.
    static void rebuildBendQuadsPreservingPhi0(const GeneralMesh<METAL, PR>& mesh,
                                               BendCache& bc) {
        std::unordered_map<uint64_t, PR> oldPhi;
        oldPhi.reserve(bc.quads.size() * 2);
        for (const auto& q : bc.quads) oldPhi.emplace(edgeKey(q.p1, q.p2), q.phi0);
        buildBendQuads(mesh, bc.quads);
        for (auto& q : bc.quads) {
            auto it = oldPhi.find(edgeKey(q.p1, q.p2));
            if (it != oldPhi.end()) q.phi0 = it->second;
        }
    }

    // Build (or validate) mesh `mi`'s tear bookkeeping. Returns nullptr when
    // the mesh has no usable edge/facet topology.
    //
    // Invalidation has TWO triggers, because tearing deliberately keeps every
    // cheap topology key constant:
    //   (a) the usual lifetimeId / numPoints / numFacets key — a genuinely
    //       different mesh in this slot;
    //   (b) a HEAL check: every facet we recorded as dead must still be
    //       degenerate in the live index buffer. Scene::pack() regenerates
    //       adjacency from the initializer on reset but does NOT change
    //       lifetimeId (it is the stable identity, by design), so (a) alone
    //       would leave a reset cloth whole on screen and shredded in the
    //       solver. One un-degenerated dead facet is proof the topology was
    //       rebuilt underneath us; drop everything and start over.
    TearState* ensureTearState(const GeneralMesh<METAL, PR>& mesh, Index mi,
                               Index n, Index numFacets) {
        if (mi >= (Index)tearStates.size()) return nullptr;
        TearState& ts = tearStates[mi];
        const Index* F = mesh.adjacency.facets.ptr;
        const Index* E = mesh.adjacency.edges.ptr;
        if (!F || !E || numFacets == 0) return nullptr;
        const Index numEdges = (Index)mesh.adjacency.edges.size / 2;
        if (numEdges == 0) return nullptr;

        bool stale = ts.lifetimeId != mesh.lifetimeId
                  || ts.numPoints != n
                  || ts.numFacets != numFacets
                  || (Index)ts.edgeAlive.size() != numEdges;
        if (!stale) {
            for (Index f = 0; f < numFacets; ++f) {
                if (ts.facetAlive[f]) continue;
                const Index v0 = F[f*3+0];
                if (F[f*3+1] != v0 || F[f*3+2] != v0) { stale = true; break; }
            }
        }
        if (!stale) return &ts;

        ts.lifetimeId = mesh.lifetimeId;
        ts.numPoints  = n;
        ts.numFacets  = numFacets;
        ts.facetAlive.assign((size_t)numFacets, 1);
        ts.edgeAlive.assign((size_t)numEdges, 0);   // seed DEAD, revive below
        ts.edgeFacet0.assign((size_t)numEdges, kNoFacet);
        ts.edgeFacet1.assign((size_t)numEdges, kNoFacet);
        ts.bendDirty = false;
        ts.facetsDirty = false;

        // edge (min,max) -> the 1-2 facets that use it. Built from facets, the
        // same source buildBendQuads walks, so the two agree on what an
        // interior edge is.
        std::unordered_map<uint64_t, std::pair<Index, Index>> ef;
        ef.reserve((size_t)numFacets * 3);
        for (Index f = 0; f < numFacets; ++f) {
            const Index v[3] = { F[f*3+0], F[f*3+1], F[f*3+2] };
            if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;
            if (v[0] >= n || v[1] >= n || v[2] >= n) continue;
            for (int e = 0; e < 3; ++e) {
                const uint64_t key = edgeKey(v[e], v[(e+1)%3]);
                auto it = ef.find(key);
                if (it == ef.end()) ef.emplace(key, std::make_pair(f, kNoFacet));
                else if (it->second.second == kNoFacet) it->second.second = f;
                // A third incident face (non-manifold) is ignored: it would
                // survive the tear as an orphan, which is strictly safer than
                // silently overwriting one of the two we already know about.
            }
        }

        for (Index e = 0; e < numEdges; ++e) {
            const Index a = E[e*2], b = E[e*2+1];
            // `numEdges` is the PADDED upper bound; the tail is uninitialised
            // pool memory reinterpreted as indices (see the projection loop).
            // Those slots stay dead, so "alive" implies "real edge".
            if (a >= n || b >= n || a == b) continue;
            auto it = ef.find(edgeKey(a, b));
            if (it == ef.end()) continue;   // edge with no facet: not tearable
            ts.edgeFacet0[e] = it->second.first;
            ts.edgeFacet1[e] = it->second.second;
            ts.edgeAlive[e] = 1;
        }
        return &ts;
    }

    // Iteration-count-corrected stiffness: applying k' = 1-(1-k)^(1/n) once
    // per sweep converges to k after n sweeps, so doubling `iterations` does
    // not silently double the cloth's stiffness.
    static PR correctedK(PR k, int iters) {
        if (iters <= 1) return k;
        if (k >= PR(1)) return PR(1);
        if (k <= PR(0)) return PR(0);
        return PR(1) - std::pow(PR(1) - k, PR(1) / PR(iters));
    }

    // Inverse mass, zero for pinned vertices — a pinned vertex is infinitely
    // heavy so a constraint moves only its partner. Static (m/mask passed in)
    // because a two-way contact evaluates it against the PARTNER mesh's
    // arrays, not just the mesh being swept.
    static PR invMassOf(const PR* m, const PR* mask, Index i) {
        const PR mi3 = m[i * 3];
        return (mask[i] != PR(0) && mi3 > PR(0)) ? PR(1) / mi3 : PR(0);
    }

    // Everything one active cloth mesh needs during a sweep, resolved once
    // per step. Contacts reach ACROSS contexts (the partner's predicted
    // block, masses and thickness), which is why this is a struct and not a
    // set of per-mesh locals.
    struct SolveCtx {
        Index mi = 0;                 // Scene::meshes array index
        PR* x = nullptr;
        PR* v = nullptr;
        PR* p = nullptr;              // predBlocks[mi].data()
        PR* cd = nullptr;             // cDispBlocks[mi].data()
        const PR* m = nullptr;
        const PR* ext = nullptr;
        const PR* mask = nullptr;
        Index n = 0;
        PR thickness = PR(0), kS = PR(0), kB = PR(0);
        const Index* edges = nullptr;
        const PR* restLen = nullptr;
        Index numEdges = 0;
        // tearStates[mi].edgeAlive.data(), or nullptr when this mesh has no
        // tear state (tearing off, or no usable topology). The projection
        // loop treats nullptr as "everything alive", so the untorn path costs
        // one null test per mesh per sweep and nothing else.
        const uint8_t* edgeAlive = nullptr;
        const Index* facets = nullptr;
        Index numFacets = 0;
        const BendCache* bend = nullptr;
        bool contactsForMesh = false;
        Index colBase = 0;
    };
    std::vector<SolveCtx> ctxs;      // one per ACTIVE cloth mesh
    std::vector<int> ctxOfMesh;      // mesh array index -> ctxs index, -1 = none

    // Overstretched edge candidates of ONE mesh, worst-first. A member (not a
    // local) so the steady state — tearing enabled, nothing overstretched —
    // never allocates.
    struct TearCand { PR strain; Index edge; };
    std::vector<TearCand> tearCands;

    void step(Scene<METAL, PR>& sceneObjects, PR dt) {
        if (dt <= PR(0)) return;
        // Flush + wait: x, and the narrow-phase contact arrays this substep's
        // broad/narrow dispatches just filled, must be GPU-complete before the
        // CPU reads them.
        MetalGlobalContext::commitAndWait();

        twoWayContactCount = 0;
        selfContactCount   = 0;
        rigidCoupleCount   = 0;
        tearsThisStep      = 0;

        auto& off      = Scene<METAL, PR>::packedMeshData.statesOffsets;
        auto& colFacet = Scene<METAL, PR>::packedCollisionData.vertColFacets;
        auto& colOff   = Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets;
        const bool haveContacts = colFacet.ptr && colOff.ptr && off.ptr;

        const Index numMeshes = (Index)sceneObjects.meshes.size();
        // Bending quads are topology-derived and cached per mesh; the pred /
        // contact-displacement blocks are per mesh too. All three are indexed
        // by the mesh ARRAY INDEX so a contact can reach its partner's block.
        if ((Index)bendCache.size() != numMeshes) bendCache.resize(numMeshes);
        if ((Index)predBlocks.size()  < numMeshes) predBlocks.resize(numMeshes);
        if ((Index)cDispBlocks.size() < numMeshes) cDispBlocks.resize(numMeshes);
        // Sized once here so every TearState's vector data() stays valid for
        // the whole step — SolveCtx caches edgeAlive.data().
        if ((Index)tearStates.size() != numMeshes) tearStates.resize(numMeshes);
        ctxs.clear();
        ctxOfMesh.assign((size_t)numMeshes, -1);

        // --- (1) build one solve context per ACTIVE cloth mesh, and predict.
        // Every cloth must be predicted before ANY constraint is projected:
        // a two-way contact reads the partner's predicted positions, so a
        // partner still sitting at x would be solved against stale geometry.
        for (Index mi = 0; mi < numMeshes; ++mi) {
            auto& mesh = sceneObjects.meshes[mi];
            if (!clothLike(mesh.behaviorType)) continue;
            if (mesh.isStatic) continue;

            PR* x = mesh.state.x.ptr;
            PR* v = mesh.state.v.ptr;
            const PR* m    = mesh.state.m.ptr;
            const PR* ext  = mesh.externalForces.externalForces.ptr;
            const PR* mask = mesh.constraints.fixedParticles.ptr;
            if (!x || !v || !m || !mask) continue;

            const Index n = (Index)mesh.state.x.size / 3;
            if (n == 0) continue;

            // Per-mesh stiffness, mapped from this mesh's own spring
            // constants and corrected for the sweep count so changing
            // `iterations` does not change the effective stiffness
            // (Müller 2007 §3.3).
            PR kSpringStretch, kSpringBend;
            springConstantsOf(mesh, kSpringStretch, kSpringBend);
            auto toPbdK = [](PR k, PR ref) {
                if (ref <= PR(0)) return PR(1);
                const PR t = k / ref;
                return t < PR(0) ? PR(0) : (t > PR(1) ? PR(1) : t);
            };

            predBlocks[mi].resize((size_t)n * 3);
            cDispBlocks[mi].assign((size_t)n * 3, PR(0));

            const Index numFacets = (Index)mesh.adjacency.facets.size / 3;
            BendCache& bc = bendCache[mi];
            // Tear state must be settled BEFORE the bend decision: a heal
            // (Scene::pack regenerated the topology) has to poison the bend
            // cache too, otherwise the quads keep the holes the mesh no
            // longer has.
            // Maintained while tearing is ON, and also while an OLD tear
            // state is still populated: turning the toggle off must stop NEW
            // tears, not stitch the existing holes back up (the facets stay
            // degenerate either way, so resurrecting their edges would leave
            // a rubber band across a visible rip). resetTearing() is the only
            // way back.
            const bool haveTearState = !tearStates[mi].facetAlive.empty();
            TearState* ts = (tearEnabled || haveTearState)
                ? ensureTearState(mesh, mi, n, numFacets) : nullptr;
            if (ts && ts->bendDirty) { bc.dirty = true; ts->bendDirty = false; }
            if (bc.lifetimeId != mesh.lifetimeId || bc.numPoints != n
                || bc.numFacets != numFacets) {
                buildBendQuads(mesh, bc.quads);
                bc.lifetimeId = mesh.lifetimeId;
                bc.numPoints = n;
                bc.numFacets = numFacets;
                bc.dirty = false;
            } else if (bc.dirty) {
                // Torn since the last step: same counts, different facets.
                // Preserve phi0 (see rebuildBendQuadsPreservingPhi0).
                rebuildBendQuadsPreservingPhi0(mesh, bc);
                bc.dirty = false;
            }

            SolveCtx c;
            c.mi   = mi;
            c.x    = x;
            c.v    = v;
            c.p    = predBlocks[mi].data();
            c.cd   = cDispBlocks[mi].data();
            c.m    = m;
            c.ext  = ext;
            c.mask = mask;
            c.n    = n;
            c.thickness = thicknessOf(mesh.behaviorParams);
            c.kS = correctedK(toPbdK(kSpringStretch, stretchRef), iterations);
            c.kB = correctedK(toPbdK(kSpringBend,    bendRef),    iterations);
            c.edges    = mesh.adjacency.edges.ptr;
            c.restLen  = mesh.adjacency.restEdgeLengths.ptr;
            c.numEdges = c.edges ? (Index)mesh.adjacency.edges.size / 2 : 0;
            c.edgeAlive = (ts && (Index)ts->edgeAlive.size() == c.numEdges)
                ? ts->edgeAlive.data() : nullptr;
            c.facets    = mesh.adjacency.facets.ptr;
            c.numFacets = numFacets;
            c.bend = &bc;
            c.contactsForMesh = haveContacts && mi + 1 < (Index)off.size;
            c.colBase = c.contactsForMesh ? off.ptr[mi] : Index(0);

            // predict: symplectic Euler on the external forces only. Spring
            // forces are NOT applied here — the distance constraints below
            // replace them. Fixed particles (w == 0) stay put.
            for (Index i = 0; i < n; ++i) {
                const PR w = invMassOf(m, mask, i);
                const Vec3 accel = (w > PR(0) && ext) ? vertexAt(ext, i) * w : Vec3();
                const Vec3 vel = vertexAt(v, i) + accel * dt;
                vertexRef(c.p, i) = (w > PR(0)) ? vertexAt(x, i) + vel * dt
                                                : vertexAt(x, i);
            }

            ctxOfMesh[mi] = (int)ctxs.size();
            ctxs.push_back(c);
        }
        if (ctxs.empty()) return;

        // --- (1b) TEARING. Runs on the PREDICTED positions, after every mesh
        // has been predicted and BEFORE the first projection sweep. Order is
        // the whole point: once the sweeps run, the distance constraints have
        // already pulled every edge back to (near) rest, so post-solve strain
        // measures the solver's residual, not the load. The prediction is the
        // only place the overstretch is visible.
        //
        // The budget is PER MESH, not per scene: two independent cloths under
        // independent loads should not throttle each other, and a per-scene
        // cap would make tear speed depend on how many other cloths exist.
        if (tearEnabled && maxTearsPerStep > 0) {
            for (const SolveCtx& c : ctxs) {
                if (!c.edgeAlive || !c.restLen || !c.edges) continue;
                TearState& ts = tearStates[c.mi];
                Index* F = sceneObjects.meshes[c.mi].adjacency.facets.ptr;
                if (!F) continue;

                // Candidates, worst-first. Reused across meshes/steps so the
                // common (nothing overstretched) path allocates nothing.
                tearCands.clear();
                for (Index e = 0; e < c.numEdges; ++e) {
                    if (!ts.edgeAlive[e]) continue;
                    const Index ea = c.edges[e*2], eb = c.edges[e*2+1];
                    if (ea >= c.n || eb >= c.n || ea == eb) continue;
                    const PR rest = c.restLen[e];
                    if (!(rest > PR(1e-9))) continue;
                    // Both ends pinned: breaking it moves nothing (the
                    // projection is already a no-op there) and it is the
                    // pinned row the user hung the cloth from. Leave it.
                    if (invMassOf(c.m, c.mask, ea) <= PR(0)
                     && invMassOf(c.m, c.mask, eb) <= PR(0)) continue;
                    const PR len = (vertexAt(c.p, eb) - vertexAt(c.p, ea)).norm();
                    if (!std::isfinite(len)) continue;   // blown-up vertex
                    const PR strain = len / rest;
                    if (strain > tearRatio)
                        tearCands.push_back(TearCand{ strain, e });
                }
                if (tearCands.empty()) continue;

                const size_t budget = std::min((size_t)maxTearsPerStep,
                                               tearCands.size());
                std::partial_sort(tearCands.begin(),
                                  tearCands.begin() + (ptrdiff_t)budget,
                                  tearCands.end(),
                                  [](const TearCand& a, const TearCand& b) {
                                      return a.strain > b.strain;
                                  });

                // Local, NOT ts.facetsDirty: that flag is sticky until phase 2
                // consumes it, so reusing it here would re-run the secondary
                // sweep on steps where nothing broke.
                bool tore = false;
                for (size_t k = 0; k < budget; ++k) {
                    const Index e = tearCands[k].edge;
                    if (!ts.edgeAlive[e]) continue;   // killed as a dangler
                    ts.edgeAlive[e] = 0;
                    // Degenerate every facet that used this edge. Collapsing
                    // to (v0,v0,v0) is what removes the triangle from the GPU
                    // narrow phase (cross(v0,v1) == 0 => early return in
                    // narrow_pt_tri) and, after phase 2, from rendering —
                    // without resizing any shared buffer or shifting a single
                    // index, which is why it is safe to do mid-step on the
                    // Metal-shared facet array.
                    const Index fs[2] = { ts.edgeFacet0[e], ts.edgeFacet1[e] };
                    for (int j = 0; j < 2; ++j) {
                        const Index f = fs[j];
                        if (f == kNoFacet || f >= ts.numFacets) continue;
                        if (!ts.facetAlive[f]) continue;
                        ts.facetAlive[f] = 0;
                        const Index v0 = F[f*3+0];
                        F[f*3+1] = v0;
                        F[f*3+2] = v0;
                    }
                    ts.bendDirty = true;
                    ts.facetsDirty = true;
                    tore = true;
                    ++tearsThisStep;
                    ++tearCount;
                }

                // Secondary rule: an edge whose incident facets are ALL dead
                // now spans open air. Left alive it would keep pulling the
                // two lips of the hole together — a rubber band across a rip.
                // These are bookkeeping, not new tears, so they do NOT spend
                // budget and cannot cascade (their facets are already dead).
                if (tore) {
                    for (Index e = 0; e < c.numEdges; ++e) {
                        if (!ts.edgeAlive[e]) continue;
                        const Index f0 = ts.edgeFacet0[e];
                        if (f0 == kNoFacet || f0 >= ts.numFacets) continue;
                        if (ts.facetAlive[f0]) continue;
                        const Index f1 = ts.edgeFacet1[e];
                        if (f1 != kNoFacet && f1 < ts.numFacets
                            && ts.facetAlive[f1]) continue;
                        ts.edgeAlive[e] = 0;
                    }
                }
            }
        }

        // Standard PBD distance constraint C = |p_b - p_a| - rest.
        auto projectDistance = [&](const SolveCtx& c, Index a, Index b,
                                   PR rest, PR k) {
            if (a == b) return;
            const PR wa = invMassOf(c.m, c.mask, a);
            const PR wb = invMassOf(c.m, c.mask, b);
            const PR wsum = wa + wb;
            if (wsum <= PR(0)) return;
            const Vec3 d = vertexAt(c.p, b) - vertexAt(c.p, a);
            const PR len = d.norm();
            if (len < PR(1e-9)) return;
            // Paper form: Δp = ±(w/Σw)(len-rest)·d/len. The extra /len
            // (not in the paper's s) folds d's normalization into the
            // scalar so `d` stays unnormalized below — one divide
            // instead of normalizing the vector.
            const PR s = k * (len - rest) / (len * wsum);
            vertexRef(c.p, a) += d * (wa * s);
            vertexRef(c.p, b) -= d * (wb * s);
        };

        // eq 29: Δp_i = -(w_i sqrt(1-d^2)(acos(d) - phi0) / Σ_j w_j|q_j|^2) q_i
        auto projectBend = [&](const SolveCtx& c, const BendQuad& bq, PR k) {
            const Index idx[4] = { bq.p1, bq.p2, bq.p3, bq.p4 };
            PR w[4];
            PR wAny = PR(0);
            for (int j = 0; j < 4; ++j) {
                w[j] = invMassOf(c.m, c.mask, idx[j]); wAny += w[j];
            }
            if (wAny <= PR(0)) return;

            Vec3 n1, n2; PR l23, l24, d;
            if (!dihedral(c.p, bq.p1, bq.p2, bq.p3, bq.p4, n1, n2, l23, l24, d))
                return;
            const PR s2 = PR(1) - d*d;
            // Flat / fully folded: the arccos gradient is singular and the
            // correction vanishes anyway. Skipping is the standard guard.
            if (s2 < PR(1e-12)) return;
            const PR sq = std::sqrt(s2);

            const Vec3 p1 = vertexAt(c.p, bq.p1);
            const Vec3 P2 = vertexAt(c.p, bq.p2) - p1;
            const Vec3 P3 = vertexAt(c.p, bq.p3) - p1;
            const Vec3 P4 = vertexAt(c.p, bq.p4) - p1;

            // (25) q3 = (P2 x n2 + (n1 x P2) d) / |P2 x P3|
            const Vec3 q3 = (P2.cross(n2) + n1.cross(P2) * d) / l23;
            // (26) q4 = (P2 x n1 + (n2 x P2) d) / |P2 x P4|
            const Vec3 q4 = (P2.cross(n1) + n2.cross(P2) * d) / l24;
            // (27) q2 = -(P3 x n2 + (n1 x P3)d)/|P2 x P3|
            //          -(P4 x n1 + (n2 x P4)d)/|P2 x P4|
            const Vec3 q2 = -((P3.cross(n2) + n1.cross(P3) * d) / l23)
                            -((P4.cross(n1) + n2.cross(P4) * d) / l24);
            // (28) q1 = -q2 - q3 - q4
            const Vec3 q1 = -q2 - q3 - q4;

            const Vec3 qs[4] = { q1, q2, q3, q4 };
            PR denom = PR(0);
            for (int j = 0; j < 4; ++j) denom += w[j] * qs[j].dot(qs[j]);
            if (denom < PR(1e-12)) return;

            const PR C = std::acos(d) - bq.phi0;
            const PR scale = -k * sq * C / denom;
            for (int j = 0; j < 4; ++j) {
                if (w[j] <= PR(0)) continue;
                vertexRef(c.p, idx[j]) += qs[j] * (w[j] * scale);
            }
        };

        // Visit every DEDUPED contact of vertex i as (row, normal, d(x)).
        // Shared by the projection sweeps and the velocity update below so
        // the two can never disagree about which contacts are active.
        auto forEachContact = [&](const SolveCtx& c, Index i, auto&& fn) {
            if (!c.contactsForMesh) return;
            const Index begin = colOff.ptr[c.colBase + i];
            const Index end   = colOff.ptr[c.colBase + i + 1];
            for (Index r = begin; r < end; ++r) {
                // Dedup coincident (targetObj, targetTri) contacts — the
                // hash broad phases list one per incident query triangle,
                // and summing them over-corrects. Analytic rows carry
                // indexPair.target == 0 for every shape, so this key only
                // separates them because objPair.target is the collider's
                // mesh ARRAY INDEX (collider_pipeline_rework.md §4).
                bool dup = false;
                for (Index q = begin; q < r; ++q)
                    // .target == the kernel's .y lane (IndexPair is a union
                    // of {query,target} / {point,triangle}).
                    if (colFacet.ptr[q].objPair.target == colFacet.ptr[r].objPair.target &&
                        colFacet.ptr[q].indexPair.target == colFacet.ptr[r].indexPair.target) {
                        dup = true; break;
                    }
                if (dup) continue;

                const NarrowCollision& row = colFacet.ptr[r];
                const auto& nd = row.collisionNormalAndDistance;
                const Vec3 raw((PR)nd.x, (PR)nd.y, (PR)nd.z);
                const PR nlen = raw.norm();
                if (nlen < PR(1e-6)) continue;
                fn(row, raw / nlen, (PR)nd.w);
            }
        };

        // A contact is TWO-WAY when its target mesh is itself a live cloth in
        // this solve — including a SELF row (target == query mesh), which the
        // narrow phase emits with adjacency-adjacent facets already excluded.
        // Everything else (Float floor, analytic collider, Rigid, static
        // cloth) keeps the one-sided push: those targets have no predicted
        // block to correct, and their motion is not ours to change.
        auto twoWayTarget = [&](Index t) -> const SolveCtx* {
            if (t >= numMeshes) return nullptr;
            const int ci = ctxOfMesh[t];
            if (ci < 0) return nullptr;
            const auto& partner = sceneObjects.meshes[t];
            if (!clothLike(partner.behaviorType) || partner.isStatic) return nullptr;
            return &ctxs[(size_t)ci];
        };

        // --- (2) Gauss-Seidel constraint projection, ITERATION-OUTER /
        // MESH-INNER. Sweeping all meshes per iteration (instead of solving
        // one mesh to convergence and moving on) is what lets a cross-cloth
        // contact see, and be seen by, the partner's own internal constraints
        // within the same substep.
        for (int it = 0; it < iterations; ++it) {
            for (const SolveCtx& c : ctxs) {
                if (c.restLen) {
                    for (Index e = 0; e < c.numEdges; ++e) {
                        const Index ea = c.edges[e*2], eb = c.edges[e*2+1];
                        // `numEdges` is an UPPER BOUND: MeshGridInitializerParams
                        // allocates 2(k-1)k + 2(k-1)^2 edge slots but
                        // MeshAdjacencyInitializer only writes the deduplicated
                        // subset (161 of 210 for an 8x8 grid). The tail is
                        // uninitialised pool memory — floats reinterpreted as
                        // indices — so it must be skipped exactly the way
                        // MeshAdjacency::recomputeRestLengths already does
                        // (mesh_state.hpp `inRange`). Without this the solver
                        // reads x/m far outside the mesh and segfaults.
                        if (ea >= c.n || eb >= c.n) continue;
                        // TEARING: a broken edge has no cloth left to
                        // constrain. Skipping it here is what actually opens
                        // the hole — degenerating the facets alone would leave
                        // the two lips stitched by the stretch constraint.
                        if (c.edgeAlive && !c.edgeAlive[e]) continue;
                        projectDistance(c, ea, eb, c.restLen[e], c.kS);
                    }
                }
                if (c.kB > PR(0)) {
                    for (const auto& bq : c.bend->quads) projectBend(c, bq, c.kB);
                }

                // --- (3) contacts as inequality constraints. Same contact
                // set and same (distance < thickness) gate the integrate
                // kernel uses (D-016), applied to the predicted position.
                for (Index i = 0; c.contactsForMesh && i < c.n; ++i) {
                    forEachContact(c, i,
                        [&](const NarrowCollision& row, const Vec3& nrm, PR d0) {
                        const Index t = row.objPair.target;
                        const SolveCtx* pc = twoWayTarget(t);

                        if (!pc) {
                            // ONE-SIDED. The kernel pushes by
                            // (thickness - distance) ONCE per substep, and that
                            // push never feeds back into velocity. PBD's
                            // v = (p-x)/dt turns every position delta into
                            // velocity, so re-applying a FIXED push each
                            // Gauss-Seidel sweep accumulates and launches the
                            // cloth. Make the contact a real inequality
                            // instead: the narrow phase measured `distance`
                            // along n from x, so the signed distance of the
                            // CURRENT predicted position is
                            // distance + n.(p - x). Once the push satisfies
                            // it, later sweeps are no-ops.
                            //
                            // COUPLING: when the target is a DYNAMIC rigid
                            // body, this row is not one-sided at all — the
                            // deficit is split mass-weighted between the
                            // cloth vertex and the body, and the body's share
                            // accumulates in rigidDelta for the frame-end
                            // writeback to Bullet. Everything else (Float
                            // floor, static Rigid, analytic-tagged Float)
                            // keeps the exact old behavior: wB is 0 there, so
                            // share_c is 1 and the push below is byte-
                            // identical to what it was.
                            bool coupled = false;
                            PR wB = PR(0);
                            if (t < numMeshes) {
                                const auto& tm = sceneObjects.meshes[t];
                                if (tm.behaviorType == BehaviorType::Rigid
                                    && tm.applyGravity
                                    && tm.rigidBodyMass > PR(0)
                                    && tm.rigidBodyHandle
                                       != ysim::physics::kInvalidBodyHandle
                                    && (Index)rigidDelta.size() > t) {
                                    coupled = true;
                                    wB = PR(1) / tm.rigidBodyMass;
                                }
                            }
                            // A pinned cloth vertex (wc == 0) is immovable but
                            // must still be able to push a dynamic body, so the
                            // old unconditional pinned skip becomes a
                            // "nobody can move" test. With wB == 0 this
                            // reproduces the old skip exactly.
                            const PR wc = invMassOf(c.m, c.mask, i);
                            const PR wsum = wc + wB;
                            if (wsum <= PR(0)) return;
                            // The narrow phase measured d0 against the
                            // frame-frozen body, so the body's own accumulated
                            // motion this frame has to be subtracted here —
                            // otherwise every substep re-pays a deficit the
                            // body already absorbed and the pair runs away.
                            PR distance = d0
                                + nrm.dot(vertexAt(c.p, i) - vertexAt(c.x, i));
                            if (coupled) distance -= nrm.dot(rigidDelta[t]);
                            if (distance >= c.thickness) return;
                            const PR deficit = c.thickness - distance;
                            vertexRef(c.p, i) += nrm * (deficit * (wc / wsum));
                            if (coupled) {
                                // The body moves AGAINST the normal the cloth
                                // entered along.
                                Vec3 back = nrm * (deficit * (wB / wsum));
                                rigidDelta[t] -= back;
                                ++rigidCoupleCount;
                            }
                            return;
                        }

                        // TWO-WAY vertex-triangle contact (Müller 2007
                        // eq 12/13). Unlike the one-sided path this is
                        // re-linearized from the LIVE predicted geometry each
                        // sweep, so the triangle may move under the vertex.
                        // A pinned QUERY vertex is NOT skipped here — wq = 0
                        // simply hands the whole correction to the triangle.
                        if (!pc->facets) return;
                        const Index tri = row.indexPair.target;
                        if (tri >= pc->numFacets) return;
                        const Index i1 = pc->facets[tri*3+0];
                        const Index i2 = pc->facets[tri*3+1];
                        const Index i3 = pc->facets[tri*3+2];
                        if (i1 >= pc->n || i2 >= pc->n || i3 >= pc->n) return;
                        // TEARING: contact rows for THIS substep were built by
                        // the narrow phase before step() ran, so a row can name
                        // a facet the tear pass degenerated a few lines ago.
                        // The nlenG guard below already catches it (a
                        // (v,v,v) triangle has a zero cross product), but a
                        // duplicate-index test is cheaper, exact, and says
                        // out loud that a torn facet has no contact.
                        if (i1 == i2 || i2 == i3 || i1 == i3) return;
                        // Self rows never name an incident facet (the narrow
                        // phase excludes adjacency), but a degenerate row must
                        // not build a constraint of a vertex against itself.
                        if (t == c.mi && (i == i1 || i == i2 || i == i3)) return;

                        const Vec3 q  = vertexAt(c.p, i);
                        const Vec3 P1 = vertexAt(pc->p, i1);
                        const Vec3 P2 = vertexAt(pc->p, i2);
                        const Vec3 P3 = vertexAt(pc->p, i3);
                        const Vec3 e1 = P2 - P1, e2 = P3 - P1;
                        const Vec3 ngeom = e1.cross(e2);
                        const PR nlenG = ngeom.norm();
                        if (nlenG < PR(1e-12)) return;   // degenerate triangle
                        const Vec3 nh = ngeom / nlenG;

                        // eq (13) is eq (12) with the cross-product order
                        // flipped — i.e. a SIGN. Which one applies is the side
                        // the vertex entered from, and the narrow phase
                        // already oriented the row normal that way at substep
                        // start, so read sigma off it instead of guessing from
                        // the current (possibly already-corrected) geometry.
                        const PR sigma = (nrm.dot(nh) >= PR(0)) ? PR(1) : PR(-1);
                        const Vec3 sn = nh * sigma;
                        const PR h = std::max(c.thickness, pc->thickness);
                        const PR C = sn.dot(q - P1) - h;
                        // Inequality: already separated => nothing to do. This
                        // is what makes re-projecting every sweep safe.
                        if (C >= PR(0)) return;

                        // Barycentric coordinates of q's projection onto the
                        // triangle plane, clamped into the triangle so a
                        // near-edge / near-vertex contact still distributes a
                        // convex combination (Σb = 1) rather than an
                        // extrapolating one.
                        PR b1, b2, b3;
                        const PR d00 = e1.dot(e1), d01 = e1.dot(e2);
                        const PR d11 = e2.dot(e2);
                        const Vec3 vq = q - P1;
                        const PR d20 = vq.dot(e1), d21 = vq.dot(e2);
                        const PR bden = d00*d11 - d01*d01;
                        if (!(std::abs(bden) > PR(1e-20))) {
                            b1 = b2 = b3 = PR(1) / PR(3);
                        } else {
                            b2 = (d11*d20 - d01*d21) / bden;
                            b3 = (d00*d21 - d01*d20) / bden;
                            b1 = PR(1) - b2 - b3;
                            if (b1 < PR(0)) b1 = PR(0);
                            if (b2 < PR(0)) b2 = PR(0);
                            if (b3 < PR(0)) b3 = PR(0);
                            const PR bs = b1 + b2 + b3;
                            if (bs < PR(1e-12)) b1 = b2 = b3 = PR(1) / PR(3);
                            else { b1 /= bs; b2 /= bs; b3 /= bs; }
                        }

                        const PR wq = invMassOf(c.m,  c.mask,  i);
                        const PR w1 = invMassOf(pc->m, pc->mask, i1);
                        const PR w2 = invMassOf(pc->m, pc->mask, i2);
                        const PR w3 = invMassOf(pc->m, pc->mask, i3);
                        const PR wden = wq + b1*b1*w1 + b2*b2*w2 + b3*b3*w3;
                        if (wden < PR(1e-12)) return;   // both sides pinned
                        const PR s = C / wden;

                        // grad_q C = sigma*n, grad_pj C = -bj*sigma*n (the
                        // normal's OWN derivative is deliberately dropped —
                        // the standard PBD contact distribution; the
                        // constraint VALUE still follows the paper).
                        // Contact stiffness is 1: this is a hard contact,
                        // matching the one-sided push.
                        const Vec3 dq = sn * (-s * wq);
                        vertexRef(c.p,  i) += dq;
                        vertexRef(c.cd, i) += dq;
                        const Index ids[3] = { i1, i2, i3 };
                        const PR    bws[3] = { b1, b2, b3 };
                        const PR    wws[3] = { w1, w2, w3 };
                        for (int j = 0; j < 3; ++j) {
                            if (wws[j] <= PR(0)) continue;
                            const Vec3 dp = sn * (s * wws[j] * bws[j]);
                            vertexRef(pc->p,  ids[j]) += dp;
                            vertexRef(pc->cd, ids[j]) += dp;
                        }
                        ++twoWayContactCount;
                        if (t == c.mi) ++selfContactCount;
                    });
                }
            }
        }

        // --- (4) velocity update from the position delta, then commit.
        const PR invDt = PR(1) / dt;
        const PR velScale = PR(1) - damping;
        for (const SolveCtx& c : ctxs) {
            for (Index i = 0; i < c.n; ++i) {
                if (c.mask[i] == PR(0)) { vertexRef(c.v, i) = Vec3(); continue; }
                const Vec3 pi = vertexAt(c.p, i);
                if (!std::isfinite(pi.x) || !std::isfinite(pi.y)
                    || !std::isfinite(pi.z)) {
                    // Blown-up vertex: hold the last good position, kill its
                    // velocity. Same intent as the kernel's sanitize guard.
                    vertexRef(c.v, i) = Vec3();
                    ++sanitizeCount;
                    continue;
                }
                // DEPENETRATION IS NOT MOTION. v = (p-x)/dt reports the WHOLE
                // position delta as velocity, including the part of a contact
                // correction that only undid overlap the vertex was already in
                // when the substep began. A cloth born 0.3 m inside a box gets
                // rescued in one sweep, and at subh = 1/3000 that 0.31 m
                // teleport reads as ~930 m/s: the cloth leaves the scene
                // (measured, AC-10). physics.metal's integrate_cloth never has
                // this problem because its `pos += (thickness - distance) * n`
                // push is applied to the position ONLY and never touches vel.
                //
                // The invariant that separates the two cases: a contact may
                // reverse the approach the vertex actually made this substep
                // (that is ordinary collision response, restitution <= 1), but
                // it may never send the vertex out FASTER than it came in —
                // anything past that is pre-existing overlap being undone, i.e.
                // a teleport, and it must not become velocity. `freeDelta` is
                // the unconstrained predict-step displacement (the same
                // (v + a*dt)*dt the predict loop used), so -freeDelta.n is
                // exactly the approach along this contact's normal.
                const PR wi = invMassOf(c.m, c.mask, i);
                const Vec3 accel = (wi > PR(0) && c.ext)
                    ? vertexAt(c.ext, i) * wi : Vec3();
                const Vec3 freeDelta = (vertexAt(c.v, i) + accel * dt) * dt;
                Vec3 delta = pi - vertexAt(c.x, i);
                forEachContact(c, i,
                    [&](const NarrowCollision& row, const Vec3& nrm, PR) {
                    // Two-way rows are clamped once, below, off the ACCUMULATED
                    // displacement they actually produced; clamping them here
                    // as well would remove the same velocity twice.
                    if (twoWayTarget(row.objPair.target)) return;
                    const PR dn = delta.dot(nrm);
                    if (dn <= PR(0)) return;
                    const PR approach = std::max(PR(0), -freeDelta.dot(nrm));
                    if (dn > approach) delta -= nrm * (dn - approach);
                });
                // Same split, generalized: the two-way corrections of this
                // vertex sum to `cd`, whose direction is the only normal that
                // survives combining several triangles. Remove at most what
                // the contacts actually pushed (cdLen) beyond the approach
                // speed — anything more would eat the vertex's own motion.
                const Vec3 cd = vertexAt(c.cd, i);
                const PR cdLen = cd.norm();
                if (cdLen > PR(1e-12)) {
                    const Vec3 nc = cd / cdLen;
                    const PR dn = delta.dot(nc);
                    const PR approach = std::max(PR(0), -freeDelta.dot(nc));
                    if (dn > approach)
                        delta -= nc * std::min(dn - approach, cdLen);
                }
                vertexRef(c.v, i) = delta * (invDt * velScale);
                vertexRef(c.x, i) = pi;
            }
        }
    }
};
