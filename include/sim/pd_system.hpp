#pragma once
// Fragment header: included in order by main.cpp immediately AFTER
// sim/pbd_system.hpp and before sim/material_quat.hpp — the same slot, and
// for the same reason: collision_pod.hpp closes cleanly, while
// scene/spatial_hash/bvh/bruteforce each END with a dangling
// `template <...>` line that heads the NEXT fragment, so nothing can be
// inserted between those. PdSystem only needs Scene/GeneralMesh as declared
// template names (mesh_state.hpp forward-declares both); its bodies are
// dependent. Not independently compilable by design.
//
// CPU Projective Dynamics (Bouaziz et al. 2014) restricted to spring
// constraints, i.e. Liu, Bargteil, O'Brien, Kavan 2013 "Fast Simulation of
// Mass-Spring Systems" — block coordinate descent alternating a LOCAL
// per-spring projection with a GLOBAL linear solve whose matrix is CONSTANT
// for a fixed topology / timestep / stiffness, so it is factored once
// (sparse LDLT) and only back-substituted per iteration.
//
// THIRD sibling solver next to SymplecticSystem (GPU, force-based) and
// PbdSystem (CPU, position projection), selected at runtime by
// `Simulator::usePd` — the same A-option wiring PbdSystem uses (see
// docs/design/pbd-system-handoff.md §4): NOT a System-type swap, because
// main.cpp instantiates exactly one Simulator/System pair, and PD borrows the
// System's timing (`subh`) rather than duplicating it. One source of truth
// for h / subSteps / acctime.
//
// Runs on the CPU against the METAL shared-storage buffers directly
// (MeshState::x/v/m and Scene::packedCollisionData are all
// StorageModeShared, so `.ptr` is host-addressable).
//
// Why PD and not just PBD: PBD's stiffness is a dimensionless [0,1]
// projection weight that has to be mapped from the mesh's spring constants
// through an arbitrary reference (PbdSystem::stretchRef/bendRef). PD's
// weights ARE the spring constants — the same kstretch/kbend the force
// kernels integrate — so there is no ref-mapping here and none is wanted.
// The mesh inspector's numbers mean the same thing to the symplectic path
// and to PD.
//
// Contacts: NOT the paper's implicit collision handling (Bouaziz 2014 §6
// adds each collision as a unilateral half-space constraint INSIDE the
// energy, whose w·I lands on the system diagonal, and §8 keeps the
// factorization current with rank updates/downdates). A contact here never
// enters the matrix — that is the price of keeping ONE prefactored system
// for the whole run. Instead the same contact machinery PbdSystem::step
// carries (one-sided pushes with rigid-body coupling, two-way
// vertex-triangle rows including self contacts) is run as an inequality
// projection pass once per local/global iteration, after every mesh's
// global solve — see step (3) below for why that is safe and what it
// trades away.
//
// ponytail: one commitAndWait per substep, same cost the PBD path pays. The
// factorization is amortized but the solve is CPU-serial; port the local
// step + a Jacobi/Chebyshev global step to Metal if PD earns its keep.

#include <Eigen/Sparse>
#include <memory>
#include <unordered_map>

template <typename BE, typename PR>
struct PdSystem {};

template <typename PR>
struct PdSystem<METAL, PR> {
    // Local/global sweeps per substep. Liu 2013 converges monotonically but
    // slowly (block coordinate descent); 10 is the paper's interactive range.
    // Changing this does NOT change the effective stiffness — unlike PBD,
    // where each sweep applies a fraction of the correction, PD's global
    // solve targets the SAME energy minimum every iteration, so more
    // iterations only mean "closer to it". No correctedK equivalent needed.
    int iterations = 10;
    // Fraction of velocity removed per substep. 0 = no artificial damping
    // (the implicit-Euler solve is already dissipative).
    PR damping = PR(0);

    // Anomaly (NaN/Inf) counter for the frame; Simulator reads nothing from
    // this today — the guard's job is to keep a blown-up vertex from
    // poisoning the whole mesh. Same role as PbdSystem::sanitizeCount.
    uint32_t sanitizeCount = 0;

    // Two-way contact statistics for the LAST step() call. `selfContactCount`
    // is the subset whose target mesh is the query mesh itself. Reset at the
    // top of step(); read by the self-tests. Same contract as PbdSystem's.
    uint32_t twoWayContactCount = 0;
    uint32_t selfContactCount   = 0;
    // Number of one-sided contact rows that fed a coupled rigid body this
    // step(). Reset at the top of step() like the counters above.
    uint32_t rigidCoupleCount   = 0;

    // Soft-pin penalty weight. A pinned vertex gets wPin on its diagonal and
    // wPin * (its CURRENT position) in the RHS, instead of being eliminated
    // from the system. Two reasons: (a) the matrix stays CONSTANT when a pin
    // MOVES (kinematic / dragged pins only change the RHS, no refactor), and
    // (b) elimination would shrink the system and force a rebuild on every
    // pin toggle. SPD is preserved either way. wPin is ~4 decades above the
    // largest physical entry (k ~ 1e5-1e6, m/h² ~ 1e4 at subh = 1/480), so
    // the residual pin drift is ~1e-4 m INSIDE the solve — and it never
    // reaches the mesh anyway: the commit loop below refuses to write x for a
    // pinned vertex at all, exactly like PbdSystem step (4).
    static constexpr double kPinWeight = 1e8;

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
    // path applies at upload time). Used here UNMAPPED: in PD the spring
    // constant is literally the quadratic weight of that constraint's energy
    // term, so kstretch/kbend go straight into the global matrix.
    //
    // `shear` has no separate constraint set, same as PBD: adjacency exposes
    // edges (stretch) and opposite-vertex pairs (bend) only. On a
    // triangulated grid the shear diagonals ARE mesh edges, so they are
    // already weighted with the stretch coefficient.
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
    // a const buffer, vertexRef write-maps a mutable one.
    using Vec3 = tinym::vec3_base<PR>;
    using Vec3Ref = tinym::vec3_view_base<PR>;
    static Vec3 vertexAt(const PR* base, Index i) {
        return tinym::vec3_at(base + i * 3);
    }
    static Vec3Ref vertexRef(PR* base, Index i) { return Vec3Ref(base + i * 3); }

    // Inverse mass, zero for pinned vertices — a pinned vertex is infinitely
    // heavy, so the momentum target holds it in place and a contact moves
    // only its partner. Static (m/mask passed in) because a two-way contact
    // evaluates it against the PARTNER mesh's arrays, not just the mesh
    // being solved. Same contract as PbdSystem::invMassOf.
    static PR invMassOf(const PR* m, const PR* mask, Index i) {
        const PR mi3 = m[i * 3];
        return (mask[i] != PR(0) && mi3 > PR(0)) ? PR(1) / mi3 : PR(0);
    }

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
    // Identical contract to PbdSystem::rigidDelta — the Simulator applies
    // whichever solver's vector is live.
    std::vector<Vec3> rigidDelta;

    // Zero the per-frame rigid coupling accumulator. Called once per FRAME
    // by Simulator::update (not per substep).
    void beginFrameRigid(Index numMeshes) {
        rigidDelta.assign((size_t)numMeshes, Vec3());
    }

    // One spring = one PD constraint. `bend` selects which of the two mesh
    // coefficients weights it, and is stored INSTEAD of a baked k so that
    // editing kstretch/kbend in the inspector only refactors the matrix — it
    // must NOT re-measure `rest`, which for bend springs was sampled from the
    // pose at cache-build time (see buildSprings).
    struct Spring { Index a, b; double rest; bool bend; };
    static constexpr Index kConsumed = (Index)-1;

    // Per-mesh cached topology + prefactored global matrix, parallel to
    // Scene::meshes (same shape as PbdSystem::bendCache).
    struct MeshCache {
        int    lifetimeId = -2;   // -1 is a legal mesh value, so seed elsewhere
        Index  numPoints = 0, numFacets = 0, numEdges = 0;
        std::vector<Spring> springs;
        // Matrix-invalidating scalars, compared verbatim each substep.
        double h = 0.0, kS = -1.0, kB = -1.0;
        // Copies of the two per-vertex arrays that enter the matrix. The pin
        // mask decides which diagonals carry wPin and the masses ARE the
        // M/h² diagonal, so a change in either must refactor. O(n) memcmp
        // per substep, which is nothing next to the solve.
        std::vector<PR> pinMask, mass;
        // Eigen's solvers derive from internal::noncopyable, so the object
        // cannot live by value inside a std::vector element (resize needs
        // move-insertable). unique_ptr restores movability without copying
        // the factorization.
        std::unique_ptr<Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>> factor;
        bool valid = false;       // false => factorization failed, skip mesh
        // Per-substep solve state, kept here only so the allocations persist
        // across substeps (every value is rewritten each step). n x 3: the
        // system is scalar n x n and the three coordinates are INDEPENDENT
        // (a spring's energy is isotropic, so the same L couples x, y and
        // z), which is the whole reason one factorization serves all of
        // them. Eigen solves the three columns in one back-substitution
        // call. `q` must be PER MESH (not a shared scratch): a two-way
        // contact writes into the PARTNER mesh's iterate, so every active
        // cloth's block stays live for the whole iteration phase — the same
        // reason PbdSystem::predBlocks is per mesh.
        Eigen::MatrixXd rhsBase, rhs, q;
        // Accumulated two-way contact displacement (flat 3N), zeroed each
        // substep. The velocity update needs to know how much of a vertex's
        // position delta came from contact projection so depenetration does
        // not become velocity — PbdSystem::cDispBlocks, same idea.
        std::vector<PR> cd;
    };
    std::vector<MeshCache> cache;   // parallel to Scene::meshes

    // Everything one active cloth mesh needs during the iteration phase,
    // resolved once per substep. Contacts reach ACROSS contexts (the
    // partner's live iterate, masses, facets and thickness), which is why
    // this is a struct and not a set of per-mesh locals — the same shape as
    // PbdSystem::SolveCtx.
    struct SolveCtx {
        Index mi = 0;                 // Scene::meshes array index
        PR* x = nullptr;
        PR* v = nullptr;
        PR* cd = nullptr;             // MeshCache::cd.data()
        const PR* m = nullptr;
        const PR* ext = nullptr;
        const PR* mask = nullptr;
        Index n = 0;
        PR thickness = PR(0);
        double kS = 0.0, kB = 0.0;
        const Index* facets = nullptr;
        Index numFacets = 0;
        MeshCache* mc = nullptr;      // springs, factor, rhs/q matrices
        bool contactsForMesh = false;
        Index colBase = 0;
    };
    std::vector<SolveCtx> ctxs;      // one per ACTIVE cloth mesh
    std::vector<int> ctxOfMesh;      // mesh array index -> ctxs index, -1 = none

    // Stretch springs from the edge list + bend springs from the facet list.
    //
    // Bend model is Provot's opposite-vertex spring (the two vertices facing
    // each other across an interior edge), NOT a dihedral-angle constraint:
    // PD needs every constraint to be a quadratic distance-to-a-projected-set
    // energy, and the dihedral form is only expressible that way through a
    // per-quad linear operator (Bouaziz 2014 §5.4 bending element) — real
    // matrix blocks, not a scalar Laplacian entry. Deliberately out of scope;
    // the opposite-vertex spring resists the same deformation to first order
    // and keeps the assembly a plain graph Laplacian.
    //
    // Rest lengths: stretch reads adjacency.restEdgeLengths (the initializer's
    // measurement); bend is measured from the CURRENT positions here, the same
    // "re-measure rest from live geometry" convention
    // MeshAdjacency::recomputeRestLengths and PbdSystem::buildBendQuads use.
    static void buildSprings(const GeneralMesh<METAL, PR>& mesh, Index n,
                             std::vector<Spring>& out) {
        out.clear();
        const PR* pos = mesh.state.x.ptr;
        if (!pos || n == 0) return;

        const Index* edges   = mesh.adjacency.edges.ptr;
        const PR*    restLen = mesh.adjacency.restEdgeLengths.ptr;
        const Index  numEdges = edges ? (Index)mesh.adjacency.edges.size / 2 : 0;
        if (edges && restLen) {
            for (Index e = 0; e < numEdges; ++e) {
                const Index ea = edges[e*2], eb = edges[e*2+1];
                // `numEdges` is an UPPER BOUND: MeshGridInitializerParams
                // allocates 2(k-1)k + 2(k-1)^2 edge slots but
                // MeshAdjacencyInitializer only writes the deduplicated
                // subset (161 of 210 for an 8x8 grid). The tail is
                // uninitialised pool memory — floats reinterpreted as
                // indices — so it must be skipped exactly the way
                // MeshAdjacency::recomputeRestLengths already does
                // (mesh_state.hpp `inRange`). Without this the assembly
                // writes matrix entries far outside the system and the
                // triplet fill segfaults.
                if (ea >= n || eb >= n || ea == eb) continue;
                const double r = (double)restLen[e];
                if (!(r > 1e-9)) continue;
                out.push_back(Spring{ ea, eb, r, false });
            }
        }

        const Index* facets = mesh.adjacency.facets.ptr;
        if (!facets) return;
        const Index numFacets = (Index)mesh.adjacency.facets.size / 3;

        // edge (min,max) -> opposite vertex of the first face seen. Same map
        // PbdSystem::buildBendQuads builds; PD keeps only the opposite PAIR
        // (p3, p4) because its bend element is the p3-p4 spring.
        std::unordered_map<uint64_t, Index> firstOpp;
        firstOpp.reserve((size_t)numFacets * 3);

        for (Index f = 0; f < numFacets; ++f) {
            const Index v[3] = { facets[f*3+0], facets[f*3+1], facets[f*3+2] };
            if (v[0] >= n || v[1] >= n || v[2] >= n) continue;
            for (int e = 0; e < 3; ++e) {
                const Index a = v[e], b = v[(e+1)%3], o = v[(e+2)%3];
                const uint64_t key = a < b
                    ? ((uint64_t)a << 32) | (uint32_t)b
                    : ((uint64_t)b << 32) | (uint32_t)a;
                auto it = firstOpp.find(key);
                if (it == firstOpp.end()) { firstOpp.emplace(key, o); continue; }
                // Second incident face closes the pair. A third (non-manifold
                // geometry) finds the consumed marker and is dropped rather
                // than emitting a constraint against vertex (Index)-1.
                if (it->second == kConsumed) continue;
                const Index p3 = it->second, p4 = o;
                it->second = kConsumed;
                if (p3 == p4 || p3 >= n || p4 >= n) continue;
                const Vec3 d = vertexAt(pos, p4) - vertexAt(pos, p3);
                const double r = (double)d.norm();
                if (!(r > 1e-9)) continue;
                out.push_back(Spring{ p3, p4, r, true });
            }
        }
    }

    void step(Scene<METAL, PR>& sceneObjects, PR dt) {
        if (dt <= PR(0)) return;
        // Flush + wait: x, and the narrow-phase contact arrays this substep's
        // broad/narrow dispatches just filled, must be GPU-complete before the
        // CPU reads them. Same reason PbdSystem::step opens with it.
        MetalGlobalContext::commitAndWait();

        twoWayContactCount = 0;
        selfContactCount   = 0;
        rigidCoupleCount   = 0;

        auto& off      = Scene<METAL, PR>::packedMeshData.statesOffsets;
        auto& colFacet = Scene<METAL, PR>::packedCollisionData.vertColFacets;
        auto& colOff   = Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets;
        const bool haveContacts = colFacet.ptr && colOff.ptr && off.ptr;

        const double h = (double)dt;
        const double invH2 = 1.0 / (h * h);
        const Index numMeshes = (Index)sceneObjects.meshes.size();

        if (cache.size() != sceneObjects.meshes.size())
            cache.resize(sceneObjects.meshes.size());
        ctxs.clear();
        ctxOfMesh.assign((size_t)numMeshes, -1);

        // --- (1) per mesh: cache/refactor the global matrix, build the
        // momentum target and warm start. Every active cloth must reach its
        // warm-started iterate before ANY contact is projected: a two-way
        // contact reads the partner's live iterate, so a partner still
        // sitting at x would be solved against stale geometry — the same
        // predict-all-first rule PbdSystem::step (1) documents.
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

            PR kSpringStretch, kSpringBend;
            springConstantsOf(mesh, kSpringStretch, kSpringBend);
            const double kS = (double)kSpringStretch;
            const double kB = (double)kSpringBend;

            MeshCache& mc = cache[mi];
            const Index numFacets = (Index)mesh.adjacency.facets.size / 3;
            const Index numEdges  = mesh.adjacency.edges.ptr
                                  ? (Index)mesh.adjacency.edges.size / 2 : 0;

            // (a) Topology: rebuild only on mesh identity / count changes, so
            // the bend rest lengths measured from the live pose are not
            // silently re-sampled from a deformed configuration mid-run.
            bool refactor = false;
            if (mc.lifetimeId != mesh.lifetimeId || mc.numPoints != n
                || mc.numFacets != numFacets || mc.numEdges != numEdges) {
                buildSprings(mesh, n, mc.springs);
                mc.lifetimeId = mesh.lifetimeId;
                mc.numPoints  = n;
                mc.numFacets  = numFacets;
                mc.numEdges   = numEdges;
                refactor = true;
            }

            // (b) Matrix scalars + the two per-vertex arrays that enter it.
            if (mc.h != h || mc.kS != kS || mc.kB != kB) refactor = true;
            if (mc.pinMask.size() != (size_t)n || mc.mass.size() != (size_t)n) {
                refactor = true;
            } else {
                for (Index i = 0; i < n && !refactor; ++i)
                    if (mc.pinMask[i] != mask[i] || mc.mass[i] != m[i * 3])
                        refactor = true;
            }

            if (refactor) {
                mc.h = h; mc.kS = kS; mc.kB = kB;
                mc.pinMask.resize(n);
                mc.mass.resize(n);
                for (Index i = 0; i < n; ++i) {
                    mc.pinMask[i] = mask[i];
                    mc.mass[i]    = m[i * 3];
                }

                // A = M/h² + L + wPin·P, scalar n x n.
                //   M/h²: the implicit-Euler momentum term. A massless
                //         non-pinned vertex would leave a zero diagonal, and
                //         if it is also spring-isolated the row is singular,
                //         so the mass is floored at a value far below every
                //         physical entry (it changes nothing where a real
                //         mass or a spring exists).
                //   L:    graph Laplacian of the spring set, weight k.
                //   wPin: soft pin, see kPinWeight.
                std::vector<Eigen::Triplet<double>> trip;
                trip.reserve((size_t)n + mc.springs.size() * 4);
                for (Index i = 0; i < n; ++i) {
                    const double mi3 = (double)m[i * 3];
                    double diag = (mi3 > 0.0 ? mi3 : 1e-9) * invH2;
                    if (mask[i] == PR(0)) diag += kPinWeight;
                    trip.emplace_back((int)i, (int)i, diag);
                }
                for (const auto& s : mc.springs) {
                    const double k = s.bend ? kB : kS;
                    if (!(k > 0.0)) continue;
                    trip.emplace_back((int)s.a, (int)s.a,  k);
                    trip.emplace_back((int)s.b, (int)s.b,  k);
                    trip.emplace_back((int)s.a, (int)s.b, -k);
                    trip.emplace_back((int)s.b, (int)s.a, -k);
                }
                Eigen::SparseMatrix<double> A((int)n, (int)n);
                // setFromTriplets SUMS duplicates, which is exactly the
                // accumulate-per-incident-spring assembly above.
                A.setFromTriplets(trip.begin(), trip.end());
                mc.factor = std::make_unique<
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>();
                mc.factor->compute(A);
                mc.valid = (mc.factor->info() == Eigen::Success);
                if (!mc.valid)
                    std::cerr << "[PdSystem] LDLT factorization failed on mesh "
                              << mesh.id << " — skipping it this run\n";
            }
            // A mesh whose matrix would not factor is left to the previous
            // frame's state rather than integrated with garbage.
            if (!mc.valid || !mc.factor) continue;

            mc.rhsBase.resize((int)n, 3);
            mc.rhs.resize((int)n, 3);
            mc.q.resize((int)n, 3);
            mc.cd.assign((size_t)n * 3, PR(0));

            // Momentum target s = x + h·v + h²·M⁻¹·f_ext. This is the ONLY
            // place external forces enter; spring forces are the energy
            // terms the local/global loop minimizes, not an explicit force.
            // Pinned vertices target their current position. The iterate q
            // starts AT the momentum target — the unconstrained prediction,
            // which is also the descent's warm start (Bouaziz Alg. 1 l.2).
            for (Index i = 0; i < n; ++i) {
                const PR w = invMassOf(m, mask, i);
                const Vec3 xi = vertexAt(x, i);
                Vec3 si = xi;
                if (w > PR(0)) {
                    const Vec3 accel = ext ? vertexAt(ext, i) * w : Vec3();
                    si = xi + (vertexAt(v, i) + accel * dt) * dt;
                }
                mc.q((int)i, 0) = (double)si.x;
                mc.q((int)i, 1) = (double)si.y;
                mc.q((int)i, 2) = (double)si.z;

                const double mi3 = (double)m[i * 3];
                const double wm = (mi3 > 0.0 ? mi3 : 1e-9) * invH2;
                mc.rhsBase((int)i, 0) = wm * (double)si.x;
                mc.rhsBase((int)i, 1) = wm * (double)si.y;
                mc.rhsBase((int)i, 2) = wm * (double)si.z;
                if (mask[i] == PR(0)) {
                    // Pin term uses the CURRENT position, so a pin that moved
                    // since the factorization is honoured without a refactor.
                    mc.rhsBase((int)i, 0) += kPinWeight * (double)xi.x;
                    mc.rhsBase((int)i, 1) += kPinWeight * (double)xi.y;
                    mc.rhsBase((int)i, 2) += kPinWeight * (double)xi.z;
                }
            }

            SolveCtx c;
            c.mi   = mi;
            c.x    = x;
            c.v    = v;
            c.cd   = mc.cd.data();
            c.m    = m;
            c.ext  = ext;
            c.mask = mask;
            c.n    = n;
            c.thickness = thicknessOf(mesh.behaviorParams);
            c.kS = kS;
            c.kB = kB;
            c.facets    = mesh.adjacency.facets.ptr;
            c.numFacets = numFacets;
            c.mc = &mc;
            c.contactsForMesh = haveContacts && mi + 1 < (Index)off.size;
            c.colBase = c.contactsForMesh ? off.ptr[mi] : Index(0);
            ctxOfMesh[mi] = (int)ctxs.size();
            ctxs.push_back(c);
        }
        if (ctxs.empty()) return;

        // Read/write one vertex of a context's live iterate. The iterate is
        // double (the solve's precision); contact math runs in PR like PBD's
        // and the corrections are added back into the double iterate.
        auto qVec = [](const SolveCtx& c, Index i) -> Vec3 {
            return Vec3((PR)c.mc->q((int)i, 0),
                        (PR)c.mc->q((int)i, 1),
                        (PR)c.mc->q((int)i, 2));
        };
        auto qAdd = [](const SolveCtx& c, Index i, const Vec3& d) {
            c.mc->q((int)i, 0) += (double)d.x;
            c.mc->q((int)i, 1) += (double)d.y;
            c.mc->q((int)i, 2) += (double)d.z;
        };

        // Visit every DEDUPED contact of vertex i as (row, normal, d(x)).
        // Shared by the contact projection and the velocity update below
        // so the two can never disagree about which contacts are active.
        // Lifted verbatim from PbdSystem::step — see there for the dedup
        // rationale (one row per incident query triangle would
        // over-correct; analytic rows carry indexPair.target == 0 for
        // every shape so the key leans on objPair.target being the
        // collider's mesh ARRAY INDEX, collider_pipeline_rework.md §4).
        auto forEachContact = [&](const SolveCtx& c, Index i, auto&& fn) {
            if (!c.contactsForMesh) return;
            const Index begin = colOff.ptr[c.colBase + i];
            const Index end   = colOff.ptr[c.colBase + i + 1];
            for (Index r = begin; r < end; ++r) {
                bool dup = false;
                for (Index qi = begin; qi < r; ++qi)
                    if (colFacet.ptr[qi].objPair.target == colFacet.ptr[r].objPair.target &&
                        colFacet.ptr[qi].indexPair.target == colFacet.ptr[r].indexPair.target) {
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
        // cloth) keeps the one-sided push: those targets have no iterate to
        // correct, and their motion is not ours to change (except the
        // rigid-coupled share below).
        auto twoWayTarget = [&](Index t) -> const SolveCtx* {
            if (t >= numMeshes) return nullptr;
            const int ci = ctxOfMesh[t];
            if (ci < 0) return nullptr;
            const auto& partner = sceneObjects.meshes[t];
            if (!clothLike(partner.behaviorType) || partner.isStatic) return nullptr;
            return &ctxs[(size_t)ci];
        };

        // --- (2) local/global block coordinate descent (Liu 2013 §3),
        // ITERATION-OUTER / MESH-INNER like PbdSystem step (2): sweeping all
        // meshes per iteration is what lets a cross-cloth contact see, and be
        // seen by, the partner's own internal constraints within the substep.
        for (int it = 0; it < iterations; ++it) {
            for (SolveCtx& c : ctxs) {
                MeshCache& mc = *c.mc;
                mc.rhs = mc.rhsBase;
                // LOCAL: each spring's auxiliary variable d is the closest
                // point of its constraint manifold (the sphere of radius
                // `rest`) to the current edge vector — i.e. the edge
                // direction scaled to rest length. Closed form, per spring,
                // embarrassingly parallel.
                for (const auto& s : mc.springs) {
                    const double k = s.bend ? c.kB : c.kS;
                    if (!(k > 0.0)) continue;
                    const int a = (int)s.a, b = (int)s.b;
                    double ex = mc.q(a,0) - mc.q(b,0);
                    double ey = mc.q(a,1) - mc.q(b,1);
                    double ez = mc.q(a,2) - mc.q(b,2);
                    const double len = std::sqrt(ex*ex + ey*ey + ez*ez);
                    // Coincident endpoints: the direction is undefined. Leave
                    // d equal to the (zero) edge vector so the constraint
                    // contributes NOTHING this iteration. Projecting onto an
                    // arbitrary direction would inject a full rest-length
                    // impulse along a random axis; pulling to coincide would
                    // be worse still.
                    if (len > 1e-9) {
                        const double sc = s.rest / len;
                        ex *= sc; ey *= sc; ez *= sc;
                    }
                    mc.rhs(a,0) += k * ex; mc.rhs(a,1) += k * ey; mc.rhs(a,2) += k * ez;
                    mc.rhs(b,0) -= k * ex; mc.rhs(b,1) -= k * ey; mc.rhs(b,2) -= k * ez;
                }
                // GLOBAL: one back-substitution against the prefactored A,
                // three RHS columns at once. This is the step that makes PD
                // implicit — every vertex sees every other through A⁻¹, which
                // is why stiff springs do not explode the way the symplectic
                // path does at the same substep count.
                mc.q = mc.factor->solve(mc.rhs);
            }

            // --- (3) contacts, ONCE PER ITERATION, after every mesh's global
            // solve. The machinery is PbdSystem step (3)'s, ported onto the
            // PD iterate: one-sided pushes with rigid-body coupling, and
            // two-way vertex-triangle rows (Müller 2007 eq 12/13) when the
            // target is a live cloth — self rows included.
            //
            // Re-applying every iteration is safe because each row is a real
            // INEQUALITY re-linearized against the live iterate (the narrow
            // phase measured `distance` along n from x, so d(q) = d0 +
            // n·(q - x)): once separated it is a no-op, nothing accumulates.
            // Running contacts INSIDE the loop — instead of once post-solve,
            // as this solver first shipped — lets the next iteration's
            // spring projections see the pushed geometry (the stretch a push
            // creates is re-solved instead of committed), and leaves the
            // LAST operation of the loop a contact pass, so the committed q
            // satisfies the substep's contact planes.
            //
            // What this is NOT: the paper's implicit collision (Bouaziz 2014
            // §6) puts the contact's w·I on the system diagonal, so its
            // global solve balances contact against momentum and elasticity
            // in one system. Here the contact never enters the matrix (see
            // header), so each global solve pulls a penetrating vertex back
            // toward the energy minimum and the projection re-pushes it —
            // the alternation converges to a compromise, and the
            // depenetration/velocity split in (4) keeps what was only
            // overlap-undoing out of the velocity, exactly like PBD.
            for (SolveCtx& c : ctxs) {
                if (!c.contactsForMesh) continue;
                for (Index i = 0; i < c.n; ++i) {
                    forEachContact(c, i,
                        [&](const NarrowCollision& row, const Vec3& nrm, PR d0) {
                        const Index t = row.objPair.target;
                        const SolveCtx* pc = twoWayTarget(t);

                        if (!pc) {
                            // ONE-SIDED, with cloth→rigid coupling. Ported
                            // from PbdSystem step (3) — see there for the
                            // full rationale. A pinned cloth vertex (wc == 0)
                            // is immovable but must still be able to push a
                            // dynamic body, so the skip is a "nobody can
                            // move" test, not a pin test.
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
                            const PR wc = invMassOf(c.m, c.mask, i);
                            const PR wsum = wc + wB;
                            if (wsum <= PR(0)) return;
                            // The narrow phase measured d0 against the
                            // frame-frozen body, so the body's own
                            // accumulated motion this frame has to be
                            // subtracted — otherwise every iteration re-pays
                            // a deficit the body already absorbed and the
                            // pair runs away.
                            PR distance = d0
                                + nrm.dot(qVec(c, i) - vertexAt(c.x, i));
                            if (coupled) distance -= nrm.dot(rigidDelta[t]);
                            if (distance >= c.thickness) return;
                            const PR deficit = c.thickness - distance;
                            qAdd(c, i, nrm * (deficit * (wc / wsum)));
                            if (coupled) {
                                // The body moves AGAINST the normal the
                                // cloth entered along.
                                rigidDelta[t] -= nrm * (deficit * (wB / wsum));
                                ++rigidCoupleCount;
                            }
                            return;
                        }

                        // TWO-WAY vertex-triangle contact (Müller 2007
                        // eq 12/13), re-linearized from the LIVE iterates
                        // each iteration so the triangle may move under the
                        // vertex. A pinned QUERY vertex is NOT skipped here —
                        // wq = 0 simply hands the whole correction to the
                        // triangle.
                        if (!pc->facets) return;
                        const Index tri = row.indexPair.target;
                        if (tri >= pc->numFacets) return;
                        const Index i1 = pc->facets[tri*3+0];
                        const Index i2 = pc->facets[tri*3+1];
                        const Index i3 = pc->facets[tri*3+2];
                        if (i1 >= pc->n || i2 >= pc->n || i3 >= pc->n) return;
                        // Self rows never name an incident facet (the narrow
                        // phase excludes adjacency), but a degenerate row must
                        // not build a constraint of a vertex against itself.
                        if (t == c.mi && (i == i1 || i == i2 || i == i3)) return;

                        const Vec3 qv = qVec(c, i);
                        const Vec3 P1 = qVec(*pc, i1);
                        const Vec3 P2 = qVec(*pc, i2);
                        const Vec3 P3 = qVec(*pc, i3);
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
                        const PR hthk = std::max(c.thickness, pc->thickness);
                        const PR C = sn.dot(qv - P1) - hthk;
                        // Inequality: already separated => nothing to do. This
                        // is what makes re-projecting every iteration safe.
                        if (C >= PR(0)) return;

                        // Barycentric coordinates of qv's projection onto the
                        // triangle plane, clamped into the triangle so a
                        // near-edge / near-vertex contact still distributes a
                        // convex combination (Σb = 1) rather than an
                        // extrapolating one.
                        PR b1, b2, b3;
                        const PR d00 = e1.dot(e1), d01 = e1.dot(e2);
                        const PR d11 = e2.dot(e2);
                        const Vec3 vq = qv - P1;
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
                        qAdd(c, i, dq);
                        vertexRef(c.cd, i) += dq;
                        const Index ids[3] = { i1, i2, i3 };
                        const PR    bws[3] = { b1, b2, b3 };
                        const PR    wws[3] = { w1, w2, w3 };
                        for (int j = 0; j < 3; ++j) {
                            if (wws[j] <= PR(0)) continue;
                            const Vec3 dp = sn * (s * wws[j] * bws[j]);
                            qAdd(*pc, ids[j], dp);
                            vertexRef(pc->cd, ids[j]) += dp;
                        }
                        ++twoWayContactCount;
                        if (t == c.mi) ++selfContactCount;
                    });
                }
            }
        }

        // --- (4) velocity update from the position delta, then commit.
        // Logic copied from pbd_system.hpp step (4) — that header is the
        // source of truth for WHY, and is deliberately not included or
        // modified from here. Short version:
        //   * pinned vertices keep x EXACTLY (never written) and lose v,
        //     which is also what makes the soft pin above harmless;
        //   * a non-finite iterate holds the last good position and
        //     zeroes velocity instead of poisoning the mesh;
        //   * DEPENETRATION IS NOT MOTION: v = (q-x)/h reports the whole
        //     delta as velocity, including the part of a contact push
        //     that only undid overlap the vertex was ALREADY in when the
        //     substep began. A cloth born 0.3 m inside a box would read
        //     ~930 m/s at subh = 1/3000 and leave the scene (measured,
        //     AC-10). The invariant: a contact may reverse the approach
        //     the vertex actually made this substep, but may never send
        //     it out FASTER than it came in. freeDelta is the
        //     unconstrained (v + a·h)·h displacement, so -freeDelta·n is
        //     exactly the approach along that contact's normal. One-sided
        //     rows clamp per row; two-way rows clamp once, off the
        //     ACCUMULATED displacement they actually produced (cd) —
        //     clamping them per row as well would remove the same
        //     velocity twice.
        const PR invDt = PR(1) / dt;
        const PR velScale = PR(1) - damping;
        for (const SolveCtx& c : ctxs) {
            const MeshCache& mc = *c.mc;
            for (Index i = 0; i < c.n; ++i) {
                if (c.mask[i] == PR(0)) { vertexRef(c.v, i) = Vec3(); continue; }
                const Vec3 pi((PR)mc.q((int)i,0), (PR)mc.q((int)i,1),
                              (PR)mc.q((int)i,2));
                if (!std::isfinite(pi.x) || !std::isfinite(pi.y)
                    || !std::isfinite(pi.z)) {
                    vertexRef(c.v, i) = Vec3();
                    ++sanitizeCount;
                    continue;
                }
                const PR wi = invMassOf(c.m, c.mask, i);
                const Vec3 accel = (wi > PR(0) && c.ext)
                    ? vertexAt(c.ext, i) * wi : Vec3();
                const Vec3 freeDelta = (vertexAt(c.v, i) + accel * dt) * dt;
                Vec3 delta = pi - vertexAt(c.x, i);
                forEachContact(c, i,
                    [&](const NarrowCollision& row, const Vec3& nrm, PR) {
                    if (twoWayTarget(row.objPair.target)) return;
                    const PR dn = delta.dot(nrm);
                    if (dn <= PR(0)) return;
                    const PR approach = std::max(PR(0), -freeDelta.dot(nrm));
                    if (dn > approach) delta -= nrm * (dn - approach);
                });
                // Same split, generalized: the two-way corrections of this
                // vertex sum to `cd`, whose direction is the only normal
                // that survives combining several triangles. Remove at most
                // what the contacts actually pushed (cdLen) beyond the
                // approach speed — anything more would eat the vertex's own
                // motion.
                const Vec3 cdv = vertexAt(c.cd, i);
                const PR cdLen = cdv.norm();
                if (cdLen > PR(1e-12)) {
                    const Vec3 nc = cdv / cdLen;
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
