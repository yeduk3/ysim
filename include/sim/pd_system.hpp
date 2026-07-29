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
// constraints, i.e. Liu, Bargteil, Popović 2013 "Fast Simulation of
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
    };
    std::vector<MeshCache> cache;   // parallel to Scene::meshes

    // Scratch, grown and reused across substeps. n x 3: the system is scalar
    // n x n and the three coordinates are INDEPENDENT (a spring's energy is
    // isotropic, so the same L couples x, y and z), which is the whole reason
    // one factorization serves all of them. Eigen solves the three columns in
    // one back-substitution call.
    Eigen::MatrixXd sTarget, rhsBase, rhs, q;

    // Stretch springs from the edge list + bend springs from the facet list.
    //
    // Bend model is Provot's opposite-vertex spring (the two vertices facing
    // each other across an interior edge), NOT a dihedral-angle constraint:
    // PD needs every constraint to be a quadratic distance-to-a-projected-set
    // energy, and the dihedral form is only expressible that way through a
    // per-quad linear operator (Bouaziz 2014 §4.3 bending element) — real
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

        auto& off      = Scene<METAL, PR>::packedMeshData.statesOffsets;
        auto& colFacet = Scene<METAL, PR>::packedCollisionData.vertColFacets;
        auto& colOff   = Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets;
        const bool haveContacts = colFacet.ptr && colOff.ptr && off.ptr;

        const double h = (double)dt;
        const double invH2 = 1.0 / (h * h);

        if (cache.size() != sceneObjects.meshes.size())
            cache.resize(sceneObjects.meshes.size());

        for (Index mi = 0; mi < (Index)sceneObjects.meshes.size(); ++mi) {
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
            const PR thickness = thicknessOf(mesh.behaviorParams);

            PR kSpringStretch, kSpringBend;
            springConstantsOf(mesh, kSpringStretch, kSpringBend);
            const double kS = (double)kSpringStretch;
            const double kB = (double)kSpringBend;

            // Inverse mass, zero for pinned vertices — a pinned vertex is
            // infinitely heavy, so the momentum target holds it in place and
            // external forces never enter its row.
            auto invMass = [&](Index i) -> PR {
                const PR mi3 = m[i * 3];
                return (mask[i] != PR(0) && mi3 > PR(0)) ? PR(1) / mi3 : PR(0);
            };

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
                    const double mi = (double)m[i * 3];
                    double diag = (mi > 0.0 ? mi : 1e-9) * invH2;
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

            sTarget.resize((int)n, 3);
            rhsBase.resize((int)n, 3);
            rhs.resize((int)n, 3);

            // --- (1) momentum target s = x + h·v + h²·M⁻¹·f_ext. This is the
            // ONLY place external forces enter; spring forces are the energy
            // terms the local/global loop minimizes, not an explicit force.
            // Pinned vertices target their current position.
            for (Index i = 0; i < n; ++i) {
                const PR w = invMass(i);
                const Vec3 xi = vertexAt(x, i);
                Vec3 si = xi;
                if (w > PR(0)) {
                    const Vec3 accel = ext ? vertexAt(ext, i) * w : Vec3();
                    si = xi + (vertexAt(v, i) + accel * dt) * dt;
                }
                sTarget((int)i, 0) = (double)si.x;
                sTarget((int)i, 1) = (double)si.y;
                sTarget((int)i, 2) = (double)si.z;

                const double mi = (double)m[i * 3];
                const double wm = (mi > 0.0 ? mi : 1e-9) * invH2;
                rhsBase((int)i, 0) = wm * (double)si.x;
                rhsBase((int)i, 1) = wm * (double)si.y;
                rhsBase((int)i, 2) = wm * (double)si.z;
                if (mask[i] == PR(0)) {
                    // Pin term uses the CURRENT position, so a pin that moved
                    // since the factorization is honoured without a refactor.
                    rhsBase((int)i, 0) += kPinWeight * (double)xi.x;
                    rhsBase((int)i, 1) += kPinWeight * (double)xi.y;
                    rhsBase((int)i, 2) += kPinWeight * (double)xi.z;
                }
            }

            // --- (2) local/global block coordinate descent (Liu 2013 §3).
            // q starts at the momentum target — the unconstrained prediction,
            // which is also the descent's warm start.
            q = sTarget;
            for (int it = 0; it < iterations; ++it) {
                rhs = rhsBase;
                // LOCAL: each spring's auxiliary variable d is the closest
                // point of its constraint manifold (the sphere of radius
                // `rest`) to the current edge vector — i.e. the edge
                // direction scaled to rest length. Closed form, per spring,
                // embarrassingly parallel.
                for (const auto& s : mc.springs) {
                    const double k = s.bend ? kB : kS;
                    if (!(k > 0.0)) continue;
                    const int a = (int)s.a, b = (int)s.b;
                    double ex = q(a,0) - q(b,0);
                    double ey = q(a,1) - q(b,1);
                    double ez = q(a,2) - q(b,2);
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
                    rhs(a,0) += k * ex; rhs(a,1) += k * ey; rhs(a,2) += k * ez;
                    rhs(b,0) -= k * ex; rhs(b,1) -= k * ey; rhs(b,2) -= k * ez;
                }
                // GLOBAL: one back-substitution against the prefactored A,
                // three RHS columns at once. This is the step that makes PD
                // implicit — every vertex sees every other through A⁻¹, which
                // is why stiff springs do not explode the way the symplectic
                // path does at the same substep count.
                q = mc.factor->solve(rhs);
            }

            // Visit every DEDUPED contact of vertex i as (normal, d(x)).
            // Shared by the contact projection and the velocity update below
            // so the two can never disagree about which contacts are active.
            // Lifted verbatim from PbdSystem::step — see there for the dedup
            // rationale (one row per incident query triangle would
            // over-correct; analytic rows carry indexPair.target == 0 for
            // every shape so the key leans on objPair.target being the
            // collider's mesh ARRAY INDEX, collider_pipeline_rework.md §4).
            const bool contactsForMesh = haveContacts && mi + 1 < (Index)off.size;
            const Index colBase = contactsForMesh ? off.ptr[mi] : Index(0);
            auto forEachContact = [&](Index i, auto&& fn) {
                if (!contactsForMesh) return;
                const Index begin = colOff.ptr[colBase + i];
                const Index end   = colOff.ptr[colBase + i + 1];
                for (Index c = begin; c < end; ++c) {
                    bool dup = false;
                    for (Index qi = begin; qi < c; ++qi)
                        if (colFacet.ptr[qi].objPair.target == colFacet.ptr[c].objPair.target &&
                            colFacet.ptr[qi].indexPair.target == colFacet.ptr[c].indexPair.target) {
                            dup = true; break;
                        }
                    if (dup) continue;

                    const auto& nd = colFacet.ptr[c].collisionNormalAndDistance;
                    const Vec3 raw((PR)nd.x, (PR)nd.y, (PR)nd.z);
                    const PR nlen = raw.norm();
                    if (nlen < PR(1e-6)) continue;
                    fn(raw / nlen, (PR)nd.w);
                }
            };

            // --- (3) contacts, POST-solve and applied ONCE.
            // Not folded into the iteration: a contact is an inequality, and
            // PD's global step would immediately pull the vertex back toward
            // the momentum target, so re-projecting it every sweep would
            // fight the solve and (via v = (q-x)/h) accumulate into velocity
            // — the launch failure PbdSystem documents at step (3)/(4). One
            // post-solve projection is the same treatment physics.metal's
            // integrate_cloth gives it: position only.
            //
            // Signed distance of the CURRENT iterate: the narrow phase
            // measured `distance` along n from x, so d(q) = d0 + n·(q - x).
            if (contactsForMesh) {
                for (Index i = 0; i < n; ++i) {
                    if (mask[i] == PR(0)) continue;
                    const Vec3 xi = vertexAt(x, i);
                    forEachContact(i, [&](const Vec3& nrm, PR d0) {
                        const double dx = q((int)i,0) - (double)xi.x;
                        const double dy = q((int)i,1) - (double)xi.y;
                        const double dz = q((int)i,2) - (double)xi.z;
                        const double distance = (double)d0
                            + (double)nrm.x * dx + (double)nrm.y * dy
                            + (double)nrm.z * dz;
                        if (distance >= (double)thickness) return;
                        const double push = (double)thickness - distance;
                        q((int)i,0) += (double)nrm.x * push;
                        q((int)i,1) += (double)nrm.y * push;
                        q((int)i,2) += (double)nrm.z * push;
                    });
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
            //     exactly the approach along that contact's normal.
            const PR invDt = PR(1) / dt;
            const PR velScale = PR(1) - damping;
            for (Index i = 0; i < n; ++i) {
                if (mask[i] == PR(0)) { vertexRef(v, i) = Vec3(); continue; }
                const Vec3 pi((PR)q((int)i,0), (PR)q((int)i,1), (PR)q((int)i,2));
                if (!std::isfinite(pi.x) || !std::isfinite(pi.y)
                    || !std::isfinite(pi.z)) {
                    vertexRef(v, i) = Vec3();
                    ++sanitizeCount;
                    continue;
                }
                const PR wi = invMass(i);
                const Vec3 accel = (wi > PR(0) && ext) ? vertexAt(ext, i) * wi : Vec3();
                const Vec3 freeDelta = (vertexAt(v, i) + accel * dt) * dt;
                Vec3 delta = pi - vertexAt(x, i);
                forEachContact(i, [&](const Vec3& nrm, PR) {
                    const PR dn = delta.dot(nrm);
                    if (dn <= PR(0)) return;
                    const PR approach = std::max(PR(0), -freeDelta.dot(nrm));
                    if (dn > approach) delta -= nrm * (dn - approach);
                });
                vertexRef(v, i) = delta * (invDt * velScale);
                vertexRef(x, i) = pi;
            }
        }
    }
};
