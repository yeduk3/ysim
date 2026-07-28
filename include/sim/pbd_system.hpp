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

    // Predicted positions, one flat 3N block per mesh. Grown, never shrunk.
    std::vector<PR> pred;

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
    };
    std::vector<BendCache> bendCache;   // parallel to Scene::meshes

    // Anomaly (NaN/Inf) counter for the frame; Simulator reads nothing from
    // this today — the guard's job is to keep a blown-up vertex from
    // poisoning the whole mesh.
    uint32_t sanitizeCount = 0;

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
            for (int e = 0; e < 3; ++e) {
                const Index a = v[e], b = v[(e+1)%3], o = v[(e+2)%3];
                const uint64_t key = a < b
                    ? ((uint64_t)a << 32) | (uint32_t)b
                    : ((uint64_t)b << 32) | (uint32_t)a;
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

    // Iteration-count-corrected stiffness: applying k' = 1-(1-k)^(1/n) once
    // per sweep converges to k after n sweeps, so doubling `iterations` does
    // not silently double the cloth's stiffness.
    static PR correctedK(PR k, int iters) {
        if (iters <= 1) return k;
        if (k >= PR(1)) return PR(1);
        if (k <= PR(0)) return PR(0);
        return PR(1) - std::pow(PR(1) - k, PR(1) / PR(iters));
    }

    void step(Scene<METAL, PR>& sceneObjects, PR dt) {
        if (dt <= PR(0)) return;
        // Flush + wait: x, and the narrow-phase contact arrays this substep's
        // broad/narrow dispatches just filled, must be GPU-complete before the
        // CPU reads them.
        MetalGlobalContext::commitAndWait();

        auto& off      = Scene<METAL, PR>::packedMeshData.statesOffsets;
        auto& colFacet = Scene<METAL, PR>::packedCollisionData.vertColFacets;
        auto& colOff   = Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets;
        const bool haveContacts = colFacet.ptr && colOff.ptr && off.ptr;

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
            const PR kS = correctedK(toPbdK(kSpringStretch, stretchRef), iterations);
            const PR kB = correctedK(toPbdK(kSpringBend,    bendRef),    iterations);

            pred.resize((size_t)n * 3);
            PR* p = pred.data();

            // Inverse mass, zero for pinned vertices — a pinned vertex is
            // infinitely heavy so a constraint moves only its partner.
            auto invMass = [&](Index i) -> PR {
                const PR mi3 = m[i * 3];
                return (mask[i] != PR(0) && mi3 > PR(0)) ? PR(1) / mi3 : PR(0);
            };

            // --- (1) predict: symplectic Euler on the external forces only.
            // Spring forces are NOT applied here — the distance constraints
            // below replace them. Fixed particles (w == 0) stay put.
            for (Index i = 0; i < n; ++i) {
                const PR w = invMass(i);
                const Vec3 accel = (w > PR(0) && ext) ? vertexAt(ext, i) * w : Vec3();
                const Vec3 vel = vertexAt(v, i) + accel * dt;
                vertexRef(p, i) = (w > PR(0)) ? vertexAt(x, i) + vel * dt
                                              : vertexAt(x, i);
            }

            // Standard PBD distance constraint C = |p_b - p_a| - rest.
            auto projectDistance = [&](Index a, Index b, PR rest, PR k) {
                if (a == b) return;
                const PR wa = invMass(a), wb = invMass(b);
                const PR wsum = wa + wb;
                if (wsum <= PR(0)) return;
                const Vec3 d = vertexAt(p, b) - vertexAt(p, a);
                const PR len = d.norm();
                if (len < PR(1e-9)) return;
                // Paper form: Δp = ±(w/Σw)(len-rest)·d/len. The extra /len
                // (not in the paper's s) folds d's normalization into the
                // scalar so `d` stays unnormalized below — one divide
                // instead of normalizing the vector.
                const PR s = k * (len - rest) / (len * wsum);
                vertexRef(p, a) += d * (wa * s);
                vertexRef(p, b) -= d * (wb * s);
            };

            const Index* edges   = mesh.adjacency.edges.ptr;
            const PR*    restLen = mesh.adjacency.restEdgeLengths.ptr;
            const Index  numEdges = edges ? (Index)mesh.adjacency.edges.size / 2 : 0;

            // Bending: dihedral-angle constraint (Müller 2007 eq 21-29), one
            // per interior edge. Quads are topology-derived, so rebuild only
            // when the mesh identity or counts change.
            if ((Index)bendCache.size() != (Index)sceneObjects.meshes.size())
                bendCache.resize(sceneObjects.meshes.size());
            BendCache& bc = bendCache[mi];
            const Index numFacets = (Index)mesh.adjacency.facets.size / 3;
            if (bc.lifetimeId != mesh.lifetimeId || bc.numPoints != n
                || bc.numFacets != numFacets) {
                buildBendQuads(mesh, bc.quads);
                bc.lifetimeId = mesh.lifetimeId;
                bc.numPoints = n;
                bc.numFacets = numFacets;
            }

            // eq 29: Δp_i = -(w_i sqrt(1-d^2)(acos(d) - phi0) / Σ_j w_j|q_j|^2) q_i
            auto projectBend = [&](const BendQuad& bq, PR k) {
                const Index idx[4] = { bq.p1, bq.p2, bq.p3, bq.p4 };
                PR w[4];
                PR wAny = PR(0);
                for (int j = 0; j < 4; ++j) { w[j] = invMass(idx[j]); wAny += w[j]; }
                if (wAny <= PR(0)) return;

                Vec3 n1, n2; PR l23, l24, d;
                if (!dihedral(p, bq.p1, bq.p2, bq.p3, bq.p4, n1, n2, l23, l24, d))
                    return;
                const PR s2 = PR(1) - d*d;
                // Flat / fully folded: the arccos gradient is singular and the
                // correction vanishes anyway. Skipping is the standard guard.
                if (s2 < PR(1e-12)) return;
                const PR sq = std::sqrt(s2);

                const Vec3 p1 = vertexAt(p, bq.p1);
                const Vec3 P2 = vertexAt(p, bq.p2) - p1;
                const Vec3 P3 = vertexAt(p, bq.p3) - p1;
                const Vec3 P4 = vertexAt(p, bq.p4) - p1;

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
                    vertexRef(p, idx[j]) += qs[j] * (w[j] * scale);
                }
            };

            // Visit every DEDUPED contact of vertex i as (normal, d(x)).
            // Shared by the projection sweeps and the velocity update below so
            // the two can never disagree about which contacts are active.
            const bool contactsForMesh = haveContacts && mi + 1 < (Index)off.size;
            const Index colBase = contactsForMesh ? off.ptr[mi] : Index(0);
            auto forEachContact = [&](Index i, auto&& fn) {
                if (!contactsForMesh) return;
                const Index begin = colOff.ptr[colBase + i];
                const Index end   = colOff.ptr[colBase + i + 1];
                for (Index c = begin; c < end; ++c) {
                    // Dedup coincident (targetObj, targetTri) contacts — the
                    // hash broad phases list one per incident query triangle,
                    // and summing them over-corrects. Analytic rows carry
                    // indexPair.target == 0 for every shape, so this key only
                    // separates them because objPair.target is the collider's
                    // mesh ARRAY INDEX (collider_pipeline_rework.md §4).
                    bool dup = false;
                    for (Index q = begin; q < c; ++q)
                        // .target == the kernel's .y lane (IndexPair is a union
                        // of {query,target} / {point,triangle}).
                        if (colFacet.ptr[q].objPair.target == colFacet.ptr[c].objPair.target &&
                            colFacet.ptr[q].indexPair.target == colFacet.ptr[c].indexPair.target) {
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

            // --- (2) Gauss-Seidel constraint projection.
            for (int it = 0; it < iterations; ++it) {
                if (restLen) {
                    for (Index e = 0; e < numEdges; ++e) {
                        const Index ea = edges[e*2], eb = edges[e*2+1];
                        // `numEdges` is an UPPER BOUND: MeshGridInitializerParams
                        // allocates 2(k-1)k + 2(k-1)^2 edge slots but
                        // MeshAdjacencyInitializer only writes the deduplicated
                        // subset (161 of 210 for an 8x8 grid). The tail is
                        // uninitialised pool memory — floats reinterpreted as
                        // indices — so it must be skipped exactly the way
                        // MeshAdjacency::recomputeRestLengths already does
                        // (mesh_state.hpp `inRange`). Without this the solver
                        // reads x/m far outside the mesh and segfaults.
                        if (ea >= n || eb >= n) continue;
                        projectDistance(ea, eb, restLen[e], kS);
                    }
                }
                if (kB > PR(0)) {
                    for (const auto& bq : bc.quads) projectBend(bq, kB);
                }

                // --- (3) contacts as inequality constraints. Same contact
                // set and same (distance < thickness) gate the integrate
                // kernel uses (D-016), applied to the predicted position.
                for (Index i = 0; contactsForMesh && i < n; ++i) {
                    if (mask[i] == PR(0)) continue;
                    forEachContact(i, [&](const Vec3& nrm, PR d0) {
                        // The kernel pushes by (thickness - distance)
                        // ONCE per substep, and that push never feeds back
                        // into velocity. PBD's v = (p-x)/dt turns every
                        // position delta into velocity, so re-applying a
                        // FIXED push each Gauss-Seidel sweep accumulates
                        // and launches the cloth. Make the contact a real
                        // inequality instead: the narrow phase measured
                        // `distance` along n from x, so the signed
                        // distance of the CURRENT predicted position is
                        // distance + n.(p - x). Once the push satisfies
                        // it, later sweeps are no-ops.
                        const PR distance = d0
                            + nrm.dot(vertexAt(p, i) - vertexAt(x, i));
                        if (distance >= thickness) return;
                        vertexRef(p, i) += nrm * (thickness - distance);
                    });
                }
            }

            // --- (4) velocity update from the position delta, then commit.
            const PR invDt = PR(1) / dt;
            const PR velScale = PR(1) - damping;
            for (Index i = 0; i < n; ++i) {
                if (mask[i] == PR(0)) { vertexRef(v, i) = Vec3(); continue; }
                const Vec3 pi = vertexAt(p, i);
                if (!std::isfinite(pi.x) || !std::isfinite(pi.y)
                    || !std::isfinite(pi.z)) {
                    // Blown-up vertex: hold the last good position, kill its
                    // velocity. Same intent as the kernel's sanitize guard.
                    vertexRef(v, i) = Vec3();
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
