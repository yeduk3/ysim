#pragma once
// Fragment header: included in order by main.cpp immediately AFTER
// sim/pd_system.hpp and before sim/material_quat.hpp — the same slot PdSystem
// occupies, and for the same reason (Scene/GeneralMesh are only needed as
// dependent template names; the fragments after this one end mid-template).
// Not independently compilable by design.
//
// CPU implicit Euler cloth solver — Baraff & Witkin, "Large Steps in Cloth
// Simulation", SIGGRAPH 1998. FOURTH sibling next to SymplecticSystem (GPU,
// explicit forces), PbdSystem (CPU, position projection) and PdSystem (CPU,
// projective dynamics), selected at runtime by `Simulator::useLargeSteps` —
// the same A-option wiring the other two CPU solvers use (see
// docs/design/pbd-system-handoff.md §4 and docs/design/implicit-euler-system.md):
// NOT a System-type swap. Timing (h / subSteps / acctime) stays owned by the
// symplectic System; this solver borrows `subh` per substep.
//
// One linear system PER MESH (no cross-mesh coupling): BW98's constraint
// filter is a per-vertex operator, so a two-way cloth-cloth contact cannot be
// expressed by it at all. Two-way / self contacts are PdSystem's job — see
// §"Contacts" below.
//
//   (M − h·∂f/∂v − h²·∂f/∂x)·Δv = h·( f0 + h·(∂f/∂x)·v )        (BW98 eq. 5)
//   v += Δv;   x += h·v
//
// solved by BW98 §5's MODIFIED PCG: every CG operand passes through a
// per-vertex filter S_i that deletes the constrained direction(s), so the
// constrained components of Δv keep the value seeded in z and the solve
// happens in the remaining subspace. S_i = 0 pins a vertex outright.
//
// MATERIAL MODEL (BW98 §4), condition-function form. For a condition C(x)
// with per-vertex gradients C_m = ∂C/∂x_m and second derivatives C_mn:
//
//   f_m   = −k·C_m·C − kd·C_m·Ċ,            Ċ = Σ_n C_n·v_n
//   K_mn  = −k·(C_m C_nᵀ + C_mn·C) − kd·C_mn·Ċ
//   V_mn  = −kd·C_m C_nᵀ
//
// applied to stretch (C_u, C_v per triangle), shear (C per triangle) and
// bend (C = θ − θ0 per interior edge, Gauss-Newton: C_mn dropped). That is
// one accumulator, `accumCondition`, called four times.
//
// DELIBERATE DEVIATIONS from the reference implementation
// (…/LargeStepInClothSimulationXCode/main.cpp), each of which is a bug there
// that would show up as "the port explodes":
//
//  1. Damping rate. The reference computes Ċ per vertex (`Cui.dot(v_i)`);
//     BW98 §4.5 defines ONE scalar Ċ = Σ_n C_n·v_n per condition. Per-vertex
//     damping is not the derivative of anything and does not vanish on a
//     rigid motion.
//  2. Bend gradient. THE one that decides whether this solver runs at all:
//     ∂n/∂x = (I − nnᵀ)·(∂N/∂x)/|N|, and dropping the projection (as the
//     reference does) leaves an error ∝ cosθ — O(1) on any shallow crease,
//     enough to flip ∂θ/∂x's sign and make bending pump energy. Ported with
//     the projection kept; LS-3 is the numeric gradient check, and with the
//     projection dropped it fails outright (and LS-1/2/4 diverge to ~1e26).
//     Two smaller ones in the same function: the reference's `sinTheta` is
//     sqrt(1/(1−cos²θ)) clamped to 1, i.e. 1/sinθ clamped, so its
//     invSinTheta is identically 1 and the 1/sinθ factor is silently
//     dropped; and θ = acos(cosθ) is UNSIGNED, so a fold either way reads
//     the same and the rest angle can only be 0. Here sinθ = (n1×n2)·ê is
//     signed, θ = atan2(sinθ, cosθ), and θ0 is measured from the rest pose,
//     so a curved rest shape is representable. (The reference's ∂N/∂x blocks
//     themselves are right — its sign convention absorbs the transpose this
//     port writes explicitly.)
//  3. Preconditioner. The reference sets `invP = diag(A)` and
//     `P = 1/diag(A)`, i.e. it preconditions with diag(A) instead of its
//     inverse (still SPD, so it converges — just slowly, and the δ0 tolerance
//     is then measured in a different norm than the residual). Here
//     P⁻¹ = 1/diag(A) as in BW98, and δ0 uses the same operator.
//  4. Stiffness weighting. BW98's C carries the triangle's material AREA, so
//     the energy scales like k·a²·strain² — i.e. the same cloth gets softer
//     as the mesh is refined, and the mesh inspector's k (a spring constant
//     for every other solver here) would mean something else again. This port
//     divides the per-triangle stiffness by the rest area, making the energy
//     k·a·strain² — a per-AREA energy, resolution independent, and k keeps
//     its ~N/m reading. `stretchScale` / `shearScale` / `bendScale` are the
//     calibration knobs on top (bending especially: the inspector's kbend is
//     a spring constant, NOT a bending modulus [N·m], so its default scale is
//     small and expected to be tuned per fabric).
//  5. Collisions. The reference's `resolveCollision()` is dead code (its call
//     site is commented out) and rebuilds S from a hard-coded plane. Here the
//     filter is built from the live narrow-phase rows, and the penetration
//     recovery is RATE LIMITED (`contactRecovery`, a fraction of the
//     violation per substep) rather than "undo it all in one h", which is the
//     first thing that blows a stiff implicit solve up.
//
// Material coordinates: ysim meshes carry no UV, so each triangle's material
// frame is its REST triangle flattened isometrically (ū_i at the origin, ū_j
// on +x, ū_k in the upper half plane) — the same construction
// PdSystem::buildStrainTriangles uses, exact for a flat rest pose and a
// per-triangle approximation otherwise.
//
// Runs on the CPU against the METAL shared-storage buffers directly
// (MeshState::x/v/m and Scene::packedCollisionData are StorageModeShared, so
// `.ptr` is host-addressable) — hence the METAL specialization with a CPU
// body, exactly like PdSystem.

template <typename BE, typename PR> struct LargeStepsSystem {};

template <typename PR>
struct LargeStepsSystem<METAL, PR> {
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Mat3d = Eigen::Matrix3d;
    using Trip  = Eigen::Triplet<double>;
    using Vec3  = tinym::vec3_base<PR>;

    // Per-section timing. Wired by main's applyProfilerLevel, so it is
    // non-null ONLY at the InFrame tier — every section below is gated on it
    // and costs nothing at the other tiers. Samples ACCUMULATE per frame, so
    // a section's frame value is the sum over substeps and meshes.
    profiler::FrameProfiler* profiler = nullptr;

    // ── Knobs (all reachable from the solver panel) ──────────────────────
    // CG budget. BW98 runs "a few" iterations per step; 100 is the reference
    // implementation's cap and effectively never binds on a 20x20 sheet.
    int    maxCGIter = 100;
    // ε² of BW98's ‖r‖²_{P⁻¹} < ε²·‖filter(b)‖²_{P⁻¹} test.
    double cgTol2    = 1e-4;
    // Per-family stiffness calibration (see deviation 4).
    double stretchScale = 1.0;
    double shearScale   = 1.0;
    double bendScale    = 1e-6;
    // Condition damping kd, as a FRACTION of the family's k (BW98 §4.5 keeps
    // them separate; tying them means one knob keeps the ratio sane).
    double dampRatio = 1e-3;
    // Isotropic air drag: f += −airDrag·v, so it lands on the dfdv diagonal.
    double airDrag = 0.01;
    bool   shearOn = true;
    bool   bendOn  = true;
    // Contact filter (BW98 §5.3) built from the narrow-phase rows.
    bool   contactsEnabled = true;
    // Fraction of the remaining penetration removed per substep. 1.0 = "undo
    // it in one h", which is exactly the impulse that blows up a stiff solve.
    double contactRecovery = 0.2;
    // Hard ceiling on the seeded separation speed (m/s), so one badly
    // measured deep contact cannot launch a vertex.
    double maxRecoverySpeed = 1.0;

    // ── Read-only stats (solver panel) ───────────────────────────────────
    int      lastIterations = 0;
    int      lastContacts   = 0;
    int      lastCoupledRows = 0;
    double   lastResidual   = 0.0;
    uint32_t anomalyCount   = 0;

    static bool clothLike(BehaviorType b) {
        return b == BehaviorType::TriangularCloth
            || b == BehaviorType::FastGridCloth;
    }
    static PR thicknessOf(const BehaviorParams<PR>& bp) {
        if (auto* c = std::get_if<ClothBehaviorParams<PR>>(&bp)) return c->thickness;
        if (auto* g = std::get_if<FastGridClothBehaviorParams<PR>>(&bp)) return g->thickness;
        return PR(0);
    }
    // Stretch / shear / bend constants as the mesh inspector states them.
    // FastGridCloth carries no separate shear number — its GPU twin folds
    // shear into the diagonal springs — so it reuses kstretch there.
    static void stiffnessOf(const GeneralMesh<METAL, PR>& mesh,
                            double& kStretch, double& kShear, double& kBend) {
        kStretch = kShear = kBend = 0.0;
        if (auto* c = std::get_if<ClothBehaviorParams<PR>>(&mesh.behaviorParams)) {
            kStretch = (double)c->stretch;
            kShear   = (double)c->shear;
            kBend    = (double)c->bend;
        } else if (auto* g = std::get_if<FastGridClothBehaviorParams<PR>>(&mesh.behaviorParams)) {
            kStretch = (double)g->kstretch;
            kShear   = (double)g->kstretch;
            kBend    = (double)g->kbend;
        }
        const double s = (double)mesh.clothStiffnessScale;
        kStretch *= s; kShear *= s; kBend *= s;
    }
    static Vec3d posAt(const PR* base, Index i) {
        return Vec3d((double)base[i*3], (double)base[i*3+1], (double)base[i*3+2]);
    }
    static Mat3d skew(const Vec3d& v) {
        Mat3d m;
        m <<     0, -v.z(),  v.y(),
              v.z(),      0, -v.x(),
             -v.y(),  v.x(),      0;
        return m;
    }

    // ── Per-mesh topology cache ──────────────────────────────────────────
    // A triangle's material frame, flattened once from the REST pose, stored
    // as the six ∂w/∂x coefficients BW98 needs:
    //   w_u = Σ_m dwu[m]·x_m,   w_v = Σ_m dwv[m]·x_m
    // (the u/v derivatives of the flattened frame are constants of the
    // element, so both conditions and all their derivatives fall out of them).
    struct Tri {
        Index v[3];
        double dwu[3], dwv[3];
        double area;        // material-space rest area a_t
    };
    // Interior edge (i,j) with the two opposite vertices k (facet 0) and
    // p (facet 1) — the reference's {k, j, i, p} quad, re-lettered so the
    // shared edge reads first. theta0 is the REST dihedral angle, measured
    // from the live pose at build time (same convention as PbdSystem's bend
    // quads and PdSystem's rest operators).
    struct Quad {
        Index i, j, k, p;
        double theta0;
        double weight;      // |e|² / (a_k + a_p), the shell bending weight
    };
    struct MeshCache {
        int   lifetimeId = -1;
        Index numPoints  = 0;
        Index numFacets  = 0;
        std::vector<Tri>  tris;
        std::vector<Quad> quads;
        uint32_t degenerate = 0;

        // ── Cached assembly target (buildPattern) ────────────────────────
        // A = M − h·dfdv − h²·dfdx is assembled IN PLACE: its sparsity is a
        // pure function of the topology (the element stencils plus the
        // diagonal), so the pattern is built once per mesh lifetime and only
        // the values are overwritten per substep. Measured motive: the old
        // per-substep triplet path (build ~1.6M triplets → setFromTriplets →
        // three sparse-sparse ops) was 318 ms of a 380 ms frame at 10k
        // vertices — 82% of the whole solver, against 60 ms for the PCG it
        // was feeding.
        Eigen::SparseMatrix<double> A;
        // (element, m, n, column) → index into A.valuePtr() of the block's
        // FIRST row. Rows 3r, 3r+1, 3r+2 are consecutive within a column
        // because the pattern always carries the full 3x3 block, so one index
        // per column addresses all three. 27 ints per triangle (3x3 pairs),
        // 48 per bend quad (4x4 pairs).
        std::vector<int> triSlot, quadSlot;
        std::vector<int> diagSlot;      // dof → value index of (i,i)
        bool patValid = false;
    };
    std::vector<MeshCache> cache;    // parallel to Scene::meshes

    // Scratch, reused across substeps so the assembly does not re-allocate
    // every 1/180 s.
    Eigen::VectorXd f0, vVec, bVec, zVec, dvVec, rVec, cVec, qVec, sVec, precon;
    // dfdx·v, accumulated 3x3 block by 3x3 block during assembly — this is
    // the ONLY thing the old code kept a separate K matrix for (b = h(f0 +
    // h·K·v)), so accumulating it inline retires K entirely.
    Eigen::VectorXd Kv;
    // Assembly cursor: the mesh's value array, the current element's slot
    // table, and the substep's h. Members rather than parameters so
    // accumCondition's signature stays as BW98 writes it.
    double*    curAv   = nullptr;
    const int* curSlot = nullptr;
    double     asmH = 0.0, asmH2 = 0.0;
    // The matrix the PCG runs against — the active mesh's cached A.
    const Eigen::SparseMatrix<double>* Aptr = nullptr;
    std::vector<Mat3d> S;
    std::vector<uint8_t> contactCount;

    // ── Cloth → rigid coupling ───────────────────────────────────────────
    // Positional correction owed to each RIGID mesh, by mesh ARRAY INDEX.
    // Identical contract to PbdSystem/PdSystem::rigidDelta — the Simulator
    // teleports the body by it at frame completion and zeroes the entry — so
    // the same applyClothRigidDeltas serves whichever solver ran.
    //
    // NOT reset per step(): the narrow phase measured its distance against
    // the FRAME-FROZEN body, so the correction accumulates over the frame's
    // substeps and is pushed to Bullet once. beginFrameRigid() owns the
    // window.
    //
    // Without this the filter is a ONE-WAY kinematic constraint on cloth
    // velocity and nothing else: a falling ball feels no cloth at all and
    // simply passes through a sheet that is dutifully being pushed aside.
    std::vector<Vec3> rigidDelta;

    void beginFrameRigid(Index numMeshes) {
        if ((Index)rigidDelta.size() != numMeshes)
            rigidDelta.assign((size_t)numMeshes, Vec3());
        else
            for (auto& d : rigidDelta) d = Vec3();
    }

    // One contact whose target is a DYNAMIC rigid body. Revisited after the
    // integrate below to hand the body its mass-weighted share of whatever
    // penetration is left — the reaction the filter itself cannot express.
    struct CoupledPlane {
        Index  vert;
        Index  target;
        Vec3d  n;
        double dist;        // signed distance at the PRE-integrate position
        double nDotXOld;    // n·x(before), so the post-integrate distance is
                            // dist + (n·x_new − nDotXOld)
        double wB;          // 1/body mass
        double wc;          // 1/vertex mass (0 when pinned)
    };
    std::vector<CoupledPlane> coupled;

    // ── Condition accumulator (BW98 §4.2 / §4.5) ─────────────────────────
    // `hess` is C_mn; pass nullptr for a Gauss-Newton family (bend).
    template <int NV>
    void accumCondition(const Index (&vid)[NV], double C,
                        const Vec3d (&grad)[NV],
                        Mat3d (*hess)[NV],            // hess[m][n], or nullptr
                        double k, double kd,
                        const Eigen::VectorXd& v) {
        // Ċ = Σ_n C_n·v_n — ONE scalar per condition (deviation 1).
        double Cdot = 0.0;
        if (kd != 0.0)
            for (int m = 0; m < NV; ++m)
                Cdot += grad[m].dot(v.segment<3>(vid[m] * 3));

        for (int m = 0; m < NV; ++m) {
            f0.segment<3>(vid[m] * 3) += -k * grad[m] * C - kd * grad[m] * Cdot;
            for (int n = 0; n < NV; ++n) {
                Mat3d K = -k * (grad[m] * grad[n].transpose());
                if (hess) {
                    K += -k * hess[m][n] * C;
                    if (kd != 0.0) K += -kd * hess[m][n] * Cdot;
                }
                // The dfdx block lands in A scaled by −h², and its product
                // with v is b's second term — both taken here, so neither K
                // nor a sparse matvec is ever materialized.
                const int* slot = curSlot + (m * NV + n) * 3;
                scatterBlock(slot, -asmH2 * K);
                Kv.segment<3>(vid[m] * 3) += K * v.segment<3>(vid[n] * 3);
                if (kd != 0.0)   // dfdv block, scaled by −h
                    scatterBlock(slot,
                                 (asmH * kd) * (grad[m] * grad[n].transpose()));
            }
        }
    }

    // Add a 3x3 block into A's value array. `slot[c]` is the value index of
    // the block's first row within column c, and the two rows below it are
    // contiguous — see MeshCache::triSlot.
    void scatterBlock(const int* slot, const Mat3d& blk) {
        for (int c = 0; c < 3; ++c) {
            double* p = curAv + slot[c];
            p[0] += blk(0, c);
            p[1] += blk(1, c);
            p[2] += blk(2, c);
        }
    }

    // ── Sparsity pattern (built once per mesh lifetime) ──────────────────
    // The pattern is the union of every element stencil plus the full
    // diagonal, taken UNCONDITIONALLY: a runtime skip (a degenerate triangle,
    // a fold-flat dihedral, shearOn/bendOn/airDrag toggled off) then only
    // decides whether a slot is written, never which slots exist. That is
    // what makes the slot tables valid for the whole mesh lifetime — keying
    // them on emission order instead would silently mis-assemble the first
    // time a bend quad went flat.
    void buildPattern(MeshCache& mc, Index n) {
        const Index dof = n * 3;
        // Vertex adjacency (including self) of the assembled stencils.
        std::vector<std::vector<Index>> adj((size_t)n);
        auto stencil = [&](const Index* vid, int NV) {
            for (int m = 0; m < NV; ++m)
                for (int q = 0; q < NV; ++q)
                    adj[(size_t)vid[m]].push_back(vid[q]);
        };
        for (const Tri& t : mc.tris) stencil(t.v, 3);
        for (const Quad& q : mc.quads) {
            const Index vid[4] = { q.i, q.j, q.k, q.p };
            stencil(vid, 4);
        }
        for (Index i = 0; i < n; ++i) adj[(size_t)i].push_back(i);
        Index nnzBlocks = 0;
        for (auto& a : adj) {
            std::sort(a.begin(), a.end());
            a.erase(std::unique(a.begin(), a.end()), a.end());
            nnzBlocks += (Index)a.size();
        }

        // Column-major build: column 3c+cc holds three rows per neighbour of
        // c, ascending, so inserting in this order never re-sorts. Reserving
        // the exact per-column count keeps insert() O(1).
        mc.A.resize((int)dof, (int)dof);
        mc.A.setZero();
        Eigen::VectorXi room((int)dof);
        for (Index c = 0; c < n; ++c)
            for (int cc = 0; cc < 3; ++cc)
                room[(int)(c*3+cc)] = 3 * (int)adj[(size_t)c].size();
        mc.A.reserve(room);
        for (Index c = 0; c < n; ++c)
            for (int cc = 0; cc < 3; ++cc)
                for (Index r : adj[(size_t)c])
                    for (int rr = 0; rr < 3; ++rr)
                        mc.A.insert((int)(r*3+rr), (int)(c*3+cc)) = 0.0;
        mc.A.makeCompressed();
        (void)nnzBlocks;

        // (row, col) → value index. Inner indices are sorted within a column.
        const int* outer = mc.A.outerIndexPtr();
        const int* inner = mc.A.innerIndexPtr();
        auto slotOf = [&](Index r, Index c) {
            const int* b = inner + outer[(int)c];
            const int* e = inner + outer[(int)c + 1];
            return (int)(std::lower_bound(b, e, (int)r) - inner);
        };
        auto fill = [&](std::vector<int>& out, const Index* vid, int NV) {
            for (int m = 0; m < NV; ++m)
                for (int q = 0; q < NV; ++q)
                    for (int cc = 0; cc < 3; ++cc)
                        out.push_back(slotOf(vid[m]*3, vid[q]*3 + cc));
        };
        mc.triSlot.clear();  mc.triSlot.reserve(mc.tris.size() * 27);
        for (const Tri& t : mc.tris) fill(mc.triSlot, t.v, 3);
        mc.quadSlot.clear(); mc.quadSlot.reserve(mc.quads.size() * 48);
        for (const Quad& q : mc.quads) {
            const Index vid[4] = { q.i, q.j, q.k, q.p };
            fill(mc.quadSlot, vid, 4);
        }
        mc.diagSlot.resize((size_t)dof);
        for (Index i = 0; i < dof; ++i) mc.diagSlot[(size_t)i] = slotOf(i, i);
        mc.patValid = true;
    }

    // ── Cache build ──────────────────────────────────────────────────────
    void rebuildCache(const GeneralMesh<METAL, PR>& mesh, MeshCache& mc,
                      Index n, Index numFacets) {
        mc.tris.clear(); mc.quads.clear(); mc.degenerate = 0;
        mc.patValid = false;   // topology moved → every cached slot is stale
        const PR* pos = mesh.state.x.ptr;
        const Index* F = mesh.adjacency.facets.ptr;
        if (!pos || !F) return;
        constexpr double kEps = 1e-12;

        std::vector<double> triArea((size_t)numFacets, 0.0);
        for (Index f = 0; f < numFacets; ++f) {
            const Index a = F[f*3+0], b = F[f*3+1], c = F[f*3+2];
            if (a >= n || b >= n || c >= n) continue;
            if (a == b || b == c || a == c) { ++mc.degenerate; continue; }
            const Vec3d xi = posAt(pos, a);
            const Vec3d e1 = posAt(pos, b) - xi;
            const Vec3d e2 = posAt(pos, c) - xi;
            const double l1 = e1.norm();
            if (!(l1 > kEps)) { ++mc.degenerate; continue; }
            // Isometric flattening: du/dv of the rest triangle.
            const double du1 = l1,             dv1 = 0.0;
            const double du2 = e1.dot(e2)/l1,  dv2 = e1.cross(e2).norm()/l1;
            const double det = du1*dv2 - du2*dv1;
            if (!(std::abs(det) > kEps)) { ++mc.degenerate; continue; }
            Tri t;
            t.v[0] = a; t.v[1] = b; t.v[2] = c;
            t.dwu[0] = (dv1 - dv2)/det; t.dwu[1] =  dv2/det; t.dwu[2] = -dv1/det;
            t.dwv[0] = (du2 - du1)/det; t.dwv[1] = -du2/det; t.dwv[2] =  du1/det;
            t.area   = 0.5 * std::abs(det);
            triArea[(size_t)f] = t.area;
            mc.tris.push_back(t);
        }

        // Interior edges → bend quads. Edge key map built from the facets,
        // the same source PbdSystem::buildBendQuads walks.
        std::unordered_map<uint64_t, std::pair<Index, Index>> ef;
        ef.reserve((size_t)numFacets * 3);
        auto key = [](Index a, Index b) -> uint64_t {
            const uint64_t lo = (uint64_t)std::min(a, b), hi = (uint64_t)std::max(a, b);
            return (lo << 32) | hi;
        };
        for (Index f = 0; f < numFacets; ++f) {
            const Index v[3] = { F[f*3+0], F[f*3+1], F[f*3+2] };
            if (v[0] >= n || v[1] >= n || v[2] >= n) continue;
            if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) continue;
            for (int e = 0; e < 3; ++e) {
                auto it = ef.find(key(v[e], v[(e+1)%3]));
                if (it == ef.end()) ef.emplace(key(v[e], v[(e+1)%3]),
                                               std::make_pair(f, (Index)-1));
                else if (it->second.second == (Index)-1) it->second.second = f;
            }
        }
        auto opposite = [&](Index f, Index a, Index b) -> Index {
            for (int s = 0; s < 3; ++s) {
                const Index vs = F[f*3+s];
                if (vs != a && vs != b) return vs;
            }
            return (Index)-1;
        };
        for (const auto& kv : ef) {
            const Index f0i = kv.second.first, f1i = kv.second.second;
            if (f1i == (Index)-1) continue;                  // boundary edge
            const Index i = (Index)(kv.first >> 32), j = (Index)(kv.first & 0xffffffffu);
            const Index k = opposite(f0i, i, j), p = opposite(f1i, i, j);
            if (k == (Index)-1 || p == (Index)-1) continue;
            Quad q; q.i = i; q.j = j; q.k = k; q.p = p;
            double theta;
            if (!dihedral(pos, q, theta, nullptr)) continue;
            q.theta0 = theta;
            const double elen = (posAt(pos, j) - posAt(pos, i)).norm();
            const double aSum = triArea[(size_t)f0i] + triArea[(size_t)f1i];
            q.weight = (aSum > kEps) ? (elen*elen / aSum) : 0.0;
            if (!(q.weight > 0.0)) continue;
            mc.quads.push_back(q);
        }

        mc.lifetimeId = mesh.lifetimeId;
        mc.numPoints  = n;
        mc.numFacets  = numFacets;
    }

    // Dihedral angle of a bend quad and, optionally, its four gradients.
    // θ is signed against the shared edge, so a fold either way is distinct
    // and θ0 can be non-zero for a mesh whose rest pose is already curved.
    //
    //   N1 = ik × ij,  N2 = ij × ip,  n1 = N1/|N1|, n2 = N2/|N2|
    //   cosθ = n1·n2,  sinθ = (n1 × n2)·ê,  θ = atan2(sinθ, cosθ)
    //   ∂θ/∂x_m = −(∂n1/∂x_m·n2 + ∂n2/∂x_m·n1) / sinθ
    //
    // ∂n/∂x is taken as ∂N/∂x / |N| — the normalization's own derivative is
    // dropped, as in the reference and in most BW98 implementations; the
    // dropped term is ∝ n1 n1ᵀ, which the subsequent dot with n2 damps.
    // Signs derived from N1, N2 directly (deviation 2).
    static bool dihedral(const PR* pos, const Quad& q, double& theta,
                         Vec3d (*grad)[4]) {
        constexpr double kEps = 1e-9;
        const Vec3d xi = posAt(pos, q.i), xj = posAt(pos, q.j);
        const Vec3d xk = posAt(pos, q.k), xp = posAt(pos, q.p);
        const Vec3d ik = xk - xi, ij = xj - xi, ip = xp - xi;
        const Vec3d N1 = ik.cross(ij), N2 = ij.cross(ip);
        const double lN1 = N1.norm(), lN2 = N2.norm(), lij = ij.norm();
        if (lN1 < kEps || lN2 < kEps || lij < kEps) return false;
        const Vec3d n1 = N1 / lN1, n2 = N2 / lN2, e = ij / lij;
        const double cosT = std::max(-1.0, std::min(1.0, n1.dot(n2)));
        const double sinT = n1.cross(n2).dot(e);
        theta = std::atan2(sinT, cosT);
        if (!grad) return true;
        if (std::abs(sinT) < 1e-7) return false;   // ∂θ/∂x is 0/0 at the fold-flat pose
        // ∂(cosθ)/∂x_m with the NORMALIZATION derivative kept:
        //   ∂n1/∂x = (I − n1n1ᵀ)·(∂N1/∂x)/|N1|
        // so n2ᵀ∂n1/∂x = ((n2 − cosθ·n1)/|N1|)ᵀ·∂N1/∂x. Dropping the
        // projection (as the reference does) leaves an error ∝ cosθ, which is
        // O(1) for anything but a right-angle fold — big enough to flip the
        // gradient's sign on a shallow crease, so it is kept. The projected
        // vectors are a1/a2 below; with ∂N/∂x = skew(w) the transposed
        // product is just a cross product: skew(w)ᵀa = a × w.
        const Vec3d a1 = (n2 - cosT * n1) / lN1;
        const Vec3d a2 = (n1 - cosT * n2) / lN2;
        // ∂N1/∂x_m = skew(w) with N1 = ik × ij;  ∂N2/∂x_m likewise, N2 = ij × ip.
        const Vec3d w1i = ij - ik, w1j = ik, w1k = -ij;
        const Vec3d w2i = ip - ij, w2j = -ip, w2p = ij;
        const double invSin = 1.0 / sinT;
        (*grad)[0] = -invSin * (a1.cross(w1i) + a2.cross(w2i));   // i
        (*grad)[1] = -invSin * (a1.cross(w1j) + a2.cross(w2j));   // j
        (*grad)[2] = -invSin * (a1.cross(w1k));                   // k
        (*grad)[3] = -invSin * (a2.cross(w2p));                   // p
        return true;
    }

    // ── BW98 §5 modified PCG ─────────────────────────────────────────────
    // filter(a)_i = S_i·a_i, applied to every operand so the solve stays in
    // the unconstrained subspace and Δv keeps z's value on the constrained
    // directions.
    void filterInPlace(Eigen::VectorXd& a) const {
        const Index n = (Index)S.size();
        for (Index i = 0; i < n; ++i)
            a.segment<3>(i*3) = S[(size_t)i] * a.segment<3>(i*3);
    }

    // Null-profiler-safe section helper: a moved-from/default ScopedTimer has
    // profiler == nullptr and finalizes to nothing.
    profiler::FrameProfiler::ScopedTimer sect(const char* name) {
        return profiler ? profiler->scoped(name)
                        : profiler::FrameProfiler::ScopedTimer{};
    }

    // Solves S·A·Δv = S·b with Δv seeded at z. precon holds P⁻¹ = 1/diag(A).
    void modifiedPCG(Index n) {
        dvVec = zVec;
        computeDelta0(n);
        const Eigen::SparseMatrix<double>& Amat = *Aptr;
        rVec = bVec - Amat * dvVec;
        filterInPlace(rVec);
        cVec = precon.cwiseProduct(rVec);
        filterInPlace(cVec);
        double dNew = rVec.dot(cVec);
        lastIterations = 0;
        int it = 0;
        while (dNew > cgTol2 * delta0_ && it < maxCGIter) {
            qVec = Amat * cVec;
            filterInPlace(qVec);
            const double cq = cVec.dot(qVec);
            // cq <= 0 means A is not positive definite along this direction —
            // BW98's K keeps the C_mn·C term, which IS indefinite in
            // compression. CG has no answer there, so stop with what the
            // iterate has instead of taking a negative step length.
            if (!(cq > 1e-30)) break;
            const double alpha = dNew / cq;
            dvVec += alpha * cVec;
            rVec  -= alpha * qVec;
            sVec = precon.cwiseProduct(rVec);
            const double dOld = dNew;
            dNew = rVec.dot(sVec);
            cVec = sVec + (dNew / dOld) * cVec;
            filterInPlace(cVec);
            ++it;
        }
        lastIterations = it;
        lastResidual   = dNew;
        if (!dvVec.allFinite()) {
            ++anomalyCount;
            dvVec.setZero();
        }
        (void)n;
    }
    double delta0_ = 0.0;
    void computeDelta0(Index n) {
        Eigen::VectorXd bf = bVec;
        filterInPlace(bf);
        delta0_ = bf.dot(precon.cwiseProduct(bf));
        if (!(delta0_ > 0.0)) delta0_ = 1.0;   // nothing to solve; tolerance is then absolute
        (void)n;
    }

    // ── One substep ──────────────────────────────────────────────────────
    void step(Scene<METAL, PR>& sceneObjects, PR dt) {
        if (dt <= PR(0)) return;
        // x, v and this substep's narrow-phase rows must be GPU-complete
        // before the CPU reads them — same opener as PbdSystem/PdSystem::step.
        MetalGlobalContext::commitAndWait();

        const double h = (double)dt;
        const Index numMeshes = (Index)sceneObjects.meshes.size();
        if (cache.size() != sceneObjects.meshes.size())
            cache.resize(sceneObjects.meshes.size());

        auto& off      = Scene<METAL, PR>::packedMeshData.statesOffsets;
        auto& colFacet = Scene<METAL, PR>::packedCollisionData.vertColFacets;
        auto& colOff   = Scene<METAL, PR>::packedCollisionData.vertColFacetsOffsets;
        const bool haveContacts = contactsEnabled && colFacet.ptr && colOff.ptr
                               && off.ptr;
        lastContacts = 0;
        lastCoupledRows = 0;

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
            const Index numFacets = (Index)mesh.adjacency.facets.size / 3;

            MeshCache& mc = cache[(size_t)mi];
            if (mc.lifetimeId != mesh.lifetimeId || mc.numPoints != n
                || mc.numFacets != numFacets)
                rebuildCache(mesh, mc, n, numFacets);
            if (mc.tris.empty()) continue;
            if (!mc.patValid) buildPattern(mc, n);

            double kStretch, kShear, kBend;
            stiffnessOf(mesh, kStretch, kShear, kBend);

            const Index dof = n * 3;
            // ls_assemble = force + Jacobian evaluation scattered straight
            // into A's cached value array. Finalized explicitly (no extra
            // brace level).
            auto scAsm = sect("ls_assemble");
            f0.setZero(dof);
            Kv.setZero(dof);
            vVec.resize(dof);
            for (Index i = 0; i < dof; ++i) vVec[i] = (double)v[i];
            // A := 0, then A += M − h·dfdv − h²·dfdx as the elements are
            // walked. The mass diagonal goes in first so a mesh with no
            // surviving element still leaves A non-singular.
            curAv  = mc.A.valuePtr();
            asmH   = h;
            asmH2  = h * h;
            Aptr   = &mc.A;
            std::fill(curAv, curAv + mc.A.nonZeros(), 0.0);
            for (Index i = 0; i < n; ++i) {
                const double mi3 = (double)m[i*3];
                const double mm = (mi3 > 0.0) ? mi3 : 1e-9;
                for (int a = 0; a < 3; ++a)
                    curAv[mc.diagSlot[(size_t)(i*3+a)]] += mm;
            }

            // --- external forces (gravity + wind, already force units) ---
            if (ext)
                for (Index i = 0; i < dof; ++i) f0[i] += (double)ext[i];

            // --- stretch + shear, per triangle ---
            for (size_t ti = 0; ti < mc.tris.size(); ++ti) {
                const Tri& t = mc.tris[ti];
                curSlot = mc.triSlot.data() + ti * 27;
                const Vec3d x0 = posAt(x, t.v[0]);
                const Vec3d x1 = posAt(x, t.v[1]);
                const Vec3d x2 = posAt(x, t.v[2]);
                const Vec3d wu = t.dwu[0]*x0 + t.dwu[1]*x1 + t.dwu[2]*x2;
                const Vec3d wv = t.dwv[0]*x0 + t.dwv[1]*x1 + t.dwv[2]*x2;
                const double lwu = wu.norm(), lwv = wv.norm();
                if (lwu < 1e-9 || lwv < 1e-9) continue;
                const Vec3d nu = wu / lwu, nv = wv / lwv;
                const double a = t.area;
                // Per-AREA stiffness (deviation 4): k_t = k·scale / a.
                const double kS  = kStretch * stretchScale / a;
                const double kSh = kShear   * shearScale   / a;
                const Index vid[3] = { t.v[0], t.v[1], t.v[2] };

                // C_u = a(|w_u| − 1), C_v = a(|w_v| − 1)  (rest |w| == 1 by
                // construction of the flattened frame).
                const Mat3d uTerm = (Mat3d::Identity() - nu*nu.transpose()) / lwu;
                const Mat3d vTerm = (Mat3d::Identity() - nv*nv.transpose()) / lwv;
                // DEFINITENESS GATE (BW98 §4.4's stability remark; the
                // reference implementation has none and blows up the moment a
                // contact compresses the sheet — measured: 24 contacts at
                // frame 20 of ls_analytic_sphere, |v| 46 at 22, 2.9e4 at 23).
                //
                // K_mn = −k(C_m C_nᵀ + C_mn·C) is negative semidefinite —
                // which is what makes A = M − h·dfdv − h²·dfdx SPD, hence
                // solvable by CG — only while C_mn·C is PSD. C_mn itself is
                // PSD for the stretch conditions (a rank-1 dwu⊗dwu coefficient
                // times the PSD (I − ŵŵᵀ)/|w|), so the product's sign is C's:
                // in TENSION the term is kept, in COMPRESSION it is dropped
                // (Gauss-Newton). Without the gate diag(A) goes negative, the
                // preconditioner 1/diag(A) flips sign, and CG walks a negative
                // curvature direction with a huge step length.
                {
                    Vec3d g[3]; Mat3d H[3][3];
                    const double Cu = a * (lwu - 1.0);
                    for (int mm = 0; mm < 3; ++mm) {
                        g[mm] = a * t.dwu[mm] * nu;
                        for (int nn = 0; nn < 3; ++nn)
                            H[mm][nn] = a * t.dwu[mm] * t.dwu[nn] * uTerm;
                    }
                    accumCondition<3>(vid, Cu, g, Cu > 0.0 ? &H[0] : nullptr,
                                      kS, kS * dampRatio, vVec);
                    const double Cv = a * (lwv - 1.0);
                    for (int mm = 0; mm < 3; ++mm) {
                        g[mm] = a * t.dwv[mm] * nv;
                        for (int nn = 0; nn < 3; ++nn)
                            H[mm][nn] = a * t.dwv[mm] * t.dwv[nn] * vTerm;
                    }
                    accumCondition<3>(vid, Cv, g, Cv > 0.0 ? &H[0] : nullptr,
                                      kS, kS * dampRatio, vVec);
                }
                // C_shear = a·(w_u·w_v). Gauss-Newton unconditionally: its
                // C_mn = a(dwu_m·dwv_n + dwv_m·dwu_n)·I has a zero diagonal
                // block structure and is indefinite for EITHER sign of C, so
                // there is no branch that could keep it safely.
                if (shearOn && kSh != 0.0) {
                    Vec3d g[3];
                    for (int mm = 0; mm < 3; ++mm)
                        g[mm] = a * (t.dwu[mm]*wv + t.dwv[mm]*wu);
                    accumCondition<3>(vid, a * wu.dot(wv), g,
                                      (Mat3d (*)[3])nullptr,
                                      kSh, kSh * dampRatio, vVec);
                }
            }

            // --- bend, per interior edge (Gauss-Newton: C_mn dropped) ---
            if (bendOn && kBend != 0.0) {
                for (size_t qi = 0; qi < mc.quads.size(); ++qi) {
                    const Quad& q = mc.quads[qi];
                    curSlot = mc.quadSlot.data() + qi * 48;
                    double theta; Vec3d g[4];
                    if (!dihedral(x, q, theta, &g)) continue;
                    const Index vid[4] = { q.i, q.j, q.k, q.p };
                    // k_θ = kbend·bendScale·|e|²/(a_k+a_p): the inspector's
                    // kbend is a spring constant, so the shell weight and the
                    // scale together are what turn it into a bending modulus.
                    const double kT = kBend * bendScale * q.weight;
                    accumCondition<4>(vid, theta - q.theta0, g,
                                      (Mat3d (*)[4])nullptr,
                                      kT, kT * dampRatio, vVec);
                }
            }

            // --- air drag: f += −airDrag·v  (a dfdv diagonal, so A += h·drag) ---
            if (airDrag != 0.0) {
                for (Index i = 0; i < dof; ++i) f0[i] += -airDrag * vVec[i];
                for (Index i = 0; i < dof; ++i)
                    curAv[mc.diagSlot[(size_t)i]] += h * airDrag;
            }

            // b = h(f0 + h·dfdx·v). A is already complete: the mass diagonal
            // went in before the element walk, each element scattered its
            // −h²·dfdx and −h·dfdv, and Kv carries dfdx·v from the same walk.
            bVec = h * (f0 + h * Kv);
            scAsm.finalize();

            // --- constraint filter S and the seeded velocity z ---
            {
                auto scFil = sect("ls_filter");
                buildFilter(sceneObjects, mesh, mi, n, x, v, m, mask,
                            haveContacts, colFacet, colOff, off, h);

                // P⁻¹ = 1/diag(A) (deviation 3).
                precon.resize(dof);
                for (Index i = 0; i < dof; ++i) {
                    const double d = curAv[mc.diagSlot[(size_t)i]];
                    precon[i] = (std::abs(d) > 1e-30) ? 1.0 / d : 1.0;
                }
            }

            {
                auto scCg = sect("ls_pcg");
                modifiedPCG(n);
            }
            // Iteration count parked in a "timing" column so it lands in the
            // same CSV — the units are iterations, not ms. Needed to read
            // ls_pcg: cost/iteration is what a GPU port would actually move.
            if (profiler) profiler->addSample("ls_cg_iters",
                                              (double)lastIterations);

            // --- integrate; pinned vertices are held outright ---
            for (Index i = 0; i < n; ++i) {
                if (mask[i] == PR(0)) {
                    v[i*3] = v[i*3+1] = v[i*3+2] = PR(0);
                    continue;
                }
                for (int a = 0; a < 3; ++a) {
                    const double vn = (double)v[i*3+a] + dvVec[i*3+a];
                    if (!std::isfinite(vn)) { ++anomalyCount; continue; }
                    v[i*3+a] = (PR)vn;
                    x[i*3+a] = (PR)((double)x[i*3+a] + h * vn);
                }
            }

            // Cloth → rigid reaction, measured on the positions just written.
            if (!coupled.empty() && !rigidDelta.empty())
                settleCoupledBodies(x, (double)thicknessOf(mesh.behaviorParams));
        }
    }

    // S_i = 0 for a pinned vertex; I − n nᵀ for a contact (up to two
    // independent normals; a third makes it 0). z_i seeds the constrained
    // component of Δv with a RATE-LIMITED separation speed.
    template <typename ColFacetT, typename ColOffT, typename OffT>
    void buildFilter(Scene<METAL, PR>& sceneObjects,
                     const GeneralMesh<METAL, PR>& mesh, Index mi, Index n,
                     const PR* x, const PR* v, const PR* m3, const PR* mask,
                     bool haveContacts, ColFacetT& colFacet, ColOffT& colOff,
                     OffT& off, double h) {
        S.assign((size_t)n, Mat3d::Identity());
        contactCount.assign((size_t)n, 0);
        zVec.setZero(n * 3);
        coupled.clear();
        for (Index i = 0; i < n; ++i)
            if (mask[i] == PR(0)) S[(size_t)i].setZero();

        if (!haveContacts) return;
        if (mi + 1 >= (Index)off.size) return;
        const Index colBase = off.ptr[mi];
        const double thk = (double)thicknessOf(mesh.behaviorParams);
        const Index numMeshes = (Index)sceneObjects.meshes.size();

        for (Index i = 0; i < n; ++i) {
            if (mask[i] == PR(0)) continue;
            const Index begin = colOff.ptr[colBase + i];
            const Index end   = colOff.ptr[colBase + i + 1];
            for (Index r = begin; r < end; ++r) {
                // Dedup on (target mesh, target primitive) — the same key
                // PbdSystem/PdSystem use; one row per incident triangle of
                // the same target would over-count the constraint.
                bool dup = false;
                for (Index qi = begin; qi < r; ++qi)
                    if (colFacet.ptr[qi].objPair.target == colFacet.ptr[r].objPair.target &&
                        colFacet.ptr[qi].indexPair.target == colFacet.ptr[r].indexPair.target) {
                        dup = true; break;
                    }
                if (dup) continue;
                const auto& row = colFacet.ptr[r];
                const Index t = row.objPair.target;
                // TWO-WAY targets (another live cloth, or self) cannot be
                // expressed by a per-vertex filter — BW98 has no coupled
                // form. Those rows are skipped here BY DESIGN and are
                // PdSystem's job; see the header.
                if (t < numMeshes) {
                    const auto& partner = sceneObjects.meshes[t];
                    if (clothLike(partner.behaviorType) && !partner.isStatic)
                        continue;
                }
                const auto& nd = row.collisionNormalAndDistance;
                Vec3d nrm((double)nd.x, (double)nd.y, (double)nd.z);
                const double nlen = nrm.norm();
                if (nlen < 1e-6) continue;
                nrm /= nlen;
                const double dist = (double)nd.w;      // signed, measured at x

                // TARGET VELOCITY. The filter constrains the cloth vertex's
                // WORLD velocity, so a target that is itself moving has to
                // enter the row or the constraint says the wrong thing: with
                // v_target dropped, "do not move into the surface" lets a
                // surface descending at 3 m/s sweep straight through a
                // vertex sitting still — which is exactly what ls_cloth_ball
                // showed. Everything is therefore measured RELATIVE to the
                // target, and the seeded velocity puts the vertex back on the
                // target's own normal velocity plus the separation term.
                // Static / Float / analytic targets carry zero here, so this
                // reduces to the previous behaviour for them.
                Vec3d vTargetBody = Vec3d::Zero();
                double wB = 0.0;
                bool   coupledRow = false;
                if (t < numMeshes) {
                    const auto& tm = sceneObjects.meshes[t];
                    vTargetBody = Vec3d((double)tm.rigidLinearVel.x,
                                        (double)tm.rigidLinearVel.y,
                                        (double)tm.rigidLinearVel.z);
                    // Only a DYNAMIC body with a live handle can take a share
                    // of the push — same test PbdSystem/PdSystem apply.
                    if (tm.behaviorType == BehaviorType::Rigid
                        && tm.applyGravity
                        && tm.rigidBodyMass > PR(0)
                        && tm.rigidBodyHandle != ysim::physics::kInvalidBodyHandle
                        && (Index)rigidDelta.size() > t) {
                        coupledRow = true;
                        wB = 1.0 / (double)tm.rigidBodyMass;
                    }
                }
                const Vec3d vi((double)v[i*3], (double)v[i*3+1], (double)v[i*3+2]);
                const double vnBody = nrm.dot(vTargetBody);
                const double vn     = nrm.dot(vi) - vnBody;   // RELATIVE
                const bool violating = dist < thk;
                if (!violating && vn >= 0.0) continue;  // separating and clear

                // Filter: remove this normal direction from the solve.
                uint8_t& cnt = contactCount[(size_t)i];
                if (cnt == 0)      S[(size_t)i] -= nrm * nrm.transpose();
                else if (cnt == 1) {
                    // Orthogonalize against what is left, so two nearly
                    // parallel normals do not delete the same DOF twice.
                    Vec3d n2 = S[(size_t)i] * nrm;
                    const double l2 = n2.norm();
                    if (l2 > 1e-6) { n2 /= l2; S[(size_t)i] -= n2 * n2.transpose(); }
                } else {
                    S[(size_t)i].setZero();
                }
                if (cnt < 255) ++cnt;
                ++lastContacts;

                // z: seed the constrained component with a separation speed
                // that removes `contactRecovery` of the violation this
                // substep, capped (deviation 5). Δv is a velocity CHANGE, so
                // the current normal velocity is subtracted out.
                double vSep = 0.0;
                if (violating) {
                    vSep = contactRecovery * (thk - dist) / h;
                    if (vSep > maxRecoverySpeed) vSep = maxRecoverySpeed;
                }
                // Final normal velocity wanted: the TARGET's own (so the
                // vertex is carried along) plus the separation term. vn is
                // already relative, so the change asked for is (vSep − vn).
                const Vec3d dz = nrm * (vSep - vn);
                zVec.segment<3>(i*3) += dz;

                if (coupledRow) {
                    const double mi3 = (double)m3[i * 3];
                    CoupledPlane cp;
                    cp.vert     = i;
                    cp.target   = t;
                    cp.n        = nrm;
                    cp.dist     = dist;
                    cp.nDotXOld = nrm.dot(Vec3d((double)x[i*3], (double)x[i*3+1],
                                                (double)x[i*3+2]));
                    cp.wB       = wB;
                    cp.wc       = (mask[i] != PR(0) && mi3 > 0.0) ? 1.0 / mi3 : 0.0;
                    coupled.push_back(cp);
                }
            }
        }
    }

    // Hand every coupled body its share of the penetration that SURVIVED the
    // solve, mass-weighted — the Newton reaction the velocity filter has no
    // way to express (PbdSystem/PdSystem do the same thing at the same point
    // in their step, and the Simulator applies all three identically).
    //
    // Measured AFTER the integrate, against the new positions: what is left
    // then is exactly what the cloth could not absorb, so a light sheet under
    // a heavy ball keeps handing the ball almost nothing (it gets pushed out
    // of the way instead) while a pinned sheet — wc == 0, nobody else can
    // move — hands over the whole deficit and holds the ball up.
    void settleCoupledBodies(const PR* x, double thickness) {
        for (const CoupledPlane& cp : coupled) {
            const Vec3d xn((double)x[cp.vert*3], (double)x[cp.vert*3+1],
                           (double)x[cp.vert*3+2]);
            double distNew = cp.dist + (cp.n.dot(xn) - cp.nDotXOld);
            // rigidDelta is tinym (x/y/z members); the plane math is Eigen.
            const Vec3& acc = rigidDelta[(size_t)cp.target];
            distNew -= cp.n.dot(Vec3d((double)acc.x, (double)acc.y,
                                      (double)acc.z));
            const double deficit = thickness - distNew;
            if (!(deficit > 0.0)) continue;
            const double denom = cp.wc + cp.wB;
            if (!(denom > 0.0)) continue;
            const double share = deficit * (cp.wB / denom);
            if (!(share > 0.0)) continue;
            // The body moves AGAINST the normal the cloth entered along.
            rigidDelta[(size_t)cp.target].x -= (PR)(cp.n.x() * share);
            rigidDelta[(size_t)cp.target].y -= (PR)(cp.n.y() * share);
            rigidDelta[(size_t)cp.target].z -= (PR)(cp.n.z() * share);
            ++lastCoupledRows;
        }
    }
};
