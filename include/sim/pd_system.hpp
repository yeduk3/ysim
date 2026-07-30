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
// Contacts are handled by TWO different mechanisms, split on SIDEDNESS.
//
// ONE-SIDED rows — the target has no live iterate in this solve (Float
// floor, analytic collider, Rigid body, static cloth) — are the paper's
// implicit collision handling (Bouaziz 2014 §6): every detected contact
// becomes a unilateral half-space constraint C = { q : n·(q - b) >= 0 }
// with A_i = B_i = I, so its w_c·I lands on the SYSTEM DIAGONAL, its local
// step projects the vertex onto the plane when the plane is violated and is
// the IDENTITY when it is not (the unilateral definition — this is what
// keeps a separated vertex from being glued to the plane), and its w_c·p_i
// enters the RHS. The global solve then balances contact against momentum
// and elasticity in ONE system instead of alternating a solve that pulls a
// penetrating vertex back and a projection that re-pushes it.
//
// Keeping the factorization current is §8's problem, solved here without
// rank updates: the whole constraint set is summarized per substep as a
// per-vertex contact-WEIGHT vector (the epoch). When that vector differs
// from the one the current factor holds, factorize() re-runs on the cached
// base matrix, REUSING the symbolic analysis — a contact only ever adds to
// diagonal entries the mass term already created, so the sparsity pattern
// is invariant and analyzePattern() runs once per base rebuild. An empty
// contact set against an empty cached epoch costs nothing at all.
//
// TWO-WAY rows — the target is another live cloth of this solve, self rows
// included — are in the energy TOO, as the SAME per-vertex constraint kind:
// one vertex-triangle row becomes up to FOUR positional constraints
// (A_i = B_i = I), one per participating vertex, whose targets p are the
// mass-weighted split of the row's depenetration and are RE-TARGETED from
// the live iterates of both meshes at every local step. Identity (p = own
// iterate) whenever the row is separated, exactly like the one-sided planes,
// so a permanently-resident row glues nothing. There is NO post-solve
// projection pass any more: the global solve is the last operation of the
// iteration, and every contact of the substep — one-sided and two-way — is
// balanced against momentum and elasticity inside it.
//
// Be honest about what this is and is not. It is NOT the paper's §6, which
// only ever covers obstacle (one-sided) collisions. It is NOT a merged
// multi-mesh system, and it is NOT the exact 4-vertex constraint either:
// that would need a real off-diagonal A_i block (the barycentric combination
// of the four vertices) instead of four independent diagonal ones. It is the
// §6 mechanism GENERALIZED per-vertex, with the barycentric coupling — and,
// for a cross-cloth row, the coupling between the two separate systems —
// carried by RE-LINEARIZATION: each iteration recomputes the split from the
// current iterates and hands each side its own share in its own matrix.
// Across meshes that is a Jacobi iteration (each system sees the partner's
// PREVIOUS iterate), which converges more slowly than one merged solve would
// but never fights it the way the old projection pass did. A SELF row
// (target == query mesh) lands all four shares in ONE system, so there the
// split is resolved inside a single solve.
//
// Cloth→rigid coupling is in neither paper: it keeps the
// PbdSystem::rigidDelta contract and is applied ONCE per substep after the
// iteration loop, off the realized push — step (3b).
//
// Parallelism (Liu 2013 §3.3 / §8): the local step is embarrassingly
// parallel over springs, and the global step's three coordinate columns are
// independent back-substitutions against ONE factor. Both are dispatched
// over GCD here — Apple Clang has no OpenMP. See kParallelMinVerts for why
// small meshes still run the identical code serially.
//
// ponytail: one commitAndWait per substep, same cost the PBD path pays. The
// factorization is amortized and the sweeps are multi-core, but a single
// back-substitution is still inherently sequential; port the local step + a
// Jacobi/Chebyshev global step to Metal if PD earns its keep.

#include <Eigen/Sparse>
#include <dispatch/dispatch.h>
#include <memory>
#include <thread>
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
    // One-sided (half-space plane) rows resolved this step(), the other half
    // of the substep's contact set. Counted for the same reason the two-way
    // ones are: a test that wants to know the contact path RAN cannot read
    // that off the contact ENERGY, which is legitimately zero whenever every
    // plane is satisfied (the unilateral identity branch).
    uint32_t oneSidedContactCount = 0;
    // Number of one-sided contact rows that fed a coupled rigid body this
    // step(). Reset at the top of step() like the counters above.
    uint32_t rigidCoupleCount   = 0;

    // How many times a contact-epoch change forced factorize() since this
    // system was constructed. MONOTONIC — never reset, by design: the only
    // thing worth knowing about it is its rate, and a probe/self-test that
    // samples it twice gets that from the difference. Base refactors
    // (topology / h / stiffness / mass / pin changes) are NOT counted here;
    // they are counted by nothing, because they are already gated on scalars
    // the user changed deliberately.
    uint32_t contactRefactorCount = 0;

    // ── DEBUG: Eq.8 objective probe (off by default, near-zero cost) ──────
    // When true, step() records the value of the objective the local/global
    // descent minimizes, ONE SAMPLE PER ITERATION, for the substep it is
    // currently running — so after a step() the vectors describe the LAST
    // substep. Off, the whole mechanism is a handful of `if (false)`
    // branches per substep phase; nothing is allocated and nothing is
    // computed. Exists for the self-test (PD-8) and for a live GUI probe.
    //
    // The objective is exactly the quadratic the global solve minimizes for
    // a FIXED auxiliary set (Bouaziz 2014 Eq. 8, plus this implementation's
    // soft pin, which is a real energy term of the same system):
    //
    //   F(q; d, p) = Σ_i (m_i/2h²)|q_i - s_i|²          momentum
    //              + Σ_pinned (wPin/2)|q_i - x_i|²      soft pin
    //              + Σ_springs (k/2)|q_a - q_b - d|²    elasticity
    //              + Σ_contacts (w/2)|q_i - p_i|²       contacts
    //
    // and it is sampled at the point of the iteration where the LOCAL step
    // has just re-projected d and p FROM the current iterate — i.e. what is
    // recorded for iterate q^k is E(q^k) = F(q^k; d*(q^k), p*(q^k)), the
    // objective with the auxiliaries at their optimum. That is the sequence
    // the paper's no-line-search argument is about: local step minimizes F
    // over (d, p), global step minimizes F over q, so E(q^k) is weakly
    // decreasing with no safeguards. Sampling F with the PREVIOUS
    // iteration's auxiliaries instead would only measure the global solve's
    // half of the descent and would hide a diverging local step.
    //
    // `debugObjectiveSeq` is the full F. `debugObjectiveSmooth` is the
    // momentum + pin + spring part only — the sub-objective whose local step
    // IS an exact projection onto a convex set. The contact part is not:
    // a two-way row's target is a linearized mass-weighted split of a
    // vertex-triangle inequality (and, across meshes, a Jacobi read of the
    // partner's previous iterate), so contact re-targeting can in principle
    // raise F between iterations. Both are recorded so a test can gate the
    // part that is actually guaranteed and REPORT the rest.
    bool debugObjectiveProbe = false;
    std::vector<double> debugObjectiveSeq;      // momentum+pin+spring+contact
    std::vector<double> debugObjectiveSmooth;   // momentum+pin+spring

    // Weight of one unilateral contact constraint, as a multiple of the
    // vertex's own momentum weight m_i/h². 1.0 means a violated contact
    // pulls exactly as hard as the momentum term, which halves the residual
    // penetration per local/global iteration — with the projection target
    // re-linearized every iteration the effective behavior is stiff without
    // the matrix being stiff. A member, not a constexpr, so a GUI hook can
    // tune it live (it only changes the epoch, so the cost of a change is
    // one factorize). >> 1 makes the system progressively worse conditioned
    // and re-introduces the locking Bouaziz 2014 §9 warns about; << 1 lets
    // the cloth sink.
    double kContactWeightScale = 1.0;

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

    // One incident spring of a vertex, as stored in the CSR adjacency below.
    // `sign` is +1 where the vertex is that spring's endpoint a and -1 where
    // it is b — exactly the two signs the RHS assembly applies.
    struct Incident { Index spring; int32_t sign; };

    // One ONE-SIDED contact of `vert`, resolved once per substep from the
    // deduped narrow-phase rows and held for the whole iteration loop: a
    // unilateral half-space constraint on a single vertex (A = B = I).
    // A vertex may carry SEVERAL of these — the corner of a box gives it one
    // plane per face — and each is its own constraint, so each adds its own
    // w to the diagonal and its own w·p to the RHS.
    //
    // `d0` is the narrow phase's signed distance measured from x along n, so
    // the live distance of an iterate q is d0 + n·(q - x); `w` is that
    // constraint's weight (see kContactWeightScale). `coupled`/`wB`/`target`
    // carry the dynamic-rigid-body coupling, which is not part of the
    // constraint itself — it is settled once per substep in step (3b).
    struct ContactPlane {
        Index vert = 0;
        Vec3  n;
        PR    d0 = PR(0);
        double w = 0.0;
        Index target = 0;
        bool  coupled = false;
        PR    wB = PR(0);
    };

    // One TWO-WAY vertex-triangle row of this substep, resolved once (step
    // 1b) and re-evaluated from the live iterates at every local step. It
    // spans up to two meshes, which is why the list hangs off PdSystem and
    // not off MeshCache.
    //
    // Only the data the LIVE re-evaluation needs is stored: who the four
    // vertices are, which contexts own them, the narrow phase's row normal
    // (the frozen ENTRY SIDE — sigma is read off it, never off the possibly
    // already-corrected current geometry), and the two constant-per-substep
    // per-vertex quantities: inverse mass (the split's weights) and the
    // constraint weight w_two that vertex's diagonal carries. Geometry —
    // normal, barycentrics, constraint value — is NOT stored: it is exactly
    // what has to be recomputed each iteration.
    //
    // A zero `w` marks a vertex that carries NO constraint (zero inverse
    // mass: pinned or massless). Its share of the split would be zero
    // anyway, so it is left out of both the diagonal and the RHS instead of
    // being fed an identity constraint that only churns the epoch.
    struct TwoWayRow {
        int   qc = 0, pc = 0;      // ctxs indices of query / target mesh
        Index vert = 0;            // query vertex, in qc
        Index i1 = 0, i2 = 0, i3 = 0;  // target triangle, in pc
        Vec3  nrm;                 // narrow-phase row normal (sigma source)
        PR    iq = PR(0), i1w = PR(0), i2w = PR(0), i3w = PR(0);  // inv mass
        double wq = 0.0, w1 = 0.0, w2 = 0.0, w3 = 0.0;            // w_two
    };
    // Cleared and rebuilt every substep by step (1b).
    std::vector<TwoWayRow> twoWayRows;

    // Vertex count below which every per-vertex / per-spring sweep runs
    // serially. A GCD round trip costs tens of microseconds of queue hop and
    // worker wake-up; a few hundred spring projections finish in less than
    // that, so parallelising a small sheet is a pure loss. The self-test
    // meshes are all far under this — they exercise the SERIAL schedule,
    // which is the reason both schedules must run the SAME lambda rather than
    // two copies of the loop that can drift apart.
    static constexpr Index kParallelMinVerts = 1024;

    // Run `body(begin, end)` over the half-open range [0, count).
    //
    // dispatch_apply is SYNCHRONOUS: it returns only after every chunk has
    // completed, so a captured reference can never outlive the call and the
    // substep keeps its sequential ordering across phases.
    //
    // Chunked, NOT one dispatch per element: per-element bookkeeping costs
    // more than a spring projection costs to compute. `body` must touch only
    // state OWNED by the range it is handed — both call sites below are
    // written so that each index has exactly one writer.
    template <typename F>
    static void forRange(bool parallel, size_t count, F&& body) {
        if (count == 0) return;
        if (!parallel) { body(size_t(0), count); return; }
        // ~2 chunks per core so a straggler chunk can be absorbed, floored at
        // a chunk size that is worth a worker's wake-up.
        static const size_t kChunks =
            (size_t)std::max(1u, 2u * std::thread::hardware_concurrency());
        const size_t chunk = std::max<size_t>((count + kChunks - 1) / kChunks,
                                              256);
        const size_t blocks = (count + chunk - 1) / chunk;
        // Captured by POINTER: a block capturing a C++ lambda by value would
        // copy it, and the copy semantics of a capture-by-reference lambda are
        // a needless thing to reason about when the callee outlives the call.
        auto* fn = &body;
        dispatch_apply(blocks, DISPATCH_APPLY_AUTO, ^(size_t bi) {
            const size_t b = bi * chunk;
            (*fn)(b, std::min(count, b + chunk));
        });
    }

    // Per-mesh cached topology + prefactored global matrix, parallel to
    // Scene::meshes (same shape as PbdSystem::bendCache).
    struct MeshCache {
        int    lifetimeId = -2;   // -1 is a legal mesh value, so seed elsewhere
        Index  numPoints = 0, numFacets = 0, numEdges = 0;
        std::vector<Spring> springs;
        // Vertex -> incident springs, CSR: the entries of vertex i are
        // adjEntries[adjOffsets[i] .. adjOffsets[i+1]). Built once WITH
        // `springs` (same topology trigger) because it is a pure function of
        // them. Exists so the local step's RHS assembly can be a GATHER — one
        // writer per row — instead of the scatter a serial loop can afford;
        // see the two LOCAL passes in step (2).
        std::vector<Index>    adjOffsets;   // n + 1
        std::vector<Incident> adjEntries;   // 2 * springs.size()
        // Projected auxiliary variables d_i of the local step, flat 3 per
        // spring. Lives here so the allocation survives substeps; every entry
        // is rewritten by pass 1 of every iteration, so it carries no state.
        std::vector<double> springD;
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
        // The CONTACT-FREE system M/h² + L + wPin·P, kept so a contact epoch
        // can be applied by copying it and touching only diagonal values —
        // the triplet assembly (and the symbolic analysis that follows it)
        // then runs only when the base itself changes.
        Eigen::SparseMatrix<double> Abase;
        // One-sided contact planes of THIS substep, and the per-vertex sum of
        // the weights of EVERY constraint this mesh's vertices carry — the
        // planes above AND this mesh's share of the two-way rows, which may
        // have been created by a DIFFERENT mesh's query vertex. That sum is
        // the epoch key. `epochW` is the copy of that vector the current
        // factorization actually holds; `contactW` empty means "no contacts",
        // and empty == empty is the zero-cost path. A base rebuild clears
        // epochW, which is how a change of h / mass / k (all of which move
        // w_c) is composed into the contact epoch.
        std::vector<ContactPlane> planes;
        std::vector<double> contactW, epochW;
        // contactW carries at least one non-zero (it is sized-and-zeroed
        // before the sweep, and only collapsed to empty afterwards if this
        // stayed false).
        bool hasContactW = false;
        // False only after a contact refactor FAILED for this mesh: the
        // substep then runs it contact-free, which means two-way rows created
        // by another mesh must not deposit RHS terms here either — their
        // weight is not on this diagonal any more.
        bool contactsLive = true;
        // Per-substep solve state, kept here only so the allocations persist
        // across substeps (every value is rewritten each step). n x 3: the
        // system is scalar n x n and the three coordinates are INDEPENDENT
        // (a spring's energy is isotropic, so the same L couples x, y and
        // z), which is the whole reason one factorization serves all of
        // them — and the reason the three columns can be back-substituted
        // CONCURRENTLY, one per coordinate axis. `q` must be PER MESH
        // (not a shared scratch): a two-way
        // contact writes into the PARTNER mesh's iterate, so every active
        // cloth's block stays live for the whole iteration phase — the same
        // reason PbdSystem::predBlocks is per mesh.
        Eigen::MatrixXd rhsBase, rhs, q;
        // Momentum target s of THIS substep, kept only for the objective
        // probe (the solve itself needs it folded into rhsBase, not raw).
        // Written by a single gated assignment in step (1) and never resized
        // when the probe is off, so an unprobed run never allocates it.
        Eigen::MatrixXd sMom;
        // Contact-attributable displacement estimator for the TWO-WAY rows
        // (flat 3N), zeroed each substep. The velocity update needs to know
        // how much of a vertex's position delta came from contact so
        // depenetration does not become velocity — PbdSystem::cDispBlocks,
        // same idea. With the rows in the energy there is no direct push to
        // sum any more; what accumulates here is the per-iteration TARGET
        // OFFSET (p - q) each row asked this vertex for. See the estimator
        // note in the local step's contact block for why that is the right
        // side to err on.
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

    // Transpose the spring list into the per-vertex CSR adjacency the local
    // step gathers over. Counting sort, two passes, no map: buildSprings has
    // already guaranteed a < n, b < n and a != b for every spring, so every
    // entry lands in range and no vertex sees the same spring twice.
    //
    // Why this exists at all: the RHS contribution of a spring is +k·d on
    // endpoint a and -k·d on endpoint b. A parallel loop over SPRINGS would
    // have two threads writing the same vertex row (a race), and per-thread
    // RHS copies would cost an n x 3 reduction per iteration. Inverting the
    // relation makes the assembly a loop over VERTICES with a single writer
    // per row, which needs no atomics and no reduction.
    static void buildIncidence(const std::vector<Spring>& springs, Index n,
                               std::vector<Index>& offsets,
                               std::vector<Incident>& entries) {
        offsets.assign((size_t)n + 1, 0);
        for (const auto& s : springs) { ++offsets[s.a + 1]; ++offsets[s.b + 1]; }
        for (Index i = 0; i < n; ++i) offsets[i + 1] += offsets[i];
        entries.resize(springs.size() * 2);
        std::vector<Index> cursor(offsets.begin(), offsets.end() - 1);
        for (size_t si = 0; si < springs.size(); ++si) {
            const Spring& s = springs[si];
            entries[cursor[s.a]++] = Incident{ (Index)si, +1 };
            entries[cursor[s.b]++] = Incident{ (Index)si, -1 };
        }
    }

    void step(Scene<METAL, PR>& sceneObjects, PR dt) {
        if (dt <= PR(0)) return;
        // Flush + wait: x, and the narrow-phase contact arrays this substep's
        // broad/narrow dispatches just filled, must be GPU-complete before the
        // CPU reads them. Same reason PbdSystem::step opens with it.
        MetalGlobalContext::commitAndWait();

        twoWayContactCount   = 0;
        selfContactCount     = 0;
        oneSidedContactCount = 0;
        rigidCoupleCount     = 0;
        // Cleared per SUBSTEP (step() is one substep), which is what makes
        // the vectors describe the last substep of the last update().
        if (debugObjectiveProbe) {
            debugObjectiveSeq.clear();
            debugObjectiveSmooth.clear();
        }

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
                || mc.numFacets != numFacets || mc.numEdges != numEdges
                || mc.adjOffsets.size() != (size_t)n + 1) {
                buildSprings(mesh, n, mc.springs);
                // The gather adjacency is a function of the spring list only,
                // so it is rebuilt exactly when the springs are. Stiffness
                // edits refactor the MATRIX but never reach here — and must
                // not, for the same reason `rest` must not be re-measured.
                buildIncidence(mc.springs, n, mc.adjOffsets, mc.adjEntries);
                mc.springD.assign(mc.springs.size() * 3, 0.0);
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
                mc.Abase.resize((int)n, (int)n);
                // setFromTriplets SUMS duplicates, which is exactly the
                // accumulate-per-incident-spring assembly above.
                mc.Abase.setFromTriplets(trip.begin(), trip.end());
                mc.factor = std::make_unique<
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>();
                // Symbolic analysis ONCE per base rebuild. Every later
                // (contact-epoch) refactor is a bare factorize() against the
                // SAME pattern — see step (1b) — which is where the halved
                // refactor cost comes from.
                mc.factor->analyzePattern(mc.Abase);
                mc.factor->factorize(mc.Abase);
                mc.valid = (mc.factor->info() == Eigen::Success);
                // The contact weights are m_i/h²-proportional, so a base
                // rebuild (which is exactly what a change of h, mass, pin
                // mask or topology triggers) has invalidated them too.
                mc.epochW.clear();
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

            // q still holds the momentum target s verbatim here (the warm
            // start IS s), so the probe's copy is one gated assignment.
            if (debugObjectiveProbe) mc.sMom = mc.q;

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
        // Weight of ONE constraint carried by vertex i of context c: the
        // vertex's own momentum weight m_i/h², scaled. The mass is floored
        // exactly as the matrix diagonal floors it, so a massless vertex gets
        // a consistent (tiny) pair of weights rather than a contact that
        // outranks its own inertia. One-sided planes and two-way rows use the
        // SAME family — a vertex weighs every constraint it carries by its
        // own inertia, whichever mesh's query created it.
        auto momentumW = [&](const SolveCtx& c, Index i) -> double {
            const double mi3 = (double)c.m[i * 3];
            return kContactWeightScale * (mi3 > 0.0 ? mi3 : 1e-9) * invH2;
        };
        // Two-way rows use the identical weight: a vertex weighs every
        // constraint it carries by its own inertia. Measured alternatives
        // (a per-vertex CAP that splits one momentum weight across the rows
        // a vertex carries; global scales 0.1 / 2 / 4 / 8) all made the
        // folded-sheet self-collision case WORSE, so there is no second knob
        // here — kContactWeightScale is the only one, shared with the planes.
        auto twoWayW = [&](const SolveCtx& c, Index i) -> double {
            return momentumW(c, i);
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
        // cloth) is ONE-SIDED: those targets have no iterate to correct, and
        // their motion is not ours to change (except the rigid-coupled share
        // in (3b)). Both families are energy constraints now; sidedness only
        // selects the SHAPE — a one-sided row is a half-space plane on ONE
        // vertex, a two-way row is a mass-weighted split over FOUR — so this
        // predicate is still the single place the two paths part.
        auto twoWayTarget = [&](Index t) -> const SolveCtx* {
            if (t >= numMeshes) return nullptr;
            const int ci = ctxOfMesh[t];
            if (ci < 0) return nullptr;
            const auto& partner = sceneObjects.meshes[t];
            if (!clothLike(partner.behaviorType) || partner.isStatic) return nullptr;
            return &ctxs[(size_t)ci];
        };

        // --- (1b) resolve this substep's WHOLE contact set — one-sided planes
        // AND two-way rows — and bring every factorization in line with it
        // (Bouaziz 2014 §6 constraints, §8's "keep the factorization current"
        // done by re-factorizing rather than by rank updates).
        //
        // Runs AFTER the ctx loop because a row's sidedness is only decidable
        // once every live cloth of the substep is known (twoWayTarget reads
        // ctxOfMesh).
        //
        // The matrix only ever sees the per-vertex SUM of the weights of the
        // constraints that vertex carries, so that vector IS the epoch: two
        // substeps whose contact sets differ only in which triangle a row
        // named produce the same matrix and must not refactor. Everything
        // else about a constraint (normals, offsets, targets) is pure RHS
        // data and changes freely without a refactor.
        //
        // THREE passes, not one, because a two-way row deposits weight in the
        // TARGET mesh's vector as well as the query mesh's: the whole sweep
        // has to finish before any epoch can be compared.
        twoWayRows.clear();
        for (SolveCtx& c : ctxs) {
            MeshCache& mc = *c.mc;
            mc.planes.clear();
            mc.contactW.assign((size_t)c.n, 0.0);
            mc.hasContactW = false;
            mc.contactsLive = true;
        }
        for (size_t ci = 0; ci < ctxs.size(); ++ci) {
            SolveCtx& c = ctxs[ci];
            MeshCache& mc = *c.mc;
            if (!c.contactsForMesh) continue;
            for (Index i = 0; i < c.n; ++i) {
                forEachContact(c, i,
                    [&](const NarrowCollision& row, const Vec3& nrm, PR d0) {
                    const Index t = row.objPair.target;
                    if (const SolveCtx* pc = twoWayTarget(t)) {
                        // ---- TWO-WAY row: up to four per-vertex constraints.
                        // Only the STATIC guards live here (index validity,
                        // a self row naming its own vertex, everybody pinned);
                        // the geometric ones (degenerate triangle, degenerate
                        // barycentric denominator) depend on the live iterate
                        // and are re-tested every local step.
                        if (!pc->facets) return;
                        const Index tri = row.indexPair.target;
                        if (tri >= pc->numFacets) return;
                        TwoWayRow tw;
                        tw.i1 = pc->facets[tri*3+0];
                        tw.i2 = pc->facets[tri*3+1];
                        tw.i3 = pc->facets[tri*3+2];
                        if (tw.i1 >= pc->n || tw.i2 >= pc->n || tw.i3 >= pc->n)
                            return;
                        // Self rows never name an incident facet (the narrow
                        // phase excludes adjacency), but a degenerate row must
                        // not build a constraint of a vertex against itself.
                        if (t == c.mi
                            && (i == tw.i1 || i == tw.i2 || i == tw.i3)) return;
                        tw.qc   = (int)ci;
                        tw.pc   = ctxOfMesh[t];
                        tw.vert = i;
                        tw.nrm  = nrm;
                        tw.iq  = invMassOf(c.m,   c.mask,   i);
                        tw.i1w = invMassOf(pc->m, pc->mask, tw.i1);
                        tw.i2w = invMassOf(pc->m, pc->mask, tw.i2);
                        tw.i3w = invMassOf(pc->m, pc->mask, tw.i3);
                        // Every side infinitely heavy: nothing this row could
                        // ever ask for. (The live wden test still guards the
                        // barycentric-degenerate case.)
                        if (tw.iq <= PR(0) && tw.i1w <= PR(0)
                            && tw.i2w <= PR(0) && tw.i3w <= PR(0)) return;
                        // A zero-inverse-mass vertex takes no share, so it
                        // carries no constraint — see TwoWayRow.
                        MeshCache& pmc = *pc->mc;
                        if (tw.iq > PR(0)) {
                            tw.wq = twoWayW(c, i);
                            mc.contactW[i] += tw.wq;
                            mc.hasContactW = true;
                        }
                        const Index tids[3] = { tw.i1, tw.i2, tw.i3 };
                        const PR    tiw[3]  = { tw.i1w, tw.i2w, tw.i3w };
                        double*     tout[3] = { &tw.w1, &tw.w2, &tw.w3 };
                        for (int j = 0; j < 3; ++j) {
                            if (tiw[j] <= PR(0)) continue;
                            *tout[j] = twoWayW(*pc, tids[j]);
                            pmc.contactW[tids[j]] += *tout[j];
                            pmc.hasContactW = true;
                        }
                        twoWayRows.push_back(tw);
                        // Counted ONCE PER ROW PER SUBSTEP, here in the
                        // resolve — the local step visits the same row
                        // `iterations` times and counting there would report
                        // the iteration count, not the contact count.
                        ++twoWayContactCount;
                        if (t == c.mi) ++selfContactCount;
                        return;
                    }
                    // ---- ONE-SIDED row: unilateral half-space plane.
                    ContactPlane pl;
                    pl.vert   = i;
                    pl.n      = nrm;
                    pl.d0     = d0;
                    pl.target = t;
                    // Same coupling test PbdSystem::step (3) applies: only
                    // a DYNAMIC rigid body with a live Bullet handle can
                    // take a share of the push.
                    if (t < numMeshes) {
                        const auto& tm = sceneObjects.meshes[t];
                        if (tm.behaviorType == BehaviorType::Rigid
                            && tm.applyGravity
                            && tm.rigidBodyMass > PR(0)
                            && tm.rigidBodyHandle
                               != ysim::physics::kInvalidBodyHandle
                            && (Index)rigidDelta.size() > t) {
                            pl.coupled = true;
                            pl.wB = PR(1) / tm.rigidBodyMass;
                        }
                    }
                    pl.w = momentumW(c, i);
                    mc.contactW[i] += pl.w;
                    mc.hasContactW = true;
                    mc.planes.push_back(pl);
                    ++oneSidedContactCount;
                });
            }
        }
        for (SolveCtx& c : ctxs) {
            MeshCache& mc = *c.mc;
            // Collapse to empty so that "no contacts now, none in the factor"
            // stays the zero-cost comparison it was.
            if (!mc.hasContactW) mc.contactW.clear();
            // No contacts now and none in the factor => the base
            // factorization is already the right one, untouched.
            if (mc.contactW == mc.epochW) continue;

            // Values-only update: every diagonal entry exists in Abase (the
            // mass term emplaces all n of them), so coeffRef never inserts
            // and the pattern analyzePattern saw is preserved exactly.
            Eigen::SparseMatrix<double> A = mc.Abase;
            for (Index i = 0; i < c.n; ++i) {
                const double w = mc.contactW.empty() ? 0.0 : mc.contactW[i];
                if (w != 0.0) A.coeffRef((int)i, (int)i) += w;
            }
            mc.factor->factorize(A);
            ++contactRefactorCount;
            if (mc.factor->info() == Eigen::Success) {
                mc.epochW = mc.contactW;
            } else {
                // Should not happen — A is Abase plus a non-negative
                // diagonal, so it is at least as positive definite. Fall back
                // to the contact-free system rather than back-substituting
                // against a broken factor: the substep loses its contacts,
                // the mesh keeps its physics.
                std::cerr << "[PdSystem] contact refactor failed on mesh index "
                          << c.mi << " — running this substep contact-free\n";
                mc.factor->factorize(mc.Abase);
                ++contactRefactorCount;
                mc.planes.clear();
                mc.contactW.clear();
                mc.epochW.clear();
                // Two-way rows are shared with another mesh's system, so they
                // cannot be dropped from the list here — the partner's factor
                // still holds their weight. This flag is how the local step
                // knows to stop depositing THIS mesh's share.
                mc.contactsLive = false;
            }
        }

        // --- (2) local/global block coordinate descent (Liu 2013 §3),
        // ITERATION-OUTER like PbdSystem step (2): sweeping all meshes per
        // iteration is what lets a cross-cloth contact see, and be seen by,
        // the partner's own internal constraints within the substep.
        //
        // Each iteration is THREE phases, in this order, because a two-way row
        // writes RHS rows of TWO meshes and neither may be solved before both
        // have been assembled:
        //   LOCAL springs   — per mesh, parallel, independent;
        //   LOCAL contacts  — ONE serial block over every mesh's planes and
        //                     every two-way row of the substep;
        //   GLOBAL          — per mesh back-substitution.
        // The global solve is therefore the LAST operation of the iteration,
        // and there is nothing after it: every contact of the substep is an
        // energy term the solve balances, not a push applied on top of it.
        for (int it = 0; it < iterations; ++it) {
            // Objective accumulators for THIS iteration; every term is added
            // where the local step produces the auxiliary it belongs to, so
            // the probe never re-derives a projection and can never disagree
            // with the one the solve actually used. See debugObjectiveProbe.
            double probeSmooth = 0.0, probeContact = 0.0;

            // ---- LOCAL, springs.
            for (SolveCtx& c : ctxs) {
                MeshCache& mc = *c.mc;
                mc.rhs = mc.rhsBase;
                // Everything below runs on ONE schedule decision: a mesh
                // small enough that dispatch would dominate keeps the serial
                // walk, through the same lambdas.
                const bool par = (c.n >= kParallelMinVerts);
                const size_t numSprings = mc.springs.size();

                // LOCAL pass 1, parallel over SPRINGS: each spring's
                // auxiliary variable d is the closest point of its constraint
                // manifold (the sphere of radius `rest`) to the current edge
                // vector — i.e. the edge direction scaled to rest length.
                // Closed form and independent per spring; spring i reads only
                // the (frozen) iterate and owns springD[3i .. 3i+2], so the
                // pass is write-disjoint with no synchronisation at all.
                //
                // d is computed for EVERY spring, including the k == 0 ones
                // the assembly skips: a branch here would only trade an
                // arithmetic op for a mispredict, and the stale slot is never
                // read.
                {
                    const Spring* sp = mc.springs.data();
                    const Eigen::MatrixXd& q = mc.q;
                    double* dOut = mc.springD.data();
                    forRange(par, numSprings, [&](size_t begin, size_t end) {
                        for (size_t si = begin; si < end; ++si) {
                            const Spring& s = sp[si];
                            const int a = (int)s.a, b = (int)s.b;
                            double ex = q(a,0) - q(b,0);
                            double ey = q(a,1) - q(b,1);
                            double ez = q(a,2) - q(b,2);
                            const double len = std::sqrt(ex*ex + ey*ey + ez*ez);
                            // Coincident endpoints: the direction is
                            // undefined. Leave d equal to the (zero) edge
                            // vector so the constraint contributes NOTHING
                            // this iteration. Projecting onto an arbitrary
                            // direction would inject a full rest-length
                            // impulse along a random axis; pulling to coincide
                            // would be worse still.
                            if (len > 1e-9) {
                                const double sc = s.rest / len;
                                ex *= sc; ey *= sc; ez *= sc;
                            }
                            double* d = dOut + si * 3;
                            d[0] = ex; d[1] = ey; d[2] = ez;
                        }
                    });
                }

                // LOCAL pass 2, parallel over VERTICES: assemble the spring
                // part of the RHS by GATHER. Vertex i sums sign * k * d over
                // its incident springs — the same +k·d on endpoint a / -k·d
                // on endpoint b the scatter applied, re-associated so that
                // row i has exactly one writer.
                //
                // The summation ORDER differs from the scatter's (a row now
                // accumulates in incidence order rather than spring order), so
                // results differ in the last bits. That is the accepted price
                // of dropping the race; nothing here depends on bit equality.
                {
                    const Index*    offs = mc.adjOffsets.data();
                    const Incident* ent  = mc.adjEntries.data();
                    const Spring*   sp   = mc.springs.data();
                    const double*   dIn  = mc.springD.data();
                    Eigen::MatrixXd& rhs = mc.rhs;
                    const double kS = c.kS, kB = c.kB;
                    forRange(par, (size_t)c.n, [&](size_t begin, size_t end) {
                        for (size_t i = begin; i < end; ++i) {
                            double gx = 0.0, gy = 0.0, gz = 0.0;
                            for (Index e = offs[i]; e < offs[i+1]; ++e) {
                                const Incident in = ent[e];
                                const double k = sp[in.spring].bend ? kB : kS;
                                if (!(k > 0.0)) continue;
                                const double w = k * (double)in.sign;
                                const double* d = dIn + (size_t)in.spring * 3;
                                gx += w * d[0]; gy += w * d[1]; gz += w * d[2];
                            }
                            rhs((int)i,0) += gx;
                            rhs((int)i,1) += gy;
                            rhs((int)i,2) += gz;
                        }
                    });
                }

                // PROBE (off => one predictable branch per mesh per
                // iteration): momentum + soft pin + spring energy of this
                // mesh, at the CURRENT iterate and with the d pass 1 just
                // projected from it. Serial and double-precision on purpose —
                // it is a diagnostic, and a parallel reduction would make the
                // reported value schedule-dependent.
                if (debugObjectiveProbe) {
                    const Eigen::MatrixXd& q = mc.q;
                    for (Index i = 0; i < c.n; ++i) {
                        const double mi3 = (double)c.m[i * 3];
                        const double wm = (mi3 > 0.0 ? mi3 : 1e-9) * invH2;
                        const Vec3 xi = vertexAt(c.x, i);
                        const double xv[3] = { (double)xi.x, (double)xi.y,
                                               (double)xi.z };
                        for (int a = 0; a < 3; ++a) {
                            const double dm = q((int)i, a) - mc.sMom((int)i, a);
                            probeSmooth += 0.5 * wm * dm * dm;
                            if (c.mask[i] == PR(0)) {
                                const double dp = q((int)i, a) - xv[a];
                                probeSmooth += 0.5 * kPinWeight * dp * dp;
                            }
                        }
                    }
                    const double* dIn = mc.springD.data();
                    for (size_t si = 0; si < numSprings; ++si) {
                        const Spring& s = mc.springs[si];
                        const double k = s.bend ? c.kB : c.kS;
                        if (!(k > 0.0)) continue;
                        const double* d = dIn + si * 3;
                        double acc = 0.0;
                        for (int a = 0; a < 3; ++a) {
                            const double t = q((int)s.a, a) - q((int)s.b, a)
                                           - d[a];
                            acc += t * t;
                        }
                        probeSmooth += 0.5 * k * acc;
                    }
                }
            }

            // ---- LOCAL, contacts. ONE serial block for BOTH families, and
            // serial on purpose: several constraints of one vertex write the
            // same RHS row (a box corner's planes, a taco fold's self rows),
            // and the contact set is orders of magnitude smaller than the
            // spring set. It spans meshes, so it cannot live inside the
            // per-mesh loop above.
            //
            // Every constraint here is the same shape: A = B = I, a target p,
            // and the weight w already sitting on that vertex's diagonal. A
            // constraint that is SATISFIED sets p to the vertex's own iterate
            // — the identity map, which contributes w·q_i to a row whose
            // diagonal carries w, i.e. exactly nothing. That branch is the
            // whole reason a contact may sit in the matrix for the entire
            // substep without gluing anything.
            {
                // Deposit w·p into vertex i of context `cc`, unless that
                // mesh's factor lost its contact weights (refactor failure).
                auto emit = [&](const SolveCtx& cc, Index i, double w,
                                const Vec3& p) {
                    if (!(w > 0.0) || !cc.mc->contactsLive) return;
                    cc.mc->rhs((int)i, 0) += w * (double)p.x;
                    cc.mc->rhs((int)i, 1) += w * (double)p.y;
                    cc.mc->rhs((int)i, 2) += w * (double)p.z;
                    // PROBE: (w/2)|q_i - p_i|², accumulated HERE and not in a
                    // second pass, so exactly the constraints that reached the
                    // RHS (weight > 0, contacts live) are the ones counted.
                    if (debugObjectiveProbe) {
                        const Eigen::MatrixXd& q = cc.mc->q;
                        const double ex = (double)p.x - q((int)i, 0);
                        const double ey = (double)p.y - q((int)i, 1);
                        const double ez = (double)p.z - q((int)i, 2);
                        probeContact += 0.5 * w * (ex*ex + ey*ey + ez*ez);
                    }
                };

                // ONE-SIDED planes: p is the closest point of
                // { q : n·(q - b) >= thickness } to the live iterate.
                for (SolveCtx& c : ctxs) {
                    MeshCache& mc = *c.mc;
                    for (const ContactPlane& pl : mc.planes) {
                        const Index i = pl.vert;
                        const Vec3 qi = qVec(c, i);
                        // The narrow phase measured d0 against the
                        // frame-frozen body, so a coupled body's already
                        // accumulated motion must not be re-paid here.
                        PR dist = pl.d0 + pl.n.dot(qi - vertexAt(c.x, i));
                        if (pl.coupled) dist -= pl.n.dot(rigidDelta[pl.target]);
                        Vec3 p = qi;
                        if (dist < c.thickness)
                            p += pl.n * (c.thickness - dist);
                        emit(c, i, pl.w, p);
                    }
                }

                // TWO-WAY rows: the vertex-triangle inequality (Müller 2007
                // eq 12/13) evaluated from the LIVE iterates of both meshes,
                // then handed to the four vertices as four positional targets
                // instead of being applied as four pushes. The four offsets
                // are exactly the corrections the projection pass used to
                // add — same mass-weighted split, same clamped barycentrics,
                // same frozen entry side — so the constraint the energy holds
                // is the one the narrow phase reported; only WHO applies it
                // changed (the global solve, balanced against momentum and
                // elasticity, instead of an unconditional post-solve add).
                for (const TwoWayRow& tw : twoWayRows) {
                    const SolveCtx& c  = ctxs[(size_t)tw.qc];
                    const SolveCtx& pc = ctxs[(size_t)tw.pc];
                    const Vec3 qv = qVec(c,  tw.vert);
                    const Vec3 P1 = qVec(pc, tw.i1);
                    const Vec3 P2 = qVec(pc, tw.i2);
                    const Vec3 P3 = qVec(pc, tw.i3);
                    // Offsets default to ZERO, i.e. every early-out below
                    // lands on the identity constraint rather than skipping
                    // the emit: the weight is on the diagonal for the whole
                    // substep, so an omitted RHS term would not be "no
                    // constraint", it would be a pull toward the origin.
                    Vec3 dq, d1, d2, d3;
                    const Vec3 e1 = P2 - P1, e2 = P3 - P1;
                    const Vec3 ngeom = e1.cross(e2);
                    const PR nlenG = ngeom.norm();
                    if (nlenG >= PR(1e-12)) {       // else: degenerate triangle
                        const Vec3 nh = ngeom / nlenG;
                        // eq (13) is eq (12) with the cross-product order
                        // flipped — i.e. a SIGN. Which one applies is the side
                        // the vertex entered from, and the narrow phase
                        // already oriented the row normal that way at substep
                        // start, so read sigma off it instead of guessing from
                        // the current (possibly already-corrected) geometry.
                        const PR sigma = (tw.nrm.dot(nh) >= PR(0))
                                       ? PR(1) : PR(-1);
                        const Vec3 sn = nh * sigma;
                        const PR hthk = std::max(c.thickness, pc.thickness);
                        const PR C = sn.dot(qv - P1) - hthk;
                        // Unilateral: already separated => IDENTITY, which is
                        // what makes a resident row safe to re-evaluate every
                        // iteration. Nothing accumulates.
                        if (C < PR(0)) {
                            // Barycentric coordinates of qv's projection onto
                            // the triangle plane, clamped into the triangle so
                            // a near-edge / near-vertex contact still
                            // distributes a convex combination (Σb = 1) rather
                            // than an extrapolating one.
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
                                if (bs < PR(1e-12)) b1 = b2 = b3 = PR(1)/PR(3);
                                else { b1 /= bs; b2 /= bs; b3 /= bs; }
                            }
                            const PR wden = tw.iq + b1*b1*tw.i1w
                                          + b2*b2*tw.i2w + b3*b3*tw.i3w;
                            if (wden > PR(1e-12)) {
                                // grad_q C = sigma*n, grad_pj C = -bj*sigma*n
                                // (the normal's OWN derivative is deliberately
                                // dropped — the standard PBD contact
                                // distribution; the constraint VALUE still
                                // follows the paper). Contact stiffness is 1:
                                // this is a hard contact, matching the
                                // one-sided target.
                                const PR s = C / wden;
                                dq = sn * (-s * tw.iq);
                                d1 = sn * ( s * tw.i1w * b1);
                                d2 = sn * ( s * tw.i2w * b2);
                                d3 = sn * ( s * tw.i3w * b3);
                            }
                        }
                    }
                    emit(c,  tw.vert, tw.wq, qv + dq);
                    emit(pc, tw.i1,   tw.w1, P1 + d1);
                    emit(pc, tw.i2,   tw.w2, P2 + d2);
                    emit(pc, tw.i3,   tw.w3, P3 + d3);
                    // cd ESTIMATOR — see step (4) for what it is used for.
                    // With the row in the energy there is no direct push to
                    // sum, and the displacement the solve actually attributes
                    // to this constraint is not separable from the rest of the
                    // RHS. What is accumulated instead is the TARGET OFFSET
                    // (p - q) each iteration asked for.
                    //
                    // cd is a SECOND-CHOICE estimator and only covers what the
                    // per-row clamp in (4) cannot see: a vertex's share of a
                    // row somebody ELSE queried. Where the row is the vertex's
                    // own, (4) clamps along that row's frozen normal instead,
                    // because summing offsets across roles cancels — a
                    // self-fold vertex is pushed +n as a query and -n as a
                    // triangle vertex of the neighbouring row, and the summed
                    // direction then clamps neither (measured: 0.16 m of extra
                    // peak height on the folded sheet).
                    //
                    // Why that one, of the estimators available: (4) uses cd
                    // as a DIRECTION plus an upper bound on how much velocity
                    // may be removed, and never removes below the vertex's own
                    // approach speed. Under-estimating cd therefore LEAKS
                    // depenetration into velocity — the invariant this whole
                    // mechanism exists to protect — while over-estimating it
                    // only degrades to the one-sided rows' behaviour (clamp
                    // exactly to the approach speed), which is the same
                    // treatment the plane constraints already get and is
                    // strictly safe. Summing the per-iteration offsets
                    // over-estimates (each iteration re-linearizes the same
                    // penetration and the solve realizes only a fraction of
                    // each ask), so it errs on the safe side by construction.
                    // The final-sweep "remaining deficit" alternative errs the
                    // wrong way: a converged row asks for ~0 and would clamp
                    // nothing at all.
                    // The QUERY share dq is deliberately NOT accumulated: the
                    // query vertex's clamp reads this row's own frozen normal
                    // in (4), and folding dq into cd would re-create the very
                    // cancellation cd exists to avoid (dq along +n against
                    // triangle shares along -n).
                    if (pc.mc->contactsLive) {
                        vertexRef(pc.cd, tw.i1) += d1;
                        vertexRef(pc.cd, tw.i2) += d2;
                        vertexRef(pc.cd, tw.i3) += d3;
                    }
                }
            }

            // PROBE: one sample per iteration, taken between the local and
            // global steps — i.e. E(q^it) with the auxiliaries at their
            // optimum for q^it. The global solve that follows can only
            // lower it.
            if (debugObjectiveProbe) {
                debugObjectiveSmooth.push_back(probeSmooth);
                debugObjectiveSeq.push_back(probeSmooth + probeContact);
            }

            // ---- GLOBAL, per mesh.
            for (SolveCtx& c : ctxs) {
                MeshCache& mc = *c.mc;
                const bool par = (c.n >= kParallelMinVerts);
                // Back-substitution against the prefactored A. This is
                // the step that makes PD implicit — every vertex sees every
                // other through A⁻¹, which is why stiff springs do not explode
                // the way the symplectic path does at the same substep count.
                //
                // The three coordinate columns share A but are otherwise
                // independent, so they are solved as three CONCURRENT
                // per-axis back-substitutions (Liu 2013 §8). SimplicialLDLT
                // is safe to share across them: solve() is const, its only
                // mutable member (m_info) is written by compute()/factorize()
                // and merely read here, and each call allocates its own
                // temporaries — there is no shared scratch to collide over.
                // q is column-major, so the three destination columns are
                // disjoint contiguous spans.
                //
                // Below the gate the three columns go in ONE call, which is
                // what this always did: for a small system the extra solve
                // set-up costs more than the axes save.
                if (par) {
                    MeshCache* mcp = &mc;
                    dispatch_apply(3, DISPATCH_APPLY_AUTO, ^(size_t ax) {
                        const Eigen::Index col = (Eigen::Index)ax;
                        mcp->q.col(col) = mcp->factor->solve(mcp->rhs.col(col));
                    });
                } else {
                    mc.q = mc.factor->solve(mc.rhs);
                }
            }
        }

        // --- (3b) cloth → rigid-body coupling, ONCE per substep. In neither
        // paper: the constraint above is unilateral against a plane the body
        // owns, and the body's own answer to being pushed is Bullet's, not
        // this solver's. The rigidDelta contract is PbdSystem's — the narrow
        // phase measured d0 against the FRAME-FROZEN body, so the share
        // accumulates across the frame's substeps and is written back once.
        //
        // Not per iteration: the iterate is a fixed point being converged, so
        // charging the body for every intermediate push would pay the same
        // depenetration `iterations` times. The realized push (how far the
        // converged q actually travelled along n, never negative) is the
        // honest quantity, split mass-weighted.
        //
        // A PINNED cloth vertex cannot travel at all, so its realized push is
        // zero — but it must still move the body, or a pinned sheet would be
        // penetrable. There the whole remaining deficit goes to the body,
        // which is the "nobody else can move" semantics of the old one-sided
        // push (wc == 0 => share_B == 1).
        for (SolveCtx& c : ctxs) {
            const MeshCache& mc = *c.mc;
            for (const ContactPlane& pl : mc.planes) {
                if (!pl.coupled) continue;
                const Index i = pl.vert;
                const PR wc = invMassOf(c.m, c.mask, i);
                const Vec3 qf = qVec(c, i);
                PR share;
                if (wc > PR(0)) {
                    const PR realized =
                        std::max(PR(0), (qf - vertexAt(c.x, i)).dot(pl.n));
                    share = realized * (pl.wB / (wc + pl.wB));
                } else {
                    PR dist = pl.d0 + pl.n.dot(qf - vertexAt(c.x, i))
                            - pl.n.dot(rigidDelta[pl.target]);
                    share = std::max(PR(0), c.thickness - dist);
                }
                if (share <= PR(0)) continue;
                // The body moves AGAINST the normal the cloth entered along.
                rigidDelta[pl.target] -= pl.n * share;
                ++rigidCoupleCount;
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
        //     exactly the approach along that contact's normal. EVERY row
        //     of a vertex clamps per row — one-sided planes and the two-way
        //     rows this vertex is the QUERY of, both along the frozen row
        //     normal. Only its TRIANGLE-vertex share of other vertices'
        //     rows has no normal of its own to read, and that is what cd
        //     is for. Both families are energy terms now, so neither delta
        //     is a literal push any more; the clamp does not need it to be,
        //     it needs a direction and a bound (see the cd estimator note).
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
                // EVERY row of this vertex, both families: the direction the
                // contact could have pushed this vertex along is EXACTLY the
                // row normal, whichever mechanism applied it, so a two-way row
                // is clamped here as well and not left to `cd` alone. (This
                // is not double-removal: the clamp drives delta·n down to
                // `approach` and is idempotent for a repeated normal; only
                // genuinely different normals compose.) Measured on the
                // folded-sheet case, restricting the two-way rows to the cd
                // clamp alone leaks ~0.16 m of extra peak height, because a
                // vertex of a self-fold is the QUERY of some rows (pushed
                // along +n) and a TRIANGLE vertex of others (pushed along -n)
                // and its cd sum cancels to a direction that clamps neither.
                // The sidedness predicate is deliberately NOT re-applied: a
                // row the resolve REJECTED (bad triangle index, everybody
                // pinned) then clamps a push nobody made, which can only
                // remove velocity down to the approach speed and never below
                // it — the same conservative direction the plane rows already
                // err in when a refactor failure drops them.
                forEachContact(c, i,
                    [&](const NarrowCollision& row, const Vec3& nrm, PR) {
                    const PR dn = delta.dot(nrm);
                    if (dn <= PR(0)) return;
                    const PR approach = std::max(PR(0), -freeDelta.dot(nrm));
                    if (dn > approach) delta -= nrm * (dn - approach);
                });
                // The rows above only cover the vertex in its QUERY role
                // (forEachContact walks ITS contact list). Its TRIANGLE-vertex
                // share of somebody else's row is covered by `cd`, whose
                // direction is the only normal that survives combining several
                // triangles. Remove at most what those rows asked for (cdLen)
                // beyond the approach speed — anything more would eat the
                // vertex's own motion.
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
