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
// CPU Projective Dynamics (Bouaziz et al. 2014) — block coordinate descent
// alternating a LOCAL per-constraint projection with a GLOBAL linear solve
// whose matrix is CONSTANT for a fixed topology / timestep / stiffness, so it
// is factored once (sparse LDLT) and only back-substituted per iteration.
//
// SUPPORTED SOLVER x BEHAVIOR COMBINATIONS — stated explicitly rather than
// left to be inferred, because the two cloth behaviors now run DIFFERENT
// material models and silently reinterpreting one as the other is exactly
// what docs/design/pd-continuum-cloth-contact.md §3.5 forbids:
//
//   TriangularCloth  in-plane: CONTINUUM triangle strain potential
//                              (Bouaziz §5.3 / design doc §3) — per-triangle
//                              deformation gradient, thin SVD, singular-value
//                              limiting. NO edge stretch springs.
//                    bending:  COTANGENT mean-curvature potential (design doc
//                              §4) — a per-vertex Laplace–Beltrami row against
//                              the rest curvature sphere. NO opposite-vertex
//                              springs. This behavior therefore builds NO
//                              `Spring` at all: its whole material model is
//                              the two continuum elements.
//   FastGridCloth    in-plane + bending: the LEGACY mass-spring model (Liu,
//                              Bargteil, O'Brien, Kavan 2013 "Fast Simulation
//                              of Mass-Spring Systems"), unchanged. Its GPU
//                              twin is the source of truth for that mesh
//                              family and is deliberately out of scope here.
//   Rigid / Float    not simulated by this solver at all (clothLike gate).
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
// COUPLED rows — the target is another live cloth of this solve, self rows
// included — are the design doc's §5.3 vertex-triangle constraint, ONE row
// per contact over FOUR unknowns:
//
//   r_c(q) = q_v - Σ_a β_a q_a = A_c q,   A_c = [1, -β₁, -β₂, -β₃]
//   E_c(q) = (w_c/2)·min_{p ∈ H_c} ‖A_c q - p‖²,   H_c = { r : nᵀr >= d }
//
// Local step: p_c = Π_{H_c}(A_c q) — the half-space projection of the
// RELATIVE vector, which is the IDENTITY whenever the row is separated (the
// unilateral branch, so a resident row glues nothing). Global step:
// K += w_c·A_cᵀA_c, b += w_c·A_cᵀp_c. A_cᵀA_c is a DENSE 4x4 block, i.e.
// real off-diagonal coupling between the query vertex and the three triangle
// vertices — NOT the four independent diagonal identity constraints the
// previous implementation decomposed it into, which design doc §5.3
// explicitly forbids ("네 개의 독립적인 identity target과 대각 weight로
// 분해해서는 안 된다").
//
// A cross-cloth row therefore couples unknowns of TWO meshes, and §5.4 says
// exactly what that costs: those meshes must share ONE linear system. So the
// solve is MERGED — every active cloth's vertices are concatenated into one
// scalar unknown vector of size N_total = Σ n_mesh (SolveCtx::solveBase is
// the per-mesh offset), and there is ONE matrix and ONE SimplicialLDLT
// factor for the whole substep:
//
//   K_glob = blockdiag(per-mesh contact-free bases)
//          + Σ_c w_c·A_cᵀA_c            (coupled rows, cross-block entries)
//          + Σ_planes w·e_ie_iᵀ          (one-sided rows, diagonal)
//
// Still THREE RHS COLUMNS against ONE scalar matrix: A_c's coefficients
// (1, -β_a) are scalars common to the three spatial axes, exactly like G_t's
// and L_i's, so x/y/z remain independent back-substitutions against the same
// factor. The merge changes WHICH unknowns share a matrix, never how many
// matrices the axes need.
//
// Keeping the factorization current is §8's/§5.6's problem, solved here
// without rank updates: the whole constraint set of the substep is
// summarized as an EPOCH KEY — the one-sided plane weights plus, per coupled
// row, the exact tuple (query ctx, vertex, target ctx, triangle vertices),
// its frozen β and its w_c. When the key equals the one the current factor
// holds, NOTHING happens: no assembly, no analyzePattern, no factorize. An
// empty key against an empty held key is the zero-cost quiescent path.
// Otherwise the epoch is re-assembled and re-factorized. Coupled rows add
// entries the base pattern does not have, so their presence forces
// analyzePattern too; a plane-only epoch keeps the old values-only diagonal
// update against the base pattern, which is why the one-sided scenes pay
// exactly what they paid before.
//
// Per §5.6 the epoch FREEZES β (computed once, at the substep's warm-started
// iterate) and the entry side sigma (read off the narrow-phase row normal),
// while the geometric normal n IS re-evaluated at every local step — §5.6
// item 3 permits it, and it is the same convention the previous mechanism
// used, so a sliding contact's half-space follows the triangle within the
// substep without moving the matrix.
//
// The coupled family has TWO members, which differ only in where A_c's
// coefficients come from. §5.3's vertex-triangle row uses the barycentrics of
// the contact point; §5.5's EDGE-EDGE row uses the two segments'
// interpolation coordinates,
//
//   r_c(q) = (1-s)q₁ + s q₂ - (1-t)q₃ - t q₄ = A_c q,
//   A_c = [(1-s), s, -(1-t), -t]
//
// against the same half-space, with the same local projection and the same
// w_c·A_cᵀA_c / w_c·A_cᵀp_c assembly. They are ONE struct (ContactRow, tagged
// by RowKind) and one set of code paths — see the struct for why.
//
// Edge-edge DETECTION is this file's own CPU pass (step 1b-e): a uniform hash
// grid over edge AABBs plus a clamped segment-segment closest-point test. It
// is PROXIMITY based. §5.5's "검출 단계에는 swept edge–edge 또는 CCD가 별도로
// 필요하다" is NOT satisfied by it and is not claimed to be: a pair that
// crosses completely within one substep is still missed, exactly like the
// vertex-triangle path's, and swept/CCD detection stays future work.
//
// Be honest about what the rest of this is and is not. It IS §5.3/§5.4/§5.5 as
// written. It is NOT the paper's §6, which only ever covers obstacle
// (one-sided) collisions — the coupled row is this design's extension. It does
// NOT include dynamic rigid bodies in the merged unknown (§7's partitioned
// approximation, step (3b)).
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
#include <algorithm>
#include <limits>
#include <memory>
#include <thread>
#include <unordered_map>

template <typename BE, typename PR>
struct PdSystem {};

template <typename PR>
struct PdSystem<METAL, PR> {
    // Local/global sweeps per substep. Liu 2013 converges monotonically but
    // slowly (block coordinate descent); 5-10 is the paper's interactive range.
    // Changing this does NOT change the effective stiffness — unlike PBD,
    // where each sweep applies a fraction of the correction, PD's global
    // solve targets the SAME energy minimum every iteration, so more
    // iterations only mean "closer to it". No correctedK equivalent needed,
    // and PD-7 gates exactly that property (it passes at 1x and 4x this
    // number), which is what makes this a pure CONVERGENCE knob and safe to
    // set from a convergence measurement.
    //
    // 16, RAISED FROM 10 with the continuum strain element (design doc §3).
    // The budget is not a free parameter here: it is the number of sweeps the
    // CONTACT fixed point needs, and the continuum element needs more of them
    // than the stretch springs it replaced. Measured on the PD-5 scene (a free
    // 7x7 sheet dropped on a corner-pinned 9x9 sheet, 3 substeps, 60 trials,
    // "resolved" = every top-sheet vertex still above the pinned sheet's local
    // surface at frame 60):
    //     sweeps   8*    10      12      15      20
    //     resolved  -   56/60   60/60   60/60   60/60
    //   (the spring model resolved the same scene 30/30 at 10 sweeps)
    // The knee is 12; 16 keeps 33% headroom over it. Why the element needs
    // more: it supplies measurably LESS off-diagonal coupling than the spring
    // set for the same kstretch — on the 9x9 sheet the assembled off-diagonal
    // sum is -2.56e7 against the springs' -4.16e7, and on a right-triangle
    // split grid the coupling across the SPLIT DIAGONAL is exactly zero (the
    // element's off-diagonal is the cotangent of the opposite angle, and that
    // angle is 90 degrees at rest), so a contact push is redistributed over
    // fewer paths per sweep and the unilateral rows take longer to settle.
    // Cost: 60% more local/global sweeps per substep. NOT measured in the
    // renderer — this is a correctness setting, and the perf number owed for
    // it is a separate InFrame measurement.
    int iterations = 16;
    // Fraction of velocity removed per substep. 0 = no artificial damping
    // (the implicit-Euler solve is already dissipative).
    PR damping = PR(0);

    // Anomaly (NaN/Inf) counter for the frame; Simulator reads nothing from
    // this today — the guard's job is to keep a blown-up vertex from
    // poisoning the whole mesh. Same role as PbdSystem::sanitizeCount.
    uint32_t sanitizeCount = 0;

    // COUPLED vertex-triangle row statistics for the LAST step() call — one
    // count per §5.3 row, not per participating vertex. `selfContactCount` is
    // the subset whose target mesh is the query mesh itself. Reset at the top
    // of step(); read by the self-tests and the GUI. The name is kept
    // (twoWay) because the GUI/test contract is "rows whose target is a live
    // cloth", which the coupled formulation did not change.
    uint32_t twoWayContactCount = 0;
    uint32_t selfContactCount   = 0;
    // One-sided (half-space plane) rows resolved this step(), the other half
    // of the substep's contact set. Counted for the same reason the two-way
    // ones are: a test that wants to know the contact path RAN cannot read
    // that off the contact ENERGY, which is legitimately zero whenever every
    // plane is satisfied (the unilateral identity branch).
    uint32_t oneSidedContactCount = 0;
    // Coupled EDGE-EDGE rows resolved this step() (design doc §5.5) — one per
    // §5.5 row, same per-substep scope and same reset point as the two above.
    uint32_t edgeEdgeContactCount = 0;

    // ── Edge-edge contact (design doc §5.5) ──────────────────────────────
    // Master gate for the edge-edge detection pass. ON by default for the
    // TriangularCloth PD path: §5.5 is what makes grazing and self contact
    // work at all, so the shipped configuration is the one with it, and a
    // feature reachable only by editing code is not reachable. The GUI
    // exposes it as "엣지-엣지 접촉" beside the contact counters so the A/B
    // that shows what it buys can be run interactively as well as by PD-20.
    //
    // What it gates is DETECTION ONLY. The rows it produces go into the same
    // `contactRows` vector as the vertex-triangle ones and are solved by the
    // same merged system — there is no second solver path to turn off.
    bool edgeContactsEnabled = true;
    // Edge-edge rows the per-substep CAP discarded, summed since construction.
    // MONOTONIC, same contract as contactRefactorCount: what matters is
    // whether it ever moves. Truncating a contact set silently is exactly the
    // "detection missed the pair" failure design doc §6.2 says no stiffness
    // can fix, so it is counted and surfaced rather than absorbed. The rows
    // kept are the DEEPEST ones, so a non-zero here means shallow proximities
    // were dropped, not that a deep penetration went unconstrained.
    uint32_t droppedEdgeRowCount = 0;
    // Number of one-sided contact rows that fed a coupled rigid body this
    // step(). Reset at the top of step() like the counters above.
    uint32_t rigidCoupleCount   = 0;

    // ── Residual penetration of the CONTACT SOLVE (design doc §6.3) ───────
    // Measured AFTER the local/global loop, at the committed iterate q, over
    // every ACTIVE contact row of the substep — one-sided planes as
    // max(0, thickness - dist(q)), coupled rows as max(0, d - nᵀA_c q) with
    // the SAME frozen β and entry side the local step used. Metres; the GUI
    // shows mm.
    //
    // FRAME-SCOPED, not per step() — "이번 frame의 최대 침투 깊이와 평균 침투
    // 깊이" (§6.3). max is over the frame's substeps, mean is over every row
    // of every substep of the frame. Reset by beginFrameContactStats(), which
    // Simulator::update calls once per frame beside beginFrameRigid().
    //
    // The scope is not cosmetic. A resting sheet's contact rows fire in SOME
    // substeps of a frame and not others (the narrow phase reports a row only
    // once a vertex is actually past the surface, while the constraint's
    // target is `thickness` above it), and the pattern is phase-correlated
    // with the substep index — measured, a sheet can go 180 consecutive
    // frames with rows in its early substeps and none in its last. Per-substep
    // scope would therefore report 0 for most frames of a contact that is
    // continuously active, both here and in the GUI.
    //
    // Why these are separate numbers from the contact COUNTS above, and why
    // both exist (design doc §1 clause 5, §6.2): a finite w_c leaves residual
    // penetration even when detection is perfect, and NO stiffness fixes a
    // pair the broad/narrow phase never reported. A row that is present and
    // still overlapping shows up HERE; a missed pair shows up as a count that
    // is too low (or as zero rows on a visibly interpenetrating frame). One
    // number cannot separate the two, so there are two.
    //
    // meanPenetrationDepth averages over the active rows INCLUDING the
    // satisfied ones (penetration 0), i.e. it is the mean over the contact
    // set, not over the violated subset — a set that is 90% satisfied should
    // read as mostly-resolved.
    double maxPenetrationDepth  = 0.0;
    double meanPenetrationDepth = 0.0;
    // How many rows the two numbers above were measured over this frame. 0
    // means NOTHING WAS MEASURED, which a reader must not confuse with "no
    // penetration": both depths are also 0 then. Every consumer that averages
    // these across frames needs this to tell the two apart.
    uint32_t penetrationRowCount = 0;
    // Running Σ of the frame's per-row penetrations; meanPenetrationDepth is
    // this over penetrationRowCount. Kept separate so the published mean is
    // never a mean of substep means (which would weight a substep with one
    // row like a substep with fifty).
    double penetrationSum = 0.0;

    // Open a new penetration-accumulation window. Called once per FRAME by
    // Simulator::update, next to beginFrameRigid() and for the same reason:
    // step() is one SUBSTEP, and these are frame quantities.
    void beginFrameContactStats() {
        maxPenetrationDepth  = 0.0;
        meanPenetrationDepth = 0.0;
        penetrationRowCount  = 0;
        penetrationSum       = 0.0;
    }

    // How many times a contact-epoch change forced factorize() since this
    // system was constructed. MONOTONIC — never reset, by design: the only
    // thing worth knowing about it is its rate, and a probe/self-test that
    // samples it twice gets that from the difference. Base refactors
    // (topology / h / stiffness / mass / pin changes) are NOT counted here;
    // they are counted by nothing, because they are already gated on scalars
    // the user changed deliberately.
    uint32_t contactRefactorCount = 0;
    // How many of those factorize() calls FAILED and dropped the SUBSTEP's
    // whole contact set (the fallback branch in step (1c)). MONOTONIC, same
    // contract as contactRefactorCount. It should stay at 0 forever — K_glob
    // is the base plus Σ w_c·A_cᵀA_c plus a non-negative diagonal, i.e. the
    // base plus a sum of PSD terms — which is exactly why it is worth
    // surfacing: a non-zero here is the "높은 stiffness에서 factorization 실패"
    // case design doc §9.3 asks to be measured, and it makes the
    // silent-loss-of-contacts failure mode visible in the GUI instead of only
    // in stderr. Dropping is now ALL-OR-NOTHING (one merged system, so there
    // is no per-mesh subset to keep), which the fallback below spells out.
    uint32_t contactFactorFailCount = 0;
    // Matrix entries (i, j) the coupled assembly deposited whose two columns
    // lie in DIFFERENT meshes' blocks of the merged system — the mechanism
    // §5.4 exists for, counted at assembly time so a test can gate that the
    // cross-mesh coupling was actually built and not merely intended. Per
    // EPOCH ASSEMBLY (rewritten whenever K_glob is re-assembled), so it
    // describes the matrix the current factor holds. 0 for self-only or
    // single-cloth contact sets, which is correct: those couple inside one
    // block.
    uint32_t crossBlockEntryCount = 0;

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
    //              + Σ_planes  (w/2)|q_i - p_i|²        one-sided contacts
    //              + Σ_rows_c  (w_c/2)‖A_c q - p_c‖²    coupled contacts
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
    // momentum + pin + spring + strain + bending part only.
    //
    // What the split MEANT changed with the coupled rows, and in the solver's
    // favour: a coupled row's local step is now Π_{H_c}(A_c q), the exact
    // Euclidean projection of the relative vector onto a HALF-SPACE — a
    // convex set, so for a fixed H_c the contact term is a genuine block of
    // the same coordinate descent as the rest, exactly like the one-sided
    // planes. The previous mechanism's target was a linearized mass-weighted
    // split re-derived from both meshes' live iterates (a Jacobi read across
    // meshes), which had no such guarantee. The one thing that still moves
    // between iterations is H_c itself, because the geometric normal n is
    // re-evaluated per local step (§5.6 item 3) while β stays frozen — so the
    // descent argument is exact PER ITERATION and only approximate across a
    // normal that rotates. Both sequences are still recorded so a test can
    // gate the full objective and REPORT the smooth part.
    bool debugObjectiveProbe = false;
    std::vector<double> debugObjectiveSeq;      // momentum+pin+spring+contact
    std::vector<double> debugObjectiveSmooth;   // momentum+pin+spring

    // CONTACT STIFFNESS SCALE s_c (design doc §6.1). The weight of one
    // contact constraint is
    //
    //     w_c = s_c · m_eff / h²
    //
    // i.e. s_c is measured in units of the participating inertia, not in
    // N/m: 1.0 means a violated contact pulls exactly as hard as the momentum
    // term of the mass behind it, which halves the residual penetration per
    // local/global iteration — with the projection target re-linearized every
    // iteration the effective behavior is stiff without the matrix being
    // stiff. Expressing it this way is what lets ONE slider mean the same
    // thing on two meshes of different mass (§6.1).
    //
    // m_eff, per the doc, is the inverse of the sum of the row's
    // inverse-mass-weighted coefficients. For a ONE-SIDED plane, where only
    // the query vertex has a nonzero coefficient, that reduces exactly to
    // m_eff = 1/w_v^m = m_i — the vertex's own mass, which is what planeW
    // below uses.
    //
    // A member, not a constexpr, so a GUI hook can tune it live: it enters
    // only the epoch vector, so the cost of a change is ONE factorize on the
    // next substep and never a solver re-init. >> 1 makes the system
    // progressively worse conditioned and re-introduces the locking Bouaziz
    // 2014 §9 warns about; << 1 lets the cloth sink. What raising it does NOT
    // fix is design doc §6.2's list — a missed pair, a wrong normal, a
    // tunnelled vertex — which is what maxPenetrationDepth above is for.
    double kContactWeightScale = 1.0;

    // ── Triangle strain potential (design doc §3.3) ──────────────────────
    // Admissible in-plane singular values: the local projection clamps both
    // singular values of the 3x2 deformation gradient into [min, max]. That
    // is the paper's SINGULAR-VALUE LIMITING model, which the design doc
    // makes the implemented one while leaving the numbers to this slice. A
    // DEGENERATE band (min == max == 1) is the COROTATED model P = UVᵀ —
    // the same code path, clamping to a point instead of an interval.
    //
    // DEFAULT = the degenerate band, CALIBRATED rather than assumed. Inside
    // the band the element contributes exactly zero force (P == F), so a
    // band of half-width ε is FREE PLAY: between two contact supports a span
    // L apart, a vertex can dip ~L·sqrt(2ε)/2 at no energy cost. On the
    // two-cloth stack (PD-5: L ~ 0.083 m, contact thickness 0.01 m) that is
    // 13 mm at ε = 0.05 and 6 mm at ε = 0.01 — enough for the two sheets to
    // interpenetrate past the thickness locally, flip a row's frozen entry
    // side and TUNNEL. Measured, 3 substeps x 10 iterations, top-sheet
    // centroid after 60 frames (pass = still resting, ~0.6):
    //     ε = 0.05  -> -0.46   FALLS THROUGH
    //     ε = 0.01  -> -0.72   FALLS THROUGH
    //     ε = 0.001 ->  rests  (dip ~1.9 mm, under the 10 mm thickness)
    //     ε = 0     ->  rests
    // A band is a real cloth feature but not a safe DEFAULT at this
    // thickness/substep budget: widen it deliberately (inspector "스트레인
    // 한계" row) for a stretchy fabric, and raise thickness or substeps with
    // it.
    //
    // PER MESH, because it is a fabric property, not a solver setting: the
    // values live in ClothBehaviorParams (sigmaMin/sigmaMax), edited per
    // object in the mesh inspector, and are copied into MeshCache each
    // substep. MATERIAL parameters, not solver parameters: the global matrix
    // carries only w_s·A_t·G_tᵀG_t, which does not depend on either value,
    // so both are live knobs that never invalidate a factorization (unlike
    // kstretch, which IS in the matrix and is in the refactor trigger). That
    // is also what makes them legal under PD-7 — changing the iteration
    // count cannot change them, and changing them cannot change the
    // iteration count's meaning.

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

    // Per-mesh strain-limiting band (see the §3.3 block above). Only
    // TriangularCloth carries one — FastGridCloth has no strain elements, so
    // its (never-read) band is the degenerate default. Sanitized here, not at
    // the edit site: an inverted or non-finite band from a hand-written scene
    // file must not turn the SVD clamp into an empty interval.
    static void strainSigmaOf(const BehaviorParams<PR>& bp,
                              double& sigMin, double& sigMax) {
        sigMin = 1.0; sigMax = 1.0;
        if (auto* c = std::get_if<ClothBehaviorParams<PR>>(&bp)) {
            double lo = (double)c->sigmaMin, hi = (double)c->sigmaMax;
            if (!std::isfinite(lo) || !(lo > 0.0)) lo = 1.0;
            if (!std::isfinite(hi) || !(hi > 0.0)) hi = 1.0;
            if (lo > hi) hi = lo;
            sigMin = lo; sigMax = hi;
        }
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
    //
    // UNITS CAVEAT for TriangularCloth: `kstretch` is reused verbatim as the
    // strain potential's PER-AREA weight w_s [N/m ≡ J/m²], not as a spring
    // constant [N/m of extension] — the same number, a different quadratic
    // it multiplies (w_s·A_t·|F-P|² with F dimensionless, vs k·|Δx|²).
    //
    // SAME CAVEAT for `kbend`, one dimension further (design doc §4.3), and
    // this one needs a CONVERSION rather than just a note. The bending
    // element's weight w_b multiplies A_i·|L_i q - p_i|², in which L_i q is a
    // CURVATURE [1/m] and A_i an area [m²], so w_b carries [J] — a bending
    // MODULUS. `kbend` is not one: it was picked for the opposite-vertex
    // DISTANCE spring this element replaces, i.e. an [N/m] whose energy is
    // QUARTIC in curvature (the chord of a bent quad shortens like κ²) and
    // therefore nearly free at small bends. Read verbatim as a modulus the
    // same number is a steel plate: on a grid of spacing L it puts ~20·w_b/L²
    // on the diagonal against the spring model's ~4·kbend, i.e. 5/L² times
    // more — three decades at L ~ 3 cm.
    //
    // MEASURED, not argued (full self-test suite, w_b = kbend·Ā^s for the
    // exponent-free scales below; Ā = the mesh's mean mixed Voronoi area):
    //     w_b = kbend        PD-5 FALLS THROUGH, PD-7 sag drifts 5.4e-2 m
    //                        with the iteration count (16 vs 64) — 54x its
    //                        own tolerance. Not "stiff but fine": the
    //                        element's block is ~1e4x the momentum term, so
    //                        16 sweeps do not reach the fixed point and the
    //                        contact rows never settle. PD-8's objective
    //                        reads 101.9 against the spring model's 0.51.
    //     w_b = kbend·Ā/4    passes.
    //     w_b = kbend·Ā      passes, PD-7 drift 3.2e-6 (the spring model's
    //                        own figure was 5.5e-5), PD-8 objective 0.63.
    // So the conversion is w_b = kbend·Ā, and Ā is GEOMETRY, not a fitted
    // constant: it is the area a single vertex owns, which is exactly the
    // factor that turns a per-vertex stiffness into a per-area modulus and
    // puts the element's diagonal back at the ~kbend scale the spring model
    // had. Cost of the choice, stated plainly: w_b then moves with mesh
    // resolution (Ā ~ L²), so refining a mesh softens its bending — the same
    // resolution dependence the spring model had, and NOT the mesh
    // independence §9.1 clause 4 asks of the STRAIN element. Buying that back
    // means giving kbend the units of a modulus, which is a scene-data
    // migration (every scene's kbend, the GUI field's range, the symplectic
    // path that reads the same number) and is deliberately not in this slice.
    //
    // Ā lives in BendCache::areaScale, built with the rows and invalidated
    // with them.
    //
    // How much in-plane stiffness the same number buys, DERIVED on the
    // diagonal-split grid this engine generates (spacing L, per cell: 1
    // horizontal + 1 vertical + 1 diagonal spring against 2 triangles). For a
    // uniform in-plane strain e applied along direction θ:
    //     E_spring / (kL²/2)  = |e|²·(3/2 + sin2θ)      anisotropic
    //     E_strain / (w_sL²/2) = |e|²                    isotropic
    // so at w_s = k the spring set is stiffer by 3/2 + sin2θ — between 1/2
    // and 5/2 depending on direction, with the DIRECTION-AVERAGED ratio
    // exactly 3/2 (⟨sin2θ⟩ = 0). The assembled matrices agree: on PD-5's 9x9
    // sheet the strain off-diagonal sum is -2.56e7 against the stretch
    // springs' -4.16e7, a ratio of 1.63.
    //
    // The mapping is deliberately left at w_s = kstretch and NOT scaled by
    // 3/2, because no single scalar can reproduce an anisotropic model with
    // an isotropic one, and because the 3/2 mapping was MEASURED not to
    // improve the one scene that is sensitive to it (PD-5: 16/30 at 1.5x
    // against 24/30 at 1x, and 30/30 at both 0.5x and 3x — the response is
    // non-monotone, so any scalar picked to make it pass would be fitted to
    // noise). What that scene actually needed was solver CONVERGENCE, see
    // PdSystem::iterations.
    //
    // The numbers are NOT interchangeable across the two behaviors, which is
    // precisely what the supported-combination table at the top records.
    // PD-7's gate is unaffected: it only requires w_s to be independent of
    // the iteration count, which a constant is.
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

    // ── Triangle strain element (design doc §3.1/§3.2) ───────────────────
    // One rest triangle's CONSTANT operators. Built once per topology, from
    // the pose the cache was built at — the same "rest = live pose at build"
    // convention buildSprings uses for its bend springs.
    //
    // The rest triangle is flattened isometrically into 2D:
    //   ū_i = (0,0), ū_j = (|e1|, 0),
    //   ū_k = (e1·e2/|e1|, |e1 x e2|/|e1|),      e1 = x_j-x_i, e2 = x_k-x_i
    // and D_m = [ū_j-ū_i, ū_k-ū_i] (2x2, COLUMNS), A_t = ½|det D_m|.
    //
    // `G` is G_t = D_m^{-T}·S ∈ R^{2x3} with S = [[-1,1,0],[-1,0,1]], the
    // operator that acts on the THREE VERTEX SCALARS of one coordinate axis.
    // The identity it satisfies, which the local step relies on:
    //
    //   D_sᵀ = S·Q_t                    (Q_t ∈ R^{3x3} stacks q_i,q_j,q_k as
    //                                    ROWS, so S·Q_t has rows (q_j-q_i)ᵀ,
    //                                    (q_k-q_i)ᵀ — exactly D_sᵀ)
    //   F_t  = D_s·D_m^{-1}             (§3.2, 3x2)
    //   F_tᵀ = D_m^{-T}·D_sᵀ = D_m^{-T}·S·Q_t = G_t·Q_t          ✓
    //
    // Componentwise that is F(a, r) = Σ_s G(r, s)·q(v_s, a) for spatial axis
    // a and material index r — the form both the local step and the RHS
    // gather below are written in.
    //
    // Note G's COLUMNS sum to zero (S's do, and D_m^{-T} is applied on the
    // left), so G_tᵀG_t has zero row sums exactly like a graph Laplacian:
    // the element is translation invariant and only positive SEMI-definite
    // on its own, which is why the mass term is what makes the system SPD.
    struct StrainTri {
        Index  v[3];
        double G[2][3];
        double area;      // A_t
    };
    // One incident triangle of a vertex: which triangle, and which of its
    // three local slots (0/1/2) this vertex occupies — the slot IS the column
    // of G_t whose 2-vector weights this vertex's RHS contribution.
    struct TriIncident { Index tri; uint32_t slot; };

    // ── Cotangent mean-curvature bending element (design doc §4.1) ───────
    // One entry of a Laplace–Beltrami ROW. Row i is
    //
    //   L_i q = (1/A_i) Σ_{j∈N(i)} c̃_ij (q_j - q_i),
    //   c̃_ij = ½(cot α_ij + cot β_ij),
    //
    // stored FLAT: one entry per one-ring neighbour j with coefficient
    // c̃_ij/A_i, plus ONE diagonal entry (i, -Σ_j c̃_ij/A_i). The row's
    // coefficients therefore sum to exactly zero, so L_i annihilates a
    // translation and L_iᵀL_i has zero row sums — the same property that
    // makes the strain element translation invariant, and what keeps PD-3's
    // centre-of-mass argument true with the element in the matrix.
    //
    // SIGN CONVENTION: positive off-diagonal, negative diagonal (the
    // "q_j - q_i" direction spelled out above). The energy is quadratic in
    // L_i q so the overall sign cancels, but the REST curvature v_i⁰ and the
    // live v_i must be measured with the same one (design doc §4.1) — hence
    // one builder produces both, from these same entries.
    struct BendEntry { Index col; double c; };
    // Transpose incidence: "row `row` touches me with coefficient c". The
    // RHS gather needs it because L_iᵀ scatters row i over the WHOLE one-ring
    // of i, so a vertex's RHS collects from every row that names it — not
    // just from its own. Same one-writer-per-row reason as buildTriIncidence.
    struct BendTIncident { Index row; double c; };

    // Everything the bending element caches per mesh, built once per topology
    // from the rest (= live at build) pose. Grouped in one struct rather than
    // eight parallel MeshCache members because they are built, cleared and
    // invalidated as a unit.
    struct BendCache {
        // Mixed Voronoi area A_i (Meyer et al. 2003), n entries. ZERO marks a
        // vertex that carries NO constraint — a degenerate one-ring, an area
        // under tolerance, or a vertex no valid triangle references (design
        // doc §4.1's guard). Such a vertex simply has an EMPTY row.
        std::vector<double> area;
        std::vector<Index>     rowOffsets;   // n + 1
        std::vector<BendEntry> rowEntries;   // Σ_i (|N(i)| + 1)
        std::vector<Index>         tOffsets; // n + 1
        std::vector<BendTIncident> tEntries; // same count as rowEntries
        std::vector<double> v0;      // 3n, rest curvature vector v_i⁰
        std::vector<double> v0len;   // n,  ‖v_i⁰‖
        // Projected auxiliaries p_i of the local step, flat 3 per vertex.
        // SEEDED with v_i⁰ and only ever overwritten by a well-conditioned
        // projection, which is what makes §4.2's fallback chain fall out of
        // the storage instead of needing a branch: previous direction if one
        // was stored, else the rest curvature direction (the seed), else
        // exactly zero (the seed again, when ‖v_i⁰‖ = 0).
        std::vector<double> p;
        // CALIBRATION (see the kbend units caveat on springConstantsOf): the
        // scalar that turns the mesh's `kbend` into the element's weight,
        // w_b = kbend · areaScale. Geometry, not a tunable: it is the mesh's
        // MEAN mixed area Ā, so it is fixed by the same topology build as the
        // rows and moves only when they do.
        double areaScale = 0.0;
        // Vertices whose constraint was skipped, plus triangles rejected from
        // the cotangent/area accumulation. Diagnostics only.
        uint32_t degenerate = 0;
        void clear() {
            area.clear(); rowOffsets.clear(); rowEntries.clear();
            tOffsets.clear(); tEntries.clear();
            v0.clear(); v0len.clear(); p.clear();
            areaScale = 0.0;
            degenerate = 0;
        }
    };

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

    // Which of the two COUPLED constraint families a row belongs to.
    //
    // ONE struct with a tag, deliberately, instead of a second `EdgeRow` type
    // and a second copy of the assembly / local / penetration / epoch-key /
    // clamp code. The two constraints are the SAME object in the design doc:
    // §5.3 is r_c = q_v - Σβ_a q_a and §5.5 is r_c = (1-s)q₁ + s q₂ -
    // (1-t)q₃ - t q₄, and both are "A_c over four vertices, projected onto the
    // same half-space, assembled as w_c·A_cᵀA_c / w_c·A_cᵀp_c". Written in the
    // generic A_c = [α₀ α₁ α₂ α₃] form below, FOUR of the five shared code
    // paths become literally identical and the fifth (the local step) differs
    // in exactly two places: where the geometric normal comes from, and
    // whether column 0 takes a `cd` share. Duplicating five paths to avoid one
    // tag would be five places for the two families to drift apart.
    enum class RowKind : uint8_t { VertexTriangle, EdgeEdge };

    // One COUPLED contact row of this substep — design doc §5.3
    // (vertex-triangle) or §5.5 (edge-edge) — resolved once (step 1b) and
    // re-projected from the live iterates at every local step. It spans up to
    // two meshes, which is why the list hangs off PdSystem and not MeshCache.
    //
    // The constraint is r_c(q) = Σ_a α_a q_a = A_c q against the half-space
    // H_c = { r : nᵀr >= d }, so the FROZEN part of it is exactly what §5.6
    // says freezes for one contact epoch:
    //   * which four vertices participate (and in which contexts),
    //   * the interpolation coefficients α — the barycentrics β for a
    //     vertex-triangle row, (s, t) for an edge-edge one. They are the
    //     matrix's coefficients, so re-deriving them per iteration would
    //     rewrite K_glob per iteration and destroy the factorization reuse
    //     §5.6 is about. Both are measured ONCE, at the substep's
    //     warm-started iterate.
    //   * the entry side, carried by `nrm`: for a vertex-triangle row the
    //     narrow-phase row normal, for an edge-edge row the detection-time
    //     oriented edge-cross normal. sigma is read off it and never off the
    //     possibly already-corrected current geometry,
    //   * d = the row's contact thickness,
    //   * w_c = s_c·m_eff/h² with m_eff from the frozen α (§6.1), which is
    //     what makes the weight constant over the epoch and therefore usable
    //     as part of the epoch key.
    // The geometric normal direction is NOT frozen: §5.6 item 3 allows the
    // half-space target to be re-evaluated in the local step, and it is —
    // only its SIGN is pinned by `nrm`. (The one exception is
    // `frozenNormal` below.)
    //
    // The four participating vertices are stored PER COLUMN rather than as
    // "one query + three targets": an edge-edge row's first two columns are
    // one mesh and its last two another, and the vertex-triangle row's 1 + 3
    // split is only a special case of that. `col[a]` is the COLUMN of the
    // merged system (solveBase + local index), precomputed so both the
    // assembly and the RHS deposit index the global vector without
    // re-deriving the offset.
    //
    // Column order:
    //   VertexTriangle : q_v, then the target triangle (1, 2, 3),
    //                    α = [1, -β₁, -β₂, -β₃]
    //   EdgeEdge       : q₁, q₂ of the first edge, then q₃, q₄ of the second,
    //                    α = [(1-s), s, -(1-t), -t]
    // Both sum to zero, so A_c annihilates a rigid translation of the four
    // vertices and w_c·A_cᵀA_c has zero row sums.
    struct ContactRow {
        RowKind kind = RowKind::VertexTriangle;
        int    cx[4]  = {0, 0, 0, 0};      // ctxs index of each vertex
        Index  vi[4]  = {0, 0, 0, 0};      // local vertex index within it
        int    col[4] = {0, 0, 0, 0};      // merged-system columns
        double alpha[4] = {0.0, 0.0, 0.0, 0.0};   // A_c coefficients
        PR     iw[4] = {PR(0), PR(0), PR(0), PR(0)};   // inverse masses
        Vec3   nrm;                    // frozen entry side (sigma source)
        PR     thk = PR(0);            // d
        double mEff = 0.0;             // 1/Σ α_a² w_a^m
        double w = 0.0;                // w_c = s_c·m_eff/h²
        bool   cross = false;          // the columns span >1 mesh block
        // EDGE-EDGE only: the frozen interpolation coordinates of §5.5. They
        // ARE α (up to sign), kept separately because the epoch key names them
        // and because a reader checking this row against the doc wants to see
        // them rather than reconstruct them.
        PR     s = PR(0), t = PR(0);
        // EDGE-EDGE only: the two edges were NEAR PARALLEL at detection, so
        // their cross product is not a usable normal direction. `nrm` is then
        // used VERBATIM for the whole epoch instead of being re-evaluated per
        // local step — a deterministic fallback, and the one place §5.6 item
        // 3's "the normal may be re-evaluated" is deliberately not taken,
        // because re-evaluating a degenerate cross product per iteration would
        // make the half-space jump with the round-off.
        bool   frozenNormal = false;
    };
    // Cleared and rebuilt every substep by step (1b) / (1b-e).
    std::vector<ContactRow> contactRows;

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
        // ── Triangle strain elements (TriangularCloth only; empty for
        // FastGridCloth, which keeps the legacy spring model). Built on the
        // SAME topology trigger as `springs`, for the same reason: the rest
        // operators are measured from the live pose and must not be
        // re-sampled from a deformed configuration mid-run.
        std::vector<StrainTri> strainTris;
        // Projected auxiliaries P_t of the local step, 6 doubles per triangle
        // laid out ROW-MAJOR as the 3x2 matrix P: p[t*6 + a*2 + r] = P(a, r).
        // Seeded with the REST frame at build time and only ever overwritten
        // by a SUCCESSFUL projection, which is what makes "reuse the previous
        // P_t" the automatic behaviour when an SVD comes back non-finite
        // (design doc §3.3 last paragraph) — the slot simply is not touched.
        std::vector<double> strainP;
        // Vertex -> incident triangles, CSR, exactly parallel to
        // adjOffsets/adjEntries and there for the same reason: the strain RHS
        // is assembled as a GATHER so every row has a single writer.
        std::vector<Index>       triOffsets;   // n + 1
        std::vector<TriIncident> triEntries;   // 3 * strainTris.size()
        // Triangles rejected by the last cache build (zero-length first edge,
        // singular D_m, area under tolerance, repeated/out-of-range index).
        // Diagnostics only — a degenerate triangle simply carries no
        // constraint (design doc §3.1).
        uint32_t degenerateTriCount = 0;
        // ── Edge-edge contact source data (design doc §5.5). Built on the
        // SAME topology trigger as everything above, because it is pure
        // topology: `edgeV` is the mesh's edge list SANITIZED once (2 indices
        // per edge, in range, non-degenerate — adjacency.edges.size is the
        // initializer's UPPER BOUND and its tail is uninitialised pool memory,
        // see buildSprings), and ring* is the one-ring CSR derived from it.
        //
        // The one-ring is what the SELF-pair admissibility filter needs. The
        // existing self filters are "not the vertex's own triangle" (CPU broad
        // phase) and "not an incident facet" (GPU narrow, sceneVertexAdjFacets)
        // — i.e. one-ring in spirit, which is also what design doc §5.4 asks
        // for ("인접 triangle, 동일 one-ring, 잘못된 자기 참조는 collision
        // 단계에서 제외"). For an edge PAIR that means: no shared vertex, and
        // no endpoint of one edge in the one-ring of an endpoint of the other,
        // which rejects every pair of edges belonging to the same or to two
        // vertex-adjacent triangles.
        std::vector<Index> edgeV;                     // 2 per usable edge
        std::vector<Index> ringOffsets, ringEntries;  // one-ring CSR
        // ── Cotangent bending element (TriangularCloth only; empty for
        // FastGridCloth). Built on the SAME topology trigger as `springs` and
        // `strainTris`, and for the same reason: the cotangents, the mixed
        // areas and the rest curvature are measured from the live pose and
        // must not be re-sampled from a deformed configuration mid-run
        // (design doc §4.1's near-isometry assumption is what lets them be
        // frozen at all).
        BendCache bend;
        // Matrix-invalidating scalars, compared verbatim each substep. kS is
        // BOTH the spring stretch constant (FastGridCloth) and the per-area
        // strain weight w_s (TriangularCloth) — one runtime knob, two
        // meanings, see springConstantsOf.
        double h = 0.0, kS = -1.0, kB = -1.0;
        // Per-mesh strain band, copied from ClothBehaviorParams EVERY substep
        // (strainSigmaOf) — NOT matrix-invalidating (see the §3.3 block on
        // the band), so it lives outside the refactor comparison above and an
        // inspector edit takes effect on the very next substep for free.
        double sigMin = 1.0, sigMax = 1.0;
        // Copies of the two per-vertex arrays that enter the matrix. The pin
        // mask decides which diagonals carry wPin and the masses ARE the
        // M/h² diagonal, so a change in either must refactor. O(n) memcmp
        // per substep, which is nothing next to the solve.
        std::vector<PR> pinMask, mass;
        // Eigen's solvers derive from internal::noncopyable, so the object
        // cannot live by value inside a std::vector element (resize needs
        // move-insertable). unique_ptr restores movability without copying
        // the factorization.
        //
        // THIS FACTOR NO LONGER SOLVES ANYTHING. Since the coupled contact
        // rows of §5.3 span two meshes, the substep is solved by ONE merged
        // factorization over every active cloth (see globFactor). What this
        // one is still for is the per-mesh VALIDITY gate: it is factorized
        // once per BASE REBUILD (topology / h / stiffness / mass / pin
        // change — not per substep, and never per contact epoch), and a mesh
        // whose own contact-free block will not factor is excluded from the
        // merge instead of poisoning every other cloth's solve with it. It is
        // also what the strain/bending self-tests read to assert the assembled
        // block is SPD.
        std::unique_ptr<Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>> factor;
        bool valid = false;       // false => base won't factor, skip mesh
        // The CONTACT-FREE block M/h² + L + wPin·P of this mesh. It is the
        // diagonal block this mesh contributes to the merged base matrix, so
        // it is rebuilt only when the base itself changes and the merged base
        // is re-assembled from the blocks at the same moment.
        Eigen::SparseMatrix<double> Abase;
        // One-sided contact planes of THIS substep, and the per-vertex sum of
        // the weights of the planes this mesh's vertices carry. COUPLED rows
        // are NOT in this vector any more: their contribution is a dense 4x4
        // block, not a diagonal weight, so they live in the global epoch key
        // (PdSystem::epochKey) with their β and w_c instead.
        //
        // `epochW` is the copy of contactW the current merged factorization
        // holds; `contactW` empty means "no planes on this mesh". A base
        // rebuild clears epochW (through the global key), which is how a
        // change of h / mass / k — all of which move w_c — is composed into
        // the contact epoch.
        std::vector<ContactPlane> planes;
        std::vector<double> contactW, epochW;
        // contactW carries at least one non-zero (it is sized-and-zeroed
        // before the sweep, and only collapsed to empty afterwards if this
        // stayed false).
        bool hasContactW = false;
        // Per-substep solve state, kept here only so the allocations persist
        // across substeps (every value is rewritten each step). n x 3: the
        // solve is scalar and the three coordinates are INDEPENDENT (every
        // operator's coefficients — a spring's, G_t's, L_i's, A_c's — are
        // common to the three axes), which is the whole reason one
        // factorization serves all of them.
        //
        // These stay PER MESH even though the solve itself is merged: every
        // local pass is written against one mesh's own arrays, and a coupled
        // row deposits into the PARTNER mesh's rhs the same way it deposits
        // into its own. The merged step gathers them into rhsGlob and
        // scatters the answer back — see the GLOBAL block of step (2).
        Eigen::MatrixXd rhsBase, rhs, q;
        // Momentum target s of THIS substep, kept only for the objective
        // probe (the solve itself needs it folded into rhsBase, not raw).
        // Written by a single gated assignment in step (1) and never resized
        // when the probe is off, so an unprobed run never allocates it.
        Eigen::MatrixXd sMom;
        // Contact-attributable displacement estimator for the COUPLED rows
        // (flat 3N), zeroed each substep, and only for the vertices in their
        // TRIANGLE role. The velocity update needs to know how much of a
        // vertex's position delta came from contact so depenetration does not
        // become velocity — PbdSystem::cDispBlocks, same idea. With the rows
        // in the energy there is no direct push to sum any more; what
        // accumulates here is the per-iteration inverse-mass SHARE of the
        // row's violation, -n·m_eff·β_a·w_a^m·g. The derivation and the
        // "over-estimate is the safe side" argument are in the local step's
        // contact block.
        std::vector<PR> cd;
    };
    std::vector<MeshCache> cache;   // parallel to Scene::meshes

    // ── The MERGED global cloth system (design doc §5.4) ─────────────────
    // Every active cloth's vertices concatenated into ONE scalar unknown
    // vector of size nTotal = Σ n_mesh, so that a cross-cloth A_c row can put
    // its off-diagonal entries in the same matrix as the two meshes' own
    // elasticity. "mesh별 독립 factorization과 상대 mesh의 이전 iterate를 읽는
    // Jacobi 교환은 근사 경로로만 분류한다" — this is the exact path instead.
    //
    // The merged system is ALWAYS used, even for a single cloth and even with
    // no coupled rows at all. The alternative (per-mesh factors until a
    // cross-mesh row appears, then switch) buys nothing that matters: with
    // one cloth the merged matrix IS the per-mesh matrix, and with several
    // the base is block diagonal, which is what LDLT's fill-reducing ordering
    // would produce anyway. What it WOULD cost is two assembly/epoch paths
    // that have to stay in agreement. The property that had to be preserved —
    // "a quiescent scene does zero factorization work" — is preserved by the
    // epoch key, not by the per-mesh split: an unchanged key skips assembly,
    // analysis and factorization outright.
    //
    // Kbase is blockdiag(MeshCache::Abase), rebuilt only when some mesh's
    // base is rebuilt or the set/order of active cloths changes. Kglob is the
    // matrix the current factor actually holds (base + this epoch's contact
    // terms), kept so a test can inspect the assembled coupling.
    Eigen::SparseMatrix<double> Kbase, Kglob;
    std::unique_ptr<Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>> globFactor;
    bool globValid = false;
    Index nTotal = 0;
    // Merged RHS / iterate scratch, nTotal x 3. The per-mesh MeshCache::rhs
    // and ::q stay the working arrays every local pass reads and writes (the
    // whole rest of the solver is per-mesh and does not need to know about
    // the merge); the global step gathers those into rhsGlob, solves, and
    // scatters the answer back into each mesh's q. Two nTotal x 3 copies per
    // iteration against a sparse back-substitution — not a cost worth a
    // rewrite of every pass.
    Eigen::MatrixXd rhsGlob, qGlob;
    // Contact epoch key (design doc §5.6) and the copy the current
    // factorization holds. Flat doubles, built in a deterministic order:
    //   planes  : (ctx index, vertex, weight) per non-zero plane weight
    //   coupled : (qc, vert, pc, i1, i2, i3, β₁, β₂, β₃, w_c) per row
    // Indices go in as doubles, which is exact well past any mesh size. Empty
    // == empty is the quiescent zero-cost comparison.
    std::vector<double> epochKey, epochKeyHeld;
    // Whether the pattern the current analyzePattern() saw includes coupled
    // rows. A plane-only epoch adds only DIAGONAL entries, which the base
    // pattern already has, so it can be applied values-only against the base
    // analysis exactly as it was before this slice; coupled rows add entries
    // the base does not have and force a fresh symbolic analysis, in and out.
    bool epochHasRows = false;
    // Layout the current Kbase was assembled from: (mesh array index, n) per
    // active cloth, in ctx order. A change here means the merged system's
    // columns moved and the base must be re-assembled.
    std::vector<Index> baseLayout;
    // False only after the merged factorization FAILED for this substep: it
    // then runs contact-free (both families dropped, since one factor now
    // carries them all) and the local step deposits no contact RHS at all.
    bool contactsLive = true;

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
        // Offset of this mesh's vertices in the PACKED COLLISION arrays
        // (Scene::packedMeshData.statesOffsets) — narrow-phase indexing.
        Index colBase = 0;
        // Offset of this mesh's block in the MERGED linear system. Distinct
        // from colBase on purpose: the collision packing spans every mesh in
        // the scene, the merged solve only the ACTIVE cloths.
        Index solveBase = 0;
    };
    std::vector<SolveCtx> ctxs;      // one per ACTIVE cloth mesh
    std::vector<int> ctxOfMesh;      // mesh array index -> ctxs index, -1 = none

    // Stretch springs from the edge list + bend springs from the facet list.
    //
    // Bend model is Provot's opposite-vertex spring (the two vertices facing
    // each other across an interior edge), NOT a dihedral-angle constraint:
    // PD needs every constraint to be a quadratic distance-to-a-projected-set
    // energy, and the dihedral form is only expressible that way through a
    // per-vertex linear operator — real matrix blocks, not a scalar Laplacian
    // entry. That operator is exactly what buildBendRows builds for
    // TriangularCloth; keeping the spring here is what makes FastGridCloth's
    // assembly a plain graph Laplacian, unchanged from the GPU twin's model.
    //
    // Rest lengths: stretch reads adjacency.restEdgeLengths (the initializer's
    // measurement); bend is measured from the CURRENT positions here, the same
    // "re-measure rest from live geometry" convention
    // MeshAdjacency::recomputeRestLengths and PbdSystem::buildBendQuads use.
    //
    // FASTGRIDCLOTH ONLY. Design doc §3.5 removes the per-edge distance
    // spring (and the diagonal-as-implicit-shear that came with it) from the
    // TriangularCloth PD path in favour of the continuum strain element, and
    // §4 removes the opposite-vertex bend spring below in favour of the
    // cotangent element — so that behavior now builds NO springs at all and
    // never calls this. The spring passes of the solve see an empty list for
    // it and cost nothing. Nothing here is conditional any more precisely
    // because the ONE caller left is the legacy mass-spring behavior, whose
    // whole model is this function.
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

    // ── Edge list + one-ring for edge-edge contact (design doc §5.5) ─────
    // Sanitize the mesh's edge list ONCE per topology and invert it into a
    // one-ring CSR, so the per-substep detection pass does neither.
    //
    // The sanitation is not optional and not cosmetic: `adjacency.edges.size /
    // 2` is the initializer's UPPER BOUND (210 slots against 161 written edges
    // on an 8x8 grid) and the tail is uninitialised pool memory reinterpreted
    // as indices — the exact hazard buildSprings documents at length. The
    // guard here is the same one, minus the rest-length test, because a valid
    // edge with a zero REST length is still a perfectly good edge to collide
    // (its rest length says nothing about its current geometry, and
    // TriangularCloth does not build springs from it anyway); the length that
    // matters is measured live at detection time.
    static void buildEdgeList(const GeneralMesh<METAL, PR>& mesh, Index n,
                              std::vector<Index>& edgeV,
                              std::vector<Index>& ringOffsets,
                              std::vector<Index>& ringEntries) {
        edgeV.clear();
        ringOffsets.assign((size_t)n + 1, 0);
        ringEntries.clear();
        const Index* edges = mesh.adjacency.edges.ptr;
        if (!edges || n == 0) return;
        const Index numEdges = (Index)mesh.adjacency.edges.size / 2;
        edgeV.reserve((size_t)numEdges * 2);
        for (Index e = 0; e < numEdges; ++e) {
            const Index a = edges[e*2], b = edges[e*2+1];
            if (a >= n || b >= n || a == b) continue;
            edgeV.push_back(a);
            edgeV.push_back(b);
        }
        // Counting-sort inversion, same two-pass shape as buildIncidence: the
        // loop above already guarantees both endpoints are in range.
        for (size_t k = 0; k < edgeV.size(); k += 2) {
            ++ringOffsets[edgeV[k]   + 1];
            ++ringOffsets[edgeV[k+1] + 1];
        }
        for (Index i = 0; i < n; ++i) ringOffsets[i+1] += ringOffsets[i];
        ringEntries.resize(edgeV.size());
        std::vector<Index> cursor(ringOffsets.begin(), ringOffsets.end() - 1);
        for (size_t k = 0; k < edgeV.size(); k += 2) {
            const Index a = edgeV[k], b = edgeV[k+1];
            ringEntries[cursor[a]++] = b;
            ringEntries[cursor[b]++] = a;
        }
    }

    // Is `b` in the one-ring of `a`? Linear over a row that is ~6 long on a
    // regular grid — a set lookup would cost more than the scan.
    static bool inOneRing(const MeshCache& mc, Index a, Index b) {
        if ((size_t)a + 1 >= mc.ringOffsets.size()) return false;
        for (Index k = mc.ringOffsets[a]; k < mc.ringOffsets[a+1]; ++k)
            if (mc.ringEntries[k] == b) return true;
        return false;
    }

    // Closest points of the two SEGMENTS [p1,p2] and [p3,p4], returned as the
    // interpolation coordinates (s, t) of design doc §5.5 — the textbook
    // clamped segment-segment routine (Ericson, RTCD §5.1.9). Returns false
    // for a degenerate input (either segment shorter than `eps`), which is the
    // one case §5.5's A_c cannot be built for: a zero-length "edge" makes two
    // of the four columns coincide.
    //
    // The result is EXACTLY the constraint's r_c at the detection pose:
    //   c1 - c2 = (1-s)p₁ + s p₂ - (1-t)p₃ - t p₄ = A_c q,
    // so the caller reads the entry side off it without re-deriving anything.
    static bool closestSegSeg(const Vec3& p1, const Vec3& p2,
                              const Vec3& p3, const Vec3& p4,
                              PR& sOut, PR& tOut) {
        const PR eps = PR(1e-12);
        const Vec3 d1 = p2 - p1, d2 = p4 - p3, r = p1 - p3;
        const PR a = d1.dot(d1), e = d2.dot(d2);
        if (a <= eps || e <= eps) return false;
        const PR f = d2.dot(r), cdot = d1.dot(r), b = d1.dot(d2);
        const PR denom = a * e - b * b;
        auto clamp01 = [](PR v) {
            return v < PR(0) ? PR(0) : (v > PR(1) ? PR(1) : v);
        };
        // denom == 0 <=> the segments are parallel: any s is as good, so pick
        // the start and let the t clamp below place the pair.
        PR s = (denom > eps) ? clamp01((b * f - cdot * e) / denom) : PR(0);
        PR t = (b * s + f) / e;
        if (t < PR(0))      { t = PR(0); s = clamp01(-cdot / a); }
        else if (t > PR(1)) { t = PR(1); s = clamp01((b - cdot) / a); }
        sOut = s; tOut = t;
        return true;
    }

    // Uniform hash-grid cell key. 21 bits per axis, biased so negative cell
    // coordinates stay in range — the same encoding CpuSpatialHash::cellKey
    // uses, kept identical on purpose so the two grids cannot disagree about
    // what a cell is.
    static inline uint64_t edgeCellKey(int32_t x, int32_t y, int32_t z) {
        const uint64_t ux = (uint64_t)(uint32_t)(x + 0x100000) & 0x1FFFFFull;
        const uint64_t uy = (uint64_t)(uint32_t)(y + 0x100000) & 0x1FFFFFull;
        const uint64_t uz = (uint64_t)(uint32_t)(z + 0x100000) & 0x1FFFFFull;
        return (ux << 42) | (uy << 21) | uz;
    }

    // One edge of the substep's detection pass: which cloth it belongs to,
    // which of that cloth's edges it is, and its margin-inflated AABB at the
    // warm-started iterate.
    struct EdgeBox {
        int   cx = 0;          // ctxs index
        Index e = 0;           // edge index within that ctx's MeshCache::edgeV
        Index a = 0, b = 0;    // its two vertices, local to the ctx
        PR    lo[3] = {PR(0), PR(0), PR(0)};
        PR    hi[3] = {PR(0), PR(0), PR(0)};
    };
    // One bucket of the grid. GENERATION STAMPED rather than cleared, the
    // same trick CpuSpatialHash::Cell uses: a substep bumps `edgeGridGen` and
    // a bucket whose gen is stale is treated as empty and reused in place, so
    // the map keeps its nodes and the vectors keep their capacity instead of
    // being destroyed and rebuilt 180 times a second.
    struct EdgeCell { uint32_t gen = 0; std::vector<uint32_t> items; };
    // Reused across substeps so the detection pass allocates nothing in
    // steady state. Members, not locals, for exactly that reason.
    std::vector<EdgeBox> edgeBoxes;
    std::unordered_map<uint64_t, EdgeCell> edgeGrid;
    uint32_t edgeGridGen = 0;
    std::vector<uint32_t> edgeCandidates;

    // ── Triangle strain rest cache (design doc §3.1) ─────────────────────
    // Build the constant per-triangle operators from the pose the mesh is in
    // RIGHT NOW (the "rest = live pose at build" convention, see
    // buildSprings), and seed each triangle's auxiliary P_t with its own rest
    // frame so the local step's non-finite fallback always has something
    // valid to fall back to on the very first iteration.
    //
    // Degenerate triangles create NO constraint and are counted instead: a
    // singular D_m has no inverse, and a near-zero area would blow G_t up to
    // the point where its block dominates the whole matrix.
    static void buildStrainTriangles(const GeneralMesh<METAL, PR>& mesh, Index n,
                                     std::vector<StrainTri>& out,
                                     std::vector<double>& pOut,
                                     uint32_t& degenerate) {
        out.clear(); pOut.clear(); degenerate = 0;
        const PR* pos = mesh.state.x.ptr;
        const Index* facets = mesh.adjacency.facets.ptr;
        if (!pos || !facets || n == 0) return;
        const Index numFacets = (Index)mesh.adjacency.facets.size / 3;
        out.reserve((size_t)numFacets);
        pOut.reserve((size_t)numFacets * 6);

        // Area/determinant floor. Absolute, not relative: G_t scales like
        // 1/length, so what has to stay bounded is the ABSOLUTE flattened
        // area, and 1e-12 m² is ~1e-6 m of edge — six decades under any mesh
        // this solver runs.
        constexpr double kTriEps = 1e-12;

        for (Index f = 0; f < numFacets; ++f) {
            const Index a = facets[f*3+0], b = facets[f*3+1], c = facets[f*3+2];
            // Same uninitialised-tail guard buildSprings documents for edges.
            if (a >= n || b >= n || c >= n) continue;
            if (a == b || b == c || a == c) { ++degenerate; continue; }

            const Vec3 xi = vertexAt(pos, a);
            const Vec3 e1 = vertexAt(pos, b) - xi;
            const Vec3 e2 = vertexAt(pos, c) - xi;
            const double l1 = (double)e1.norm();
            if (!(l1 > kTriEps)) { ++degenerate; continue; }

            // Isometric flattening: ū_i at the origin, ū_j on the +x axis,
            // ū_k in the upper half plane. Lengths and the enclosed angle are
            // preserved by construction, so A_t below IS the 3D rest area.
            const double u2x = l1;
            const double u3x = (double)e1.dot(e2) / l1;
            const double u3y = (double)e1.cross(e2).norm() / l1;
            // D_m = [ū_j-ū_i, ū_k-ū_i], columns.
            const double dm00 = u2x, dm01 = u3x;
            const double dm10 = 0.0, dm11 = u3y;
            const double det = dm00 * dm11 - dm01 * dm10;   // = u2x * u3y
            const double area = 0.5 * std::abs(det);
            if (!(std::abs(det) > kTriEps) || !(area > kTriEps)) {
                ++degenerate; continue;
            }
            // D_m^{-1}, closed form for 2x2.
            const double i00 =  dm11 / det, i01 = -dm01 / det;
            const double i10 = -dm10 / det, i11 =  dm00 / det;

            StrainTri t;
            t.v[0] = a; t.v[1] = b; t.v[2] = c;
            t.area = area;
            // G_t = D_m^{-T}·S with S = [[-1,1,0],[-1,0,1]]. Row r of
            // D_m^{-T} is COLUMN r of D_m^{-1}; post-multiplying by S makes
            // slot 0 the negated sum of the other two, which is where the
            // element's translation invariance comes from.
            {
                const double inv[2][2] = { { i00, i01 }, { i10, i11 } };
                for (int r = 0; r < 2; ++r) {
                    const double m0 = inv[0][r];   // D_m^{-T}(r, 0)
                    const double m1 = inv[1][r];   // D_m^{-T}(r, 1)
                    t.G[r][0] = -(m0 + m1);
                    t.G[r][1] = m0;
                    t.G[r][2] = m1;
                }
            }
            out.push_back(t);

            // Rest frame F_rest = D_s_rest·D_m^{-1}: a 3x2 with ORTHONORMAL
            // columns (the flattening is an isometry), i.e. both singular
            // values exactly 1, so it is a fixed point of the projection and
            // a legitimate P_t seed / fallback.
            const double ds[3][2] = {
                { (double)e1.x, (double)e2.x },
                { (double)e1.y, (double)e2.y },
                { (double)e1.z, (double)e2.z },
            };
            for (int ax = 0; ax < 3; ++ax) {
                pOut.push_back(ds[ax][0] * i00 + ds[ax][1] * i10);  // r = 0
                pOut.push_back(ds[ax][0] * i01 + ds[ax][1] * i11);  // r = 1
            }
        }
    }

    // Transpose the triangle list into the per-vertex CSR the strain RHS
    // gathers over. Same counting sort as buildIncidence, and there for the
    // same reason: a loop over TRIANGLES would have three threads writing one
    // vertex row.
    static void buildTriIncidence(const std::vector<StrainTri>& tris, Index n,
                                  std::vector<Index>& offsets,
                                  std::vector<TriIncident>& entries) {
        offsets.assign((size_t)n + 1, 0);
        for (const auto& t : tris)
            for (int s = 0; s < 3; ++s) ++offsets[t.v[s] + 1];
        for (Index i = 0; i < n; ++i) offsets[i + 1] += offsets[i];
        entries.resize(tris.size() * 3);
        std::vector<Index> cursor(offsets.begin(), offsets.end() - 1);
        for (size_t ti = 0; ti < tris.size(); ++ti)
            for (int s = 0; s < 3; ++s)
                entries[cursor[tris[ti].v[s]]++] =
                    TriIncident{ (Index)ti, (uint32_t)s };
    }

    // ── Cotangent bending rest cache (design doc §4.1) ───────────────────
    // Build the constant Laplace–Beltrami rows, the mixed Voronoi areas and
    // the rest curvature from the pose the mesh is in RIGHT NOW (the same
    // "rest = live pose at build" convention buildSprings and
    // buildStrainTriangles use), and seed p_i with v_i⁰.
    //
    // Everything is measured on the REST mesh and FROZEN: the paper's
    // near-isometry assumption says the cotangents and areas of a deforming
    // cloth stay close to their rest values, which is what keeps L_i — and
    // therefore the global matrix — constant (design doc §4.1).
    //
    // DETERMINISM: the edge weights are accumulated into a SORTED vector, not
    // a hash map. A map's iteration order would change the summation order of
    // a row's coefficients between runs, and with it the last bits of every
    // matrix entry — self-test noise for no gain.
    static void buildBendRows(const GeneralMesh<METAL, PR>& mesh, Index n,
                              BendCache& bc) {
        bc.clear();
        const PR* pos = mesh.state.x.ptr;
        const Index* facets = mesh.adjacency.facets.ptr;
        if (!pos || !facets || n == 0) return;
        const Index numFacets = (Index)mesh.adjacency.facets.size / 3;

        // Cotangent clamp (design doc §9.2's NaN gate). cot blows up as an
        // angle approaches 0 or π, and a sliver triangle would otherwise put
        // an arbitrarily large coefficient into the matrix. 1e4 is ~6e-3
        // degrees from the limit — far outside anything a usable mesh
        // contains, so it bounds the damage without touching real geometry.
        constexpr double kCotMax  = 1e4;
        // Absolute area floor, same reasoning as buildStrainTriangles's:
        // the row's coefficients scale like 1/A_i, so what has to stay
        // bounded is the ABSOLUTE area.
        constexpr double kAreaEps = 1e-12;

        // (a) cotangent edge weights + mixed Voronoi areas, one pass over the
        // facets. Both are pure per-triangle accumulations.
        std::vector<std::pair<uint64_t, double>> ew;
        ew.reserve((size_t)numFacets * 3);
        bc.area.assign((size_t)n, 0.0);

        for (Index f = 0; f < numFacets; ++f) {
            const Index v[3] = { facets[f*3+0], facets[f*3+1], facets[f*3+2] };
            // Same uninitialised-tail guard buildSprings documents for edges.
            if (v[0] >= n || v[1] >= n || v[2] >= n) continue;
            if (v[0] == v[1] || v[1] == v[2] || v[0] == v[2]) {
                ++bc.degenerate; continue;
            }
            const Vec3 P[3] = { vertexAt(pos, v[0]), vertexAt(pos, v[1]),
                                vertexAt(pos, v[2]) };
            // |u x w| is the SAME 2A at all three corners, so one cross
            // product serves every cotangent of this triangle.
            const double twoA = (double)(P[1] - P[0]).cross(P[2] - P[0]).norm();
            if (!(twoA > kAreaEps)) { ++bc.degenerate; continue; }
            const double triArea = 0.5 * twoA;

            // cot at corner k = (u·w)/|u x w| with u, w the two edges leaving
            // it; len2[k] is the SQUARED length of the edge OPPOSITE k.
            double cot[3], len2[3];
            for (int k = 0; k < 3; ++k) {
                const Vec3 u = P[(k+1)%3] - P[k];
                const Vec3 w = P[(k+2)%3] - P[k];
                const double c = (double)u.dot(w) / twoA;
                cot[k] = std::min(std::max(c, -kCotMax), kCotMax);
                const Vec3 o = P[(k+2)%3] - P[(k+1)%3];
                len2[k] = (double)o.dot(o);
            }
            // c̃_ij = ½(cot α + cot β): each incident triangle contributes ½
            // of the angle it OPPOSES. A boundary edge is seen by one
            // triangle only and therefore keeps ½·cot of the single existing
            // angle — the doc's "use the one angle that exists", spelled as
            // omitting the missing term rather than doubling the present one.
            for (int k = 0; k < 3; ++k) {
                const Index a = v[(k+1)%3], b = v[(k+2)%3];
                const uint64_t key = a < b
                    ? ((uint64_t)a << 32) | (uint32_t)b
                    : ((uint64_t)b << 32) | (uint32_t)a;
                ew.emplace_back(key, 0.5 * cot[k]);
            }

            // Mixed Voronoi area (Meyer et al. 2003): the true Voronoi region
            // where the triangle is non-obtuse, and a plain area split where
            // it is not (the circumcentre leaves the triangle there, so the
            // Voronoi formula would hand a corner a negative area).
            int obtuse = -1;
            for (int k = 0; k < 3; ++k) if (cot[k] < 0.0) obtuse = k;
            if (obtuse >= 0) {
                bc.area[v[obtuse]]       += triArea * 0.5;
                bc.area[v[(obtuse+1)%3]] += triArea * 0.25;
                bc.area[v[(obtuse+2)%3]] += triArea * 0.25;
            } else {
                // A_k += (1/8)(|PQ|²cot R + |PR|²cot Q). With len2 indexed by
                // the OPPOSITE corner, |P_k P_{k+1}|² is len2[k+2] and
                // |P_k P_{k+2}|² is len2[k+1], which is why both terms read
                // the same index as their cotangent.
                for (int k = 0; k < 3; ++k)
                    bc.area[v[k]] += 0.125 * (len2[(k+1)%3] * cot[(k+1)%3]
                                            + len2[(k+2)%3] * cot[(k+2)%3]);
            }
        }

        // (b) merge duplicate edges. Sorting first makes the merge a linear
        // scan AND fixes the accumulation order.
        std::sort(ew.begin(), ew.end(),
                  [](const std::pair<uint64_t, double>& a,
                     const std::pair<uint64_t, double>& b) {
                      return a.first < b.first;
                  });
        size_t uniq = 0;
        for (size_t e = 0; e < ew.size(); ) {
            size_t g = e;
            double acc = 0.0;
            while (g < ew.size() && ew[g].first == ew[e].first) acc += ew[g++].second;
            ew[uniq++] = { ew[e].first, acc };
            e = g;
        }
        ew.resize(uniq);

        // (c) row lengths. A vertex carries a constraint only if its mixed
        // area cleared the floor AND it has at least one one-ring neighbour;
        // everything else gets an EMPTY row and is counted (design doc §4.1).
        std::vector<Index> deg((size_t)n, 0);
        for (const auto& e : ew) {
            ++deg[(Index)(e.first >> 32)];
            ++deg[(Index)(e.first & 0xffffffffull)];
        }
        bc.rowOffsets.assign((size_t)n + 1, 0);
        for (Index i = 0; i < n; ++i) {
            const bool live = (bc.area[i] > kAreaEps) && (deg[i] > 0);
            if (!live) { bc.area[i] = 0.0; ++bc.degenerate; }
            // +1 for the diagonal entry.
            bc.rowOffsets[i + 1] = live ? deg[i] + 1 : Index(0);
        }
        for (Index i = 0; i < n; ++i) bc.rowOffsets[i + 1] += bc.rowOffsets[i];

        // (d) fill the rows: the one-ring entries c̃_ij/A_i first, then the
        // diagonal -Σ_j c̃_ij/A_i in the row's last slot, so the row sums to
        // zero BY CONSTRUCTION rather than by a second accumulation that
        // could round differently.
        bc.rowEntries.resize((size_t)bc.rowOffsets[n]);
        std::vector<Index> cursor(bc.rowOffsets.begin(), bc.rowOffsets.end() - 1);
        for (const auto& e : ew) {
            const Index a = (Index)(e.first >> 32);
            const Index b = (Index)(e.first & 0xffffffffull);
            if (bc.area[a] > 0.0)
                bc.rowEntries[cursor[a]++] = BendEntry{ b, e.second / bc.area[a] };
            if (bc.area[b] > 0.0)
                bc.rowEntries[cursor[b]++] = BendEntry{ a, e.second / bc.area[b] };
        }
        for (Index i = 0; i < n; ++i) {
            if (!(bc.area[i] > 0.0)) continue;
            double s = 0.0;
            for (Index k = bc.rowOffsets[i]; k < cursor[i]; ++k)
                s += bc.rowEntries[k].c;
            bc.rowEntries[cursor[i]] = BendEntry{ i, -s };
        }

        // (e) rest curvature v_i⁰ = L_i·x_rest, its norm, and the p_i seed.
        bc.v0.assign((size_t)n * 3, 0.0);
        bc.v0len.assign((size_t)n, 0.0);
        for (Index i = 0; i < n; ++i) {
            if (!(bc.area[i] > 0.0)) continue;
            double acc[3] = { 0.0, 0.0, 0.0 };
            for (Index k = bc.rowOffsets[i]; k < bc.rowOffsets[i+1]; ++k) {
                const BendEntry& en = bc.rowEntries[k];
                for (int ax = 0; ax < 3; ++ax)
                    acc[ax] += en.c * (double)pos[(size_t)en.col * 3 + ax];
            }
            for (int ax = 0; ax < 3; ++ax) bc.v0[(size_t)i*3 + ax] = acc[ax];
            bc.v0len[i] = std::sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        }
        bc.p = bc.v0;

        // Mean mixed area over the LIVE rows — the kbend -> w_b conversion,
        // see BendCache::areaScale.
        {
            double acc = 0.0; Index live = 0;
            for (Index i = 0; i < n; ++i)
                if (bc.area[i] > 0.0) { acc += bc.area[i]; ++live; }
            bc.areaScale = live ? acc / (double)live : 0.0;
        }

        // (f) transpose incidence, counting sort over the rows in index order
        // (so this is deterministic for the same reason (b) is).
        bc.tOffsets.assign((size_t)n + 1, 0);
        for (const BendEntry& en : bc.rowEntries) ++bc.tOffsets[en.col + 1];
        for (Index i = 0; i < n; ++i) bc.tOffsets[i + 1] += bc.tOffsets[i];
        bc.tEntries.resize(bc.rowEntries.size());
        std::vector<Index> tcur(bc.tOffsets.begin(), bc.tOffsets.end() - 1);
        for (Index i = 0; i < n; ++i)
            for (Index k = bc.rowOffsets[i]; k < bc.rowOffsets[i+1]; ++k) {
                const BendEntry& en = bc.rowEntries[k];
                bc.tEntries[tcur[en.col]++] = BendTIncident{ i, en.c };
            }
    }

    // ── Local projection of one bending element (design doc §4.2) ────────
    // p_i = ‖v_i⁰‖·v_i/‖v_i‖ — the closest point of the SPHERE of radius
    // ‖v_i⁰‖ to the current curvature vector v_i = L_i q. Writes `p` only
    // when the direction is well conditioned; otherwise the slot is left
    // alone, which IS §4.2's deterministic fallback chain (see BendCache::p).
    //
    // The threshold is absolute AND relative: the absolute floor keeps a
    // genuinely flat patch (‖v_i‖ ~ roundoff) from being normalized at all,
    // and the relative one keeps a nearly-flattened patch of a CURVED rest
    // mesh from having its roundoff-direction amplified to the full rest
    // length ‖v_i⁰‖.
    static void projectBend(const double v[3], double v0len, double p[3]) {
        const double len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (!std::isfinite(len)) return;
        if (!(len > std::max(1e-12, 1e-6 * v0len))) return;
        const double s = v0len / len;
        p[0] = v[0] * s; p[1] = v[1] * s; p[2] = v[2] * s;
    }

    // v_i = L_i·pos of one cached row, at the flat 3N position array `pos`.
    // Shared by the diagnostics below; the local step has its own copy of the
    // same three lines because its iterate is an Eigen matrix, not a flat
    // array (exactly the split strainF / LOCAL pass 3 already have).
    template <typename T>
    static void bendV(const BendCache& bc, Index i, const T* pos, double v[3]) {
        v[0] = v[1] = v[2] = 0.0;
        for (Index k = bc.rowOffsets[i]; k < bc.rowOffsets[i+1]; ++k) {
            const BendEntry& en = bc.rowEntries[k];
            for (int ax = 0; ax < 3; ++ax)
                v[ax] += en.c * (double)pos[(size_t)en.col * 3 + ax];
        }
    }

    // ── Local projection of one strain element (design doc §3.3) ─────────
    // thin SVD F = UΣVᵀ (U 3x2, Σ 2x2, V 2x2), singular values clamped into
    // [sigMin, sigMax], P = U·clamp(Σ)·Vᵀ. With sigMin == sigMax == 1 this is
    // the corotated projection P = UVᵀ; with a band it is strain limiting.
    //
    // Both F and P are the ROW-MAJOR 3x2 layout MeshCache::strainP uses:
    // m[a*2 + r]. Returns false and leaves `P` UNTOUCHED when F is not finite
    // or the SVD did not converge — the caller's slot then still holds the
    // previous iteration's P_t (or, on the first iteration, the rest frame),
    // which is the "never let NaN reach the global RHS" rule of §3.3.
    static bool projectStrain(const double F[6], double sigMin, double sigMax,
                              double P[6]) {
        Eigen::Matrix<double, 3, 2> Fm;
        for (int ax = 0; ax < 3; ++ax)
            for (int r = 0; r < 2; ++r) Fm(ax, r) = F[ax * 2 + r];
        if (!Fm.allFinite()) return false;
        // Options as a TEMPLATE parameter, not a runtime flag: for a
        // fixed-size 3x2 Eigen only permits thin U statically (the runtime
        // path asserts cols >= rows). Fixed size keeps this allocation-free,
        // which matters — this runs once per triangle per iteration.
        Eigen::JacobiSVD<Eigen::Matrix<double, 3, 2>,
                         Eigen::ComputeThinU | Eigen::ComputeThinV> svd(Fm);
        if (svd.info() != Eigen::Success) return false;
        Eigen::Vector2d s = svd.singularValues();
        for (int r = 0; r < 2; ++r)
            s[r] = std::min(std::max(s[r], sigMin), sigMax);
        const Eigen::Matrix<double, 3, 2> Pm =
            svd.matrixU() * s.asDiagonal() * svd.matrixV().transpose();
        if (!Pm.allFinite()) return false;
        for (int ax = 0; ax < 3; ++ax)
            for (int r = 0; r < 2; ++r) P[ax * 2 + r] = Pm(ax, r);
        return true;
    }

    // F_t of one cached triangle at the flat 3N position array `pos`, in the
    // same row-major 3x2 layout. F(a, r) = Σ_s G(r, s)·pos(v_s, a) — the
    // F_tᵀ = G_t·Q_t identity spelled out per component (see StrainTri).
    template <typename T>
    static void strainF(const StrainTri& t, const T* pos, double F[6]) {
        for (int ax = 0; ax < 3; ++ax) {
            const double p0 = (double)pos[(size_t)t.v[0] * 3 + ax];
            const double p1 = (double)pos[(size_t)t.v[1] * 3 + ax];
            const double p2 = (double)pos[(size_t)t.v[2] * 3 + ax];
            F[ax*2+0] = t.G[0][0]*p0 + t.G[0][1]*p1 + t.G[0][2]*p2;
            F[ax*2+1] = t.G[1][0]*p0 + t.G[1][1]*p1 + t.G[1][2]*p2;
        }
    }

    // ── Diagnostics (self-tests §9.1, GUI probes) ────────────────────────
    // Total triangle strain energy Σ_t (w_s·A_t/2)|F_t - P_t|² of the CACHED
    // mesh `mi`, evaluated at the flat 3N position array `pos` with P_t taken
    // from the SAME projectStrain the local step runs (so a bug in the
    // projection cannot hide behind a second implementation here). Zero for a
    // mesh with no strain cache — FastGridCloth, or a mesh never stepped.
    double strainEnergyAt(Index mi, const PR* pos) const {
        if ((size_t)mi >= cache.size() || !pos) return 0.0;
        const MeshCache& mc = cache[(size_t)mi];
        const double ws = mc.kS;
        double E = 0.0;
        for (const StrainTri& t : mc.strainTris) {
            double F[6], P[6];
            strainF(t, pos, F);
            // Seeded with F so a failed projection reports ZERO energy for
            // that triangle rather than a garbage one; the solver's own
            // fallback (keep the previous P_t) is not reproducible from here.
            for (int z = 0; z < 6; ++z) P[z] = F[z];
            projectStrain(F, mc.sigMin, mc.sigMax, P);
            double acc = 0.0;
            for (int z = 0; z < 6; ++z) {
                const double d = F[z] - P[z];
                acc += d * d;
            }
            E += 0.5 * ws * t.area * acc;
        }
        return E;
    }
    // Total bending energy Σ_i (w_b·A_i/2)‖L_i q - p_i‖² of the CACHED mesh
    // `mi` at the flat 3N position array `pos`, with p_i taken from the SAME
    // projectBend the local step runs (so the diagnostic cannot disagree with
    // the solve). Zero for a mesh with no bending cache. The p_i used here is
    // seeded from the CURRENT v_i rather than from the solver's live slot:
    // a probe must not depend on how many iterations the solver happens to
    // have run, and where the projection is well conditioned the two agree
    // exactly anyway.
    double bendEnergyAt(Index mi, const PR* pos) const {
        if ((size_t)mi >= cache.size() || !pos) return 0.0;
        const MeshCache& mc = cache[(size_t)mi];
        const BendCache& bc = mc.bend;
        const double wb = mc.kB * bc.areaScale;
        double E = 0.0;
        for (Index i = 0; i < (Index)bc.area.size(); ++i) {
            const double w = wb * bc.area[i];
            if (!(w > 0.0)) continue;
            double v[3], p[3];
            bendV(bc, i, pos, v);
            p[0] = v[0]; p[1] = v[1]; p[2] = v[2];
            projectBend(v, bc.v0len[i], p);
            double acc = 0.0;
            for (int ax = 0; ax < 3; ++ax) {
                const double d = v[ax] - p[ax];
                acc += d * d;
            }
            E += 0.5 * w * acc;
        }
        return E;
    }
    // Largest ‖v_i⁰‖ over the cached rows — the rest curvature the bending
    // element is holding the mesh to. Zero for a flat rest pose, which is
    // what PD-12's flat clause reads.
    double bendRestCurvatureMax(Index mi) const {
        if ((size_t)mi >= cache.size()) return 0.0;
        double hi = 0.0;
        for (double c : cache[(size_t)mi].bend.v0len) hi = std::max(hi, c);
        return hi;
    }
    // Σ_t A_t of the cached mesh — the rest area the strain elements cover,
    // which is what turns a known uniform stretch into a closed-form energy.
    double strainRestArea(Index mi) const {
        if ((size_t)mi >= cache.size()) return 0.0;
        double A = 0.0;
        for (const StrainTri& t : cache[(size_t)mi].strainTris) A += t.area;
        return A;
    }

    // ── (1b-e) EDGE-EDGE contact detection + resolve (design doc §5.5) ───
    // Called from step() right after the vertex-triangle resolve, against the
    // SAME warm-started iterate, so both families freeze their interpolation
    // coefficients at one pose (§5.6) and both land in `contactRows` before
    // the epoch key is built.
    //
    // WHY THIS PASS EXISTS AT ALL. The narrow phase this solver reads is
    // vertex-triangle only, end to end: the CPU broad phase emits (vertex,
    // face) rows and the GPU narrow kernel refines them. Two sheets crossing
    // edge-past-edge therefore produce NO row of any kind, at any thickness,
    // and no contact stiffness fixes that — design doc §6.2's last bullet
    // ("vertex–triangle만 있고 edge–edge 검출이 없음") names it as one of the
    // failures stiffness cannot address. So the constraint needs a detection
    // source of its own.
    //
    // WHY A LOCAL GRID AND NOT CpuSpatialHash. That class IS a CPU uniform
    // grid and its cellKey/generation-stamping idiom is reused here verbatim,
    // but its instance cannot be: `faces`/`grid`/`seen` are shared members
    // built by collectFaceBoxes (hardcoded to triangles: it walks
    // adjacency.facets and stores a FaceRec per face) and consumed by
    // probeMeshVertices (hardcoded to points: it walks vertices and emits
    // indexPair.point/.triangle). A second pass with a different item type
    // cannot run on the same instance without a template refactor of a class
    // the live self-collision path depends on. The grid this pass needs is
    // ~40 lines; refactoring that one to serve both is a bigger change with a
    // bigger blast radius, for a pass whose cost has not been measured yet.
    //
    // DETECTION IS PROXIMITY, NOT SWEPT. §5.5 says swept edge-edge or CCD is
    // "별도로 필요하다" — separately needed — and it still is: a pair that
    // crosses completely inside one substep is missed here exactly as it is
    // missed by the vertex-triangle path. What this pass guarantees is the
    // pair that is WITHIN THICKNESS at the substep's warm-started iterate.
    void resolveEdgeContacts(Scene<METAL, PR>& sceneObjects, double invH2) {
        edgeEdgeContactCount = 0;
        if (!edgeContactsEnabled || ctxs.empty()) return;

        // ---- which cloths take part, and their AABBs at the iterate.
        // TriangularCloth ONLY: FastGridCloth runs the legacy spring model
        // with its GPU twin as the source of truth (see the supported
        // combination table at the top), and it has no edge cache built.
        struct Part { PR lo[3], hi[3]; bool self = false; };
        std::vector<int>  partIdx;      // ctxs indices taking part
        std::vector<Part> parts;        // parallel to partIdx
        partIdx.reserve(ctxs.size());
        parts.reserve(ctxs.size());
        for (size_t ci = 0; ci < ctxs.size(); ++ci) {
            const SolveCtx& c = ctxs[ci];
            if (!c.mc || c.mc->edgeV.empty()) continue;
            const auto& mesh = sceneObjects.meshes[c.mi];
            if (mesh.behaviorType != BehaviorType::TriangularCloth) continue;
            if (!mesh.collidable) continue;
            Part p;
            p.self = mesh.selfCollide;
            const Eigen::MatrixXd& q = c.mc->q;
            for (int a = 0; a < 3; ++a) {
                p.lo[a] =  std::numeric_limits<PR>::max();
                p.hi[a] = -std::numeric_limits<PR>::max();
            }
            bool finite = true;
            for (Index i = 0; i < c.n && finite; ++i)
                for (int a = 0; a < 3; ++a) {
                    const double v = q((int)i, a);
                    if (!std::isfinite(v)) { finite = false; break; }
                    p.lo[a] = std::min(p.lo[a], (PR)v);
                    p.hi[a] = std::max(p.hi[a], (PR)v);
                }
            // A blown-up mesh has no meaningful box; leaving it out costs it
            // its edge rows for this substep, which is the same thing the
            // sanitizer in step (4) does with its positions.
            if (!finite) continue;
            // Inflate by this cloth's own thickness, so the pair test below is
            // a conservative superset of "some edge pair is within max(d)".
            for (int a = 0; a < 3; ++a) {
                p.lo[a] -= c.thickness;
                p.hi[a] += c.thickness;
            }
            partIdx.push_back((int)ci);
            parts.push_back(p);
        }
        if (parts.empty()) return;

        // ---- live PAIRS. A self pair needs the per-object 자기충돌 gate
        // (GeneralMesh::selfCollide), the same gate the vertex-triangle self
        // path is driven by; a cross pair needs only that the two boxes touch.
        // No overlap => the pass never looks at those meshes' edges at all,
        // which is the mesh-level early-out that keeps a scene of distant
        // cloths at the cost of one box test per pair.
        const size_t np = parts.size();
        std::vector<uint8_t> pairLive(np * np, 0);
        bool anyPair = false;
        for (size_t i = 0; i < np; ++i)
            for (size_t j = i; j < np; ++j) {
                bool live;
                if (i == j) {
                    live = parts[i].self;
                } else {
                    live = true;
                    for (int a = 0; a < 3 && live; ++a)
                        if (parts[i].hi[a] < parts[j].lo[a]
                            || parts[j].hi[a] < parts[i].lo[a]) live = false;
                }
                if (!live) continue;
                pairLive[i * np + j] = 1;
                pairLive[j * np + i] = 1;
                anyPair = true;
            }
        if (!anyPair) return;
        // ctxs index -> parts index, so the inner loops can test pairLive
        // without searching partIdx.
        std::vector<int> partOfCtx(ctxs.size(), -1);
        for (size_t k = 0; k < partIdx.size(); ++k)
            partOfCtx[(size_t)partIdx[k]] = (int)k;

        // ---- edge AABBs at the iterate. A mesh with no live pair at all is
        // skipped here, not just at query time, so it costs nothing.
        edgeBoxes.clear();
        double maxExtent = 0.0;
        for (size_t k = 0; k < np; ++k) {
            bool any = false;
            for (size_t j = 0; j < np && !any; ++j)
                if (pairLive[k * np + j]) any = true;
            if (!any) continue;
            const SolveCtx& c = ctxs[(size_t)partIdx[k]];
            const Eigen::MatrixXd& q = c.mc->q;
            const std::vector<Index>& ev = c.mc->edgeV;
            const PR margin = c.thickness;
            for (size_t e = 0; e < ev.size() / 2; ++e) {
                EdgeBox eb;
                eb.cx = partIdx[k];
                eb.e  = (Index)e;
                eb.a  = ev[e*2];
                eb.b  = ev[e*2+1];
                bool ok = true;
                for (int a = 0; a < 3; ++a) {
                    const double va = q((int)eb.a, a), vb = q((int)eb.b, a);
                    if (!std::isfinite(va) || !std::isfinite(vb)) { ok = false; break; }
                    eb.lo[a] = (PR)std::min(va, vb) - margin;
                    eb.hi[a] = (PR)std::max(va, vb) + margin;
                    maxExtent = std::max(maxExtent,
                                         (double)(eb.hi[a] - eb.lo[a]));
                }
                if (ok) edgeBoxes.push_back(eb);
            }
        }
        if (edgeBoxes.size() < 2) return;

        // ---- grid. Cell size ~ the largest inflated edge box, so one edge
        // spans at most 2 cells per axis and the insert stays O(edges).
        const double cs = std::max(maxExtent, 1e-6);
        const double invCs = 1.0 / cs;
        ++edgeGridGen;
        const uint32_t gen = edgeGridGen;
        auto cellRange = [&](const EdgeBox& eb, int lo[3], int hi[3]) {
            for (int a = 0; a < 3; ++a) {
                lo[a] = (int)std::floor((double)eb.lo[a] * invCs);
                hi[a] = (int)std::floor((double)eb.hi[a] * invCs);
                // A box that somehow spans a huge number of cells (a
                // non-finite slipped through, an absurd thickness) would turn
                // the insert quadratic; clamp the span rather than hang.
                if (hi[a] - lo[a] > 8) hi[a] = lo[a] + 8;
            }
        };
        for (uint32_t ei = 0; ei < (uint32_t)edgeBoxes.size(); ++ei) {
            int lo[3], hi[3];
            cellRange(edgeBoxes[ei], lo, hi);
            for (int z = lo[2]; z <= hi[2]; ++z)
                for (int y = lo[1]; y <= hi[1]; ++y)
                    for (int x = lo[0]; x <= hi[0]; ++x) {
                        EdgeCell& cell = edgeGrid[edgeCellKey(x, y, z)];
                        if (cell.gen != gen) { cell.gen = gen; cell.items.clear(); }
                        cell.items.push_back(ei);
                    }
        }

        // ---- narrow test. The outer loop is ASCENDING and each candidate
        // list is sorted and deduplicated, so the row order this produces is a
        // deterministic function of the geometry and NOT of the hash map's
        // bucket order. That matters directly: the rows go into the epoch key,
        // and a key that permuted itself every substep would refactor the
        // merged system every substep for a contact set that never changed.
        const size_t rowsBefore = contactRows.size();
        for (uint32_t ei = 0; ei < (uint32_t)edgeBoxes.size(); ++ei) {
            const EdgeBox& A = edgeBoxes[ei];
            const int pa = partOfCtx[(size_t)A.cx];
            edgeCandidates.clear();
            int lo[3], hi[3];
            cellRange(A, lo, hi);
            for (int z = lo[2]; z <= hi[2]; ++z)
                for (int y = lo[1]; y <= hi[1]; ++y)
                    for (int x = lo[0]; x <= hi[0]; ++x) {
                        auto it = edgeGrid.find(edgeCellKey(x, y, z));
                        if (it == edgeGrid.end() || it->second.gen != gen)
                            continue;
                        for (uint32_t ej : it->second.items)
                            if (ej > ei) edgeCandidates.push_back(ej);
                    }
            std::sort(edgeCandidates.begin(), edgeCandidates.end());
            edgeCandidates.erase(std::unique(edgeCandidates.begin(),
                                             edgeCandidates.end()),
                                 edgeCandidates.end());

            for (uint32_t ej : edgeCandidates) {
                const EdgeBox& B = edgeBoxes[ej];
                const int pb = partOfCtx[(size_t)B.cx];
                if (pa < 0 || pb < 0 || !pairLive[(size_t)pa * np + (size_t)pb])
                    continue;
                // Exact box test — the grid only says "same cell".
                bool sep = false;
                for (int a = 0; a < 3 && !sep; ++a)
                    if (A.hi[a] < B.lo[a] || B.hi[a] < A.lo[a]) sep = true;
                if (sep) continue;

                const SolveCtx& ca = ctxs[(size_t)A.cx];
                const SolveCtx& cb = ctxs[(size_t)B.cx];
                if (A.cx == B.cx) {
                    // SELF pair admissibility (design doc §5.4). No shared
                    // vertex, and no endpoint in the other's one-ring — which
                    // between them reject every pair of edges of the same
                    // triangle and of two vertex-adjacent triangles. See
                    // MeshCache::edgeV for why this is the filter.
                    if (A.a == B.a || A.a == B.b || A.b == B.a || A.b == B.b)
                        continue;
                    const MeshCache& mc = *ca.mc;
                    if (inOneRing(mc, A.a, B.a) || inOneRing(mc, A.a, B.b)
                        || inOneRing(mc, A.b, B.a) || inOneRing(mc, A.b, B.b))
                        continue;
                }

                const Vec3 p1 = qVecOf(ca, A.a), p2 = qVecOf(ca, A.b);
                const Vec3 p3 = qVecOf(cb, B.a), p4 = qVecOf(cb, B.b);
                PR s, t;
                if (!closestSegSeg(p1, p2, p3, p4, s, t)) continue;
                // r_c = A_c q at the detection pose, by construction.
                const Vec3 r = (p1 * (PR(1) - s) + p2 * s)
                             - (p3 * (PR(1) - t) + p4 * t);
                const PR dist = r.norm();
                const PR thk = std::max(ca.thickness, cb.thickness);
                if (!(dist < thk)) continue;

                ContactRow cr;
                cr.kind = RowKind::EdgeEdge;
                cr.cx[0] = cr.cx[1] = A.cx;
                cr.cx[2] = cr.cx[3] = B.cx;
                cr.vi[0] = A.a; cr.vi[1] = A.b;
                cr.vi[2] = B.a; cr.vi[3] = B.b;
                cr.alpha[0] = (double)(PR(1) - s);
                cr.alpha[1] = (double)s;
                cr.alpha[2] = -(double)(PR(1) - t);
                cr.alpha[3] = -(double)t;
                cr.s = s; cr.t = t;
                cr.cross = (A.cx != B.cx);
                cr.iw[0] = invMassOf(ca.m, ca.mask, A.a);
                cr.iw[1] = invMassOf(ca.m, ca.mask, A.b);
                cr.iw[2] = invMassOf(cb.m, cb.mask, B.a);
                cr.iw[3] = invMassOf(cb.m, cb.mask, B.b);
                // "모든 참여 정점의 inverse mass가 0이면 constraint를 만들지
                // 않는다" (§6.1) — the same clause the §5.3 rows obey.
                if (cr.iw[0] <= PR(0) && cr.iw[1] <= PR(0)
                    && cr.iw[2] <= PR(0) && cr.iw[3] <= PR(0)) continue;

                // ENTRY SIDE. n₀ is the geometric edge-cross normal, oriented
                // so that nᵀr >= 0 at DETECTION — the side the two edges are
                // on relative to each other right now, before the solve has
                // corrected anything. Frozen for the epoch as `nrm`, exactly
                // the ContactRow convention the vertex-triangle rows use for
                // the narrow phase's row normal.
                const Vec3 cross = (p2 - p1).cross(p4 - p3);
                const PR clen = cross.norm();
                if (clen >= PR(1e-9)) {
                    const Vec3 nh = cross / clen;
                    cr.nrm = nh * ((nh.dot(r) >= PR(0)) ? PR(1) : PR(-1));
                    cr.frozenNormal = false;
                } else {
                    // NEAR PARALLEL: the cross product carries no direction.
                    // Fall back to the separation direction itself and FREEZE
                    // it for the whole epoch (see ContactRow::frozenNormal) —
                    // deterministic, and it is the direction the pair is
                    // actually separated along. With no separation either
                    // there is nothing to build a half-space from at all.
                    if (!(dist > PR(1e-9))) continue;
                    cr.nrm = r / dist;
                    cr.frozenNormal = true;
                }

                double wden = 0.0;
                for (int a = 0; a < 4; ++a)
                    wden += cr.alpha[a] * cr.alpha[a] * (double)cr.iw[a];
                // m_eff = 1/Σ α_a² w_a^m — the natural extension of §6.1's
                // formula, whose denominator is A_c's inverse-mass quadratic
                // form; for α = [1, -β] it IS §6.1 verbatim. A vanishing form
                // means every vertex A_c actually weighs is pinned, so the row
                // could move nothing.
                if (!(wden > 1e-12)) continue;
                cr.mEff = 1.0 / wden;
                cr.w    = kContactWeightScale * cr.mEff * invH2;
                cr.thk  = thk;
                cr.col[0] = (int)(ca.solveBase + A.a);
                cr.col[1] = (int)(ca.solveBase + A.b);
                cr.col[2] = (int)(cb.solveBase + B.a);
                cr.col[3] = (int)(cb.solveBase + B.b);
                contactRows.push_back(cr);
            }
        }

        // ---- CAP. An edge pass is quadratic in the worst case (two sheets
        // laid flat on each other put every edge within thickness of several
        // others), and every row it keeps is 16 matrix entries plus a
        // factorization the epoch has to pay for. The budget is 4 rows per
        // vertex of the MERGED system.
        //
        // DEVIATION from "per mesh pair", stated rather than hidden: the
        // budget is GLOBAL. There is one merged system and one factorization,
        // so the quantity that actually blows up is the TOTAL row count, and a
        // per-pair budget would let k pairs cost k times the cap while each
        // one looked compliant. Rows are kept DEEPEST FIRST, so what a cap hit
        // discards is the shallowest proximities — the ones whose unilateral
        // projection is closest to the identity anyway — and the count of them
        // is published (droppedEdgeRowCount) instead of the truncation being
        // silent.
        const size_t cap = (size_t)nTotal * 4;
        size_t rowsNow = contactRows.size() - rowsBefore;
        if (rowsNow > cap) {
            // Depth of a row at the detection pose = how far inside the
            // thickness the pair already is. STABLE sort, so equal depths keep
            // the deterministic detection order the loop above produced.
            std::vector<uint32_t> order(rowsNow);
            for (size_t k = 0; k < rowsNow; ++k) order[k] = (uint32_t)k;
            std::vector<double> depth(rowsNow);
            for (size_t k = 0; k < rowsNow; ++k) {
                const ContactRow& cr = contactRows[rowsBefore + k];
                Vec3 r;
                for (int a = 0; a < 4; ++a)
                    r += qVecOf(ctxs[(size_t)cr.cx[a]], cr.vi[a])
                       * (PR)cr.alpha[a];
                depth[k] = (double)(cr.thk - cr.nrm.dot(r));
            }
            std::stable_sort(order.begin(), order.end(),
                [&](uint32_t x, uint32_t y) { return depth[x] > depth[y]; });
            std::vector<ContactRow> kept;
            kept.reserve(cap);
            for (size_t k = 0; k < cap; ++k)
                kept.push_back(contactRows[rowsBefore + order[k]]);
            // Restore the deterministic detection order among the survivors,
            // for the same epoch-key reason the detection loop is ordered.
            std::sort(kept.begin(), kept.end(),
                [](const ContactRow& x, const ContactRow& y) {
                    for (int a = 0; a < 4; ++a) {
                        if (x.cx[a] != y.cx[a]) return x.cx[a] < y.cx[a];
                        if (x.vi[a] != y.vi[a]) return x.vi[a] < y.vi[a];
                    }
                    return false;
                });
            contactRows.resize(rowsBefore);
            for (const ContactRow& k : kept) contactRows.push_back(k);
            droppedEdgeRowCount += (uint32_t)(rowsNow - cap);
            rowsNow = cap;
        }
        edgeEdgeContactCount = (uint32_t)rowsNow;
    }

    // Read one vertex of a context's live iterate. Same body as step()'s local
    // `qVec` lambda; a static so resolveEdgeContacts can use it too rather
    // than the two growing their own copies.
    static Vec3 qVecOf(const SolveCtx& c, Index i) {
        return Vec3((PR)c.mc->q((int)i, 0),
                    (PR)c.mc->q((int)i, 1),
                    (PR)c.mc->q((int)i, 2));
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
        edgeEdgeContactCount = 0;
        rigidCoupleCount     = 0;
        contactsLive         = true;
        // NOT reset here: the penetration accumulators are frame-scoped, and
        // step() is one substep. beginFrameContactStats() owns their window.
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
        // Set by any mesh whose contact-free BLOCK was rebuilt below; the
        // merged base is re-assembled from the blocks when it is.
        bool anyBaseRebuilt = false;
        // Running column offset of the merged system, handed to each ctx as
        // its solveBase in the order the cloths are accepted.
        Index nTotalNew = 0;

        // --- (1) per mesh: cache/rebuild the mesh's contact-free BLOCK of the
        // merged matrix, build the momentum target and warm start. Every
        // active cloth must reach its warm-started iterate before ANY contact
        // is resolved: the coupled rows freeze their barycentrics at that
        // iterate (§5.6), so a partner still sitting at x would freeze β
        // against stale geometry — the same predict-all-first rule
        // PbdSystem::step (1) documents.
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
                // TriangularCloth's whole material model is the two continuum
                // elements below, so it builds NO springs at all — neither
                // the edge stretch ones (design doc §3.5) nor the
                // opposite-vertex bend ones (§4). FastGridCloth keeps the
                // legacy spring model, unchanged.
                const bool strainCloth =
                    (mesh.behaviorType == BehaviorType::TriangularCloth);
                if (strainCloth) mc.springs.clear();
                else             buildSprings(mesh, n, mc.springs);
                // The gather adjacency is a function of the spring list only,
                // so it is rebuilt exactly when the springs are. Stiffness
                // edits refactor the MATRIX but never reach here — and must
                // not, for the same reason `rest` must not be re-measured.
                buildIncidence(mc.springs, n, mc.adjOffsets, mc.adjEntries);
                mc.springD.assign(mc.springs.size() * 3, 0.0);
                // Same trigger, same reason: the rest operators are measured
                // from the live pose, so they may only be re-sampled when the
                // mesh identity or its counts actually changed.
                if (strainCloth) {
                    buildStrainTriangles(mesh, n, mc.strainTris, mc.strainP,
                                         mc.degenerateTriCount);
                    buildBendRows(mesh, n, mc.bend);
                    // Edge-edge contact source (design doc §5.5).
                    // TriangularCloth ONLY — FastGridCloth keeps the legacy
                    // model end to end and must not start paying for a
                    // detection pass it has no constraint family for.
                    buildEdgeList(mesh, n, mc.edgeV, mc.ringOffsets,
                                  mc.ringEntries);
                } else {
                    mc.strainTris.clear();
                    mc.strainP.clear();
                    mc.degenerateTriCount = 0;
                    mc.bend.clear();
                    mc.edgeV.clear();
                    mc.ringOffsets.clear();
                    mc.ringEntries.clear();
                }
                buildTriIncidence(mc.strainTris, n, mc.triOffsets,
                                  mc.triEntries);
                mc.lifetimeId = mesh.lifetimeId;
                mc.numPoints  = n;
                mc.numFacets  = numFacets;
                mc.numEdges   = numEdges;
                refactor = true;
            }

            // (b) Matrix scalars + the two per-vertex arrays that enter it.
            // kS covers BOTH in-plane models: it is the spring constant of
            // the FastGridCloth Laplacian AND the strain weight w_s of the
            // TriangularCloth blocks, both of which sit in Abase, so this one
            // comparison is the whole w_s invalidation. The strain BAND
            // (MeshCache::sigMin/sigMax, from ClothBehaviorParams)
            // deliberately does NOT appear here — it only moves P_t, i.e.
            // the RHS, and refactoring on it would be pure cost.
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
                //   L:    graph Laplacian of the spring set, weight k, PLUS
                //         the continuum strain blocks w_s·A_t·G_tᵀG_t of the
                //         triangle elements (design doc §3.4) — 3x3 dense
                //         per triangle instead of 2x2 per edge, but the SAME
                //         scalar system: G_t's coefficients are common to the
                //         three spatial axes, so one factorization still
                //         serves all three RHS columns.
                //   wPin: soft pin, see kPinWeight.
                std::vector<Eigen::Triplet<double>> trip;
                // The bending term is the outer product of a row over the
                // one-ring+self, so it costs (deg+1)² triplets per vertex —
                // ~64 on a regular grid, where the average row is 7 long.
                // Reserved from the actual row lengths rather than guessed.
                size_t bendTrip = 0;
                for (Index i = 0; i < (Index)mc.bend.area.size(); ++i) {
                    const size_t len = (size_t)(mc.bend.rowOffsets[i+1]
                                              - mc.bend.rowOffsets[i]);
                    bendTrip += len * len;
                }
                trip.reserve((size_t)n + mc.springs.size() * 4
                             + mc.strainTris.size() * 9 + bendTrip);
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
                // K += w_s·A_t·G_tᵀG_t, one dense 3x3 block on (i, j, k).
                // Symmetric and PSD by construction (it is a Gram matrix),
                // with zero row sums — the element resists deformation, not
                // translation.
                for (const StrainTri& t : mc.strainTris) {
                    const double w = kS * t.area;
                    if (!(w > 0.0)) continue;
                    for (int r = 0; r < 3; ++r)
                        for (int s = 0; s < 3; ++s)
                            trip.emplace_back((int)t.v[r], (int)t.v[s],
                                w * (t.G[0][r]*t.G[0][s] + t.G[1][r]*t.G[1][s]));
                }
                // K += w_b·A_i·L_iᵀL_i (design doc §4.3), assembled as the
                // OUTER PRODUCT of the row — symmetric and PSD by
                // construction, never symmetrized after the fact. Row i spans
                // the one-ring of i, so L_iᵀL_i couples the TWO-ring and the
                // pattern is wider than the spring graph's; it is built once
                // per topology into Abase, and a contact epoch still only
                // touches diagonal entries the mass term already created, so
                // the symbolic analysis is unaffected.
                {
                    const BendCache& bc = mc.bend;
                    for (Index i = 0; i < (Index)bc.area.size(); ++i) {
                        const double w = kB * bc.areaScale * bc.area[i];
                        if (!(w > 0.0)) continue;
                        const Index b0 = bc.rowOffsets[i];
                        const Index b1 = bc.rowOffsets[i+1];
                        for (Index r = b0; r < b1; ++r) {
                            const BendEntry& er = bc.rowEntries[r];
                            for (Index s = b0; s < b1; ++s) {
                                const BendEntry& es = bc.rowEntries[s];
                                trip.emplace_back((int)er.col, (int)es.col,
                                                  w * er.c * es.c);
                            }
                        }
                    }
                }
                mc.Abase.resize((int)n, (int)n);
                // setFromTriplets SUMS duplicates, which is exactly the
                // accumulate-per-incident-spring assembly above.
                mc.Abase.setFromTriplets(trip.begin(), trip.end());
                mc.factor = std::make_unique<
                    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>();
                // VALIDITY GATE ONLY — this factor never solves (see
                // MeshCache::factor). It runs once per base rebuild so a mesh
                // whose own block is not SPD is dropped from the merge here,
                // instead of failing the whole scene's merged factorization
                // and dropping everybody's contacts with it.
                mc.factor->analyzePattern(mc.Abase);
                mc.factor->factorize(mc.Abase);
                mc.valid = (mc.factor->info() == Eigen::Success);
                // The contact weights are m_i/h²-proportional, so a base
                // rebuild (which is exactly what a change of h, mass, pin
                // mask or topology triggers) has invalidated them too. The
                // merged base is re-assembled for the same reason.
                mc.epochW.clear();
                anyBaseRebuilt = true;
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
            // Live per-mesh material band — every substep, no refactor (see
            // MeshCache::sigMin).
            strainSigmaOf(mesh.behaviorParams, mc.sigMin, mc.sigMax);
            c.kS = kS;
            c.kB = kB;
            c.facets    = mesh.adjacency.facets.ptr;
            c.numFacets = numFacets;
            c.mc = &mc;
            c.contactsForMesh = haveContacts && mi + 1 < (Index)off.size;
            c.colBase = c.contactsForMesh ? off.ptr[mi] : Index(0);
            c.solveBase = nTotalNew;
            nTotalNew += n;
            ctxOfMesh[mi] = (int)ctxs.size();
            ctxs.push_back(c);
        }
        if (ctxs.empty()) return;

        // --- (1a) the MERGED base matrix (design doc §5.4). Kbase is the
        // block diagonal of the active cloths' contact-free blocks, in ctx
        // order, with mesh k's block placed at (solveBase_k, solveBase_k).
        // Re-assembled only when the column layout moved or some mesh rebuilt
        // its own block — never per substep and never per contact epoch.
        nTotal = nTotalNew;
        {
            std::vector<Index> layout;
            layout.reserve(ctxs.size() * 2);
            for (const SolveCtx& c : ctxs) { layout.push_back(c.mi);
                                             layout.push_back(c.n); }
            if (anyBaseRebuilt || layout != baseLayout) {
                baseLayout.swap(layout);
                size_t nnz = 0;
                for (const SolveCtx& c : ctxs) nnz += (size_t)c.mc->Abase.nonZeros();
                std::vector<Eigen::Triplet<double>> trip;
                trip.reserve(nnz);
                for (const SolveCtx& c : ctxs) {
                    const int b = (int)c.solveBase;
                    const Eigen::SparseMatrix<double>& A = c.mc->Abase;
                    for (int k = 0; k < A.outerSize(); ++k)
                        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k);
                             it; ++it)
                            trip.emplace_back(b + (int)it.row(),
                                              b + (int)it.col(), it.value());
                }
                Kbase.resize((int)nTotal, (int)nTotal);
                Kbase.setFromTriplets(trip.begin(), trip.end());
                if (!globFactor)
                    globFactor = std::make_unique<
                        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>>>();
                // A base rebuild invalidates the epoch outright: every
                // contact weight is m/h²-proportional and every column may
                // have moved. Cleared here so step (1c) cannot compare the
                // new key against a key the new matrix never held.
                epochKeyHeld.clear();
                epochHasRows = false;
                anyBaseRebuilt = true;
            }
            rhsGlob.resize((int)nTotal, 3);
            qGlob.resize((int)nTotal, 3);
        }

        // Read/write one vertex of a context's live iterate. The iterate is
        // double (the solve's precision); contact math runs in PR like PBD's
        // and the corrections are added back into the double iterate.
        auto qVec = [](const SolveCtx& c, Index i) -> Vec3 {
            return qVecOf(c, i);
        };
        // Weight of ONE ONE-SIDED contact constraint on vertex i of context
        // c: w_c = s_c · m_eff / h² (design doc §6.1). The row's only nonzero
        // coefficient is the query vertex's, so the doc's
        // m_eff = 1/(w_v^m + Σ β_a² w_a^m) collapses to m_eff = 1/w_v^m = m_i
        // and this is s_c · m_i / h².
        //
        // The mass is floored at 1e-9 exactly as the matrix diagonal floors
        // it (see the M/h² assembly), so a massless vertex gets a consistent
        // pair of weights rather than a contact that outranks its own
        // inertia. The floor is the CONVENTION, deliberately kept: m_eff is
        // read off the same array the momentum term is, or the contact would
        // be weighed against an inertia the matrix does not have.
        auto planeW = [&](const SolveCtx& c, Index i) -> double {
            const double mi3 = (double)c.m[i * 3];
            return kContactWeightScale * (mi3 > 0.0 ? mi3 : 1e-9) * invH2;
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
        // in (3b)). Both families are energy constraints; sidedness only
        // selects the SHAPE — a one-sided row is a half-space plane on ONE
        // vertex (A = I, diagonal), a coupled row is §5.3's A_c over FOUR —
        // so this predicate is still the single place the two paths part.
        auto twoWayTarget = [&](Index t) -> const SolveCtx* {
            if (t >= numMeshes) return nullptr;
            const int ci = ctxOfMesh[t];
            if (ci < 0) return nullptr;
            const auto& partner = sceneObjects.meshes[t];
            if (!clothLike(partner.behaviorType) || partner.isStatic) return nullptr;
            return &ctxs[(size_t)ci];
        };

        // --- (1b) resolve this substep's WHOLE contact set — one-sided planes
        // AND coupled §5.3 rows — freezing per §5.6 everything the MATRIX
        // depends on: which vertices, their barycentric coefficients β, and
        // the weight w_c those imply.
        //
        // Runs AFTER the ctx loop because a row's sidedness is only decidable
        // once every live cloth of the substep is known (twoWayTarget reads
        // ctxOfMesh), and because β is measured at the WARM-STARTED iterate,
        // which every ctx has only just reached.
        contactRows.clear();
        for (SolveCtx& c : ctxs) {
            MeshCache& mc = *c.mc;
            mc.planes.clear();
            mc.contactW.assign((size_t)c.n, 0.0);
            mc.hasContactW = false;
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
                        // ---- COUPLED vertex-triangle row (design doc §5.3).
                        // Index validity and "everybody pinned" are static
                        // guards; the geometry is measured ONCE, here, at the
                        // warm-started iterate, and frozen for the epoch.
                        if (!pc->facets) return;
                        const Index tri = row.indexPair.target;
                        if (tri >= pc->numFacets) return;
                        ContactRow cr;
                        cr.kind = RowKind::VertexTriangle;
                        const Index t1 = pc->facets[tri*3+0];
                        const Index t2 = pc->facets[tri*3+1];
                        const Index t3 = pc->facets[tri*3+2];
                        if (t1 >= pc->n || t2 >= pc->n || t3 >= pc->n)
                            return;
                        // A degenerate facet would give A_c repeated columns
                        // and a rank-deficient 4x4 block for no constraint.
                        if (t1 == t2 || t2 == t3 || t1 == t3)
                            return;
                        // Self rows never name an incident facet (the narrow
                        // phase excludes adjacency), but a degenerate row must
                        // not build a constraint of a vertex against itself —
                        // that A_c would have a zero row.
                        if (t == c.mi && (i == t1 || i == t2 || i == t3)) return;
                        const int pcIdx = ctxOfMesh[t];
                        cr.cx[0] = (int)ci;
                        cr.cx[1] = cr.cx[2] = cr.cx[3] = pcIdx;
                        cr.vi[0] = i;
                        cr.vi[1] = t1; cr.vi[2] = t2; cr.vi[3] = t3;
                        cr.nrm  = nrm;
                        cr.cross = ((int)ci != pcIdx);
                        cr.iw[0] = invMassOf(c.m,   c.mask,   i);
                        cr.iw[1] = invMassOf(pc->m, pc->mask, t1);
                        cr.iw[2] = invMassOf(pc->m, pc->mask, t2);
                        cr.iw[3] = invMassOf(pc->m, pc->mask, t3);
                        // "모든 참여 정점의 inverse mass가 0이면 constraint를
                        // 만들지 않는다" (§6.1).
                        if (cr.iw[0] <= PR(0) && cr.iw[1] <= PR(0)
                            && cr.iw[2] <= PR(0) && cr.iw[3] <= PR(0)) return;

                        // FROZEN β: the barycentric coordinates of the query
                        // vertex's projection onto the target triangle, at the
                        // warm-started iterate, CLAMPED into the triangle so
                        // Σβ = 1 with every β >= 0 (a convex combination, so
                        // A_c annihilates a rigid translation exactly).
                        const Vec3 qv = qVec(c,  i);
                        const Vec3 P1 = qVec(*pc, t1);
                        const Vec3 P2 = qVec(*pc, t2);
                        const Vec3 P3 = qVec(*pc, t3);
                        const Vec3 e1 = P2 - P1, e2 = P3 - P1;
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
                        cr.alpha[0] = 1.0;
                        cr.alpha[1] = -(double)b1;
                        cr.alpha[2] = -(double)b2;
                        cr.alpha[3] = -(double)b3;

                        // w_c = s_c·m_eff/h² with
                        // m_eff = 1/(w_v^m + β₁²w_1^m + β₂²w_2^m + β₃²w_3^m)
                        // (§6.1). The denominator is A_c's inverse-mass form,
                        // so it is exactly the quantity a one-shot rigid split
                        // of this row would divide by — which is also why the
                        // cd estimator in the local step reuses it. Zero here
                        // means every vertex A_c actually WEIGHS is pinned
                        // (e.g. β puts all of the load on pinned corners), so
                        // the row could not move anything: no constraint.
                        //
                        // NOT clamped from above, because the doc does not
                        // clamp it and because the case that would need it is
                        // narrow: with a FREE query vertex m_eff <= m_query
                        // outright (w_v^m alone already bounds the sum below),
                        // so the weight can only run away when the query is
                        // PINNED and β concentrates on pinned triangle
                        // vertices too. That row then behaves as the locking
                        // §6.2 warns about rather than as a singularity —
                        // still finite, still below kPinWeight — and it is
                        // visible as a spike in the refactor/penetration
                        // read-outs rather than silently absorbed by a magic
                        // ceiling.
                        double wden = 0.0;
                        for (int a = 0; a < 4; ++a)
                            wden += cr.alpha[a] * cr.alpha[a] * (double)cr.iw[a];
                        if (!(wden > 1e-12)) return;
                        cr.mEff = 1.0 / wden;
                        cr.w    = kContactWeightScale * cr.mEff * invH2;
                        cr.thk  = std::max(c.thickness, pc->thickness);
                        cr.col[0] = (int)(c.solveBase   + i);
                        cr.col[1] = (int)(pc->solveBase + t1);
                        cr.col[2] = (int)(pc->solveBase + t2);
                        cr.col[3] = (int)(pc->solveBase + t3);
                        contactRows.push_back(cr);
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
                    // "모든 참여 정점의 inverse mass가 0이면 constraint를
                    // 만들지 않는다" (design doc §6.1) — for a one-sided row
                    // the query vertex is the ONLY participant, so a pinned /
                    // massless one creates no constraint at all. It used to
                    // get a plane whose weight was the floored mass (huge,
                    // hence effectively rigid): harmless, because the pin wins
                    // in the matrix and step (4) never writes a pinned vertex
                    // anyway, but NOT free — it put a weight on that vertex's
                    // epoch key, so a pinned vertex sliding in and out of
                    // contact churned the contact epoch and refactored the
                    // whole mesh for a row that could never move anything.
                    //
                    // The plane is still KEPT when it feeds a dynamic rigid
                    // body: that is not a constraint (it is the partitioned
                    // reaction of §7, settled in step (3b)) and dropping it
                    // would make a pinned sheet penetrable by the body.
                    const PR wi = invMassOf(c.m, c.mask, i);
                    if (wi > PR(0)) {
                        pl.w = planeW(c, i);
                        mc.contactW[i] += pl.w;
                        mc.hasContactW = true;
                    } else if (!pl.coupled) {
                        return;
                    }
                    mc.planes.push_back(pl);
                    // Counts every RETAINED plane, including a coupling-only
                    // one: it is a real contact, it is just not a constraint.
                    // (The penetration sweep below is the other way round —
                    // it measures only rows that carry a constraint, because
                    // residual penetration is a statement about the SOLVE.)
                    ++oneSidedContactCount;
                });
            }
        }
        for (SolveCtx& c : ctxs)
            // Collapse to empty so that "no planes now, none in the factor"
            // stays the zero-cost comparison it was.
            if (!c.mc->hasContactW) c.mc->contactW.clear();

        // --- (1b-e) the EDGE-EDGE half of the coupled set (design doc §5.5).
        // Same substep, same warm-started iterate, same `contactRows` vector,
        // and therefore the same epoch key, the same matrix, the same local
        // step and the same penetration sweep — the whole point of the row
        // being one struct with a kind tag. See resolveEdgeContacts.
        resolveEdgeContacts(sceneObjects, invH2);

        // --- (1c) contact EPOCH and the merged factorization (design doc
        // §5.6). The key summarizes everything about the contact set that the
        // MATRIX depends on: every plane's diagonal weight, and every coupled
        // row's four columns, its frozen β and its w_c. Two substeps with the
        // same key produce the same K_glob, so the factor is reused verbatim —
        // no assembly, no symbolic analysis, no numeric factorization. Empty
        // against empty is the quiescent zero-cost path, exactly as before.
        //
        // The cost this policy accepts, stated rather than hidden: β is part
        // of the key, so a SLIDING contact whose barycentrics move at all
        // re-factorizes the merged system every substep. §5.6 says to fix the
        // coefficients per substep first and measure before lengthening the
        // epoch; quantizing β to widen the key is exactly that later tuning
        // and is deliberately not guessed at here.
        {
            epochKey.clear();
            for (size_t ci = 0; ci < ctxs.size(); ++ci) {
                const MeshCache& mc = *ctxs[ci].mc;
                for (size_t i = 0; i < mc.contactW.size(); ++i)
                    if (mc.contactW[i] != 0.0) {
                        epochKey.push_back((double)ci);
                        epochKey.push_back((double)i);
                        epochKey.push_back(mc.contactW[i]);
                    }
            }
            // Coupled rows of BOTH kinds, in one format: the kind tag, the
            // four (ctx, vertex) pairs, the four A_c coefficients and w_c —
            // everything K_glob's contact term is a function of and nothing
            // else. For an edge-edge row the coefficients ARE (1-s, s, -(1-t),
            // -t), so the "verts, s, t, w_c" §5.6 asks to freeze is exactly
            // what goes in, and the kind tag keeps a §5.3 row from ever
            // hashing equal to a §5.5 row that happens to name the same four
            // vertices with the same coefficients.
            for (const ContactRow& cr : contactRows) {
                epochKey.push_back((double)(int)cr.kind);
                for (int a = 0; a < 4; ++a) {
                    epochKey.push_back((double)cr.cx[a]);
                    epochKey.push_back((double)cr.vi[a]);
                }
                for (int a = 0; a < 4; ++a) epochKey.push_back(cr.alpha[a]);
                epochKey.push_back(cr.w);
            }

            if (anyBaseRebuilt || epochKey != epochKeyHeld) {
                const bool rowsNow = !contactRows.empty();
                // A plane-only epoch touches ONLY diagonal entries the base
                // already has, so it keeps the pre-slice values-only path and
                // its one-symbolic-analysis-per-base-rebuild contract. Coupled
                // rows add entries the base does not have, so the pattern
                // changes when they appear AND when they go away again.
                const bool needAnalyze = anyBaseRebuilt || rowsNow
                                       || epochHasRows;
                crossBlockEntryCount = 0;
                if (rowsNow) {
                    std::vector<Eigen::Triplet<double>> ct;
                    size_t planeCount = 0;
                    for (const SolveCtx& c : ctxs)
                        planeCount += c.mc->contactW.size();
                    ct.reserve(contactRows.size() * 16 + planeCount);
                    for (const SolveCtx& c : ctxs) {
                        const MeshCache& mc = *c.mc;
                        for (size_t i = 0; i < mc.contactW.size(); ++i)
                            if (mc.contactW[i] != 0.0) {
                                const int g = (int)(c.solveBase + (Index)i);
                                ct.emplace_back(g, g, mc.contactW[i]);
                            }
                    }
                    // K += w_c·A_cᵀA_c: the 4x4 outer product of A_c's
                    // coefficients over the row's four COLUMNS of the merged
                    // system. 16 entries, of which the off-diagonal ones are
                    // the coupling §5.3/§5.5 are about — and, when the columns
                    // reach into two meshes' blocks, the CROSS-BLOCK entries
                    // §5.4 needs one system for. Symmetric and PSD by
                    // construction (a Gram matrix), with zero row sums since
                    // Σα = 0: the row resists the relative separation, not a
                    // rigid translation of the four vertices together.
                    //
                    // IDENTICAL for both kinds. A vertex-triangle row's
                    // α = [1, -β] puts its cross-block entries in the 6 slots
                    // that mix column 0 with a triangle column; an edge-edge
                    // row's α = [(1-s), s, -(1-t), -t] splits 2+2 instead, so
                    // "is this entry cross-block" is answered by the COLUMNS'
                    // ctxs, not by a hardcoded query slot.
                    for (const ContactRow& cr : contactRows) {
                        for (int r = 0; r < 4; ++r)
                            for (int s = 0; s < 4; ++s) {
                                const double val =
                                    cr.w * cr.alpha[r] * cr.alpha[s];
                                if (val == 0.0) continue;
                                ct.emplace_back(cr.col[r], cr.col[s], val);
                                if (cr.cx[r] != cr.cx[s]) ++crossBlockEntryCount;
                            }
                    }
                    Eigen::SparseMatrix<double> C((int)nTotal, (int)nTotal);
                    C.setFromTriplets(ct.begin(), ct.end());
                    Kglob = Kbase + C;
                } else {
                    // Values-only diagonal update against the base pattern:
                    // every diagonal entry exists in Kbase (each block's mass
                    // term emplaces all of them), so coeffRef never inserts.
                    Kglob = Kbase;
                    for (const SolveCtx& c : ctxs) {
                        const MeshCache& mc = *c.mc;
                        for (size_t i = 0; i < mc.contactW.size(); ++i)
                            if (mc.contactW[i] != 0.0) {
                                const int g = (int)(c.solveBase + (Index)i);
                                Kglob.coeffRef(g, g) += mc.contactW[i];
                            }
                    }
                }
                if (needAnalyze) globFactor->analyzePattern(Kglob);
                globFactor->factorize(Kglob);
                // Base rebuilds are NOT contact refactors (they are gated on
                // scalars the user changed deliberately) — same contract the
                // counter always had.
                if (!anyBaseRebuilt) ++contactRefactorCount;
                globValid = (globFactor->info() == Eigen::Success);
                if (globValid) {
                    epochKeyHeld = epochKey;
                    epochHasRows = rowsNow;
                    for (SolveCtx& c : ctxs) c.mc->epochW = c.mc->contactW;
                } else {
                    // Should not happen — K_glob is the base plus a sum of PSD
                    // terms. Fall back to the CONTACT-FREE merged system: with
                    // one factor for every cloth there is no per-mesh subset to
                    // keep, so the substep loses its whole contact set and
                    // keeps its physics. (Pre-merge this dropped one mesh's
                    // contacts; the coarser blast radius is the price §5.4's
                    // single system charges.)
                    std::cerr << "[PdSystem] merged contact factorization failed"
                                 " — running this substep contact-free\n";
                    ++contactFactorFailCount;
                    contactRows.clear();
                    for (SolveCtx& c : ctxs) {
                        c.mc->planes.clear();
                        c.mc->contactW.clear();
                        c.mc->epochW.clear();
                    }
                    contactsLive = false;
                    Kglob = Kbase;
                    globFactor->analyzePattern(Kglob);
                    globFactor->factorize(Kglob);
                    ++contactRefactorCount;
                    globValid = (globFactor->info() == Eigen::Success);
                    epochKeyHeld.clear();
                    epochHasRows = false;
                }
            }
            // A merged system that will not factor at all leaves every cloth
            // to the previous frame's state rather than integrating garbage.
            if (!globValid) return;
        }

        // --- (2) local/global block coordinate descent (Liu 2013 §3),
        // ITERATION-OUTER like PbdSystem step (2): sweeping all meshes per
        // iteration is what lets a cross-cloth contact see, and be seen by,
        // the partner's own internal constraints within the substep.
        //
        // Each iteration is THREE phases, in this order, because a coupled row
        // writes RHS rows of TWO meshes and neither may be solved before both
        // have been assembled:
        //   LOCAL springs   — per mesh, parallel, independent;
        //   LOCAL contacts  — ONE serial block over every mesh's planes and
        //                     every coupled row of the substep;
        //   GLOBAL          — ONE merged back-substitution over every cloth.
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

                // LOCAL pass 3, parallel over TRIANGLES (design doc §3.3):
                // project each element's deformation gradient onto the
                // admissible set. Independent per triangle — element ti reads
                // the frozen iterate and owns strainP[6ti .. 6ti+5] — so this
                // is write-disjoint exactly like the spring pass, and it is
                // the SAME schedule decision (`par`) so a small sheet keeps
                // the serial walk through the identical lambda.
                //
                // A failed projection leaves the slot alone, which is the
                // "reuse the previous P_t" fallback of §3.3 — no branch here,
                // it falls out of projectStrain's contract.
                {
                    const size_t numTris = mc.strainTris.size();
                    if (numTris) {
                        const StrainTri* tr = mc.strainTris.data();
                        const Eigen::MatrixXd& q = mc.q;
                        double* pOut = mc.strainP.data();
                        const double smin = mc.sigMin;
                        const double smax = mc.sigMax;
                        forRange(par, numTris, [&](size_t begin, size_t end) {
                            for (size_t ti = begin; ti < end; ++ti) {
                                const StrainTri& t = tr[ti];
                                // F(a, r) = Σ_s G(r, s)·q(v_s, a); see the
                                // F_tᵀ = G_t·Q_t identity on StrainTri.
                                double F[6];
                                for (int ax = 0; ax < 3; ++ax) {
                                    const double p0 = q((int)t.v[0], ax);
                                    const double p1 = q((int)t.v[1], ax);
                                    const double p2 = q((int)t.v[2], ax);
                                    F[ax*2+0] = t.G[0][0]*p0 + t.G[0][1]*p1
                                              + t.G[0][2]*p2;
                                    F[ax*2+1] = t.G[1][0]*p0 + t.G[1][1]*p1
                                              + t.G[1][2]*p2;
                                }
                                projectStrain(F, smin, smax, pOut + ti * 6);
                            }
                        });
                    }
                }

                // LOCAL pass 4, parallel over VERTICES: the strain part of
                // the RHS, b(:,a) += Σ_t w_s·A_t·G_tᵀ·P_tᵀ(:,a), by GATHER.
                //
                // Derivation of the per-vertex form, so the slot indexing
                // below is checkable rather than trusted: the element's
                // energy is (w_s A_t/2)|G_t·Q_t - P_tᵀ|², whose row for the
                // vertex in local slot s and axis a is
                //   w_s·A_t · Σ_r G(r, s)·P_tᵀ(r, a)
                //   = w_s·A_t · Σ_r G(r, s)·P_t(a, r)
                //   = w_s·A_t · (P_t · g)_a,      g = G_t(:, s) ∈ R²
                // i.e. exactly the doc's "w_s·A_t·G_tᵀ·P_tᵀ" restricted to
                // the one row this vertex owns. Consistency check against the
                // matrix: replacing P_t by F_t(q) makes this equal to
                // (w_s·A_t·G_tᵀG_t·q) on the same row, which is the diagonal
                // block assembled above — so at F == P the element
                // contributes zero net force, as it must.
                {
                    if (!mc.strainTris.empty() && c.kS > 0.0) {
                        const Index*       offs = mc.triOffsets.data();
                        const TriIncident* ent  = mc.triEntries.data();
                        const StrainTri*   tr   = mc.strainTris.data();
                        const double*      pIn  = mc.strainP.data();
                        Eigen::MatrixXd&   rhs  = mc.rhs;
                        const double ws = c.kS;
                        forRange(par, (size_t)c.n, [&](size_t begin, size_t end) {
                            for (size_t i = begin; i < end; ++i) {
                                double gx = 0.0, gy = 0.0, gz = 0.0;
                                for (Index e = offs[i]; e < offs[i+1]; ++e) {
                                    const TriIncident in = ent[e];
                                    const StrainTri& t = tr[in.tri];
                                    const double w = ws * t.area;
                                    const double g0 = t.G[0][in.slot];
                                    const double g1 = t.G[1][in.slot];
                                    const double* P = pIn + (size_t)in.tri * 6;
                                    gx += w * (P[0]*g0 + P[1]*g1);
                                    gy += w * (P[2]*g0 + P[3]*g1);
                                    gz += w * (P[4]*g0 + P[5]*g1);
                                }
                                rhs((int)i,0) += gx;
                                rhs((int)i,1) += gy;
                                rhs((int)i,2) += gz;
                            }
                        });
                    }
                }

                // LOCAL pass 5, parallel over VERTICES (design doc §4.2):
                // evaluate v_i = L_i q against the live iterate and project
                // it onto the sphere of radius ‖v_i⁰‖. Independent per vertex
                // — row i reads the frozen iterate and owns p[3i .. 3i+2] —
                // so it is write-disjoint exactly like the strain pass, on
                // the SAME schedule decision.
                //
                // An ill-conditioned direction leaves the slot alone, which
                // is §4.2's fallback chain; no branch here, it falls out of
                // projectBend's contract.
                {
                    const BendCache& bc = mc.bend;
                    if (!bc.rowEntries.empty()) {
                        const Index*     offs = bc.rowOffsets.data();
                        const BendEntry* ent  = bc.rowEntries.data();
                        const double*    v0l  = bc.v0len.data();
                        const double*    ar   = bc.area.data();
                        const Eigen::MatrixXd& q = mc.q;
                        double* pOut = mc.bend.p.data();
                        forRange(par, (size_t)c.n, [&](size_t begin, size_t end) {
                            for (size_t i = begin; i < end; ++i) {
                                if (!(ar[i] > 0.0)) continue;
                                double v[3] = { 0.0, 0.0, 0.0 };
                                for (Index k = offs[i]; k < offs[i+1]; ++k) {
                                    const BendEntry& en = ent[k];
                                    for (int ax = 0; ax < 3; ++ax)
                                        v[ax] += en.c * q((int)en.col, ax);
                                }
                                projectBend(v, v0l[i], pOut + i * 3);
                            }
                        });
                    }
                }

                // LOCAL pass 6, parallel over VERTICES: the bending part of
                // the RHS, b += Σ_i w_b·A_i·L_iᵀ·p_i, by GATHER.
                //
                // L_iᵀ scatters row i over every vertex the row NAMES, so
                // vertex k collects w_b·A_i·L_i(k)·p_i from every row i that
                // touches it — its own row and the rows of its one-ring —
                // which is exactly what the transpose incidence enumerates.
                // Consistency check against the matrix, the same one the
                // strain pass documents: replacing p_i by L_i q makes this
                // equal to (w_b·A_i·L_iᵀL_i·q) on the same row, so at
                // L_i q == p_i the element contributes zero net force.
                {
                    const BendCache& bc = mc.bend;
                    if (!bc.tEntries.empty() && c.kB > 0.0) {
                        const Index*          offs = bc.tOffsets.data();
                        const BendTIncident*  ent  = bc.tEntries.data();
                        const double*         ar   = bc.area.data();
                        const double*         pIn  = bc.p.data();
                        Eigen::MatrixXd&      rhs  = mc.rhs;
                        const double wb = c.kB * bc.areaScale;
                        forRange(par, (size_t)c.n, [&](size_t begin, size_t end) {
                            for (size_t i = begin; i < end; ++i) {
                                double gx = 0.0, gy = 0.0, gz = 0.0;
                                for (Index e = offs[i]; e < offs[i+1]; ++e) {
                                    const BendTIncident in = ent[e];
                                    const double w = wb * ar[in.row] * in.c;
                                    const double* p = pIn + (size_t)in.row * 3;
                                    gx += w * p[0];
                                    gy += w * p[1];
                                    gz += w * p[2];
                                }
                                rhs((int)i,0) += gx;
                                rhs((int)i,1) += gy;
                                rhs((int)i,2) += gz;
                            }
                        });
                    }
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
                    // Strain elements belong to the SMOOTH part too: their
                    // local step is an exact closest-point projection onto
                    // the admissible set (Procrustes/SVD), so the block
                    // coordinate descent argument covers them verbatim even
                    // though that set is not convex. Omitting them would make
                    // PD-8 gate an incomplete objective — and one whose
                    // missing term the global solve is actively lowering.
                    for (size_t ti = 0; ti < mc.strainTris.size(); ++ti) {
                        const StrainTri& t = mc.strainTris[ti];
                        const double w = c.kS * t.area;
                        if (!(w > 0.0)) continue;
                        const double* P = mc.strainP.data() + ti * 6;
                        double acc = 0.0;
                        for (int ax = 0; ax < 3; ++ax) {
                            const double p0 = q((int)t.v[0], ax);
                            const double p1 = q((int)t.v[1], ax);
                            const double p2 = q((int)t.v[2], ax);
                            for (int r = 0; r < 2; ++r) {
                                const double fv = t.G[r][0]*p0 + t.G[r][1]*p1
                                                + t.G[r][2]*p2;
                                const double d = fv - P[ax*2 + r];
                                acc += d * d;
                            }
                        }
                        probeSmooth += 0.5 * w * acc;
                    }
                    // Bending elements are SMOOTH for the same reason: p_i is
                    // the exact closest point of the sphere ‖p‖ = ‖v_i⁰‖ to
                    // L_i q (design doc §4.2). That set is not convex either
                    // — it is a sphere, as the strain step's is a Stiefel set
                    // — and the block coordinate descent argument does not
                    // need it to be, only that the local step MINIMIZES the
                    // term it owns.
                    {
                        const BendCache& bc = mc.bend;
                        for (Index i = 0; i < (Index)bc.area.size(); ++i) {
                            const double w = c.kB * bc.areaScale * bc.area[i];
                            if (!(w > 0.0)) continue;
                            const double* p = bc.p.data() + (size_t)i * 3;
                            double v[3] = { 0.0, 0.0, 0.0 };
                            for (Index k = bc.rowOffsets[i];
                                 k < bc.rowOffsets[i+1]; ++k) {
                                const BendEntry& en = bc.rowEntries[k];
                                for (int ax = 0; ax < 3; ++ax)
                                    v[ax] += en.c * q((int)en.col, ax);
                            }
                            double acc = 0.0;
                            for (int ax = 0; ax < 3; ++ax) {
                                const double d = v[ax] - p[ax];
                                acc += d * d;
                            }
                            probeSmooth += 0.5 * w * acc;
                        }
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
            // The two families differ only in the operator: a plane is
            // A = B = I on one vertex, a coupled row is A_c over four. Both
            // are unilateral — a SATISFIED constraint projects to the
            // IDENTITY (p = A q), which contributes w·Aᵀ·A q against a matrix
            // holding w·AᵀA, i.e. exactly nothing. That branch is the whole
            // reason a contact may sit in the matrix for the entire substep
            // without gluing anything, and it is what design doc §9.3's
            // "분리된 뒤 identity projection이 인력을 만들지 않는다" asks for.
            {
                // Deposit w·p into vertex i of context `cc`, unless the
                // substep lost its contact weights (refactor failure).
                auto emit = [&](const SolveCtx& cc, Index i, double w,
                                const Vec3& p) {
                    if (!(w > 0.0) || !contactsLive) return;
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

                // COUPLED rows — BOTH kinds (design doc §5.3 and §5.5). ONE
                // constraint per contact over four vertices:
                //
                //   r_c   = A_c q = Σ_a α_a q_a                   (α FROZEN)
                //   g     = max(0, d - nᵀr_c)                     (violation)
                //   p_c   = Π_{H_c}(r_c) = r_c + n·g              (local step)
                //   b    += w_c·A_cᵀp_c                           (global step)
                //
                // so the RHS deposit is w_c·α_a·p_c on each of the four
                // columns and NOT four separate per-vertex targets. With α in
                // the row, this loop is kind-agnostic everywhere except the
                // normal, below.
                //
                // g == 0 (separated) makes p_c = r_c = A_c q, i.e. the RHS
                // deposit is exactly w_c·A_cᵀA_c q, which the matrix block
                // cancels: a satisfied row contributes NOTHING. That is the
                // unilateral branch, and it is what lets a row stay resident
                // for the whole substep without gluing the two sheets.
                //
                // The NORMAL is re-evaluated here every local step from the
                // live iterate (§5.6 item 3) — the triangle's cross product
                // for a §5.3 row, the two edge directions' for a §5.5 one —
                // while its SIGN comes from the frozen entry side in `nrm`,
                // which must never be re-read off geometry the solve has
                // already corrected. α and w_c are frozen for the epoch,
                // because they are the matrix.
                for (const ContactRow& cr : contactRows) {
                    if (!contactsLive) break;
                    const SolveCtx* cc[4];
                    Vec3 P[4];
                    Vec3 r;
                    for (int a = 0; a < 4; ++a) {
                        cc[a] = &ctxs[(size_t)cr.cx[a]];
                        P[a]  = qVec(*cc[a], cr.vi[a]);
                        r    += P[a] * (PR)cr.alpha[a];
                    }
                    Vec3 n;
                    PR   g = PR(0);
                    // The geometric direction the half-space is measured
                    // along. Degenerate (a collapsed triangle, two collinear
                    // edges) leaves g at 0, i.e. the identity projection —
                    // the same "no usable normal, no constraint this
                    // iteration" branch the §5.3 rows always had.
                    Vec3 ngeom;
                    if (cr.frozenNormal) {
                        // §5.5 near-parallel fallback: the entry-side normal
                        // is the whole normal, for the whole epoch.
                        ngeom = cr.nrm;
                    } else if (cr.kind == RowKind::VertexTriangle) {
                        ngeom = (P[2] - P[1]).cross(P[3] - P[1]);
                    } else {
                        ngeom = (P[1] - P[0]).cross(P[3] - P[2]);
                    }
                    const PR nlenG = ngeom.norm();
                    if (nlenG >= PR(1e-12)) {
                        const Vec3 nh = ngeom / nlenG;
                        n = nh * ((cr.nrm.dot(nh) >= PR(0)) ? PR(1) : PR(-1));
                        g = cr.thk - n.dot(r);
                        if (g < PR(0)) g = PR(0);
                    }
                    const Vec3 p = (g > PR(0)) ? r + n * g : r;
                    // b(col_a) += w_c·α_a·p_c.
                    const double wr = cr.w;
                    const double px = (double)p.x, py = (double)p.y,
                                 pz = (double)p.z;
                    for (int a = 0; a < 4; ++a) {
                        if (cr.alpha[a] == 0.0) continue;
                        const double s = wr * cr.alpha[a];
                        Eigen::MatrixXd& rhs = cc[a]->mc->rhs;
                        rhs((int)cr.vi[a], 0) += s * px;
                        rhs((int)cr.vi[a], 1) += s * py;
                        rhs((int)cr.vi[a], 2) += s * pz;
                    }
                    // PROBE: E_c = (w_c/2)‖A_c q - p_c‖² = (w_c/2)·g², since
                    // the projection moves r_c along n by exactly g.
                    if (debugObjectiveProbe)
                        probeContact += 0.5 * wr * (double)g * (double)g;

                    // cd ESTIMATOR — see step (4). DERIVATION, because the
                    // quantity changed with the mechanism.
                    //
                    // The RHS term this row adds to column a is w_c·α_a·p_c
                    // while the matrix holds w_c·α_a·(A_c q) on the same row,
                    // so the row's net pull on column a is
                    //     w_c·α_a·(p_c - A_c q) = w_c·α_a·n·g,
                    // i.e. along +n where α_a > 0 and along -n where it is
                    // negative. The DIRECTIONS the clamp needs are therefore
                    // exactly ±n, unchanged from the previous mechanism.
                    //
                    // For the MAGNITUDE, cd wants a displacement, not a force.
                    // The displacement that closes g in one shot, split by
                    // inverse mass, is
                    //     Δ_a = n·m_eff·α_a·w_a^m·g
                    // (check: nᵀΣ_a α_aΔ_a = m_eff·g·Σ_a α_a²w_a^m = g
                    // exactly, since m_eff is the inverse of that sum). For a
                    // §5.3 row α = [1, -β] recovers the previous +n·m_eff·w_v·g
                    // on the query and -n·m_eff·β_a·w_a·g on the triangle.
                    // m_eff and α are already frozen on the row, so this costs
                    // four multiplies.
                    //
                    // WHICH COLUMNS TAKE A SHARE differs by kind, and that is
                    // the second of the two places this loop is not
                    // kind-agnostic:
                    //
                    //   §5.3 — columns 1..3 only. cd is a SECOND-CHOICE
                    //   estimator there, covering what the per-row clamp in
                    //   (4) cannot see: a vertex's share of a row somebody
                    //   ELSE queried. Where the row is the vertex's OWN, (4)
                    //   clamps along that row's frozen narrow-phase normal
                    //   instead, because summing offsets across roles cancels
                    //   — a self-fold vertex is pushed +n as a query and -n as
                    //   a triangle vertex of the neighbouring row, and the
                    //   summed direction then clamps neither (measured: 0.16 m
                    //   of extra peak height on the folded sheet). So the
                    //   QUERY share is deliberately NOT accumulated.
                    //
                    //   §5.5 — ALL FOUR columns. An edge-edge row has no query
                    //   vertex and, decisively, is NOT in the narrow-phase
                    //   arrays at all: it was found by this file's own
                    //   detection pass, so forEachContact in (4) will never
                    //   walk it and cd is the row's ONLY clamp channel. Every
                    //   participating vertex must therefore take its share
                    //   here or its depenetration leaks into velocity.
                    //
                    // Why summing the per-ITERATION asks is the right side to
                    // err on: (4) uses cd as a DIRECTION plus an upper bound
                    // on how much velocity may be removed, and never removes
                    // below the vertex's own approach speed. Under-estimating
                    // LEAKS depenetration into velocity — the invariant this
                    // mechanism exists to protect — while over-estimating only
                    // degrades to the one-sided rows' behaviour (clamp exactly
                    // to the approach speed), which is strictly safe. The
                    // global solve realizes only a fraction of each ask, so
                    // summing them over the iteration loop over-estimates by
                    // construction. The "remaining deficit at the last sweep"
                    // alternative errs the wrong way: a converged row asks for
                    // ~0 and would clamp nothing at all.
                    if (g > PR(0)) {
                        const PR sc = (PR)(cr.mEff * (double)g);
                        const int aBegin =
                            (cr.kind == RowKind::VertexTriangle) ? 1 : 0;
                        for (int a = aBegin; a < 4; ++a)
                            vertexRef(cc[a]->cd, cr.vi[a]) +=
                                n * (sc * (PR)cr.alpha[a] * cr.iw[a]);
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

            // ---- GLOBAL: ONE merged back-substitution over every cloth
            // (design doc §5.4). Gather the per-mesh RHS blocks into the
            // merged vector, solve, scatter the answer back into each mesh's
            // iterate. The per-mesh rhs/q arrays stay the working storage
            // every local pass reads and writes; only this step knows about
            // the merge, which is what keeps the coupling change from
            // rippling through six parallel sweeps.
            //
            // Back-substitution against the prefactored K_glob is the step
            // that makes PD implicit — every vertex sees every other through
            // K⁻¹, which is why stiff elements do not explode the way the
            // symplectic path does at the same substep count. With the
            // coupled rows in the matrix, "every other" now reaches ACROSS
            // meshes in the same solve, which is precisely what §5.4 asks for
            // and what the Jacobi exchange it replaced could not do.
            //
            // The three coordinate columns share K but are otherwise
            // independent — A_c's coefficients (1, -β_a) are scalars common
            // to the three axes, exactly like G_t's and L_i's — so they are
            // solved as three CONCURRENT per-axis back-substitutions (Liu
            // 2013 §8). SimplicialLDLT is safe to share across them: solve()
            // is const, its only mutable member (m_info) is written by
            // compute()/factorize() and merely read here, and each call
            // allocates its own temporaries. qGlob is column-major, so the
            // three destination columns are disjoint contiguous spans.
            //
            // Below the gate the three columns go in ONE call: for a small
            // system the extra solve set-up costs more than the axes save.
            for (const SolveCtx& c : ctxs)
                rhsGlob.middleRows((int)c.solveBase, (int)c.n) = c.mc->rhs;
            if (nTotal >= kParallelMinVerts) {
                PdSystem* sys = this;
                dispatch_apply(3, DISPATCH_APPLY_AUTO, ^(size_t ax) {
                    const Eigen::Index col = (Eigen::Index)ax;
                    sys->qGlob.col(col) =
                        sys->globFactor->solve(sys->rhsGlob.col(col));
                });
            } else {
                qGlob = globFactor->solve(rhsGlob);
            }
            for (const SolveCtx& c : ctxs)
                c.mc->q = qGlob.middleRows((int)c.solveBase, (int)c.n);
        }

        // --- (3a) residual penetration of the CONTACT SOLVE (design doc §6.3,
        // §1 clause 5). ONE extra pass over the planes and the rows, at the
        // converged iterate — the same q step (4) is about to commit — using
        // the SAME half-space evaluation the local step used, so what is
        // reported is the violation of the constraint the solve actually
        // held, not of a freshly re-derived one.
        //
        // Runs BEFORE (3b) on purpose: the local step read rigidDelta at its
        // pre-(3b) value, so measuring after (3b) would subtract a body
        // motion the solve never saw and under-report.
        //
        // Only rows that CARRY a constraint are measured (see the plane
        // retention note above): the residual of a row with no weight is not
        // this solver's residual.
        {
            double penSum = 0.0, penMax = 0.0;
            size_t penRows = 0;
            for (const SolveCtx& c : ctxs) {
                const MeshCache& mc = *c.mc;
                for (const ContactPlane& pl : mc.planes) {
                    if (!(pl.w > 0.0)) continue;
                    const Index i = pl.vert;
                    PR dist = pl.d0 + pl.n.dot(qVec(c, i) - vertexAt(c.x, i));
                    if (pl.coupled) dist -= pl.n.dot(rigidDelta[pl.target]);
                    const double pen =
                        std::max(0.0, (double)(c.thickness - dist));
                    penMax = std::max(penMax, pen);
                    penSum += pen;
                    ++penRows;
                }
            }
            // BOTH coupled kinds, through the same generic A_c form the local
            // step uses — a §5.5 row's residual is measured exactly like a
            // §5.3 row's, so the GUI's penetration read-out covers the edge
            // family the moment it exists.
            for (const ContactRow& cr : contactRows) {
                Vec3 P[4], r;
                for (int a = 0; a < 4; ++a) {
                    P[a] = qVec(ctxs[(size_t)cr.cx[a]], cr.vi[a]);
                    r   += P[a] * (PR)cr.alpha[a];
                }
                Vec3 ngeom;
                if (cr.frozenNormal)                          ngeom = cr.nrm;
                else if (cr.kind == RowKind::VertexTriangle)
                    ngeom = (P[2] - P[1]).cross(P[3] - P[1]);
                else
                    ngeom = (P[1] - P[0]).cross(P[3] - P[2]);
                const PR nlenG = ngeom.norm();
                // A degenerate operator projected to the identity in the local
                // step too, so it has no residual to report.
                if (nlenG < PR(1e-12)) continue;
                const Vec3 nh = ngeom / nlenG;
                // The SAME half-space the local step held: frozen entry side,
                // frozen α, live geometric normal — never a freshly re-derived
                // constraint.
                const Vec3 sn = nh * ((cr.nrm.dot(nh) >= PR(0)) ? PR(1)
                                                                : PR(-1));
                const double pen =
                    std::max(0.0, (double)(cr.thk - sn.dot(r)));
                penMax = std::max(penMax, pen);
                penSum += pen;
                ++penRows;
            }
            // Fold this substep into the FRAME's window (see the members).
            maxPenetrationDepth = std::max(maxPenetrationDepth, penMax);
            penetrationSum     += penSum;
            penetrationRowCount += (uint32_t)penRows;
            meanPenetrationDepth = penetrationRowCount
                ? penetrationSum / (double)penetrationRowCount : 0.0;
        }

        // --- (3b) cloth → rigid-body coupling, ONCE per substep, as the
        // NEWTON PAIR of the cloth-side penalty. The plane row's force on the
        // cloth at the converged iterate is F = w_c·deficit along n (w_c =
        // s_c·m_i/h²), so the body's equal-and-opposite positional response
        // over one substep is
        //     Δ_B = F·h²/m_B = s_c·deficit·(m_i/m_B)
        //         = s_c·deficit·(wB/wc),
        // implemented with the bounded fraction wB/(wc + wB) (identical for
        // wc >> wB, and it makes the pinned branch wc == 0 collapse into the
        // same expression) and clamped at the deficit itself so an extreme
        // s_c teleports the body at most out of contact. The deficit is
        // measured against the rigidDelta already accumulated this frame, so
        // no substep re-pays a correction (the narrow phase froze d0 at the
        // frame start).
        //
        // Both rejected alternatives are MEASURED failures on pd_cloth_ball
        // (0.1 kg ball dropped on a corner-pinned sheet):
        //   * "realized push" (how far the vertex travelled): reaction dries
        //     up as the solve converges → the ball ratchets down through the
        //     sheet at ~0.15 m/s regardless of s_c.
        //   * per-sweep deficit split (PbdSystem step (3) transplanted): the
        //     iteration loop pays the SAME standing deficit 16 times per
        //     substep, so the ball is held up by rigidDelta alone while the
        //     cloth carries nothing — ball-induced extra sag 3 mm vs PBD's
        //     140 mm, i.e. the ball's weight never reaches the fabric. PBD
        //     gets away with per-sweep payment because its cloth share moves
        //     the vertex IMMEDIATELY (Gauss-Seidel), closing the deficit
        //     within the sweep; PD's cloth only moves at the global solve,
        //     so the sweep loop double-counts.
        // The Newton pair makes the support the cloth gives the body exactly
        // what the body's weight costs the cloth — equilibrium then requires
        // a standing deficit whose penalty force carries m_B·g, which is the
        // sag the user actually sees.
        //
        // A PINNED cloth vertex (wc == 0, plane kept coupling-only) pays the
        // whole deficit — the "nobody else can move" branch, as before.
        for (SolveCtx& c : ctxs) {
            const MeshCache& mc = *c.mc;
            for (const ContactPlane& pl : mc.planes) {
                if (!pl.coupled) continue;
                const Index i = pl.vert;
                const PR wc = invMassOf(c.m, c.mask, i);
                const Vec3 qf = qVec(c, i);
                PR dist = pl.d0 + pl.n.dot(qf - vertexAt(c.x, i))
                        - pl.n.dot(rigidDelta[pl.target]);
                const PR deficit = std::max(PR(0), c.thickness - dist);
                if (deficit <= PR(0)) continue;
                const PR frac = (PR)kContactWeightScale
                              * (pl.wB / (wc + pl.wB));
                const PR share = deficit * std::min(PR(1), frac);
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
        //     of a vertex clamps per row — one-sided planes and the §5.3
        //     rows this vertex is the QUERY of, both along the frozen row
        //     normal, which for a coupled row is the direction the solve's
        //     own pull w_c·(p_c - A_c q) = w_c·n·g points in. Only its
        //     TRIANGLE-vertex share of other vertices' rows pulls along
        //     -β_a·n instead and has no normal of its own to read, and that
        //     is what cd is for. Both families are energy terms, so neither
        //     delta is a literal push; the clamp does not need it to be, it
        //     needs a direction and a bound (see the cd estimator note).
        //     EDGE-EDGE (§5.5) rows are NOT covered by the forEachContact
        //     walk below at all — they never entered the narrow-phase
        //     arrays it reads, because this file's own detection pass found
        //     them. `cd` is their ONLY clamp channel, which is why the local
        //     step accumulates all FOUR of their columns into it.
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
                // row normal, whichever mechanism applied it, so a coupled row
                // is clamped here as well and not left to `cd` alone. (This
                // is not double-removal: the clamp drives delta·n down to
                // `approach` and is idempotent for a repeated normal; only
                // genuinely different normals compose.) Measured on the
                // folded-sheet case, restricting the coupled rows to the cd
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
