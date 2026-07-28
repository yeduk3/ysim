#pragma once
// ysim validity self-test (runSelfTest) — 40+ BDD correctness blocks.
// Restored from the pre-split src/main.cpp and moved here to sit with the
// other validity tests (doctest suite in this dir). PERF benches were
// dropped; Block 24 (the refit-bench smoke test) went with them.
//
// Fragment header: included once by src/main.cpp after all sim modules
// (needs the full engine + GL helpers in scope); invoked via `--self-test`.
// Not standalone.

static int runSelfTest() {
    using Backend = METAL;
    int failures = 0;
    auto pass = [&](const char* name) {
        std::cerr << "[self-test PASS] " << name << "\n";
    };
    auto fail = [&](const char* name, const std::string& reason) {
        std::cerr << "[self-test FAIL] " << name << ": " << reason << "\n";
        ++failures;
    };

    auto pumpFrames = [](auto& sim, int n) {
        for (int i = 0; i < n; ++i) sim.update();
    };

    // Reset Scene-side static state so the helper is independent of any
    // prior main() body (and stays robust if the Estimator extends the
    // self-test to include multiple back-to-back synthetic scenes).
    auto resetScene = []() {
        for (auto& m : Scene<Backend, Precision>::meshes) m.initializer = nullptr;
        Scene<Backend, Precision>::meshes.clear();
        for (auto& r : Scene<Backend, Precision>::requestsGeneralMeshes)
            delete r.initializer;
        Scene<Backend, Precision>::requestsGeneralMeshes.clear();
        Scene<Backend, Precision>::numMeshes = 0;
        Scene<Backend, Precision>::nextMeshId = 0;
        Scene<Backend, Precision>::dirty = true;
        Scene<Backend, Precision>::environment = SceneEnvironment{};
    };

    // Phase-2 hybrid test migration: load a scene fixture (the scene SETUP as
    // JSON) and apply it through the same applySnapshot path the GUI/CLI use.
    // Assertions stay in C++; only the inline scene construction moves to
    // JSON. Resolved under <project-root>/test/fixtures/. Records a failure
    // (and returns false) if the fixture is missing/invalid.
    auto loadFixture = [&](auto& sim, const std::string& name) -> bool {
        const std::string path =
            ysim_paths::projectRoot() + "/test/fixtures/" + name;
        auto r = scene_format::readFromFile(path);
        if (!r.ok) {
            fail("fixture-load", path + ": " + r.error.message);
            return false;
        }
        sim.applySnapshot(r.value, scene_format::sceneDir(path));
        return true;
    };

    // The shared synthetic scene (4×4 TriangularCloth + cube + static ground)
    // now lives in test/fixtures/synthetic.json. applySnapshot already clears
    // the prior scene, so no separate resetScene() is needed. Regenerate the
    // fixture with `YSIM_DUMP_FIXTURES=1 ./build/ysim --self-test` if the
    // synthetic scene definition changes (see the Block-1 dump below).
    auto buildSyntheticScene = [&](auto& sim) {
        loadFixture(sim, "synthetic.json");
    };

    // Bring up Metal eagerly. SKIP-not-FAIL when the device or metallib is
    // missing — the Estimator runs verify.sh inside a Linux container with
    // no Metal at all, and the build + JSON-layer doctest binaries are
    // valid signal independent of GPU availability. Treating Metal-absent
    // as a failure would make the gate unrunnable anywhere except the
    // user's macOS host.
    auto skip = [&](const char* name, const std::string& reason) {
        std::cerr << "[self-test SKIP] " << name << ": " << reason << "\n";
    };

    // ---- Block 30: D-037 — NullRigidPhysicsBackend contract round-trip. -----
    // Pure-C++ null backend; no Metal calls; runs on macOS + Linux containers.
    // Foundation for slice B-2 (rigid backend impl) and B-3 (Rigid behavior wiring).
    // Placed ABOVE the Metal-less SKIP gate so the contract IS exercised on
    // Linux verify.sh (folds Estimator turn 33 WARNING; D-038 records the
    // relocation as part of the B-2′ Euler-impl slice).
    {
        // ---- Clause 1 — Lifecycle ---------------------------------------
        {
            ysim::physics::NullRigidPhysicsBackend backend;
            const char* name = backend.backendName();
            bool nameOk = (name != nullptr && std::string(name) == "Null");
            bool initOk = backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));
            backend.setGravity(tinym::vec3(0.0f, -1.0f, 0.0f));  // must not crash
            backend.shutdown();                                   // must not crash
            bool reinitOk = backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

            if (nameOk && initOk && reinitOk) {
                pass("D-037 / NullRigidPhysicsBackend lifecycle: name + init + setGravity + shutdown + re-init");
            } else {
                fail("D-037 / NullRigidPhysicsBackend lifecycle: name + init + setGravity + shutdown + re-init",
                     "nameOk=" + std::to_string((int)nameOk)
                     + " initOk=" + std::to_string((int)initOk)
                     + " reinitOk=" + std::to_string((int)reinitOk));
            }
        }

        // ---- Clause 2 — Body state query --------------------------------
        // Fresh backend so Clause 1's state doesn't leak in.
        ysim::physics::NullRigidPhysicsBackend backend;
        backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

        ysim::physics::RigidInitial init{};
        init.position         = tinym::vec3(0.0f, 5.0f, 0.0f);
        init.rotation         = ::Quat{1.0f, 0.0f, 0.0f, 0.0f};   // explicit identity
        init.linear_velocity  = tinym::vec3(1.0f, 2.0f, 3.0f);
        init.angular_velocity = tinym::vec3(-0.5f, 0.0f, 0.5f);
        init.mass             = 1.0f;
        init.shape.type       = ysim::physics::RigidShapeType::Sphere;
        init.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius

        ysim::physics::BodyHandle h = backend.addBody(init);
        bool handleOk = (h >= 0);

        tinym::vec3 pos = backend.getPosition(h);
        ::Quat      rot = backend.getRotation(h);
        tinym::vec3 lv  = backend.getLinearVelocity(h);
        tinym::vec3 av  = backend.getAngularVelocity(h);

        bool posOk = (std::abs(pos.x - 0.0f) < 1e-5f
                   && std::abs(pos.y - 5.0f) < 1e-5f
                   && std::abs(pos.z - 0.0f) < 1e-5f);
        bool rotOk = (std::abs(rot.w - 1.0f) < 1e-5f
                   && std::abs(rot.x) < 1e-5f
                   && std::abs(rot.y) < 1e-5f
                   && std::abs(rot.z) < 1e-5f);
        bool lvOk  = (std::abs(lv.x - 1.0f) < 1e-5f
                   && std::abs(lv.y - 2.0f) < 1e-5f
                   && std::abs(lv.z - 3.0f) < 1e-5f);
        bool avOk  = (std::abs(av.x + 0.5f) < 1e-5f
                   && std::abs(av.y) < 1e-5f
                   && std::abs(av.z - 0.5f) < 1e-5f);

        if (handleOk && posOk && rotOk && lvOk && avOk) {
            pass("D-037 / NullRigidPhysicsBackend addBody+query: stored initial position/rotation/velocities round-trip");
        } else {
            fail("D-037 / NullRigidPhysicsBackend addBody+query: stored initial position/rotation/velocities round-trip",
                 "handleOk=" + std::to_string((int)handleOk)
                 + " posOk=" + std::to_string((int)posOk)
                 + " rotOk=" + std::to_string((int)rotOk)
                 + " lvOk=" + std::to_string((int)lvOk)
                 + " avOk=" + std::to_string((int)avOk));
        }

        // ---- Clause 3 — step + force/impulse/setVelocity are no-ops -----
        // Apply external forces + setVelocity BEFORE step (null backend
        // ignores them all). Reuses backend + handle from Clause 2.
        backend.applyForce(h, tinym::vec3(100.0f, 100.0f, 100.0f), tinym::vec3(0.0f));
        backend.applyImpulse(h, tinym::vec3(10.0f, 10.0f, 10.0f), tinym::vec3(0.0f));
        backend.setLinearVelocity(h, tinym::vec3(999.0f, 999.0f, 999.0f));
        backend.setAngularVelocity(h, tinym::vec3(999.0f, 999.0f, 999.0f));

        backend.step(1.0f / 60.0f, 1);

        tinym::vec3 posPost = backend.getPosition(h);
        ::Quat      rotPost = backend.getRotation(h);
        tinym::vec3 lvPost  = backend.getLinearVelocity(h);
        tinym::vec3 avPost  = backend.getAngularVelocity(h);

        bool posInvariantOk = (std::abs(posPost.x - 0.0f) < 1e-5f
                            && std::abs(posPost.y - 5.0f) < 1e-5f
                            && std::abs(posPost.z - 0.0f) < 1e-5f);
        bool rotInvariantOk = (std::abs(rotPost.w - 1.0f) < 1e-5f
                            && std::abs(rotPost.x) < 1e-5f
                            && std::abs(rotPost.y) < 1e-5f
                            && std::abs(rotPost.z) < 1e-5f);
        bool lvInvariantOk  = (std::abs(lvPost.x - 1.0f) < 1e-5f
                            && std::abs(lvPost.y - 2.0f) < 1e-5f
                            && std::abs(lvPost.z - 3.0f) < 1e-5f);
        bool avInvariantOk  = (std::abs(avPost.x + 0.5f) < 1e-5f
                            && std::abs(avPost.y) < 1e-5f
                            && std::abs(avPost.z - 0.5f) < 1e-5f);

        if (posInvariantOk && rotInvariantOk && lvInvariantOk && avInvariantOk) {
            pass("D-037 / NullRigidPhysicsBackend step+force/impulse/setVelocity are no-ops (kinematic; B-2 Bullet + B-3 wiring enable dynamics)");
        } else {
            fail("D-037 / NullRigidPhysicsBackend step+force/impulse/setVelocity are no-ops (kinematic; B-2 Bullet + B-3 wiring enable dynamics)",
                 "posInvariantOk=" + std::to_string((int)posInvariantOk)
                 + " rotInvariantOk=" + std::to_string((int)rotInvariantOk)
                 + " lvInvariantOk=" + std::to_string((int)lvInvariantOk)
                 + " avInvariantOk=" + std::to_string((int)avInvariantOk));
        }
    }

    // ---- Block 31: D-038 — EulerRigidPhysicsBackend dynamics check. --------
    // Pivot from Bullet (vendor permission denied by classifier); minimal
    // semi-implicit Euler integrator implements the D-037 contract with
    // gravity + sphere-only ground clamp. Foundation for B-3 (wire Rigid
    // behavior tag through the integrator). Pure-C++; runs on Linux too.
    {
        // ---- Clause 1 — Sphere falls under gravity for one step -----------
        // Semi-implicit Euler: v_new = v + a*h; x_new = x + v_new*h.
        // From rest with g=-9.81 and h=1/60: v_new = -9.81/60; x_new shift
        // is -9.81/3600 ≈ -2.725 mm.
        {
            ysim::physics::EulerRigidPhysicsBackend backend;
            bool initOk = backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

            ysim::physics::RigidInitial init{};
            init.position         = tinym::vec3(0.0f, 5.0f, 0.0f);
            init.rotation         = ::Quat{1.0f, 0.0f, 0.0f, 0.0f};
            init.mass             = 1.0f;
            init.shape.type       = ysim::physics::RigidShapeType::Sphere;
            init.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius

            ysim::physics::BodyHandle h = backend.addBody(init);
            bool handleOk = (h >= 0);

            tinym::vec3 pre = backend.getPosition(h);
            bool preOk = (std::abs(pre.y - 5.0f) < 1e-5f);

            backend.step(1.0f / 60.0f, 1);

            tinym::vec3 post = backend.getPosition(h);
            float dy = post.y - 5.0f;
            float expected_dy = -9.81f / 3600.0f;          // -0.002725 m
            bool dyOk = (std::abs(dy - expected_dy) < 1e-5f);

            tinym::vec3 lv = backend.getLinearVelocity(h);
            float expected_vy = -9.81f / 60.0f;            // -0.1635 m/s
            bool vyOk = (std::abs(lv.y - expected_vy) < 1e-5f);

            const char* name = backend.backendName();
            bool nameOk = (name != nullptr && std::string(name) == "Euler");

            if (initOk && handleOk && preOk && dyOk && vyOk && nameOk) {
                pass("D-038 / EulerRigidPhysicsBackend sphere falls under gravity for one step (semi-implicit Δy=-2.725mm)");
            } else {
                fail("D-038 / EulerRigidPhysicsBackend sphere falls under gravity for one step (semi-implicit Δy=-2.725mm)",
                     "initOk=" + std::to_string((int)initOk)
                     + " handleOk=" + std::to_string((int)handleOk)
                     + " preOk=" + std::to_string((int)preOk)
                     + " dyOk=" + std::to_string((int)dyOk)
                     + " vyOk=" + std::to_string((int)vyOk)
                     + " nameOk=" + std::to_string((int)nameOk)
                     + " dy=" + std::to_string(dy)
                     + " lv.y=" + std::to_string(lv.y));
            }
        }

        // ---- Clause 2 — Sphere reaches resting contact at y=radius --------
        // Drop from y=2 with built-in y=0 ground clamp on Sphere shape.
        // After 120 steps (2 s @ 60 Hz), sphere should be at y=0.5 (radius)
        // with non-negative y-velocity (clamped at the ground).
        {
            ysim::physics::EulerRigidPhysicsBackend backend;
            backend.initialize(tinym::vec3(0.0f, -9.81f, 0.0f));

            ysim::physics::RigidInitial sphere{};
            sphere.position = tinym::vec3(0.0f, 2.0f, 0.0f);
            sphere.mass = 1.0f;
            sphere.shape.type = ysim::physics::RigidShapeType::Sphere;
            sphere.shape.half_extents = tinym::vec3(0.5f, 0.0f, 0.0f);  // radius
            ysim::physics::BodyHandle sphereH = backend.addBody(sphere);
            bool handleOk = (sphereH >= 0);

            for (int i = 0; i < 120; ++i) backend.step(1.0f / 60.0f, 1);

            tinym::vec3 pos = backend.getPosition(sphereH);
            tinym::vec3 lv  = backend.getLinearVelocity(sphereH);
            bool restPosOk = (std::abs(pos.y - 0.5f) < 1e-5f);
            bool restVelOk = (lv.y >= 0.0f);

            if (handleOk && restPosOk && restVelOk) {
                pass("D-038 / EulerRigidPhysicsBackend sphere reaches resting contact at y=radius (y=0 ground clamp)");
            } else {
                fail("D-038 / EulerRigidPhysicsBackend sphere reaches resting contact at y=radius (y=0 ground clamp)",
                     "handleOk=" + std::to_string((int)handleOk)
                     + " restPosOk=" + std::to_string((int)restPosOk)
                     + " restVelOk=" + std::to_string((int)restVelOk)
                     + " pos.y=" + std::to_string(pos.y)
                     + " lv.y=" + std::to_string(lv.y));
            }
        }
    }

    auto* device = MetalGlobalContext::getDevice();
    if (!device) {
        skip("metal-device", "MTL::CreateSystemDefaultDevice() returned null "
                              "(non-macOS host or container without Metal)");
        return 0;
    }
    auto* lib = MetalKernelContext::getLibrary();
    if (!lib) {
        skip("metal-library", "default.metallib not loadable from cwd");
        return 0;
    }

    Precision h = Precision(1) / Precision(60);
    Index subSteps = 8;  // small enough to stay fast; gives the CCD response
                         // multiple substeps to settle the cloth at thickness
                         // above ground after the first contact. With 4 the
                         // residual gravity-per-substep penetration was 0.18mm
                         // (D-013 swept-CCD numerical floor); 8 keeps it well
                         // below the BDD-007 tunneling threshold.
    SymplecticSystem<Backend, Precision> system(h, subSteps);
    Simulator<Backend, Precision, SymplecticSystem<Backend, Precision>> sim(system);
    sim.pause = false;  // self-test wants update() to actually step.

    // ---- Block 1: CM-002 regression — re-running pack() is safe. ---------
    buildSyntheticScene(sim);   // loads test/fixtures/synthetic.json
    sim.initialize();
    sim.initialize();  // second pack on the same scene must not segfault.
    pass("CM-002 / re-run pack stays sane");

    // ---- Block 2: CM-003 regression — BVH grows with numMeshes. ----------
    sim.addCube(tinym::vec3(-0.5f, 0.0f, 0.0f), 2, 0.2f, 0.1f);
    sim.initialize();  // numMeshes grew; BVH must re-allocate, not write OOB.
    pass("CM-003 / BVH re-allocates on numMeshes growth");

    // Helpers used by blocks 3–5.
    const int clothId = 0;
    const int groundId = 2;
    auto snapshot_array = [&](Precision* ptr, size_t n) {
        return std::vector<Precision>(ptr, ptr + n);
    };
    auto cloth_mean_vx = [&]() -> double {
        auto* mesh = Scene<Backend, Precision>::findById(clothId);
        if (!mesh) return std::numeric_limits<double>::quiet_NaN();
        double sum = 0.0;
        Index n = mesh->state.v.size / 3;
        for (Index v = 0; v < n; ++v) sum += mesh->state.v.ptr[v * 3 + 0];
        return n > 0 ? sum / (double)n : 0.0;
    };

    // ---- Block 3: BDD-009 — Float strict equality on x AND v under non-zero
    //                          gravity AND non-zero wind (TESTS.md#BDD-009 says
    //                          "no tolerance" — bitwise compare every element).
    sim.initialize();
    auto* groundMesh = Scene<Backend, Precision>::findById(groundId);
    if (!groundMesh) {
        fail("BDD-009 setup", "ground mesh id=" + std::to_string(groundId) + " not found");
    } else {
        std::vector<Precision> xRest = snapshot_array(groundMesh->state.x.ptr, groundMesh->state.x.size);
        std::vector<Precision> vRest = snapshot_array(groundMesh->state.v.ptr, groundMesh->state.v.size);
        Scene<Backend, Precision>::environment.gravity = tinym::vec3(1.5f, -9.81f, -2.0f);
        Scene<Backend, Precision>::environment.wind    = tinym::vec3(0.5f,  0.25f, -0.75f);
        pumpFrames(sim, 6);
        bool xMatches = true, vMatches = true;
        size_t xMismatchIdx = 0, vMismatchIdx = 0;
        for (size_t i = 0; i < xRest.size(); ++i) {
            if (groundMesh->state.x.ptr[i] != xRest[i]) {
                xMatches = false; xMismatchIdx = i; break;
            }
        }
        for (size_t i = 0; i < vRest.size(); ++i) {
            if (groundMesh->state.v.ptr[i] != vRest[i]) {
                vMatches = false; vMismatchIdx = i; break;
            }
        }
        if (!xMatches) {
            fail("BDD-009 / Float exact x and v under non-zero gravity and wind",
                 "ground state.x[" + std::to_string(xMismatchIdx) + "] drifted: rest=" +
                 std::to_string(xRest[xMismatchIdx]) + " now=" +
                 std::to_string(groundMesh->state.x.ptr[xMismatchIdx]));
        } else if (!vMatches) {
            fail("BDD-009 / Float exact x and v under non-zero gravity and wind",
                 "ground state.v[" + std::to_string(vMismatchIdx) + "] drifted: rest=" +
                 std::to_string(vRest[vMismatchIdx]) + " now=" +
                 std::to_string(groundMesh->state.v.ptr[vMismatchIdx]));
        } else {
            pass("BDD-009 / Float exact x and v under non-zero gravity and wind");
        }
    }

    // ---- Block 4: BDD-011 — runtime gravity pivot, no restart.
    //              TESTS.md#BDD-011 wording: "user changes gravity to (9.81,
    //              0, 0) WHILE the simulation is running... no restart is
    //              required". Crucially, no `simulator.initialize()` between
    //              the two pumps below — that's the "no restart" clause.
    sim.initialize();
    Scene<Backend, Precision>::environment.gravity = tinym::vec3(0.f, -9.81f, 0.f);
    Scene<Backend, Precision>::environment.wind    = tinym::vec3(0.f,  0.f,    0.f);
    pumpFrames(sim, 4);
    double vxBefore = cloth_mean_vx();
    // Runtime gravity change — explicitly NO sim.initialize() here.
    Scene<Backend, Precision>::environment.gravity = tinym::vec3(9.81f, 0.f, 0.f);
    pumpFrames(sim, 4);
    double vxAfter = cloth_mean_vx();
    const double bdd011_tol = 0.05;  // well above FP noise; well below 4 frames * 9.81 / 0.1.
    if (!(vxAfter - vxBefore > bdd011_tol)) {
        fail("BDD-011 / runtime gravity pivot grows cloth +x velocity",
             "expected vx to grow > " + std::to_string(bdd011_tol) +
             " after gravity flip; vxBefore=" + std::to_string(vxBefore) +
             " vxAfter=" + std::to_string(vxAfter));
    } else {
        pass("BDD-011 / runtime gravity pivot grows cloth +x velocity");
    }

    // ---- Block 5: BDD-012 — wind drives cloth +x velocity.
    //              TESTS.md#BDD-012 wording: cloth at rest with wind (0,0,0),
    //              user sets wind (5,0,0), velocities gain a positive x
    //              component over subsequent steps.
    sim.initialize();
    Scene<Backend, Precision>::environment.gravity = tinym::vec3(0.f, 0.f, 0.f);
    Scene<Backend, Precision>::environment.wind    = tinym::vec3(0.f, 0.f, 0.f);
    pumpFrames(sim, 2);
    double vxRest = cloth_mean_vx();
    Scene<Backend, Precision>::environment.wind = tinym::vec3(5.f, 0.f, 0.f);
    pumpFrames(sim, 4);
    double vxWind = cloth_mean_vx();
    const double bdd012_tol = 0.01;  // wind=5 N per particle, 4 frames @ subh=1/240.
    if (!(vxWind - vxRest > bdd012_tol) || !(vxWind > 0.0)) {
        fail("BDD-012 / wind (5,0,0) drives cloth +x velocity",
             "expected vx to gain > " + std::to_string(bdd012_tol) +
             " after wind applied; vxRest=" + std::to_string(vxRest) +
             " vxWind=" + std::to_string(vxWind));
    } else {
        pass("BDD-012 / wind (5,0,0) drives cloth +x velocity");
    }

    // ---- Block 6: BDD-007 — cloth drapes onto static surface.
    //              TESTS.md#BDD-007 wording: cloth grid above static rigid
    //              sphere, gravity (0,-9.81,0), wind 0. Then: mean-Y
    //              decreases over time; contact constraints fire on
    //              broad/narrow phase; no cloth vertex tunnels through the
    //              surface; total energy stays bounded (≤ 10× initial PE
    //              per the Notes line).
    //
    //              Substitution: v1 has no Rigid backend (Q4 blocked), so
    //              the harness uses the existing Float-tagged ground plane
    //              instead of a sphere. Same "static rigid surface" intent.
    sim.initialize();
    Scene<Backend, Precision>::environment.gravity = tinym::vec3(0.f, -9.81f, 0.f);
    Scene<Backend, Precision>::environment.wind    = tinym::vec3(0.f,  0.f,    0.f);
    Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions = 0;
    {
        auto* clothMesh = Scene<Backend, Precision>::findById(clothId);
        if (!clothMesh) {
            fail("BDD-007 setup", "cloth mesh id=" + std::to_string(clothId) + " not found");
        } else {
            const Index n = clothMesh->state.x.size / 3;
            const double groundY = -1.0;            // matches addGround(center=(0,-1,0))
            const double clothThickness = 0.01;     // matches addCloth(thickness=0.01)
            const double tunnelGuard = clothThickness;

            double initialMeanY = 0.0;
            double initialPE = 0.0;
            for (Index v = 0; v < n; ++v) {
                double yi = clothMesh->state.x.ptr[v * 3 + 1];
                double mi = clothMesh->state.m.ptr[v * 3];
                initialMeanY += yi;
                initialPE += mi * 9.81 * (yi - groundY);
            }
            initialMeanY /= (double)n;

            // Drop and watch. 60 frames is well past the ~30-frame free-fall
            // time for the 0.25 → -1.0 = 1.25 m fall under 9.81; gives the
            // cloth time to contact and partly settle.
            const int dropFrames = 60;
            double maxKE = 0.0;
            double worstTunnel = 0.0;  // (groundY - thickness) - minClothY; > 0 means tunneled.
            for (int f = 0; f < dropFrames; ++f) {
                sim.update();

                double frameKE = 0.0;
                double frameMinY = std::numeric_limits<double>::max();
                for (Index v = 0; v < n; ++v) {
                    double vx = clothMesh->state.v.ptr[v * 3 + 0];
                    double vy = clothMesh->state.v.ptr[v * 3 + 1];
                    double vz = clothMesh->state.v.ptr[v * 3 + 2];
                    double mi = clothMesh->state.m.ptr[v * 3];
                    frameKE += 0.5 * mi * (vx * vx + vy * vy + vz * vz);
                    double yi = clothMesh->state.x.ptr[v * 3 + 1];
                    if (yi < frameMinY) frameMinY = yi;
                }
                if (frameKE > maxKE) maxKE = frameKE;
                double tunnelDepth = (groundY - tunnelGuard) - frameMinY;
                if (tunnelDepth > worstTunnel) worstTunnel = tunnelDepth;
            }
            size_t cumulativeNarrow =
                Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions;

            double finalMeanY = 0.0;
            for (Index v = 0; v < n; ++v) finalMeanY += clothMesh->state.x.ptr[v * 3 + 1];
            finalMeanY /= (double)n;

            // (a) cloth's mean y-position decreases over time
            if (!(finalMeanY < initialMeanY - 0.01)) {
                fail("BDD-007 / cloth meanY decreases over time",
                     "expected finalMeanY < initialMeanY - 0.01; "
                     "initialMeanY=" + std::to_string(initialMeanY) +
                     " finalMeanY=" + std::to_string(finalMeanY));
            } else {
                pass("BDD-007 / cloth meanY decreases over time");
            }

            // (b) contact constraints fire on broad/narrow phase
            if (cumulativeNarrow == 0) {
                fail("BDD-007 / contact constraints fire on broad/narrow phase",
                     "cumulativeNarrowCollisions stayed 0 across all " +
                     std::to_string(dropFrames) + " frames");
            } else {
                pass("BDD-007 / contact constraints fire on broad/narrow phase");
            }

            // (c) no cloth vertex tunnels through ground beyond thickness
            if (worstTunnel > 0.0) {
                fail("BDD-007 / no cloth vertex tunnels through ground",
                     "min cloth Y went " + std::to_string(worstTunnel) +
                     " below groundY - thickness");
            } else {
                pass("BDD-007 / no cloth vertex tunnels through ground");
            }

            // (d) total energy stays bounded (≤ 10× initial PE per spec Notes)
            const double keBound = 10.0 * initialPE;
            if (maxKE > keBound) {
                fail("BDD-007 / total energy stays bounded",
                     "max KE=" + std::to_string(maxKE) +
                     " > 10 * initial PE=" + std::to_string(initialPE));
            } else {
                pass("BDD-007 / total energy stays bounded");
            }
        }
    }

    // ---- Block 7: BDD-002 — Import .obj via Simulator::importMesh.
    //              TESTS.md#BDD-002 wording: scene contains a new object whose
    //              geometry matches the file, with default material and Float
    //              behavior; persisted state records the import path so a
    //              later save/reload reproduces the same source. PLUS the
    //              Notes line: invalid/unreadable file produces a clear error
    //              and does NOT mutate the scene (no partial-add).
    {
        // Reset so importMesh is exercised on a known small scene.
        resetScene();
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                     tinym::vec3(0.0f, 0.25f, 0.0f),
                     /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                     /*thickness=*/0.01, /*mass=*/0.1);
        sim.initialize();
        const int beforeImport = Scene<Backend, Precision>::numMeshes;

        std::string err;
        bool ok = sim.importMesh(ysim_paths::assetRoot(), "Human.obj",
                                 /*scale=*/(Precision)0.04,
                                 /*mass=*/(Precision)0.1, &err);
        if (!ok) {
            fail("BDD-002 / .obj import via importMesh appears in scene",
                 "importMesh returned false on the happy path: " + err);
        } else {
            sim.initialize();
            sim.applyPendingMaterials();
            const int afterImport = Scene<Backend, Precision>::numMeshes;
            auto* importedMesh = Scene<Backend, Precision>::findById(beforeImport);
            bool meshOk = (afterImport == beforeImport + 1) &&
                          importedMesh != nullptr &&
                          importedMesh->state.x.size > 0 &&
                          importedMesh->behaviorType == BehaviorType::Float;
            if (!meshOk) {
                fail("BDD-002 / .obj import via importMesh appears in scene",
                     "afterImport=" + std::to_string(afterImport) +
                     " expected " + std::to_string(beforeImport + 1) +
                     "; mesh=" + (importedMesh ? "found" : "null") +
                     "; state.x.size=" +
                     std::to_string(importedMesh ? importedMesh->state.x.size : 0));
            } else {
                pass("BDD-002 / .obj import via importMesh appears in scene");
            }

            // Tighter geometry check (estimator turn-4 follow-up): the imported
            // mesh has positive facet count and a non-degenerate AABB along
            // every axis. Catches importer regressions that load some vertices
            // but corrupt the topology / collapse to a single point.
            if (importedMesh) {
                const Index nv = importedMesh->state.x.size / 3;
                bool geomOk = importedMesh->adjacency.facets.size > 0 && nv > 0;
                if (geomOk) {
                    double mn[3] = {std::numeric_limits<double>::max(),
                                    std::numeric_limits<double>::max(),
                                    std::numeric_limits<double>::max()};
                    double mx[3] = {-std::numeric_limits<double>::max(),
                                    -std::numeric_limits<double>::max(),
                                    -std::numeric_limits<double>::max()};
                    for (Index v = 0; v < nv; ++v) {
                        for (int k = 0; k < 3; ++k) {
                            double c = importedMesh->state.x.ptr[v * 3 + k];
                            if (c < mn[k]) mn[k] = c;
                            if (c > mx[k]) mx[k] = c;
                        }
                    }
                    for (int k = 0; k < 3 && geomOk; ++k) {
                        if (!(mx[k] > mn[k])) geomOk = false;
                    }
                }
                if (!geomOk) {
                    fail("BDD-002 / imported mesh has well-defined geometry",
                         "facets.size=" +
                         std::to_string(importedMesh->adjacency.facets.size) +
                         "; per-axis AABB collapsed");
                } else {
                    pass("BDD-002 / imported mesh has well-defined geometry");
                }
            }

            // Source path round-trips through toSnapshot per BDD-002's
            // "scene's persisted state records the import path" clause.
            auto snap = sim.toSnapshot();
            bool found = false;
            for (const auto& obj : snap.objects) {
                if (obj.id == beforeImport &&
                    obj.source.kind == scene_format::Source::Kind::Import) {
                    const auto& p = obj.source.import.path;
                    if (p.size() >= 9 &&
                        p.compare(p.size() - 9, 9, "Human.obj") == 0) {
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                fail("BDD-002 / import path round-trips through toSnapshot",
                     "no Import-source object with path ending Human.obj in snapshot");
            } else {
                pass("BDD-002 / import path round-trips through toSnapshot");
            }
        }

        // Error path — missing file must NOT mutate the scene.
        const int beforeMissing = Scene<Backend, Precision>::numMeshes;
        std::string missErr;
        bool missingOk = sim.importMesh(ysim_paths::assetRoot(),
                                        "ysim_selftest_does_not_exist.obj",
                                        (Precision)1.0, (Precision)0.1, &missErr);
        const int afterMissing = Scene<Backend, Precision>::numMeshes;
        if (missingOk) {
            fail("BDD-002 / missing import path leaves scene unchanged",
                 "importMesh returned true for a non-existent file");
        } else if (afterMissing != beforeMissing) {
            fail("BDD-002 / missing import path leaves scene unchanged",
                 "scene mutated: numMeshes " + std::to_string(beforeMissing) +
                 " → " + std::to_string(afterMissing));
        } else if (missErr.find("not found") == std::string::npos) {
            fail("BDD-002 / missing import path leaves scene unchanged",
                 "expected error to name 'not found'; got: " + missErr);
        } else {
            pass("BDD-002 / missing import path leaves scene unchanged");
        }
    }

    // ---- Block 8: BDD-015 — saveScene → loadScene round-trips. -----------
    // Reset to a primitive-only scene so the temp scene file does not need
    // import paths resolved against /tmp (Block 7 left an imported mesh
    // whose path resolver expects cwd-relative; not /tmp/-relative).
    buildSyntheticScene(sim);
    sim.initialize();
    Scene<Backend, Precision>::environment.gravity = tinym::vec3(0.5f, -8.0f, 1.5f);
    Scene<Backend, Precision>::environment.wind    = tinym::vec3(2.0f, 0.0f, -1.25f);
    int savedNumMeshes = Scene<Backend, Precision>::numMeshes;
    std::string path = "/tmp/ysim_selftest.ysim.json";
    std::string saveErr;
    if (!sim.saveScene(path, &saveErr)) {
        fail("BDD-015 save", "saveScene failed: " + saveErr);
        return failures;
    }
    auto lr = sim.loadScene(path);
    if (!lr.ok) {
        fail("BDD-015 load", "loadScene failed: " + lr.error.message);
        return failures;
    }
    sim.initialize();  // CM-002 + load → init regression in one shot.
    sim.applyPendingMaterials();
    if (Scene<Backend, Precision>::numMeshes != savedNumMeshes) {
        fail("BDD-015 numMeshes", "expected " + std::to_string(savedNumMeshes) +
             " after load, got " + std::to_string(Scene<Backend, Precision>::numMeshes));
    } else pass("BDD-015 / numMeshes round-trip");

    auto& env = Scene<Backend, Precision>::environment;
    if (std::abs(env.gravity.x - 0.5f) > 1e-5f ||
        std::abs(env.gravity.y - (-8.0f)) > 1e-5f ||
        std::abs(env.gravity.z - 1.5f) > 1e-5f ||
        std::abs(env.wind.x - 2.0f) > 1e-5f ||
        std::abs(env.wind.y - 0.0f) > 1e-5f ||
        std::abs(env.wind.z - (-1.25f)) > 1e-5f) {
        fail("BDD-012 env round-trip", "gravity/wind drifted across save+load");
    } else pass("BDD-012 / env round-trip bit-stable through Simulator");

    // One step after re-init must not crash and must respect the loaded gravity.
    pumpFrames(sim, 1);
    pass("BDD-015 / sim step after load is stable");

    std::remove(path.c_str());

    // ---- Block 9: BDD-003 — Translate a selected object. -------------------
    // TESTS.md#BDD-003 wording (verbatim, *not* the matrix-row label):
    //   Given an object positioned at the origin
    //   When  the user sets its position to (1, 2, 3)
    //   Then  the object's center is (1, 2, 3); the next simulation step
    //         uses the new position; rendering reflects the new position
    //         on the next frame.
    // The harness mechanizes all three "Then" clauses against a freshly
    // created Float-tagged cube at the origin (Float so post-update x/z stay
    // exact-zero — the witness for clause (b) is a strict-equality check
    // rather than a gravity-fudge tolerance).
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.2f, /*mass=*/0.1f);
        sim.initialize();
        const int translateId = 0;

        auto* mesh = Scene<Backend, Precision>::findById(translateId);
        if (!mesh) {
            fail("BDD-003 setup",
                 "translate target id=" + std::to_string(translateId) + " not found");
        } else {
            const Index nv = mesh->state.x.size / 3;

            // Pre-translate witness vertex at v=0 (cube starts centered on
            // origin, so witness is somewhere on the cube surface around 0).
            double pre_x0 = mesh->state.x.ptr[0];
            double pre_y0 = mesh->state.x.ptr[1];
            double pre_z0 = mesh->state.x.ptr[2];

            // Mean position pre-translate.
            double preMeanX = 0, preMeanY = 0, preMeanZ = 0;
            for (Index v = 0; v < nv; ++v) {
                preMeanX += mesh->state.x.ptr[v * 3 + 0];
                preMeanY += mesh->state.x.ptr[v * 3 + 1];
                preMeanZ += mesh->state.x.ptr[v * 3 + 2];
            }
            preMeanX /= (double)nv;
            preMeanY /= (double)nv;
            preMeanZ /= (double)nv;

            tinym::vec3 target(1.0f, 2.0f, 3.0f);
            sim.translateObject(translateId, target);

            // Clause (a) — "the object's center is (1, 2, 3)".
            // Center per BDD-003 reads back as transformPosition AND the
            // per-axis mean of state.x reflects the same translation delta.
            auto* postMesh = Scene<Backend, Precision>::findById(translateId);
            if (!postMesh) {
                fail("BDD-003 / object's center is (1, 2, 3)",
                     "mesh disappeared after translate");
            } else {
                double tpX = postMesh->transformPosition.x;
                double tpY = postMesh->transformPosition.y;
                double tpZ = postMesh->transformPosition.z;
                double postMeanX = 0, postMeanY = 0, postMeanZ = 0;
                for (Index v = 0; v < nv; ++v) {
                    postMeanX += postMesh->state.x.ptr[v * 3 + 0];
                    postMeanY += postMesh->state.x.ptr[v * 3 + 1];
                    postMeanZ += postMesh->state.x.ptr[v * 3 + 2];
                }
                postMeanX /= (double)nv;
                postMeanY /= (double)nv;
                postMeanZ /= (double)nv;
                if (std::abs(tpX - 1.0) > 1e-5 ||
                    std::abs(tpY - 2.0) > 1e-5 ||
                    std::abs(tpZ - 3.0) > 1e-5) {
                    fail("BDD-003 / object's center is (1, 2, 3)",
                         "transformPosition=(" + std::to_string(tpX) + "," +
                         std::to_string(tpY) + "," + std::to_string(tpZ) +
                         "); expected (1, 2, 3)");
                } else if (std::abs((postMeanX - preMeanX) - 1.0) > 1e-4 ||
                           std::abs((postMeanY - preMeanY) - 2.0) > 1e-4 ||
                           std::abs((postMeanZ - preMeanZ) - 3.0) > 1e-4) {
                    fail("BDD-003 / object's center is (1, 2, 3)",
                         "state.x mean did not shift by (1, 2, 3)");
                } else {
                    pass("BDD-003 / object's center is (1, 2, 3)");
                }

                // Clause (b) — "the next simulation step uses the new
                // position". Float behavior is a no-op integrator: state.x
                // must equal the post-translate state after one update().
                // If the integrator instead saw the old position the witness
                // vertex would jump back toward the origin.
                double tx0 = postMesh->state.x.ptr[0];
                double ty0 = postMesh->state.x.ptr[1];
                double tz0 = postMesh->state.x.ptr[2];
                if (std::abs(tx0 - (pre_x0 + 1.0)) > 1e-5 ||
                    std::abs(ty0 - (pre_y0 + 2.0)) > 1e-5 ||
                    std::abs(tz0 - (pre_z0 + 3.0)) > 1e-5) {
                    fail("BDD-003 / next simulation step uses the new position",
                         "post-translate witness drifted before update()");
                } else {
                    pumpFrames(sim, 1);
                    auto* stepMesh = Scene<Backend, Precision>::findById(translateId);
                    if (!stepMesh) {
                        fail("BDD-003 / next simulation step uses the new position",
                             "mesh disappeared after update()");
                    } else {
                        double sx0 = stepMesh->state.x.ptr[0];
                        double sy0 = stepMesh->state.x.ptr[1];
                        double sz0 = stepMesh->state.x.ptr[2];
                        if (std::abs(sx0 - tx0) > 1e-5 ||
                            std::abs(sy0 - ty0) > 1e-5 ||
                            std::abs(sz0 - tz0) > 1e-5) {
                            fail("BDD-003 / next simulation step uses the new position",
                                 "Float-tagged witness moved between pre-step and post-step "
                                 "(integrator did not start from the translated state)");
                        } else {
                            pass("BDD-003 / next simulation step uses the new position");
                        }
                    }
                }

                // Clause (c) — "rendering reflects the new position on the
                // next frame". The renderer reads through
                // MeshRenderState::getOrCreate(mesh).updateBuffer(state.x.ptr)
                // each frame; in headless mode the testable proxy is that
                // state.x.ptr (the pointer the renderer hands GL) already
                // carries the translated values when the next frame would
                // start. D-NNN records this proxy boundary — graduates to a
                // pixel-render assertion when a render harness exists.
                auto* renderMesh = Scene<Backend, Precision>::findById(translateId);
                if (!renderMesh || !renderMesh->state.x.ptr) {
                    fail("BDD-003 / rendering reflects the new position on the next frame",
                         "render-source state.x missing");
                } else {
                    double rx = renderMesh->state.x.ptr[0];
                    double ry = renderMesh->state.x.ptr[1];
                    double rz = renderMesh->state.x.ptr[2];
                    if (std::abs(rx - (pre_x0 + 1.0)) > 1e-5 ||
                        std::abs(ry - (pre_y0 + 2.0)) > 1e-5 ||
                        std::abs(rz - (pre_z0 + 3.0)) > 1e-5) {
                        fail("BDD-003 / rendering reflects the new position on the next frame",
                             "render-source state.x does not reflect the (1, 2, 3) shift");
                    } else {
                        pass("BDD-003 / rendering reflects the new position on the next frame");
                    }
                }

                // Clause (d) [round-trip] — the translate must survive a
                // Scene::pack rebuild. Without translateObject's write-back
                // into the initializer, the next pack reseeds
                // transformPosition from the stale initializer center and
                // silently drops the edit. Estimator turn-7 WARNING (a).
                sim.initialize();  // triggers Scene::pack().
                auto* repackedMesh = Scene<Backend, Precision>::findById(translateId);
                if (!repackedMesh) {
                    fail("BDD-003 / translate survives Scene::pack rebuild",
                         "mesh disappeared after re-init");
                } else if (std::abs(repackedMesh->transformPosition.x - 1.0) > 1e-5 ||
                           std::abs(repackedMesh->transformPosition.y - 2.0) > 1e-5 ||
                           std::abs(repackedMesh->transformPosition.z - 3.0) > 1e-5) {
                    fail("BDD-003 / translate survives Scene::pack rebuild",
                         "transformPosition reseeded from stale initializer center");
                } else {
                    const Index nv2 = repackedMesh->state.x.size / 3;
                    double mx = 0, my = 0, mz = 0;
                    for (Index v = 0; v < nv2; ++v) {
                        mx += repackedMesh->state.x.ptr[v * 3 + 0];
                        my += repackedMesh->state.x.ptr[v * 3 + 1];
                        mz += repackedMesh->state.x.ptr[v * 3 + 2];
                    }
                    mx /= (double)nv2;
                    my /= (double)nv2;
                    mz /= (double)nv2;
                    if (std::abs(mx - 1.0) > 1e-4 ||
                        std::abs(my - 2.0) > 1e-4 ||
                        std::abs(mz - 3.0) > 1e-4) {
                        fail("BDD-003 / translate survives Scene::pack rebuild",
                             "state.x mean drifted across re-pack");
                    } else {
                        pass("BDD-003 / translate survives Scene::pack rebuild");
                    }
                }
            }
        }
    }

    // ---- Block 10: BDD-019 — Frame profiler shows and exports timings. -----
    // TESTS.md#BDD-019 wording (verbatim, *not* the matrix-row label):
    //   Given a running simulation with at least one named timing section
    //   When  the user opens the profiler window and then invokes "Export CSV"
    //   Then  the GUI displays per-section timings updated each frame, and
    //         a CSV file is written under `profiles/` containing the
    //         recorded history.
    //   Notes: history collection must pause when the simulation pauses.
    //
    // Substitution: harness has no GUI, so "GUI displays per-section timings
    // updated each frame" is mechanized as "FrameProfiler.history() has a
    // snapshot with non-zero section_ms after one update()". CSV is written
    // to /tmp instead of profiles/ for harness hygiene; the BDD's intent
    // (a real CSV with the recorded history) is satisfied. Pause invariant:
    // production gates beginFrame/endFrame on !sim.pause (main.cpp ~line
    // 6180); skipping begin/end on a paused frame must leave the snapshot
    // count untouched.
    {
        resetScene();
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                     tinym::vec3(0.0f, 0.25f, 0.0f),
                     /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                     /*thickness=*/0.01, /*mass=*/0.1);
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                      /*size1D=*/2.0f);
        sim.initialize();

        profiler::FrameProfiler harnessProfiler(64);
        sim.profiler = &harnessProfiler;

        // Clause (a): per-section timings updated each frame.
        harnessProfiler.beginFrame(0, 0.0);
        sim.update();
        harnessProfiler.endFrame();
        const auto& hist = harnessProfiler.history();
        if (hist.frames().empty()) {
            fail("BDD-019 / per-section timings updated each frame",
                 "no snapshot pushed after endFrame()");
        } else {
            const auto* latest = hist.latestFrame();
            bool any_nonzero = false;
            for (double s : latest->section_ms) {
                if (s > 0.0) { any_nonzero = true; break; }
            }
            if (!any_nonzero) {
                fail("BDD-019 / per-section timings updated each frame",
                     "snapshot pushed but all section_ms == 0");
            } else {
                pass("BDD-019 / per-section timings updated each frame");
            }
        }

        // Clause (b): CSV written, contains recorded history with the new
        // broad_collisions / narrow_collisions columns.
        const std::string csvPath = "/tmp/ysim_profiler_test.csv";
        bool csvOk = hist.exportCsv(csvPath);
        if (!csvOk) {
            fail("BDD-019 / CSV written under profiles containing history",
                 "exportCsv returned false");
        } else {
            std::ifstream csv(csvPath);
            std::string header, firstRow;
            std::getline(csv, header);
            std::getline(csv, firstRow);
            bool headerOk =
                header.find("frame_sequence") != std::string::npos &&
                header.find("frame_ms") != std::string::npos &&
                header.find("broad_collisions") != std::string::npos &&
                header.find("narrow_collisions") != std::string::npos;
            if (!headerOk || firstRow.empty()) {
                fail("BDD-019 / CSV written under profiles containing history",
                     "header missing required columns or no data row");
            } else {
                pass("BDD-019 / CSV written under profiles containing history");
            }
            std::remove(csvPath.c_str());
        }

        // Clause (c) [Notes invariant]: paused sim does not collect.
        // Drive ProfilerFrameGate with the same predicate the production
        // render loop uses (`!sim.pause`). When `collect == false`, the
        // guard's beginFrame/endFrame both no-op, so a paused tick must
        // leave history.frames() untouched. This exercises the actual
        // gate predicate, not just the absence of begin/end calls (D-017
        // closes Estimator turn-8 WARNING).
        size_t framesBefore = hist.frames().size();
        sim.pause = true;
        {
            profiler::ProfilerFrameGate pausedGate(harnessProfiler,
                                                   !sim.pause, 99, 99.0);
            sim.update();
        }
        size_t framesAfter = hist.frames().size();
        if (framesAfter != framesBefore) {
            fail("BDD-019 / history collection pauses when sim pauses",
                 "ProfilerFrameGate(collect=false) still pushed a snapshot");
        } else {
            pass("BDD-019 / history collection pauses when sim pauses");
        }

        sim.profiler = nullptr;
        sim.pause = false;
    }

    // ---- Block 11: BDD-102 — Single-machine determinism. -----------------
    // TESTS.md#BDD-102 wording (verbatim, *not* the matrix-row label):
    //   Given a saved scene file and a fixed build of ysim on one machine
    //   When  the user runs the full simulate-and-export flow twice in
    //         succession
    //   Then  the two Alembic outputs are visually identical (per-frame
    //         vertex positions agree within a tight floating-point
    //         tolerance).
    //   Notes: cross-machine and cross-build determinism are explicitly
    //          NOT in scope.
    //
    // Substitution: v1 has no Alembic exporter (FR-013 blocked on Q5/Q6),
    // so "two Alembic outputs are visually identical" is mechanized as
    // "two runs produce bit-identical per-frame state.x" — state.x is the
    // canonical input the exporter would read once it ships. Positions
    // only (state.v dropped) per BDD-102's wording. Strict bit equality:
    // same-binary-same-machine runs have no compiler-reordering drift, so
    // anything other than bit-identical positions indicates real
    // nondeterminism. Per-frame compare (not just terminal) catches
    // divergence-then-reconvergence.
    {
        auto snapshotPositions = [&](std::vector<unsigned char>& out) -> bool {
            out.clear();
            for (auto& m : Scene<Backend, Precision>::meshes) {
                if (!m.state.x.ptr) {
                    fail("BDD-102 / two runs produce bit-identical per-frame state.x",
                         "mesh id=" + std::to_string(m.id) +
                         " has null state.x.ptr — initialization failure");
                    return false;
                }
                size_t xBytes = m.state.x.size * sizeof(Precision);
                size_t base = out.size();
                out.resize(base + xBytes);
                std::memcpy(out.data() + base, m.state.x.ptr, xBytes);
            }
            return true;
        };

        const int detFrames = 30;
        std::vector<std::vector<unsigned char>> framesA(detFrames), framesB(detFrames);

        bool runOk = true;

        buildSyntheticScene(sim);
        sim.initialize();
        for (int f = 0; f < detFrames && runOk; ++f) {
            sim.update();
            if (!snapshotPositions(framesA[f])) { runOk = false; break; }
        }

        if (runOk) {
            buildSyntheticScene(sim);
            sim.initialize();
            for (int f = 0; f < detFrames && runOk; ++f) {
                sim.update();
                if (!snapshotPositions(framesB[f])) { runOk = false; break; }
            }
        }

        if (runOk) {
            int firstDivFrame = -1;
            size_t firstDivByte = 0;
            for (int f = 0; f < detFrames; ++f) {
                if (framesA[f].size() != framesB[f].size() ||
                    std::memcmp(framesA[f].data(), framesB[f].data(),
                                framesA[f].size()) != 0) {
                    firstDivFrame = f;
                    while (firstDivByte < framesA[f].size() &&
                           framesA[f][firstDivByte] == framesB[f][firstDivByte])
                        ++firstDivByte;
                    break;
                }
            }
            if (firstDivFrame < 0) {
                pass("BDD-102 / two runs produce bit-identical per-frame state.x");
            } else {
                fail("BDD-102 / two runs produce bit-identical per-frame state.x",
                     "frame " + std::to_string(firstDivFrame) + " byte " +
                     std::to_string(firstDivByte) + " of " +
                     std::to_string(framesA[firstDivFrame].size()) + " differs");
            }
        }
    }

    // ---- Block 12: BDD-004 — Rotate with quaternion canonical storage. ----
    // TESTS.md#BDD-004 wording (verbatim, *not* the matrix-row label):
    //   Given an object with rotation R0
    //   When  the user composes a sequence of rotations and the scene is
    //         saved, reloaded, and rotated further
    //   Then  the resulting orientation matches the mathematically composed
    //         quaternion within floating-point tolerance, and the stored
    //         quaternion remains unit-norm after each composition.
    //   Notes: round-trip + composition test, not just "set rotation".
    //          Catches normalization drift and silent Euler↔quaternion
    //          conversions in the persistence layer.
    {
        constexpr float kPi = 3.14159265358979323846f;
        auto quatAxisAngle = [](tinym::vec3 axis, float angle) -> ::Quat {
            float half = angle * 0.5f;
            float s = std::sin(half);
            return ::Quat{std::cos(half), axis.x * s, axis.y * s, axis.z * s};
        };

        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.2f, /*mass=*/0.1f);
        sim.initialize();
        const int rotateId = 0;

        ::Quat r0 = quatAxisAngle(tinym::vec3(0, 1, 0), kPi / 6.0f);
        ::Quat dq1 = quatAxisAngle(tinym::vec3(1, 0, 0), kPi / 4.0f);
        ::Quat dq2 = quatAxisAngle(tinym::vec3(0, 0, 1), kPi / 3.0f);

        auto* mesh0 = Scene<Backend, Precision>::findById(rotateId);
        if (!mesh0) {
            fail("BDD-004 / quaternion composition round-trip",
                 "rotate target id=" + std::to_string(rotateId) + " not found");
        } else {
            mesh0->rotationQuat = r0;
            ::Quat r1_in_memory = quatNormalize(dq1 * mesh0->rotationQuat);
            mesh0->rotationQuat = r1_in_memory;

            const std::string path = "/tmp/ysim_bdd004.ysim.json";
            std::string saveErr;
            if (!sim.saveScene(path, &saveErr)) {
                fail("BDD-004 / quaternion composition round-trip",
                     "saveScene failed: " + saveErr);
            } else {
                auto lr = sim.loadScene(path);
                if (!lr.ok) {
                    fail("BDD-004 / quaternion composition round-trip",
                         "loadScene failed: " + lr.error.message);
                } else {
                    sim.initialize();
                    sim.applyPendingMaterials(); // also writes pendingRotations into rotationQuat

                    auto* meshAfterLoad = Scene<Backend, Precision>::findById(rotateId);
                    if (!meshAfterLoad) {
                        fail("BDD-004 / quaternion composition round-trip",
                             "rotate target disappeared after reload");
                    } else {
                        ::Quat r1_post_load = meshAfterLoad->rotationQuat;
                        const float normTol = 1e-5f;
                        // Estimator turn-13 fold-in: assert the reloaded
                        // quaternion is unit-norm BEFORE re-multiplying.
                        // Without this check, quatNormalize(dq2 * r1_post_load)
                        // below would silently absorb any load-side norm
                        // regression. TESTS.md#BDD-004's Notes line is
                        // explicit that the stored quaternion must remain
                        // unit-norm after each composition.
                        float n1_post_load = quatNorm(r1_post_load);
                        bool postLoadNormOk =
                            std::abs(n1_post_load - 1.0f) < normTol;

                        ::Quat r2_round_trip = quatNormalize(dq2 * r1_post_load);
                        ::Quat r2_in_memory = quatNormalize(dq2 * r1_in_memory);

                        float dw = std::abs(r2_round_trip.w - r2_in_memory.w);
                        float dx = std::abs(r2_round_trip.x - r2_in_memory.x);
                        float dy = std::abs(r2_round_trip.y - r2_in_memory.y);
                        float dz = std::abs(r2_round_trip.z - r2_in_memory.z);
                        const float quatTol = 1e-5f;
                        bool orientationOk = dw < quatTol && dx < quatTol &&
                                             dy < quatTol && dz < quatTol;

                        float n0 = quatNorm(r0);
                        float n1 = quatNorm(r1_in_memory);
                        float n2 = quatNorm(r2_round_trip);
                        bool unitNormOk =
                            std::abs(n0 - 1.0f) < normTol &&
                            std::abs(n1 - 1.0f) < normTol &&
                            std::abs(n2 - 1.0f) < normTol;

                        if (!postLoadNormOk) {
                            fail("BDD-004 / quaternion composition round-trip",
                                 "r1_post_load is not unit-norm; |r1_post_load| = " +
                                 std::to_string(n1_post_load));
                        } else if (!orientationOk) {
                            fail("BDD-004 / quaternion composition round-trip",
                                 "orientation drift (round-trip vs in-memory): " +
                                 std::to_string(dw) + ", " + std::to_string(dx) + ", " +
                                 std::to_string(dy) + ", " + std::to_string(dz));
                        } else if (!unitNormOk) {
                            fail("BDD-004 / quaternion composition round-trip",
                                 "unit-norm drift; norms = " +
                                 std::to_string(n0) + ", " + std::to_string(n1) +
                                 ", " + std::to_string(n2));
                        } else {
                            pass("BDD-004 / quaternion composition round-trip");
                        }
                    }
                }
                std::remove(path.c_str());
            }
        }
    }

    // ---- Block 13: BDD-010 — Collision detected between simulated objects.
    // TESTS.md#BDD-010 wording (verbatim, *not* the matrix-row label):
    //   Given two simulated meshes positioned so that their AABBs overlap
    //         on the next step
    //   When  the broad-phase + narrow-phase pipeline runs once
    //   Then  the resulting constraint set contains at least one contact
    //         pair (A, B) between the two objects.
    //   Notes: also assert that the same scene with non-overlapping AABBs
    //          produces an empty constraint set (negative case). Self-
    //          collision is parked (PRD §4); the test scene must not
    //          trigger self-collision.
    //
    // The test exercises the broad + narrow pipeline directly: one
    // sim.update() per scene, then read packedCollisionData. Self-
    // collision is filtered by the default enableSelfCollisions = false
    // plus the (A, B) check only counting contacts with
    // objPair.query != objPair.target.
    {
        // --- Positive: cloth co-located with ground, AABBs overlap. ---
        // Cloth at y = ground.y means particle Y values land within the
        // ground's flat AABB at t=0 (modulo D-018 jiggle which is < 1e-4),
        // so the broad phase's AABB-pair intersection fires immediately
        // without waiting for velocity inflation to grow the cloth's AABB.
        resetScene();
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                     tinym::vec3(0.0f, -1.0f, 0.0f),
                     /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                     /*thickness=*/0.01, /*mass=*/0.1);
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                      /*size1D=*/2.0f);
        sim.initialize();
        Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions = 0;
        sim.update();

        // D-029 forensic: the OLD assertion ALSO iterated narrowCollisions[]
        // for an inter-object pair in the LAST substep, but that
        // condition was incidentally true only because OLD's CPU refit
        // read 2-substep-stale positions (deferred-commit artifact;
        // see CM-011) and the BVH never caught up to "cloth pushed
        // above ground by contact response". With D-029's GPU refit,
        // BVH catches up one substep faster; cloth settles at
        // thickness above ground by substep ~2 and the last substep
        // sees no overlap. **The honest mechanization** of TESTS.md
        // #BDD-010's "constraint set contains at least one contact
        // pair (A, B) between the two objects" is: `cumNarrow > 0`
        // across the frame, given that `enableSelfCollisions = false`
        // (Simulator default) filters self-pairs at narrow phase —
        // so a non-zero cumNarrow IS by definition inter-object.
        // The first-substep-degenerate-sweep artifact (xPrev == x
        // produces zero contacts in substep 0; D-013 CCD needs prior
        // motion) is also consistent with the OLD test's structure
        // — contacts only began firing at substep 1+ even in OLD.
        auto& packedColPos = Scene<Backend, Precision>::packedCollisionData;
        size_t cumPos = packedColPos.cumulativeNarrowCollisions;
        if (cumPos == 0) {
            fail("BDD-010 / overlapping AABBs produce a contact pair between two distinct objects",
                 "cumulativeNarrowCollisions == 0 — pipeline didn't fire across all substeps");
        } else {
            pass("BDD-010 / overlapping AABBs produce a contact pair between two distinct objects");
        }

        // --- Negative: cloth far above ground, AABBs disjoint. ---
        resetScene();
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                     tinym::vec3(0.0f, 10.0f, 0.0f),
                     /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                     /*thickness=*/0.01, /*mass=*/0.1);
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                      /*size1D=*/2.0f);
        sim.initialize();
        Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions = 0;
        sim.update();

        size_t cumNeg = Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions;
        if (cumNeg != 0) {
            fail("BDD-010 / non-overlapping AABBs produce empty constraint set",
                 "cumulativeNarrowCollisions == " + std::to_string(cumNeg) +
                 " on a scene with disjoint AABBs");
        } else {
            pass("BDD-010 / non-overlapping AABBs produce empty constraint set");
        }
    }

    // ---- Block 14: BDD-017 — Ray-pick selects nearest hit object. ---------
    // TESTS.md#BDD-017 wording (verbatim, *not* the matrix-row label):
    //   Given a scene with two objects whose screen-space projections do
    //         not overlap
    //   When  the user clicks on one object's screen position
    //   Then  that object becomes the selected object; the inspector
    //         displays its parameters.
    //   Notes: also test the overlapping case — the front-most object
    //          (smallest ray t) wins.
    //
    // Substitution: harness has no GLFW/ImGui, so "click on screen
    // position" is mechanized as a world-space Ray fed directly to
    // BroadPhase::queryClickRay (mirroring the production callback at
    // src/main.cpp:6577 minus the camera unprojection). The BDD's
    // load-bearing claim — that the ray hits the clicked object's id —
    // is fully exercised by the BVH query + smallest-tmin walk.
    // "Inspector displays its parameters" is BDD-018's concern.
    {
        const Index kNoHit = static_cast<Index>(-1);
        auto pickClosest = [&]() -> Index {
            auto& rt = Scene<Backend, Precision>::rayTracedData;
            Index n = rt.numClickRayCollisions[0];
            if (n == 0) return kNoHit;
            Index closest = rt.clickRayCollisions[0].obj;
            float tmin = rt.clickRayCollisions[0].tmin;
            for (Index i = 1; i < n; ++i) {
                if (rt.clickRayCollisions[i].tmin < tmin) {
                    tmin = rt.clickRayCollisions[i].tmin;
                    closest = rt.clickRayCollisions[i].obj;
                }
            }
            return closest;
        };

        // --- Clause (a): non-overlapping screen projections. -----------
        // cubeA at x=-1.5, cubeB at x=+1.5 (both y=0, z=0, size=0.5). Their
        // AABBs are disjoint along x; rays cast straight down -z through
        // each cube's x line miss the other.
        resetScene();
        sim.addCube(tinym::vec3(-1.5f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.addCube(tinym::vec3( 1.5f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        // Production refits the scene-level BVH on every sim.update() before
        // the click callback runs. One update() here mirrors that —
        // Float-tagged cubes don't move, so AABBs are stable post-refit.
        sim.update();
        const Index cubeAId = 0;
        const Index cubeBId = 1;

        // Reset selectedObj so the assertions below read a freshly-written
        // value (production callback at src/main.cpp:6718 writes
        // simulator->selectedObj on every click; harness mirrors).
        sim.selectedObj = -1;

        Ray rayA;
        rayA.origin = tinym::vec3(-1.5f, 0.0f,  10.0f);
        rayA.dir    = tinym::vec3( 0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayA);
        Index pickedA = pickClosest();
        sim.selectedObj = static_cast<int>(pickedA);

        bool clauseAOk = (sim.selectedObj == static_cast<int>(cubeAId));
        if (clauseAOk) {
            Ray rayB;
            rayB.origin = tinym::vec3( 1.5f, 0.0f,  10.0f);
            rayB.dir    = tinym::vec3( 0.0f, 0.0f, -1.0f);
            Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
            sim.collisionPipeline.broadPhase.queryClickRay(rayB);
            Index pickedB = pickClosest();
            sim.selectedObj = static_cast<int>(pickedB);
            if (sim.selectedObj != static_cast<int>(cubeBId)) {
                fail("BDD-017 / ray hits the clicked object's id (non-overlapping)",
                     "expected sim.selectedObj=" + std::to_string(cubeBId) +
                     " (cubeB), got " + std::to_string(sim.selectedObj));
                clauseAOk = false;
            } else {
                pass("BDD-017 / ray hits the clicked object's id (non-overlapping)");
            }
        } else {
            fail("BDD-017 / ray hits the clicked object's id (non-overlapping)",
                 "expected sim.selectedObj=" + std::to_string(cubeAId) +
                 " (cubeA), got " + std::to_string(sim.selectedObj));
        }

        // --- Clause (b): overlapping case, front-most (smallest tmin). -
        // cubeFront at z=+2 (closer to ray origin at z=+10), cubeBack at
        // z=-2. Both share x=0; ray straight down -z passes through both.
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f,  2.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.addCube(tinym::vec3(0.0f, 0.0f, -2.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        sim.update(); // refit scene-level BVH AABBs (see clause a comment).
        const Index cubeFrontId = 0;
        const Index cubeBackId  = 1;

        Ray rayDeep;
        rayDeep.origin = tinym::vec3(0.0f, 0.0f, 10.0f);
        rayDeep.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayDeep);
        Index numHits = Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];

        // Walk the full hit list to verify BOTH cubes participated —
        // queryClickRay writes per-triangle hits, so numHits >= 2 alone
        // is satisfiable by a single cube and doesn't prove the back
        // cube was reached. The both-cubes check closes that gap.
        bool sawFront = false, sawBack = false;
        for (Index i = 0; i < numHits; ++i) {
            Index hitObj = Scene<Backend, Precision>::rayTracedData
                .clickRayCollisions[i].obj;
            if (hitObj == cubeFrontId) sawFront = true;
            if (hitObj == cubeBackId)  sawBack  = true;
        }

        Index pickedDeep = pickClosest();
        sim.selectedObj = static_cast<int>(pickedDeep);
        if (numHits < 2) {
            fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
                 "expected >=2 hits along through-line, got " +
                 std::to_string(numHits));
        } else if (!sawFront || !sawBack) {
            fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
                 "ray hits did not include both cubes (sawFront=" +
                 std::to_string(sawFront) + ", sawBack=" +
                 std::to_string(sawBack) + ")");
        } else if (sim.selectedObj != static_cast<int>(cubeFrontId)) {
            fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
                 "expected sim.selectedObj=" + std::to_string(cubeFrontId) +
                 " (cubeFront, cubeBack=" + std::to_string(cubeBackId) +
                 "), got " + std::to_string(sim.selectedObj));
        } else {
            pass("BDD-017 / overlapping case: front-most object (smallest ray t) wins");
        }
    }

    // ---- Block 15: FR-004 UI — rotate-object via Simulator::rotateObject. -
    // FR-004 wording (FRD.md): "the user shall rotate the center of any
    // selected object, with rotation stored canonically as a quaternion;
    // the same quaternion is used for rendering, simulation, and
    // persistence." Block 12 (BDD-004) already covers the math/persistence
    // side. This block covers the UI side: rotateObject mutates state.x
    // by rotating around transformPosition pivot, and the next sim step /
    // render reads the rotated positions.
    //
    // Substitution for "rendering reflects": same as Block 9's clause (c)
    // — headless harness has no pixel render, so the testable proxy is
    // "render-source state.x already carries the rotated values".
    {
        constexpr float kPi15 = 3.14159265358979323846f;
        auto quatAxisAngleLocal = [](tinym::vec3 axis, float angle) {
            float half = angle * 0.5f;
            float s = std::sin(half);
            return ::Quat{std::cos(half), axis.x * s, axis.y * s, axis.z * s};
        };

        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        const int rotateId = 0;

        auto* mesh0 = Scene<Backend, Precision>::findById(rotateId);
        if (!mesh0) {
            fail("FR-004 UI / rotate sets state.x rotated around pivot",
                 "rotate target id=" + std::to_string(rotateId) + " not found");
        } else {
            // Witness vertex: state.x[0] (some corner of the cube). cube is
            // centered on transformPosition=(0,0,0), so the pivot equals
            // origin and (x, y, z) -> (-y, x, z) under a 90-degree Z rotation.
            double x0 = mesh0->state.x.ptr[0];
            double y0 = mesh0->state.x.ptr[1];
            double z0 = mesh0->state.x.ptr[2];

            ::Quat newAbs = quatAxisAngleLocal(tinym::vec3(0, 0, 1), kPi15 / 2.0f);
            sim.rotateObject(rotateId, newAbs);

            auto* meshAfter = Scene<Backend, Precision>::findById(rotateId);
            double rx = meshAfter->state.x.ptr[0];
            double ry = meshAfter->state.x.ptr[1];
            double rz = meshAfter->state.x.ptr[2];

            const double posTol = 1e-5;
            bool rotateOk =
                std::abs(rx - (-y0)) < posTol &&
                std::abs(ry - ( x0)) < posTol &&
                std::abs(rz - ( z0)) < posTol;

            if (!rotateOk) {
                fail("FR-004 UI / rotate sets state.x rotated around pivot",
                     "expected 90deg-Z (" + std::to_string(-y0) + ", " +
                     std::to_string(x0) + ", " + std::to_string(z0) +
                     ") got (" + std::to_string(rx) + ", " +
                     std::to_string(ry) + ", " + std::to_string(rz) + ")");
            } else {
                pass("FR-004 UI / rotate sets state.x rotated around pivot");

                // Float-tagged cube doesn't move; strict equality after one
                // sim.update() — mirrors Block 9's clause (b)/(c) shape.
                pumpFrames(sim, 1);
                auto* meshStep = Scene<Backend, Precision>::findById(rotateId);
                double sx = meshStep->state.x.ptr[0];
                double sy = meshStep->state.x.ptr[1];
                double sz = meshStep->state.x.ptr[2];
                if (std::abs(sx - rx) > posTol ||
                    std::abs(sy - ry) > posTol ||
                    std::abs(sz - rz) > posTol) {
                    fail("FR-004 UI / next sim step preserves rotated state.x",
                         "Float-tagged witness drifted between rotate and post-step");
                } else {
                    pass("FR-004 UI / next sim step preserves rotated state.x");
                }
            }
        }
    }

    // ---- Block 16: D-023 — translateObject / rotateObject refit BVH. ------
    // Estimator turn-17 WARNING: state.x mutations from the inspector edits
    // were not refitting the scene-level BVH, so click-pick on a paused
    // sim hit the old pose. D-023 fixes this with one refit() call at the
    // end of each function. Block 16 mechanizes the invariant so a
    // regression that drops the refit() call surfaces as a hard FAIL.
    {
        // --- Clause (a): translateObject refits the BVH. ---------------
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        sim.update();  // populate BVH AABBs
        const Index translateRefitId = 0;

        sim.translateObject(translateRefitId, tinym::vec3(5.0f, 0.0f, 0.0f));

        // Ray at NEW position (5, 0, 0): expect hit.
        Ray rayNewT;
        rayNewT.origin = tinym::vec3(5.0f, 0.0f, 10.0f);
        rayNewT.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayNewT);
        Index hitsNewT =
            Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];

        // Ray at OLD position (0, 0, 0): expect miss.
        Ray rayOldT;
        rayOldT.origin = tinym::vec3(0.0f, 0.0f, 10.0f);
        rayOldT.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayOldT);
        Index hitsOldT =
            Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];

        if (hitsNewT == 0) {
            fail("FR-003 / translateObject refits BVH so click-pick reads new pose",
                 "ray at NEW position (5, 0, 0) returned 0 hits — BVH still on old pose");
        } else if (hitsOldT != 0) {
            fail("FR-003 / translateObject refits BVH so click-pick reads new pose",
                 "ray at OLD position (0, 0, 0) returned " +
                 std::to_string(hitsOldT) + " hits — BVH still includes old pose");
        } else {
            pass("FR-003 / translateObject refits BVH so click-pick reads new pose");
        }

        // --- Clause (b): rotateObject refits the BVH. ------------------
        // 90deg-Z rotation leaves an axis-aligned cube's AABB unchanged
        // (rotational symmetry). Use 45deg-Z so the AABB grows from
        // ±0.25 to ±0.25*sqrt(2) ≈ ±0.354 in xy. Witness ray at x=0.30
        // is OUTSIDE the original AABB and INSIDE the rotated AABB.
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        sim.update();
        const Index rotateRefitId = 0;

        Ray rayWitness;
        rayWitness.origin = tinym::vec3(0.30f, 0.0f, 10.0f);
        rayWitness.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);

        // Pre-condition: ray at x=0.30 must NOT hit pre-rotate cube.
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayWitness);
        Index hitsPre =
            Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];

        // Apply 45deg-Z rotation.
        constexpr float kPi16 = 3.14159265358979323846f;
        auto qAxisAngle16 = [](tinym::vec3 a, float ang) {
            float h = ang * 0.5f;
            return ::Quat{std::cos(h), a.x * std::sin(h),
                          a.y * std::sin(h), a.z * std::sin(h)};
        };
        ::Quat newAbsR = qAxisAngle16(tinym::vec3(0, 0, 1), kPi16 / 4.0f);
        sim.rotateObject(rotateRefitId, newAbsR);

        // Post-rotate: same witness ray must now hit.
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayWitness);
        Index hitsPost =
            Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];

        if (hitsPre != 0) {
            fail("FR-004 UI / rotateObject refits BVH so click-pick reads new pose",
                 "pre-rotate witness at x=0.30 unexpectedly hit (hits=" +
                 std::to_string(hitsPre) + ") — scene setup is wrong");
        } else if (hitsPost == 0) {
            fail("FR-004 UI / rotateObject refits BVH so click-pick reads new pose",
                 "post-rotate witness at x=0.30 returned 0 hits — BVH still on pre-rotate AABB");
        } else {
            pass("FR-004 UI / rotateObject refits BVH so click-pick reads new pose");
        }
    }

    // ---- Block 17: D-024 — click-pick uses triangle-precision ranking. ----
    // The bug: queryClickRay's leaf write used AABB-tmin, so a rotated
    // mesh with a leaf AABB extending toward the camera could win the
    // smallest-tmin race even when its actual triangle was farther
    // along the ray than a smaller object's triangle. The user-reported
    // case (Plane rotated by quat (1, 2, 0, 0) normalized) actually
    // had the Plane geometrically in front of the cube — so picking
    // Plane was correct in that specific scene. To DISCRIMINATE
    // AABB-precision from triangle-precision, this test uses a scene
    // where the Plane's AABB extends toward the camera (so old AABB-
    // tmin makes Plane win) but its actual triangle plane crosses the
    // click ray BEHIND the cube (so triangle-tmin makes the cube win).
    //
    // Setup: large ground (size 30) at y=-1 tilted 60deg around X. Plane
    // normal post-rotation = (0, 0.5, 0.866); plane passes through
    // (0, -1, 0). Click ray from (0, 0, 10) toward -z crosses the
    // plane at z = -0.577 (t = 10.577). Cube at origin: triangle hit
    // at t = 9.75. Cube triangle is closer → click must select cube.
    // Plane AABB after rotation: z extent ≈ ±7.5; camera at z=10 sees
    // AABB tmin = 2.5 (much smaller than cube's 9.75) — old AABB-only
    // logic would have picked Plane.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                      /*size1D=*/30.0f);
        sim.initialize();
        sim.update();
        const Index cubeIdQ   = 0;
        const Index groundIdQ = 1;

        // 60deg around X via axis-angle quat: (cos(30deg), sin(30deg), 0, 0).
        constexpr float kPi17 = 3.14159265358979323846f;
        float halfAngle = (kPi17 / 3.0f) * 0.5f; // 60deg / 2 = 30deg
        ::Quat tilt60{std::cos(halfAngle), std::sin(halfAngle), 0.0f, 0.0f};
        sim.rotateObject(static_cast<int>(groundIdQ), tilt60);

        Ray rayClick;
        rayClick.origin = tinym::vec3(0.0f, 0.0f, 10.0f);
        rayClick.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(rayClick);

        // Production-style smallest-tmin walk on the populated buffer.
        auto& rt17 = Scene<Backend, Precision>::rayTracedData;
        Index num17 = rt17.numClickRayCollisions[0];
        int picked17 = -1;
        if (num17 > 0) {
            picked17 = static_cast<int>(rt17.clickRayCollisions[0].obj);
            float bestT17 = rt17.clickRayCollisions[0].tmin;
            for (Index i = 1; i < num17; ++i) {
                if (rt17.clickRayCollisions[i].tmin < bestT17) {
                    bestT17 = rt17.clickRayCollisions[i].tmin;
                    picked17 = static_cast<int>(rt17.clickRayCollisions[i].obj);
                }
            }
        }

        if (picked17 != static_cast<int>(cubeIdQ)) {
            fail("FR-002 / click-pick selects nearest triangle, not nearest AABB",
                 "tilted ground stole the click; expected cube id=" +
                 std::to_string(cubeIdQ) + " got " + std::to_string(picked17) +
                 " (groundId=" + std::to_string(groundIdQ) + ")");
        } else {
            pass("FR-002 / click-pick selects nearest triangle, not nearest AABB");
        }
    }

    // ---- Block 18: D-025 — rotateObject survives Scene::pack rebuild. ----
    // Mirrors BDD-003 clause (d) shape (translate-survives-re-pack) for
    // rotation. Re-pack rebuilds state.x from the initializer's geometry,
    // which loses rotateObject's effect on state.x. D-025 closes the gap
    // by writing pendingRotations[id] inside rotateObject AND auto-
    // calling applyPendingMaterials() from Simulator::initialize().
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        const int rotateRoundTripId = 0;

        auto* m0_pre = Scene<Backend, Precision>::findById(rotateRoundTripId);
        if (!m0_pre) {
            fail("FR-004 / rotateObject survives Scene::pack rebuild",
                 "cube id=" + std::to_string(rotateRoundTripId) + " not found pre-rotate");
        } else {
            // 90deg-Z rotation; witness vertex (x, y, z) -> (-y, x, z).
            constexpr float kPi18 = 3.14159265358979323846f;
            float halfA = (kPi18 / 2.0f) * 0.5f;
            ::Quat newAbs = ::Quat{std::cos(halfA), 0.0f, 0.0f, std::sin(halfA)};
            sim.rotateObject(rotateRoundTripId, newAbs);

            double rx = m0_pre->state.x.ptr[0];
            double ry = m0_pre->state.x.ptr[1];
            double rz = m0_pre->state.x.ptr[2];

            // Force a re-pack: addCube + sim.initialize().
            sim.addCube(tinym::vec3(5.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();  // pack + auto-applyPendingMaterials (D-025)

            auto* m0_after = Scene<Backend, Precision>::findById(rotateRoundTripId);
            if (!m0_after) {
                fail("FR-004 / rotateObject survives Scene::pack rebuild",
                     "cube id=" + std::to_string(rotateRoundTripId) +
                     " disappeared after re-pack");
            } else {
                const double posTol = 1e-5;
                double rrx = m0_after->state.x.ptr[0];
                double rry = m0_after->state.x.ptr[1];
                double rrz = m0_after->state.x.ptr[2];
                if (std::abs(rrx - rx) > posTol ||
                    std::abs(rry - ry) > posTol ||
                    std::abs(rrz - rz) > posTol) {
                    fail("FR-004 / rotateObject survives Scene::pack rebuild",
                         "post-repack state.x[0] drifted from rotated value: "
                         "expected (" + std::to_string(rx) + ", " +
                         std::to_string(ry) + ", " + std::to_string(rz) +
                         ") got (" + std::to_string(rrx) + ", " +
                         std::to_string(rry) + ", " + std::to_string(rrz) + ")");
                } else {
                    pass("FR-004 / rotateObject survives Scene::pack rebuild");
                }
            }
        }
    }

    // ---- Block 19: D-026 / CM-008 — scene-swap-at-same-count rebuilds Float-mesh BVH. ----
    // Reproduces the harness pattern that motivated CM-008's workaround
    // (objTrees.clear() between scenes) WITHOUT calling clear(). The
    // production fix (D-026 — lifetimeId gate on BroadPhase::build's
    // Float-mesh skip) makes the workaround unnecessary. Bug-probe
    // condition: revert the lifetimeId clause in the skip → expect
    // FAIL with "ray hit nothing" diagnostic (stale slot-0 leaf AABB
    // at x=-3 is missed by the ray cast at x=+3).
    {
        const Index kNoHit19 = static_cast<Index>(-1);
        auto pickClosest19 = [&]() -> Index {
            auto& rt = Scene<Backend, Precision>::rayTracedData;
            Index n = rt.numClickRayCollisions[0];
            if (n == 0) return kNoHit19;
            Index closest = rt.clickRayCollisions[0].obj;
            float tmin = rt.clickRayCollisions[0].tmin;
            for (Index i = 1; i < n; ++i) {
                if (rt.clickRayCollisions[i].tmin < tmin) {
                    tmin = rt.clickRayCollisions[i].tmin;
                    closest = rt.clickRayCollisions[i].obj;
                }
            }
            return closest;
        };

        // Phase 1: build a tree for cube at x=-3. Populates objTrees[0]
        // with builtForLifetimeId set to whatever lifetimeMeshCount was
        // here. Crucially: NO objTrees.clear() before this — we rely on
        // the production fix (D-026) to skip-correctly across scenes.
        resetScene();
        sim.addCube(tinym::vec3(-3.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        sim.update();

        // Phase 2: resetScene WITHOUT manual objTrees.clear(); add a new
        // cube at +3. The new mesh has a different lifetimeId, so the
        // Float-mesh skip in BroadPhase::build must NOT fire for slot 0.
        // Without D-026, slot 0's stale leaf AABB at x=-3 persists and
        // the ray cast at x=+3 hits nothing.
        resetScene();
        sim.addCube(tinym::vec3(3.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        sim.update();
        const Index newCubeId = 0;

        Ray ray19;
        ray19.origin = tinym::vec3(3.0f, 0.0f, 10.0f);
        ray19.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
        Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
        sim.collisionPipeline.broadPhase.queryClickRay(ray19);
        Index picked19 = pickClosest19();

        if (picked19 == kNoHit19) {
            fail("CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH",
                 "ray at x=+3 hit nothing — stale tree slot 0 still has "
                 "AABB at x=-3 (production fix D-026 reverted/missing?)");
        } else if (picked19 != static_cast<Index>(newCubeId)) {
            fail("CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH",
                 "expected pickClosest id " + std::to_string(newCubeId) +
                 " (new cube at +3), got " + std::to_string(picked19));
        } else {
            pass("CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH");
        }
    }

    // ---- Block 20: D-027 — setMaterial closes BDD-005's data-layer clauses. ----
    // Renderer-side clause ("preview render reflects the lower roughness")
    // is parked under the PBR-preview-shader slice; this block mechanizes
    // the data-layer subset: setMaterial writes all 5 fields to mesh.material;
    // a sim step preserves them (BDD-103 backend-boundary); save/load
    // round-trips them; addCube re-pack preserves them (D-025 sister
    // mechanization).
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        const int matMeshId = 0;

        auto* m0 = Scene<Backend, Precision>::findById(matMeshId);
        if (!m0) {
            fail("BDD-005 / setMaterial writes all 5 fields to mesh.material",
                 "cube id=" + std::to_string(matMeshId) + " not found");
        } else {
            ::Material edited;
            edited.baseColor = tinym::vec3(0.2f, 0.3f, 0.4f);
            edited.metallic = 0.7f;
            edited.roughness = 0.1f;
            edited.specularWeight = 0.8f;
            edited.emissionColor = tinym::vec3(0.05f, 0.10f, 0.15f);

            const float matTol = 1e-6f;
            auto matEqual = [&](const ::Material& a, const ::Material& b) {
                return std::abs(a.baseColor.x - b.baseColor.x) < matTol
                    && std::abs(a.baseColor.y - b.baseColor.y) < matTol
                    && std::abs(a.baseColor.z - b.baseColor.z) < matTol
                    && std::abs(a.metallic - b.metallic) < matTol
                    && std::abs(a.roughness - b.roughness) < matTol
                    && std::abs(a.specularWeight - b.specularWeight) < matTol
                    && std::abs(a.emissionColor.x - b.emissionColor.x) < matTol
                    && std::abs(a.emissionColor.y - b.emissionColor.y) < matTol
                    && std::abs(a.emissionColor.z - b.emissionColor.z) < matTol;
            };

            // Phase 1: setMaterial writes all 5 fields.
            sim.setMaterial(matMeshId, edited);
            if (!matEqual(m0->material, edited)) {
                fail("BDD-005 / setMaterial writes all 5 fields to mesh.material",
                     "mesh.material differs from edited after setMaterial: "
                     "got baseColor=(" + std::to_string(m0->material.baseColor.x) + ", " +
                     std::to_string(m0->material.baseColor.y) + ", " +
                     std::to_string(m0->material.baseColor.z) + ") metallic=" +
                     std::to_string(m0->material.metallic) + " roughness=" +
                     std::to_string(m0->material.roughness) + " specWeight=" +
                     std::to_string(m0->material.specularWeight) + " emission=(" +
                     std::to_string(m0->material.emissionColor.x) + ", " +
                     std::to_string(m0->material.emissionColor.y) + ", " +
                     std::to_string(m0->material.emissionColor.z) + ")");
            } else {
                pass("BDD-005 / setMaterial writes all 5 fields to mesh.material");
            }

            // Phase 2: a sim step preserves material (BDD-103 backend-boundary —
            // the simulation kernels must not clobber material fields).
            sim.update();
            auto* m0_postStep = Scene<Backend, Precision>::findById(matMeshId);
            if (!m0_postStep || !matEqual(m0_postStep->material, edited)) {
                fail("BDD-005 / material survives one sim step",
                     "material drifted after sim.update() — kernel clobber?");
            } else {
                pass("BDD-005 / material survives one sim step");
            }

            // Phase 3: save → reset → load → init round-trip.
            const std::string matSavePath = "/tmp/bdd005_material_roundtrip.ysim.json";
            std::string saveErr;
            if (!sim.saveScene(matSavePath, &saveErr)) {
                fail("BDD-005 / material round-trips through saveScene/loadScene",
                     "saveScene failed: " + saveErr);
            } else {
                resetScene();
                auto lr20 = sim.loadScene(matSavePath);
                if (!lr20.ok) {
                    fail("BDD-005 / material round-trips through saveScene/loadScene",
                         "loadScene failed: " + lr20.error.message);
                } else {
                    sim.initialize();  // auto-applyPendingMaterials writes mesh.material (D-025)
                    auto* m0_postLoad = Scene<Backend, Precision>::findById(matMeshId);
                    if (!m0_postLoad) {
                        fail("BDD-005 / material round-trips through saveScene/loadScene",
                             "cube id=0 disappeared after load");
                    } else if (!matEqual(m0_postLoad->material, edited)) {
                        fail("BDD-005 / material round-trips through saveScene/loadScene",
                             "post-load material differs from saved edit: "
                             "got roughness=" + std::to_string(m0_postLoad->material.roughness) +
                             " (expected " + std::to_string(edited.roughness) + ")");
                    } else {
                        pass("BDD-005 / material round-trips through saveScene/loadScene");
                    }
                }
            }

            // Phase 4: addCube + sim.initialize() forces re-pack; setMaterial
            // edit must survive (D-025 sister mechanization for material).
            // The Phase 3 load already populated pendingMaterials[0] for the
            // same edit; we re-apply via setMaterial to exercise the edit-time
            // path explicitly (rather than relying on the load-time path).
            auto* m0_preRepack = Scene<Backend, Precision>::findById(matMeshId);
            if (m0_preRepack) {
                sim.setMaterial(matMeshId, edited);  // ensures pendingMaterials[0] is set
            }
            sim.addCube(tinym::vec3(5.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            auto* m0_postRepack = Scene<Backend, Precision>::findById(matMeshId);
            if (!m0_postRepack) {
                fail("BDD-005 / material survives Scene::pack rebuild",
                     "cube id=0 disappeared after re-pack");
            } else if (!matEqual(m0_postRepack->material, edited)) {
                fail("BDD-005 / material survives Scene::pack rebuild",
                     "post-repack material differs — pendingMaterials write-back broken? "
                     "got roughness=" + std::to_string(m0_postRepack->material.roughness) +
                     " (expected " + std::to_string(edited.roughness) + ")");
            } else {
                pass("BDD-005 / material survives Scene::pack rebuild");
            }
        }
    }

    // ---- Block 21: D-029 — GPU bottom-up combine matches CPU reference. ----
    // The new atomic single-dispatch bottomUpBoxes kernel must produce
    // tree[] interior-node AABBs bit-equal to the CPU reference
    // bottomUpCombine() for the same leaf input. Mechanization:
    //   1. Build a tess=2 cube via the production GPU path (initialize
    //      triggers BroadPhase::build → per-mesh BVH::build →
    //      bottomUpBoxesGPU).
    //   2. Snapshot tree[] (GPU result).
    //   3. Re-dispatch buildLeafGPU to reset leaves to a clean state,
    //      then run CPU bottomUpCombine — this overwrites tree[]
    //      interior nodes with the CPU reference.
    //   4. Assert tree[i].min/max bit-equal between gpu and cpu for
    //      every interior node (0..numPrimitives-2).
    // Stricter than "just root matches" — catches localized race
    // conditions per PLANNER.md step 7.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();

        auto* mesh21 = Scene<Backend, Precision>::findById(0);
        auto& bp21 = sim.collisionPipeline.broadPhase;
        if (!mesh21 || bp21.objTrees.empty()) {
            fail("D-029 / GPU bottom-up combine matches CPU reference",
                 "mesh or objTrees[0] missing");
        } else {
            auto& objTree = bp21.objTrees[0];
            Index numPrimitives21 = mesh21->adjacency.facets.size / 3;
            Index numNodes21 = (numPrimitives21 > 0) ? (2 * numPrimitives21 - 1) : 0;
            if (numNodes21 < 2) {
                fail("D-029 / GPU bottom-up combine matches CPU reference",
                     "tess=2 cube produced numNodes=" + std::to_string(numNodes21) +
                     " (need >= 2 for interior-node comparison)");
            } else {
                // Snapshot GPU result into raw float arrays (BVHNode
                // has no default constructor; can't std::vector<BVHNode>(N)).
                // 6 floats per node: min.x/y/z, max.x/y/z.
                std::vector<float> gpuFlat(numNodes21 * 6);
                for (Index i = 0; i < numNodes21; ++i) {
                    const auto& n = objTree.tree[i];
                    gpuFlat[i*6+0] = (float)n.min.x;
                    gpuFlat[i*6+1] = (float)n.min.y;
                    gpuFlat[i*6+2] = (float)n.min.z;
                    gpuFlat[i*6+3] = (float)n.max.x;
                    gpuFlat[i*6+4] = (float)n.max.y;
                    gpuFlat[i*6+5] = (float)n.max.z;
                }

                // Re-dispatch buildLeafGPU to reset leaves to the same
                // initial state, then CPU bottomUpCombine produces the
                // reference for interior nodes.
                objTree.buildLeafGPU();
                MetalGlobalContext::commitAndWait();
                objTree.bottomUpCombine();

                // Compare interior nodes (0..numPrimitives-2). The leaf
                // range (numPrim-1..2*numPrim-2) is set identically by
                // both paths, so we focus on the differential surface.
                bool allMatch = true;
                Index mismatchAt = -1;
                for (Index i = 0; i < numPrimitives21 - 1; ++i) {
                    const auto& c = objTree.tree[i];
                    float gmnx = gpuFlat[i*6+0], gmny = gpuFlat[i*6+1], gmnz = gpuFlat[i*6+2];
                    float gmxx = gpuFlat[i*6+3], gmxy = gpuFlat[i*6+4], gmxz = gpuFlat[i*6+5];
                    if (gmnx != c.min.x || gmny != c.min.y || gmnz != c.min.z ||
                        gmxx != c.max.x || gmxy != c.max.y || gmxz != c.max.z) {
                        allMatch = false;
                        mismatchAt = i;
                        break;
                    }
                }
                if (allMatch) {
                    pass("D-029 / GPU bottom-up combine matches CPU reference");
                } else {
                    const auto& c = objTree.tree[mismatchAt];
                    fail("D-029 / GPU bottom-up combine matches CPU reference",
                         "interior node " + std::to_string(mismatchAt) +
                         " AABB mismatch: gpu min=(" +
                         std::to_string(gpuFlat[mismatchAt*6+0]) + "," +
                         std::to_string(gpuFlat[mismatchAt*6+1]) + "," +
                         std::to_string(gpuFlat[mismatchAt*6+2]) + ") max=(" +
                         std::to_string(gpuFlat[mismatchAt*6+3]) + "," +
                         std::to_string(gpuFlat[mismatchAt*6+4]) + "," +
                         std::to_string(gpuFlat[mismatchAt*6+5]) + ") vs cpu min=(" +
                         std::to_string((float)c.min.x) + "," +
                         std::to_string((float)c.min.y) + "," +
                         std::to_string((float)c.min.z) + ") max=(" +
                         std::to_string((float)c.max.x) + "," +
                         std::to_string((float)c.max.y) + "," +
                         std::to_string((float)c.max.z) + ")");
                }
            }
        }
    }

    // ---- Block 22: D-029 fix-turn — N=1 BVH safely bypasses bottom-up combine. ----
    // Estimator turn-23 BLOCK: bottomUpBoxesGPU ran the new atomic
    // kernel for numPrimitives == 1, but buildTree_* short-circuits
    // at `idx == numPrimitives - 1` BEFORE writing treeParent[childA/B]
    // — so for N=1 (only thread is idx=0, the leaf-root) treeParent[0]
    // is uninitialized → bottomUpBoxes reads garbage → UB. Fix: the
    // guard in bottomUpBoxesGPU now short-circuits when numPrimitives
    // <= 1 (the tree's single node IS the leaf-root; nothing to
    // combine). This block exercises the N=1 path end-to-end: write
    // a single-triangle .obj to /tmp, import it, run sim.initialize +
    // sim.update (exercises BVH::build + BVH::refit), then assert
    // tree[0]'s AABB matches the triangle bounds AND a ray-pick query
    // walks the BVH correctly for N=1.
    {
        const std::string objPath = "/tmp/bdd_d029_n1.obj";
        {
            std::ofstream f(objPath);
            f << "v -0.5 0.0 -0.5\n"
              << "v  0.5 0.0 -0.5\n"
              << "v  0.0 0.0  0.5\n"
              << "f 1 2 3\n";
        }

        resetScene();
        std::string err22;
        bool ok22 = sim.importMesh("/tmp", "bdd_d029_n1.obj",
                                   /*scale=*/(Precision)1.0,
                                   /*mass=*/(Precision)0.1, &err22);
        if (!ok22) {
            fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                 "importMesh returned false: " + err22);
        } else {
            sim.initialize();   // BVH::build with N=1 — buildLeafGPU writes
                                // single leaf at tree[0]; bottomUpBoxesGPU
                                // early-returns under the new guard.
            sim.update();       // BVH::refit with N=1 — same path; must not crash.

            auto* mesh22 = Scene<Backend, Precision>::findById(0);
            auto& bp22 = sim.collisionPipeline.broadPhase;
            if (!mesh22 || bp22.objTrees.empty()) {
                fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                     "mesh or objTrees[0] missing after import + init");
            } else {
                auto& objTree22 = bp22.objTrees[0];
                const float aabbTol = 1e-5f;
                bool aabbOk =
                    std::abs((float)objTree22.tree[0].min.x - (-0.5f)) < aabbTol &&
                    std::abs((float)objTree22.tree[0].max.x - ( 0.5f)) < aabbTol &&
                    std::abs((float)objTree22.tree[0].min.z - (-0.5f)) < aabbTol &&
                    std::abs((float)objTree22.tree[0].max.z - ( 0.5f)) < aabbTol;
                if (!aabbOk) {
                    fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                         "tree[0] AABB doesn't match triangle bounds; got min=(" +
                         std::to_string((float)objTree22.tree[0].min.x) + "," +
                         std::to_string((float)objTree22.tree[0].min.y) + "," +
                         std::to_string((float)objTree22.tree[0].min.z) + ") max=(" +
                         std::to_string((float)objTree22.tree[0].max.x) + "," +
                         std::to_string((float)objTree22.tree[0].max.y) + "," +
                         std::to_string((float)objTree22.tree[0].max.z) + ")");
                } else {
                    // Stricter: ray-pick from above should hit the triangle.
                    Ray ray22;
                    ray22.origin = tinym::vec3(0.0f, 10.0f, 0.0f);
                    ray22.dir    = tinym::vec3(0.0f, -1.0f, 0.0f);
                    Scene<Backend, Precision>::rayTracedData
                        .numClickRayCollisions[0] = 0;
                    bp22.queryClickRay(ray22);
                    Index nHits22 = Scene<Backend, Precision>::rayTracedData
                        .numClickRayCollisions[0];
                    if (nHits22 == 0) {
                        fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                             "queryClickRay missed the single triangle "
                             "(broad-phase walk broken for N=1)");
                    } else {
                        pass("D-029 fix / N=1 BVH safely bypasses bottom-up combine");
                    }
                }
            }
        }

        std::remove(objPath.c_str());
    }

    // ---- Block 23: D-030 — hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}. ----
    // Builds a tess=2 cube (same scene as Block 21). Captures CPU
    // reference via bottomUpCombine(). Then for each test D in the
    // sweep, resets the leaves via buildLeafGPU and runs
    // bottomUpHybrid(D); asserts every interior node's AABB is
    // bit-equal to the CPU reference. D=0 → pure CPU. D=1, D=2 →
    // hybrid frontier (GPU does the first D levels from leaf;
    // CPU completes via bottomUpCombineWithSkip). D=1000 → pure
    // GPU (kernel walks to root, equivalent to D-029).
    //
    // Stricter than single-D: the sweep catches off-by-one
    // frontier-marking bugs (depth check at wrong loop position;
    // depth counter incremented at wrong site; treeVisitCounts
    // not zeroed before dispatch).
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();

        auto* mesh23 = Scene<Backend, Precision>::findById(0);
        auto& bp23 = sim.collisionPipeline.broadPhase;
        if (!mesh23 || bp23.objTrees.empty()) {
            fail("D-030 / hybrid bottom-up matches CPU reference for D in {0,1,2,1000}",
                 "mesh or objTrees[0] missing");
        } else {
            auto& objTree23 = bp23.objTrees[0];
            Index numPrimitives23 = mesh23->adjacency.facets.size / 3;
            Index numNodes23 = (numPrimitives23 > 0) ? (2*numPrimitives23 - 1) : 0;

            if (numNodes23 < 2) {
                fail("D-030 / hybrid bottom-up matches CPU reference for D in {0,1,2,1000}",
                     "tess=2 cube produced numNodes=" + std::to_string(numNodes23));
            } else {
                // Capture CPU reference: reset leaves, then bottomUpCombine().
                objTree23.buildLeafGPU();
                MetalGlobalContext::commitAndWait();
                objTree23.bottomUpCombine();

                std::vector<float> ref23(numNodes23 * 6);
                for (Index i = 0; i < numNodes23; ++i) {
                    const auto& n = objTree23.tree[i];
                    ref23[i*6+0]=(float)n.min.x; ref23[i*6+1]=(float)n.min.y; ref23[i*6+2]=(float)n.min.z;
                    ref23[i*6+3]=(float)n.max.x; ref23[i*6+4]=(float)n.max.y; ref23[i*6+5]=(float)n.max.z;
                }

                // We need a sceneBox for bottomUpHybrid. bottomUpBoxes
                // only reads sceneBox._pad0 (= numPrimitives), so a
                // minimal sceneBox suffices.
                AABB4 sceneBox23;
                sceneBox23.i0 = numPrimitives23;

                const int Ds[] = {0, 1, 2, 1000};
                bool allMatch = true;
                int failingD = -1;
                Index failingNode = (Index)-1;
                for (int d : Ds) {
                    // Reset leaves to the same starting state so every
                    // D iteration receives identical input.
                    objTree23.buildLeafGPU();
                    objTree23.bottomUpHybrid(sceneBox23, d);

                    for (Index i = 0; i < numPrimitives23 - 1; ++i) {
                        const auto& n = objTree23.tree[i];
                        if ((float)n.min.x != ref23[i*6+0] ||
                            (float)n.min.y != ref23[i*6+1] ||
                            (float)n.min.z != ref23[i*6+2] ||
                            (float)n.max.x != ref23[i*6+3] ||
                            (float)n.max.y != ref23[i*6+4] ||
                            (float)n.max.z != ref23[i*6+5]) {
                            allMatch = false;
                            failingD = d;
                            failingNode = i;
                            break;
                        }
                    }
                    if (!allMatch) break;
                }

                if (allMatch) {
                    pass("D-030 / hybrid bottom-up matches CPU reference for D in {0,1,2,1000}");
                } else {
                    const auto& n = objTree23.tree[failingNode];
                    fail("D-030 / hybrid bottom-up matches CPU reference for D in {0,1,2,1000}",
                         "D=" + std::to_string(failingD) + " node " +
                         std::to_string(failingNode) + " mismatch: hybrid min=(" +
                         std::to_string((float)n.min.x) + "," +
                         std::to_string((float)n.min.y) + "," +
                         std::to_string((float)n.min.z) + ") max=(" +
                         std::to_string((float)n.max.x) + "," +
                         std::to_string((float)n.max.y) + "," +
                         std::to_string((float)n.max.z) + ") vs ref min=(" +
                         std::to_string(ref23[failingNode*6+0]) + "," +
                         std::to_string(ref23[failingNode*6+1]) + "," +
                         std::to_string(ref23[failingNode*6+2]) + ") max=(" +
                         std::to_string(ref23[failingNode*6+3]) + "," +
                         std::to_string(ref23[failingNode*6+4]) + "," +
                         std::to_string(ref23[failingNode*6+5]) + ")");
                }
            }
        }
    }

    // ---- Block 25: D-032 — FBO PBR render reflects roughness change. ----
    // Brings up a hidden GL context (HiddenGLContext: GLFW with
    // GLFW_VISIBLE=false + GLEW), loads the production shader
    // (shader.vert/geom/frag), uploads a hand-built 24-vertex cube
    // (4 verts per face x 6 faces, so MeshGL::computeNormal produces
    // per-face normals → sharp specular), allocates a 256x256 RGBA8
    // + depth FBO via include/framebuffer.hpp's Framebuffer, then
    // renders twice with `roughness ∈ {0.1, 0.9}` and `glReadPixels`
    // each pass. Asserts max per-channel byte diff > 30 — the shader
    // must visibly respond to the roughness uniform.
    //
    // Closes BDD-005's render-side clause (parked manual-test-only
    // when D-028 PBR preview shader shipped). The standing structural
    // WARNING introduced by D-028 is now mechanized.
    //
    // SKIP-safe: if GLFW/GLEW init fails (no display server / no GL
    // driver) or the shader files aren't loadable from cwd (run
    // outside build/), SKIP rather than FAIL — those are unsupported
    // environments per ESTIMATOR.md, same discipline as Block 22's
    // N=1 BVH guard.
    //
    // Load-bearing bug-probe: commenting out the roughness setUniform
    // call inside MeshGL::draw makes the two renders identical (max
    // diff = 0) and this block FAILs with the expected diagnostic.
    // That probe proves the shader actually consumes the roughness
    // uniform per-render — the proximate verification D-028's
    // standing-WARNING was waiting on.
    {
        HiddenGLContext glctx(256, 256);
        if (!glctx.ok) {
            skip("fbo-glfw-init",
                 "glfwInit/createWindow/glewInit failed — no GL on this host");
        } else {
            Program fboShader;
            fboShader.loadShader("shader.vert", "shader.geom", "shader.frag");
            if (!fboShader.programID) {
                skip("fbo-shader-load",
                     "shader.vert/geom/frag not loadable from cwd "
                     "(run --self-test from build/)");
            } else {
                // Hand-built cube: 24 vertices (4 per face x 6 faces),
                // 12 triangles. Each face has its own 4 verts so
                // MeshGL::computeNormal gives per-face normals (sharp
                // specular). Vertex order per face: BL, BR, TR, TL
                // (counter-clockwise viewed from outside the cube).
                const float s = 0.5f;
                float cubeVerts[24 * 3] = {
                    // +Z (front)
                    -s, -s,  s,   s, -s,  s,   s,  s,  s,  -s,  s,  s,
                    // -Z (back)
                     s, -s, -s,  -s, -s, -s,  -s,  s, -s,   s,  s, -s,
                    // +X (right)
                     s, -s,  s,   s, -s, -s,   s,  s, -s,   s,  s,  s,
                    // -X (left)
                    -s, -s, -s,  -s, -s,  s,  -s,  s,  s,  -s,  s, -s,
                    // +Y (top)
                    -s,  s,  s,   s,  s,  s,   s,  s, -s,  -s,  s, -s,
                    // -Y (bottom)
                    -s, -s, -s,   s, -s, -s,   s, -s,  s,  -s, -s,  s,
                };
                unsigned int cubeIdx[12 * 3];
                for (int f = 0; f < 6; ++f) {
                    unsigned int b = f * 4;
                    cubeIdx[f*6+0] = b+0; cubeIdx[f*6+1] = b+1; cubeIdx[f*6+2] = b+2;
                    cubeIdx[f*6+3] = b+0; cubeIdx[f*6+4] = b+2; cubeIdx[f*6+5] = b+3;
                }
                float cubeNormals[24 * 3] = {0};
                MeshGL<CPU> cubeMesh(24, cubeVerts, 12, cubeIdx, cubeNormals);
                // 2026-05-15 (A2 split): MeshGL ctor no longer auto-calls
                // computeNormal — the production path now owns normalPtr
                // contents externally. BDD-005's hand-built cube relies on
                // winding-derived per-face normals so we invoke explicitly.
                cubeMesh.computeNormal();
                cubeMesh.updateBuffer();

                // FBO setup via existing Framebuffer struct. Note:
                // Framebuffer::attachTexture2D(int, GLint, ...) only
                // auto-derives format+type for GL_RGBA32F (see
                // framebuffer.hpp::TextureFormat::generate). For RGBA8
                // we construct the TextureFormat explicitly.
                Framebuffer fbo;
                fbo.init(glctx.window, 256, 256);
                TextureFormat tf;
                tf.internalFormat = GL_RGBA8;
                tf.format         = GL_RGBA;
                tf.type           = GL_UNSIGNED_BYTE;
                fbo.attachTexture2D(/*nTexture=*/1, tf, 256, 256);
                fbo.attachRenderBuffer(GL_DEPTH_COMPONENT24);

                // Camera and light collocated at (0.7, 0.7, 3) — both
                // looking at origin. With camera ~near +Z axis, the
                // half-vector H ≈ view direction; NdotH on the +Z face
                // ranges from ~0.87 (far corner) to ~1.0 (near
                // corner), so the specular hotspot sits inside one
                // corner of the face. That gives the roughness 0.1
                // case a sharp small bright spot vs roughness 0.9's
                // spread-out lower-magnitude glow — discriminating
                // across the image.
                fboShader.use();
                // shadowMap (sampler2DShadow) must not share unit 0 with
                // idBuffer (sampler2D) — same-unit mixed sampler types make
                // every draw GL_INVALID_OPERATION on macOS. Shadows stay
                // off (shadowsOn defaults 0); only the unit is separated.
                fboShader.setUniform("shadowMap", 2);
                tinym::mat4 M(1.0f);
                tinym::mat4 V = tinym::lookAt(
                    tinym::vec3(0.7f, 0.7f, 3.0f),
                    tinym::vec3(0.0f, 0.0f, 0.0f),
                    tinym::vec3(0.0f, 1.0f, 0.0f));
                tinym::mat4 P = tinym::perspective(
                    /*fovy_rad=*/0.7854f /*45deg*/,
                    /*aspect=*/1.0f, /*near=*/0.1f, /*far=*/100.0f);
                fboShader.setUniform("M", M);
                fboShader.setUniform("V", V);
                fboShader.setUniform("P", P);
                fboShader.setUniform("lightPosition",
                                     tinym::vec3(0.7f, 0.7f, 3.0f));
                // Light radiance. Production defaults to ~(160,160,160)
                // which saturates the harness's small framebuffer to
                // pure white everywhere (clamped 255 after gamma).
                // Use a smaller value so non-specular pixels land in
                // the discriminating range (~50–230) and the specular
                // hotspot's roughness-dependent falloff differs between
                // the two renders.
                fboShader.setUniform("lightColor",
                                     tinym::vec3(3.0f, 3.0f, 3.0f));

                auto renderAndReadback = [&](float roughness) -> std::vector<uint8_t> {
                    fbo.bind();
                    glViewport(0, 0, 256, 256);
                    glEnable(GL_DEPTH_TEST);
                    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                    cubeMesh.draw(fboShader,
                                  // Silver-ish metallic: at metallic=1
                                  // F0 = baseColor, so specular term
                                  // dominates and roughness changes
                                  // produce visually large shifts. With
                                  // non-metallic (F0=0.04), specular is
                                  // only ~4% of incident — diffuse swamps
                                  // it and the GGX peak (which is
                                  // sub-pixel-thin at low roughness) is
                                  // invisible in a rasterized image.
                                  tinym::vec3(0.85f, 0.85f, 0.85f),  // baseColor: silver
                                  /*metallic=*/1.0f,
                                  roughness,
                                  /*specularWeight=*/1.0f,
                                  tinym::vec3(0.0f));                  // emissionColor
                    glFinish();
                    std::vector<uint8_t> px(256 * 256 * 4);
                    glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE,
                                 px.data());
                    fbo.unbind();
                    return px;
                };

                std::vector<uint8_t> roughLo = renderAndReadback(0.1f);
                std::vector<uint8_t> roughHi = renderAndReadback(0.9f);

                int maxDiff = 0;
                for (std::size_t i = 0; i < roughLo.size(); ++i) {
                    int d = std::abs((int)roughLo[i] - (int)roughHi[i]);
                    if (d > maxDiff) maxDiff = d;
                }

                const int threshold = 30;
                if (maxDiff > threshold) {
                    pass("BDD-005 / FBO PBR render reflects roughness change");
                } else {
                    fail("BDD-005 / FBO PBR render reflects roughness change",
                         "max per-channel pixel diff " + std::to_string(maxDiff)
                         + " <= threshold " + std::to_string(threshold)
                         + " — shader did not visibly respond to roughness "
                         "uniform change between 0.1 and 0.9");
                }
            }
        }
    }

    // ---- Block 25b: SHADOW — directional shadow map darkens the receiver. --
    // Same HiddenGLContext + readback discipline as Block 25. Renders a
    // ground quad + a floating cube twice: once with shadowsOn=0, once with
    // a real depth pass bound and shadowsOn=1 (the exact light-VP math the
    // render loop uses). The shadowed render must be measurably darker on
    // average (shadows only ever REMOVE direct light) yet not black (the
    // ambient term survives). Catches: broken light VP, depth-compare
    // misconfiguration, bias/projection errors that shadow everything.
    {
        HiddenGLContext glctx(256, 256);
        if (!glctx.ok) {
            skip("shadow-glfw-init", "no GL on this host");
        } else {
            Program mainSh, depthSh;
            mainSh.loadShader("shader.vert", "shader.geom", "shader.frag");
            depthSh.loadShader("shadow.vert", "shadow.frag");
            if (!mainSh.programID || !depthSh.programID) {
                skip("shadow-shader-load",
                     "shader/shadow programs not loadable from cwd");
            } else {
                // Ground quad y=0 (±3) + cube (side 1) centered at y=0.8.
                const float g = 3.0f;
                float planeVerts[4 * 3] = {
                    -g, 0,  g,   g, 0,  g,   g, 0, -g,  -g, 0, -g,
                };
                unsigned int planeIdx[6] = {0, 1, 2, 0, 2, 3};
                float planeNormals[4 * 3] = {0};
                MeshGL<CPU> planeMesh(4, planeVerts, 2, planeIdx, planeNormals);
                planeMesh.computeNormal();
                planeMesh.updateBuffer();

                const float s = 0.5f, cy = 0.8f;
                float cubeVerts[24 * 3] = {
                    -s, cy-s,  s,   s, cy-s,  s,   s, cy+s,  s,  -s, cy+s,  s,
                     s, cy-s, -s,  -s, cy-s, -s,  -s, cy+s, -s,   s, cy+s, -s,
                     s, cy-s,  s,   s, cy-s, -s,   s, cy+s, -s,   s, cy+s,  s,
                    -s, cy-s, -s,  -s, cy-s,  s,  -s, cy+s,  s,  -s, cy+s, -s,
                    -s, cy+s,  s,   s, cy+s,  s,   s, cy+s, -s,  -s, cy+s, -s,
                    -s, cy-s, -s,   s, cy-s, -s,   s, cy-s,  s,  -s, cy-s,  s,
                };
                unsigned int cubeIdx[12 * 3];
                for (int f = 0; f < 6; ++f) {
                    unsigned int b = f * 4;
                    cubeIdx[f*6+0] = b+0; cubeIdx[f*6+1] = b+1; cubeIdx[f*6+2] = b+2;
                    cubeIdx[f*6+3] = b+0; cubeIdx[f*6+4] = b+2; cubeIdx[f*6+5] = b+3;
                }
                float cubeNormals[24 * 3] = {0};
                MeshGL<CPU> cubeMesh(24, cubeVerts, 12, cubeIdx, cubeNormals);
                cubeMesh.computeNormal();
                cubeMesh.updateBuffer();

                // Shadow depth FBO — mirrors the render loop's setup.
                const int kRes = 512;
                GLuint sFbo = 0, sTex = 0;
                glGenTextures(1, &sTex);
                glBindTexture(GL_TEXTURE_2D, sTex);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kRes,
                             kRes, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
                const float kBorder[4] = {1, 1, 1, 1};
                glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, kBorder);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                                GL_COMPARE_REF_TO_TEXTURE);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
                glGenFramebuffers(1, &sFbo);
                glBindFramebuffer(GL_FRAMEBUFFER, sFbo);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                                       GL_TEXTURE_2D, sTex, 0);
                glDrawBuffer(GL_NONE);
                glReadBuffer(GL_NONE);
                bool fboOk = glCheckFramebufferStatus(GL_FRAMEBUFFER)
                             == GL_FRAMEBUFFER_COMPLETE;
                glBindFramebuffer(GL_FRAMEBUFFER, 0);

                if (!fboOk) {
                    skip("shadow-fbo", "depth-only FBO incomplete on this host");
                } else {
                    // Light VP — identical math to the render loop.
                    const tinym::vec3 lightDir =
                        tinym::vec3(50.0f, 50.0f, 30.0f).normalize();
                    const float R = 8.0f, zn = 1.0f, zf = 80.0f;
                    tinym::mat4 lightView = tinym::lookAt(
                        lightDir * 40.0f, tinym::vec3(0, 0, 0),
                        tinym::vec3(0, 1, 0));
                    tinym::mat4 lightProj(
                        tinym::vec4(1.0f / R, 0.0f, 0.0f, 0.0f),
                        tinym::vec4(0.0f, 1.0f / R, 0.0f, 0.0f),
                        tinym::vec4(0.0f, 0.0f, -2.0f / (zf - zn), 0.0f),
                        tinym::vec4(0.0f, 0.0f, -(zf + zn) / (zf - zn), 1.0f));
                    tinym::mat4 lightVP = lightProj * lightView;

                    Framebuffer fbo;
                    fbo.init(glctx.window, 256, 256);
                    TextureFormat tf;
                    tf.internalFormat = GL_RGBA8;
                    tf.format         = GL_RGBA;
                    tf.type           = GL_UNSIGNED_BYTE;
                    fbo.attachTexture2D(1, tf, 256, 256);
                    fbo.attachRenderBuffer(GL_DEPTH_COMPONENT24);

                    auto renderScene = [&](int shadowsOn) -> double {
                        if (shadowsOn) {
                            glBindFramebuffer(GL_FRAMEBUFFER, sFbo);
                            glViewport(0, 0, kRes, kRes);
                            glEnable(GL_DEPTH_TEST);
                            glClear(GL_DEPTH_BUFFER_BIT);
                            depthSh.use();
                            depthSh.setUniform("LightVP", lightVP);
                            planeMesh.drawDepthOnly();
                            cubeMesh.drawDepthOnly();
                            glBindFramebuffer(GL_FRAMEBUFFER, 0);
                        }
                        fbo.bind();
                        glViewport(0, 0, 256, 256);
                        glEnable(GL_DEPTH_TEST);
                        glClearColor(0, 0, 0, 1);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        mainSh.use();
                        tinym::mat4 M(1.0f);
                        tinym::mat4 V = tinym::lookAt(
                            tinym::vec3(0.0f, 4.0f, 4.0f),
                            tinym::vec3(0.0f, 0.0f, 0.0f),
                            tinym::vec3(0.0f, 1.0f, 0.0f));
                        tinym::mat4 P = tinym::perspective(0.7854f, 1.0f,
                                                           0.1f, 100.0f);
                        mainSh.setUniform("M", M);
                        mainSh.setUniform("V", V);
                        mainSh.setUniform("P", P);
                        mainSh.setUniform("LightVP", lightVP);
                        mainSh.setUniform("lightColor", tinym::vec3(3, 3, 3));
                        // Outline path off (no id buffer bound).
                        mainSh.setUniform("hoveredId", -1);
                        mainSh.setUniform("selectedId", -1);
                        glActiveTexture(GL_TEXTURE2);
                        glBindTexture(GL_TEXTURE_2D, sTex);
                        mainSh.setUniform("shadowMap", 2);
                        mainSh.setUniform("shadowsOn", shadowsOn);
                        auto drawBoth = [&]() {
                            planeMesh.draw(mainSh, tinym::vec3(0.8f, 0.8f, 0.8f),
                                           0.0f, 0.8f, 1.0f, tinym::vec3(0.0f));
                            cubeMesh.draw(mainSh, tinym::vec3(0.8f, 0.8f, 0.8f),
                                          0.0f, 0.8f, 1.0f, tinym::vec3(0.0f));
                        };
                        drawBoth();
                        glFinish();
                        std::vector<uint8_t> px(256 * 256 * 4);
                        glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE,
                                     px.data());
                        fbo.unbind();
                        double sum = 0.0;
                        for (size_t i = 0; i < px.size(); i += 4)
                            sum += (px[i] + px[i+1] + px[i+2]) / 3.0;
                        return sum / (256.0 * 256.0);
                    };

                    const double meanLit    = renderScene(0);
                    const double meanShadow = renderScene(1);
                    const bool darker   = meanShadow < meanLit - 1.0;
                    const bool notBlack = meanShadow > 2.0;
                    if (darker && notBlack) {
                        pass("SHADOW-1 / depth pass darkens receiver (ambient survives)");
                    } else {
                        fail("SHADOW-1 / depth pass darkens receiver (ambient survives)",
                             "meanLit=" + std::to_string(meanLit)
                             + " meanShadow=" + std::to_string(meanShadow)
                             + " darker=" + std::to_string((int)darker)
                             + " notBlack=" + std::to_string((int)notBlack));
                    }

                    // GEOM-1: large floor stays lit at grazing camera
                    // angles. Regression for the shader.geom edge-distance
                    // NaN — a 50-unit quad viewed from inside its footprint
                    // puts vertices behind / barely in front of the near
                    // plane; the projected coordinates blow up and a NaN in
                    // GEdgeDistance painted whole faces LineColor (black).
                    {
                        const float bg = 25.0f;
                        float bigVerts[4 * 3] = {
                            -bg, 0,  bg,   bg, 0,  bg,
                             bg, 0, -bg,  -bg, 0, -bg,
                        };
                        unsigned int bigIdx[6] = {0, 1, 2, 0, 2, 3};
                        float bigNormals[4 * 3] = {0};
                        MeshGL<CPU> bigMesh(4, bigVerts, 2, bigIdx, bigNormals);
                        bigMesh.computeNormal();
                        bigMesh.updateBuffer();

                        auto grazeMean = [&](tinym::vec3 eye,
                                             tinym::vec3 look) {
                            fbo.bind();
                            glViewport(0, 0, 256, 256);
                            glEnable(GL_DEPTH_TEST);
                            glClearColor(0, 0, 0, 1);
                            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                            mainSh.use();
                            tinym::mat4 M(1.0f);
                            tinym::mat4 V = tinym::lookAt(
                                eye, look, tinym::vec3(0, 1, 0));
                            tinym::mat4 P = tinym::perspective(
                                0.7854f, 1.0f, 0.1f, 1000.0f);
                            mainSh.setUniform("M", M);
                            mainSh.setUniform("V", V);
                            mainSh.setUniform("P", P);
                            mainSh.setUniform("LightVP", lightVP);
                            mainSh.setUniform("lightColor", tinym::vec3(3, 3, 3));
                            mainSh.setUniform("hoveredId", -1);
                            mainSh.setUniform("selectedId", -1);
                            glActiveTexture(GL_TEXTURE2);
                            glBindTexture(GL_TEXTURE_2D, sTex);
                            mainSh.setUniform("shadowMap", 2);
                            mainSh.setUniform("shadowsOn", 0);
                            bigMesh.draw(mainSh, tinym::vec3(0.8f, 0.8f, 0.8f),
                                         0.0f, 0.8f, 1.0f, tinym::vec3(0.0f));
                            glFinish();
                            std::vector<uint8_t> px(256 * 256 * 4);
                            glReadPixels(0, 0, 256, 256, GL_RGBA,
                                         GL_UNSIGNED_BYTE, px.data());
                            fbo.unbind();
                            double sum = 0.0;
                            for (size_t i = 0; i < px.size(); i += 4)
                                sum += (px[i] + px[i+1] + px[i+2]) / 3.0;
                            return sum / (256.0 * 256.0);
                        };

                        // Inside the floor footprint, low, looking outward
                        // — corners land behind / hugging the near plane.
                        const double m1 = grazeMean({0, 1, 5}, {0, 0, -10});
                        const double m2 = grazeMean({10, 0.5f, 10},
                                                    {-20, 0, -20});
                        const double m3 = grazeMean({0, 0.3f, 0}, {25, 0, 0});
                        const double worst = std::min(m1, std::min(m2, m3));
                        // Floor fills roughly the lower half of the frame;
                        // lit gray averaged with black sky clears this
                        // easily. NaN-black floors measured ~0-3 here.
                        if (worst > 20.0)
                            pass("GEOM-1 / large floor lit at grazing angles (no NaN blackout)");
                        else
                            fail("GEOM-1 / large floor lit at grazing angles (no NaN blackout)",
                                 "means " + std::to_string(m1) + " "
                                 + std::to_string(m2) + " "
                                 + std::to_string(m3));
                    }
                }
                glDeleteFramebuffers(1, &sFbo);
                glDeleteTextures(1, &sTex);
            }
        }
    }


    // ---- Block 26: BDD-018 — Inspector edits propagate live. ----------------
    // TESTS.md#BDD-018 wording (verbatim, *not* the matrix-row label):
    //   Given a running simulation with an object selected
    //   When  the user changes the object's color or behavior tag in the
    //         inspector
    //   Then  the change is visible in the very next rendered frame; if the
    //         behavior changed, the next simulation step dispatches through
    //         the new behavior.
    //   Notes: must not require pause/resume of the simulation.
    //
    // Mechanization shape: option (b) from the slice brief — construct a
    // mesh_inspector::MeshInspectorTarget exactly the way production does at
    // src/main.cpp:8349-8387 (same field assignments, same lambda bodies that
    // wrap Simulator::setMaterial / translateObject / rotateObject) and
    // invoke each callback directly with synthetic edit values. This
    // mechanizes the seam between inspector UI and Simulator without
    // introducing an ImGui-side harness; ImGui itself stays upstream of the
    // seam and is covered by the user's manual visual gate.
    //
    // Spec substitutions documented per PLANNER.md spec-substitution
    // discipline:
    //   - "color or behavior tag" → harness covers color (D-005 5-tuple via
    //     D-027 setMaterial) AND translate (D-014) AND rotate (D-021) — the
    //     three inspector edit paths implemented today. Behavior-tag edit is
    //     parked under BDD-006 / Q2 (in-place behavior switching). Returns
    //     to scope when BDD-006 ships.
    //   - "visible in the very next rendered frame" → mechanized as
    //     "the value the renderer reads each frame (mesh.material.* /
    //     mesh.transformPosition / state.x) is updated in place by the time
    //     the callback returns." No FBO render; this BDD's load-bearing
    //     claim is on the *input* data path, not output pixels.
    //   - "must not require pause/resume" → mechanized as "no
    //     sim.initialize() between callback fire and next sim.update()."
    //
    // Stricter-than-spec per PLANNER.md procedure step 7: each clause
    // asserts exactly-one-setter-call via a counter captured by reference,
    // not just "value propagated somewhere." Double-fire bugs in the
    // inspector wiring (a regression that would invoke setMaterial twice
    // per widget interaction) would slip past a loose assertion.
    //
    // Float cube at origin: Float pins state.x against gravity so the
    // pre/post-update state.x is stable for strict-equality witness math.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        const int inspId = 0;

        auto* m0 = Scene<Backend, Precision>::findById(inspId);
        if (!m0) {
            fail("BDD-018 / material inspector edit propagates live",
                 "cube id=" + std::to_string(inspId) + " not found after init");
        } else {
            // Pump 1 frame to let any cold-start drift settle. Float
            // integrator is a no-op so this is defensive; state.x stays
            // pinned at the addCube layout.
            pumpFrames(sim, 1);

            // Counters for exactly-one-setter-call assertions.
            int materialCalls = 0;
            int translateCalls = 0;
            int rotateCalls = 0;

            // Build target the way production does (src/main.cpp:8349-8387).
            // Same field assignments, same lambda bodies. The counters are
            // the only deviation — they let the harness assert "callback
            // fired exactly once" without modifying the Simulator side.
            mesh_inspector::MeshInspectorTarget target;
            target.mesh_id = m0->id;
            target.behavior_label = behaviorTypeName(m0->behaviorType);
            target.shape_label = shapeTypeName(m0->shapeType);
            target.base_color = &m0->material.baseColor;
            target.transform_position = &m0->transformPosition;
            target.on_translate = [&sim, &translateCalls](int id, tinym::vec3 v) {
                ++translateCalls;
                sim.translateObject(id, v);
            };
            target.rotation_wxyz = &m0->rotationQuat.w;
            target.on_rotate = [&sim, &rotateCalls](int id, float w, float x,
                                                    float y, float z) {
                ++rotateCalls;
                sim.rotateObject(id, ::Quat{w, x, y, z});
            };
            target.metallic = &m0->material.metallic;
            target.roughness = &m0->material.roughness;
            target.specular_weight = &m0->material.specularWeight;
            target.emission_color = &m0->material.emissionColor;
            target.on_material_edit = [&sim, &materialCalls](int id,
                                                             tinym::vec3 bc,
                                                             float mt, float rg,
                                                             float sw,
                                                             tinym::vec3 ec) {
                ++materialCalls;
                ::Material m;
                m.baseColor = bc;
                m.metallic = mt;
                m.roughness = rg;
                m.specularWeight = sw;
                m.emissionColor = ec;
                sim.setMaterial(id, m);
            };

            // ---- Material clause ----
            // Synthetic widget edit: baseColor=red, metallic=0.5,
            // roughness=0.25, specularWeight=0.7, emission=zero.
            target.on_material_edit(target.mesh_id,
                                    tinym::vec3(1.0f, 0.0f, 0.0f),
                                    0.5f, 0.25f, 0.7f,
                                    tinym::vec3(0.0f, 0.0f, 0.0f));

            const float matTol = 1e-6f;
            auto* mMat = Scene<Backend, Precision>::findById(inspId);
            bool ptrAlias = mMat && (target.base_color == &mMat->material.baseColor);
            bool matValuesOk =
                materialCalls == 1 &&
                mMat &&
                std::abs(mMat->material.baseColor.r - 1.0f) < matTol &&
                std::abs(mMat->material.baseColor.g - 0.0f) < matTol &&
                std::abs(mMat->material.baseColor.b - 0.0f) < matTol &&
                std::abs(mMat->material.metallic - 0.5f) < matTol &&
                std::abs(mMat->material.roughness - 0.25f) < matTol &&
                std::abs(mMat->material.specularWeight - 0.7f) < matTol &&
                std::abs(mMat->material.emissionColor.x - 0.0f) < matTol &&
                std::abs(mMat->material.emissionColor.y - 0.0f) < matTol &&
                std::abs(mMat->material.emissionColor.z - 0.0f) < matTol;

            // "Must not require pause/resume" — pump once WITHOUT
            // sim.initialize() between callback fire and pump.
            sim.update();
            auto* mMatPost = Scene<Backend, Precision>::findById(inspId);
            bool matSurvivesStep =
                mMatPost &&
                std::abs(mMatPost->material.baseColor.r - 1.0f) < matTol &&
                std::abs(mMatPost->material.metallic - 0.5f) < matTol &&
                std::abs(mMatPost->material.roughness - 0.25f) < matTol;

            if (!matValuesOk || !matSurvivesStep || !ptrAlias) {
                fail("BDD-018 / material inspector edit propagates live",
                     "materialCalls=" + std::to_string(materialCalls)
                     + " ptrAlias=" + std::to_string((int)ptrAlias)
                     + " survivesStep=" + std::to_string((int)matSurvivesStep)
                     + " got baseColor=("
                     + std::to_string(mMat ? mMat->material.baseColor.r : -1.0f) + ","
                     + std::to_string(mMat ? mMat->material.baseColor.g : -1.0f) + ","
                     + std::to_string(mMat ? mMat->material.baseColor.b : -1.0f) + ")"
                     + " metallic=" + std::to_string(mMat ? mMat->material.metallic : -1.0f)
                     + " roughness=" + std::to_string(mMat ? mMat->material.roughness : -1.0f)
                     + " specWeight=" + std::to_string(mMat ? mMat->material.specularWeight : -1.0f));
            } else {
                pass("BDD-018 / material inspector edit propagates live");
            }

            // ---- Translate clause ----
            // Snapshot witness position right before translate so the delta
            // assertion is on a fresh baseline (one prior sim.update() ran
            // for the material clause; Float pins state.x but be defensive).
            auto* mPreT = Scene<Backend, Precision>::findById(inspId);
            const double pre_x = mPreT->state.x.ptr[0];
            const double pre_y = mPreT->state.x.ptr[1];
            const double pre_z = mPreT->state.x.ptr[2];

            target.on_translate(target.mesh_id, tinym::vec3(1.0f, 2.0f, 3.0f));

            const double posTol = 1e-5;
            auto* mT = Scene<Backend, Precision>::findById(inspId);
            bool transValuesOk =
                translateCalls == 1 &&
                mT &&
                std::abs(mT->transformPosition.x - 1.0f) < posTol &&
                std::abs(mT->transformPosition.y - 2.0f) < posTol &&
                std::abs(mT->transformPosition.z - 3.0f) < posTol &&
                std::abs(mT->state.x.ptr[0] - (pre_x + 1.0)) < posTol &&
                std::abs(mT->state.x.ptr[1] - (pre_y + 2.0)) < posTol &&
                std::abs(mT->state.x.ptr[2] - (pre_z + 3.0)) < posTol;

            // "Must not require pause/resume" — pump once WITHOUT
            // sim.initialize() between callback fire and pump.
            sim.update();
            auto* mTPost = Scene<Backend, Precision>::findById(inspId);
            bool transSurvivesStep =
                mTPost &&
                std::abs(mTPost->state.x.ptr[0] - (pre_x + 1.0)) < posTol &&
                std::abs(mTPost->state.x.ptr[1] - (pre_y + 2.0)) < posTol &&
                std::abs(mTPost->state.x.ptr[2] - (pre_z + 3.0)) < posTol;

            if (!transValuesOk || !transSurvivesStep) {
                fail("BDD-018 / translate inspector edit propagates live",
                     "translateCalls=" + std::to_string(translateCalls)
                     + " survivesStep=" + std::to_string((int)transSurvivesStep)
                     + " tp=(" + std::to_string(mT ? mT->transformPosition.x : -1.0f) + ","
                     + std::to_string(mT ? mT->transformPosition.y : -1.0f) + ","
                     + std::to_string(mT ? mT->transformPosition.z : -1.0f) + ")"
                     + " state.x[0]=(" + std::to_string(mT ? mT->state.x.ptr[0] : 0.0) + ","
                     + std::to_string(mT ? mT->state.x.ptr[1] : 0.0) + ","
                     + std::to_string(mT ? mT->state.x.ptr[2] : 0.0) + ")"
                     + " expected=(" + std::to_string(pre_x + 1.0) + ","
                     + std::to_string(pre_y + 2.0) + ","
                     + std::to_string(pre_z + 3.0) + ")");
            } else {
                pass("BDD-018 / translate inspector edit propagates live");
            }

            // ---- Rotate clause ----
            // 90°-Z quaternion: q = (cos(45°), 0, 0, sin(45°)).
            // Pivot = transformPosition = (1, 2, 3) after translate clause.
            // Pre-rotate world position of vertex 0 = (pre_x + 1, pre_y + 2,
            // pre_z + 3). Offset from pivot = (pre_x, pre_y, pre_z).
            // 90°-Z rotation maps (dx, dy, dz) → (-dy, dx, dz).
            // Post-rotate world = pivot + (-pre_y, pre_x, pre_z)
            //                   = (1 - pre_y, 2 + pre_x, 3 + pre_z).
            // Hand-computed witness math (NOT via D-022's rotateVector) to
            // avoid using the implementation to verify the implementation.
            constexpr float kPi26 = 3.14159265358979323846f;
            const float halfAngle26 = (kPi26 * 0.5f) * 0.5f;  // 90° / 2
            ::Quat q90Z{std::cos(halfAngle26), 0.0f, 0.0f, std::sin(halfAngle26)};

            target.on_rotate(target.mesh_id, q90Z.w, q90Z.x, q90Z.y, q90Z.z);

            auto* mR = Scene<Backend, Precision>::findById(inspId);
            const double quatTol = 1e-5;
            bool rotValuesOk =
                rotateCalls == 1 &&
                mR &&
                std::abs(mR->rotationQuat.w - q90Z.w) < quatTol &&
                std::abs(mR->rotationQuat.x - q90Z.x) < quatTol &&
                std::abs(mR->rotationQuat.y - q90Z.y) < quatTol &&
                std::abs(mR->rotationQuat.z - q90Z.z) < quatTol &&
                std::abs(mR->state.x.ptr[0] - (1.0 - pre_y)) < posTol &&
                std::abs(mR->state.x.ptr[1] - (2.0 + pre_x)) < posTol &&
                std::abs(mR->state.x.ptr[2] - (3.0 + pre_z)) < posTol;

            // "Must not require pause/resume" — pump once WITHOUT
            // sim.initialize() between callback fire and pump.
            sim.update();
            auto* mRPost = Scene<Backend, Precision>::findById(inspId);
            bool rotSurvivesStep =
                mRPost &&
                std::abs(mRPost->state.x.ptr[0] - (1.0 - pre_y)) < posTol &&
                std::abs(mRPost->state.x.ptr[1] - (2.0 + pre_x)) < posTol &&
                std::abs(mRPost->state.x.ptr[2] - (3.0 + pre_z)) < posTol;

            if (!rotValuesOk || !rotSurvivesStep) {
                fail("BDD-018 / rotate inspector edit propagates live",
                     "rotateCalls=" + std::to_string(rotateCalls)
                     + " survivesStep=" + std::to_string((int)rotSurvivesStep)
                     + " rotQuat=(" + std::to_string(mR ? mR->rotationQuat.w : -1.0f) + ","
                     + std::to_string(mR ? mR->rotationQuat.x : -1.0f) + ","
                     + std::to_string(mR ? mR->rotationQuat.y : -1.0f) + ","
                     + std::to_string(mR ? mR->rotationQuat.z : -1.0f) + ")"
                     + " state.x[0]=(" + std::to_string(mR ? mR->state.x.ptr[0] : 0.0) + ","
                     + std::to_string(mR ? mR->state.x.ptr[1] : 0.0) + ","
                     + std::to_string(mR ? mR->state.x.ptr[2] : 0.0) + ")"
                     + " expected=(" + std::to_string(1.0 - pre_y) + ","
                     + std::to_string(2.0 + pre_x) + ","
                     + std::to_string(3.0 + pre_z) + ")");
            } else {
                pass("BDD-018 / rotate inspector edit propagates live");
            }
        }
    }

    // ---- Block 27: D-034 — Program::loadShader returns programID=0 on missing files. ----
    // Mechanizes the D-034 loader-failure contract: after this slice's
    // refactor of `printLog()` / `linkShader()` to stop calling exit(1),
    // a `loadShader` call with nonexistent shader files must return
    // silently with `programID == 0`. Block 25's documented
    // `if (!fboShader.programID) skip(...)` path now actually fires
    // when shaders are missing — previously the loader aborted the
    // process via printLog → exit(1) before the SKIP could observe.
    //
    // Bug-probe (a): temporarily revert printLog's exit(1) removal in
    // include/program.hpp and rebuild → Block 27's loadShader call
    // aborts the harness mid-test instead of returning cleanly. That
    // probe is load-bearing for the fix — the loud-FAIL diagnostic
    // and the harness's process-abort are the two observable
    // failure modes the loud-FAIL test must distinguish.
    //
    // SKIP-safe: if GLFW/GLEW init fails (no display server / no GL
    // driver), skip rather than fail. Same discipline as Block 25.
    {
        HiddenGLContext glctx(64, 64);
        if (!glctx.ok) {
            skip("loadshader-no-gl",
                 "GL context unavailable — loader contract check needs a "
                 "live program to glCreateProgram against");
        } else {
            Program p;
            p.loadShader("__ysim_nonexistent_vert__.vert",
                         "__ysim_nonexistent_geom__.geom",
                         "__ysim_nonexistent_frag__.frag");
            if (p.programID == 0) {
                pass("D-034 / Program::loadShader returns programID=0 on missing shader files");
            } else {
                fail("D-034 / Program::loadShader returns programID=0 on missing shader files",
                     "expected programID=0 after loading 3 nonexistent shader files, "
                     "got programID=" + std::to_string(p.programID)
                     + " — loader's failure path still doesn't honor the SKIP contract");
            }
        }
    }

    // ---- Block 28: D-035 — Quat ⇌ AxisAngle / Euler XYZ round-trip math. ----
    // FR-004 Notes: "UI may expose Euler/axis-angle as input affordances."
    // The inspector's rotation panel uses the new free-function helpers
    // (quatFromAxisAngle / quatToAxisAngle / quatFromEulerXYZ /
    // quatToEulerXYZ) to convert between the canonical Quat storage and
    // user-facing input modes. Widget rendering is manual-test-only
    // (no headless ImGui in the harness); this block covers ONLY the
    // math layer.
    //
    // Conventions (D-035):
    //   - Euler XYZ is intrinsic Tait-Bryan (R_z * R_y * R_x — apply X
    //     first in body frame, then Y, then Z). Matches Blender default.
    //   - Axis-angle output is in [0, π] with auto-normalized axis;
    //     identity-quat fallback returns angle=0 axis=(1, 0, 0).
    //   - Gimbal-lock fallback for Euler extraction is lossy by design.
    //     Round-trip tests SKIP singular inputs deliberately.
    //
    // Bug-probe (a): swap q.x / q.y inside quatFromAxisAngle → AxisAngle
    //   round-trip clause FAILs.
    // Bug-probe (b): change Euler `from` order to qx * qy * qz without
    //   changing `to` → Euler round-trip clause FAILs (the two
    //   conventions disagree and the round-trip drifts).
    {
        const float roundTripTol = 1e-5f;
        auto quatComponentEqual = [](const ::Quat& a, const ::Quat& b, float tol) {
            return std::abs(a.w - b.w) < tol
                && std::abs(a.x - b.x) < tol
                && std::abs(a.y - b.y) < tol
                && std::abs(a.z - b.z) < tol;
        };
        // Turn-30 fix: q and -q are the same rotation. After D-035's
        // antipodal canonicalization at extraction time, the rebuilt
        // quat is always in the qBack.w >= 0 form, so a negative-w
        // input rebuilds to its positive-w antipode. The assertion
        // accepts either componentwise match or sign-flipped match.
        auto quatAntipodalEqual = [&](const ::Quat& a, const ::Quat& b, float tol) {
            ::Quat negB{-b.w, -b.x, -b.y, -b.z};
            return quatComponentEqual(a, b, tol)
                || quatComponentEqual(a, negB, tol);
        };

        // --- AxisAngle round-trip clause ---
        // Three non-degenerate forward-built probes + identity edge +
        // two direct-construction negative-w probes (turn-30 BLOCK
        // regression protection: pre-fix, q.w < 0 inputs hit
        // s = sqrt(1 - qw²) = 0 → divide-by-zero NaN).
        struct AxisAngleProbe {
            ::Quat q;
            const char* label;
        };
        // Forward-built quats (positive-w by construction since
        // quatFromAxisAngle uses cos(angle/2) with angle ∈ [0, π]).
        ::Quat qForward1 = quatFromAxisAngle(tinym::vec3(1.0f, 0.0f, 0.0f), 0.5f);
        ::Quat qForward2 = quatFromAxisAngle(tinym::vec3(0.0f, 1.0f, 0.0f), 1.5f);
        ::Quat qForward3 = quatFromAxisAngle(tinym::vec3(1.0f, 1.0f, 1.0f), 2.0f);

        // Turn-30 negative-w probes: directly synthesize quats with
        // q.w < 0 to exercise the antipodal canonicalization at the
        // extractor entry. q = (-1, 0, 0, 0) is the antipodal identity
        // (also the result of a 360° rotation around any axis); q with
        // -cos(π/4) is the antipodal form of a 90°-Y rotation.
        const float kHalfPi4_cos = std::cos(3.14159265358979323846f * 0.25f);
        const float kHalfPi4_sin = std::sin(3.14159265358979323846f * 0.25f);
        ::Quat qAntipodalIdent{-1.0f, 0.0f, 0.0f, 0.0f};
        ::Quat qAntipodal90Y{-kHalfPi4_cos, 0.0f, -kHalfPi4_sin, 0.0f};

        const AxisAngleProbe aaProbes[] = {
            { qForward1, "x-axis 0.5 rad (positive-w)" },
            { qForward2, "y-axis 1.5 rad (positive-w)" },
            { qForward3, "diagonal 2.0 rad (positive-w)" },
            { qAntipodalIdent, "antipodal identity q=(-1,0,0,0) — turn-30 BLOCK probe" },
            { qAntipodal90Y, "antipodal 90°-Y q=(-c,0,-s,0) — turn-30 BLOCK probe" },
        };

        bool aaOk = true;
        std::string aaFailReason;
        for (const auto& probe : aaProbes) {
            tinym::vec3 axisOut;
            float angleOut = 0.0f;
            quatToAxisAngle(probe.q, axisOut, angleOut);
            ::Quat qBack = quatFromAxisAngle(axisOut, angleOut);
            // Reject NaN/inf explicitly — without the fix, the
            // negative-w probes return NaN axis components and
            // quatFromAxisAngle would propagate them; the antipodal-
            // equal comparison would then short-circuit to false but
            // the failure reason should call out the corruption.
            bool finite = std::isfinite(qBack.w) && std::isfinite(qBack.x)
                       && std::isfinite(qBack.y) && std::isfinite(qBack.z);
            if (!finite || !quatAntipodalEqual(probe.q, qBack, roundTripTol)) {
                aaOk = false;
                aaFailReason = std::string("AxisAngle round-trip drifted for ")
                             + probe.label
                             + ": q=(" + std::to_string(probe.q.w) + "," + std::to_string(probe.q.x)
                             + "," + std::to_string(probe.q.y) + "," + std::to_string(probe.q.z) + ")"
                             + " axisOut=(" + std::to_string(axisOut.x) + ","
                             + std::to_string(axisOut.y) + "," + std::to_string(axisOut.z) + ")"
                             + " angleOut=" + std::to_string(angleOut)
                             + " qBack=(" + std::to_string(qBack.w) + "," + std::to_string(qBack.x)
                             + "," + std::to_string(qBack.y) + "," + std::to_string(qBack.z) + ")"
                             + " finite=" + std::to_string((int)finite);
                break;
            }
        }
        // Identity edge: angle=0 axis canonical fallback.
        if (aaOk) {
            ::Quat qIdent{1.0f, 0.0f, 0.0f, 0.0f};
            tinym::vec3 axisOut;
            float angleOut = 1.0f;  // sentinel; expect overwritten to 0.
            quatToAxisAngle(qIdent, axisOut, angleOut);
            if (std::abs(angleOut) > roundTripTol
                || std::abs(axisOut.x - 1.0f) > roundTripTol
                || std::abs(axisOut.y) > roundTripTol
                || std::abs(axisOut.z) > roundTripTol) {
                aaOk = false;
                aaFailReason = "identity-quat fallback: expected angle=0 axis=(1,0,0), "
                               "got angle=" + std::to_string(angleOut)
                             + " axis=(" + std::to_string(axisOut.x) + ","
                             + std::to_string(axisOut.y) + "," + std::to_string(axisOut.z) + ")";
            }
        }

        if (aaOk) {
            pass("D-035 / Quat ⇌ AxisAngle round-trips within 1e-5 for non-degenerate inputs");
        } else {
            fail("D-035 / Quat ⇌ AxisAngle round-trips within 1e-5 for non-degenerate inputs",
                 aaFailReason);
        }

        // --- Euler XYZ round-trip clause ---
        // Test points: 3 non-singular triples. Avoid pitch (Y) near ±π/2
        // (gimbal lock band). The round-trip is asserted on Quat
        // components, not on Euler triples, because Euler equivalence is
        // modulo 2π and the extracted triple may differ from the input
        // even when the resulting rotation is the same.
        struct EulerProbe {
            float x, y, z;
            const char* label;
        };
        const EulerProbe eulerProbes[] = {
            { 0.1f, 0.2f, 0.3f, "(0.1, 0.2, 0.3) rad" },
            { -0.4f, 0.5f, -0.6f, "(-0.4, 0.5, -0.6) rad" },
            { 1.0f, 0.0f, 0.0f, "(1.0, 0.0, 0.0) rad — pure X rotation" },
        };

        bool eulerOk = true;
        std::string eulerFailReason;
        for (const auto& probe : eulerProbes) {
            ::Quat q = quatFromEulerXYZ(probe.x, probe.y, probe.z);
            float xOut = 0.0f, yOut = 0.0f, zOut = 0.0f;
            quatToEulerXYZ(q, xOut, yOut, zOut);
            ::Quat qBack = quatFromEulerXYZ(xOut, yOut, zOut);
            if (!quatComponentEqual(q, qBack, roundTripTol)) {
                eulerOk = false;
                eulerFailReason = std::string("Euler XYZ round-trip drifted for ")
                                + probe.label
                                + ": q=(" + std::to_string(q.w) + "," + std::to_string(q.x)
                                + "," + std::to_string(q.y) + "," + std::to_string(q.z) + ")"
                                + " qBack=(" + std::to_string(qBack.w) + "," + std::to_string(qBack.x)
                                + "," + std::to_string(qBack.y) + "," + std::to_string(qBack.z) + ")"
                                + " extracted=(" + std::to_string(xOut) + ","
                                + std::to_string(yOut) + "," + std::to_string(zOut) + ")";
                break;
            }
        }

        if (eulerOk) {
            pass("D-035 / Quat ⇌ Euler XYZ round-trips within 1e-5 for non-singular angles");
        } else {
            fail("D-035 / Quat ⇌ Euler XYZ round-trips within 1e-5 for non-singular angles",
                 eulerFailReason);
        }
    }

    // ---- Block 29: BDD-006 — Assign behavior type to an object. ------------
    // TESTS.md#BDD-006 wording (verbatim):
    //   Given an object with BehaviorType::Float
    //   When  the user changes its behavior to Cloth via the inspector
    //   Then  the object's behavior tag is Cloth, its parameter struct
    //         is populated with cloth defaults, and the next simulation
    //         step dispatches it through the cloth pipeline.
    //   Notes: cover all three v1 behaviors (Float, Cloth, Rigid).
    //          Selecting a reserved-but-not-shipped behavior must not be
    //          possible from the v1 UI.
    //
    // Mechanization: Simulator::changeBehavior(meshId, newType) is the
    // in-place mutator (D-036). Block 29 covers Float ↔ TriangularCloth
    // (BDD's literal scenario) plus FastGridCloth grid-only enforcement
    // (Q2's UX constraint). Rigid dispatch is parked under
    // BDD-006-RIGID-DISPATCH-PARKED (slice B-3); the harness doesn't
    // assert Rigid pipeline dispatch — applyEnvironmentForces dispatch
    // above zeros external forces only for Float, so a Rigid-tagged
    // mesh accumulates gravity force into externalForces but the
    // integrator's spring-force path skips Rigid, leaving state.x
    // unchanged-ish (kinematic-ish). Outside Block 29's scope.
    //
    // Bug-probes (LOAD-BEARING):
    //   (a) comment out mesh->behaviorType = newType in changeBehavior
    //       → Clause 1 + Clause 2 FAIL (tag stuck at original value).
    //   (b) skip the dynamic_cast<MeshGridInitializer> guard for
    //       FastGridCloth → Clause 3 FAILs (cube accepted as
    //       FastGridCloth-eligible).
    //   (c) make Reserved (Elastic) return true instead of false
    //       → Clause 4 (defensive) FAILs.
    {
        auto* device = MetalGlobalContext::getDevice();
        if (!device) {
            // The Metal-less host already SKIPped at the top of
            // runSelfTest; reaching this point implies Metal IS
            // present, so we don't re-skip here.
        }

        // --- Clause 1: Float → TriangularCloth dispatch (cube falls under gravity) ---
        {
            resetScene();
            sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            const int bhMeshId = 0;
            auto* m0 = Scene<Backend, Precision>::findById(bhMeshId);
            if (!m0) {
                fail("BDD-006 / Float → TriangularCloth in-place; tag + cloth defaults + cloth pipeline",
                     "cube id=0 not found after init");
            } else {
                if (m0->behaviorType != BehaviorType::Float) {
                    fail("BDD-006 / Float → TriangularCloth in-place; tag + cloth defaults + cloth pipeline",
                         "pre-changeBehavior tag was not Float — test scaffolding broken");
                } else {
                    // Pump 1 frame as Float — state.x stable.
                    pumpFrames(sim, 1);
                    auto* mFloat = Scene<Backend, Precision>::findById(bhMeshId);
                    const Index nv = mFloat->state.x.size / 3;
                    double preMeanY = 0.0;
                    for (Index v = 0; v < nv; ++v) preMeanY += mFloat->state.x.ptr[v*3+1];
                    preMeanY /= (double)nv;

                    bool ok = sim.changeBehavior(bhMeshId, BehaviorType::TriangularCloth);
                    auto* mPost = Scene<Backend, Precision>::findById(bhMeshId);
                    bool tagOk = mPost && (mPost->behaviorType == BehaviorType::TriangularCloth);
                    bool paramsOk = false;
                    if (mPost) {
                        if (auto* clothP = std::get_if<ClothBehaviorParams<Precision>>(&mPost->behaviorParams)) {
                            paramsOk = (std::abs(clothP->stretch - 1e5f) < 1.0f)
                                    && (std::abs(clothP->thickness - 0.01f) < 1e-5f);
                        }
                    }

                    // Pump 1 frame as Cloth — state.x should drop under gravity.
                    pumpFrames(sim, 1);
                    auto* mAfterStep = Scene<Backend, Precision>::findById(bhMeshId);
                    double postMeanY = 0.0;
                    for (Index v = 0; v < nv; ++v) postMeanY += mAfterStep->state.x.ptr[v*3+1];
                    postMeanY /= (double)nv;
                    bool clothDispatched = (postMeanY < preMeanY - 1e-4);

                    if (ok && tagOk && paramsOk && clothDispatched) {
                        pass("BDD-006 / Float → TriangularCloth in-place; tag + cloth defaults + cloth pipeline");
                    } else {
                        fail("BDD-006 / Float → TriangularCloth in-place; tag + cloth defaults + cloth pipeline",
                             "changeBehavior=" + std::to_string((int)ok)
                             + " tagOk=" + std::to_string((int)tagOk)
                             + " paramsOk=" + std::to_string((int)paramsOk)
                             + " preMeanY=" + std::to_string(preMeanY)
                             + " postMeanY=" + std::to_string(postMeanY)
                             + " (expected postMeanY < preMeanY under gravity)");
                    }
                }
            }
        }

        // --- Clause 2: TriangularCloth → Float dispatch (D-041 re-init) ---
        // Continue from clause 1's scene (cloth mid-fall). Switch back to
        // Float and confirm: (i) tag transitions correctly, (ii) the new
        // D-041 dirty discipline re-initializes the scene on the next
        // update so the cloth resets to its initializer-derived spawn
        // positions (state.x reset; state.v zeroed). The OLD D-036 "state.x
        // preserved" contract is intentionally invalidated by D-041 —
        // behavior change is one of the three operations that now forces
        // a full pack rebuild. Stability of the cloth POST-rebuild (Float
        // pipeline applies no forces) is verified by snapshotting the
        // post-rebuild state and re-pumping.
        {
            const int bhMeshId = 0;
            auto* mPre = Scene<Backend, Precision>::findById(bhMeshId);
            if (!mPre) {
                fail("BDD-006 / TriangularCloth → Float in-place; tag + Float pipeline (D-041 re-init then stable)",
                     "mesh id=0 missing — clause 1 scaffold broken");
            } else if (mPre->behaviorType != BehaviorType::TriangularCloth) {
                fail("BDD-006 / TriangularCloth → Float in-place; tag + Float pipeline (D-041 re-init then stable)",
                     "pre-state not TriangularCloth — clause 1 left scene in wrong state");
            } else {
                bool ok = sim.changeBehavior(bhMeshId, BehaviorType::Float);

                // Pump 1 frame — this triggers D-041's dirty-init path
                // (changeBehavior set dirty=true; update sees it and runs
                // Simulator::initialize before the pause-check). State.x
                // is rebuilt from the initializer's spawn geometry.
                pumpFrames(sim, 1);
                auto* mPost = Scene<Backend, Precision>::findById(bhMeshId);
                bool tagOk = mPost && (mPost->behaviorType == BehaviorType::Float);

                // Now snapshot the POST-rebuild state.x and pump another
                // frame as Float — should be perfectly stable because
                // Float applies no forces (no integrator step for Float).
                const Index nv = mPost->state.x.size / 3;
                std::vector<float> snapshot(nv * 3);
                std::memcpy(snapshot.data(), mPost->state.x.ptr, sizeof(float) * nv * 3);

                pumpFrames(sim, 1);
                auto* mAfter = Scene<Backend, Precision>::findById(bhMeshId);
                bool stable = true;
                const float posTol = 1e-5f;
                for (Index v = 0; v < nv * 3; ++v) {
                    if (std::abs(mAfter->state.x.ptr[v] - snapshot[v]) > posTol) {
                        stable = false;
                        break;
                    }
                }

                if (ok && tagOk && stable) {
                    pass("BDD-006 / TriangularCloth → Float in-place; tag + Float pipeline (D-041 re-init then stable)");
                } else {
                    fail("BDD-006 / TriangularCloth → Float in-place; tag + Float pipeline (D-041 re-init then stable)",
                         "changeBehavior=" + std::to_string((int)ok)
                         + " tagOk=" + std::to_string((int)tagOk)
                         + " stable=" + std::to_string((int)stable)
                         + " (expected: D-041 reinit then 1 Float frame leaves state.x unchanged)");
                }
            }
        }

        // --- Clause 3: FastGridCloth grid-only enforcement (setter rejects on non-grid) ---
        {
            resetScene();
            sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            const int bhMeshId = 0;
            bool rejected = !sim.changeBehavior(bhMeshId, BehaviorType::FastGridCloth);
            auto* mCube = Scene<Backend, Precision>::findById(bhMeshId);
            bool tagUnchanged = mCube && (mCube->behaviorType == BehaviorType::Float);

            // Reverse direction: grid mesh should accept FastGridCloth.
            resetScene();
            sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                         tinym::vec3(0.0f, 0.25f, 0.0f));
            sim.initialize();
            const int gridMeshId = 0;
            bool accepted = sim.changeBehavior(gridMeshId, BehaviorType::FastGridCloth);
            auto* mGrid = Scene<Backend, Precision>::findById(gridMeshId);
            bool gridTagOk = mGrid && (mGrid->behaviorType == BehaviorType::FastGridCloth);
            bool gridParamsOk = false;
            if (mGrid) {
                if (auto* gp = std::get_if<FastGridClothBehaviorParams<Precision>>(&mGrid->behaviorParams)) {
                    gridParamsOk = (gp->particleNum1D == 4);
                }
            }

            if (rejected && tagUnchanged && accepted && gridTagOk && gridParamsOk) {
                pass("BDD-006 / FastGridCloth only valid on square-regular grid (setter reject; grid accept)");
            } else {
                fail("BDD-006 / FastGridCloth only valid on square-regular grid (setter reject; grid accept)",
                     "cube-rejected=" + std::to_string((int)rejected)
                     + " cube-tagUnchanged=" + std::to_string((int)tagUnchanged)
                     + " grid-accepted=" + std::to_string((int)accepted)
                     + " grid-tagOk=" + std::to_string((int)gridTagOk)
                     + " grid-paramsOk=" + std::to_string((int)gridParamsOk));
            }
        }

        // --- Clause 4 (defensive): Reserved-not-shipped behaviors rejected by setter ---
        {
            resetScene();
            sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            const int bhMeshId = 0;
            bool elasticRejected = !sim.changeBehavior(bhMeshId, BehaviorType::Elastic);
            bool fluidRejected = !sim.changeBehavior(bhMeshId, BehaviorType::Fluid);
            bool generatorRejected = !sim.changeBehavior(bhMeshId, BehaviorType::Generator);
            auto* m = Scene<Backend, Precision>::findById(bhMeshId);
            bool tagStillFloat = m && (m->behaviorType == BehaviorType::Float);

            if (elasticRejected && fluidRejected && generatorRejected && tagStillFloat) {
                pass("BDD-006 / reserved-not-shipped behaviors rejected by setter (Elastic / Fluid / Generator)");
            } else {
                fail("BDD-006 / reserved-not-shipped behaviors rejected by setter (Elastic / Fluid / Generator)",
                     "elasticRejected=" + std::to_string((int)elasticRejected)
                     + " fluidRejected=" + std::to_string((int)fluidRejected)
                     + " generatorRejected=" + std::to_string((int)generatorRejected)
                     + " tagStillFloat=" + std::to_string((int)tagStillFloat));
            }
        }

        // --- Clause 5: Rigid round-trip through saveScene/loadScene (D-036 turn-32 addendum) ---
        // User scenario: toggle a mesh to Rigid via inspector → save scene
        // → reload → Rigid tag must survive the round-trip. Pre fix-turn
        // 32, this broke because scene_format::isReservedBehavior listed
        // "Rigid" as reserved-not-shipped (rejected on load even though
        // changeBehavior accepted the tag at runtime).
        {
            resetScene();
            sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            const int bhMeshId = 0;
            bool tagSet = sim.changeBehavior(bhMeshId, BehaviorType::Rigid);
            auto* mPre = Scene<Backend, Precision>::findById(bhMeshId);
            bool preTagOk = mPre && (mPre->behaviorType == BehaviorType::Rigid);

            const std::string path = "/tmp/ysim_bdd006_rigid_roundtrip.ysim.json";
            std::string saveErr;
            bool saveOk = sim.saveScene(path, &saveErr);

            bool loadOk = false;
            bool postTagOk = false;
            std::string loadErr;
            if (saveOk) {
                auto lr = sim.loadScene(path);
                if (lr.ok) {
                    sim.initialize();
                    sim.applyPendingMaterials();
                    auto* mPost = Scene<Backend, Precision>::findById(bhMeshId);
                    postTagOk = mPost && (mPost->behaviorType == BehaviorType::Rigid);
                    loadOk = true;
                } else {
                    loadErr = lr.error.message;
                }
            }

            if (tagSet && preTagOk && saveOk && loadOk && postTagOk) {
                pass("BDD-006 / Rigid round-trip through saveScene/loadScene preserves the Rigid tag (D-036 addendum, turn-32 fix-turn)");
            } else {
                fail("BDD-006 / Rigid round-trip through saveScene/loadScene preserves the Rigid tag (D-036 addendum, turn-32 fix-turn)",
                     "tagSet=" + std::to_string((int)tagSet)
                     + " preTagOk=" + std::to_string((int)preTagOk)
                     + " saveOk=" + std::to_string((int)saveOk)
                     + " saveErr=" + saveErr
                     + " loadOk=" + std::to_string((int)loadOk)
                     + " loadErr=" + loadErr
                     + " postTagOk=" + std::to_string((int)postTagOk));
            }
            std::remove(path.c_str());
        }

        // --- Clause 6: changeBehavior immediately syncs broad-phase cached behavior arrays (D-036 turn-32 addendum) ---
        // The broad-phase reads `objTrees[idx].objBehavior` for the
        // per-pair Float-skip; after changeBehavior the cached value
        // would otherwise stay stale until the next full rebuild
        // (D-026's skip-rebuild gate matches on lifetimeId, which
        // doesn't change for in-place behavior switches). Clause asserts
        // the in-place sync writes happen immediately on the mutation.
        {
            resetScene();
            sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                        /*size=*/0.5f, /*mass=*/0.1f);
            sim.initialize();
            pumpFrames(sim, 1);  // populate broad-phase caches

            // Pre-state: cube is Float; objTrees objBehavior cache should
            // reflect that. objTrees gets rebuilt across resetScene
            // boundaries because the skip-rebuild gate at BroadPhase::build
            // requires `objBehavior == Float` (the value being checked),
            // so non-Float residuals from prior clauses naturally invalidate.
            //
            // shBroadPhase.meshBehaviors is NOT asserted pre-state: its
            // rebuildMeshKinds() short-circuits on (ptr && size==numMeshes),
            // so residual values from prior clauses can leak through (e.g.,
            // Clause 3 leaves it at FastGridCloth=1 even though the new
            // cube is Float=4). The cache-sync write is the only mechanism
            // that updates the mirror without a full rebuild — exactly
            // what this clause's post-state assertion exercises.
            bool preObjBehaviorOk = false;
            if (!sim.collisionPipeline.broadPhase.objTrees.empty()) {
                preObjBehaviorOk = (sim.collisionPipeline.broadPhase.objTrees[0].objBehavior
                                    == BehaviorType::Float);
            }

            bool changed = sim.changeBehavior(0, BehaviorType::TriangularCloth);

            // Post-state: BEFORE any further BroadPhase::build / refit /
            // rebuildMeshKinds, BOTH caches must reflect the new tag.
            // objTrees post-check is load-bearing (BVH/SCENE skip-gate
            // depends on it); meshBehaviors post-check is load-bearing
            // (GPU-side spatial-hashing depends on it).
            bool postObjBehaviorOk = false;
            if (!sim.collisionPipeline.broadPhase.objTrees.empty()) {
                postObjBehaviorOk = (sim.collisionPipeline.broadPhase.objTrees[0].objBehavior
                                     == BehaviorType::TriangularCloth);
            }
            bool postMeshBehaviorsOk = true;
            if (sim.shBroadPhase.meshBehaviors.ptr
                && sim.shBroadPhase.meshBehaviors.size > 0) {
                postMeshBehaviorsOk = (sim.shBroadPhase.meshBehaviors[0]
                                       == (uint32_t)BehaviorType::TriangularCloth);
            }

            if (changed && preObjBehaviorOk
                && postObjBehaviorOk && postMeshBehaviorsOk) {
                pass("BDD-006 / changeBehavior immediately syncs broad-phase cached behavior arrays (D-036 addendum, turn-32 fix-turn)");
            } else {
                fail("BDD-006 / changeBehavior immediately syncs broad-phase cached behavior arrays (D-036 addendum, turn-32 fix-turn)",
                     "changed=" + std::to_string((int)changed)
                     + " preObjBehaviorOk=" + std::to_string((int)preObjBehaviorOk)
                     + " postObjBehaviorOk=" + std::to_string((int)postObjBehaviorOk)
                     + " postMeshBehaviorsOk=" + std::to_string((int)postMeshBehaviorsOk));
            }
        }
    }

    // Block 30 + Block 31 (D-037 + D-038) relocated to ABOVE the Metal-less
    // SKIP gate near the top of runSelfTest — pure-C++ backends run on Linux
    // containers too. Folds Estimator turn 33 WARNING.

    // ---- Block 32: D-039 — Rigid behavior wires through Simulator. ---------
    // BDD-008's "falls" half: addCube + changeBehavior(0, Rigid) triggers
    // ensureRigidBackendBody; pumping update() steps the backend + applies
    // Δpos to state.x. Closes the user's "Rigid bodies should fall under
    // gravity" goal end-to-end (B-1 contract + B-2′ Euler + B-3 wiring).
    // Lives BELOW the Metal-less SKIP gate because Simulator::initialize
    // touches Metal — Linux SKIPs along with Block 1-29 per D-012.
    {
        resetScene();
        // Cube at y=5, tess=2 (8 vertices), size=0.2, mass=1.
        sim.addCube(tinym::vec3(0.0f, 5.0f, 0.0f), 2, 0.2f, 1.0f);
        sim.initialize();

        auto& m = Scene<Backend, Precision>::meshes[0];
        // Snapshot pre-state.  state.x[1] = vertex 0's y component.
        Precision y_initial = m.state.x.ptr[1];

        // Switch to Rigid — ensureRigidBackendBody fires.
        bool changed = sim.changeBehavior(0, BehaviorType::Rigid);
        bool handleOk = (m.rigidBodyHandle != ysim::physics::kInvalidBodyHandle);
        Precision center_y_initial = (Precision)m.rigidLastBodyPos.y;

        // Unpause so update() doesn't early-return.
        sim.pause = false;

        // Pump 30 frames at default h=1/60. With g=-9.81 semi-implicit:
        // Δy_total ≈ -g*h^2*N*(N+1)/2 ≈ -1.27 m at N=30. Vertex y_initial
        // should drop by > 0.5 m (well within the accumulated fall).
        for (int i = 0; i < 30; ++i) sim.update();

        Precision y_post = m.state.x.ptr[1];
        Precision center_y_post = (Precision)m.rigidLastBodyPos.y;

        bool vertexFellOk = (y_post < y_initial - Precision(0.5));
        bool centerFellOk = (center_y_post < center_y_initial - Precision(0.5));

        if (changed && handleOk && vertexFellOk && centerFellOk) {
            pass("BDD-008 / cube tagged Rigid falls under gravity in Simulator::update (D-039)");
        } else {
            fail("BDD-008 / cube tagged Rigid falls under gravity in Simulator::update (D-039)",
                 "changed=" + std::to_string((int)changed)
                 + " handleOk=" + std::to_string((int)handleOk)
                 + " vertexFellOk=" + std::to_string((int)vertexFellOk)
                 + " centerFellOk=" + std::to_string((int)centerFellOk)
                 + " y_initial=" + std::to_string(y_initial)
                 + " y_post=" + std::to_string(y_post)
                 + " center_y_initial=" + std::to_string(center_y_initial)
                 + " center_y_post=" + std::to_string(center_y_post));
        }

        // Re-pause so subsequent blocks aren't affected.
        sim.pause = true;
    }

    // ---- Block 33: D-040 — BDD-008 "rests at floor" via Bullet. -----------
    // Drops a Rigid-tagged cube onto an EXPLICIT scene-installed ground
    // plane (D-040 "scene-objects-only" addendum — no virtual y=0 plane;
    // Float grid registered as static Bullet body via
    // ensureRigidStaticGround). Pumps 240 frames (4 s @ 60 Hz). Asserts
    // body-center y ≈ ground_y + half_extent within tolerance.
    // Closes Estimator turn-35 BLOCK on BDD-008.
    {
        resetScene();
        // Ground at y=-1, XZ plane (matches main()'s default scene).
        // mesh-id 0 (Float-tagged grid).
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0.0f, -1.0f, 0.0f), 5.0f);
        // Cube above ground; size=0.2 → half-extent=0.1; starts at y=2.
        // mesh-id 1 (Float-tagged cube, will toggle to Rigid).
        sim.addCube(tinym::vec3(0.0f, 2.0f, 0.0f), 2, 0.2f, 1.0f);
        sim.initialize();

        bool changed = sim.changeBehavior(1, BehaviorType::Rigid);
        auto& m = Scene<Backend, Precision>::meshes[1];
        bool handleOk = (m.rigidBodyHandle != ysim::physics::kInvalidBodyHandle);

        sim.pause = false;
        for (int i = 0; i < 240; ++i) sim.update();

        const Precision center_y = (Precision)m.rigidLastBodyPos.y;
        // Ground at y=-1; cube half-extent=0.1; analytic rest center_y
        // = -1 + 0.1 = -0.9. Bullet's btStaticPlaneShape vs btBoxShape
        // contact processing has a sloppy margin (~0.04-0.08m of allowed
        // penetration under default solver settings + 1.0 mass + 0.5
        // friction). Empirically the cube settles at ~ -0.97 (≈7 cm
        // penetration). Accept [-1.05, -0.80] — covers default Bullet
        // residual without being so wide that no-collision passes.
        bool restPosOk = (center_y > Precision(-1.05) && center_y < Precision(-0.80));

        bool restVelOk = false;
        if (handleOk) {
            // Sample the rigid backend's reported velocities directly. The
            // delta-loop's rigidLastBodyPos doesn't carry velocity; query
            // through a small accessor — but rigid_ is private. Use a
            // proxy: post-rest, Δpos between consecutive frames should be
            // tiny. Step one more frame and compare lastBodyPos delta.
            const Precision prev_y = center_y;
            sim.update();
            const Precision next_y = (Precision)m.rigidLastBodyPos.y;
            restVelOk = (std::abs(next_y - prev_y) < Precision(0.001));
        }

        if (changed && handleOk && restPosOk && restVelOk) {
            pass("BDD-008 / cube tagged Rigid rests on explicit scene ground (addGround at y=-1) after 240 frames (D-040)");
        } else {
            fail("BDD-008 / cube tagged Rigid rests on explicit scene ground (addGround at y=-1) after 240 frames (D-040)",
                 "changed=" + std::to_string((int)changed)
                 + " handleOk=" + std::to_string((int)handleOk)
                 + " restPosOk=" + std::to_string((int)restPosOk)
                 + " restVelOk=" + std::to_string((int)restVelOk)
                 + " center_y=" + std::to_string(center_y));
        }

        sim.pause = true;
    }

    // ---- Block 34: compacted-id model — removeMesh + addX recompacts. -----
    // Original D-041 bug: deleting the middle mesh then adding one caused
    // a mesh.id collision (addGeneralMesh used numMeshes++; removeMesh
    // decremented it so addX reused a surviving id) → MeshRenderState GL
    // slot collision → render corruption.
    //
    // Redesign (per user directive): id is NOT a monotone identity. It is
    // the compacted [0, numMeshes) slot that governs all mesh access
    // (findById AND statesOffsets / objTrees / faceObj / objPair). add /
    // removeMesh / Scene::pack keep id == array index. removeMesh erases
    // then renumbers survivors and rebuilds the id-keyed render cache, so
    // the collision is impossible by construction (ids are always a
    // contiguous unique 0..n-1 run and renderState is re-registered under
    // the new ids). This block now asserts that NEW contract.
    {
        resetScene();
        // Mimic main scene structure: cloth + middle Float-import-like
        // mesh + ground. Use Cube + Ground primitives so the test doesn't
        // depend on an external .obj path.
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5f,
                     tinym::vec3(0.0f, 0.5f, 0.0f),
                     1e3, 1e3, 1e3, 0.01f, 0.1f);                 // id 0
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), 2, 0.2f, 0.1f); // id 1 (Human-like middle)
        sim.addGround(PlaneDirection::XZPlane,
                      tinym::vec3(0.0f, -1.0f, 0.0f), 5.0f);       // id 2
        sim.initialize();

        const int beforeIds[] = {0, 1, 2};
        bool beforeOk =
            Scene<Backend, Precision>::meshes.size() == 3 &&
            Scene<Backend, Precision>::meshes[0].id == beforeIds[0] &&
            Scene<Backend, Precision>::meshes[1].id == beforeIds[1] &&
            Scene<Backend, Precision>::meshes[2].id == beforeIds[2];

        // Remove the middle (id=1) — mirrors "Delete Human" from main.
        sim.removeMesh(1);

        // Add a new sphere. Under compaction: removeMesh(1) renumbers
        // survivors to [cloth=0, ground=1]; addSphere appends → id=2.
        sim.addSphere(tinym::vec3(2.0f, 0.0f, 0.0f), /*tess=*/3, /*size=*/0.2f);

        // Trigger pre-pause init via 1 update tick. (sim.pause stays
        // true; the dirty check fires before the pause-return.)
        pumpFrames(sim, 1);

        // After re-init: meshes are [cloth(0), ground(1), sphere(2)] —
        // a contiguous 0..n-1 run with id == array index for every slot.
        bool sizeOk = Scene<Backend, Precision>::meshes.size() == 3;
        bool clothOk  = sizeOk && Scene<Backend, Precision>::meshes[0].id == 0;
        bool groundOk = sizeOk && Scene<Backend, Precision>::meshes[1].id == 1;
        bool sphereOk = sizeOk && Scene<Backend, Precision>::meshes[2].id == 2;
        // The core compacted-id invariant: id == array index everywhere.
        // This is what makes objPair usable as both a findById key and a
        // statesOffsets subscript, and is what structurally rules out the
        // original MeshRenderState id-collision.
        bool idEqIndexOk = sizeOk;
        for (int i = 0; sizeOk && i < 3; ++i)
            idEqIndexOk &= (Scene<Backend, Precision>::meshes[i].id == i);
        bool uniqueOk = sizeOk &&
            (Scene<Backend, Precision>::meshes[0].id !=
             Scene<Backend, Precision>::meshes[1].id) &&
            (Scene<Backend, Precision>::meshes[0].id !=
             Scene<Backend, Precision>::meshes[2].id) &&
            (Scene<Backend, Precision>::meshes[1].id !=
             Scene<Backend, Precision>::meshes[2].id);

        if (beforeOk && sizeOk && clothOk && groundOk && sphereOk
            && idEqIndexOk && uniqueOk) {
            pass("compacted-id / removeMesh + addX recompacts ids to contiguous 0..n-1 (id == index, no MeshRenderState collision)");
        } else {
            fail("compacted-id / removeMesh + addX recompacts ids to contiguous 0..n-1 (id == index, no MeshRenderState collision)",
                 "beforeOk=" + std::to_string((int)beforeOk)
                 + " sizeOk=" + std::to_string((int)sizeOk)
                 + " clothOk(0)=" + std::to_string((int)clothOk)
                 + " groundOk(1)=" + std::to_string((int)groundOk)
                 + " sphereOk(2)=" + std::to_string((int)sphereOk)
                 + " idEqIndexOk=" + std::to_string((int)idEqIndexOk)
                 + " uniqueOk=" + std::to_string((int)uniqueOk));
        }
    }

    // ---- Block 35: narrow-phase fires after middle-mesh removal. ----------
    // Original D-041 bug: deleting the middle mesh stopped cloth-vs-ground
    // narrow phase even though broad phase still paired them — objPair
    // carried mesh.id while the narrow kernel indexes
    // scenePackedPositionsOffsets[objPair] (an INDEX), and the old
    // monotone scheme let id ≠ index after a middle removal.
    //
    // Redesign: removeMesh recompacts ids so id == array index is RESTORED
    // immediately after the delete (not left divergent). That makes
    // objPair simultaneously a valid findById key and a valid
    // statesOffsets subscript, structurally eliminating the mismatch.
    // This block now asserts the post-remove state has id == index AND
    // that narrow phase actually fires for cloth-vs-ground.
    {
        resetScene();
        // Place cloth just above ground (gap ≈ 0.1 m at cloth-bottom)
        // so 10 outer frames is enough to make contact under gravity.
        sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5f,
                     tinym::vec3(0.0f, -0.5f, 0.0f),
                     1e3, 1e3, 1e3, 0.01f, 0.1f);                  // id 0 (TriangularCloth)
        sim.addCube(tinym::vec3(2.0f, 0.0f, 0.0f), 2, 0.2f, 0.1f);  // id 1 (Float; mimics "Human" middle mesh)
        sim.addGround(PlaneDirection::XZPlane,
                      tinym::vec3(0.0f, -1.0f, 0.0f), 2.0f);        // id 2 (Float-grid ground)
        sim.initialize();

        // Delete the middle mesh — this is the user's "Delete Human" step.
        sim.removeMesh(1);

        // After remove + recompaction: survivors are [cloth, ground] with
        // ids renumbered to [0, 1] == their array indices. objPair now
        // works as both a findById key and a statesOffsets subscript, so
        // narrow phase must fire for cloth-vs-ground.
        auto& packedCol = Scene<Backend, Precision>::packedCollisionData;
        packedCol.cumulativeNarrowCollisions = 0;
        sim.pause = false;

        // Pump enough frames for cloth to fall from y=-0.5 onto ground at
        // y=-1. With h=1/60 + cloth-side subStep=60 + cloth-bottom 0.25 m
        // above ground, 30 outer frames easily clears the gap.
        for (int i = 0; i < 30; ++i) sim.update();

        size_t cumNarrow = packedCol.cumulativeNarrowCollisions;
        bool clothId  = Scene<Backend, Precision>::meshes[0].id == 0;
        bool groundId = Scene<Backend, Precision>::meshes[1].id == 1;
        // The redesign's guarantee: after a middle-mesh removal ids are
        // recompacted so id == array index for every survivor (this is
        // what structurally eliminates the original objPair mismatch).
        bool idEqIndexOk =
            Scene<Backend, Precision>::meshes.size() == 2 &&
            Scene<Backend, Precision>::meshes[0].id == 0 &&
            Scene<Backend, Precision>::meshes[1].id == 1;

        if (clothId && groundId && idEqIndexOk && cumNarrow > 0) {
            pass("compacted-id / narrow-phase fires for cloth-vs-ground after middle-mesh removal (id == index restored)");
        } else {
            fail("compacted-id / narrow-phase fires for cloth-vs-ground after middle-mesh removal (id == index restored)",
                 "clothId(0)=" + std::to_string((int)clothId)
                 + " groundId(1)=" + std::to_string((int)groundId)
                 + " idEqIndexOk=" + std::to_string((int)idEqIndexOk)
                 + " cumNarrow=" + std::to_string(cumNarrow));
        }

        sim.pause = true;
    }

    // ---- Block 36: D-042 R-1 — preview populated at addX time. ------------
    // After R-1 each addX call invokes initializer->populatePreview() to
    // fill PreviewState.x / .facets / .n directly from the initializer's
    // parameters (no pool, no MeshState, no Scene::pack). MeshGL can bind
    // to these stable heap buffers — R-2 will wire that. Block 36 verifies
    // that all 4 initializer kinds (Cube/Sphere/Grid/File) populate
    // preview correctly: x.size > 0, facets.size > 0, n.size == x.size.
    // The vertex count matches the initializer's numPoints (sanity check).
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), 2, 0.2f, 0.1f);   // id 0: MeshCubeInitializer
        sim.addSphere(tinym::vec3(1.0f, 0.0f, 0.0f), 3, 0.2f);        // id 1: MeshSphereInitializer
        sim.addCloth(/*pn1D=*/4, 0.5f,
                     tinym::vec3(2.0f, 0.0f, 0.0f),
                     1e3, 1e3, 1e3, 0.01f, 0.1f);                    // id 2: MeshGridInitializer
        sim.addGround(PlaneDirection::XZPlane,
                      tinym::vec3(0.0f, -1.0f, 0.0f), 1.0f);          // id 3: MeshGridInitializer

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool sizeOk = reqs.size() == 4;

        bool cubeOk = sizeOk
            && reqs[0].preview.x.size() == reqs[0].initializer->getParams()->numPoints * 3
            && reqs[0].preview.facets.size() == reqs[0].initializer->getParams()->numFacets * 3
            && reqs[0].preview.n.size() == reqs[0].preview.x.size();
        bool sphereOk = sizeOk
            && reqs[1].preview.x.size() == reqs[1].initializer->getParams()->numPoints * 3
            && reqs[1].preview.facets.size() == reqs[1].initializer->getParams()->numFacets * 3
            && reqs[1].preview.n.size() == reqs[1].preview.x.size();
        bool clothOk = sizeOk
            && reqs[2].preview.x.size() == reqs[2].initializer->getParams()->numPoints * 3
            && reqs[2].preview.facets.size() == reqs[2].initializer->getParams()->numFacets * 3
            && reqs[2].preview.n.size() == reqs[2].preview.x.size();
        bool groundOk = sizeOk
            && reqs[3].preview.x.size() == reqs[3].initializer->getParams()->numPoints * 3
            && reqs[3].preview.facets.size() == reqs[3].initializer->getParams()->numFacets * 3
            && reqs[3].preview.n.size() == reqs[3].preview.x.size();

        if (cubeOk && sphereOk && clothOk && groundOk) {
            pass("D-042 R-1 / addX populates PreviewState immediately for all 4 initializer kinds (Cube/Sphere/Grid/File-ish)");
        } else {
            fail("D-042 R-1 / addX populates PreviewState immediately for all 4 initializer kinds (Cube/Sphere/Grid/File-ish)",
                 "sizeOk=" + std::to_string((int)sizeOk)
                 + " cubeOk=" + std::to_string((int)cubeOk)
                 + " sphereOk=" + std::to_string((int)sphereOk)
                 + " clothOk=" + std::to_string((int)clothOk)
                 + " groundOk=" + std::to_string((int)groundOk));
        }
    }

    // ---- Block 37: D-042 R-2 — MeshGL binds to PreviewState pointers. -----
    // R-1 populated `request.preview`. R-2 publishes those preview pointers
    // to MeshRenderState via `registerPreviewBinding` immediately after every
    // `scene.addGeneralMesh`. Block 37 verifies the binding is registered
    // BEFORE `simulator.initialize()` runs, proving the MeshGL materialization
    // path can pick up the stable preview pointers (instead of the volatile
    // packed sub-views). Pure-CPU verification — no GL context needed; the
    // actual MeshGL ctor still defers until `getOrCreate(mesh)` first fires
    // in the GUI render loop.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 1.0f, 0.0f), 2, 0.2f, 1.0f);

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        auto& req = reqs[0];
        const auto* binding = sim.renderState.previewBinding(req.id);
        bool bindingExists = (binding != nullptr);
        // 2026-05-15 (A2 split): binding follows the render-side pointers so
        // cube renders the unwelded per-face copy with flat normals. The
        // render*Ptr() accessors fall back to physics x/n/facets when the
        // initializer didn't populate the render topology (sphere/grid/file),
        // so this single check covers both paths.
        bool xMatch       = bindingExists && (binding->xPtr == (void*)req.preview.renderXPtr());
        bool numVMatch    = bindingExists && (binding->numVerts == req.preview.numRenderPoints()) && (binding->numVerts > 0);
        bool facetPMatch  = bindingExists && (binding->facetPtr == req.preview.renderFacetsPtr());
        bool facetCMatch  = bindingExists && (binding->numFacets == req.preview.numRenderFacets()) && (binding->numFacets > 0);
        bool nMatch       = bindingExists && (binding->normalPtr == (void*)req.preview.renderNPtr());

        if (oneReq && bindingExists && xMatch && numVMatch && facetPMatch && facetCMatch && nMatch) {
            pass("D-042 R-2 / addX registers PreviewBinding immediately (pre-initialize) with stable preview pointers");
        } else {
            fail("D-042 R-2 / addX registers PreviewBinding immediately (pre-initialize) with stable preview pointers",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " bindingExists=" + std::to_string((int)bindingExists)
                 + " xMatch="        + std::to_string((int)xMatch)
                 + " numVMatch="     + std::to_string((int)numVMatch)
                 + " facetPMatch="   + std::to_string((int)facetPMatch)
                 + " facetCMatch="   + std::to_string((int)facetCMatch)
                 + " nMatch="        + std::to_string((int)nMatch));
        }
    }

    // ---- Block 38: D-042 R-3 — Scene::pack memcpys preview to packed. -----
    // R-3 makes PreviewState the source of truth for vertex/facet/normal
    // data at pack time. The initializer's regen (mesh.initialize()) still
    // runs (preserves adjacency derivation), but R-3's memcpy block
    // overrides state.x / state.n / adjacency.facets with preview's data —
    // so any future edit to preview (R-4) propagates through Scene::pack.
    //
    // Sentinel mechanic: addCube populates preview.x with the cube's
    // natural geometry. We mutate preview.x[1] to 99.0f (distinct from
    // the cube's natural y at vertex 0 for tess=2 cube at origin).
    // sim.initialize() runs pack; pack's mesh.initialize() writes the
    // initializer's natural value first, then R-3's memcpy overrides
    // with 99.0f. Assertion checks that 99.0f survived into
    // packedMeshData.x.
    //
    // Bug-probe: removing the R-3 memcpy block → packed.x[1] retains the
    // initializer's natural value → assertion FAILs.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), 2, 0.2f, 1.0f);

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        bool hasPreviewData = oneReq
            && reqs[0].preview.x.size() >= 9
            && reqs[0].preview.facets.size() >= 9
            && reqs[0].preview.n.size() >= 9;

        const Precision sentinelY = Precision(99.0f);
        if (oneReq && reqs[0].preview.x.size() >= 9) {
            reqs[0].preview.x[1] = sentinelY;
        }

        sim.initialize();

        bool meshExists = !Scene<Backend, Precision>::meshes.empty();
        Precision packedY = meshExists ? Scene<Backend, Precision>::meshes[0].state.x[1] : Precision(0);
        bool packedHasSentinel = meshExists && (packedY == sentinelY);

        bool countMatch = meshExists
            && oneReq
            && (Scene<Backend, Precision>::meshes[0].state.x.size / 3
                == reqs[0].preview.numPoints());

        if (oneReq && hasPreviewData && meshExists && packedHasSentinel && countMatch) {
            pass("D-042 R-3 / Scene::pack memcpys preview x into packed sub-view (sentinel survives initializer regen)");
        } else {
            fail("D-042 R-3 / Scene::pack memcpys preview x into packed sub-view (sentinel survives initializer regen)",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " hasPreviewData=" + std::to_string((int)hasPreviewData)
                 + " meshExists=" + std::to_string((int)meshExists)
                 + " packedY=" + std::to_string(packedY)
                 + " packedHasSentinel=" + std::to_string((int)packedHasSentinel)
                 + " countMatch=" + std::to_string((int)countMatch));
        }
    }

    // ---- Block 39: D-042 R-4 — rotateObject writes preview.x. -------------
    // R-3 made Scene::pack memcpy preview→packed; R-4 makes rotateObject
    // dual-write state.x AND preview.x so preview stays consistent with
    // the rotated state across pack rebuild + pre-init rendering.
    // Mirrors R-3's translate dual-write pattern.
    //
    // Mechanic: addCube places vertex 0 at (-h, -h, -h) = (-0.1, -0.1, -0.1)
    // for tess=2, size=0.2 cube at origin. 90° Y rotation around the cube's
    // transformPosition pivot (also origin) maps (x, y, z) → (z, y, -x), so
    // vertex 0 → (-0.1, -0.1, 0.1). Assert preview.x[0..2] within 1e-4.
    //
    // Bug-probe: removing the R-4 preview write loop → preview.x unchanged
    // from addX time → assertion FAILs.
    {
        resetScene();
        sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), 2, 0.2f, 1.0f);
        sim.initialize();  // populates Scene::meshes so findById succeeds.

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        bool hasPreview = oneReq && reqs[0].preview.x.size() >= 3;

        Precision pre_x = hasPreview ? reqs[0].preview.x[0] : Precision(0);
        Precision pre_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);
        Precision pre_z = hasPreview ? reqs[0].preview.x[2] : Precision(0);

        ::Quat q90;
        q90.w = std::cos(0.78539816339f);  // π/4
        q90.x = 0.0f;
        q90.y = std::sin(0.78539816339f);
        q90.z = 0.0f;
        sim.rotateObject(0, q90);

        Precision post_x = hasPreview ? reqs[0].preview.x[0] : Precision(0);
        Precision post_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);
        Precision post_z = hasPreview ? reqs[0].preview.x[2] : Precision(0);

        // primitive::cube's +X face is emitted first, so vertex 0 lives at
        // the (+h, -h, -h) corner = (0.1, -0.1, -0.1) for size=0.2 at origin.
        // 90° Y rotation around origin maps (x, y, z) → (z, y, -x), so
        // vertex 0 → (-0.1, -0.1, -0.1).
        const Precision tol = Precision(1e-4f);
        bool xOk = std::fabs(post_x - Precision(-0.1f)) < tol;
        bool yOk = std::fabs(post_y - Precision(-0.1f)) < tol;
        bool zOk = std::fabs(post_z - Precision(-0.1f)) < tol;
        bool moved = std::fabs(post_x - pre_x) > tol
                     || std::fabs(post_z - pre_z) > tol;

        if (oneReq && hasPreview && xOk && yOk && zOk && moved) {
            pass("D-042 R-4 / rotateObject writes preview.x (90° Y rotation reflects in preview)");
        } else {
            fail("D-042 R-4 / rotateObject writes preview.x (90° Y rotation reflects in preview)",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " hasPreview=" + std::to_string((int)hasPreview)
                 + " pre_v0=(" + std::to_string(pre_x) + "," + std::to_string(pre_y) + "," + std::to_string(pre_z) + ")"
                 + " post_v0=(" + std::to_string(post_x) + "," + std::to_string(post_y) + "," + std::to_string(post_z) + ")"
                 + " xOk=" + std::to_string((int)xOk)
                 + " yOk=" + std::to_string((int)yOk)
                 + " zOk=" + std::to_string((int)zOk)
                 + " moved=" + std::to_string((int)moved));
        }
    }

    // ---- Block 40: D-042 R-5 — Simulator::update resyncs packed→preview. -
    // R-3 made Scene::pack memcpy preview→packed; R-4 made edits write
    // preview; R-5 closes the loop with a packed→preview resync at the
    // very end of Simulator::update. After the resync, preview reflects
    // post-substep simulated positions (cloth integrator output, rigid
    // Δpos, narrow-phase constraint resolution).
    //
    // Mechanic: addCloth at y=0.25 (TriangularCloth — gravity drops it).
    // Initialize, unpause, pump 30 frames. Without the resync, preview.x
    // stays frozen at addX-time positions while state.x falls under
    // gravity. With the resync, preview.x[1] tracks state.x[1] down.
    //
    // Bug-probe: removing the R-5 resync loop → preview.x[1] stays ≈ 0.25;
    // state.x[1] falls to ≈ 0.12; assertion FAILs on both `post_y < 0.20`
    // and `|state_y - post_y| < 1e-5`.
    {
        resetScene();
        sim.addCloth(/*particleNum1D=*/2, /*size1D=*/0.5f,
                     tinym::vec3(0.0f, 0.25f, 0.0f),
                     /*kstretch=*/1e3f, /*kshear=*/1e3f, /*kbend=*/1e3f,
                     /*thickness=*/0.01f, /*mass=*/0.1f);
        sim.initialize();

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        bool hasPreview = oneReq && reqs[0].preview.x.size() >= 3;

        Precision pre_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);

        sim.pause = false;
        for (int i = 0; i < 30; ++i) sim.update();

        Precision post_y = hasPreview ? reqs[0].preview.x[1] : Precision(0);
        bool meshExists = !Scene<Backend, Precision>::meshes.empty();
        Precision state_y = meshExists ? Scene<Backend, Precision>::meshes[0].state.x[1] : Precision(0);

        bool fellOk    = (post_y < Precision(0.20f));
        bool movedOk   = (post_y < pre_y - Precision(0.01f));
        bool resyncOk  = meshExists && (std::fabs(state_y - post_y) < Precision(1e-5f));

        if (oneReq && hasPreview && meshExists && fellOk && movedOk && resyncOk) {
            pass("D-042 R-5 / Simulator::update resyncs packed state.x into preview.x (cloth falls in preview after 30 frames)");
        } else {
            fail("D-042 R-5 / Simulator::update resyncs packed state.x into preview.x (cloth falls in preview after 30 frames)",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " hasPreview=" + std::to_string((int)hasPreview)
                 + " meshExists=" + std::to_string((int)meshExists)
                 + " pre_y=" + std::to_string(pre_y)
                 + " post_y=" + std::to_string(post_y)
                 + " state_y=" + std::to_string(state_y)
                 + " fellOk=" + std::to_string((int)fellOk)
                 + " movedOk=" + std::to_string((int)movedOk)
                 + " resyncOk=" + std::to_string((int)resyncOk));
        }
    }

    // ---- Block 41: D-042 R-6 — preview ≡ state.x round-trip invariant. ----
    // R-3 memcpys preview→packed at Scene::pack; R-4 keeps preview in sync
    // with state.x during edits (dual-write); R-5 memcpys state.x→preview
    // at update end. After R-3+R-4+R-5, per-mesh preview.x and packed
    // state.x are byte-equal at every observable boundary. Block 41
    // codifies that invariant so any future slice that breaks it FAILs
    // noisily. Future blocks can equivalently read either side — the BC
    // alias property the D-042 design promised.
    //
    // Bug-probe: corrupting the resync write (e.g., write state.x[0]+0.01f
    // into preview.x[0]) breaks byte-equality on frame 1 — memcmp != 0.
    {
        resetScene();
        sim.addCloth(/*particleNum1D=*/2, /*size1D=*/0.5f,
                     tinym::vec3(0.0f, 0.25f, 0.0f),
                     /*kstretch=*/1e3f, /*kshear=*/1e3f, /*kbend=*/1e3f,
                     /*thickness=*/0.01f, /*mass=*/0.1f);
        sim.initialize();

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        bool meshExists = !Scene<Backend, Precision>::meshes.empty();

        sim.pause = false;

        int frameMismatch = -1;
        int eqAllFrames = 0;
        if (oneReq && meshExists) {
            auto& m = Scene<Backend, Precision>::meshes[0];
            for (int f = 0; f < 5; ++f) {
                sim.update();
                const size_t bytes = (size_t)(m.state.x.size) * sizeof(Precision);
                if (reqs[0].preview.x.size() * sizeof(Precision) != bytes) {
                    frameMismatch = f;
                    break;
                }
                if (std::memcmp(m.state.x.ptr, reqs[0].preview.x.data(), bytes) != 0) {
                    frameMismatch = f;
                    break;
                }
                eqAllFrames++;
            }
        }

        if (oneReq && meshExists && frameMismatch < 0 && eqAllFrames == 5) {
            pass("D-042 R-6 / state.x = preview.x byte-equal after every Simulator::update (5-frame cloth round-trip)");
        } else {
            fail("D-042 R-6 / state.x = preview.x byte-equal after every Simulator::update (5-frame cloth round-trip)",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " meshExists=" + std::to_string((int)meshExists)
                 + " frameMismatch=" + std::to_string(frameMismatch)
                 + " eqAllFrames=" + std::to_string(eqAllFrames));
        }
    }

    // ---- Block 42: D-042 R-7 — preview.n recomputed in R-5 resync. -------
    // R-7 adds `req.preview.recomputeNormals()` to the R-5 resync so per-
    // vertex normals track simulated deformation. Pre-R-7 preview.n held
    // addX-time normals — visually stale post-sim.
    //
    // Mechanic (final): addCloth + initialize (preview.n correctly
    // populated by populatePreview → recomputeNormals at addX time). Then
    // CORRUPT preview.n[0] = 99.0f (a non-normalized sentinel; recomputed
    // normals are unit-length so |component| > 1 implies the field is
    // stale-or-corrupt). One sim.update() triggers the R-5 resync → R-7
    // recomputeNormals → preview.n[0] gets RESTORED to a valid component
    // in [-1, +1]. (The PLAN sketched a deformation-witness shape, but
    // free-falling cloth without anchor falls rigidly — no relative
    // deformation, identical normals before/after — so the slice pivoted
    // to a corrupt-and-restore sentinel that directly proves the
    // recompute fired. Deformation coverage is still implicit: any
    // future slice that breaks recomputeNormals' interaction with state.x
    // FAILs both Block 42 + Blocks 40/41's cascading state-x checks.)
    //
    // Bug-probe: removing the R-7 recomputeNormals() call → preview.n[0]
    // stays at 99.0f after sim.update → assertion FAILs.
    {
        resetScene();
        // Use the same 4-particle cloth as Block 40/41 (proven safe under
        // sim.update). A Float-tagged cube hung the test in earlier R-7
        // attempts — likely Metal command-queue interaction with the cube's
        // 50 substeps × broad/narrow phase. The recomputeNormals test is
        // shape-agnostic; cloth works.
        sim.addCloth(/*particleNum1D=*/2, /*size1D=*/0.5f,
                     tinym::vec3(0.0f, 0.25f, 0.0f),
                     /*kstretch=*/1e3f, /*kshear=*/1e3f, /*kbend=*/1e3f,
                     /*thickness=*/0.01f, /*mass=*/0.1f);
        sim.initialize();

        auto& reqs = Scene<Backend, Precision>::requestsGeneralMeshes;
        bool oneReq = (reqs.size() == 1);
        bool hasN   = oneReq && reqs[0].preview.n.size() >= 3;

        const Precision sentinel = Precision(99.0f);
        Precision corrupted_before = Precision(0);
        if (hasN) {
            reqs[0].preview.n[0] = sentinel;
            corrupted_before = reqs[0].preview.n[0];
        }

        sim.pause = false;
        sim.update();

        Precision restored = hasN ? reqs[0].preview.n[0] : Precision(0);

        // Recomputed normals are unit-length per axis, so |component| <= 1.
        // The sentinel 99.0 must have been overwritten by R-7's recompute.
        bool corruptedOk   = (corrupted_before == sentinel);
        bool restoredOk    = (std::fabs(restored) <= Precision(1.0f) + Precision(1e-3f));
        bool actuallyDiff  = (std::fabs(restored - sentinel) > Precision(1.0f));

        if (oneReq && hasN && corruptedOk && restoredOk && actuallyDiff) {
            pass("D-042 R-7 / preview.n recomputed in R-5 resync (corrupted normal restored after update)");
        } else {
            fail("D-042 R-7 / preview.n recomputed in R-5 resync (corrupted normal restored after update)",
                 std::string("oneReq=") + std::to_string((int)oneReq)
                 + " hasN=" + std::to_string((int)hasN)
                 + " corrupted_before=" + std::to_string(corrupted_before)
                 + " restored=" + std::to_string(restored)
                 + " corruptedOk=" + std::to_string((int)corruptedOk)
                 + " restoredOk=" + std::to_string((int)restoredOk)
                 + " actuallyDiff=" + std::to_string((int)actuallyDiff));
        }
    }

    // ---- Block COL-ORDER: collision response is creation-order agnostic. --
    // Two overlapping non-Float meshes (cloth+cloth) must BOTH be registered
    // as a collision query (point provider) so both receive a response.
    // Before the bidirectional-registration fix the old `checked` set deduped
    // each pair to a single direction (query == the lower / earlier mesh
    // index), so the later-created mesh was detected-but-never-responded —
    // collision response then depended on creation order. The witness here
    // is direction-symmetric: regardless of which cloth is added first, the
    // pair {0,1} must yield narrow collisions with objPair.query == 0 AND
    // objPair.query == 1 across the stepped frames.
    {
        auto bothMeshesQueried = [&](bool swapOrder) -> bool {
            resetScene();
            // Two 4x4 cloth grids in the XY plane, ~half a thickness apart
            // in Z so every point sits inside the other sheet's contact
            // band → guaranteed point-triangle pairs both ways.
            tinym::vec3 cA(0.0f, 0.0f, 0.000f);
            tinym::vec3 cB(0.0f, 0.0f, 0.005f);
            if (!swapOrder) {
                sim.addCloth(4, 0.5f, cA, 1e3, 1e3, 1e3, 0.01f, 0.1f);
                sim.addCloth(4, 0.5f, cB, 1e3, 1e3, 1e3, 0.01f, 0.1f);
            } else {
                sim.addCloth(4, 0.5f, cB, 1e3, 1e3, 1e3, 0.01f, 0.1f);
                sim.addCloth(4, 0.5f, cA, 1e3, 1e3, 1e3, 0.01f, 0.1f);
            }
            sim.initialize();
            // Zero gravity: keep the sheets overlapping deterministically so
            // contacts fire every frame instead of free-falling apart.
            Scene<Backend, Precision>::environment.gravity = tinym::vec3(0.f);
            Scene<Backend, Precision>::environment.wind    = tinym::vec3(0.f);
            bool seen0 = false, seen1 = false;
            auto& pc = Scene<Backend, Precision>::packedCollisionData;
            for (int f = 0; f < 6; ++f) {
                sim.update();
                for (Index i = 0; i < pc.numNarrowCollisions[0]; ++i) {
                    Index qy = pc.narrowCollisions[i].objPair.query;
                    if (qy == 0) seen0 = true;
                    if (qy == 1) seen1 = true;
                }
            }
            return seen0 && seen1;
        };
        if (bothMeshesQueried(false))
            pass("COL-ORDER A / both meshes responded (cloth0 then cloth1)");
        else
            fail("COL-ORDER A / both meshes responded (cloth0 then cloth1)",
                 "only one mesh registered as collision query");
        if (bothMeshesQueried(true))
            pass("COL-ORDER B / both meshes responded (cloth1 then cloth0)");
        else
            fail("COL-ORDER B / both meshes responded (cloth1 then cloth0)",
                 "only one mesh registered as collision query");
    }

    // ---- Block FG-SCALE: FastGridCloth rest lengths must track scale. -----
    // A grid plane turned into FastGridCloth then scaled must have ALL 6
    // directional rest lengths follow the scaled geometry. The non-uniform
    // case (3x X, 1x Y) is the one a single-scalar model could not express:
    // stretchRestX != stretchRestY and bendRestX != bendRestY.
    {
        const Precision S = 0.6f;
        const int       N = 4;                          // 4x4 grid
        const Precision base = S / Precision(N - 1);     // 0.2 grid spacing
        const Precision tol  = 1e-3f;
        // Expected per-direction rest for column spacing dx, row spacing dy.
        auto checkRest = [&](const char* name, Precision dx, Precision dy) {
            auto* m = Scene<Backend, Precision>::findById(0);
            if (!m) { fail(name, "mesh id 0 missing"); return; }
            auto* fp = std::get_if<FastGridClothBehaviorParams<Precision>>(
                           &m->behaviorParams);
            if (!fp) { fail(name, "behaviorParams not FastGridCloth"); return; }
            Precision wsX = dx,            wsY = dy;
            Precision wsh = std::sqrt(dx*dx + dy*dy);     // both diagonals
            Precision wbX = 2*dx,          wbY = 2*dy;
            auto near = [&](Precision a, Precision b){ return std::abs(a-b) < tol; };
            bool ok = near(fp->stretchRestX, wsX) && near(fp->stretchRestY, wsY)
                   && near(fp->shearRestA, wsh)   && near(fp->shearRestB, wsh)
                   && near(fp->bendRestX, wbX)    && near(fp->bendRestY, wbY);
            if (ok) pass(name);
            else fail(name,
                "sX=" + std::to_string(fp->stretchRestX) + "/" + std::to_string(wsX)
                + " sY=" + std::to_string(fp->stretchRestY) + "/" + std::to_string(wsY)
                + " shA=" + std::to_string(fp->shearRestA) + "/" + std::to_string(wsh)
                + " shB=" + std::to_string(fp->shearRestB) + "/" + std::to_string(wsh)
                + " bX=" + std::to_string(fp->bendRestX) + "/" + std::to_string(wbX)
                + " bY=" + std::to_string(fp->bendRestY) + "/" + std::to_string(wbY));
        };
        auto freshGrid = [&]() {
            resetScene();
            sim.addPlane(PlaneDirection::XYPlane, tinym::vec3(0.0f), N, S);
            sim.initialize();
        };

        // Order A: plane → 옷감(FastGridCloth) → uniform scale 3x.
        freshGrid();
        sim.changeBehavior(0, BehaviorType::FastGridCloth);
        sim.initialize();                       // dirty pack consumes the toggle
        checkRest("FG-SCALE A0 / rest == grid spacing before scale", base, base);
        sim.scaleObject(0, tinym::vec3(3.0f, 3.0f, 3.0f));
        checkRest("FG-SCALE A1 / uniform 3x after changeBehavior->scale",
                  base*3, base*3);
        sim.initialize();                       // re-pack must keep scaled rest
        checkRest("FG-SCALE A2 / scaled rest survives a re-pack",
                  base*3, base*3);

        // Order B: plane → scale(while Float) → 옷감(FastGridCloth).
        freshGrid();
        sim.scaleObject(0, tinym::vec3(3.0f, 3.0f, 3.0f));
        sim.changeBehavior(0, BehaviorType::FastGridCloth);
        sim.initialize();                       // dirty pack builds scaled grid
        checkRest("FG-SCALE B / uniform 3x after scale->changeBehavior",
                  base*3, base*3);

        // Order C: NON-UNIFORM scale (X 3x, Y 1x) — the 6-value witness.
        freshGrid();
        sim.changeBehavior(0, BehaviorType::FastGridCloth);
        sim.initialize();
        sim.scaleObject(0, tinym::vec3(3.0f, 1.0f, 1.0f));
        checkRest("FG-SCALE C1 / non-uniform 3x/1x (changeBehavior->scale)",
                  base*3, base*1);
        sim.initialize();
        checkRest("FG-SCALE C2 / non-uniform survives re-pack",
                  base*3, base*1);
    }

    // ---- Block KIN: BVH kinematic body — load, sim-time playback, scrub,
    //                  motion-file swap. Skips when the asset dir is absent
    //                  (e.g. a stripped checkout).
    {
        const std::string dir = bvhAssetDir();
        if (!std::filesystem::exists(dir + "/WalkLoopA.bvh")) {
            skip("KIN", "no WalkLoopA.bvh under " + dir);
        } else {
            resetScene();
            sim.pause = false;
            if (!sim.addKinematicBody(dir + "/WalkLoopA.bvh",
                                      tinym::vec3(0, 0, 0), 1.8f)) {
                fail("KIN-1 / addKinematicBody loads WalkLoopA.bvh", "load failed");
            } else {
                sim.initialize();
                auto* m = Scene<Backend, Precision>::findById(0);
                auto* kin = sim.kinematicOf(0);
                if (!m || !kin || !m->state.x.ptr) {
                    fail("KIN-1 / kinematic mesh realized after pack",
                         "mesh or initializer missing");
                } else {
                    // 1) Playback: verts move across frames and stay finite.
                    auto x0 = snapshot_array(m->state.x.ptr, m->state.x.size);
                    pumpFrames(sim, 2);
                    MetalGlobalContext::commitAndWait();
                    bool moved = false, finite = true;
                    for (size_t i = 0; i < x0.size(); ++i) {
                        if (m->state.x.ptr[i] != x0[i]) moved = true;
                        if (!std::isfinite((double)m->state.x.ptr[i])) finite = false;
                    }
                    if (moved && finite)
                        pass("KIN-1 / FK playback advances proxy verts (finite)");
                    else
                        fail("KIN-1 / FK playback advances proxy verts (finite)",
                             "moved=" + std::to_string((int)moved)
                             + " finite=" + std::to_string((int)finite));

                    // 2) playing=false freezes the body even while the sim runs.
                    kin->playing = false;
                    pumpFrames(sim, 1);
                    MetalGlobalContext::commitAndWait();
                    auto x1 = snapshot_array(m->state.x.ptr, m->state.x.size);
                    pumpFrames(sim, 2);
                    MetalGlobalContext::commitAndWait();
                    bool frozen = true;
                    for (size_t i = 0; i < x1.size(); ++i)
                        if (m->state.x.ptr[i] != x1[i]) { frozen = false; break; }
                    if (frozen) pass("KIN-2 / playing=false freezes pose");
                    else        fail("KIN-2 / playing=false freezes pose", "verts drifted");

                    // 3) Scrub re-poses immediately (paused sim included).
                    sim.pause = true;
                    sim.setKinematicTime(0, 0.5);
                    bool scrubbed = false;
                    for (size_t i = 0; i < x1.size(); ++i)
                        if (m->state.x.ptr[i] != x1[i]) { scrubbed = true; break; }
                    if (scrubbed) pass("KIN-3 / scrub re-poses while paused");
                    else          fail("KIN-3 / scrub re-poses while paused", "pose unchanged");
                    sim.pause = false;

                    // 4) Motion-file swap reloads + re-packs cleanly.
                    if (std::filesystem::exists(dir + "/jogCurve.bvh")) {
                        bool swapped = sim.setKinematicFile(0, dir + "/jogCurve.bvh");
                        sim.initialize();
                        auto* m2 = Scene<Backend, Precision>::findById(0);
                        auto* kin2 = sim.kinematicOf(0);
                        bool ok = swapped && m2 && kin2
                               && kin2->motion.valid()
                               && kin2->params.filePath == dir + "/jogCurve.bvh"
                               && m2->state.x.size
                                    == (Index)kin2->params.numPoints * 3;
                        pumpFrames(sim, 1);
                        MetalGlobalContext::commitAndWait();
                        if (ok) pass("KIN-4 / motion-file swap re-packs + steps");
                        else    fail("KIN-4 / motion-file swap re-packs + steps",
                                     "swapped=" + std::to_string((int)swapped));
                    } else {
                        skip("KIN-4", "jogCurve.bvh missing");
                    }

                    // 5) User translation survives reset (translateObject
                    //    must write back params.center — the re-pack in
                    //    reset() reseeds transformPosition from it).
                    {
                        sim.translateObject(0, tinym::vec3(2.0f, 0.0f, -1.0f));
                        sim.reset();
                        MetalGlobalContext::commitAndWait();
                        auto* m5 = Scene<Backend, Precision>::findById(0);
                        auto* kin5 = sim.kinematicOf(0);
                        bool posKept = m5
                            && std::fabs(m5->transformPosition.x - 2.0f) < 1e-5f
                            && std::fabs(m5->transformPosition.y - 0.0f) < 1e-5f
                            && std::fabs(m5->transformPosition.z + 1.0f) < 1e-5f;
                        bool timeReset = kin5 && kin5->localTime == 0.0;
                        if (posKept && timeReset)
                            pass("KIN-5 / translate survives reset + time rewinds");
                        else
                            fail("KIN-5 / translate survives reset + time rewinds",
                                 std::string("posKept=") + std::to_string((int)posKept)
                                 + " timeReset=" + std::to_string((int)timeReset));
                    }

                    // 6) Loop seam: positions snap (no backward root sweep),
                    //    rotations still interpolate. Mid-seam pose must
                    //    match the last frame's root position, not the
                    //    last/first average.
                    {
                        auto* kin6 = sim.kinematicOf(0);
                        const auto& mo = kin6->motion;
                        const float dur = mo.duration();
                        const float ft = mo.frameTime;
                        bvh::Pose lastP, midSeam, firstP;
                        mo.evaluate(dur - ft, true, lastP);          // frame N-1
                        mo.evaluate(dur - 0.5f * ft, true, midSeam); // seam a=0.5
                        mo.evaluate(0.0f, true, firstP);             // frame 0
                        const auto& rL = lastP.world[0].t;
                        const auto& rM = midSeam.world[0].t;
                        const auto& rF = firstP.world[0].t;
                        // Clip drift makes last/first root differ; mid-seam
                        // must sit on the LAST frame, not between them.
                        float drift = 0, toLast = 0;
                        for (int c = 0; c < 3; ++c) {
                            drift += std::fabs(rL[c] - rF[c]);
                            toLast += std::fabs(rM[c] - rL[c]);
                        }
                        bool meaningful = drift > 1e-3f;  // WalkLoopA: ~11 in Z
                        bool snapped = toLast < 1e-4f;
                        if (meaningful && snapped)
                            pass("KIN-6 / loop seam holds root position (no backward sweep)");
                        else
                            fail("KIN-6 / loop seam holds root position (no backward sweep)",
                                 "drift=" + std::to_string(drift)
                                 + " toLast=" + std::to_string(toLast));
                    }
                }
            }
        }
    }

    // ---- Block MG: motion graphs (Kovar 2002) — metric correctness, ----
    //                transition continuity, random-walk liveness, and the
    //                mode dispatch end-to-end. Skips without assets.
    {
        const std::string dir = bvhAssetDir();
        if (!std::filesystem::exists(dir + "/WalkLoopA.bvh") ||
            !std::filesystem::exists(dir + "/walkToJog.bvh") ||
            !std::filesystem::exists(dir + "/jogCurve.bvh")) {
            skip("MG", "BVH assets missing under " + dir);
        } else {
            std::string lerr;
            bvh::Motion mWalk = bvh::load(dir + "/WalkLoopA.bvh", &lerr);
            bvh::Motion mToJog = bvh::load(dir + "/walkToJog.bvh", &lerr);

            // 1) Streaming cost matrix == brute closed form == literal
            //    point-cloud SSD under the returned alignment (spot grid).
            {
                mograph::Skeleton sk = mograph::Skeleton::extract(mWalk);
                mograph::Clip ca, cb;
                mograph::sampleClip(mWalk, sk, mWalk.frameTime, "A", ca);
                mograph::sampleClip(mToJog, sk, mWalk.frameTime, "B", cb);
                mograph::FrameCloud fa, fb;
                fa.build(sk, ca);
                fb.build(sk, cb);
                const int k = 10;
                mograph::CostMatrix D;
                mograph::computeCostMatrix(fa, fb, k, sk.height, D);
                float worst = -1.0f;
                if (!D.empty()) {
                    worst = 0.0f;
                    for (int i = 0; i < D.ra; i += 5) {
                        for (int jj = 0; jj < D.rb; jj += 7) {
                            auto r = mograph::optimalAlign(fa, i, fb,
                                                           jj + k - 1, k,
                                                           sk.height);
                            const float lit = mograph::costDirect(
                                fa, i, fb, jj + k - 1, k, sk.height, r.xf);
                            worst = std::max(worst,
                                             std::fabs(D.at(i, jj) - r.cost));
                            worst = std::max(worst, std::fabs(lit - r.cost));
                        }
                    }
                }
                if (worst >= 0.0f && worst < 1e-3f)
                    pass("MG-1 / cost matrix == closed form == literal SSD");
                else
                    fail("MG-1 / cost matrix == closed form == literal SSD",
                         "worst=" + std::to_string(worst));

                // 2) A known ground-plane transform is recovered (cost → 0).
                mograph::XformXZ T{0.8f, 3.0f, -2.0f};
                mograph::Clip ct = ca;
                for (auto& f : ct.frames) T.applyPose(f);
                mograph::FrameCloud ftc;
                ftc.build(sk, ct);
                auto rec =
                    mograph::optimalAlign(fa, 3, ftc, 3 + k - 1, k, sk.height);
                if (rec.cost < 1e-3f)
                    pass("MG-2 / optimal alignment recovers a known transform");
                else
                    fail("MG-2 / optimal alignment recovers a known transform",
                         "residual cost=" + std::to_string(rec.cost));
            }

            // 3..6) End-to-end through the Simulator API.
            resetScene();
            sim.pause = false;
            if (!sim.addKinematicBody(dir + "/WalkLoopA.bvh",
                                      tinym::vec3(0, 0, 0), 1.8f)) {
                fail("MG-3 / kinematic body for graph tests", "load failed");
            } else {
                sim.initialize();
                auto* m = Scene<Backend, Precision>::findById(0);
                auto* kin = sim.kinematicOf(0);
                auto maxRootStep =
                    [](const std::vector<mograph::LocalPose>& fr) {
                        float mx = 0.0f;
                        for (size_t f = 1; f < fr.size(); ++f) {
                            float d = 0.0f;
                            for (int c = 0; c < 3; ++c) {
                                const float e =
                                    fr[f].rootPos[c] - fr[f - 1].rootPos[c];
                                d += e * e;
                            }
                            mx = std::max(mx, std::sqrt(d));
                        }
                        return mx;
                    };

                // 3) Transition build + composite continuity (no teleport
                //    bigger than the sources' own frame-to-frame motion).
                bool built = sim.buildKinematicTransition(0, "WalkLoopA.bvh",
                                                          "walkToJog.bvh");
                if (!built || !m || !kin || !kin->graphActive() ||
                    kin->motionMode != 2) {
                    fail("MG-3 / transition build A→B engages graph mode",
                         "status=" + (kin ? kin->graphStatus : "no kin"));
                } else {
                    const auto& ses = kin->graphSession;
                    const float src =
                        std::max(maxRootStep(ses.clips[0].frames),
                                 maxRootStep(ses.clips[1].frames));
                    const float comp = maxRootStep(ses.track);
                    if (comp < 2.0f * src + 1e-4f)
                        pass("MG-3 / transition composite is continuous");
                    else
                        fail("MG-3 / transition composite is continuous",
                             "step " + std::to_string(comp) + " vs src " +
                                 std::to_string(src));
                }

                // 4) Transition mode animates the proxy (finite) and scrub
                //    re-poses while paused.
                if (built && m && kin) {
                    auto x0 = snapshot_array(m->state.x.ptr, m->state.x.size);
                    pumpFrames(sim, 2);
                    MetalGlobalContext::commitAndWait();
                    bool moved = false, finite = true;
                    for (size_t i = 0; i < x0.size(); ++i) {
                        if (m->state.x.ptr[i] != x0[i]) moved = true;
                        if (!std::isfinite((double)m->state.x.ptr[i]))
                            finite = false;
                    }
                    sim.pause = true;
                    auto x1 = snapshot_array(m->state.x.ptr, m->state.x.size);
                    sim.setKinematicTime(0, kin->graphSession.duration() * 0.5);
                    bool scrubbed = false;
                    for (size_t i = 0; i < x1.size(); ++i)
                        if (m->state.x.ptr[i] != x1[i]) { scrubbed = true; break; }
                    sim.pause = false;
                    if (moved && finite && scrubbed)
                        pass("MG-4 / transition playback + paused scrub");
                    else
                        fail("MG-4 / transition playback + paused scrub",
                             "moved=" + std::to_string((int)moved) +
                                 " finite=" + std::to_string((int)finite) +
                                 " scrubbed=" + std::to_string((int)scrubbed));
                }

                // 5) Random walk: build over three clips, animate, and bake
                //    far ahead without losing finiteness.
                if (m && kin) {
                    kin->graphSelFiles = {"WalkLoopA.bvh", "walkToJog.bvh",
                                          "jogCurve.bvh"};
                    built = sim.buildKinematicWalk(0);
                    bool ok = built && kin->graphActive() &&
                              kin->motionMode == 1 &&
                              kin->graphSession.graph.stats.transitionsKept > 0;
                    if (ok) {
                        auto x0 = snapshot_array(m->state.x.ptr,
                                                 m->state.x.size);
                        pumpFrames(sim, 2);
                        MetalGlobalContext::commitAndWait();
                        bool moved = false, finite = true;
                        for (size_t i = 0; i < x0.size(); ++i) {
                            if (m->state.x.ptr[i] != x0[i]) moved = true;
                            if (!std::isfinite((double)m->state.x.ptr[i]))
                                finite = false;
                        }
                        sim.setKinematicTime(0, 60.0);  // bake ~1800 frames
                        for (size_t i = 0; i < m->state.x.size && finite; ++i)
                            if (!std::isfinite((double)m->state.x.ptr[i]))
                                finite = false;
                        const auto& wb = kin->graphSession.walker;
                        const bool baked =
                            wb.bakedStart + (long long)wb.baked.size() >= 1800;
                        ok = moved && finite && baked && !wb.stuck;
                        if (!ok)
                            fail("MG-5 / random walk builds + streams",
                                 "moved=" + std::to_string((int)moved) +
                                     " finite=" + std::to_string((int)finite) +
                                     " baked=" + std::to_string((int)baked) +
                                     " stuck=" + std::to_string((int)wb.stuck));
                        else
                            pass("MG-5 / random walk builds + streams");
                    } else {
                        fail("MG-5 / random walk builds + streams",
                             "status=" + kin->graphStatus);
                    }
                }

                // 6) Incompatible skeleton is rejected and the body falls
                //    back to (still working) single-clip playback.
                if (m && kin &&
                    std::filesystem::exists(
                        dir + "/j_Uber_054_SMK_CHANG1_01.bvh")) {
                    built = sim.buildKinematicTransition(
                        0, "WalkLoopA.bvh", "j_Uber_054_SMK_CHANG1_01.bvh");
                    const bool rejected = !built && !kin->graphActive();
                    sim.setKinematicMode(0, 0);
                    auto x0 = snapshot_array(m->state.x.ptr, m->state.x.size);
                    pumpFrames(sim, 2);
                    MetalGlobalContext::commitAndWait();
                    bool moved = false, finite = true;
                    for (size_t i = 0; i < x0.size(); ++i) {
                        if (m->state.x.ptr[i] != x0[i]) moved = true;
                        if (!std::isfinite((double)m->state.x.ptr[i]))
                            finite = false;
                    }
                    if (rejected && moved && finite)
                        pass("MG-6 / incompatible skeleton rejected, single clip intact");
                    else
                        fail("MG-6 / incompatible skeleton rejected, single clip intact",
                             "rejected=" + std::to_string((int)rejected) +
                                 " moved=" + std::to_string((int)moved) +
                                 " finite=" + std::to_string((int)finite));
                } else {
                    skip("MG-6", "j_Uber asset missing");
                }

                // 7) Oriented point cloud: a leaf joint's local rotation moves
                //    no position, so the origins-only metric is blind to it,
                //    but joint-axis markers register it. The streaming ==
                //    closed-form == literal identities still hold with markers.
                {
                    mograph::Skeleton sk = mograph::Skeleton::extract(mWalk);
                    mograph::Clip base;
                    mograph::sampleClip(mWalk, sk, mWalk.frameTime, "base", base);
                    std::vector<int> childCount(sk.joints.size(), 0);
                    for (size_t j = 0; j < sk.joints.size(); ++j)
                        if (sk.joints[j].parent >= 0)
                            childCount[sk.joints[j].parent]++;
                    int leaf = -1;
                    for (size_t j = 0; j < sk.joints.size(); ++j)
                        if (childCount[j] == 0) { leaf = int(j); break; }
                    mograph::Clip rot = base;
                    const mograph::Quatf spin =
                        mograph::Quatf::axisAngle(1, 0, 0, 1.0f);  // 1 rad
                    if (leaf >= 0)
                        for (auto& fr : rot.frames)
                            fr.rot[leaf] = (fr.rot[leaf] * spin).normalized();
                    const int k = 10;
                    const float ms = 0.1f * sk.height;
                    mograph::FrameCloud o0a, o0b, m1a, m1b;
                    o0a.build(sk, base, 0.0f);
                    o0b.build(sk, rot, 0.0f);
                    m1a.build(sk, base, ms);
                    m1b.build(sk, rot, ms);
                    mograph::CostMatrix Do, Dm;
                    mograph::computeCostMatrix(o0a, o0b, k, sk.height, Do);
                    mograph::computeCostMatrix(m1a, m1b, k, sk.height, Dm);
                    const float co = Do.empty() ? -1.0f : Do.at(0, 0);
                    const float cm = Dm.empty() ? -1.0f : Dm.at(0, 0);
                    float resid = -1.0f;
                    if (!Dm.empty()) {
                        auto r = mograph::optimalAlign(m1a, 0, m1b, k - 1, k,
                                                       sk.height);
                        const float lit = mograph::costDirect(m1a, 0, m1b, k - 1,
                                                              k, sk.height, r.xf);
                        resid = std::max(std::fabs(Dm.at(0, 0) - r.cost),
                                         std::fabs(lit - r.cost));
                    }
                    const bool leafBlind = co >= 0.0f && co < 1e-4f;
                    const bool markerSees = cm > 1e-3f;
                    const bool identsHold = resid >= 0.0f && resid < 1e-3f;
                    if (leaf >= 0 && leafBlind && markerSees && identsHold)
                        pass("MG-7 / joint-axis markers capture orientation the "
                             "origins miss");
                    else
                        fail("MG-7 / joint-axis markers capture orientation the "
                             "origins miss",
                             "leaf=" + std::to_string(leaf) + " co=" +
                                 std::to_string(co) + " cm=" +
                                 std::to_string(cm) + " resid=" +
                                 std::to_string(resid));
                }

                // 8) DTW timewarp recovers a known 2x time stretch, and the
                //    blend build bakes a continuous track through the API.
                {
                    mograph::Skeleton sk = mograph::Skeleton::extract(mWalk);
                    mograph::Clip a, b;
                    mograph::sampleClip(mWalk, sk, mWalk.frameTime, "A", a);
                    // Half dt over the same duration → ~2x the frames of A.
                    mograph::sampleClip(mWalk, sk, mWalk.frameTime * 0.5f, "B2x",
                                        b);
                    const int k = 10;
                    const float ms = 0.1f * sk.height;
                    mograph::FrameCloud fa, fb;
                    fa.build(sk, a, ms);
                    fb.build(sk, b, ms);
                    mograph::CostMatrix D;
                    mograph::computeCostMatrix(fa, fb, k, sk.height, D);
                    mograph::WarpPath wp;
                    const bool got = mograph::dtwPath(D, 2, wp);
                    bool mono = got;
                    for (size_t c = 1; c < wp.cells.size(); ++c)
                        if (wp.cells[c].first < wp.cells[c - 1].first ||
                            wp.cells[c].second < wp.cells[c - 1].second)
                            mono = false;
                    float slope = -1.0f;
                    if (got && wp.cells.size() > 1) {
                        const int da = wp.cells.back().first - wp.cells.front().first;
                        const int db = wp.cells.back().second - wp.cells.front().second;
                        if (da > 0) slope = float(db) / float(da);
                    }
                    const bool slopeOk = slope > 1.4f && slope < 2.6f;
                    if (got && mono && slopeOk)
                        pass("MG-8 / DTW recovers a 2x time-stretch warp");
                    else
                        fail("MG-8 / DTW recovers a 2x time-stretch warp",
                             "got=" + std::to_string((int)got) + " mono=" +
                                 std::to_string((int)mono) + " slope=" +
                                 std::to_string(slope));

                    const bool built = sim.buildKinematicBlend(
                        0, "WalkLoopA.bvh", "walkToJog.bvh");
                    auto* kinb = sim.kinematicOf(0);
                    if (built && kinb && kinb->graphActive() &&
                        kinb->motionMode == 3) {
                        auto maxStep =
                            [](const std::vector<mograph::LocalPose>& fr) {
                                float mx = 0.0f;
                                for (size_t f = 1; f < fr.size(); ++f) {
                                    float d = 0.0f;
                                    for (int c = 0; c < 3; ++c) {
                                        const float e = fr[f].rootPos[c] -
                                                        fr[f - 1].rootPos[c];
                                        d += e * e;
                                    }
                                    mx = std::max(mx, std::sqrt(d));
                                }
                                return mx;
                            };
                        const auto& ses = kinb->graphSession;
                        const float src =
                            std::max(maxStep(ses.clips[0].frames),
                                     maxStep(ses.clips[1].frames));
                        const float comp = maxStep(ses.track);
                        if (comp < 2.0f * src + 1e-4f)
                            pass("MG-8b / DTW blend track is continuous");
                        else
                            fail("MG-8b / DTW blend track is continuous",
                                 "step " + std::to_string(comp) + " vs src " +
                                     std::to_string(src));
                    } else {
                        fail("MG-8b / DTW blend track is continuous",
                             "build/mode failed: " +
                                 (kinb ? kinb->graphStatus : "no kin"));
                    }
                    sim.setKinematicMode(0, 0);  // restore for later blocks
                }
            }
        }
    }

    // ---- Block MBS: interactive motion blend space — preset skeleton ----
    //                 compatibility, inverse-distance weighting, and the
    //                 N-way pose blend (corner ≈ pure clip, center = mix).
    {
        const std::string dir = bvhAssetDir();
        const std::vector<std::string> presets{
            "WalkLoopA.bvh", "jogCurve.bvh", "SneakLoopA.bvh", "StrutLoopA.bvh"};
        bool haveAll = true;
        for (const auto& f : presets)
            if (!std::filesystem::exists(dir + "/" + f)) haveAll = false;
        if (!haveAll) {
            skip("MBS", "blend-space preset assets missing under " + dir);
        } else {
            resetScene();
            sim.pause = false;
            if (!sim.addKinematicBody(dir + "/WalkLoopA.bvh",
                                      tinym::vec3(0, 0, 0), 1.8f)) {
                fail("MBS-1 / kinematic body for blend space", "load failed");
            } else {
                sim.initialize();
                auto* kin = sim.kinematicOf(0);
                const bool built = sim.buildKinematicBlendSpace(0);
                // MBS-1: all four presets are skeleton-compatible + retained.
                if (built && kin && kin->graphActive() && kin->motionMode == 4 &&
                    kin->graphSession.clips.size() == presets.size())
                    pass("MBS-1 / 4 presets build a blend space (skeletons compatible)");
                else
                    fail("MBS-1 / 4 presets build a blend space (skeletons compatible)",
                         "clips=" +
                             std::to_string(kin ? kin->graphSession.clips.size() : 0) +
                             " status=" + (kin ? kin->graphStatus : "no kin"));

                if (built && kin) {
                    auto& ses = kin->graphSession;
                    // Locate the clip placed at the Run corner (0,1).
                    int runIdx = -1;
                    for (size_t i = 0; i < ses.clipCoords.size(); ++i)
                        if (std::fabs(ses.clipCoords[i][0]) < 1e-3f &&
                            std::fabs(ses.clipCoords[i][1] - 1.0f) < 1e-3f)
                            runIdx = int(i);
                    // MBS-2: cursor on a sample point ⇒ that clip's weight → 1,
                    // and the weights are a normalized partition (Σ=1).
                    std::vector<float> w;
                    ses.blendWeights({0.0f, 1.0f}, w);
                    float sum = 0.0f;
                    for (float x : w) sum += x;
                    const bool peak = runIdx >= 0 && w[size_t(runIdx)] > 0.9f;
                    if (peak && std::fabs(sum - 1.0f) < 1e-4f)
                        pass("MBS-2 / cursor on a sample snaps to that clip (w>0.9, Σw=1)");
                    else
                        fail("MBS-2 / cursor on a sample snaps to that clip (w>0.9, Σw=1)",
                             "wRun=" + std::to_string(runIdx >= 0 ? w[size_t(runIdx)] : -1.0f) +
                                 " sum=" + std::to_string(sum));

                    // MBS-3: the diamond center is a near-uniform 4-way mix.
                    std::vector<float> wc;
                    ses.blendWeights({0.0f, 0.0f}, wc);
                    float spread = 0.0f;
                    for (float x : wc) spread = std::max(spread, std::fabs(x - 0.25f));
                    if (spread < 0.05f)
                        pass("MBS-3 / blend-space center is near-uniform (4 clips ≈ 0.25)");
                    else
                        fail("MBS-3 / blend-space center is near-uniform (4 clips ≈ 0.25)",
                             "max|w-0.25|=" + std::to_string(spread));

                    // MBS-4: N-way blend — at a pure corner the live pose
                    // matches that clip alone; at center it differs (real mix).
                    const int ri = runIdx >= 0 ? runIdx : 0;
                    ses.cursor = {0.0f, 1.0f};  // pure Run
                    bvh::Pose pRun;
                    ses.samplePose(0.0, pRun);
                    // Reference = that clip ALONE at phase 0. Under phase
                    // registration its phase-0 is the gait-aligned frame (a warp
                    // LUT lookup), not raw frame 0, so sample through the LUT.
                    bvh::Pose pRunRef;
                    mograph::LocalPose runRef0;
                    const std::vector<float> emptyLut;
                    const std::vector<float>& runLut =
                        (ses.registerPhase && size_t(ri) < ses.clipPhaseLUT.size())
                            ? ses.clipPhaseLUT[size_t(ri)]
                            : emptyLut;
                    mograph::sampleClipPhaseLUT(ses.clips[size_t(ri)], runLut, 0.0f,
                                                runRef0, true);
                    mograph::fk(ses.skel, runRef0, pRunRef);
                    float dCorner = 0.0f;
                    for (size_t j = 0; j < pRun.world.size(); ++j)
                        for (int c = 0; c < 3; ++c)
                            dCorner = std::max(dCorner,
                                               std::fabs(pRun.world[j].t[c] -
                                                         pRunRef.world[j].t[c]));
                    ses.cursor = {0.0f, 0.0f};  // center mix
                    bvh::Pose pMix;
                    ses.samplePose(0.0, pMix);
                    float dMix = 0.0f;
                    for (size_t j = 0; j < pMix.world.size(); ++j)
                        for (int c = 0; c < 3; ++c)
                            dMix = std::max(dMix,
                                            std::fabs(pMix.world[j].t[c] -
                                                      pRunRef.world[j].t[c]));
                    if (dCorner < 0.05f * ses.skel.height && dMix > dCorner)
                        pass("MBS-4 / N-way blend: corner≈pure clip, center is a real mix");
                    else
                        fail("MBS-4 / N-way blend: corner≈pure clip, center is a real mix",
                             "dCorner=" + std::to_string(dCorner) + " dMix=" +
                                 std::to_string(dMix) + " h=" +
                                 std::to_string(ses.skel.height));

                    // MBS-5: blend-result preview reflects the cursor — the
                    // ghost geometry is finite, body-sized, and differs between
                    // two cursor positions (the live real-time-reflection claim,
                    // GL aside).
                    std::vector<float> gA, gB;
                    ses.cursor = {0.0f, 1.0f};  // Run corner
                    kin->writeBlendGhost(0.25, true, tinym::vec3(1, 1, 1), Quat{},
                                         tinym::vec3(0, 0, 0), gA);
                    ses.cursor = {0.0f, -1.0f};  // Walk corner
                    kin->writeBlendGhost(0.25, true, tinym::vec3(1, 1, 1), Quat{},
                                         tinym::vec3(0, 0, 0), gB);
                    bool finite = !gA.empty() &&
                                  gA.size() == size_t(kin->proxy.numVerts) * 3 &&
                                  gA.size() == gB.size();
                    for (float v : gA) finite = finite && std::isfinite(v);
                    for (float v : gB) finite = finite && std::isfinite(v);
                    float gd = 0.0f;
                    for (size_t i = 0; finite && i < gA.size(); ++i)
                        gd = std::max(gd, std::fabs(gA[i] - gB[i]));
                    if (finite && gd > 1e-4f)
                        pass("MBS-5 / blend preview ghost reflects cursor (Run≠Walk, finite)");
                    else
                        fail("MBS-5 / blend preview ghost reflects cursor (Run≠Walk, finite)",
                             "finite=" + std::to_string(int(finite)) +
                                 " maxDiff=" + std::to_string(gd));
                    ses.cursor = {0.0f, 0.0f};

                    // MBS-6: the "opposition" metric — clip features separate a
                    // still pose from active locomotion (vigour + translation).
                    std::string e1, e2;
                    bvh::Motion mStand = bvh::load(dir + "/standStill.bvh", &e1);
                    bvh::Motion mWalk2 = bvh::load(dir + "/WalkLoopA.bvh", &e2);
                    if (mStand.valid() && mWalk2.valid()) {
                        mograph::Skeleton skS = mograph::Skeleton::extract(mStand);
                        mograph::Skeleton skW = mograph::Skeleton::extract(mWalk2);
                        mograph::Clip cS, cW;
                        mograph::sampleClip(mStand, skS, mStand.frameTime, "stand", cS);
                        mograph::sampleClip(mWalk2, skW, mWalk2.frameTime, "walk", cW);
                        mograph::ClipFeatures fS = mograph::clipFeatures(skS, cS);
                        mograph::ClipFeatures fW = mograph::clipFeatures(skW, cW);
                        if (fW.energy > fS.energy && fW.rootSpeed >= fS.rootSpeed)
                            pass("MBS-6 / clip features separate still from active locomotion");
                        else
                            fail("MBS-6 / clip features separate still from active locomotion",
                                 "standE=" + std::to_string(fS.energy) + " walkE=" +
                                     std::to_string(fW.energy) + " standSpd=" +
                                     std::to_string(fS.rootSpeed) + " walkSpd=" +
                                     std::to_string(fW.rootSpeed));
                    } else {
                        skip("MBS-6", "standStill/WalkLoopA missing");
                    }

                    // MBS-7: every curated preset builds with all 4 clips
                    // retained (verifies the bundled file names + compatibility).
                    bool allOk = true;
                    std::string bad;
                    for (size_t pi = 0; pi < blendPresets().size(); ++pi) {
                        auto* k = sim.kinematicOf(0);
                        if (!k) { allOk = false; bad += "no-kin "; break; }
                        k->blendSpaceFiles.assign(blendPresets()[pi].files.begin(),
                                                  blendPresets()[pi].files.end());
                        const bool b = sim.buildKinematicBlendSpace(0);
                        auto* k2 = sim.kinematicOf(0);
                        const size_t nc = k2 ? k2->graphSession.clips.size() : 0;
                        if (!b || nc != 4) {
                            allOk = false;
                            bad += std::string(blendPresets()[pi].name) + "(" +
                                   std::to_string(nc) + ") ";
                        }
                    }
                    if (allOk)
                        pass("MBS-7 / all curated presets build with 4 compatible clips");
                    else
                        fail("MBS-7 / all curated presets build with 4 compatible clips",
                             "bad: " + bad);

                    // MBS-8: the one-shot preview (clamp) removes the end-of-
                    // cycle seam. loop wraps phase 1→0 (a jump back toward the
                    // start for non-looping clips); clamp holds the last frame,
                    // so its end discontinuity is never worse than loop's.
                    if (auto* k = sim.kinematicOf(0)) {
                        if (k->graphActive() && k->motionMode == 4) {
                            auto& s = k->graphSession;
                            s.cursor = {0.0f, 1.0f};
                            auto endJump = [&](bool loop) {
                                bvh::Pose pa, pb;
                                s.sampleBlendPhase(0.96f, loop, pa);
                                s.sampleBlendPhase(1.0f, loop, pb);
                                float d = 0.0f;
                                for (size_t j = 0; j < pa.world.size(); ++j)
                                    for (int c = 0; c < 3; ++c)
                                        d = std::max(d, std::fabs(pa.world[j].t[c] -
                                                                  pb.world[j].t[c]));
                                return d;
                            };
                            const float jLoop = endJump(true);
                            const float jClamp = endJump(false);
                            if (jClamp <= jLoop + 1e-4f)
                                pass("MBS-8 / one-shot clamp end-jump ≤ loop wrap (seam removed)");
                            else
                                fail("MBS-8 / one-shot clamp end-jump ≤ loop wrap (seam removed)",
                                     "jLoop=" + std::to_string(jLoop) + " jClamp=" +
                                         std::to_string(jClamp));
                            s.cursor = {0.0f, 0.0f};

                            // MBS-9: the blended pose is CONTINUOUS in the cursor
                            // at large t — no pop. A step discontinuity (fmod-
                            // modulus jump from a weight-dependent cycle, or the
                            // old argmax reference switch in blendPoseN) would
                            // fire on ANY nonzero nudge → dSmall ≈ dBig. A
                            // continuous blend scales with the nudge → dBig ≫
                            // dSmall. Sampled on the most disparate preset so the
                            // gradient is steep but must still be proportional.
                            const double tBig = 50.0;
                            auto poseAt = [&](float cx) {
                                s.cursor = {cx, 0.0f};
                                bvh::Pose p;
                                s.samplePose(tBig, p);
                                return p;
                            };
                            auto poseDiff = [](const bvh::Pose& a, const bvh::Pose& b) {
                                float d = 0.0f;
                                for (size_t j = 0; j < a.world.size() &&
                                                   j < b.world.size(); ++j)
                                    for (int c = 0; c < 3; ++c)
                                        d = std::max(d, std::fabs(a.world[j].t[c] -
                                                                  b.world[j].t[c]));
                                return d;
                            };
                            const bvh::Pose p0 = poseAt(0.0f);
                            const bvh::Pose pSmall = poseAt(0.01f);
                            const bvh::Pose pBig = poseAt(0.10f);
                            const float dSmall = poseDiff(p0, pSmall);
                            const float dBig = poseDiff(p0, pBig);
                            // 10× the nudge must move the pose markedly more than
                            // 1× — i.e. no flat step. (A pop gives dBig ≈ dSmall.)
                            if (dBig >= 3.0f * dSmall - 1e-3f)
                                pass("MBS-9 / blended pose continuous in cursor at large t (no pop)");
                            else
                                fail("MBS-9 / blended pose continuous in cursor at large t (no pop)",
                                     "dSmall=" + std::to_string(dSmall) + " dBig=" +
                                         std::to_string(dBig) + " (step-like)");

                            // MBS-10: the blended root TRAVELS forward (velocity-
                            // integrated) yet never teleports — net travel grows
                            // over time while every consecutive-frame root step
                            // stays small (raw jogCurve would teleport ~30 at the
                            // seam). Driven via samplePose at 60 fps over 8 s.
                            s.cursor = {0.0f, 0.0f};
                            s.resetBlendTravel();
                            float maxStep = 0.0f, pathLen = 0.0f;
                            bvh::Pose prev, cur;
                            const double fdt = 1.0 / 60.0;
                            for (int q = 0; q <= 480; ++q) {
                                s.samplePose(q * fdt, cur);
                                if (cur.world.empty()) continue;
                                if (q > 0 && !prev.world.empty()) {
                                    const float dx = cur.world[0].t[0] - prev.world[0].t[0];
                                    const float dz = cur.world[0].t[2] - prev.world[0].t[2];
                                    const float step = std::sqrt(dx * dx + dz * dz);
                                    pathLen += step;
                                    maxStep = std::max(maxStep, step);
                                }
                                prev = cur;
                            }
                            // moves a meaningful distance (path, robust to curving)
                            // yet no single-frame teleport.
                            if (pathLen > 0.5f * s.skel.height &&
                                maxStep < 0.05f * s.skel.height)
                                pass("MBS-10 / blend-space root travels (curving) without seam teleport");
                            else
                                fail("MBS-10 / blend-space root travels (curving) without seam teleport",
                                     "pathLen=" + std::to_string(pathLen) +
                                         " maxStep=" + std::to_string(maxStep) +
                                         " h=" + std::to_string(s.skel.height));
                            s.cursor = {0.0f, 0.0f};
                            s.resetBlendTravel();
                        }
                    }

                    // MBS-11: convex weights keep the blend inside the sample
                    // hull even when extrapolating past a corner — all weights
                    // ≥0 and Σ=1 (no overshoot). P0.
                    {
                        const bool savedCW = ses.convexWeights;
                        ses.convexWeights = true;
                        std::vector<float> wEx;
                        ses.blendWeights({0.0f, 2.0f}, wEx);  // beyond the Run corner
                        float mn = 1e9f, sm = 0.0f;
                        for (float x : wEx) { mn = std::min(mn, x); sm += x; }
                        ses.convexWeights = savedCW;
                        if (mn >= -1e-6f && std::fabs(sm - 1.0f) < 1e-4f)
                            pass("MBS-11 / convex weights stay ≥0 and Σ=1 under extrapolation");
                        else
                            fail("MBS-11 / convex weights stay ≥0 and Σ=1 under extrapolation",
                                 "min=" + std::to_string(mn) + " sum=" + std::to_string(sm));
                    }

                    // MBS-12: phase registration (P1) built a per-clip cyclic
                    // warp LUT — one entry per clip, correct length, frames in
                    // range, at least one populated; plus a pure check of the
                    // path→LUT resampler on a synthetic half-rate diagonal.
                    {
                        bool ok = ses.clipPhaseLUT.size() == ses.clips.size();
                        int nonEmpty = 0;
                        for (size_t i = 0; ok && i < ses.clipPhaseLUT.size(); ++i) {
                            const auto& lut = ses.clipPhaseLUT[i];
                            if (lut.empty()) continue;  // linear fallback allowed
                            ++nonEmpty;
                            const int nf = int(ses.clips[i].frames.size());
                            if (int(lut.size()) != mograph::Session::kPhaseLUTRes) { ok = false; break; }
                            for (float v : lut)
                                if (v < 0.0f || v >= float(nf)) { ok = false; break; }
                        }
                        if (nonEmpty == 0) ok = false;  // registration must run
                        mograph::WarpPath synth;
                        for (int a = 0; a < 16; ++a) synth.cells.push_back({a, a / 2});
                        auto slut = mograph::Session::buildPhaseLUTFromPath(synth, 20, 10, 4, 64);
                        bool sok = slut.size() == 64;
                        for (float v : slut) if (v < 0.0f || v >= 10.0f) sok = false;
                        if (ok && sok)
                            pass("MBS-12 / phase-registration LUTs built, in-range; resampler sane");
                        else
                            fail("MBS-12 / phase-registration LUTs built, in-range; resampler sane",
                                 "lutOk=" + std::to_string(int(ok)) + " nonEmpty=" +
                                     std::to_string(nonEmpty) + " synthOk=" + std::to_string(int(sok)));
                    }
                }

                // ── Verbs & Adverbs two-motion keytime blend (motion_verb.hpp) ─
                // Fully synthetic (no asset / no Metal): a 6-joint skeleton with
                // named feet + two stepped foot-bob clips of DIFFERENT length but
                // the SAME gait structure, plus a constant chest lean that
                // differs between them so the adverb blend is observable.
                {
                    mograph::Skeleton sk;
                    sk.height = 1.0f;
                    auto addJ = [&](const char* nm, int par, float ox, float oy,
                                    float oz) {
                        mograph::Skeleton::J j;
                        j.name = nm;
                        j.parent = par;
                        j.offset = {ox, oy, oz};
                        j.isEndSite = false;
                        sk.joints.push_back(j);
                    };
                    addJ("Center", -1, 0, 0, 0);       // 0 root
                    addJ("LeftHip", 0, -0.2f, 0, 0);   // 1
                    addJ("LeftToes", 1, 0, -1, 0);     // 2 foot L
                    addJ("RightHip", 0, 0.2f, 0, 0);   // 3
                    addJ("RightToes", 3, 0, -1, 0);    // 4 foot R
                    addJ("Chest", 0, 0, 0.5f, 0);      // 5 style channel
                    // Stepped gait: left planted φ<0.6, right swing φ∈[0.1,0.5).
                    // Pitching a hip about +X lifts its toe (Y: -1 planted → 0 up).
                    auto authorBob = [](int N, float leanZ) {
                        mograph::Clip c;
                        c.dt = 1.0f / 30.0f;
                        c.frames.resize(size_t(N));
                        const float H = 1.5707963f;  // π/2 lift
                        for (int f = 0; f < N; ++f) {
                            const float ph = float(f) / float(N);
                            mograph::LocalPose& p = c.frames[size_t(f)];
                            p.rootPos = {0, 0, 0};
                            p.rot.assign(6, mograph::Quatf::identity());
                            const float thL = (ph < 0.6f) ? 0.0f : H;
                            const float thR =
                                (ph >= 0.1f && ph < 0.5f) ? H : 0.0f;
                            p.rot[1] = mograph::Quatf::axisAngle(1, 0, 0, thL);
                            p.rot[3] = mograph::Quatf::axisAngle(1, 0, 0, thR);
                            p.rot[5] = mograph::Quatf::axisAngle(0, 0, 1, leanZ);
                        }
                        return c;
                    };
                    mograph::Clip cA = authorBob(40, 0.0f);
                    mograph::Clip cB = authorBob(30, 0.6f);  // shorter + leaned

                    // VAB-1: feet located by name (LeftToes / RightToes).
                    {
                        const auto ft = mograph::findFeet(sk);
                        if (ft[0] == 2 && ft[1] == 4)
                            pass("VAB-1 / findFeet locates left/right foot joints by name");
                        else
                            fail("VAB-1 / findFeet locates left/right foot joints by name",
                                 "got {" + std::to_string(ft[0]) + "," +
                                     std::to_string(ft[1]) + "}");
                    }

                    // VAB-2: detection finds a full ordered cycle (ok=true),
                    // monotone, in-range, starting at the planted-at-start frame.
                    bool okA = false, okB = false;
                    const auto kA = mograph::detectKeytimes(sk, cA, 0, 39, &okA);
                    const auto kB = mograph::detectKeytimes(sk, cB, 0, 29, &okB);
                    {
                        auto mono = [](const std::array<int, 5>& k, int hi) {
                            return k[0] >= 0 && k[0] < k[1] && k[1] < k[2] &&
                                   k[2] < k[3] && k[3] < k[4] && k[4] <= hi;
                        };
                        const bool shapeA = okA && mono(kA, 39) && kA[0] == 0 &&
                                            kA[1] >= 2 && kA[1] <= 6 &&
                                            kA[2] >= 18 && kA[2] <= 22 &&
                                            kA[3] >= 22 && kA[3] <= 26;
                        const bool shapeB = okB && mono(kB, 29) && kB[0] == 0 &&
                                            kB[1] >= 1 && kB[1] <= 5 &&
                                            kB[2] >= 13 && kB[2] <= 17 &&
                                            kB[3] >= 16 && kB[3] <= 20;
                        if (shapeA && shapeB)
                            pass("VAB-2 / keytime detection: ordered in-range cycle for both clips");
                        else
                            fail("VAB-2 / keytime detection: ordered in-range cycle for both clips",
                                 "A ok=" + std::to_string(int(okA)) + " {" +
                                     std::to_string(kA[0]) + "," + std::to_string(kA[1]) +
                                     "," + std::to_string(kA[2]) + "," + std::to_string(kA[3]) +
                                     "," + std::to_string(kA[4]) + "} B ok=" +
                                     std::to_string(int(okB)) + " {" + std::to_string(kB[0]) +
                                     "," + std::to_string(kB[1]) + "," + std::to_string(kB[2]) +
                                     "," + std::to_string(kB[3]) + "," + std::to_string(kB[4]) + "}");
                    }

                    // Build the blend: 1 tag, A at 0% / B at 100%.
                    mograph::VerbBlend vb;
                    vb.skel = sk;
                    vb.dt = cA.dt;
                    vb.tags = {"sad"};
                    vb.query = {50.0f};
                    {
                        mograph::VerbExample eA, eB;
                        eA.clip = cA; eA.key = kA; eA.adverb = {0.0f}; eA.name = "A";
                        eB.clip = cB; eB.key = kB; eB.adverb = {100.0f}; eB.name = "B";
                        mograph::pinClipInPlace(eA.clip);
                        mograph::pinClipInPlace(eB.clip);
                        vb.ex = {eA, eB};
                        vb.rebuild();
                    }

                    // VAB-3: canonical breakpoints monotone 0→1, and each event
                    // canon[k] warps to that example's own key[k] frame (k=0..3) —
                    // for BOTH examples sharing the one canon: A's RFD aligns with
                    // B's RFD, etc. (the structural-registration guarantee).
                    {
                        bool canonOk = vb.canon[0] == 0.0f &&
                                       std::fabs(vb.canon[4] - 1.0f) < 1e-5f;
                        for (int s = 0; s < 4; ++s)
                            if (vb.canon[s] >= vb.canon[s + 1]) canonOk = false;
                        bool alignOk = true;
                        for (int k = 0; k < 4 && alignOk; ++k) {
                            const float fa = mograph::verbWarpFrame(
                                vb.ex[0].key, vb.canon, vb.canon[k]);
                            const float fb = mograph::verbWarpFrame(
                                vb.ex[1].key, vb.canon, vb.canon[k]);
                            if (std::fabs(fa - float(vb.ex[0].key[k])) > 1e-2f ||
                                std::fabs(fb - float(vb.ex[1].key[k])) > 1e-2f)
                                alignOk = false;
                        }
                        if (canonOk && alignOk)
                            pass("VAB-3 / shared phase aligns same-event frames across both clips");
                        else
                            fail("VAB-3 / shared phase aligns same-event frames across both clips",
                                 "canonOk=" + std::to_string(int(canonOk)) +
                                     " alignOk=" + std::to_string(int(alignOk)));
                    }

                    // VAB-4: adverb RBF cardinals — query at an example's tag value
                    // returns that example, midpoint splits 50/50 (1-tag exact lerp).
                    {
                        std::vector<float> w0, w1, wm;
                        vb.weights({0.0f}, w0);
                        vb.weights({100.0f}, w1);
                        vb.weights({50.0f}, wm);
                        const bool ok = w0.size() == 2 && w1.size() == 2 &&
                                        std::fabs(w0[0] - 1.0f) < 1e-3f &&
                                        std::fabs(w0[1]) < 1e-3f &&
                                        std::fabs(w1[0]) < 1e-3f &&
                                        std::fabs(w1[1] - 1.0f) < 1e-3f &&
                                        std::fabs(wm[0] - 0.5f) < 1e-3f &&
                                        std::fabs(wm[1] - 0.5f) < 1e-3f;
                        if (ok)
                            pass("VAB-4 / adverb RBF: cardinal at samples, 50/50 at the midpoint");
                        else
                            fail("VAB-4 / adverb RBF: cardinal at samples, 50/50 at the midpoint",
                                 "wm={" + std::to_string(wm.empty() ? -9.0f : wm[0]) + "," +
                                     std::to_string(wm.size() < 2 ? -9.0f : wm[1]) + "}");
                    }

                    // VAB-5: the blended pose actually mixes the two — the chest
                    // lean (0 in A, 0.6 in B) lands halfway at the 50% query, and
                    // strictly between the pure-A and pure-B poses.
                    {
                        auto chestAngle = [&](const std::vector<float>& q) {
                            const auto saved = vb.query;
                            vb.query = q;
                            mograph::LocalPose lp;
                            vb.sampleMixed(0.3f, lp);
                            vb.query = saved;
                            float wq = lp.rot.size() > 5 ? std::fabs(lp.rot[5].w) : 1.0f;
                            if (wq > 1.0f) wq = 1.0f;
                            return 2.0f * std::acos(wq);  // angle from identity
                        };
                        const float a0 = chestAngle({0.0f});
                        const float a1 = chestAngle({100.0f});
                        const float am = chestAngle({50.0f});
                        if (a0 < 0.05f && std::fabs(a1 - 0.6f) < 0.05f &&
                            a0 < am && am < a1 && std::fabs(am - 0.3f) < 0.05f)
                            pass("VAB-5 / blended pose interpolates the style channel by adverb");
                        else
                            fail("VAB-5 / blended pose interpolates the style channel by adverb",
                                 "a0=" + std::to_string(a0) + " am=" + std::to_string(am) +
                                     " a1=" + std::to_string(a1));
                    }

                    // VAB-6: the phase clock is the mean cycle duration and the
                    // world-pose sampler runs (non-empty, right joint count).
                    {
                        const double expect =
                            0.5 * (double(kA[4] - kA[0]) + double(kB[4] - kB[0])) *
                            double(cA.dt);
                        bvh::Pose wp;
                        const bool sok = vb.sample(0.0, wp);
                        if (std::fabs(vb.cycleSec - expect) < 1e-4 && sok &&
                            wp.world.size() == sk.joints.size())
                            pass("VAB-6 / cycle clock = mean cycle, world sampler runs");
                        else
                            fail("VAB-6 / cycle clock = mean cycle, world sampler runs",
                                 "cycleSec=" + std::to_string(vb.cycleSec) + " expect=" +
                                     std::to_string(expect) + " sampleOk=" +
                                     std::to_string(int(sok)));
                    }

                    // VAB-9: signed weights (convexWeights off) EXTRAPOLATE past
                    // an endpoint — querying 150% drives the chest lean beyond
                    // pure B (100%); convex clamps it back at B. Confirms the
                    // negative-weight path actually reaches the pose (not dropped
                    // by an internal w≤0 skip).
                    {
                        auto chestAt = [&](float qv, bool convex) {
                            const auto sq = vb.query;
                            const bool sc = vb.convexWeights;
                            vb.query = {qv};
                            vb.convexWeights = convex;
                            mograph::LocalPose lp;
                            vb.sampleMixed(0.3f, lp);
                            vb.query = sq;
                            vb.convexWeights = sc;
                            float w = lp.rot.size() > 5 ? std::fabs(lp.rot[5].w) : 1.0f;
                            if (w > 1.0f) w = 1.0f;
                            return 2.0f * std::acos(w);
                        };
                        const float pureB = chestAt(100.0f, false);  // ≈0.6
                        const float ext = chestAt(150.0f, false);    // extrapolate
                        const float clamp = chestAt(150.0f, true);   // convex clamp
                        if (ext > pureB + 0.1f && std::fabs(clamp - pureB) < 0.05f)
                            pass("VAB-9 / signed weights extrapolate past an endpoint; convex clamps");
                        else
                            fail("VAB-9 / signed weights extrapolate past an endpoint; convex clamps",
                                 "pureB=" + std::to_string(pureB) + " ext=" +
                                     std::to_string(ext) + " clamp=" +
                                     std::to_string(clamp));
                    }
                }

                // VAB-7: full Simulator path on real assets — build a 2-motion
                // blend from WalkLoopA + SneakLoopA and sample through the normal
                // pose pipeline. Asserts the build succeeds, every sampled frame
                // is finite with the proxy joint count, and the adverb query
                // actually drives the pose (0% ≠ 100%).
                if (auto* k = sim.kinematicOf(0)) {
                    k->verbFiles = {"WalkLoopA.bvh", "SneakLoopA.bvh"};
                    const bool built = sim.buildKinematicVerb(0);
                    bool finite = built && k->verbActive();
                    bvh::Pose p0, p1;
                    if (finite) {
                        for (int q = 0; q <= 30 && finite; ++q) {
                            bvh::Pose p;
                            k->sampleWorldPose(q * 0.05, p);
                            if (p.world.size() != k->motion.joints.size()) {
                                finite = false;
                                break;
                            }
                            for (const auto& jx : p.world)
                                for (int c = 0; c < 3; ++c)
                                    if (!std::isfinite(jx.t[c])) finite = false;
                        }
                        k->verbBlend.query = {0.0f};
                        k->sampleWorldPose(0.4, p0);
                        k->verbBlend.query = {100.0f};
                        k->sampleWorldPose(0.4, p1);
                    }
                    float dmax = 0.0f;
                    for (size_t j = 0; finite && j < p0.world.size() &&
                                       j < p1.world.size(); ++j)
                        for (int c = 0; c < 3; ++c)
                            dmax = std::max(dmax, std::fabs(p0.world[j].t[c] -
                                                            p1.world[j].t[c]));
                    if (finite && dmax > 1e-3f)
                        pass("VAB-7 / Simulator builds 2-motion blend from real assets; adverb drives a finite pose");
                    else
                        fail("VAB-7 / Simulator builds 2-motion blend from real assets; adverb drives a finite pose",
                             "built=" + std::to_string(int(built)) + " finite=" +
                                 std::to_string(int(finite)) + " dmax=" +
                                 std::to_string(dmax));

                    // VAB-8: the blend-preview ghost (writeVerbGhost) yields
                    // finite proxy verts that change with the adverb query — so
                    // the strobe re-poses live as the slider moves.
                    std::vector<float> g0, g1;
                    k->verbBlend.query = {0.0f};
                    k->writeVerbGhost(0.3, tinym::vec3(1, 1, 1), Quat{},
                                      k->params.center, g0);
                    k->verbBlend.query = {100.0f};
                    k->writeVerbGhost(0.3, tinym::vec3(1, 1, 1), Quat{},
                                      k->params.center, g1);
                    bool ghostOk = built && !g0.empty() && g0.size() == g1.size();
                    float gd = 0.0f;
                    for (size_t i = 0; ghostOk && i < g0.size(); ++i) {
                        if (!std::isfinite(g0[i]) || !std::isfinite(g1[i]))
                            ghostOk = false;
                        gd = std::max(gd, std::fabs(g0[i] - g1[i]));
                    }
                    if (ghostOk && gd > 1e-4f)
                        pass("VAB-8 / blend-preview ghost is finite and re-poses with the adverb query");
                    else
                        fail("VAB-8 / blend-preview ghost is finite and re-poses with the adverb query",
                             "ghostOk=" + std::to_string(int(ghostOk)) + " gd=" +
                                 std::to_string(gd));

                    // VAB-10: the blended root TRAVELS (no treadmill) — net path
                    // grows over ~10 s of open-ended play while every
                    // consecutive-frame root step stays small (no seam teleport).
                    {
                        k->verbBlend.query = {50.0f};
                        float maxStep = 0.0f, pathLen = 0.0f;
                        bvh::Pose prev, cur;
                        const double fdt = 1.0 / 60.0;
                        for (int q = 0; q <= 600; ++q) {
                            k->sampleWorldPose(q * fdt, cur);
                            if (cur.world.empty()) continue;
                            if (q > 0 && !prev.world.empty()) {
                                const float dx = cur.world[0].t[0] - prev.world[0].t[0];
                                const float dz = cur.world[0].t[2] - prev.world[0].t[2];
                                const float step = std::sqrt(dx * dx + dz * dz);
                                pathLen += step;
                                maxStep = std::max(maxStep, step);
                            }
                            prev = cur;
                        }
                        const float h = k->verbBlend.skel.height;
                        if (pathLen > 0.5f * h && maxStep < 0.05f * h)
                            pass("VAB-10 / blended root travels across cycles without seam teleport");
                        else
                            fail("VAB-10 / blended root travels across cycles without seam teleport",
                                 "pathLen=" + std::to_string(pathLen) + " maxStep=" +
                                     std::to_string(maxStep) + " h=" + std::to_string(h));
                    }

                    // VAB-11 (root sign regression): the SAME forward motion
                    // captured at two different WORLD yaws must re-root to the SAME
                    // travel direction (re-rooting removes world yaw). If the
                    // position alignment is the inverse rotation of the heading
                    // alignment, the off-yaw clip travels OPPOSITE the on-yaw one —
                    // the moonwalk the user saw. Clip walks its own +local forward
                    // placed at world yaw `yaw`; built on the real skeleton.
                    {
                        const mograph::Skeleton rs = k->skel();
                        auto fwdClip = [&](float yaw) {
                            mograph::Clip c;
                            c.dt = 1.0f / 30.0f;
                            c.frames.resize(40);
                            const float cy = std::cos(yaw), sy = std::sin(yaw);
                            for (int f = 0; f < 40; ++f) {
                                auto& p = c.frames[size_t(f)];
                                p.rot.assign(rs.joints.size(), mograph::Quatf::identity());
                                if (!p.rot.empty()) p.rot[0] = mograph::Quatf::yaw(yaw);
                                const float d = 0.05f * float(f);    // forward distance
                                p.rootPos = {d * sy, 0.9f, d * cy};  // R_Y(yaw)·(0,0,d)
                            }
                            return c;
                        };
                        mograph::VerbBlend mv;
                        mv.skel = rs;
                        mv.tags = {"t"};
                        mv.convexWeights = false;
                        mograph::VerbExample ea, eb;
                        ea.clip = fwdClip(0.0f);     // forward = +Z, yaw0 = 0
                        ea.key = {0, 10, 20, 30, 39};
                        ea.adverb = {0.0f};
                        eb.clip = fwdClip(1.5708f);  // forward = +X, yaw0 = 90°
                        eb.key = {0, 10, 20, 30, 39};
                        eb.adverb = {100.0f};
                        mv.ex = {ea, eb};
                        mv.rebuild();
                        auto dir = [&](float q) {
                            mv.query = {q};
                            bvh::Pose s0, s1;
                            mv.sample(0.0, s0);
                            mv.sample(2.0 * mv.cycleSec, s1);
                            const float dx = s1.world[0].t[0] - s0.world[0].t[0];
                            const float dz = s1.world[0].t[2] - s0.world[0].t[2];
                            return std::array<float, 3>{dx, dz,
                                                        std::sqrt(dx * dx + dz * dz)};
                        };
                        const auto da = dir(0.0f), db = dir(100.0f);
                        const float cosang =
                            (da[2] > 1e-4f && db[2] > 1e-4f)
                                ? (da[0] * db[0] + da[1] * db[1]) / (da[2] * db[2])
                                : 0.0f;
                        if (da[2] > 0.3f && db[2] > 0.3f && cosang > 0.9f)
                            pass("VAB-11 / re-rooted travel is world-yaw invariant (root sign matches heading)");
                        else
                            fail("VAB-11 / re-rooted travel is world-yaw invariant (root sign matches heading)",
                                 "|a|=" + std::to_string(da[2]) + " |b|=" +
                                     std::to_string(db[2]) + " cos=" +
                                     std::to_string(cosang));
                    }

                    // VAB-12 (tag weight routing): the adverb weight must reach
                    // the ROOT with the same sign/index it reaches the pose, so
                    // each endpoint's travel matches ITS clip. Two clips facing
                    // the SAME way (yaw0=0, so re-rooting is a no-op) but
                    // TRANSLATING perpendicular: A→+Z, B→+X. Then dir(q=0) must be
                    // +Z (pure A) and dir(q=100) must be +X (pure B). A flipped
                    // weight sign, a swapped w[0]/w[1] index, or routing the wrong
                    // weight to exDeltaL would rotate or swap these — invisible to
                    // VAB-11 (same-direction clips) and to a "never reverses" test
                    // (perpendicular, not opposite). Real Walk+Sneak both ≈+Z, so
                    // this needs the synthetic perpendicular pair to have teeth.
                    {
                        const mograph::Skeleton rs = k->skel();
                        auto travelClip = [&](float dirAng) {
                            mograph::Clip c;
                            c.dt = 1.0f / 30.0f;
                            c.frames.resize(40);
                            const float cd = std::cos(dirAng), sd = std::sin(dirAng);
                            for (int f = 0; f < 40; ++f) {
                                auto& p = c.frames[size_t(f)];
                                p.rot.assign(rs.joints.size(),
                                             mograph::Quatf::identity());  // yaw0=0
                                const float d = 0.05f * float(f);
                                p.rootPos = {d * sd, 0.9f, d * cd};  // travels @dirAng
                            }
                            return c;
                        };
                        mograph::VerbBlend mv;
                        mv.skel = rs;
                        mv.tags = {"t"};
                        mv.convexWeights = false;
                        mograph::VerbExample ea, eb;
                        ea.clip = travelClip(0.0f);       // A travels +Z
                        ea.key = {0, 10, 20, 30, 39};
                        ea.adverb = {0.0f};
                        eb.clip = travelClip(1.5708f);    // B travels +X
                        eb.key = {0, 10, 20, 30, 39};
                        eb.adverb = {100.0f};
                        mv.ex = {ea, eb};
                        mv.rebuild();
                        auto dir = [&](float q) {
                            mv.query = {q};
                            bvh::Pose s0, s1;
                            mv.sample(0.0, s0);
                            mv.sample(2.0 * mv.cycleSec, s1);
                            const float dx = s1.world[0].t[0] - s0.world[0].t[0];
                            const float dz = s1.world[0].t[2] - s0.world[0].t[2];
                            return std::array<float, 3>{
                                dx, dz, std::sqrt(dx * dx + dz * dz)};
                        };
                        const auto d0 = dir(0.0f), d1 = dir(100.0f);
                        // q=0 → +Z (dz>0, |dx| small); q=100 → +X (dx>0, |dz| small).
                        const float cosA0 = d0[2] > 1e-4f ? d0[1] / d0[2] : 0.0f;
                        const float cosX1 = d1[2] > 1e-4f ? d1[0] / d1[2] : 0.0f;
                        if (d0[2] > 0.3f && d1[2] > 0.3f && cosA0 > 0.9f &&
                            cosX1 > 0.9f)
                            pass("VAB-12 / adverb weight routes to the root with correct sign+index (endpoints match their clip)");
                        else
                            fail("VAB-12 / adverb weight routes to the root with correct sign+index (endpoints match their clip)",
                                 "d0=(" + std::to_string(d0[0]) + "," +
                                     std::to_string(d0[1]) + ") d1=(" +
                                     std::to_string(d1[0]) + "," +
                                     std::to_string(d1[1]) + ") cosA0=" +
                                     std::to_string(cosA0) + " cosX1=" +
                                     std::to_string(cosX1));
                    }

                    // VAB-13 (real-asset endpoint travel, the user's exact case):
                    // Walk(0%) + Sneak(100%) where SneakLoopA travels world −Z and
                    // faces ~143° while Walk faces ~12°. After re-root BOTH pure
                    // endpoints must travel FORWARD (net dz>0) and face the forward
                    // hemisphere (|heading|<90°) on the live sampleWorldPose path —
                    // i.e. the kinematic forward, not the clip's raw world heading.
                    // The pre-d9c1f85 world-frame blend left Sneak going raw −Z
                    // (the "perfectly flipped 100%" the user reported); the
                    // synthetic VAB-11/12 clips travel along their facing so they
                    // miss it. This guards the real off-axis clip on the live path.
                    {
                        auto endpoint = [&](float q) {
                            k->verbBlend.query = {q};
                            bvh::Pose pa, pb;
                            k->sampleWorldPose(0.5, pa);
                            k->sampleWorldPose(5.0, pb);  // ~4.5 s of travel
                            float dx = 0.0f, dz = 0.0f, hd = 0.0f;
                            if (!pa.world.empty() && !pb.world.empty()) {
                                dx = pb.world[0].t[0] - pa.world[0].t[0];
                                dz = pb.world[0].t[2] - pa.world[0].t[2];
                                hd = std::atan2(pb.world[0].R[2], pb.world[0].R[0]);
                            }
                            return std::array<float, 3>{dx, dz, hd};
                        };
                        const auto e0 = endpoint(0.0f), e1 = endpoint(100.0f);
                        const float half = 1.5708f;  // 90°
                        const bool fwd0 = e0[1] > 0.0f && std::fabs(e0[2]) < half;
                        const bool fwd1 = e1[1] > 0.0f && std::fabs(e1[2]) < half;
                        if (fwd0 && fwd1)
                            pass("VAB-13 / real Walk+Sneak: both pure endpoints travel forward after re-root (off-axis clip not flipped)");
                        else
                            fail("VAB-13 / real Walk+Sneak: both pure endpoints travel forward after re-root (off-axis clip not flipped)",
                                 "walk dz=" + std::to_string(e0[1]) + " head=" +
                                     std::to_string(e0[2]) + " | sneak dz=" +
                                     std::to_string(e1[1]) + " head=" +
                                     std::to_string(e1[2]));
                    }

                    // VAB-14 (preview matches the body): the blend-preview ghost
                    // samples sampleMixed, which must re-root each clip like the
                    // live body (xz+yaw removed) — only the per-cycle travel may
                    // differ. At q=100 (Sneak-dominant, raw heading ~143°) the
                    // ghost root must face FORWARD (|heading|<90°), same hemisphere
                    // as the body. Without the re-root sampleMixed returned the raw
                    // ~143° heading → the "preview renders flipped motion" the user
                    // saw. Also asserts the in-place ghost root and the body root
                    // share the same heading at the same phase (preview == body
                    // pose, modulo travel).
                    {
                        k->verbBlend.query = {100.0f};
                        const float ph = 0.3f;
                        mograph::LocalPose lp;
                        k->verbBlend.sampleMixed(ph, lp);
                        bvh::Pose ghost;
                        mograph::fk(k->verbBlend.skel, lp, ghost);
                        bvh::Pose body;
                        k->verbBlend.sample(double(ph) * k->verbBlend.cycleSec,
                                            body);
                        float gh = 9.0f, bh = 9.0f;
                        if (!ghost.world.empty())
                            gh = std::atan2(ghost.world[0].R[2],
                                            ghost.world[0].R[0]);
                        if (!body.world.empty())
                            bh = std::atan2(body.world[0].R[2],
                                            body.world[0].R[0]);
                        const float half = 1.5708f;
                        if (std::fabs(gh) < half && std::fabs(bh) < half &&
                            std::fabs(gh - bh) < 1e-3f)
                            pass("VAB-14 / blend preview re-roots like the body (faces forward, not the clip's raw heading)");
                        else
                            fail("VAB-14 / blend preview re-roots like the body (faces forward, not the clip's raw heading)",
                                 "ghostHeading=" + std::to_string(gh) +
                                     " bodyHeading=" + std::to_string(bh));
                    }

                    // VAB-15 (N-motion path, mode 6): the keytime blend
                    // generalizes past two motions. Three synthetic clips
                    // (yaw0=0, traveling +Z / +X / −Z) with adverbs 0/50/100 →
                    // the RBF cardinals must satisfy w_i(adverb_j)=δ_ij for n=3
                    // (exercises the n×n solve, not the 2-sample linear
                    // shortcut), AND the N-way blend must route the cardinal
                    // weight to the ROOT: at the middle example's adverb the body
                    // travels that example's direction (+X), via blendPoseN — the
                    // n>2 path, not the n==2 blendPose used by modes-5 endpoints.
                    {
                        const mograph::Skeleton rs = k->skel();
                        auto travelClip = [&](float dirAng) {
                            mograph::Clip c;
                            c.dt = 1.0f / 30.0f;
                            c.frames.resize(40);
                            const float cd = std::cos(dirAng), sd = std::sin(dirAng);
                            for (int f = 0; f < 40; ++f) {
                                auto& p = c.frames[size_t(f)];
                                p.rot.assign(rs.joints.size(),
                                             mograph::Quatf::identity());
                                const float d = 0.05f * float(f);
                                p.rootPos = {d * sd, 0.9f, d * cd};
                            }
                            return c;
                        };
                        mograph::VerbBlend mv;
                        mv.skel = rs;
                        mv.tags = {"t"};
                        mv.convexWeights = false;
                        const float angs[3] = {0.0f, 1.5708f, 3.14159f};  // +Z,+X,−Z
                        const float advs[3] = {0.0f, 50.0f, 100.0f};
                        for (int i = 0; i < 3; ++i) {
                            mograph::VerbExample e;
                            e.clip = travelClip(angs[i]);
                            e.key = {0, 10, 20, 30, 39};
                            e.adverb = {advs[i]};
                            mv.ex.push_back(e);
                        }
                        mv.rebuild();
                        // Each of the THREE cardinal endpoints must route its
                        // weight to the root so the body travels THAT clip's
                        // direction: q=0→+Z, q=50→+X, q=100→−Z. The −Z third clip
                        // is the teeth: a 2-cap on the root accumulate (the old
                        // `for i<2` bug) or a dropped n>2 example zeroes q=100's
                        // travel; a mis-route tilts the +X middle off-axis.
                        auto dir = [&](float q) {
                            mv.query = {q};
                            bvh::Pose s0, s1;
                            mv.sample(0.0, s0);
                            mv.sample(2.0 * mv.cycleSec, s1);
                            float dx = 0.0f, dz = 0.0f;
                            if (!s0.world.empty() && !s1.world.empty()) {
                                dx = s1.world[0].t[0] - s0.world[0].t[0];
                                dz = s1.world[0].t[2] - s0.world[0].t[2];
                            }
                            return std::array<float, 3>{
                                dx, dz, std::sqrt(dx * dx + dz * dz)};
                        };
                        const auto z0 = dir(0.0f), x1 = dir(50.0f), z2 = dir(100.0f);
                        const bool nReady = (mv.ex.size() == 3);
                        const bool okZ0 = z0[2] > 0.3f && z0[1] > 0.0f &&
                                          std::fabs(z0[0]) < 0.5f * z0[2];
                        const bool okX1 = x1[2] > 0.3f && x1[0] > 0.0f &&
                                          std::fabs(x1[1]) < 0.5f * x1[2];
                        const bool okZ2 = z2[2] > 0.3f && z2[1] < 0.0f &&
                                          std::fabs(z2[0]) < 0.5f * z2[2];
                        if (nReady && okZ0 && okX1 && okZ2)
                            pass("VAB-15 / N-motion (3) keytime blend: each cardinal routes the root to its clip (+Z / +X / −Z)");
                        else
                            fail("VAB-15 / N-motion (3) keytime blend: each cardinal routes the root to its clip (+Z / +X / −Z)",
                                 "n=" + std::to_string(mv.ex.size()) + " z0=(" +
                                     std::to_string(z0[0]) + "," +
                                     std::to_string(z0[1]) + ") x1=(" +
                                     std::to_string(x1[0]) + "," +
                                     std::to_string(x1[1]) + ") z2=(" +
                                     std::to_string(z2[0]) + "," +
                                     std::to_string(z2[1]) + ")");
                    }

                    // VAB-16 (real N-motion build path, mode 6): the wiring —
                    // not just the engine — must load >2 files. Build a 3-motion
                    // verb blend through buildKinematicVerb under motionMode=6
                    // using three compatible MBS clips. The old `for i<2` loop
                    // would bake only 2 examples; verbActive() keyed on `==5`
                    // alone would report inactive under mode 6. Assert three
                    // examples baked, active, and a finite forward sample.
                    {
                        k->verbFiles = {"WalkLoopA.bvh", "jogCurve.bvh",
                                        "SneakLoopA.bvh"};
                        k->verbAdverb = {{0, 0}, {50, 0}, {100, 0}};
                        k->verbTags = {"t"};
                        k->motionMode = 6;
                        const bool built = sim.buildKinematicVerb(0);
                        const int nEx = int(k->verbBlend.ex.size());
                        const bool active = k->verbActive() && k->motionMode == 6;
                        bvh::Pose pa, pb;
                        k->verbBlend.query = {50.0f};
                        // Probe forward travel WITHIN cycle 0 — the middle motion
                        // (jogCurve) curves, so over many loops the (correct)
                        // heading accumulation turns the path and straight-Z is no
                        // longer "forward". One cycle has no loop rotation yet, so
                        // dz>0 still means forward. (No-moonwalk is VAB-23's job.)
                        const double cyc16 = k->verbBlend.cycleSec;
                        k->sampleWorldPose(0.05 * cyc16, pa);
                        k->sampleWorldPose(0.85 * cyc16, pb);
                        float dz = 0.0f;
                        bool finite = !pa.world.empty() && !pb.world.empty();
                        if (finite) {
                            dz = pb.world[0].t[2] - pa.world[0].t[2];
                            finite = std::isfinite(dz);
                        }
                        if (built && nEx == 3 && active && finite && dz > 0.0f)
                            pass("VAB-16 / real build loads 3 motions under mode 6 (verbActive, 3 examples, forward travel)");
                        else
                            fail("VAB-16 / real build loads 3 motions under mode 6 (verbActive, 3 examples, forward travel)",
                                 "built=" + std::to_string(built) + " nEx=" +
                                     std::to_string(nEx) + " active=" +
                                     std::to_string(active) + " dz=" +
                                     std::to_string(dz));
                    }

                    // VAB-17 (keytime preview writer): the filmstrip ghosts one
                    // motion's clip at each of its 5 keytime frames — writeGhost
                    // at those frames (what drawGhostPreviews renders) must yield
                    // finite, non-empty poses, and distinct gait events give
                    // distinct vertex sets (a collapsed keytime or broken sampler
                    // would not). Reuses VAB-16's built 3-motion blend.
                    if (k->verbActive() && !k->verbBlend.ex.empty()) {
                        bool finite = true, distinct = false;
                        std::vector<float> prev;
                        const auto& ke = k->verbBlend.ex[0];
                        for (int kk = 0; kk < 5 && finite; ++kk) {
                            std::vector<float> g;
                            k->writeGhost(ke.clip, ke.key[kk], tinym::vec3(1, 1, 1),
                                          Quat{}, tinym::vec3(0, 0, 0), g, ke.key[kk]);
                            if (g.empty()) { finite = false; break; }
                            for (float v : g)
                                if (!std::isfinite(v)) { finite = false; break; }
                            if (!prev.empty() && g.size() == prev.size())
                                for (size_t i = 0; i < g.size(); ++i)
                                    if (std::fabs(g[i] - prev[i]) > 1e-4f) {
                                        distinct = true;
                                        break;
                                    }
                            prev = g;
                        }
                        if (finite && distinct)
                            pass("VAB-17 / keytime preview: writeGhost yields finite distinct poses at the 5 keytime frames");
                        else
                            fail("VAB-17 / keytime preview: writeGhost yields finite distinct poses at the 5 keytime frames",
                                 "finite=" + std::to_string(finite) + " distinct=" +
                                     std::to_string(distinct));
                    } else {
                        skip("VAB-17", "verb blend not active");
                    }

                    // VAB-18 (루프 연장 = clip doubling): the loop-extend toggle
                    // appends a copy so keytimes/range span two loops.
                    // doubleClipFrames must 2× the frame count, copy joint
                    // rotations verbatim (2nd-half pose == 1st-half), and CONTINUE
                    // the root forward (copy rootPos = original + per-cycle delta)
                    // so a cycle placed in the 2nd half still travels.
                    if (k->verbActive() && !k->verbBlend.ex.empty()) {
                        mograph::Clip c = k->verbBlend.ex[0].clip;
                        const int n = int(c.frames.size());
                        const auto pos0 = c.frames[0].rootPos;
                        const auto posL = c.frames[n - 1].rootPos;
                        mograph::doubleClipFrames(c);
                        const bool len2x = int(c.frames.size()) == 2 * n;
                        bool rotCopied = len2x && n > 1;
                        if (rotCopied) {
                            const auto& a = c.frames[1].rot;
                            const auto& b = c.frames[n + 1].rot;
                            rotCopied = (a.size() == b.size());
                            for (size_t j = 0; j < a.size() && rotCopied; ++j)
                                if (std::fabs(a[j].x - b[j].x) +
                                        std::fabs(a[j].y - b[j].y) +
                                        std::fabs(a[j].z - b[j].z) +
                                        std::fabs(a[j].w - b[j].w) > 1e-5f)
                                    rotCopied = false;
                        }
                        bool rootCont = len2x;
                        if (rootCont) {
                            const auto pn = c.frames[n].rootPos;  // first copy frame
                            rootCont =
                                std::fabs(pn[0] - (pos0[0] + posL[0] - pos0[0])) <
                                    1e-3f &&
                                std::fabs(pn[2] - (pos0[2] + posL[2] - pos0[2])) <
                                    1e-3f;
                        }
                        if (len2x && rotCopied && rootCont)
                            pass("VAB-18 / 루프 연장 doubles the clip (rotations copied, root continued)");
                        else
                            fail("VAB-18 / 루프 연장 doubles the clip (rotations copied, root continued)",
                                 "len2x=" + std::to_string(len2x) + " rotCopied=" +
                                     std::to_string(rotCopied) + " rootCont=" +
                                     std::to_string(rootCont));
                    } else {
                        skip("VAB-18", "verb blend not active");
                    }

                    // VAB-19 (N-blend preset): applying the preset configures +
                    // builds a 3-motion mode-6 blend in ONE call — 3 examples,
                    // tag "speed", adverbs 0/50/100, the hand-tuned keytimes land
                    // (clamped), and motions 1&2 are loop-extended (clips doubled
                    // so a 135/175-frame keytime is reachable). Skips if the
                    // preset assets are absent from this build's asset dir.
                    {
                        const auto& P = verbPresets()[0];
                        const std::string adir = bvhAssetDir();
                        bool haveFiles = true;
                        for (const char* f : P.files)
                            if (!std::filesystem::exists(adir + "/" + std::string(f)))
                                haveFiles = false;
                        if (!haveFiles) {
                            skip("VAB-19", "preset assets missing");
                        } else {
                            const bool ok = sim.setKinematicVerbPreset(0, 0);
                            const int nEx =
                                k->verbActive() ? int(k->verbBlend.ex.size()) : 0;
                            bool tagOk = k->verbBlend.tags.size() == 1 &&
                                         k->verbBlend.tags[0] == "speed";
                            bool advOk = nEx == 3;
                            for (int i = 0; i < nEx && advOk; ++i)
                                advOk = std::fabs(k->verbBlend.ex[i].adverb[0] -
                                                  P.adverbs[i][0]) < 1e-3f;
                            bool keyOk = nEx == 3, noClamp = nEx == 3;
                            for (int i = 0; i < nEx && keyOk; ++i) {
                                const int nf =
                                    int(k->verbBlend.ex[i].clip.frames.size());
                                for (int s2 = 0; s2 < 5; ++s2) {
                                    int want = P.keys[i][s2];
                                    if (want > nf - 1) {
                                        want = nf - 1;
                                        noClamp = false;  // clip too short → unfaithful
                                    }
                                    if (k->verbBlend.ex[i].key[s2] != want) {
                                        keyOk = false;
                                        break;
                                    }
                                }
                            }
                            const bool loopOk = k->motionSlots[0].loopSel == 1 &&
                                                k->motionSlots[1].loopSel == 1 &&
                                                k->motionSlots[2].loopSel == 0;
                            const bool modeOk =
                                ok && k->motionMode == 6 && k->verbPreset == 0;
                            if (modeOk && nEx == 3 && tagOk && advOk && keyOk &&
                                noClamp && loopOk)
                                pass("VAB-19 / N-blend preset configures+builds a 3-motion mode-6 blend (keytimes/tag/adverbs/loop)");
                            else
                                fail("VAB-19 / N-blend preset configures+builds a 3-motion mode-6 blend (keytimes/tag/adverbs/loop)",
                                     "ok=" + std::to_string(ok) + " nEx=" +
                                         std::to_string(nEx) + " tag=" +
                                         std::to_string(tagOk) + " adv=" +
                                         std::to_string(advOk) + " key=" +
                                         std::to_string(keyOk) + " noClamp=" +
                                         std::to_string(noClamp) + " loop=" +
                                         std::to_string(loopOk) + " mode=" +
                                         std::to_string(k->motionMode) + " preset=" +
                                         std::to_string(k->verbPreset));
                        }
                    }

                    // VAB-20 (N>2 adverb extrapolation moves the pose): the
                    // convex blendPoseN* drop w≤0, so driving the adverb PAST an
                    // example had no pose effect for N≥3 (only the 2-motion slerp
                    // path extrapolated). sampleMixed now routes a signed-weight
                    // query to blendPoseNSigned. 3 clips with a distinct joint-1
                    // yaw (0 / 0.3 / 0.6 rad) at adverbs 0/50/100; query=150 (past
                    // motion 2) must bend joint 1 FURTHER than the pure motion-2
                    // pose at query=100 (the convex path would clamp to 0.6).
                    {
                        const mograph::Skeleton rs = k->skel();
                        auto poseClip = [&](float yaw1) {
                            mograph::Clip c;
                            c.dt = 1.0f / 30.0f;
                            c.frames.resize(8);
                            for (int f = 0; f < 8; ++f) {
                                auto& p = c.frames[size_t(f)];
                                p.rot.assign(rs.joints.size(),
                                             mograph::Quatf::identity());
                                if (rs.joints.size() > 1)
                                    p.rot[1] = mograph::Quatf::yaw(yaw1);
                                p.rootPos = {0.0f, 0.9f, 0.05f * float(f)};
                            }
                            return c;
                        };
                        mograph::VerbBlend mv;
                        mv.skel = rs;
                        mv.tags = {"t"};
                        mv.convexWeights = false;  // extrapolation enabled
                        const float yaws[3] = {0.0f, 0.3f, 0.6f};
                        const float adv[3] = {0.0f, 50.0f, 100.0f};
                        for (int i = 0; i < 3; ++i) {
                            mograph::VerbExample e;
                            e.clip = poseClip(yaws[i]);
                            e.key = {0, 2, 4, 6, 7};
                            e.adverb = {adv[i]};
                            mv.ex.push_back(e);
                        }
                        mv.rebuild();
                        auto j1yaw = [&](float q) {
                            mv.query = {q};
                            mograph::LocalPose lp;
                            mv.sampleMixed(0.0f, lp);
                            if (lp.rot.size() < 2) return 0.0f;
                            std::array<float, 9> R;
                            lp.rot[1].toMat3(R);
                            return std::atan2(R[2], R[0]);
                        };
                        const float a100 = j1yaw(100.0f), a150 = j1yaw(150.0f);
                        if (a150 > a100 + 0.05f && a100 > 0.4f)
                            pass("VAB-20 / N>2 adverb extrapolation bends the pose past the example (signed blend, not dropped)");
                        else
                            fail("VAB-20 / N>2 adverb extrapolation bends the pose past the example (signed blend, not dropped)",
                                 "a100=" + std::to_string(a100) + " a150=" +
                                     std::to_string(a150));
                    }

                    // VAB-21 (주기끝 = 착지 + 1 loop length): verbSetCycleEndFull
                    // sets key[4] = key[0] + one loop = nf/2 when 루프 연장 doubled
                    // the clip. Force motion 0 loop-extended, set LFD=5, press the
                    // button → cycleEnd = 5 + nf/2 (clamped, kept after LFU).
                    if (k->verbActive() && !k->verbBlend.ex.empty()) {
                        sim.verbSetLoopSel(0, 0, 1);     // force loop-extend (2×)
                        sim.verbSetKeytime(0, 0, 0, 5);  // LFD = 5
                        sim.verbSetCycleEndFull(0, 0);
                        const int nf = int(k->verbBlend.ex[0].clip.frames.size());
                        const auto& key = k->verbBlend.ex[0].key;
                        int want = 5 + nf / 2;
                        if (want > nf - 1) want = nf - 1;
                        if (want <= key[3]) want = std::min(nf - 1, key[3] + 1);
                        const bool ok = key[0] == 5 && key[4] == want &&
                                        k->motionSlots[0].loopSel == 1;
                        if (ok)
                            pass("VAB-21 / 주기끝 = 착지 + one loop length (nf/2 when loop-extended)");
                        else
                            fail("VAB-21 / 주기끝 = 착지 + one loop length (nf/2 when loop-extended)",
                                 "key0=" + std::to_string(key[0]) + " key3=" +
                                     std::to_string(key[3]) + " key4=" +
                                     std::to_string(key[4]) + " nf=" +
                                     std::to_string(nf) + " want=" +
                                     std::to_string(want));
                    } else {
                        skip("VAB-21", "verb blend not active");
                    }

                    // VAB-22 (2-tag, 4-motion preset): the curvy×jog preset uses
                    // 2 adverb tags and 4 motions — exercises the N-tag preset
                    // apply (2D adverbs) + the 4-slot pool. Assert 4 examples,
                    // tags curvy/jog, 2D adverbs match, keytimes land without
                    // clamp, only motion 4 loop-extended. Skips if assets absent.
                    {
                        const auto& P = verbPresets()[1];
                        const std::string adir = bvhAssetDir();
                        bool haveFiles = true;
                        for (const char* f : P.files)
                            if (!std::filesystem::exists(adir + "/" + std::string(f)))
                                haveFiles = false;
                        if (!haveFiles) {
                            skip("VAB-22", "preset assets missing");
                        } else {
                            const bool ok = sim.setKinematicVerbPreset(0, 1);
                            const int nEx =
                                k->verbActive() ? int(k->verbBlend.ex.size()) : 0;
                            const bool tagOk = k->verbBlend.tags.size() == 2 &&
                                               k->verbBlend.tags[0] == "curvy" &&
                                               k->verbBlend.tags[1] == "jog";
                            bool advOk = nEx == 4;
                            for (int i = 0; i < nEx && advOk; ++i)
                                advOk =
                                    std::fabs(k->verbBlend.ex[i].adverb[0] -
                                              P.adverbs[i][0]) < 1e-3f &&
                                    std::fabs(k->verbBlend.ex[i].adverb[1] -
                                              P.adverbs[i][1]) < 1e-3f;
                            bool keyOk = nEx == 4, noClamp = nEx == 4;
                            for (int i = 0; i < nEx && keyOk; ++i) {
                                const int nf =
                                    int(k->verbBlend.ex[i].clip.frames.size());
                                for (int s2 = 0; s2 < 5; ++s2) {
                                    int want = P.keys[i][s2];
                                    if (want > nf - 1) { want = nf - 1; noClamp = false; }
                                    if (k->verbBlend.ex[i].key[s2] != want) {
                                        keyOk = false;
                                        break;
                                    }
                                }
                            }
                            const bool loopOk = k->motionSlots[0].loopSel == 0 &&
                                                k->motionSlots[1].loopSel == 0 &&
                                                k->motionSlots[2].loopSel == 0 &&
                                                k->motionSlots[3].loopSel == 1;
                            const bool modeOk =
                                ok && k->motionMode == 6 && k->verbPreset == 1;
                            if (modeOk && nEx == 4 && tagOk && advOk && keyOk &&
                                noClamp && loopOk)
                                pass("VAB-22 / 2-tag 4-motion preset (curvy/jog) builds; 2D adverbs + keytimes land");
                            else
                                fail("VAB-22 / 2-tag 4-motion preset (curvy/jog) builds; 2D adverbs + keytimes land",
                                     "ok=" + std::to_string(ok) + " nEx=" +
                                         std::to_string(nEx) + " tag=" +
                                         std::to_string(tagOk) + " adv=" +
                                         std::to_string(advOk) + " key=" +
                                         std::to_string(keyOk) + " noClamp=" +
                                         std::to_string(noClamp) + " loop=" +
                                         std::to_string(loopOk));
                        }
                    }

                    // VAB-23 (loop preserves root rotation): a turning blend keeps
                    // turning across loops — heading accumulates per cycle instead
                    // of resetting. A synthetic clip whose root heading advances
                    // ~0.57 rad/cycle → after 3 loops the body heading ≈ 3×0.57,
                    // not 0. (The old code accumulated only position.)
                    {
                        const mograph::Skeleton rs = k->skel();
                        mograph::Clip c;
                        c.dt = 1.0f / 30.0f;
                        c.frames.resize(20);
                        for (int f = 0; f < 20; ++f) {
                            auto& p = c.frames[size_t(f)];
                            p.rot.assign(rs.joints.size(), mograph::Quatf::identity());
                            p.rot[0] = mograph::Quatf::yaw(0.03f * float(f));
                            p.rootPos = {0.0f, 0.9f, 0.04f * float(f)};
                        }
                        mograph::VerbBlend mv;
                        mv.skel = rs;
                        mv.tags = {"t"};
                        mograph::VerbExample e;
                        e.clip = c;
                        e.key = {0, 5, 10, 14, 19};
                        e.adverb = {0};
                        mograph::VerbExample e2 = e;
                        e2.adverb = {100};
                        mv.ex.push_back(e);
                        mv.ex.push_back(e2);  // ready() needs ≥2
                        mv.rebuild();
                        mv.query = {0};
                        auto headAt = [&](double t) {
                            bvh::Pose o;
                            mv.sample(t, o);
                            return o.world.empty()
                                       ? 99.0f
                                       : std::atan2(o.world[0].R[2], o.world[0].R[0]);
                        };
                        const double cyc = mv.cycleSec;
                        const float h0 = headAt(0.0);          // cycle 0 → ≈0
                        const float h3 = headAt(3.0 * cyc);    // cycle 3 → ≈3·dYaw
                        const float dYaw = 0.03f * 19.0f;      // key[4]=19 heading
                        const float expect = 3.0f * dYaw;
                        // Translation must follow the heading (NOT moonwalk): the
                        // forward (+Z) travel curves the SAME way the heading
                        // turns. heading turns +, so forward tilts toward +x → the
                        // looped path's x is positive at cycle 2. A position
                        // rotation opposite the heading (the moonwalk bug) gives
                        // x<0.
                        bvh::Pose p2;
                        mv.sample(2.0 * cyc, p2);
                        const float x2 = p2.world.empty() ? 0.0f : p2.world[0].t[0];
                        if (std::fabs(h0) < 0.05f && std::fabs(h3 - expect) < 0.15f &&
                            x2 > 0.1f)
                            pass("VAB-23 / looped blend preserves root rotation; translation follows heading (no moonwalk)");
                        else
                            fail("VAB-23 / looped blend preserves root rotation; translation follows heading (no moonwalk)",
                                 "h0=" + std::to_string(h0) + " h3=" +
                                     std::to_string(h3) + " expect=" +
                                     std::to_string(expect) + " x2=" +
                                     std::to_string(x2));
                    }

                    k->verbBlend = mograph::VerbBlend{};  // restore for later blocks
                    k->verbKtPreview = -1;
                    k->verbPreset = -1;
                    k->motionMode = 5;
                }
                sim.setKinematicMode(0, 0);
            }
        }
    }

    // ---- Block RIG: rigid mesh stays glued to its Bullet body even when ----
    // the narrow-phase response shoves its verts (kinematic walker overlap).
    {
        resetScene();
        sim.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 20, 50.0f);
        bool haveKin = sim.addKinematicBody(bvhAssetDir() + "/WalkLoopA.bvh",
                                            tinym::vec3(0, 0, 0));
        if (haveKin) {
            // Rigid cube spawned inside the walker's path.
            sim.addCube(tinym::vec3(0.0f, 0.6f, 0.0f), 2, 0.4f, 0.1f,
                        BehaviorType::Rigid);
            sim.initialize();
            MetalGlobalContext::commitAndWait();
            pumpFrames(sim, 60);
            MetalGlobalContext::commitAndWait();
            auto* m = Scene<Backend, Precision>::findById(2);
            tinym::vec3 c{0, 0, 0};
            const Index n = m->state.x.size / 3;
            for (Index i = 0; i < n; ++i) {
                c.x += (float)m->state.x.ptr[i*3+0];
                c.y += (float)m->state.x.ptr[i*3+1];
                c.z += (float)m->state.x.ptr[i*3+2];
            }
            c.x /= (float)n; c.y /= (float)n; c.z /= (float)n;
            const tinym::vec3 body = sim.rigidBodyPositionOf(2);
            const float err = std::fabs(c.x - body.x) + std::fabs(c.y - body.y)
                            + std::fabs(c.z - body.z);
            if (err < 1e-3f)
                pass("RIG-1 / rigid mesh centroid tracks Bullet body under contact response");
            else
                fail("RIG-1 / rigid mesh centroid tracks Bullet body under contact response",
                     "err=" + std::to_string(err));
        } else {
            skip("RIG-1", "WalkLoopA.bvh missing");
        }
    }

    // ---- Block NAN-GUARD: integrator world-bounds guard keeps an --------
    // exploding cloth finite so the frame%10 full BVH rebuild can never
    // see Inf/NaN positions (the GPU-wedge incident: a data-dependent
    // build loop on NaN spins forever and survives kill -9 — reboot only).
    // Scene reproduces the original wedge exactly: default kstretch=1e5
    // cloth explodes under the explicit integrator within one frame.
    //
    // WEDGE-SAFE PROTOCOL: assert finiteness at frame 9 — BEFORE the
    // first frame%10 rebuild at frame 10 — and bail out on failure, so a
    // regression of the guard fails the suite instead of wedging the GPU.
    {
        resetScene();
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 4.0f); // id 0
        sim.addCloth(8, 0.8f, tinym::vec3(0.0f, 0.4f, 0.0f));               // id 1, default 1e5 springs → explodes
        sim.initialize();

        auto allFiniteInBox = [&](double bound) {
            auto* cloth = Scene<Backend, Precision>::findById(1);
            for (Index i = 0; i < cloth->state.x.size; ++i) {
                const double c = (double)cloth->state.x.ptr[i];
                if (!std::isfinite(c) || std::fabs(c) > bound) return false;
            }
            return true;
        };

        // The anomaly halt auto-pauses on the first bad frame, so step
        // with pause forced off each iteration: the explosion then keeps
        // being driven THROUGH the sanitizer and across frame%10 full
        // BVH rebuilds (the wedge path), while haltCount records that
        // the auto-pause actually fired.
        int haltCount = 0;
        auto forceStep = [&](int n) {
            for (int i = 0; i < n; ++i) {
                sim.pause = false;
                sim.update();
                if (sim.pause) ++haltCount;
            }
        };

        forceStep(9);
        MetalGlobalContext::commitAndWait();
        // Box bound 1e4 must match YSIM_WORLD_BOUND in physics.metal; the
        // +1 slack covers the post-clamp collision-response push.
        if (!allFiniteInBox(1.0e4 + 1.0)) {
            fail("NAN-GUARD-1 / exploding cloth finite+boxed at frame 9 (pre-BVH-rebuild gate)",
                 "non-finite or out-of-box vertex at frame 9 — guard regressed; "
                 "skipping further frames to avoid GPU wedge");
        } else if (haltCount == 0) {
            fail("NAN-GUARD-1 / anomaly halt auto-pauses on the exploding cloth",
                 "sim.pause never set across 9 frames of explosion");
        } else {
            pass("NAN-GUARD-1 / exploding cloth finite+boxed at frame 9 and anomaly halt fired");
            // Safe to cross frame%10 rebuilds now. Halt-and-force-unpause
            // alternates (halted iterations don't step), so 231 forced
            // iterations ≈ 115 stepped frames ≈ 11 full BVH rebuilds on
            // the clamped explosion — plenty to prove the wedge is gone.
            forceStep(231);
            MetalGlobalContext::commitAndWait();
            if (allFiniteInBox(1.0e4 + 1.0))
                pass("NAN-GUARD-2 / finite+boxed through 240 forced frames and the frame%10 full BVH rebuilds");
            else
                fail("NAN-GUARD-2 / finite+boxed through 240 forced frames and the frame%10 full BVH rebuilds",
                     "non-finite or out-of-box vertex after 240 frames");
        }
        sim.pause = true;
    }

    // ---- Block PBD: CPU PBD system (Simulator::usePbd) ------------------
    // Same 8x8 cloth with the default kstretch=1e5 springs that the NAN-GUARD
    // block above uses precisely BECAUSE it explodes under the symplectic
    // integrator at these substep counts. PBD projects positions instead of
    // integrating stiff forces, so the identical scene must stay bounded,
    // hold its rest lengths, and not fall through the ground.
    {
        resetScene();
        sim.usePbd = true;
        sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 4.0f); // id 0
        sim.addCloth(8, 0.8f, tinym::vec3(0.0f, 0.4f, 0.0f));               // id 1
        sim.initialize();

        auto clothStats = [&](double& minY, double& meanEdgeErr, bool& finite) {
            auto* cloth = Scene<Backend, Precision>::findById(1);
            minY = 1e30; meanEdgeErr = 0.0; finite = true;
            if (!cloth) { finite = false; return; }
            const Index n = cloth->state.x.size / 3;
            for (Index i = 0; i < n; ++i) {
                for (int c = 0; c < 3; ++c)
                    if (!std::isfinite((double)cloth->state.x.ptr[i * 3 + c])) finite = false;
                minY = std::min(minY, (double)cloth->state.x.ptr[i * 3 + 1]);
            }
            const Index* e = cloth->adjacency.edges.ptr;
            const Precision* rest = cloth->adjacency.restEdgeLengths.ptr;
            const Index numEdges = e ? cloth->adjacency.edges.size / 2 : 0;
            if (!e || !rest || numEdges == 0) return;
            double acc = 0.0;
            for (Index k = 0; k < numEdges; ++k) {
                const Index a = e[k * 2], b = e[k * 2 + 1];
                double d = 0.0;
                for (int c = 0; c < 3; ++c) {
                    const double t = (double)cloth->state.x.ptr[b * 3 + c]
                                   - (double)cloth->state.x.ptr[a * 3 + c];
                    d += t * t;
                }
                const double r = (double)rest[k];
                if (r > 1e-9) acc += std::fabs(std::sqrt(d) - r) / r;
            }
            meanEdgeErr = acc / (double)numEdges;
        };

        double y0 = 0, err0 = 0; bool fin0 = true;
        clothStats(y0, err0, fin0);

        sim.pause = false;
        for (int f = 0; f < 60; ++f) sim.update();
        MetalGlobalContext::commitAndWait();

        double y1 = 0, err1 = 0; bool fin1 = true;
        clothStats(y1, err1, fin1);

        if (!fin1)
            fail("PBD-1 / stiff cloth stays finite under PBD where the symplectic path explodes",
                 "non-finite vertex after 60 frames");
        else
            pass("PBD-1 / stiff cloth stays finite under PBD where the symplectic path explodes");

        // Inextensibility is what PBD buys: mean |len-rest|/rest must stay
        // small. Loose bound (10%) — this is a blow-up detector, not a
        // convergence benchmark.
        if (err1 > 0.10)
            fail("PBD-2 / distance constraints hold rest lengths",
                 "mean edge strain " + std::to_string(err1) + " > 0.10 (was "
                 + std::to_string(err0) + " at rest)");
        else
            pass("PBD-2 / distance constraints hold rest lengths");

        // Contact projection: cloth starts at y=0.4 above a ground plane at
        // y=0, must fall (y decreases) but never sink appreciably below it.
        if (!(y1 < y0 - 1e-4))
            fail("PBD-3 / cloth falls under gravity",
                 "min y did not decrease (" + std::to_string(y0) + " -> "
                 + std::to_string(y1) + ")");
        else if (y1 < -0.05)
            fail("PBD-3 / cloth does not tunnel through the ground",
                 "min y = " + std::to_string(y1) + " (ground at 0)");
        else
            pass("PBD-3 / cloth falls under gravity and rests on the ground");

        // PBD-4: the per-mesh stretch coefficient must actually drive PBD (it
        // used to run on a fixed solver-global stiffness). Same scene, same
        // solver settings, only the mesh's kstretch changed: a 100x softer
        // cloth must stretch measurably more. Fails if springConstantsOf /
        // the k mapping is bypassed.
        auto runAndMeasureStrain = [&](Precision kstretch) -> double {
            resetScene();
            sim.usePbd = true;
            sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 4.0f);
            sim.addCloth(8, 0.8f, tinym::vec3(0.0f, 0.4f, 0.0f),
                         kstretch, kstretch, Precision(3e5));
            sim.initialize();
            sim.pause = false;
            for (int f = 0; f < 60; ++f) sim.update();
            MetalGlobalContext::commitAndWait();
            double y = 0, err = 0; bool fin = true;
            clothStats(y, err, fin);
            sim.pause = true;
            return fin ? err : std::numeric_limits<double>::quiet_NaN();
        };
        const double strainStiff = runAndMeasureStrain(Precision(1e5));
        const double strainSoft  = runAndMeasureStrain(Precision(1e3));
        if (!std::isfinite(strainStiff) || !std::isfinite(strainSoft))
            fail("PBD-4 / per-mesh stretch coefficient drives the PBD solver",
                 "non-finite cloth in one of the two runs");
        else if (!(strainSoft > strainStiff * 1.5))
            fail("PBD-4 / per-mesh stretch coefficient drives the PBD solver",
                 "kstretch 1e3 strain " + std::to_string(strainSoft)
                 + " not meaningfully above kstretch 1e5 strain "
                 + std::to_string(strainStiff)
                 + " — mesh coefficients are being ignored");
        else
            pass("PBD-4 / per-mesh stretch coefficient drives the PBD solver");

        // PBD-5: dihedral bending constraint (Müller 2007 eq 21-29), which
        // replaced the opposite-vertex distance spring.
        using Pbd = PbdSystem<Backend, Precision>;
        const double kPi = std::acos(-1.0);

        // Build the quads from a fresh FLAT cloth, then step and re-measure
        // the same quads against their stored phi0.
        auto runBendDeviation = [&](Precision kbend, size_t* quadCount,
                                    double* maxRestDev) -> double {
            resetScene();
            sim.usePbd = true;
            sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 4.0f);
            sim.addCloth(8, 0.8f, tinym::vec3(0.0f, 0.4f, 0.0f),
                         Precision(1e5), Precision(1e5), kbend);
            sim.initialize();
            auto* cloth = Scene<Backend, Precision>::findById(1);
            if (!cloth) return std::numeric_limits<double>::quiet_NaN();
            std::vector<Pbd::BendQuad> quads;
            Pbd::buildBendQuads(*cloth, quads);
            if (quadCount) *quadCount = quads.size();
            if (maxRestDev) {
                double mx = 0.0;
                for (const auto& q : quads)
                    mx = std::max(mx, std::fabs((double)q.phi0 - kPi));
                *maxRestDev = mx;
            }
            sim.pause = false;
            for (int f = 0; f < 60; ++f) sim.update();
            MetalGlobalContext::commitAndWait();
            sim.pause = true;
            cloth = Scene<Backend, Precision>::findById(1);
            if (!cloth) return std::numeric_limits<double>::quiet_NaN();
            double acc = 0.0; int cnt = 0;
            for (const auto& q : quads) {
                Pbd::Vec3 n1, n2; Precision l23, l24, d;
                if (!Pbd::dihedral(cloth->state.x.ptr, q.p1, q.p2, q.p3, q.p4,
                                   n1, n2, l23, l24, d)) continue;
                acc += std::fabs(std::acos((double)d) - (double)q.phi0);
                ++cnt;
            }
            return cnt ? acc / (double)cnt : std::numeric_limits<double>::quiet_NaN();
        };

        size_t quadCount = 0;
        double maxRestDev = 0.0;
        const double devStiff = runBendDeviation(Precision(1e6), &quadCount, &maxRestDev);
        const double devSoft  = runBendDeviation(Precision(1e4), nullptr, nullptr);

        // (a) The two normals are DELIBERATELY built with opposite
        // orientation (n1 from p2xp3, n2 from p2xp4), so a flat rest sheet
        // must give d = -1, i.e. phi0 = pi. A quad ordering that "fixed" the
        // winding to make the normals agree would land at 0 instead and the
        // gradients (21-28) would no longer match the constraint.
        if (quadCount == 0)
            fail("PBD-5 / dihedral bend quads built from facet adjacency",
                 "no interior-edge quads found on an 8x8 cloth");
        else if (maxRestDev > 0.05)
            fail("PBD-5 / flat rest sheet has phi0 == pi (opposite-normal convention)",
                 "max |phi0 - pi| = " + std::to_string(maxRestDev)
                 + " rad over " + std::to_string(quadCount) + " quads");
        else
            pass("PBD-5 / dihedral bend quads built, flat rest sheet gives phi0 == pi");

        // (b) The constraint must actually resist bending: a 100x stiffer
        // kbend has to hold the sheet closer to its rest dihedral angle.
        if (!std::isfinite(devStiff) || !std::isfinite(devSoft))
            fail("PBD-6 / bend stiffness resists dihedral deviation",
                 "non-finite dihedral measurement");
        else if (!(devStiff < devSoft))
            fail("PBD-6 / bend stiffness resists dihedral deviation",
                 "kbend 1e6 deviation " + std::to_string(devStiff)
                 + " not below kbend 1e4 deviation " + std::to_string(devSoft));
        else
            pass("PBD-6 / bend stiffness resists dihedral deviation");

        sim.pause = true;
        sim.usePbd = false;
    }

    if (failures == 0) {
        std::cerr << "[self-test] all checks passed\n";
        return 0;
    }
    std::cerr << "[self-test] " << failures << " failure(s)\n";
    return 1;
}
