#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

struct Simulator {
    System& system;

    Scene<BE, PR> scene;

    //using BroadPhase = BVH<BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE, PR>;
    using BroadPhase = BVH<BE, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT>;
    using NarrowPhase = BruteForce<METAL, PR>;
    CollisionPipeline<BroadPhase, NarrowPhase> collisionPipeline;

    // Spatial-hashing broadphase. Runs alongside the BVH so the active broad
    // phase can be flipped at runtime (Step 10 A/B compare).
    //   useSpatialHashing == false -> BVH path (default)
    //   useSpatialHashing == true  -> SH path
    // Click-ray pickup and showBox/showSceneBox always go through the BVH —
    // SpatialHashing is broadphase-only.
    SpatialHashing<METAL, PR> shBroadPhase;
    bool useSpatialHashing = false;

    // Multi-level (hgrid) spatial-hash broadphase. Sibling to shBroadPhase;
    // selected when useMultiLevelSH == true (takes priority over the
    // single-level path). Oversized primitives (the floor) are excluded by an
    // AABB-diagonal threshold inside the struct. BVH stays alive for picking.
    MultiLevelSpatialHashing<METAL, PR> mlBroadPhase;
    bool useMultiLevelSH = false;

    // BVH path A/B toggle (only meaningful when useSpatialHashing == false).
    //   false (default) -> baseline queryPoints (per-leaf-hit global atomicAdd)
    //   true            -> queryPointsSegmented (per-TG private + reduce)
    // Both paths share the same broad-phase outer loop and feed the same
    // packedCollisionData.broadCollisions/numBroadCollisions, so the narrow
    // phase consumes identical output regardless of toggle. Kept here so a
    // hotkey can flip it at runtime for side-by-side comparison without a
    // re-pack (PSOs + scratch buffers are allocated unconditionally).
    bool useSegmentedBVHQuery = false;

    // Slice (c) A/B toggle. false (default) = ORIGINAL pipeline:
    // spheres collide via the triangle-soup narrow_pt_tri exactly as
    // before c-1 (narrow_pt_analytic not dispatched, no sphere gate).
    // true = analytic sphere path. Flipped at runtime with the 'A'
    // key for side-by-side comparison.
    bool useAnalyticPrimitive = false;

    // Substep-cadence knobs for the BVH collision path. 1 == today's
    // behavior (refit + broad + narrow every substep). i % period == 0
    // fires at i==0 for ANY period, so even period == subSteps runs once
    // per frame (substep 0). The two are independent:
    //   refitSubstepPeriod — how often the BVH leaf AABBs are refit.
    //   cdSubstepPeriod    — how often the broad phase RE-TRAVERSES to
    //                        refresh the candidate vertex-triangle pair set.
    // Between broad detections the pair set is HELD; the narrow phase still
    // runs EVERY substep on that held set, recomputing penetration depth
    // from current positions so the contact response stays stable ("keep
    // previous info" = keep the PAIRS, recompute the geometry — naive
    // whole-contact reuse explodes in the fixed-push integrator). Used by
    // --bench-cadence to sweep the cost/penetration tradeoff.
    Index refitSubstepPeriod = 1;  // refit (+ enlargeTrajectory) every N substeps
    Index cdSubstepPeriod    = 1;  // broad re-traversal (pair refresh) every N substeps

    // Per-substep stdout log for the SH path. Toggling this on also enables
    // shBroadPhase.verbose (forces a commit after the broad-phase dispatch so
    // the broad-phase ms is faithful). Useful when the GUI window can't keep
    // up because the substep loop itself is slow.
    bool logSHPerSubstep = false;



    // sim viewer?
    bool pause = true;
    bool checkCollision = true;
    bool enableSelfCollisions = false;
    Index frame = 0;
    // 진행 바가 시뮬레이션할 목표 프레임 수. frame이 여기에 도달하면
    // update()가 pause를 켜고, 그 상태에서 다시 재생하면 frame 0부터
    // 다시 시작한다 (reset == frame 0). ImGui::InputInt 바인딩용 int.
    int targetFrames = 300;
    profiler::FrameProfiler* profiler = nullptr;

    // Profiling/sync tier (update() sync refactor). InFrame = per-section
    // commitAndWait (historical default); PerFrame/None run the sim loop
    // fully async (only the frame-boundary sync). syncEachPhase() gates the
    // 3 per-substep phase syncs (refit/detect/narrow) on InFrame.
    sim_config::ProfileLevel profileLevel = sim_config::ProfileLevel::InFrame;
    bool syncEachPhase() const {
        return profileLevel == sim_config::ProfileLevel::InFrame;
    }
    // Experiment: in PerFrame mode, commit (NO wait) after each substep so the GPU
    // pipelines substeps instead of running one giant per-frame command buffer.
    // The frame-boundary commitAndWait still provides the single sync. Default OFF
    // ⇒ PerFrame behaves as before (one buffer/frame).
    bool perFrameSubstepCommit = false;


    PR margin = 0.015;
    PR radius = 0.012;

    // object select
    int selectedObj = -1;
    // Master cluster-mode switch (the profiler window's "Cluster mode"
    // checkbox aliases this). When on, the two-mesh cluster-VF broad phase
    // runs and the inspector exposes the per-object Sub-object BVH panel.
    // Lives on the Simulator (NOT BroadPhase) so it survives the '0' reset,
    // which reconstructs BroadPhase from scratch; applyClusterMode()
    // re-imprints it onto the fresh BroadPhase after every rebuild.
    bool clusterModeOn = false;
    // Camera-follow target: when >= 0, the render loop drives the viewport
    // orbit pivot (camera.look) to this mesh's live animated root every
    // frame. Toggled per kinematic body in the inspector. Cleared to -1
    // when the toggle is turned off or the followed mesh disappears.
    int cameraFollowObjId = -1;
    // Hover state, written by the cursor callback after sampling the id
    // FBO (glReadPixels into the R32I color attachment). -1 means
    // "cursor outside any mesh" — outline pass treats negative ids as
    // no-match. Click selection still uses the BVH ray cast at
    // mouseButtonCallback time; hover is purely visual.
    int hoveredObj = -1;

    // Active picking mode + per-vertex hover/select state (point mode).
    // hovered*/selected*Vert are render-vertex indices; *VertObj is the
    // owning compacted mesh id. -1 = none. Written by the cursor / mouse
    // callbacks after sampling the point id pass (.r = obj, .b = vert).
    SelectionMode selectionMode = SelectionMode::Object;
    int hoveredVert = -1, hoveredVertObj = -1;
    int selectedVert = -1, selectedVertObj = -1;
    // Point panel "reference another point" mode: when true, the next
    // viewport vertex click is consumed as a position source — its
    // world pos is copied into the selected vertex (instead of changing
    // the selection). Auto-cancelled on leaving Point mode (req 3).
    bool pointRefPickActive = false;

    // Renderer-side per-mesh GL state (D-011). D-042 R-2: MeshGL now binds
    // to PreviewState heap pointers published via registerPreviewBinding at
    // addX time; the prior clear()-on-initialize band-aid retires because
    // preview buffers are stable across Scene::pack reallocations.
    MeshRenderState renderState;

    // Motion-preview ghost pass: one reusable scratch proxy-mesh GL binding,
    // re-uploaded per strobe frame. Sized to the previewed body's proxy and
    // rebuilt only when that size changes (body/skeleton swap).
    std::vector<float> ghostVerts_, ghostNormals_;
    std::vector<unsigned int> ghostFacets_;
    MeshGL<CPU> ghostGL_;
    bool ghostReady_ = false;
    size_t ghostNV_ = 0, ghostNF_ = 0;

    void ensureGhostGL(const kinematic::BodyProxy& proxy) {
        const size_t nv = size_t(proxy.numVerts), nf = size_t(proxy.numFacets());
        if (ghostReady_ && ghostNV_ == nv && ghostNF_ == nf) return;
        ghostVerts_.assign(nv * 3, 0.0f);
        ghostNormals_.assign(nv * 3, 0.0f);
        ghostFacets_.resize(nf * 3);
        for (size_t i = 0; i < nf * 3; ++i)
            ghostFacets_[i] = (unsigned int)proxy.facets[i];
        // Leaks the prior VAO/buffers on a size change (MeshGL has no dtor);
        // bounded — only fires on a body/skeleton swap, never per frame.
        ghostGL_ = MeshGL<CPU>(nv, ghostVerts_.data(), nf, ghostFacets_.data(),
                               ghostNormals_.data());
        ghostNV_ = nv;
        ghostNF_ = nf;
        ghostReady_ = true;
    }

    // Load + sample the selected preview files into the body's per-clip cache
    // (only when stale). Sampled onto the body's own skeleton so the proxy
    // matches; a joint-count mismatch leaves the cache empty (preview skipped).
    void ensurePreviewClips(MeshKinematicInitializer<BE, PR>* kin) {
        auto load = [&](const std::string& file, std::string& cached,
                        mograph::Clip& clip, bool loopExt) {
            if (file.empty()) return;
            if (cached == file && !clip.frames.empty()) return;  // hit
            cached = file;
            clip.frames.clear();
            bvh::Motion m = bvh::load(bvhAssetDir() + "/" + file, nullptr);
            if (!m.valid() || m.joints.size() != kin->motion.joints.size())
                return;  // unreadable / incompatible with this proxy
            mograph::sampleClip(m, kin->skel(), kin->motion.frameTime, file, clip);
            // 루프 연장: the preview/range clip mirrors the doubled build clip,
            // so the frame-range slider + strobe span both loops.
            if (loopExt) mograph::doubleClipFrames(clip);
        };
        const int n = kin->numSlots();
        for (int i = 0; i < n && i < int(kin->motionSlots.size()); ++i) {
            auto& s = kin->motionSlots[i];
            if (!s.preview && kin->previewPlay != i + 1) continue;  // only needed
            load(kin->slotFile(i), s.cachedFile, s.cachedClip, s.loopSel != 0);
        }
    }

    // One-shot preview playback clock. Runs OUTSIDE the physics loop (driven by
    // the render frame's wall dt) and only while paused — unpausing or leaving
    // blend mode cancels it; reaching the clip end clears it (so it plays once
    // and disappears).
    void advancePreviewPlayback(double dt) {
        for (auto& mesh : scene.meshes) {
            auto* kin =
                dynamic_cast<MeshKinematicInitializer<BE, PR>*>(mesh.initializer);
            if (!kin || kin->previewPlay == 0) continue;
            if (!pause) { kin->previewPlay = 0; continue; }
            if (kin->previewPlay < 0) {  // -1 = blended one-shot (blend space OR 2-blend)
                const bool blendOk = kin->motionMode == 4 && kin->graphActive();
                const bool verbOk = kin->verbActive();
                if (!blendOk && !verbOk) {
                    kin->previewPlay = 0;
                    continue;
                }
                const double cycle = verbOk ? kin->verbBlend.cycleSec
                                            : kin->graphSession.blendCycleSec;
                kin->previewPlayTime += dt;
                if (cycle <= 1e-6 || kin->previewPlayTime >= cycle)
                    kin->previewPlay = 0;  // played one cycle → vanish
                continue;
            }
            const int si = kin->previewPlay - 1;
            if (si >= kin->numSlots() || si >= int(kin->motionSlots.size())) {
                kin->previewPlay = 0;
                continue;
            }
            const mograph::Clip& c = kin->motionSlots[si].cachedClip;
            const auto rg = kin->slotRange(si);  // active window only
            // max(span,1): a 1-frame window still plays one frame (dur>0) instead
            // of vanishing before it renders. span>=1 is bit-identical.
            const double dur =
                c.frames.empty() ? 0.0 : double(std::max(rg[1] - rg[0], 1)) * c.dt;
            kin->previewPlayTime += dt;
            if (c.frames.empty() || kin->previewPlayTime >= dur)
                kin->previewPlay = 0;  // played the window once → vanish
        }
    }

    // Begin a one-shot opaque playback of the clip in slot `slotIdx` (0-based).
    // No-op unless paused (the feature is paused-only by design).
    void startPreviewPlayback(int meshId, int slotIdx) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !pause) return;
        if (slotIdx < 0 || slotIdx >= kin->numSlots() ||
            slotIdx >= int(kin->motionSlots.size()))
            return;
        ensurePreviewClips(kin);
        if (kin->motionSlots[slotIdx].cachedClip.frames.empty()) return;
        kin->previewPlay = slotIdx + 1;
        kin->previewPlayTime = 0.0;
    }

    // Begin a one-shot opaque playback of the BLENDED result through one gait
    // cycle (sampled live at the cursor, so dragging the pad morphs it as it
    // plays). Paused-only; blend-space mode only.
    void startBlendPlayback(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !pause || kin->motionMode != 4 || !kin->graphActive()) return;
        kin->previewPlay = -1;
        kin->previewPlayTime = 0.0;
    }

    // Begin a one-shot opaque playback of the 2-motion blended result through
    // one gait cycle (sampled live at the adverb query). Paused-only; mode 5.
    void startVerbPlayback(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !pause || !kin->verbActive()) return;
        kin->previewPlay = -1;
        kin->previewPlayTime = 0.0;
    }

    // Stop + remove the in-progress one-shot preview (slot or blend).
    void stopPreviewPlayback(int meshId) {
        if (auto* kin = kinematicOf(meshId)) {
            kin->previewPlay = 0;
            kin->previewPlayTime = 0.0;
        }
    }

    // Translucent strobe of the selected blend clips (motion 1 + motion 2,
    // independently toggled, user-colored), N evenly-spaced frames each — plus
    // the opaque one-shot playback when active. All re-rooted to the body's
    // transform. Drawn after the opaque pass.
    void drawGhostPreviews(Program& shader) {
        for (auto& mesh : scene.meshes) {
            auto* kin =
                dynamic_cast<MeshKinematicInitializer<BE, PR>*>(mesh.initializer);
            if (!kin) continue;
            const int n = kin->numSlots();
            if (n <= 0) continue;          // slot-based modes only (transition/DTW/blend)
            bool wantStrobe = false;
            for (int i = 0; i < n && i < int(kin->motionSlots.size()); ++i)
                if (kin->motionSlots[i].preview) wantStrobe = true;
            const bool wantPlay = kin->previewPlay != 0;
            const bool wantBlend = kin->blendPreview && kin->motionMode == 4 &&
                                   kin->graphActive();
            const bool wantVerb = kin->verbPreview && kin->verbActive();
            const bool wantKt = kin->verbActive() && kin->verbKtPreview >= 0 &&
                                kin->verbKtPreview < int(kin->verbBlend.ex.size());
            if (!wantStrobe && !wantPlay && !wantBlend && !wantVerb && !wantKt)
                continue;
            ensurePreviewClips(kin);
            ensureGhostGL(kin->proxy);

            shader.setUniform("checkerOn", 0);
            shader.setUniform("hoveredId", -1);   // keep outline off the ghosts
            shader.setUniform("selectedId", -1);

            // Strobe pass (translucent). The 동선 stays visible even while a
            // clip plays opaque on top (existing trail + moving preview).
            if (wantStrobe) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                shader.setUniform("opacity", 0.15f);
                const int N = 7;
                // Strobe spans ONLY the slot's active window [fs, fe]; the
                // range-start frame fs anchors the body (writeGhost anchorFrame).
                auto strobe = [&](const mograph::Clip& c,
                                  const std::array<float, 3>& col, int fs, int fe) {
                    const int nf = int(c.frames.size());
                    if (nf <= 0) return;
                    const tinym::vec3 base(col[0], col[1], col[2]);
                    const tinym::vec3 emis(col[0] * 0.5f, col[1] * 0.5f,
                                           col[2] * 0.5f);
                    const int span = fe - fs;
                    for (int i = 0; i < N; ++i) {
                        const int f = N <= 1 || span <= 0
                                          ? fs
                                          : fs + int(std::lround(double(i) /
                                                                 (N - 1) * span));
                        kin->writeGhost(c, f, mesh.scale, mesh.rotationQuat,
                                        mesh.transformPosition, ghostVerts_, fs);
                        ghostGL_.computeNormal();
                        ghostGL_.updateBuffer();
                        ghostGL_.draw(shader, base, 0.0f, 1.0f, 0.0f, emis);
                    }
                };
                for (int i = 0; i < n && i < int(kin->motionSlots.size()); ++i)
                    if (kin->motionSlots[i].preview) {
                        const auto rg = kin->slotRange(i);
                        strobe(kin->motionSlots[i].cachedClip,
                               kin->motionSlots[i].color, rg[0], rg[1]);
                    }
            }

            // One-shot opaque playback (full color, depth-writing). previewPlay
            // < 0 is the blended result (sampled live at the cursor through one
            // cycle); > 0 is a single clip slot.
            if (wantPlay && kin->previewPlay < 0) {
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
                shader.setUniform("opacity", 1.0f);
                const bool isVerb = kin->verbActive();  // mode 5 vs blend space
                std::vector<float> w;
                double cycle;
                if (isVerb) {
                    kin->verbBlend.weights(kin->verbBlend.query, w);
                    cycle = kin->verbBlend.cycleSec;
                } else {
                    kin->graphSession.blendWeights(kin->graphSession.cursor, w);
                    cycle = kin->graphSession.blendCycleSec;
                }
                tinym::vec3 col(0, 0, 0);
                float sw = 0.0f;
                for (int i = 0; i < int(w.size()) &&
                                i < int(kin->motionSlots.size()); ++i) {
                    const auto& cc = kin->motionSlots[i].color;
                    col.x += cc[0] * w[i]; col.y += cc[1] * w[i]; col.z += cc[2] * w[i];
                    sw += w[i];
                }
                if (sw > 1e-6f) { col.x /= sw; col.y /= sw; col.z /= sw; }
                else col = tinym::vec3(0.85f, 0.85f, 0.9f);
                const tinym::vec3 emis(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f);
                const double phase = cycle > 1e-6 ? kin->previewPlayTime / cycle : 0.0;
                if (isVerb)
                    kin->writeVerbGhost(phase, mesh.scale, mesh.rotationQuat,
                                        mesh.transformPosition, ghostVerts_);
                else
                    kin->writeBlendGhost(phase, /*loop=*/false, mesh.scale,
                                         mesh.rotationQuat, mesh.transformPosition,
                                         ghostVerts_);
                if (!ghostVerts_.empty()) {
                    ghostGL_.computeNormal();
                    ghostGL_.updateBuffer();
                    ghostGL_.draw(shader, col, 0.0f, 1.0f, 0.0f, emis);
                }
            } else if (wantPlay &&
                       kin->previewPlay - 1 < int(kin->motionSlots.size())) {
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
                shader.setUniform("opacity", 1.0f);
                const int si = kin->previewPlay - 1;
                const auto& slot = kin->motionSlots[si];
                const mograph::Clip& c = slot.cachedClip;
                const auto& col = slot.color;
                const tinym::vec3 base(col[0], col[1], col[2]);
                const tinym::vec3 emis(col[0] * 0.5f, col[1] * 0.5f, col[2] * 0.5f);
                // Play inside the active window: clock starts at the range-start
                // frame, which also anchors the body (anchorFrame = rg[0]).
                const auto rg = kin->slotRange(si);
                const double tOff = double(rg[0]) * (c.dt > 0 ? c.dt : 0.0);
                kin->writeGhostAtTime(c, tOff + kin->previewPlayTime, mesh.scale,
                                      mesh.rotationQuat, mesh.transformPosition,
                                      ghostVerts_, rg[0]);
                ghostGL_.computeNormal();
                ghostGL_.updateBuffer();
                ghostGL_.draw(shader, base, 0.0f, 1.0f, 0.0f, emis);
            }

            // Blend-result preview: translucent strobe of the live blended gait
            // cycle. Color = slot colors mixed by the current weights, and the
            // poses come from samplePose at the live cursor — so dragging the
            // pad re-tints AND re-poses every ghost in real time.
            if (wantBlend) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                shader.setUniform("opacity", 0.22f);
                std::vector<float> w;
                kin->graphSession.blendWeights(kin->graphSession.cursor, w);
                tinym::vec3 col(0, 0, 0);
                float sw = 0.0f;
                for (int i = 0; i < int(w.size()) &&
                                i < int(kin->motionSlots.size()); ++i) {
                    const auto& c = kin->motionSlots[i].color;
                    col.x += c[0] * w[i]; col.y += c[1] * w[i]; col.z += c[2] * w[i];
                    sw += w[i];
                }
                if (sw > 1e-6f) { col.x /= sw; col.y /= sw; col.z /= sw; }
                else col = tinym::vec3(0.85f, 0.85f, 0.9f);
                const tinym::vec3 emis(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f);
                const int N = 7;
                for (int i = 0; i < N; ++i) {
                    kin->writeBlendGhost(double(i) / N, /*loop=*/true, mesh.scale,
                                         mesh.rotationQuat, mesh.transformPosition,
                                         ghostVerts_);
                    if (ghostVerts_.empty()) continue;
                    ghostGL_.computeNormal();
                    ghostGL_.updateBuffer();
                    ghostGL_.draw(shader, col, 0.0f, 1.0f, 0.0f, emis);
                }
            }

            // 2-motion keytime blend preview (mode 5): translucent strobe of the
            // live blended cycle. Color = slot colors mixed by the adverb
            // weights; poses come from verbBlend at the live query — so moving
            // the slider re-tints AND re-poses every ghost in real time.
            if (wantVerb) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                shader.setUniform("opacity", 0.22f);
                std::vector<float> w;
                kin->verbBlend.weights(kin->verbBlend.query, w);
                tinym::vec3 col(0, 0, 0);
                float sw = 0.0f;
                for (int i = 0; i < int(w.size()) &&
                                i < int(kin->motionSlots.size()); ++i) {
                    const auto& c = kin->motionSlots[i].color;
                    col.x += c[0] * w[i]; col.y += c[1] * w[i]; col.z += c[2] * w[i];
                    sw += w[i];
                }
                if (sw > 1e-6f) { col.x /= sw; col.y /= sw; col.z /= sw; }
                else col = tinym::vec3(0.85f, 0.85f, 0.9f);
                const tinym::vec3 emis(col.x * 0.5f, col.y * 0.5f, col.z * 0.5f);
                const int N = 7;
                for (int i = 0; i < N; ++i) {
                    kin->writeVerbGhost(double(i) / N, mesh.scale,
                                        mesh.rotationQuat, mesh.transformPosition,
                                        ghostVerts_);
                    if (ghostVerts_.empty()) continue;
                    ghostGL_.computeNormal();
                    ghostGL_.updateBuffer();
                    ghostGL_.draw(shader, col, 0.0f, 1.0f, 0.0f, emis);
                }
            }

            // Keytime filmstrip (mode 5/6 tuning): static ghosts of ONE motion's
            // clip at its 5 keytime frames, spread along X. Translucent; each
            // re-poses live as that keytime's slider is dragged so you set timing
            // by the pose. Anchored to each frame itself so the strip rows align.
            if (wantKt) {
                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                glDepthMask(GL_FALSE);
                shader.setUniform("opacity", 0.35f);
                const int m = kin->verbKtPreview;
                const auto& ke = kin->verbBlend.ex[m];
                std::array<float, 3> col{0.8f, 0.8f, 0.85f};
                if (m < int(kin->motionSlots.size())) col = kin->motionSlots[m].color;
                const tinym::vec3 base(col[0], col[1], col[2]);
                const tinym::vec3 emis(col[0] * 0.5f, col[1] * 0.5f, col[2] * 0.5f);
                const float dx = 1.3f * std::max(0.2f, float(mesh.scale.x));
                for (int k = 0; k < 5; ++k) {
                    tinym::vec3 pos = mesh.transformPosition;
                    pos.x += float(k - 2) * dx;  // centered 5-pose strip
                    kin->writeGhost(ke.clip, ke.key[k], mesh.scale,
                                    mesh.rotationQuat, pos, ghostVerts_, ke.key[k]);
                    if (ghostVerts_.empty()) continue;
                    ghostGL_.computeNormal();
                    ghostGL_.updateBuffer();
                    ghostGL_.draw(shader, base, 0.0f, 1.0f, 0.0f, emis);
                }
            }

            shader.setUniform("opacity", 1.0f);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
        }
    }

    // D-040: rigid physics backend swapped Euler → Bullet 3.25 (B-3 BLOCK
    // fix-turn). Bullet now ships as a third parallel symbol (Null +
    // Euler + Bullet); the type-swap is one line. RIGID-BACKEND-PORTABILITY
    // (D-037) governs the contract. The fix-turn is what closes Estimator
    // turn-35 BLOCK on BDD-008's "box rests at floor" — Bullet provides
    // real box-vs-plane contact resolution which the Euler backend lacked.
    ysim::physics::BulletRigidPhysicsBackend rigid_;

    Simulator(System& system) : system(system) {}

    // D-039: bounding-sphere radius heuristic for ensureRigidBackendBody.
    // Returns 0.5 × max bbox extent across state.x. Exact for Cube; coarse
    // approximation for arbitrary meshes (Euler backend ground clamp is
    // sphere-only anyway). Zero-vertex falls back to 0.5.
    PR inferRigidRadius(const GeneralMesh<BE, PR>& mesh) const {
        if (mesh.state.x.size == 0 || mesh.state.x.ptr == nullptr) return PR(0.5);
        const PR* px = mesh.state.x.ptr;
        const Index n = mesh.state.x.size / 3;
        PR xmin = px[0], xmax = px[0];
        PR ymin = px[1], ymax = px[1];
        PR zmin = px[2], zmax = px[2];
        for (Index i = 1; i < n; ++i) {
            const PR vx = px[i*3+0];
            const PR vy = px[i*3+1];
            const PR vz = px[i*3+2];
            if (vx < xmin) xmin = vx; if (vx > xmax) xmax = vx;
            if (vy < ymin) ymin = vy; if (vy > ymax) ymax = vy;
            if (vz < zmin) zmin = vz; if (vz > zmax) zmax = vz;
        }
        const PR dx = xmax - xmin, dy = ymax - ymin, dz = zmax - zmin;
        PR d = dx; if (dy > d) d = dy; if (dz > d) d = dz;
        return PR(0.5) * d;
    }

    // D-040: shape inference. Cube primitives use Box (real box-vs-plane
    // contact in Bullet — closes BDD-008's "rests at floor" clause).
    // Everything else falls back to Sphere with bbox-half radius.
    ysim::physics::RigidShapeType inferRigidShapeType(
        const GeneralMesh<BE, PR>& mesh) const {
        if (dynamic_cast<MeshCubeInitializer<BE, PR>*>(mesh.initializer)) {
            return ysim::physics::RigidShapeType::Box;
        }
        return ysim::physics::RigidShapeType::Sphere;
    }

    // D-040 addendum (2026-05-14, "scene-objects-only" fix): register a
    // Float-tagged grid mesh as a static Bullet collision body so Rigid
    // bodies rest on the user's actual ground instead of a virtual y=0
    // plane. PlaneDirection → normal mapping mirrors MeshGridInitializer's
    // own grid-orientation logic. The mesh's transformPosition seeds the
    // plane's signed-distance-from-origin via dot(normal, center). Only
    // grids are handled — non-grid Float meshes (e.g., imported .obj
    // FloatMesh) are NOT auto-collidable; user can tag them Rigid or a
    // future slice adds StaticMesh triangle-soup support.
    void ensureRigidStaticGround(int meshId) {
        if (meshId < 0 || meshId >= (int)Scene<BE, PR>::meshes.size()) return;
        auto& mesh = Scene<BE, PR>::meshes[meshId];
        if (mesh.behaviorType != BehaviorType::Float) return;
        auto* g = dynamic_cast<MeshGridInitializer<BE, PR>*>(mesh.initializer);
        if (!g) return;

        tinym::vec3 normal;
        switch (g->params.dir) {
            case PlaneDirection::XZPlane: normal = tinym::vec3(0.0f, 1.0f, 0.0f); break;
            case PlaneDirection::XYPlane: normal = tinym::vec3(0.0f, 0.0f, 1.0f); break;
            case PlaneDirection::YZPlane: normal = tinym::vec3(1.0f, 0.0f, 0.0f); break;
        }
        const tinym::vec3 c = mesh.transformPosition;
        const float distance = normal.x * (float)c.x + normal.y * (float)c.y + normal.z * (float)c.z;

        ysim::physics::RigidInitial planeInit{};
        planeInit.mass = 0.0f;  // static body — no integration, infinite mass
        planeInit.shape.type = ysim::physics::RigidShapeType::Plane;
        planeInit.shape.normal = normal;
        planeInit.shape.half_extents = tinym::vec3(0.0f, distance, 0.0f);
        rigid_.addBody(planeInit);
    }

    // D-039 / D-040: idempotent rigid-body provisioning. Adds a backend
    // body for the given mesh IFF it's Rigid-tagged and lacks a handle.
    // Called by changeBehavior(Rigid) and by initialize()'s post-pack
    // sweep. CM-012: no exit/abort; silent early-return on guards.
    void ensureRigidBackendBody(int meshId) {
        if (meshId < 0 || meshId >= (int)Scene<BE, PR>::meshes.size()) return;
        auto& mesh = Scene<BE, PR>::meshes[meshId];
        if (mesh.behaviorType != BehaviorType::Rigid) return;
        if (mesh.rigidBodyHandle != ysim::physics::kInvalidBodyHandle) return;

        ysim::physics::RigidInitial init{};
        // Spawn the body at the mesh's VERTEX CENTROID, not transformPosition.
        // The per-step rigid sync (see update()) snaps the centroid onto the
        // body origin; transformPosition == the initializer offset, which
        // equals the centroid only for origin-authored shapes (sphere/cube).
        // A file OBJ whose local centroid sits off-origin (camel: +0.30y)
        // would otherwise jerk by -(localCentroid*scale) on the first step.
        // Falls back to transformPosition when verts aren't packed yet.
        tinym::vec3 spawn = mesh.transformPosition;
        const Index nv = mesh.state.x.size / 3;
        if (nv > 0 && mesh.state.x.ptr) {
            tinym::vec3 c{0, 0, 0};
            const PR* xp = mesh.state.x.ptr;
            for (Index vi = 0; vi < nv; ++vi) {
                c.x += (float)xp[vi*3+0];
                c.y += (float)xp[vi*3+1];
                c.z += (float)xp[vi*3+2];
            }
            spawn = tinym::vec3(c.x / nv, c.y / nv, c.z / nv);
        }
        init.position = spawn;
        init.rotation = mesh.rotationQuat;
        // Apply Gravity=false → Bullet static body (mass=0): it never
        // falls, never receives impulses, but still serves as a
        // collider that dynamic bodies bounce off — i.e. "Float-like"
        // per user request. Apply Gravity=true → dynamic (mass=1).
        init.mass     = mesh.applyGravity ? 1.0f : 0.0f;
        init.shape.type = inferRigidShapeType(mesh);
        const float halfExtent = (float)inferRigidRadius(mesh);
        if (init.shape.type == ysim::physics::RigidShapeType::Box) {
            init.shape.half_extents = tinym::vec3(halfExtent, halfExtent, halfExtent);
        } else {
            init.shape.half_extents = tinym::vec3(halfExtent, 0.0f, 0.0f);
        }

        mesh.rigidBodyHandle = rigid_.addBody(init);
        mesh.rigidLastBodyPos = init.position;
    }

    // Drop the existing Bullet body for a mesh and re-provision it from
    // scratch. Used when a state change (e.g. Apply Gravity toggle on a
    // Rigid mesh) flips the required Bullet mass between dynamic and
    // static — Bullet does not support changing a body's mass cleanly
    // in place, so we recycle the slot.
    void recreateRigidBackendBody(int meshIdx) {
        if (meshIdx < 0 || meshIdx >= (int)Scene<BE, PR>::meshes.size()) return;
        auto& mesh = Scene<BE, PR>::meshes[meshIdx];
        if (mesh.rigidBodyHandle != ysim::physics::kInvalidBodyHandle) {
            rigid_.removeBody(mesh.rigidBodyHandle);
            mesh.rigidBodyHandle = ysim::physics::kInvalidBodyHandle;
        }
        ensureRigidBackendBody(meshIdx);
    }

    // D-042 R-2: after every scene.addGeneralMesh, publish the R-1 preview
    // pointers to MeshRenderState so MeshGL materializes from the stable
    // heap-owned PreviewState buffers (not the volatile packed sub-views).
    // CPU-only — safe to call from the harness without a GL context.
    //
    // 2026-05-15 (A2 split): when the initializer populated render-side
    // buffers (currently cube only), bind MeshGL to those instead — gives
    // flat per-face shading without averaging across welded seams.
    // Sphere/grid/file initializers leave render buffers empty so the
    // render*Ptr/numRender* accessors fall back to x/n/facets transparently.
    void registerPreviewBindingFor(
            typename Scene<BE, PR>::RequestGeneralMesh& req) {
        renderState.registerPreviewBinding(req.id,
            req.preview.renderXPtr(), req.preview.numRenderPoints(),
            req.preview.renderFacetsPtr(), req.preview.numRenderFacets(),
            req.preview.renderNPtr());
    }

    void registerPreviewBindingForLastRequest() {
        if (Scene<BE, PR>::requestsGeneralMeshes.empty()) return;
        registerPreviewBindingFor(Scene<BE, PR>::requestsGeneralMeshes.back());
    }

    void addClothFile(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.001, PR mass=0.1) {
        scene.addGeneralMesh(
                new MeshFileInitializer<BE, PR>({prefix, fileName, offset, scale, mass}),
                BehaviorType::TriangularCloth,
                ClothBehaviorParams<PR>{kstretch, kshear, kbend, thickness}
                );
        registerPreviewBindingForLastRequest();
    };

    void addFloatMesh(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR mass=0.1) {
        scene.addGeneralMesh(
                new MeshFileInitializer<BE, PR>({prefix, fileName, offset, scale, mass}),
                BehaviorType::Float,
                FloatBehaviorParams<PR>{}
                );
        registerPreviewBindingForLastRequest();
    }

    // BDD-002 import path. Path-existence guard runs *before* addFloatMesh so
    // the scene is not partially mutated when the file is missing — without
    // this, MeshFileInitializer's constructor would silently load an empty
    // ObjData (loadObject is graceful on open-fail) and queue a zero-vertex
    // mesh, violating BDD-002's "no partial-add" clause.
    bool importMesh(const std::string& prefix, const std::string& fileName,
                    PR scale, PR mass = PR(0.1), std::string* error = nullptr,
                    BehaviorType behavior = BehaviorType::Float) {
        std::string fullPath = prefix.empty() ? fileName : (prefix + "/" + fileName);
        std::ifstream probe(fullPath);
        if (!probe.good()) {
            if (error) *error = "file not found: " + fullPath;
            scene_log::logObject(
                "OBJ 가져오기 실패: " + fullPath + " (파일 없음)", false);
            return false;
        }
        // UI add-OBJ path passes BehaviorType::Rigid; self-tests use the
        // Float default. Behavior variants other than Cloth use the Float
        // params struct as a placeholder (D-036 — Rigid dispatch reads
        // behaviorType, not the variant alternative).
        BehaviorParams<PR> params = (behavior == BehaviorType::TriangularCloth)
            ? BehaviorParams<PR>{ClothBehaviorParams<PR>{PR(1e5), PR(1e5), PR(3e5), PR(0.01)}}
            : BehaviorParams<PR>{FloatBehaviorParams<PR>{}};
        scene.addGeneralMesh(
            new MeshFileInitializer<BE, PR>(
                {prefix, fileName, tinym::vec3(0), scale, mass}),
            behavior,
            params);
        registerPreviewBindingForLastRequest();
        scene_log::logObject("OBJ 가져오기: " + fileName + " (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
        return true;
    }
    // Parallel to importMesh(): loads any 3D model format via Assimp
    // (always triangulated). The "3D 모델 파일" UI button routes here.
    bool importModel(const std::string& prefix, const std::string& fileName,
                     PR scale, PR mass = PR(0.1), std::string* error = nullptr,
                     BehaviorType behavior = BehaviorType::Float) {
        std::string fullPath = prefix.empty() ? fileName : (prefix + "/" + fileName);
        std::ifstream probe(fullPath);
        if (!probe.good()) {
            if (error) *error = "file not found: " + fullPath;
            scene_log::logObject(
                "3D 모델 가져오기 실패: " + fullPath + " (파일 없음)", false);
            return false;
        }
        BehaviorParams<PR> params = (behavior == BehaviorType::TriangularCloth)
            ? BehaviorParams<PR>{ClothBehaviorParams<PR>{PR(1e5), PR(1e5), PR(3e5), PR(0.01)}}
            : BehaviorParams<PR>{FloatBehaviorParams<PR>{}};
        auto* init = new AssimpMeshFileInitializer<BE, PR>(
            {prefix, fileName, tinym::vec3(0), scale, mass});
        if (init->params.numPoints == 0 || init->params.numFacets == 0) {
            delete init;
            if (error) *error = "assimp: no triangulated geometry: " + fullPath;
            scene_log::logObject(
                "3D 모델 가져오기 실패: " + fullPath + " (지오메트리 없음)", false);
            return false;
        }
        scene.addGeneralMesh(init, behavior, params);
        registerPreviewBindingForLastRequest();
        scene_log::logObject("3D 모델 가져오기: " + fileName + " (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
        return true;
    }
    void addClothGridFast(Index particleNum1D = 200, PR size1D = 100, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.001, PR mass=0.1) {
        //particleNum1D(particleNum1D), particleNum2D(particleNum1D*particleNum1D), particleDataNum(particleNum2D*3),{
        //size1D(size1D), stretchRestLength(size1D/PR(particleNum1D-1)), shearRestLength(stretchRestLength*std::sqrt(2)), bendRestLength(stretchRestLength*2)
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XYPlane,
                tinym::vec3(0),
                particleNum1D,
                size1D,
                mass,
                true
            }),
            BehaviorType::FastGridCloth,
            // Freshly built grid is uniform: X==Y stretch, A==B shear,
            // X==Y bend. size1D/(particleNum1D-1): pn points → pn-1
            // segments, matching MeshGridInitializer's grid spacing.
            FastGridClothBehaviorParams<PR>{
                particleNum1D,
                size1D/(particleNum1D-1),                  // stretchRestX
                size1D/(particleNum1D-1),                  // stretchRestY
                size1D/(particleNum1D-1)*std::sqrtf(2),    // shearRestA
                size1D/(particleNum1D-1)*std::sqrtf(2),    // shearRestB
                size1D/(particleNum1D-1)*2,                // bendRestX
                size1D/(particleNum1D-1)*2,                // bendRestY
                kstretch,
                kshear,
                kbend,
                thickness
            }
        );
        registerPreviewBindingForLastRequest();
    }

    // FastGridCloth on a HORIZONTAL (XZ) plane centered at `center`,
    // so it can fall under gravity onto a primitive below — addClothGridFast
    // hardwires an XY (vertical) plane at the origin, which is unusable for
    // a drape/collision scene. Same FastGridCloth rest-length derivation as
    // addClothGridFast; only the plane orientation + center differ. Added
    // (not widened) so the original keeps its callers/behavior.
    void addClothGridFastAt(Index particleNum1D, PR size1D, tinym::vec3 center,
                            PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5,
                            PR thickness=0.001, PR mass=0.1) {
        uint32_t seed = static_cast<uint32_t>(Scene<BE, PR>::numMeshes);
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XZPlane,
                center,
                particleNum1D,
                size1D,
                mass,
                true, // jiggle
                seed
            }),
            BehaviorType::FastGridCloth,
            FastGridClothBehaviorParams<PR>{
                particleNum1D,
                size1D/(particleNum1D-1),                  // stretchRestX
                size1D/(particleNum1D-1),                  // stretchRestY
                size1D/(particleNum1D-1)*std::sqrtf(2),    // shearRestA
                size1D/(particleNum1D-1)*std::sqrtf(2),    // shearRestB
                size1D/(particleNum1D-1)*2,                // bendRestX
                size1D/(particleNum1D-1)*2,                // bendRestY
                kstretch,
                kshear,
                kbend,
                thickness
            }
        );
        registerPreviewBindingForLastRequest();
    }

    void addCloth(Index particleNum1D, PR size1D, tinym::vec3 center, PR kstretch=1e5, PR kshear=1e5, PR kbend=3e5, PR thickness=0.01, PR mass=0.1) {
        // numMeshes is the about-to-be-assigned id (addGeneralMesh does
        // numMeshes++); used as the deterministic RNG seed per D-018.
        uint32_t seed = static_cast<uint32_t>(Scene<BE, PR>::numMeshes);
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                PlaneDirection::XZPlane,
                center,
                particleNum1D,
                size1D,
                mass,
                true, // jiggle
                seed
            }),
            BehaviorType::TriangularCloth,
            ClothBehaviorParams<PR>{kstretch, kshear, kbend, thickness}
        );
        registerPreviewBindingForLastRequest();
    };
    void addSphere(tinym::vec3 center, Index tessellation, PR size, PR mass=PR(0.1),
                   BehaviorType behavior=BehaviorType::Float) {
        scene.addGeneralMesh(
            new MeshSphereInitializer<BE,PR>(MeshSphereInitializerParams<PR>(
                center, tessellation, size, mass)),
            behavior,
            FloatBehaviorParams<PR>{});
        registerPreviewBindingForLastRequest();
        scene_log::logObject("구 추가 (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
    }

    void addCube(tinym::vec3 center, Index tessellation, PR size, PR mass=PR(0.1),
                 BehaviorType behavior=BehaviorType::Float) {
        scene.addGeneralMesh(
            new MeshCubeInitializer<BE,PR>(MeshCubeInitializerParams<PR>(
                center, tessellation, size, mass)),
            behavior,
            FloatBehaviorParams<PR>{});
        registerPreviewBindingForLastRequest();
        scene_log::logObject("정육면체 추가 (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
    }

    void addCylinder(tinym::vec3 center, Index tessellation, PR size, PR mass=PR(0.1),
                     BehaviorType behavior=BehaviorType::Float) {
        scene.addGeneralMesh(
            new MeshCylinderInitializer<BE,PR>(MeshCylinderInitializerParams<PR>(
                center, tessellation, size, mass)),
            behavior,
            FloatBehaviorParams<PR>{});
        registerPreviewBindingForLastRequest();
        scene_log::logObject("원기둥 추가 (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
    }

    // BVH kinematic body: sphere-joint / cylinder-link proxy driven by
    // the motion file. One mesh = one id, so any part selects the body.
    // Returns false (and leaves the scene untouched) on a parse failure.
    bool addKinematicBody(const std::string& bvhPath, tinym::vec3 center,
                          PR targetHeight = PR(1.8), PR mass = PR(0.1)) {
        std::string err;
        auto* init = MeshKinematicInitializer<BE, PR>::create(
            bvhPath, center, targetHeight, mass, &err);
        if (!init) {
            std::cerr << "[addKinematicBody] " << err << std::endl;
            return false;
        }
        scene.addGeneralMesh(init, BehaviorType::Kinematic,
                             FloatBehaviorParams<PR>{});
        registerPreviewBindingForLastRequest();
        scene_log::logObject("키네마틱 바디 추가 (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
        return true;
    }

    // The kinematic initializer for `meshId`, or nullptr when the mesh
    // doesn't exist / isn't Kinematic. The initializer pointer is owned by
    // the request (shared with the live mesh), so it is stable across
    // re-packs — playback state lives there for exactly that reason.
    // Self-test probe: live backend body position for a Rigid mesh
    // (zero vector when the mesh/handle is missing).
    tinym::vec3 rigidBodyPositionOf(int meshId) {
        auto* m = Scene<BE, PR>::findById(meshId);
        if (!m || m->rigidBodyHandle == ysim::physics::kInvalidBodyHandle)
            return tinym::vec3(0.0f, 0.0f, 0.0f);
        return rigid_.getPosition(m->rigidBodyHandle);
    }

    MeshKinematicInitializer<BE, PR>* kinematicOf(int meshId) {
        auto* m = Scene<BE, PR>::findById(meshId);
        if (!m || m->behaviorType != BehaviorType::Kinematic) return nullptr;
        return dynamic_cast<MeshKinematicInitializer<BE, PR>*>(m->initializer);
    }

    // Live world-space position of a kinematic body's animated root, used by
    // the camera-follow path. Reads the centroid of the root joint's proxy
    // sphere (proxy.parts[0] — build() appends joint spheres in skeleton
    // order, so parts[0] is the BVH root joint) straight from the realized
    // vertex buffer (state.x), which already carries the FK pose plus the
    // full user transform written each frame in update(). Returns false (out
    // untouched) when the mesh is missing / not kinematic / not yet realized.
    bool kinematicRootWorldPos(int meshId, tinym::vec3& out) {
        auto* m = Scene<BE, PR>::findById(meshId);
        auto* kin = kinematicOf(meshId);
        if (!m || !kin || !m->state.x.ptr) return false;
        if (kin->proxy.parts.empty()) return false;
        const auto& p = kin->proxy.parts[0];
        if (p.vertCount == 0) return false;
        double sx = 0, sy = 0, sz = 0;
        Index counted = 0;
        for (Index v = 0; v < p.vertCount; ++v) {
            const Index idx = (p.vertStart + v) * 3;
            if (idx + 2 >= m->state.x.size) break;
            sx += m->state.x[idx];
            sy += m->state.x[idx + 1];
            sz += m->state.x[idx + 2];
            ++counted;
        }
        if (counted == 0) return false;
        const float inv = 1.0f / float(counted);
        out = tinym::vec3(float(sx) * inv, float(sy) * inv, float(sz) * inv);
        return true;
    }

    // Scrub: jump playback to `timeSec` and re-pose immediately so a
    // paused sim still shows the frame under the slider (update() is
    // gated on !pause and would otherwise apply it only on resume).
    // xPrev is snapped to the new pose — a scrub is a teleport, not a
    // swept motion the narrow phase should respond to.
    void setKinematicTime(int meshId, double timeSec) {
        auto* m = Scene<BE, PR>::findById(meshId);
        auto* kin = kinematicOf(meshId);
        if (!m || !kin || !m->state.x.ptr) return;
        kin->localTime = timeSec;
        kin->graphSession.resetBlendTravel();  // seek → restart travel integrator
        // A pending re-pack (file swap marked dirty) can leave the live
        // buffer sized for the OLD proxy; writing the new pose would
        // overrun it. The pack realizes localTime anyway.
        if (m->state.x.size != (Index)kin->params.numPoints * 3) return;
        kin->writePose(timeSec, m->scale, m->rotationQuat,
                       m->transformPosition, m->state.x.ptr);
        if (m->state.xPrev.ptr)
            std::memcpy(m->state.xPrev.ptr, m->state.x.ptr,
                        m->state.x.size * sizeof(PR));
    }

    // Swap the BVH file behind a kinematic body. Topology changes with
    // the skeleton, so: reload into the (request-owned) initializer,
    // regenerate the request preview + its render binding, drop the GL
    // cache entry, and mark dirty for a full re-pack.
    bool setKinematicFile(int meshId, const std::string& path) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        std::string err;
        if (!kin->reloadMotion(path, &err)) {
            std::cerr << "[setKinematicFile] " << err << std::endl;
            return false;
        }
        for (auto& r : Scene<BE, PR>::requestsGeneralMeshes) {
            if (r.id != meshId) continue;
            r.initializer->populatePreview(r.preview);
            registerPreviewBindingFor(r);
            break;
        }
        renderState.removeById(meshId);
        Scene<BE, PR>::dirty = true;
        scene_log::logObject("키네마틱 모션 변경 (id " +
            std::to_string(meshId) + "): " + path);
        return true;
    }

    // ── Motion-graph controls (Kovar 2002; include/motion_graph.hpp) ──────
    // Mode only switches the pose-source dispatch in writePose — geometry,
    // topology, and the single-clip state stay untouched, so flipping back
    // to 단일 클립 always works.
    void setKinematicMode(int meshId, int mode) {
        auto* kin = kinematicOf(meshId);
        if (!kin || kin->motionMode == mode) return;
        kin->motionMode = mode;
        kin->invalidateRebase();  // graphActive() flips → frame-0 source changes
        setKinematicTime(meshId, 0.0);
    }

    // The graph session's reference skeleton must be the proxy's skeleton,
    // so the reference clip becomes the loaded file first (the normal
    // file-swap re-pack path) whenever it differs.
    bool ensureKinematicRefFile(int meshId, const std::string& path) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        if (kin->params.filePath == path) return true;
        return setKinematicFile(meshId, path);
    }

    // Best Kovar transition fileA → fileB baked into a finite, scrubbable
    // composite. Always leaves graphStatus describing the outcome.
    // Raw active window {start,end} (end<0 = full) for a clip slot, fed to the
    // session builders so the baked track starts at the window's first frame.
    static std::array<int, 2> slotRawRange(MeshKinematicInitializer<BE, PR>* kin,
                                           int slot) {
        if (kin && slot >= 0 && slot < int(kin->motionSlots.size()))
            return {kin->motionSlots[slot].rangeStart,
                    kin->motionSlots[slot].rangeEnd};
        return {0, -1};
    }

    bool buildKinematicTransition(int meshId, const std::string& fileA,
                                  const std::string& fileB) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        const std::string dir = bvhAssetDir();
        if (!ensureKinematicRefFile(meshId, dir + "/" + fileA)) {
            kin->graphStatus = "실패: " + fileA + " 로드 불가";
            return false;
        }
        mograph::SessionParams p;
        p.thresholdFrac = kin->graphThreshold;
        p.markerScaleFrac = kin->graphMarkerFrac;
        std::string err;
        bool ok;
        const auto rgA = slotRawRange(kin, 0), rgB = slotRawRange(kin, 1);
        if (fileB == fileA) {
            ok = kin->graphSession.buildTransition(kin->motion, fileA,
                                                   kin->motion, fileB, p, &err,
                                                   rgA, rgB);
        } else {
            std::string lerr;
            bvh::Motion mb = bvh::load(dir + "/" + fileB, &lerr);
            if (!mb.valid()) {
                kin->graphStatus = "실패: " + lerr;
                return false;
            }
            ok = kin->graphSession.buildTransition(kin->motion, fileA, mb,
                                                   fileB, p, &err, rgA, rgB);
        }
        if (!ok) {
            kin->graphStatus = "실패: " + err;
            return false;
        }
        kin->motionMode = 2;
        kin->invalidateRebase();
        kin->transFileA = fileA;
        kin->transFileB = fileB;
        {
            char buf[192];
            std::snprintf(buf, sizeof buf,
                          "%s[%d] → %s[%d] · 비용 %.3f · %.0fms",
                          fileA.c_str(), kin->graphSession.trans.i,
                          fileB.c_str(), kin->graphSession.trans.j,
                          kin->graphSession.trans.cost,
                          kin->graphSession.buildMs);
            kin->graphStatus = buf;
            if (kin->graphSession.trans.aboveThreshold)
                kin->graphStatus += " · 임계값 초과(최선값 사용)";
        }
        setKinematicTime(meshId, 0.0);
        scene_log::logObject("모션 전환 생성 (id " + std::to_string(meshId) +
                             "): " + fileA + " → " + fileB);
        return true;
    }

    // DTW timewarp blend fileA → fileB baked into a finite, scrubbable track.
    bool buildKinematicBlend(int meshId, const std::string& fileA,
                             const std::string& fileB) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        const std::string dir = bvhAssetDir();
        if (!ensureKinematicRefFile(meshId, dir + "/" + fileA)) {
            kin->graphStatus = "실패: " + fileA + " 로드 불가";
            return false;
        }
        mograph::SessionParams p;
        p.thresholdFrac = kin->graphThreshold;
        p.markerScaleFrac = kin->graphMarkerFrac;
        std::string err;
        bool ok;
        const auto rgA = slotRawRange(kin, 0), rgB = slotRawRange(kin, 1);
        if (fileB == fileA) {
            ok = kin->graphSession.buildBlend(kin->motion, fileA, kin->motion,
                                              fileB, p, &err, rgA, rgB);
        } else {
            std::string lerr;
            bvh::Motion mb = bvh::load(dir + "/" + fileB, &lerr);
            if (!mb.valid()) {
                kin->graphStatus = "실패: " + lerr;
                return false;
            }
            ok = kin->graphSession.buildBlend(kin->motion, fileA, mb, fileB, p,
                                              &err, rgA, rgB);
        }
        if (!ok) {
            kin->graphStatus = "실패: " + err;
            return false;
        }
        kin->motionMode = 3;
        kin->invalidateRebase();
        kin->transFileA = fileA;
        kin->transFileB = fileB;
        {
            char buf[208];
            std::snprintf(buf, sizeof buf,
                          "%s ~DTW~ %s · 전환 %d프레임 · 비용 %.3f · %.0fms",
                          fileA.c_str(), fileB.c_str(),
                          kin->graphSession.trans.blendFrames,
                          kin->graphSession.trans.cost,
                          kin->graphSession.buildMs);
            kin->graphStatus = buf;
            if (kin->graphSession.trans.aboveThreshold)
                kin->graphStatus += " · 임계값 초과(최선값 사용)";
        }
        setKinematicTime(meshId, 0.0);
        scene_log::logObject("모션 전환 생성 (id " + std::to_string(meshId) +
                             "): " + fileA + " ~ " + fileB);
        return true;
    }

    // Motion graph + random walk over the initializer's graphSelFiles.
    bool buildKinematicWalk(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        if (kin->graphSelFiles.empty()) {
            kin->graphStatus = "실패: 그래프에 넣을 클립을 선택하세요";
            return false;
        }
        const std::string dir = bvhAssetDir();
        // Reference = the currently loaded file when it is selected,
        // otherwise the first selection.
        const std::string curName =
            std::filesystem::path(kin->params.filePath).filename().string();
        std::string ref = kin->graphSelFiles.front();
        for (const auto& f : kin->graphSelFiles)
            if (f == curName) { ref = f; break; }
        if (!ensureKinematicRefFile(meshId, dir + "/" + ref)) {
            kin->graphStatus = "실패: " + ref + " 로드 불가";
            return false;
        }
        std::vector<std::pair<bvh::Motion, std::string>> loaded;
        std::vector<std::string> unreadable;
        for (const auto& f : kin->graphSelFiles) {
            if (f == ref) continue;
            std::string lerr;
            bvh::Motion m = bvh::load(dir + "/" + f, &lerr);
            if (!m.valid()) { unreadable.push_back(f); continue; }
            loaded.emplace_back(std::move(m), f);
        }
        std::vector<const bvh::Motion*> ms{&kin->motion};
        std::vector<std::string> names{ref};
        for (const auto& lm : loaded) {
            ms.push_back(&lm.first);
            names.push_back(lm.second);
        }
        mograph::SessionParams p;
        p.thresholdFrac = kin->graphThreshold;
        p.markerScaleFrac = kin->graphMarkerFrac;
        p.seed = kin->walkSeed;
        std::string err;
        std::vector<std::string> incompatible;
        if (!kin->graphSession.buildRandomWalk(ms, names, p, &err,
                                               &incompatible)) {
            kin->graphStatus = "실패: " + err;
            return false;
        }
        kin->motionMode = 1;
        kin->invalidateRebase();
        const auto& st = kin->graphSession.graph.stats;
        {
            char buf[160];
            std::snprintf(buf, sizeof buf,
                          "노드 %d · 엣지 %d · 전환 %d · %.0fms",
                          st.nodesScc, st.edgesKept, st.transitionsKept,
                          st.buildMs);
            kin->graphStatus = buf;
        }
        std::string dropped;
        for (int c : st.clipsDroppedBySCC)
            dropped += (dropped.empty() ? "" : ", ") +
                       kin->graphSession.clips[c].name;
        for (const auto& f : incompatible)
            dropped += (dropped.empty() ? "" : ", ") + f + "(스켈레톤 불일치)";
        for (const auto& f : unreadable)
            dropped += (dropped.empty() ? "" : ", ") + f + "(로드 실패)";
        if (!dropped.empty())
            kin->graphStatus += " · 제외: " + dropped;
        setKinematicTime(meshId, 0.0);
        scene_log::logObject("모션 그래프 빌드 (id " + std::to_string(meshId) +
                             "): " + std::to_string(names.size()) + "개 클립");
        return true;
    }

    // New random path through the existing graph (no rebuild).
    void reseedKinematicWalk(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !kin->graphActive() || kin->motionMode != 1) return;
        kin->walkSeed = kin->walkSeed * 1664525u + 1013904223u;
        kin->graphSession.reseed(kin->walkSeed);
        kin->invalidateRebase();
        setKinematicTime(meshId, 0.0);
    }

    // ── Interactive blend space (motionMode 4) ────────────────────────────
    // Build an N-clip blend space from the initializer's presets. Reference =
    // first file (becomes the proxy skeleton). Skeleton-incompatible/unreadable
    // clips are dropped and reported, mirroring buildKinematicWalk.
    bool buildKinematicBlendSpace(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        if (kin->blendSpaceFiles.empty()) {
            kin->graphStatus = "실패: 블렌드 클립이 없습니다";
            return false;
        }
        const std::string dir = bvhAssetDir();
        const std::string ref = kin->blendSpaceFiles.front();
        if (!ensureKinematicRefFile(meshId, dir + "/" + ref)) {
            kin->graphStatus = "실패: " + ref + " 로드 불가";
            return false;
        }
        // Pass 1: load every non-ref clip into stable storage (pointer-stable
        // for buildBlendSpace, which takes addresses).
        struct Loaded { bvh::Motion m; std::string name; std::array<float, 2> co; int slot; };
        std::vector<Loaded> loaded;
        std::vector<std::string> unreadable;
        for (size_t i = 0; i < kin->blendSpaceFiles.size(); ++i) {
            const std::string& f = kin->blendSpaceFiles[i];
            if (f == ref) continue;
            const std::array<float, 2> co =
                i < kin->blendSpaceCoords.size() ? kin->blendSpaceCoords[i]
                                                 : std::array<float, 2>{0.0f, 0.0f};
            std::string lerr;
            bvh::Motion m = bvh::load(dir + "/" + f, &lerr);
            if (!m.valid()) { unreadable.push_back(f); continue; }
            loaded.push_back({std::move(m), f, co, int(i)});
        }
        // Pass 2: ref first (skeleton source), then the rest in file order so
        // coords stay parallel.
        std::vector<const bvh::Motion*> ms{&kin->motion};
        std::vector<std::string> names{ref};
        std::vector<std::array<float, 2>> coords{
            kin->blendSpaceCoords.empty() ? std::array<float, 2>{0.0f, 0.0f}
                                          : kin->blendSpaceCoords.front()};
        for (auto& lm : loaded) {
            ms.push_back(&lm.m);
            names.push_back(lm.name);
            coords.push_back(lm.co);
        }
        // Per-clip active windows in SESSION order (ref/slot-0 first, then the
        // loaded clips). Raw {start,end}; end<0 = full. buildBlendSpace clamps
        // against each clip's own length and trims before pin/velocity.
        auto rawRange = [&](int slotIdx) -> std::array<int, 2> {
            if (slotIdx >= 0 && slotIdx < int(kin->motionSlots.size()))
                return {kin->motionSlots[slotIdx].rangeStart,
                        kin->motionSlots[slotIdx].rangeEnd};
            return {0, -1};
        };
        std::vector<std::array<int, 2>> ranges{rawRange(0)};
        for (auto& lm : loaded) ranges.push_back(rawRange(lm.slot));
        mograph::SessionParams p;
        p.thresholdFrac = kin->graphThreshold;
        p.markerScaleFrac = kin->graphMarkerFrac;
        std::string err;
        std::vector<std::string> incompatible;
        const auto rmode = kin->blendAbsoluteRoot
                               ? mograph::Session::RootMode::Absolute
                               : mograph::Session::RootMode::Relative;
        if (!kin->graphSession.buildBlendSpace(ms, names, coords, p, &err,
                                               &incompatible, &ranges, rmode)) {
            kin->graphStatus = "실패: " + err;
            return false;
        }
        kin->motionMode = 4;
        kin->invalidateRebase();
        {
            char buf[160];
            std::snprintf(buf, sizeof buf, "블렌드 스페이스 · 클립 %d개",
                          (int)kin->graphSession.clips.size());
            kin->graphStatus = buf;
        }
        std::string dropped;
        for (const auto& f : incompatible)
            dropped += (dropped.empty() ? "" : ", ") + f + "(스켈레톤 불일치)";
        for (const auto& f : unreadable)
            dropped += (dropped.empty() ? "" : ", ") + f + "(로드 실패)";
        if (!dropped.empty()) kin->graphStatus += " · 제외: " + dropped;
        setKinematicTime(meshId, 0.0);
        scene_log::logObject("블렌드 스페이스 빌드 (id " + std::to_string(meshId) +
                             "): " + std::to_string(names.size()) + "개 클립");
        return true;
    }

    // Live cursor for the blend space (GUI pad drag / 1D slider). Re-roots so
    // the body stays at the object origin as the frame-0 pose shifts with the
    // mix.
    void setKinematicBlendCursor(int meshId, float x, float y) {
        auto* kin = kinematicOf(meshId);
        if (!kin || kin->motionMode != 4) return;
        kin->graphSession.cursor = {x, y};
        kin->invalidateRebase();
    }

    // Apply a curated blend preset: fill blendSpaceFiles with its 4 clips and,
    // when a space is already built, rebuild so the change is live. presetIdx
    // < 0 = 자율선택 (manual) — just marks the body custom, keeps current files.
    void setKinematicBlendPreset(int meshId, int presetIdx) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return;
        if (presetIdx < 0) { kin->blendPreset = -1; return; }
        if (presetIdx >= int(blendPresets().size())) return;
        const auto& p = blendPresets()[presetIdx];
        kin->blendSpaceFiles.assign(p.files.begin(), p.files.end());
        kin->blendPreset = presetIdx;
        for (size_t i = 0; i < kin->motionSlots.size(); ++i) {
            auto& s = kin->motionSlots[i];
            s.cachedFile.clear();   // re-cache previews
            // Apply the preset's per-clip window (default {0,-1} = full).
            const auto rg = i < p.ranges.size() ? p.ranges[i]
                                                : std::array<int, 2>{0, -1};
            s.rangeStart = rg[0];
            s.rangeEnd = rg[1];
        }
        if (kin->motionMode == 4 && kin->graphActive())
            buildKinematicBlendSpace(meshId);  // rebuild live (re-asserts mode 4)
    }

    // Apply an N-blend preset (mode 6): set the files / colors / loop-extend /
    // windows / tag / adverbs, build, then OVERRIDE the auto-detected keytimes
    // with the preset's hand-tuned ones (clamped to the built clip) and rebuild.
    // presetIdx < 0 just clears the active-preset marker (manual mode).
    bool setKinematicVerbPreset(int meshId, int presetIdx) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        if (presetIdx < 0) { kin->verbPreset = -1; return true; }
        if (presetIdx >= int(verbPresets().size())) return false;
        const auto& p = verbPresets()[presetIdx];
        const int n = int(p.files.size());
        kin->motionMode = 6;
        kin->verbFiles.assign(p.files.begin(), p.files.end());
        kin->verbTags.assign(p.tags.begin(), p.tags.end());
        kin->verbAdverb.assign(size_t(n), {0.0f, 0.0f});
        for (int i = 0; i < n && i < int(p.adverbs.size()); ++i)
            kin->verbAdverb[i] = p.adverbs[i];  // [curvy%, jog%]
        kin->verbQuery = {50.0f, 50.0f};      // query starts mid (pad center)
        kin->verbExtrapolate = p.extrapolate;  // buildKinematicVerb reads this
        for (int i = 0; i < n && i < int(kin->motionSlots.size()); ++i) {
            auto& s = kin->motionSlots[i];
            s.cachedFile.clear();  // re-cache previews at the new files/length
            if (i < int(p.colors.size())) s.color = p.colors[i];
            s.loopSel = i < int(p.loopSel.size()) ? p.loopSel[i] : 0;
            const auto rg =
                i < int(p.ranges.size()) ? p.ranges[i] : std::array<int, 2>{0, -1};
            s.rangeStart = rg[0];
            s.rangeEnd = rg[1];
        }
        if (!buildKinematicVerb(meshId)) return false;  // sets verbStatus on fail
        // Override the auto-detected keytimes with the preset's (clamped).
        for (int i = 0; i < n && i < int(kin->verbBlend.ex.size()); ++i) {
            if (i >= int(p.keys.size())) continue;
            auto& key = kin->verbBlend.ex[i].key;
            const int nf = int(kin->verbBlend.ex[i].clip.frames.size());
            for (int s2 = 0; s2 < 5; ++s2) {
                const int v = p.keys[i][s2];
                key[s2] = v < 0 ? 0 : (v > nf - 1 ? nf - 1 : v);
            }
        }
        kin->verbBlend.tags = kin->verbTags;
        syncVerbBlend(kin);  // push tag/adverb/query + rebuild (uses new keytimes)
        kin->verbPreset = presetIdx;
        setKinematicTime(meshId, 0.0);
        return true;
    }

    // ── Two-motion keytime blend (motionMode 5; Verbs & Adverbs) ──────────
    // Load the two files, retarget both onto the proxy skeleton, pin in place,
    // auto-detect foot keytimes inside each clip's active-frame window, and
    // assemble verbBlend from the initializer's tag/adverb staging. Reference =
    // verbFiles[0] (becomes the proxy skeleton); the other must be skeleton-
    // compatible. Keytimes/tags/adverbs are editable afterwards (live).
    bool buildKinematicVerb(int meshId) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return false;
        if (kin->verbFiles.size() < 2) {
            kin->verbStatus = "실패: 모션 2개가 필요합니다";
            return false;
        }
        const std::string dir = bvhAssetDir();
        const std::string ref = kin->verbFiles[0];
        if (!ensureKinematicRefFile(meshId, dir + "/" + ref)) {
            kin->verbStatus = "실패: " + ref + " 로드 불가";
            return false;
        }
        const mograph::Skeleton& skel = kin->skel();  // proxy reference skeleton
        const float refDt =
            kin->motion.frameTime > 1e-6f ? kin->motion.frameTime : 1.0f / 30.0f;
        auto loadClip = [&](const std::string& f, mograph::Clip& out,
                            std::string& err) -> bool {
            if (f == ref)
                return mograph::sampleClip(kin->motion, skel, refDt, f, out);
            bvh::Motion m = bvh::load(dir + "/" + f, &err);
            if (!m.valid()) return false;
            if (!skel.compatible(mograph::Skeleton::extract(m))) {
                err = f + " 스켈레톤 불일치";
                return false;
            }
            return mograph::sampleClip(m, skel, refDt, f, out);
        };
        const int nt = std::max(1, int(kin->verbTags.size()));
        mograph::VerbBlend vb;
        vb.skel = skel;
        vb.dt = refDt;
        vb.convexWeights = !kin->verbExtrapolate;  // signed weights ⇒ extrapolation
        vb.tags = kin->verbTags;
        vb.query.assign(size_t(nt), 0.0f);
        for (int t = 0; t < nt && t < 2; ++t) vb.query[t] = kin->verbQuery[t];
        // Mode 5 uses exactly two; mode 6 loads the whole list. Cap at the
        // preview-slot pool so every motion keeps a file picker + range window.
        const int nMot =
            std::min(int(kin->verbFiles.size()), int(kin->motionSlots.size()));
        for (int i = 0; i < nMot; ++i) {
            mograph::Clip c;
            std::string lerr;
            if (!loadClip(kin->verbFiles[i], c, lerr)) {
                kin->verbStatus = "실패: " + lerr;
                return false;
            }
            // 루프 연장: append a copy so keytimes/window span two loops.
            if (i < int(kin->motionSlots.size()) && kin->motionSlots[i].loopSel)
                mograph::doubleClipFrames(c);
            const int nf = int(c.frames.size());
            // Window from the slot's RAW stored range, clamped to the (possibly
            // doubled) build clip — not slotRange (which clamps to the preview
            // cache, unloaded when a preset sets the range without a preview).
            int a = i < int(kin->motionSlots.size()) ? kin->motionSlots[i].rangeStart : 0;
            int b = i < int(kin->motionSlots.size()) ? kin->motionSlots[i].rangeEnd : -1;
            if (b < 0 || b > nf - 1) b = nf - 1;
            if (a < 0 || a > nf - 1) a = 0;
            if (b <= a) { a = 0; b = nf - 1; }  // empty/invalid → whole clip
            // NOT pinned: the root motion travels (blended absolute/relative).
            // Foot Y (detection) is invariant under the old pin anyway.
            mograph::VerbExample e;
            bool ok = false;
            e.key = mograph::detectKeytimes(skel, c, a, b, &ok);
            e.range = {a, b};
            e.name = kin->verbFiles[i];
            e.clip = std::move(c);
            e.adverb.assign(size_t(nt), 0.0f);
            for (int t = 0; t < nt && t < 2 && i < int(kin->verbAdverb.size());
                 ++t)
                e.adverb[t] = kin->verbAdverb[i][t];
            vb.ex.push_back(std::move(e));
        }
        vb.rebuild();
        if (!vb.ready()) {
            kin->verbStatus = "실패: 블렌드를 만들 수 없습니다";
            return false;
        }
        const bool detA = vb.ex[0].key[0] < vb.ex[0].key[4];
        const int builtMot = int(vb.ex.size());
        kin->verbBlend = std::move(vb);
        if (kin->motionMode != 6) kin->motionMode = 5;  // mode 6 preserved
        kin->invalidateRebase();
        {
            char buf[224];
            std::snprintf(buf, sizeof buf,
                          "%d-모션 블렌드 · 주기 %.2fs · 태그 %d개%s", builtMot,
                          kin->verbBlend.cycleSec, nt,
                          detA ? "" : " · 키타임 자동검출 실패(균등분할)");
            kin->verbStatus = buf;
        }
        setKinematicTime(meshId, 0.0);
        std::string names = kin->verbFiles[0];
        for (int i = 1; i < builtMot && i < int(kin->verbFiles.size()); ++i)
            names += " + " + kin->verbFiles[i];
        scene_log::logObject(std::to_string(builtMot) + "-모션 블렌드 생성 (id " +
                             std::to_string(meshId) + "): " + names);
        return true;
    }

    // Push the initializer's tag/adverb/query staging into a built verbBlend and
    // rebuild its RBF (cheap). No-op until built.
    void syncVerbBlend(MeshKinematicInitializer<BE, PR>* kin) {
        if (!kin || !kin->verbBlend.ready()) return;
        const int nt = std::max(1, int(kin->verbTags.size()));
        kin->verbBlend.convexWeights = !kin->verbExtrapolate;
        kin->verbBlend.tags = kin->verbTags;
        kin->verbBlend.query.assign(size_t(nt), 0.0f);
        for (int t = 0; t < nt && t < 2; ++t)
            kin->verbBlend.query[t] = kin->verbQuery[t];
        for (size_t i = 0; i < kin->verbBlend.ex.size(); ++i) {
            kin->verbBlend.ex[i].adverb.assign(size_t(nt), 0.0f);
            for (int t = 0; t < nt && t < 2 && i < kin->verbAdverb.size(); ++t)
                kin->verbBlend.ex[i].adverb[t] = kin->verbAdverb[i][t];
        }
        kin->verbBlend.rebuild();
        kin->invalidateRebase();
    }

    // Edit one keytime handle (which: 0=LFD 1=RFU 2=RFD 3=LFU 4=cycleEnd) and
    // keep the five strictly increasing within the clip; rebuilds the warp/clock.
    void verbSetKeytime(int meshId, int exIdx, int which, int frame) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !kin->verbBlend.ready()) return;
        if (exIdx < 0 || exIdx >= int(kin->verbBlend.ex.size())) return;
        if (which < 0 || which > 4) return;
        auto& key = kin->verbBlend.ex[exIdx].key;
        const int nf = int(kin->verbBlend.ex[exIdx].clip.frames.size());
        key[which] = frame < 0 ? 0 : (frame > nf - 1 ? nf - 1 : frame);
        for (int s = 1; s < 5; ++s)  // forward sweep from the edited handle
            if (key[s] <= key[s - 1]) key[s] = std::min(nf - 1, key[s - 1] + 1);
        for (int s = 3; s >= 0; --s)  // backward sweep fixes any pile-up at nf-1
            if (key[s] >= key[s + 1]) key[s] = std::max(0, key[s + 1] - 1);
        kin->verbBlend.rebuild();
        kin->invalidateRebase();
    }

    // Set 주기끝(cycleEnd, key[4]) = 왼발착지(LFD, key[0]) + one full motion-loop
    // length, so the cycle spans exactly one loop from the landing. The loop
    // length is the ORIGINAL clip (= nf/2 when 루프 연장 doubled it, else nf);
    // clamped to the clip and kept strictly after 왼발이륙. Most useful WITH 루프
    // 연장: LFD + period lands in the appended 2nd loop, at the same pose as the
    // landing, for a clean phase seam.
    void verbSetCycleEndFull(int meshId, int exIdx) {
        auto* kin = kinematicOf(meshId);
        if (!kin || !kin->verbBlend.ready()) return;
        if (exIdx < 0 || exIdx >= int(kin->verbBlend.ex.size())) return;
        auto& key = kin->verbBlend.ex[exIdx].key;
        const int nf = int(kin->verbBlend.ex[exIdx].clip.frames.size());
        if (nf < 2) return;
        const bool dbl = exIdx < int(kin->motionSlots.size()) &&
                         kin->motionSlots[exIdx].loopSel;
        const int period = dbl ? nf / 2 : nf;  // one loop = original clip length
        int end = key[0] + period;
        if (end > nf - 1) end = nf - 1;
        if (end <= key[3]) end = std::min(nf - 1, key[3] + 1);  // keep increasing
        key[4] = end;
        kin->verbBlend.rebuild();
        kin->invalidateRebase();
    }

    // 루프 연장: double (or restore) a motion's clip so its keytimes + window
    // span two loops. Stores the per-slot flag (used at the next build) and, if
    // already built, doubles/halves that motion's clip IN PLACE + re-detects its
    // keytimes for the new length — no reload. Other motions are untouched.
    void verbSetLoopSel(int meshId, int slot, int loopSel) {
        auto* kin = kinematicOf(meshId);
        if (!kin || slot < 0 || slot >= int(kin->motionSlots.size())) return;
        const int want = loopSel > 0 ? 1 : 0;
        if (want == kin->motionSlots[slot].loopSel) return;  // no change
        kin->motionSlots[slot].loopSel = want;
        // Re-double / restore the preview cache so the frame-range slider + strobe
        // pick up the new length, and reset that slot's window to full.
        kin->motionSlots[slot].cachedFile.clear();
        kin->motionSlots[slot].rangeStart = 0;
        kin->motionSlots[slot].rangeEnd = -1;
        if (kin->verbBlend.ready() && slot < int(kin->verbBlend.ex.size())) {
            auto& e = kin->verbBlend.ex[slot];
            const int n = int(e.clip.frames.size());
            if (want && n >= 2)
                mograph::doubleClipFrames(e.clip);          // N → 2N
            else if (!want && n >= 2)
                e.clip.frames.resize(size_t(n / 2));        // 2N → N (drop copy)
            const int nf = int(e.clip.frames.size());
            bool ok = false;
            e.key = mograph::detectKeytimes(kin->verbBlend.skel, e.clip, 0,
                                            nf - 1, &ok);
            e.range = {0, nf - 1};
            kin->verbBlend.rebuild();
            kin->invalidateRebase();
        }
    }

    void verbAddTag(int meshId, const std::string& name) {
        auto* kin = kinematicOf(meshId);
        if (!kin || kin->verbTags.size() >= 2) return;
        kin->verbTags.push_back(
            name.empty() ? ("태그 " + std::to_string(kin->verbTags.size() + 1))
                         : name);
        syncVerbBlend(kin);
    }

    void verbRemoveTag(int meshId, int idx) {
        auto* kin = kinematicOf(meshId);
        if (!kin || idx < 0 || idx >= int(kin->verbTags.size()) ||
            kin->verbTags.size() <= 1)
            return;
        kin->verbTags.erase(kin->verbTags.begin() + idx);
        for (auto& a : kin->verbAdverb) {  // shift the removed column out
            if (idx == 0) a[0] = a[1];
            a[1] = 0.0f;
        }
        if (idx == 0) kin->verbQuery[0] = kin->verbQuery[1];
        kin->verbQuery[1] = 0.0f;
        syncVerbBlend(kin);
    }

    void verbSetTagName(int meshId, int idx, const std::string& name) {
        auto* kin = kinematicOf(meshId);
        if (!kin || idx < 0 || idx >= int(kin->verbTags.size())) return;
        kin->verbTags[idx] = name;
        if (kin->verbBlend.ready() && idx < int(kin->verbBlend.tags.size()))
            kin->verbBlend.tags[idx] = name;
    }

    void verbSetAdverb(int meshId, int exIdx, int tagIdx, float val) {
        auto* kin = kinematicOf(meshId);
        if (!kin || exIdx < 0 || exIdx >= int(kin->verbAdverb.size()) ||
            tagIdx < 0 || tagIdx > 1)
            return;
        kin->verbAdverb[exIdx][tagIdx] = val;
        syncVerbBlend(kin);
    }

    void verbSetQuery(int meshId, int tagIdx, float val) {
        auto* kin = kinematicOf(meshId);
        if (!kin || tagIdx < 0 || tagIdx > 1) return;
        kin->verbQuery[tagIdx] = val;
        if (kin->verbBlend.ready()) {
            const int nt = std::max(1, int(kin->verbTags.size()));
            kin->verbBlend.query.resize(size_t(nt), 0.0f);
            if (tagIdx < nt) kin->verbBlend.query[tagIdx] = val;
        }
        kin->invalidateRebase();
    }

    void verbSetExtrapolate(int meshId, bool on) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return;
        kin->verbExtrapolate = on;
        kin->verbBlend.convexWeights = !on;  // live (only read at eval time)
        kin->invalidateRebase();
    }

    // ── N-motion blend (motionMode 6) motion-list edits ──────────────────────
    // Append / drop a motion in the STAGING list (verbFiles + verbAdverb); the
    // built verbBlend re-bakes on the next 블렌드 생성, exactly like editing a
    // file. Capped at the motionSlots preview pool so every motion keeps a file
    // picker + frame window; floored at 2 (the minimum to blend).
    // ponytail: 4-motion ceiling = motionSlots pool; grow that vector to lift it.
    void verbAddMotion(int meshId, const std::string& file) {
        auto* kin = kinematicOf(meshId);
        if (!kin) return;
        if (int(kin->verbFiles.size()) >= int(kin->motionSlots.size())) return;
        kin->verbFiles.push_back(file.empty() ? "WalkLoopA.bvh" : file);
        kin->verbAdverb.push_back({0.0f, 0.0f});
        kin->verbPreset = -1;  // manual edit → 자율선택
    }
    void verbRemoveMotion(int meshId, int idx) {
        auto* kin = kinematicOf(meshId);
        if (!kin || idx < 0 || idx >= int(kin->verbFiles.size())) return;
        if (int(kin->verbFiles.size()) <= 2) return;  // need ≥2 to blend
        kin->verbFiles.erase(kin->verbFiles.begin() + idx);
        if (idx < int(kin->verbAdverb.size()))
            kin->verbAdverb.erase(kin->verbAdverb.begin() + idx);
        kin->verbPreset = -1;  // manual edit → 자율선택
    }

    void addGround(PlaneDirection dir, tinym::vec3 center, PR size1D, PR mass=0.1) {
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                dir,
                center,
                2,
                size1D,
                mass,
                false // jiggle
            }),
            BehaviorType::Float,
            FloatBehaviorParams<PR>{}
        );
        registerPreviewBindingForLastRequest();
    };

    // Parallel to addGround but with a caller-chosen particleNum1D so the
    // plane carries a real grid topology. addGround stays a fixed 2x2
    // quad for static collision floors; addPlane is the authoring entry
    // for a subdividable sheet the user can later retag as cloth via the
    // inspector behavior dropdown (changeBehavior → TriangularCloth /
    // FastGridCloth — both require the MeshGridInitializer grid topology
    // this produces). Created as Float; the cloth swap is a separate
    // user gesture, matching the existing create-then-retag flow.
    void addPlane(PlaneDirection dir, tinym::vec3 center, Index particleNum1D,
                  PR size1D, PR mass=0.1,
                  BehaviorType behavior=BehaviorType::Float) {
        if (particleNum1D < 2) particleNum1D = 2;
        scene.addGeneralMesh(
            new MeshGridInitializer<BE, PR>({
                dir,
                center,
                particleNum1D,
                size1D,
                mass,
                false // jiggle
            }),
            behavior,
            FloatBehaviorParams<PR>{}
        );
        registerPreviewBindingForLastRequest();
        scene_log::logObject("평면 추가 (id " +
            std::to_string((int)Scene<BE, PR>::numMeshes - 1) + ")");
    };

    // BDD-003: translate the named mesh by mutating state.x (and state.xPrev)
    // in place rather than introducing a per-mesh model matrix. xPrev moves
    // with x by the same delta so D-013's swept-CCD does not see the
    // user-driven teleport as a tunneling event. state.v is unchanged —
    // translating does not reset velocity.
    void translateObject(int meshId, tinym::vec3 newPos) {
        auto* mesh = Scene<BE, PR>::findById(meshId);
        if (!mesh) return;
        tinym::vec3 delta = newPos - mesh->transformPosition;
        if (!mesh->state.x.ptr) return;
        const Index n = mesh->state.x.size / 3;
        for (Index i = 0; i < n; ++i) {
            mesh->state.x.ptr[i*3+0] += delta.x;
            mesh->state.x.ptr[i*3+1] += delta.y;
            mesh->state.x.ptr[i*3+2] += delta.z;
            if (mesh->state.xPrev.ptr) {
                mesh->state.xPrev.ptr[i*3+0] += delta.x;
                mesh->state.xPrev.ptr[i*3+1] += delta.y;
                mesh->state.xPrev.ptr[i*3+2] += delta.z;
            }
        }
        // S3-3: no preview dual-write. state.x is mutated in place
        // above for immediate effect; the initializer center write
        // below makes a structural re-pack rebuild at the new position
        // deterministically; syncPreviewFromState re-derives preview.
        mesh->transformPosition = newPos;
        // Write back to the initializer's center/offset so a subsequent
        // Scene::pack() (triggered by create/import/load flows) rebuilds
        // state.x from the translated position. Without this the next
        // re-pack reseeds transformPosition from the stale initializer
        // and silently drops the edit. Mirrors the cascade in Scene::pack
        // and Simulator::toSnapshot — when a new initializer subtype
        // ships, all three sites need the corresponding case added.
        if (auto* g  = dynamic_cast<MeshGridInitializer  <BE, PR>*>(mesh->initializer)) {
            g->params.center = newPos;
        } else if (auto* sp = dynamic_cast<MeshSphereInitializer<BE, PR>*>(mesh->initializer)) {
            sp->params.center = newPos;
        } else if (auto* cb = dynamic_cast<MeshCubeInitializer  <BE, PR>*>(mesh->initializer)) {
            cb->params.center = newPos;
        } else if (auto* cy = dynamic_cast<MeshCylinderInitializer<BE, PR>*>(mesh->initializer)) {
            cy->params.center = newPos;
        } else if (auto* f  = dynamic_cast<MeshFileInitializer  <BE, PR>*>(mesh->initializer)) {
            f->params.offset = newPos;
        } else if (auto* kb = dynamic_cast<MeshKinematicInitializer<BE, PR>*>(mesh->initializer)) {
            // Without this, reset/re-pack reseeds transformPosition from
            // the stale authored center and drops the user's translation
            // (Scene::pack's kinematic branch reads params.center).
            kb->params.center = newPos;
        } else if (auto* a  = dynamic_cast<AssimpMeshFileInitializer<BE, PR>*>(mesh->initializer)) {
            // Sibling of MeshFileInitializer (see Scene::pack cascade): the
            // cast above won't catch it. Write back so a subsequent re-pack
            // (reset/import/load) rebuilds state.x at the translated pose
            // instead of the stale (0,0,0) import offset.
            a->params.offset = newPos;
        }
        // D-023: refit the BVH so click-pick reads the new pose
        // immediately, even on a paused sim before the next sim.update().
        // refit() covers per-mesh tree refit AND the SCENE-level rebuild
        // (BroadPhase::refit at src/main.cpp:3961 — single call covers both
        // levels).
        collisionPipeline.broadPhase.refit();
        // S1 refactor: translation is an in-place edit — it overwrites
        // existing pack memory (state.x / xPrev / preview) and refits
        // the BVH above. It does NOT change topology or buffer sizes, so
        // it must NOT trigger a full re-pack. The old D-041 `dirty=true`
        // here forced a realloc-ing re-initialize on every transform and
        // (compounded by the leak-forward pool reset) was the driver of
        // the cold multi-scene RSS blow-up. Structural changes still set
        // dirty at their own sites.
    }

    // FR-004 / D-021: set the named mesh's absolute orientation to newQuat.
    // Particles in state.x and state.xPrev are rotated around the mesh's
    // transformPosition pivot by the delta from the current rotationQuat;
    // mesh.rotationQuat is updated to newQuat (normalized). xPrev moves
    // with x by the same delta (D-013 invariant) so the next narrow phase
    // does not see the rotate as a tunneling event. state.v is unchanged
    // — rotating does not reset velocity.
    //
    // D-042 R-4 (2026-05-14): pack-roundtrip is now covered. The preview
    // write-back below mirrors state.x's rotation into req.preview.x;
    // Scene::pack's R-3 memcpy carries it into the rebuilt packed state.x.
    // The per-call `pendingRotations[meshId] = newAbs;` stash is RETIRED
    // here — re-applying via applyPendingMaterials would double-rotate
    // since preview already carries the rotation. loadScene's separate
    // deferred-at-load pendingRotations stash is unchanged.
    void rotateObject(int meshId, ::Quat newQuat) {
        auto* mesh = Scene<BE, PR>::findById(meshId);
        if (!mesh) return;
        if (!mesh->state.x.ptr) return;

        Quat newAbs = quatNormalize(newQuat);
        Quat delta  = quatNormalize(newAbs * quatConjugate(mesh->rotationQuat));
        tinym::vec3 pivot = mesh->transformPosition;

        const Index n = mesh->state.x.size / 3;
        for (Index i = 0; i < n; ++i) {
            tinym::vec3 p_curr(mesh->state.x.ptr[i*3+0],
                               mesh->state.x.ptr[i*3+1],
                               mesh->state.x.ptr[i*3+2]);
            tinym::vec3 p_rot = pivot + rotateVector(delta, p_curr - pivot);
            mesh->state.x.ptr[i*3+0] = p_rot.x;
            mesh->state.x.ptr[i*3+1] = p_rot.y;
            mesh->state.x.ptr[i*3+2] = p_rot.z;
            if (mesh->state.xPrev.ptr) {
                tinym::vec3 prev(mesh->state.xPrev.ptr[i*3+0],
                                 mesh->state.xPrev.ptr[i*3+1],
                                 mesh->state.xPrev.ptr[i*3+2]);
                tinym::vec3 prev_rot = pivot + rotateVector(delta, prev - pivot);
                mesh->state.xPrev.ptr[i*3+0] = prev_rot.x;
                mesh->state.xPrev.ptr[i*3+1] = prev_rot.y;
                mesh->state.xPrev.ptr[i*3+2] = prev_rot.z;
            }
        }
        // S3-3: no preview dual-write. state.x/xPrev rotated in place
        // above for immediate effect; the request + initializer params
        // carry the absolute orientation so a structural re-pack
        // rebuilds the rotated geometry deterministically;
        // syncPreviewFromState re-derives preview.
        for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
            if (req.id == meshId) {
                req.rotationQuat = newAbs;
                if (req.initializer)
                    req.initializer->getParams()->rotationQuat = newAbs;
                break;
            }
        }
        mesh->rotationQuat = newAbs;
        // D-023: refit the BVH so click-pick reads the rotated pose
        // immediately, even on a paused sim before the next sim.update().
        collisionPipeline.broadPhase.refit();
        // D-025 / D-042 R-4 (2026-05-14): the pre-R-4 pattern
        // `pendingRotations[meshId] = newAbs;` is RETIRED here. R-4's
        // preview write-back above already lands the rotation in
        // PreviewState, and Scene::pack's R-3 memcpy carries it into
        // packed state.x at the next re-init. The applyPendingMaterials
        // re-apply path would now double-rotate (preview → packed by
        // memcpy → re-applied by rotateObject again), so we no longer
        // stash from this call site. loadScene's separate
        // `pendingRotations[id] = q` stash for deferred-at-load
        // rotations is UNCHANGED — that path applies the saved rotation
        // exactly once at the first post-load initialize.
        // S1 refactor: rotation is an in-place, length-preserving edit
        // (state.x / xPrev / preview overwritten in place, BVH refit
        // done by the caller path). No topology/size change → no
        // re-pack. Removing the old D-041 `dirty=true` also stops the
        // post-load re-init cascade (loadScene→initialize→
        // applyPendingMaterials→rotateObject used to re-dirty and force
        // initialize #2/#3, the cold-load pool growth). Structural
        // changes still set dirty at their own sites.
    }

    // Set the named mesh's absolute per-axis scale to newScale. Mirrors
    // rotateObject: state.x / state.xPrev and the per-request preview are
    // scaled about the transformPosition pivot by the delta from the
    // current mesh.scale; mesh.scale + req.scale are updated to the new
    // absolute factor so Scene::pack restores it and the next call
    // composes its delta against the true current scale. state.v is
    // unchanged. reset() re-applies the stored scale to the regenerated
    // preview (scale-then-rotate, standard TRS reconstruction order).
    void scaleObject(int meshId, tinym::vec3 newScale) {
        auto* mesh = Scene<BE, PR>::findById(meshId);
        if (!mesh) return;
        if (!mesh->state.x.ptr) return;

        // Reject non-positive factors (degenerate / mirrored geometry):
        // clamp to a small epsilon so the mesh never collapses to zero
        // volume or inverts winding.
        auto clampPos = [](float v) { return v < 1e-4f ? 1e-4f : v; };
        tinym::vec3 absS(clampPos(newScale.x),
                         clampPos(newScale.y),
                         clampPos(newScale.z));
        tinym::vec3 cur = mesh->scale;
        tinym::vec3 d(absS.x / clampPos(cur.x),
                      absS.y / clampPos(cur.y),
                      absS.z / clampPos(cur.z));
        tinym::vec3 pivot = mesh->transformPosition;

        auto scaleAbout = [&](PR* base) {
            tinym::vec3 p((float)base[0], (float)base[1], (float)base[2]);
            tinym::vec3 q = p - pivot;
            base[0] = (PR)(pivot.x + q.x * d.x);
            base[1] = (PR)(pivot.y + q.y * d.y);
            base[2] = (PR)(pivot.z + q.z * d.z);
        };

        const Index n = mesh->state.x.size / 3;
        for (Index i = 0; i < n; ++i) {
            scaleAbout(&mesh->state.x.ptr[i*3]);
            if (mesh->state.xPrev.ptr) scaleAbout(&mesh->state.xPrev.ptr[i*3]);
        }
        // S3-3: no preview dual-write. state.x/xPrev scaled in place
        // above; request + initializer params carry the absolute scale
        // for deterministic re-pack; syncPreviewFromState re-derives
        // preview.
        for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
            if (req.id == meshId) {
                req.scale = absS;
                if (req.initializer)
                    req.initializer->getParams()->scale = absS;
                break;
            }
        }
        mesh->scale = absS;

        // S2 refactor: scale is now a fully in-place edit. Unlike
        // translate/rotate it changes edge lengths, so cloth spring
        // rest lengths must be re-measured from the scaled geometry —
        // otherwise every spring is pre-stressed and a stiff explicit
        // integrator blows up (this is exactly the old scale-on-load
        // crash). recomputeRestLengths overwrites the EXISTING rest
        // arrays in place (no realloc), so we no longer need the
        // D-041 `dirty=true` re-pack. Non-cloth meshes have no springs.
        if (mesh->behaviorType == BehaviorType::TriangularCloth
         || mesh->behaviorType == BehaviorType::FastGridCloth) {
            MeshAdjacencyInitializer<BE, PR>::recomputeRestLengths(
                mesh->state, mesh->adjacency);
        }
        // FastGridCloth solves against the scalar rest lengths in its
        // behavior params, which recomputeRestLengths above does NOT
        // touch (it only rewrites the adjacency arrays). state.x/xPrev
        // were just scaled in place, so re-derive the scalars from the
        // scaled grid and sync both the live mesh and its request copy
        // (so a dirty-triggered re-pack rebuilds with the new rest).
        if (mesh->behaviorType == BehaviorType::FastGridCloth) {
            if (auto* fp = std::get_if<FastGridClothBehaviorParams<PR>>(
                    &mesh->behaviorParams)) {
                recomputeFastGridRest<PR>(mesh->state.x.ptr,
                                          mesh->state.x.size / 3, *fp);
                for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
                    if (req.id == meshId) {
                        req.behaviorParams = mesh->behaviorParams;
                        break;
                    }
                }
            }
        }
        // D-023 parity: refit the BVH so click-pick reads the new extent
        // immediately, even on a paused sim before the next sim.update().
        collisionPipeline.broadPhase.refit();
    }

    // ── Point-selection vertex ops ────────────────────────────────────
    // Render-vertex (gl_VertexID over the bound MeshGL buffer = preview
    // renderXPtr) → physics-vertex index. For grid/sphere/file the two
    // are identical (no welding split); cube uses renderToPhysics.
    static Index renderToPhysicsVid(
            typename Scene<BE, PR>::RequestGeneralMesh& req, int rv) {
        if (rv < 0) return (Index)-1;
        if (req.preview.hasRender()) {
            if ((size_t)rv < req.preview.renderToPhysics.size())
                return (Index)req.preview.renderToPhysics[rv];
            return (Index)-1;
        }
        return (Index)rv;
    }

    // Inverse of renderToPhysicsVid for display: first render vert that
    // maps to `pvid`, else pvid itself (meshes with no separate render
    // mapping have render == physics).
    static int physicsToRenderVid(
            typename Scene<BE, PR>::RequestGeneralMesh& req, Index pvid) {
        if (req.preview.hasRender()) {
            for (size_t rv = 0; rv < req.preview.renderToPhysics.size(); ++rv)
                if ((Index)req.preview.renderToPhysics[rv] == pvid)
                    return (int)rv;
        }
        return (int)pvid;
    }

    typename Scene<BE, PR>::RequestGeneralMesh* findRequest(int id) {
        for (auto& r : Scene<BE, PR>::requestsGeneralMeshes)
            if (r.id == id) return &r;
        return nullptr;
    }

    // Current world position of a (object, render-vertex) pair, read
    // from the live packed state.x. Returns false if the pair is
    // invalid (mesh gone, vertex out of range).
    bool vertexWorldPos(int objId, int renderVert, tinym::vec3& out) {
        auto* mesh = Scene<BE, PR>::findById(objId);
        auto* req  = findRequest(objId);
        if (!mesh || !req || !mesh->state.x.ptr) return false;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1 || pvid*3+2 >= (Index)mesh->state.x.size)
            return false;
        out = tinym::vec3((float)mesh->state.x.ptr[pvid*3+0],
                          (float)mesh->state.x.ptr[pvid*3+1],
                          (float)mesh->state.x.ptr[pvid*3+2]);
        return true;
    }

    bool isVertexFixed(int objId, int renderVert) {
        auto* mesh = Scene<BE, PR>::findById(objId);
        auto* req  = findRequest(objId);
        if (!mesh || !req || !mesh->constraints.fixedParticles.ptr) return false;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1
            || pvid >= (Index)mesh->constraints.fixedParticles.size) return false;
        return mesh->constraints.fixedParticles[pvid] == PR(0);
    }

    // Pin / unpin a vertex. Mutates the live constraint mask AND mirrors
    // into RequestGeneralMesh::fixedVertices (the pack-surviving, scene-
    // persisted source of truth). A pin records the vertex's CURRENT
    // world position so the constraint round-trips through save/load.
    void setVertexFixed(int objId, int renderVert, bool fixed) {
        auto* mesh = Scene<BE, PR>::findById(objId);
        auto* req  = findRequest(objId);
        if (!mesh || !req || !mesh->constraints.fixedParticles.ptr) return;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1
            || pvid >= (Index)mesh->constraints.fixedParticles.size) return;

        auto it = std::find_if(req->fixedVertices.begin(),
                               req->fixedVertices.end(),
                               [pvid](const FixedVertex& f){ return f.vid == pvid; });
        if (fixed) {
            mesh->constraints.fixedParticles[pvid] = PR(0);
            tinym::vec3 p;
            if (!vertexWorldPos(objId, renderVert, p)) return;
            if (it == req->fixedVertices.end())
                req->fixedVertices.push_back(FixedVertex{(uint32_t)pvid, p});
            else
                it->pos = p;
        } else {
            mesh->constraints.fixedParticles[pvid] = PR(1);
            if (it != req->fixedVertices.end())
                req->fixedVertices.erase(it);
        }
        scene_log::logConstraint(
            (fixed ? "점 고정: obj " : "점 고정 해제: obj ")
            + std::to_string(objId) + ", 점 "
            + std::to_string(renderVert));
    }

    // Register a reference-point coincidence constraint: the follower
    // vertex (followerObj, followerRenderVert) must track the leader
    // vertex (leaderObj, leaderRenderVert). render→physics-vid mapped on
    // both ends. At most one constraint per follower vertex — a repeat
    // pick of the same follower replaces its leader. A no-op (and the
    // existing constraint, if any, is removed) when follower == leader.
    //
    // Auto-snap: the FIRST constraint between an ordered object pair
    // (followerObj→leaderObj, distinct objects) also translates the
    // whole follower object by (leaderPos − followerPos) so the two
    // reference points coincide immediately. The 2nd+ constraint
    // between the same pair only adds the per-vertex lock (no whole-
    // object move — those points are pulled together by the integrator).
    // Returns true on success.
    bool setReferenceConstraint(int followerObj, int followerRenderVert,
                                int leaderObj,   int leaderRenderVert) {
        auto* fReq = findRequest(followerObj);
        auto* lReq = findRequest(leaderObj);
        if (!fReq || !lReq) return false;
        Index fv = renderToPhysicsVid(*fReq, followerRenderVert);
        Index lv = renderToPhysicsVid(*lReq, leaderRenderVert);
        if (fv == (Index)-1 || lv == (Index)-1) return false;

        // World positions BEFORE any mutation, for the auto-snap delta.
        tinym::vec3 fPos, lPos;
        bool havePos = vertexWorldPos(followerObj, followerRenderVert, fPos)
                     && vertexWorldPos(leaderObj,  leaderRenderVert,  lPos);

        auto& list = Scene<BE, PR>::referenceConstraints;
        auto same = [&](const ReferencePointConstraint& c) {
            return c.objPair.query == (Index)followerObj
                && c.vertexPair.query == fv;
        };
        list.erase(std::remove_if(list.begin(), list.end(), same),
                   list.end());
        if (followerObj == leaderObj && fv == lv) return false;

        // First between this object pair? (checked after the same-
        // follower erase, before adding the new one.)
        bool firstBetweenPair =
            std::none_of(list.begin(), list.end(),
                [&](const ReferencePointConstraint& c) {
                    return c.objPair.query  == (Index)followerObj
                        && c.objPair.target == (Index)leaderObj;
                });

        ReferencePointConstraint c;
        c.objPair.query     = (Index)followerObj;
        c.vertexPair.query  = fv;
        c.objPair.target    = (Index)leaderObj;
        c.vertexPair.target = lv;
        list.push_back(c);

        if (firstBetweenPair && followerObj != leaderObj && havePos) {
            if (auto* fMesh = Scene<BE, PR>::findById(followerObj)) {
                tinym::vec3 delta = lPos - fPos;
                translateObject(followerObj,
                                fMesh->transformPosition + delta);
            }
        }
        // Mutating the constraint set must trigger the same next-frame
        // clean re-init every other scene mutation uses (translate /
        // rotate / behavior / removeMesh). translateObject already sets
        // dirty on the auto-snap path; set it unconditionally so the
        // no-snap (2nd+ constraint) path is covered too. Without this
        // the sim keeps running on packed/BVH state built for the old
        // constraint configuration → stale-state segfault.
        Scene<BE, PR>::dirty = true;
        scene_log::logConstraint(
            "참조점 설정: obj " + std::to_string(followerObj)
            + " 점 " + std::to_string(followerRenderVert)
            + " → obj " + std::to_string(leaderObj)
            + " 점 " + std::to_string(leaderRenderVert));
        return true;
    }

    struct PointRefView {
        bool selectedIsFollower;
        int otherObj;
        int otherVert;  // render-vid for display
    };

    // Reference constraints touching (objId, renderVert): the point as
    // follower (query) → other is the leader; or as leader (target) →
    // other is the follower. Ordered to match
    // removeReferenceConstraintForPoint's scan.
    std::vector<PointRefView> referenceConstraintsForPoint(int objId,
                                                           int renderVert) {
        std::vector<PointRefView> out;
        auto* req = findRequest(objId);
        if (!req) return out;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1) return out;
        for (const auto& c : Scene<BE, PR>::referenceConstraints) {
            if (c.objPair.query == (Index)objId
                && c.vertexPair.query == pvid) {
                auto* o = findRequest((int)c.objPair.target);
                out.push_back({true, (int)c.objPair.target,
                    o ? physicsToRenderVid(*o, c.vertexPair.target)
                      : (int)c.vertexPair.target});
            } else if (c.objPair.target == (Index)objId
                       && c.vertexPair.target == pvid) {
                auto* o = findRequest((int)c.objPair.query);
                out.push_back({false, (int)c.objPair.query,
                    o ? physicsToRenderVid(*o, c.vertexPair.query)
                      : (int)c.vertexPair.query});
            }
        }
        return out;
    }

    // Erase the `idx`-th constraint touching (objId, renderVert) using
    // the same ordered scan as referenceConstraintsForPoint.
    bool removeReferenceConstraintForPoint(int objId, int renderVert,
                                           int idx) {
        auto* req = findRequest(objId);
        if (!req) return false;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1) return false;
        auto& cons = Scene<BE, PR>::referenceConstraints;
        int seen = 0;
        for (auto it = cons.begin(); it != cons.end(); ++it) {
            bool match =
                (it->objPair.query == (Index)objId
                 && it->vertexPair.query == pvid)
             || (it->objPair.target == (Index)objId
                 && it->vertexPair.target == pvid);
            if (!match) continue;
            if (seen == idx) {
                cons.erase(it);
                scene_log::logConstraint(
                    "참조점 해제: obj " + std::to_string(objId)
                    + " 점 " + std::to_string(renderVert));
                // Same contract as setReferenceConstraint: force a
                // clean next-frame initialize() so the simulation
                // pipeline (packed buffers, BVH, rigid backend) is
                // rebuilt without the removed constraint instead of
                // continuing on now-stale state (segfault repro).
                Scene<BE, PR>::dirty = true;
                return true;
            }
            ++seen;
        }
        return false;
    }

    // Set by loadScene; consumed once by initialize() after the first
    // post-load pack. Triggers the reference-constraint auto-snap that
    // live setReferenceConstraint applies at creation time.
    bool pendingRefSnap = false;

    // Re-apply the creation-time auto-snap for loaded constraints: for
    // the FIRST constraint of each ordered (follower→leader) object
    // pair (distinct objects), translate the whole follower object so
    // its constrained vertex coincides with the leader's. Mirrors
    // setReferenceConstraint's snap; uses post-pack state.x positions.
    void applyLoadedReferenceSnaps() {
        std::set<std::pair<Index, Index>> snappedPairs;
        for (const auto& c : Scene<BE, PR>::referenceConstraints) {
            if (c.objPair.query == c.objPair.target) continue;
            auto key = std::make_pair(c.objPair.query, c.objPair.target);
            if (!snappedPairs.insert(key).second) continue;  // not first

            auto* fMesh = Scene<BE, PR>::findById((int)c.objPair.query);
            auto* lMesh = Scene<BE, PR>::findById((int)c.objPair.target);
            if (!fMesh || !lMesh) continue;
            if (!fMesh->state.x.ptr || !lMesh->state.x.ptr) continue;
            Index fb = c.vertexPair.query * 3;
            Index lb = c.vertexPair.target * 3;
            if (fb + 2 >= (Index)fMesh->state.x.size) continue;
            if (lb + 2 >= (Index)lMesh->state.x.size) continue;

            tinym::vec3 fpos((float)fMesh->state.x.ptr[fb+0],
                             (float)fMesh->state.x.ptr[fb+1],
                             (float)fMesh->state.x.ptr[fb+2]);
            tinym::vec3 lpos((float)lMesh->state.x.ptr[lb+0],
                             (float)lMesh->state.x.ptr[lb+1],
                             (float)lMesh->state.x.ptr[lb+2]);
            tinym::vec3 delta = lpos - fpos;
            translateObject((int)c.objPair.query,
                            fMesh->transformPosition + delta);
        }
    }

    // Move a single vertex to an absolute world position. Mirrors
    // translateObject's dual-write (state.x + state.xPrev + preview) but
    // for ONE vertex. Deliberately does NOT mark the scene dirty: a
    // re-pack would recompute rest lengths from the deformed preview
    // (the user's "rest == preview" invariant) which is fine for a
    // pinned corner but undesirable mid-edit; broadPhase.refit() keeps
    // picking correct without a full reinit. If the vertex is pinned,
    // its stored constraint position is updated so the pin holds here.
    void translateVertexTo(int objId, int renderVert, tinym::vec3 newPos) {
        auto* mesh = Scene<BE, PR>::findById(objId);
        auto* req  = findRequest(objId);
        if (!mesh || !req || !mesh->state.x.ptr) return;
        Index pvid = renderToPhysicsVid(*req, renderVert);
        if (pvid == (Index)-1 || pvid*3+2 >= (Index)mesh->state.x.size) return;

        Index b = pvid*3;
        tinym::vec3 cur((float)mesh->state.x.ptr[b+0],
                        (float)mesh->state.x.ptr[b+1],
                        (float)mesh->state.x.ptr[b+2]);
        tinym::vec3 d = newPos - cur;

        mesh->state.x.ptr[b+0] = (PR)newPos.x;
        mesh->state.x.ptr[b+1] = (PR)newPos.y;
        mesh->state.x.ptr[b+2] = (PR)newPos.z;
        if (mesh->state.xPrev.ptr) {
            mesh->state.xPrev.ptr[b+0] += (PR)d.x;
            mesh->state.xPrev.ptr[b+1] += (PR)d.y;
            mesh->state.xPrev.ptr[b+2] += (PR)d.z;
        }

        // S3-3: no preview dual-write (syncPreviewFromState re-derives
        // preview from state.x every frame). If the vertex is pinned,
        // its held position lives on the request (fixedVertices) and is
        // re-applied by pack — that is the only per-vertex edit that
        // survives a structural re-pack (gap-2 decision (a)).
        for (auto& fv : req->fixedVertices)
            if (fv.vid == pvid) { fv.pos = newPos; break; }

        collisionPipeline.broadPhase.refit();
    }

    // S3-2: material is a pure in-place value overwrite — write the
    // live mesh AND the request mirror (so a structural re-pack
    // preserves it; pack copies req.material onto the rebuilt mesh).
    // NO Scene::dirty / re-pack: a colour-slider drag firing this every
    // frame must not trigger a realloc-ing re-pack (that was the
    // pool-fragment-accumulation → intermittent mid-run exit). Material
    // does not affect AABBs/geometry, so nothing else need refresh.
    void setMaterial(int meshId, const Material& mat) {
        auto* mesh = Scene<BE,PR>::findById(meshId);
        if (!mesh) return;
        mesh->material = mat;
        for (auto& req : Scene<BE,PR>::requestsGeneralMeshes)
            if (req.id == meshId) { req.material = mat; break; }
    }

    // removeMesh erases the matching request, frees its initializer, then
    // RENUMBERS every surviving request so ids stay the compacted
    // [0, numMeshes) sequence that governs all mesh access (findById,
    // statesOffsets / objTrees / faceObj subscripts, objPair). Scene::pack
    // rebuilds meshes in this same request order and stamps meshes[i].id = i,
    // so id == array index == packed index everywhere after the next
    // (dirty-triggered) re-init.
    //
    // Because id is volatile (any remove shifts the ids of all later
    // meshes), every id-keyed side table is reconciled here:
    //  - selectedObj is remapped via the stable lifetimeId (so selection
    //    follows the same mesh, not whatever now occupies the old slot).
    //  - renderState (MeshGL + preview bindings, keyed by id) is rebuilt
    //    under the new ids — the GL buffers belong to each request's
    //    stable PreviewState heap, so re-registering by new id and
    //    letting getOrCreate re-materialize is correct and collision-free
    //    (this is what the old monotone-nextMeshId scheme existed to
    //    avoid; we pay one MeshGL rebuild on the rare user remove instead
    //    of leaking the id space forever).
    //  - pendingMaterials / pendingRotations are loadScene-deferred and
    //    are always already consumed+cleared by applyPendingMaterials
    //    (run inside initialize()) before any interactive removeMesh, so
    //    clearing them here is a no-op in real flows and strictly safer
    //    than carrying now-stale id keys.
    void removeMesh(int meshId) {
        auto& reqs = Scene<BE, PR>::requestsGeneralMeshes;
        auto it = std::find_if(reqs.begin(), reqs.end(),
            [meshId](const auto& r) { return r.id == meshId; });
        if (it == reqs.end()) return;

        // Capture the selected mesh's stable identity BEFORE the erase so
        // selection can be re-resolved after ids shift.
        int selectedLifetime = -1;
        if (selectedObj >= 0) {
            auto sit = std::find_if(reqs.begin(), reqs.end(),
                [this](const auto& r) { return r.id == selectedObj; });
            if (sit != reqs.end()) selectedLifetime = sit->lifetimeId;
        }
        bool removingSelected = (selectedObj == meshId);

        // Snapshot oldId→lifetimeId for ALL requests before the erase, and
        // the removed request's lifetimeId. Reference constraints store
        // volatile object ids; after renumbering they must be re-resolved
        // through the stable lifetimeId (same scheme as selection above),
        // and any constraint touching the removed mesh must be dropped.
        std::unordered_map<int, int> oldIdToLifetime;
        for (const auto& r : reqs) oldIdToLifetime[r.id] = r.lifetimeId;
        const int removedLifetime = it->lifetimeId;

        delete it->initializer;
        reqs.erase(it);
        if (Scene<BE, PR>::numMeshes > 0) Scene<BE, PR>::numMeshes--;
        scene_log::logObject("오브젝트 삭제 (id " +
            std::to_string(meshId) + ")");

        // Compact ids back to [0, size) in request order.
        for (int k = 0; k < (int)reqs.size(); ++k) reqs[k].id = k;

        // Reconcile reference-point constraints against the new ids.
        // Physics vids (vertexPair) are unaffected — each surviving
        // mesh's own geometry is unchanged; only its packed slot/id
        // moved. Drop constraints whose query OR target mesh was
        // removed (or can't be re-resolved).
        {
            std::unordered_map<int, int> lifetimeToNewId;
            for (const auto& r : reqs) lifetimeToNewId[r.lifetimeId] = r.id;
            auto remap = [&](Index& objId) -> bool {
                auto lit = oldIdToLifetime.find((int)objId);
                if (lit == oldIdToLifetime.end()) return false;
                if (lit->second == removedLifetime) return false;
                auto nit = lifetimeToNewId.find(lit->second);
                if (nit == lifetimeToNewId.end()) return false;
                objId = (Index)nit->second;
                return true;
            };
            auto& cons = Scene<BE, PR>::referenceConstraints;
            cons.erase(std::remove_if(cons.begin(), cons.end(),
                [&](ReferencePointConstraint& c) {
                    return !remap(c.objPair.query)
                        || !remap(c.objPair.target);
                }), cons.end());
        }

        // Re-resolve the selection against the new ids via lifetimeId.
        selectedObj = -1;
        if (!removingSelected && selectedLifetime >= 0) {
            for (auto& r : reqs)
                if (r.lifetimeId == selectedLifetime) { selectedObj = r.id; break; }
        }

        // Rebuild the entire id-keyed render cache under the new ids.
        renderState.clear();
        renderState.clearPreviewBindings();
        for (auto& r : reqs) registerPreviewBindingFor(r);

        Scene<BE, PR>::dirty = true;
    }

    // BDD-006 / D-036: in-place behavior switch. Preserves mesh.id +
    // transform + shape + state.x + state.v; the next sim.update()
    // dispatches the new behavior. Returns false on invalid request
    // (mesh not found, reserved-not-shipped behavior, FastGridCloth
    // on non-grid topology). Reserved (Elastic / Fluid / Generator)
    // are rejected here AND hidden from the inspector dropdown — see
    // mesh_inspector_gui.cpp's behavior combo.
    //
    // BDD-006-RIGID-DISPATCH-PARKED (standing constraint): Rigid tag
    // is accepted, but applyEnvironmentForces' dispatch above treats
    // anything that isn't Float/TriangularCloth/FastGridCloth as a
    // non-wind-susceptible body with no specific dispatch — effectively
    // kinematic until slice B-3 wires IRigidPhysicsBackend's Bullet impl.
    bool changeBehavior(int meshId, BehaviorType newType) {
        auto* mesh = Scene<BE, PR>::findById(meshId);
        if (!mesh) return false;
        // D-036 turn-32 addendum: keep broad-phase's cached behavior
        // arrays in lock-step with mesh.behaviorType so the per-pair
        // Float-skip in BroadPhase reflects the new tag immediately
        // (instead of lagging until the next full rebuild).
        // `objTrees[idx].objBehavior` is set at build() but the
        // skip-rebuild gate keeps the stale value otherwise (D-026);
        // `shBroadPhase.meshBehaviors[idx]` short-circuits when
        // size matches numMeshes. Guards handle the not-yet-allocated
        // edge (changeBehavior before initialize): next rebuild then
        // picks up the correct value from mesh->behaviorType.
        auto syncBroadPhaseCaches = [&](BehaviorType bt) {
            if (Scene<BE, PR>::meshes.empty()) return;
            Index idx = (Index)(mesh - &Scene<BE, PR>::meshes[0]);
            if (idx < 0 || idx >= (Index)Scene<BE, PR>::meshes.size()) return;
            if (idx < (Index)collisionPipeline.broadPhase.objTrees.size()) {
                collisionPipeline.broadPhase.objTrees[idx].objBehavior = bt;
            }
            if (shBroadPhase.meshBehaviors.ptr
                && idx < (Index)shBroadPhase.meshBehaviors.size) {
                shBroadPhase.meshBehaviors[idx] = (uint32_t)bt;
            }
        };
        // D-039 helper: meshId derived from pointer arithmetic above.
        auto meshIdOf = [&]() -> int {
            if (Scene<BE, PR>::meshes.empty()) return -1;
            return (int)(mesh - &Scene<BE, PR>::meshes[0]);
        };
        // D-041: any accepted behavior transition marks scene dirty so
        // Simulator::update's pre-pause dirty-check re-initializes on
        // the next frame (full pack-rebuild + rigid backend reset).
        // Also sync mesh.behaviorType + behaviorParams to the matching
        // requestsGeneralMeshes entry so the next Scene::pack rebuilds
        // the mesh with the NEW tag (without this, pack reverts to the
        // request's original Float / Cloth tag and the user's toggle
        // is silently lost across the dirty-triggered pack rebuild).
        auto markDirtyAndAccept = [&]() {
            for (auto& r : Scene<BE, PR>::requestsGeneralMeshes) {
                if (r.id == mesh->id) {
                    r.behaviorType   = mesh->behaviorType;
                    r.behaviorParams = mesh->behaviorParams;
                    break;
                }
            }
            Scene<BE, PR>::dirty = true;
            return true;
        };
        switch (newType) {
            case BehaviorType::Float:
                mesh->behaviorType = BehaviorType::Float;
                mesh->behaviorParams = FloatBehaviorParams<PR>{};
                // D-039: clear rigid linkage on Float transition. Backend
                // slot stays (Euler removeBody is slot-leak per D-038);
                // mesh stops following any prior rigid body.
                mesh->rigidBodyHandle = ysim::physics::kInvalidBodyHandle;
                mesh->rigidLastBodyPos = tinym::vec3{};
                syncBroadPhaseCaches(BehaviorType::Float);
                return markDirtyAndAccept();
            case BehaviorType::TriangularCloth:
                mesh->behaviorType = BehaviorType::TriangularCloth;
                mesh->behaviorParams = ClothBehaviorParams<PR>{
                    PR(1e5), PR(1e5), PR(3e5), PR(0.01)
                };
                syncBroadPhaseCaches(BehaviorType::TriangularCloth);
                return markDirtyAndAccept();
            case BehaviorType::FastGridCloth: {
                // Only valid on a square-regular grid topology. The
                // sole initializer that produces one is
                // MeshGridInitializer; dynamic_cast is the standing
                // discriminator. pn1D < 2 is degenerate (single line
                // of particles) and rejected by the same gate.
                auto* g = dynamic_cast<MeshGridInitializer<BE, PR>*>(mesh->initializer);
                if (!g) return false;
                Index pn1D = g->params.particleNum1D;
                if (pn1D < 2) return false;
                PR size1D = g->params.size1D;
                // Grid spacing is size1D / (pn1D - 1): pn1D points span
                // pn1D-1 segments. Using pn1D here (the prior code, and
                // the same off-by-one in addClothGridFast) made every
                // FastGridCloth stretch spring's rest length shorter than
                // the actual edge, so the sheet contracted on the first
                // step and never settled. Mirrors MeshGridInitializer's
                // `length = size1D/(particleNum1D-1)` at construction.
                PR rest = size1D / PR(pn1D - 1);
                PR restD = rest * std::sqrt(PR(2));
                PR restB = rest * PR(2);
                mesh->behaviorType = BehaviorType::FastGridCloth;
                // Seed uniform (X==Y, A==B); the dirty-pack that follows
                // re-derives all 6 from the actual (possibly scaled)
                // geometry via recomputeFastGridRest.
                mesh->behaviorParams = FastGridClothBehaviorParams<PR>{
                    static_cast<uint>(pn1D),
                    rest,  rest,
                    restD, restD,
                    restB, restB,
                    PR(1e5), PR(1e5), PR(3e5), PR(0.001)
                };
                syncBroadPhaseCaches(BehaviorType::FastGridCloth);
                return markDirtyAndAccept();
            }
            case BehaviorType::Rigid:
                // D-039: Rigid integrator dispatch now real at the outer
                // Simulator::update C++ layer (state.x follows backend
                // body via Δpos). BDD-006-RIGID-DISPATCH-PARKED RETIRED.
                mesh->behaviorType = BehaviorType::Rigid;
                // BehaviorParams variant has no Rigid alternative; the
                // existing variant value persists (harmless — the
                // simulator's dispatch reads behaviorType, not the
                // variant's alternative, for Rigid-tagged meshes).
                syncBroadPhaseCaches(BehaviorType::Rigid);
                ensureRigidBackendBody(meshIdOf());
                return markDirtyAndAccept();
            case BehaviorType::Elastic:
            case BehaviorType::Fluid:
            case BehaviorType::Generator:
            default:
                // Reserved-not-shipped per PRD v1.
                return false;
        }
    }

    void memoryAllocation() {
        //for(auto& plane : sceneObjects.planes) plane.memoryAllocation(pool);
        for(auto& mesh : scene.meshes) mesh.memoryAllocation();
    }


    // D-042 R-8 (2026-05-15): user-facing "reset to addX-time geometry"
    // entry point. Distinct from initialize() because post-D-042 the
    // dirty-rebuild path (Simulator::update → if(dirty) initialize()) is
    // expected to PRESERVE in-flight sim state through Scene::pack's R-3
    // memcpy(preview → state.x); only an explicit user gesture (e.g., the
    // "0" hotkey) should clobber preview back to initializer truth.
    //
    // Mechanism: repopulate each request's preview from its initializer,
    // then re-apply the request's stored scale and rotationQuat to that
    // fresh (unit-scale, identity-orientation) preview around the mesh's
    // transformPosition pivot, in standard TRS reconstruction order
    // (scale first, then rotate). Translates have already write-backed
    // center/offset into initializer params (see translateObject), so
    // post-reset positions reflect those edits; the scale + rotation
    // re-apply below makes the full transform survive reset
    // symmetrically. Pivot matches the one scaleObject / rotateObject
    // used (mesh->transformPosition == initializer center/offset, since
    // translate write-backs keep them in sync).
    //
    // FUTURE DIRECTION: when a per-frame position cache (Alembic / ring
    // buffer) ships, reset() should load frame[0] of the cache instead of
    // re-running populatePreview — the cache is the authoritative "first
    // frame" record while populatePreview is just an initializer-param
    // regen that loses any pre-sim deformation. Until then this is the
    // closest approximation.
    // S3-3: reset = rebuild the pack from the requests. pack now bakes
    // the full transform (scale→rotate→translate) deterministically
    // from each request's initializer params, and syncPreviewFromState
    // re-derives preview, so the old manual preview repopulate +
    // scale/rotate pass here is redundant. A structural rebuild from
    // the request IS the reset (sim state returns to the authored
    // configuration; transforms/pins/constraints are preserved because
    // they live on the request).
    // Rewind every kinematic body's playback clock to frame 0. The clock
    // lives on the initializer precisely so it SURVIVES ordinary re-packs
    // (file swap, mesh add) — so initialize() must NOT do this. The two
    // callers that DO mean "fresh run" (reset, target-frame restart) call
    // this explicitly; a following initialize() then bakes the t=0 pose.
    void rewindKinematicClocks() {
        for (auto& req : Scene<BE, PR>::requestsGeneralMeshes)
            if (auto* kin = dynamic_cast<MeshKinematicInitializer<BE, PR>*>(req.initializer))
                kin->localTime = 0.0;
    }

    void reset() {
        rewindKinematicClocks();
        initialize();
    }

    // Imprint the master cluster-mode switch onto the BroadPhase: the same
    // flag combo --bench-cluster-compare uses. Called after every BroadPhase
    // (re)construction (initialize) and whenever the profiler toggle flips,
    // so the '0' reset can't silently drop cluster mode. Clears each tree's
    // build cache so the per-object split takes on the next build.
    void applyClusterMode() {
        auto& bp = collisionPipeline.broadPhase;
        bool on = clusterModeOn;
        bp.twoMeshExperiment = on;
        bp.useSubObjectBVH   = on;
        bp.clusterNonGridBVH = on;
        bp.clusterVFPipeline = on;
        if (on) {
            bp.subTopMode = 1; bp.fusedRefitEnlarge = true;
            if (bp.subBvhSplitS < 2) bp.subBvhSplitS = 3;  // sane global fallback for inherit (-1) meshes
        }
        for (auto& tr : bp.objTrees) tr.builtForLifetimeId = -1;
    }

    // Cheap per-frame check: re-imprint cluster mode only when the BroadPhase
    // no longer matches the master switch (the profiler checkbox flipped it).
    // No-op (no rebuild) when already consistent, so safe to call every frame.
    void reconcileClusterMode() {
        auto& bp = collisionPipeline.broadPhase;
        bool cur = bp.twoMeshExperiment && bp.clusterVFPipeline;
        if (cur != clusterModeOn) applyClusterMode();
    }

    void initialize() {
        GlobalAutoAllocator<BE>::globalInitialize(1<<20);
        // D-041: rewind the global pool's bump markers BEFORE Scene::pack
        // reallocates all packedMeshData buffers. The pool's backing
        // memory (Metal buffers / CPU heap) stays live; only marker=0
        // resets so the next set of VectorBase allocations reuses the
        // existing capacity. Without this, every initialize leaked the
        // prior pack's allocations forward, growing the pool unboundedly.
        //
        // Force Scene::dirty=true so pack does a FULL rebuild (the soft-
        // reset path at Scene::pack:1909 would skip reallocation, leaving
        // packedMeshData with stale pointers that BroadPhase::build below
        // would overwrite via its own pool allocations). Safe overall
        // because Scene::pack + BroadPhase::build + shBroadPhase.build
        // refresh every VectorBase that downstream consumers see.
        GlobalAutoAllocator<BE>::reset();
        Scene<BE, PR>::dirty = true;
        std::cout << "[Simulator Init] Memory pool allocated" << std::endl;

        Scene<BE, PR>::pack();
        // D-042 R-2: `renderState.clear()` call retired here — MeshGL is now
        // bound to PreviewState heap pointers (R-1's std::vector buffers),
        // which are stable across Scene::pack reallocations. The pre-R-2
        // band-aid existed because MeshGL captured packed sub-view pointers
        // that became dangling whenever pack rebuilt; preview heap is
        // immune. `MeshRenderState::clear()` itself remains as a public API
        // for future forced-rebuild paths if needed.
        // GlobalAutoAllocator::reset() above rewound the pool; pack()
        // re-handed those offsets to packedMeshData/packedCollisionData.
        // The broad-phase BVH/SH persist pool-backed buffers whose build()
        // only reallocates on a size/identity change — so a same-mesh-count
        // reset would skip realloc and leave positions/indices/tree (and
        // Float-skipped objTrees) ALIASING the freshly packed collision
        // data, corrupting it on the next refit and crashing at the first
        // real collision. Force-fresh the broad phases so the build() calls
        // below take the realloc branch and get non-aliased pool memory.
        // (BVH()/SpatialHashing() ctors only fetch cached PSOs — cheap.)
        collisionPipeline.broadPhase     = decltype(collisionPipeline.broadPhase){};
        collisionPipeline.broadPhaseTest = decltype(collisionPipeline.broadPhaseTest){};
        shBroadPhase                     = decltype(shBroadPhase){};
        mlBroadPhase                     = decltype(mlBroadPhase){};

        // Broad-phase method headless hook: YSIM_BROADPHASE=sh|ml selects the
        // single-level or multi-level spatial hash (default BVH). Tunables:
        // YSIM_ML_LEVELS=<L>, YSIM_FLOOR_DIAG=<world-diag exclusion threshold>.
        if (const char* bp = std::getenv("YSIM_BROADPHASE")) {
            std::string s(bp);
            if (s == "sh") { useSpatialHashing = true;  useMultiLevelSH = false; }
            else if (s == "ml") { useMultiLevelSH = true; useSpatialHashing = false; }
        }
        if (const char* lv = std::getenv("YSIM_ML_LEVELS")) {
            int L = std::atoi(lv);
            if (L >= 1) mlBroadPhase.numLevels = L;
        }
        if (const char* fd = std::getenv("YSIM_FLOOR_DIAG")) {
            float d = (float)std::atof(fd);
            if (d > 0.f) mlBroadPhase.floorExcludeDiag = d;
        }

        // Sub-object (multi-root) LBVH headless hook: YSIM_SUBOBJECT=<s>
        // enables the Phase-1 multi-root path on square cloths with split s
        // and turns on the one-shot CPU validator. Mirrors YSIM_NO_ENLARGE.
        if (const char* so = std::getenv("YSIM_SUBOBJECT")) {
            int s = std::atoi(so);
            if (s >= 1) {
                collisionPipeline.broadPhase.useSubObjectBVH = true;
                collisionPipeline.broadPhase.subBvhSplitS = s;
                collisionPipeline.broadPhase.validateSubObject = true;
            }
        }
        // Sub-object top-phase mode headless hook (needs YSIM_SUBOBJECT for the
        // grouped path). Default 0 = CPU SAP; YSIM_SUBTOP=gpu (or 1) = GPU brute.
        if (const char* st = std::getenv("YSIM_SUBTOP")) {
            std::string v(st);
            collisionPipeline.broadPhase.subTopMode = (v == "gpu" || v == "1") ? 1 : 0;
        }

        // Re-imprint the GUI master cluster-mode onto the freshly
        // reconstructed BroadPhase (line ~10388 wiped it) BEFORE build, so
        // the '0' reset preserves cluster mode + per-object splits.
        applyClusterMode();

        collisionPipeline.broadPhase.build(scene);
        shBroadPhase.build(scene);
        mlBroadPhase.build(scene);

        // Anomaly flag is pool-backed like everything above, so the
        // GlobalAutoAllocator::reset() at the top of this function turned
        // any prior allocation into an aliased stale pointer. Re-allocate
        // fresh every initialize (covers reset / load / changeBehavior
        // rebuilds) and clear it — a new run starts un-flagged.
        system.anomalyFlag = VectorBase<BE, uint32_t>(1);
        system.anomalyFlag[0] = 0u;

        //Scene<BE, PR>::initialize();

        frame = 0;

        // D-025: auto-apply pendingRotations + pendingMaterials so any
        // edit-time rotation (rotateObject) or load-time rotation
        // (loadScene) survives Scene::pack rebuild. Existing explicit
        // applyPendingMaterials() calls (after loadScene in main.cpp /
        // runSelfTest) become no-ops because the maps are cleared here.
        applyPendingMaterials();

        // Re-apply the reference-constraint auto-snap once after a load.
        // Live setReferenceConstraint translates the follower so the
        // first constraint between an object pair coincides; loadScene
        // pushes constraints directly (bypassing that), so without this
        // the follower starts far from the (possibly moving Rigid)
        // leader and ref_constraint_copy_pos teleports it across a large
        // gap on substep 0 → stiff-spring blow-up → NaN → GPU OOB.
        // Runs AFTER applyPendingMaterials so scale/rotation are final.
        if (pendingRefSnap) {
            applyLoadedReferenceSnaps();
            pendingRefSnap = false;
        }

        // D-039: rigid backend per-scene reset. Stale handles from prior
        // scene are cleared; the sweep below idempotently re-adds backend
        // bodies for currently-tagged Rigid meshes (covers persistence
        // load + post-pack-rebuild + post-resetScene flows).
        rigid_.shutdown();
        rigid_.initialize(tinym::vec3(
            (float)Scene<BE, PR>::environment.gravity.x,
            (float)Scene<BE, PR>::environment.gravity.y,
            (float)Scene<BE, PR>::environment.gravity.z));
        // D-040 addendum (2026-05-14): the implicit y=0 plane was
        // removed because it interfered with user-installed ground
        // meshes (e.g., `addGround(XZPlane, (0,-1,0), 5)`). Instead,
        // sweep Float-tagged grid meshes via ensureRigidStaticGround
        // — Rigid bodies now collide against the user's ACTUAL scene
        // objects, not a virtual plane. Non-grid Float meshes
        // (imported .obj FloatMesh, etc.) are NOT auto-collidable;
        // user can tag them Rigid for collision or a future slice
        // adds StaticMesh triangle-soup support.
        for (auto& m : Scene<BE, PR>::meshes) {
            m.rigidBodyHandle  = ysim::physics::kInvalidBodyHandle;
            m.rigidLastBodyPos = tinym::vec3{};
        }
        for (int i = 0; i < (int)Scene<BE, PR>::meshes.size(); ++i) {
            ensureRigidStaticGround(i);
            ensureRigidBackendBody(i);
        }

        std::cout << "[Simulator Init] All scene objects are initialized" << std::endl;
    }


    // Per-frame fill of each mesh's externalForces buffer from
    // Scene::environment. Float-tagged meshes are left at exactly zero so
    // BDD-009's strict-equality clause does not depend on integration
    // cancellation. Wind is force-per-particle (no mass scaling) and only
    // applied to wind-susceptible behaviors (cloth) — see FR-012.
    void applyEnvironmentForces() {
        const auto& env = Scene<BE, PR>::environment;
        for (auto& mesh : Scene<BE, PR>::meshes) {
            PR* ext = mesh.externalForces.externalForces.ptr;
            const PR* mass = mesh.state.m.ptr;
            if (!ext) continue;
            const Index numPoints = mesh.state.x.size / 3;
            if (mesh.behaviorType == BehaviorType::Float ||
                mesh.behaviorType == BehaviorType::Rigid ||
                mesh.behaviorType == BehaviorType::Kinematic) {
                // D-039: Rigid bodies are integrated by the rigid backend;
                // the cloth-side externalForces buffer is unused. Zero
                // matches Float's pattern + retires BDD-006-RIGID-DISPATCH
                // -PARKED's "gravity accumulates into Rigid-tagged" claim.
                std::memset(ext, 0, sizeof(PR) * numPoints * 3);
                continue;
            }
            const bool windSusceptible =
                (mesh.behaviorType == BehaviorType::TriangularCloth ||
                 mesh.behaviorType == BehaviorType::FastGridCloth);
            // Per-mesh UI toggles gate gravity / wind contributions. When
            // both are cleared the buffer is zeroed (same shape as the
            // Float/Rigid early-return above).
            const bool gravityOn = mesh.applyGravity;
            const bool windOn    = mesh.applyWind && windSusceptible;
            for (Index p = 0; p < numPoints; ++p) {
                Index b = p * 3;
                PR mp = mass ? mass[b] : PR(1);
                if (gravityOn) {
                    ext[b    ] = (PR)env.gravity.x * mp;
                    ext[b + 1] = (PR)env.gravity.y * mp;
                    ext[b + 2] = (PR)env.gravity.z * mp;
                } else {
                    ext[b    ] = PR(0);
                    ext[b + 1] = PR(0);
                    ext[b + 2] = PR(0);
                }
                if (windOn) {
                    ext[b    ] += (PR)env.wind.x;
                    ext[b + 1] += (PR)env.wind.y;
                    ext[b + 2] += (PR)env.wind.z;
                }
            }
        }
    }

    void update() {
        // D-041: dirty-check BEFORE pause gate so any mutation (add /
        // translate / rotate / behavior / material / remove) gets a
        // full re-initialize on the very next frame even when paused.
        // initialize() forces dirty=true → Scene::pack does full rebuild
        // → pool markers reset → fresh allocations. Without this the
        // user could mutate state while paused (e.g., move a mesh in
        // the Inspector) and not see the change reflected in collision
        // / rigid-body / cloth pipelines until they unpaused.
        if (Scene<BE, PR>::dirty) initialize();
        if(pause) return;

        // Anomaly halt: the integrators' world-bounds guard raised the
        // flag (a vertex went NaN/Inf or escaped the world box) on a
        // previous frame's GPU work. Pause so the user sees the scene
        // frozen at the first bad frame instead of a clamped explosion;
        // clearing the flag here means unpausing resumes detection afresh
        // (a still-diverging scene re-pauses on the next frame).
        if (system.anomalyFlag.ptr && system.anomalyFlag[0] != 0u) {
            system.anomalyFlag[0] = 0u;
            pause = true;
            scene_log::logObject(
                "시뮬레이션 자동 정지: 비정상 위치/속도 감지 (NaN·폭주) — 적분기 가드 발동");
            std::cerr << "[Simulator] anomaly halt: non-finite/out-of-box "
                         "vertex detected; simulation paused\n";
            return;
        }

        // Slice (c) c-0: keep the analytic-primitive array current.
        // Once per update() suffices for v1 (Q2: primitives static);
        // c-4 moves this per-substep when Bullet drives primitive
        // motion. Inert — no consumer yet.
        Scene<BE, PR>::refreshAnalyticShapes();

        // 진행 바 로직: 직전 실행에서 목표 프레임에 도달해 멈춰 있었다면,
        // 다시 재생을 누른 이 시점에서 frame 0으로 되돌린 뒤 진행한다.
        //
        // 여기서 reset()/initialize()를 직접 부르면 pool 마커 rewind +
        // Scene::pack 재할당이 collision/substep 상태가 살아있는 update()
        // 한복판에서 일어나, 재시작 몇 프레임 뒤 dangling 포인터로 segfault
        // (S3-2 커밋이 고친 "intermittent mid-run exit"와 동일 패턴).
        // 안전한 재초기화 지점은 update() 맨 위의 `if (dirty) initialize()`
        // 하나뿐이므로, 여기서는 dirty만 세우고 이번 프레임을 건너뛴다.
        // 다음 update()의 최상단이 frame=0으로 재초기화한 뒤 진행한다.
        {
            const Index tgt = (Index)(targetFrames < 1 ? 1 : targetFrames);
            if (frame >= tgt) {
                // Fresh run: rewind kinematic clocks too. initialize() (next
                // update's top) only resets `frame`, so without this the body
                // would resume mid-clip while the sim restarts from frame 0.
                // Pure field sets — no pool/pack work, safe mid-update.
                rewindKinematicClocks();
                Scene<BE, PR>::dirty = true;
                return;
            }
        }
        //std::cout << "[Simulator Update] Start update" << std::endl;
        
        
        if(frame % 10 == 0) {
            // BVH is always rebuilt — click-ray and showBox/showSceneBox use it
            // even when the active broadphase is SpatialHashing.
            if (profiler) {
                auto scope = profiler->scoped("bvh_build");
                collisionPipeline.broadPhase.build(scene);
            } else {
                collisionPipeline.broadPhase.build(scene);
            }
            if (useSpatialHashing) {
                if (profiler) {
                    auto scope = profiler->scoped("sh_build");
                    shBroadPhase.build(scene);
                } else {
                    shBroadPhase.build(scene);
                }
            }
            if (useMultiLevelSH) {
                if (profiler) {
                    auto scope = profiler->scoped("mlsh_build");
                    mlBroadPhase.build(scene);
                } else {
                    mlBroadPhase.build(scene);
                }
            }
        }


        applyEnvironmentForces();

        // D-040 (folds Estimator turn-35 WARNING): runtime gravity edits
        // via the Environment widget mutate Scene::environment.gravity
        // directly. Push that to the rigid backend each frame so live
        // gravity changes propagate. Cheap (per-frame btVector3 copy).
        // Per-mesh Apply Gravity for Rigid bodies is handled at body-
        // creation time via mass=0 (static, "Float-like") vs mass=1
        // (dynamic). See ensureRigidBackendBody.
        rigid_.setGravity(tinym::vec3(
            (float)Scene<BE, PR>::environment.gravity.x,
            (float)Scene<BE, PR>::environment.gravity.y,
            (float)Scene<BE, PR>::environment.gravity.z));

        // Wind force on Rigid bodies. The cloth-side path adds wind into
        // the per-particle externalForces buffer (applyEnvironmentForces);
        // Bullet-driven Rigid bodies bypass that buffer, so we route wind
        // through applyForce instead. at_world_point = body position →
        // zero lever arm → no torque (matches cloth's "uniform push" feel).
        // Gated on mesh.applyWind, mirroring the cloth gate. Static Rigid
        // (mass=0, gravity-off) silently ignores the force per Bullet's
        // own contract. Bullet clears queued forces after each step, so
        // re-applying every frame is correct.
        const tinym::vec3 windForce(
            (float)Scene<BE, PR>::environment.wind.x,
            (float)Scene<BE, PR>::environment.wind.y,
            (float)Scene<BE, PR>::environment.wind.z);
        if (windForce.x != 0.0f || windForce.y != 0.0f || windForce.z != 0.0f) {
            for (auto& m : Scene<BE, PR>::meshes) {
                if (m.behaviorType != BehaviorType::Rigid) continue;
                if (!m.applyWind) continue;
                if (m.rigidBodyHandle == ysim::physics::kInvalidBodyHandle) continue;
                rigid_.applyForce(m.rigidBodyHandle, windForce, m.transformPosition);
            }
        }

        // D-039: step the rigid backend once per outer ysim frame. Bullet's
        // stepSimulation(h, 1, h) takes exactly one substep of h.
        rigid_.step(static_cast<float>(system.h), 1);

        // D-039: apply Δpos = backend.getPosition(handle) - rigidLastBodyPos
        // to every vertex of state.x AND state.xPrev for each Rigid-tagged
        // mesh with a valid handle. Translation only (rotation propagation
        // deferred to a future slice — sphere is rot-symmetric; cube has
        // zero angular_velocity in default scenes). Belt-and-suspenders:
        // lazily re-add if the handle is somehow invalid.
        for (int mi = 0; mi < (int)Scene<BE, PR>::meshes.size(); ++mi) {
            auto& m = Scene<BE, PR>::meshes[mi];
            if (m.behaviorType != BehaviorType::Rigid) continue;
            if (m.rigidBodyHandle == ysim::physics::kInvalidBodyHandle) {
                ensureRigidBackendBody(mi);
                if (m.rigidBodyHandle == ysim::physics::kInvalidBodyHandle) continue;
            }
            const tinym::vec3 now = rigid_.getPosition(m.rigidBodyHandle);
            // Snap the mesh CENTROID to the backend body instead of
            // accumulating per-frame deltas. The narrow-phase response
            // also writes Rigid verts (they are broad-phase queries), and
            // those displacements are invisible to Bullet — with the old
            // incremental delta they accumulated forever, so a kinematic
            // body walking through a rigid shoved its mesh permanently
            // off its Bullet body (the "floats up until reset" report;
            // reset only looked like a fix because it rewound the walker
            // to frame 0). Centroid snap re-rigidifies every frame: the
            // pack bakes verts centered on transformPosition == the
            // body's spawn position, so body-now minus current centroid
            // IS the correction (translation-only, like the old delta).
            const Index nVerts = m.state.x.size / 3;
            PR* xp = m.state.x.ptr;
            PR* xPrev = m.state.xPrev.ptr;
            tinym::vec3 centroid{0, 0, 0};
            for (Index vi = 0; vi < nVerts; ++vi) {
                centroid.x += (float)xp[vi*3+0];
                centroid.y += (float)xp[vi*3+1];
                centroid.z += (float)xp[vi*3+2];
            }
            if (nVerts > 0) {
                centroid.x /= (float)nVerts;
                centroid.y /= (float)nVerts;
                centroid.z /= (float)nVerts;
            }
            const tinym::vec3 dp{
                now.x - centroid.x,
                now.y - centroid.y,
                now.z - centroid.z
            };
            for (Index vi = 0; vi < nVerts; ++vi) {
                xp[vi*3+0] += static_cast<PR>(dp.x);
                xp[vi*3+1] += static_cast<PR>(dp.y);
                xp[vi*3+2] += static_cast<PR>(dp.z);
                if (xPrev) {
                    xPrev[vi*3+0] += static_cast<PR>(dp.x);
                    xPrev[vi*3+1] += static_cast<PR>(dp.y);
                    xPrev[vi*3+2] += static_cast<PR>(dp.z);
                }
            }
            // D-040 addendum (2026-05-14, "reset-stuck" fix): DO NOT update
            // mesh.transformPosition from the delta-loop. transformPosition
            // is the authorial spawn position (seeded by Scene::pack from
            // mesh.initializer->params.center; explicit edits go through
            // Simulator::translateObject which writes back to the initializer
            // per D-015). Carrying delta updates into transformPosition
            // breaks re-initialize: on the next sim.initialize() the new
            // Bullet body would start at the cube's last rested floor
            // position (with v=0), Bullet's contact solver settles it
            // immediately, and the cube appears frozen on resume. The
            // visible motion of the rigid body comes from state.x writes
            // above; Inspector's Position widget reads transformPosition
            // and stays at the authorial spawn during sim (acceptable —
            // the live position is shown by the render itself).
            m.rigidLastBodyPos = now;
        }

        // Kinematic bodies: prescribed BVH motion, one-way coupling. Once
        // per outer frame (like the Rigid block above): advance playback
        // by the frame step h — motion time IS simulation time, so the
        // pause gate above freezes playback and a slow sim slows the body
        // in lockstep with the cloth it pushes — then rewrite the proxy
        // verts from the FK pose. xPrev gets the PREVIOUS pose (the
        // substep snapshot loop skips Kinematic) so the swept narrow
        // phase sees the true frame motion. The user transform must be
        // re-applied here every frame because these writes replace the
        // pack-baked geometry; params/transformPosition are the same
        // sources pack reads (D-015: inspector edits write back to the
        // initializer).
        for (auto& m : Scene<BE, PR>::meshes) {
            if (m.behaviorType != BehaviorType::Kinematic) continue;
            auto* kin = dynamic_cast<MeshKinematicInitializer<BE, PR>*>(m.initializer);
            if (!kin || !m.state.x.ptr) continue;
            if (kin->playing && kin->motion.valid()) {
                kin->localTime += (double)system.h * kin->playSpeed;
                // Random walks run open-ended (the baker streams frames on
                // demand); single clips and transition composites wrap or
                // clamp over their finite duration as before.
                // Mode 5 (2-motion blend) travels open-ended like a random walk:
                // NOT wrapped, so the absolute per-cycle root accumulate and the
                // relative integrator see a monotone clock.
                const bool walking =
                    (kin->motionMode == 1 && kin->graphActive()) || kin->verbActive();
                const double dur = kin->activeDuration();
                if (!walking) {
                    if (!kin->loop)
                        kin->localTime = std::min(kin->localTime, dur);
                    else if (dur > 0.0)
                        kin->localTime = std::fmod(kin->localTime, dur);
                }
            }
            if (m.state.xPrev.ptr)
                std::memcpy(m.state.xPrev.ptr, m.state.x.ptr,
                            m.state.x.size * sizeof(PR));
            kin->writePose(kin->localTime, m.scale, m.rotationQuat,
                           m.transformPosition, m.state.x.ptr);
        }

        // Push the per-frame sync tier into the broad + narrow phases: InFrame
        // syncs + reads each substep; None/PerFrame run them async.
        collisionPipeline.broadPhase.syncEachPhase = syncEachPhase();
        collisionPipeline.narrowPhase.syncEachPhase = syncEachPhase();

        for(int i = 0; i < system.subSteps; i++) {

            //if(i % 10 == 0) checkCollision = true;
            //else checkCollision = false;
            checkCollision = true;

            if(Scene<BE, PR>::numMeshes > 0 && checkCollision) {
                //MetalGlobalContext::commitAndWait();

                // Substep-cadence gates. Clamp to >=1; both fire at i==0 of
                // every frame (0 % p == 0 for any p) so the BVH is valid
                // before the first query and period==subSteps == once/frame.
                //
                // Temporal-coherence pair reuse (the STABLE "keep previous
                // info" — naive whole-contact reuse explodes because the
                // integrator re-applies a STORED penetration push every
                // substep, accumulating energy):
                //   * doRefit → BVH AABB maintenance every refitP.
                //   * doBroad → re-traverse the BVH to refresh the candidate
                //               vertex-triangle PAIR set every cdP. Between
                //               detections the pair set (broadCollisions) is
                //               HELD: only a fresh broad detect resets
                //               numBroadCollisions, so skipping preserves it.
                //   * narrow  → runs EVERY substep on the held pairs,
                //               recomputing penetration depth+normal from the
                //               CURRENT positions, so the contact push is
                //               always fresh (self-limiting) and the response
                //               is stable. Penetration appears only when the
                //               held pair set stops covering the true contacts
                //               (cloth drifted onto triangles not in the set).
                const Index refitP = (refitSubstepPeriod < 1) ? Index(1) : refitSubstepPeriod;
                const Index cdP    = (cdSubstepPeriod    < 1) ? Index(1) : cdSubstepPeriod;
                const bool doRefit = (i % refitP == 0);
                const bool doBroad = (i % cdP    == 0);

                if (useMultiLevelSH) {
                    // Multi-level (hgrid) spatial hash. Same surface as the
                    // single-level path; detectCollisions rebuilds the grid and
                    // emits its own mlsh_* per-stage scopes. refit() is a no-op.
                    mlBroadPhase.verbose = logSHPerSubstep;
                    if (doRefit) {
                        if (profiler) {
                            auto scope = profiler->scoped("broad_refit");
                            mlBroadPhase.refit();
                        } else {
                            mlBroadPhase.refit();
                        }
                    }
                    if (doBroad) {
                        if (profiler) {
                            auto scope = profiler->scoped("broad_detect");
                            mlBroadPhase.detectCollisions(margin, enableSelfCollisions);
                        } else {
                            mlBroadPhase.detectCollisions(margin, enableSelfCollisions);
                        }
                        if (logSHPerSubstep) {
                            mlBroadPhase.printLastStats(std::cout, frame, i);
                        }
                    }
                } else if (useSpatialHashing) {
                    // Mirror the toggle into the SH instance so detectCollisions
                    // commits the broad-phase dispatch and fills heavy-cell stats.
                    shBroadPhase.verbose = logSHPerSubstep;
                    //shBroadPhase.cellSizeFactor = shCellSizeFactor;
                    if (doRefit) {
                        if (profiler) {
                            auto scope = profiler->scoped("broad_refit");
                            shBroadPhase.refit();
                        } else {
                            shBroadPhase.refit();
                        }
                    }
                    // detectCollisions emits its own per-stage sh_* scopes.
                    if (doBroad) {
                        if (profiler) {
                            auto scope = profiler->scoped("broad_detect");
                            shBroadPhase.detectCollisions(margin, enableSelfCollisions);
                        } else {
                            shBroadPhase.detectCollisions(margin, enableSelfCollisions);
                        }
                        if (logSHPerSubstep) {
                            shBroadPhase.printLastStats(std::cout, frame, i);
                        }
                    }
                } else {
                    if (doRefit) {
                      if (collisionPipeline.broadPhase.fusedRefitEnlarge) {
                        // Fused path (NEW): one swept refit+enlarge call,
                        // replacing the refit()+enlargeTrajectory() pair in the
                        // legacy branch below. One objTree pass + one TLAS build.
                        if (profiler) {
                            auto scope = profiler->scoped("broad_refit_swept");
                            collisionPipeline.broadPhase.refitSwept(system.subh);
                        } else {
                            collisionPipeline.broadPhase.refitSwept(system.subh);
                        }
                      } else {
                        if (profiler) {
                            auto scope = profiler->scoped("broad_refit");
                            collisionPipeline.broadPhase.refit();
                        } else {
                            collisionPipeline.broadPhase.refit();
                        }
                        // Inflate per-mesh AABBs by velocity * subh so a thin
                        // mesh moving a full substep's distance still overlaps
                        // its target's AABB in the broad-phase intersect test.
                        // Without this, a flat cloth (~zero-thickness Y AABB)
                        // crossing a flat ground in one substep is missed —
                        // CM-005's root cause.
                        // YSIM_NO_ENLARGE env: skip the swept-AABB enlarge
                        // pass (profiling experiment). Section column stays in
                        // the CSV at ~0 ms; broad pairs may drop / thin
                        // collisions may be missed (CM-005 trade-off).
                        const bool skipEnlarge = std::getenv("YSIM_NO_ENLARGE") != nullptr;
                        if (profiler) {
                            auto scope = profiler->scoped("broad_enlarge_trajectory");
                            if (!skipEnlarge)
                                collisionPipeline.broadPhase.enlargeTrajectory(system.subh);
                        } else {
                            if (!skipEnlarge)
                                collisionPipeline.broadPhase.enlargeTrajectory(system.subh);
                        }
                      } // end legacy two-pass (refit + enlarge) branch
                    }
                    if (doBroad) {
                        auto& bp = collisionPipeline.broadPhase;
                        auto runDetect = [&]() {
                            if (bp.twoMeshExperiment && bp.clusterVFPipeline)
                                bp.detectCollisionsCluster(margin);
                            else if (bp.twoMeshExperiment)
                                bp.detectCollisionsTwoMesh(margin, enableSelfCollisions);
                            else if (useSegmentedBVHQuery)
                                bp.detectCollisionsSegmented(margin, enableSelfCollisions, useAnalyticPrimitive);
                            else
                                bp.detectCollisions(margin, enableSelfCollisions, useAnalyticPrimitive);
                        };
                        if (profiler) {
                            auto scope = profiler->scoped("broad_detect");
                            runDetect();
                        } else {
                            runDetect();
                        }
                        // Temporal-coherence dump (YSIM_PAIR_DUMP=<path>): one line
                        // per broad CHECK = "<checkIdx> <numPairs> a:b a:b ...". Only
                        // the cluster-VF path has cluster pairs; pairs are coherent
                        // here because queryEnd() commitAndWaits under InFrame.
                        // Offline script computes pair-persistence ratio + run lengths.
                        if (bp.clusterVFPipeline && bp.clusterPairCount.ptr) {
                            static std::ofstream pairDump = []() {
                                std::ofstream f;
                                if (const char* p = std::getenv("YSIM_PAIR_DUMP")) f.open(p);
                                return f;
                            }();
                            static uint64_t checkIdx = 0;
                            if (pairDump.is_open()) {
                                uint32_t np = bp.clusterPairCount[0];
                                if (np > bp.clusterPairCap) np = bp.clusterPairCap;
                                pairDump << checkIdx << ' ' << np;
                                for (uint32_t pi = 0; pi < np; ++pi)
                                    pairDump << ' ' << bp.clusterPairBuf.ptr[pi].a
                                             << ':' << bp.clusterPairBuf.ptr[pi].b;
                                pairDump << '\n';
                                ++checkIdx;
                            }
                        }
                    }
                }

                // Slice (c-2): the analytic narrow path is driven by broad
                // markers that ONLY the BVH broad phase emits. Under the SH
                // broad path no markers exist, so disable analytic there and
                // let spheres fall back to the (correct) triangle-soup
                // narrow_pt_tri path (skipSphere stays 0). BVH path = full
                // analytic. Default broad is BVH, so the toggle behaves as
                // expected in normal use.
                //
                // Narrow runs EVERY substep on the held broad pair set,
                // recomputing contact geometry from current positions → the
                // response stays fresh/stable regardless of cdP.
                const bool analyticNarrow =
                    useAnalyticPrimitive && !useSpatialHashing && !useMultiLevelSH;
                if (profiler) {
                    auto scope = profiler->scoped("narrow_phase");
                    collisionPipeline.narrowPhase.narrowAndSortByVertices(radius, margin, analyticNarrow);
                } else {
                    collisionPipeline.narrowPhase.narrowAndSortByVertices(radius, margin, analyticNarrow);
                }

                // Per-substep collision counters are CPU reads of GPU atomics —
                // they force coherence, so only InFrame pays them. PerFrame is
                // frame-granularity only (no inside-frame breakdown); None has
                // no profiler. This keeps the substep loop async off InFrame.
                if (profiler && syncEachPhase()) {
                    // Broad pairs counted only on a fresh detect (numBroad
                    // persists between, would double-count otherwise); narrow
                    // contacts counted every substep since narrow runs always.
                    auto& packedCol = Scene<BE, PR>::packedCollisionData;
                    profiler->addCollisionCounts(
                        doBroad ? static_cast<uint64_t>(packedCol.numBroadCollisions[0]) : uint64_t(0),
                        static_cast<uint64_t>(packedCol.numNarrowCollisions[0]));
                }
            }

            // Snapshot start-of-substep positions into xPrev so the NEXT
            // substep's CCD narrow check (D-013) sees the prior swept
            // segment instead of a degenerate snapshot. One-substep lag is
            // acceptable for cloth-on-static surfaces: the integrator's
            // contact response in the next substep pushes tunneled
            // particles back before the cloth drifts further than thickness.
            // GPU snapshot (xPrev := x) so the substep loop needs no CPU read
            // of x — None/PerFrame run fully async. Excludes Float (never
            // moves) and Kinematic (keeps its per-FRAME xPrev = previous pose
            // so the swept narrow phase sees real frame motion), matching the
            // old CPU memcpy loop. Rides the current encoder (no sync).
            system.snapshotXPrev(scene);

            if (profiler) {
                auto scope = profiler->scoped("system_update");
                system.update(scene);
            } else {
                system.update(scene);
            }

            // Experiment: PerFrame + perFrameSubstepCommit → flush this substep's
            // command buffer to the GPU without waiting, so the GPU executes it
            // while the CPU encodes the next substep (pipelining) instead of one
            // giant per-frame buffer. The boundary commitAndWait below is the sync.
            if (perFrameSubstepCommit
                && profileLevel == sim_config::ProfileLevel::PerFrame) {
                MetalGlobalContext::commit();
            }
        }

        if (profiler) {
            auto scope = profiler->scoped("metal_commit");
            MetalGlobalContext::commitAndWait();
        } else {
            MetalGlobalContext::commitAndWait();
        }
        // Per-frame collision-counter log is debug output: print only when the
        // profiler is attached (InFrame). PerFrame/None keep the loop silent so
        // stdout isn't a per-frame bottleneck. The commitAndWait above stays
        // unconditionally — preview/render reads state.x on the CPU (req: keep
        // CPU-sync commits, drop only debug-log-driven ones).
        if (profiler) {
            std::cout << Scene<BE, PR>::packedCollisionData.numBroadCollisions[0] << ", "
                << Scene<BE, PR>::packedCollisionData.numNarrowCollisions[0] << '\n';
        }

        system.acctime += system.h;
        frame++;

        // 목표 프레임 수에 도달하면 시뮬레이션을 멈춘다. 진행 바는
        // 100%로 채워진 채 유지되고, 다시 재생을 누르면 위쪽 restart
        // 로직이 frame 0으로 되돌린다.
        {
            const Index tgt = (Index)(targetFrames < 1 ? 1 : targetFrames);
            if (frame >= tgt) pause = true;
        }

        // S3-4: preview is a pure projection of the packed state. The
        // post-update sync is now one call; uploadMeshes() also calls
        // it every render frame so a paused / just-edited scene shows
        // the current state.x without relying on per-edit dual-writes.
        syncPreviewFromState();

        //collisionPipeline.broadPhase.build(sceneObjects.squareClothes[0].x, sceneObjects.squareClothes[0].facet);
        //std::cout << "[Simulator Update] Finished update" << std::endl;
    }

    // S3-4: project the packed state into each request's PreviewState
    // (positions + recomputed normals + render-topology resync). This
    // is THE single place preview is derived from the pack — preview is
    // never an independent source of truth. Called post-update AND every
    // render frame (uploadMeshes), so paused/just-edited scenes show the
    // current state.x without per-edit preview dual-writes. Size-guarded
    // (defensive for future no-preview initializers). Was D-042 R-5/R-7.
    void syncPreviewFromState() {
        for (auto& mesh : Scene<BE, PR>::meshes) {
            if (!mesh.state.x.ptr) continue;
            const size_t stateVerts = (size_t)(mesh.state.x.size / 3);
            for (auto& req : Scene<BE, PR>::requestsGeneralMeshes) {
                if (req.id != mesh.id) continue;
                if (req.preview.numPoints() != stateVerts) break;
                if (req.preview.x.size() < stateVerts * 3) break;
                std::memcpy(req.preview.x.data(),
                            mesh.state.x.ptr,
                            stateVerts * 3 * sizeof(PR));
                req.preview.recomputeNormals();
                req.preview.resyncRenderFromPhysics();
                break;
            }
        }
    }

    // Render-side per-frame mesh upload. Called by the GUI loop, NOT by
    // update() — touching GL (renderState.getOrCreate → glGenVertexArrays)
    // from a non-GL-context process (e.g. the --self-test harness) was the
    // last GL coupling left after D-011. update() is now pure simulation.
    void uploadMeshes() {
        // preview = projection of the packed state, refreshed every
        // render frame (covers paused / post-edit, pause-independent).
        syncPreviewFromState();
        if (profiler) {
            auto scope = profiler->scoped("mesh_upload");
            for(auto& mesh : scene.meshes)
                renderState.getOrCreate(mesh).updateBuffer();
        } else {
            for(auto& mesh : scene.meshes)
                renderState.getOrCreate(mesh).updateBuffer();
        }
    }

    void draw(Program& shader) {
        for(auto& mesh : scene.meshes) {
            // Plane checkerboard render option. Set per mesh (off for all
            // non-plane meshes). The pattern lives in the plane's LOCAL
            // frame so it does not slide when the plane translates: origin =
            // the plane center (transformPosition), and the two in-plane
            // axes (picked from the grid's PlaneDirection) are rotated by the
            // mesh orientation and kept unit-length, so shader.frag's floor()
            // gives 1-world-unit cells (scale enters via the world vertices).
            bool checkerOn = false;
            if (mesh.checkerboard) {
                if (auto* g = dynamic_cast<MeshGridInitializer<BE, PR>*>(mesh.initializer)) {
                    tinym::vec3 u(1.f, 0.f, 0.f), v(0.f, 0.f, 1.f);
                    switch (g->params.dir) {
                        case PlaneDirection::XYPlane: u = tinym::vec3(1.f,0.f,0.f); v = tinym::vec3(0.f,1.f,0.f); break;
                        case PlaneDirection::YZPlane: u = tinym::vec3(0.f,1.f,0.f); v = tinym::vec3(0.f,0.f,1.f); break;
                        case PlaneDirection::XZPlane: u = tinym::vec3(1.f,0.f,0.f); v = tinym::vec3(0.f,0.f,1.f); break;
                    }
                    const auto R = quatToMat3(mesh.rotationQuat);
                    auto rot = [&](const tinym::vec3& a) {
                        return tinym::vec3(R[0]*a.x + R[1]*a.y + R[2]*a.z,
                                           R[3]*a.x + R[4]*a.y + R[5]*a.z,
                                           R[6]*a.x + R[7]*a.y + R[8]*a.z);
                    };
                    shader.setUniform("checkerOrigin", mesh.transformPosition);
                    shader.setUniform("checkerU", rot(u));
                    shader.setUniform("checkerV", rot(v));
                    checkerOn = true;
                }
            }
            shader.setUniform("checkerOn", checkerOn ? 1 : 0);
            // Blend-source tint: a kinematic body in blend playback with the
            // toggle on is colored by mixing its two source-clip colors by the
            // live blend weight (1 → A's color, 0 → B's, crossfade between).
            tinym::vec3 drawColor = mesh.material.baseColor;
            if (auto* kin =
                    dynamic_cast<MeshKinematicInitializer<BE, PR>*>(mesh.initializer)) {
                if (kin->blendColorize && kin->motionMode == 3 &&
                    kin->graphActive() && kin->motionSlots.size() >= 2) {
                    const float wa = kin->graphSession.blendWeightA(kin->localTime);
                    if (wa >= 0.0f) {
                        const auto& a = kin->motionSlots[0].color;
                        const auto& b = kin->motionSlots[1].color;
                        const float wb = 1.0f - wa;
                        drawColor = tinym::vec3(a[0]*wa + b[0]*wb,
                                                a[1]*wa + b[1]*wb,
                                                a[2]*wa + b[2]*wb);
                    }
                }
            }
            auto& gl = renderState.getOrCreate(mesh);
            // Per-object cluster render: when this mesh's clusterRender pill is
            // on and its sub-object BVH has built a per-prim cluster map,
            // multi-draw it colored by cluster instead of the single PBR draw.
            auto& vizTrees = collisionPipeline.broadPhase.objTrees;
            size_t mi = (size_t)(&mesh - &scene.meshes[0]);
            bool clusterViz = mesh.clusterRender && mi < vizTrees.size()
                && vizTrees[mi].groupOfPrim.ptr && vizTrees[mi].numGroups > 0;
            if (clusterViz) {
                gl.buildClusters(vizTrees[mi].groupOfPrim.ptr,
                                 vizTrees[mi].numGroups, vizTrees[mi].numGroups);
                gl.drawClusters(shader, mesh.material.metallic, mesh.material.roughness,
                                mesh.material.specularWeight, mesh.material.emissionColor);
            } else {
                gl.draw(shader,
                    drawColor,
                    mesh.material.metallic,
                    mesh.material.roughness,
                    mesh.material.specularWeight,
                    mesh.material.emissionColor);  // D-028
            }
        }

        // Translucent strobe of the two blend clips (opaque pass done first so
        // ghosts depth-test against the scene but don't write depth).
        drawGhostPreviews(shader);

        if(selectedObj >= 0) {
            // Reserved for a future selected-mesh overlay pass.
        }
    }

    // ID-pass driver: paints the R32I attachment of the caller's idFbo
    // with each mesh's GeneralMesh::id. Used by the per-pixel hover
    // test (cursor → glReadPixels) and by the outline detection in the
    // main shader's pass-2 fragment stage.
    void drawIds(Program& idShader) {
        for(auto& mesh : scene.meshes) {
            renderState.getOrCreate(mesh).drawIdOnly(idShader, mesh.id);
        }
    }

    // Shadow-pass driver: depth-only draw of every mesh into the
    // caller's shadow FBO (caller binds FBO, sets viewport + LightVP).
    void drawDepth() {
        for(auto& mesh : scene.meshes) {
            renderState.getOrCreate(mesh).drawDepthOnly();
        }
    }

    // Point-id pass driver (point selection mode). Caller binds the id
    // FBO, runs a depth-only triangle pre-pass, sets glPointSize +
    // GL_LEQUAL, then calls this. mesh.id == compacted slot so the .r
    // channel doubles as a statesOffsets / findById key.
    void drawPointIds(Program& pointIdShader) {
        for(auto& mesh : scene.meshes) {
            renderState.getOrCreate(mesh).drawPointsIdOnly(pointIdShader, mesh.id);
        }
    }

    // On-screen overlay: every selectable vertex as a black dot, then
    // the hovered vertex (light yellow) and selected vertex (yellow).
    // Caller sets M/V/P on pointShader and enables depth test.
    void drawSelectablePoints(Program& pointShader) {
        constexpr float kDot = 5.0f;
        // 1. Every selectable vertex → black.
        pointShader.setUniform("uColor", tinym::vec3(0.0f, 0.0f, 0.0f));
        glPointSize(kDot);
        for(auto& mesh : scene.meshes)
            renderState.getOrCreate(mesh).drawPoints(pointShader);

        // The next overlays sit exactly on a black dot → LEQUAL so they
        // win the depth tie and paint on top.
        glDepthFunc(GL_LEQUAL);

        // 2. Constrained (pinned) vertices → red, replacing the black
        // dot. fixedVertices lives on the request (physics vid); map to
        // render vertices (identity for grid/sphere/file, renderToPhysics
        // for cube) and redraw each in red.
        pointShader.setUniform("uColor", tinym::vec3(0.9f, 0.1f, 0.1f));
        glPointSize(kDot);
        for (auto& mesh : scene.meshes) {
            auto* req = findRequest(mesh.id);
            if (!req || req->fixedVertices.empty()) continue;
            auto& gl = renderState.getOrCreate(mesh);
            for (const auto& fv : req->fixedVertices) {
                if (req->preview.hasRender()) {
                    for (size_t rv = 0; rv < req->preview.renderToPhysics.size(); ++rv)
                        if (req->preview.renderToPhysics[rv] == fv.vid)
                            gl.drawOnePoint(pointShader, (int)rv);
                } else {
                    gl.drawOnePoint(pointShader, (int)fv.vid);
                }
            }
        }

        // 3. Hover (light yellow) then 4. select (yellow), drawn last so
        // they win over both black and red.
        if (hoveredVertObj >= 0 && hoveredVert >= 0) {
            if (auto* m = Scene<BE, PR>::findById(hoveredVertObj)) {
                pointShader.setUniform("uColor", tinym::vec3(1.0f, 1.0f, 0.55f));
                glPointSize(kDot + 4.0f);
                renderState.getOrCreate(*m).drawOnePoint(pointShader, hoveredVert);
            }
        }
        if (selectedVertObj >= 0 && selectedVert >= 0) {
            if (auto* m = Scene<BE, PR>::findById(selectedVertObj)) {
                pointShader.setUniform("uColor", tinym::vec3(1.0f, 0.8f, 0.0f));
                glPointSize(kDot + 6.0f);
                renderState.getOrCreate(*m).drawOnePoint(pointShader, selectedVert);
            }
        }
        glDepthFunc(GL_LESS);
        glPointSize(1.0f);
    }

    void debugEachBoxes(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        collisionPipeline.broadPhase.showBox();
    }
    void debugSceneBox(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        collisionPipeline.broadPhase.showSceneBox();
    }

    VectorBase<BE, PR> debugSelfCollisionNormals, debugObjCollisionNormals;
    DebugLineGL<CPU> debugSelfCollisions;
    DebugLineGL<CPU> debugObjCollisions;
    void prepareDebugCollisions() {
        typename Scene<BE, PR>::PackedCollisionData& packedCol = Scene<BE, PR>::packedCollisionData;
        typename Scene<BE, PR>::PackedMeshData& packedMesh = Scene<BE, PR>::packedMeshData;
        if(packedCol.numNarrowCollisions[0] <= 0) return;

        if(!debugSelfCollisionNormals.ptr) {
            debugSelfCollisionNormals = VectorBase<BE, PR>(packedCol.maxNumCollisions*6);
            debugObjCollisionNormals = VectorBase<BE, PR>(packedCol.maxNumCollisions*6);
        }

        Index selfBase = 0;
        Index objBase = 0;
        for(Index cid = 0; cid < packedCol.numNarrowCollisions[0]; ++cid) {
            NarrowCollision& nc = packedCol.narrowCollisions[cid];

            auto& packedMesh = Scene<BE, PR>::packedMeshData;
            // Global vertex = statesOffsets[objIndex] + localPointIndex.
            // The old code added objPair.query (the small mesh INDEX, e.g.
            // 1, 2…) instead of obase (statesOffsets[query], the packed
            // vertex base = sum of all prior meshes' point counts). It
            // happened to look right only for object index 0 (obase==0);
            // for any later mesh — e.g. an imported Human at index >= 1 —
            // the debug marker resolved to a wildly wrong packed vertex,
            // which is exactly the "충돌 인덱스가 이상함" symptom. Mirrors
            // the correct convention at narrow grouping (obase + pid) and
            // bruteforce.metal (statesOffsets[objPair.x] + indexPair.x).
            Index obase = packedMesh.statesOffsets[nc.objPair.query];
            Index ppid = obase + nc.indexPair.point;
            tinym::vec3_view v(packedMesh.x.ptr + ppid*3);
            tinym::vec3_view n(nc.collisionNormalAndDistance.v);
            tinym::vec3 t = v+n*.2f;

            if(nc.objPair.query == nc.objPair.target) {
                debugSelfCollisionNormals[selfBase  ] = v[0];
                debugSelfCollisionNormals[selfBase+1] = v[1];
                debugSelfCollisionNormals[selfBase+2] = v[2];
                debugSelfCollisionNormals[selfBase+3] = t[0];
                debugSelfCollisionNormals[selfBase+4] = t[1];
                debugSelfCollisionNormals[selfBase+5] = t[2];
                selfBase += 6;
            }
            else {
                debugObjCollisionNormals[objBase  ] = v[0];
                debugObjCollisionNormals[objBase+1] = v[1];
                debugObjCollisionNormals[objBase+2] = v[2];
                debugObjCollisionNormals[objBase+3] = t[0];
                debugObjCollisionNormals[objBase+4] = t[1];
                debugObjCollisionNormals[objBase+5] = t[2];
                objBase += 6;
            }
        }
        if(selfBase > 0) {
            if(!debugSelfCollisions.vertexPtr) debugSelfCollisions = DebugLineGL<CPU>(selfBase/3, debugSelfCollisionNormals.ptr);
            else debugSelfCollisions.updateBuffer(debugSelfCollisionNormals.ptr, selfBase/3);
        }
        if(objBase > 0) {
            if(!debugObjCollisions.vertexPtr) debugObjCollisions = DebugLineGL<CPU>(objBase/3, debugObjCollisionNormals.ptr);
            else debugObjCollisions.updateBuffer(debugObjCollisionNormals.ptr, objBase/3);
        }
    }
    void showSelfCollisions() {
        if(debugSelfCollisions.vertexNum > 0)
            debugSelfCollisions.draw();
    }
    void showObjCollisions() {
        if(debugObjCollisions.vertexNum > 0)
            debugObjCollisions.draw();
    }

    void debugCollisions(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");

        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        prepareDebugCollisions();
        showSelfCollisions();
        showObjCollisions();
    }

    Program debugLineShader;
    DebugLineGL<CPU> debugLineGL;
    std::vector<tinym::vec3> debugLines;
    void clearDebugLines() { debugLines.clear(); }
    void addDebugLines(tinym::vec3& a, tinym::vec3& b) {
        debugLines.push_back(a);
        debugLines.push_back(b);
    }
    void showDebugLines(tinym::mat4& V, tinym::mat4& P) {
        if(! debugLineShader.programID) debugLineShader.loadShader("line.vert", "line.frag");
        if(! debugLineGL.vao) debugLineGL = DebugLineGL<CPU>(debugLines.size(), (float*)debugLines.data());
        else debugLineGL.updateBuffer((float*)debugLines.data(), debugLines.size());

        debugLineShader.use();
        debugLineShader.setUniform("V", V);
        debugLineShader.setUniform("P", P);
        glLineWidth(2.5f);
        debugLineGL.draw();
    }

    static const char* planeDirectionName(PlaneDirection d) {
        switch (d) {
            case PlaneDirection::XYPlane: return "XYPlane";
            case PlaneDirection::YZPlane: return "YZPlane";
            case PlaneDirection::XZPlane: return "XZPlane";
        }
        return "XZPlane";
    }

    static bool planeDirectionFromName(const std::string& name, PlaneDirection& out) {
        if (name == "XYPlane") { out = PlaneDirection::XYPlane; return true; }
        if (name == "YZPlane") { out = PlaneDirection::YZPlane; return true; }
        if (name == "XZPlane") { out = PlaneDirection::XZPlane; return true; }
        return false;
    }

    scene_format::SceneSnapshot toSnapshot() {
        using namespace scene_format;
        SceneSnapshot s;
        s.environment.gravity = {Scene<BE,PR>::environment.gravity.x,
                                  Scene<BE,PR>::environment.gravity.y,
                                  Scene<BE,PR>::environment.gravity.z};
        s.environment.wind = {Scene<BE,PR>::environment.wind.x,
                               Scene<BE,PR>::environment.wind.y,
                               Scene<BE,PR>::environment.wind.z};
        s.environment.backgroundColor = {
            Scene<BE,PR>::environment.backgroundColor.x,
            Scene<BE,PR>::environment.backgroundColor.y,
            Scene<BE,PR>::environment.backgroundColor.z};

        // Persist the live solver timing (시뮬레이션 환경 panel).
        s.simulation.timePerFrame = (double)system.h;
        s.simulation.subSteps     = (int)system.subSteps;

        auto encodeOne = [&](int id, GeneralMeshInitializer<BE,PR>* init,
                              BehaviorType btype, const BehaviorParams<PR>& bparams,
                              const ::Material& mat, const ::Quat& rot,
                              const tinym::vec3& scale,
                              const tinym::vec3* transformOverride,
                              const std::string& name,
                              bool applyGravity, bool applyWind, bool isStatic,
                              double clothStiffnessScale) {
            Object o;
            o.id = id;
            o.name = name;
            o.applyGravity = applyGravity;
            o.applyWind = applyWind;
            o.isStatic = isStatic;
            o.clothStiffnessScale = clothStiffnessScale;
            o.material.baseColor = {mat.baseColor.x, mat.baseColor.y, mat.baseColor.z};
            o.material.metallic = mat.metallic;
            o.material.roughness = mat.roughness;
            o.material.specularWeight = mat.specularWeight;
            o.material.emissionColor = {mat.emissionColor.x, mat.emissionColor.y, mat.emissionColor.z};

            if (auto* g = dynamic_cast<MeshGridInitializer<BE,PR>*>(init)) {
                o.source.kind = Source::Kind::Primitive;
                o.source.primitive.shape = "grid";
                o.source.primitive.size = (double)g->params.size1D;
                o.source.primitive.tessellation = (int)g->params.particleNum1D;
                o.source.primitive.direction = planeDirectionName(g->params.dir);
                o.source.primitive.mass = (double)g->params.mass;
                o.source.primitive.jiggle = g->params.jiggle;
                o.transform.position = {g->params.center.x, g->params.center.y, g->params.center.z};
            } else if (auto* sp = dynamic_cast<MeshSphereInitializer<BE,PR>*>(init)) {
                o.source.kind = Source::Kind::Primitive;
                o.source.primitive.shape = "sphere";
                o.source.primitive.size = (double)sp->params.size;
                o.source.primitive.tessellation = (int)sp->params.tessellation;
                o.source.primitive.mass = (double)sp->params.mass;
                o.transform.position = {sp->params.center.x, sp->params.center.y, sp->params.center.z};
            } else if (auto* cb = dynamic_cast<MeshCubeInitializer<BE,PR>*>(init)) {
                o.source.kind = Source::Kind::Primitive;
                o.source.primitive.shape = "cube";
                o.source.primitive.size = (double)cb->params.size;
                o.source.primitive.tessellation = (int)cb->params.tessellation;
                o.source.primitive.mass = (double)cb->params.mass;
                o.transform.position = {cb->params.center.x, cb->params.center.y, cb->params.center.z};
            } else if (auto* cy = dynamic_cast<MeshCylinderInitializer<BE,PR>*>(init)) {
                o.source.kind = Source::Kind::Primitive;
                o.source.primitive.shape = "cylinder";
                o.source.primitive.size = (double)cy->params.size;
                o.source.primitive.tessellation = (int)cy->params.tessellation;
                o.source.primitive.mass = (double)cy->params.mass;
                o.transform.position = {cy->params.center.x, cy->params.center.y, cy->params.center.z};
            } else if (auto* f = dynamic_cast<MeshFileInitializer<BE,PR>*>(init)) {
                o.source.kind = Source::Kind::Import;
                std::string p = f->params.prefix;
                if (!p.empty() && p.back() != '/') p.push_back('/');
                o.source.import.path = p + f->params.fileName;
                o.source.import.scale = (double)f->params.scale;
                o.source.import.mass = (double)f->params.mass;
                o.transform.position = {f->params.offset.x, f->params.offset.y, f->params.offset.z};
            } else if (auto* a = dynamic_cast<AssimpMeshFileInitializer<BE,PR>*>(init)) {
                // Sibling of MeshFileInitializer; needs its own case so
                // saveScene persists the imported model's path and position.
                o.source.kind = Source::Kind::Import;
                std::string p = a->params.prefix;
                if (!p.empty() && p.back() != '/') p.push_back('/');
                o.source.import.path = p + a->params.fileName;
                o.source.import.scale = (double)a->params.scale;
                o.source.import.mass = (double)a->params.mass;
                o.transform.position = {a->params.offset.x, a->params.offset.y, a->params.offset.z};
            }
            // Realized-mesh path overrides the initializer-derived position
            // with the live GeneralMesh::transformPosition so BDD-003 edits
            // round-trip through saveScene/loadScene.
            if (transformOverride) {
                o.transform.position = {transformOverride->x,
                                        transformOverride->y,
                                        transformOverride->z};
            }
            o.transform.rotation = {rot.w, rot.x, rot.y, rot.z};
            o.transform.scale = {scale.x, scale.y, scale.z};

            o.behavior.type = behaviorTypeName(btype);
            o.behavior.params = nlohmann::json::object();
            std::visit([&](auto&& p) {
                using P = std::decay_t<decltype(p)>;
                if constexpr (std::is_same_v<P, ClothBehaviorParams<PR>>) {
                    o.behavior.params["stretch"] = p.stretch;
                    o.behavior.params["shear"]   = p.shear;
                    o.behavior.params["bend"]    = p.bend;
                    o.behavior.params["thickness"] = p.thickness;
                } else if constexpr (std::is_same_v<P, FastGridClothBehaviorParams<PR>>) {
                    o.behavior.params["particle_num_1d"] = p.particleNum1D;
                    o.behavior.params["stretch_rest_x"] = p.stretchRestX;
                    o.behavior.params["stretch_rest_y"] = p.stretchRestY;
                    o.behavior.params["shear_rest_a"]   = p.shearRestA;
                    o.behavior.params["shear_rest_b"]   = p.shearRestB;
                    o.behavior.params["bend_rest_x"]    = p.bendRestX;
                    o.behavior.params["bend_rest_y"]    = p.bendRestY;
                    o.behavior.params["k_stretch"]    = p.kstretch;
                    o.behavior.params["k_shear"]      = p.kshear;
                    o.behavior.params["k_bend"]       = p.kbend;
                    o.behavior.params["thickness"]    = p.thickness;
                } else { /* FloatBehaviorParams */ }
            }, bparams);

            s.objects.push_back(std::move(o));
        };

        // Realized meshes (post-pack) take precedence; otherwise pending requests
        // describe the authored intent.
        if (!Scene<BE,PR>::meshes.empty()) {
            for (auto& m : Scene<BE,PR>::meshes) {
                encodeOne(m.id, m.initializer, m.behaviorType, m.behaviorParams,
                          m.material, m.rotationQuat,
                          m.scale,
                          &m.transformPosition,
                          "object_" + std::to_string(m.id),
                          m.applyGravity, m.applyWind, m.isStatic,
                          (double)m.clothStiffnessScale);
            }
        } else {
            for (auto& r : Scene<BE,PR>::requestsGeneralMeshes) {
                // S3-2: request owns material + rotation now.
                encodeOne(r.id, r.initializer, r.behaviorType, r.behaviorParams,
                          r.material, r.rotationQuat,
                          r.scale,
                          nullptr,
                          "object_" + std::to_string(r.id),
                          r.applyGravity, r.applyWind, r.isStatic,
                          (double)r.clothStiffnessScale);
            }
        }
        // Pinned-vertex constraints live on the request (source of
        // truth, survives pack). Match by id and copy onto the snapshot.
        for (auto& o : s.objects) {
            for (auto& r : Scene<BE,PR>::requestsGeneralMeshes) {
                if (r.id != o.id) continue;
                for (const auto& fv : r.fixedVertices) {
                    scene_format::FixedParticle f;
                    f.vid = (int)fv.vid;
                    f.pos = {fv.pos.x, fv.pos.y, fv.pos.z};
                    o.fixedParticles.push_back(f);
                }
                break;
            }
        }
        // Reference-point constraints are scene-level (cross-object
        // capable). objPair/vertexPair already hold object ids + physics
        // vertex ids, so this is a flat copy onto the snapshot.
        for (const auto& c : Scene<BE,PR>::referenceConstraints) {
            scene_format::ReferenceConstraint rc;
            rc.queryObject  = (int)c.objPair.query;
            rc.queryVertex  = (int)c.vertexPair.query;
            rc.targetObject = (int)c.objPair.target;
            rc.targetVertex = (int)c.vertexPair.target;
            s.referenceConstraints.push_back(rc);
        }
        return s;
    }

    bool saveScene(const std::string& path, std::string* error = nullptr) {
        auto snap = toSnapshot();
        std::string localErr;
        bool ok = scene_format::writeToFile(snap, path,
                                            error ? error : &localErr);
        if (ok) {
            scene_log::logSceneIO("씬 저장: " + path);
        } else {
            scene_log::logSceneIO(
                "씬 저장 실패: " + path + " (" +
                (error ? *error : localErr) + ")", false);
        }
        return ok;
    }

    scene_format::Result<scene_format::SceneSnapshot> loadScene(const std::string& path) {
        auto r = scene_format::readFromFile(path);
        if (!r.ok) {
            scene_log::logSceneIO(
                "씬 불러오기 실패: " + path + " (" + r.error.message + ")",
                false);
            return r;
        }
        applySnapshot(r.value, scene_format::sceneDir(path));
        scene_log::logSceneIO("씬 불러오기: " + path + " (오브젝트 " +
            std::to_string(r.value.objects.size()) + "개)");
        return r;
    }

    // Apply a parsed scene snapshot to the live simulator — the single
    // scene-build path shared by GUI File>Load (via loadScene), the CLI
    // --scene flag, and the headless RunConfig builder. `sceneDir` resolves
    // relative import paths (empty when the snapshot has no source file).
    void applySnapshot(const scene_format::SceneSnapshot& snap,
                       const std::string& sceneDir) {
        // BDD-016: only mutate the scene after parse + structural validation succeed.
        // Match the pack()-side ownership convention: meshes are non-owning
        // views over requestsGeneralMeshes' initializer pointers. Clear the
        // views first (without deleting), then free the canonical owner.
        for (auto& m : Scene<BE,PR>::meshes) m.initializer = nullptr;
        Scene<BE,PR>::meshes.clear();
        for (auto& r : Scene<BE,PR>::requestsGeneralMeshes) delete r.initializer;
        Scene<BE,PR>::requestsGeneralMeshes.clear();
        // Scene-boundary churn: drop old reference constraints; the
        // snapshot's are restored after the objects are rebuilt below.
        Scene<BE,PR>::referenceConstraints.clear();
        Scene<BE,PR>::numMeshes = 0;
        // D-041 turn-2: reset id counter at the scene boundary so
        // save/load round-trips assign ids 0, 1, 2, ... in load order.
        Scene<BE,PR>::nextMeshId = 0;
        Scene<BE,PR>::dirty = true;
        // D-042 R-3 (2026-05-14) + turn-38 BLOCK fix-turn: drop both the
        // materialized MeshGL cache AND the pending preview bindings.
        // `getOrCreate(mesh)` checks `state` (materialized map) BEFORE the
        // preview-binding map, so clearing previewBindings alone is not
        // enough — same-process scene reload reuses `state[id]` from the
        // prior scene with stale VAO/VBO counts + pointers into freed
        // PreviewState heap. Both clears together is the safe sweep at
        // scene-boundary churn. Closes R-2 Estimator turn-37 WARNING.
        renderState.clearPreviewBindings();
        renderState.clear();

        // Scene-boundary churn: selection/hover indices are per-runtime
        // scene. The old scene's selectedVert/hoveredVert (vertex ids in
        // the PREVIOUS mesh's vertex space) would otherwise dangle into
        // the freshly loaded scene — drawSelectablePoints' drawOnePoint,
        // the point id-pass and the inspector all index live buffers by
        // these. Reset them so the loaded scene starts with no stale
        // (out-of-range) selection → bad-memory-access crash on load.
        selectedObj = -1;
        selectedVert = -1;
        selectedVertObj = -1;
        hoveredObj = -1;
        hoveredVert = -1;
        hoveredVertObj = -1;
        pointRefPickActive = false;

        Scene<BE,PR>::environment.gravity = tinym::vec3(
            (float)snap.environment.gravity[0],
            (float)snap.environment.gravity[1],
            (float)snap.environment.gravity[2]);
        Scene<BE,PR>::environment.wind = tinym::vec3(
            (float)snap.environment.wind[0],
            (float)snap.environment.wind[1],
            (float)snap.environment.wind[2]);
        Scene<BE,PR>::environment.backgroundColor = tinym::vec3(
            (float)snap.environment.backgroundColor[0],
            (float)snap.environment.backgroundColor[1],
            (float)snap.environment.backgroundColor[2]);

        // Restore solver timing into the live system. r.value.simulation
        // already carries the engine defaults (1/60 s, 60 substeps) when
        // the scene file has no "simulation" block (old scenes), so this
        // is an unconditional assign — backward compat is handled by the
        // loader's default-init, not a branch here. subh must be
        // recomputed (it is the per-substep integrator dt; see the
        // 시뮬레이션 환경 panel for the same resync rationale).
        system.h        = (PR)snap.simulation.timePerFrame;
        system.subSteps = (size_t)(snap.simulation.subSteps > 0
                                   ? snap.simulation.subSteps : 1);
        system.subh     = system.h / (PR)system.subSteps;

        for (auto& o : snap.objects) {
            BehaviorType btype = BehaviorType::Float;
            BehaviorParams<PR> bparams = FloatBehaviorParams<PR>{};
            if (o.behavior.type == "TriangularCloth") {
                btype = BehaviorType::TriangularCloth;
                ClothBehaviorParams<PR> p{};
                p.stretch  = o.behavior.params.value("stretch",  PR(0));
                p.shear    = o.behavior.params.value("shear",    PR(0));
                p.bend     = o.behavior.params.value("bend",     PR(0));
                p.thickness = o.behavior.params.value("thickness", PR(0));
                bparams = p;
            } else if (o.behavior.type == "FastGridCloth") {
                btype = BehaviorType::FastGridCloth;
                FastGridClothBehaviorParams<PR> p{};
                p.particleNum1D = o.behavior.params.value("particle_num_1d", 0u);
                // Back-compat: pre-6-value scenes only stored the single
                // stretch_rest/shear_rest/bend_rest scalars — fan them out
                // to X==Y / A==B. Newer scenes carry all 6 directly.
                PR legacyS = o.behavior.params.value("stretch_rest", PR(0));
                PR legacyShear = o.behavior.params.value("shear_rest", PR(0));
                PR legacyBend  = o.behavior.params.value("bend_rest",  PR(0));
                p.stretchRestX = o.behavior.params.value("stretch_rest_x", legacyS);
                p.stretchRestY = o.behavior.params.value("stretch_rest_y", legacyS);
                p.shearRestA   = o.behavior.params.value("shear_rest_a", legacyShear);
                p.shearRestB   = o.behavior.params.value("shear_rest_b", legacyShear);
                p.bendRestX    = o.behavior.params.value("bend_rest_x", legacyBend);
                p.bendRestY    = o.behavior.params.value("bend_rest_y", legacyBend);
                p.kstretch      = o.behavior.params.value("k_stretch",    PR(0));
                p.kshear        = o.behavior.params.value("k_shear",      PR(0));
                p.kbend         = o.behavior.params.value("k_bend",       PR(0));
                p.thickness     = o.behavior.params.value("thickness",    PR(0));
                bparams = p;
            } else if (o.behavior.type == "Rigid") {
                // D-036 turn-32 addendum: Rigid is tag-set only until
                // slice B-3 wires IRigidPhysicsBackend. No params to
                // read; FloatBehaviorParams placeholder satisfies the
                // variant (BehaviorParams has no Rigid alternative —
                // the simulator's dispatch reads behaviorType, not the
                // variant's alternative, for Rigid-tagged meshes).
                btype = BehaviorType::Rigid;
                bparams = FloatBehaviorParams<PR>{};
            }

            tinym::vec3 pos((float)o.transform.position[0],
                            (float)o.transform.position[1],
                            (float)o.transform.position[2]);

            GeneralMeshInitializer<BE,PR>* init = nullptr;
            if (o.source.kind == scene_format::Source::Kind::Primitive) {
                if (o.source.primitive.shape == "sphere") {
                    init = new MeshSphereInitializer<BE,PR>(MeshSphereInitializerParams<PR>(
                        pos,
                        (Index)o.source.primitive.tessellation,
                        (PR)o.source.primitive.size,
                        (PR)o.source.primitive.mass));
                } else if (o.source.primitive.shape == "cube") {
                    init = new MeshCubeInitializer<BE,PR>(MeshCubeInitializerParams<PR>(
                        pos,
                        (Index)o.source.primitive.tessellation,
                        (PR)o.source.primitive.size,
                        (PR)o.source.primitive.mass));
                } else if (o.source.primitive.shape == "cylinder") {
                    init = new MeshCylinderInitializer<BE,PR>(MeshCylinderInitializerParams<PR>(
                        pos,
                        (Index)o.source.primitive.tessellation,
                        (PR)o.source.primitive.size,
                        (PR)o.source.primitive.mass));
                } else {
                    PlaneDirection dir = PlaneDirection::XZPlane;
                    planeDirectionFromName(o.source.primitive.direction, dir);
                    init = new MeshGridInitializer<BE,PR>(MeshGridInitializerParams<PR>(
                        dir, pos,
                        (Index)o.source.primitive.tessellation,
                        (PR)o.source.primitive.size,
                        (PR)o.source.primitive.mass,
                        o.source.primitive.jiggle,
                        static_cast<uint32_t>(o.id))); // D-018: seed from saved mesh id
                }
            } else {
                std::string resolved = scene_format::resolveImportPath(sceneDir, o.source.import.path);
                std::string prefix, file;
                auto slash = resolved.find_last_of('/');
                if (slash != std::string::npos) {
                    prefix = resolved.substr(0, slash);
                    file = resolved.substr(slash + 1);
                } else {
                    file = resolved;
                }
                init = new MeshFileInitializer<BE,PR>(MeshFileInitializerParams<PR>(
                    prefix, file, pos,
                    (PR)o.source.import.scale,
                    (PR)o.source.import.mass));
            }
            scene.addGeneralMesh(init, btype, bparams);
            registerPreviewBindingForLastRequest();  // D-042 R-2

            if (Scene<BE,PR>::numMeshes > 0) {
                int idx = Scene<BE,PR>::numMeshes - 1;
                if (idx < (int)Scene<BE,PR>::requestsGeneralMeshes.size()) {
                    int meshId = Scene<BE,PR>::requestsGeneralMeshes[idx].id;
                    (void)meshId;
                    // S3-2: material is request-owned; pack copies it
                    // onto the rebuilt mesh (no pendingMaterials defer).
                    Scene<BE,PR>::requestsGeneralMeshes[idx].material = {
                        tinym::vec3((float)o.material.baseColor[0],
                                    (float)o.material.baseColor[1],
                                    (float)o.material.baseColor[2]),
                        (float)o.material.metallic,
                        (float)o.material.roughness,
                        (float)o.material.specularWeight,
                        tinym::vec3((float)o.material.emissionColor[0],
                                    (float)o.material.emissionColor[1],
                                    (float)o.material.emissionColor[2])
                    };
                    // S3-3: the request's initializer params own the
                    // transform; pack bakes scale→rotate→translate from
                    // them deterministically. No pendingRotations defer,
                    // no preview scale pre-bake — pack measures cloth
                    // rest lengths from the already-transformed state.x.
                    ::Quat q;
                    q.w = (float)o.transform.rotation[0];
                    q.x = (float)o.transform.rotation[1];
                    q.y = (float)o.transform.rotation[2];
                    q.z = (float)o.transform.rotation[3];
                    auto& rq = Scene<BE,PR>::requestsGeneralMeshes[idx];
                    tinym::vec3 sc((float)o.transform.scale[0],
                                   (float)o.transform.scale[1],
                                   (float)o.transform.scale[2]);
                    rq.scale = sc;
                    rq.rotationQuat = q;
                    if (rq.initializer) {
                        rq.initializer->getParams()->scale = sc;
                        rq.initializer->getParams()->rotationQuat = q;
                    }
                    // Write env-force toggles directly into the request
                    // (RequestGeneralMesh.applyGravity / applyWind). pack()
                    // will carry them into the realized mesh next
                    // initialize, and they survive reset() identically.
                    Scene<BE,PR>::requestsGeneralMeshes[idx].applyGravity = o.applyGravity;
                    Scene<BE,PR>::requestsGeneralMeshes[idx].applyWind    = o.applyWind;
                    Scene<BE,PR>::requestsGeneralMeshes[idx].isStatic     = o.isStatic;
                    Scene<BE,PR>::requestsGeneralMeshes[idx].clothStiffnessScale =
                        (PR)o.clothStiffnessScale;
                    // Pinned-vertex constraints. Scene::pack re-applies
                    // these into constraints + state.x + preview, so the
                    // pin AND its location are restored on load.
                    auto& reqFV = Scene<BE,PR>::requestsGeneralMeshes[idx].fixedVertices;
                    reqFV.clear();
                    for (const auto& f : o.fixedParticles) {
                        FixedVertex fv;
                        fv.vid = (uint32_t)f.vid;
                        fv.pos = tinym::vec3((float)f.pos[0],
                                             (float)f.pos[1],
                                             (float)f.pos[2]);
                        reqFV.push_back(fv);
                    }
                }
            }
        }
        // Restore scene-level reference-point constraints. Stored as
        // object id + physics vid (same space the integrator consumes),
        // so this is a flat copy back into the Scene-static list.
        for (const auto& rc : snap.referenceConstraints) {
            ReferencePointConstraint c;
            c.objPair.query     = (Index)rc.queryObject;
            c.vertexPair.query  = (Index)rc.queryVertex;
            c.objPair.target    = (Index)rc.targetObject;
            c.vertexPair.target = (Index)rc.targetVertex;
            Scene<BE,PR>::referenceConstraints.push_back(c);
        }
        // Schedule the creation-time auto-snap to run once after the
        // first post-load pack (see initialize()). Only when something
        // was actually restored.
        pendingRefSnap = !Scene<BE,PR>::referenceConstraints.empty();
    }

    // S3-2/S3-3: retired. Material is request-owned (pack copies
    // req.material onto the rebuilt mesh); rotation/scale are baked by
    // pack from the request's initializer params. The pendingMaterials
    // / pendingRotations side-maps and the D-025 deferred re-apply are
    // gone. Kept as an empty no-op so existing call sites
    // (initialize / loadScene flows / harness) compile unchanged.
    void applyPendingMaterials() {}
};


template <typename BE, typename PR>
