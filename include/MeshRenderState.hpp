#ifndef YSIM_MESH_RENDER_STATE_HPP
#define YSIM_MESH_RENDER_STATE_HPP

// Renderer-side container for per-mesh GL state, keyed by mesh.id.
// Replaces the `MeshGL<CPU> meshGL` field that used to sit on
// `GeneralMesh<BE,PR>` (D-011) — moving this out lets `Simulator::initialize`
// run without an active GL context, which was the structural blocker
// behind CM-002, CM-003, CM-004, and the persistence-slice WARNING.
//
// D-042 R-2 (2026-05-14): MeshGL's bound pointer migrates from packed
// sub-views (volatile across Scene::pack reallocations) to PreviewState
// heap-owned per-mesh buffers (stable). `registerPreviewBinding` is the
// CPU-only API that addX-time code uses to publish the stable pointer
// set; `getOrCreate` consumes the pending binding when first materializing
// the MeshGL inside a live GL context. The legacy packed-sub-view fallback
// remains for any code path that bypasses the binding step (paranoia /
// future hand-built MeshGL via `getOrCreate`).

#include <cstdint>
#include <cstddef>
#include <unordered_map>

#include "MeshGL.hpp"

class MeshRenderState {
public:
    // D-042 R-2: pending preview-binding metadata published at addX time.
    // void*-typed so the class stays non-templated (only `Simulator<METAL,
    // float>` exists in v1 but the API should not lock in PR).
    struct PreviewBinding {
        void* xPtr = nullptr;
        std::size_t numVerts = 0;
        std::uint32_t* facetPtr = nullptr;
        std::size_t numFacets = 0;
        void* normalPtr = nullptr;
    };

    // D-042 R-2: publish a stable preview-pointer binding for mesh `id`.
    // Idempotent (last-write-wins) so addX after removeMesh on the same
    // slot (would only happen if D-041 turn-2 nextMeshId monotone guarantee
    // were violated) overwrites cleanly. No GL calls — safe to call from
    // the harness without a live GL context.
    template <typename PR>
    void registerPreviewBinding(int id, PR* xPtr, std::size_t numVerts,
                                std::uint32_t* facetPtr, std::size_t numFacets,
                                PR* normalPtr) {
        PreviewBinding b;
        b.xPtr = (void*)xPtr;
        b.numVerts = numVerts;
        b.facetPtr = facetPtr;
        b.numFacets = numFacets;
        b.normalPtr = (void*)normalPtr;
        previewBindings[id] = b;
    }

    // D-042 R-2: read-only accessor used by Block 37 + future harness probes
    // to verify the binding registration is eager + correct. Returns nullptr
    // when no binding is registered for `id` (or when it was already consumed
    // by a prior `getOrCreate(mesh)` call inside a live GL context).
    const PreviewBinding* previewBinding(int id) const {
        auto it = previewBindings.find(id);
        return (it == previewBindings.end()) ? nullptr : &it->second;
    }

    // D-042 R-2: drop both the pending preview binding AND any materialized
    // MeshGL for `id`. Called from Simulator::removeMesh so the map doesn't
    // grow unboundedly after repeated remove/add cycles.
    void removeById(int id) {
        previewBindings.erase(id);
        state.erase(id);
    }

    template <typename Mesh>
    MeshGL<CPU>& getOrCreate(Mesh& mesh) {
        auto it = state.find(mesh.id);
        if (it != state.end()) return it->second;

        // D-042 R-2: prefer the PreviewState binding (heap-stable) over the
        // packed sub-view (volatile across Scene::pack reallocations) when
        // first materializing the MeshGL. The pending binding entry is
        // consumed so subsequent lookups hit the cached MeshGL.
        auto pit = previewBindings.find(mesh.id);
        if (pit != previewBindings.end()) {
            auto& pb = pit->second;
            it = state.emplace(std::piecewise_construct,
                std::forward_as_tuple(mesh.id),
                std::forward_as_tuple(
                    pb.numVerts,
                    (float*)pb.xPtr,
                    pb.numFacets,
                    pb.facetPtr,
                    (float*)pb.normalPtr
                )).first;
            previewBindings.erase(pit);
            return it->second;
        }

        // Legacy fallback: construct from packed sub-views (pre-R-2 path).
        // D-042 R-7 (2026-05-14) audit: every Simulator::addX wrapper +
        // loadScene calls registerPreviewBinding BEFORE any MeshGL can be
        // materialized, so in production this branch is effectively dead
        // code — reachable only by a hand-built MeshGL construction that
        // bypasses the addX path (e.g., a future test harness or third-
        // party integration). Retained for safety + parallel-symbol
        // invariant. R-7 cleanup chose comment-only documentation over
        // removal: keeping the path makes accidental future bypass a
        // graceful degradation rather than a NULL deref.
        it = state.emplace(std::piecewise_construct,
            std::forward_as_tuple(mesh.id),
            std::forward_as_tuple(
                mesh.state.x.size / 3,
                mesh.state.x.ptr,
                mesh.adjacency.facets.size / 3,
                mesh.adjacency.facets.ptr,
                mesh.state.n.ptr
            )).first;
        return it->second;
    }

    void clear() { state.clear(); }
    bool has(int id) const { return state.find(id) != state.end(); }

    // D-042 R-3 (2026-05-14): drop all pending preview bindings without
    // touching the materialized MeshGL map. Called at scene-boundary churn
    // (Simulator::loadScene + future production "New Scene" path) so stale
    // bindings (pointing into a freed PreviewState heap from a prior
    // scene's requestsGeneralMeshes) can't be picked up by the next
    // getOrCreate(mesh) call. Folds R-2 Estimator turn-37 WARNING.
    void clearPreviewBindings() { previewBindings.clear(); }

private:
    std::unordered_map<int, MeshGL<CPU>> state;
    std::unordered_map<int, PreviewBinding> previewBindings;
};

#endif  // YSIM_MESH_RENDER_STATE_HPP
