#ifndef YSIM_MESH_RENDER_STATE_HPP
#define YSIM_MESH_RENDER_STATE_HPP

// Renderer-side container for per-mesh GL state, keyed by mesh.id.
// Replaces the `MeshGL<CPU> meshGL` field that used to sit on
// `GeneralMesh<BE,PR>` (D-011) — moving this out lets `Simulator::initialize`
// run without an active GL context, which was the structural blocker
// behind CM-002, CM-003, CM-004, and the persistence-slice WARNING.
//
// Lazy: `getOrCreate(mesh)` constructs the GL state on first encounter
// using duck-typed accessors (`mesh.id`, `mesh.state.x.{ptr,size}`,
// `mesh.state.n.ptr`, `mesh.adjacency.facets.{ptr,size}`). The renderer
// then forwards `updateBuffer` / `draw` to the cached MeshGL each frame.
//
// `clear()` should be called whenever Scene::pack() reallocates packed
// buffers — the cached MeshGL captured raw pointers into the *previous*
// pack's storage and is no longer valid. Simulator::initialize is the
// canonical site for that call.

#include <unordered_map>

#include "MeshGL.hpp"

class MeshRenderState {
public:
    template <typename Mesh>
    MeshGL<CPU>& getOrCreate(Mesh& mesh) {
        auto it = state.find(mesh.id);
        if (it == state.end()) {
            it = state.emplace(std::piecewise_construct,
                std::forward_as_tuple(mesh.id),
                std::forward_as_tuple(
                    mesh.state.x.size / 3,
                    mesh.state.x.ptr,
                    mesh.adjacency.facets.size / 3,
                    mesh.adjacency.facets.ptr,
                    mesh.state.n.ptr
                )).first;
        }
        return it->second;
    }

    void clear() { state.clear(); }
    bool has(int id) const { return state.find(id) != state.end(); }

private:
    std::unordered_map<int, MeshGL<CPU>> state;
};

#endif  // YSIM_MESH_RENDER_STATE_HPP
