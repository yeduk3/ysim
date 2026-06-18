#pragma once
#include "backend/Backend.hpp"
#include "core/Types.hpp"
#include "core/BehaviorParams.hpp"
#include "core/SimState.hpp"
#include "core/Topology.hpp"
#include "mesh/ObjLoader.hpp"   // GL-free OBJ load for FileMesh Float colliders

#include "tinym.hpp"
#include <string>
#include <vector>
#include <random>   // deterministic cloth jiggle (D-018)
#include <unordered_map>
#include <algorithm>   // std::max (packed-facet slab sizing)

// Cloth params are sent to the spring/integrate kernels verbatim via setBytes;
// lock the byte layout against physics.metal ClothParams (4 floats, DECISIONS C22).
static_assert(sizeof(ClothBehaviorParams<float>) == 16,
              "ClothBehaviorParams<float> must be byte-identical to metal ClothParams");

// STATIC scene description (DECISIONS C9). Holds per-object blueprints +
// derived topology offsets; NO live pos/vel (those live in SimState).
// realize() seeds the initial SimState from the static blueprint.
//
// ponytail: this pass realizes grid-cloth + ground geometry on the CPU
// (the real layout). File-mesh (OBJ) load + GPU topology pack (facets/
// adjacency for spring forces & BVH) are the documented next ports —
// see PORT_MAP.md (mesh + Scene rows).
struct ObjectDesc {
    enum struct Kind { GridCloth, Ground, FileMesh };
    Index id = 0;
    BehaviorType behavior = BehaviorType::Float;
    ShapeType shape = ShapeType::Mesh;
    bool isStatic = false;
    Material material;
    Kind kind = Kind::FileMesh;
    int gridN = 0;                 // GridCloth: gridN x gridN particles
    float sizeWorld = 1.0f;        // GridCloth / Ground: world extent
    tinym::vec3 origin = tinym::vec3(0.0f);
    float scale = 1.0f;            // FileMesh
    std::string filePath;          // FileMesh
    ClothBehaviorParams<float> cloth{};
    Index vertexCount = 0;         // filled by realize()
};

template <typename BE, typename PR>
struct Scene {
    std::vector<ObjectDesc> objects;
    std::vector<Index> statesOffsets;   // size objects+1, vertex start per mesh
    std::vector<MeshTopology<BE, PR>> topology;  // index-aligned with objects
    SceneEnvironment environment;

    // Scene-wide PACKED facet table (Stage 3 narrow prereq, Gap 1). narrow_pt_tri
    // resolves a target triangle as packedFacets[tri + packedFacetsOffsets[tObjId]]
    // with target-LOCAL vertex ids (resolved against statesOffsets[tObjId] at read
    // time). packedFacets concatenates every object's mesh-local topology[i].facets
    // (3 Index / tri); packedFacetsOffsets is in FACET units (cumulative numFacets,
    // size objects+1). Built once in realize() after topology; whole-buffer binds.
    VectorBase<BE, Index> packedFacets;          // sum(numFacets)*3, mesh-local ids
    VectorBase<BE, Index> packedFacetsOffsets;   // objects+1, facet-unit prefix sum

    Index add(ObjectDesc d) {
        d.id = (Index)objects.size();
        objects.push_back(d);
        return d.id;
    }

    // Compute vertex counts + contiguous offsets, then seed SimState
    // positions and masses. Non-deformable meshes get mass 0 (fixed/
    // kinematic convention) so the integrator leaves them in place.
    void realize(SimState<BE, PR>& s) {
        statesOffsets.assign(objects.size() + 1, 0);
        std::unordered_map<size_t, ObjMesh> objCache;   // FileMesh loads, by index

        Index total = 0;
        for (size_t i = 0; i < objects.size(); ++i) {
            auto& o = objects[i];
            switch (o.kind) {
                case ObjectDesc::Kind::GridCloth:
                    o.vertexCount = Index(o.gridN) * Index(o.gridN);
                    break;
                case ObjectDesc::Kind::Ground:
                    o.vertexCount = 4;
                    break;
                case ObjectDesc::Kind::FileMesh: {
                    // Load NOW so vertexCount is known before allocate (TASK).
                    ObjMesh mesh = loadObjMesh(o.filePath, o.scale, o.origin);
                    o.vertexCount = mesh.vertexCount;     // 0 on failure -> empty slot
                    objCache.emplace(i, std::move(mesh));
                    break;
                }
            }
            statesOffsets[i] = total;
            total += o.vertexCount;
        }
        statesOffsets[objects.size()] = total;

        s.allocate(total);
        if (total == 0) return;
        s.realizeAux(statesOffsets);   // GPU mirror of statesOffsets (integrate slot 18)

        for (size_t i = 0; i < objects.size(); ++i) {
            auto& o = objects[i];
            bool isCloth = (o.behavior == BehaviorType::TriangularCloth ||
                            o.behavior == BehaviorType::FastGridCloth);
            Index base = statesOffsets[i];
            const ObjMesh* cached = nullptr;
            if (auto it = objCache.find(i); it != objCache.end()) cached = &it->second;
            seedGeometry(o, s, base, isCloth, cached);
        }

        // Topology: GridCloth -> full spring build; Float colliders (Ground +
        // FileMesh) -> facets-only build (BVH/narrow targets, no springs).
        // Cloth's spring/BVH path is unchanged; the Float topologies sit ready
        // for the later TLAS/narrow port (free-fall stays the same this pass —
        // Karras12BVH still builds over topology[0] only).
        topology.assign(objects.size(), MeshTopology<BE, PR>{});
        for (size_t i = 0; i < objects.size(); ++i) {
            const auto& o = objects[i];
            if (o.kind == ObjectDesc::Kind::GridCloth && o.gridN > 1) {
                topology[i].build(o.gridN, statesOffsets[i], s);
            } else if (o.kind == ObjectDesc::Kind::Ground) {
                topology[i].buildFacetsOnly(groundFacets());      // 2 triangles
            } else if (o.kind == ObjectDesc::Kind::FileMesh) {
                if (auto it = objCache.find(i);
                    it != objCache.end() && it->second.ok)
                    topology[i].buildFacetsOnly(it->second.facets);
            }
        }

        buildPackedFacets();   // Gap 1: scene-wide packed facet table (narrow slot 5/6)
    }

    // Concatenate every object's mesh-local facets into the scene-wide packed
    // table the narrow kernel binds at slots 5/6 (Gap 1). Offsets are in FACET
    // units (the kernel does packedFacets[tri + packedFacetsOffsets[tObjId]]).
    // Empty objects contribute 0 facets but still advance the prefix sum so the
    // offset array stays index-aligned with objects[].
    void buildPackedFacets() {
        std::vector<Index> offsets(objects.size() + 1, 0);
        Index totalFacets = 0;
        for (size_t i = 0; i < objects.size(); ++i) {
            offsets[i] = totalFacets;
            totalFacets += topology[i].numFacets;
        }
        offsets[objects.size()] = totalFacets;

        packedFacetsOffsets = VectorBase<BE, Index>(objects.size() + 1, Index(0));
        for (size_t i = 0; i < offsets.size(); ++i) packedFacetsOffsets[i] = offsets[i];

        // Always size >= 1 so the buffer is a valid pool allocation even with no
        // collider facets (degenerate scene). Pack 3 mesh-local ids per triangle.
        Index slabIdx = std::max<Index>(totalFacets * 3, 1);
        packedFacets = VectorBase<BE, Index>(slabIdx, Index(0));
        for (size_t i = 0; i < objects.size(); ++i) {
            const auto& topo = topology[i];
            Index base3 = offsets[i] * 3;
            for (Index k = 0; k < topo.numFacets * 3; ++k)
                packedFacets[base3 + k] = topo.facets.ptr[k];   // facets is const here
        }
    }

private:
    static void seedGeometry(const ObjectDesc& o, SimState<BE, PR>& s, Index base,
                             bool isCloth, const ObjMesh* cached) {
        auto put = [&](Index v, PR px, PR py, PR pz) {
            s.x[(base + v) * 3 + 0] = px;
            s.x[(base + v) * 3 + 1] = py;
            s.x[(base + v) * 3 + 2] = pz;
            s.xPrev[(base + v) * 3 + 0] = px;
            s.xPrev[(base + v) * 3 + 1] = py;
            s.xPrev[(base + v) * 3 + 2] = pz;
            // mass: deformable cloth participates; everything else fixed (m=0).
            s.m[base + v] = isCloth ? PR(1) : PR(0);
        };
        if (o.kind == ObjectDesc::Kind::GridCloth) {
            int N = o.gridN;
            PR half = PR(o.sizeWorld) * PR(0.5);
            PR step = (N > 1) ? PR(o.sizeWorld) / PR(N - 1) : PR(0);
            for (int r = 0; r < N; ++r)
                for (int c = 0; c < N; ++c) {
                    Index v = Index(r) * Index(N) + Index(c);
                    put(v, PR(o.origin.x) - half + step * c,
                           PR(o.origin.y),
                           PR(o.origin.z) - half + step * r);
                }
            // JIGGLE (port of MeshGridInitializer main.cpp:1180-1391): break the
            // coplanar degeneracy so springs are live. XZ grid -> Y is the plane
            // normal. Deterministic seed (=o.id, D-018). MUST run before topology
            // build so rest lengths measure the jiggled config (rest stays
            // consistent with the simulated state). Magnitude <=1e-4 << rest edge.
            if (isCloth && N > 1) {
                std::mt19937 rng((uint32_t)o.id);
                std::uniform_real_distribution<PR> jig(PR(0), PR(1.0 / 10000.0));
                for (Index v = 0; v < Index(N) * Index(N); ++v) {
                    PR dy = jig(rng);
                    s.x[(base + v) * 3 + 1] += dy;
                    s.xPrev[(base + v) * 3 + 1] += dy;  // keep xPrev == x at t0
                }
            }
        } else if (o.kind == ObjectDesc::Kind::Ground) {
            PR h = PR(o.sizeWorld) * PR(0.5);
            put(0, PR(o.origin.x) - h, PR(o.origin.y), PR(o.origin.z) - h);
            put(1, PR(o.origin.x) + h, PR(o.origin.y), PR(o.origin.z) - h);
            put(2, PR(o.origin.x) - h, PR(o.origin.y), PR(o.origin.z) + h);
            put(3, PR(o.origin.x) + h, PR(o.origin.y), PR(o.origin.z) + h);
        } else if (o.kind == ObjectDesc::Kind::FileMesh) {
            if (!cached || !cached->ok) return;          // load failed -> nothing
            for (Index v = 0; v < cached->vertexCount; ++v)
                put(v, PR(cached->positions[v].x),
                       PR(cached->positions[v].y),
                       PR(cached->positions[v].z));
        }
    }

    // 4-vert XZ quad -> 2 triangles. v0=(-,-) v1=(+,-) v2=(-,+) v3=(+,+).
    // Winding is irrelevant: narrow_pt_tri auto-orients the contact normal
    // toward the query point (one-way Float collider). Matches the grid
    // even-cell rule addFacet(p00,p01,p11)/addFacet(p00,p11,p10).
    static std::vector<Index> groundFacets() {
        return { 0, 2, 3,   0, 3, 1 };
    }
};
