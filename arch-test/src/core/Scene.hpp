#pragma once
#include "backend/Backend.hpp"
#include "core/Types.hpp"
#include "core/BehaviorParams.hpp"
#include "core/SimState.hpp"

#include "tinym.hpp"
#include <string>
#include <vector>

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
    SceneEnvironment environment;

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
        Index total = 0;
        for (size_t i = 0; i < objects.size(); ++i) {
            auto& o = objects[i];
            switch (o.kind) {
                case ObjectDesc::Kind::GridCloth: o.vertexCount = Index(o.gridN) * Index(o.gridN); break;
                case ObjectDesc::Kind::Ground:    o.vertexCount = 4; break;
                case ObjectDesc::Kind::FileMesh:  o.vertexCount = 0; break; // OBJ load deferred
            }
            statesOffsets[i] = total;
            total += o.vertexCount;
        }
        statesOffsets[objects.size()] = total;

        s.allocate(total);
        if (total == 0) return;

        bool isCloth = false;
        for (size_t i = 0; i < objects.size(); ++i) {
            auto& o = objects[i];
            isCloth = (o.behavior == BehaviorType::TriangularCloth ||
                       o.behavior == BehaviorType::FastGridCloth);
            Index base = statesOffsets[i];
            seedGeometry(o, s, base, isCloth);
        }
    }

private:
    static void seedGeometry(const ObjectDesc& o, SimState<BE, PR>& s, Index base, bool isCloth) {
        auto put = [&](Index v, PR px, PR py, PR pz) {
            s.x[(base + v) * 3 + 0] = px;
            s.x[(base + v) * 3 + 1] = py;
            s.x[(base + v) * 3 + 2] = pz;
            s.xPrev[(base + v) * 3 + 0] = px;
            s.xPrev[(base + v) * 3 + 1] = py;
            s.xPrev[(base + v) * 3 + 2] = pz;
            // mass: deformable cloth participates; everything else fixed.
            s.m[base + v] = isCloth ? PR(o.cloth.thickness > 0 ? 1 : 1) : PR(0);
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
        } else if (o.kind == ObjectDesc::Kind::Ground) {
            PR h = PR(o.sizeWorld) * PR(0.5);
            put(0, PR(o.origin.x) - h, PR(o.origin.y), PR(o.origin.z) - h);
            put(1, PR(o.origin.x) + h, PR(o.origin.y), PR(o.origin.z) - h);
            put(2, PR(o.origin.x) - h, PR(o.origin.y), PR(o.origin.z) + h);
            put(3, PR(o.origin.x) + h, PR(o.origin.y), PR(o.origin.z) + h);
        }
    }
};
