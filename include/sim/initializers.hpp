#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

enum struct PlaneDirection : Index {
    XYPlane,
    YZPlane,
    XZPlane,
};

//! Special class for grid cloth
template <typename PR>
struct MeshGridInitializerParams : InitializerParams<PR> {

    // Specifics
    Index particleNum1D;
    PR size1D;
    bool jiggle;
    PlaneDirection dir;
    tinym::vec3 center;
    // Per-mesh RNG seed for jiggle; must be deterministic across runs of the
    // same scene (D-018). Production wires this from mesh.id (addCloth reads
    // Scene::numMeshes pre-call; loadScene passes o.id). Value is irrelevant
    // when jiggle == false — addGround leaves it at the default.
    uint32_t seed;

    MeshGridInitializerParams(PlaneDirection dir, tinym::vec3 center, Index particleNum1D, PR size1D, PR mass, bool jiggle, uint32_t seed = 0)
        : dir(dir), center(center),
        particleNum1D(particleNum1D),
        InitializerParams<PR>(
                particleNum1D*particleNum1D, // numPoints
                2*(particleNum1D-1)*(particleNum1D-1), // numFacets
                2*(particleNum1D-1)*particleNum1D+2*(particleNum1D-1)*(particleNum1D-1), // numEdges
                mass),
        size1D(size1D), jiggle(jiggle), seed(seed) {}
};

template <typename BE, typename PR>
struct MeshGridInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshGridInitializerParams<PR>;
    ParamsType params;

    MeshGridInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        std::cout << "[MeshGridSpringInitializer initialize] start" << std::endl;

        state.memoryAllocation(params); // numPoints
        adjacency.memoryAllocation(params); // numPoints, numFacets, numEdges

        PR halfSize = params.size1D / 2.0;
        PR length = params.size1D/PR(params.particleNum1D-1);

        // D-018: per-mesh seeded RNG. Two runs of the same scene produce
        // bit-identical jiggle because the seed is derived from mesh.id
        // (preserved across save/load). Replaces global rand() (CM-007,
        // graduated to OLD_MISTAKES).
        std::mt19937 rng(params.seed);
        std::uniform_real_distribution<PR> jiggleDist(PR(0), PR(1.0/10000.0));

        for (int row = 0; row < params.particleNum1D; ++row) {
            for (int col = 0; col < params.particleNum1D; ++col) {
                int base = (row*params.particleNum1D + col)*3;

                PR px =  col*length - halfSize;
                PR py = -row*length + halfSize;
                PR pz = params.jiggle ? jiggleDist(rng) : PR(0);
                
                switch(params.dir) {
                    case PlaneDirection::XYPlane:
                        state.x[base  ] = px+params.center.x;
                        state.x[base+1] = py+params.center.y;
                        state.x[base+2] = pz+params.center.z;
                        break;
                    case PlaneDirection::YZPlane:
                        state.x[base  ] = pz+params.center.x;
                        state.x[base+1] = px+params.center.y;
                        state.x[base+2] = -py+params.center.z;
                        break;
                    case PlaneDirection::XZPlane:
                        state.x[base  ] = px+params.center.x;
                        state.x[base+1] = pz+params.center.y;
                        state.x[base+2] = -py+params.center.z;
                        break;
                    default: break;
                }
            }
        }
        std::cout << "[MeshGridSpringInitializer initialize] position set" << std::endl;

        if(adjacency.vertexAdjFacets.ptr) return;

        Index fIdx = 0;
        for (Index row = 0; row < params.particleNum1D - 1; ++row) {
            for (Index col = 0; col < params.particleNum1D - 1; ++col) {
                Index p00 = (row * params.particleNum1D + col);
                Index p10 = (row * params.particleNum1D + col + 1);
                Index p01 = ((row + 1) * params.particleNum1D + col);
                Index p11 = ((row + 1) * params.particleNum1D + col + 1);
                // p00   p10
                //
                // p01   p11

                auto addFacet = [&](Index a, Index b, Index c) {
                    adjacency.facets[fIdx++] = a;
                    adjacency.facets[fIdx++] = b;
                    adjacency.facets[fIdx++] = c;
                };

                if (((row + col) & 1) == 0) { // even
                    // diagonal: p00 - p11
                    addFacet(p00, p01, p11);
                    addFacet(p00, p11, p10);
                } else { // odd
                    // diagonal: p10 - p01
                    addFacet(p00, p01, p10);
                    addFacet(p10, p01, p11);
                }
            }
        }
        std::cout << "[MeshGridSpringInitializer initialize] facets set" << std::endl;


        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        // Mirror the grid generation above into preview's std::vector<>
        // buffers. Match facet topology (even/odd diagonal alternation)
        // so MeshGL renders the same triangulation the simulator sees
        // post-pack. Jiggle is omitted from preview (it's a per-substep
        // micro-perturbation; visual rendering doesn't need it and the
        // simulator will re-apply on pack).
        const PR halfSize = params.size1D / PR(2);
        const PR length = params.size1D / PR(params.particleNum1D - 1);
        const Index nPts = params.numPoints;
        const Index nFacets = params.numFacets;
        preview.x.assign(nPts * 3, PR(0));
        for (int row = 0; row < params.particleNum1D; ++row) {
            for (int col = 0; col < params.particleNum1D; ++col) {
                const int base = (row * params.particleNum1D + col) * 3;
                const PR px =  col * length - halfSize;
                const PR py = -row * length + halfSize;
                switch (params.dir) {
                    case PlaneDirection::XYPlane:
                        preview.x[base  ] = px + (PR)params.center.x;
                        preview.x[base+1] = py + (PR)params.center.y;
                        preview.x[base+2] = (PR)params.center.z;
                        break;
                    case PlaneDirection::YZPlane:
                        preview.x[base  ] = (PR)params.center.x;
                        preview.x[base+1] = px + (PR)params.center.y;
                        preview.x[base+2] = -py + (PR)params.center.z;
                        break;
                    case PlaneDirection::XZPlane:
                        preview.x[base  ] = px + (PR)params.center.x;
                        preview.x[base+1] = (PR)params.center.y;
                        preview.x[base+2] = -py + (PR)params.center.z;
                        break;
                    default: break;
                }
            }
        }
        preview.facets.assign(nFacets * 3, 0);
        Index fIdx = 0;
        auto addFacet = [&](uint32_t a, uint32_t b, uint32_t c) {
            preview.facets[fIdx++] = a;
            preview.facets[fIdx++] = b;
            preview.facets[fIdx++] = c;
        };
        for (Index row = 0; row < params.particleNum1D - 1; ++row) {
            for (Index col = 0; col < params.particleNum1D - 1; ++col) {
                const uint32_t p00 = (uint32_t)(row * params.particleNum1D + col);
                const uint32_t p10 = (uint32_t)(row * params.particleNum1D + col + 1);
                const uint32_t p01 = (uint32_t)((row + 1) * params.particleNum1D + col);
                const uint32_t p11 = (uint32_t)((row + 1) * params.particleNum1D + col + 1);
                if (((row + col) & 1) == 0) {
                    addFacet(p00, p01, p11);
                    addFacet(p00, p11, p10);
                } else {
                    addFacet(p00, p01, p10);
                    addFacet(p10, p01, p11);
                }
            }
        }
        preview.recomputeNormals();
    }

    // Deterministic cloth jiggle, applied at Scene::pack time ONLY when
    // the mesh's current behavior is a cloth type. A perfectly coplanar
    // sheet is degenerate for the cloth solver / point-triangle narrow
    // phase (zero out-of-plane stiffness, coplanar self-contacts with
    // garbage indices); a sub-visible (<=1e-4) normal-axis perturbation
    // breaks the degeneracy. This is NOT the constructor `params.jiggle`
    // flag path (that writes the initializer regen which Scene::pack's
    // R-3 memcpy then clobbers with the flat preview, so it never
    // reached the sim). Here we perturb the post-memcpy state.x directly.
    //
    // Idempotent / accumulation-free by construction: pack always calls
    // this on the freshly memcpy'd FLAT preview, never on an already-
    // jiggled buffer, and the RNG is reseeded from (params.seed ^ id)
    // every call. So Rigid pack -> flat state.x; Cloth pack -> flat +
    // identical deterministic noise. Toggling Rigid<->Cloth any number
    // of times reproduces the exact same cloth rest configuration (the
    // recomputeRestLengths call right after measures this jiggled
    // state, so rest length stays consistent with what is simulated).
    void applyClothJiggle(PR* x, Index numPoints, uint32_t idSeed) {
        if (!x || numPoints <= 0) return;
        int axis; // plane-normal component, mirrors initialize()'s pz slot
        switch (params.dir) {
            case PlaneDirection::XYPlane: axis = 2; break; // z
            case PlaneDirection::YZPlane: axis = 0; break; // x
            case PlaneDirection::XZPlane: axis = 1; break; // y
            default:                      axis = 1; break;
        }
        std::mt19937 rng(params.seed ^ idSeed);
        std::uniform_real_distribution<PR> jiggleDist(PR(0), PR(1.0/10000.0));
        for (Index p = 0; p < numPoints; ++p)
            x[p*3 + axis] += jiggleDist(rng);
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

template <typename PR>
struct MeshFileInitializerParams : InitializerParams<PR> {
    // Specifics
    std::string prefix, fileName;
    tinym::vec3 offset;
    PR scale;

    MeshFileInitializerParams(std::string prefix, std::string fileName, tinym::vec3 offset, PR scale, PR mass) 
        : prefix(prefix), fileName(fileName), offset(offset), scale(scale), InitializerParams<PR>(0,0,0,mass) {}
};

template <typename BE, typename PR>
struct MeshFileInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshFileInitializerParams<PR>;
    ParamsType params;
    ObjData data;

    MeshFileInitializer(ParamsType params) : params(params) {
        data.loadObject(params.prefix, params.fileName);

        this->params.numPoints = data.nVertices;
        this->params.numFacets = data.nElements3;

        std::set<std::pair<int,int>> edges;
        for (const auto& face : data.elements3) {
            int n = 3;
            for (int i = 0; i < n; ++i) {
                int a = face[i];
                int b = face[(i + 1) % n];

                if (a > b) std::swap(a, b);
                edges.insert({a, b});
            }
        }
        this->params.numEdges = edges.size();
    }

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params); // numPoints
        adjacency.memoryAllocation(params); // numPoints, numFacets, numEdges

        for(Index vid = 0; vid < params.numPoints; vid++) {
            Index vbase = vid*3;
            state.x[vbase  ] = data.vertices[vid].x*params.scale + params.offset.x;
            state.x[vbase+1] = data.vertices[vid].y*params.scale + params.offset.y;
            state.x[vbase+2] = data.vertices[vid].z*params.scale + params.offset.z;
        }

        if(adjacency.vertexAdjFacets.ptr) return;
        for(Index fid = 0; fid < params.numFacets; fid++) {
            Index fbase = fid*3;
            adjacency.facets[fbase  ] = data.elements3[fid].x;
            adjacency.facets[fbase+1] = data.elements3[fid].y;
            adjacency.facets[fbase+2] = data.elements3[fid].z;
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        const Index nPts = (Index)params.numPoints;
        const Index nFacets = (Index)params.numFacets;
        preview.x.assign(nPts * 3, PR(0));
        for (Index vid = 0; vid < nPts; ++vid) {
            const Index vbase = vid * 3;
            preview.x[vbase    ] = (PR)data.vertices[vid].x * params.scale + (PR)params.offset.x;
            preview.x[vbase + 1] = (PR)data.vertices[vid].y * params.scale + (PR)params.offset.y;
            preview.x[vbase + 2] = (PR)data.vertices[vid].z * params.scale + (PR)params.offset.z;
        }
        preview.facets.assign(nFacets * 3, 0);
        for (Index fid = 0; fid < nFacets; ++fid) {
            const Index fbase = fid * 3;
            preview.facets[fbase    ] = (uint32_t)data.elements3[fid].x;
            preview.facets[fbase + 1] = (uint32_t)data.elements3[fid].y;
            preview.facets[fbase + 2] = (uint32_t)data.elements3[fid].z;
        }
        preview.recomputeNormals();
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

// Parallel to MeshFileInitializer (objreader/ObjData::loadObject path) but
// loads via Assimp (any format, always triangulated). Reuses the same
// MeshFileInitializerParams and the same ObjData -> MeshState/MeshAdjacency
// conversion so downstream is unchanged. Added (not modifying the OBJ path).
template <typename BE, typename PR>
struct AssimpMeshFileInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshFileInitializerParams<PR>;
    ParamsType params;
    ObjData data;

    AssimpMeshFileInitializer(ParamsType params) : params(params) {
        std::string err;
        if (!loadModelWithAssimp(params.prefix, params.fileName, data, &err)) {
            std::cerr << err << std::endl;
            // data stays empty -> numPoints/numFacets = 0 (graceful, like a
            // failed loadObject); importModel() probes the file beforehand.
        }

        this->params.numPoints = data.nVertices;
        this->params.numFacets = data.nElements3;

        std::set<std::pair<int,int>> edges;
        for (const auto& face : data.elements3) {
            int n = 3;
            for (int i = 0; i < n; ++i) {
                int a = face[i];
                int b = face[(i + 1) % n];

                if (a > b) std::swap(a, b);
                edges.insert({a, b});
            }
        }
        this->params.numEdges = edges.size();
    }

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params); // numPoints
        adjacency.memoryAllocation(params); // numPoints, numFacets, numEdges

        for(Index vid = 0; vid < params.numPoints; vid++) {
            Index vbase = vid*3;
            state.x[vbase  ] = data.vertices[vid].x*params.scale + params.offset.x;
            state.x[vbase+1] = data.vertices[vid].y*params.scale + params.offset.y;
            state.x[vbase+2] = data.vertices[vid].z*params.scale + params.offset.z;
        }

        if(adjacency.vertexAdjFacets.ptr) return;
        for(Index fid = 0; fid < params.numFacets; fid++) {
            Index fbase = fid*3;
            adjacency.facets[fbase  ] = data.elements3[fid].x;
            adjacency.facets[fbase+1] = data.elements3[fid].y;
            adjacency.facets[fbase+2] = data.elements3[fid].z;
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        const Index nPts = (Index)params.numPoints;
        const Index nFacets = (Index)params.numFacets;
        preview.x.assign(nPts * 3, PR(0));
        for (Index vid = 0; vid < nPts; ++vid) {
            const Index vbase = vid * 3;
            preview.x[vbase    ] = (PR)data.vertices[vid].x * params.scale + (PR)params.offset.x;
            preview.x[vbase + 1] = (PR)data.vertices[vid].y * params.scale + (PR)params.offset.y;
            preview.x[vbase + 2] = (PR)data.vertices[vid].z * params.scale + (PR)params.offset.z;
        }
        preview.facets.assign(nFacets * 3, 0);
        for (Index fid = 0; fid < nFacets; ++fid) {
            const Index fbase = fid * 3;
            preview.facets[fbase    ] = (uint32_t)data.elements3[fid].x;
            preview.facets[fbase + 1] = (uint32_t)data.elements3[fid].y;
            preview.facets[fbase + 2] = (uint32_t)data.elements3[fid].z;
        }
        preview.recomputeNormals();
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

#include "primitive_geometry.hpp"

template <typename PR>
struct MeshSphereInitializerParams : InitializerParams<PR> {
    Index tessellation;
    PR size;
    tinym::vec3 center;

    MeshSphereInitializerParams(tinym::vec3 center, Index tessellation, PR size, PR mass)
        : InitializerParams<PR>(
              primitive::sphereVertexCount((int)tessellation),
              primitive::sphereFacetCount((int)tessellation),
              primitive::sphereEdgeCount((int)tessellation),
              mass),
          tessellation(tessellation), size(size), center(center) {}
};

template <typename BE, typename PR>
struct MeshSphereInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshSphereInitializerParams<PR>;
    ParamsType params;

    MeshSphereInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params);
        adjacency.memoryAllocation(params);

        auto geom = primitive::sphere(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});

        for (Index v = 0; v < params.numPoints; ++v) {
            Index vbase = v * 3;
            state.x[vbase    ] = (PR)geom.positions[vbase    ];
            state.x[vbase + 1] = (PR)geom.positions[vbase + 1];
            state.x[vbase + 2] = (PR)geom.positions[vbase + 2];
        }

        if (adjacency.vertexAdjFacets.ptr) return;
        for (Index f = 0; f < params.numFacets; ++f) {
            Index fbase = f * 3;
            adjacency.facets[fbase    ] = geom.facets[fbase    ];
            adjacency.facets[fbase + 1] = geom.facets[fbase + 1];
            adjacency.facets[fbase + 2] = geom.facets[fbase + 2];
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        auto geom = primitive::sphere(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});
        preview.x.assign(geom.positions.begin(), geom.positions.end());
        preview.facets.assign(geom.facets.begin(), geom.facets.end());
        preview.recomputeNormals();
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

template <typename PR>
struct MeshCylinderInitializerParams : InitializerParams<PR> {
    Index tessellation;
    PR size;
    tinym::vec3 center;

    MeshCylinderInitializerParams(tinym::vec3 center, Index tessellation, PR size, PR mass)
        : InitializerParams<PR>(
              primitive::cylinderVertexCount((int)tessellation),
              primitive::cylinderFacetCount((int)tessellation),
              primitive::cylinderEdgeCount((int)tessellation),
              mass),
          tessellation(tessellation), size(size), center(center) {}
};

template <typename BE, typename PR>
struct MeshCylinderInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshCylinderInitializerParams<PR>;
    ParamsType params;

    MeshCylinderInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params);
        adjacency.memoryAllocation(params);

        auto geom = primitive::cylinder(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});

        for (Index v = 0; v < params.numPoints; ++v) {
            Index vbase = v * 3;
            state.x[vbase    ] = (PR)geom.positions[vbase    ];
            state.x[vbase + 1] = (PR)geom.positions[vbase + 1];
            state.x[vbase + 2] = (PR)geom.positions[vbase + 2];
        }

        if (adjacency.vertexAdjFacets.ptr) return;
        for (Index f = 0; f < params.numFacets; ++f) {
            Index fbase = f * 3;
            adjacency.facets[fbase    ] = geom.facets[fbase    ];
            adjacency.facets[fbase + 1] = geom.facets[fbase + 1];
            adjacency.facets[fbase + 2] = geom.facets[fbase + 2];
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        auto geom = primitive::cylinder(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});
        preview.x.assign(geom.positions.begin(), geom.positions.end());
        preview.facets.assign(geom.facets.begin(), geom.facets.end());
        preview.recomputeNormals();
    }

    InitializerParams<PR>* getParams() override { return &params; }
};

template <typename PR>
struct MeshCubeInitializerParams : InitializerParams<PR> {
    Index tessellation;
    PR size;
    tinym::vec3 center;

    MeshCubeInitializerParams(tinym::vec3 center, Index tessellation, PR size, PR mass)
        : InitializerParams<PR>(
              primitive::cubeVertexCount((int)tessellation),
              primitive::cubeFacetCount((int)tessellation),
              primitive::cubeEdgeCount((int)tessellation),
              mass),
          tessellation(tessellation), size(size), center(center) {}
};

template <typename BE, typename PR>
struct MeshCubeInitializer : GeneralMeshInitializer<BE, PR> {
    using ParamsType = MeshCubeInitializerParams<PR>;
    ParamsType params;

    MeshCubeInitializer(ParamsType params) : params(params) {}

    void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) override {
        state.memoryAllocation(params);
        adjacency.memoryAllocation(params);

        auto geom = primitive::cube(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});

        for (Index v = 0; v < params.numPoints; ++v) {
            Index vbase = v * 3;
            state.x[vbase    ] = (PR)geom.positions[vbase    ];
            state.x[vbase + 1] = (PR)geom.positions[vbase + 1];
            state.x[vbase + 2] = (PR)geom.positions[vbase + 2];
        }

        if (adjacency.vertexAdjFacets.ptr) return;
        for (Index f = 0; f < params.numFacets; ++f) {
            Index fbase = f * 3;
            adjacency.facets[fbase    ] = geom.facets[fbase    ];
            adjacency.facets[fbase + 1] = geom.facets[fbase + 1];
            adjacency.facets[fbase + 2] = geom.facets[fbase + 2];
        }

        MeshAdjacencyInitializer<BE, PR>::initialize(state, adjacency);
    }

    void populatePreview(PreviewState<PR>& preview) override {
        auto geom = primitive::cube(
            (float)params.size, (int)params.tessellation,
            {params.center.x, params.center.y, params.center.z});
        // Physics buffers: welded single-manifold (cloth springs cross seams).
        preview.x.assign(geom.positions.begin(), geom.positions.end());
        preview.facets.assign(geom.facets.begin(), geom.facets.end());
        preview.recomputeNormals();
        // 2026-05-15 (A2 split): render buffers carry the unwelded per-face
        // copies so MeshGL renders flat per-face normals (crisp cube edges).
        // primitive::cube guarantees these are populated when called for the
        // cube primitive; copying through PR (preview is template-PR) handles
        // the float→double case for future CPU-double scenes.
        preview.renderX.assign(geom.renderPositions.begin(), geom.renderPositions.end());
        preview.renderFacets.assign(geom.renderFacets.begin(), geom.renderFacets.end());
        preview.renderN.assign(geom.renderNormals.begin(), geom.renderNormals.end());
        preview.renderToPhysics.assign(geom.renderToPhysics.begin(),
                                       geom.renderToPhysics.end());
    }

    InitializerParams<PR>* getParams() override { return &params; }
};


struct alignas(8) IndexPair {
    union {
        struct { Index query, target; }; 
        struct { Index point, triangle; }; 
        struct { Index edge1, edge2; };
    };

    bool operator<(const IndexPair& o) const {
        if(query == o.query) return target < o.target;
        return query < o.query;
    }
};

// Per-mesh collision geometry class. Mirrored into the GPU
// `meshShapes` buffer (Scene::pack → MetalGlobalContext slot 10) as
// the raw enum value, so the numeric order is GPU-visible — APPEND
// new entries, never renumber existing ones. Assigned in Scene::pack
// from the request's initializer subtype (the same dynamic_cast
// cascade that seeds transformPosition).
//
// Sphere/Cube/Cylinder are analytic primitives: a future slice (c)
// keys an analytic broad/narrow path on these so they skip the
// BVH/SH mesh pipeline. Until then this is classification metadata
// only — no collision code branches on it yet, so assigning it is
// behaviorally inert (safe to land independently).
