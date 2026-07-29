#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the preamble;
// relies on that preamble (using Index, std includes) and on earlier
// fragments. Not independently compilable by design.

template <typename BE>
struct DebugLineGL {};
template <>
struct DebugLineGL<CPU> {
    GLuint vao = 0;
    GLuint vertexBuffer = 0;

    float* vertexPtr = nullptr;
    size_t vertexNum = 0;
    size_t capacityVertices = 0;

    DebugLineGL() = default;

    DebugLineGL(size_t vertexNum, float* vertexPtr)
        : vertexPtr(vertexPtr), vertexNum(vertexNum), capacityVertices(vertexNum) {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     capacityVertices * sizeof(float) * 3,
                     vertexPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    }

    void updateBuffer(float* newVertexPtr, size_t newVertexNum = 0) {
        if (newVertexNum > 0) vertexNum = newVertexNum;
        vertexPtr = newVertexPtr;

        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);

        if (vertexNum > capacityVertices) {
            capacityVertices = vertexNum;
            glBufferData(GL_ARRAY_BUFFER,
                         capacityVertices * sizeof(float) * 3,
                         vertexPtr,
                         GL_DYNAMIC_DRAW);
        } else {
            glBufferSubData(GL_ARRAY_BUFFER,
                            0,
                            vertexNum * sizeof(float) * 3,
                            vertexPtr);
        }
    }

    void draw() {
        glBindVertexArray(vao);
        glDrawArrays(GL_LINES, 0, vertexNum);
    }
};

template <typename BE>
struct DebugPointGL {};
template <>
struct DebugPointGL<CPU> {
    GLuint vao;
    GLuint vertexBuffer;

    float* vertexPtr;
    size_t vertexNum;

    DebugPointGL() : vertexPtr(nullptr), vertexNum(0) {}
    DebugPointGL(size_t vertexNum, float* vertexPtr) : vertexNum(vertexNum), vertexPtr(vertexPtr) {
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        
        glGenBuffers(1, &vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     vertexNum * sizeof(float) * 3,
                     vertexPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);
    }

    void updateBuffer(float* newVertexPtr) {
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, newVertexPtr);
    }

    void draw() {
        glBindVertexArray(vao);
        
        glDrawArrays(GL_POINTS, 0, vertexNum);
    }
};


template <typename BE, typename PR>
struct Scene;

// General Mesh Deinfe!

template <typename BE, typename PR>
struct GeneralMesh;


enum struct BehaviorType : Index {
    TriangularCloth,
    FastGridCloth,
    Elastic,
    Rigid,
    Float,
    Fluid,
    Generator,
    // BVH-driven kinematic body: prescribed motion, one-way coupling.
    // Collision-wise it acts like Float (query-skipped, target-only) but
    // its vertices are rewritten every frame from the FK pose. Appended
    // last — scene_format serializes the enum by value.
    Kinematic,
};

enum struct InitializerType : Index {
    MeshGridSpring,
    MeshFile,

};



template <typename BE, typename PR>
struct ExternalForces {
    VectorBase<BE, PR> externalForces;
};

template <typename BE, typename PR>
struct MeshState {
    using Vector = VectorBase<BE, PR>;
    // xPrev: the start-of-substep position, copied from x by the simulator
    // right before the integrator runs. Consumed by the swept-segment-vs-
    // triangle narrow phase (D-013, closes CM-005) so cloth-on-static-ground
    // contacts fire for every substep whose trajectory crosses the surface,
    // not only substeps whose sample-time position happens to be within
    // radius+thickness of the surface.
    Vector x, xPrev, v, f, m, n;
    template <typename InitializerParams>
    void memoryAllocation(InitializerParams& params) {
        Index numData = params.numPoints*3;
        if(!x.ptr) x = Vector(numData);
        if(!xPrev.ptr) xPrev = Vector(numData);
        if(!v.ptr) v = Vector(numData, 0);
        if(!f.ptr) f = Vector(numData, 0);
        if(!m.ptr) m = Vector(numData, params.mass);
        if(!n.ptr) n = Vector(numData);
    }
};

template <typename BE, typename PR>
struct MeshAdjacency {
    using Vector = VectorBase<BE, PR>;
    using Vectorui = VectorBase<BE, Index>;
    //! Length: numPrimitives * numVerticesPerPrimitive
    //! For constructing BVH.
    Vectorui facets, edges;
    //! Length: numPrimitives
    //! Each index holds the rest area/length of corresponding primitive.
    Vector restFacetAreas, restEdgeLengths, restOppLengths;
    Vectorui vertexAdjFacets, vertexAdjFacetsOffsets;
    Vectorui vertexAdjEdges, vertexAdjEdgesOffsets;
    //Vectorui edgeAdjEdges, edgeAdjEdgesOffsets;
    Vectorui vertexOppVertices, vertexOppVerticesOffsets; // for spring
    template <typename InitializerParams>
    void memoryAllocation(InitializerParams& params) {
        if(params.numFacets > 0) {
            if(!facets.ptr) 
                facets = Vectorui(params.numFacets*3);
            if(!restFacetAreas.ptr) 
                restFacetAreas = Vector(params.numFacets);
        }
        if(params.numEdges > 0) {
            if(!edges.ptr) 
                edges = Vectorui(params.numEdges*2);
            if(!restEdgeLengths.ptr) 
                restEdgeLengths = Vector(params.numEdges);
        }
        if(params.numPoints > 0) {
            if(!vertexAdjFacetsOffsets.ptr)
                vertexAdjFacetsOffsets = Vectorui(params.numPoints+1, 0);
            if(!vertexAdjEdgesOffsets.ptr)
                vertexAdjEdgesOffsets = Vectorui(params.numPoints+1, 0);
            if(!vertexOppVerticesOffsets.ptr)
                vertexOppVerticesOffsets = Vectorui(params.numPoints+1, 0);
        }
    }
};

struct EdgeInfo {
    int v0=-1, v1=-1; // v0 < v1
    int f0=-1, f1=-1; // f0: (v1, v0, o0), f1: (v0, v1, o1)
    int o0=-1, o1=-1; // o0 in f0, o1 in f1
};

template <typename PR>
struct InitializerParams {
    // Commons
    Index numPoints, numFacets, numEdges;
    PR mass;
    // S3: object-level transform owned by the initializer params so a
    // structural re-pack rebuilds the FULL transformed geometry
    // deterministically from the request alone (no preview carrier).
    // `center` (position) already lives on each derived params struct;
    // these add the missing rotation + per-axis scale. Pivot for both
    // is `center`. Applied scale → rotate → translate, matching
    // reset()'s order. Defaults = identity (no-op until set).
    tinym::vec3 scale{1, 1, 1};
    ::Quat rotationQuat{};
    InitializerParams(Index numPoints, Index numFacets, Index numEdges, PR mass) : numPoints(numPoints), numFacets(numFacets), numEdges(numEdges), mass(mass) {}
};
template <typename BE, typename PR>
struct GeneralMeshInitializer {
    virtual ~GeneralMeshInitializer() = default;
    virtual void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) = 0;
    virtual InitializerParams<PR>* getParams() = 0;

    // D-042 R-1 (2026-05-14): populate a PreviewState directly from the
    // initializer's parameters — no pool, no MeshState. Called at
    // addGeneralMesh time so the Scene can render the mesh BEFORE pack.
    // Default no-op: subtypes that haven't migrated yet still compile;
    // their preview just stays empty and MeshGL will skip rendering them
    // (R-2 wires MeshGL to PreviewState — until then this is parallel
    // symbol infrastructure only).
    virtual void populatePreview(PreviewState<PR>& /*preview*/) {}
};
//! Suppose that the positions and facets are given
template <typename BE, typename PR>
struct MeshAdjacencyInitializer {
    using Vector = VectorBase<BE, PR>;

    static void initialize(MeshState<BE, PR>& state, MeshAdjacency<BE, PR>& adjacency) {
        //std::cout << "[MeshSpringInitializer initialize] start" << std::endl;
        Index maxNumEdgeInfos = adjacency.facets.size;
        Index numPoints = state.x.size/3;
        DynamicMemoryAllocator<BE> tempPool;
        VectorBase<BE, EdgeInfo> tempEdgeInfos(tempPool.template alloc<EdgeInfo>(maxNumEdgeInfos));
        VectorBase<BE, EdgeInfo> edgeInfos(tempPool.template alloc<EdgeInfo>(maxNumEdgeInfos));
        //std::cout << "[MeshSpringInitializer initialize] temp vector allocated" << std::endl;

        // vertex adj facets
        
        Index numFacets = adjacency.facets.size/3;
        
        for(Index fid = 0; fid < numFacets; fid++) {
            Index fbase = fid*3;
            Index a = adjacency.facets[fbase];
            Index b = adjacency.facets[fbase+1];
            Index c = adjacency.facets[fbase+2];

            adjacency.vertexAdjFacetsOffsets[a+1]++;
            adjacency.vertexAdjFacetsOffsets[b+1]++;
            adjacency.vertexAdjFacetsOffsets[c+1]++;
        }

        for(Index vid = 0; vid < numPoints; ++vid) 
            adjacency.vertexAdjFacetsOffsets[vid+1] += adjacency.vertexAdjFacetsOffsets[vid];

        //std::cout << "[MeshSpringInitializer initialize] vertex adj facets offsets set" << std::endl;

        adjacency.vertexAdjFacets = VectorBase<BE, Index>(adjacency.vertexAdjFacetsOffsets[numPoints]);
        
        VectorBase<BE, Index> offsets(tempPool.template zeros<Index>(numPoints));
        for(Index fid = 0; fid < numFacets; ++fid) {
            Index fbase = fid*3;
            Index v0 = adjacency.facets[fbase];
            Index v1 = adjacency.facets[fbase+1];
            Index v2 = adjacency.facets[fbase+2];

            Index v0base = offsets[v0]+adjacency.vertexAdjFacetsOffsets[v0];
            Index v1base = offsets[v1]+adjacency.vertexAdjFacetsOffsets[v1];
            Index v2base = offsets[v2]+adjacency.vertexAdjFacetsOffsets[v2];

            adjacency.vertexAdjFacets[v0base] = fid;
            adjacency.vertexAdjFacets[v1base] = fid;
            adjacency.vertexAdjFacets[v2base] = fid;

            offsets[v0]++;
            offsets[v1]++;
            offsets[v2]++;
        }
        //std::cout << "[MeshSpringInitializer initialize] vertex adjacent facets set" << std::endl;
        //for(Index i = 0; i < 10; ++i) {
        //    std::cout << i << "-th adjacent facets: ";
        //    for(Index fi = adjacency.vertexAdjFacetsOffsets[i]; fi < adjacency.vertexAdjFacetsOffsets[i+1]; ++fi) {
        //        std::cout << adjacency.vertexAdjFacets[fi] << ", ";
        //    }
        //    std::cout << std::endl;
        //}


        
        // Fill temp edge infos (opposite edges are inserted twice)
        Index eIdx = 0;
        auto fillEdgeInfos = [&](Index edgeIndex, Index v0, Index v1, Index o0, Index f0) {
            if(v0 > v1) {
                Index temp = v0;
                v0 = v1;
                v1 = temp;
            }

            tempEdgeInfos[edgeIndex].v0 = v0;
            tempEdgeInfos[edgeIndex].v1 = v1;
            tempEdgeInfos[edgeIndex].o0 = o0;
            tempEdgeInfos[edgeIndex].o1 = -1;
            tempEdgeInfos[edgeIndex].f0 = f0;
            tempEdgeInfos[edgeIndex].f1 = -1;

        };
        for(Index fid = 0; fid < numFacets; ++fid) {
            Index fbase = fid*3;
            Index v0 = adjacency.facets[fbase];
            Index v1 = adjacency.facets[fbase+1];
            Index v2 = adjacency.facets[fbase+2];

            fillEdgeInfos(eIdx++, v0, v1, v2, fid);
            fillEdgeInfos(eIdx++, v1, v2, v0, fid);
            fillEdgeInfos(eIdx++, v2, v0, v1, fid);
        }
        //std::cout << "[MeshSpringInitializer initialize] temp edges are filled" << std::endl;

        // Sort the temp edge infos to reduce
        std::sort(tempEdgeInfos.ptr, tempEdgeInfos.ptr+eIdx, [](EdgeInfo& a, EdgeInfo& b) {
            return a.v0 < b.v0 || (a.v0 == b.v0 && a.v1 < b.v1);
        });
        //std::cout << "[MeshSpringInitializer initialize] temp edges are sorted" << std::endl;
        //std::cout << " ---- test output ---- " << std::endl;
        //for(int i = 0; i < 10; i++) 
        //    std::cout << tempEdgeInfos[i].v0 << ", " << tempEdgeInfos[i].v1 << " in facet id " << tempEdgeInfos[i].f0 << std::endl;

        // Reduce temp edge infos into unique edge infos
        // And initialize restEdgeLengths
        auto edgeLength = [&](Index vid0, Index vid1) {
            auto v0 = tinym::vec3_view(state.x.ptr+vid0*3);
            auto v1 = tinym::vec3_view(state.x.ptr+vid1*3);
            auto l = v1-v0;
            return l.norm();
        };
        adjacency.edges[0] = tempEdgeInfos[0].v0;
        adjacency.edges[1] = tempEdgeInfos[0].v1;
        edgeInfos[0] = tempEdgeInfos[0];
        Index edgeid = 0;
        adjacency.restEdgeLengths[edgeid] = edgeLength(tempEdgeInfos[0].v0, tempEdgeInfos[0].v1);
        adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[0].v0+1]++;
        adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[0].v1+1]++;
        //Index
        for(Index ei = 1; ei < eIdx; ++ei) {
            Index edgeBase = edgeid*2;
            if(tempEdgeInfos[ei].v0 != adjacency.edges[edgeBase] || tempEdgeInfos[ei].v1 != adjacency.edges[edgeBase+1]) {
                edgeid++;
                edgeBase = edgeid*2;

                adjacency.edges[edgeBase  ] = tempEdgeInfos[ei].v0;
                adjacency.edges[edgeBase+1] = tempEdgeInfos[ei].v1;
                
                adjacency.restEdgeLengths[edgeid] = edgeLength(tempEdgeInfos[ei].v0, tempEdgeInfos[ei].v1);
                adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[ei].v0+1]++;
                adjacency.vertexAdjEdgesOffsets[tempEdgeInfos[ei].v1+1]++;

                edgeInfos[edgeid] = tempEdgeInfos[ei];
            } else {
                edgeInfos[edgeid].o1 = tempEdgeInfos[ei].o0;
                edgeInfos[edgeid].f1 = tempEdgeInfos[ei].f0;
                adjacency.vertexOppVerticesOffsets[edgeInfos[edgeid].o0+1]++;
                adjacency.vertexOppVerticesOffsets[edgeInfos[edgeid].o1+1]++;
            }
        }
        Index edgeNum = edgeid+1;
        //for(int i = 0; i < 10; i++) {
        //    std::cout << adjacency.edges[i*2] << ", " << adjacency.edges[i*2+1] << ": " << adjacency.restEdgeLengths[i] << std::endl;
        //    Index vid0 = adjacency.edges[i*2];
        //    Index vid1 = adjacency.edges[i*2+1];
        //    std::cout << state.x[vid0*3] << ", " << state.x[vid0*3+1] << ", " << state.x[vid0*3+2] << std::endl;
        //    std::cout << state.x[vid1*3] << ", " << state.x[vid1*3+1] << ", " << state.x[vid1*3+2] << std::endl;
        //}

        for(Index i = 0; i < numPoints; ++i) {
            adjacency.vertexOppVerticesOffsets[i+1] += adjacency.vertexOppVerticesOffsets[i];
            adjacency.vertexAdjEdgesOffsets[i+1] += adjacency.vertexAdjEdgesOffsets[i];
        }
        //std::cout << "vertexOppVerticesOffsets: " << std::endl;
        //for(int i = 0; i < 10; i++) 
        //    std::cout << adjacency.vertexOppVerticesOffsets[i] << std::endl;
        //std::cout << "..." << adjacency.vertexOppVerticesOffsets[numPoints] << std::endl;
        //std::cout << "vertexAdjEdgesOffsets: " << std::endl;
        //for(int i = 0; i < 10; i++) 
        //    std::cout << adjacency.vertexAdjEdgesOffsets[i] << std::endl;
        //std::cout << "..." << adjacency.vertexAdjEdgesOffsets[numPoints] << std::endl;
        //std::cout << "edgeInfos: " << std::endl;
        //for(Index i = 0; i < 10; i++)
        //    std::cout << edgeInfos[i].v0 << " " << edgeInfos[i].v1 << " " << edgeInfos[i].o0 << " " << edgeInfos[i].o1 << " " << edgeInfos[i].f0 << " " << edgeInfos[i].f1 << std::endl;

        
        // set vertexOppVertices and vertexAdjEdges
        VectorBase<BE, Index> oppOffsets(tempPool.template zeros<Index>(numPoints));
        VectorBase<BE, Index> adjOffsets(tempPool.template zeros<Index>(numPoints));
        //std::cout << "[MeshSpringInitializer initialize] oppOffsets allocated" << std::endl;
        adjacency.vertexOppVertices = VectorBase<BE, Index>(adjacency.vertexOppVerticesOffsets[numPoints], 0);
        adjacency.restOppLengths = VectorBase<BE, PR>(adjacency.vertexOppVerticesOffsets[numPoints]);
        adjacency.vertexAdjEdges = VectorBase<BE, Index>(adjacency.vertexAdjEdgesOffsets[numPoints], 0);
        //std::cout << "[MeshSpringInitializer initialize] vertex opposite vertices allocated" << std::endl;
        for(Index ei = 0; ei < edgeNum; ++ei) {
            if(edgeInfos[ei].o1 != -1) {
                Index o0 = edgeInfos[ei].o0;
                Index o1 = edgeInfos[ei].o1;

                Index o0base = oppOffsets[o0]+adjacency.vertexOppVerticesOffsets[o0];
                Index o1base = oppOffsets[o1]+adjacency.vertexOppVerticesOffsets[o1];

                adjacency.vertexOppVertices[o0base] = o1;
                adjacency.vertexOppVertices[o1base] = o0;

                adjacency.restOppLengths[o0base] = edgeLength(o0, o1);
                adjacency.restOppLengths[o1base] = edgeLength(o0, o1);

                oppOffsets[o0]++;
                oppOffsets[o1]++;
            }

            Index v0 = edgeInfos[ei].v0;
            Index v1 = edgeInfos[ei].v1;

            Index v0base = adjOffsets[v0]+adjacency.vertexAdjEdgesOffsets[v0];
            Index v1base = adjOffsets[v1]+adjacency.vertexAdjEdgesOffsets[v1];

            adjacency.vertexAdjEdges[v0base] = ei;
            adjacency.vertexAdjEdges[v1base] = ei;

            adjOffsets[v0]++;
            adjOffsets[v1]++;
        }
        //std::cout << "[MeshSpringInitializer initialize] Opposite vertices set" << std::endl;
        //for(Index i = 0; i < 10; i++) {
        //    std::cout << i << "-th opposite: ";
        //    for(Index oi = adjacency.vertexOppVerticesOffsets[i]; oi < adjacency.vertexOppVerticesOffsets[i+1]; ++oi) {
        //        std::cout << adjacency.vertexOppVertices[oi] << ", ";
        //    }
        //    std::cout << std::endl;
        //}
        //for(Index i = 0; i < 10; i++) {
        //    std::cout << i << "-th adj edges: ";
        //    for(Index ei = adjacency.vertexAdjEdgesOffsets[i]; ei < adjacency.vertexAdjEdgesOffsets[i+1]; ++ei) {
        //        std::cout << adjacency.vertexAdjEdges[ei] << ", ";
        //    }
        //    std::cout << std::endl;
        //}
    }

    // Re-measure the stretch (restEdgeLengths) and bend (restOppLengths)
    // rest quantities from the CURRENT state.x, reusing the topology
    // (edges / vertexOppVertices) that `initialize()` already built.
    //
    // Why this exists: Scene::pack() runs mesh.initialize() (which calls
    // the function above and seeds rest lengths from the initializer's
    // param-regenerated geometry) and ONLY THEN overrides state.x with
    // the per-request PreviewState (D-042 R-3 memcpy). Any preview-only
    // edit — scaleObject / rotateObject write the new geometry into
    // preview.x, never back into the initializer params — would leave
    // rest lengths describing the un-edited grid while the simulated
    // particles sit at the edited positions, pre-stressing every spring
    // (a 2x scaled sheet starts at 100% strain → instant blow-up, even
    // in free fall). The invariant is: rest length == the geometry the
    // user actually sees (the preview state). pack() calls this right
    // after the R-3 memcpy so that invariant holds unconditionally,
    // including under scale (the scale question raised in review) and
    // rotation.
    static void recomputeRestLengths(MeshState<BE, PR>& state,
                                     MeshAdjacency<BE, PR>& adjacency) {
        // A pool allocation that overflowed capacity returns a null
        // sub-view (see ByteMemoryPool::alloc → "[Pool] Tried to
        // allocate more than tha capacity"). Large imports (Human.obj:
        // 24461 verts / 48918 tris) hit this. Deref-guard so a failed
        // allocation degrades to "no rest update" instead of a segfault.
        if (!state.x.ptr || state.x.size == 0) return;
        const Index numPoints = state.x.size / 3;
        // edges / vertexOppVertices are allocated to params.numEdges /
        // an upper-bound offset total, but MeshAdjacencyInitializer only
        // *writes* the deduplicated subset and never reads past it. The
        // unwritten tail is uninitialized pool memory (large garbage,
        // especially after a big import churns the pool). Treat any
        // endpoint outside [0, numPoints) as a tail slot and skip it —
        // the physics shaders never consume those entries anyway, so
        // leaving their rest value stale is correct and crash-free.
        auto inRange = [&](Index v) { return v < numPoints; }; // Index is unsigned
        auto dist = [&](Index a, Index b) {
            auto va = tinym::vec3_view(state.x.ptr + a*3);
            auto vb = tinym::vec3_view(state.x.ptr + b*3);
            return (vb - va).norm();
        };
        if (adjacency.edges.ptr && adjacency.restEdgeLengths.ptr) {
            Index numEdges = adjacency.edges.size / 2;
            for (Index e = 0; e < numEdges; ++e) {
                Index a = adjacency.edges[e*2];
                Index b = adjacency.edges[e*2+1];
                if (!inRange(a) || !inRange(b)) continue;
                adjacency.restEdgeLengths[e] = dist(a, b);
            }
        }
        if (adjacency.vertexOppVertices.ptr
            && adjacency.vertexOppVerticesOffsets.ptr
            && adjacency.restOppLengths.ptr) {
            for (Index v = 0; v < numPoints; ++v)
                for (Index k = adjacency.vertexOppVerticesOffsets[v];
                     k < adjacency.vertexOppVerticesOffsets[v+1]; ++k) {
                    Index o = adjacency.vertexOppVertices[k];
                    if (!inRange(o)) continue;
                    adjacency.restOppLengths[k] = dist(v, o);
                }
        }
    }
};

