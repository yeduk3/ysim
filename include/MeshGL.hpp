#ifndef YSIM_MESH_GL_HPP
#define YSIM_MESH_GL_HPP

// OpenGL render state for a single mesh: VAO + VBOs (vertex / normal /
// facet / optional texcoord), normal computation, draw + per-frame
// upload helpers. Lifted out of GeneralMesh so the simulation-side
// data structure no longer requires a live GL context — see D-011 and
// CM-002/003/004's shared cause: the unit-test net could not reach
// Simulator::initialize() because mesh.initialize() called glGenVertexArrays.
//
// The CPU specialization is the only one v1 ships; the primary template
// stays for future backends. `MeshGL<CPU>` is owned by `MeshRenderState`
// (include/MeshRenderState.hpp) keyed by mesh id, NOT by GeneralMesh.

#include <GL/glew.h>
#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "program.hpp"
#include "tinym.hpp"

template <typename BE>
struct MeshGL {};

struct CPU;  // forward — defined in src/main.cpp as `struct CPU : Backend {}`.

template <>
struct MeshGL<CPU> {
    GLuint vao;
    GLuint vertexBuffer, normalBuffer, facetBuffer, texCoordBuffer;

    size_t vertexNum;
    size_t facetNum;
    float* vertexPtr;
    unsigned int* facetPtr;
    float* normalPtr;

    MeshGL() {}
    MeshGL(size_t vertexNum, float* vertexPtr, size_t facetNum, unsigned int* facetPtr, float* normalPtr, float* texCoordPtr=nullptr)
        : vertexNum(vertexNum), vertexPtr(vertexPtr), facetNum(facetNum), facetPtr(facetPtr), normalPtr(normalPtr) {
        std::cout << "[MeshGL] Try to create..." << std::endl;
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

        glGenBuffers(1, &facetBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     facetNum * sizeof(unsigned int) * 3,
                     facetPtr,
                     GL_STATIC_DRAW);

        // 2026-05-15 (A2 split): computeNormal() removed. Pre-A2 it
        // overwrote normalPtr with winding-derived vertex normals (cross
        // (v1-v0, v2-v0) per triangle); for primitive::cube that produces
        // INWARD normals (the emit-order winding gives -outward) AND
        // undoes the flat per-face normals stored in preview.renderN.
        // PreviewState::recomputeNormals + ::recomputeRenderNormals now
        // keep normalPtr current; MeshGL just uploads what's there.

        glGenBuffers(1, &normalBuffer);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferData(GL_ARRAY_BUFFER,
                     vertexNum * sizeof(float) * 3,
                     normalPtr,
                     GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), 0);

        if(texCoordPtr) {
            glGenBuffers(1, &texCoordBuffer);
            glBindBuffer(GL_ARRAY_BUFFER, texCoordBuffer);
            glBufferData(GL_ARRAY_BUFFER,
                         vertexNum * sizeof(float) * 2,
                         texCoordPtr,
                         GL_STATIC_DRAW);
            glEnableVertexAttribArray(2);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
        }
        std::cout << "[MeshGL] Created!" << std::endl;
    }

    void computeNormal() {
        Eigen::Map<const Eigen::Matrix<float, 3, Eigen::Dynamic>> V(vertexPtr, 3, vertexNum);
        Eigen::Map<Eigen::Matrix<unsigned int, 3, Eigen::Dynamic>> F(facetPtr, 3, facetNum);
        Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> N(normalPtr, 3, vertexNum);

        N.setZero();

        for (size_t i = 0; i < facetNum; ++i) {
            unsigned int i0 = F(0, i);
            unsigned int i1 = F(1, i);
            unsigned int i2 = F(2, i);

            auto v0 = V.col(i0);
            auto v1 = V.col(i1);
            auto v2 = V.col(i2);

            Eigen::Vector3f cross = (v1 - v0).cross(v2 - v0);

            N.col(i0) += cross;
            N.col(i1) += cross;
            N.col(i2) += cross;
        }

        for (size_t i = 0; i < vertexNum; ++i) {
            float len = N.col(i).norm();
            if (len > 1e-7f) {
                N.col(i) /= len;
            }
        }
    }

    // 2026-05-15 (A2 split): re-upload the buffers from the pointers
    // stored at ctor time (set by MeshRenderState::registerPreviewBinding
    // to preview.renderXPtr / renderNPtr — preview owns these vectors,
    // their data() is stable across in-place value mutations from R-5
    // resync). The caller must keep normalPtr current — PreviewState's
    // recomputeNormals + recomputeRenderNormals do that every frame in
    // Simulator::update's R-5 resync. No more in-MeshGL recomputation
    // (which would clobber the flat per-face normals from cube).
    void updateBuffer() {
        glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, vertexPtr);
        glBindBuffer(GL_ARRAY_BUFFER, normalBuffer);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexNum * sizeof(float) * 3, normalPtr);
    }

    // PBD tearing (phase 2): re-upload the INDEX buffer from the same
    // facetPtr the ctor bound (preview.facets — a heap vector whose data()
    // is stable, since a tear only overwrites index VALUES in place and
    // never resizes). Kept GL_STATIC_DRAW on purpose: glBufferSubData on a
    // static-hinted buffer is legal, and a torn frame is rare (a handful of
    // frames in a whole run) — hinting DYNAMIC would pessimize the common
    // case where the topology never changes at all.
    //
    // Must be called from the GL thread. Simulator::update only marks the
    // mesh dirty (CPU-side); the upload happens in uploadMeshes().
    void updateFacetBuffer() {
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
        glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                        facetNum * sizeof(unsigned int) * 3, facetPtr);
    }

    // D-028: PBR preview signature. Takes the 5 D-005 material primitives
    // rather than a Material struct to avoid coupling this header to
    // main.cpp's Material type (same convention as MeshInspectorTarget's
    // on_material_edit callback per D-027). Shader-side uniform names
    // mirror the Material field names exactly so the binding is
    // grep-able from either direction.
    void draw(Program& shader,
              const tinym::vec3& baseColor,
              float metallic,
              float roughness,
              float specularWeight,
              const tinym::vec3& emissionColor) {
        shader.setUniform("baseColor",      baseColor);
        shader.setUniform("metallic",       metallic);
        shader.setUniform("roughness",      roughness);
        shader.setUniform("specularWeight", specularWeight);
        shader.setUniform("emissionColor",  emissionColor);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);

        glDrawElements(GL_TRIANGLES, facetNum*3, GL_UNSIGNED_INT, 0);
    }

    // ── Cluster visualization (debug) ─────────────────────────────────
    // Multi-draw the mesh colored by sub-object BVH cluster: a cluster-
    // sorted index buffer (built on demand, keyed by cluster count) + per-
    // cluster (offset,count) ranges, each drawn with a hashed baseColor
    // through the normal PBR shader. No shader/attribute changes — just one
    // extra draw call per non-empty cluster.
    // ponytail: O(#clusters) draw calls/frame while the toggle is on; fine
    // for a debug viz, gate to low s if it ever bites perf.
    GLuint clusterEBO = 0;
    std::vector<uint32_t> clusterOffset, clusterCount, clusterId;  // parallel, per drawn cluster
    int clusterVizKey = -1;

    static tinym::vec3 clusterColor(uint32_t g) {
        // golden-ratio hue hash → vivid, well-separated colors per cluster
        float h = std::fmod((float)g * 0.61803398875f, 1.0f) * 6.0f;
        float x = 1.0f - std::fabs(std::fmod(h, 2.0f) - 1.0f);
        float r, gr, b;
        if      (h < 1) { r=1; gr=x; b=0; } else if (h < 2) { r=x; gr=1; b=0; }
        else if (h < 3) { r=0; gr=1; b=x; } else if (h < 4) { r=0; gr=x; b=1; }
        else if (h < 5) { r=x; gr=0; b=1; } else            { r=1; gr=0; b=x; }
        return tinym::vec3(0.15f + 0.85f*r, 0.15f + 0.85f*gr, 0.15f + 0.85f*b);
    }

    // Rebuild the cluster-sorted index buffer when `key` changes (key =
    // cluster count, so it rebuilds on a split-s change). groupOfPrim[f] is
    // the cluster of original facet f, aligned with facetPtr.
    void buildClusters(const uint32_t* groupOfPrim, int numClusters, int key) {
        if (key == clusterVizKey && clusterEBO) return;
        clusterVizKey = key;
        std::vector<uint32_t> idx(facetNum * 3);
        std::vector<uint32_t> cnt(numClusters, 0), base(numClusters, 0);
        for (size_t f = 0; f < facetNum; ++f) cnt[groupOfPrim[f]]++;
        uint32_t acc = 0;
        clusterOffset.clear(); clusterCount.clear(); clusterId.clear();
        for (int g = 0; g < numClusters; ++g) {
            base[g] = acc;
            if (cnt[g]) { clusterOffset.push_back(acc * 3); clusterCount.push_back(cnt[g] * 3); clusterId.push_back((uint32_t)g); }
            acc += cnt[g];
        }
        std::vector<uint32_t> cur = base;
        for (size_t f = 0; f < facetNum; ++f) {
            uint32_t g = groupOfPrim[f], p = cur[g]++;
            idx[p*3+0] = facetPtr[f*3+0]; idx[p*3+1] = facetPtr[f*3+1]; idx[p*3+2] = facetPtr[f*3+2];
        }
        if (!clusterEBO) glGenBuffers(1, &clusterEBO);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, clusterEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, idx.size() * sizeof(uint32_t), idx.data(), GL_DYNAMIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);  // restore the VAO's element binding
    }

    void drawClusters(Program& shader, float metallic, float roughness,
                      float specularWeight, const tinym::vec3& emissionColor) {
        shader.setUniform("metallic",       metallic);
        shader.setUniform("roughness",      roughness);
        shader.setUniform("specularWeight", specularWeight);
        shader.setUniform("emissionColor",  emissionColor);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, clusterEBO);
        for (size_t i = 0; i < clusterId.size(); ++i) {
            shader.setUniform("baseColor", clusterColor(clusterId[i]));
            glDrawElements(GL_TRIANGLES, (GLsizei)clusterCount[i], GL_UNSIGNED_INT,
                           (const void*)(size_t)(clusterOffset[i] * sizeof(uint32_t)));
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
    }

    // ID-pass draw: writes `meshId` into the R32I color attachment of
    // the currently-bound framebuffer. Uses the same VAO (position
    // attribute at location 0) — id.vert ignores the normal attribute
    // at location 1 even though it's bound. The caller is responsible
    // for binding the id framebuffer, clearing it, and setting M/V/P.
    void drawIdOnly(Program& shader, int meshId) {
        shader.setUniform("uMeshId", meshId);
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
        glDrawElements(GL_TRIANGLES, facetNum*3, GL_UNSIGNED_INT, 0);
    }

    // Depth-only draw for the directional shadow pass: no per-mesh
    // uniforms at all (shadow.vert reads only LightVP, set once by the
    // caller). Same VAO — position at location 0.
    void drawDepthOnly() {
        glBindVertexArray(vao);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
        glDrawElements(GL_TRIANGLES, facetNum*3, GL_UNSIGNED_INT, 0);
    }

    // Vertex-id pass: one GL_POINTS per render vertex; idpoint.frag
    // writes (meshId, depth, gl_VertexID) into the RGBA32F attachment.
    // Host sets glPointSize + depth func before calling. Same VAO as
    // draw() — position attribute at location 0; idpoint.vert ignores
    // the normal attribute.
    void drawPointsIdOnly(Program& shader, int meshId) {
        shader.setUniform("uMeshId", meshId);
        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, (GLsizei)vertexNum);
    }

    // On-screen overlay: every render vertex as a flat-colored point
    // (caller sets uColor + glPointSize).
    void drawPoints(Program& shader) {
        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, (GLsizei)vertexNum);
    }

    // On-screen overlay: a single render vertex (hover / select dot).
    void drawOnePoint(Program& shader, int vid) {
        if (vid < 0 || (size_t)vid >= vertexNum) return;
        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, vid, 1);
    }
};

#endif  // YSIM_MESH_GL_HPP
