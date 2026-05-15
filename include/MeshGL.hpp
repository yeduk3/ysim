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

#include <cstddef>
#include <iostream>

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
};

#endif  // YSIM_MESH_GL_HPP
