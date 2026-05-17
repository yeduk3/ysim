#pragma once

// Assimp-based 3D model loader. Parallel to objreader.hpp's ObjData::loadObject:
// it does NOT replace the OBJ path — it fills the same ObjData intermediate so
// the existing MeshFileInitializer -> MeshState/MeshAdjacency pipeline is reused
// unchanged (see AssimpMeshFileInitializer in main.cpp).
//
// Reads any format Assimp supports (.obj/.fbx/.gltf/.dae/.stl/.ply/...), always
// triangulated (aiProcess_Triangulate, as required by the simulator), with
// per-vertex normals generated when the asset lacks them (parity with the OBJ
// reader's syncedNormals fallback).

#include "objreader.hpp"   // ObjData, tinym

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <string>

// Fill `out` (an ObjData) from `prefix`/`fileName` via Assimp. All sub-meshes
// in the scene are flattened (aiProcess_PreTransformVertices) and merged into a
// single vertex/triangle list. Returns false and sets *err on failure; `out` is
// left empty in that case.
inline bool loadModelWithAssimp(const std::string& prefix,
                                const std::string& fileName,
                                ObjData& out,
                                std::string* err = nullptr) {
    const std::string fullPath =
        prefix.empty() ? fileName : (prefix + "/" + fileName);

    Assimp::Importer importer;
    const unsigned int flags =
        aiProcess_Triangulate |          // simulator requires triangles
        aiProcess_JoinIdenticalVertices |
        aiProcess_GenSmoothNormals |     // only generated if absent
        aiProcess_PreTransformVertices;  // bake node transforms, merge meshes

    const aiScene* scene = importer.ReadFile(fullPath, flags);
    if (!scene || (scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) ||
        !scene->mRootNode || scene->mNumMeshes == 0) {
        if (err) *err = std::string("assimp: ") + importer.GetErrorString();
        return false;
    }

    out = ObjData();
    out.prefix   = prefix.empty() ? "" : (prefix + "/");
    out.fileName = fileName;
    out.maxPos = tinym::vec3(-987654321.f);
    out.minPos = tinym::vec3( 987654321.f);

    GLuint vertexOffset = 0;
    for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi) {
        const aiMesh* mesh = scene->mMeshes[mi];
        if (!mesh || mesh->mNumVertices == 0) continue;

        const bool hasNormals = mesh->HasNormals();
        for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi) {
            const aiVector3D& p = mesh->mVertices[vi];
            out.vertices.push_back({p.x, p.y, p.z});

            if (p.x > out.maxPos.x) out.maxPos.x = p.x;
            if (p.x < out.minPos.x) out.minPos.x = p.x;
            if (p.y > out.maxPos.y) out.maxPos.y = p.y;
            if (p.y < out.minPos.y) out.minPos.y = p.y;
            if (p.z > out.maxPos.z) out.maxPos.z = p.z;
            if (p.z < out.minPos.z) out.minPos.z = p.z;

            if (hasNormals) {
                const aiVector3D& n = mesh->mNormals[vi];
                out.normals.push_back({n.x, n.y, n.z});
                out.syncedNormals.push_back({n.x, n.y, n.z});
            } else {
                out.normals.push_back({0.f, 0.f, 0.f});
                out.syncedNormals.push_back({0.f, 0.f, 0.f});
            }

            if (mesh->HasTextureCoords(0)) {
                const aiVector3D& t = mesh->mTextureCoords[0][vi];
                out.texCoords.push_back({t.x, t.y, 0.f});
            }
        }

        for (unsigned int fi = 0; fi < mesh->mNumFaces; ++fi) {
            const aiFace& f = mesh->mFaces[fi];
            if (f.mNumIndices != 3) continue;  // post-Triangulate: always 3
            out.elements3.push_back({vertexOffset + f.mIndices[0],
                                     vertexOffset + f.mIndices[1],
                                     vertexOffset + f.mIndices[2]});
        }
        vertexOffset += mesh->mNumVertices;
    }

    if (out.vertices.empty() || out.elements3.empty()) {
        if (err) *err = "assimp: model has no triangulated geometry: " + fullPath;
        out = ObjData();
        return false;
    }

    out.nVertices      = (GLuint)out.vertices.size();
    out.nElements3     = (GLuint)out.elements3.size();
    out.nElements4     = 0;
    out.nNormals       = (GLuint)out.normals.size();
    out.nSyncedNormals = (GLuint)out.syncedNormals.size();
    out.nTextures      = (GLuint)out.texCoords.size();
    out.center = (out.maxPos + out.minPos) * 0.5f;
    out.scale  = out.maxPos - out.minPos;
    out.isOk   = false;  // GL buffers not generated here (parity with loadObject)

    std::cout << "--- Assimp Model Loaded: " << fullPath
              << " (v=" << out.nVertices << ", tri=" << out.nElements3
              << ") ---" << std::endl;
    return true;
}
