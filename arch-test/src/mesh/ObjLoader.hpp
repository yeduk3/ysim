#pragma once
#include "backend/Backend.hpp"        // Index
#include "tinym.hpp"                   // tinym::vec3
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>
#include <iostream>
#include <limits>
#include <algorithm>
#include <cctype>

// Minimal GL-free Wavefront OBJ loader for Float colliders. Ports the parse
// algorithm of include/objreader.hpp (ObjData::loadObject) but drops every
// GL/normal/material/texcoord concern — a Float collider needs only vertex
// positions (for SimState.x) and triangle facets (for BVH + narrow targets).
//
// Output convention matches MeshTopology.facets / narrow_pt_tri:
//   · positions are scale*v + origin (applied here, once)
//   · facets store MESH-LOCAL vertex indices [0, vertexCount); the per-object
//     vertex base (statesOffsets[i]) is added downstream by the GPU kernels.
struct ObjMesh {
    std::vector<tinym::vec3> positions;   // size = vertexCount, transformed
    std::vector<Index>       facets;      // size = 3*numFacets, local indices
    Index vertexCount = 0;
    Index numFacets   = 0;                // triangles (quads already split)
    bool  ok          = false;
};

inline ObjMesh loadObjMesh(const std::string& path, float scale, tinym::vec3 origin) {
    ObjMesh m;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ObjLoader] cannot open " << path << "\n";
        return m;
    }

    std::vector<tinym::vec3> raw;     // untransformed verts (for neg-index wrap)
    std::vector<std::string> faceLines;

    std::string type;
    while (file >> type) {
        if (type == "v") {
            float x, y, z; file >> x >> y >> z;
            raw.push_back({x, y, z});
        } else if (type == "f") {
            std::string rest; std::getline(file, rest);
            faceLines.push_back(rest);
        } else {
            // skip vt / vn / mtllib / usemtl / o / g / s / comments / l / ...
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    const int nV = (int)raw.size();
    m.positions.reserve(nV);
    for (const auto& v : raw)
        m.positions.push_back({ v.x * scale + origin.x,
                                v.y * scale + origin.y,
                                v.z * scale + origin.z });
    m.vertexCount = (Index)nV;

    // Per face: extract leading vertex index of each whitespace token,
    // handling v / v/t / v//n / v/t/n and negative (relative) indices.
    auto vertOf = [&](const std::string& tok) -> int {
        int slash = (int)tok.find('/');
        int idx = std::stoi(slash < 0 ? tok : tok.substr(0, slash));
        return (idx < 0) ? idx + nV : idx - 1;   // OBJ is 1-based
    };

    std::regex tokRe("[^\\s]+");
    for (const auto& f : faceLines) {
        std::vector<int> e;
        for (auto it = std::sregex_iterator(f.begin(), f.end(), tokRe);
             it != std::sregex_iterator(); ++it) {
            const std::string s = it->str();
            if (!s.empty() && (std::isdigit((unsigned char)s[0]) || s[0] == '-'))
                e.push_back(vertOf(s));
        }
        auto tri = [&](int a, int b, int c) {
            m.facets.push_back((Index)a);
            m.facets.push_back((Index)b);
            m.facets.push_back((Index)c);
        };
        if (e.size() == 3)      tri(e[0], e[1], e[2]);
        else if (e.size() == 4){tri(e[0], e[1], e[2]); tri(e[0], e[2], e[3]); }
        // ngons (>4) or degenerate (<3) skipped — Human.obj is all quads.
    }
    m.numFacets = (Index)(m.facets.size() / 3);
    m.ok = (m.vertexCount > 0 && m.numFacets > 0);
    std::cout << "[ObjLoader] " << path << " verts=" << m.vertexCount
              << " tris=" << m.numFacets << (m.ok ? " OK\n" : " BAD\n");
    return m;
}
