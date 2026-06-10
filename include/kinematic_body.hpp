#ifndef YSIM_KINEMATIC_BODY_HPP
#define YSIM_KINEMATIC_BODY_HPP

// CPU-pure sphere/cylinder proxy mesh for a BVH-driven kinematic body.
// Header-only, no Metal / GLFW / Eigen — pairs with bvh_motion.hpp the way
// primitive_geometry.hpp pairs with the primitive initializers.
//
// A kinematic body renders/collides as ONE concatenated triangle mesh:
//   - a sphere per joint (End Sites included), and
//   - a cylinder per parent→child link (degenerate links skipped),
// so it carries a single mesh id (whole-body selection for free) and its
// triangles ride the existing broad/narrow collision pipeline unchanged.
//
// build() fixes the topology once (facet indices never change); per frame
// writeVertices() repositions every vertex from an FK pose — the analogue
// of skinning, which is the intended future swap-in point (SMPL etc.).

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "bvh_motion.hpp"
#include "primitive_geometry.hpp"

namespace kinematic {

using Index = primitive::Index;

struct ProxyConfig {
    int sphereTessellation = 10;
    int cylinderTessellation = 8;
    // Radii as fractions of the skeleton's rest height, so the proxy looks
    // proportionate regardless of the BVH file's units (they vary ~7x
    // across assets/BVH).
    float jointRadiusFrac = 0.030f;
    float linkRadiusFrac = 0.020f;
};

struct BodyProxy {
    struct Part {
        int joint = -1;       // joint index in bvh::Motion::joints
        int parent = -1;      // -1 → sphere at `joint`; else cylinder parent→joint
        Index vertStart = 0;  // first vertex (vec3 index, not float index)
        Index vertCount = 0;
    };

    std::vector<Part> parts;
    std::vector<Index> facets;       // concatenated, vertex-offset applied
    Index numVerts = 0;
    Index numEdges = 0;              // closed-manifold sum (Euler per part)
    float jointRadius = 0.0f;        // in BVH file units
    float linkRadius = 0.0f;         // in BVH file units

    // Canonical templates, unit-radius, centered at origin.
    std::vector<float> unitSphere;   // r=1
    std::vector<float> unitCylinder; // r=1, y in [-0.5, 0.5]

    Index numFacets() const { return Index(facets.size() / 3); }

    void build(const bvh::Motion& motion, const ProxyConfig& cfg = {}) {
        parts.clear();
        facets.clear();
        numVerts = 0;
        numEdges = 0;

        const float h = motion.restHeight();
        jointRadius = (h > 0.0f ? h : 1.0f) * cfg.jointRadiusFrac;
        linkRadius = (h > 0.0f ? h : 1.0f) * cfg.linkRadiusFrac;

        auto sphereGeom = primitive::sphere(2.0f, cfg.sphereTessellation);  // r=1
        auto cylGeom = primitive::cylinder(1.0f, cfg.cylinderTessellation);
        unitSphere = sphereGeom.positions;
        unitCylinder = cylGeom.positions;
        // primitive::cylinder(size=1) → r=0.5, y∈[-0.5,0.5]. Normalize the
        // radial extent to 1 so writeVertices can scale x/z by linkRadius
        // and y by the live bone length directly.
        for (size_t v = 0; v < unitCylinder.size() / 3; ++v) {
            unitCylinder[v * 3 + 0] *= 2.0f;
            unitCylinder[v * 3 + 2] *= 2.0f;
        }
        const Index sphereV = Index(unitSphere.size() / 3);
        const Index cylV = Index(unitCylinder.size() / 3);

        auto appendPart = [&](const primitive::Geometry& g, Index nV,
                              int joint, int parent, int tess, bool isSphere) {
            Part p;
            p.joint = joint;
            p.parent = parent;
            p.vertStart = numVerts;
            p.vertCount = nV;
            parts.push_back(p);
            for (Index f : g.facets) facets.push_back(f + numVerts);
            numVerts += nV;
            numEdges += Index(isSphere ? primitive::sphereEdgeCount(tess)
                                       : primitive::cylinderEdgeCount(tess));
        };

        for (size_t j = 0; j < motion.joints.size(); ++j) {
            appendPart(sphereGeom, sphereV, int(j), -1,
                       cfg.sphereTessellation, true);
        }
        for (size_t j = 0; j < motion.joints.size(); ++j) {
            const int parent = motion.joints[j].parent;
            if (parent < 0) continue;
            const auto& o = motion.joints[j].offset;
            const float len2 = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
            if (len2 < 1e-10f) continue;  // zero-length link (stacked joints)
            appendPart(cylGeom, cylV, int(j), parent,
                       cfg.cylinderTessellation, false);
        }
    }

    // Repositions all proxy vertices for `pose`, then applies the body
    // transform: out = bodyR * (bodyScale ⊙ (uniformScale * v)) + bodyT.
    //   - uniformScale: unit normalization (e.g. targetHeight/restHeight).
    //   - bodyR: row-major 3x3 from the mesh's rotationQuat.
    //   - out: 3 * numVerts floats.
    // Joint spheres ignore joint rotation (rotation-invariant); link
    // cylinders are framed from the live parent→child segment, so lengths
    // follow the animated pose exactly.
    void writeVertices(const bvh::Pose& pose,
                       float uniformScale,
                       const std::array<float, 3>& bodyScale,
                       const std::array<float, 9>& bodyR,
                       const std::array<float, 3>& bodyT,
                       float* out) const {
        auto emit = [&](Index slot, float x, float y, float z) {
            const float sx = x * uniformScale * bodyScale[0];
            const float sy = y * uniformScale * bodyScale[1];
            const float sz = z * uniformScale * bodyScale[2];
            out[slot * 3 + 0] = bodyR[0] * sx + bodyR[1] * sy + bodyR[2] * sz + bodyT[0];
            out[slot * 3 + 1] = bodyR[3] * sx + bodyR[4] * sy + bodyR[5] * sz + bodyT[1];
            out[slot * 3 + 2] = bodyR[6] * sx + bodyR[7] * sy + bodyR[8] * sz + bodyT[2];
        };

        for (const Part& p : parts) {
            if (p.parent < 0) {
                const auto& c = pose.world[p.joint].t;
                for (Index v = 0; v < p.vertCount; ++v) {
                    emit(p.vertStart + v,
                         c[0] + jointRadius * unitSphere[v * 3 + 0],
                         c[1] + jointRadius * unitSphere[v * 3 + 1],
                         c[2] + jointRadius * unitSphere[v * 3 + 2]);
                }
            } else {
                const auto& a = pose.world[p.parent].t;
                const auto& b = pose.world[p.joint].t;
                float dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
                const float len = std::sqrt(dx * dx + dy * dy + dz * dz);
                std::array<float, 3> axisY{0.0f, 1.0f, 0.0f};
                if (len > 1e-8f) { axisY = {dx / len, dy / len, dz / len}; }
                // Orthonormal frame around the bone axis (any roll is fine
                // for a surface of revolution).
                std::array<float, 3> ref =
                    std::fabs(axisY[1]) < 0.9f ? std::array<float, 3>{0, 1, 0}
                                               : std::array<float, 3>{1, 0, 0};
                std::array<float, 3> axisX{
                    axisY[1] * ref[2] - axisY[2] * ref[1],
                    axisY[2] * ref[0] - axisY[0] * ref[2],
                    axisY[0] * ref[1] - axisY[1] * ref[0]};
                const float xl = std::sqrt(axisX[0] * axisX[0] +
                                           axisX[1] * axisX[1] +
                                           axisX[2] * axisX[2]);
                for (auto& c : axisX) c /= (xl > 1e-12f ? xl : 1.0f);
                const std::array<float, 3> axisZ{
                    axisY[1] * axisX[2] - axisY[2] * axisX[1],
                    axisY[2] * axisX[0] - axisY[0] * axisX[2],
                    axisY[0] * axisX[1] - axisY[1] * axisX[0]};
                const float mx = a[0] + dx * 0.5f;
                const float my = a[1] + dy * 0.5f;
                const float mz = a[2] + dz * 0.5f;
                for (Index v = 0; v < p.vertCount; ++v) {
                    const float lx = unitCylinder[v * 3 + 0] * linkRadius;
                    const float ly = unitCylinder[v * 3 + 1] * len;
                    const float lz = unitCylinder[v * 3 + 2] * linkRadius;
                    emit(p.vertStart + v,
                         mx + axisX[0] * lx + axisY[0] * ly + axisZ[0] * lz,
                         my + axisX[1] * lx + axisY[1] * ly + axisZ[1] * lz,
                         mz + axisX[2] * lx + axisY[2] * ly + axisZ[2] * lz);
                }
            }
        }
    }
};

}  // namespace kinematic

#endif  // YSIM_KINEMATIC_BODY_HPP
