#version 410 core

// RGBA32F id+depth buffer for VERTEX picking.
//   .r = float(uMeshId)     — owning mesh id (compacted slot)
//   .g = gl_FragCoord.z     — window-space depth in [0, 1]
//   .b = float(vVertexId)   — render-vertex index within that mesh
//   .a = 1.0
// Only fragments produced by the GL_POINTS draw write here; the
// triangle depth pre-pass runs with glColorMask off so non-point
// pixels keep the clear value (.r = -1, .b = -1 → "no vertex"). The
// cursor callback reads .r and .b; the point pass is depth-tested
// (GL_LEQUAL) against the solid mesh so occluded vertices are not
// pickable ("보이는 것만").
out vec4 FragOut;

uniform int uMeshId;

flat in int vVertexId;

void main() {
    FragOut = vec4(float(uMeshId), gl_FragCoord.z, float(vVertexId), 1.0);
}
