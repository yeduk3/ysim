#version 410 core

// RGBA32F id+depth buffer.
//   .r = float(uMeshId)   — −1.0 means "background / no object"
//   .g = gl_FragCoord.z   — window-space depth in [0, 1]
//   .b, .a = reserved
// Sampled by the cursor callback (glReadPixels, RGBA float) for hover
// detection and by shader.frag's outline pass: the silhouette test
// reads both the id channel (for the 5×5 neighborhood id-mismatch
// check) and the depth channel (to suppress outline when the neighbor
// is too far apart in depth — e.g. boundary against the far background
// or across large self-occluding gaps).
out vec4 FragOut;

uniform int uMeshId;

void main() {
    FragOut = vec4(float(uMeshId), gl_FragCoord.z, 0.0, 1.0);
}
