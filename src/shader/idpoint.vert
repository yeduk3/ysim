#version 410 core

// Point id-pass vertex shader. Mirrors id.vert (position attribute at
// location 0, M/V/P) but additionally forwards the per-vertex index so
// the fragment stage can write it into the .b channel. gl_VertexID is
// the index into the bound vertex buffer (the render-vertex id, since
// MeshGL binds preview.renderXPtr). Point size is set host-side via
// glPointSize before the GL_POINTS draw.
layout(location = 0) in vec3 in_Position;

uniform mat4 M;
uniform mat4 V;
uniform mat4 P;

flat out int vVertexId;

void main() {
    vVertexId = gl_VertexID;
    gl_Position = P*V*M * vec4(in_Position, 1);
}
