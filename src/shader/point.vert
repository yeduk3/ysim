#version 410 core

// On-screen point overlay (selectable-vertex dots + hover/select
// highlight). Position-only; flat color comes from a uniform in the
// fragment stage, point size from host-side glPointSize.
layout(location = 0) in vec3 in_Position;

uniform mat4 M;
uniform mat4 V;
uniform mat4 P;

void main() {
    gl_Position = P*V*M * vec4(in_Position, 1);
}
