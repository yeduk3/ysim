#version 410 core

layout(location = 0) in vec3 in_Position;

uniform mat4 V;
uniform mat4 P;

void main() {
    gl_Position = P*V*vec4(in_Position, 1);
}
