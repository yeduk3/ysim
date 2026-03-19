#version 410 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Normal;

out vec4 VPosition;
out vec3 VNormal;

uniform mat4 M;
uniform mat4 V;
uniform mat4 P;


void main() {
    VPosition = (V*M * vec4(in_Position, 1));
    VNormal = mat3(V*M)*in_Normal;
    gl_Position = P*V*M * vec4(in_Position, 1);
}

