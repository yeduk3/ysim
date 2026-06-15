#version 410 core

layout(location = 0) in vec3 in_Position;
layout(location = 1) in vec3 in_Normal;

out vec4 VPosition;
out vec3 VNormal;
out vec4 VShadowCoord;
// World-space position (M * in_Position). Carried through the geometry
// shader to the fragment stage so the world-space checkerboard pattern
// can be derived per-fragment (see shader.frag's checker* uniforms).
out vec3 VWorldPos;

uniform mat4 M;
uniform mat4 V;
uniform mat4 P;
// Light-space clip transform for shadow mapping (identity-safe: when the
// host never sets it, the coord degenerates and the frag shader's
// shadowsOn gate keeps shadows off anyway).
uniform mat4 LightVP;


void main() {
    VPosition = (V*M * vec4(in_Position, 1));
    VNormal = mat3(V*M)*in_Normal;
    VShadowCoord = LightVP * M * vec4(in_Position, 1);
    VWorldPos = (M * vec4(in_Position, 1)).xyz;
    gl_Position = P*V*M * vec4(in_Position, 1);
}

