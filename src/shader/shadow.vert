#version 410 core

// Depth-only pass for the directional shadow map. Mesh positions are
// world-space (M is identity in the main pass too), so light-space clip
// position is just LightVP * position.

layout(location = 0) in vec3 in_Position;

uniform mat4 LightVP;

void main() {
    gl_Position = LightVP * vec4(in_Position, 1.0);
}
