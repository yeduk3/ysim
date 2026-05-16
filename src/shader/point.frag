#version 410 core

// Flat-colored point. Used three ways in point-selection mode:
//   black  (0,0,0)        — every selectable vertex
//   light yellow (1,1,.55)— hovered vertex
//   yellow (1,.8,0)       — selected vertex
out vec4 FragOut;

uniform vec3 uColor;

void main() {
    FragOut = vec4(uColor, 1.0);
}
