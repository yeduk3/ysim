#version 410 core

in vec3 Normal;
in vec4 VPos;

out vec4 FragColor;

uniform vec3 lightPosition = vec3(10, 10, 5);
uniform mat4 V;
uniform vec3 lightColor = vec3(160);
uniform vec3 diffuseColor = vec3(10);
uniform vec3 specularColor = vec3(0.3);
uniform float shininess = 2.f;

// all computationis in view space
vec4 phong() {
    
    
    vec4 lp = V * vec4(lightPosition, 1);
    vec3 l = lp.xyz/lp.w - VPos.xyz/VPos.w;
    vec3 L = normalize(l);
    vec3 N = normalize(Normal);
    if (!gl_FrontFacing) {
        N = -N;
    }
    vec3 R = 2 * dot(L, N) * N - L;
    vec3 I = lightColor / dot(l, l);

    vec3 ambient = diffuseColor * vec3(0.02);
    vec3 diffuse = I * diffuseColor * max(dot(L, N), 0);
    vec3 specular = I * specularColor * pow(max(R.z, 0), shininess);

    vec3 color = ambient + diffuse + specular;

    return vec4(pow(color, vec3(1/2.2)), 1);
}

void main() {
    FragColor = phong();
}

