#version 410 core

in vec3 GNormal;
in vec4 GPosition;
noperspective in vec3 GEdgeDistance;

out vec4 FragColor;

uniform vec3 lightPosition = vec3(50, 50, 30);
uniform mat4 V;
uniform vec3 lightColor = vec3(160, 160, 160);
uniform vec3 diffuseColor = vec3(1.0);
uniform vec3 specularColor = vec3(0.3);
uniform float shininess = 2.f;

// all computationis in view space
vec4 phong() {
    
    
    vec4 lp = V * vec4(lightPosition, 1);
    vec3 l = lp.xyz/lp.w - GPosition.xyz/GPosition.w;
    vec3 L = normalize(l);
    vec3 N = normalize(GNormal);
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

uniform float LineWidth = 0.1;
uniform vec4 LineColor = vec4(0, 0, 0, 1);

void main() {
    FragColor = phong();
    //FragColor = vec4(0,0,0,1);

    // Wireframe
    float d = min( GEdgeDistance.x, GEdgeDistance.y );
    d = min( d, GEdgeDistance.z );
    float mixVal = smoothstep( LineWidth - 1, LineWidth + 1, d);
    FragColor = mix( LineColor, FragColor, mixVal );
}
