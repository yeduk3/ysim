#version 410 core

in vec3 GNormal;
in vec4 GPosition;
noperspective in vec3 GEdgeDistance;

out vec4 FragColor;

uniform vec3 lightPosition = vec3(50, 50, 30);
uniform mat4 V;
uniform vec3 lightColor = vec3(160, 160, 160);

// D-028: OpenPBR v1 subset uniforms (D-005). All five drive the
// pbrPreview() BRDF below. Defaults match Material{}'s defaults so a
// shader-bound-but-mesh-unbound draw renders the same as the prior
// Phong default surface.
uniform vec3  baseColor      = vec3(1.0);
uniform float metallic       = 0.0;
uniform float roughness      = 0.5;
uniform float specularWeight = 1.0;
uniform vec3  emissionColor  = vec3(0.0);

const float PI = 3.14159265359;

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

float distributionGGX(vec3 N, vec3 H, float a) {
    float a2    = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float d     = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

float geometrySmith(vec3 N, vec3 V_, vec3 L, float a) {
    float k     = (a + 1.0);
    k = (k * k) / 8.0;
    float NdotV = max(dot(N, V_), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}

// D-028: GGX-Smith microfacet preview. Single directional light (the
// existing `lightPosition` uniform is reinterpreted as a *position* and
// a normalized direction-from-fragment-to-light is computed in view
// space — keeps the falloff-free directional feel while reusing the
// existing UI control), plus a small constant ambient term to keep
// the shadowed side readable. No IBL / shadows / multi-light (out of
// v1 scope). Verified manually via the GUI roughness slider — see
// PROJECT_STATE.md's standing structural WARNING entry.
vec4 pbrPreview() {
    vec4 lp = V * vec4(lightPosition, 1.0);
    vec3 fragViewPos = GPosition.xyz / GPosition.w;
    vec3 L = normalize(lp.xyz / lp.w - fragViewPos);
    vec3 N = normalize(GNormal);
    if (!gl_FrontFacing) N = -N;
    vec3 Vv = normalize(-fragViewPos);
    vec3 H  = normalize(L + Vv);

    float a  = roughness * roughness;
    vec3  F0 = mix(vec3(0.04), baseColor, metallic) * specularWeight;
    vec3  F  = fresnelSchlick(max(dot(H, Vv), 0.0), F0);
    float D  = distributionGGX(N, H, a);
    float G  = geometrySmith(N, Vv, L, a);

    vec3  numerator = D * G * F;
    float denom     = 4.0 * max(dot(N, Vv), 0.0) * max(dot(N, L), 0.0) + 1e-4;
    vec3  specular  = numerator / denom;

    vec3  kS = F;
    vec3  kD = (vec3(1.0) - kS) * (1.0 - metallic);
    float NdotL = max(dot(N, L), 0.0);

    // D-028 follow-on: `lightColor` is the direct radiance uploaded by
    // the host (SceneEnvironment.lightColor * lightIntensity, set per
    // frame from the Environment panel). No magnitude rescale here —
    // the inspector controls magnitude explicitly. Ambient at 0.10 *
    // baseColor keeps the shadowed side readable.
    vec3 Li      = lightColor;
    vec3 ambient = vec3(0.10) * baseColor;
    vec3 color   = ambient + (kD * baseColor / PI + specular) * Li * NdotL + emissionColor;

    return vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}

uniform float LineWidth = 0.1;
uniform vec4 LineColor = vec4(0, 0, 0, 1);

void main() {
    FragColor = pbrPreview();

    // Wireframe overlay (unchanged from the prior Phong path).
    float d = min( GEdgeDistance.x, GEdgeDistance.y );
    d = min( d, GEdgeDistance.z );
    float mixVal = smoothstep( LineWidth - 1, LineWidth + 1, d);
    FragColor = mix( LineColor, FragColor, mixVal );
}
