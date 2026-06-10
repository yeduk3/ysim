#version 410 core

layout( triangles ) in;
layout( triangle_strip, max_vertices = 3 ) out;

out vec3 GNormal;
out vec4 GPosition;
out vec4 GShadowCoord;
noperspective out vec3 GEdgeDistance;

in vec3 VNormal[];
in vec4 VPosition[];
in vec4 VShadowCoord[];

uniform mat4 ViewportMatrix;

void main() {
    // The screen-space edge-distance trick divides by clip w BEFORE
    // clipping runs. A triangle crossing the near plane has a vertex
    // with w <= 0; its projected position is garbage and the acos()
    // below sees an argument outside [-1,1] -> NaN -> the whole face
    // renders as LineColor (black). Large floors hit this at grazing
    // camera angles. Guard: skip the wireframe for such triangles
    // (huge edge distance == no line), and clamp the acos domain
    // against float round-off for healthy triangles.
    float ha, hb, hc;
    if (gl_in[0].gl_Position.w <= 0.0 ||
        gl_in[1].gl_Position.w <= 0.0 ||
        gl_in[2].gl_Position.w <= 0.0) {
        ha = hb = hc = 1e7;
    } else {
        // Transform each vertex into viewport space
        vec3 p0 = vec3(ViewportMatrix * (gl_in[0].gl_Position / gl_in[0].gl_Position.w));
        vec3 p1 = vec3(ViewportMatrix * (gl_in[1].gl_Position / gl_in[1].gl_Position.w));
        vec3 p2 = vec3(ViewportMatrix * (gl_in[2].gl_Position / gl_in[2].gl_Position.w));
        // Find the altitudes (ha, hb and hc)
        float a = length(p1 - p2);
        float b = length(p2 - p0);
        float c = length(p1 - p0);
        float alpha = acos( clamp((b*b + c*c- a*a) / (2.0*b*c), -1.0, 1.0) );
        float beta = acos( clamp((a*a + c*c - b*b) / (2.0*a*c), -1.0, 1.0) );
        ha = abs( c * sin( beta ) );
        hb = abs( c * sin( alpha ) );
        hc = abs( b * sin( alpha ) );
    }
    // Send the triangle along with the edge distances
    GEdgeDistance = vec3( ha, 0, 0 );
    GNormal = VNormal[0];
    GPosition = VPosition[0];
    GShadowCoord = VShadowCoord[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    GEdgeDistance = vec3( 0, hb, 0);
    GNormal = VNormal[1];
    GPosition = VPosition[1];
    GShadowCoord = VShadowCoord[1];
    gl_Position = gl_in[1].gl_Position;
    EmitVertex();
    GEdgeDistance = vec3( 0, 0, hc );
    GNormal = VNormal[2];
    GPosition = VPosition[2];
    GShadowCoord = VShadowCoord[2];
    gl_Position = gl_in[2].gl_Position;
    EmitVertex();
    EndPrimitive();
}
