#version 330

// Adapted from
// 1. https://gist.github.com/cmbruns/3c184d303e665ee2e987e4c1c2fe4b56
// 2. https://asliceofrendering.com/scene%20helper/2020/01/05/InfiniteGrid/

in vec4 fragVert;
in vec4 fragNdc;

out vec4 finalColor; // Final fragment color

uniform mat4 mvp;           // Model-View-Projection matrix
uniform mat4 matModel;          // Model matrix
uniform mat4 matView;          // View matrix
uniform mat4 matProjection;          // Projection matrix

vec4 gridColor(vec4 v)
{
    vec4 vInW = inverse(matView) * v/v.w; // Vertex to world frame
    vec2 coord = vInW.xy;
    vec2 derivative = fwidth(coord);
    vec2 grid = abs(fract(coord - 0.5) - 0.5) / derivative;
    float line = min(grid.x, grid.y);
    float minimumy = min(derivative.y, 1);
    float minimumx = min(derivative.x, 1);
    vec4 color = vec4(0.5, 0.5, 0.5, 1.0 - min(line, 1.0));
    // y axis
    if(vInW.x > -minimumx && vInW.x < minimumx)
        color.xyz = vec3(0,1,0);
    // x axis
    if(vInW.y > -minimumy && vInW.y < minimumy)
        color.xyz = vec3(1,0,0);
    return color;
}

void main()
{
    // simple but expensive 2: discard pixels not on the plane
    if (fragVert.w > 0)
        discard;

    float clipSpaceDepth = (fragNdc.z / fragNdc.w + 1.0) / 2.0;
    gl_FragDepth = clipSpaceDepth;
    float depth = clipSpaceDepth * 2  - 1;

    float m22 = -matProjection[2][2];
    float m32 = -matProjection[3][2];    

    float near = (2.0f*m32)/(2.0f*m22-2.0f);
    float far = ((m22-1.0f)*near)/(m22+1.0);

    finalColor = gridColor(fragVert);
    finalColor.a *= sqrt(1/length(fragVert.xyz/fragVert.w));
}