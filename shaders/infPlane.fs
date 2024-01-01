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
    vec4 color = vec4(1.0, 1.0, 1.0, 1.0 - min(line, 1.0));
    color.a *= sqrt(1/length(v.xyz/v.w)) * float(v.w < 0);
    // y axis
    if(vInW.x > -minimumx*0.5 && vInW.x < minimumx*0.5 && color.a != 0)
        color = vec4(0,1,0,1);
    // x axis
    if(vInW.y > -minimumy*0.5 && vInW.y < minimumy*0.5 && color.a != 0)
        color = vec4(1,0,0,1);
    return color;
}

void main()
{
    gl_FragDepth = (fragNdc.z / fragNdc.w + 1.0) / 2.0;

    finalColor = gridColor(fragVert);
}